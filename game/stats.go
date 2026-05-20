package game

import "time"

// defaultStatsCacheTTL is the fallback TTL for cached stats entries.
// Even if StatsDirty is false, entries older than this are recalculated
// to prevent stale values in edge cases.
const defaultStatsCacheTTL = 250 * time.Millisecond

// statsCacheEntry wraps a ComputedStats value with a timestamp so the
// cache can expire entries after the configured TTL.
type statsCacheEntry struct {
	stats     ComputedStats
	cachedAt  time.Time
}

// -----------------------------------------------------------------------------------------
// Description of passive stats mechanics:
// -----------------------------------------------------------------------------------------

// Effect — Amount of life removed when an entity collides or deals an impact.
// Measured in life points.

// Resistance — Adds to the owner's maximum life (survivability cap). This value
// is summed with the entity's base max life. It also increases the amount of
// life restored when a regeneration event occurs (adds directly to current life).

// Agility — Increases the movement speed of entities.

// Range — Increases the lifetime of a cast/summoned entity, measured in milliseconds.

// Intelligence — Probability-based stat that increases the chance to
// spawn/trigger a summoned entity.

// Utility — Reduces the cooldown time between actions as a percentage (1 Utility = 1% reduction),
// allowing for more frequent actions. It also increases the chance to trigger life-regeneration events.

// -----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------

// ComputedStats holds the final, summed values of all passive stats for an entity.
// These values are calculated on-demand from the entity's active object layers
// and, if applicable, inherited from its caster.
type ComputedStats struct {
	Effect       float64 // Collision damage dealt.
	Resistance   float64 // Bonus to maximum life and regeneration amount.
	Agility      float64 // Increases movement speed.
	Range        float64 // Increases lifetime of summoned entities.
	Intelligence float64 // Increases chance to spawn/trigger summoned entities.
	Utility      float64 // Reduces action cooldowns and increases life-regen chance.
}

// CalculateStats computes the final stat values for a given StatSource.
// It sums the stats from the source's active object layers and recursively
// adds the stats from its caster, if one exists.
// Results are cached per entity and reused until StatsDirty is set or the
// cache TTL expires (default 250ms), whichever comes first.
func (s *GameServer) CalculateStats(source interface{}, mapState *MapState) ComputedStats {
	var totalStats ComputedStats
	var objectLayers []ObjectLayerState
	var casterID string
	var entityID string
	var dirty bool

	switch e := source.(type) {
	case *PlayerState:
		entityID = e.ID
		objectLayers = e.ObjectLayers
		casterID = "" // Players are not cast
		dirty = e.StatsDirty
	case *BotState:
		entityID = e.ID
		objectLayers = e.ObjectLayers
		casterID = e.CasterID
		dirty = e.StatsDirty
	case *ResourceState:
		entityID = e.ID
		objectLayers = e.ObjectLayers
		casterID = ""
		dirty = e.StatsDirty
	default:
		return totalStats // Return zero stats for unknown types
	}

	// Return cached stats if the entity is not dirty and the TTL hasn't expired.
	if !dirty {
		if entry, ok := s.statsCache[entityID]; ok {
			ttl := s.statsCacheTTL
			if ttl == 0 {
				ttl = defaultStatsCacheTTL
			}
			if time.Since(entry.cachedAt) < ttl {
				return entry.stats
			}
		}
	}

	// 1. Sum stats from the entity's own active layers.
	for _, layer := range objectLayers {
		if !layer.Active {
			continue
		}
		if data, ok := s.GetObjectLayerData(layer.ItemID); ok {
			totalStats.Effect += float64(data.Data.Stats.Effect)
			totalStats.Resistance += float64(data.Data.Stats.Resistance)
			totalStats.Agility += float64(data.Data.Stats.Agility)
			totalStats.Range += float64(data.Data.Stats.Range)
			totalStats.Intelligence += float64(data.Data.Stats.Intelligence)
			totalStats.Utility += float64(data.Data.Stats.Utility)
		}
	}

	// 2. If the entity was summoned/cast, add the caster's stats.
	if casterID != "" {
		var casterSource interface{}
		// Find the caster, which can be a player or another bot.
		if p, ok := mapState.players[casterID]; ok {
			casterSource = p
		} else if b, ok := mapState.bots[casterID]; ok {
			casterSource = b
		}

		// If the caster is found, recursively calculate their stats and add them.
		if casterSource != nil {
			casterStats := s.CalculateStats(casterSource, mapState)
			totalStats.Effect += casterStats.Effect
			totalStats.Resistance += casterStats.Resistance
			totalStats.Agility += casterStats.Agility
			totalStats.Range += casterStats.Range
			totalStats.Intelligence += casterStats.Intelligence
			totalStats.Utility += casterStats.Utility
		}
	}

	// Store in cache with current timestamp and clear dirty flag.
	s.statsCache[entityID] = statsCacheEntry{
		stats:    totalStats,
		cachedAt: time.Now(),
	}
	switch e := source.(type) {
	case *PlayerState:
		e.StatsDirty = false
	case *BotState:
		e.StatsDirty = false
	case *ResourceState:
		e.StatsDirty = false
	}

	return totalStats
}

// ApplyResistanceStat updates an entity's MaxLife based on its Resistance stat.
// This should be called whenever an entity's layers change.
func (s *GameServer) ApplyResistanceStat(entity interface{}, mapState *MapState) {
	stats := s.CalculateStats(entity, mapState)
	switch e := entity.(type) {
	case *PlayerState:
		e.MaxLife = s.entityBaseMaxLife + stats.Resistance
	case *BotState:
		e.MaxLife = s.entityBaseMaxLife + stats.Resistance
	case *ResourceState:
		e.MaxLife = s.entityBaseMaxLife + stats.Resistance
	}
}

// CalculateActionCooldown computes the effective action cooldown for an entity
// based on its Utility stat.
func (s *GameServer) CalculateActionCooldown(stats ComputedStats) time.Duration {
	// Utility stat reduces the base cooldown as a percentage: 1 Utility = 1% reduction.
	// Consistent with Agility (1% speed) and Intelligence (1% spawn chance).
	reductionFactor := 1.0 - (stats.Utility / 100.0)
	if reductionFactor < 0 {
		reductionFactor = 0
	}
	currentCooldown := time.Duration(float64(s.entityBaseActionCooldown) * reductionFactor)

	// Ensure the cooldown does not fall below the minimum defined threshold.
	if currentCooldown < s.entityBaseMinActionCooldown {
		return s.entityBaseMinActionCooldown
	}
	return currentCooldown
}

// CalculateMovementSpeed computes the effective movement speed for an entity
// based on its Agility stat. Speed is measured in grid units per second.
func (s *GameServer) CalculateMovementSpeed(stats ComputedStats) float64 {
	// Agility stat increases the base speed.
	// We'll model this as a percentage increase: 1 Agility = +1% speed.
	speedMultiplier := 1.0 + (stats.Agility / 100.0)
	return s.entityBaseSpeed * speedMultiplier
}

// InvalidateStats marks an entity as needing a stats re-calculation and
// removes its entry from the cache. Call this whenever an entity's
// ObjectLayers (active flags, additions, removals) change.
func (s *GameServer) InvalidateStats(entity interface{}) {
	switch e := entity.(type) {
	case *PlayerState:
		e.StatsDirty = true
		delete(s.statsCache, e.ID)
	case *BotState:
		e.StatsDirty = true
		delete(s.statsCache, e.ID)
	case *ResourceState:
		e.StatsDirty = true
		delete(s.statsCache, e.ID)
	}
}
