package game

import "time"

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

// Utility — Reduces the cooldown time between actions, allowing for more frequent
// actions. It also increases the chance to trigger life-regeneration events.

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
func (s *GameServer) CalculateStats(source interface{}, mapState *MapState) ComputedStats {
	var totalStats ComputedStats
	var objectLayers []ObjectLayerState
	var casterID string

	switch e := source.(type) {
	case *PlayerState:
		objectLayers = e.ObjectLayers
		casterID = "" // Players are not cast
	case *BotState:
		objectLayers = e.ObjectLayers
		casterID = e.CasterID
	default:
		return totalStats // Return zero stats for unknown types
	}

	// 1. Sum stats from the entity's own active layers.
	for _, layer := range objectLayers {
		if !layer.Active {
			continue
		}
		if data, ok := s.objectLayerDataCache[layer.ItemID]; ok {
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
	}
}

// CalculateActionCooldown computes the effective action cooldown for an entity
// based on its Utility stat.
func (s *GameServer) CalculateActionCooldown(stats ComputedStats) time.Duration {
	// Utility stat reduces the base cooldown, measured in milliseconds.
	utilityReduction := time.Duration(stats.Utility) * time.Millisecond
	currentCooldown := s.entityBaseActionCooldown - utilityReduction

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
