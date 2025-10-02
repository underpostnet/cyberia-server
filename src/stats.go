package game

// Description of passive stats mechanics:

// Effect (Collision damage) — Amount of life removed when
// an entity collides or deals an impact. Measured in life points.

// Resistance — Adds to the owner's maximum life (survivability cap).
// This value is summed with the entity's base max life.

// Agility — Reduces the cooldown time between actions, allowing for more frequent actions.
// Measured in milliseconds of cooldown reduction.

// Range — Lifetime of a caster/summoned entity, measured in milliseconds.

import "time"

// Intelligence — Probability-based stat that increases chance to
// spawn/trigger a caster-bot or to trigger life-regeneration events.
// Expressed as a probability or chance modifier.

// Utility — Amount of life restored when a regeneration event occurs
// (adds directly to current life).

// ComputedStats holds the final, summed values of all passive stats for an entity.
// These values are calculated on-demand from the entity's active object layers
// and, if applicable, inherited from its caster.
type ComputedStats struct {
	Effect       float64 // Collision damage dealt.
	Resistance   float64 // Bonus to maximum life.
	Agility      float64 // Affects action/input rates (conceptual).
	Range        float64 // Modifies lifetime of summoned entities.
	Intelligence float64 // Influences probability-based skills (e.g., spawn chance, regen chance).
	Utility      float64 // Modifies quantity of effects (e.g., life regenerated).
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
// based on its Agility stat.
func (s *GameServer) CalculateActionCooldown(stats ComputedStats) time.Duration {
	// Agility stat reduces the base cooldown, measured in milliseconds.
	agilityBonus := time.Duration(stats.Agility) * time.Millisecond
	currentCooldown := s.entityBaseActionCooldown - agilityBonus

	// Ensure the cooldown does not fall below the minimum defined threshold.
	if currentCooldown < s.entityBaseMinActionCooldown {
		return s.entityBaseMinActionCooldown
	}
	return currentCooldown
}
