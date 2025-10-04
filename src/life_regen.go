package game

import (
	"math"
	"math/rand"
)

const lifeRegenChance = 0.35 // 35% chance to regenerate on action

// handleProbabilisticRegen gives an entity a chance to regenerate life when they perform an action.
// This is called when a player sets a target or a bot decides on a new path. The chance to trigger
// is increased by Utility, and the amount regenerated is increased by Resistance.
func (s *GameServer) handleProbabilisticRegen(entity interface{}, mapState *MapState) {
	stats := s.CalculateStats(entity, mapState)

	// Utility stat increases the chance of the skill activating.
	// We'll model this as a linear increase, capping the effective chance at 95%.
	triggerChance := math.Min(lifeRegenChance+(stats.Utility/100.0), 0.95)

	if rand.Float64() >= triggerChance {
		return // Regeneration did not trigger
	}

	switch e := entity.(type) {
	case *PlayerState:
		// Resistance stat increases the amount of life regenerated.
		regenAmount := e.LifeRegen + stats.Resistance
		if regenAmount > 0 && e.Life < e.MaxLife {
			e.Life = e.Life + regenAmount
			if e.Life > e.MaxLife {
				e.Life = e.MaxLife
			}
		}
	case *BotState:
		// Resistance stat increases the amount of life regenerated.
		regenAmount := e.LifeRegen + stats.Resistance
		if regenAmount > 0 && e.Life < e.MaxLife {
			e.Life = e.Life + regenAmount
			if e.Life > e.MaxLife {
				e.Life = e.MaxLife
			}
		}
	}
}
