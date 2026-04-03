package game

import (
	"math"
	"math/rand"
)

// handleProbabilisticRegen gives an entity a chance to regenerate life when they perform an action.
// This is called when a player sets a target or a bot decides on a new path. The chance to trigger
// is increased by Utility, and the amount regenerated is increased by Resistance.
func (s *GameServer) handleProbabilisticRegen(entity interface{}, mapState *MapState) {
	stats := s.CalculateStats(entity, mapState)

	// Utility stat increases the chance of the skill activating.
	triggerChance := math.Min(s.lifeRegenChance+(stats.Utility/100.0), s.maxChance)

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
			// FCT: show the regen as a green floating number at the player's position.
			if regenInt := int(regenAmount + 0.5); regenInt > 0 {
				sendFCT(e, FCTTypeRegen, e.Pos.X, e.Pos.Y, regenInt)
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
