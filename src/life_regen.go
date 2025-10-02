package game

import (
	"math/rand"
)

const lifeRegenChance = 0.35 // 35% chance to regenerate on action

// handleProbabilisticRegen gives an entity a chance to regenerate life when they perform an action.
// This is called when a player sets a target or a bot decides on a new path.
func (s *GameServer) handleProbabilisticRegen(entity interface{}) {
	if rand.Float64() >= lifeRegenChance {
		return // Regeneration did not trigger
	}

	switch e := entity.(type) {
	case *PlayerState:
		if e.LifeRegen > 0 && e.Life < e.MaxLife {
			e.Life = e.Life + e.LifeRegen
			if e.Life > e.MaxLife {
				e.Life = e.MaxLife
			}
		}
	case *BotState:
		if e.LifeRegen > 0 && e.Life < e.MaxLife {
			e.Life = e.Life + e.LifeRegen
			if e.Life > e.MaxLife {
				e.Life = e.MaxLife
			}
		}
	}
}
