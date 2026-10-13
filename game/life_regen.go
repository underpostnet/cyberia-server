package game

import (
	"math/rand"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// handleProbabilisticRegen gives an entity a chance to regenerate life on an
// action: a player's tap or a bot's new path. Utility raises the chance,
// resistance the amount, on top of the entity's own base regeneration.
func (s *Sim) handleProbabilisticRegen(id world.EntityID, mapState *world.MapState) {
	v := mapState.Vt(id)
	if v.Life >= v.MaxLife {
		return
	}
	stats := s.CalculateStats(id, mapState)
	if rand.Float64() >= s.Chance(s.LifeRegenChance, stats.Utility*content.StatScales.Utility) {
		return
	}
	amount := mapState.Mt(id).LifeRegen + stats.Resistance*content.StatRegenPerResistance
	if amount <= 0 {
		return
	}
	s.RestoreLife(mapState, v, mapState.Tf(id).Pos, amount)
}
