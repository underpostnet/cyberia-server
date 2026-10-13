package game

import "math/rand"

// handleProbabilisticRegen gives an entity a chance to regenerate life on an
// action: a player's tap or a bot's new path. Utility raises the chance,
// resistance the amount, on top of the entity's own base regeneration.
func (s *Sim) handleProbabilisticRegen(id EntityID, mapState *MapState) {
	v := mapState.vt(id)
	if v.Life >= v.MaxLife {
		return
	}
	stats := s.CalculateStats(id, mapState)
	if rand.Float64() >= s.chance(s.lifeRegenChance, stats.Utility*statScales.Utility) {
		return
	}
	amount := mapState.mt(id).LifeRegen + stats.Resistance*statRegenPerResistance
	if amount <= 0 {
		return
	}
	s.restoreLife(mapState, v, mapState.tf(id).Pos, amount)
}

// restoreLife adds up to amount life, never past MaxLife. Every AOI viewer sees the same green
// "+N" for the life actually restored, for players and bots alike.
func (s *Content) restoreLife(mapState *MapState, v *Vitals, at Vec2, amount float64) {
	gained := min(v.Life+amount, v.MaxLife) - v.Life
	if gained <= 0 {
		return
	}
	v.Life += gained
	if shown := int(gained + 0.5); shown > 0 {
		s.broadcastFCT(mapState, FCTRegen, at.X, at.Y, shown)
	}
}
