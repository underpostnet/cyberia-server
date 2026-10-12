package game

import "math/rand"

// handleProbabilisticRegen gives an entity a chance to regenerate life on an
// action: a player's tap or a bot's new path. Utility raises the chance,
// resistance the amount, on top of the entity's own base regeneration.
func (s *GameServer) handleProbabilisticRegen(entity interface{}, mapState *MapState) {
	var base *EntityBase
	var lifeRegen float64
	switch e := entity.(type) {
	case *PlayerState:
		base, lifeRegen = &e.EntityBase, e.LifeRegen
	case *BotState:
		base, lifeRegen = &e.EntityBase, e.LifeRegen
	default:
		return
	}
	v := mapState.vt(base.ID)
	if v.Life >= v.MaxLife {
		return
	}
	stats := s.CalculateStats(entity, mapState)
	if rand.Float64() >= s.chance(s.lifeRegenChance, stats.Utility*statScales.Utility) {
		return
	}
	amount := lifeRegen + stats.Resistance*statRegenPerResistance
	if amount <= 0 {
		return
	}
	restoreLife(mapState, v, mapState.tf(base.ID).Pos, amount)
}

// restoreLife adds up to amount life, never past MaxLife. Every AOI viewer sees the same green
// "+N" for the life actually restored, for players and bots alike.
func restoreLife(mapState *MapState, v *Vitals, at Vec2, amount float64) {
	gained := min(v.Life+amount, v.MaxLife) - v.Life
	if gained <= 0 {
		return
	}
	v.Life += gained
	if shown := int(gained + 0.5); shown > 0 {
		broadcastFCT(mapState, FCTRegen, at.X, at.Y, shown)
	}
}
