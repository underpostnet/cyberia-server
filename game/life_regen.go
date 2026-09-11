package game

import "math/rand"

// handleProbabilisticRegen gives an entity a chance to regenerate life on an
// action: a player's tap or a bot's new path. Utility raises the chance,
// resistance the amount, on top of the entity's own base regeneration.
func (s *GameServer) handleProbabilisticRegen(entity interface{}, mapState *MapState) {
	var mortal *Mortal
	var base *EntityBase
	var lifeRegen float64
	switch e := entity.(type) {
	case *PlayerState:
		mortal, base, lifeRegen = &e.Mortal, &e.EntityBase, e.LifeRegen
	case *BotState:
		mortal, base, lifeRegen = &e.Mortal, &e.EntityBase, e.LifeRegen
	default:
		return
	}
	if mortal.Life >= mortal.MaxLife {
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
	mortal.Life = min(mortal.Life+amount, mortal.MaxLife)
	// FCT: the same green "+N" for every AOI viewer, for players and bots alike,
	// so regeneration is visible feedback rather than a silently refilling bar.
	if shown := int(amount + 0.5); shown > 0 {
		broadcastFCT(mapState, FCTRegen, base.Pos.X, base.Pos.Y, shown)
	}
}
