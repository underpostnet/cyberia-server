package sim

import (
	"time"

	"cyberia-server/game/world"
)

// HandlePlayerTapAction gates skills and regeneration with the authoritative cooldown.
func (s *Sim) HandlePlayerTapAction(player *world.Session, mapState *world.MapState, target world.Vec2) {
	if mapState.Vt(player.ID).Ghost() || player.Frozen {
		return
	}
	if !s.acceptSkillAction(player.ID, mapState, time.Now()) {
		return
	}
	s.handleProbabilisticRegen(player.ID, mapState)
	s.dispatchSkillsForEntity(player.ID, mapState, target, true)
}

func (s *Sim) acceptSkillAction(id world.EntityID, mapState *world.MapState, now time.Time) bool {
	state := mapState.Mt(id)
	if mapState.Vt(id).Ghost() || now.Before(state.NextSkillAt) {
		return false
	}
	state.NextSkillAt = now.Add(s.CalculateActionCooldown(s.CalculateStats(id, mapState)))
	return true
}

// Bot action scheduling owns its cooldown.
func (s *Sim) handleBotSkills(id world.EntityID, mapState *world.MapState, target world.Vec2) {
	s.dispatchSkillsForEntity(id, mapState, target, true)
}
