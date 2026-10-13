package game

import "time"

// HandlePlayerTapAction gates skills and regeneration with the authoritative cooldown.
func (s *Sim) HandlePlayerTapAction(player *Session, mapState *MapState, target Vec2) {
	if mapState.vt(player.ID).ghost() || player.Frozen {
		return
	}
	if !s.acceptSkillAction(player.ID, mapState, time.Now()) {
		return
	}
	s.handleProbabilisticRegen(player.ID, mapState)
	s.dispatchSkillsForEntity(player.ID, mapState, target, true)
}

func (s *Sim) acceptSkillAction(id EntityID, mapState *MapState, now time.Time) bool {
	state := mapState.mt(id)
	if mapState.vt(id).ghost() || now.Before(state.NextSkillAt) {
		return false
	}
	state.NextSkillAt = now.Add(s.CalculateActionCooldown(s.CalculateStats(id, mapState)))
	return true
}

// Bot action scheduling owns its cooldown.
func (s *Sim) handleBotSkills(id EntityID, mapState *MapState, target Vec2) {
	s.dispatchSkillsForEntity(id, mapState, target, true)
}
