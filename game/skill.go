package game

import "time"

// SkillDefinition defines the properties of a single skill logic event
// associated with a trigger item.
type SkillDefinition struct {
	LogicEventID         string
	Name                 string
	Description          string
	SummonedEntityItemID string
}

// HandlePlayerTapAction gates skills and regeneration with the authoritative cooldown.
func (s *GameServer) HandlePlayerTapAction(player *Session, mapState *MapState, target Vec2) {
	if mapState.vt(player.ID).ghost() || player.Frozen {
		return
	}
	if !s.acceptSkillAction(player.ID, mapState, time.Now()) {
		return
	}
	s.handleProbabilisticRegen(player.ID, mapState)
	s.dispatchSkillsForEntity(player.ID, mapState, target, true)
}

func (s *GameServer) acceptSkillAction(id EntityID, mapState *MapState, now time.Time) bool {
	state := mapState.mt(id)
	if mapState.vt(id).ghost() || now.Before(state.NextSkillAt) {
		return false
	}
	state.NextSkillAt = now.Add(s.CalculateActionCooldown(s.CalculateStats(id, mapState)))
	return true
}

// Bot action scheduling owns its cooldown.
func (s *GameServer) handleBotSkills(id EntityID, mapState *MapState, target Vec2) {
	s.dispatchSkillsForEntity(id, mapState, target, true)
}
