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
func (s *GameServer) HandlePlayerTapAction(player *PlayerState, mapState *MapState, target Point) {
	if player.IsGhost() || player.Frozen {
		return
	}
	if !s.acceptSkillAction(player, mapState, time.Now()) {
		return
	}
	s.handleProbabilisticRegen(player, mapState)
	s.dispatchSkillsForEntity(player, mapState, target, true)
}

func (s *GameServer) acceptSkillAction(entity statSource, mapState *MapState, now time.Time) bool {
	state := entity.StatState()
	if state.IsGhost() || now.Before(state.NextSkillAt) {
		return false
	}
	state.NextSkillAt = now.Add(s.CalculateActionCooldown(s.CalculateStats(entity, mapState)))
	return true
}

// Bot action scheduling owns its cooldown.
func (s *GameServer) handleBotSkills(bot *BotState, mapState *MapState, target Point) {
	s.dispatchSkillsForEntity(bot, mapState, target, true)
}
