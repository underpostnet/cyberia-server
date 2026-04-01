package game

// SkillDefinition defines the properties of a skill, including the ordered list of
// logic handlers to execute when the trigger item is used.
type SkillDefinition struct {
	LogicEventIDs []string
}

// HandlePlayerActionSkills checks if a player's action triggers any skills
// based on their active items and calls the corresponding skill logic.
// Called from readPump (handlers.go) AFTER s.mu is released — callerHoldsLock = false.
func (s *GameServer) HandlePlayerActionSkills(player *PlayerState, mapState *MapState, target Point) {
	s.dispatchSkillsForEntity(player, mapState, target, false)
}

// handleBotSkills checks and executes skills for a bot when it takes an "action".
// Called from updateBots (ai.go) which runs inside the game loop lock — callerHoldsLock = true.
func (s *GameServer) handleBotSkills(bot *BotState, mapState *MapState, target Point) {
	s.dispatchSkillsForEntity(bot, mapState, target, true)
}

// HandleOnKillSkills checks for skills that trigger when an entity is killed by a bot

func (s *GameServer) HandleOnKillSkills(killerBot *BotState, victim interface{}, mapState *MapState) { // The killer is a skill projectile bot. The actual killer is the caster.
	// The killer is a skill projectile bot. The actual killer is the caster.
	if killerBot.CasterID == "" {
		return
	}

	var caster interface{}

	// Find the caster, which can be a player or a bot.
	if p, ok := mapState.players[killerBot.CasterID]; ok {
		caster = p
	} else if b, ok := mapState.bots[killerBot.CasterID]; ok {
		caster = b
	} else {
		return // Caster not found in the map.
	}

	// --- Core Mechanic: on Kill Events ---
	// This is not a skill, but a fundamental rule: the victor loots the vanquished.
	s.executeCoinDropOnKill(caster, victim, s)

}

// GetAssociatedSkillItemIDs returns the unique list of logicEventIDs associated with a trigger item.
func (s *GameServer) GetAssociatedSkillItemIDs(itemID string) []string {
	associatedIDs := make(map[string]struct{})

	if skillDefs, ok := s.skillConfig[itemID]; ok {
		for _, skillDef := range skillDefs {
			for _, id := range skillDef.LogicEventIDs {
				associatedIDs[id] = struct{}{}
			}
		}
	}

	// Convert map keys to a slice.
	result := make([]string, 0, len(associatedIDs))
	for id := range associatedIDs {
		result = append(result, id)
	}

	return result
}
