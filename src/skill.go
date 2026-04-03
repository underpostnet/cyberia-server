package game

// SkillDefinition defines the properties of a skill, including the ordered list of
// logic handlers to execute when the trigger item is used.
type SkillDefinition struct {
	LogicEventIDs []string
}

// HandlePlayerTapAction is the canonical handler for every tap/click event from
// the client.  It must:
//  1. Always attempt to trigger skills (dispatchSkillsForEntity) — skills fire on
//     every TAP, probability-gated by Intelligence only; movement must NOT gate
//     skill execution.
//  2. Optionally trigger probabilistic life regen.
//
// Movement path calculation remains a separate concern handled in readPump
// (handlers.go) and is gated by the Utility cooldown, independent of this call.
//
// Called from readPump AFTER s.mu is released — callerHoldsLock = false.
func (s *GameServer) HandlePlayerTapAction(player *PlayerState, mapState *MapState, target Point) {
	s.handleProbabilisticRegen(player, mapState)
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
	// Fountain & Sink kill transfer: victor loots the vanquished.
	// Also sends FCT (Floating Combat Text) economy event to both parties.
	s.ExecuteKillTransfer(caster, victim)

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
