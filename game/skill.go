package game

// SkillDefinition defines the properties of a single skill logic event
// associated with a trigger item.
type SkillDefinition struct {
	LogicEventID         string
	Name                 string
	Description          string
	SummonedEntityItemID string
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
// Called from phaseInput (inside simTicker) which holds s.mu — callerHoldsLock = true.
func (s *GameServer) HandlePlayerTapAction(player *PlayerState, mapState *MapState, target Point) {
	s.handleProbabilisticRegen(player, mapState)
	s.dispatchSkillsForEntity(player, mapState, target, true)
}

// handleBotSkills checks and executes skills for a bot when it takes an "action".
// Called from updateBots (ai.go) which runs inside the game loop lock — callerHoldsLock = true.
func (s *GameServer) handleBotSkills(bot *BotState, mapState *MapState, target Point) {
	s.dispatchSkillsForEntity(bot, mapState, target, true)
}

// GetAssociatedSkillItemIDs returns the unique list of logicEventIDs associated with a trigger item.
func (s *GameServer) GetAssociatedSkillItemIDs(itemID string) []string {
	associatedIDs := make(map[string]struct{})

	if skillDefs, ok := s.skillConfig[itemID]; ok {
		for _, skillDef := range skillDefs {
			if skillDef.LogicEventID != "" {
				associatedIDs[skillDef.LogicEventID] = struct{}{}
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
