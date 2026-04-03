package game

import "log"

// SkillContext carries all information needed by a skill handler.
// It unifies player and bot execution into a single call signature.
type SkillContext struct {
	// Caster is the entity that triggered the skill (*PlayerState or *BotState).
	Caster interface{}
	// MapState is the map the caster belongs to.
	MapState *MapState
	// Target is the world-space point the caster acted upon (tap destination).
	Target Point
	// TriggerItemID is the ObjectLayer item that matched the skillConfig entry.
	TriggerItemID string
	// CallerHoldsLock is true when the server mutex is already held by the caller
	// (e.g. inside the game-loop / updateBots). Skill implementations that need
	// the lock must check this flag to avoid a deadlock.
	CallerHoldsLock bool
}

// SkillHandlerFunc is the signature every skill implementation must satisfy.
type SkillHandlerFunc func(s *GameServer, ctx SkillContext)

// skillRegistry maps logicEventID → handler. Populated by InitSkills() during
// server startup. New skills only need to add one entry here.
var skillRegistry = map[string]SkillHandlerFunc{}

// registerSkill adds (or replaces) a handler for the given logicEventID.
func registerSkill(logicEventID string, handler SkillHandlerFunc) {
	skillRegistry[logicEventID] = handler
}

// InitSkills wires all built-in skill implementations into the registry.
// Called from ApplyInstanceConfig after all server parameters are set.
func (s *GameServer) InitSkills() {
	registerSkill("atlas_pistol_mk2_logic", func(s *GameServer, ctx SkillContext) {
		s.executeProjectileSkill(ctx)
	})
	registerSkill("doppelganger", func(s *GameServer, ctx SkillContext) {
		s.executeDoppelgangerSkill(ctx)
	})
	registerSkill("coin_drop_or_transaction", func(s *GameServer, ctx SkillContext) {
		// Economy mechanic: coin transfer is triggered automatically on kill
		// via HandleOnKillSkills → ExecuteKillTransfer. No manual action needed.
	})
}

// DispatchSkill routes a single logicEventID to its registered handler.
// Unknown IDs are logged and silently ignored.
func (s *GameServer) DispatchSkill(logicID string, ctx SkillContext) {
	handler, ok := skillRegistry[logicID]
	if !ok {
		log.Printf("Unknown logicEventID '%s' for item '%s'", logicID, ctx.TriggerItemID)
		return
	}
	handler(s, ctx)
}

// dispatchSkillsForEntity iterates the entity's active ObjectLayers, looks up
// matching entries in skillConfig, and dispatches each logicEventID.
// This replaces the duplicated switch blocks in HandlePlayerTapAction and handleBotSkills.
func (s *GameServer) dispatchSkillsForEntity(caster interface{}, mapState *MapState, target Point, callerHoldsLock bool) {
	var layers []ObjectLayerState
	switch e := caster.(type) {
	case *PlayerState:
		layers = e.ObjectLayers
	case *BotState:
		layers = e.ObjectLayers
	default:
		return
	}

	for _, layer := range layers {
		if !layer.Active {
			continue
		}
		if skillDefs, ok := s.skillConfig[layer.ItemID]; ok {
			for _, skillDef := range skillDefs {
				for _, logicID := range skillDef.LogicEventIDs {
					s.DispatchSkill(logicID, SkillContext{
						Caster:          caster,
						MapState:        mapState,
						Target:          target,
						TriggerItemID:   layer.ItemID,
						CallerHoldsLock: callerHoldsLock,
					})
				}
			}
		}
	}
}
