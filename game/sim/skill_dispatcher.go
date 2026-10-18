package sim

import (
	"cyberia-server/game/content"
	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// SkillContext carries all information needed by a skill handler.
// It unifies player and bot execution into a single call signature.
type SkillContext struct {
	// CasterID is the player or bot that triggered the skill.
	CasterID world.EntityID
	// MapState is the map the caster belongs to.
	MapState *world.MapState
	// Target is the world-space point the caster acted upon (tap destination).
	Target world.Vec2
	// TriggerItemID is the ObjectLayer item that matched the skillConfig entry.
	TriggerItemID string
	// SummonedEntityItemID is the item ID of the entity this skill should spawn
	// (e.g. "atlas_pistol_mk2_bullet", "hatchet-skill"). Comes from SkillDefinition.
	SummonedEntityItemID string
	// CallerHoldsLock is true when the server mutex is already held by the caller
	// (e.g. inside the game-loop / updateBots). Skill implementations that need
	// the lock must check this flag to avoid a deadlock.
	CallerHoldsLock bool
}

// SkillHandlerFunc is the signature every skill implementation must satisfy.
type SkillHandlerFunc func(s *Sim, ctx SkillContext)

// skillRegistry maps logicEventID → handler. New skills only need to add one
// entry in init.
var skillRegistry = map[string]SkillHandlerFunc{}

// heldSkillRegistry maps logicEventID → handler the simulation runs every tick on each item an
// entity holds, active or not. A tap never dispatches these.
var heldSkillRegistry = map[string]SkillHandlerFunc{}

// registerSkill adds (or replaces) a handler for the given logicEventID.
func registerSkill(logicEventID string, handler SkillHandlerFunc) {
	skillRegistry[logicEventID] = handler
}

// registerHeldSkill adds (or replaces) a held-item handler for the given logicEventID.
func registerHeldSkill(logicEventID string, handler SkillHandlerFunc) {
	heldSkillRegistry[logicEventID] = handler
}

// IsHeldSkill reports whether logicEventID runs on held items instead of on a tap.
func IsHeldSkill(logicEventID string) bool {
	_, ok := heldSkillRegistry[logicEventID]
	return ok
}

// SupportsSkill reports whether this server runs a handler for logicEventID.
func SupportsSkill(logicEventID string) bool {
	_, tap := skillRegistry[logicEventID]
	return tap || IsHeldSkill(logicEventID)
}

// init wires the built-in skill handlers into the registries.
func init() {
	registerSkill("projectile", func(s *Sim, ctx SkillContext) {
		s.executeProjectileSkill(ctx)
	})
	registerSkill("doppelganger", func(s *Sim, ctx SkillContext) {
		s.executeDoppelgangerSkill(ctx)
	})
	registerSkill("coin_drop_or_transaction", func(s *Sim, ctx SkillContext) {
		// Economy mechanic: every kill scatters the victim's coin stake as a
		// grid drop (handleBotDeath / handlePlayerDeath → spawnDrops). No
		// manual action needed.
	})
	registerHeldSkill("zero-waste-auto-consume-heal", func(s *Sim, ctx SkillContext) {
		s.executeZeroWasteHeal(ctx)
	})
}

// DispatchSkill routes a single logicEventID to its registered handler.
// Unknown IDs are logged and silently ignored.
func (s *Sim) DispatchSkill(logicID string, ctx SkillContext) {
	handler, ok := skillRegistry[logicID]
	if !ok {
		logx.Debugf("Unknown logicEventID '%s' for item '%s'", logicID, ctx.TriggerItemID)
		return
	}
	handler(s, ctx)
}

// dispatchSkillsForEntity iterates the entity's active ObjectLayers, looks up
// matching entries in skillConfig, and dispatches each logicEventID.
// This replaces the duplicated switch blocks in HandlePlayerTapAction and handleBotSkills.
func (s *Sim) dispatchSkillsForEntity(casterID world.EntityID, mapState *world.MapState, target world.Vec2, callerHoldsLock bool) {
	for _, layer := range mapState.Ap(casterID).Layers {
		if !layer.Active {
			continue
		}
		if skillDefs, ok := s.SkillConfig[layer.ItemID]; ok {
			for _, skillDef := range skillDefs {
				s.DispatchSkill(skillDef.LogicEventID, SkillContext{
					CasterID:             casterID,
					MapState:             mapState,
					Target:               target,
					TriggerItemID:        layer.ItemID,
					SummonedEntityItemID: skillDef.SummonedEntityItemID,
					CallerHoldsLock:      callerHoldsLock,
				})
			}
		}
	}
}

// dispatchHeldSkills runs the held-item skills of every item the caster holds, active or not, once
// per item however many stacks hold it. A handler may spend units, so each item is read from the
// live layers. Caller holds s.mu.
func (s *Sim) dispatchHeldSkills(casterID world.EntityID, mapState *world.MapState) {
	ran := make(map[string]bool)
	for i := 0; i < len(mapState.Ap(casterID).Layers); i++ {
		layer := mapState.Ap(casterID).Layers[i]
		if ran[layer.ItemID] || content.LayerQuantity(mapState.Ap(casterID).Layers, layer.ItemID) < 1 {
			continue
		}
		ran[layer.ItemID] = true
		for _, skillDef := range s.SkillConfig[layer.ItemID] {
			if handler, ok := heldSkillRegistry[skillDef.LogicEventID]; ok {
				handler(s, SkillContext{
					CasterID:             casterID,
					MapState:             mapState,
					TriggerItemID:        layer.ItemID,
					SummonedEntityItemID: skillDef.SummonedEntityItemID,
					CallerHoldsLock:      true,
				})
			}
		}
	}
}
