package sim

import (
	"math/rand"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// SkillContext is what one skill handler runs on. Caller holds s.mu.
type SkillContext struct {
	CasterID world.EntityID
	MapState *world.MapState
	// Aim is the unit vector a directional skill flies along.
	Aim world.Vec2
	// TriggerItemID is the item that carries the skill.
	TriggerItemID string
	// SummonedEntityItemID is the item of the entity the skill spawns.
	SummonedEntityItemID string
}

// skillHandler runs one skill logic. chance is the base chance of a manual
// skill before intelligence; an automatic skill has none. reach is the cells a
// directional skill flies, the range of its auto aim.
type skillHandler struct {
	chance func(s *Sim) float64
	reach  func(s *Sim, stats content.ComputedStats) float64
	run    func(s *Sim, ctx SkillContext)
}

// skillHandlers holds the handler of each manual and automatic logic of
// content.SkillLogics. An event logic runs in its gameplay event: the coin
// drop runs in the death drops.
var skillHandlers = map[string]skillHandler{
	"projectile": {
		chance: func(s *Sim) float64 { return s.projectileSpawnChance },
		reach: func(s *Sim, stats content.ComputedStats) float64 {
			return s.EntityBaseSpeed * s.projectileSpeedMultiplier * content.SummonLifetime(s.projectileLifetimeMs, stats).Seconds()
		},
		run: (*Sim).executeProjectileSkill,
	},
	"doppelganger": {
		chance: func(s *Sim) float64 { return s.doppelgangerSpawnChance },
		run:    (*Sim).executeDoppelgangerSkill,
	},
	"zero-waste-auto-consume-heal": {run: (*Sim).executeZeroWasteHeal},
}

// skillRoll draws the chance roll of one skill activation.
var skillRoll = rand.Float64

// SupportsSkill reports whether this server runs logicEventID.
func SupportsSkill(logicEventID string) bool {
	logic, known := content.SkillLogics[logicEventID]
	_, handled := skillHandlers[logicEventID]
	return known && (handled || logic.Activation == content.SkillEvent)
}

// BaseSkillReach is the reach of a directional skill logic with no stat, 0 for
// a logic that aims at no direction.
func (s *Sim) BaseSkillReach(logicEventID string) float64 {
	if reach := skillHandlers[logicEventID].reach; reach != nil {
		return reach(s, content.ComputedStats{})
	}
	return 0
}

// SkillOutcome is the answer of the world to one skill activation.
type SkillOutcome string

const (
	SkillCast        SkillOutcome = ""            // the skill ran
	SkillMisfire     SkillOutcome = "misfire"     // the attempt was valid; its chance roll failed
	SkillCooldown    SkillOutcome = "cooldown"    // the action cooldown of the caster runs
	SkillUnavailable SkillOutcome = "unavailable" // no active item of the caster carries the manual skill
	SkillBlocked     SkillOutcome = "blocked"     // the caster is a ghost or in stasis
)

// SkillAim is how one activation aims.
type SkillAim struct {
	Mode      world.AimMode
	Direction world.Vec2     // AimDirection: never zero
	Preferred world.EntityID // AimAuto: the target to keep while it stays a candidate
}

// SkillResult is the answer to one activation: its outcome, the target an
// auto aim took (0 for none) and the unit vector a directional skill flew
// along (zero for a skill that aims at no direction).
type SkillResult struct {
	Outcome SkillOutcome
	Target  world.EntityID
	Aim     world.Vec2
}

// Accepted reports whether the attempt spent the action cooldown: a cast or a misfire.
func (r SkillResult) Accepted() bool { return r.Outcome == SkillCast || r.Outcome == SkillMisfire }

// ActivateSkill attempts the manual skill skillID of a caster. A valid attempt
// reserves the action cooldown, resolves its aim, and rolls the chance of the
// skill once; the skill runs on a pass. A refused attempt changes nothing.
// Caller holds s.mu.
func (s *Sim) ActivateSkill(id world.EntityID, mapState *world.MapState, skillID string, aim SkillAim, now time.Time) SkillResult {
	var result SkillResult
	if mapState.Vt(id).Ghost() {
		result.Outcome = SkillBlocked
		return result
	}
	trigger, def, ok := s.manualSkill(mapState, id, skillID)
	if !ok {
		result.Outcome = SkillUnavailable
		return result
	}
	mortal := mapState.Mt(id)
	if now.Before(mortal.NextSkillAt) {
		result.Outcome = SkillCooldown
		return result
	}
	stats := s.CalculateStats(id, mapState)
	mortal.NextSkillAt = now.Add(s.CalculateActionCooldown(stats))
	handler := skillHandlers[def.LogicEventID]
	if content.SkillLogics[def.LogicEventID].Targeting == content.TargetDirection {
		result.Target, result.Aim = s.resolveAim(mapState, id, aim, handler.reach(s, stats))
		// An idle caster turns to the facing nearest the cast.
		if mapState.Mo(id).Mode == world.IDLE && result.Aim != (world.Vec2{}) {
			mapState.Mo(id).Direction = directionFromVector(result.Aim.X, result.Aim.Y)
		}
	}
	if skillRoll() >= s.SummonChance(handler.chance(s), stats) {
		result.Outcome = SkillMisfire
		return result
	}
	handler.run(s, SkillContext{
		CasterID:             id,
		MapState:             mapState,
		Aim:                  result.Aim,
		TriggerItemID:        trigger,
		SummonedEntityItemID: def.SummonedEntityItemID,
	})
	result.Outcome = SkillCast
	return result
}

// resolveAim is the target an aim takes (0 for none) and the unit vector it
// flies along, at any angle. An auto aim flies at the center of its target; one
// with no target in reach, and a facing aim, fly along the facing.
func (s *Sim) resolveAim(ms *world.MapState, caster world.EntityID, aim SkillAim, reach float64) (world.EntityID, world.Vec2) {
	switch aim.Mode {
	case world.AimDirection:
		return 0, aim.Direction.Unit()
	case world.AimAuto:
		if target := s.autoTarget(ms, caster, aim.Preferred, reach); target != 0 {
			from, to := entityCenter(ms, caster), entityCenter(ms, target)
			if toward := (world.Vec2{X: to.X - from.X, Y: to.Y - from.Y}).Unit(); toward != (world.Vec2{}) {
				return target, toward
			}
		}
	}
	x, y := getDirectionVector(ms.Mo(caster).Direction)
	return 0, world.Vec2{X: x, Y: y}
}

// manualSkill finds the manual skill skillID on the active items of a caster,
// with the item that carries it. Two active items may carry one skill id only
// with one definition; a conflict refuses the skill.
func (s *Sim) manualSkill(mapState *world.MapState, id world.EntityID, skillID string) (string, content.SkillDefinition, bool) {
	var trigger string
	var found content.SkillDefinition
	for _, layer := range mapState.Ap(id).Layers {
		if !layer.Active || layer.Quantity < 1 {
			continue
		}
		for _, def := range s.SkillConfig[layer.ItemID] {
			if def.SkillID != skillID || content.SkillLogics[def.LogicEventID].Activation != content.SkillManual {
				continue
			}
			if trigger == "" {
				trigger, found = layer.ItemID, def
			} else if def.LogicEventID != found.LogicEventID || def.SummonedEntityItemID != found.SummonedEntityItemID {
				logx.Warnf("[Skills] %s and %s define skill %q two ways; it does not run", trigger, layer.ItemID, skillID)
				return "", content.SkillDefinition{}, false
			}
		}
	}
	return trigger, found, trigger != ""
}

// runAutomaticSkills runs the automatic skills of every item the caster holds,
// active or not, once per item however many stacks hold it. A handler may
// spend units, so each item is read from the live layers. Caller holds s.mu.
func (s *Sim) runAutomaticSkills(casterID world.EntityID, mapState *world.MapState) {
	ran := make(map[string]bool)
	for i := 0; i < len(mapState.Ap(casterID).Layers); i++ {
		layer := mapState.Ap(casterID).Layers[i]
		if ran[layer.ItemID] || content.LayerQuantity(mapState.Ap(casterID).Layers, layer.ItemID) < 1 {
			continue
		}
		ran[layer.ItemID] = true
		for _, def := range s.SkillConfig[layer.ItemID] {
			if content.SkillLogics[def.LogicEventID].Activation != content.SkillAutomatic {
				continue
			}
			skillHandlers[def.LogicEventID].run(s, SkillContext{
				CasterID:             casterID,
				MapState:             mapState,
				TriggerItemID:        layer.ItemID,
				SummonedEntityItemID: def.SummonedEntityItemID,
			})
		}
	}
}
