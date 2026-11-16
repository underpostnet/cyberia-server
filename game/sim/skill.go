package sim

import (
	"math/rand"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// handleTapRegen rolls the regeneration of a tap at most once per action
// cooldown. It shares the period of the skills, not their cooldown.
func (s *Sim) handleTapRegen(id world.EntityID, mapState *world.MapState, now time.Time) {
	mortal := mapState.Mt(id)
	if now.Before(mortal.NextRegenAt) {
		return
	}
	mortal.NextRegenAt = now.Add(s.CalculateActionCooldown(s.CalculateStats(id, mapState)))
	s.handleProbabilisticRegen(id, mapState)
}

// castBotSkill attempts one manual skill of the active items of a bot, picked
// at random, toward target; at the target itself it aims along the facing.
func (s *Sim) castBotSkill(id world.EntityID, mapState *world.MapState, target world.Vec2) {
	var skillID string
	candidates := 0
	for _, layer := range mapState.Ap(id).Layers {
		if !layer.Active {
			continue
		}
		for _, def := range s.SkillConfig[layer.ItemID] {
			if content.SkillLogics[def.LogicEventID].Activation != content.SkillManual {
				continue
			}
			candidates++
			if rand.Intn(candidates) == 0 {
				skillID = def.SkillID
			}
		}
	}
	if candidates == 0 {
		return
	}
	pos := mapState.Tf(id).Pos
	aim := SkillAim{Mode: world.AimDirection, Direction: world.Vec2{X: target.X - pos.X, Y: target.Y - pos.Y}}
	if aim.Direction == (world.Vec2{}) {
		aim = SkillAim{Mode: world.AimFacing}
	}
	s.ActivateSkill(id, mapState, skillID, aim, time.Now())
}
