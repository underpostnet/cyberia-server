package sim

import (
	"math/rand"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// executeDoppelgangerSkill is the "doppelganger" skill handler.
// It has a chance to spawn a temporary, wandering clone of the caster.
// Works for players and bots alike: the context names the caster by id.
func (s *Sim) executeDoppelgangerSkill(ctx SkillContext) {
	casterStats := s.CalculateStats(ctx.CasterID, ctx.MapState)

	if rand.Float64() >= s.SummonChance(s.doppelgangerSpawnChance, casterStats) {
		return
	}

	casterID := ctx.CasterID
	casterPos := ctx.MapState.Tf(casterID).Pos
	casterDims := ctx.MapState.Tf(casterID).Size
	casterLayers := ctx.MapState.Ap(casterID).Layers

	// Acquire the lock only if the caller doesn't already hold it.
	if !ctx.CallerHoldsLock {
		s.Mu.Lock()
		defer s.Mu.Unlock()
	}

	botLifetime := content.SummonLifetime(s.doppelgangerLifetimeMs, casterStats)

	// Find the caster's active skin layer. Fall back to the first active layer if no skin is found.
	var doppelgangerLayers []world.ObjectLayerState
	var fallbackLayer *world.ObjectLayerState
	for i := range casterLayers {
		layer := &casterLayers[i]
		if !layer.Active {
			continue
		}
		if fallbackLayer == nil {
			fallbackLayer = layer
		}
		if itemData, ok := s.GetObjectLayerData(layer.ItemID); ok {
			if itemData.Data.Item.Type == "skin" {
				doppelgangerLayers = []world.ObjectLayerState{{
					ItemID:   layer.ItemID,
					Active:   true,
					Quantity: 1,
				}}
				break
			}
		}
	}
	if len(doppelgangerLayers) == 0 && fallbackLayer != nil {
		doppelgangerLayers = []world.ObjectLayerState{{
			ItemID:   fallbackLayer.ItemID,
			Active:   true,
			Quantity: 1,
		}}
	}

	doppelgangerBot := s.NewEntityID()
	brain := &world.Brain{
		SpawnCenter: casterPos,
		SpawnRadius: s.doppelgangerSpawnRadius,
		ExpiresAt:   time.Now().Add(botLifetime),
	}

	ctx.MapState.AddBrain(doppelgangerBot, brain)
	ctx.MapState.Spawn(doppelgangerBot, world.EntityData{Kind: world.EntityBot}, world.Transform{Pos: casterPos, Size: casterDims}, world.Appearance{Layers: doppelgangerLayers})
	ctx.MapState.AddVitals(doppelgangerBot, world.Vitals{MaxLife: s.EntityBaseMaxLife, Life: s.EntityBaseMaxLife * s.doppelgangerInitialLifeFraction})
	ctx.MapState.AddMotion(doppelgangerBot, world.Motion{})
	ctx.MapState.AddAgent(doppelgangerBot, world.Agent{Behavior: content.BehaviorPassive, CasterID: casterID})
	ctx.MapState.AddMortal(doppelgangerBot, &world.Mortal{})
	s.ApplyResistanceStat(doppelgangerBot, ctx.MapState)
	ctx.MapState.Vt(doppelgangerBot).Life = ctx.MapState.Vt(doppelgangerBot).MaxLife
}
