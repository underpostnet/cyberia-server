package game

import (
	"math/rand"
	"time"
)

// executeDoppelgangerSkill is the "doppelganger" skill handler.
// It has a chance to spawn a temporary, wandering clone of the caster.
// Works for players and bots alike: the context names the caster by id.
func (s *Sim) executeDoppelgangerSkill(ctx SkillContext) {
	casterStats := s.CalculateStats(ctx.CasterID, ctx.MapState)

	if rand.Float64() >= s.summonChance(s.doppelgangerSpawnChance, casterStats) {
		return
	}

	casterID := ctx.CasterID
	casterPos := ctx.MapState.tf(casterID).Pos
	casterDims := ctx.MapState.tf(casterID).Size
	casterLayers := ctx.MapState.ap(casterID).Layers

	// Acquire the lock only if the caller doesn't already hold it.
	if !ctx.CallerHoldsLock {
		s.mu.Lock()
		defer s.mu.Unlock()
	}

	botLifetime := summonLifetime(s.doppelgangerLifetimeMs, casterStats)

	// Find the caster's active skin layer. Fall back to the first active layer if no skin is found.
	var doppelgangerLayers []ObjectLayerState
	var fallbackLayer *ObjectLayerState
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
				doppelgangerLayers = []ObjectLayerState{{
					ItemID:   layer.ItemID,
					Active:   true,
					Quantity: 1,
				}}
				break
			}
		}
	}
	if len(doppelgangerLayers) == 0 && fallbackLayer != nil {
		doppelgangerLayers = []ObjectLayerState{{
			ItemID:   fallbackLayer.ItemID,
			Active:   true,
			Quantity: 1,
		}}
	}

	doppelgangerBot := s.newEntityID()
	brain := &Brain{
		SpawnCenter: casterPos,
		SpawnRadius: s.doppelgangerSpawnRadius,
		ExpiresAt:   time.Now().Add(botLifetime),
	}

	ctx.MapState.addBrain(doppelgangerBot, brain)
	ctx.MapState.spawn(doppelgangerBot, EntityData{Kind: EntityBot}, Transform{Pos: casterPos, Size: casterDims}, Appearance{Layers: doppelgangerLayers})
	ctx.MapState.addVitals(doppelgangerBot, Vitals{MaxLife: s.entityBaseMaxLife, Life: s.entityBaseMaxLife * s.doppelgangerInitialLifeFraction})
	ctx.MapState.addMotion(doppelgangerBot, Motion{})
	ctx.MapState.addAgent(doppelgangerBot, Agent{Behavior: BehaviorPassive, CasterID: casterID})
	ctx.MapState.addMortal(doppelgangerBot, &Mortal{})
	s.ApplyResistanceStat(doppelgangerBot, ctx.MapState)
	ctx.MapState.vt(doppelgangerBot).Life = ctx.MapState.vt(doppelgangerBot).MaxLife
}
