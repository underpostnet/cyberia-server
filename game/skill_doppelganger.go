package game

import (
	"math/rand"
	"time"
)

// executeDoppelgangerSkill is the "doppelganger" skill handler.
// It has a chance to spawn a temporary, wandering clone of the caster.
// Works for both PlayerState and BotState via SkillContext.
func (s *GameServer) executeDoppelgangerSkill(ctx SkillContext) {
	casterStats := s.CalculateStats(ctx.Caster, ctx.MapState)

	if rand.Float64() >= s.summonChance(s.doppelgangerSpawnChance, casterStats) {
		return
	}

	// Extract caster properties via type switch.
	var casterID EntityID
	var mapCode string
	var casterPos Vec2
	var casterDims Vec2
	var casterLayers []ObjectLayerState
	switch e := ctx.Caster.(type) {
	case *PlayerState:
		casterID = e.ID
		mapCode = e.MapCode
		casterPos = ctx.MapState.tf(e.ID).Pos
		casterDims = ctx.MapState.tf(e.ID).Size
		casterLayers = ctx.MapState.ap(e.ID).Layers
	case *BotState:
		casterID = e.ID
		mapCode = e.MapCode
		casterPos = ctx.MapState.tf(e.ID).Pos
		casterDims = ctx.MapState.tf(e.ID).Size
		casterLayers = ctx.MapState.ap(e.ID).Layers
	}

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

	doppelgangerBot := &BotState{
		EntityBase: EntityBase{
			ID: s.newEntityID(),
		},
		MapCode:     mapCode,
		Behavior:    BehaviorPassive,
		SpawnCenter: casterPos,
		SpawnRadius: s.doppelgangerSpawnRadius,
		ExpiresAt:   time.Now().Add(botLifetime),
		CasterID:    casterID,
	}

	ctx.MapState.bots[doppelgangerBot.ID] = doppelgangerBot
	ctx.MapState.spawn(doppelgangerBot.ID, EntityData{Kind: EntityBot}, Transform{Pos: casterPos, Size: casterDims}, Appearance{Layers: doppelgangerLayers})
	ctx.MapState.addVitals(doppelgangerBot.ID, Vitals{MaxLife: s.entityBaseMaxLife, Life: s.entityBaseMaxLife * s.doppelgangerInitialLifeFraction})
	ctx.MapState.addMotion(doppelgangerBot.ID, Motion{})
	s.ApplyResistanceStat(doppelgangerBot, ctx.MapState)
	ctx.MapState.vt(doppelgangerBot.ID).Life = ctx.MapState.vt(doppelgangerBot.ID).MaxLife
}
