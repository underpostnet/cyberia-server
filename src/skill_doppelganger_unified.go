package game

import (
	"math"
	"math/rand"
	"time"

	"github.com/google/uuid"
)

// executeDoppelgangerSkill is the unified "doppelganger" handler.
// It has a chance to spawn a temporary, wandering clone of the caster.
// Works for both PlayerState and BotState via SkillContext.
func (s *GameServer) executeDoppelgangerSkill(ctx SkillContext) {
	casterStats := s.CalculateStats(ctx.Caster, ctx.MapState)

	// Intelligence stat increases the chance of the skill activating.
	if rand.Float64() >= math.Min(s.doppelgangerSpawnChance+(casterStats.Intelligence/100.0), s.maxChance) {
		return
	}

	// Extract caster properties via type switch.
	var casterID, mapCode string
	var casterPos Point
	var casterDims Dimensions
	var casterColor ColorRGBA
	var casterLayers []ObjectLayerState
	switch e := ctx.Caster.(type) {
	case *PlayerState:
		casterID = e.ID
		mapCode = e.MapCode
		casterPos = e.Pos
		casterDims = e.Dims
		casterColor = e.Color
		casterLayers = e.ObjectLayers
	case *BotState:
		casterID = e.ID
		mapCode = e.MapCode
		casterPos = e.Pos
		casterDims = e.Dims
		casterColor = e.Color
		casterLayers = e.ObjectLayers
	}

	// Acquire the lock only if the caller doesn't already hold it.
	if !ctx.CallerHoldsLock {
		s.mu.Lock()
		defer s.mu.Unlock()
	}

	// Range stat increases the doppelganger's lifetime in milliseconds.
	botLifetime := (time.Duration(s.doppelgangerLifetimeMs) * time.Millisecond) +
		(time.Duration(casterStats.Range) * time.Millisecond)

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
		ID:           uuid.New().String(),
		MapCode:      mapCode,
		Pos:          casterPos,
		Dims:         casterDims,
		Behavior:     "passive",
		SpawnCenter:  casterPos,
		SpawnRadius:  s.doppelgangerSpawnRadius,
		ObjectLayers: doppelgangerLayers,
		ExpiresAt:    time.Now().Add(botLifetime),
		CasterID:     casterID,
		MaxLife:      s.entityBaseMaxLife,
		Life:         s.entityBaseMaxLife * s.doppelgangerInitialLifeFraction,
		Color:        casterColor,
	}

	ctx.MapState.bots[doppelgangerBot.ID] = doppelgangerBot
	s.ApplyResistanceStat(doppelgangerBot, ctx.MapState)
	doppelgangerBot.Life = doppelgangerBot.MaxLife
}
