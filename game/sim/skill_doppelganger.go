package sim

import (
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// executeDoppelgangerSkill is the "doppelganger" skill handler. It spawns a
// temporary, wandering clone of the caster, a player or a bot.
func (s *Sim) executeDoppelgangerSkill(ctx SkillContext) {
	casterStats := s.CalculateStats(ctx.CasterID, ctx.MapState)

	casterID := ctx.CasterID
	casterPos := ctx.MapState.Tf(casterID).Pos
	casterDims := ctx.MapState.Tf(casterID).Size
	casterLayers := ctx.MapState.Ap(casterID).Layers

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
	ctx.MapState.AddAgent(doppelgangerBot, s.summonAgent(ctx.MapState, content.BehaviorPassive, casterID))
	ctx.MapState.AddMortal(doppelgangerBot, &world.Mortal{})
	s.ApplyResistanceStat(doppelgangerBot, ctx.MapState)
	ctx.MapState.Vt(doppelgangerBot).Life = ctx.MapState.Vt(doppelgangerBot).MaxLife
}
