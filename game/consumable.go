// Package game — consumable.go
//
// A consumable heals its resistance as life. Two paths consume one: a collector
// that misses life consumes a consume-on-pickup-heal world token at once
// (loot.go), and the zero-waste-auto-consume-heal skill consumes a held unit by
// itself once the missing life takes the whole heal. Both apply the effect
// through consumeItem.
//
// Cross-process contract:
//
//	item_consume (server→viewers) — who consumed which item, and its loot
//	                               behavior; the client plays its effect
//
// Caller MUST hold s.mu for every function here (they run inside the tick).
package game

import "time"

// consumableHeal is the life one unit of itemID restores: its resistance, at the stat scale.
func (s *GameServer) consumableHeal(itemID string) float64 {
	data, ok := s.GetObjectLayerData(itemID)
	if !ok {
		return 0
	}
	return float64(data.Data.Stats.Resistance) * statScales.Resistance
}

// canConsume reports whether a consumable changes anything for the consumer: a
// heal needs missing life.
func canConsume(consumer *Vitals) bool {
	return consumer.Life < consumer.MaxLife
}

// consumeItem applies the effect of one unit of itemID to the consumer: it
// restores the item's heal, never past max life, and tells every viewer.
func (s *GameServer) consumeItem(mapState *MapState, id EntityID, itemID string) {
	pos := mapState.tf(id).Pos
	restoreLife(mapState, mapState.vt(id), pos, s.consumableHeal(itemID))
	build, _ := s.lootBuild(itemID)
	event := ItemConsume{EntityID: id, ItemID: itemID, Behavior: build.Behavior}
	broadcastAt(mapState, "item_consume", event, pos.X, pos.Y)
}

// executeZeroWasteHeal consumes one held unit of the trigger item once the
// caster's missing life takes its whole heal and the consume cooldown is over.
// The action cooldown rule times the next one.
func (s *GameServer) executeZeroWasteHeal(ctx SkillContext) {
	base, mortal := casterBody(ctx.Caster)
	if base == nil {
		return
	}
	now := time.Now()
	heal := s.consumableHeal(ctx.TriggerItemID)
	vitals := ctx.MapState.vt(base.ID)
	if vitals.ghost() || now.Before(mortal.NextConsumeAt) || heal <= 0 || vitals.MaxLife-vitals.Life < heal {
		return
	}
	if layerQuantity(ctx.MapState.ap(base.ID).Layers, ctx.TriggerItemID) < 1 {
		return
	}
	s.consumeItem(ctx.MapState, base.ID, ctx.TriggerItemID)
	removeLayerItem(&ctx.MapState.ap(base.ID).Layers, ctx.TriggerItemID, 1)
	mortal.NextConsumeAt = now.Add(s.CalculateActionCooldown(s.CalculateStats(ctx.Caster, ctx.MapState)))
}
