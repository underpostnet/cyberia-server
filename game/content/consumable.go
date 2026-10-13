// Package content — consumable.go
//
// A consumable heals its resistance as life. Two paths consume one: a collector
// that misses life consumes a consume-on-pickup-heal world token at once
// (loot.go), and the zero-waste-auto-consume-heal skill consumes a held unit by
// itself once the missing life takes the whole heal. Both apply the effect
// through ConsumeItem.
//
// Cross-process contract:
//
//	item_consume (server→viewers) — who consumed which item, and its loot
//	                               behavior; the client plays its effect
//
// Caller MUST hold s.mu for every function here (they run inside the tick).
package content

import "cyberia-server/game/world"

// ConsumableHeal is the life one unit of itemID restores: its resistance, at the stat scale.
func (s *Content) ConsumableHeal(itemID string) float64 {
	data, ok := s.GetObjectLayerData(itemID)
	if !ok {
		return 0
	}
	return float64(data.Data.Stats.Resistance) * StatScales.Resistance
}

// canConsume reports whether a consumable changes anything for the consumer: a
// heal needs missing life.
func canConsume(consumer *world.Vitals) bool {
	return consumer.Life < consumer.MaxLife
}

// ConsumeItem applies the effect of one unit of itemID to the consumer: it
// restores the item's heal, never past max life, and tells every viewer.
func (s *Content) ConsumeItem(mapState *world.MapState, id world.EntityID, itemID string) {
	pos := mapState.Tf(id).Pos
	s.RestoreLife(mapState, mapState.Vt(id), pos, s.ConsumableHeal(itemID))
	build, _ := s.lootBuild(itemID)
	event := world.ItemConsume{EntityID: id, ItemID: itemID, Behavior: build.Behavior}
	s.broadcastAt(mapState, "item_consume", event, pos.X, pos.Y)
}

// RestoreLife adds up to amount life, never past MaxLife. Every AOI viewer sees the same green
// "+N" for the life actually restored, for players and bots alike.
func (s *Content) RestoreLife(mapState *world.MapState, v *world.Vitals, at world.Vec2, amount float64) {
	gained := min(v.Life+amount, v.MaxLife) - v.Life
	if gained <= 0 {
		return
	}
	v.Life += gained
	if shown := int(gained + 0.5); shown > 0 {
		s.BroadcastFCT(mapState, world.FCTRegen, at.X, at.Y, shown)
	}
}
