// Package game — consumable.go
//
// A consumed item restores the buff factor of its loot default as a part of
// max life. Two paths consume one: item_use spends a unit of a consumable from
// the inventory, and a consume-on-pickup-heal world token is consumed by its
// collector (loot.go). Both apply the effect through consumeItem.
//
// Cross-process contract:
//
//	item_use     (client→server) — the item id of an owned consumable
//	item_use_ack (server→client) — accepted or the rejection reason; the
//	                               inventory and life arrive in the snapshot
//	item_consume (server→viewers) — who consumed which item, and its loot
//	                               behavior; the client plays its effect
//
// Caller MUST hold s.mu for every handler here (they run inside phaseInput).
package game

import "time"

// Rejection reasons echoed back on item_use_ack. Nothing is consumed on a rejection.
const (
	itemUseRejectDead          = "dead"
	itemUseRejectNotOwned      = "not_owned"
	itemUseRejectNotConsumable = "not_consumable"
	itemUseRejectCooldown      = "cooldown"
	itemUseRejectFullLife      = "full_life"
)

// isConsumable reports whether itemID is an item of the consumable type: the
// items item_use spends.
func (s *GameServer) isConsumable(itemID string) bool {
	return s.itemType(itemID) == "consumable"
}

// canConsume reports whether a consumable changes anything for the consumer: a
// heal needs missing life.
func canConsume(consumer *Mortal) bool {
	return consumer.Life < consumer.MaxLife
}

// consumeItem applies the effect of one unit of itemID to the consumer: it
// restores the buff factor of the item's loot default as a part of max life.
func (s *GameServer) consumeItem(mapState *MapState, base *EntityBase, consumer *Mortal, itemID string) {
	build, _ := s.lootBuild(itemID)
	restoreLife(mapState, consumer, base.Pos, buffFactor(build, itemID)*consumer.MaxLife)
	event := ItemConsume{EntityID: base.ID, ItemID: itemID, Behavior: build.Behavior}
	broadcastAt(mapState, "item_consume", event, base.Pos.X, base.Pos.Y)
}

// itemUseRejection is why the player cannot use itemID now, or "".
func (s *GameServer) itemUseRejection(player *PlayerState, itemID string, now time.Time) string {
	switch {
	case player.IsGhost():
		return itemUseRejectDead
	case s.playerItemQuantity(player, itemID) < 1:
		return itemUseRejectNotOwned
	case !s.isConsumable(itemID):
		return itemUseRejectNotConsumable
	case now.Before(player.NextItemUseAt):
		return itemUseRejectCooldown
	case !canConsume(&player.Mortal):
		return itemUseRejectFullLife
	}
	return ""
}

// handleItemUse consumes one unit of an owned consumable. The unit leaves the
// inventory only once its effect applied, and the action cooldown rule starts
// the next use, so a repeated request inside it is rejected.
func (s *GameServer) handleItemUse(player *PlayerState, mapState *MapState, cmd *InputCommand) {
	now := time.Now()
	reason := s.itemUseRejection(player, cmd.ItemID, now)
	if reason == "" {
		s.consumeItem(mapState, &player.EntityBase, &player.Mortal, cmd.ItemID)
		s.removePlayerItem(player, cmd.ItemID, 1)
		player.NextItemUseAt = now.Add(s.CalculateActionCooldown(s.CalculateStats(player, mapState)))
	}
	sendAck(player, "item_use_ack", reason, map[string]interface{}{"itemId": cmd.ItemID})
}
