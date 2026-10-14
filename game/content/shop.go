// Package content — shop.go
//
// Authoritative vendor transactions. engine-cyberia owns the catalog: a
// CyberiaAction carrying `shopItems` makes the entity on its source cell a
// vendor, and the rows arrive with the world over gRPC (action.go). The client
// reads the same catalog by action code over REST and renders it; it never
// computes a balance or mutates an inventory.
//
// Cross-process contract:
//
//	shop_buy (client→server) — entity + item id the player tapped Buy on
//	shop_ack (server→client) — accepted/rejected, so the presentation layer can
//	                           play the purchase FX or surface the reason
//
// Caller MUST hold s.mu for every handler here (they run inside phaseInput).
package content

import (
	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// Rejection reasons echoed back on shop_ack. The client maps them to a message.
const (
	shopRejectNoVendor     = "no_vendor"
	shopRejectNotForSale   = "not_for_sale"
	shopRejectOutOfRange   = "out_of_range"
	shopRejectInsufficient = "insufficient_funds"
	shopRejectFull         = "inventory_full"
)

// shopBuyMaxQty caps a single purchase. Authoritative: the client offers the
// same range in its quantity picker, but the wire value is never trusted.
const shopBuyMaxQty = 10

// shopItemFor returns the catalog row an action sells under itemID, or nil.
func shopItemFor(action *CyberiaAction, itemID string) *ActionShopItem {
	if action == nil || itemID == "" {
		return nil
	}
	for i := range action.ShopItems {
		if action.ShopItems[i].ItemID == itemID {
			return &action.ShopItems[i]
		}
	}
	return nil
}

// botHasShop reports whether a bot is a live vendor — its bound action carries
// a non-empty catalog and it is not in its dead/respawn state. Feeds
// botHasUsableAction, so a shop advertises itself overhead with the same
// attention icon, particle orbit, and border as a pending action-talk.
//
// Caller MUST hold s.mu.
func (s *Content) botHasShop(id world.EntityID, ms *world.MapState) bool {
	if ms == nil || ms.Vt(id).Ghost() {
		return false
	}
	action := s.actionCache[id]
	return action != nil && len(action.ShopItems) > 0
}

// shopPrice normalises a catalog row's price: an unset currency defaults to
// coins (matching the CyberiaAction schema default) and a missing quantity to
// free. Returns the currency item id and how many of it the purchase costs.
func (s *Content) shopPrice(item *ActionShopItem) (string, int) {
	priceItemID := item.PriceItemID
	if priceItemID == "" {
		priceItemID = s.CoinItemID
	}
	qty := item.PriceQty
	if qty < 0 {
		qty = 0
	}
	return priceItemID, qty
}

// shopBuyQuantity clamps the requested unit count into [1, shopBuyMaxQty].
// A client that sends 0 (or an older client that sends no quantity at all)
// means one unit.
func shopBuyQuantity(requested int) int {
	return min(max(requested, 1), shopBuyMaxQty)
}

// HandleShopBuy is the authoritative purchase path: validate the vendor, the
// catalog row, and the player's balance, then move the price out and the items
// in as one mutation. The updated inventory reaches the client through the next
// AOI self-player block; shop_ack only tells the presentation layer whether the
// transaction landed.
//
// Caller MUST hold s.mu.
func (s *Content) HandleShopBuy(player *world.Session, cmd *world.InputCommand) {
	if s.Vt(player.ID, player.MapCode).Ghost() {
		return
	}
	qty := shopBuyQuantity(cmd.Quantity)
	id := cmd.EntityID
	ms := s.findBot(id)
	if ms == nil {
		s.sendShopAck(player, cmd.EntityID, cmd.ItemID, 0, shopRejectNoVendor)
		return
	}
	if !s.inPlayerView(player, id, ms) {
		s.sendShopAck(player, cmd.EntityID, cmd.ItemID, 0, shopRejectOutOfRange)
		return
	}
	item := shopItemFor(s.actionCache[id], cmd.ItemID)
	if item == nil {
		s.sendShopAck(player, cmd.EntityID, cmd.ItemID, 0, shopRejectNotForSale)
		return
	}
	// All-or-nothing: a partial fill would silently charge for fewer units than
	// the player confirmed in the quantity picker.
	priceItemID, unitPrice := s.shopPrice(item)
	total := unitPrice * qty
	if s.playerItemQuantity(player, priceItemID) < total {
		s.sendShopAck(player, cmd.EntityID, cmd.ItemID, 0, shopRejectInsufficient)
		return
	}
	if !s.ExchangeItems(player, []Stack{{ItemID: priceItemID, Qty: total}}, []Stack{{ItemID: item.ItemID, Qty: qty}}) {
		s.sendShopAck(player, cmd.EntityID, cmd.ItemID, 0, shopRejectFull)
		return
	}
	// A purchase is an inventory gain like any other — reconcile collect
	// objectives so a quest step satisfied by buying advances immediately.
	s.AdvancePlayerQuestsOnGain(player)
	s.sendShopAck(player, cmd.EntityID, item.ItemID, qty, "")

	logx.Debugf("[SHOP] player %s bought %dx %q from %s for %d %s",
		player.ID, qty, item.ItemID, id, total, priceItemID)
}

// sendShopAck notifies the buyer of the outcome. An empty reason means the
// purchase was applied; anything else is a rejection code the client renders.
func (s *Content) sendShopAck(player *world.Session, entityID world.EntityID, itemID string, quantity int, reason string) {
	world.SendAck(player, "shop_ack", reason, map[string]interface{}{
		"entityId": entityID,
		"itemId":   itemID,
		"quantity": quantity,
	})
}
