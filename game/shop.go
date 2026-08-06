// Package game — shop.go
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
package game

import (
	"cyberia-server/logx"
)

// Rejection reasons echoed back on shop_ack. The client maps them to a message.
const (
	shopRejectNoVendor     = "no_vendor"
	shopRejectNotForSale   = "not_for_sale"
	shopRejectOutOfRange   = "out_of_range"
	shopRejectInsufficient = "insufficient_funds"
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
func (s *GameServer) botHasShop(bot *BotState) bool {
	if bot == nil || bot.IsGhost() {
		return false
	}
	action := s.actionCache[bot.ID]
	return action != nil && len(action.ShopItems) > 0
}

// shopPrice normalises a catalog row's price: an unset currency defaults to
// coins (matching the CyberiaAction schema default) and a missing quantity to
// free. Returns the currency item id and how many of it the purchase costs.
func (s *GameServer) shopPrice(item *ActionShopItem) (string, int) {
	priceItemID := item.PriceItemID
	if priceItemID == "" {
		priceItemID = s.coinItemID
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
	if requested < 1 {
		return 1
	}
	if requested > shopBuyMaxQty {
		return shopBuyMaxQty
	}
	return requested
}

// handleShopBuy is the authoritative purchase path: validate the vendor, the
// catalog row, and the player's balance, then move the price out and the items
// in as one mutation. The updated inventory reaches the client through the next
// AOI self-player block; shop_ack only tells the presentation layer whether the
// transaction landed.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleShopBuy(player *PlayerState, cmd *InputCommand) {
	if player.IsGhost() {
		return
	}
	qty := shopBuyQuantity(cmd.Quantity)
	bot := s.findBot(cmd.EntityID)
	if bot == nil {
		s.sendShopAck(player, cmd.EntityID, cmd.ItemID, 0, shopRejectNoVendor)
		return
	}
	if !botInPlayerRange(player, bot) {
		s.sendShopAck(player, cmd.EntityID, cmd.ItemID, 0, shopRejectOutOfRange)
		return
	}
	item := shopItemFor(s.actionCache[bot.ID], cmd.ItemID)
	if item == nil {
		s.sendShopAck(player, cmd.EntityID, cmd.ItemID, 0, shopRejectNotForSale)
		return
	}
	// Trading is an interaction: assert the modal protection here rather than
	// trusting the client to still hold it, so a purchase can never resolve
	// while the buyer is exposed to damage.
	s.holdProviderFreeze(player, bot.ID)

	// All-or-nothing: a partial fill would silently charge for fewer units than
	// the player confirmed in the quantity picker.
	priceItemID, unitPrice := s.shopPrice(item)
	total := unitPrice * qty
	if s.playerItemQuantity(player, priceItemID) < total {
		s.sendShopAck(player, cmd.EntityID, cmd.ItemID, 0, shopRejectInsufficient)
		return
	}

	s.removePlayerItem(player, priceItemID, total)
	s.addPlayerItem(player, item.ItemID, qty)
	// A purchase is an inventory gain like any other — reconcile collect
	// objectives so a quest step satisfied by buying advances immediately.
	s.advancePlayerQuestsOnGain(player)
	s.sendShopAck(player, cmd.EntityID, item.ItemID, qty, "")

	logx.Debugf("[SHOP] player %s bought %dx %q from %s for %d %s",
		player.ID, qty, item.ItemID, bot.ID, total, priceItemID)
}

// sendShopAck notifies the buyer of the outcome. An empty reason means the
// purchase was applied; anything else is a rejection code the client renders.
func (s *GameServer) sendShopAck(player *PlayerState, entityID, itemID string, quantity int, reason string) {
	sendMessage(player, "shop_ack", map[string]interface{}{
		"entityId": entityID,
		"itemId":   itemID,
		"quantity": quantity,
		"ok":       reason == "",
		"reason":   reason,
	})
}
