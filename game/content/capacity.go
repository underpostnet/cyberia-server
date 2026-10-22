// Package content — capacity.go
//
// Stack capacity of the player containers. The stat contract generates both
// rules, InventoryCapacity and StorageCapacity, from engine-cyberia
// SharedDefaultsCyberia.js. The level comes from the server's own progression.
//
// A slot holds one stack: one item id at any quantity. More of a held item
// joins its first stack; a new item needs an open slot. Coins never take a
// slot. A running assembly holds the slots of its ingredients and of its
// outputs, so its payout and its refund always fit.
//
// Caller MUST hold s.mu.
package content

import (
	"slices"

	"cyberia-server/game/world"
)

// Stack is one item id at a quantity. Slot is its cell in a container; an
// amount to grant or take has none.
type Stack struct {
	ItemID string `json:"itemId"`
	Qty    int    `json:"qty"`
	Slot   int    `json:"slot"`
	// CID is the definition a stored stack was granted with.
	CID string `json:"-"`
}

// InventoryLayers is the carried inventory of a player. While it waits to
// respawn that is the loadout the respawn restores, not the dead visuals.
func (s *Content) InventoryLayers(player *world.Session) *[]world.ObjectLayerState {
	ms := s.Maps[player.MapCode]
	if ms.Vt(player.ID).Ghost() {
		return &ms.Mt(player.ID).PreRespawnObjectLayers
	}
	return &ms.Ap(player.ID).Layers
}

func (s *Content) playerLevel(player *world.Session) int {
	return s.EntityProgression(player.ID, s.Maps[player.MapCode]).Level
}

// InventorySlots is the open inventory slot count of the player.
func (s *Content) InventorySlots(player *world.Session) int {
	return InventoryCapacity.Slots(s.playerLevel(player))
}

// StorageSlots is the open personal storage slot count of the player.
func (s *Content) StorageSlots(player *world.Session) int {
	return StorageCapacity.Slots(s.playerLevel(player))
}

// takesSlot reports whether a stack of itemID takes an inventory slot.
func (s *Content) takesSlot(itemID string) bool {
	return itemID != "" && itemID != s.CoinItemID && itemID != s.GhostItemID
}

// exchanged is the bag once takes leave it and gives join it, and the change of
// the wallet.
func (s *Content) exchanged(player *world.Session, takes, gives []Stack) ([]world.ObjectLayerState, int) {
	bag := slices.Clone(*s.InventoryLayers(player))
	coins := 0
	for _, take := range takes {
		if take.ItemID == s.CoinItemID {
			coins -= take.Qty
		} else {
			RemoveLayerItem(&bag, take.ItemID, take.Qty)
		}
	}
	for _, give := range gives {
		switch {
		case give.ItemID == "" || give.Qty <= 0:
		case give.ItemID == s.CoinItemID:
			coins += give.Qty
		default:
			bag = s.stackInto(bag, give.ItemID, give.CID, give.Qty)
		}
	}
	return bag, coins
}

// InventoryFits reports whether the player can hold gives once takes leave
// its inventory. A player over capacity may trade stacks but never add one.
func (s *Content) InventoryFits(player *world.Session, takes, gives []Stack) bool {
	bag, _ := s.exchanged(player, takes, gives)
	return s.fitsBag(player, bag)
}

// ExchangeItems is the one path that adds items to a player: it removes takes
// and adds gives as one change, or changes nothing. It refuses when the player
// lacks a take, the result does not fit the inventory capacity, or a credit
// passes the wallet maximum.
func (s *Content) ExchangeItems(player *world.Session, takes, gives []Stack) bool {
	return s.exchangeRefusal(player, takes, gives) == ""
}

// exchangeRefusal applies ExchangeItems and says why it changed nothing, or "".
func (s *Content) exchangeRefusal(player *world.Session, takes, gives []Stack) string {
	for _, take := range takes {
		if take.Qty > 0 && s.playerItemQuantity(player, take.ItemID) < take.Qty {
			if take.ItemID == s.CoinItemID {
				return RefuseInsufficient
			}
			return RefuseNoStack
		}
	}
	bag, coins := s.exchanged(player, takes, gives)
	if int64(coins) > s.coinRoom(player) {
		return RefuseWalletFull
	}
	if !s.fitsBag(player, bag) {
		return RefuseFull
	}
	*s.InventoryLayers(player) = bag
	if coins != 0 {
		s.AddCoins(player.ID, s.Maps[player.MapCode], coins)
	}
	return ""
}

// removePlayerItem deducts qty of itemID from the carried inventory. Coins
// route through the flat balance; other items drain matching ObjectLayer rows.
func (s *Content) removePlayerItem(player *world.Session, itemID string, qty int) {
	if qty <= 0 {
		return
	}
	if itemID == s.CoinItemID {
		s.AddCoins(player.ID, s.Maps[player.MapCode], -qty)
		return
	}
	RemoveLayerItem(s.InventoryLayers(player), itemID, qty)
}

// playerItemQuantity returns how many of itemID the player carries. Coins
// route through the flat balance; everything else sums matching rows.
func (s *Content) playerItemQuantity(player *world.Session, itemID string) int {
	if itemID == s.CoinItemID {
		return int(player.Coins)
	}
	return LayerQuantity(*s.InventoryLayers(player), itemID)
}
