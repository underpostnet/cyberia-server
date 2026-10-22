// Package content — transfer.go
//
// One change of the goods of a player that names stacks by place. A sale, a
// burn, a drop and a trade take units of chosen stacks: a slot of the bag or
// of the personal storage, with its definition. ApplyTransfer takes them, moves
// the coins and grants stacks into the bag as one change, or changes nothing
// and says why. A worn stack that leaves whole comes off when the equipment
// rules allow it; else the transfer is refused.
//
// Caller MUST hold s.mu.
package content

import (
	"slices"

	"cyberia-server/game/world"
)

// Place is one cell of a player container: a bag slot or a personal storage slot.
type Place struct {
	Box  world.Box
	Slot int
}

// Debit is units of the stack in one place.
type Debit struct {
	Place
	Qty int
}

// Transfer is one change of the goods of a player.
type Transfer struct {
	Takes []Debit
	// Coins is credited when positive and debited when negative.
	Coins int64
	// Gives enter the bag; a stack with a CID keeps it.
	Gives []Stack
}

// PlacedStack is the stack in a place, and whether it is worn. False when the
// place holds none.
func (s *Content) PlacedStack(player *world.Session, place Place) (Stack, bool, bool) {
	switch place.Box {
	case world.BoxBag:
		layers := *s.InventoryLayers(player)
		if at := s.bagStack(layers, place.Slot); at >= 0 {
			layer := layers[at]
			return Stack{ItemID: layer.ItemID, CID: layer.CID, Qty: layer.Quantity, Slot: layer.Slot}, layer.Active, true
		}
	case world.BoxStorage:
		stacks := s.PersonalStorage[player.PlayerID]
		if at := slotStack(stacks, place.Slot); at >= 0 {
			return stacks[at], false, true
		}
	}
	return Stack{}, false, false
}

// transferPlan is the bag, the personal storage and the wallet change a
// transfer leaves. Nothing changes until commitTransfer.
type transferPlan struct {
	bag     []world.ObjectLayerState
	storage []Stack
	coins   int64
}

// planTransfer computes what t leaves of the goods of the player, or the
// refusal.
func (s *Content) planTransfer(player *world.Session, t Transfer) (transferPlan, string) {
	bag := slices.Clone(*s.InventoryLayers(player))
	storage := slices.Clone(s.PersonalStorage[player.PlayerID])
	for _, take := range t.Takes {
		if take.Qty < 1 {
			return transferPlan{}, RefuseNoStack
		}
		switch take.Box {
		case world.BoxBag:
			at := s.bagStack(bag, take.Slot)
			if at < 0 || bag[at].Quantity < take.Qty {
				return transferPlan{}, RefuseNoStack
			}
			if bag[at].Active && bag[at].Quantity == take.Qty && !s.bankableWhileWorn(player, bag[at].ItemID) {
				return transferPlan{}, RefuseEquipped
			}
			if bag[at].Quantity -= take.Qty; bag[at].Quantity == 0 {
				bag = slices.Delete(bag, at, at+1)
			}
		case world.BoxStorage:
			at := slotStack(storage, take.Slot)
			if at < 0 || storage[at].Qty < take.Qty {
				return transferPlan{}, RefuseNoStack
			}
			if storage[at].Qty -= take.Qty; storage[at].Qty == 0 {
				storage = slices.Delete(storage, at, at+1)
			}
		default:
			return transferPlan{}, RefuseNoStack
		}
	}
	switch {
	case int64(player.Coins)+t.Coins < 0:
		return transferPlan{}, RefuseInsufficient
	case t.Coins > s.coinRoom(player):
		return transferPlan{}, RefuseWalletFull
	}
	for _, give := range t.Gives {
		bag = s.stackInto(bag, give.ItemID, give.CID, give.Qty)
	}
	if !s.fitsBag(player, bag) {
		return transferPlan{}, RefuseFull
	}
	return transferPlan{bag: bag, storage: storage, coins: t.Coins}, ""
}

// commitTransfer makes a plan the goods of the player.
func (s *Content) commitTransfer(player *world.Session, plan transferPlan) {
	ms := s.Maps[player.MapCode]
	*s.InventoryLayers(player) = plan.bag
	if len(plan.storage) > 0 || len(s.PersonalStorage[player.PlayerID]) > 0 {
		if s.PersonalStorage == nil {
			s.PersonalStorage = make(map[world.PlayerID][]Stack)
		}
		s.PersonalStorage[player.PlayerID] = plan.storage
	}
	if plan.coins != 0 {
		s.AddCoins(player.ID, ms, int(plan.coins))
	}
	s.ApplyResistanceStat(player.ID, ms)
}

// ApplyTransfer applies t to the player, or changes nothing and returns the
// refusal.
func (s *Content) ApplyTransfer(player *world.Session, t Transfer) string {
	plan, reason := s.planTransfer(player, t)
	if reason == "" {
		s.commitTransfer(player, plan)
	}
	return reason
}
