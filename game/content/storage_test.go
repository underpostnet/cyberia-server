package content

import (
	"testing"

	"cyberia-server/game/world"
	"cyberia-server/game/world/worldtest"
)

// An authored capacity is taken as-is and only clamped at the ceiling the
// client can render — the vault wraps into rows, so it need not be a square.
func TestStorageCapacityClampsToTheRenderableCeiling(t *testing.T) {
	cases := []struct{ authored, want int }{
		{-5, 0}, {0, 0}, {1, 1}, {7, 7}, {25, 25},
		{StorageMaxSlots, StorageMaxSlots},
		{StorageMaxSlots + 100, StorageMaxSlots},
	}
	for _, c := range cases {
		if got := storageCapacity(c.authored); got != c.want {
			t.Fatalf("storageCapacity(%d) = %d, want %d", c.authored, got, c.want)
		}
	}
}

// moveFixture places a level-1 player "p-1" whose stacks take the slots in order.
func moveFixture(layers ...world.ObjectLayerState) (*Content, *world.Session) {
	s := &Content{World: &world.World{}, CoinItemID: "coin"}
	s.SeatStacks(layers, 0)
	player := worldtest.PlacePlayer(s.World, 1, layers)
	player.PlayerID = "p-1"
	return s, player
}

func move(s *Content, player *world.Session, from world.Box, fromSlot int, to world.Box, toSlot, qty int) {
	s.HandleItemMove(player, &world.InputCommand{Move: world.ItemMove{From: from, FromSlot: fromSlot, To: to, ToSlot: toSlot, Qty: qty}})
}

// bagAt is the bag stack in slot; an empty one has no item.
func bagAt(s *Content, player *world.Session, slot int) world.ObjectLayerState {
	layers := *s.InventoryLayers(player)
	if at := s.bagStack(layers, slot); at >= 0 {
		return layers[at]
	}
	return world.ObjectLayerState{}
}

func storedAt(s *Content, player *world.Session, slot int) Stack {
	stacks := s.PlayerStorage(player)
	if at := slotStack(stacks, slot); at >= 0 {
		return stacks[at]
	}
	return Stack{}
}

// A move onto an empty slot splits the stack, and a move onto a stack of the
// same item stacks the units back. Each split stack takes its own slot.
func TestMoveSplitsAndStacksInsideTheBag(t *testing.T) {
	s, player := moveFixture(world.ObjectLayerState{ItemID: "gem", Quantity: 5})

	move(s, player, world.BoxBag, 0, world.BoxBag, 3, 2)
	if a, b := bagAt(s, player, 0), bagAt(s, player, 3); a.ItemID != "gem" || a.Quantity != 3 || b.ItemID != "gem" || b.Quantity != 2 {
		t.Fatalf("split = %+v and %+v, want gem 3 in slot 0 and gem 2 in slot 3", a, b)
	}
	if got := s.occupiedSlots(player, *s.InventoryLayers(player)); got != 2 {
		t.Fatalf("occupied = %d, want 2: each split stack takes a slot", got)
	}

	move(s, player, world.BoxBag, 3, world.BoxBag, 0, 2)
	if a, b := bagAt(s, player, 0), bagAt(s, player, 3); a.Quantity != 5 || b.ItemID != "" {
		t.Fatalf("stacked = %+v and %+v, want gem 5 in slot 0 and an empty slot 3", a, b)
	}
}

// Whole stacks of one box swap. Part of a stack never displaces another item,
// and a stack that leaves frees its slot while the other stacks stay put.
func TestMoveSwapsWholeStacksAndLeavesTheOthersInPlace(t *testing.T) {
	s, player := moveFixture(
		world.ObjectLayerState{ItemID: "gem", Quantity: 2},
		world.ObjectLayerState{ItemID: "ore", Quantity: 1},
		world.ObjectLayerState{ItemID: "herb", Quantity: 1},
	)

	move(s, player, world.BoxBag, 0, world.BoxBag, 1, 1)
	if bagAt(s, player, 0).Quantity != 2 || bagAt(s, player, 1).ItemID != "ore" {
		t.Fatal("part of a stack displaced another item")
	}
	move(s, player, world.BoxBag, 0, world.BoxBag, 1, 2)
	if bagAt(s, player, 0).ItemID != "ore" || bagAt(s, player, 1).ItemID != "gem" {
		t.Fatalf("swap = %+v, %+v", bagAt(s, player, 0), bagAt(s, player, 1))
	}

	move(s, player, world.BoxBag, 1, world.BoxStorage, 0, 2)
	if bagAt(s, player, 1).ItemID != "" || bagAt(s, player, 0).ItemID != "ore" || bagAt(s, player, 2).ItemID != "herb" {
		t.Fatal("a stack that leaves must free its slot and leave the others in place")
	}
}

// A slot outside the capacity takes nothing: a locked bag or storage slot, the
// wallet for an item, a bag slot for coins, and the storage for coins.
func TestMoveRefusesClosedSlotsAndMisplacedCoins(t *testing.T) {
	s, player := moveFixture(world.ObjectLayerState{ItemID: "gem", Quantity: 2})
	player.Coins = 30

	move(s, player, world.BoxBag, 0, world.BoxBag, InventoryCapacity.Slots(1), 1)
	move(s, player, world.BoxBag, 0, world.BoxStorage, StorageCapacity.Slots(1), 1)
	move(s, player, world.BoxBag, 0, world.BoxBag, world.WalletSlot, 1)
	move(s, player, world.BoxBag, world.WalletSlot, world.BoxBag, 2, 5)
	move(s, player, world.BoxBag, world.WalletSlot, world.BoxStorage, 0, 5)
	if bagAt(s, player, 0).Quantity != 2 || len(s.PlayerStorage(player)) != 0 || player.Coins != 30 {
		t.Fatalf("bag %+v, storage %+v, coins %d: every move must be refused", bagAt(s, player, 0), s.PlayerStorage(player), player.Coins)
	}
}

// A split needs a free slot: a slot an assembly holds for its payout is not
// free. A full bag refuses a new stack and still takes more of an item it holds.
func TestAFullBagRefusesNewStacksAndTakesMoreOfAHeldItem(t *testing.T) {
	open := InventoryCapacity.Slots(1)
	layers := make([]world.ObjectLayerState, 0, open)
	for i := range open - 1 {
		layers = append(layers, world.ObjectLayerState{ItemID: string(rune('a' + i)), Quantity: 2})
	}
	s, player := moveFixture(layers...)
	s.crafts = map[world.EntityID]*ActiveCraft{player.ID: {Outputs: []Stack{{ItemID: "tool", Qty: 1}}}}
	s.PersonalStorage = map[world.PlayerID][]Stack{"p-1": {{ItemID: "a", Qty: 4, Slot: 0}, {ItemID: "z", Qty: 1, Slot: 1}}}

	move(s, player, world.BoxBag, 0, world.BoxBag, open-1, 1)
	if bagAt(s, player, 0).Quantity != 2 || bagAt(s, player, open-1).ItemID != "" {
		t.Fatal("a split into the slot an assembly holds must be refused")
	}
	move(s, player, world.BoxStorage, 1, world.BoxBag, open-1, 1)
	if storedAt(s, player, 1).ItemID != "z" {
		t.Fatal("a full bag must refuse a new item")
	}
	move(s, player, world.BoxStorage, 0, world.BoxBag, 0, 4)
	if bagAt(s, player, 0).Quantity != 6 || storedAt(s, player, 0).ItemID != "" {
		t.Fatalf("bag %+v: a full bag still stacks more of a held item", bagAt(s, player, 0))
	}
}

// Units cross between the bag and a container onto an empty slot or onto a
// stack of the same item. Another item refuses them, and no unit is lost.
func TestMoveCarriesUnitsBetweenTheBagAndAContainer(t *testing.T) {
	s, player := moveFixture(world.ObjectLayerState{ItemID: "gem", Quantity: 5}, world.ObjectLayerState{ItemID: "ore", Quantity: 1})

	move(s, player, world.BoxBag, 0, world.BoxStorage, 4, 2)
	move(s, player, world.BoxBag, 0, world.BoxStorage, 4, 1)
	move(s, player, world.BoxBag, 1, world.BoxStorage, 4, 1)
	if got := storedAt(s, player, 4); got.ItemID != "gem" || got.Qty != 3 || bagAt(s, player, 0).Quantity != 2 || bagAt(s, player, 1).ItemID != "ore" {
		t.Fatalf("storage %+v, bag %+v %+v", got, bagAt(s, player, 0), bagAt(s, player, 1))
	}

	move(s, player, world.BoxStorage, 4, world.BoxBag, 5, 9)
	if got := bagAt(s, player, 5); got.ItemID != "gem" || got.Quantity != 3 || len(s.PlayerStorage(player)) != 0 {
		t.Fatalf("withdrawal = %+v: the whole stack lands in slot 5", got)
	}
}

// A worn stack keeps its flag when it moves whole inside the bag, and keeps it
// on the part that stays when it splits. Banking it whole takes it off; the
// last worn skin stays.
func TestAWornStackMovesUnderTheEquipmentRules(t *testing.T) {
	s, player := moveFixture(
		world.ObjectLayerState{ItemID: "anon", Active: true, Quantity: 1},
		world.ObjectLayerState{ItemID: "sword", Active: true, Quantity: 3},
	)
	s.EquipmentRules = EquipmentRulesConfig{RequireSkin: true}
	s.ObjectLayerDataCache = map[string]*ObjectLayer{"anon": {Data: ObjectLayerData{Item: Item{Type: "skin"}}}}

	move(s, player, world.BoxBag, 1, world.BoxBag, 4, 1)
	if !bagAt(s, player, 1).Active || bagAt(s, player, 4).Active {
		t.Fatal("a split must leave the worn part worn and the new stack unworn")
	}
	move(s, player, world.BoxBag, 1, world.BoxBag, 6, 2)
	if !bagAt(s, player, 6).Active {
		t.Fatal("a whole worn stack stays worn in its new slot")
	}
	move(s, player, world.BoxBag, 6, world.BoxStorage, 0, 2)
	if storedAt(s, player, 0).ItemID != "sword" || worn(s, player, "sword") {
		t.Fatal("a banked stack is no longer worn")
	}
	move(s, player, world.BoxBag, 0, world.BoxStorage, 1, 1)
	if bagAt(s, player, 0).ItemID != "anon" {
		t.Fatal("the last worn skin cannot be banked")
	}
}

// Personal storage belongs to the player identity and never mixes with an NPC
// vault of the same player. A finished session leaves neither behind.
func TestPersonalStorageIsOwnedApartFromVaults(t *testing.T) {
	s, player := moveFixture(world.ObjectLayerState{ItemID: "gem", Quantity: 3})
	s.Vaults = map[world.PlayerID]map[string][]Stack{"p-1": {"bank": {{ItemID: "ore", Qty: 1}}}}

	move(s, player, world.BoxBag, 0, world.BoxStorage, 0, 2)
	if got := s.PlayerStorage(player); len(got) != 1 || got[0] != (Stack{ItemID: "gem", Qty: 2}) {
		t.Fatalf("personal storage must hold the gem: %+v", got)
	}
	if vaults := s.PlayerVaults(player); len(vaults) != 1 || len(vaults[0].Items) != 1 || vaults[0].Items[0].ItemID != "ore" {
		t.Fatalf("an NPC vault must not see personal storage: %+v", vaults)
	}

	s.ForgetPlayer("p-1")
	if len(s.PlayerStorage(player)) != 0 || len(s.PlayerVaults(player)) != 0 {
		t.Fatal("a finished session leaves no container behind")
	}
}

// A vault at a terminal in view takes coins from the wallet into a slot, and
// gives them back to the wallet. A terminal out of view takes nothing.
func TestAVaultTradesCoinsWithTheWallet(t *testing.T) {
	s, player := moveFixture()
	player.Coins = 30
	player.AOI = world.Rectangle{MinX: -10, MinY: -10, MaxX: 10, MaxY: 10}
	terminal := worldtest.PlaceBot(s.World, 2, world.Vitals{Life: 1})
	s.actionCache = map[world.EntityID]*CyberiaAction{terminal: {Code: "bank", StorageSlots: 4}}
	vault := func(cmd world.ItemMove) {
		s.HandleItemMove(player, &world.InputCommand{EntityID: terminal, Move: cmd})
	}

	vault(world.ItemMove{From: world.BoxBag, FromSlot: world.WalletSlot, To: world.BoxVault, ToSlot: 2, Qty: 20})
	if got := s.Vaults["p-1"]["bank"]; player.Coins != 10 || len(got) != 1 || got[0] != (Stack{ItemID: "coin", Qty: 20, Slot: 2}) {
		t.Fatalf("coins %d, vault %+v", player.Coins, got)
	}
	vault(world.ItemMove{From: world.BoxVault, FromSlot: 2, To: world.BoxBag, ToSlot: world.WalletSlot, Qty: 5})
	if player.Coins != 15 || s.Vaults["p-1"]["bank"][0].Qty != 15 {
		t.Fatalf("coins %d after a withdrawal of 5", player.Coins)
	}

	player.AOI = world.Rectangle{MinX: 50, MinY: 50, MaxX: 60, MaxY: 60}
	vault(world.ItemMove{From: world.BoxVault, FromSlot: 2, To: world.BoxBag, ToSlot: world.WalletSlot, Qty: 5})
	if player.Coins != 15 {
		t.Fatal("a terminal out of view must take no move")
	}
}

// Personal storage opens a slot for each level step: a locked slot takes no
// stack until the level opens it.
func TestPersonalStorageOpensSlotsWithTheLevel(t *testing.T) {
	s, player := moveFixture(world.ObjectLayerState{ItemID: "gem", Quantity: 2})
	locked := StorageCapacity.Slots(1)

	move(s, player, world.BoxBag, 0, world.BoxStorage, locked, 1)
	if len(s.PlayerStorage(player)) != 0 {
		t.Fatal("a locked storage slot takes nothing")
	}
	s.SetProgression(player.ID, NewEntityProgression(6, false, s.ProgressionConfig()))
	move(s, player, world.BoxBag, 0, world.BoxStorage, locked, 1)
	if storedAt(s, player, locked).Qty != 1 {
		t.Fatal("level 6 opens the slot")
	}
}

func worn(s *Content, player *world.Session, itemID string) bool {
	for _, layer := range *s.InventoryLayers(player) {
		if layer.ItemID == itemID && layer.Active {
			return true
		}
	}
	return false
}
