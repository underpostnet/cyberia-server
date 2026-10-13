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

// A deposit leaves the player's inventory and lands in the vault; a withdrawal
// does the reverse and drops the stack once drained. Quantities are clamped to
// what the source actually holds, so a spoofed count can never mint items.
func TestStorageTransferMovesAcrossTheBoundary(t *testing.T) {
	s := &Content{
		World:      &world.World{},
		CoinItemID: "coin",

		Storage: map[StorageKey][]StorageSlot{},
	}
	player := worldtest.PlacePlayer(s.World, 1, []world.ObjectLayerState{{ItemID: "hatchet", Quantity: 2}})

	// Deposit more than held → clamped to the 2 the player actually has.
	slots := s.storageDeposit(player, nil, 2, "hatchet", 9)
	if len(slots) != 1 || slots[0].Qty != 2 {
		t.Fatalf("deposit must clamp to the held count: %+v", slots)
	}
	if s.playerItemQuantity(player, "hatchet") != 0 {
		t.Fatal("a deposit must leave the player's inventory")
	}

	// A second deposit of the same item merges into its stack.
	s.Ap(player.ID, player.MapCode).Layers = append(s.Ap(player.ID, player.MapCode).Layers, world.ObjectLayerState{ItemID: "hatchet", Quantity: 1})
	slots = s.storageDeposit(player, slots, 2, "hatchet", 1)
	if len(slots) != 1 || slots[0].Qty != 3 {
		t.Fatalf("same-item deposit must merge, got %+v", slots)
	}

	// A worn item is taken off as it is banked; only the equipment rules hold one back, and this
	// server configures none. See TestStorageDepositTakesOffAWornSkinWhenAnotherIsActive.
	s.Ap(player.ID, player.MapCode).Layers = append(s.Ap(player.ID, player.MapCode).Layers,
		world.ObjectLayerState{ItemID: "helmet", Quantity: 1, Active: true})
	if slots = s.storageDeposit(player, slots, 2, "helmet", 1); len(slots) != 2 {
		t.Fatalf("a worn item with nothing holding it back is storable, got %+v", slots)
	}
	if s.playerItemQuantity(player, "helmet") != 0 {
		t.Fatal("what is banked leaves the player")
	}

	// A full vault refuses a new stack and keeps the item with the player.
	s.Ap(player.ID, player.MapCode).Layers = append(s.Ap(player.ID, player.MapCode).Layers, world.ObjectLayerState{ItemID: "gem", Quantity: 4})
	if got := s.storageDeposit(player, slots, 2, "gem", 4); len(got) != 2 {
		t.Fatalf("a full vault must refuse a new stack, got %+v", got)
	}
	if s.playerItemQuantity(player, "gem") != 4 {
		t.Fatal("a refused deposit must not take the item")
	}

	// Partial withdrawal keeps the stack; draining it removes the stack.
	slots = s.storageWithdraw(player, slots, "hatchet", 1)
	if len(slots) != 2 || slots[0].Qty != 2 || s.playerItemQuantity(player, "hatchet") != 1 {
		t.Fatalf("partial withdrawal must keep the stack: %+v", slots)
	}
	slots = s.storageWithdraw(player, slots, "hatchet", 99)
	if len(slots) != 1 || s.playerItemQuantity(player, "hatchet") != 3 {
		t.Fatalf("draining a stack must remove it and return everything: %+v", slots)
	}
}

// A skin the player is wearing can be banked as long as another skin stays on: requireSkin keeps
// an entity dressed, it does not pin one particular skin to it for good.
func TestStorageDepositTakesOffAWornSkinWhenAnotherIsActive(t *testing.T) {
	server := &Content{
		World: &world.World{},
		EquipmentRules: EquipmentRulesConfig{
			ActiveItemTypes: map[string]bool{"skin": true, "weapon": true},
			OnePerType:      true,
			RequireSkin:     true,
		},
		ObjectLayerDataCache: map[string]*ObjectLayer{
			"anon": {Data: ObjectLayerData{Item: Item{Type: "skin"}}},
			"punk": {Data: ObjectLayerData{Item: Item{Type: "skin"}}},
		},
	}
	player := worldtest.PlacePlayer(server.World, 1, []world.ObjectLayerState{
		{ItemID: "anon", Active: true, Quantity: 1},
		{ItemID: "punk", Active: true, Quantity: 1},
	})

	slots := server.storageDeposit(player, nil, 8, "anon", 1)
	if len(slots) != 1 || slots[0].ItemID != "anon" {
		t.Fatalf("the skin belongs in the vault: %+v", slots)
	}
	for _, layer := range server.Ap(player.ID, player.MapCode).Layers {
		if layer.ItemID == "anon" {
			t.Fatalf("a banked skin is no longer carried: %+v", server.Ap(player.ID, player.MapCode).Layers)
		}
	}

	// The last active skin stays: with nothing else dressed, the vault does not take it.
	slots = server.storageDeposit(player, slots, 8, "punk", 1)
	if len(slots) != 1 {
		t.Fatalf("the last worn skin cannot be banked: %+v", slots)
	}
}
