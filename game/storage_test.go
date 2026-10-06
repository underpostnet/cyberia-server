package game

import "testing"

// An authored capacity is taken as-is and only clamped at the ceiling the
// client can render — the vault wraps into rows, so it need not be a square.
func TestStorageCapacityClampsToTheRenderableCeiling(t *testing.T) {
	cases := []struct{ authored, want int }{
		{-5, 0}, {0, 0}, {1, 1}, {7, 7}, {25, 25},
		{storageMaxSlots, storageMaxSlots},
		{storageMaxSlots + 100, storageMaxSlots},
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
	s := &GameServer{
		coinItemID: "coin",

		storage: map[storageKey][]StorageSlot{},
	}
	player := &PlayerState{
		EntityBase: EntityBase{ID: "p1", ObjectLayers: []ObjectLayerState{
			{ItemID: "hatchet", Quantity: 2},
		}},
	}

	// Deposit more than held → clamped to the 2 the player actually has.
	slots := s.storageDeposit(player, nil, 2, "hatchet", 9)
	if len(slots) != 1 || slots[0].Qty != 2 {
		t.Fatalf("deposit must clamp to the held count: %+v", slots)
	}
	if s.playerItemQuantity(player, "hatchet") != 0 {
		t.Fatal("a deposit must leave the player's inventory")
	}

	// A second deposit of the same item merges into its stack.
	player.ObjectLayers = append(player.ObjectLayers, ObjectLayerState{ItemID: "hatchet", Quantity: 1})
	slots = s.storageDeposit(player, slots, 2, "hatchet", 1)
	if len(slots) != 1 || slots[0].Qty != 3 {
		t.Fatalf("same-item deposit must merge, got %+v", slots)
	}

	// A worn item is taken off as it is banked; only the equipment rules hold one back, and this
	// server configures none. See TestStorageDepositTakesOffAWornSkinWhenAnotherIsActive.
	player.ObjectLayers = append(player.ObjectLayers,
		ObjectLayerState{ItemID: "helmet", Quantity: 1, Active: true})
	if slots = s.storageDeposit(player, slots, 2, "helmet", 1); len(slots) != 2 {
		t.Fatalf("a worn item with nothing holding it back is storable, got %+v", slots)
	}
	if s.playerItemQuantity(player, "helmet") != 0 {
		t.Fatal("what is banked leaves the player")
	}

	// A full vault refuses a new stack and keeps the item with the player.
	player.ObjectLayers = append(player.ObjectLayers, ObjectLayerState{ItemID: "gem", Quantity: 4})
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

// item_ops is a trust boundary: every op names an item and a positive count,
// and the list never exceeds the largest vault.
func TestParseInputBoundsItemOps(t *testing.T) {
	op := itemOp{ItemID: "hatchet", Qty: 1, ToVault: true}
	full := make([]itemOp, storageMaxSlots)
	for i := range full {
		full[i] = op
	}
	if _, ok := parseInput("item_ops", &inputPayload{EntityID: "vault", Ops: full}); !ok {
		t.Fatal("a list at the cap must pass")
	}
	for name, ops := range map[string][]itemOp{
		"empty":    nil,
		"over cap": append(full, op),
		"zero qty": {{ItemID: "hatchet", Qty: 0}},
		"no item":  {{Qty: 1}},
	} {
		if _, ok := parseInput("item_ops", &inputPayload{EntityID: "vault", Ops: ops}); ok {
			t.Fatalf("%s: must be rejected", name)
		}
	}
}

// A skin the player is wearing can be banked as long as another skin stays on: requireSkin keeps
// an entity dressed, it does not pin one particular skin to it for good.
func TestStorageDepositTakesOffAWornSkinWhenAnotherIsActive(t *testing.T) {
	server := &GameServer{
		equipmentRules: EquipmentRulesConfig{
			ActiveItemTypes: map[string]bool{"skin": true, "weapon": true},
			OnePerType:      true,
			RequireSkin:     true,
		},
		objectLayerDataCache: map[string]*ObjectLayer{
			"anon": {Data: ObjectLayerData{Item: Item{Type: "skin"}}},
			"punk": {Data: ObjectLayerData{Item: Item{Type: "skin"}}},
		},
	}
	player := &PlayerState{EntityBase: EntityBase{ID: "p1", ObjectLayers: []ObjectLayerState{
		{ItemID: "anon", Active: true, Quantity: 1},
		{ItemID: "punk", Active: true, Quantity: 1},
	}}}

	slots := server.storageDeposit(player, nil, 8, "anon", 1)
	if len(slots) != 1 || slots[0].ItemID != "anon" {
		t.Fatalf("the skin belongs in the vault: %+v", slots)
	}
	for _, layer := range player.ObjectLayers {
		if layer.ItemID == "anon" {
			t.Fatalf("a banked skin is no longer carried: %+v", player.ObjectLayers)
		}
	}

	// The last active skin stays: with nothing else dressed, the vault does not take it.
	slots = server.storageDeposit(player, slots, 8, "punk", 1)
	if len(slots) != 1 {
		t.Fatalf("the last worn skin cannot be banked: %+v", slots)
	}
}
