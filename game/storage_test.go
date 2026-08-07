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

func TestStorageSlotAtLocatesByIndex(t *testing.T) {
	slots := []StorageSlot{
		{ItemID: "coin", Qty: 5, Index: 0},
		{ItemID: "hatchet", Qty: 1, Index: 7},
	}
	if at := storageSlotAt(slots, 7); at != 1 {
		t.Fatalf("want the hatchet at position 1, got %d", at)
	}
	if at := storageSlotAt(slots, 3); at != -1 {
		t.Fatalf("a free index must not match, got %d", at)
	}
	if at := storageSlotAt(nil, 0); at != -1 {
		t.Fatal("an empty vault holds nothing")
	}
}

// A deposit leaves the player's inventory and lands in the vault; a withdrawal
// does the reverse and drops the slot once drained. Quantities are clamped to
// what the source actually holds, so a spoofed count can never mint items.
func TestStorageTransferMovesAcrossTheBoundary(t *testing.T) {
	s := &GameServer{
		coinItemID: "coin",
		statsCache: map[string]statsCacheEntry{},
		storage:    map[storageKey][]StorageSlot{},
	}
	player := &PlayerState{
		EntityBase: EntityBase{ID: "p1", ObjectLayers: []ObjectLayerState{
			{ItemID: "hatchet", Quantity: 2},
		}},
	}

	// Deposit more than held → clamped to the 2 the player actually has.
	slots := s.storageDeposit(player, nil, 25, &InputCommand{
		ItemID: "hatchet", Quantity: 9, ToIndex: 6,
	})
	if len(slots) != 1 || slots[0].Qty != 2 || slots[0].Index != 6 {
		t.Fatalf("deposit must clamp to the held count and land on the slot: %+v", slots)
	}
	if s.playerItemQuantity(player, "hatchet") != 0 {
		t.Fatal("a deposit must leave the player's inventory")
	}

	// A second deposit of the same item merges into the occupied cell.
	player.ObjectLayers = append(player.ObjectLayers, ObjectLayerState{ItemID: "hatchet", Quantity: 1})
	slots = s.storageDeposit(player, slots, 25, &InputCommand{
		ItemID: "hatchet", Quantity: 1, ToIndex: 6,
	})
	if len(slots) != 1 || slots[0].Qty != 3 {
		t.Fatalf("same-item deposit must merge, got %+v", slots)
	}

	// A different item may not land on an occupied cell.
	player.ObjectLayers = append(player.ObjectLayers, ObjectLayerState{ItemID: "coin", Quantity: 4})
	if got := s.storageDeposit(player, slots, 25, &InputCommand{
		ItemID: "coin", Quantity: 4, ToIndex: 6,
	}); len(got) != 1 {
		t.Fatalf("a foreign item must not occupy a taken slot, got %+v", got)
	}

	// Out-of-range indices are rejected rather than clamped.
	if got := s.storageDeposit(player, slots, 25, &InputCommand{
		ItemID: "coin", Quantity: 1, ToIndex: 25,
	}); len(got) != 1 {
		t.Fatalf("a slot past the capacity must be rejected, got %+v", got)
	}

	// An equipped item is stock the player is wearing — it may never be banked.
	player.ObjectLayers = append(player.ObjectLayers,
		ObjectLayerState{ItemID: "helmet", Quantity: 1, Active: true})
	if got := s.storageDeposit(player, slots, 25, &InputCommand{
		ItemID: "helmet", Quantity: 1, ToIndex: 0,
	}); len(got) != 1 {
		t.Fatalf("an active item must not be storable, got %+v", got)
	}

	// Partial withdrawal keeps the slot; draining it removes the slot.
	slots = s.storageWithdraw(player, slots, &InputCommand{Quantity: 1, FromIndex: 6})
	if len(slots) != 1 || slots[0].Qty != 2 || s.playerItemQuantity(player, "hatchet") != 1 {
		t.Fatalf("partial withdrawal must keep the slot: %+v", slots)
	}
	slots = s.storageWithdraw(player, slots, &InputCommand{Quantity: 99, FromIndex: 6})
	if len(slots) != 0 || s.playerItemQuantity(player, "hatchet") != 3 {
		t.Fatalf("draining a slot must remove it and return everything: %+v", slots)
	}
}

// Dragging a stack onto a free slot either relocates it whole or splits it,
// depending on the count the player picked in the split card.
func TestStorageRelocateSplitsOrMovesWhole(t *testing.T) {
	whole := storageRelocate([]StorageSlot{{ItemID: "hatchet", Qty: 4, Index: 2}}, 0, -1, 9, 4)
	if len(whole) != 1 || whole[0].Index != 9 || whole[0].Qty != 4 {
		t.Fatalf("a full-count move must relocate the stack: %+v", whole)
	}

	// A zero quantity is the unsized drag of a single item — still whole.
	unsized := storageRelocate([]StorageSlot{{ItemID: "hatchet", Qty: 1, Index: 2}}, 0, -1, 9, 0)
	if len(unsized) != 1 || unsized[0].Index != 9 {
		t.Fatalf("an unsized move must relocate the stack: %+v", unsized)
	}

	split := storageRelocate([]StorageSlot{{ItemID: "hatchet", Qty: 4, Index: 2}}, 0, -1, 9, 1)
	if len(split) != 2 {
		t.Fatalf("a partial count must leave a remainder behind: %+v", split)
	}
	if split[0].Index != 2 || split[0].Qty != 3 {
		t.Fatalf("the source keeps the remainder in place: %+v", split[0])
	}
	if split[1].Index != 9 || split[1].Qty != 1 || split[1].ItemID != "hatchet" {
		t.Fatalf("the split lands on the target slot: %+v", split[1])
	}
}

// Dropping a stack onto one holding the same item merges them. A full count
// drains the source slot away; a partial one leaves the remainder behind.
func TestStorageRelocateMergesOntoTheSameItem(t *testing.T) {
	stock := func() []StorageSlot {
		return []StorageSlot{
			{ItemID: "hatchet", Qty: 4, Index: 2},
			{ItemID: "hatchet", Qty: 5, Index: 9},
		}
	}

	merged := storageRelocate(stock(), 0, 1, 9, 4)
	if len(merged) != 1 {
		t.Fatalf("a full merge must drain the source slot: %+v", merged)
	}
	if merged[0].Index != 9 || merged[0].Qty != 9 {
		t.Fatalf("the target absorbs the whole stack: %+v", merged[0])
	}

	partial := storageRelocate(stock(), 0, 1, 9, 1)
	if len(partial) != 2 {
		t.Fatalf("a partial merge must keep the source slot: %+v", partial)
	}
	if partial[0].Index != 2 || partial[0].Qty != 3 {
		t.Fatalf("the source keeps the remainder: %+v", partial[0])
	}
	if partial[1].Index != 9 || partial[1].Qty != 6 {
		t.Fatalf("the target absorbs only the picked count: %+v", partial[1])
	}
}
