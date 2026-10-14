package net

import (
	"testing"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// item_move is a trust boundary: both places name a known box and a slot in
// range, the wallet slot belongs to the bag, a vault move names its terminal,
// and one move joins the bag and at most one container.
func TestParseInputBoundsItemMoves(t *testing.T) {
	valid := inputPayload{From: "bag", FromSlot: 0, To: "storage", ToSlot: content.StorageMaxSlots - 1, Qty: 3}
	cmd, ok := parseInput("item_move", &valid)
	want := world.ItemMove{From: world.BoxBag, To: world.BoxStorage, ToSlot: content.StorageMaxSlots - 1, Qty: 3}
	if !ok || cmd.Kind != world.InputKindItemMove || cmd.Move != want {
		t.Fatalf("a storage move must pass: %+v", cmd.Move)
	}
	if _, ok := parseInput("item_move", &inputPayload{From: "bag", FromSlot: world.WalletSlot, To: "vault", ToSlot: 2, Qty: 5, EntityID: 7}); !ok {
		t.Fatal("coins from the wallet into a vault must pass")
	}
	for name, p := range map[string]inputPayload{
		"unknown box":        {From: "bag", To: "chest", Qty: 1},
		"slot past the cap":  {From: "bag", To: "bag", ToSlot: content.StorageMaxSlots, Qty: 1},
		"wallet of storage":  {From: "storage", FromSlot: world.WalletSlot, To: "bag", Qty: 1},
		"zero units":         {From: "bag", To: "storage", ToSlot: 1},
		"vault without npc":  {From: "bag", To: "vault", Qty: 1},
		"terminal not vault": {From: "bag", To: "storage", Qty: 1, EntityID: 7},
		"two containers":     {From: "storage", To: "vault", Qty: 1, EntityID: 7},
	} {
		if _, ok := parseInput("item_move", &p); ok {
			t.Fatalf("%s: must be rejected", name)
		}
	}
}
