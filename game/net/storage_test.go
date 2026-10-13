package net

import (
	"testing"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// item_ops is a trust boundary: every op names an item and a positive count,
// and the list never exceeds the largest vault.
func TestParseInputBoundsItemOps(t *testing.T) {
	op := world.ItemOp{ItemID: "hatchet", Qty: 1, ToVault: true}
	full := make([]world.ItemOp, content.StorageMaxSlots)
	for i := range full {
		full[i] = op
	}
	if _, ok := parseInput("item_ops", &inputPayload{EntityID: 7, Ops: full}); !ok {
		t.Fatal("a list at the cap must pass")
	}
	for name, ops := range map[string][]world.ItemOp{
		"empty":    nil,
		"over cap": append(full, op),
		"zero qty": {{ItemID: "hatchet", Qty: 0}},
		"no item":  {{Qty: 1}},
	} {
		if _, ok := parseInput("item_ops", &inputPayload{EntityID: 7, Ops: ops}); ok {
			t.Fatalf("%s: must be rejected", name)
		}
	}
}
