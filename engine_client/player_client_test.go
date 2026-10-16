package engine_client

import (
	"reflect"
	"testing"

	"cyberia-server/game/persist"
)

func TestPlayerStateCrossesTheWireUnchanged(t *testing.T) {
	state := persist.State{
		Coins:   12,
		XP:      9000,
		Bag:     []persist.Stack{{ItemID: "sword", CID: "cid-s", Quantity: 1, Slot: 2, Active: true}},
		Storage: []persist.Stack{{ItemID: "gem", CID: "cid-g", Quantity: 40, Slot: 0}},
		Vaults:  []persist.Vault{{ActionCode: "bank", Stacks: []persist.Stack{{ItemID: "gem", CID: "cid-g", Quantity: 3, Slot: 1}}}},
		Held:    []persist.Held{{Stack: persist.Stack{ItemID: "relic", CID: "cid-r", Quantity: 1}, Container: persist.ContainerBag, Reason: persist.HeldUnknown}},
		Quests: []persist.Quest{{Code: "q1", Status: "completed", StartedAtMs: 1, CompletedAtMs: 2,
			Steps: []persist.QuestStep{{StepID: "s1", Objectives: []persist.Objective{{Current: 3, Required: 3}}}}}},
		Position: persist.Position{MapCode: "m", X: 4.5, Y: 6},
	}
	if got := stateFromProto(stateToProto(state)); !reflect.DeepEqual(got, state) {
		t.Fatalf("round trip:\n got %+v\nwant %+v", got, state)
	}
}
