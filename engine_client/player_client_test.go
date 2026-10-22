package engine_client

import (
	"reflect"
	"testing"
	"time"

	"cyberia-server/game/persist"
	pb "cyberia-server/gen/proto"
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

// A drop carries its stack and lifetime, a pickup the record it claims.
func TestTheEffectOfAJointCommitCrossesTheWire(t *testing.T) {
	drop := jointToProto(persist.JointRequest{Op: "drop-1", Kind: "drop", Effect: persist.Effect{Drop: &persist.WorldDrop{
		MapCode: "m", X: 2.5, Y: 3, ItemID: "gel", CID: "cid-gel", Quantity: 4, Lifetime: 45 * time.Second,
	}}})
	want := &pb.WorldDrop{MapCode: "m", X: 2.5, Y: 3, ItemId: "gel", OlCid: "cid-gel", Quantity: 4, LifetimeMs: 45000}
	if drop.GetKind() != "drop" || drop.GetClaimDropId() != "" || !reflect.DeepEqual(drop.GetDrop(), want) {
		t.Fatalf("drop: %+v", drop)
	}
	pickup := jointToProto(persist.JointRequest{Op: "pickup-1", Kind: "pickup", Effect: persist.Effect{Claim: "drop-1"}})
	if pickup.GetDrop() != nil || pickup.GetClaimDropId() != "drop-1" {
		t.Fatalf("pickup: %+v", pickup)
	}
}

func TestAListedWorldDropKeepsItsRecord(t *testing.T) {
	expires := time.UnixMilli(1_900_000_000_000)
	got := worldDropsFromProto([]*pb.WorldDrop{{
		DropId: "drop-1", MapCode: "m", X: 1, Y: 2, ItemId: "gel", OlCid: "cid-gel", Quantity: 3,
		OwnerPlayerId: "p1", ExpiresAtMs: expires.UnixMilli(),
	}})
	want := []persist.WorldDrop{{ID: "drop-1", MapCode: "m", X: 1, Y: 2, ItemID: "gel", CID: "cid-gel", Quantity: 3, Owner: "p1", ExpiresAt: expires}}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("got %+v", got)
	}
}

// A market operation and a market view cross the wire unchanged.
func TestAMarketOperationAndViewCrossTheWire(t *testing.T) {
	op := marketOpToProto(&persist.MarketOp{MarketCode: "bazaar", ListingFee: 2, SaleFee: 1, ClaimIDs: []string{"c1"}, Coins: 4,
		Stacks: []persist.Stack{{ItemID: "gel", CID: "cid-gel", Quantity: 2}}})
	if op.GetMarketCode() != "bazaar" || op.GetListingFee() != 2 || op.GetSaleFee() != 1 || op.GetCoins() != 4 ||
		op.GetStacks()[0].GetQuantity() != 2 || marketOpToProto(nil) != nil {
		t.Fatalf("op: %+v", op)
	}
	ends := time.UnixMilli(1_900_000_000_000)
	view := marketViewFromProto(&pb.ListMarketResponse{
		Listings: []*pb.MarketListing{{ListingId: "l1", Mode: "auction", MinBid: 6, EndsAtMs: ends.UnixMilli(), Leading: true}},
		Next:     "n",
		Claims: []*pb.MarketClaim{{ClaimId: "c1", Coins: 3, MarketCode: "bazaar"},
			{ClaimId: "c2", Stack: &pb.PlayerStack{ItemId: "gel", OlCid: "x", Quantity: 1}}},
		ScopeKey: "u1",
		Revision: 3,
	})
	if l := view.Listings[0]; l.MinBid != 6 || !l.EndsAt.Equal(ends) || !l.Leading || view.Next != "n" ||
		view.ScopeKey != "u1" || view.Revision != 3 || view.Claims[0].MarketCode != "bazaar" {
		t.Fatalf("listing: %+v", l)
	}
	if view.Claims[0].Quantity != 0 || view.Claims[1].ItemID != "gel" || view.Claims[1].Quantity != 1 {
		t.Fatalf("claims: %+v", view.Claims)
	}
}
