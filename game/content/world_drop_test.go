package content

import (
	"testing"
	"time"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
	"cyberia-server/game/world/worldtest"
)

// accountFixture is the economy fixture with an account player and a fake
// engine.
func accountFixture(t *testing.T) (*Content, *world.Session, *recorder, *fakeDurability) {
	t.Helper()
	s, player, _, rec := economyFixture(t)
	durability := newFakeDurability()
	s.Durability = durability
	player.PlayerID, player.AccountID = "prof-1", "acc-1"
	return s, player, rec, durability
}

func dropAtFeet(qty int) *world.InputCommand {
	return &world.InputCommand{Move: world.ItemMove{From: world.BoxBag, FromSlot: 0, Qty: qty}, TargetX: -1, TargetY: -1}
}

// An account drop commits the state without the stack and its record at
// once; the token waits closed and does not age until that commit applies.
func TestAnAccountDropCommitsItsRecordWithTheState(t *testing.T) {
	s, player, rec, durability := accountFixture(t)
	ms := worldtest.Map(s.World)
	s.HandleItemDrop(player, dropAtFeet(2))
	ack := rec.last("item_drop_ack")
	token := ack["dropId"].(world.EntityID)
	op := durability.only(t)
	drop := durability.effects[op].Drop
	if ack["status"] != DropPending || drop == nil || drop.ItemID != "med-gel" || drop.CID != "cid-med" ||
		drop.Quantity != 2 || drop.MapCode != "test" || drop.Lifetime != lootLifetime {
		t.Fatalf("the drop links its record: ack=%v drop=%+v", ack, drop)
	}
	if bag := durability.members[op][0].State.Bag; len(bag) != 2 || bag[0].Quantity != 3 {
		t.Fatalf("the linked state holds the rest of the stack: %+v", bag)
	}
	if b := ms.Br(token); b.WorldDrop != op || b.Lock.Op != op || !b.ExpiresAt.IsZero() || s.SettleDropLock(ms, token, time.Now()) {
		t.Fatalf("the token waits closed and does not age: %+v", b)
	}

	s.SettleLinkedOps(time.Now())
	if len(durability.unlinked) != 0 {
		t.Fatal("a pending drop stays linked")
	}
	durability.links[op] = persist.Applied
	s.SettleLinkedOps(time.Now())
	if b := ms.Br(token); b.Lock.Op != "" || b.ExpiresAt.IsZero() || !s.SettleDropLock(ms, token, time.Now()) {
		t.Fatalf("the applied drop opens the token and starts its lifetime: %+v", b)
	}
	if ack := rec.last("item_drop_ack"); ack["status"] != DropPublic || len(durability.unlinked) != 1 {
		t.Fatalf("its owner learns the drop is public, and the link ends: %v", ack)
	}
}

// A drop that never applies removes its token: the saved state still holds
// the stack.
func TestARefusedDropRemovesItsToken(t *testing.T) {
	s, player, rec, durability := accountFixture(t)
	ms := worldtest.Map(s.World)
	s.HandleItemDrop(player, dropAtFeet(1))
	token := rec.last("item_drop_ack")["dropId"].(world.EntityID)
	durability.links[durability.only(t)] = persist.Refused
	s.SettleLinkedOps(time.Now())
	if ms.IsKind(token, world.EntityBot) {
		t.Fatal("the token of a refused drop leaves")
	}
}

// A state that cannot link now refuses the drop and keeps the stack.
func TestADropThatCannotLinkKeepsTheStack(t *testing.T) {
	s, player, rec, durability := accountFixture(t)
	durability.refuse = true
	s.HandleItemDrop(player, dropAtFeet(1))
	if ack := rec.last("item_drop_ack"); ack["reason"] != RefuseSaving || stackAt(s, player, 0).Qty != 5 {
		t.Fatalf("the drop waits for the earlier commit: %v", ack)
	}
}

// worldDropToken places an open world drop of 2 med-gel under the player.
func worldDropToken(s *Content, player *world.Session) world.EntityID {
	ms := worldtest.Map(s.World)
	s.RestoreWorldDrops([]persist.WorldDrop{{
		ID: "drop-r", MapCode: "test", ItemID: "med-gel", CID: "cid-med", Quantity: 2, ExpiresAt: time.Now().Add(time.Minute),
	}}, time.Now())
	for id := range ms.OfKind(world.EntityBot) {
		if ms.Br(id).WorldDrop == "drop-r" {
			ms.Tf(id).Pos, ms.Tf(player.ID).Size = ms.Tf(player.ID).Pos, world.Vec2{X: 1, Y: 1}
			return id
		}
	}
	return 0
}

// A pickup of a world drop commits the state with the stack and the claim of
// the record; the collector receives the stack once the claim applied.
func TestAPickupClaimsTheRecordBeforeTheStackArrives(t *testing.T) {
	s, player, _, durability := accountFixture(t)
	ms := worldtest.Map(s.World)
	token := worldDropToken(s, player)
	if s.ResolveDropCollector(ms, token) != player {
		t.Fatal("an account player takes a world drop")
	}
	s.ClaimWorldDrop(ms, token, player)
	op := durability.only(t)
	if durability.effects[op].Claim != "drop-r" || durability.members[op][0].State.Bag[0].Quantity != 7 {
		t.Fatalf("the claim links the record and the state with the stack: %+v", durability.members[op])
	}
	if stackAt(s, player, 0).Qty != 5 || s.SettleDropLock(ms, token, time.Now()) || !ms.Br(token).ExpiresAt.IsZero() {
		t.Fatal("the stack waits for the claim, and the token stays closed")
	}
	applied, refused := s.FinalStates(player)
	if applied.Bag[0].Quantity != 7 || refused.Bag[0].Quantity != 5 {
		t.Fatalf("a collector that leaves closes with the stack only when the claim applies: %+v %+v", applied.Bag, refused.Bag)
	}

	durability.links[op] = persist.Applied
	before := durability.captures
	s.SettleLinkedOps(time.Now())
	if stackAt(s, player, 0).Qty != 7 || ms.IsKind(token, world.EntityBot) {
		t.Fatalf("the applied claim gives the stack and removes the token: qty=%d", stackAt(s, player, 0).Qty)
	}
	if durability.captures != before+1 || len(durability.unlinked) != 1 {
		t.Fatal("the collector captures its state before the link ends")
	}
}

// A claim of a record that is gone removes the token; any other refusal
// opens it again with its lifetime.
func TestARefusedPickupRemovesOrReopensTheToken(t *testing.T) {
	for _, c := range []struct {
		refusal string
		stays   bool
	}{{persist.RefusalDropGone, false}, {persist.RefusalStaleSession, true}} {
		s, player, _, durability := accountFixture(t)
		ms := worldtest.Map(s.World)
		token := worldDropToken(s, player)
		expires := ms.Br(token).ExpiresAt
		s.ClaimWorldDrop(ms, token, player)
		op := durability.only(t)
		durability.links[op], durability.refusals[op] = persist.Refused, c.refusal
		s.SettleLinkedOps(time.Now())
		if stackAt(s, player, 0).Qty != 5 || ms.IsKind(token, world.EntityBot) != c.stays {
			t.Fatalf("%s: no stack, token kept=%v", c.refusal, ms.IsKind(token, world.EntityBot))
		}
		if c.stays && (ms.Br(token).Lock.Op != "" || !ms.Br(token).ExpiresAt.Equal(expires)) {
			t.Fatalf("%s: the token opens again with its expiry: %+v", c.refusal, ms.Br(token))
		}
	}
}

// Goods never cross between a guest session and a profile: a guest never
// takes a world drop, an account player never takes a guest drop.
func TestDropsStayWithTheirPrincipal(t *testing.T) {
	s, player, _, _ := economyFixture(t)
	ms := worldtest.Map(s.World)
	token := worldDropToken(s, player)
	if s.ResolveDropCollector(ms, token) != nil {
		t.Fatal("a guest never takes a world drop")
	}
	ms.Br(token).WorldDrop = ""
	player.AccountID = "acc-1"
	if s.ResolveDropCollector(ms, token) != nil {
		t.Fatal("an account player never takes a guest drop")
	}
}

// A restore puts back each record of a map of this world that has not
// expired, open at once until its own expiry.
func TestARestorePutsBackTheLiveRecordsOfThisWorld(t *testing.T) {
	s, _, _, _ := economyFixture(t)
	ms := worldtest.Map(s.World)
	now := time.Now()
	expires := now.Add(30 * time.Second)
	restored := s.RestoreWorldDrops([]persist.WorldDrop{
		{ID: "d1", MapCode: "test", ItemID: "knife", CID: "cid-knife", Quantity: 1, Owner: "prof-1", ExpiresAt: expires},
		{ID: "d2", MapCode: "gone", ItemID: "knife", CID: "cid-knife", Quantity: 1, ExpiresAt: expires},
		{ID: "d3", MapCode: "test", ItemID: "knife", CID: "cid-knife", Quantity: 1, ExpiresAt: now.Add(-time.Second)},
	}, now)
	if restored != 1 {
		t.Fatalf("one record comes back, got %d", restored)
	}
	for id := range ms.OfKind(world.EntityBot) {
		if b := ms.Br(id); b.WorldDrop == "d1" {
			if !b.ExpiresAt.Equal(expires) || b.Lock.PlayerID != "" || b.LootOrigin != world.LootOriginPlayer {
				t.Fatalf("the token is open until the expiry of its record: %+v", b)
			}
			if b.DropOwner != "prof-1" || !b.OwnerHold {
				t.Fatalf("its owner takes it back only after it stepped off it: %+v", b)
			}
			return
		}
	}
	t.Fatal("the record of this world comes back")
}
