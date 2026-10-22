package sim

import (
	"testing"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
)

const portalID world.EntityID = 50

func fee(v uint32) *uint32 { return &v }

// portalFixture puts the player on a portal to map "dest" that wears gate-stone
// and gate-rune, and carries gate-moss inactive. The economy charges fallback
// per transit unless a row of an active portal item names a fee.
func portalFixture(t *testing.T, fallback uint32, rows ...*pb.ItemPolicy) (*Sim, *world.MapState, *world.Session) {
	t.Helper()
	s, ms, player := movementFixture(t)
	s.ObjectLayerDataCache = map[string]*content.ObjectLayer{
		"gate-stone": {Cid: "cid-stone"}, "gate-rune": {Cid: "cid-rune"}, "gate-moss": {Cid: "cid-moss"},
	}
	applyEconomy(s, fallback, rows...)
	ms.Spawn(portalID, world.EntityData{Kind: world.EntityPortal},
		world.Transform{Pos: world.Vec2{X: 1, Y: 1}, Size: world.Vec2{X: 1, Y: 1}},
		world.Appearance{Layers: []world.ObjectLayerState{
			{ItemID: "gate-stone", Active: true}, {ItemID: "gate-rune", Active: true}, {ItemID: "gate-moss"},
		}})
	ms.AddPortal(portalID, world.Portal{TargetMap: "dest", TargetCell: world.Cell{X: 5, Y: 5}})
	s.Maps["dest"] = world.NewMapState("dest", 20, 20)
	player.Coins = 50
	return s, ms, player
}

func applyEconomy(s *Sim, fallback uint32, rows ...*pb.ItemPolicy) {
	s.ApplyConfig(&pb.InstanceConfig{PortalHoldTimeMs: 1000, EconomyRules: &pb.EconomyRules{
		Code: "standard", Revision: uint64(fallback) + 1,
		Fees:         &pb.OperationFees{Portal: fallback},
		ItemDefaults: &pb.ItemPolicy{Trade: true, WorldDrop: true},
		ItemPolicies: rows,
	}}, content.ProgressionRules{MaxLevel: 10, XpPerLevel: 100})
}

// holdThrough starts the hold, lets it run out and checks the portal again.
func holdThrough(s *Sim, ms *world.MapState, player *world.Session) {
	s.checkPortal(player, ms)
	player.TimeOnPortal = time.Now().Add(-2 * s.PortalHoldTime)
	s.checkPortal(player, ms)
}

func TestPortalFeeResolution(t *testing.T) {
	for _, c := range []struct {
		name     string
		fallback uint32
		rows     []*pb.ItemPolicy
		want     int
	}{
		{"no portal item names a fee: the economy fee", 4, nil, 4},
		{"one portal item names a fee", 4, []*pb.ItemPolicy{{ItemId: "gate-stone", OlCid: "cid-stone", PortalFee: fee(3)}}, 3},
		{"two portal items add up, the economy fee never joins", 4, []*pb.ItemPolicy{
			{ItemId: "gate-stone", OlCid: "cid-stone", PortalFee: fee(3)},
			{ItemId: "gate-rune", OlCid: "cid-rune", PortalFee: fee(5)},
		}, 8},
		{"an explicit zero makes the portal free", 4, []*pb.ItemPolicy{{ItemId: "gate-stone", OlCid: "cid-stone", PortalFee: fee(0)}}, 0},
		{"an inactive portal item adds nothing", 4, []*pb.ItemPolicy{{ItemId: "gate-moss", OlCid: "cid-moss", PortalFee: fee(9)}}, 4},
		{"a row compiled for another definition adds nothing", 4, []*pb.ItemPolicy{{ItemId: "gate-stone", OlCid: "cid-old", PortalFee: fee(3)}}, 4},
		{"a row without a fee adds nothing", 4, []*pb.ItemPolicy{{ItemId: "gate-stone", OlCid: "cid-stone", Trade: true}}, 4},
		{"the sum stops at the fee maximum", 4, []*pb.ItemPolicy{
			{ItemId: "gate-stone", OlCid: "cid-stone", PortalFee: fee(content.PortalFeeMax)},
			{ItemId: "gate-rune", OlCid: "cid-rune", PortalFee: fee(5)},
		}, content.PortalFeeMax},
	} {
		t.Run(c.name, func(t *testing.T) {
			s, ms, _ := portalFixture(t, c.fallback, c.rows...)
			if got := s.PortalFee(ms, portalID); got != c.want {
				t.Fatalf("portal fee = %d, want %d", got, c.want)
			}
		})
	}
}

// A transit charges the fee the hold showed, once.
func TestATransitChargesTheQuotedFeeOnce(t *testing.T) {
	s, ms, player := portalFixture(t, 4, &pb.ItemPolicy{ItemId: "gate-rune", OlCid: "cid-rune", PortalFee: fee(7)})
	s.checkPortal(player, ms)
	if !player.OnPortal || player.PortalQuote.Fee != 7 || player.PortalQuote.Portal != portalID {
		t.Fatalf("the hold quotes the fee of the portal: %+v", player.PortalQuote)
	}
	player.TimeOnPortal = time.Now().Add(-2 * s.PortalHoldTime)
	s.checkPortal(player, ms)
	if player.MapCode != "dest" || player.Coins != 43 {
		t.Fatalf("the transit charges 7 once: map=%s coins=%d", player.MapCode, player.Coins)
	}
	s.checkPortal(player, s.Maps["dest"])
	if player.Coins != 43 {
		t.Fatalf("nothing charges again after the transit, coins=%d", player.Coins)
	}
}

// A fee that changes during the hold starts it again at the new fee: the
// stale quote never charges.
func TestAFeeChangeDuringTheHoldStartsItAgain(t *testing.T) {
	s, ms, player := portalFixture(t, 4)
	s.checkPortal(player, ms)
	player.TimeOnPortal = time.Now().Add(-2 * s.PortalHoldTime)
	applyEconomy(s, 6)
	s.checkPortal(player, ms)
	if player.MapCode != "test" || player.Coins != 50 || player.PortalQuote.Fee != 6 {
		t.Fatalf("the hold restarts at the new fee: map=%s coins=%d quote=%+v", player.MapCode, player.Coins, player.PortalQuote)
	}
	if time.Since(player.TimeOnPortal) > s.PortalHoldTime {
		t.Fatal("the hold starts again from zero")
	}
}

// A player who cannot pay stays on the portal and keeps its coins.
func TestAPlayerWhoCannotPayTheFeeStays(t *testing.T) {
	s, ms, player := portalFixture(t, 4)
	player.Coins = 3
	holdThrough(s, ms, player)
	if player.MapCode != "test" || player.Coins != 3 {
		t.Fatalf("no transit and no charge: map=%s coins=%d", player.MapCode, player.Coins)
	}
}
