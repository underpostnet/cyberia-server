package sim

import (
	"testing"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/persist"
	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
)

// Two players touch one public drop in the same tick: one takes it, once, and
// the units neither grow nor shrink.
func TestOnePlayerClaimsAPublicDrop(t *testing.T) {
	s, ms, owner := movementFixture(t)
	owner.PlayerID = "owner"
	s.ApplyConfig(&pb.InstanceConfig{EconomyRules: &pb.EconomyRules{
		ItemDefaults: &pb.ItemPolicy{Trade: true, WorldDrop: true},
	}}, content.ProgressionRules{MaxLevel: 10, XpPerLevel: 100})
	s.ObjectLayerDataCache = map[string]*content.ObjectLayer{"gel": {Cid: "cid-gel"}}
	ms.Ap(owner.ID).Layers = []world.ObjectLayerState{{ItemID: "gel", CID: "cid-gel", Quantity: 3, Slot: 0}}
	other := woundedPlayer(s.World, ms, 2, world.Vec2{X: 1, Y: 1}, 100)
	other.MapCode, other.PlayerID = "test", "other"
	ms.AddPlayer(other.ID)
	for s.NewEntityID() < other.ID {
	}

	owner.InputQueue = []world.InputCommand{{Kind: world.InputKindItemDrop, Sequence: 1, TargetX: -1, TargetY: -1,
		Move: world.ItemMove{From: world.BoxBag, FromSlot: 0, Qty: 2}}}
	s.PhaseInput(0, ms)
	tokens := drops(ms)
	if len(tokens) != 1 || itemQty(s.World, owner, "gel") != 1 {
		t.Fatalf("one token of 2 gel leaves the bag: tokens=%d left=%d", len(tokens), itemQty(s.World, owner, "gel"))
	}
	token := tokens[0]
	ms.Br(token).CollectableAt = time.Time{}
	for _, id := range []world.EntityID{owner.ID, other.ID} {
		ms.Tf(id).Pos = ms.Tf(token).Pos
	}
	s.PhaseLoot(0, ms)
	total := itemQty(s.World, owner, "gel") + itemQty(s.World, other, "gel")
	if ms.IsKind(token, world.EntityBot) || total != 3 {
		t.Fatalf("one claim takes the token whole: token left=%v gel=%d", ms.IsKind(token, world.EntityBot), total)
	}
}

// The owner of a token dropped at its feet takes it back only after it
// stepped off the token once.
func TestOwnerStepsOffBeforeTakingItsDropBack(t *testing.T) {
	s, ms, owner := movementFixture(t)
	owner.PlayerID = "owner"
	s.ApplyConfig(&pb.InstanceConfig{EconomyRules: &pb.EconomyRules{
		ItemDefaults: &pb.ItemPolicy{Trade: true, WorldDrop: true},
	}}, content.ProgressionRules{MaxLevel: 10, XpPerLevel: 100})
	s.ObjectLayerDataCache = map[string]*content.ObjectLayer{"gel": {Cid: "cid-gel"}}
	ms.Ap(owner.ID).Layers = []world.ObjectLayerState{{ItemID: "gel", CID: "cid-gel", Quantity: 3, Slot: 0}}
	for s.NewEntityID() < owner.ID {
	}

	owner.InputQueue = []world.InputCommand{{Kind: world.InputKindItemDrop, Sequence: 1, TargetX: -1, TargetY: -1,
		Move: world.ItemMove{From: world.BoxBag, FromSlot: 0, Qty: 3}}}
	s.PhaseInput(0, ms)
	token := drops(ms)[0]
	ms.Br(token).CollectableAt = time.Time{}
	at := ms.Tf(token).Pos

	ms.Tf(owner.ID).Pos = at
	s.PhaseLoot(0, ms)
	if !ms.IsKind(token, world.EntityBot) {
		t.Fatal("the owner standing on its new drop does not take it back")
	}
	ms.Tf(owner.ID).Pos = world.Vec2{X: at.X + 5, Y: at.Y}
	s.PhaseLoot(0, ms)
	ms.Tf(owner.ID).Pos = at
	s.PhaseLoot(0, ms)
	if ms.IsKind(token, world.EntityBot) || itemQty(s.World, owner, "gel") != 3 {
		t.Fatalf("the owner takes the drop back after it stepped off: gel=%d", itemQty(s.World, owner, "gel"))
	}
}

// A world rebuild keeps the stacks players dropped on the maps that remain,
// and replaces the tokens the maps place.
func TestARebuildKeepsWhatPlayersDropped(t *testing.T) {
	s := lootServer(t)
	buildWorld(s, []string{"m1", "m2"}, lootEntity(3, 3, "gel"))
	now := time.Now()
	s.RestoreWorldDrops([]persist.WorldDrop{
		{ID: "d1", MapCode: "m1", ItemID: "gel", Quantity: 2, ExpiresAt: now.Add(time.Minute)},
		{ID: "d2", MapCode: "m2", ItemID: "gel", Quantity: 1, ExpiresAt: now.Add(time.Minute)},
	}, now)
	s.RebuildWorld(&pb.InstanceMessage{Code: "loot-test", MapCodes: []string{"m1"}},
		[]*pb.MapDataMessage{{Code: "m1", GridX: 20, GridY: 20, Entities: []*pb.EntityMessage{lootEntity(3, 3, "gel")}}}, nil, nil, nil, nil)
	ms := s.Maps["m1"]
	kept := 0
	for _, id := range drops(ms) {
		if ms.Br(id).WorldDrop == "d1" && ms.Br(id).DropQuantity == 2 {
			kept++
		}
	}
	if kept != 1 || len(worldTokens(ms)) != 1 {
		t.Fatalf("the drop stays and the map places its own token once: kept=%d world=%d", kept, len(worldTokens(ms)))
	}
}
