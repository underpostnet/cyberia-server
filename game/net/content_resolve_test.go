package net

import (
	"context"
	"errors"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/persist"
	"cyberia-server/game/sim"
	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
)

// fakeSource is the engine a placement resolves labels against: it answers
// the content of a label as the engine does, the definition with its skill and
// what the skill summons.
type fakeSource struct {
	mu      sync.Mutex
	calls   map[string]int
	layers  map[string]*content.ObjectLayer
	skills  map[string][]content.SkillDefinition
	release string
	err     error
	delay   time.Duration
}

func (f *fakeSource) FetchItemContent(ctx context.Context, itemID string) (*content.ItemContent, error) {
	f.mu.Lock()
	if f.calls == nil {
		f.calls = map[string]int{}
	}
	f.calls[itemID]++
	err := f.err
	c := &content.ItemContent{ReleaseID: f.release, Layers: map[string]*content.ObjectLayer{},
		Skills: map[string][]content.SkillDefinition{}, Unavailable: map[string]string{}}
	queue := []string{itemID}
	for len(queue) > 0 {
		label := queue[0]
		queue = queue[1:]
		if f.layers[label] == nil {
			c.Unavailable[label] = "unbound"
			continue
		}
		c.Layers[label] = f.layers[label]
		for _, def := range f.skills[label] {
			c.Skills[label] = append(c.Skills[label], def)
			if summoned := def.SummonedEntityItemID; summoned != "" && summoned != label && c.Layers[summoned] == nil {
				queue = append(queue, summoned)
			}
		}
	}
	f.mu.Unlock()
	if f.delay > 0 {
		select {
		case <-time.After(f.delay):
		case <-ctx.Done():
			return nil, ctx.Err()
		}
	}
	if err != nil {
		return nil, err
	}
	return c, nil
}

func (f *fakeSource) callsOf(itemID string) int {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.calls[itemID]
}

func layer(itemID, cid, itemType string) *content.ObjectLayer {
	return &content.ObjectLayer{Cid: cid, Data: content.ObjectLayerData{Item: content.Item{ID: itemID, Type: itemType}}}
}

// travelerState is a saved state from another world: one label this world
// names, one it resolves as granted, one bound to another definition here, one
// with no binding here, and one the engine answers for another label.
func travelerState() persist.State {
	return persist.State{
		Bag: []persist.Stack{
			{ItemID: "sword", CID: "cid-sword", Quantity: 1, Slot: 0},
			{ItemID: "lantern", CID: "cid-lantern", Quantity: 1, Slot: 1},
			{ItemID: "relic", CID: "cid-relic-v1", Quantity: 1, Slot: 2},
			{ItemID: "ghost", CID: "cid-ghost", Quantity: 1, Slot: 3},
			{ItemID: "mask", CID: "cid-mask", Quantity: 1, Slot: 4},
		},
		Position: persist.Position{MapCode: "test", X: 7, Y: 9},
	}
}

func travelerSource() *fakeSource {
	return &fakeSource{layers: map[string]*content.ObjectLayer{
		"lantern": layer("lantern", "cid-lantern", "weapon"),
		"relic":   layer("relic", "cid-relic-v2", "weapon"),
		"mask":    layer("other-mask", "cid-mask", "weapon"),
	}}
}

func TestATravelerLoadsTheStacksTheEngineResolvesAsGranted(t *testing.T) {
	engine := &fakeEngine{saved: travelerState()}
	srv := newAccountServer(t, engine)
	source := travelerSource()
	srv.game.SetContentSource(source)
	conn := connect(t, srv, testTicket)
	readInitData(t, conn)
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })

	srv.game.Mu.Lock()
	var player *world.Session
	for _, p := range srv.game.Sessions {
		player = p
	}
	bag := map[string]string{}
	for _, l := range *srv.game.InventoryLayers(player) {
		bag[l.ItemID] = l.CID
	}
	held := map[string]string{}
	for _, h := range srv.game.Held[player.PlayerID] {
		held[h.Stack.ItemID] = h.Reason
	}
	srv.game.Mu.Unlock()

	if bag["sword"] != "cid-sword" || bag["lantern"] != "cid-lantern" {
		t.Fatalf("a warm label and a resolved one load with their definitions: %v", bag)
	}
	want := map[string]string{"relic": persist.HeldRebound, "ghost": persist.HeldUnknown, "mask": persist.HeldUnknown}
	for itemID, reason := range want {
		if held[itemID] != reason {
			t.Fatalf("%s is held as %s, never replaced by another definition: %v", itemID, reason, held)
		}
	}
	if source.callsOf("sword") != 0 || source.callsOf("lantern") != 1 {
		t.Fatalf("only the labels the world lacks are fetched, once: %v", source.calls)
	}
}

func TestAResolutionThatCannotReachTheEngineRefusesThePlacement(t *testing.T) {
	engine := &fakeEngine{saved: travelerState()}
	srv := newAccountServer(t, engine)
	srv.game.SetContentSource(&fakeSource{err: errors.New("engine unreachable")})
	conn := connect(t, srv, testTicket)
	if reason := readSessionEnd(t, conn); reason != EndUnavailable {
		t.Fatalf("session_end reason = %q, want %q", reason, EndUnavailable)
	}
	waitFor(t, "the lease release", func() bool {
		engine.mu.Lock()
		defer engine.mu.Unlock()
		return engine.released == 1
	})
	if n := srv.clientCount(); n != 0 {
		t.Fatalf("a refused placement leaves no player, got %d", n)
	}
	if commit, ok := engine.lastCommit(); ok && len(commit.State.Bag) != len(travelerState().Bag) {
		t.Fatalf("the profile keeps every stack it had: %+v", commit.State.Bag)
	}
}

func TestConcurrentPlacementsFetchALabelOnce(t *testing.T) {
	source := &fakeSource{layers: map[string]*content.ObjectLayer{"lantern": layer("lantern", "cid-lantern", "weapon")}, delay: 50 * time.Millisecond}
	resolver := &contentResolver{source: source, inflight: map[string]*resolveCall{}}
	var wg sync.WaitGroup
	var found atomic.Int32
	for range 8 {
		wg.Add(1)
		go func() {
			defer wg.Done()
			if c, err := resolver.fetch(context.Background(), "lantern"); err == nil && c.Layers["lantern"].Cid == "cid-lantern" {
				found.Add(1)
			}
		}()
	}
	wg.Wait()
	if source.callsOf("lantern") != 1 || found.Load() != 8 {
		t.Fatalf("one fetch serves every waiter: calls=%d found=%d", source.callsOf("lantern"), found.Load())
	}
	if c, _ := resolver.fetch(context.Background(), "lantern"); c == nil || source.callsOf("lantern") != 2 {
		t.Fatal("a settled fetch leaves nothing behind: the next placement asks the engine again")
	}
}

func TestAWaiterThatGivesUpLeavesTheFetchToTheOthers(t *testing.T) {
	source := &fakeSource{layers: map[string]*content.ObjectLayer{"lantern": layer("lantern", "cid-lantern", "weapon")}, delay: 80 * time.Millisecond}
	resolver := &contentResolver{source: source, inflight: map[string]*resolveCall{}}
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Millisecond)
	defer cancel()
	if _, err := resolver.fetch(ctx, "lantern"); !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("a waiter past its deadline stops waiting, got %v", err)
	}
	if c, err := resolver.fetch(context.Background(), "lantern"); err != nil || c.Layers["lantern"].Cid != "cid-lantern" {
		t.Fatalf("the fetch goes on for the next waiter: %v %v", c, err)
	}
	if source.callsOf("lantern") != 1 {
		t.Fatalf("calls = %d, want 1", source.callsOf("lantern"))
	}
}

func TestAPlayerInTheWorldReceivesTheDefinitionsResolvedForATraveler(t *testing.T) {
	engine := &fakeEngine{guest: true, saved: travelerState()}
	srv := newAccountServer(t, engine)
	srv.game.SetContentSource(travelerSource())
	observer := connect(t, srv, testTicket)
	var joined struct {
		ObjectLayers map[string]OLMeta `json:"objectLayers"`
	}
	readPayload(t, observer, "metadata", &joined)
	if joined.ObjectLayers["sword"].Cid != "cid-sword" || joined.ObjectLayers["lantern"].Cid != "" {
		t.Fatalf("the observer joins with the warm definitions only: %v", joined.ObjectLayers)
	}
	waitFor(t, "the observer to register", func() bool { return srv.clientCount() == 1 })

	engine.mu.Lock()
	engine.guest = false
	engine.mu.Unlock()
	traveler := connect(t, srv, testTicket)
	readInitData(t, traveler)
	waitFor(t, "the traveler to register", func() bool { return srv.clientCount() == 2 })
	srv.game.Mu.Lock()
	srv.game.phaseReplication(srv.game.Maps["test"])
	srv.game.Mu.Unlock()

	var shared struct {
		ObjectLayers map[string]OLMeta `json:"objectLayers"`
	}
	readPayload(t, observer, "metadata", &shared)
	if shared.ObjectLayers["lantern"].Cid != "cid-lantern" || shared.ObjectLayers["relic"].Cid != "cid-relic-v2" {
		t.Fatalf("the observer receives the definitions resolved for the traveler: %v", shared.ObjectLayers)
	}
	if _, resent := shared.ObjectLayers["sword"]; resent {
		t.Fatal("a definition the observer holds is not sent again")
	}
	if content := srv.game.RuntimeMetrics().Content; content.Outcomes["resolved"] != 2 || content.Outcomes["resident"] != 1 ||
		content.Outcomes["unbound"] != 1 || content.Outcomes["mismatch"] != 1 || content.DefinitionsShared != 2 {
		t.Fatalf("content metrics = %+v", content)
	}
}

// gunner is a player of another world whose active gun fires a projectile
// that summons a bullet. This world warms neither.
func gunner(source *fakeSource) persist.State {
	source.layers["gun"] = layer("gun", "cid-gun", "weapon")
	source.layers["bullet"] = layer("bullet", "cid-bullet", "skill")
	source.skills = map[string][]content.SkillDefinition{
		"gun": {{LogicEventID: "projectile", Name: "Shot", SummonedEntityItemID: "bullet"}},
	}
	return persist.State{
		Bag:      []persist.Stack{{ItemID: "gun", CID: "cid-gun", Quantity: 1, Slot: 0, Active: true}},
		Position: persist.Position{MapCode: "test", X: 7, Y: 9},
	}
}

// placedPlayer is the one player of the server, its map and the reasons of its held stacks.
func placedPlayer(srv *wsTestServer) (*world.Session, *world.MapState, map[string]string) {
	srv.game.Mu.Lock()
	defer srv.game.Mu.Unlock()
	var player *world.Session
	for _, p := range srv.game.Sessions {
		player = p
	}
	held := map[string]string{}
	for _, h := range srv.game.Held[player.PlayerID] {
		held[h.Stack.ItemID] = h.Reason
	}
	return player, srv.game.Maps[player.MapCode], held
}

func TestATravelerFiresTheSkillOfAnItemThisWorldDoesNotWarm(t *testing.T) {
	source := &fakeSource{layers: map[string]*content.ObjectLayer{}}
	engine := &fakeEngine{saved: gunner(source)}
	srv := newAccountServer(t, engine)
	if err := srv.game.ApplyInstanceConfig(&pb.InstanceConfig{
		EntityBaseMaxLife: 100, InitialLifeFraction: 1, AoiRadius: 10,
		SkillRules: &pb.SkillRules{ProjectileSpawnChance: 1, ProjectileLifetimeMs: 5000, ProjectileWidth: 1, ProjectileHeight: 1},
	}); err != nil {
		t.Fatal(err)
	}
	srv.game.SetContentSource(source)
	conn := connect(t, srv, testTicket)
	init := readInitData(t, conn)
	if len(init.SkillMap["gun"]) != 1 || init.SkillMap["gun"][0].Name != "Shot" {
		t.Fatalf("the traveler joins with the skill of its gun: %v", init.SkillMap)
	}
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })

	player, ms, held := placedPlayer(srv)
	if len(held) != 0 {
		t.Fatalf("nothing is held: %v", held)
	}
	srv.game.Mu.Lock()
	sim.ThawPlayer(player)
	srv.game.HandlePlayerTapAction(player, ms, world.Vec2{X: 12, Y: 9})
	var bullets int
	for id := range ms.IDs() {
		for _, l := range ms.Ap(id).Layers {
			if l.ItemID == "bullet" {
				bullets++
			}
		}
	}
	srv.game.Mu.Unlock()
	if bullets != 1 {
		t.Fatalf("a tap fires the resolved projectile and spawns one bullet, got %d", bullets)
	}
	if source.callsOf("gun") != 1 || source.callsOf("bullet") != 0 {
		t.Fatalf("one fetch carries the gun and what its skill summons: %v", source.calls)
	}
}

func TestAWorldHoldsTheItemWhoseContentItCannotRunAndRunsNoneOfIt(t *testing.T) {
	cases := []struct {
		name   string
		change func(source *fakeSource)
		reason string
	}{
		{"a handler this server does not run", func(f *fakeSource) {
			f.skills["gun"] = []content.SkillDefinition{{LogicEventID: "teleport", SummonedEntityItemID: "bullet"}}
		}, persist.HeldUnsupported},
		{"content of another release", func(f *fakeSource) { f.release = "r9" }, persist.HeldIncompatible},
		{"a summon the engine cannot carry", func(f *fakeSource) { delete(f.layers, "bullet") }, persist.HeldUnknown},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			source := &fakeSource{layers: map[string]*content.ObjectLayer{}}
			engine := &fakeEngine{saved: gunner(source)}
			tc.change(source)
			srv := newAccountServer(t, engine)
			srv.game.SetContentSource(source)
			conn := connect(t, srv, testTicket)
			readInitData(t, conn)
			waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })

			_, _, held := placedPlayer(srv)
			if held["gun"] != tc.reason {
				t.Fatalf("the gun is held as %q: %v", tc.reason, held)
			}
			srv.game.Mu.Lock()
			defer srv.game.Mu.Unlock()
			if _, cached := srv.game.ObjectLayerDataCache["bullet"]; cached || srv.game.SkillConfig["gun"] != nil {
				t.Fatal("no part of the refused content runs")
			}
		})
	}
}

func TestAFetchThatLosesTheEngineOnceAsksAgain(t *testing.T) {
	source := &flakySource{fakeSource: fakeSource{layers: map[string]*content.ObjectLayer{"lantern": layer("lantern", "cid-lantern", "weapon")}}}
	resolver := &contentResolver{source: source, inflight: map[string]*resolveCall{}}
	c, err := resolver.fetch(context.Background(), "lantern")
	if err != nil || c.Layers["lantern"] == nil || source.callsOf("lantern") != 2 {
		t.Fatalf("the second attempt answers: %v %v calls=%d", c, err, source.callsOf("lantern"))
	}
}

// flakySource fails its first answer.
type flakySource struct {
	fakeSource
	failed atomic.Bool
}

func (f *flakySource) FetchItemContent(ctx context.Context, itemID string) (*content.ItemContent, error) {
	c, err := f.fakeSource.FetchItemContent(ctx, itemID)
	if !f.failed.Swap(true) {
		return nil, errors.New("connection reset")
	}
	return c, err
}

func TestAReloadRunsTheSkillTheReleaseOfTheWorldDefines(t *testing.T) {
	source := &fakeSource{layers: map[string]*content.ObjectLayer{}}
	engine := &fakeEngine{saved: gunner(source)}
	srv := newAccountServer(t, engine)
	release := "r1"
	source.release = release
	srv.game.FollowRelease(func() string { return release })
	srv.game.SetContentSource(source)
	conn := connect(t, srv, testTicket)
	readInitData(t, conn)
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })

	source.mu.Lock()
	source.skills["gun"] = []content.SkillDefinition{{LogicEventID: "projectile", Name: "Burst", SummonedEntityItemID: "bullet"}}
	source.mu.Unlock()
	srv.game.RefreshResolvedContent(context.Background())
	srv.game.Mu.Lock()
	name := srv.game.SkillConfig["gun"][0].Name
	srv.game.Mu.Unlock()
	if name != "Burst" {
		t.Fatalf("a reload takes the skill the engine serves now, got %q", name)
	}

	release = "r2"
	srv.game.RefreshResolvedContent(context.Background())
	srv.game.Mu.Lock()
	defer srv.game.Mu.Unlock()
	if srv.game.SkillConfig["gun"] != nil {
		t.Fatal("no skill of another release runs after a promotion")
	}
	if _, cached := srv.game.ObjectLayerDataCache["gun"]; !cached {
		t.Fatal("the definition stays while the player holds it")
	}
}

func TestAPlayerInTheWorldReceivesTheSkillsResolvedForATraveler(t *testing.T) {
	source := &fakeSource{layers: map[string]*content.ObjectLayer{}}
	engine := &fakeEngine{guest: true, saved: gunner(source)}
	srv := newAccountServer(t, engine)
	srv.game.SetContentSource(source)
	observer := connect(t, srv, testTicket)
	readPayload(t, observer, "metadata", &struct{}{})
	waitFor(t, "the observer to register", func() bool { return srv.clientCount() == 1 })

	engine.mu.Lock()
	engine.guest = false
	engine.mu.Unlock()
	readInitData(t, connect(t, srv, testTicket))
	waitFor(t, "the traveler to register", func() bool { return srv.clientCount() == 2 })
	srv.game.Mu.Lock()
	srv.game.phaseReplication(srv.game.Maps["test"])
	srv.game.Mu.Unlock()

	var shared struct {
		ObjectLayers map[string]OLMeta          `json:"objectLayers"`
		SkillMap     map[string][]SkillMapEntry `json:"skillMap"`
	}
	readPayload(t, observer, "metadata", &shared)
	if shared.ObjectLayers["bullet"].Cid != "cid-bullet" || len(shared.SkillMap["gun"]) != 1 || len(shared.SkillMap["bullet"]) != 0 {
		t.Fatalf("the observer receives the gun, its bullet and its skill: %v %v", shared.ObjectLayers, shared.SkillMap)
	}
}
