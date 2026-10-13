package game

import (
	"cyberia-server/game/world"
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

// consumeLoot is a loot default that makes its collector consume the item.
func consumeLoot(itemIDs ...string) *pb.EntityTypeDefault {
	return &pb.EntityTypeDefault{EntityType: "loot", LiveItemIds: itemIDs, Behavior: BehaviorConsumeOnPickupHeal}
}

// lootServer is a server with the given defaults and an object-layer cache that
// binds the gel and salve consumables, each healing 10, and the crate resource.
func lootServer(t *testing.T, defaults ...*pb.EntityTypeDefault) *GameServer {
	t.Helper()
	s := NewGameServer()
	if err := s.ApplyInstanceConfig(&pb.InstanceConfig{EntityBaseMaxLife: 100, EntityDefaults: defaults}); err != nil {
		t.Fatal(err)
	}
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{
		"gel":   {Data: ObjectLayerData{Item: Item{ID: "gel", Type: "consumable"}, Stats: Stats{Resistance: 10}}},
		"salve": {Data: ObjectLayerData{Item: Item{ID: "salve", Type: "consumable"}, Stats: Stats{Resistance: 10}}},
		"crate": {Data: ObjectLayerData{Item: Item{ID: "crate", Type: "resource"}}},
	}); err != nil {
		t.Fatal(err)
	}
	return s
}

// buildWorld builds an instance of 20×20 maps, each with the given entities.
func buildWorld(s *GameServer, mapCodes []string, entities ...*pb.EntityMessage) {
	maps := make([]*pb.MapDataMessage, 0, len(mapCodes))
	for _, code := range mapCodes {
		maps = append(maps, &pb.MapDataMessage{Code: code, GridX: 20, GridY: 20, Entities: entities})
	}
	s.BuildWorldFromInstance(&pb.InstanceMessage{Code: "loot-test", MapCodes: mapCodes}, maps, nil, nil, nil)
}

func lootEntity(x, y int32, itemIDs ...string) *pb.EntityMessage {
	return &pb.EntityMessage{EntityType: "loot", InitCellX: x, InitCellY: y, ObjectLayerItemIds: itemIDs}
}

func worldTokens(ms *world.MapState) []world.EntityID {
	var out []world.EntityID
	for _, id := range drops(ms) {
		if ms.Br(id).LootOrigin == world.LootOriginWorld {
			out = append(out, id)
		}
	}
	return out
}

// onlyToken is the one world token of a map.
func onlyToken(t *testing.T, ms *world.MapState) world.EntityID {
	t.Helper()
	tokens := worldTokens(ms)
	if len(tokens) != 1 {
		t.Fatalf("map holds %d world tokens, want 1", len(tokens))
	}
	return tokens[0]
}

// woundedPlayer stands a player with the given life at the point, on map "m".
func woundedPlayer(s *world.World, ms *world.MapState, id world.EntityID, at world.Vec2, life float64) *world.Session {
	player := &world.Session{ID: id, MapCode: "m"}
	if s.Sessions == nil {
		s.Sessions = map[world.EntityID]*world.Session{}
	}
	s.Sessions[id] = player
	ms.Spawn(id, world.EntityData{Kind: world.EntityPlayer}, world.Transform{Pos: at, Size: world.Vec2{X: 1, Y: 1}}, world.Appearance{})
	ms.AddVitals(id, world.Vitals{Life: life, MaxLife: 100})
	ms.AddMotion(id, world.Motion{})
	ms.AddMortal(id, &world.Mortal{})
	return player
}

// gelMap builds map "m" with one placed gel token and returns the map and the token.
func gelMap(t *testing.T, s *GameServer) (*world.MapState, world.EntityID) {
	t.Helper()
	buildWorld(s, []string{"m"}, lootEntity(4, 4, "gel"))
	ms := s.Maps["m"]
	return ms, onlyToken(t, ms)
}

func TestAMapHoldsOnlyTheLootItPlaces(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	buildWorld(s, []string{"empty"})
	if got := len(worldTokens(s.Maps["empty"])); got != 0 {
		t.Fatalf("a map that places no loot holds %d tokens", got)
	}

	codes := []string{"north", "south"}
	buildWorld(s, codes, lootEntity(2, 3, "gel"), lootEntity(8, 8, "gel"))
	// A rebuild replaces every map, so tokens never accumulate.
	buildWorld(s, codes, lootEntity(2, 3, "gel"), lootEntity(8, 8, "gel"))
	for _, code := range codes {
		tokens := worldTokens(s.Maps[code])
		if len(tokens) != 2 {
			t.Fatalf("map %q holds %d gel tokens, want its 2 placements", code, len(tokens))
		}
		for _, token := range tokens {
			if at := s.Maps[code].Tf(token).Pos; at != (world.Vec2{X: 2, Y: 3}) && at != (world.Vec2{X: 8, Y: 8}) {
				t.Fatalf("a token left its placed cell: %+v", at)
			}
			if s.Maps[code].Br(token).LootOutcome != world.LootOutcomeConsume || s.Maps[code].Br(token).LootCollection != world.LootCollectFirstContact ||
				len(activeLayers(s.Maps[code].Ap(token).Layers)) != 1 {
				t.Fatalf("map %q gel token = %+v", code, token)
			}
			if !s.Maps[code].Br(token).CollectableAt.IsZero() || !s.Maps[code].Br(token).ExpiresAt.IsZero() || len(s.Maps[code].Br(token).LootContributors) != 0 {
				t.Fatalf("a world token is collectable at once, never expires and has no contributor: %+v", token)
			}
		}
	}
}

func TestWorldLootShowsOneItemBoundOrNot(t *testing.T) {
	cases := map[string]struct {
		defaults []*pb.EntityTypeDefault
		entities []*pb.EntityMessage
		want     map[string]world.LootOutcome
	}{
		"a default alone": {defaults: []*pb.EntityTypeDefault{consumeLoot("gel")}},
		"a placement of two items": {
			defaults: []*pb.EntityTypeDefault{consumeLoot("gel", "salve")},
			entities: []*pb.EntityMessage{lootEntity(2, 2, "gel", "salve")},
		},
		"a default without the consume behavior": {
			defaults: []*pb.EntityTypeDefault{{EntityType: "loot", LiveItemIds: []string{"gel"}}},
			entities: []*pb.EntityMessage{lootEntity(2, 2, "gel")},
			want:     map[string]world.LootOutcome{"gel": world.LootOutcomeItem},
		},
		"an unbound item to consume": {
			defaults: []*pb.EntityTypeDefault{consumeLoot("orb")},
			entities: []*pb.EntityMessage{lootEntity(2, 2, "orb")},
			want:     map[string]world.LootOutcome{"orb": world.LootOutcomeConsume},
		},
		"a placed resource to consume": {
			defaults: []*pb.EntityTypeDefault{consumeLoot("crate")},
			entities: []*pb.EntityMessage{lootEntity(2, 2, "crate")},
			want:     map[string]world.LootOutcome{"crate": world.LootOutcomeConsume},
		},
		"a placed resource to grant": {
			entities: []*pb.EntityMessage{lootEntity(2, 2, "crate")},
			want:     map[string]world.LootOutcome{"crate": world.LootOutcomeItem},
		},
	}
	for name, tc := range cases {
		s := lootServer(t, tc.defaults...)
		buildWorld(s, []string{"m"}, tc.entities...)
		got := map[string]world.LootOutcome{}
		for _, token := range worldTokens(s.Maps["m"]) {
			got[s.Maps["m"].Br(token).DropItemID] = s.Maps["m"].Br(token).LootOutcome
		}
		if len(got) != len(tc.want) {
			t.Errorf("%s: tokens = %v, want %v", name, got, tc.want)
		}
		for itemID, outcome := range tc.want {
			if got[itemID] != outcome {
				t.Errorf("%s: %s outcome = %v, want %v", name, itemID, got[itemID], outcome)
			}
		}
	}
}

func TestConsumingAPickupHealsByTheItemOnceAndGrantsNothing(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	ms, token := gelMap(t, s)
	first := woundedPlayer(s.World, ms, 101, ms.Tf(token).Pos, 50)
	second := woundedPlayer(s.World, ms, 102, ms.Tf(token).Pos, 50)

	for range 3 {
		s.phaseLoot(0, ms)
	}

	if ok := ms.IsKind(token, world.EntityBot); ok {
		t.Fatal("the consumed token stays on the map")
	}
	if healed := ms.Vt(first.ID).Life + ms.Vt(second.ID).Life - 100; healed != 10 {
		t.Fatalf("one gel restored %v life, want its resistance 10", healed)
	}
	if len(ms.Ap(first.ID).Layers)+len(ms.Ap(second.ID).Layers) != 0 {
		t.Fatal("a consumed pickup entered an inventory")
	}
}

func TestConsumingAPickupNeverPassesMaxLife(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	ms, token := gelMap(t, s)
	player := woundedPlayer(s.World, ms, 103, ms.Tf(token).Pos, 95)

	s.phaseLoot(0, ms)

	if ms.Vt(player.ID).Life != 100 {
		t.Fatalf("life = %v, want max life 100", ms.Vt(player.ID).Life)
	}
}

func TestAPickupGoesToTheFirstEligibleCollector(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	ms, token := gelMap(t, s)

	ghost := woundedPlayer(s.World, ms, 105, ms.Tf(token).Pos, 10)
	ms.Vt(ghost.ID).RespawnAt = time.Now().Add(time.Minute)
	frozen := woundedPlayer(s.World, ms, 106, ms.Tf(token).Pos, 10)
	frozen.Frozen = true
	for i, behavior := range []string{BehaviorSkill, BehaviorCoin, BehaviorDrop, BehaviorProvider} {
		bot := world.EntityID(200 + i)
		addBot(ms, bot, *ms.Tf(token), &world.Brain{}, world.Motion{}, behavior)
		ms.Vt(bot).Life = 10
	}

	s.phaseLoot(0, ms)

	if ok := ms.IsKind(token, world.EntityBot); !ok {
		t.Fatal("an ineligible entity took the token")
	}
	if ms.Vt(ghost.ID).Life != 10 || ms.Vt(frozen.ID).Life != 10 {
		t.Fatalf("ineligible players healed: ghost=%v frozen=%v", ms.Vt(ghost.ID).Life, ms.Vt(frozen.ID).Life)
	}

	wounded := woundedPlayer(s.World, ms, 108, ms.Tf(token).Pos, 60)
	s.phaseLoot(0, ms)
	if ok := ms.IsKind(token, world.EntityBot); ok || ms.Vt(wounded.ID).Life != 70 {
		t.Fatalf("a wounded player must consume the token: life=%v", ms.Vt(wounded.ID).Life)
	}
}

func TestACollectorAtFullLifeKeepsAPickupAndTheTokenComesBack(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	ms, token := gelMap(t, s)
	at := *ms.Tf(token)
	player := woundedPlayer(s.World, ms, 104, at.Pos, 100)
	const bot world.EntityID = 109
	addBot(ms, bot, world.Transform{Pos: world.Vec2{X: 15, Y: 15}, Size: at.Size}, &world.Brain{}, world.Motion{}, BehaviorHostile)

	s.phaseLoot(0, ms)
	if ok := ms.IsKind(token, world.EntityBot); ok || ms.Vt(player.ID).Life != 100 || s.playerItemQuantity(player, "gel") != 1 {
		t.Fatalf("a full-life player must keep the gel: life=%v gel=%d", ms.Vt(player.ID).Life, s.playerItemQuantity(player, "gel"))
	}
	if len(s.lootRespawns["m"]) != 1 {
		t.Fatalf("respawns = %+v, want the kept token back like a consumed one", s.lootRespawns["m"])
	}

	s.lootRespawns["m"][0].at = time.Now()
	ms.Tf(player.ID).Pos = world.Vec2{X: -10, Y: -10}
	ms.Tf(bot).Pos = at.Pos
	s.phaseLoot(0, ms)
	if layerQuantity(ms.Ap(bot).Layers, "gel") != 1 || ms.Vt(bot).Life != 100 {
		t.Fatalf("a full-life bot must keep the gel: life=%v layers=%+v", ms.Vt(bot).Life, ms.Ap(bot).Layers)
	}
}

func TestAPickupHealsByItsResistanceAndComesBackAfterItsRespawnTime(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	placed := lootEntity(4, 4, "gel")
	placed.RespawnDurationMs = 5000
	buildWorld(s, []string{"m"}, placed)
	ms := s.Maps["m"]
	token := onlyToken(t, ms)
	cell := ms.Tf(token).Pos
	player := woundedPlayer(s.World, ms, 103, cell, 50)

	now := time.Now()
	s.phaseLoot(0, ms)
	if ms.Vt(player.ID).Life != 60 || len(worldTokens(ms)) != 0 {
		t.Fatalf("life=%v tokens=%d, want a heal of 10 that consumes the token", ms.Vt(player.ID).Life, len(worldTokens(ms)))
	}
	if len(s.lootRespawns["m"]) != 1 || s.lootRespawns["m"][0].at.Sub(now) < 5*time.Second {
		t.Fatalf("respawns = %+v, want the gel back after 5 s", s.lootRespawns["m"])
	}

	s.phaseLoot(0, ms)
	if len(worldTokens(ms)) != 0 {
		t.Fatal("the token came back before its respawn time")
	}
	s.lootRespawns["m"][0].at = now
	ms.Tf(player.ID).Pos = world.Vec2{X: -10, Y: -10}
	s.phaseLoot(0, ms)
	back := onlyToken(t, ms)
	if ms.Tf(back).Pos != cell || ms.Br(back).LootOutcome != world.LootOutcomeConsume || ms.Mt(back).RespawnDuration != 5*time.Second ||
		len(s.lootRespawns["m"]) != 0 {
		t.Fatalf("the token must come back on its cell with its respawn time: %+v", back)
	}
}

func TestAPickupWithoutARespawnTimeTakesTheDefault(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	buildWorld(s, []string{"m"}, lootEntity(4, 4, "gel"), lootEntity(10, 10, "salve"))
	ms := s.Maps["m"]
	for _, token := range worldTokens(ms) {
		woundedPlayer(s.World, ms, 1000+token, ms.Tf(token).Pos, 50)
	}

	now := time.Now()
	s.phaseLoot(0, ms)

	// The granted salve never comes back; the consumed gel does, after the default respawn time.
	if len(s.lootRespawns["m"]) != 1 || s.lootRespawns["m"][0].itemID != "gel" ||
		s.lootRespawns["m"][0].at.Sub(now) < respawnDurationDefaultMs*time.Millisecond {
		t.Fatalf("respawns = %+v, want only the gel after the default respawn time", s.lootRespawns["m"])
	}
}

func TestABotConsumesAPickup(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	ms, token := gelMap(t, s)
	const bot world.EntityID = 300
	addBot(ms, bot, *ms.Tf(token), &world.Brain{}, world.Motion{}, BehaviorHostile)
	ms.Vt(bot).Life = 30

	s.phaseLoot(0, ms)

	if ok := ms.IsKind(token, world.EntityBot); ok || ms.Vt(bot).Life != 40 {
		t.Fatalf("the bot must consume the token: life=%v", ms.Vt(bot).Life)
	}
}

func TestAGrantPickupEntersTheInventoryUnused(t *testing.T) {
	s := lootServer(t)
	buildWorld(s, []string{"m"}, lootEntity(4, 4, "salve"))
	ms := s.Maps["m"]
	player := woundedPlayer(s.World, ms, 103, ms.Tf(onlyToken(t, ms)).Pos, 100)

	s.phaseLoot(0, ms)

	if s.playerItemQuantity(player, "salve") != 1 || ms.Vt(player.ID).Life != 100 {
		t.Fatalf("a grant pickup must reach the inventory unused: qty=%d life=%v",
			s.playerItemQuantity(player, "salve"), ms.Vt(player.ID).Life)
	}
}

func TestDeathDropsKeepTheContributorRace(t *testing.T) {
	s, ms, contributor := movementFixture(t)
	for range 1000 { // drops take ids after the players
		s.NewEntityID()
	}
	s.coinItemID = "coin"
	outsider := woundedPlayer(s.World, ms, 107, world.Vec2{}, 50)
	ledger := map[world.EntityID]float64{contributor.ID: 5}
	s.spawnDrops(ms, world.Vec2{X: 10, Y: 10},
		EntityTypeDefaultConfig{DropItemIDs: []string{"shard"}}, 7, ledger)

	tokens := drops(ms)
	if len(tokens) != 2 {
		t.Fatalf("got %d death drops, want an item and a coin token", len(tokens))
	}
	for i, token := range tokens {
		// Scattered tokens may overlap: set them apart so each one is collected alone.
		ms.Tf(token).Pos = world.Vec2{X: float64(3 + 10*i), Y: 3}
		if ms.Br(token).LootOrigin != world.LootOriginDeath || ms.Br(token).LootCollection != world.LootCollectContributors ||
			ms.Br(token).ExpiresAt.IsZero() || !ms.Br(token).CollectableAt.After(time.Now()) {
			t.Fatalf("death drop = %+v", token)
		}
		want := world.LootOutcomeItem
		if ms.Br(token).DropItemID == "coin" {
			want = world.LootOutcomeCoin
		}
		if ms.Br(token).LootOutcome != want {
			t.Fatalf("%s outcome = %v, want %v", ms.Br(token).DropItemID, ms.Br(token).LootOutcome, want)
		}
		ms.Br(token).CollectableAt = time.Time{}
	}

	for _, token := range tokens {
		ms.Tf(contributor.ID).Pos = world.Vec2{X: 19, Y: 0}
		ms.Tf(outsider.ID).Pos = ms.Tf(token).Pos
		s.phaseLoot(0, ms)
		if ok := ms.IsKind(token, world.EntityBot); !ok {
			t.Fatalf("a non-contributor took %s", ms.Br(token).DropItemID)
		}
		ms.Tf(outsider.ID).Pos = world.Vec2{X: 19, Y: 19}
		ms.Tf(contributor.ID).Pos = ms.Tf(token).Pos
		s.phaseLoot(0, ms)
		if ok := ms.IsKind(token, world.EntityBot); ok {
			t.Fatalf("the contributor could not take %s", ms.Br(token).DropItemID)
		}
	}
	if contributor.Coins != 7 || s.playerItemQuantity(contributor, "shard") != 1 {
		t.Fatalf("coins = %d shards = %d, want 7 and 1", contributor.Coins, s.playerItemQuantity(contributor, "shard"))
	}
}

func TestOnlyAWorldTokenAdmitsEveryViewer(t *testing.T) {
	worldToken := &world.Brain{LootCollection: world.LootCollectFirstContact}
	death := &world.Brain{LootCollection: world.LootCollectContributors, LootContributors: map[world.EntityID]struct{}{1: {}}}
	if !lootAdmits(worldToken, 99) || !lootAdmits(death, 1) || lootAdmits(death, 2) {
		t.Fatal("collection policy admits the wrong viewers")
	}
	consuming := &world.Brain{LootOutcome: world.LootOutcomeConsume}
	if lootConsumes(death, &world.Vitals{Life: 10, MaxLife: 100}) || !lootConsumes(consuming, &world.Vitals{Life: 99, MaxLife: 100}) ||
		lootConsumes(consuming, &world.Vitals{Life: 100, MaxLife: 100}) {
		t.Fatal("only a consuming token heals, and only a collector that misses life")
	}
}
