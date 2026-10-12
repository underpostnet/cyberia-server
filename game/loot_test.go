package game

import (
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

func worldTokens(ms *MapState) []*BotState {
	var out []*BotState
	for _, bot := range drops(ms) {
		if bot.LootOrigin == LootOriginWorld {
			out = append(out, bot)
		}
	}
	return out
}

// onlyToken is the one world token of a map.
func onlyToken(t *testing.T, ms *MapState) *BotState {
	t.Helper()
	tokens := worldTokens(ms)
	if len(tokens) != 1 {
		t.Fatalf("map holds %d world tokens, want 1", len(tokens))
	}
	return tokens[0]
}

// woundedPlayer stands a player with the given life at the point, on map "m".
func woundedPlayer(ms *MapState, id EntityID, at Vec2, life float64) *PlayerState {
	player := &PlayerState{}
	player.ID = id
	player.MapCode = "m"
	ms.players[id] = player
	ms.spawn(id, EntityData{Kind: EntityPlayer}, Transform{Pos: at, Size: Vec2{X: 1, Y: 1}}, Appearance{})
	ms.addVitals(id, Vitals{Life: life, MaxLife: 100})
	ms.addMotion(id, Motion{})
	return player
}

// gelMap builds map "m" with one placed gel token and returns the map and the token.
func gelMap(t *testing.T, s *GameServer) (*MapState, *BotState) {
	t.Helper()
	buildWorld(s, []string{"m"}, lootEntity(4, 4, "gel"))
	ms := s.maps["m"]
	return ms, onlyToken(t, ms)
}

func TestAMapHoldsOnlyTheLootItPlaces(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	buildWorld(s, []string{"empty"})
	if got := len(worldTokens(s.maps["empty"])); got != 0 {
		t.Fatalf("a map that places no loot holds %d tokens", got)
	}

	codes := []string{"north", "south"}
	buildWorld(s, codes, lootEntity(2, 3, "gel"), lootEntity(8, 8, "gel"))
	// A rebuild replaces every map, so tokens never accumulate.
	buildWorld(s, codes, lootEntity(2, 3, "gel"), lootEntity(8, 8, "gel"))
	for _, code := range codes {
		tokens := worldTokens(s.maps[code])
		if len(tokens) != 2 {
			t.Fatalf("map %q holds %d gel tokens, want its 2 placements", code, len(tokens))
		}
		for _, token := range tokens {
			if at := s.maps[code].tf(token.ID).Pos; at != (Vec2{X: 2, Y: 3}) && at != (Vec2{X: 8, Y: 8}) {
				t.Fatalf("a token left its placed cell: %+v", at)
			}
			if token.LootOutcome != LootOutcomeConsume || token.LootCollection != LootCollectFirstContact ||
				len(activeLayers(s.maps[code].ap(token.ID).Layers)) != 1 {
				t.Fatalf("map %q gel token = %+v", code, token)
			}
			if !token.CollectableAt.IsZero() || !token.ExpiresAt.IsZero() || len(token.LootContributors) != 0 {
				t.Fatalf("a world token is collectable at once, never expires and has no contributor: %+v", token)
			}
		}
	}
}

func TestWorldLootShowsOneItemBoundOrNot(t *testing.T) {
	cases := map[string]struct {
		defaults []*pb.EntityTypeDefault
		entities []*pb.EntityMessage
		want     map[string]LootOutcome
	}{
		"a default alone": {defaults: []*pb.EntityTypeDefault{consumeLoot("gel")}},
		"a placement of two items": {
			defaults: []*pb.EntityTypeDefault{consumeLoot("gel", "salve")},
			entities: []*pb.EntityMessage{lootEntity(2, 2, "gel", "salve")},
		},
		"a default without the consume behavior": {
			defaults: []*pb.EntityTypeDefault{{EntityType: "loot", LiveItemIds: []string{"gel"}}},
			entities: []*pb.EntityMessage{lootEntity(2, 2, "gel")},
			want:     map[string]LootOutcome{"gel": LootOutcomeItem},
		},
		"an unbound item to consume": {
			defaults: []*pb.EntityTypeDefault{consumeLoot("orb")},
			entities: []*pb.EntityMessage{lootEntity(2, 2, "orb")},
			want:     map[string]LootOutcome{"orb": LootOutcomeConsume},
		},
		"a placed resource to consume": {
			defaults: []*pb.EntityTypeDefault{consumeLoot("crate")},
			entities: []*pb.EntityMessage{lootEntity(2, 2, "crate")},
			want:     map[string]LootOutcome{"crate": LootOutcomeConsume},
		},
		"a placed resource to grant": {
			entities: []*pb.EntityMessage{lootEntity(2, 2, "crate")},
			want:     map[string]LootOutcome{"crate": LootOutcomeItem},
		},
	}
	for name, tc := range cases {
		s := lootServer(t, tc.defaults...)
		buildWorld(s, []string{"m"}, tc.entities...)
		got := map[string]LootOutcome{}
		for _, token := range worldTokens(s.maps["m"]) {
			got[token.DropItemID] = token.LootOutcome
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
	first := woundedPlayer(ms, 101, ms.tf(token.ID).Pos, 50)
	second := woundedPlayer(ms, 102, ms.tf(token.ID).Pos, 50)

	for range 3 {
		s.phaseLoot(0, ms)
	}

	if _, ok := ms.bots[token.ID]; ok {
		t.Fatal("the consumed token stays on the map")
	}
	if healed := ms.vt(first.ID).Life + ms.vt(second.ID).Life - 100; healed != 10 {
		t.Fatalf("one gel restored %v life, want its resistance 10", healed)
	}
	if len(ms.ap(first.ID).Layers)+len(ms.ap(second.ID).Layers) != 0 {
		t.Fatal("a consumed pickup entered an inventory")
	}
}

func TestConsumingAPickupNeverPassesMaxLife(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	ms, token := gelMap(t, s)
	player := woundedPlayer(ms, 103, ms.tf(token.ID).Pos, 95)

	s.phaseLoot(0, ms)

	if ms.vt(player.ID).Life != 100 {
		t.Fatalf("life = %v, want max life 100", ms.vt(player.ID).Life)
	}
}

func TestAPickupGoesToTheFirstEligibleCollector(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	ms, token := gelMap(t, s)

	ghost := woundedPlayer(ms, 105, ms.tf(token.ID).Pos, 10)
	ms.vt(ghost.ID).RespawnAt = time.Now().Add(time.Minute)
	frozen := woundedPlayer(ms, 106, ms.tf(token.ID).Pos, 10)
	frozen.Frozen = true
	for i, behavior := range []string{BehaviorSkill, BehaviorCoin, BehaviorDrop, BehaviorProvider} {
		bot := &BotState{EntityBase: EntityBase{ID: EntityID(200 + i)}, Behavior: behavior}
		ms.bots[bot.ID] = bot
		ms.spawn(bot.ID, EntityData{Kind: EntityBot}, *ms.tf(token.ID), Appearance{})
		ms.addVitals(bot.ID, Vitals{Life: 10, MaxLife: 100})
	}

	s.phaseLoot(0, ms)

	if _, ok := ms.bots[token.ID]; !ok {
		t.Fatal("an ineligible entity took the token")
	}
	if ms.vt(ghost.ID).Life != 10 || ms.vt(frozen.ID).Life != 10 {
		t.Fatalf("ineligible players healed: ghost=%v frozen=%v", ms.vt(ghost.ID).Life, ms.vt(frozen.ID).Life)
	}

	wounded := woundedPlayer(ms, 108, ms.tf(token.ID).Pos, 60)
	s.phaseLoot(0, ms)
	if _, ok := ms.bots[token.ID]; ok || ms.vt(wounded.ID).Life != 70 {
		t.Fatalf("a wounded player must consume the token: life=%v", ms.vt(wounded.ID).Life)
	}
}

func TestACollectorAtFullLifeKeepsAPickupAndTheTokenComesBack(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	ms, token := gelMap(t, s)
	at := *ms.tf(token.ID)
	player := woundedPlayer(ms, 104, at.Pos, 100)
	bot := &BotState{EntityBase: EntityBase{ID: 109},
		Behavior: BehaviorHostile, MapCode: "m"}
	ms.bots[bot.ID] = bot
	ms.spawn(bot.ID, EntityData{Kind: EntityBot}, Transform{Pos: Vec2{X: 15, Y: 15}, Size: at.Size}, Appearance{})
	ms.addVitals(bot.ID, Vitals{Life: 100, MaxLife: 100})

	s.phaseLoot(0, ms)
	if _, ok := ms.bots[token.ID]; ok || ms.vt(player.ID).Life != 100 || s.playerItemQuantity(player, "gel") != 1 {
		t.Fatalf("a full-life player must keep the gel: life=%v gel=%d", ms.vt(player.ID).Life, s.playerItemQuantity(player, "gel"))
	}
	if len(ms.lootRespawns) != 1 {
		t.Fatalf("respawns = %+v, want the kept token back like a consumed one", ms.lootRespawns)
	}

	ms.lootRespawns[0].at = time.Now()
	ms.tf(player.ID).Pos = Vec2{X: -10, Y: -10}
	ms.tf(bot.ID).Pos = at.Pos
	s.phaseLoot(0, ms)
	if layerQuantity(ms.ap(bot.ID).Layers, "gel") != 1 || ms.vt(bot.ID).Life != 100 {
		t.Fatalf("a full-life bot must keep the gel: life=%v layers=%+v", ms.vt(bot.ID).Life, ms.ap(bot.ID).Layers)
	}
}

func TestAPickupHealsByItsResistanceAndComesBackAfterItsRespawnTime(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	placed := lootEntity(4, 4, "gel")
	placed.RespawnDurationMs = 5000
	buildWorld(s, []string{"m"}, placed)
	ms := s.maps["m"]
	token := onlyToken(t, ms)
	cell := ms.tf(token.ID).Pos
	player := woundedPlayer(ms, 103, cell, 50)

	now := time.Now()
	s.phaseLoot(0, ms)
	if ms.vt(player.ID).Life != 60 || len(worldTokens(ms)) != 0 {
		t.Fatalf("life=%v tokens=%d, want a heal of 10 that consumes the token", ms.vt(player.ID).Life, len(worldTokens(ms)))
	}
	if len(ms.lootRespawns) != 1 || ms.lootRespawns[0].at.Sub(now) < 5*time.Second {
		t.Fatalf("respawns = %+v, want the gel back after 5 s", ms.lootRespawns)
	}

	s.phaseLoot(0, ms)
	if len(worldTokens(ms)) != 0 {
		t.Fatal("the token came back before its respawn time")
	}
	ms.lootRespawns[0].at = now
	ms.tf(player.ID).Pos = Vec2{X: -10, Y: -10}
	s.phaseLoot(0, ms)
	back := onlyToken(t, ms)
	if ms.tf(back.ID).Pos != cell || back.LootOutcome != LootOutcomeConsume || back.RespawnDuration != 5*time.Second ||
		len(ms.lootRespawns) != 0 {
		t.Fatalf("the token must come back on its cell with its respawn time: %+v", back)
	}
}

func TestAPickupWithoutARespawnTimeTakesTheDefault(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	buildWorld(s, []string{"m"}, lootEntity(4, 4, "gel"), lootEntity(10, 10, "salve"))
	ms := s.maps["m"]
	for _, token := range worldTokens(ms) {
		woundedPlayer(ms, 1000+token.ID, ms.tf(token.ID).Pos, 50)
	}

	now := time.Now()
	s.phaseLoot(0, ms)

	// The granted salve never comes back; the consumed gel does, after the default respawn time.
	if len(ms.lootRespawns) != 1 || ms.lootRespawns[0].token.DropItemID != "gel" ||
		ms.lootRespawns[0].at.Sub(now) < respawnDurationDefaultMs*time.Millisecond {
		t.Fatalf("respawns = %+v, want only the gel after the default respawn time", ms.lootRespawns)
	}
}

func TestABotConsumesAPickup(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	ms, token := gelMap(t, s)
	bot := &BotState{EntityBase: EntityBase{ID: 300},
		Behavior: BehaviorHostile, MapCode: "m"}
	ms.bots[bot.ID] = bot
	ms.spawn(bot.ID, EntityData{Kind: EntityBot}, *ms.tf(token.ID), Appearance{})
	ms.addVitals(bot.ID, Vitals{Life: 30, MaxLife: 100})

	s.phaseLoot(0, ms)

	if _, ok := ms.bots[token.ID]; ok || ms.vt(bot.ID).Life != 40 {
		t.Fatalf("the bot must consume the token: life=%v", ms.vt(bot.ID).Life)
	}
}

func TestAGrantPickupEntersTheInventoryUnused(t *testing.T) {
	s := lootServer(t)
	buildWorld(s, []string{"m"}, lootEntity(4, 4, "salve"))
	ms := s.maps["m"]
	player := woundedPlayer(ms, 103, ms.tf(onlyToken(t, ms).ID).Pos, 100)

	s.phaseLoot(0, ms)

	if s.playerItemQuantity(player, "salve") != 1 || ms.vt(player.ID).Life != 100 {
		t.Fatalf("a grant pickup must reach the inventory unused: qty=%d life=%v",
			s.playerItemQuantity(player, "salve"), ms.vt(player.ID).Life)
	}
}

func TestDeathDropsKeepTheContributorRace(t *testing.T) {
	s, ms, contributor := movementFixture(t)
	s.lastEntityID = 1000 // drops take ids after the players
	s.coinItemID = "coin"
	outsider := woundedPlayer(ms, 107, Vec2{}, 50)
	ledger := map[EntityID]float64{contributor.ID: 5}
	s.spawnDrops(ms, "test", Vec2{X: 10, Y: 10},
		EntityTypeDefaultConfig{DropItemIDs: []string{"shard"}}, 7, ledger)

	tokens := drops(ms)
	if len(tokens) != 2 {
		t.Fatalf("got %d death drops, want an item and a coin token", len(tokens))
	}
	for i, token := range tokens {
		// Scattered tokens may overlap: set them apart so each one is collected alone.
		ms.tf(token.ID).Pos = Vec2{X: float64(3 + 10*i), Y: 3}
		if token.LootOrigin != LootOriginDeath || token.LootCollection != LootCollectContributors ||
			token.ExpiresAt.IsZero() || !token.CollectableAt.After(time.Now()) {
			t.Fatalf("death drop = %+v", token)
		}
		want := LootOutcomeItem
		if token.DropItemID == "coin" {
			want = LootOutcomeCoin
		}
		if token.LootOutcome != want {
			t.Fatalf("%s outcome = %v, want %v", token.DropItemID, token.LootOutcome, want)
		}
		token.CollectableAt = time.Time{}
	}

	for _, token := range tokens {
		ms.tf(contributor.ID).Pos = Vec2{X: 19, Y: 0}
		ms.tf(outsider.ID).Pos = ms.tf(token.ID).Pos
		s.phaseLoot(0, ms)
		if _, ok := ms.bots[token.ID]; !ok {
			t.Fatalf("a non-contributor took %s", token.DropItemID)
		}
		ms.tf(outsider.ID).Pos = Vec2{X: 19, Y: 19}
		ms.tf(contributor.ID).Pos = ms.tf(token.ID).Pos
		s.phaseLoot(0, ms)
		if _, ok := ms.bots[token.ID]; ok {
			t.Fatalf("the contributor could not take %s", token.DropItemID)
		}
	}
	if contributor.Coins != 7 || s.playerItemQuantity(contributor, "shard") != 1 {
		t.Fatalf("coins = %d shards = %d, want 7 and 1", contributor.Coins, s.playerItemQuantity(contributor, "shard"))
	}
}

func TestOnlyAWorldTokenAdmitsEveryViewer(t *testing.T) {
	world := &BotState{LootCollection: LootCollectFirstContact}
	death := &BotState{LootCollection: LootCollectContributors, LootContributors: map[EntityID]struct{}{1: {}}}
	if !lootAdmits(world, 99) || !lootAdmits(death, 1) || lootAdmits(death, 2) {
		t.Fatal("collection policy admits the wrong viewers")
	}
	consuming := &BotState{LootOutcome: LootOutcomeConsume}
	if lootConsumes(death, &Vitals{Life: 10, MaxLife: 100}) || !lootConsumes(consuming, &Vitals{Life: 99, MaxLife: 100}) ||
		lootConsumes(consuming, &Vitals{Life: 100, MaxLife: 100}) {
		t.Fatal("only a consuming token heals, and only a collector that misses life")
	}
}
