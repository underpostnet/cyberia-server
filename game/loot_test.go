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

// withBuff states the buff factor of a default's live item.
func withBuff(def *pb.EntityTypeDefault, buff float64) *pb.EntityTypeDefault {
	def.DefaultObjectLayers = append(def.DefaultObjectLayers, &pb.DefaultObjectLayerState{
		ItemId: def.LiveItemIds[0], Active: true, Quantity: 1, BuffFactor: &buff,
	})
	return def
}

// lootServer is a server with the given defaults and an object-layer cache that
// binds the gel and salve consumables and the crate resource.
func lootServer(t *testing.T, defaults ...*pb.EntityTypeDefault) *GameServer {
	t.Helper()
	s := NewGameServer()
	if err := s.ApplyInstanceConfig(&pb.InstanceConfig{EntityBaseMaxLife: 100, EntityDefaults: defaults}); err != nil {
		t.Fatal(err)
	}
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{
		"gel":   {Data: ObjectLayerData{Item: Item{ID: "gel", Type: "consumable"}}},
		"salve": {Data: ObjectLayerData{Item: Item{ID: "salve", Type: "consumable"}}},
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

// woundedPlayer stands a player with the given life on the token.
func woundedPlayer(ms *MapState, id string, token *BotState, life float64) *PlayerState {
	player := &PlayerState{}
	player.ID = id
	player.Pos = token.Pos
	player.Dims = Dimensions{Width: 1, Height: 1}
	player.Life, player.MaxLife = life, 100
	ms.players[id] = player
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
			if token.Pos != (Point{X: 2, Y: 3}) && token.Pos != (Point{X: 8, Y: 8}) {
				t.Fatalf("a token left its placed cell: %+v", token.Pos)
			}
			if token.LootOutcome != LootOutcomeConsume || token.LootCollection != LootCollectFirstContact ||
				len(activeLayers(token.ObjectLayers)) != 1 {
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
	first := woundedPlayer(ms, "first", token, 50)
	second := woundedPlayer(ms, "second", token, 50)

	for range 3 {
		s.phaseLoot(0, ms)
	}

	if _, ok := ms.bots[token.ID]; ok {
		t.Fatal("the consumed token stays on the map")
	}
	if healed := first.Life + second.Life - 100; healed != 10 {
		t.Fatalf("one gel restored %v life, want the default buff of 10%% of max life", healed)
	}
	if len(first.ObjectLayers)+len(second.ObjectLayers) != 0 {
		t.Fatal("a consumed pickup entered an inventory")
	}
}

func TestConsumingAPickupNeverPassesMaxLife(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	ms, token := gelMap(t, s)
	player := woundedPlayer(ms, "p", token, 95)

	s.phaseLoot(0, ms)

	if player.Life != 100 {
		t.Fatalf("life = %v, want max life 100", player.Life)
	}
}

func TestAPickupWaitsForSomeoneItBenefits(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	ms, token := gelMap(t, s)

	full := woundedPlayer(ms, "full", token, 100)
	ghost := woundedPlayer(ms, "ghost", token, 10)
	ghost.RespawnTime = time.Now().Add(time.Minute)
	frozen := woundedPlayer(ms, "frozen", token, 10)
	frozen.Frozen = true
	for _, behavior := range []string{BehaviorSkill, BehaviorCoin, BehaviorDrop, BehaviorProvider} {
		bot := &BotState{EntityBase: EntityBase{ID: behavior, Pos: token.Pos, Dims: token.Dims},
			Mortal: Mortal{Life: 10, MaxLife: 100}, Behavior: behavior}
		ms.bots[bot.ID] = bot
	}

	s.phaseLoot(0, ms)

	if _, ok := ms.bots[token.ID]; !ok {
		t.Fatal("an ineligible entity consumed the token")
	}
	if full.Life != 100 || ghost.Life != 10 || frozen.Life != 10 {
		t.Fatalf("ineligible players healed: full=%v ghost=%v frozen=%v", full.Life, ghost.Life, frozen.Life)
	}

	full.Life = 60
	s.phaseLoot(0, ms)
	if _, ok := ms.bots[token.ID]; ok || full.Life != 70 {
		t.Fatalf("a wounded player must consume the token: life=%v", full.Life)
	}
}

func TestAPickupHealsByItsBuffFactorAndComesBackAfterItsRespawnTime(t *testing.T) {
	s := lootServer(t, withBuff(consumeLoot("gel"), 0.25))
	placed := lootEntity(4, 4, "gel")
	placed.RespawnDurationMs = 5000
	buildWorld(s, []string{"m"}, placed)
	ms := s.maps["m"]
	token := onlyToken(t, ms)
	player := woundedPlayer(ms, "p", token, 50)

	now := time.Now()
	s.phaseLoot(0, ms)
	if player.Life != 75 || len(worldTokens(ms)) != 0 {
		t.Fatalf("life=%v tokens=%d, want a 25%% heal that consumes the token", player.Life, len(worldTokens(ms)))
	}
	if len(ms.lootRespawns) != 1 || ms.lootRespawns[0].at.Sub(now) < 5*time.Second {
		t.Fatalf("respawns = %+v, want the gel back after 5 s", ms.lootRespawns)
	}

	s.phaseLoot(0, ms)
	if len(worldTokens(ms)) != 0 {
		t.Fatal("the token came back before its respawn time")
	}
	ms.lootRespawns[0].at = now
	player.Pos = Point{X: -10, Y: -10}
	s.phaseLoot(0, ms)
	back := onlyToken(t, ms)
	if back.Pos != token.Pos || back.LootOutcome != LootOutcomeConsume || back.RespawnDuration != 5*time.Second ||
		len(ms.lootRespawns) != 0 {
		t.Fatalf("the token must come back on its cell with its respawn time: %+v", back)
	}
}

func TestAPickupWithoutARespawnTimeTakesTheDefault(t *testing.T) {
	s := lootServer(t, consumeLoot("gel"))
	buildWorld(s, []string{"m"}, lootEntity(4, 4, "gel"), lootEntity(10, 10, "salve"))
	ms := s.maps["m"]
	for _, token := range worldTokens(ms) {
		woundedPlayer(ms, token.DropItemID, token, 50)
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
	bot := &BotState{EntityBase: EntityBase{ID: "raider", Pos: token.Pos, Dims: token.Dims},
		Mortal: Mortal{Life: 30, MaxLife: 100}, Behavior: BehaviorHostile}
	ms.bots[bot.ID] = bot

	s.phaseLoot(0, ms)

	if _, ok := ms.bots[token.ID]; ok || bot.Life != 40 {
		t.Fatalf("the bot must consume the token: life=%v", bot.Life)
	}
}

func TestAGrantPickupEntersTheInventoryUnused(t *testing.T) {
	s := lootServer(t)
	buildWorld(s, []string{"m"}, lootEntity(4, 4, "salve"))
	ms := s.maps["m"]
	player := woundedPlayer(ms, "p", onlyToken(t, ms), 100)

	s.phaseLoot(0, ms)

	if s.playerItemQuantity(player, "salve") != 1 || player.Life != 100 {
		t.Fatalf("a grant pickup must reach the inventory unused: qty=%d life=%v",
			s.playerItemQuantity(player, "salve"), player.Life)
	}
}

func TestDeathDropsKeepTheContributorRace(t *testing.T) {
	s, ms, contributor := movementFixture(t)
	s.coinItemID = "coin"
	outsider := woundedPlayer(ms, "outsider", &BotState{}, 50)
	ledger := map[string]float64{contributor.ID: 5}
	s.spawnDrops(ms, "test", Point{X: 10, Y: 10},
		EntityTypeDefaultConfig{DropItemIDs: []string{"shard"}}, 7, ledger)

	tokens := drops(ms)
	if len(tokens) != 2 {
		t.Fatalf("got %d death drops, want an item and a coin token", len(tokens))
	}
	for i, token := range tokens {
		// Scattered tokens may overlap: set them apart so each one is collected alone.
		token.Pos = Point{X: float64(3 + 10*i), Y: 3}
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
		contributor.Pos = Point{X: 19, Y: 0}
		outsider.Pos = token.Pos
		s.phaseLoot(0, ms)
		if _, ok := ms.bots[token.ID]; !ok {
			t.Fatalf("a non-contributor took %s", token.DropItemID)
		}
		outsider.Pos = Point{X: 19, Y: 19}
		contributor.Pos = token.Pos
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
	death := &BotState{LootCollection: LootCollectContributors, LootContributors: map[string]struct{}{"a": {}}}
	if !lootAdmits(world, "anyone") || !lootAdmits(death, "a") || lootAdmits(death, "b") {
		t.Fatal("collection policy admits the wrong viewers")
	}
	if !lootBenefits(death, &Mortal{Life: 100, MaxLife: 100}) {
		t.Fatal("an item or a coin benefits a collector at full life")
	}
}
