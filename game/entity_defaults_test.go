package game

import (
	"testing"

	pb "cyberia-server/gen/proto"
)

// The reported document: a wood resource whose default carries a knife in its inventory, marked
// active by an override. Every entity type reads its stack the same way, so the case is pinned
// once here and the spawn paths all go through it.
func resourceServer() *GameServer {
	return &GameServer{
		entityDefaults: map[string]EntityTypeDefaultConfig{},
		entityDefaultBuilds: []EntityTypeDefaultConfig{{
			EntityType:  "resource",
			LiveItemIDs: []string{"wood-1"},
			DeadItemIDs: []string{"wood-extracted-1"},
			DropItemIDs: []string{"wood-drop-1"},
			DefaultObjectLayers: []ObjectLayerState{
				{ItemID: "wood-1", Active: true, Quantity: 1},
				{ItemID: "wood-extracted-1", Active: false, Quantity: 0},
				{ItemID: "wood-drop-1", Active: false, Quantity: 2},
				{ItemID: "tim-knife", Active: true, Quantity: 1},
			},
		}},
	}
}

func layerOf(layers []ObjectLayerState, itemID string) (ObjectLayerState, int) {
	for index, layer := range layers {
		if layer.ItemID == itemID {
			return layer, index
		}
	}
	return ObjectLayerState{}, -1
}

func TestSpawnObjectLayersSeedsTheWholeInventoryOverThePlacedSkin(t *testing.T) {
	layers := resourceServer().spawnObjectLayers("resource", []string{"wood-1"}, true)

	skin, skinIndex := layerOf(layers, "wood-1")
	if skinIndex != 0 || !skin.Active {
		t.Fatalf("the placed skin leads the stack and is active: %+v", layers)
	}
	// The override is what puts the knife on a live entity, and it is drawn over the skin.
	knife, knifeIndex := layerOf(layers, "tim-knife")
	if knifeIndex <= skinIndex || !knife.Active || knife.Quantity != 1 {
		t.Fatalf("an overridden carried item must spawn active, above the skin: %+v", layers)
	}
	// Lifecycle slots are seeded but inactive, so a death activates a slot instead of appending one.
	dead, deadIndex := layerOf(layers, "wood-extracted-1")
	if deadIndex == -1 || dead.Active {
		t.Fatalf("the dead id is seeded inactive: %+v", layers)
	}
	// The override sizes the drop bundle, and that size survives the spawn.
	drop, dropIndex := layerOf(layers, "wood-drop-1")
	if dropIndex == -1 || drop.Active || drop.Quantity != 2 {
		t.Fatalf("the drop id keeps its override quantity: %+v", layers)
	}
}

func TestSpawnObjectLayersNeverDuplicatesAPlacedID(t *testing.T) {
	layers := resourceServer().spawnObjectLayers("resource", []string{"wood-1", "wood-1", ""}, true)
	if len(layers) != 4 {
		t.Fatalf("a repeated or empty placed id adds no slot: %+v", layers)
	}
}

// A solid-colour entity is placed without sprites on purpose, so it carries no default layers.
func TestSpawnObjectLayersLeavesAColouredEntityBare(t *testing.T) {
	if layers := resourceServer().spawnObjectLayers("resource", nil, false); len(layers) != 0 {
		t.Fatalf("want no layers, got %+v", layers)
	}
	layers := resourceServer().spawnObjectLayers("resource", []string{"wood-1"}, false)
	if len(layers) != 1 || layers[0].ItemID != "wood-1" {
		t.Fatalf("only what the author placed: %+v", layers)
	}
}

// A build seeded before engine-cyberia derived its inventory still names its live ids.
func TestSpawnObjectLayersFallsBackToLiveItemIDs(t *testing.T) {
	server := &GameServer{
		entityDefaults: map[string]EntityTypeDefaultConfig{
			"bot": {EntityType: "bot", LiveItemIDs: []string{"purple"}},
		},
	}
	layers := server.spawnObjectLayers("bot", nil, true)
	if len(layers) != 1 || layers[0].ItemID != "purple" || !layers[0].Active {
		t.Fatalf("want the live default, got %+v", layers)
	}
}

// End to end over the wire shape engine-cyberia actually sends: the instance config carries the
// resolved inventory, a map places the resource by its skin alone, and the snapshot has to carry
// the overridden item the entity is wearing.
func TestResourceSpawnedFromInstanceConfigCarriesTheOverriddenItem(t *testing.T) {
	server := &GameServer{
		entityDefaults:      map[string]EntityTypeDefaultConfig{},
		entityBaseMaxLife:   100,
		initialLifeFraction: 1,
		maps:                map[string]*MapState{},
	}
	server.ApplyInstanceConfig(&pb.InstanceConfig{
		EntityDefaults: []*pb.EntityTypeDefault{{
			EntityType:  "resource",
			LiveItemIds: []string{"wood-1"},
			DeadItemIds: []string{"wood-extracted-1"},
			DropItemIds: []string{"wood-drop-1"},
			DefaultObjectLayers: []*pb.DefaultObjectLayerState{
				{ItemId: "wood-1", Active: true, Quantity: 1},
				{ItemId: "wood-extracted-1", Active: false, Quantity: 0},
				{ItemId: "wood-drop-1", Active: false, Quantity: 2},
				{ItemId: "tim-knife", Active: true, Quantity: 1},
			},
		}},
	})

	mapState := &MapState{resources: map[string]*ResourceState{}}
	server.buildResource(mapState, "fallback-map-0", &pb.EntityMessage{
		ObjectLayerItemIds: []string{"wood-1"},
		DimX:               2,
		DimY:               2,
	})
	if len(mapState.resources) != 1 {
		t.Fatalf("want one resource, got %d", len(mapState.resources))
	}

	var spawned *ResourceState
	for _, resource := range mapState.resources {
		spawned = resource
	}
	// What the wire carries to the client is the active set, in stack order.
	sent := activeLayers(spawned.ObjectLayers)
	if len(sent) != 2 || sent[0].ItemID != "wood-1" || sent[1].ItemID != "tim-knife" {
		t.Fatalf("the client must receive the skin and the item worn over it, got %+v", sent)
	}
}

// The equipment rules hold at spawn, not only on an activation request: one active item per
// governed type, and anything the rules do not govern passes through.
func TestSpawnObjectLayersHoldsTheEquipmentRules(t *testing.T) {
	server := resourceServer()
	server.equipmentRules = EquipmentRulesConfig{
		ActiveItemTypes: map[string]bool{"skin": true, "breastplate": true, "weapon": true},
		OnePerType:      true,
	}
	server.objectLayerDataCache = map[string]*ObjectLayer{
		"wood-1":    {Data: ObjectLayerData{Item: Item{Type: "skin"}}},
		"tim-knife": {Data: ObjectLayerData{Item: Item{Type: "weapon"}}},
	}

	// A skin and a weapon are different slots: the rules allow the combination, so both render.
	layers := server.spawnObjectLayers("resource", []string{"wood-1"}, true)
	skin, _ := layerOf(layers, "wood-1")
	knife, _ := layerOf(layers, "tim-knife")
	if !skin.Active || !knife.Active {
		t.Fatalf("a skin and a weapon may both be worn: %+v", layers)
	}

	// A second skin contests a claimed slot, so it spawns inactive.
	server.objectLayerDataCache["tim-knife"] = &ObjectLayer{Data: ObjectLayerData{Item: Item{Type: "skin"}}}
	layers = server.spawnObjectLayers("resource", []string{"wood-1"}, true)
	skin, _ = layerOf(layers, "wood-1")
	knife, _ = layerOf(layers, "tim-knife")
	if !skin.Active || knife.Active {
		t.Fatalf("one skin at a time, the placed one first: %+v", layers)
	}
}

// Taking an item off is not taking it away: a skin the equipment rules unseat stays carried with
// its stack, which is what lets the player bank it or put it back on.
func TestNormalizeLoadoutKeepsTheStackOfWhatItUnseats(t *testing.T) {
	server := &GameServer{
		equipmentRules: EquipmentRulesConfig{
			ActiveItemTypes: map[string]bool{"skin": true},
			OnePerType:      true,
		},
		objectLayerDataCache: map[string]*ObjectLayer{
			"anon":   {Data: ObjectLayerData{Item: Item{Type: "skin"}}},
			"kaneki": {Data: ObjectLayerData{Item: Item{Type: "skin"}}},
		},
	}
	out := server.normalizeLoadout([]ObjectLayerState{
		{ItemID: "anon", Active: true, Quantity: 1},
		{ItemID: "kaneki", Active: true, Quantity: 1},
	})
	unseated, _ := layerOf(out, "kaneki")
	if unseated.Active || unseated.Quantity != 1 {
		t.Fatalf("the unseated skin keeps its stack: %+v", out)
	}
}

// Placement decides what an entity has; the build decides what it wears. An override that names a
// second weapon reaches the entity with the same answer engine-cyberia wrote into the map, so the
// two layers of the platform agree on which one is on.
func TestSpawnObjectLayersTakesTheBuildStateForAPlacedID(t *testing.T) {
	server := &GameServer{
		entityDefaults: map[string]EntityTypeDefaultConfig{},
		entityDefaultBuilds: []EntityTypeDefaultConfig{{
			EntityType:  "bot",
			LiveItemIDs: []string{"purple", "atlas_pistol_mk2"},
			DefaultObjectLayers: []ObjectLayerState{
				{ItemID: "purple", Active: true, Quantity: 1},
				// The override wears the hatchet, so the pistol the map places is carried, not worn.
				{ItemID: "atlas_pistol_mk2", Active: false, Quantity: 1},
				{ItemID: "hatchet", Active: true, Quantity: 1},
			},
		}},
	}
	layers := server.spawnObjectLayers("bot", []string{"purple", "atlas_pistol_mk2", "hatchet"}, true)
	pistol, _ := layerOf(layers, "atlas_pistol_mk2")
	hatchet, _ := layerOf(layers, "hatchet")
	if pistol.Active || pistol.Quantity != 1 {
		t.Fatalf("a placed id the build does not wear is carried, not worn: %+v", layers)
	}
	if !hatchet.Active {
		t.Fatalf("the weapon the override names is the one worn: %+v", layers)
	}
}

// A build that says nothing about an id must keep dropping it: worlds configured before drop
// chances existed carry no map at all, and reading that as "never" would empty every loot table.
func TestDropChanceDefaultsToAlways(t *testing.T) {
	build := EntityTypeDefaultConfig{EntityType: "resource", DropItemIDs: []string{"wood-drop-1"}}
	if got := dropChance(build, "wood-drop-1"); got != 1 {
		t.Fatalf("unstated id: want 1, got %v", got)
	}
	if got := dropChance(build, "absent"); got != 1 {
		t.Fatalf("unknown id: want 1, got %v", got)
	}
}

func TestDropChanceHonoursAndClampsWhatTheBuildStates(t *testing.T) {
	build := EntityTypeDefaultConfig{
		EntityType:  "resource",
		DropItemIDs: []string{"common", "rare", "never", "broken-low", "broken-high"},
		DropChances: map[string]float64{
			"rare":        0.25,
			"never":       0,
			"broken-low":  -3,
			"broken-high": 7.5,
		},
	}
	for _, tc := range []struct {
		itemID string
		want   float64
	}{
		{"common", 1},      // stated by no row
		{"rare", 0.25},     // honoured verbatim
		{"never", 0},       // a deliberate zero survives, which is why the wire field is optional
		{"broken-low", 0},  // clamped, not rejected
		{"broken-high", 1}, // clamped toward the old always-drops behaviour
	} {
		if got := dropChance(build, tc.itemID); got != tc.want {
			t.Errorf("%s: want %v, got %v", tc.itemID, tc.want, got)
		}
	}
}
