package game

import (
	"testing"

	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
)

// End to end over the wire shape engine-cyberia actually sends: the instance config carries the
// resolved inventory, a map places the resource by its skin alone, and the snapshot has to carry
// the overridden item the entity is wearing.
func TestResourceSpawnedFromInstanceConfigCarriesTheOverriddenItem(t *testing.T) {
	server := NewGameServer()
	server.EntityBaseMaxLife = 100
	server.InitialLifeFraction = 1
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

	mapState := &world.MapState{}
	server.buildResource(mapState, &pb.EntityMessage{
		ObjectLayerItemIds: []string{"wood-1"},
		DimX:               2,
		DimY:               2,
	})
	if mapState.Count(world.EntityResource) != 1 {
		t.Fatalf("want one resource, got %d", mapState.Count(world.EntityResource))
	}

	var spawned world.EntityID
	for resource := range mapState.OfKind(world.EntityResource) {
		spawned = resource
	}
	// What the wire carries to the client is the active set, in stack order.
	sent := activeLayers(mapState.Ap(spawned).Layers)
	if len(sent) != 2 || sent[0].ItemID != "wood-1" || sent[1].ItemID != "tim-knife" {
		t.Fatalf("the client must receive the skin and the item worn over it, got %+v", sent)
	}
}
