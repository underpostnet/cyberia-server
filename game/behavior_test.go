package game

import (
	"testing"

	pb "cyberia-server/gen/proto"
)

func TestForegroundCarriesOnlyTheBehaviorItsDefaultBinds(t *testing.T) {
	s := &GameServer{
		entityDefaults: map[string]EntityTypeDefaultConfig{},
		entityDefaultBuilds: []EntityTypeDefaultConfig{
			{EntityType: "foreground", LiveItemIDs: []string{"canopy"}},
			{EntityType: "foreground", LiveItemIDs: []string{"roof"}, Behavior: "overhead-occlusion"},
		},
	}
	ms := &MapState{foregrounds: map[string]ForegroundState{}}
	s.buildForeground(ms, &pb.EntityMessage{ObjectLayerItemIds: []string{"roof"}, DimX: 4, DimY: 3})
	s.buildForeground(ms, &pb.EntityMessage{ObjectLayerItemIds: []string{"canopy"}, DimX: 2, DimY: 2})

	want := map[string]string{"roof": "overhead-occlusion", "canopy": ""}
	for _, fg := range ms.foregrounds {
		item := fg.ObjectLayers[0].ItemID
		if fg.Behavior != want[item] {
			t.Fatalf("foreground %q: want behavior %q, got %q", item, want[item], fg.Behavior)
		}
	}
}
