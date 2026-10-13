package game

import (
	"testing"

	pb "cyberia-server/gen/proto"
)

func TestForegroundCarriesOnlyTheBehaviorItsDefaultBinds(t *testing.T) {
	s := &Sim{Content: &Content{
		World:          &World{},
		entityDefaults: map[string]EntityTypeDefaultConfig{},
		entityDefaultBuilds: []EntityTypeDefaultConfig{
			{EntityType: "foreground", LiveItemIDs: []string{"canopy"}},
			{EntityType: "foreground", LiveItemIDs: []string{"roof"}, Behavior: "overhead-occlusion"},
		},
	}}
	ms := &MapState{}
	s.buildForeground(ms, &pb.EntityMessage{ObjectLayerItemIds: []string{"roof"}, DimX: 4, DimY: 3})
	s.buildForeground(ms, &pb.EntityMessage{ObjectLayerItemIds: []string{"canopy"}, DimX: 2, DimY: 2})

	want := map[string]bool{"roof": true, "canopy": false}
	for id := range ms.IDs() {
		row := ms.Row(id)
		item := row.Appearance.Layers[0].ItemID
		if got := row.Data.Flags&FlagOverheadOcclusion != 0; got != want[item] {
			t.Fatalf("foreground %q: want occlusion %v, got %v", item, want[item], got)
		}
	}
}
