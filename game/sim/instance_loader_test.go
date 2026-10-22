package sim

import (
	"testing"

	pb "cyberia-server/gen/proto"
)

// The first build leaves the world revision at zero; each rebuild moves it and
// reads the entry spawn again, so a client map of the older world is stale.
func TestARebuildMovesTheWorldRevision(t *testing.T) {
	s := lootServer(t)
	buildWorld(s, []string{"m1"})
	if s.WorldRevision() != 0 {
		t.Fatalf("revision after the first build = %d, want 0", s.WorldRevision())
	}
	spawn := &pb.PlayerSpawn{SourceMapCode: "m1", SourceCellX: 2, SourceCellY: 3}
	for want := uint64(1); want <= 2; want++ {
		s.RebuildWorld(&pb.InstanceMessage{Code: "loot-test", MapCodes: []string{"m1"}, PlayerSpawn: spawn},
			[]*pb.MapDataMessage{{Code: "m1", GridX: 20, GridY: 20}}, nil, nil, nil, nil)
		if s.WorldRevision() != want {
			t.Fatalf("revision after rebuild %d = %d", want, s.WorldRevision())
		}
	}
	if s.PlayerSpawn != (PlayerSpawnConfig{MapCode: "m1", CellX: 2, CellY: 3}) {
		t.Fatalf("the rebuild reads the entry spawn: %+v", s.PlayerSpawn)
	}
}
