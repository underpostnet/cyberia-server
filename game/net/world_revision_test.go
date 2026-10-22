package net

import (
	"testing"

	pb "cyberia-server/gen/proto"
)

// A client joins with the world revision and hears each rebuild once, so its
// map views read the content again.
func TestAClientHearsEachWorldRebuildOnce(t *testing.T) {
	srv := newAccountServer(t, &fakeEngine{guest: true})
	conn := connect(t, srv, testTicket)
	var joined struct {
		WorldRevision *uint64 `json:"worldRevision"`
	}
	readPayload(t, conn, "metadata", &joined)
	if joined.WorldRevision == nil || *joined.WorldRevision != 0 {
		t.Fatalf("the join names the world revision: %v", joined.WorldRevision)
	}
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })

	srv.game.RebuildWorld(&pb.InstanceMessage{Code: "test", MapCodes: []string{"test"}},
		[]*pb.MapDataMessage{{Code: "test", GridX: 40, GridY: 40}}, nil, nil, nil, nil)
	for tick := 0; tick < 2; tick++ {
		srv.game.Mu.Lock()
		srv.game.phaseReplication(srv.game.Maps["test"])
		srv.game.Mu.Unlock()
	}
	var rebuilt struct {
		WorldRevision uint64 `json:"worldRevision"`
	}
	readPayload(t, conn, "metadata", &rebuilt)
	if rebuilt.WorldRevision != 1 {
		t.Fatalf("revision after the rebuild = %d, want 1", rebuilt.WorldRevision)
	}
	srv.game.Mu.Lock()
	defer srv.game.Mu.Unlock()
	for _, client := range srv.game.clients {
		if client.worldSeen != 1 {
			t.Fatalf("the client holds revision %d after one delta", client.worldSeen)
		}
	}
}
