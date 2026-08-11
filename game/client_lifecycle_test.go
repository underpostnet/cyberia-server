package game

import (
	"testing"
)

func lifecycleServer() (*GameServer, *MapState, *MapState) {
	home := &MapState{players: map[string]*PlayerState{}, gridW: 20, gridH: 20}
	away := &MapState{players: map[string]*PlayerState{}, gridW: 20, gridH: 20}
	s := &GameServer{
		maps:       map[string]*MapState{"home": home, "away": away},
		clients:    map[string]*Client{},
		register:   make(chan *Client, 4),
		unregister: make(chan *Client, 4),
	}
	s.SetConnectionLimits(DefaultConnectionLimits())
	return s, home, away
}

func joinedClient(s *GameServer, mapState *MapState, id, mapCode, ip string) *Client {
	player := &PlayerState{EntityBase: EntityBase{ID: id}, MapCode: mapCode}
	client := &Client{playerID: id, playerState: player, ip: ip, limiter: newInputLimiter(s.limits)}
	player.Client = client
	mapState.players[id] = player
	s.clients[id] = client
	s.guard.admit(ip)
	return client
}

// detachClient must clear world presence, the client registry, and the guard
// slot. Any one of these left behind keeps a dead player in the simulation.
func TestDetachClientClearsEveryReference(t *testing.T) {
	s, home, _ := lifecycleServer()
	client := joinedClient(s, home, "p1", "home", "10.0.0.1")

	if total, _ := s.guard.counts(); total != 1 {
		t.Fatalf("setup: guard should hold 1 connection, got %d", total)
	}

	s.detachClient(client)

	if _, still := home.players["p1"]; still {
		t.Fatal("the player must leave the map")
	}
	if _, still := s.clients["p1"]; still {
		t.Fatal("the client must leave the registry")
	}
	if total, addresses := s.guard.counts(); total != 0 || addresses != 0 {
		t.Fatalf("the guard slot must be freed, got (%d conns, %d addresses)", total, addresses)
	}
}

// A portal moves a player between maps. Teardown must find it wherever it is,
// not only on the map its MapCode last named.
func TestDetachClientFindsPlayerAfterAMapChange(t *testing.T) {
	s, home, away := lifecycleServer()
	client := joinedClient(s, home, "p1", "home", "10.0.0.1")

	// Simulate a portal: the player moved but MapCode is stale.
	delete(home.players, "p1")
	away.players["p1"] = client.playerState

	s.detachClient(client)

	if _, still := away.players["p1"]; still {
		t.Fatal("the player must be removed from the map it actually occupies")
	}
}

// Every disconnect path can call detachClient. The guard slot must be freed
// exactly once, or repeated disconnects would corrupt the count.
func TestDetachClientReleasesTheGuardSlotOnlyOnce(t *testing.T) {
	s, home, _ := lifecycleServer()
	other := joinedClient(s, home, "p2", "home", "10.0.0.1")
	client := joinedClient(s, home, "p1", "home", "10.0.0.1")

	if total, _ := s.guard.counts(); total != 2 {
		t.Fatalf("setup: want 2 connections, got %d", total)
	}

	for i := 0; i < 5; i++ {
		s.detachClient(client)
	}

	total, _ := s.guard.counts()
	if total != 1 {
		t.Fatalf("repeated teardown must free one slot, got %d connections left", total)
	}
	s.detachClient(other)
	if total, _ := s.guard.counts(); total != 0 {
		t.Fatalf("want 0 connections after both detach, got %d", total)
	}
}

// The register handshake can time out. The connect path already put the player
// in the world, so it must take it back out.
func TestRegisterTimeoutLeavesNoPlayerBehind(t *testing.T) {
	s, home, _ := lifecycleServer()
	client := joinedClient(s, home, "p1", "home", "10.0.0.1")

	// This is what HandleConnections runs when the register send times out.
	s.detachClient(client)

	if len(home.players) != 0 {
		t.Fatalf("no player may survive a failed register, found %d", len(home.players))
	}
	if total, _ := s.guard.counts(); total != 0 {
		t.Fatalf("no guard slot may survive a failed register, found %d", total)
	}
}

// A full unregister queue must not pin the read goroutine. readPump falls back
// to a direct teardown, which must be equivalent.
func TestUnregisterFallbackTearsDownWhenQueueIsFull(t *testing.T) {
	s, home, _ := lifecycleServer()
	client := joinedClient(s, home, "p1", "home", "10.0.0.1")

	for i := 0; i < cap(s.unregister); i++ {
		s.unregister <- &Client{playerID: "filler"}
	}

	// The select in readPump's defer, with a full queue.
	select {
	case s.unregister <- client:
		t.Fatal("the queue is full: the send must not succeed")
	default:
		s.detachClient(client)
	}

	if _, still := home.players["p1"]; still {
		t.Fatal("the fallback path must remove the player")
	}
	if total, _ := s.guard.counts(); total != 0 {
		t.Fatalf("the fallback path must free the guard slot, got %d", total)
	}
}
