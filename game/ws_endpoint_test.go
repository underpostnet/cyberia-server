package game

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"testing"
	"time"

	"cyberia-server/serial"

	"github.com/gorilla/websocket"
)

// wsTestServer builds a minimal but real GameServer: one walkable map, a client
// listener, and an HTTP server exposing the true /ws handler. It exercises the
// same path a live client takes.
type wsTestServer struct {
	game *GameServer
	http *httptest.Server
	url  string
}

func newWSTestServer(t *testing.T, limits ConnectionLimits) *wsTestServer {
	t.Helper()

	mapState := &MapState{
		gridW:       40,
		gridH:       40,
		pathfinder:  NewPathfinder(40, 40),
		players:     map[string]*PlayerState{},
		bots:        map[string]*BotState{},
		obstacles:   map[string]ObjectState{},
		foregrounds: map[string]ObjectState{},
		portals:     map[string]*PortalState{},
		floors:      map[string]*FloorState{},
		resources:   map[string]*ResourceState{},
		statics:     map[string]*StaticState{},
	}

	s := NewGameServer()
	s.SetConnectionLimits(limits)
	s.maps["test"] = mapState
	s.tickRate, s.snapshotRate = 60, 20
	s.tickDuration = computeTickDuration(60)
	s.aoiRadius = 10
	s.defaultPlayerWidth, s.defaultPlayerHeight = 2, 2
	s.entityBaseMaxLife, s.initialLifeFraction = 100, 1
	s.entityBaseSpeed = 5
	s.entityBaseActionCooldown = 100 * time.Millisecond
	s.entityBaseMinActionCooldown = 50 * time.Millisecond
	s.sumStatsLimit = 100
	s.statsCache = map[string]statsCacheEntry{}
	s.storage = map[storageKey][]StorageSlot{}
	s.entityDefaults = map[string]EntityTypeDefaultConfig{}
	s.playerSpawn = PlayerSpawnConfig{Random: true}

	go s.listenForClients()

	httpSrv := httptest.NewServer(http.HandlerFunc(s.HandleConnections))
	t.Cleanup(httpSrv.Close)

	return &wsTestServer{game: s, http: httpSrv, url: "ws" + strings.TrimPrefix(httpSrv.URL, "http")}
}

// playerCount reports how many players the world holds across every map.
func (w *wsTestServer) playerCount() int {
	w.game.mu.Lock()
	defer w.game.mu.Unlock()
	n := 0
	for _, m := range w.game.maps {
		n += len(m.players)
	}
	return n
}

func (w *wsTestServer) clientCount() int {
	w.game.mu.Lock()
	defer w.game.mu.Unlock()
	return len(w.game.clients)
}

// waitFor polls until cond holds or the deadline passes.
func waitFor(t *testing.T, what string, cond func() bool) {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		if cond() {
			return
		}
		time.Sleep(5 * time.Millisecond)
	}
	t.Fatalf("timed out waiting for %s", what)
}

func dial(t *testing.T, url string) (*websocket.Conn, *http.Response, error) {
	t.Helper()
	return websocket.DefaultDialer.Dial(url, nil)
}

// readInitData consumes frames until init_data arrives and returns its payload.
func readInitData(t *testing.T, conn *websocket.Conn) InitPayload {
	t.Helper()
	conn.SetReadDeadline(time.Now().Add(5 * time.Second))
	for i := 0; i < 20; i++ {
		_, raw, err := conn.ReadMessage()
		if err != nil {
			t.Fatalf("read init_data: %v", err)
		}
		msg, err := serial.Unpack(raw)
		if err != nil {
			continue
		}
		if msg.Type == "init_data" {
			var payload InitPayload
			if err := json.Unmarshal(msg.Payload, &payload); err != nil {
				t.Fatalf("decode init_data: %v", err)
			}
			return payload
		}
	}
	t.Fatal("init_data never arrived")
	return InitPayload{}
}

func sendFrame(t *testing.T, conn *websocket.Conn, msgType string, payload any) error {
	t.Helper()
	pack, err := serial.Pack(msgType, payload)
	if err != nil {
		t.Fatalf("pack %s: %v", msgType, err)
	}
	return conn.WriteMessage(websocket.BinaryMessage, pack)
}

// The ordinary case: a client connects, gets init_data with the real grid, and
// the world holds exactly one player. Closing frees every reference.
func TestWSConnectAndCleanDisconnect(t *testing.T) {
	srv := newWSTestServer(t, DefaultConnectionLimits())

	conn, _, err := dial(t, srv.url)
	if err != nil {
		t.Fatalf("dial: %v", err)
	}
	init := readInitData(t, conn)
	if init.GridW != 40 || init.GridH != 40 {
		t.Fatalf("init_data carries the wrong grid: %dx%d", init.GridW, init.GridH)
	}
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })
	if srv.playerCount() != 1 {
		t.Fatalf("want 1 player in the world, got %d", srv.playerCount())
	}

	conn.WriteMessage(websocket.CloseMessage,
		websocket.FormatCloseMessage(websocket.CloseNormalClosure, ""))
	conn.Close()

	waitFor(t, "the player to leave the world", func() bool { return srv.playerCount() == 0 })
	waitFor(t, "the client registry to clear", func() bool { return srv.clientCount() == 0 })
	waitFor(t, "the guard slot to be freed", func() bool {
		total, _ := srv.game.guard.counts()
		return total == 0
	})
}

// Acceptance: a handful of synthetic clients behave like ordinary players.
//
// The per-address ceiling is raised here on purpose. Every test client dials
// from 127.0.0.1, and this case covers the join / tap / cleanup path, not the
// admission policy — TestWSPerAddressConnectionLimitIsEnforced owns that.
func TestWSFiveSyntheticClientsAllConnectAndClearOnExit(t *testing.T) {
	limits := DefaultConnectionLimits()
	limits.MaxConnectionsPerIP = 0 // one address, many simulated players
	srv := newWSTestServer(t, limits)

	const clients = 5
	conns := make([]*websocket.Conn, 0, clients)
	for i := 0; i < clients; i++ {
		conn, _, err := dial(t, srv.url)
		if err != nil {
			t.Fatalf("client %d dial: %v", i, err)
		}
		readInitData(t, conn)
		conns = append(conns, conn)
	}
	waitFor(t, "all clients to register", func() bool { return srv.clientCount() == clients })
	if got := srv.playerCount(); got != clients {
		t.Fatalf("want %d players, got %d", clients, got)
	}

	// Each client taps, as the load test does.
	for i, conn := range conns {
		for tap := 1; tap <= 5; tap++ {
			if err := sendFrame(t, conn, "player_action", map[string]any{
				"x": 10 + tap, "y": 12, "tick": 0, "seq": tap,
			}); err != nil {
				t.Fatalf("client %d tap %d: %v", i, tap, err)
			}
		}
	}

	for _, conn := range conns {
		conn.Close()
	}
	waitFor(t, "every player to leave", func() bool { return srv.playerCount() == 0 })
	waitFor(t, "every guard slot to be freed", func() bool {
		total, addresses := srv.game.guard.counts()
		return total == 0 && addresses == 0
	})
}

// Acceptance: excessive connections from one address are controlled.
func TestWSPerAddressConnectionLimitIsEnforced(t *testing.T) {
	limits := DefaultConnectionLimits()
	limits.MaxConnectionsPerIP = 3
	limits.ConnectRatePerIP = 1000
	limits.ConnectBurstPerIP = 1000
	srv := newWSTestServer(t, limits)

	var held []*websocket.Conn
	for i := 0; i < 3; i++ {
		conn, _, err := dial(t, srv.url)
		if err != nil {
			t.Fatalf("connection %d must be admitted: %v", i, err)
		}
		readInitData(t, conn)
		held = append(held, conn)
	}

	_, resp, err := dial(t, srv.url)
	if err == nil {
		t.Fatal("the fourth connection from one address must be refused")
	}
	if resp == nil || resp.StatusCode != http.StatusServiceUnavailable {
		t.Fatalf("want 503 for a refused connection, got %v", resp)
	}
	// The refusal must not have cost a world slot.
	if got := srv.playerCount(); got != 3 {
		t.Fatalf("a refused connection must not create a player: %d players", got)
	}

	// Freeing one lets the next in.
	held[0].Close()
	waitFor(t, "the freed slot", func() bool {
		total, _ := srv.game.guard.counts()
		return total == 2
	})
	conn, _, err := dial(t, srv.url)
	if err != nil {
		t.Fatalf("a freed slot must accept a new connection: %v", err)
	}
	readInitData(t, conn)
	conn.Close()
	for _, c := range held[1:] {
		c.Close()
	}
}

// Acceptance: the connect rate from one address is controlled.
func TestWSConnectRateLimitIsEnforced(t *testing.T) {
	limits := DefaultConnectionLimits()
	limits.MaxConnectionsPerIP = 1000
	limits.ConnectRatePerIP = 1
	limits.ConnectBurstPerIP = 4
	srv := newWSTestServer(t, limits)

	admittedCount := 0
	var conns []*websocket.Conn
	for i := 0; i < 12; i++ {
		conn, _, err := dial(t, srv.url)
		if err != nil {
			continue
		}
		admittedCount++
		conns = append(conns, conn)
	}
	for _, c := range conns {
		c.Close()
	}
	if admittedCount > 5 {
		t.Fatalf("a burst of 12 must be throttled to about the burst size, %d got through", admittedCount)
	}
	if admittedCount == 0 {
		t.Fatal("the burst allowance must let some connections through")
	}
}

// Acceptance: excessive input is rate limited and the client is disconnected,
// and the disconnect leaves nothing behind.
func TestWSInputFloodEvictsTheClientAndCleansUp(t *testing.T) {
	limits := DefaultConnectionLimits()
	limits.MessageRate = 20
	limits.MessageBurst = 20
	limits.MaxStrikes = 5
	srv := newWSTestServer(t, limits)

	conn, _, err := dial(t, srv.url)
	if err != nil {
		t.Fatalf("dial: %v", err)
	}
	readInitData(t, conn)
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })

	// Flood well past the burst plus the strike allowance.
	for i := 1; i <= 400; i++ {
		if err := sendFrame(t, conn, "player_action", map[string]any{
			"x": 10, "y": 10, "tick": 0, "seq": i,
		}); err != nil {
			break // the server has closed the connection
		}
	}

	waitFor(t, "the flooding client to be evicted", func() bool { return srv.playerCount() == 0 })
	waitFor(t, "the guard slot to be freed after eviction", func() bool {
		total, _ := srv.game.guard.counts()
		return total == 0
	})
	if srv.game.counters.wsEvictedTotal.Load() == 0 {
		t.Fatal("the eviction must be counted")
	}
	conn.Close()
}

// Acceptance: an abnormal client sending malformed frames is disconnected.
func TestWSMalformedFrameFloodEvictsTheClient(t *testing.T) {
	limits := DefaultConnectionLimits()
	limits.MaxStrikes = 5
	srv := newWSTestServer(t, limits)

	conn, _, err := dial(t, srv.url)
	if err != nil {
		t.Fatalf("dial: %v", err)
	}
	readInitData(t, conn)
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })

	for i := 0; i < 50; i++ {
		if err := conn.WriteMessage(websocket.BinaryMessage, []byte("{not json")); err != nil {
			break
		}
		time.Sleep(time.Millisecond)
	}

	waitFor(t, "the malformed client to be evicted", func() bool { return srv.playerCount() == 0 })
	conn.Close()
}

// A tap with a non-finite or absurd target must be rejected at the boundary and
// must never reach the pathfinder. The connection survives an occasional one.
func TestWSAbsurdTapTargetsAreRejectedWithoutKillingTheSession(t *testing.T) {
	limits := DefaultConnectionLimits()
	limits.MaxStrikes = 1000 // isolate validation from eviction
	srv := newWSTestServer(t, limits)

	conn, _, err := dial(t, srv.url)
	if err != nil {
		t.Fatalf("dial: %v", err)
	}
	readInitData(t, conn)
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })

	// json.Unmarshal rejects NaN/Inf literals, so send the absurd finite values
	// a hostile client can actually put on the wire.
	for _, target := range []struct{ x, y float64 }{
		{1e18, 1e18}, {-1e18, 0}, {maxTapCoordinate + 1, 5},
	} {
		if err := sendFrame(t, conn, "player_action", map[string]any{
			"x": target.x, "y": target.y, "tick": 0, "seq": 1,
		}); err != nil {
			t.Fatalf("send absurd tap: %v", err)
		}
	}
	// A valid tap afterwards proves the session is still usable.
	if err := sendFrame(t, conn, "player_action", map[string]any{
		"x": 12, "y": 12, "tick": 0, "seq": 2,
	}); err != nil {
		t.Fatalf("valid tap after absurd ones: %v", err)
	}

	time.Sleep(100 * time.Millisecond)
	if srv.playerCount() != 1 {
		t.Fatal("a few bad taps must not drop the session")
	}
	conn.Close()
	waitFor(t, "cleanup", func() bool { return srv.playerCount() == 0 })
}

// Acceptance: after a load run ends, the server holds no residue.
func TestWSNoResidueAfterAConcurrentConnectAndDisconnectCycle(t *testing.T) {
	limits := DefaultConnectionLimits()
	limits.MaxConnectionsPerIP = 1000
	limits.ConnectRatePerIP = 10000
	limits.ConnectBurstPerIP = 10000
	srv := newWSTestServer(t, limits)

	const rounds, perRound = 3, 12
	for round := 0; round < rounds; round++ {
		var wg sync.WaitGroup
		for i := 0; i < perRound; i++ {
			wg.Add(1)
			go func() {
				defer wg.Done()
				conn, _, err := websocket.DefaultDialer.Dial(srv.url, nil)
				if err != nil {
					return
				}
				pack, _ := serial.Pack("player_action", map[string]any{"x": 9, "y": 9, "tick": 0, "seq": 1})
				conn.WriteMessage(websocket.BinaryMessage, pack)
				conn.Close()
			}()
		}
		wg.Wait()
	}

	waitFor(t, "every player to be cleared", func() bool { return srv.playerCount() == 0 })
	waitFor(t, "every client to be cleared", func() bool { return srv.clientCount() == 0 })
	waitFor(t, "every guard slot to be freed", func() bool {
		total, addresses := srv.game.guard.counts()
		return total == 0 && addresses == 0
	})
}

// The load-test profile: many concurrent connections from one address, which
// the default guard would refuse. With limits off every one is admitted, and
// the teardown still leaves nothing behind.
func TestWSUnlimitedProfileAcceptsAWideSameAddressRun(t *testing.T) {
	srv := newWSTestServer(t, UnlimitedConnectionLimits())

	const clients = 120 // past MaxConnectionsPerIP (64) and ConnectBurstPerIP (32)
	var mu sync.Mutex
	var conns []*websocket.Conn
	var refusals int

	var wg sync.WaitGroup
	for i := 0; i < clients; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			conn, _, err := websocket.DefaultDialer.Dial(srv.url, nil)
			mu.Lock()
			defer mu.Unlock()
			if err != nil {
				refusals++
				return
			}
			conns = append(conns, conn)
		}()
	}
	wg.Wait()

	if refusals != 0 {
		t.Fatalf("with limits disabled no connection may be refused, %d were", refusals)
	}
	waitFor(t, "every client to register", func() bool { return srv.clientCount() == clients })

	// Sustained input well past the default 30/s budget must not evict anyone.
	for _, conn := range conns {
		for seq := 1; seq <= 100; seq++ {
			sendFrame(t, conn, "player_action", map[string]any{"x": 10, "y": 10, "tick": 0, "seq": seq})
		}
	}
	time.Sleep(200 * time.Millisecond)
	if srv.game.counters.wsEvictedTotal.Load() != 0 {
		t.Fatal("no client may be evicted with rate limiting disabled")
	}
	if got := srv.clientCount(); got != clients {
		t.Fatalf("every client must survive the flood, %d of %d left", got, clients)
	}

	for _, conn := range conns {
		conn.Close()
	}
	waitFor(t, "every player to be cleared", func() bool { return srv.playerCount() == 0 })
	waitFor(t, "every guard slot to be freed", func() bool {
		total, addresses := srv.game.guard.counts()
		return total == 0 && addresses == 0
	})
}
