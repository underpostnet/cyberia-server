package net

import (
	"encoding/json"
	"strings"
	"testing"
	"time"

	"cyberia-server/game/sim"
	"cyberia-server/game/world"

	"github.com/gorilla/websocket"
)

// socialClient is one player of a social test: its socket, its own entity id
// and its sequence counter.
type socialClient struct {
	conn *websocket.Conn
	init InitPayload
	id   world.EntityID
	seq  int
}

// newSocialServer spawns every player on one cell, so all of them are in
// each other's view.
func newSocialServer(t *testing.T) *wsTestServer {
	t.Helper()
	srv := newWSTestServer(t, DefaultConnectionLimits())
	srv.game.PlayerSpawn = sim.PlayerSpawnConfig{MapCode: "test", CellX: 5, CellY: 5}
	return srv
}

func (w *wsTestServer) join(t *testing.T) *socialClient {
	t.Helper()
	want := w.clientCount() + 1
	conn, _, err := dial(t, w.url)
	if err != nil {
		t.Fatalf("dial: %v", err)
	}
	t.Cleanup(func() { conn.Close() })
	c := &socialClient{conn: conn, init: readInitData(t, conn)}
	waitFor(t, "the client to register", func() bool { return w.clientCount() == want })
	w.game.Mu.Lock()
	for id, p := range w.game.Sessions {
		if p.PlayerID == c.init.PlayerID {
			c.id = id
		}
	}
	w.game.Mu.Unlock()
	return c
}

// tick applies the queued events of every player, then replicates.
func (w *wsTestServer) tick() {
	w.game.Mu.Lock()
	defer w.game.Mu.Unlock()
	for _, ms := range w.game.Maps {
		w.game.PhaseInput(0, ms)
		w.game.phaseReplication(ms)
	}
}

// send pushes one event and waits until the server queued it.
func (c *socialClient) send(t *testing.T, w *wsTestServer, eventType string, payload map[string]any) {
	t.Helper()
	c.seq++
	payload["seq"] = c.seq
	if err := sendEvent(t, c.conn, eventType, payload); err != nil {
		t.Fatalf("send %s: %v", eventType, err)
	}
	waitFor(t, eventType+" to be queued", func() bool {
		w.game.Mu.Lock()
		defer w.game.Mu.Unlock()
		p := w.game.Sessions[c.id]
		return p != nil && len(p.InputQueue) > 0
	})
}

// next reads frames until one of msgType arrives and returns its payload.
func (c *socialClient) next(t *testing.T, msgType string, match func(json.RawMessage) bool) json.RawMessage {
	t.Helper()
	c.conn.SetReadDeadline(time.Now().Add(5 * time.Second))
	for {
		_, raw, err := c.conn.ReadMessage()
		if err != nil {
			t.Fatalf("waiting for %s: %v", msgType, err)
		}
		msg, err := Unpack(raw)
		if err == nil && msg.Type == msgType && (match == nil || match(msg.Payload)) {
			return msg.Payload
		}
	}
}

// self reads snapshots until match holds for the own block.
func (c *socialClient) self(t *testing.T, match func(SnapshotSelf) bool) SnapshotSelf {
	t.Helper()
	var snap Snapshot
	c.next(t, "snapshot", func(raw json.RawMessage) bool {
		snap = Snapshot{}
		return json.Unmarshal(raw, &snap) == nil && match(snap.Self)
	})
	return snap.Self
}

func TestWSPlayersGetASessionIdentityAndAGuestName(t *testing.T) {
	srv := newSocialServer(t)
	a := srv.join(t)
	srv.tick()
	self := a.self(t, func(SnapshotSelf) bool { return true })
	if a.init.PlayerID == "" || a.init.MaxLevel < 1 {
		t.Fatalf("init_data names the player and the level cap: %+v", a.init)
	}
	if !strings.HasPrefix(self.Name, "Guest-") || self.Storage == nil {
		t.Fatalf("the own block carries the name and the storage: name=%q storage=%v", self.Name, self.Storage)
	}
	if strings.Contains(string(mustJSON(t, self)), string(a.init.PlayerID)) {
		t.Fatal("the player id never goes in a snapshot")
	}
}

func TestWSWorldChatReachesEveryPlayerWithTheServerName(t *testing.T) {
	srv := newSocialServer(t)
	a, b := srv.join(t), srv.join(t)
	srv.tick()
	a.send(t, srv, "chat", map[string]any{"channel": "world", "text": "  hello\nworld "})
	srv.tick()

	var line struct{ Channel, FromName, Text string }
	json.Unmarshal(b.next(t, "chat", nil), &line)
	if line.Channel != "world" || line.Text != "hello world" || !strings.HasPrefix(line.FromName, "Guest-") {
		t.Fatalf("b got %+v", line)
	}
	json.Unmarshal(a.next(t, "chat", nil), &line)
	if line.Text != "hello world" {
		t.Fatalf("the sender sees its own line as the server accepted it, got %+v", line)
	}
}

func TestWSPartyChatRefusesAPlayerInNoParty(t *testing.T) {
	srv := newSocialServer(t)
	a := srv.join(t)
	a.send(t, srv, "chat", map[string]any{"channel": "party", "text": "anyone?"})
	srv.tick()
	var ack struct {
		Ok      bool
		Reason  string
		Channel string
	}
	json.Unmarshal(a.next(t, "chat_ack", nil), &ack)
	if ack.Ok || ack.Reason != "no_party" || ack.Channel != "party" {
		t.Fatalf("ack: %+v", ack)
	}
}

func TestWSInviteAcceptLeadAndDisconnect(t *testing.T) {
	srv := newSocialServer(t)
	a, b := srv.join(t), srv.join(t)
	srv.tick() // the AOI exists from the first replication on

	a.send(t, srv, "party_invite", map[string]any{"toId": b.id})
	srv.tick()
	invited := b.self(t, func(s SnapshotSelf) bool { return len(s.PartyInvites) == 1 })
	if invited.PartyInvites[0].FromID != a.id || invited.PartyInvites[0].ExpiresInMs <= 0 {
		t.Fatalf("invite: %+v", invited.PartyInvites)
	}

	b.send(t, srv, "party_reply", map[string]any{"fromId": a.id, "accept": true})
	srv.tick()
	led := a.self(t, func(s SnapshotSelf) bool { return s.Party != nil })
	if led.Party.LeaderID != a.id || len(led.Party.Members) != 2 || led.Party.Members[1].ID != b.id {
		t.Fatalf("party: %+v", led.Party)
	}

	b.send(t, srv, "chat", map[string]any{"channel": "party", "text": "on my way"})
	srv.tick()
	var line struct{ Channel, Text string }
	json.Unmarshal(a.next(t, "chat", nil), &line)
	if line.Channel != "party" || line.Text != "on my way" {
		t.Fatalf("party line: %+v", line)
	}

	a.conn.Close()
	waitFor(t, "a to leave", func() bool { return srv.clientCount() == 1 })
	srv.tick()
	if alone := b.self(t, func(s SnapshotSelf) bool { return s.Party == nil }); alone.Party != nil {
		t.Fatal("a party of one ends")
	}
}

// Personal storage is session state: a deposit shows in the own block, and a
// new session starts with none.
func TestWSPersonalStorageEndsWithTheSession(t *testing.T) {
	srv := newSocialServer(t)
	a := srv.join(t)
	srv.game.Mu.Lock()
	p := srv.game.Sessions[a.id]
	layers := srv.game.InventoryLayers(p)
	*layers = append(*layers, world.ObjectLayerState{ItemID: "gem", Quantity: 3})
	srv.game.SeatStacks(*layers, len(*layers)-1)
	slot := (*layers)[len(*layers)-1].Slot
	srv.game.Mu.Unlock()

	a.send(t, srv, "item_move", map[string]any{"from": "bag", "fromSlot": slot, "to": "storage", "toSlot": 0, "qty": 2})
	srv.tick()
	stored := a.self(t, func(s SnapshotSelf) bool { return len(s.Storage) == 1 })
	if stored.Storage[0].ItemID != "gem" || stored.Storage[0].Qty != 2 {
		t.Fatalf("storage: %+v", stored.Storage)
	}

	a.conn.Close()
	waitFor(t, "a to leave", func() bool { return srv.clientCount() == 0 })
	again := srv.join(t)
	srv.tick()
	if fresh := again.self(t, func(SnapshotSelf) bool { return true }); len(fresh.Storage) != 0 {
		t.Fatalf("a new session holds no storage: %+v", fresh.Storage)
	}
}

func mustJSON(t *testing.T, v any) []byte {
	t.Helper()
	raw, err := json.Marshal(v)
	if err != nil {
		t.Fatal(err)
	}
	return raw
}
