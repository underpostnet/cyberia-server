package net

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"strings"
	"sync"
	"testing"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/persist"
	"cyberia-server/game/world"

	"github.com/gorilla/websocket"
)

const testTicket = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"

// fakeEngine is the engine side of launches: one account, one profile, and a
// log of the calls in order. guest makes every ticket a guest ticket.
type fakeEngine struct {
	mu          sync.Mutex
	calls       []string
	refusal     string
	guest       bool
	down        bool
	saved       persist.State
	created     bool
	epoch       uint64
	commits     []persist.CommitRequest
	released    int
	consumers   map[string]bool // connection ids that consumed a ticket
	travel      string          // the refusal of a travel ticket, empty to issue one
	note        string          // the status note of the account
	noteRefusal string          // the refusal of a note, empty to take it
	friends     []FriendRecord  // the friend list of the account
}

func (f *fakeEngine) log(call string) {
	f.mu.Lock()
	f.calls = append(f.calls, call)
	f.mu.Unlock()
}

func (f *fakeEngine) ConsumeTicket(_ context.Context, ticket, instanceCode, connectionID, _ string) (Account, error) {
	f.log("consume")
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.down {
		return Account{}, errors.New("engine down")
	}
	if f.refusal != "" {
		return Account{Refusal: f.refusal}, nil
	}
	if f.consumers == nil {
		f.consumers = map[string]bool{}
	}
	f.consumers[connectionID] = true
	if f.guest {
		return Account{Principal: PrincipalGuest}, nil
	}
	return Account{Principal: PrincipalAccount, AccountID: "acc1", DisplayName: "neo"}, nil
}

func (f *fakeEngine) IssueResume(_ context.Context, lease persist.Lease, _ string) (IssuedTicket, error) {
	f.log("resume")
	f.mu.Lock()
	defer f.mu.Unlock()
	if lease.Epoch != f.epoch {
		return IssuedTicket{Refusal: persist.EndedElsewhere}, nil
	}
	return IssuedTicket{Ticket: resumeTicketOf(lease.Epoch), ExpiresAt: time.Now().Add(5 * time.Minute)}, nil
}

// resumeTicketOf is the resume ticket the fake engine issues for an epoch.
func resumeTicketOf(epoch uint64) string {
	return fmt.Sprintf("%043d", epoch)
}

func (f *fakeEngine) IssueTravel(_ context.Context, lease persist.Lease, _, targetCode string) (IssuedTicket, error) {
	f.log("travel")
	f.mu.Lock()
	defer f.mu.Unlock()
	if lease.Epoch != f.epoch {
		return IssuedTicket{Refusal: persist.EndedElsewhere}, nil
	}
	if f.travel != "" {
		return IssuedTicket{Refusal: f.travel}, nil
	}
	return IssuedTicket{Ticket: "travel-" + targetCode, ExpiresAt: time.Now().Add(time.Minute)}, nil
}

func (f *fakeEngine) AcquireSession(_ context.Context, connectionID, _, _ string) (AcquiredSession, error) {
	f.log("acquire")
	f.mu.Lock()
	defer f.mu.Unlock()
	if !f.consumers[connectionID] {
		return AcquiredSession{Refusal: "unknown"}, nil
	}
	f.epoch++
	return AcquiredSession{
		Lease:       persist.Lease{PlayerID: "prof1", Epoch: f.epoch},
		State:       f.saved,
		Created:     f.created,
		LeaseLength: time.Minute,
	}, nil
}

func (f *fakeEngine) ListFriends(context.Context, string) (FriendList, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	return FriendList{Friends: f.friends, Note: f.note}, nil
}

func (f *fakeEngine) UpdateStatusNote(_ context.Context, _, note string) (string, error) {
	f.log("note")
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.noteRefusal == "" {
		f.note = note
	}
	return f.noteRefusal, nil
}

func (f *fakeEngine) UpdateFriendship(context.Context, string, string, string, string) (string, *FriendRecord, error) {
	return "", nil, nil
}

func (f *fakeEngine) Commit(_ context.Context, req persist.CommitRequest) (persist.CommitResult, error) {
	f.log("commit")
	f.mu.Lock()
	defer f.mu.Unlock()
	if req.Epoch != f.epoch {
		return persist.CommitResult{Refusal: persist.RefusalStaleSession}, nil
	}
	f.commits = append(f.commits, req)
	f.saved = req.State
	return persist.CommitResult{}, nil
}

func (f *fakeEngine) Checkpoint(context.Context, []persist.Checkpoint) ([]persist.Ended, error) {
	return nil, nil
}

func (f *fakeEngine) Release(context.Context, persist.Lease) error {
	f.log("release")
	f.mu.Lock()
	f.released++
	f.mu.Unlock()
	return nil
}

func (f *fakeEngine) lastCommit() (persist.CommitRequest, bool) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if len(f.commits) == 0 {
		return persist.CommitRequest{}, false
	}
	return f.commits[len(f.commits)-1], true
}

func (f *fakeEngine) callLog() string {
	f.mu.Lock()
	defer f.mu.Unlock()
	return strings.Join(f.calls, ",")
}

// newAccountServer is a test server in account mode over engine, with one
// weapon "sword" bound to "cid-sword".
func newAccountServer(t *testing.T, engine *fakeEngine) *wsTestServer {
	t.Helper()
	srv := newWSTestServer(t, DefaultConnectionLimits())
	srv.game.EnableAccounts(engine, engine, "test-server")
	srv.game.ObjectLayerDataCache = map[string]*content.ObjectLayer{
		"sword": {Cid: "cid-sword", Data: content.ObjectLayerData{Item: content.Item{ID: "sword", Type: "weapon"}}},
	}
	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(cancel)
	go srv.game.durable.Run(ctx)
	return srv
}

// connect opens a socket and sends the auth message with ticket.
func connect(t *testing.T, srv *wsTestServer, ticket string) *websocket.Conn {
	t.Helper()
	conn, _, err := websocket.DefaultDialer.Dial(srv.url, nil)
	if err != nil {
		t.Fatalf("dial: %v", err)
	}
	t.Cleanup(func() { conn.Close() })
	if err := sendAuth(t, conn, ticket); err != nil {
		t.Fatalf("auth: %v", err)
	}
	return conn
}

// readPayload reads frames until one of msgType and decodes its payload into out.
func readPayload(t *testing.T, conn *websocket.Conn, msgType string, out any) {
	t.Helper()
	conn.SetReadDeadline(time.Now().Add(5 * time.Second))
	for {
		_, raw, err := conn.ReadMessage()
		if err != nil {
			t.Fatalf("%s never arrived: %v", msgType, err)
		}
		if msg, err := Unpack(raw); err == nil && msg.Type == msgType {
			json.Unmarshal(msg.Payload, out)
			return
		}
	}
}

// readSessionEnd reads frames until session_end and returns its reason.
func readSessionEnd(t *testing.T, conn *websocket.Conn) string {
	t.Helper()
	var end sessionEnd
	readPayload(t, conn, "session_end", &end)
	return end.Reason
}

func TestAccountServerRefusesWhatIsNotAValidLaunch(t *testing.T) {
	cases := []struct {
		name   string
		engine *fakeEngine
		send   func(*testing.T, *websocket.Conn)
		want   string
	}{
		{"no ticket", &fakeEngine{}, func(t *testing.T, c *websocket.Conn) { sendAuth(t, c, "") }, EndAuthRequired},
		{"malformed ticket", &fakeEngine{}, func(t *testing.T, c *websocket.Conn) { sendAuth(t, c, "short") }, EndInvalid},
		{"gameplay first", &fakeEngine{}, func(t *testing.T, c *websocket.Conn) {
			sendEvent(t, c, "player_action", map[string]any{"x": 1, "y": 1, "seq": 1})
		}, EndInvalid},
		{"replayed ticket", &fakeEngine{refusal: "replayed"}, func(t *testing.T, c *websocket.Conn) { sendAuth(t, c, testTicket) }, "replayed"},
		{"expired ticket", &fakeEngine{refusal: "expired"}, func(t *testing.T, c *websocket.Conn) { sendAuth(t, c, testTicket) }, "expired"},
		{"engine down", &fakeEngine{down: true}, func(t *testing.T, c *websocket.Conn) { sendAuth(t, c, testTicket) }, EndUnavailable},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			srv := newAccountServer(t, tc.engine)
			conn, _, err := websocket.DefaultDialer.Dial(srv.url, nil)
			if err != nil {
				t.Fatalf("dial: %v", err)
			}
			defer conn.Close()
			tc.send(t, conn)
			if got := readSessionEnd(t, conn); got != tc.want {
				t.Fatalf("session_end reason %q, want %q", got, tc.want)
			}
			waitFor(t, "the guard slot to free", func() bool { total, _ := srv.game.guard.counts(); return total == 0 })
			if srv.playerCount() != 0 || srv.clientCount() != 0 {
				t.Fatal("a refused connection leaves no player behind")
			}
		})
	}
}

func TestAuthMessageHasATimeoutAndASizeLimit(t *testing.T) {
	srv := newAccountServer(t, &fakeEngine{})
	srv.game.SetAuthTimeout(100 * time.Millisecond)
	silent, _, err := websocket.DefaultDialer.Dial(srv.url, nil)
	if err != nil {
		t.Fatalf("dial: %v", err)
	}
	defer silent.Close()
	if got := readSessionEnd(t, silent); got != EndInvalid {
		t.Fatalf("a silent connection ends as invalid, got %q", got)
	}

	big, _, err := websocket.DefaultDialer.Dial(srv.url, nil)
	if err != nil {
		t.Fatalf("dial: %v", err)
	}
	defer big.Close()
	pack, _ := Pack("auth", map[string]any{"ticket": strings.Repeat("A", 4096)})
	big.WriteMessage(websocket.BinaryMessage, pack)
	big.SetReadDeadline(time.Now().Add(5 * time.Second))
	for {
		if _, _, err := big.ReadMessage(); err != nil {
			break
		}
	}
	if srv.playerCount() != 0 {
		t.Fatal("an oversized auth message admits nobody")
	}
}

func TestAnAccountLoadsItsSavedState(t *testing.T) {
	engine := &fakeEngine{saved: persist.State{
		Coins:    42,
		XP:       0,
		Bag:      []persist.Stack{{ItemID: "sword", CID: "cid-sword", Quantity: 1, Slot: 3}},
		Storage:  []persist.Stack{{ItemID: "sword", CID: "cid-old", Quantity: 2, Slot: 0}},
		Position: persist.Position{MapCode: "test", X: 7, Y: 9},
	}}
	srv := newAccountServer(t, engine)
	conn := connect(t, srv, testTicket)
	init := readInitData(t, conn)
	if init.Session != PrincipalAccount {
		t.Fatal("init_data marks an account session")
	}
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })

	srv.game.Mu.Lock()
	var player *world.Session
	for _, p := range srv.game.Sessions {
		player = p
	}
	ms := srv.game.Maps["test"]
	pos := ms.Tf(player.ID).Pos
	bag := *srv.game.InventoryLayers(player)
	held := len(srv.game.HeldView(player))
	storage := srv.game.PlayerStorage(player)
	srv.game.Mu.Unlock()

	if player.PlayerID != "prof1" || player.AccountID != "acc1" || player.DisplayName != "neo" || player.Coins != 42 {
		t.Fatalf("the session takes the profile, the account and the coins: %+v", player)
	}
	if pos.X != 7 || pos.Y != 9 {
		t.Fatalf("the player spawns at its saved position, got %+v", pos)
	}
	var sword *world.ObjectLayerState
	for i := range bag {
		if bag[i].ItemID == "sword" {
			sword = &bag[i]
		}
	}
	if sword == nil || sword.Slot != 3 || sword.CID != "cid-sword" {
		t.Fatalf("the saved stack loads in its slot with its definition: %+v", bag)
	}
	if held != 1 || len(storage) != 0 {
		t.Fatalf("a stack of another definition is held, not loaded: held=%d storage=%v", held, storage)
	}
	waitFor(t, "the first commit", func() bool { _, ok := engine.lastCommit(); return ok })
	commit, _ := engine.lastCommit()
	if len(commit.State.Held) != 1 || commit.State.Held[0].Reason != persist.HeldRebound || commit.State.Held[0].Stack.CID != "cid-old" {
		t.Fatalf("the held stack keeps its definition in the profile: %+v", commit.State.Held)
	}
}

func TestAProfileThatLoadsUnchangedCommitsNothingAtJoin(t *testing.T) {
	engine := &fakeEngine{saved: persist.State{
		Coins:    42,
		Bag:      []persist.Stack{{ItemID: "sword", CID: "cid-sword", Quantity: 1, Slot: 3}},
		Position: persist.Position{MapCode: "test", X: 7, Y: 9},
	}}
	srv := newAccountServer(t, engine)
	conn := connect(t, srv, testTicket)
	readInitData(t, conn)
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })
	time.Sleep(200 * time.Millisecond)
	if commit, ok := engine.lastCommit(); ok {
		t.Fatalf("a profile placed at its saved position with its saved goods commits nothing: %+v", commit)
	}
}

func TestAnAccountServerAdmitsAGuestTicketAsAnEphemeralGuest(t *testing.T) {
	engine := &fakeEngine{guest: true}
	srv := newAccountServer(t, engine)
	conn := connect(t, srv, testTicket)
	if init := readInitData(t, conn); init.Session != PrincipalGuest {
		t.Fatalf("a guest ticket plays a guest session: %+v", init)
	}
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })
	srv.game.Mu.Lock()
	var guest *world.Session
	for _, p := range srv.game.Sessions {
		guest = p
	}
	srv.game.Mu.Unlock()
	if guest.AccountID != "" || !strings.HasPrefix(guest.DisplayName, "Guest-") {
		t.Fatalf("a guest has no account and a server guest name: %+v", guest)
	}
	conn.Close()
	waitFor(t, "the guest to leave", func() bool { return srv.clientCount() == 0 })
	time.Sleep(100 * time.Millisecond)
	if log := engine.callLog(); log != "consume" {
		t.Fatalf("a guest never acquires, commits, releases or resumes a profile: %s", log)
	}
}

func TestAnAccountSessionHoldsAResumeTicketOfItsEpoch(t *testing.T) {
	engine := &fakeEngine{created: true}
	srv := newAccountServer(t, engine)
	conn := connect(t, srv, testTicket)
	readInitData(t, conn)
	var resume resumePayload
	readPayload(t, conn, "resume", &resume)
	if resume.Ticket != resumeTicketOf(1) {
		t.Fatalf("the resume ticket names epoch 1: %q", resume.Ticket)
	}
}

func TestAPlayerWhoLeavesLearnsItsStateIsSaved(t *testing.T) {
	engine := &fakeEngine{created: true}
	srv := newAccountServer(t, engine)
	conn := connect(t, srv, testTicket)
	readInitData(t, conn)
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })
	if err := sendEvent(t, conn, "session_leave", map[string]any{"seq": 1}); err != nil {
		t.Fatalf("leave: %v", err)
	}
	if reason := readSessionEnd(t, conn); reason != EndLeft {
		t.Fatalf("a saved leave ends with %q, got %q", EndLeft, reason)
	}
	if log := engine.callLog(); !strings.HasSuffix(log, "commit,release") {
		t.Fatalf("the final state commits and the lease is released before session_end: %s", log)
	}
	if srv.clientCount() != 0 || srv.playerCount() != 0 {
		t.Fatal("a player who left is out of the world")
	}
}

func TestAFriendElsewhereShowsTheAvatarTheEngineNames(t *testing.T) {
	const avatar = "/api/v1/cyberia-player/appearance/v1/cid-a,cid-sword"
	engine := &fakeEngine{created: true, friends: []FriendRecord{
		{FriendshipID: "64a000000000000000000009", AccountID: "acc2", Name: "trinity", Status: "friend", Avatar: avatar},
	}}
	srv := newAccountServer(t, engine)
	conn := connect(t, srv, testTicket)
	readInitData(t, conn)
	var list struct {
		Friends []struct{ Name, Avatar string }
	}
	readPayload(t, conn, "friends", &list)
	if len(list.Friends) != 1 || list.Friends[0].Avatar != avatar {
		t.Fatalf("a friend on no server of this world keeps the avatar of its profile: %+v", list.Friends)
	}
}

func TestAStatusNoteReachesTheEngineAndComesBackWithTheFriendList(t *testing.T) {
	engine := &fakeEngine{created: true}
	srv := newAccountServer(t, engine)
	conn := connect(t, srv, testTicket)
	readInitData(t, conn)
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })
	type friends struct{ Note string }

	if err := sendEvent(t, conn, "status_note", map[string]any{"seq": 1, "text": "  out\tfarming "}); err != nil {
		t.Fatalf("note: %v", err)
	}
	waitFor(t, "the note to be queued", func() bool { srv.tick(); return strings.Contains(engine.callLog(), "note") })
	var list friends
	readPayload(t, conn, "friends", &list)
	for list.Note == "" {
		readPayload(t, conn, "friends", &list)
	}
	if list.Note != "  out\tfarming " {
		t.Fatalf("the engine note comes back with the list, got %q", list.Note)
	}

	var saved struct {
		Ok     bool
		Action string
	}
	readPayload(t, conn, "friend_ack", &saved)
	if !saved.Ok || saved.Action != "note" {
		t.Fatalf("note save: %+v", saved)
	}

	engine.mu.Lock()
	engine.noteRefusal = "too_soon"
	engine.mu.Unlock()
	if err := sendEvent(t, conn, "status_note", map[string]any{"seq": 2, "text": "again"}); err != nil {
		t.Fatalf("note: %v", err)
	}
	var ack struct {
		Ok             bool
		Reason, Action string
	}
	waitFor(t, "the second note to be queued", func() bool { srv.tick(); return strings.Count(engine.callLog(), "note") == 2 })
	readPayload(t, conn, "friend_ack", &ack)
	if ack.Ok || ack.Reason != "too_soon" || ack.Action != "note" {
		t.Fatalf("a refused note answers with friend_ack: %+v", ack)
	}
}

func TestATravelerLeavesSavedWithTheTicketOfItsTarget(t *testing.T) {
	engine := &fakeEngine{created: true}
	srv := newAccountServer(t, engine)
	srv.game.SetBuild("0123456789abcdef")
	conn := connect(t, srv, testTicket)
	if init := readInitData(t, conn); init.Build != "0123456789abcdef" {
		t.Fatalf("init_data names the build of the server: %q", init.Build)
	}
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })
	if err := sendEvent(t, conn, "session_leave", map[string]any{"seq": 1, "travel": "world-b"}); err != nil {
		t.Fatalf("leave: %v", err)
	}
	var end sessionEnd
	readPayload(t, conn, "session_end", &end)
	if end.Reason != EndLeft || end.Travel == nil || *end.Travel != (travelTicket{InstanceCode: "world-b", Ticket: "travel-world-b"}) {
		t.Fatalf("a saved leave carries the ticket of the target world: %+v", end)
	}
	if log := engine.callLog(); !strings.HasSuffix(log, "travel,commit,release") {
		t.Fatalf("the ticket is asked while the lease holds, then the final state commits and the lease is released: %s", log)
	}
	if travel := srv.game.RuntimeMetrics().Travel; travel["issued"] != 1 {
		t.Fatalf("travel outcomes = %v", travel)
	}
}

func TestATravelTheEngineRefusesLeavesWithNoTicket(t *testing.T) {
	engine := &fakeEngine{created: true, travel: "no_route"}
	srv := newAccountServer(t, engine)
	conn := connect(t, srv, testTicket)
	readInitData(t, conn)
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })
	if err := sendEvent(t, conn, "session_leave", map[string]any{"seq": 1, "travel": "world-b"}); err != nil {
		t.Fatalf("leave: %v", err)
	}
	var end sessionEnd
	readPayload(t, conn, "session_end", &end)
	if end.Reason != EndLeft || end.Travel != nil {
		t.Fatalf("a refused travel still leaves saved, with no ticket: %+v", end)
	}
	if travel := srv.game.RuntimeMetrics().Travel; travel["no_route"] != 1 {
		t.Fatalf("travel outcomes = %v", travel)
	}
}

func TestAGuestLeavesAtOnce(t *testing.T) {
	srv := newWSTestServer(t, DefaultConnectionLimits())
	conn := connect(t, srv, "")
	readInitData(t, conn)
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })
	if err := sendEvent(t, conn, "session_leave", map[string]any{"seq": 1, "travel": "world-b"}); err != nil {
		t.Fatalf("leave: %v", err)
	}
	var end sessionEnd
	readPayload(t, conn, "session_end", &end)
	if end.Reason != EndLeft || end.Travel != nil {
		t.Fatalf("a guest leaves with %q and no travel ticket, got %+v", EndLeft, end)
	}
}

func TestDisconnectCommitsTheFinalStateThenReleases(t *testing.T) {
	engine := &fakeEngine{created: true}
	srv := newAccountServer(t, engine)
	conn := connect(t, srv, testTicket)
	readInitData(t, conn)
	waitFor(t, "the client to register", func() bool { return srv.clientCount() == 1 })
	srv.game.Mu.Lock()
	for _, p := range srv.game.Sessions {
		srv.game.AddCoins(p.ID, srv.game.Maps[p.MapCode], 25)
	}
	srv.game.Mu.Unlock()

	conn.Close()
	waitFor(t, "the release", func() bool { return strings.HasSuffix(engine.callLog(), "release") })
	commit, _ := engine.lastCommit()
	if commit.State.Coins != 25 || !commit.PositionChanged || commit.State.Position.MapCode != "test" {
		t.Fatalf("the final commit carries the coins and the position: %+v", commit)
	}
}

func TestASecondLaunchEndsTheFirstSessionAfterItsFinalCommit(t *testing.T) {
	engine := &fakeEngine{created: true}
	srv := newAccountServer(t, engine)
	first := connect(t, srv, testTicket)
	readInitData(t, first)
	waitFor(t, "the first client", func() bool { return srv.clientCount() == 1 })

	second := connect(t, srv, testTicket)
	if got := readSessionEnd(t, first); got != EndElsewhere {
		t.Fatalf("the older session ends as signed in elsewhere, got %q", got)
	}
	readInitData(t, second)
	waitFor(t, "one client", func() bool { return srv.clientCount() == 1 && srv.playerCount() == 1 })
	log := engine.callLog()
	if !strings.Contains(log, "release,acquire") {
		t.Fatalf("the older session commits and releases before the new one acquires: %s", log)
	}
}

func TestAnonymousServerIgnoresTickets(t *testing.T) {
	engine := &fakeEngine{}
	srv := newWSTestServer(t, DefaultConnectionLimits())
	srv.game.accounts = engine
	conn := connect(t, srv, testTicket)
	if init := readInitData(t, conn); init.Session != PrincipalGuest {
		t.Fatal("an anonymous server admits a guest")
	}
	if engine.callLog() != "" {
		t.Fatal("an anonymous server never consumes a ticket")
	}
}

func TestBrowserOriginsMustBeAllowed(t *testing.T) {
	srv := newWSTestServer(t, DefaultConnectionLimits())
	srv.game.SetAllowedOrigins([]string{"https://client.example", "http://localhost:*"})
	for origin, want := range map[string]bool{
		"https://client.example": true,
		"http://localhost:8082":  true,
		"https://evil.example":   false,
	} {
		conn, resp, err := websocket.DefaultDialer.Dial(srv.url, http.Header{"Origin": {origin}})
		if (err == nil) != want {
			t.Fatalf("origin %s: allowed=%v, want %v (resp %v)", origin, err == nil, want, resp)
		}
		if conn != nil {
			conn.Close()
		}
	}
}
