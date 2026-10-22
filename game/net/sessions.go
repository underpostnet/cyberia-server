// Package net — sessions.go
//
// A connection becomes a player in three steps: admission (draining server,
// connection guard, origin), authentication (the first message, a launch
// ticket in account mode) and placement (spawn and saved state). A refused
// connection gets session_end and leaves nothing behind.
//
// Every connection opens with auth {ticket?}. An anonymous server admits a
// guest and never reads the ticket. An account server requires a ticket and
// takes the principal from the engine: an account plays its profile, a guest
// plays an ephemeral session.
//
// One account plays one session on this server: a new launch ends the older
// session first and adopts what it could not save. A session of the account
// that still saves on another server holds the acquisition back (saving)
// until it saved, or until handoffWait passes.
package net

import (
	"context"
	"encoding/json"
	"math"
	"math/rand"
	"net/http"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/persist"
	"cyberia-server/game/sim"
	"cyberia-server/game/world"
	"cyberia-server/logx"

	"github.com/google/uuid"
)

const (
	// engineCallBudget bounds each engine step of one authentication: the
	// ticket, the acquisition, the content of the saved stacks.
	engineCallBudget = 10 * time.Second
	// takeoverWait bounds how long a launch waits for a pending joint commit
	// of the older session of its account.
	takeoverWait = 10 * time.Second
	// handoffWait bounds how long an acquisition asks again while the session
	// the account left still saves on another server.
	handoffWait = 8 * time.Second
	// leaveWait bounds how long a player who leaves waits to learn that its
	// final state is saved.
	leaveWait = 5 * time.Second
	// travelWait bounds the engine call for the ticket of a travel.
	travelWait = 5 * time.Second
)

// identity is who a connection plays as: a guest, or an account with its
// acquired profile, the content resolved for its saved stacks and the hold
// reason of each label whose content this world refused.
type identity struct {
	account     Account
	session     AcquiredSession
	resolved    map[string]*content.ItemContent
	unavailable map[string]string
}

// sessionEnd is the payload of session_end.
type sessionEnd struct {
	Reason string `json:"reason"`
	// Travel is the launch ticket of the world the player travels to, set
	// when the engine issued one.
	Travel *travelTicket `json:"travel,omitempty"`
}

// travelTicket is the launch ticket of the world a leaving player travels to.
type travelTicket struct {
	InstanceCode string `json:"instanceCode"`
	Ticket       string `json:"ticket"`
}

// HandleConnections admits, authenticates and places one WebSocket
// connection. Admission runs before the upgrade so a refused client costs one
// HTTP response, not a WebSocket session.
func (s *GameServer) HandleConnections(w http.ResponseWriter, r *http.Request) {
	if s.IsDraining() {
		s.recordWsRefused()
		http.Error(w, "server is draining", http.StatusServiceUnavailable)
		return
	}
	ip := clientIP(r)
	if refusal := s.guard.admit(ip); refusal != admitted {
		s.recordWsRefused()
		logx.Debugf("[HandleConnections] refused ip=%s: %s", ip, refusal)
		http.Error(w, string(refusal), http.StatusServiceUnavailable)
		return
	}
	conn, err := s.upgrader.Upgrade(w, r, nil)
	if err != nil {
		logx.Debugf("Upgrade failed: %v", err)
		s.guard.release(ip)
		return
	}
	sock := NewSocket(conn, Metrics{
		Read:       s.recordWsRead,
		Write:      s.recordWsWrite,
		ReadError:  s.recordWsReadError,
		WriteError: s.recordWsWriteError,
	})
	who, refusal := s.authenticate(sock, uuid.New().String())
	s.recordAuth(refusal)
	if refusal != "" {
		logx.Infof("[Auth] refused ip=%s: %s", ip, refusal)
		s.refuse(sock, refusal)
		s.guard.release(ip)
		return
	}
	s.place(sock, ip, who)
}

// refuse ends a connection that never became a player.
func (s *GameServer) refuse(sock *Socket, reason string) {
	pack, err := Pack("session_end", sessionEnd{Reason: reason})
	if err != nil {
		sock.Close()
		return
	}
	sock.End(pack)
}

// authenticate reads the auth message. An anonymous server admits a guest; an
// account server consumes the ticket and, for an account, acquires the
// profile. It returns the identity, or the refusal.
func (s *GameServer) authenticate(sock *Socket, connectionID string) (identity, string) {
	pack, err := sock.ReadFirst(s.authTimeout)
	if err != nil {
		return identity{}, EndInvalid
	}
	msg, err := Unpack(pack)
	if err != nil || msg.Type != "auth" {
		return identity{}, EndInvalid
	}
	var auth struct {
		Ticket string `json:"ticket"`
	}
	if len(msg.Payload) > 0 && json.Unmarshal(msg.Payload, &auth) != nil {
		return identity{}, EndInvalid
	}
	if s.mode == SessionAnonymous {
		return identity{}, ""
	}
	switch {
	case auth.Ticket == "":
		return identity{}, EndAuthRequired
	case !ticketPattern.MatchString(auth.Ticket):
		return identity{}, EndInvalid
	}

	ctx, cancel := context.WithTimeout(context.Background(), engineCallBudget)
	account, err := s.accounts.ConsumeTicket(ctx, auth.Ticket, s.InstanceCode, connectionID, s.serverID)
	cancel()
	if err != nil {
		logx.Warnf("[Auth] ticket consumption failed: %v", err)
		return identity{}, EndUnavailable
	}
	if account.Refusal != "" {
		return identity{}, account.Refusal
	}
	if account.Principal == PrincipalGuest {
		return identity{}, ""
	}
	s.takeOver(account.AccountID)
	acquired, err := s.acquire(connectionID)
	if err != nil {
		logx.Warnf("[Auth] session acquisition failed account=%s: %v", account.AccountID, err)
		return identity{}, EndUnavailable
	}
	if acquired.Refusal != "" {
		return identity{}, acquired.Refusal
	}
	acquired.Lease.AccountID = account.AccountID
	if state, ok := s.durable.Unsettled(acquired.Lease.PlayerID); ok {
		logx.Infof("[Auth] account=%s adopts the unsaved state of its closed session", account.AccountID)
		acquired.State, acquired.Adopted = state, true
	}
	ctx, cancel = context.WithTimeout(context.Background(), engineCallBudget)
	defer cancel()
	resolved, unavailable, err := s.resolveSaved(ctx, acquired.State)
	if err != nil {
		logx.Warnf("[Auth] content of account=%s could not resolve: %v", account.AccountID, err)
		s.releaseAcquired(acquired)
		return identity{}, EndUnavailable
	}
	return identity{account: account, session: acquired, resolved: resolved, unavailable: unavailable}, ""
}

// acquire makes the connection the writer of its profile. While the session
// the account left still saves on another server, the engine answers saving
// and the acquisition asks again, at most handoffWait.
func (s *GameServer) acquire(connectionID string) (AcquiredSession, error) {
	ctx, cancel := context.WithTimeout(context.Background(), engineCallBudget)
	defer cancel()
	deadline := time.Now().Add(handoffWait)
	for wait := 250 * time.Millisecond; ; wait = min(wait*2, 2*time.Second) {
		acquired, err := s.accounts.AcquireSession(ctx, connectionID, s.InstanceCode, s.serverID)
		if err != nil || acquired.Refusal != EndSaving || time.Now().Add(wait).After(deadline) {
			return acquired, err
		}
		select {
		case <-ctx.Done():
			return acquired, nil
		case <-time.After(wait):
		}
	}
}

// openDurable starts the durable session of an acquired profile.
func (s *GameServer) openDurable(acquired AcquiredSession) {
	if acquired.Adopted {
		s.durable.Adopt(acquired.Lease, acquired.State)
	} else {
		s.durable.Open(acquired.Lease, acquired.State)
	}
}

// releaseAcquired ends an acquired session that never played: its state, the
// adopted one included, commits and the lease is released.
func (s *GameServer) releaseAcquired(acquired AcquiredSession) {
	s.openDurable(acquired)
	s.durable.Close(acquired.Lease.PlayerID, acquired.State)
}

// takeOver ends the session an account plays on this server. A joint commit of
// that session that is still pending decides which state the next session
// adopts, so the launch waits for it, at most takeoverWait.
func (s *GameServer) takeOver(accountID string) {
	s.Mu.Lock()
	var old *Client
	if player := s.Social.Account(accountID); player != nil {
		old = s.clients[player.ID]
	}
	s.Mu.Unlock()
	if old != nil {
		s.recordSessionEnd(EndElsewhere)
		s.endClient(old, EndElsewhere)
	}
	if pending := s.durable.PendingLink(accountID); pending != nil {
		select {
		case <-pending:
		case <-time.After(takeoverWait):
			logx.Warnf("[Auth] account=%s: a joint commit of the older session is still pending", accountID)
		}
	}
}

// place spawns the player of an authenticated connection and starts its
// session.
func (s *GameServer) place(sock *Socket, ip string, who identity) {
	account := who.account.AccountID != ""
	s.Mu.Lock()
	s.AddResolvedContent(who.resolved)
	s.MarkUnavailable(who.unavailable)
	player, ok := s.spawnPlayer(who)
	if !ok {
		s.Mu.Unlock()
		if account {
			s.releaseAcquired(who.session)
		}
		s.refuse(sock, EndUnavailable)
		s.guard.release(ip)
		return
	}
	client := &Client{
		playerID:    player.ID,
		sock:        sock,
		playerState: player,
		ip:          ip,
		limiter:     newInputLimiter(s.limits),
	}
	player.Client = client
	s.Social.Join(player)
	if account {
		s.durable.SetLease(who.session.LeaseLength)
		s.openDurable(who.session)
		s.CaptureNow(player, s.PositionOf(player) != who.session.State.Position)
	}
	mapState := s.Maps[player.MapCode]
	principal := PrincipalGuest
	if account {
		principal = PrincipalAccount
	}
	initPayload := InitPayload{
		Session:        principal,
		MaxLevel:       s.ProgressionConfig().MaxLevel,
		GridW:          mapState.GridW(),
		GridH:          mapState.GridH(),
		AoiRadius:      s.aoiRadius,
		ObjectLayers:   s.InventoryView(*s.InventoryLayers(player)),
		SkillMap:       s.buildSkillMap(),
		EntityDefaults: s.BuildEntityDefaultsSlice(),
		DeadItemIds:    s.DeadItemIDList(),
		Quests:         s.BuildQuestSnapshot(player),
		Build:          s.build,
		Adopted:        who.session.Adopted,
	}
	metadataPayload := map[string]interface{}{
		"instanceCode":   s.InstanceCode,
		"equipmentRules": s.EquipmentRules,
		"economy":        s.EconomyView(),
		"worldRevision":  s.WorldRevision(),
	}
	client.economySeen = s.EconomyKey()
	client.worldSeen = s.WorldRevision()
	s.Mu.Unlock()

	// The ObjectLayer metadata map and both JSON marshals are the costly part
	// of a join. They run outside the world lock so a burst of connections
	// cannot delay the simulation tick.
	layers, revision := s.ObjectLayers()
	metadataPayload["objectLayers"] = olMetadata(layers)
	client.olSeen = revision
	world.SendMessage(player, "init_data", initPayload)
	world.SendMessage(player, "metadata", metadataPayload)

	select {
	case s.register <- client:
		s.recordWsConnect()
	case <-time.After(5 * time.Second):
		logx.Errorf("[place] timeout waiting to register player=%s — listenForClients may be dead", player.ID)
		// The player is already in the world but no readPump will ever run to
		// unregister it. Remove it here or phaseReplication keeps serving it.
		s.detachClient(client)
		return
	}
	if account {
		go s.pushFriendCircle(player.PlayerID)
		go s.issueResume(player.PlayerID)
	}
	go client.readPump(s)
}

// spawnPlayer builds the session and the entity of an authenticated
// connection: at its saved position when the map still has that cell, else by
// the spawn rules. A guest and a new profile start with the default stack;
// any other profile loads its saved state. Caller holds s.Mu.
func (s *GameServer) spawnPlayer(who identity) (*world.Session, bool) {
	if len(s.Maps) == 0 {
		logx.Warnf("[spawnPlayer] No maps loaded — rejecting connection. Ensure INSTANCE_CODE is set and Engine gRPC is reachable.")
		return nil, false
	}
	dims := world.Vec2{X: s.DefaultPlayerWidth, Y: s.DefaultPlayerHeight}
	mapCode, cell, ok := s.savedPlacement(who.session.State.Position, dims)
	if !ok {
		if mapCode, cell, ok = s.spawnPlacement(dims); !ok {
			return nil, false
		}
	}
	mapState := s.Maps[mapCode]

	playerID := s.NewEntityID()
	player := &world.Session{
		ID:          playerID,
		PlayerID:    world.PlayerID(uuid.New().String()),
		AccountID:   who.account.AccountID,
		DisplayName: who.account.DisplayName,
		MapCode:     mapCode,
		Path:        []world.Cell{},
		TargetPos:   world.Cell{X: -1, Y: -1},
	}
	if player.AccountID != "" {
		player.PlayerID = world.PlayerID(who.session.Lease.PlayerID)
	}

	// A player is placed by nothing, so its whole stack comes from its default — read the one way
	// every entity type reads it.
	layers := s.SpawnObjectLayers("player", nil, true)
	s.SeatStacks(layers, 0)
	s.StampDefinitions(layers)
	mapState.Spawn(playerID, world.EntityData{Kind: world.EntityPlayer}, world.Transform{
		Pos:  world.Vec2{X: float64(cell.X), Y: float64(cell.Y)},
		Size: dims,
	}, world.Appearance{Layers: layers})
	mapState.AddVitals(playerID, world.Vitals{MaxLife: s.EntityBaseMaxLife})
	mapState.AddMotion(playerID, world.Motion{Direction: world.NONE, Mode: world.IDLE})
	lifeRegen := s.PlayerBaseLifeRegenMin + rand.Float64()*(s.PlayerBaseLifeRegenMax-s.PlayerBaseLifeRegenMin)
	mapState.AddMortal(playerID, &world.Mortal{LifeRegen: lifeRegen})
	mapState.AddPlayer(playerID)
	// The session goes in now: the economy and the restore read it.
	s.Sessions[playerID] = player

	if player.AccountID == "" || (who.session.Created && !who.session.Adopted) {
		s.SetProgression(playerID, content.NewEntityProgression(1, false, s.ProgressionConfig()))
		// Economy: credit the player's starting wallet (Fountain: playerSpawnCoins).
		s.FountainInitPlayer(player)
		s.ApplyResistanceStat(playerID, mapState)
	} else {
		s.RestoreState(player, who.session.State)
	}
	v := mapState.Vt(playerID)
	v.Life = v.MaxLife * s.InitialLifeFraction

	// Every join spawns frozen. The client loading overlay is its first open
	// modal; Tap-to-Start closes it and sends player_stasis false.
	sim.FreezePlayer(player, mapState)
	return player, true
}

// savedPlacement is the saved position when its map is loaded and its cell is
// walkable. Caller holds s.Mu.
func (s *GameServer) savedPlacement(pos persist.Position, dims world.Vec2) (string, world.Cell, bool) {
	ms, ok := s.Maps[pos.MapCode]
	if !ok || math.IsNaN(pos.X) || math.IsNaN(pos.Y) {
		return "", world.Cell{}, false
	}
	cell := world.Cell{X: int(math.Round(pos.X)), Y: int(math.Round(pos.Y))}
	if !ms.Pathfinder().IsWalkable(cell.X, cell.Y, dims) {
		return "", world.Cell{}, false
	}
	return pos.MapCode, cell, true
}

// spawnPlacement resolves the starting map and cell from the instance's
// PlayerSpawn config. A fixed spawn on a loaded map with a walkable cell is
// honoured verbatim; anything else falls back to a random walkable cell on a
// random map. Caller holds s.Mu.
func (s *GameServer) spawnPlacement(dims world.Vec2) (string, world.Cell, bool) {
	if !s.PlayerSpawn.Random && s.PlayerSpawn.MapCode != "" {
		if ms, ok := s.Maps[s.PlayerSpawn.MapCode]; ok {
			cell := world.Cell{X: s.PlayerSpawn.CellX, Y: s.PlayerSpawn.CellY}
			if ms.Pathfinder().IsWalkable(cell.X, cell.Y, dims) {
				return s.PlayerSpawn.MapCode, cell, true
			}
		}
	}
	mapCodes := make([]string, 0, len(s.Maps))
	for code := range s.Maps {
		mapCodes = append(mapCodes, code)
	}
	mapCode := mapCodes[rand.Intn(len(mapCodes))]
	cell, err := s.Maps[mapCode].Pathfinder().FindRandomWalkablePoint(dims)
	if err != nil {
		logx.Warnf("Could not place new player: %v", err)
		return "", world.Cell{}, false
	}
	return mapCode, cell, true
}

// detachClient removes one client from the world and frees every resource it
// holds: map presence, client registry, guard slot, socket. Safe to call from
// any disconnect path and more than once.
func (s *GameServer) detachClient(client *Client) {
	if client == nil {
		return
	}
	s.removePlayer(client)
	if client.sock != nil {
		client.sock.Close()
	}
	client.released.Do(func() { s.guard.release(client.ip) })
}

// removePlayer takes the player of a client out of the world. An account
// player's final state, a running craft refunded, goes to the persist pipeline
// first; the result tells when it is saved. Nil for a guest, or a client
// already removed.
func (s *GameServer) removePlayer(client *Client) func() bool {
	s.Mu.Lock()
	defer s.Mu.Unlock()
	player := client.playerState
	var saved func() bool
	if !client.detached && player.AccountID != "" && s.durable != nil {
		s.RefundCraft(player)
		id := string(player.PlayerID)
		applied, refused := s.FinalStates(player)
		done, mark := s.durable.CloseLinked(id, applied, refused)
		saved = func() bool {
			select {
			case <-done:
				return s.durable.Committed(id, mark)
			case <-time.After(leaveWait):
				return false
			}
		}
	}
	if player.AccountID == "" {
		s.TradeFinalStates(player)
	}
	client.detached = true
	delete(s.clients, client.playerID)
	delete(s.Sessions, client.playerID)
	s.Social.Leave(player)
	s.ForgetPlayer(player.PlayerID)
	// Sweep every map: a portal can move a player after MapCode was read.
	for _, mapState := range s.Maps {
		s.Despawn(mapState, client.playerID)
	}
	return saved
}

// leave ends a session the player closed: it leaves the world at once, and
// session_end says whether its final state is saved (EndLeft) or still on its
// way to the engine (EndLeftSaving). A player that travels gets the ticket of
// the target world, asked while its lease still holds. The target acquires the
// profile once this session released it, or once it can no longer save: the
// engine answers saving until then.
func (s *GameServer) leave(client *Client, travel string) {
	ticket := s.travelTicket(client, travel)
	end := sessionEnd{Reason: EndLeft}
	if saved := s.removePlayer(client); saved != nil && !saved() {
		end.Reason = EndLeftSaving
	}
	if ticket != "" {
		end.Travel = &travelTicket{InstanceCode: travel, Ticket: ticket}
	}
	s.recordSessionEnd(end.Reason)
	if pack, err := Pack("session_end", end); err == nil {
		client.sock.End(pack)
	}
	client.released.Do(func() { s.guard.release(client.ip) })
}

// travelTicket asks the engine for the launch ticket of a travel of an
// account session, or "" when the session has no live lease, the engine
// refuses or does not answer.
func (s *GameServer) travelTicket(client *Client, target string) string {
	player := client.playerState
	if target == "" || target == s.InstanceCode || player.AccountID == "" || s.accounts == nil {
		return ""
	}
	lease, ok := s.durable.Lease(string(player.PlayerID))
	if !ok {
		return ""
	}
	ctx, cancel := context.WithTimeout(context.Background(), travelWait)
	defer cancel()
	issued, err := s.accounts.IssueTravel(ctx, lease, s.InstanceCode, target)
	switch {
	case err != nil:
		logx.Warnf("[Session] travel of player=%s to %s: %v", player.PlayerID, target, err)
		s.recordTravel("error")
	case issued.Refusal != "":
		logx.Infof("[Session] travel of player=%s to %s refused: %s", player.PlayerID, target, issued.Refusal)
		s.recordTravel(issued.Refusal)
	default:
		s.recordTravel("issued")
		return issued.Ticket
	}
	return ""
}

// endClient ends a live session with a reason: session_end, then the normal
// teardown.
func (s *GameServer) endClient(client *Client, reason string) {
	if pack, err := Pack("session_end", sessionEnd{Reason: reason}); err == nil {
		client.sock.End(pack)
	}
	s.detachClient(client)
}

// endSessions ends the sessions the engine ended. Caller does not hold s.Mu.
func (s *GameServer) endSessions(ended []persist.Ended) {
	for _, e := range ended {
		s.Mu.Lock()
		var client *Client
		if player := s.Social.Player(world.PlayerID(e.PlayerID)); player != nil {
			client = s.clients[player.ID]
		}
		s.Mu.Unlock()
		if client == nil {
			continue
		}
		logx.Infof("[Session] player=%s ended: %s", e.PlayerID, e.Reason)
		s.recordSessionEnd(e.Reason)
		s.endClient(client, e.Reason)
	}
}

// EndAllSessions ends every session for a restart, then waits until the final
// states are committed or ctx ends. It returns how many are left unsaved.
func (s *GameServer) EndAllSessions(ctx context.Context) int {
	s.Mu.Lock()
	clients := make([]*Client, 0, len(s.clients))
	for _, client := range s.clients {
		clients = append(clients, client)
	}
	s.Mu.Unlock()
	for _, client := range clients {
		s.endClient(client, EndRestart)
	}
	if s.durable == nil {
		return 0
	}
	return s.durable.Flush(ctx)
}
