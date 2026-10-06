package game

import (
	"encoding/json"
	"math/rand"
	"net/http"
	"time"

	"cyberia-server/logx"
	"cyberia-server/serial"
	"cyberia-server/socket"

	"github.com/google/uuid"
)

// OLMeta is the JSON shape sent to the client for each ObjectLayer: the cid of the definition
// bound to the label, and its content. Matches populate_object_layer_from_json.
type OLMeta struct {
	Cid  string          `json:"cid"`
	Data ObjectLayerData `json:"data"`
}

// buildSkillMap returns a compact { triggerItemId → [SkillMapEntry] } map
// derived from the server's skillConfig — sent to clients in init_data.
func (s *GameServer) buildSkillMap() map[string][]SkillMapEntry {
	out := make(map[string][]SkillMapEntry, len(s.skillConfig))
	for triggerID, defs := range s.skillConfig {
		entries := make([]SkillMapEntry, 0, len(defs))
		for _, def := range defs {
			entries = append(entries, SkillMapEntry{
				LogicEventID:         def.LogicEventID,
				Name:                 def.Name,
				Description:          def.Description,
				SummonedEntityItemID: def.SummonedEntityItemID,
			})
		}
		out[triggerID] = entries
	}
	return out
}

// buildOLMetadataMap creates the map[itemID] → OLMeta for the metadata message.
// Must be called while s.olMu is held (read).
func (s *GameServer) buildOLMetadataMap() map[string]*OLMeta {
	s.olMu.RLock()
	defer s.olMu.RUnlock()
	// Pre-allocated capacity per type hint; typical cache has ~400 items.
	out := make(map[string]*OLMeta, len(s.objectLayerDataCache))
	for itemID, ol := range s.objectLayerDataCache {
		out[itemID] = &OLMeta{Cid: ol.Cid, Data: ol.Data}
	}
	return out
}

// HandleConnections handles WebSocket connections.
func (s *GameServer) HandleConnections(w http.ResponseWriter, r *http.Request) {
	// Admission runs before the upgrade so a refused client costs one HTTP
	// response, not a WebSocket session.
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
	// Admitted from here. Until a Client owns the slot, this closure frees it;
	// after that the Client's `released` Once does, on whichever path ends it.
	slotOwned := true
	releaseSlot := func() {
		if slotOwned {
			slotOwned = false
			s.guard.release(ip)
		}
	}

	conn, err := socket.Upgrader.Upgrade(w, r, nil)
	if err != nil {
		logx.Debugf("Upgrade failed: %v", err)
		releaseSlot()
		return
	}

	s.mu.Lock()
	logx.Debugf("[HandleConnections] s.mu acquired, building player")

	if len(s.maps) == 0 {
		logx.Warnf("[HandleConnections] No maps loaded — rejecting connection. Ensure INSTANCE_CODE is set and Engine gRPC is reachable.")
		conn.Close()
		s.mu.Unlock()
		releaseSlot()
		return
	}

	playerID := uuid.New().String()
	playerDims := Dimensions{Width: s.defaultPlayerWidth, Height: s.defaultPlayerHeight}

	// Resolve the starting map + cell from the instance's PlayerSpawn config. A
	// fixed spawn (Random=false) on a loaded map with a walkable cell is honoured
	// verbatim; anything else falls back to a random walkable cell on a random map.
	startMapCode := ""
	fixedPos := PointI{}
	hasFixedPos := false
	if !s.playerSpawn.Random && s.playerSpawn.MapCode != "" {
		if ms, ok := s.maps[s.playerSpawn.MapCode]; ok {
			cell := PointI{X: s.playerSpawn.CellX, Y: s.playerSpawn.CellY}
			if ms.pathfinder.isWalkable(cell.X, cell.Y, playerDims) {
				startMapCode = s.playerSpawn.MapCode
				fixedPos = cell
				hasFixedPos = true
			}
		}
	}
	if startMapCode == "" {
		mapCodes := make([]string, 0, len(s.maps))
		for code := range s.maps {
			mapCodes = append(mapCodes, code)
		}
		startMapCode = mapCodes[rand.Intn(len(mapCodes))]
	}
	startMapState := s.maps[startMapCode]

	startPosI := fixedPos
	if !hasFixedPos {
		p, err := startMapState.pathfinder.findRandomWalkablePoint(playerDims)
		if err != nil {
			logx.Warnf("Could not place new player: %v", err)
			conn.Close()
			s.mu.Unlock()
			releaseSlot()
			return
		}
		startPosI = p
	}
	lifeRegen := s.playerBaseLifeRegenMin + rand.Float64()*(s.playerBaseLifeRegenMax-s.playerBaseLifeRegenMin)

	// A player is placed by nothing, so its whole stack comes from its default — read the one way
	// every entity type reads it.
	playerOLs := s.spawnObjectLayers("player", nil, true)
	playerState := &PlayerState{
		EntityBase: EntityBase{
			ID:           playerID,
			Pos:          Point{X: float64(startPosI.X), Y: float64(startPosI.Y)},
			Dims:         playerDims,
			ObjectLayers: playerOLs,
		},
		Mortal: Mortal{
			Progression: NewEntityProgression(1, false, s.progressionConfig()),
			MaxLife:     s.entityBaseMaxLife,
			Life:        s.entityBaseMaxLife * s.initialLifeFraction,
		},
		MapCode:   startMapCode,
		Path:      []PointI{},
		TargetPos: PointI{-1, -1},
		Direction: NONE,
		Mode:      IDLE,
		LifeRegen: lifeRegen,
	}
	client := &Client{
		playerID: playerID,
		sock: socket.New(conn, socket.Metrics{
			Read:       s.recordWsRead,
			Write:      s.recordWsWrite,
			ReadError:  s.recordWsReadError,
			WriteError: s.recordWsWriteError,
		}),
		playerState: playerState,
		ip:          ip,
		limiter:     newInputLimiter(s.limits),
	}
	slotOwned = false // the Client owns the guard slot from here
	playerState.Client = client

	startMapState.players[playerID] = playerState

	// Economy: credit the player's starting wallet (Fountain: playerSpawnCoins).
	s.FountainInitPlayer(playerState)

	// Apply initial stats (like Resistance for MaxLife) after creation.
	s.ApplyResistanceStat(playerState, startMapState)
	playerState.Life = playerState.MaxLife * s.initialLifeFraction // Set life based on config fraction

	// Every join spawns frozen. The client loading overlay is its first open
	// modal; Tap-to-Start closes it and sends player_stasis false.
	FreezePlayer(playerState)

	// InitPayload is strictly simulation/protocol. Zero presentation: no
	// palette, no camera, no devUi, no status-icon visuals, no screen
	// factors, no interpolation window, no cell-pixel sizing, no default
	// object dimensions. The C client owns its render policy and resolves
	// every visual value through /api/v1/cyberia-client-hints using its own
	// CYBERIA_CLIENT_HINTS_CODE.
	initPayload := InitPayload{
		GridW:          startMapState.gridW,
		GridH:          startMapState.gridH,
		SnapshotRate:   s.snapshotRate,
		AoiRadius:      s.aoiRadius,
		ObjectLayers:   s.visibleInventory(playerState.ObjectLayers),
		SkillMap:       s.buildSkillMap(),
		EntityDefaults: s.buildEntityDefaultsSlice(),
		DeadItemIds:    s.deadItemIDList(),
		Quests:         s.buildQuestSnapshot(playerState),
	}
	metadataPayload := map[string]interface{}{
		"instanceCode":   s.instanceCode,
		"equipmentRules": s.equipmentRules,
	}

	s.mu.Unlock()

	// The ObjectLayer metadata map and both JSON marshals are the costly part
	// of a join. They run outside the world lock so a burst of connections
	// cannot delay the simulation tick.
	metadataPayload["objectLayers"] = s.buildOLMetadataMap()
	sendMessage(playerState, "init_data", initPayload)
	sendMessage(playerState, "metadata", metadataPayload)

	// Register the client with listenForClients.
	// Use a timeout so we get a clear log if listenForClients is dead rather
	// than hanging the HTTP handler goroutine silently.
	select {
	case s.register <- client:
		s.recordWsConnect()
	case <-time.After(5 * time.Second):
		logx.Errorf("[HandleConnections] timeout waiting to register player=%s — listenForClients may be dead", playerID)
		// The player is already in the world but no readPump will ever run to
		// unregister it. Remove it here or phaseReplication keeps serving it.
		s.detachClient(client)
		return
	}
	go client.readPump(s)
}

// detachClient removes one client from the world and frees every resource it
// holds: map presence, client registry, guard slot, socket. Safe to call from
// any disconnect path and safe to call more than once.
func (s *GameServer) detachClient(client *Client) {
	if client == nil {
		return
	}
	s.mu.Lock()
	client.detached = true
	delete(s.clients, client.playerID)
	// Sweep every map: a portal can move a player after MapCode was read.
	for _, mapState := range s.maps {
		delete(mapState.players, client.playerID)
	}
	s.mu.Unlock()

	if client.sock != nil {
		client.sock.Close()
	}
	client.released.Do(func() { s.guard.release(client.ip) })
}

// sendMessage packs a message and queues it for the player. The send never
// blocks: a full queue drops the message.
func sendMessage(player *PlayerState, msgType string, payload any) {
	if player == nil || player.Client == nil || player.Client.sock == nil {
		return
	}
	pack, err := serial.Pack(msgType, payload)
	if err != nil {
		logx.Errorf("[sendMessage] pack %q failed: %v", msgType, err)
		return
	}
	if !player.Client.sock.Send(pack) {
		logx.Debugf("Client %s queue full — dropped %q.", player.ID, msgType)
	}
}

// sendAck answers a request with the {ok, reason} pair every ack carries, plus
// the caller's own fields. An empty reason means success.
func sendAck(player *PlayerState, msgType, reason string, fields map[string]interface{}) {
	fields["ok"] = reason == ""
	fields["reason"] = reason
	sendMessage(player, msgType, fields)
}

// readPump runs the client read loop until the connection fails.
func (c *Client) readPump(server *GameServer) {
	defer func() {
		if r := recover(); r != nil {
			logx.Errorf("[readPump] PANIC player=%s: %v", c.playerID, r)
		}
		logx.Debugf("[readPump] closing player=%s", c.playerID)
		server.recordWsDisconnect()
		// Never block here. A full unregister queue must not pin this
		// goroutine, so fall back to tearing the client down directly.
		select {
		case server.unregister <- c:
		default:
			server.detachClient(c)
		}
	}()
	c.sock.Receive(func(pack []byte) { c.receiveMessage(pack, server) })
}

// inputKinds maps the inner type word of a client event to the internal input
// kind. The kind enum stays internal; only this table knows the wire words.
var inputKinds = map[string]InputKind{
	"player_action":   InputKindPlayerAction,
	"item_active":     InputKindItemActivation,
	"player_stasis":   InputKindPlayerStasis,
	"chat":            InputKindChat,
	"dialog_start":    InputKindDlgStart,
	"dialog_complete": InputKindDlgComplete,
	"dialog_cancel":   InputKindDlgCancel,
	"quest_abandon":   InputKindQuestAbandon,
	"quest_accept":    InputKindQuestAccept,
	"shop_buy":        InputKindShopBuy,
	"craft_item":      InputKindCraftItem,
	"craft_cancel":    InputKindCraftCancel,
	"storage_open":    InputKindStorageOpen,
	"item_ops":        InputKindItemOps,
}

// inputPayload holds every client event payload field. Each event type fills
// the subset it needs; the rest stay zero.
type inputPayload struct {
	Seq       uint32  `json:"seq"`
	Frame     uint32  `json:"frame"`
	Timestamp float64 `json:"timestamp"`

	X float64 `json:"x"` // player_action
	Y float64 `json:"y"`

	ItemID string `json:"itemId"` // item_active, shop_buy
	Active bool   `json:"active"` // item_active

	Stasis bool `json:"stasis"` // player_stasis

	ToID string `json:"toId"` // chat
	Text string `json:"text"`

	EntityID   string `json:"entityId"`   // dialog_*, quest_accept, shop_buy
	DialogCode string `json:"dialogCode"` // dialog_complete
	QuestCode  string `json:"questCode"`  // quest_*

	Quantity    int `json:"quantity"`    // shop_buy
	RecipeIndex int `json:"recipeIndex"` // craft_item

	Ops []itemOp `json:"ops"` // item_ops
}

// eventsPayload is the payload of "events", the one uplink input message.
type eventsPayload struct {
	Events []serial.Message `json:"events"`
}

// receiveMessage is the single client → server dispatch point. A message is a
// handshake or one batch of client events. The batch becomes one InputCommand
// per event and is enqueued whole; phaseInput applies each command once.
//
// A protocol violation evicts: a message that is not a handshake or a batch,
// a batch over maxInputQueue, or an event without a seq. An event that fails
// validation costs a strike and is still enqueued as InputKindUnknown, so its
// seq is consumed.
func (c *Client) receiveMessage(pack []byte, server *GameServer) {
	// Rate limit first: an over-budget frame costs no parsing work.
	if allowed, evict := c.limiter.allow(); !allowed {
		server.recordWsRateLimited()
		if evict {
			c.evict(server, "input rate limit")
		}
		return
	}

	msg, err := serial.Unpack(pack)
	if err == nil && msg.Type == "handshake" {
		return // already authenticated upstream; nothing to do
	}
	var batch eventsPayload
	if err != nil || msg.Type != "events" || json.Unmarshal(msg.Payload, &batch) != nil {
		c.evict(server, "bad message")
		return
	}
	if len(batch.Events) > maxInputQueue {
		c.evict(server, "event batch too large")
		return
	}

	cmds := make([]InputCommand, 0, len(batch.Events))
	for _, ev := range batch.Events {
		var p inputPayload
		if err := json.Unmarshal(ev.Payload, &p); err != nil || p.Seq == 0 {
			c.evict(server, "event without seq")
			return
		}
		cmd, ok := parseInput(ev.Type, &p)
		if !ok {
			logx.Debugf("Invalid %q event from player %s", ev.Type, c.playerID)
			if c.limiter.strike() {
				c.evict(server, "repeated protocol violations")
				return
			}
			cmd = InputCommand{Kind: InputKindUnknown}
		}
		cmd.Sequence, cmd.Frame, cmd.Timestamp = p.Seq, p.Frame, p.Timestamp
		cmds = append(cmds, cmd)
	}
	c.dispatchInputs(server, cmds)
}

// parseInput validates one event payload and builds its command. False means
// the type word is unknown or the payload fails validation.
func parseInput(msgType string, p *inputPayload) (InputCommand, bool) {
	kind, known := inputKinds[msgType]
	if !known {
		return InputCommand{}, false
	}
	cmd := InputCommand{Kind: kind}
	switch kind {
	case InputKindPlayerAction:
		// A tap target reaches the pathfinder directly. Reject anything that
		// is not a finite, plausible coordinate before it costs tick time.
		if !validTapTarget(p.X, p.Y) {
			return InputCommand{}, false
		}
		cmd.TargetX = p.X
		cmd.TargetY = p.Y
	case InputKindItemActivation:
		if p.ItemID == "" || !validIdentifier(p.ItemID) {
			return InputCommand{}, false
		}
		cmd.ItemID = p.ItemID
		cmd.Active = p.Active
	case InputKindPlayerStasis:
		cmd.Active = p.Stasis
	case InputKindChat:
		if p.ToID == "" || p.Text == "" || !validIdentifier(p.ToID) {
			return InputCommand{}, false
		}
		cmd.ItemID = p.ToID // chat target id
		cmd.ChatText = truncateRunes(p.Text, maxChatRunes)
	case InputKindDlgStart, InputKindDlgCancel:
		if p.EntityID == "" || !validIdentifier(p.EntityID) || !validIdentifier(p.ItemID) {
			return InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.ItemID
	case InputKindDlgComplete:
		if p.EntityID == "" || !validIdentifier(p.EntityID) || !validIdentifier(p.DialogCode) {
			return InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.ItemID
		cmd.DialogCode = p.DialogCode
	case InputKindQuestAbandon:
		if p.QuestCode == "" || !validIdentifier(p.QuestCode) {
			return InputCommand{}, false
		}
		cmd.ItemID = p.QuestCode
	case InputKindQuestAccept:
		if p.EntityID == "" || p.QuestCode == "" ||
			!validIdentifier(p.EntityID) || !validIdentifier(p.QuestCode) {
			return InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.QuestCode
	case InputKindShopBuy:
		if p.EntityID == "" || p.ItemID == "" ||
			!validIdentifier(p.EntityID) || !validIdentifier(p.ItemID) {
			return InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.ItemID
		cmd.Quantity = clampQuantity(p.Quantity)
	case InputKindCraftItem:
		if p.EntityID == "" || !validIdentifier(p.EntityID) || p.RecipeIndex < 0 {
			return InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.RecipeIndex = p.RecipeIndex
	case InputKindStorageOpen:
		if p.EntityID == "" || !validIdentifier(p.EntityID) {
			return InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
	case InputKindItemOps:
		// Trust boundary: the list replays under s.mu, so its length is capped.
		if p.EntityID == "" || !validIdentifier(p.EntityID) ||
			len(p.Ops) == 0 || len(p.Ops) > storageMaxSlots {
			return InputCommand{}, false
		}
		for _, op := range p.Ops {
			if op.ItemID == "" || !validIdentifier(op.ItemID) || op.Qty <= 0 {
				return InputCommand{}, false
			}
		}
		cmd.EntityID = p.EntityID
		cmd.Ops = p.Ops
	}
	return cmd, true
}

// evict closes an abusive connection. The read loop ends on the closed socket
// and readPump runs the normal teardown.
func (c *Client) evict(server *GameServer, reason string) {
	server.recordWsEvicted()
	logx.Warnf("[evict] player=%s ip=%s: %s", c.playerID, c.ip, reason)
	c.sock.Close()
}

// dispatchInputs enqueues one batch on the player's per-tick input queue.
// phaseInput drains and applies each command exactly once. A batch that does
// not fit evicts the client.
func (c *Client) dispatchInputs(server *GameServer, cmds []InputCommand) {
	fits := true
	server.mu.Lock()
	if mapState, ok := server.maps[c.playerState.MapCode]; ok {
		if player := mapState.players[c.playerID]; player != nil {
			fits = EnqueueInputs(player, cmds)
		}
	}
	server.mu.Unlock()
	if !fits {
		c.evict(server, "input queue full")
	}
}
