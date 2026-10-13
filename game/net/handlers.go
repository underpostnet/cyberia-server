package net

import (
	"encoding/json"
	"math/rand"
	"net/http"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/sim"
	"cyberia-server/game/world"
	"cyberia-server/logx"

	"github.com/google/uuid"
)

// OLMeta is the JSON shape sent to the client for each ObjectLayer: the cid of the definition
// bound to the label, and its content. Matches populate_object_layer_from_json.
type OLMeta struct {
	Cid  string                  `json:"cid"`
	Data content.ObjectLayerData `json:"data"`
}

// buildSkillMap returns a compact { triggerItemId → [SkillMapEntry] } map
// derived from the server's skillConfig — sent to clients in init_data.
func (s *GameServer) buildSkillMap() map[string][]SkillMapEntry {
	out := make(map[string][]SkillMapEntry, len(s.SkillConfig))
	for triggerID, defs := range s.SkillConfig {
		entries := make([]SkillMapEntry, 0, len(defs))
		for _, def := range defs {
			entries = append(entries, SkillMapEntry{
				LogicEventID:         def.LogicEventID,
				Name:                 def.Name,
				Description:          def.Description,
				SummonedEntityItemID: def.SummonedEntityItemID,
				Held:                 sim.IsHeldSkill(def.LogicEventID),
			})
		}
		out[triggerID] = entries
	}
	return out
}

// buildOLMetadataMap creates the map[itemID] → OLMeta for the metadata message.
// Must be called while s.olMu is held (read).
func (s *GameServer) buildOLMetadataMap() map[string]*OLMeta {
	s.OlMu.RLock()
	defer s.OlMu.RUnlock()
	// Pre-allocated capacity per type hint; typical cache has ~400 items.
	out := make(map[string]*OLMeta, len(s.ObjectLayerDataCache))
	for itemID, ol := range s.ObjectLayerDataCache {
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

	conn, err := Upgrader.Upgrade(w, r, nil)
	if err != nil {
		logx.Debugf("Upgrade failed: %v", err)
		releaseSlot()
		return
	}

	s.Mu.Lock()
	logx.Debugf("[HandleConnections] s.mu acquired, building player")

	if len(s.Maps) == 0 {
		logx.Warnf("[HandleConnections] No maps loaded — rejecting connection. Ensure INSTANCE_CODE is set and Engine gRPC is reachable.")
		conn.Close()
		s.Mu.Unlock()
		releaseSlot()
		return
	}

	playerID := s.NewEntityID()
	playerDims := world.Vec2{X: s.DefaultPlayerWidth, Y: s.DefaultPlayerHeight}

	// Resolve the starting map + cell from the instance's PlayerSpawn config. A
	// fixed spawn (Random=false) on a loaded map with a walkable cell is honoured
	// verbatim; anything else falls back to a random walkable cell on a random map.
	startMapCode := ""
	fixedPos := world.Cell{}
	hasFixedPos := false
	if !s.PlayerSpawn.Random && s.PlayerSpawn.MapCode != "" {
		if ms, ok := s.Maps[s.PlayerSpawn.MapCode]; ok {
			cell := world.Cell{X: s.PlayerSpawn.CellX, Y: s.PlayerSpawn.CellY}
			if ms.Pathfinder().IsWalkable(cell.X, cell.Y, playerDims) {
				startMapCode = s.PlayerSpawn.MapCode
				fixedPos = cell
				hasFixedPos = true
			}
		}
	}
	if startMapCode == "" {
		mapCodes := make([]string, 0, len(s.Maps))
		for code := range s.Maps {
			mapCodes = append(mapCodes, code)
		}
		startMapCode = mapCodes[rand.Intn(len(mapCodes))]
	}
	startMapState := s.Maps[startMapCode]

	startPosI := fixedPos
	if !hasFixedPos {
		p, err := startMapState.Pathfinder().FindRandomWalkablePoint(playerDims)
		if err != nil {
			logx.Warnf("Could not place new player: %v", err)
			conn.Close()
			s.Mu.Unlock()
			releaseSlot()
			return
		}
		startPosI = p
	}
	lifeRegen := s.PlayerBaseLifeRegenMin + rand.Float64()*(s.PlayerBaseLifeRegenMax-s.PlayerBaseLifeRegenMin)

	// A player is placed by nothing, so its whole stack comes from its default — read the one way
	// every entity type reads it.
	playerOLs := s.SpawnObjectLayers("player", nil, true)
	playerState := &world.Session{
		ID:        playerID,
		EngineID:  uuid.New().String(),
		MapCode:   startMapCode,
		Path:      []world.Cell{},
		TargetPos: world.Cell{X: -1, Y: -1},
	}
	client := &Client{
		playerID: playerID,
		sock: NewSocket(conn, Metrics{
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

	s.Sessions[playerID] = playerState
	startMapState.Spawn(playerID, world.EntityData{Kind: world.EntityPlayer}, world.Transform{
		Pos:  world.Vec2{X: float64(startPosI.X), Y: float64(startPosI.Y)},
		Size: playerDims,
	}, world.Appearance{Layers: playerOLs})
	startMapState.AddVitals(playerID, world.Vitals{MaxLife: s.EntityBaseMaxLife})
	startMapState.AddMotion(playerID, world.Motion{Direction: world.NONE, Mode: world.IDLE})
	startMapState.AddMortal(playerID, &world.Mortal{LifeRegen: lifeRegen})
	s.SetProgression(playerID, content.NewEntityProgression(1, false, s.ProgressionConfig()))
	startMapState.AddPlayer(playerID)

	// Economy: credit the player's starting wallet (Fountain: playerSpawnCoins).
	s.FountainInitPlayer(playerState)

	// Apply initial stats (like Resistance for MaxLife) after creation.
	s.ApplyResistanceStat(playerState.ID, startMapState)
	v := startMapState.Vt(playerID)
	v.Life = v.MaxLife * s.InitialLifeFraction // Set life based on config fraction

	// Every join spawns frozen. The client loading overlay is its first open
	// modal; Tap-to-Start closes it and sends player_stasis false.
	sim.FreezePlayer(playerState, startMapState)

	// InitPayload is strictly simulation/protocol. Zero presentation: no
	// palette, no camera, no devUi, no status-icon visuals, no screen
	// factors, no interpolation window, no cell-pixel sizing, no default
	// object dimensions. The C client owns its render policy and resolves
	// every visual value through /api/v1/cyberia-client-hints using its own
	// CYBERIA_CLIENT_HINTS_CODE.
	initPayload := InitPayload{
		EngineID:       playerState.EngineID,
		GridW:          startMapState.GridW(),
		GridH:          startMapState.GridH(),
		AoiRadius:      s.aoiRadius,
		ObjectLayers:   s.VisibleInventory(startMapState.Ap(playerID).Layers),
		SkillMap:       s.buildSkillMap(),
		EntityDefaults: s.BuildEntityDefaultsSlice(),
		DeadItemIds:    s.DeadItemIDList(),
		Quests:         s.BuildQuestSnapshot(playerState),
	}
	metadataPayload := map[string]interface{}{
		"instanceCode":   s.InstanceCode,
		"equipmentRules": s.EquipmentRules,
	}

	s.Mu.Unlock()

	// The ObjectLayer metadata map and both JSON marshals are the costly part
	// of a join. They run outside the world lock so a burst of connections
	// cannot delay the simulation tick.
	metadataPayload["objectLayers"] = s.buildOLMetadataMap()
	world.SendMessage(playerState, "init_data", initPayload)
	world.SendMessage(playerState, "metadata", metadataPayload)

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
	s.Mu.Lock()
	client.detached = true
	delete(s.clients, client.playerID)
	delete(s.Sessions, client.playerID)
	// Sweep every map: a portal can move a player after MapCode was read.
	for _, mapState := range s.Maps {
		s.Despawn(mapState, client.playerID)
	}
	s.Mu.Unlock()

	if client.sock != nil {
		client.sock.Close()
	}
	client.released.Do(func() { s.guard.release(client.ip) })
}

// Send packs one message and queues it on the socket.
func (c *Client) Send(msgType string, payload any) {
	if c.sock == nil {
		return
	}
	pack, err := Pack(msgType, payload)
	if err != nil {
		logx.Errorf("[sendMessage] pack %q failed: %v", msgType, err)
		return
	}
	if !c.sock.Send(pack) {
		logx.Debugf("Client %s queue full — dropped %q.", c.playerID, msgType)
	}
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
var inputKinds = map[string]world.InputKind{
	"player_action": world.InputKindPlayerAction,
	"item_active":   world.InputKindItemActivation,
	"player_stasis": world.InputKindPlayerStasis,
	"chat":          world.InputKindChat,
	"talk_done":     world.InputKindTalkDone,
	"quest_abandon": world.InputKindQuestAbandon,
	"quest_accept":  world.InputKindQuestAccept,
	"shop_buy":      world.InputKindShopBuy,
	"craft_item":    world.InputKindCraftItem,
	"craft_cancel":  world.InputKindCraftCancel,
	"item_ops":      world.InputKindItemOps,
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

	ToID world.EntityID `json:"toId"` // chat
	Text string         `json:"text"`

	EntityID   world.EntityID `json:"entityId"`   // talk_done, quest_accept, shop_buy, item_ops
	DialogCode string         `json:"dialogCode"` // talk_done
	QuestCode  string         `json:"questCode"`  // quest_*

	Quantity    int `json:"quantity"`    // shop_buy
	RecipeIndex int `json:"recipeIndex"` // craft_item

	Ops []world.ItemOp `json:"ops"` // item_ops
}

// eventsPayload is the payload of "events", the one uplink input message.
type eventsPayload struct {
	Events []Message `json:"events"`
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

	msg, err := Unpack(pack)
	if err == nil && msg.Type == "handshake" {
		return // already authenticated upstream; nothing to do
	}
	var batch eventsPayload
	if err != nil || msg.Type != "events" || json.Unmarshal(msg.Payload, &batch) != nil {
		c.evict(server, "bad message")
		return
	}
	if len(batch.Events) > world.MaxInputQueue {
		c.evict(server, "event batch too large")
		return
	}

	cmds := make([]world.InputCommand, 0, len(batch.Events))
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
			cmd = world.InputCommand{Kind: world.InputKindUnknown}
		}
		cmd.Sequence, cmd.Frame, cmd.Timestamp = p.Seq, p.Frame, p.Timestamp
		cmds = append(cmds, cmd)
	}
	c.dispatchInputs(server, cmds)
}

// parseInput validates one event payload and builds its command. False means
// the type word is unknown or the payload fails validation.
func parseInput(msgType string, p *inputPayload) (world.InputCommand, bool) {
	kind, known := inputKinds[msgType]
	if !known {
		return world.InputCommand{}, false
	}
	cmd := world.InputCommand{Kind: kind}
	switch kind {
	case world.InputKindPlayerAction:
		// A tap target reaches the pathfinder directly. Reject anything that
		// is not a finite, plausible coordinate before it costs tick time.
		if !sim.ValidTapTarget(p.X, p.Y) {
			return world.InputCommand{}, false
		}
		cmd.TargetX = p.X
		cmd.TargetY = p.Y
	case world.InputKindItemActivation:
		if p.ItemID == "" || !sim.ValidIdentifier(p.ItemID) {
			return world.InputCommand{}, false
		}
		cmd.ItemID = p.ItemID
		cmd.Active = p.Active
	case world.InputKindPlayerStasis:
		cmd.Active = p.Stasis
	case world.InputKindChat:
		if p.ToID == 0 || p.Text == "" {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.ToID
		cmd.ChatText = sim.TruncateRunes(p.Text, sim.MaxChatRunes)
	case world.InputKindTalkDone:
		if p.EntityID == 0 || !sim.ValidIdentifier(p.DialogCode) {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.DialogCode = p.DialogCode
	case world.InputKindQuestAbandon:
		if p.QuestCode == "" || !sim.ValidIdentifier(p.QuestCode) {
			return world.InputCommand{}, false
		}
		cmd.ItemID = p.QuestCode
	case world.InputKindQuestAccept:
		if p.EntityID == 0 || p.QuestCode == "" || !sim.ValidIdentifier(p.QuestCode) {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.QuestCode
	case world.InputKindShopBuy:
		if p.EntityID == 0 || p.ItemID == "" || !sim.ValidIdentifier(p.ItemID) {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.ItemID
		cmd.Quantity = sim.ClampQuantity(p.Quantity)
	case world.InputKindCraftItem:
		if p.EntityID == 0 || p.RecipeIndex < 0 {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.RecipeIndex = p.RecipeIndex
	case world.InputKindItemOps:
		// Trust boundary: the list replays under s.mu, so its length is capped.
		if p.EntityID == 0 || len(p.Ops) == 0 || len(p.Ops) > content.StorageMaxSlots {
			return world.InputCommand{}, false
		}
		for _, op := range p.Ops {
			if op.ItemID == "" || !sim.ValidIdentifier(op.ItemID) || op.Qty <= 0 {
				return world.InputCommand{}, false
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
func (c *Client) dispatchInputs(server *GameServer, cmds []world.InputCommand) {
	fits := true
	server.Mu.Lock()
	if player := server.Sessions[c.playerID]; player != nil {
		fits = world.EnqueueInputs(player, cmds)
	}
	server.Mu.Unlock()
	if !fits {
		c.evict(server, "input queue full")
	}
}
