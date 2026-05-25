package game

import (
	"encoding/binary"
	"encoding/json"
	"log"
	"math"
	"math/rand"
	"net/http"
	"time"

	"github.com/google/uuid"
	"github.com/gorilla/websocket"
)

// OLMeta is the JSON shape sent to the client for each ObjectLayer.
// Matches the client's parse_object_layer_json expectations.
type OLMeta struct {
	Sha256 string          `json:"sha256"`
	Data   ObjectLayerData `json:"data"`
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
	out := make(map[string]*OLMeta, len(s.objectLayerDataCache))
	for itemID, ol := range s.objectLayerDataCache {
		out[itemID] = &OLMeta{
			Sha256: ol.Sha256,
			Data:   ol.Data,
		}
	}
	return out
}

// HandleConnections handles WebSocket connections.
func (s *GameServer) HandleConnections(w http.ResponseWriter, r *http.Request) {
	upgrader := websocket.Upgrader{
		ReadBufferSize:  1024,
		WriteBufferSize: 1024,
		CheckOrigin:     func(r *http.Request) bool { return true },
	}
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println("Upgrade error:", err)
		return
	}

	s.mu.Lock()
	log.Printf("[HandleConnections] s.mu acquired, building player")

	if len(s.maps) == 0 {
		log.Println("[HandleConnections] No maps loaded — rejecting connection. Ensure INSTANCE_CODE is set and Engine gRPC is reachable.")
		conn.Close()
		s.mu.Unlock()
		return
	}

	playerID := uuid.New().String()
	playerDims := Dimensions{Width: s.defaultPlayerWidth, Height: s.defaultPlayerHeight}

	// Pick a random starting map by code
	mapCodes := make([]string, 0, len(s.maps))
	for code := range s.maps {
		mapCodes = append(mapCodes, code)
	}
	startMapCode := mapCodes[rand.Intn(len(mapCodes))]
	startMapState := s.maps[startMapCode]
	startPosI, err := startMapState.pathfinder.findRandomWalkablePoint(playerDims)
	if err != nil {
		log.Printf("Could not place new player: %v", err)
		conn.Close()
		s.mu.Unlock()
		return
	}
	lifeRegen := s.playerBaseLifeRegenMin + rand.Float64()*(s.playerBaseLifeRegenMax-s.playerBaseLifeRegenMin)

	// Copy default object layers from entityDefaults["player"].DefaultObjectLayers.
	// Falls back to liveItemIds if no defaultObjectLayers are configured.
	var playerOLs []ObjectLayerState
	if d, ok := s.entityDefaults["player"]; ok && len(d.DefaultObjectLayers) > 0 {
		playerOLs = make([]ObjectLayerState, len(d.DefaultObjectLayers))
		copy(playerOLs, d.DefaultObjectLayers)
	} else if d, ok := s.entityDefaults["player"]; ok && len(d.LiveItemIDs) > 0 {
		for _, itemID := range d.LiveItemIDs {
			playerOLs = append(playerOLs, ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
		}
	}
	playerState := &PlayerState{
		ID:            playerID,
		MapCode:       startMapCode,
		Pos:           Point{X: float64(startPosI.X), Y: float64(startPosI.Y)},
		Dims:          playerDims,
		Path:          []PointI{},
		TargetPos:     PointI{-1, -1},
		Direction:     NONE,
		Mode:          IDLE,
		SumStatsLimit: s.sumStatsLimit,
		ObjectLayers:  playerOLs,
		MaxLife:       s.entityBaseMaxLife,
		Life:          s.entityBaseMaxLife * s.initialLifeFraction,
		LifeRegen:     lifeRegen,
	}
	client := &Client{
		conn:        conn,
		playerID:    playerID,
		send:        make(chan []byte, 256),
		lastAction:  time.Now(),
		playerState: playerState,
	}
	playerState.Client = client

	startMapState.players[playerID] = playerState

	// Economy: credit the player's starting wallet (Fountain: playerSpawnCoins).
	s.FountainInitPlayer(playerState)

	// Apply initial stats (like Resistance for MaxLife) after creation.
	s.ApplyResistanceStat(playerState, startMapState)
	playerState.Life = playerState.MaxLife * s.initialLifeFraction // Set life based on config fraction

	// InitPayload is strictly simulation/protocol. Zero presentation: no
	// palette, no camera, no devUi, no status-icon visuals, no screen
	// factors, no interpolation window, no cell-pixel sizing, no default
	// object dimensions. The C client owns its render policy and resolves
	// every visual value through /api/cyberia-client-hints using its own
	// CYBERIA_CLIENT_HINTS_CODE.
	initPayload := InitPayload{
		GridW:          startMapState.gridW,
		GridH:          startMapState.gridH,
		TickRate:       s.tickRate,
		SnapshotRate:   s.snapshotRate,
		AoiRadius:      s.aoiRadius,
		SumStatsLimit:  playerState.SumStatsLimit,
		ObjectLayers:   playerState.ObjectLayers,
		SkillMap:       s.buildSkillMap(),
		EntityDefaults: s.buildEntityDefaultsSlice(),
	}
	initMsg, _ := json.Marshal(map[string]interface{}{"type": "init_data", "payload": initPayload})
	select {
	case client.send <- initMsg:
	default:
		log.Printf("Client %s init channel full.", client.playerID)
	}

	// Send metadata message with ObjectLayer data for client-side caching.
	metaPayload := map[string]interface{}{
		"objectLayers":   s.buildOLMetadataMap(),
		"apiBaseUrl":     s.engineApiBaseUrl,
		"equipmentRules": s.equipmentRules,
	}
	metaMsg, _ := json.Marshal(map[string]interface{}{"type": "metadata", "payload": metaPayload})
	select {
	case client.send <- metaMsg:
	default:
		log.Printf("Client %s metadata channel full.", client.playerID)
	}

	s.mu.Unlock()

	// Register the client with listenForClients.
	// Use a timeout so we get a clear log if listenForClients is dead rather
	// than hanging the HTTP handler goroutine silently.
	select {
	case s.register <- client:
		s.recordWsConnect()
	case <-time.After(5 * time.Second):
		log.Printf("[HandleConnections] timeout waiting to register player=%s — listenForClients may be dead", playerID)
		conn.Close()
		return
	}
	go client.writePump(s)
	go client.readPump(s)
}

// readPump handles incoming messages from the client.
// Inbound message size cap is 8 KiB — large enough for any current
// InputCommand (handshake, chat, item activation, player action) with
// headroom, small enough to bound per-client memory.
func (c *Client) readPump(server *GameServer) {
	defer func() {
		if r := recover(); r != nil {
			log.Printf("[readPump] PANIC player=%s: %v", c.playerID, r)
		}
		log.Printf("[readPump] closing player=%s", c.playerID)
		server.recordWsDisconnect()
		server.unregister <- c
		c.conn.Close()
	}()
	c.conn.SetReadLimit(8 * 1024)
	c.conn.SetReadDeadline(time.Now().Add(60 * time.Second))
	c.conn.SetPongHandler(func(string) error {
		c.conn.SetReadDeadline(time.Now().Add(60 * time.Second))
		return nil
	})
	for {
		_, message, err := c.conn.ReadMessage()
		if err != nil {
			server.recordWsReadError()
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("[readPump] player=%s read error: %v", c.playerID, err)
			}
			break
		}
		if len(message) == 0 {
			continue
		}
		server.recordWsRead(len(message))
		if message[0] <= 0x1F {
			c.handleBinaryUplink(message, server)
		} else {
			c.handleJSONUplink(message, server)
		}
	}
}

// uplinkBuf is a little-endian binary reader for client→server uplink frames.
type uplinkBuf struct {
	data []byte
	pos  int
}

func (r *uplinkBuf) u8() (byte, bool) {
	if r.pos >= len(r.data) {
		return 0, false
	}
	v := r.data[r.pos]
	r.pos++
	return v, true
}

func (r *uplinkBuf) f32() (float32, bool) {
	if r.pos+4 > len(r.data) {
		return 0, false
	}
	bits := binary.LittleEndian.Uint32(r.data[r.pos:])
	r.pos += 4
	return math.Float32frombits(bits), true
}

func (r *uplinkBuf) str() (string, bool) {
	ln, ok := r.u8()
	if !ok {
		return "", false
	}
	end := r.pos + int(ln)
	if end > len(r.data) {
		return "", false
	}
	s := string(r.data[r.pos:end])
	r.pos = end
	return s, true
}

// handleBinaryUplink decodes a binary-framed uplink message into an
// InputCommand and enqueues it on the player's per-tick input queue.
// phaseInput drains and applies it exactly once per tick.
//
// Frame layout:
//
//	[u8 kind][payload-by-kind]
//	[u8 kind][u32 clientTick][u32 sequence][payload-by-kind]
//
// The optional clientTick+sequence suffix is read with readOptionalU32;
// older clients that omit the suffix produce zero values, which the
// simulation handles gracefully.
func (c *Client) handleBinaryUplink(message []byte, server *GameServer) {
	if len(message) < 1 {
		return
	}
	r := &uplinkBuf{data: message[1:]}
	kind := InputKind(message[0])

	switch kind {
	case InputKindHandshake:
		// Already authenticated upstream; nothing to do.
		return
	case InputKindPlayerAction:
		x, okX := r.f32()
		y, okY := r.f32()
		if !okX || !okY {
			return
		}
		cmd := InputCommand{
			Kind:       kind,
			ClientTick: readOptionalU32(r),
			Sequence:   readOptionalU32(r),
			TargetX:    float64(x),
			TargetY:    float64(y),
		}
		c.dispatchInputCommand(server, cmd)
	case InputKindItemActivation:
		itemID, okID := r.str()
		activeByte, okA := r.u8()
		if !okID || !okA {
			return
		}
		cmd := InputCommand{
			Kind:       kind,
			ClientTick: readOptionalU32(r),
			Sequence:   readOptionalU32(r),
			ItemID:     itemID,
			Active:     activeByte != 0,
		}
		c.dispatchInputCommand(server, cmd)
	case InputKindFreezeStart, InputKindFreezeEnd:
		reason, _ := r.str()
		if reason == "" {
			reason = "freeze"
		}
		cmd := InputCommand{
			Kind:       kind,
			ClientTick: readOptionalU32(r),
			Sequence:   readOptionalU32(r),
			Reason:     reason,
		}
		c.dispatchInputCommand(server, cmd)
	case InputKindChat:
		toID, okTo := r.str()
		text, okText := r.str()
		if !okTo || !okText || toID == "" || text == "" {
			return
		}
		cmd := InputCommand{
			Kind:       kind,
			ClientTick: readOptionalU32(r),
			Sequence:   readOptionalU32(r),
			ItemID:     toID, // chat target id
			ChatText:   text,
		}
		c.dispatchInputCommand(server, cmd)
	case InputKindGetItemsIDs:
		itemID, ok := r.str()
		if !ok || itemID == "" {
			return
		}
		cmd := InputCommand{
			Kind:       kind,
			ClientTick: readOptionalU32(r),
			Sequence:   readOptionalU32(r),
			ItemID:     itemID,
		}
		c.dispatchInputCommand(server, cmd)
	default:
		log.Printf("[WARN] Unknown binary uplink type: 0x%02x from player %s", message[0], c.playerID)
	}
}

// readOptionalU32 reads a u32 at the current position, returning 0 if fewer
// than 4 bytes remain. Used for the optional clientTick/sequence suffix
// that older clients don't emit.
func readOptionalU32(r *uplinkBuf) uint32 {
	if r.pos+4 > len(r.data) {
		return 0
	}
	v := uint32(r.data[r.pos]) | uint32(r.data[r.pos+1])<<8 | uint32(r.data[r.pos+2])<<16 | uint32(r.data[r.pos+3])<<24
	r.pos += 4
	return v
}

// dispatchInputCommand enqueues a typed InputCommand on the player's
// per-tick input queue. phaseInput drains and applies it exactly once.
func (c *Client) dispatchInputCommand(server *GameServer, cmd InputCommand) {
	server.mu.Lock()
	mapState, mapOK := server.maps[c.playerState.MapCode]
	if mapOK {
		if player := mapState.players[c.playerID]; player != nil {
			EnqueueInput(player, cmd)
		}
	}
	server.mu.Unlock()
}

// mustMarshal marshals v to JSON, panicking on error (should never happen for fixed structs).
func mustMarshal(v interface{}) []byte {
	b, err := json.Marshal(v)
	if err != nil {
		panic(err)
	}
	return b
}

// handleJSONUplink is a thin JSON→InputCommand adapter for text-framed
// uplink messages. Decodes the message into a typed InputCommand and
// enqueues it via dispatchInputCommand. phaseInput is the authoritative
// consumer; no synchronous dispatch occurs here.
func (c *Client) handleJSONUplink(message []byte, server *GameServer) {
	var msg map[string]interface{}
	if err := json.Unmarshal(message, &msg); err != nil {
		log.Printf("Error unmarshaling JSON uplink: %v", err)
		return
	}
	typeStr, _ := msg["type"].(string)
	payload, _ := msg["payload"].(map[string]interface{})
	cmd := jsonUplinkToInputCommand(typeStr, payload)
	if cmd.Kind != InputKindUnknown {
		c.dispatchInputCommand(server, cmd)
	}
}

// jsonUplinkToInputCommand translates a JSON uplink envelope into a
// typed InputCommand. Recognises the type strings the server accepts
// plus the dialogue_* aliases for FREEZE.
func jsonUplinkToInputCommand(typeStr string, payload map[string]interface{}) InputCommand {
	cmd := InputCommand{Kind: InputKindUnknown}
	switch typeStr {
	case "player_action":
		x, _ := payload["targetX"].(float64)
		y, _ := payload["targetY"].(float64)
		cmd.Kind = InputKindPlayerAction
		cmd.TargetX = x
		cmd.TargetY = y
	case "item_activation":
		id, _ := payload["itemId"].(string)
		active, _ := payload["active"].(bool)
		cmd.Kind = InputKindItemActivation
		cmd.ItemID = id
		cmd.Active = active
	case "freeze_start", "dialogue_start":
		reason, _ := payload["reason"].(string)
		if reason == "" {
			reason = "dialogue"
		}
		cmd.Kind = InputKindFreezeStart
		cmd.Reason = reason
	case "freeze_end", "dialogue_end":
		reason, _ := payload["reason"].(string)
		if reason == "" {
			reason = "dialogue"
		}
		cmd.Kind = InputKindFreezeEnd
		cmd.Reason = reason
	case "chat":
		to, _ := payload["to"].(string)
		text, _ := payload["text"].(string)
		cmd.Kind = InputKindChat
		cmd.ItemID = to
		cmd.ChatText = text
	case "get_items_ids":
		id, _ := payload["itemId"].(string)
		cmd.Kind = InputKindGetItemsIDs
		cmd.ItemID = id
	}
	return cmd
}

// writePump writes messages to the WebSocket connection.
func (c *Client) writePump(server *GameServer) {
	ticker := time.NewTicker(30 * time.Second)
	defer func() {
		if r := recover(); r != nil {
			log.Printf("[writePump] PANIC player=%s: %v", c.playerID, r)
		}
		ticker.Stop()
		c.conn.Close()
		log.Printf("[writePump] closed player=%s", c.playerID)
	}()
	for {
		select {
		case message, ok := <-c.send:
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if !ok {
				c.conn.WriteMessage(websocket.CloseMessage, []byte{})
				return
			}
			// Frame-type heuristic: binary AOI / typed input commands have
			// a single-byte type prefix (< 0x20); JSON payloads start with
			// '{' or '['.
			msgType := websocket.TextMessage
			if len(message) > 0 && message[0] != '{' && message[0] != '[' {
				msgType = websocket.BinaryMessage
			}
			w, err := c.conn.NextWriter(msgType)
			if err != nil {
				server.recordWsWriteError()
				log.Printf("[writePump] NextWriter failed player=%s: %v", c.playerID, err)
				return
			}
			if _, err := w.Write(message); err != nil {
				server.recordWsWriteError()
				log.Printf("[writePump] write failed player=%s: %v", c.playerID, err)
				return
			}
			if err := w.Close(); err != nil {
				server.recordWsWriteError()
				log.Printf("[writePump] flush failed player=%s: %v (size=%d)", c.playerID, err, len(message))
				return
			}
			server.recordWsWrite(len(message))
		case <-ticker.C:
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if err := c.conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				server.recordWsWriteError()
				log.Printf("[writePump] ping failed player=%s: %v", c.playerID, err)
				return
			}
		}
	}
}
