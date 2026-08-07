package game

import (
	"encoding/json"
	"log"
	"math/rand"
	"net/http"
	"time"

	"cyberia-server/logx"
	"cyberia-server/serial"
	"cyberia-server/socket"

	"github.com/google/uuid"
)

// OLMeta is the JSON shape sent to the client for each ObjectLayer.
// Matches the client's populate_object_layer_from_json expectations.
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
	// Pre-allocated capacity per type hint; typical cache has ~400 items.
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
	conn, err := socket.Upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println("Upgrade error:", err)
		return
	}

	s.mu.Lock()
	logx.Debugf("[HandleConnections] s.mu acquired, building player")

	if len(s.maps) == 0 {
		log.Println("[HandleConnections] No maps loaded — rejecting connection. Ensure INSTANCE_CODE is set and Engine gRPC is reachable.")
		conn.Close()
		s.mu.Unlock()
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
			log.Printf("Could not place new player: %v", err)
			conn.Close()
			s.mu.Unlock()
			return
		}
		startPosI = p
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
		EntityBase: EntityBase{
			ID:           playerID,
			Pos:          Point{X: float64(startPosI.X), Y: float64(startPosI.Y)},
			Dims:         playerDims,
			ObjectLayers: playerOLs,
		},
		Mortal: Mortal{
			MaxLife: s.entityBaseMaxLife,
			Life:    s.entityBaseMaxLife * s.initialLifeFraction,
		},
		MapCode:       startMapCode,
		Path:          []PointI{},
		TargetPos:     PointI{-1, -1},
		Direction:     NONE,
		Mode:          IDLE,
		SumStatsLimit: s.sumStatsLimit,
		LifeRegen:     lifeRegen,
	}
	client := &Client{
		playerID: playerID,
		sock: socket.New(conn, socket.Metrics{
			Read:       s.recordWsRead,
			Write:      s.recordWsWrite,
			ReadError:  s.recordWsReadError,
			WriteError: s.recordWsWriteError,
		}),
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

	// Loading protection: every join spawns frozen under "loading" — no
	// movement, no combat, no damage, no interactions — until the client
	// confirms the player pressed Tap-to-Start (freeze_end "loading").
	FreezePlayer(playerState, "loading")

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
		ObjectLayers:   s.visibleInventory(playerState.ObjectLayers),
		SkillMap:       s.buildSkillMap(),
		EntityDefaults: s.buildEntityDefaultsSlice(),
		DeadItemIds:    s.deadItemIDList(),
		Quests:         s.buildQuestSnapshot(playerState),
	}
	sendMessage(playerState, "init_data", initPayload)

	// Send metadata message with ObjectLayer data for client-side caching.
	sendMessage(playerState, "metadata", map[string]interface{}{
		"objectLayers":   s.buildOLMetadataMap(),
		"apiBaseUrl":     s.enginePublicURL,
		"instanceCode":   s.instanceCode,
		"equipmentRules": s.equipmentRules,
	})

	s.mu.Unlock()

	// Register the client with listenForClients.
	// Use a timeout so we get a clear log if listenForClients is dead rather
	// than hanging the HTTP handler goroutine silently.
	select {
	case s.register <- client:
		s.recordWsConnect()
	case <-time.After(5 * time.Second):
		log.Printf("[HandleConnections] timeout waiting to register player=%s — listenForClients may be dead", playerID)
		client.sock.Close()
		return
	}
	go client.readPump(s)
}

// sendMessage packs a message and queues it for the player. The send never
// blocks: a full queue drops the message.
func sendMessage(player *PlayerState, msgType string, payload any) {
	if player == nil || player.Client == nil {
		return
	}
	pack, err := serial.Pack(msgType, payload)
	if err != nil {
		log.Printf("[sendMessage] pack %q failed: %v", msgType, err)
		return
	}
	if !player.Client.sock.Send(pack) {
		logx.Debugf("Client %s queue full — dropped %q.", player.ID, msgType)
	}
}

// readPump runs the client read loop until the connection fails.
func (c *Client) readPump(server *GameServer) {
	defer func() {
		if r := recover(); r != nil {
			log.Printf("[readPump] PANIC player=%s: %v", c.playerID, r)
		}
		logx.Debugf("[readPump] closing player=%s", c.playerID)
		server.recordWsDisconnect()
		server.unregister <- c
		c.sock.Close()
	}()
	c.sock.Receive(func(pack []byte) { c.receiveMessage(pack, server) })
}

// inputKinds maps a wire message type to the internal input kind. The kind
// enum stays internal; only this table knows the wire words.
var inputKinds = map[string]InputKind{
	"handshake":        InputKindHandshake,
	"player_action":    InputKindPlayerAction,
	"item_active":      InputKindItemActivation,
	"freeze_start":     InputKindFreezeStart,
	"freeze_end":       InputKindFreezeEnd,
	"chat":             InputKindChat,
	"dialog_start":     InputKindDlgStart,
	"dialog_complete":  InputKindDlgComplete,
	"dialog_cancel":    InputKindDlgCancel,
	"quest_abandon":    InputKindQuestAbandon,
	"quest_accept":     InputKindQuestAccept,
	"shop_buy":         InputKindShopBuy,
	"craft_item":       InputKindCraftItem,
	"craft_cancel":     InputKindCraftCancel,
	"storage_open":     InputKindStorageOpen,
	"storage_move":     InputKindStorageMove,
	"storage_swap":     InputKindStorageSwap,
	"storage_transfer": InputKindStorageTransfer,
}

// inputPayload holds every client → server payload field. Each message type
// fills the subset it needs; the rest stay zero.
type inputPayload struct {
	Tick uint32 `json:"tick"`
	Seq  uint32 `json:"seq"`

	X float64 `json:"x"` // player_action
	Y float64 `json:"y"`

	ItemID string `json:"itemId"` // item_active, shop_buy
	Active bool   `json:"active"` // item_active

	Reason string `json:"reason"` // freeze_start, freeze_end

	ToID string `json:"toId"` // chat
	Text string `json:"text"`

	EntityID   string `json:"entityId"`   // dialog_*, quest_accept, shop_buy
	DialogCode string `json:"dialogCode"` // dialog_complete
	QuestCode  string `json:"questCode"`  // quest_*

	Quantity    int `json:"quantity"`    // shop_buy
	RecipeIndex int `json:"recipeIndex"` // craft_item

	FromIndex int  `json:"fromIndex"` // storage_*
	ToIndex   int  `json:"toIndex"`
	Deposit   bool `json:"deposit"` // storage_transfer
}

// receiveMessage is the single client → server dispatch point. It unpacks one
// message into an InputCommand and enqueues it on the player's per-tick input
// queue. phaseInput drains and applies it exactly once per tick.
func (c *Client) receiveMessage(pack []byte, server *GameServer) {
	msg, err := serial.Unpack(pack)
	if err != nil {
		logx.Debugf("Bad message from player %s: %v", c.playerID, err)
		return
	}
	kind, known := inputKinds[msg.Type]
	if !known {
		logx.Debugf("Unknown message type %q from player %s", msg.Type, c.playerID)
		return
	}
	if kind == InputKindHandshake {
		return // already authenticated upstream; nothing to do
	}

	var p inputPayload
	if err := json.Unmarshal(msg.Payload, &p); err != nil {
		logx.Debugf("Bad %q payload from player %s: %v", msg.Type, c.playerID, err)
		return
	}

	cmd := InputCommand{Kind: kind, ClientTick: p.Tick, Sequence: p.Seq}
	switch kind {
	case InputKindPlayerAction:
		cmd.TargetX = p.X
		cmd.TargetY = p.Y
	case InputKindItemActivation:
		if p.ItemID == "" {
			return
		}
		cmd.ItemID = p.ItemID
		cmd.Active = p.Active
	case InputKindFreezeStart, InputKindFreezeEnd:
		cmd.Reason = p.Reason
		if cmd.Reason == "" {
			cmd.Reason = "freeze"
		}
	case InputKindChat:
		if p.ToID == "" || p.Text == "" {
			return
		}
		cmd.ItemID = p.ToID // chat target id
		cmd.ChatText = p.Text
	case InputKindDlgStart, InputKindDlgCancel:
		if p.EntityID == "" {
			return
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.ItemID
	case InputKindDlgComplete:
		if p.EntityID == "" {
			return
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.ItemID
		cmd.DialogCode = p.DialogCode
	case InputKindQuestAbandon:
		if p.QuestCode == "" {
			return
		}
		cmd.ItemID = p.QuestCode
	case InputKindQuestAccept:
		if p.EntityID == "" || p.QuestCode == "" {
			return
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.QuestCode
	case InputKindShopBuy:
		if p.EntityID == "" || p.ItemID == "" {
			return
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.ItemID
		cmd.Quantity = p.Quantity
	case InputKindCraftItem:
		if p.EntityID == "" {
			return
		}
		cmd.EntityID = p.EntityID
		cmd.RecipeIndex = p.RecipeIndex
	case InputKindStorageOpen, InputKindStorageMove, InputKindStorageSwap,
		InputKindStorageTransfer:
		if p.EntityID == "" {
			return
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.ItemID
		cmd.Quantity = p.Quantity
		cmd.FromIndex, cmd.ToIndex = p.FromIndex, p.ToIndex
		cmd.Deposit = p.Deposit
	}
	c.dispatchInputCommand(server, cmd)
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
