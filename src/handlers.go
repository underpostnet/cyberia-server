package game

import (
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
	Sha256        string          `json:"sha256"`
	Data          ObjectLayerData `json:"data"`
	FrameDuration int             `json:"frame_duration"`
	IsStateless   bool            `json:"is_stateless"`
}

// buildSkillMap returns a compact { triggerItemId → [logicEventIds] } map
// derived from the server's skillConfig — sent to clients in init_data.
func (s *GameServer) buildSkillMap() map[string][]string {
	out := make(map[string][]string, len(s.skillConfig))
	for triggerID, defs := range s.skillConfig {
		combined := make(map[string]struct{})
		for _, def := range defs {
			for _, id := range def.LogicEventIDs {
				combined[id] = struct{}{}
			}
		}
		ids := make([]string, 0, len(combined))
		for id := range combined {
			ids = append(ids, id)
		}
		out[triggerID] = ids
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
			Sha256:        ol.Sha256,
			Data:          ol.Data,
			FrameDuration: ol.FrameDuration,
			IsStateless:   ol.IsStateless,
		}
	}
	return out
}

// buildAtlasCacheSnapshot returns a read-locked snapshot of the atlas cache.
func (s *GameServer) buildAtlasCacheSnapshot() map[string]*AtlasData {
	s.olMu.RLock()
	defer s.olMu.RUnlock()
	// Return the map directly — it's only replaced atomically
	return s.atlasDataCache
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

	// Copy default object layers from config
	playerOLs := make([]ObjectLayerState, len(s.defaultPlayerObjectLayers))
	copy(playerOLs, s.defaultPlayerObjectLayers)
	// If no default layers are configured use the instance-level user default
	// visual (atlas if present, solid PLAYER colour otherwise).
	if len(playerOLs) == 0 {
		if d, ok := s.entityDefaults["player"]; ok && len(d.LiveItemIDs) > 0 {
			for _, itemID := range d.LiveItemIDs {
				playerOLs = append(playerOLs, ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
			}
		}
	}
	playerState := &PlayerState{
		ID:            playerID,
		MapCode:       startMapCode,
		Pos:           Point{X: float64(startPosI.X), Y: float64(startPosI.Y)},
		Dims:          playerDims,
		Color:         s.colors["PLAYER"],
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

	initPayload := InitPayload{
		GridW:                     startMapState.gridW,
		GridH:                     startMapState.gridH,
		DefaultObjectWidth:        s.defaultObjWidth,
		DefaultObjectHeight:       s.defaultObjHeight,
		CellSize:                  s.cellSize,
		Fps:                       s.fps,
		InterpolationMs:           s.interpolationMs,
		AoiRadius:                 s.aoiRadius,
		Colors:          s.colors,
		EntityDefaults:  s.buildEntityDefaultsSlice(),
		CameraSmoothing:           s.cameraSmoothing,
		CameraZoom:                s.cameraZoom,
		DefaultWidthScreenFactor:  s.defaultWidthScreenFactor,
		DefaultHeightScreenFactor: s.defaultHeightScreenFactor,
		DevUi:                     s.devUi,
		SumStatsLimit:             playerState.SumStatsLimit,
		ObjectLayers:              playerState.ObjectLayers,
		Color:                     playerState.Color,
		SkillMap:                  s.buildSkillMap(),
	}
	initMsg, _ := json.Marshal(map[string]interface{}{"type": "init_data", "payload": initPayload})
	select {
	case client.send <- initMsg:
	default:
		log.Printf("Client %s init channel full.", client.playerID)
	}

	// Send metadata message with all ObjectLayer + Atlas data for client-side caching.
	// This replaces the client's REST calls to the Engine API for OL/Atlas metadata.
	metaPayload := map[string]interface{}{
		"objectLayers": s.buildOLMetadataMap(),
		"atlasData":    s.buildAtlasCacheSnapshot(),
		"apiBaseUrl":   s.engineApiBaseUrl,
	}
	metaMsg, _ := json.Marshal(map[string]interface{}{"type": "metadata", "payload": metaPayload})
	select {
	case client.send <- metaMsg:
	default:
		log.Printf("Client %s metadata channel full.", client.playerID)
	}

	s.mu.Unlock()

	s.register <- client
	go client.writePump()
	go client.readPump(s)
}

// readPump handles incoming messages from the client.
func (c *Client) readPump(server *GameServer) {
	defer func() {
		server.unregister <- c
		c.conn.Close()
	}()
	c.conn.SetReadLimit(512)
	c.conn.SetReadDeadline(time.Now().Add(60 * time.Second))
	c.conn.SetPongHandler(func(string) error {
		c.conn.SetReadDeadline(time.Now().Add(60 * time.Second))
		return nil
	})
	for {
		_, message, err := c.conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("error: %v", err)
			}
			break
		}
		var msg map[string]interface{}
		if err := json.Unmarshal(message, &msg); err != nil {
			log.Printf("Error unmarshaling message: %v", err)
			continue
		}
		if msg["type"] == "player_action" {
			payload := msg["payload"].(map[string]interface{})
			targetX := payload["targetX"].(float64)
			targetY := payload["targetY"].(float64)

			server.mu.Lock()
			player, ok := server.maps[c.playerState.MapCode].players[c.playerID]
			if !ok {
				log.Println("Player not found in map")
				server.mu.Unlock()
				continue
			}

			// Dead players can't perform actions.
			if player.IsGhost() {
				server.mu.Unlock()
				continue
			}

			mapState, ok := server.maps[player.MapCode]
			if !ok {
				log.Println("Player map not found")
				server.mu.Unlock()
				continue
			}

			// Compute movement cooldown under lock but do NOT gate skills on it.
			// TAP is the fundamental event: skills fire on every TAP (probability-
			// gated), movement is a rendering side-effect gated by the cooldown.
			playerStats := server.CalculateStats(player, mapState)
			cooldown := server.CalculateActionCooldown(playerStats)
			movementReady := time.Since(c.lastAction) >= cooldown
			if movementReady {
				c.lastAction = time.Now()
			}

			server.mu.Unlock()

			// Skills and regen fire on EVERY valid TAP regardless of movement cooldown.
			server.HandlePlayerTapAction(player, mapState, Point{X: targetX, Y: targetY})

			// Movement path is only recalculated when the cooldown allows.
			if !movementReady {
				continue
			}

			startPosI := PointI{X: int(math.Round(player.Pos.X)), Y: int(math.Round(player.Pos.Y))}
			targetPosI := PointI{X: int(math.Round(targetX)), Y: int(math.Round(targetY))}

			newPath, err := mapState.pathfinder.Astar(startPosI, targetPosI, player.Dims)
			usedTarget := targetPosI
			if err != nil {
				closest, cerr := mapState.pathfinder.findClosestWalkablePoint(targetPosI, player.Dims)
				if cerr != nil {
					log.Printf("Pathfinding failed for player %s, no closest walkable: %v", c.playerID, err)
					server.mu.Lock()
					dx := float64(targetPosI.X) - player.Pos.X
					dy := float64(targetPosI.Y) - player.Pos.Y
					dist := math.Sqrt(dx*dx + dy*dy)
					if dist > 0 {
						dirX, dirY := dx/dist, dy/dist
						server.updatePlayerDirection(player, dirX, dirY)
						player.Mode = WALKING
						player.TargetPos = targetPosI
					} else {
						player.Mode = IDLE
					}
					server.mu.Unlock()
					continue
				}
				newPath, err = mapState.pathfinder.Astar(startPosI, closest, player.Dims)
				if err != nil {
					log.Printf("Pathfinding failed for player %s even to closest walkable: %v", c.playerID, err)
					server.mu.Lock()
					dx := float64(closest.X) - player.Pos.X
					dy := float64(closest.Y) - player.Pos.Y
					dist := math.Sqrt(dx*dx + dy*dy)
					if dist > 0 {
						dirX, dirY := dx/dist, dy/dist
						server.updatePlayerDirection(player, dirX, dirY)
						player.Mode = WALKING
						player.TargetPos = closest
					} else {
						player.Mode = IDLE
					}
					server.mu.Unlock()
					continue
				}
				usedTarget = closest
			}

			if len(newPath) > 0 {
				first := newPath[0]
				server.mu.Lock()
				player.Path = newPath
				player.TargetPos = usedTarget
				player.Mode = WALKING
				dx := float64(first.X) - player.Pos.X
				dy := float64(first.Y) - player.Pos.Y
				dist := math.Sqrt(dx*dx + dy*dy)
				if dist > 0 {
					dirX, dirY := dx/dist, dy/dist
					server.updatePlayerDirection(player, dirX, dirY)
				}
				server.mu.Unlock()
			} else {
				server.mu.Lock()
				if startPosI.X == targetPosI.X && startPosI.Y == targetPosI.Y {
					player.Mode = IDLE
				}
				server.mu.Unlock()
			}
		} else if msg["type"] == "item_activation" {
			payload, ok := msg["payload"].(map[string]interface{})
			if !ok {
				log.Printf("Invalid item_activation payload format for player %s", c.playerID)
				continue
			}
			itemId, okId := payload["itemId"].(string)
			active, okActive := payload["active"].(bool)
			if !okId || !okActive {
				log.Printf("Invalid itemId or active field in item_activation payload for player %s", c.playerID)
				continue
			}

			server.mu.Lock()
			func() { // Use a closure to manage the lock with defer
				defer server.mu.Unlock()

				player, ok := server.maps[c.playerState.MapCode].players[c.playerID]
				if !ok {
					log.Printf("Player %s not found in map %q for item activation", c.playerID, c.playerState.MapCode)
					return
				}

				// Check if player is dead - only allow dead item IDs to be activated while dead
				if (player.IsGhost() || player.Life <= 0) && active && !server.isDeadItemID("player", itemId) {
					log.Printf("Player %s is dead and cannot activate non-ghost item '%s'.", c.playerID, itemId)
					return
				}

				var targetItemIndex = -1
				for i := range player.ObjectLayers {
					if player.ObjectLayers[i].ItemID == itemId {
						targetItemIndex = i
						break
					}
				}

				if targetItemIndex == -1 {
					log.Printf("Player %s tried to activate non-existent item '%s'.", c.playerID, itemId)
					return
				}

				// --- Step 1: Tentatively apply the requested change ---
				originalState := player.ObjectLayers[targetItemIndex].Active
				player.ObjectLayers[targetItemIndex].Active = active
				log.Printf("Player %s requested to set item '%s' active state to %v. Applying and validating...", c.playerID, itemId, active)

// --- Step 2: If activating an item, deactivate all other items of the SAME type (one-active-per-type rule) ---
								// This handles swapping uniformly for ALL item types (skins, weapons, armor, etc.)
								// so the inventory always has at most one active item per type.
								if active {
									var requestedItemType string
									if itemData, ok := server.GetObjectLayerData(itemId); ok {
										requestedItemType = itemData.Data.Item.Type
									}

									if requestedItemType != "" {
										for i := range player.ObjectLayers {
											if i == targetItemIndex {
												continue // Don't touch the item we just activated.
											}
											var currentItemType string
											if itemData, ok := server.GetObjectLayerData(player.ObjectLayers[i].ItemID); ok {
												currentItemType = itemData.Data.Item.Type
											}
											if currentItemType == requestedItemType && player.ObjectLayers[i].Active {
												player.ObjectLayers[i].Active = false
												log.Printf("Swapping active items: Deactivated '%s' (type=%s) for player %s.", player.ObjectLayers[i].ItemID, currentItemType, c.playerID)
							}
						}
					}
				}

				// --- Step 3: Run validation and correction logic ---
				activeLayerCount := 0
				activeSkinCount := 0
				hasAnySkin := false
				firstSkinIndex := -1

				// First pass to count active layers/skins and find the first available skin
				for i, layer := range player.ObjectLayers {
					var isSkin bool
					if itemData, ok := server.GetObjectLayerData(layer.ItemID); ok && itemData.Data.Item.Type == "skin" {
						isSkin = true
					}

					if isSkin {
						hasAnySkin = true
						if firstSkinIndex == -1 {
							firstSkinIndex = i
						}
						if layer.Active {
							activeSkinCount++
						}
					}
					if layer.Active {
						activeLayerCount++
					}
				}

				// Correction 1: Player must have at least one active skin if they own one.
				if hasAnySkin && activeSkinCount == 0 && firstSkinIndex != -1 {
					player.ObjectLayers[firstSkinIndex].Active = true
					log.Printf("Player %s tried to deactivate the last skin. Force-activating skin '%s' to maintain a valid state.", c.playerID, player.ObjectLayers[firstSkinIndex].ItemID)
				}

				// Correction 2: Player cannot have more than maxActiveLayers active items. Revert the activation if this rule is broken.
				if activeLayerCount > server.maxActiveLayers {
					log.Printf("Player %s has more than %d active items after request. Reverting activation of '%s'.", c.playerID, server.maxActiveLayers, itemId)
					player.ObjectLayers[targetItemIndex].Active = originalState // Revert the specific change
				}

				// --- Step 4: After all corrections, recalculate stats that affect the player state directly. ---
				server.InvalidateStats(player)
				server.ApplyResistanceStat(player, server.maps[player.MapCode])
			}()
		} else if msg["type"] == "get_items_ids" {
			payload, ok := msg["payload"].(map[string]interface{})
			if !ok {
				log.Printf("Invalid get_items_ids payload format for player %s", c.playerID)
				continue
			}
			itemId, ok := payload["itemId"].(string)
			if !ok {
				log.Printf("Invalid itemId in get_items_ids payload for player %s", c.playerID)
				continue
			}

			associatedIDs := server.GetAssociatedSkillItemIDs(itemId)

			responsePayload := map[string]interface{}{
				"requestedItemId":   itemId,
				"associatedItemIds": associatedIDs,
			}
			responseMsg, err := json.Marshal(map[string]interface{}{"type": "skill_item_ids", "payload": responsePayload})
			if err != nil {
				log.Printf("Error marshaling skill_item_ids response: %v", err)
				continue
			}
			c.send <- responseMsg
		}
	}
}

// writePump writes messages to the WebSocket connection.
func (c *Client) writePump() {
	ticker := time.NewTicker(30 * time.Second)
	defer func() {
		ticker.Stop()
		c.conn.Close()
	}()
	for {
		select {
		case message, ok := <-c.send:
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if !ok {
				c.conn.WriteMessage(websocket.CloseMessage, []byte{})
				return
			}
			// Detect message type: binary AOI starts with 0x01-0x03, JSON starts with '{'
			msgType := websocket.TextMessage
			if len(message) > 0 && message[0] != '{' && message[0] != '[' {
				msgType = websocket.BinaryMessage
			}
			w, err := c.conn.NextWriter(msgType)
			if err != nil {
				return
			}
			w.Write(message)
			if err := w.Close(); err != nil {
				return
			}
		case <-ticker.C:
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if err := c.conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				return
			}
		}
	}
}
