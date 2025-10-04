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

	playerID := uuid.New().String()
	// playerDims := Dimensions{Width: float64(rand.Intn(4) + 1), Height: float64(rand.Intn(4) + 1)}
	playerDims := Dimensions{Width: 3, Height: 3}

	startMapID := rand.Intn(len(s.maps))
	startMapState := s.maps[startMapID]
	startPosI, err := startMapState.pathfinder.findRandomWalkablePoint(playerDims)
	if err != nil {
		log.Printf("Could not place new player: %v", err)
		conn.Close()
		s.mu.Unlock()
		return
	}
	lifeRegen := rand.Float64()*9 + 1 // 1 to 10 life points

	playerState := &PlayerState{
		ID:            playerID,
		MapID:         startMapID,
		Pos:           Point{X: float64(startPosI.X), Y: float64(startPosI.Y)},
		Dims:          playerDims,
		Path:          []PointI{},
		TargetPos:     PointI{-1, -1},
		Direction:     NONE,
		Mode:          IDLE,
		SumStatsLimit: 100,
		ObjectLayers: []ObjectLayerState{
			{ItemID: "anon", Active: true, Quantity: 1},
			{ItemID: "atlas_pistol_mk2", Active: true, Quantity: 1},
			{ItemID: "punk", Active: false, Quantity: 1},
			{ItemID: "coin", Active: false, Quantity: 10},
			{ItemID: "purple", Active: false, Quantity: 1},
			{ItemID: "green", Active: false, Quantity: 1},
			{ItemID: "paranoia", Active: false, Quantity: 1},
			{ItemID: "wason", Active: false, Quantity: 1},
			{ItemID: "alex", Active: false, Quantity: 1},
		},
		MaxLife:   s.entityBaseMaxLife, // Base life, will be modified by stats
		Life:      s.entityBaseMaxLife * 0.5,
		LifeRegen: lifeRegen,
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

	// Apply initial stats (like Resistance for MaxLife) after creation.
	s.ApplyResistanceStat(playerState, startMapState)
	playerState.Life = playerState.MaxLife * 0.5 // Set life to 50% of final max life

	initPayload := map[string]interface{}{
		"gridW":                     startMapState.gridW,
		"gridH":                     startMapState.gridH,
		"defaultObjectWidth":        s.defaultObjWidth,
		"defaultObjectHeight":       s.defaultObjHeight,
		"cellSize":                  s.cellSize,
		"fps":                       s.fps,
		"interpolationMs":           s.interpolationMs,
		"aoiRadius":                 s.aoiRadius,
		"colors":                    s.colors,
		"cameraSmoothing":           s.cameraSmoothing,
		"cameraZoom":                s.cameraZoom,
		"defaultWidthScreenFactor":  s.defaultWidthScreenFactor,
		"defaultHeightScreenFactor": s.defaultHeightScreenFactor,
		"devUi":                     s.devUi,
		"sumStatsLimit":             playerState.SumStatsLimit,
		"objectLayers":              playerState.ObjectLayers,
	}
	initMsg, _ := json.Marshal(map[string]interface{}{"type": "init_data", "payload": initPayload})
	select {
	case client.send <- initMsg:
	default:
		log.Printf("Client %s init channel full.", client.playerID)
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
			player, ok := server.maps[c.playerState.MapID].players[c.playerID]
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

			mapState, ok := server.maps[player.MapID]
			if !ok {
				log.Println("Player map not found")
				server.mu.Unlock()
				continue
			}

			// Rate-limit player actions based on Utility.
			playerStats := server.CalculateStats(player, mapState)
			cooldown := server.CalculateActionCooldown(playerStats)
			if time.Since(c.lastAction) < cooldown {
				server.mu.Unlock()
				continue // Action came too fast, ignore it.
			}
			c.lastAction = time.Now() // Update last action time

			server.mu.Unlock()

			// Handle probabilistic life regeneration on action
			server.handleProbabilisticRegen(player, mapState)

			// Handle skills that trigger on player action
			server.HandlePlayerActionSkills(player, mapState, Point{X: targetX, Y: targetY})

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

			if newPath != nil && len(newPath) > 0 {
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

				player, ok := server.maps[c.playerState.MapID].players[c.playerID]
				if !ok {
					log.Printf("Player %s not found in map %d for item activation", c.playerID, c.playerState.MapID)
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

				// --- Step 2: If activating a unique-type item, deactivate others of the same type (handles swapping) ---
				if active {
					var requestedItemType string
					if itemData, ok := server.objectLayerDataCache[itemId]; ok {
						requestedItemType = itemData.Data.Item.Type
					}

					if requestedItemType == "skin" {
						for i := range player.ObjectLayers {
							if i == targetItemIndex {
								continue // Don't touch the item we just activated.
							}
							var currentItemType string
							if itemData, ok := server.objectLayerDataCache[player.ObjectLayers[i].ItemID]; ok {
								currentItemType = itemData.Data.Item.Type
							}
							if currentItemType == requestedItemType && player.ObjectLayers[i].Active {
								player.ObjectLayers[i].Active = false
								log.Printf("Swapping active items: Deactivated '%s' for player %s.", player.ObjectLayers[i].ItemID, c.playerID)
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
					if itemData, ok := server.objectLayerDataCache[layer.ItemID]; ok && itemData.Data.Item.Type == "skin" {
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

				// Correction 2: Player cannot have more than 4 active items. Revert the activation if this rule is broken.
				if activeLayerCount > 4 {
					log.Printf("Player %s has more than 4 active items after request. Reverting activation of '%s'.", c.playerID, itemId)
					player.ObjectLayers[targetItemIndex].Active = originalState // Revert the specific change
				}

				// --- Step 4: After all corrections, recalculate stats that affect the player state directly. ---
				server.ApplyResistanceStat(player, server.maps[player.MapID])
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
			w, err := c.conn.NextWriter(websocket.TextMessage)
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
