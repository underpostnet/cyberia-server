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
	playerDims := Dimensions{Width: float64(rand.Intn(4) + 1), Height: float64(rand.Intn(4) + 1)}

	startMapID := rand.Intn(len(s.maps))
	startMapState := s.maps[startMapID]
	startPosI, err := startMapState.pathfinder.findRandomWalkablePoint(playerDims)
	if err != nil {
		log.Printf("Could not place new player: %v", err)
		conn.Close()
		s.mu.Unlock()
		return
	}
	maxLife := float64(rand.Intn(151) + 50) // Random between 50 and 200
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
		ObjectLayers:  []ObjectLayerState{{ItemID: "anon", Active: true}, {ItemID: "atlas_pistol_mk2", Active: true}, {ItemID: "punk", Active: false}, {ItemID: "ghost", Active: false}},
		MaxLife:       maxLife,
		Life:          maxLife * 0.5, // Set life to 50% of max life
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
			server.mu.Unlock()

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
			player, ok := server.maps[c.playerState.MapID].players[c.playerID]
			if !ok {
				log.Printf("Player %s not found in map %d for item activation", c.playerID, c.playerState.MapID)
				server.mu.Unlock()
				continue
			}

			// --- LAYER RESTRICTION LOGIC for Players ---
			// If trying to activate an item, check against the rules.
			if active {
				activeLayerCount := 0
				activeLayerTypes := make(map[string]bool)
				var requestedItemType string

				for _, layer := range player.ObjectLayers {
					if layer.Active {
						var itemType string
						if itemData, ok := server.objectLayerDataCache[layer.ItemID]; ok {
							itemType = itemData.Data.Item.Type
						} else {
							itemType = "generic"
						}
						activeLayerTypes[itemType] = true
						activeLayerCount++
					}
				}
				if itemData, ok := server.objectLayerDataCache[itemId]; ok {
					requestedItemType = itemData.Data.Item.Type
				} else {
					requestedItemType = "generic"
				}

				if activeLayerCount >= 4 {
					log.Printf("Player %s tried to activate item '%s', but already has max (4) active items. Denied.", c.playerID, itemId)
					server.mu.Unlock()
					continue
				}
				if _, exists := activeLayerTypes[requestedItemType]; exists {
					log.Printf("Player %s tried to activate item '%s' of type '%s', but this type is already active. Denied.", c.playerID, itemId, requestedItemType)
					server.mu.Unlock()
					continue
				}
			}
			// If trying to deactivate, check if it's the last active skin.
			if !active {
				// We only care about deactivating skins.
				var isTargetSkin bool
				if itemData, ok := server.objectLayerDataCache[itemId]; ok {
					isTargetSkin = itemData.Data.Item.Type == "skin"
				}

				if isTargetSkin {
					activeSkinCount := 0
					for _, layer := range player.ObjectLayers {
						if !layer.Active {
							continue
						}

						var isLayerSkin bool
						if itemData, ok := server.objectLayerDataCache[layer.ItemID]; ok {
							isLayerSkin = itemData.Data.Item.Type == "skin"
						}
						if isLayerSkin {
							activeSkinCount++
						}
					}

					// If this is the last active skin, prevent deactivation.
					if activeSkinCount <= 1 {
						log.Printf("Player %s tried to deactivate the last active skin '%s'. Denied.", c.playerID, itemId)
						server.mu.Unlock()
						continue // Skip the deactivation
					}
				}
			}

			for i := range player.ObjectLayers {
				if player.ObjectLayers[i].ItemID == itemId {
					player.ObjectLayers[i].Active = active
					log.Printf("Player %s updated item '%s' active state to %v", c.playerID, itemId, active)
					break
				}
			}
			server.mu.Unlock()
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
