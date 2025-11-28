package game

import (
	"encoding/json"
	"log"
	"math"
	"math/rand"
	"time"
)

// NewGameServer creates a new game server.
func NewGameServer() *GameServer {
	gs := &GameServer{
		maps:            make(map[int]*MapState),
		clients:         make(map[string]*Client),
		register:        make(chan *Client),
		unregister:      make(chan *Client),
		aoiRadius:       15.0,
		portalHoldTime:  2 * time.Second,
		entityBaseSpeed: 24.0,

		cellSize:         12.0,
		fps:              60,
		interpolationMs:  200,
		defaultObjWidth:  1.0,
		defaultObjHeight: 1.0,
		cameraSmoothing:  0.15,
		cameraZoom:       2,
		// production immersive settings
		defaultWidthScreenFactor:  0.9,
		defaultHeightScreenFactor: 0.9,
		devUi:                     false,
		// defaultWidthScreenFactor:    0.1,
		// defaultHeightScreenFactor:   0.9,
		// devUi:                       true,
		botsPerMap:                  10,
		botAggroRange:               10.0,
		entityBaseMaxLife:           100.0,
		entityBaseActionCooldown:    200 * time.Millisecond,
		entityBaseMinActionCooldown: 50 * time.Millisecond,
		colors: map[string]ColorRGBA{
			"BACKGROUND":       {R: 0, G: 0, B: 0, A: 255},
			"GRID_BACKGROUND":  {R: 128, G: 128, B: 128, A: 255},
			"FLOOR_BACKGROUND": {R: 0, G: 255, B: 0, A: 255},
			"OBSTACLE":         {R: 100, G: 100, B: 100, A: 255},
			"FOREGROUND":       {R: 60, G: 140, B: 60, A: 220},
			"PLAYER":           {R: 0, G: 200, B: 255, A: 255},
			"OTHER_PLAYER":     {R: 255, G: 100, B: 0, A: 255},
			"PATH":             {R: 255, G: 255, B: 0, A: 128},
			"TARGET":           {R: 255, G: 255, B: 0, A: 255},
			"AOI":              {R: 255, G: 0, B: 255, A: 51},
			"DEBUG_TEXT":       {R: 220, G: 220, B: 220, A: 255},
			"ERROR_TEXT":       {R: 255, G: 50, B: 50, A: 255},
			"PORTAL":           {R: 180, G: 50, B: 255, A: 180},
			"PORTAL_LABEL":     {R: 240, G: 240, B: 240, A: 255},
			"UI_TEXT":          {R: 255, G: 255, B: 255, A: 255},
			"MAP_BOUNDARY":     {R: 255, G: 255, B: 255, A: 255},
		},
		objectLayerDataCache: make(map[string]*ObjectLayer),
	}
	gs.createMaps()
	return gs
}

// SetObjectLayerCache sets the cache for object layer data.
// This is called from main after the server is created and data is loaded.
func (s *GameServer) SetObjectLayerCache(cache map[string]*ObjectLayer) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.objectLayerDataCache = cache
	log.Printf("Object layer cache set with %d items.", len(s.objectLayerDataCache))
}

// UpdateObjectLayerCache updates specific entries in the object layer cache.
// This is used to refresh cache entries with the latest database data.
func (s *GameServer) UpdateObjectLayerCache(updates map[string]*ObjectLayer) {
	s.mu.Lock()
	defer s.mu.Unlock()

	for itemID, layer := range updates {
		s.objectLayerDataCache[itemID] = layer
	}
	log.Printf("Object layer cache updated with %d items.", len(updates))
}

func (s *GameServer) Run() {
	go s.listenForClients()
	go s.gameLoop()
}

func (s *GameServer) listenForClients() {
	log.Println("Starting client listener...")
	for {
		select {
		case client := <-s.register:
			s.mu.Lock()
			s.clients[client.playerID] = client
			s.mu.Unlock()
		case client := <-s.unregister:
			s.mu.Lock()
			if _, ok := s.clients[client.playerID]; ok {
				delete(s.clients, client.playerID)
				playerState := client.playerState
				if playerState != nil {
					mapState, ok := s.maps[playerState.MapID]
					if ok {
						delete(mapState.players, client.playerID)
					}
				}
				close(client.send)
			}
			s.mu.Unlock()
		}
	}
}

func (s *GameServer) gameLoop() {
	ticker := time.NewTicker(100 * time.Millisecond) // 10 FPS
	defer ticker.Stop()

	for range ticker.C {
		s.mu.Lock()
		for _, mapState := range s.maps {
			// Phase 1: Handle state changes (respawn, death from collisions)
			s.handleRespawns(mapState)
			s.handleBulletCollisions(mapState)

			// Phase 2: Update positions based on current state
			for _, player := range mapState.players {
				s.updatePlayerPosition(player, mapState)
				s.checkPortal(player, mapState)
			}
			s.updateBots(mapState)

			// Phase 3: Broadcast new state to clients
			s.updateAOIs(mapState)
		}
		s.mu.Unlock()
	}
}

// ---------- Player movement and direction ----------
func (s *GameServer) updatePlayerPosition(player *PlayerState, mapState *MapState) {
	// Dead players can't move.
	if player.IsGhost() {
		return
	}

	playerStats := s.CalculateStats(player, mapState)
	speed := s.CalculateMovementSpeed(playerStats)

	if player.Mode == WALKING && len(player.Path) > 0 {
		targetNode := player.Path[0]
		dx := float64(targetNode.X) - player.Pos.X
		dy := float64(targetNode.Y) - player.Pos.Y
		dist := math.Sqrt(dx*dx + dy*dy)
		step := speed / (1000.0 / 100.0) // speed per tick (100ms)

		if dist < step {
			player.Pos = Point{X: float64(targetNode.X), Y: float64(targetNode.Y)}
			player.Path = player.Path[1:]
			if len(player.Path) == 0 {
				player.Mode = IDLE
				player.Direction = NONE
			} else {
				next := player.Path[0]
				dirX := float64(next.X) - player.Pos.X
				dirY := float64(next.Y) - player.Pos.Y
				norm := math.Sqrt(dirX*dirX + dirY*dirY)
				if norm > 0 {
					dirX /= norm
					dirY /= norm
					s.updatePlayerDirection(player, dirX, dirY)
				}
			}
		} else {
			dirX, dirY := dx/dist, dy/dist
			player.Pos.X += dirX * step
			player.Pos.Y += dirY * step
			s.updatePlayerDirection(player, dirX, dirY)
		}
	}
}

func (s *GameServer) updatePlayerDirection(player *PlayerState, dirX, dirY float64) {
	angle := math.Atan2(dirY, dirX)
	if angle < 0 {
		angle += 2 * math.Pi
	}
	directionIndex := (int(math.Round(angle/(math.Pi/4))) + 2) % 8
	player.Direction = Direction(directionIndex)
}

// ---------- Portals ----------
func (s *GameServer) checkPortal(player *PlayerState, mapState *MapState) {
	onPortal := false
	var activePortal *PortalState
	for _, portal := range mapState.portals {
		portalRect := Rectangle{MinX: portal.Pos.X, MinY: portal.Pos.Y, MaxX: portal.Pos.X + portal.Dims.Width, MaxY: portal.Pos.Y + portal.Dims.Height}
		playerRect := Rectangle{MinX: player.Pos.X, MinY: player.Pos.Y, MaxX: player.Pos.X + player.Dims.Width, MaxY: player.Pos.Y + player.Dims.Height}
		if rectsOverlap(playerRect, portalRect) {
			onPortal = true
			activePortal = portal
			break
		}
	}
	if onPortal {
		if !player.OnPortal {
			player.OnPortal = true
			player.TimeOnPortal = time.Now()
			player.ActivePortalID = activePortal.ID
		} else if time.Since(player.TimeOnPortal) > s.portalHoldTime {
			s.teleportPlayer(player, activePortal)
		}
	} else {
		player.OnPortal = false
		player.ActivePortalID = ""
	}
}

func (s *GameServer) teleportPlayer(player *PlayerState, portal *PortalState) {
	s.mu.Unlock()
	defer s.mu.Lock()

	if portal.PortalConfig == nil {
		log.Println("Teleportation failed: Portal config is nil.")
		return
	}
	destMapID := portal.PortalConfig.DestMapID
	destPortalIndex := portal.PortalConfig.DestPortalIndex
	if destMapID >= len(allMapsPortals) || destPortalIndex >= len(allMapsPortals[destMapID]) {
		log.Printf("Teleportation failed: Destination portal index out of bounds. MapID: %d, PortalIndex: %d", destMapID, destPortalIndex)
		return
	}
	destPortal := allMapsPortals[destMapID][destPortalIndex]
	destMapState, ok := s.maps[destMapID]
	if !ok {
		log.Printf("Teleportation failed: Destination map %d not found.", destMapID)
		return
	}
	rand.Seed(time.Now().UnixNano())
	spawnX := destPortal.Pos.X + (destPortal.Dims.Width / 2) + rand.Float64()*portal.PortalConfig.SpawnRadius*2 - portal.PortalConfig.SpawnRadius
	spawnY := destPortal.Pos.Y + (destPortal.Dims.Height / 2) + rand.Float64()*portal.PortalConfig.SpawnRadius*2 - portal.PortalConfig.SpawnRadius
	destPosI, err := destMapState.pathfinder.findClosestWalkablePoint(PointI{X: int(spawnX), Y: int(spawnY)}, player.Dims)
	if err != nil {
		log.Printf("Could not find a walkable spawn point: %v", err)
		return
	}
	currentMapState, ok := s.maps[player.MapID]
	if ok {
		delete(currentMapState.players, player.ID)
	}
	player.MapID = destMapID
	player.Pos = Point{X: float64(destPosI.X), Y: float64(destPosI.Y)}
	player.TargetPos = PointI{}
	player.Path = []PointI{}
	player.Mode = TELEPORTING
	player.OnPortal = false
	destMapState.players[player.ID] = player
	s.sendAOI(player)
}

// ---------- AOI and state push ----------
func (s *GameServer) updateAOIs(mapState *MapState) {
	for _, player := range mapState.players {
		player.AOI = Rectangle{
			MinX: player.Pos.X - s.aoiRadius,
			MinY: player.Pos.Y - s.aoiRadius,
			MaxX: player.Pos.X + player.Dims.Width + s.aoiRadius,
			MaxY: player.Pos.Y + player.Dims.Height + s.aoiRadius,
		}
		s.sendAOI(player)
	}
}

func (s *GameServer) sendAOI(player *PlayerState) {
	mapState, ok := s.maps[player.MapID]
	if !ok {
		log.Printf("Map %d not found for player %s.", player.MapID, player.ID)
		return
	}

	visiblePlayers := make(map[string]VisiblePlayer)
	for _, otherPlayer := range mapState.players {
		if otherPlayer.ID == player.ID {
			continue
		}
		otherRect := Rectangle{MinX: otherPlayer.Pos.X, MinY: otherPlayer.Pos.Y, MaxX: otherPlayer.Pos.X + otherPlayer.Dims.Width, MaxY: otherPlayer.Pos.Y + otherPlayer.Dims.Height}
		if rectsOverlap(player.AOI, otherRect) {
			playerData := VisiblePlayer{
				ID:           otherPlayer.ID,
				Pos:          otherPlayer.Pos,
				Dims:         otherPlayer.Dims,
				Type:         "player",
				Direction:    otherPlayer.Direction,
				Mode:         otherPlayer.Mode,
				ObjectLayers: otherPlayer.ObjectLayers,
				Life:         otherPlayer.Life,
				MaxLife:      otherPlayer.MaxLife,
			}
			if otherPlayer.IsGhost() {
				remaining := time.Until(otherPlayer.RespawnTime).Seconds()
				if remaining > 0 {
					respawnIn := math.Ceil(remaining)
					playerData.RespawnIn = &respawnIn
				}
			}
			visiblePlayers[otherPlayer.ID] = playerData
		}
	}

	visibleGridObjects := make(map[string]interface{})
	for _, obstacle := range mapState.obstacles {
		obstacleRect := Rectangle{MinX: obstacle.Pos.X, MinY: obstacle.Pos.Y, MaxX: obstacle.Pos.X + obstacle.Dims.Width, MaxY: obstacle.Pos.Y + obstacle.Dims.Height}
		if rectsOverlap(player.AOI, obstacleRect) {
			visibleGridObjects[obstacle.ID] = obstacle
		}
	}
	for _, floor := range mapState.floors {
		floorRect := Rectangle{MinX: floor.Pos.X, MinY: floor.Pos.Y, MaxX: floor.Pos.X + floor.Dims.Width, MaxY: floor.Pos.Y + floor.Dims.Height}
		if rectsOverlap(player.AOI, floorRect) {
			visibleGridObjects[floor.ID] = VisibleFloor{
				ID:           floor.ID,
				Pos:          floor.Pos,
				Dims:         floor.Dims,
				Type:         "floor",
				ObjectLayers: floor.ObjectLayers,
			}
		}
	}
	for _, portal := range mapState.portals {
		portalRect := Rectangle{MinX: portal.Pos.X, MinY: portal.Pos.Y, MaxX: portal.Pos.X + portal.Dims.Width, MaxY: portal.Pos.Y + portal.Dims.Height}
		if rectsOverlap(player.AOI, portalRect) {
			visibleGridObjects[portal.ID] = ObjectState{
				ID:          portal.ID,
				Pos:         portal.Pos,
				Dims:        portal.Dims,
				Type:        "portal",
				PortalLabel: portal.Label,
			}
		}
	}
	for _, fg := range mapState.foregrounds {
		fgRect := Rectangle{MinX: fg.Pos.X, MinY: fg.Pos.Y, MaxX: fg.Pos.X + fg.Dims.Width, MaxY: fg.Pos.Y + fg.Dims.Height}
		if rectsOverlap(player.AOI, fgRect) {
			visibleGridObjects[fg.ID] = fg
		}
	}
	for _, bot := range mapState.bots {
		botRect := Rectangle{MinX: bot.Pos.X, MinY: bot.Pos.Y, MaxX: bot.Pos.X + bot.Dims.Width, MaxY: bot.Pos.Y + bot.Dims.Height}
		if rectsOverlap(player.AOI, botRect) {
			botData := VisibleBot{
				ID:           bot.ID,
				Pos:          bot.Pos,
				Dims:         bot.Dims,
				Type:         "bot",
				Behavior:     bot.Behavior,
				Direction:    bot.Direction,
				Mode:         bot.Mode,
				Life:         bot.Life,
				MaxLife:      bot.MaxLife,
				ObjectLayers: bot.ObjectLayers,
			}
			if bot.IsGhost() {
				remaining := time.Until(bot.RespawnTime).Seconds()
				if remaining > 0 {
					respawnIn := math.Ceil(remaining)
					botData.RespawnIn = &respawnIn
				}
			}
			visibleGridObjects[bot.ID] = botData
		}
	}

	playerObj := PlayerObject{
		ID:             player.ID,
		MapID:          player.MapID,
		Pos:            player.Pos,
		Dims:           player.Dims,
		Path:           player.Path,
		TargetPos:      player.TargetPos,
		AOI:            player.AOI,
		Direction:      player.Direction,
		Mode:           player.Mode,
		OnPortal:       player.OnPortal,
		ActivePortalID: player.ActivePortalID,
		Life:           player.Life,
		MaxLife:        player.MaxLife,
		SumStatsLimit:  player.SumStatsLimit,
		ObjectLayers:   player.ObjectLayers,
	}
	if player.IsGhost() {
		remaining := time.Until(player.RespawnTime).Seconds()
		if remaining > 0 {
			respawnIn := math.Ceil(remaining)
			playerObj.RespawnIn = &respawnIn
		}
	}

	payloadMap := AOIUpdatePayload{
		PlayerID:           player.ID,
		Player:             playerObj,
		VisiblePlayers:     visiblePlayers,
		VisibleGridObjects: visibleGridObjects,
	}

	message, err := json.Marshal(map[string]interface{}{"type": "aoi_update", "payload": payloadMap})
	if err != nil {
		log.Printf("Error marshaling AOI update: %v", err)
		return
	}
	select {
	case player.Client.send <- message:
	default:
		log.Printf("Client %s message channel is full.", player.ID)
	}
}

// ===== Metrics Methods =====

// GetEntityCounts returns counts of all entity types connected to the game server
func (s *GameServer) GetEntityCounts() map[string]int {
	s.mu.Lock()
	defer s.mu.Unlock()

	counts := map[string]int{
		"players":     0,
		"bots":        0,
		"floors":      0,
		"obstacles":   0,
		"foregrounds": 0,
		"portals":     0,
		"total":       0,
	}

	for _, mapState := range s.maps {
		counts["players"] += len(mapState.players)
		counts["bots"] += len(mapState.bots)
		counts["floors"] += len(mapState.floors)
		counts["obstacles"] += len(mapState.obstacles)
		counts["foregrounds"] += len(mapState.foregrounds)
		counts["portals"] += len(mapState.portals)
	}

	counts["total"] = counts["players"] + counts["bots"] + counts["floors"] +
		counts["obstacles"] + counts["foregrounds"] + counts["portals"]

	return counts
}

// GetConnectedClientsCount returns the number of currently connected WebSocket clients
func (s *GameServer) GetConnectedClientsCount() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	return len(s.clients)
}

// GetTotalObjectLayers returns the total count of object layers across all entities
func (s *GameServer) GetTotalObjectLayers() int {
	s.mu.Lock()
	defer s.mu.Unlock()

	total := 0
	for _, mapState := range s.maps {
		for _, player := range mapState.players {
			total += len(player.ObjectLayers)
		}
		for _, bot := range mapState.bots {
			total += len(bot.ObjectLayers)
		}
		for _, floor := range mapState.floors {
			total += len(floor.ObjectLayers)
		}
	}
	return total
}

// GetActiveObjectLayers returns the count of active object layers
func (s *GameServer) GetActiveObjectLayers() int {
	s.mu.Lock()
	defer s.mu.Unlock()

	total := 0
	for _, mapState := range s.maps {
		for _, player := range mapState.players {
			for _, layer := range player.ObjectLayers {
				if layer.Active {
					total++
				}
			}
		}
		for _, bot := range mapState.bots {
			for _, layer := range bot.ObjectLayers {
				if layer.Active {
					total++
				}
			}
		}
		for _, floor := range mapState.floors {
			for _, layer := range floor.ObjectLayers {
				if layer.Active {
					total++
				}
			}
		}
	}
	return total
}

// GetInactiveObjectLayers returns the count of inactive object layers
func (s *GameServer) GetInactiveObjectLayers() int {
	s.mu.Lock()
	defer s.mu.Unlock()

	total := 0
	for _, mapState := range s.maps {
		for _, player := range mapState.players {
			for _, layer := range player.ObjectLayers {
				if !layer.Active {
					total++
				}
			}
		}
		for _, bot := range mapState.bots {
			for _, layer := range bot.ObjectLayers {
				if !layer.Active {
					total++
				}
			}
		}
		for _, floor := range mapState.floors {
			for _, layer := range floor.ObjectLayers {
				if !layer.Active {
					total++
				}
			}
		}
	}
	return total
}

// GetMapStats returns statistics for a specific map
func (s *GameServer) GetMapStats(mapID int) map[string]int {
	s.mu.Lock()
	defer s.mu.Unlock()

	stats := map[string]int{
		"players":     0,
		"bots":        0,
		"floors":      0,
		"obstacles":   0,
		"foregrounds": 0,
		"portals":     0,
	}

	mapState, ok := s.maps[mapID]
	if !ok {
		return stats
	}

	stats["players"] = len(mapState.players)
	stats["bots"] = len(mapState.bots)
	stats["floors"] = len(mapState.floors)
	stats["obstacles"] = len(mapState.obstacles)
	stats["foregrounds"] = len(mapState.foregrounds)
	stats["portals"] = len(mapState.portals)

	return stats
}

// GetServerHealth returns basic server health information
func (s *GameServer) GetServerHealth() map[string]interface{} {
	s.mu.Lock()
	defer s.mu.Unlock()

	health := map[string]interface{}{
		"running":              true,
		"connected_clients":    len(s.clients),
		"total_maps":           len(s.maps),
		"total_entities":       0,
		"total_object_layers":  0,
		"active_object_layers": 0,
	}

	totalEntities := 0
	totalObjLayers := 0
	activeObjLayers := 0

	for _, mapState := range s.maps {
		totalEntities += len(mapState.players) + len(mapState.bots) + len(mapState.floors) +
			len(mapState.obstacles) + len(mapState.foregrounds) + len(mapState.portals)

		for _, player := range mapState.players {
			totalObjLayers += len(player.ObjectLayers)
			for _, layer := range player.ObjectLayers {
				if layer.Active {
					activeObjLayers++
				}
			}
		}
		for _, bot := range mapState.bots {
			totalObjLayers += len(bot.ObjectLayers)
			for _, layer := range bot.ObjectLayers {
				if layer.Active {
					activeObjLayers++
				}
			}
		}
		for _, floor := range mapState.floors {
			totalObjLayers += len(floor.ObjectLayers)
			for _, layer := range floor.ObjectLayers {
				if layer.Active {
					activeObjLayers++
				}
			}
		}
	}

	health["total_entities"] = totalEntities
	health["total_object_layers"] = totalObjLayers
	health["active_object_layers"] = activeObjLayers

	return health
}

// GetPlayerObjectLayers returns object layer counts for all players
// Active = ObjectLayerState.Active == true
// Inactive = ObjectLayerState.Active == false
func (s *GameServer) GetPlayerObjectLayers() ObjectLayerCount {
	s.mu.Lock()
	defer s.mu.Unlock()

	count := ObjectLayerCount{Total: 0, Active: 0, Inactive: 0}

	for _, mapState := range s.maps {
		for _, player := range mapState.players {
			for _, layer := range player.ObjectLayers {
				count.Total++
				if layer.Active {
					count.Active++
				} else {
					count.Inactive++
				}
			}
		}
	}
	return count
}

// GetBotObjectLayers returns object layer counts for all bots
// Active = ObjectLayerState.Active == true
// Inactive = ObjectLayerState.Active == false
func (s *GameServer) GetBotObjectLayers() ObjectLayerCount {
	s.mu.Lock()
	defer s.mu.Unlock()

	count := ObjectLayerCount{Total: 0, Active: 0, Inactive: 0}

	for _, mapState := range s.maps {
		for _, bot := range mapState.bots {
			for _, layer := range bot.ObjectLayers {
				count.Total++
				if layer.Active {
					count.Active++
				} else {
					count.Inactive++
				}
			}
		}
	}
	return count
}

// GetFloorObjectLayers returns object layer counts for all floors
// Active = ObjectLayerState.Active == true
// Inactive = ObjectLayerState.Active == false
func (s *GameServer) GetFloorObjectLayers() ObjectLayerCount {
	s.mu.Lock()
	defer s.mu.Unlock()

	count := ObjectLayerCount{Total: 0, Active: 0, Inactive: 0}

	for _, mapState := range s.maps {
		for _, floor := range mapState.floors {
			for _, layer := range floor.ObjectLayers {
				count.Total++
				if layer.Active {
					count.Active++
				} else {
					count.Inactive++
				}
			}
		}
	}
	return count
}

// ObjectLayerCount is a helper struct returned by object layer counting methods
type ObjectLayerCount struct {
	Total    int
	Active   int
	Inactive int
}
