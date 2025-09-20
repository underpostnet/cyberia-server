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
		maps:           make(map[int]*MapState),
		clients:        make(map[string]*Client),
		register:       make(chan *Client),
		unregister:     make(chan *Client),
		aoiRadius:      15.0,
		portalHoldTime: 2 * time.Second,
		playerSpeed:    24.0,

		cellSize:         12.0,
		fps:              60,
		interpolationMs:  200,
		defaultObjWidth:  1.0,
		defaultObjHeight: 1.0,
		cameraSmoothing:  0.15,
		cameraZoom:       2,
		// production immersive settings
		// defaultWidthScreenFactor:  0.9,
		// defaultHeightScreenFactor: 0.9,
		// devUi:                     false,
		defaultWidthScreenFactor:  0.1,
		defaultHeightScreenFactor: 0.9,
		devUi:                     true,
		botsPerMap:                10,
		botAggroRange:             10.0,
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

	if player.Mode == WALKING && len(player.Path) > 0 {
		targetNode := player.Path[0]
		dx := float64(targetNode.X) - player.Pos.X
		dy := float64(targetNode.Y) - player.Pos.Y
		dist := math.Sqrt(dx*dx + dy*dy)
		step := s.playerSpeed / 10.0

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
	visiblePlayersMap := make(map[string]map[string]interface{})
	for _, otherPlayer := range mapState.players {
		if otherPlayer.ID == player.ID {
			continue
		}
		otherRect := Rectangle{MinX: otherPlayer.Pos.X, MinY: otherPlayer.Pos.Y, MaxX: otherPlayer.Pos.X + otherPlayer.Dims.Width, MaxY: otherPlayer.Pos.Y + otherPlayer.Dims.Height}
		if rectsOverlap(player.AOI, otherRect) {
			playerData := map[string]interface{}{
				"id":           otherPlayer.ID,
				"Pos":          otherPlayer.Pos,
				"Dims":         otherPlayer.Dims,
				"Type":         "player",
				"direction":    int(otherPlayer.Direction),
				"mode":         int(otherPlayer.Mode),
				"objectLayers": otherPlayer.ObjectLayers,
				"life":         otherPlayer.Life,
				"maxLife":      otherPlayer.MaxLife,
			}
			if otherPlayer.IsGhost() {
				remaining := time.Until(otherPlayer.RespawnTime).Seconds()
				if remaining > 0 {
					playerData["respawnIn"] = math.Ceil(remaining)
				}
			}
			visiblePlayersMap[otherPlayer.ID] = playerData
		}
	}
	visibleGridObjectsMap := make(map[string]map[string]interface{})
	for _, obstacle := range mapState.obstacles {
		obstacleRect := Rectangle{MinX: obstacle.Pos.X, MinY: obstacle.Pos.Y, MaxX: obstacle.Pos.X + obstacle.Dims.Width, MaxY: obstacle.Pos.Y + obstacle.Dims.Height}
		if rectsOverlap(player.AOI, obstacleRect) {
			visibleGridObjectsMap[obstacle.ID] = map[string]interface{}{"id": obstacle.ID, "Pos": obstacle.Pos, "Dims": obstacle.Dims, "Type": obstacle.Type}
		}
	}
	for _, floor := range mapState.floors {
		floorRect := Rectangle{MinX: floor.Pos.X, MinY: floor.Pos.Y, MaxX: floor.Pos.X + floor.Dims.Width, MaxY: floor.Pos.Y + floor.Dims.Height}
		if rectsOverlap(player.AOI, floorRect) {
			visibleGridObjectsMap[floor.ID] = map[string]interface{}{
				"id":           floor.ID,
				"Pos":          floor.Pos,
				"Dims":         floor.Dims,
				"Type":         "floor",
				"objectLayers": floor.ObjectLayers,
			}
		}
	}
	for _, portal := range mapState.portals {
		portalRect := Rectangle{MinX: portal.Pos.X, MinY: portal.Pos.Y, MaxX: portal.Pos.X + portal.Dims.Width, MaxY: portal.Pos.Y + portal.Dims.Height}
		if rectsOverlap(player.AOI, portalRect) {
			visibleGridObjectsMap[portal.ID] = map[string]interface{}{
				"id":          portal.ID,
				"Pos":         portal.Pos,
				"Dims":        portal.Dims,
				"Type":        "portal",
				"PortalLabel": portal.Label,
			}
		}
	}
	for _, fg := range mapState.foregrounds {
		fgRect := Rectangle{MinX: fg.Pos.X, MinY: fg.Pos.Y, MaxX: fg.Pos.X + fg.Dims.Width, MaxY: fg.Pos.Y + fg.Dims.Height}
		if rectsOverlap(player.AOI, fgRect) {
			visibleGridObjectsMap[fg.ID] = map[string]interface{}{"id": fg.ID, "Pos": fg.Pos, "Dims": fg.Dims, "Type": "foreground"}
		}
	}
	for _, bot := range mapState.bots {
		botRect := Rectangle{MinX: bot.Pos.X, MinY: bot.Pos.Y, MaxX: bot.Pos.X + bot.Dims.Width, MaxY: bot.Pos.Y + bot.Dims.Height}
		if rectsOverlap(player.AOI, botRect) {
			botData := map[string]interface{}{
				"id":           bot.ID,
				"Pos":          bot.Pos,
				"Dims":         bot.Dims,
				"Type":         "bot",
				"behavior":     bot.Behavior,
				"direction":    int(bot.Direction),
				"mode":         int(bot.Mode),
				"life":         bot.Life,
				"maxLife":      bot.MaxLife,
				"objectLayers": bot.ObjectLayers,
			}
			if bot.IsGhost() {
				remaining := time.Until(bot.RespawnTime).Seconds()
				if remaining > 0 {
					botData["respawnIn"] = math.Ceil(remaining)
				}
			}
			visibleGridObjectsMap[bot.ID] = botData
		}
	}
	playerObj := map[string]interface{}{
		"id":             player.ID,
		"MapID":          player.MapID,
		"Pos":            player.Pos,
		"Dims":           player.Dims,
		"path":           player.Path,
		"targetPos":      player.TargetPos,
		"AOI":            player.AOI,
		"direction":      int(player.Direction),
		"mode":           int(player.Mode),
		"onPortal":       player.OnPortal,
		"activePortalID": player.ActivePortalID,
		"life":           player.Life,
		"maxLife":        player.MaxLife,
		"sumStatsLimit":  player.SumStatsLimit,
		"objectLayers":   player.ObjectLayers,
	}
	if player.IsGhost() {
		remaining := time.Until(player.RespawnTime).Seconds()
		if remaining > 0 {
			playerObj["respawnIn"] = math.Ceil(remaining)
		}
	}
	payloadMap := map[string]interface{}{
		"playerID":           player.ID,
		"player":             playerObj,
		"visiblePlayers":     visiblePlayersMap,
		"visibleGridObjects": visibleGridObjectsMap,
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
