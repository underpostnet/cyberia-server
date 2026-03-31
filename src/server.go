package game

import (
	"log"
	"math"
	"math/rand"
	"time"

	pb "cyberia-server/proto"
)

// NewGameServer creates an empty game server with no hardcoded defaults.
// All configuration must be applied via ApplyInstanceConfig from gRPC data.
func NewGameServer() *GameServer {
	gs := &GameServer{
		maps:                 make(map[int]*MapState),
		clients:              make(map[string]*Client),
		register:             make(chan *Client),
		unregister:           make(chan *Client),
		objectLayerDataCache: make(map[string]*ObjectLayer),
		atlasDataCache:       make(map[string]*AtlasData),
		colors:               make(map[string]ColorRGBA),
		skillConfig:          make(map[string][]SkillDefinition),
	}
	return gs
}

// ApplyInstanceConfig applies the gRPC InstanceConfig to the game server.
// This replaces all hardcoded defaults — called during LoadAll and hot-reload.
func (s *GameServer) ApplyInstanceConfig(cfg *pb.InstanceConfig) {
	if cfg == nil {
		log.Println("[GameServer] WARNING: ApplyInstanceConfig called with nil config")
		return
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	// Rendering / camera
	s.cellSize = cfg.GetCellSize()
	s.fps = int(cfg.GetFps())
	s.interpolationMs = int(cfg.GetInterpolationMs())
	s.defaultObjWidth = cfg.GetDefaultObjWidth()
	s.defaultObjHeight = cfg.GetDefaultObjHeight()
	s.cameraSmoothing = cfg.GetCameraSmoothing()
	s.cameraZoom = cfg.GetCameraZoom()
	s.defaultWidthScreenFactor = cfg.GetDefaultWidthScreenFactor()
	s.defaultHeightScreenFactor = cfg.GetDefaultHeightScreenFactor()
	s.devUi = cfg.GetDevUi()

	// Colors
	s.colors = make(map[string]ColorRGBA, len(cfg.GetColors()))
	for _, c := range cfg.GetColors() {
		s.colors[c.GetKey()] = ColorRGBA{
			R: int(c.GetR()),
			G: int(c.GetG()),
			B: int(c.GetB()),
			A: int(c.GetA()),
		}
	}

	// World / AOI
	s.aoiRadius = cfg.GetAoiRadius()
	s.portalHoldTime = time.Duration(cfg.GetPortalHoldTimeMs()) * time.Millisecond
	s.portalSpawnRadius = cfg.GetPortalSpawnRadius()

	// Entity base stats
	s.entityBaseSpeed = cfg.GetEntityBaseSpeed()
	s.entityBaseMaxLife = cfg.GetEntityBaseMaxLife()
	s.entityBaseActionCooldown = time.Duration(cfg.GetEntityBaseActionCooldownMs()) * time.Millisecond
	s.entityBaseMinActionCooldown = time.Duration(cfg.GetEntityBaseMinActionCooldownMs()) * time.Millisecond

	// Bot defaults
	s.botAggroRange = cfg.GetBotAggroRange()

	// Player defaults
	s.defaultPlayerWidth = cfg.GetDefaultPlayerWidth()
	s.defaultPlayerHeight = cfg.GetDefaultPlayerHeight()
	s.playerBaseLifeRegenMin = cfg.GetPlayerBaseLifeRegenMin()
	s.playerBaseLifeRegenMax = cfg.GetPlayerBaseLifeRegenMax()
	s.sumStatsLimit = int(cfg.GetSumStatsLimit())
	s.maxActiveLayers = int(cfg.GetMaxActiveLayers())
	s.initialLifeFraction = cfg.GetInitialLifeFraction()

	// Default player object layers
	s.defaultPlayerObjectLayers = make([]ObjectLayerState, 0, len(cfg.GetDefaultPlayerObjectLayers()))
	for _, ol := range cfg.GetDefaultPlayerObjectLayers() {
		s.defaultPlayerObjectLayers = append(s.defaultPlayerObjectLayers, ObjectLayerState{
			ItemID:   ol.GetItemId(),
			Active:   ol.GetActive(),
			Quantity: int(ol.GetQuantity()),
		})
	}

	// Combat / death
	s.respawnDuration = time.Duration(cfg.GetRespawnDurationMs()) * time.Millisecond
	s.ghostItemID = cfg.GetGhostItemId()
	s.collisionLifeLoss = cfg.GetCollisionLifeLoss()

	// Economy
	s.coinItemID = cfg.GetCoinItemId()
	s.defaultCoinQuantity = int(cfg.GetDefaultCoinQuantity())

	// Regen
	s.lifeRegenChance = cfg.GetLifeRegenChance()
	s.maxChance = cfg.GetMaxChance()

	// Skill params
	s.bulletSpawnChance = cfg.GetBulletSpawnChance()
	s.bulletLifetimeMs = int(cfg.GetBulletLifetimeMs())
	s.bulletWidth = cfg.GetBulletWidth()
	s.bulletHeight = cfg.GetBulletHeight()
	s.bulletSpeedMultiplier = cfg.GetBulletSpeedMultiplier()
	s.doppelgangerSpawnChance = cfg.GetDoppelgangerSpawnChance()
	s.doppelgangerLifetimeMs = int(cfg.GetDoppelgangerLifetimeMs())
	s.doppelgangerSpawnRadius = cfg.GetDoppelgangerSpawnRadius()
	s.doppelgangerInitialLifeFraction = cfg.GetDoppelgangerInitialLifeFraction()

	// Floor defaults
	s.defaultFloorItemID = cfg.GetDefaultFloorItemId()

	// Player color (fallback when no object layers / sprites)
	if pc := cfg.GetDefaultPlayerColor(); pc != nil {
		s.defaultPlayerColor = ColorRGBA{
			R: int(pc.GetR()),
			G: int(pc.GetG()),
			B: int(pc.GetB()),
			A: int(pc.GetA()),
		}
	} else {
		s.defaultPlayerColor = ColorRGBA{R: 0, G: 255, B: 0, A: 255}
	}

	// Skill map
	s.skillConfig = make(map[string][]SkillDefinition, len(cfg.GetSkillConfig()))
	for _, sc := range cfg.GetSkillConfig() {
		s.skillConfig[sc.GetTriggerItemId()] = append(s.skillConfig[sc.GetTriggerItemId()], SkillDefinition{
			ItemIDs:      sc.GetSpawnedItemIds(),
			LogicEventID: sc.GetLogicEventId(),
		})
	}

	log.Printf("[GameServer] Instance config applied: cellSize=%.1f, fps=%d, aoiRadius=%.1f, entityBaseSpeed=%.1f, entityBaseMaxLife=%.1f, %d colors, %d skills, %d default player layers",
		s.cellSize, s.fps, s.aoiRadius, s.entityBaseSpeed, s.entityBaseMaxLife, len(s.colors), len(s.skillConfig), len(s.defaultPlayerObjectLayers))
}

// ReplaceObjectLayerCache atomically replaces the entire cache.
// Used by WorldBuilder for initial full load via gRPC.
func (s *GameServer) ReplaceObjectLayerCache(cache map[string]*ObjectLayer) {
	s.olMu.Lock()
	defer s.olMu.Unlock()
	s.objectLayerDataCache = cache

	typeCounts := make(map[string]int)
	for _, layer := range cache {
		itemType := layer.Data.Item.Type
		if itemType == "" {
			itemType = "(unknown)"
		}
		typeCounts[itemType]++
	}
	log.Printf("Object layer cache replaced with %d items.", len(cache))
	for itemType, count := range typeCounts {
		log.Printf("  %-20s %d", itemType, count)
	}
}

// PatchObjectLayerCache applies incremental updates and deletions.
// Used by WorldBuilder for hot-reload via gRPC manifest diffing.
func (s *GameServer) PatchObjectLayerCache(updates map[string]*ObjectLayer, deletions []string) {
	s.olMu.Lock()
	defer s.olMu.Unlock()
	for itemID, ol := range updates {
		s.objectLayerDataCache[itemID] = ol
	}
	for _, itemID := range deletions {
		delete(s.objectLayerDataCache, itemID)
	}
}

// ReplaceAtlasCache atomically replaces the atlas sprite sheet cache.
func (s *GameServer) ReplaceAtlasCache(cache map[string]*AtlasData) {
	s.olMu.Lock()
	defer s.olMu.Unlock()
	s.atlasDataCache = cache
	log.Printf("Atlas data cache replaced with %d items.", len(cache))
}

// GetObjectLayerData returns an ObjectLayer by item ID (read-locked).
func (s *GameServer) GetObjectLayerData(itemID string) (*ObjectLayer, bool) {
	s.olMu.RLock()
	defer s.olMu.RUnlock()
	ol, ok := s.objectLayerDataCache[itemID]
	return ol, ok
}

// SetEngineApiBaseUrl sets the Engine API base URL forwarded to clients.
func (s *GameServer) SetEngineApiBaseUrl(url string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.engineApiBaseUrl = url
	log.Printf("Engine API base URL set to: %s", url)
}

// EnsurePlayableState creates a minimal empty map with default config
// if no maps were loaded. This prevents panics on player connect when
// gRPC is unavailable or returns no instance data.
func (s *GameServer) EnsurePlayableState() {
	s.mu.Lock()
	defer s.mu.Unlock()

	if len(s.maps) > 0 {
		return
	}

	log.Println("[GameServer] No maps loaded — creating fallback empty map.")

	// Apply minimal defaults if no config was applied
	if s.entityBaseMaxLife <= 0 {
		s.cellSize = 32
		s.fps = 10
		s.interpolationMs = 100
		s.defaultObjWidth = 1
		s.defaultObjHeight = 1
		s.cameraSmoothing = 0.1
		s.cameraZoom = 1.0
		s.defaultWidthScreenFactor = 1
		s.defaultHeightScreenFactor = 1
		s.aoiRadius = 300
		s.entityBaseSpeed = 200
		s.entityBaseMaxLife = 100
		s.entityBaseActionCooldown = 500 * time.Millisecond
		s.entityBaseMinActionCooldown = 100 * time.Millisecond
		s.defaultPlayerWidth = 1
		s.defaultPlayerHeight = 1
		s.playerBaseLifeRegenMin = 0.5
		s.playerBaseLifeRegenMax = 1.5
		s.sumStatsLimit = 500
		s.maxActiveLayers = 4
		s.initialLifeFraction = 1.0
		s.respawnDuration = 3 * time.Second
		s.collisionLifeLoss = 10
		s.lifeRegenChance = 300
		s.maxChance = 10000
		s.defaultCoinQuantity = 1
		s.portalHoldTime = time.Second
		s.portalSpawnRadius = 3
		s.defaultPlayerColor = ColorRGBA{R: 0, G: 255, B: 0, A: 255}
		s.colors = map[string]ColorRGBA{
			"background": {R: 30, G: 30, B: 30, A: 255},
			"obstacle":   {R: 80, G: 80, B: 80, A: 255},
		}
	}

	gridW, gridH := 16, 16
	ms := &MapState{
		gridW:       gridW,
		gridH:       gridH,
		players:     make(map[string]*PlayerState),
		portals:     make(map[string]*PortalState),
		obstacles:   make(map[string]ObjectState),
		foregrounds: make(map[string]ObjectState),
		floors:      make(map[string]*FloorState),
		pathfinder:  NewPathfinder(gridW, gridH),
		bots:        make(map[string]*BotState),
	}

	// Generate floors and obstacles
	ms.generateFloors(10, 10, s.defaultFloorItemID)
	ms.pathfinder.GenerateObstacles(100, nil)
	ms.obstacles = ms.pathfinder.obstacles

	s.maps[0] = ms
	log.Printf("[GameServer] Fallback map created: %dx%d grid, %d floors, %d obstacles.",
		gridW, gridH, len(ms.floors), len(ms.obstacles))
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
	destMapState, ok := s.maps[destMapID]
	if !ok {
		log.Printf("Teleportation failed: Destination map %d not found.", destMapID)
		return
	}

	// Find a walkable spawn point near the center of the destination map
	spawnX := float64(destMapState.gridW) / 2.0
	spawnY := float64(destMapState.gridH) / 2.0
	spawnX += rand.Float64()*portal.PortalConfig.SpawnRadius*2 - portal.PortalConfig.SpawnRadius
	spawnY += rand.Float64()*portal.PortalConfig.SpawnRadius*2 - portal.PortalConfig.SpawnRadius

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

	message := s.EncodeBinaryAOI(player, mapState)
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
