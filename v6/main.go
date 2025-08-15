package main

import (
	"container/heap"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"math/rand"
	"net/http"
	"sync"
	"time"

	"github.com/google/uuid"
	"github.com/gorilla/websocket"
)

//----------------------------------------------------------------------------------------------------------------------
// 1. Data Structures & Interfaces
//----------------------------------------------------------------------------------------------------------------------

type Point struct {
	X float64 `json:"X"`
	Y float64 `json:"Y"`
}

type PointI struct {
	X, Y int
}

type Rectangle struct {
	MinX, MinY, MaxX, MaxY float64
}

type Dimensions struct {
	Width  float64 `json:"Width"`
	Height float64 `json:"Height"`
}

type Direction int

const (
	UP Direction = iota
	UP_RIGHT
	RIGHT
	DOWN_RIGHT
	DOWN
	DOWN_LEFT
	LEFT
	UP_LEFT
	NONE
)

type ObjectLayerMode int

const (
	IDLE ObjectLayerMode = iota
	WALKING
)

type ObjectState struct {
	ID          string     `json:"id"`
	Pos         Point      `json:"Pos"`
	Dims        Dimensions `json:"Dims"`
	Type        string     `json:"Type"`
	PortalLabel string     `json:"PortalLabel,omitempty"`
}

type PlayerState struct {
	ID             string          `json:"id"`
	MapID          int             `json:"MapID"`
	Pos            Point           `json:"Pos"`
	Dims           Dimensions      `json:"Dims"`
	Path           []PointI        `json:"path"`
	TargetPos      PointI          `json:"targetPos"`
	AOI            Rectangle       `json:"AOI"`
	Client         *Client         `json:"-"` // Ignore for JSON marshaling
	Direction      Direction       `json:"direction"`
	Mode           ObjectLayerMode `json:"mode"`
	OnPortal       bool            `json:"onPortal"`
	TimeOnPortal   time.Time       `json:"-"` // Ignore
	ActivePortalID string          `json:"activePortalID"`
}

type PortalConfig struct {
	DestMapID       int
	DestPortalIndex int
	SpawnRadius     float64
}

type PortalState struct {
	ID           string
	Pos          Point
	Dims         Dimensions
	PortalConfig *PortalConfig
}

type MapState struct {
	pathfinder   *Pathfinder
	obstacles    map[string]ObjectState
	portals      map[string]*PortalState
	players      map[string]*PlayerState
	gridW, gridH int
}

type AOIUpdatePayload struct {
	PlayerID           string                 `json:"playerID"`
	Player             PlayerState            `json:"player"`
	VisiblePlayers     map[string]ObjectState `json:"visiblePlayers"`
	VisibleGridObjects map[string]ObjectState `json:"visibleGridObjects"`
}

type Client struct {
	conn       *websocket.Conn
	playerID   string
	send       chan []byte
	lastAction time.Time
}

type GameServer struct {
	mu             sync.Mutex
	maps           map[int]*MapState
	clients        map[string]*Client
	register       chan *Client
	unregister     chan *Client
	aoiRadius      float64
	portalHoldTime time.Duration
	playerSpeed    float64
}

type Node struct {
	X, Y    int
	g, h, f float64
	parent  *Node
	index   int
}

type PriorityQueue []*Node

func (pq PriorityQueue) Len() int           { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool { return pq[i].f < pq[j].f }
func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}
func (pq *PriorityQueue) Push(x interface{}) {
	n := len(*pq)
	item := x.(*Node)
	item.index = n
	*pq = append(*pq, item)
}
func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	item.index = -1
	*pq = old[0 : n-1]
	return item
}

type Pathfinder struct {
	gridW, gridH int
	obstacles    map[string]ObjectState
	grid         [][]int
}

//----------------------------------------------------------------------------------------------------------------------
// 2. Core Logic & Methods
//----------------------------------------------------------------------------------------------------------------------

// NewPathfinder creates a new pathfinder with a grid of specified dimensions.
func NewPathfinder(w, h int) *Pathfinder {
	p := &Pathfinder{gridW: w, gridH: h}
	p.grid = make([][]int, h)
	for y := 0; y < h; y++ {
		p.grid[y] = make([]int, w)
	}
	return p
}

// GenerateObstacles places random obstacles on the map, avoiding specified portal rectangles.
func (pf *Pathfinder) GenerateObstacles(numObstacles int, portalRects []Rectangle) {
	pf.obstacles = make(map[string]ObjectState)
	for y := range pf.grid {
		for x := range pf.grid[y] {
			pf.grid[y][x] = 0
		}
	}

	maxObsW, maxObsH, minObsDim := int(math.Max(2, float64(pf.gridW)/15.0)), int(math.Max(2, float64(pf.gridH)/15.0)), 2
	for placed, attempts := 0, 0; placed < numObstacles && attempts < numObstacles*10; attempts++ {
		w, h := rand.Intn(maxObsW-minObsDim+1)+minObsDim, rand.Intn(maxObsH-minObsDim+1)+minObsDim
		minX, minY := rand.Intn(pf.gridW-w), rand.Intn(pf.gridH-h)
		obsRect := Rectangle{MinX: float64(minX), MinY: float64(minY), MaxX: float64(minX + w), MaxY: float64(minY + h)}

		overlap := false
		for _, pRect := range portalRects {
			if rectsOverlap(obsRect, pRect) {
				overlap = true
				break
			}
		}
		if overlap {
			continue
		}

		for _, existingObs := range pf.obstacles {
			existingRect := Rectangle{
				MinX: existingObs.Pos.X, MinY: existingObs.Pos.Y,
				MaxX: existingObs.Pos.X + existingObs.Dims.Width, MaxY: existingObs.Pos.Y + existingObs.Dims.Height,
			}
			if rectsOverlap(obsRect, existingRect) {
				overlap = true
				break
			}
		}
		if overlap {
			continue
		}

		obs := ObjectState{
			ID: uuid.New().String(), Pos: Point{X: float64(minX), Y: float64(minY)},
			Dims: Dimensions{Width: float64(w), Height: float64(h)}, Type: "obstacle",
		}
		for y := minY; y < minY+h; y++ {
			for x := minX; x < minX+w; x++ {
				if x >= 0 && x < pf.gridW && y >= 0 && y < pf.gridH {
					pf.grid[y][x] = 1
				}
			}
		}
		pf.obstacles[obs.ID] = obs
		placed++
	}
	log.Printf("Map generation complete. Placed %d obstacles.", len(pf.obstacles))
}

// generatePortals creates and places portals on a map.
func (ms *MapState) generatePortals(numPortals int) []*PortalState {
	portals := make([]*PortalState, 0, numPortals)
	for i := 0; i < numPortals; i++ {
		dims := Dimensions{Width: float64(rand.Intn(3) + 2), Height: float64(rand.Intn(3) + 2)}
		posI, err := ms.pathfinder.findRandomWalkablePoint(dims)
		if err != nil {
			log.Printf("Error placing portal: %v", err)
			continue
		}
		portal := &PortalState{
			ID: uuid.New().String(), Pos: Point{X: float64(posI.X), Y: float64(posI.Y)}, Dims: dims,
		}
		portals = append(portals, portal)
		for y := posI.Y; y < posI.Y+int(dims.Height); y++ {
			for x := posI.X; x < posI.X+int(dims.Width); x++ {
				if x >= 0 && x < ms.gridW && y >= 0 && y < ms.gridH {
					ms.pathfinder.grid[y][x] = 1
				}
			}
		}
	}
	return portals
}

// isCellWalkable checks if a single grid cell is walkable.
func (pf *Pathfinder) isCellWalkable(x, y int) bool {
	return x >= 0 && x < pf.gridW && y >= 0 && y < pf.gridH && pf.grid[y][x] == 0
}

// isWalkable checks if a rectangle of a given dimension is walkable.
func (pf *Pathfinder) isWalkable(x, y int, playerDims Dimensions) bool {
	w, h := int(math.Ceil(playerDims.Width)), int(math.Ceil(playerDims.Height))
	for dy := 0; dy < h; dy++ {
		for dx := 0; dx < w; dx++ {
			if !pf.isCellWalkable(x+dx, y+dy) {
				return false
			}
		}
	}
	return true
}

// findRandomWalkablePoint finds a random walkable point for a given dimension.
func (pf *Pathfinder) findRandomWalkablePoint(dims Dimensions) (PointI, error) {
	for attempts := 0; attempts < 200; attempts++ {
		x, y := rand.Intn(pf.gridW-int(dims.Width)), rand.Intn(pf.gridH-int(dims.Height))
		if pf.isWalkable(x, y, dims) {
			return PointI{X: x, Y: y}, nil
		}
	}
	return PointI{}, fmt.Errorf("could not find a walkable point")
}

// findClosestWalkablePoint finds the closest walkable point to a given point.
func (pf *Pathfinder) findClosestWalkablePoint(p PointI, playerDims Dimensions) (PointI, error) {
	if pf.isWalkable(p.X, p.Y, playerDims) {
		return p, nil
	}
	queue := []PointI{p}
	visited := make(map[PointI]bool)
	visited[p] = true

	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]
		if pf.isWalkable(current.X, current.Y, playerDims) {
			return current, nil
		}
		for _, move := range []PointI{{1, 0}, {-1, 0}, {0, 1}, {0, -1}} {
			neighbor := PointI{X: current.X + move.X, Y: current.Y + move.Y}
			if neighbor.X >= 0 && neighbor.X < pf.gridW && neighbor.Y >= 0 && neighbor.Y < pf.gridH && !visited[neighbor] {
				visited[neighbor] = true
				queue = append(queue, neighbor)
			}
		}
	}
	return PointI{}, fmt.Errorf("no walkable point found near (%d, %d)", p.X, p.Y)
}

// Astar performs the A* pathfinding algorithm.
func (pf *Pathfinder) Astar(start, end PointI, playerDims Dimensions) ([]PointI, error) {
	openSet := make(PriorityQueue, 0)
	heap.Init(&openSet)
	startNode := &Node{X: start.X, Y: start.Y, h: heuristic(start.X, start.Y, end.X, end.Y)}
	heap.Push(&openSet, startNode)
	gScore := make(map[PointI]float64)
	gScore[start] = 0.0

	for openSet.Len() > 0 {
		current := heap.Pop(&openSet).(*Node)
		if current.X == end.X && current.Y == end.Y {
			return reconstructPath(current), nil
		}
		for _, move := range []PointI{{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}} {
			neighborPos := PointI{X: current.X + move.X, Y: current.Y + move.Y}
			if !pf.isWalkable(neighborPos.X, neighborPos.Y, playerDims) {
				continue
			}
			tentativeGScore := gScore[PointI{X: current.X, Y: current.Y}] + 1
			if val, ok := gScore[neighborPos]; !ok || tentativeGScore < val {
				gScore[neighborPos] = tentativeGScore
				h := heuristic(neighborPos.X, neighborPos.Y, end.X, end.Y)
				heap.Push(&openSet, &Node{X: neighborPos.X, Y: neighborPos.Y, g: tentativeGScore, h: h, f: tentativeGScore + h, parent: current})
			}
		}
	}
	return nil, fmt.Errorf("could not find path")
}

// heuristic is the Manhattan distance heuristic for A*.
func heuristic(x1, y1, x2, y2 int) float64 {
	return math.Abs(float64(x1-x2)) + math.Abs(float64(y1-y2))
}

// reconstructPath reconstructs the path from the goal node.
func reconstructPath(node *Node) []PointI {
	path := make([]PointI, 0)
	for node != nil {
		path = append([]PointI{{X: node.X, Y: node.Y}}, path...)
		node = node.parent
	}
	return path
}

// rectsOverlap checks if two rectangles overlap.
func rectsOverlap(r1, r2 Rectangle) bool {
	return r1.MinX < r2.MaxX && r1.MaxX > r2.MinX && r1.MinY < r2.MaxY && r1.MaxY > r2.MinY
}

// NewGameServer creates and initializes a new GameServer.
func NewGameServer() *GameServer {
	rand.Seed(time.Now().UnixNano())
	server := &GameServer{
		clients:        make(map[string]*Client),
		maps:           make(map[int]*MapState),
		register:       make(chan *Client),
		unregister:     make(chan *Client),
		aoiRadius:      25.0,
		portalHoldTime: 5 * time.Second,
		playerSpeed:    7.0, // Increased player speed
	}

	// Portal configurations for each map
	portalConfigsMap1 := []PortalConfig{
		{DestMapID: 2, DestPortalIndex: 0, SpawnRadius: 15.0},
		{DestMapID: 1, DestPortalIndex: 1, SpawnRadius: 15.0},
	}
	portalConfigsMap2 := []PortalConfig{
		{DestMapID: 3, DestPortalIndex: 0, SpawnRadius: 15.0},
		{DestMapID: 1, DestPortalIndex: 0, SpawnRadius: 15.0},
		{DestMapID: 2, DestPortalIndex: 1, SpawnRadius: 15.0},
	}
	portalConfigsMap3 := []PortalConfig{
		{DestMapID: 1, DestPortalIndex: 0, SpawnRadius: 15.0},
		{DestMapID: 2, DestPortalIndex: 0, SpawnRadius: 15.0},
	}

	// Create and configure maps
	portalRects := make(map[int][]Rectangle)
	portals := make(map[int][]*PortalState)
	for i := 1; i <= 3; i++ {
		mapState := &MapState{
			gridW:      100,
			gridH:      100,
			pathfinder: NewPathfinder(100, 100),
			players:    make(map[string]*PlayerState),
			portals:    make(map[string]*PortalState),
		}

		// Generate portals first to avoid placing obstacles over them
		numPortals := rand.Intn(2) + 2
		portalList := mapState.generatePortals(numPortals)
		portalRects[i] = make([]Rectangle, len(portalList))
		portals[i] = portalList

		for idx, portal := range portalList {
			rect := Rectangle{MinX: portal.Pos.X, MinY: portal.Pos.Y, MaxX: portal.Pos.X + portal.Dims.Width, MaxY: portal.Pos.Y + portal.Dims.Height}
			portalRects[i][idx] = rect
			mapState.portals[portal.ID] = portal
		}

		mapState.pathfinder.GenerateObstacles(20, portalRects[i])
		server.maps[i] = mapState
	}

	// Assign portal configurations after all portals are generated
	for i := 1; i <= 3; i++ {
		portalList := portals[i]
		var configs []PortalConfig
		switch i {
		case 1:
			configs = portalConfigsMap1
		case 2:
			configs = portalConfigsMap2
		case 3:
			configs = portalConfigsMap3
		}
		for idx, portal := range portalList {
			if idx < len(configs) {
				portal.PortalConfig = &configs[idx]
			}
		}
	}
	log.Printf("Game server initialized with %d maps.", len(server.maps))
	return server
}

// run starts the game loop.
func (s *GameServer) run() {
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()
	for {
		select {
		case client := <-s.register:
			s.mu.Lock()
			playerDims := Dimensions{Width: float64(rand.Intn(2) + 2), Height: float64(rand.Intn(2) + 2)}
			startMapID := 1
			mapState := s.maps[startMapID]
			startPos, err := mapState.pathfinder.findRandomWalkablePoint(playerDims)
			if err != nil {
				log.Printf("Failed to place new player: %v", err)
				s.mu.Unlock()
				continue
			}
			newPlayer := &PlayerState{
				ID:        client.playerID,
				MapID:     startMapID,
				Pos:       Point{X: float64(startPos.X), Y: float64(startPos.Y)},
				Dims:      playerDims,
				Client:    client,
				Path:      []PointI{},
				TargetPos: PointI{},
				Direction: NONE,
				Mode:      IDLE,
			}
			s.clients[client.playerID] = client
			mapState.players[client.playerID] = newPlayer
			log.Printf("Client %s connected to map %d. Total clients: %d", client.playerID, newPlayer.MapID, len(s.clients))
			s.mu.Unlock()

			// Send initial data to the new client
			go func() {
				s.mu.Lock()
				mapState := s.maps[newPlayer.MapID]
				initData := map[string]interface{}{
					"type": "init_data",
					"payload": map[string]interface{}{
						"gridW":     mapState.gridW,
						"gridH":     mapState.gridH,
						"aoiRadius": s.aoiRadius,
					},
				}
				s.mu.Unlock()
				jsonMessage, _ := json.Marshal(initData)
				client.send <- jsonMessage
			}()

		case client := <-s.unregister:
			s.mu.Lock()
			if _, ok := s.clients[client.playerID]; ok {
				// Find which map the player is on before unregistering
				var player *PlayerState
				var mapState *MapState
				for _, m := range s.maps {
					if p, playerExists := m.players[client.playerID]; playerExists {
						player = p
						mapState = m
						break
					}
				}
				if player != nil && mapState != nil {
					delete(mapState.players, client.playerID)
					log.Printf("Removed player %s from map %d", player.ID, player.MapID)
				} else {
					log.Printf("Could not find player %s in any map during unregister", client.playerID)
				}

				delete(s.clients, client.playerID)
				close(client.send)
				log.Printf("Client %s disconnected. Total clients: %d", client.playerID, len(s.clients))
			}
			s.mu.Unlock()
		case <-ticker.C:
			s.mu.Lock()
			s.handlePlayerMovement()
			s.handlePlayerTeleport()
			s.updateClients()
			s.mu.Unlock()
		}
	}
}

// handlePlayerMovement updates player positions based on their path.
func (s *GameServer) handlePlayerMovement() {
	for _, mapState := range s.maps {
		for _, player := range mapState.players {
			if len(player.Path) > 0 {
				player.Mode = WALKING
				target := player.Path[0]
				dx, dy := float64(target.X)-player.Pos.X, float64(target.Y)-player.Pos.Y
				dist := math.Sqrt(dx*dx + dy*dy)

				if dist < 0.1 {
					player.Pos.X, player.Pos.Y = float64(target.X), float64(target.Y)
					player.Path = player.Path[1:]
					if len(player.Path) == 0 {
						player.Mode = IDLE
					}
					continue
				}

				angle := math.Atan2(dy, dx)
				player.Direction = getDirectionFromAngle(angle)

				speed := s.playerSpeed * 0.1 // Adjust speed to be frame rate independent
				if dist < speed {
					player.Pos.X, player.Pos.Y = float64(target.X), float64(target.Y)
				} else {
					player.Pos.X += math.Cos(angle) * speed
					player.Pos.Y += math.Sin(angle) * speed
				}
			}
		}
	}
}

// handlePlayerTeleport checks for players on portals and teleports them.
func (s *GameServer) handlePlayerTeleport() {
	for mapID, mapState := range s.maps {
		for _, player := range mapState.players {
			playerRect := Rectangle{MinX: player.Pos.X, MinY: player.Pos.Y, MaxX: player.Pos.X + player.Dims.Width, MaxY: player.Pos.Y + player.Dims.Height}
			onPortal := false
			for _, portal := range mapState.portals {
				portalRect := Rectangle{MinX: portal.Pos.X, MinY: portal.Pos.Y, MaxX: portal.Pos.X + portal.Dims.Width, MaxY: portal.Pos.Y + portal.Dims.Height}
				if rectsOverlap(playerRect, portalRect) {
					onPortal = true
					if !player.OnPortal || player.ActivePortalID != portal.ID {
						player.OnPortal = true
						player.TimeOnPortal = time.Now()
						player.ActivePortalID = portal.ID
					}
					break
				}
			}

			if !onPortal {
				player.OnPortal = false
				player.ActivePortalID = ""
			}

			if player.OnPortal && time.Since(player.TimeOnPortal) > s.portalHoldTime {
				s.teleportPlayer(player, mapID, mapState)
			}
		}
	}
}

// teleportPlayer teleports a player to a new map and position.
func (s *GameServer) teleportPlayer(player *PlayerState, currentMapID int, currentMap *MapState) {
	s.mu.Lock()
	defer s.mu.Unlock()

	portal, ok := currentMap.portals[player.ActivePortalID]
	if !ok || portal.PortalConfig == nil {
		log.Printf("Portal not found or not configured: %s", player.ActivePortalID)
		return
	}

	destMap, ok := s.maps[portal.PortalConfig.DestMapID]
	if !ok {
		log.Printf("Destination map %d not found", portal.PortalConfig.DestMapID)
		return
	}

	destPortals := make([]*PortalState, 0)
	for _, p := range destMap.portals {
		destPortals = append(destPortals, p)
	}

	if portal.PortalConfig.DestPortalIndex >= len(destPortals) {
		log.Printf("Destination portal index out of range on map %d", portal.PortalConfig.DestMapID)
		return
	}

	destPortal := destPortals[portal.PortalConfig.DestPortalIndex]

	// Remove player from current map
	delete(currentMap.players, player.ID)

	// Update player state for new map
	player.MapID = portal.PortalConfig.DestMapID
	player.Path = []PointI{} // Clear path
	player.TargetPos = PointI{}

	spawnPoint, err := destMap.pathfinder.findRandomWalkablePoint(player.Dims)
	if err != nil {
		log.Printf("Error finding spawn point in new map: %v", err)
		player.Pos = destPortal.Pos
	} else {
		player.Pos = Point{X: float64(spawnPoint.X), Y: float64(spawnPoint.Y)}
	}

	// Add player to the new map
	destMap.players[player.ID] = player
	log.Printf("Player %s teleported from map %d to map %d", player.ID, currentMapID, player.MapID)
}

// getDirectionFromAngle converts an angle in radians to a Direction enum.
func getDirectionFromAngle(angle float64) Direction {
	angle = math.Mod(angle+2*math.Pi, 2*math.Pi)
	if angle >= 7*math.Pi/8 && angle < 9*math.Pi/8 {
		return LEFT
	} else if angle >= 9*math.Pi/8 && angle < 11*math.Pi/8 {
		return DOWN_LEFT
	} else if angle >= 11*math.Pi/8 && angle < 13*math.Pi/8 {
		return DOWN
	} else if angle >= 13*math.Pi/8 && angle < 15*math.Pi/8 {
		return DOWN_RIGHT
	} else if angle >= 15*math.Pi/8 || angle < math.Pi/8 {
		return RIGHT
	} else if angle >= math.Pi/8 && angle < 3*math.Pi/8 {
		return UP_RIGHT
	} else if angle >= 3*math.Pi/8 && angle < 5*math.Pi/8 {
		return UP
	} else {
		return UP_LEFT
	}
}

// updateClients sends AOI updates to all connected clients.
func (s *GameServer) updateClients() {
	for mapID, mapState := range s.maps {
		for _, player := range mapState.players {
			client := player.Client
			if client == nil {
				continue
			}
			player.AOI = Rectangle{
				MinX: player.Pos.X - s.aoiRadius, MinY: player.Pos.Y - s.aoiRadius,
				MaxX: player.Pos.X + player.Dims.Width + s.aoiRadius, MaxY: player.Pos.Y + player.Dims.Height + s.aoiRadius,
			}

			visiblePlayers := make(map[string]ObjectState)
			for _, otherPlayer := range mapState.players {
				if otherPlayer.ID == player.ID {
					continue
				}
				if rectsOverlap(player.AOI, Rectangle{MinX: otherPlayer.Pos.X, MinY: otherPlayer.Pos.Y, MaxX: otherPlayer.Pos.X + otherPlayer.Dims.Width, MaxY: otherPlayer.Pos.Y + otherPlayer.Dims.Height}) {
					visiblePlayers[otherPlayer.ID] = ObjectState{
						ID: otherPlayer.ID, Pos: otherPlayer.Pos, Dims: otherPlayer.Dims, Type: "player",
					}
				}
			}

			visibleGridObjects := make(map[string]ObjectState)
			for id, obstacle := range mapState.pathfinder.obstacles {
				if rectsOverlap(player.AOI, Rectangle{MinX: obstacle.Pos.X, MinY: obstacle.Pos.Y, MaxX: obstacle.Pos.X + obstacle.Dims.Width, MaxY: obstacle.Pos.Y + obstacle.Dims.Height}) {
					visibleGridObjects[id] = obstacle
				}
			}

			for id, portal := range mapState.portals {
				if rectsOverlap(player.AOI, Rectangle{MinX: portal.Pos.X, MinY: portal.Pos.Y, MaxX: portal.Pos.X + portal.Dims.Width, MaxY: portal.Pos.Y + portal.Dims.Height}) {
					label := fmt.Sprintf("To Map %d", portal.PortalConfig.DestMapID)
					if portal.PortalConfig.DestMapID == mapID {
						label = "To Same Map"
					}
					visibleGridObjects[id] = ObjectState{
						ID: id, Pos: portal.Pos, Dims: portal.Dims, Type: "portal", PortalLabel: label,
					}
				}
			}

			payload := AOIUpdatePayload{
				PlayerID:           player.ID,
				Player:             *player,
				VisiblePlayers:     visiblePlayers,
				VisibleGridObjects: visibleGridObjects,
			}

			jsonMessage, err := json.Marshal(map[string]interface{}{
				"type":    "aoi_update",
				"payload": payload,
			})
			if err != nil {
				log.Printf("Error marshaling AOI update: %v", err)
				continue
			}
			select {
			case client.send <- jsonMessage:
			default:
				// If a client is too slow, we'll just close their connection
				log.Printf("Client %s send channel is full, closing connection.", client.playerID)
				close(client.send)
				// We don't delete from mapState.players or s.clients here; unregister handles this
			}
		}
	}
}

// readPump handles incoming messages from the WebSocket connection.
func readPump(conn *websocket.Conn, client *Client, server *GameServer) {
	defer func() {
		server.unregister <- client
		conn.Close()
	}()
	for {
		_, message, err := conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("error: %v", err)
			}
			break
		}
		client.lastAction = time.Now()
		var msg map[string]interface{}
		if err := json.Unmarshal(message, &msg); err != nil {
			log.Printf("Error unmarshaling message: %v", err)
			continue
		}
		if msg["type"] == "path_request" {
			payload := msg["payload"].(map[string]interface{})
			targetX := int(payload["targetX"].(float64))
			targetY := int(payload["targetY"].(float64))
			server.mu.Lock()

			// Find player and their current map
			var mapState *MapState
			var player *PlayerState
			for _, m := range server.maps {
				if p, ok := m.players[client.playerID]; ok {
					mapState = m
					player = p
					break
				}
			}

			if mapState == nil || player == nil {
				log.Printf("Player %s not found in any map", client.playerID)
				server.mu.Unlock()
				continue
			}

			// Find closest walkable point to the target
			targetPoint, err := mapState.pathfinder.findClosestWalkablePoint(PointI{X: targetX, Y: targetY}, player.Dims)
			if err != nil {
				log.Printf("Could not find walkable point for target: %v", err)
				server.mu.Unlock()
				continue
			}

			startPoint := PointI{X: int(player.Pos.X), Y: int(player.Pos.Y)}
			path, err := mapState.pathfinder.Astar(startPoint, targetPoint, player.Dims)

			if err != nil {
				player.Path = nil
			} else {
				player.Path = path
				player.TargetPos = targetPoint
			}
			server.mu.Unlock()
		}
	}
}

// writePump handles outgoing messages to the WebSocket connection.
func writePump(conn *websocket.Conn, client *Client) {
	ticker := time.NewTicker(30 * time.Second)
	defer func() {
		ticker.Stop()
		conn.Close()
	}()
	for {
		select {
		case message, ok := <-client.send:
			conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if !ok {
				conn.WriteMessage(websocket.CloseMessage, []byte{})
				return
			}
			w, err := conn.NextWriter(websocket.TextMessage)
			if err != nil {
				return
			}
			w.Write(message)
			if err := w.Close(); err != nil {
				return
			}
		case <-ticker.C:
			conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if err := conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				return
			}
		}
	}
}

// handleWebsocket handles new websocket connections.
func handleWebsocket(server *GameServer, w http.ResponseWriter, r *http.Request) {
	upgrader := websocket.Upgrader{
		CheckOrigin: func(r *http.Request) bool {
			return true
		},
	}
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println("Upgrade failed:", err)
		return
	}
	playerID := uuid.New().String()
	client := &Client{
		playerID: playerID,
		conn:     conn,
		send:     make(chan []byte, 256),
	}
	server.register <- client
	go writePump(conn, client)
	go readPump(conn, client, server)
}

func main() {
	gameServer := NewGameServer()
	go gameServer.run()
	http.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		handleWebsocket(gameServer, w, r)
	})
	log.Println("Server starting on :8080")
	if err := http.ListenAndServe(":8080", nil); err != nil {
		log.Fatal("ListenAndServe: ", err)
	}
}
