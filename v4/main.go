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
// These types encapsulate the data needed for the grid, pathfinding nodes, and the priority queue.
//----------------------------------------------------------------------------------------------------------------------

// Point represents a 2D coordinate.
type Point struct {
	X float64 `json:"X"`
	Y float64 `json:"Y"`
}

// PointI is an integer grid coordinate.
type PointI struct {
	X, Y int
}

// Rectangle represents a rectangle for collision detection.
type Rectangle struct {
	MinX, MinY, MaxX, MaxY float64
}

// Dimensions represents the width and height of an object, with correct JSON tags.
type Dimensions struct {
	Width  float64 `json:"Width"`
	Height float64 `json:"Height"`
}

// Direction defines the possible directions for animated objects.
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
	NONE // For objects without a specific direction (e.g., static)
)

// ObjectLayerMode defines animation modes (e.g., idle, walking).
type ObjectLayerMode int

const (
	IDLE ObjectLayerMode = iota
	WALKING
)

// ObjectState represents the state of a game object, with variable dimensions and a unique ID.
type ObjectState struct {
	ID   string     `json:"id"`
	Pos  Point      `json:"Pos"`
	Dims Dimensions `json:"Dims"`
	Type string     `json:"Type"` // e.g., "player", "obstacle", "portal"
}

// PlayerState holds the dynamic state of a player, including their path, direction, and mode.
type PlayerState struct {
	ID        string     `json:"id"`
	MapID     int        `json:"MapID"` // NEW: The map/channel the player is currently in
	Pos       Point      `json:"Pos"`
	Dims      Dimensions `json:"Dims"`
	Path      []PointI   `json:"path"`
	TargetPos PointI     `json:"targetPos"`
	AOI       Rectangle
	Client    *Client
	Direction Direction       `json:"direction"`
	Mode      ObjectLayerMode `json:"mode"`
	// Portal teleportation fields
	OnPortal           bool
	TimeOnPortal       time.Time
	ActivePortalID     string
	DestPortalConfigID string
}

// PortalConfig defines the destination of a portal.
type PortalConfig struct {
	DestMapID       int     `json:"dest_map"`
	DestPortalIndex int     `json:"dest_portal_index"`
	SpawnRadius     float64 `json:"spawn_radius"`
}

// PortalState represents a portal in the game world.
type PortalState struct {
	ID           string
	Pos          Point
	Dims         Dimensions
	PortalConfig *PortalConfig
}

// MapState holds the state for a single game map.
type MapState struct {
	pathfinder   *Pathfinder
	obstacles    map[string]ObjectState
	portals      map[string]*PortalState
	players      map[string]*PlayerState
	gridW, gridH int
}

// AOIUpdatePayload is the structure for the JSON message sent to clients.
type AOIUpdatePayload struct {
	PlayerID           string                 `json:"playerID"`
	Player             PlayerState            `json:"player"`
	VisiblePlayers     map[string]ObjectState `json:"visiblePlayers"`
	VisibleGridObjects map[string]ObjectState `json:"visibleGridObjects"`
}

// Client represents a connected player.
type Client struct {
	conn       *websocket.Conn
	playerID   string
	send       chan []byte
	isPlayer   bool
	lastAction time.Time
}

// GameServer manages the game state, connections, and game loop.
type GameServer struct {
	mu         sync.Mutex
	maps       map[int]*MapState // NEW: Map of game maps
	clients    map[string]*Client
	register   chan *Client
	unregister chan *Client
	aoiRadius  float64
}

// Node and PriorityQueue for A* pathfinding.
type Node struct {
	X, Y    int
	g, h, f float64
	parent  *Node
	index   int // heap index
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
func (pq *PriorityQueue) update(node *Node, g, h float64) {
	node.g = g
	node.h = h
	node.f = g + h
	heap.Fix(pq, node.index)
}

// Pathfinder holds the state and logic for the A* search.
type Pathfinder struct {
	gridW, gridH int
	obstacles    map[string]ObjectState
	grid         [][]int // 0 = free, 1 = obstacle
}

//----------------------------------------------------------------------------------------------------------------------
// 2. Core Logic & Methods
// These methods implement the A* algorithm and its supporting functions.
//----------------------------------------------------------------------------------------------------------------------

// NewPathfinder creates and initializes a new Pathfinder instance.
func NewPathfinder(w, h int) *Pathfinder {
	p := &Pathfinder{
		gridW:     w,
		gridH:     h,
		obstacles: make(map[string]ObjectState),
	}
	p.grid = make([][]int, h)
	for y := 0; y < h; y++ {
		p.grid[y] = make([]int, w)
	}
	return p
}

// GenerateObstacles creates a procedural grid with randomly placed obstacles.
func (pf *Pathfinder) GenerateObstacles(numObstacles int, start, end PointI, portalRects []Rectangle) {
	log.Println("Generating new map obstacles...")
	pf.obstacles = make(map[string]ObjectState)

	for y := 0; y < pf.gridH; y++ {
		for x := 0; x < pf.gridW; x++ {
			pf.grid[y][x] = 0
		}
	}

	maxObsW := int(math.Max(2, float64(pf.gridW)/15.0))
	maxObsH := int(math.Max(2, float64(pf.gridH)/15.0))
	minObsDim := 2

	attempts := 0
	placed := 0

	// Define a safe zone around the start and end points
	safeZone := Rectangle{
		MinX: float64(start.X) - 5, MinY: float64(start.Y) - 5,
		MaxX: float64(start.X) + 5, MaxY: float64(start.Y) + 5,
	}

	for placed < numObstacles && attempts < numObstacles*10 {
		w := rand.Intn(maxObsW-minObsDim+1) + minObsDim
		h := rand.Intn(maxObsH-minObsDim+1) + minObsDim
		minX := rand.Intn(pf.gridW - w)
		minY := rand.Intn(pf.gridH - h)
		maxX := minX + w
		maxY := minY + h

		obs := ObjectState{
			ID:   uuid.New().String(),
			Pos:  Point{X: float64(minX), Y: float64(minY)},
			Dims: Dimensions{Width: float64(w), Height: float64(h)},
			Type: "obstacle",
		}

		obsRect := Rectangle{MinX: float64(minX), MinY: float64(minY), MaxX: float64(maxX), MaxY: float64(maxY)}

		// Check for overlap with the safe zone and portals
		if rectsOverlap(obsRect, safeZone) {
			attempts++
			continue
		}
		overlapsPortal := false
		for _, pRect := range portalRects {
			if rectsOverlap(obsRect, pRect) {
				overlapsPortal = true
				break
			}
		}
		if overlapsPortal {
			attempts++
			continue
		}

		// Check for overlap with existing obstacles
		overlapsExisting := false
		for _, existingObs := range pf.obstacles {
			existingRect := Rectangle{
				MinX: existingObs.Pos.X, MinY: existingObs.Pos.Y,
				MaxX: existingObs.Pos.X + existingObs.Dims.Width, MaxY: existingObs.Pos.Y + existingObs.Dims.Height,
			}
			if rectsOverlap(obsRect, existingRect) {
				overlapsExisting = true
				break
			}
		}

		if overlapsExisting {
			attempts++
			continue
		}

		// Mark grid cells as occupied
		for y := minY; y < maxY; y++ {
			for x := minX; x < maxX; x++ {
				if x >= 0 && x < pf.gridW && y >= 0 && y < pf.gridH {
					pf.grid[y][x] = 1
				}
			}
		}

		pf.obstacles[obs.ID] = obs
		placed++
		log.Printf("Placed obstacle %s at (%d, %d) with dimensions %dx%d", obs.ID, minX, minY, w, h)
	}
	log.Printf("Map generation complete. Placed %d obstacles.", len(pf.obstacles))
}

// generatePortals creates and places portals on the map.
func (ms *MapState) generatePortals(numPortals int) []*PortalState {
	portals := make([]*PortalState, numPortals)
	for i := 0; i < numPortals; i++ {
		// Random dimensions for the portal
		w := float64(rand.Intn(4) + 2) // 2-5
		h := float64(rand.Intn(4) + 2) // 2-5
		dims := Dimensions{Width: w, Height: h}

		// Find a random walkable position
		posI, err := ms.pathfinder.findRandomWalkablePoint(dims)
		if err != nil {
			log.Printf("Error placing portal: %v", err)
			continue
		}
		pos := Point{X: float64(posI.X), Y: float64(posI.Y)}

		portal := &PortalState{
			ID:           uuid.New().String(),
			Pos:          pos,
			Dims:         dims,
			PortalConfig: &PortalConfig{}, // Placeholder, will be filled later
		}

		portals[i] = portal

		// Mark the portal area as occupied to prevent other objects from spawning there
		for y := int(pos.Y); y < int(pos.Y+h); y++ {
			for x := int(pos.X); x < int(pos.X+w); x++ {
				ms.pathfinder.grid[y][x] = 1
			}
		}
	}
	return portals
}

// isCellWalkable checks if a single grid cell is an obstacle.
func (pf *Pathfinder) isCellWalkable(x, y int) bool {
	if x < 0 || x >= pf.gridW || y < 0 || y >= pf.gridH {
		return false
	}
	return pf.grid[y][x] == 0
}

// isWalkable checks if a rectangular area is clear. It now takes player dimensions as an argument.
func (pf *Pathfinder) isWalkable(x, y int, playerDims Dimensions) bool {
	playerW, playerH := int(math.Ceil(playerDims.Width)), int(math.Ceil(playerDims.Height))

	// Check if any cell within the player's bounding box is an obstacle.
	for dy := 0; dy < playerH; dy++ {
		for dx := 0; dx < playerW; dx++ {
			if !pf.isCellWalkable(x+dx, y+dy) {
				return false
			}
		}
	}
	return true
}

// findRandomWalkablePoint finds a random, valid spawn location for a player. It now takes player dimensions.
func (pf *Pathfinder) findRandomWalkablePoint(playerDims Dimensions) (PointI, error) {
	attempts := 0
	for attempts < 100 { // Limit attempts to avoid infinite loops on a full map
		x := rand.Intn(pf.gridW - int(playerDims.Width))
		y := rand.Intn(pf.gridH - int(playerDims.Height))
		if pf.isWalkable(x, y, playerDims) {
			return PointI{X: x, Y: y}, nil
		}
		attempts++
	}
	return PointI{}, fmt.Errorf("could not find a walkable point after %d attempts", attempts)
}

// Astar finds the shortest path from start to end using the A* algorithm. It now takes player dimensions.
func (pf *Pathfinder) Astar(start, end PointI, playerDims Dimensions) ([]PointI, error) {
	if !pf.isWalkable(start.X, start.Y, playerDims) {
		closestStart, err := pf.findClosestWalkablePoint(start, playerDims)
		if err != nil {
			return nil, err
		}
		start = closestStart
	}
	if !pf.isWalkable(end.X, end.Y, playerDims) {
		closestEnd, err := pf.findClosestWalkablePoint(end, playerDims)
		if err != nil {
			return nil, err
		}
		end = closestEnd
	}
	openSet := make(PriorityQueue, 0)
	heap.Init(&openSet)
	startNode := &Node{X: start.X, Y: start.Y, g: 0, h: heuristic(start.X, start.Y, end.X, end.Y), parent: nil}
	heap.Push(&openSet, startNode)
	gScore := make(map[PointI]float64)
	gScore[start] = 0.0
	for openSet.Len() > 0 {
		current := heap.Pop(&openSet).(*Node)
		if current.X == end.X && current.Y == end.Y {
			return reconstructPath(current), nil
		}
		neighbors := pf.getNeighbors(current)
		for _, neighbor := range neighbors {
			if !pf.isWalkable(neighbor.X, neighbor.Y, playerDims) {
				continue
			}
			tentativeGScore := current.g + 1
			neighborPoint := PointI{X: neighbor.X, Y: neighbor.Y}
			if gScore[neighborPoint] == 0 || tentativeGScore < gScore[neighborPoint] {
				gScore[neighborPoint] = tentativeGScore
				neighbor.g = tentativeGScore
				neighbor.h = heuristic(neighbor.X, neighbor.Y, end.X, end.Y)
				neighbor.f = neighbor.g + neighbor.h
				neighbor.parent = current
				heap.Push(&openSet, neighbor)
			}
		}
	}
	return nil, fmt.Errorf("could not find path")
}

// getNeighbors returns the walkable neighbors of a node.
func (pf *Pathfinder) getNeighbors(node *Node) []*Node {
	neighbors := make([]*Node, 0, 8)
	moves := []PointI{{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}
	for _, move := range moves {
		newX, newY := node.X+move.X, node.Y+move.Y
		if newX >= 0 && newX < pf.gridW && newY >= 0 && newY < pf.gridH {
			neighbors = append(neighbors, &Node{X: newX, Y: newY})
		}
	}
	return neighbors
}

// heuristic is the Manhattan distance.
func heuristic(x1, y1, x2, y2 int) float64 {
	dx := float64(x1 - x2)
	dy := float64(y1 - y2)
	return math.Sqrt(dx*dx + dy*dy)
}

// reconstructPath rebuilds the path from the end node.
func reconstructPath(node *Node) []PointI {
	path := make([]PointI, 0)
	for node != nil {
		path = append([]PointI{{X: node.X, Y: node.Y}}, path...)
		node = node.parent
	}
	return path
}

// findClosestWalkablePoint finds the closest walkable point to a given point.
func (pf *Pathfinder) findClosestWalkablePoint(p PointI, playerDims Dimensions) (PointI, error) {
	// Simple breadth-first search to find the closest walkable cell
	queue := []PointI{p}
	visited := make(map[PointI]bool)
	visited[p] = true

	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]

		if pf.isWalkable(current.X, current.Y, playerDims) {
			return current, nil
		}

		neighbors := []PointI{
			{X: current.X + 1, Y: current.Y},
			{X: current.X - 1, Y: current.Y},
			{X: current.X, Y: current.Y + 1},
			{X: current.X, Y: current.Y - 1},
		}

		for _, neighbor := range neighbors {
			if neighbor.X >= 0 && neighbor.X < pf.gridW && neighbor.Y >= 0 && neighbor.Y < pf.gridH && !visited[neighbor] {
				visited[neighbor] = true
				queue = append(queue, neighbor)
			}
		}
	}

	return PointI{}, fmt.Errorf("no walkable point found near (%d, %d)", p.X, p.Y)
}

// rectsOverlap checks for overlap between two rectangles.
func rectsOverlap(r1, r2 Rectangle) bool {
	return r1.MinX < r2.MaxX && r1.MaxX > r2.MinX && r1.MinY < r2.MaxY && r1.MaxY > r2.MinY
}

// NewGameServer creates and initializes a new GameServer instance.
func NewGameServer() *GameServer {
	rand.Seed(time.Now().UnixNano())
	server := &GameServer{
		clients:    make(map[string]*Client),
		maps:       make(map[int]*MapState),
		register:   make(chan *Client),
		unregister: make(chan *Client),
		aoiRadius:  20.0,
	}

	// NEW: Initialize 3 maps (channels)
	for i := 1; i <= 3; i++ {
		mapState := &MapState{
			gridW:      100,
			gridH:      100,
			pathfinder: NewPathfinder(100, 100),
			obstacles:  make(map[string]ObjectState),
			portals:    make(map[string]*PortalState),
			players:    make(map[string]*PlayerState),
		}
		// Create portals before obstacles to ensure they are on clear paths
		portalCount := rand.Intn(2) + 2 // 2 to 3 portals
		portals := mapState.generatePortals(portalCount)
		for _, p := range portals {
			mapState.portals[p.ID] = p
		}

		// Collect portal rectangles for obstacle generation
		var portalRects []Rectangle
		for _, p := range portals {
			portalRects = append(portalRects, Rectangle{
				MinX: p.Pos.X,
				MinY: p.Pos.Y,
				MaxX: p.Pos.X + p.Dims.Width,
				MaxY: p.Pos.Y + p.Dims.Height,
			})
		}

		// Use the existing pathfinder to generate obstacles, ensuring they don't block portals
		mapState.pathfinder.GenerateObstacles(100, PointI{0, 0}, PointI{99, 99}, portalRects)
		mapState.obstacles = mapState.pathfinder.obstacles
		server.maps[i] = mapState
	}

	// NEW: Configure portal destinations
	// This can be done in a separate loop after all maps and portals are generated.
	portalConfigs := make(map[int][]PortalConfig)
	portalConfigs[1] = []PortalConfig{
		{DestMapID: 2, DestPortalIndex: 0, SpawnRadius: 15.0},
		{DestMapID: 1, DestPortalIndex: 2, SpawnRadius: 10.0},
		{DestMapID: 3, DestPortalIndex: 1, SpawnRadius: 20.0},
	}
	portalConfigs[2] = []PortalConfig{
		{DestMapID: 3, DestPortalIndex: 0, SpawnRadius: 15.0},
		{DestMapID: 1, DestPortalIndex: 1, SpawnRadius: 10.0},
		{DestMapID: 2, DestPortalIndex: 1, SpawnRadius: 20.0},
	}
	portalConfigs[3] = []PortalConfig{
		{DestMapID: 1, DestPortalIndex: 0, SpawnRadius: 15.0},
		{DestMapID: 2, DestPortalIndex: 2, SpawnRadius: 10.0},
		{DestMapID: 3, DestPortalIndex: 0, SpawnRadius: 20.0},
	}

	// Apply the configured destinations to the portals
	for mapID, ms := range server.maps {
		var portalList []*PortalState
		for _, p := range ms.portals {
			portalList = append(portalList, p)
		}
		for i, p := range portalList {
			if len(portalConfigs[mapID]) > i {
				p.PortalConfig = &portalConfigs[mapID][i]
			}
		}
	}

	return server
}

// Run starts the game server's main loop.
func (s *GameServer) Run() {
	go s.handleClientConnections()

	ticker := time.NewTicker(50 * time.Millisecond) // Server tick rate: 20 FPS
	defer ticker.Stop()

	for range ticker.C {
		s.mu.Lock()

		// NEW: Loop through all maps to update players
		for _, ms := range s.maps {
			// Update player positions and states
			for _, player := range ms.players {
				if len(player.Path) > 0 {
					s.updatePlayerPosition(player)
					s.checkAndHandlePortalCollision(player, ms)
				}
				// If a player is on a portal for a long time, teleport them
				s.teleportPlayer(player, ms)
			}

			// Send AOI updates for each player in this map
			for _, player := range ms.players {
				s.sendAOI(player, ms)
			}
		}
		s.mu.Unlock()
	}
}

// teleportPlayer handles the teleportation logic.
func (s *GameServer) teleportPlayer(player *PlayerState, ms *MapState) {
	if !player.OnPortal {
		return
	}

	// Teleport after 1 second of holding on the portal
	if time.Since(player.TimeOnPortal) > 1*time.Second {
		portal := ms.portals[player.ActivePortalID]
		config := portal.PortalConfig

		destMapState := s.maps[config.DestMapID]
		if destMapState == nil {
			log.Printf("Destination map %d not found for player %s", config.DestMapID, player.ID)
			return
		}

		var destPortals []*PortalState
		for _, p := range destMapState.portals {
			destPortals = append(destPortals, p)
		}

		if len(destPortals) <= config.DestPortalIndex {
			log.Printf("Destination portal index %d out of bounds for map %d", config.DestPortalIndex, config.DestMapID)
			return
		}

		destPortal := destPortals[config.DestPortalIndex]

		// Find a new, random walkable point near the destination portal
		spawnCenter := PointI{X: int(destPortal.Pos.X), Y: int(destPortal.Pos.Y)}
		newPos, err := destMapState.pathfinder.findRandomPointInRadius(spawnCenter, config.SpawnRadius, player.Dims)
		if err != nil {
			log.Printf("Could not find spawn point for player %s, placing at portal center", player.ID)
			newPos = spawnCenter
		}

		// Update player's map and position
		player.MapID = config.DestMapID
		player.Pos.X = float64(newPos.X)
		player.Pos.Y = float64(newPos.Y)
		player.Path = nil // Clear path on teleport

		// Remove player from old map and add to new one
		delete(ms.players, player.ID)
		destMapState.players[player.ID] = player

		log.Printf("Player %s teleported from map %d to map %d via portal", player.ID, ms.gridW, config.DestMapID)

		// Reset portal state
		player.OnPortal = false
		player.ActivePortalID = ""
	}
}

// findRandomPointInRadius finds a random walkable point within a given radius.
func (pf *Pathfinder) findRandomPointInRadius(center PointI, radius float64, playerDims Dimensions) (PointI, error) {
	maxAttempts := 100
	for i := 0; i < maxAttempts; i++ {
		angle := rand.Float64() * 2 * math.Pi
		dist := rand.Float64() * radius

		x := int(float64(center.X) + dist*math.Cos(angle))
		y := int(float64(center.Y) + dist*math.Sin(angle))

		if pf.isWalkable(x, y, playerDims) {
			return PointI{X: x, Y: y}, nil
		}
	}
	return PointI{}, fmt.Errorf("could not find walkable point in radius")
}

// checkAndHandlePortalCollision checks if the player is on a portal.
func (s *GameServer) checkAndHandlePortalCollision(player *PlayerState, ms *MapState) {
	playerRect := Rectangle{
		MinX: player.Pos.X,
		MinY: player.Pos.Y,
		MaxX: player.Pos.X + player.Dims.Width,
		MaxY: player.Pos.Y + player.Dims.Height,
	}

	onAnyPortal := false
	for _, portal := range ms.portals {
		portalRect := Rectangle{
			MinX: portal.Pos.X,
			MinY: portal.Pos.Y,
			MaxX: portal.Pos.X + portal.Dims.Width,
			MaxY: portal.Pos.Y + portal.Dims.Height,
		}
		if rectsOverlap(playerRect, portalRect) {
			if !player.OnPortal || player.ActivePortalID != portal.ID {
				player.OnPortal = true
				player.TimeOnPortal = time.Now()
				player.ActivePortalID = portal.ID
			}
			onAnyPortal = true
			break
		}
	}

	if !onAnyPortal {
		player.OnPortal = false
		player.ActivePortalID = ""
	}
}

// updatePlayerPosition moves the player one step along their path.
func (s *GameServer) updatePlayerPosition(player *PlayerState) {
	if len(player.Path) > 0 {
		nextPoint := player.Path[0]
		targetX := float64(nextPoint.X)
		targetY := float64(nextPoint.Y)

		dist := math.Sqrt(math.Pow(targetX-player.Pos.X, 2) + math.Pow(targetY-player.Pos.Y, 2))
		speed := 5.0
		if dist > 0.1 {
			player.Mode = WALKING
			dx := (targetX - player.Pos.X) / dist
			dy := (targetY - player.Pos.Y) / dist

			player.Pos.X += dx * speed * (50.0 / 1000.0)
			player.Pos.Y += dy * speed * (50.0 / 1000.0)

			// Determine direction
			player.Direction = getDirection(dx, dy)
		} else {
			player.Pos.X = targetX
			player.Pos.Y = targetY
			player.Path = player.Path[1:] // Pop the first element
			if len(player.Path) == 0 {
				player.Mode = IDLE
				player.Direction = NONE
			}
		}
	}
}

// getDirection determines the player's animation direction based on movement.
func getDirection(dx, dy float64) Direction {
	angle := math.Atan2(dy, dx)
	angleDeg := angle * 180 / math.Pi
	if angleDeg < 0 {
		angleDeg += 360
	}

	switch {
	case angleDeg >= 337.5 || angleDeg < 22.5:
		return RIGHT
	case angleDeg >= 22.5 && angleDeg < 67.5:
		return DOWN_RIGHT
	case angleDeg >= 67.5 && angleDeg < 112.5:
		return DOWN
	case angleDeg >= 112.5 && angleDeg < 157.5:
		return DOWN_LEFT
	case angleDeg >= 157.5 && angleDeg < 202.5:
		return LEFT
	case angleDeg >= 202.5 && angleDeg < 247.5:
		return UP_LEFT
	case angleDeg >= 247.5 && angleDeg < 292.5:
		return UP
	case angleDeg >= 292.5 && angleDeg < 337.5:
		return UP_RIGHT
	}
	return NONE
}

// sendAOI sends Area of Interest updates to a single client.
func (s *GameServer) sendAOI(player *PlayerState, ms *MapState) {
	if player.Client == nil {
		return
	}

	// Calculate the player's AOI
	minX := player.Pos.X - s.aoiRadius
	minY := player.Pos.Y - s.aoiRadius
	maxX := player.Pos.X + player.Dims.Width + s.aoiRadius
	maxY := player.Pos.Y + player.Dims.Height + s.aoiRadius
	player.AOI = Rectangle{MinX: minX, MinY: minY, MaxX: maxX, MaxY: maxY}

	visiblePlayers := make(map[string]ObjectState)
	for id, otherPlayer := range ms.players {
		if id != player.ID {
			otherPlayerRect := Rectangle{
				MinX: otherPlayer.Pos.X, MinY: otherPlayer.Pos.Y,
				MaxX: otherPlayer.Pos.X + otherPlayer.Dims.Width, MaxY: otherPlayer.Pos.Y + otherPlayer.Dims.Height,
			}
			if rectsOverlap(player.AOI, otherPlayerRect) {
				visiblePlayers[id] = ObjectState{
					ID:   otherPlayer.ID,
					Pos:  otherPlayer.Pos,
					Dims: otherPlayer.Dims,
					Type: "player",
				}
			}
		}
	}

	visibleGridObjects := make(map[string]ObjectState)
	for id, obs := range ms.obstacles {
		obsRect := Rectangle{
			MinX: obs.Pos.X, MinY: obs.Pos.Y,
			MaxX: obs.Pos.X + obs.Dims.Width, MaxY: obs.Pos.Y + obs.Dims.Height,
		}
		if rectsOverlap(player.AOI, obsRect) {
			visibleGridObjects[id] = obs
		}
	}
	// NEW: Add portals to visible objects
	for id, p := range ms.portals {
		portalRect := Rectangle{
			MinX: p.Pos.X, MinY: p.Pos.Y,
			MaxX: p.Pos.X + p.Dims.Width, MaxY: p.Pos.Y + p.Dims.Height,
		}
		if rectsOverlap(player.AOI, portalRect) {
			visibleGridObjects[id] = ObjectState{
				ID: p.ID, Pos: p.Pos, Dims: p.Dims, Type: "portal",
			}
		}
	}

	// Create and send the AOI update payload
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
		log.Printf("json marshal error: %v", err)
		return
	}
	player.Client.send <- jsonMessage
}

func (s *GameServer) handleClientConnections() {
	for {
		select {
		case client := <-s.register:
			s.mu.Lock()
			s.clients[client.playerID] = client

			// NEW: Find a random map to spawn the new player
			mapIDs := make([]int, 0, len(s.maps))
			for id := range s.maps {
				mapIDs = append(mapIDs, id)
			}
			spawnMapID := mapIDs[rand.Intn(len(mapIDs))]
			spawnMapState := s.maps[spawnMapID]

			playerDims := Dimensions{Width: 1.0, Height: 1.0}
			startPosI, err := spawnMapState.pathfinder.findRandomWalkablePoint(playerDims)
			if err != nil {
				log.Printf("Error finding spawn point: %v", err)
				startPosI = PointI{X: 10, Y: 10}
			}

			player := &PlayerState{
				ID:        client.playerID,
				MapID:     spawnMapID,
				Pos:       Point{X: float64(startPosI.X), Y: float64(startPosI.Y)},
				Dims:      playerDims,
				Path:      make([]PointI, 0),
				Client:    client,
				Direction: NONE,
				Mode:      IDLE,
				OnPortal:  false,
			}
			spawnMapState.players[player.ID] = player
			s.mu.Unlock()

			// Send initial grid data
			initData, err := json.Marshal(map[string]interface{}{
				"type": "init_data",
				"payload": map[string]interface{}{
					"playerID":  client.playerID,
					"gridW":     spawnMapState.gridW,
					"gridH":     spawnMapState.gridH,
					"aoiRadius": s.aoiRadius,
				},
			})
			if err != nil {
				log.Printf("json marshal error: %v", err)
				return
			}
			client.send <- initData
			log.Printf("Client registered: %s, spawned on map %d", client.playerID, spawnMapID)

		case client := <-s.unregister:
			s.mu.Lock()
			if _, ok := s.clients[client.playerID]; ok {
				delete(s.clients, client.playerID)
				// NEW: Find the player's map and remove them
				for _, ms := range s.maps {
					if _, ok := ms.players[client.playerID]; ok {
						delete(ms.players, client.playerID)
						break
					}
				}
				close(client.send)
				log.Printf("Client unregistered: %s", client.playerID)
			}
			s.mu.Unlock()
		}
	}
}

func (s *GameServer) handleWebSocket(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println("Upgrade error:", err)
		return
	}

	// Create a new unique playerID and client instance
	playerID := uuid.New().String()
	client := &Client{
		conn:       conn,
		playerID:   playerID,
		send:       make(chan []byte, 256),
		lastAction: time.Now(),
	}

	s.register <- client
	go writePump(conn, client)
	go readPump(conn, s, client)
}

//----------------------------------------------------------------------------------------------------------------------
// 3. Helper Functions & Main
// Functions for starting the server and handling HTTP requests.
//----------------------------------------------------------------------------------------------------------------------

var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin: func(r *http.Request) bool {
		return true
	},
}

func readPump(conn *websocket.Conn, s *GameServer, client *Client) {
	defer func() {
		s.unregister <- client
		conn.Close()
	}()
	conn.SetReadLimit(512)
	conn.SetReadDeadline(time.Now().Add(60 * time.Second))
	conn.SetPongHandler(func(string) error { conn.SetReadDeadline(time.Now().Add(60 * time.Second)); return nil })

	for {
		_, message, err := conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("read error: %v", err)
			}
			break
		}

		var req map[string]interface{}
		if err := json.Unmarshal(message, &req); err != nil {
			log.Printf("JSON unmarshal error: %v, message: %s", err, message)
			continue
		}

		reqType, ok := req["type"].(string)
		if !ok {
			log.Printf("Invalid message format: 'type' field missing or not a string")
			continue
		}

		if reqType == "path_request" {
			payload, ok := req["payload"].(map[string]interface{})
			if !ok {
				log.Printf("Invalid path_request payload format")
				continue
			}
			s.mu.Lock()
			// NEW: Find the player and their current map
			var player *PlayerState
			var mapState *MapState
			for _, ms := range s.maps {
				if p, ok := ms.players[client.playerID]; ok {
					player = p
					mapState = ms
					break
				}
			}
			s.mu.Unlock()

			if player == nil || mapState == nil {
				log.Printf("Player %s not found in any map", client.playerID)
				continue
			}

			targetPosData, ok := payload["targetPos"].(map[string]interface{})
			if !ok {
				log.Printf("Invalid targetPos format")
				continue
			}

			targetX := int(targetPosData["X"].(float64))
			targetY := int(targetPosData["Y"].(float64))

			path, err := mapState.pathfinder.Astar(
				PointI{X: int(player.Pos.X), Y: int(player.Pos.Y)},
				PointI{X: targetX, Y: targetY},
				player.Dims,
			)

			if err != nil {
				log.Printf("Pathfinding error for player %s: %v", client.playerID, err)
				// send an error message to the client
				errorMsg, _ := json.Marshal(map[string]string{"type": "error", "message": "Could not find a path to the target."})
				client.send <- errorMsg
				player.Path = nil
				player.Mode = IDLE
				player.Direction = NONE
				continue
			}

			s.mu.Lock()
			player.Path = path
			player.TargetPos = PointI{X: targetX, Y: targetY}
			s.mu.Unlock()
			client.lastAction = time.Now()
		}
	}
}

// writePump writes messages from the GameServer to the WebSocket connection.
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

			// Check if there are more messages in the queue and write them
			n := len(client.send)
			for i := 0; i < n; i++ {
				w.Write(<-client.send)
			}

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

func main() {
	gameServer := NewGameServer()
	go gameServer.Run()

	http.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		gameServer.handleWebSocket(w, r)
	})

	log.Println("Starting server on :8080")
	if err := http.ListenAndServe(":8080", nil); err != nil {
		log.Fatal("ListenAndServe: ", err)
	}
}
