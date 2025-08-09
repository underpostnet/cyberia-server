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
	X float64
	Y float64
}

// PointI is an integer grid coordinate.
type PointI struct {
	X, Y int
}

// Rectangle represents a rectangle for collision detection.
type Rectangle struct {
	MinX, MinY, MaxX, MaxY float64
}

// Node represents a single cell in the grid for the A* algorithm.
type Node struct {
	X, Y    int
	g, h, f float64
	parent  *Node
	index   int // heap index
}

// A* priority queue (min-heap) implementation.
type PriorityQueue []*Node

// Len, Less, and Swap methods are required for the heap.Interface.
func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
	return pq[i].f < pq[j].f
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}

// Push adds an item to the priority queue.
func (pq *PriorityQueue) Push(x interface{}) {
	n := len(*pq)
	item := x.(*Node)
	item.index = n
	*pq = append(*pq, item)
}

// Pop removes and returns the smallest item from the priority queue.
func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil // avoid memory leak
	item.index = -1
	*pq = old[0 : n-1]
	return item
}

// update changes the priority of a node in the queue.
func (pq *PriorityQueue) update(node *Node, g, h float64) {
	node.g = g
	node.h = h
	node.f = g + h
	heap.Fix(pq, node.index)
}

// Pathfinder holds the state and logic for the A* search.
type Pathfinder struct {
	gridW, gridH int
	obstacles    []Rectangle
	objectSize   float64
	grid         [][]int // 0 = free, 1 = obstacle
	start, end   PointI
}

//----------------------------------------------------------------------------------------------------------------------
// 2. Core Logic & Methods
// These methods implement the A* algorithm and its supporting functions.
//----------------------------------------------------------------------------------------------------------------------

// NewPathfinder creates and initializes a new Pathfinder instance.
func NewPathfinder(w, h int, objSize float64) *Pathfinder {
	p := &Pathfinder{
		gridW:      w,
		gridH:      h,
		objectSize: objSize,
	}
	p.grid = make([][]int, h)
	for y := 0; y < h; y++ {
		p.grid[y] = make([]int, w)
	}
	return p
}

// SetStartAndEnd sets the start and end points for visualization.
func (pf *Pathfinder) SetStartAndEnd(start, end PointI) {
	pf.start = start
	pf.end = end
}

// GenerateMap creates a procedural grid with randomly placed obstacles.
func (pf *Pathfinder) GenerateMap(numObstacles int, start, end PointI) {
	pf.obstacles = []Rectangle{} // Clear previous obstacles
	for y := 0; y < pf.gridH; y++ {
		for x := 0; x < pf.gridW; x++ {
			pf.grid[y][x] = 0
		}
	}

	maxObsW := int(math.Max(1, float64(pf.gridW)/10.0))
	maxObsH := int(math.Max(1, float64(pf.gridH)/10.0))

	attempts := 0
	randMaxX := pf.gridW - 1
	randMaxY := pf.gridH - 1

	for placed := 0; placed < numObstacles && attempts < numObstacles*10; attempts++ {
		w := rand.Intn(maxObsW) + 1
		h := rand.Intn(maxObsH) + 1
		minX := rand.Intn(randMaxX + 1)
		minY := rand.Intn(randMaxY + 1)
		maxX := minX + w
		maxY := minY + h
		if maxX >= pf.gridW {
			maxX = pf.gridW - 1
		}
		if maxY >= pf.gridH {
			maxY = pf.gridH - 1
		}

		obs := Rectangle{MinX: float64(minX), MinY: float64(minY), MaxX: float64(maxX), MaxY: float64(maxY)}

		// Skip placing the obstacle if it overlaps with the start or end points
		startRect := Rectangle{MinX: float64(start.X), MinY: float64(start.Y), MaxX: float64(start.X), MaxY: float64(start.Y)}
		endRect := Rectangle{MinX: float64(end.X), MinY: float64(end.Y), MaxX: float64(end.X), MaxY: float64(end.Y)}
		if rectsOverlap(obs, startRect) || rectsOverlap(obs, endRect) {
			continue
		}

		// Mark grid cells
		for y := minY; y <= maxY; y++ {
			for x := minX; x <= maxX; x++ {
				if x >= 0 && x < pf.gridW && y >= 0 && y < pf.gridH {
					pf.grid[y][x] = 1
				}
			}
		}

		pf.obstacles = append(pf.obstacles, obs)
		placed++
	}
}

// findPath runs A* on the integer grid and returns a slice of PointI.
func (pf *Pathfinder) findPath(start, end PointI) ([]PointI, PointI, error) {

	// If the end point is not walkable, find the closest walkable point.
	if !pf.isWalkable(end.X, end.Y) {
		newEnd, err := pf.findClosestWalkablePoint(end)
		if err != nil {
			return nil, PointI{}, fmt.Errorf("end point not walkable and no close alternatives found")
		}
		end = newEnd
	}

	open := make(PriorityQueue, 0)
	heap.Init(&open)

	openSet := make(map[int]*Node)
	gScore := make(map[int]float64)

	startNode := &Node{X: start.X, Y: start.Y, g: 0, h: heuristic(start.X, start.Y, end.X, end.Y)}
	startNode.f = startNode.g + startNode.h
	heap.Push(&open, startNode)
	openSet[keyFrom(start.X, start.Y, pf.gridW)] = startNode
	gScore[keyFrom(start.X, start.Y, pf.gridW)] = 0

	// Track the node in the open set that is closest to the target
	var closestNode *Node = startNode

	dirs := []struct {
		dx, dy int
		cost   float64
	}{
		{1, 0, 1}, {-1, 0, 1}, {0, 1, 1}, {0, -1, 1},
		{1, 1, math.Sqrt2}, {1, -1, math.Sqrt2}, {-1, 1, math.Sqrt2}, {-1, -1, math.Sqrt2},
	}

	for open.Len() > 0 {
		current := heap.Pop(&open).(*Node)
		ck := keyFrom(current.X, current.Y, pf.gridW)
		delete(openSet, ck)

		// Check if we've found a better candidate for the closest node
		if current.h < closestNode.h {
			closestNode = current
		}

		if ck == keyFrom(end.X, end.Y, pf.gridW) {
			path := []PointI{}
			for n := current; n != nil; n = n.parent {
				path = append(path, PointI{n.X, n.Y})
			}
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			return path, end, nil
		}

		for _, d := range dirs {
			nx, ny := current.X+d.dx, current.Y+d.dy
			if !pf.isWalkable(nx, ny) {
				continue
			}

			tentG := current.g + d.cost
			nk := keyFrom(nx, ny, pf.gridW)

			if existingNode, ok := openSet[nk]; ok {
				if tentG < existingNode.g {
					existingNode.g = tentG
					existingNode.parent = current
					gScore[nk] = tentG
					open.update(existingNode, tentG, existingNode.h)
				}
				continue
			}

			if existing, ok := gScore[nk]; ok && tentG >= existing {
				continue
			}

			gScore[nk] = tentG
			hn := heuristic(nx, ny, end.X, end.Y)
			node := &Node{X: nx, Y: ny, g: tentG, h: hn, f: tentG + hn, parent: current}
			heap.Push(&open, node)
			openSet[nk] = node
		}
	}

	// If the loop finishes without finding a path, rebuild the path to the closest node
	if closestNode != nil {
		path := []PointI{}
		for n := closestNode; n != nil; n = n.parent {
			path = append(path, PointI{n.X, n.Y})
		}
		for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
			path[i], path[j] = path[j], path[i]
		}
		return path, PointI{X: closestNode.X, Y: closestNode.Y}, nil
	}

	return nil, end, fmt.Errorf("no path found")
}

// findClosestWalkablePoint searches for the nearest walkable point to a given point.
func (pf *Pathfinder) findClosestWalkablePoint(point PointI) (PointI, error) {
	halfSize := pf.objectSize / 2.0
	clampedPoint := PointI{
		X: int(math.Max(math.Floor(halfSize), math.Min(float64(point.X), float64(pf.gridW)-math.Ceil(halfSize)))),
		Y: int(math.Max(math.Floor(halfSize), math.Min(float64(point.Y), float64(pf.gridH)-math.Ceil(halfSize)))),
	}

	if pf.isWalkable(clampedPoint.X, clampedPoint.Y) {
		return clampedPoint, nil
	}

	queue := []PointI{clampedPoint}
	visited := make(map[int]bool)
	visited[keyFrom(clampedPoint.X, clampedPoint.Y, pf.gridW)] = true

	dirs := []struct {
		dx, dy int
	}{
		{1, 0}, {-1, 0}, {0, 1}, {0, -1},
		{1, 1}, {1, -1}, {-1, 1}, {-1, -1},
	}

	for len(queue) > 0 {
		curr := queue[0]
		queue = queue[1:]

		if pf.isWalkable(curr.X, curr.Y) {
			return curr, nil
		}

		for _, d := range dirs {
			nx, ny := curr.X+d.dx, curr.Y+d.dy
			if float64(nx) >= math.Floor(halfSize) && float64(nx) <= float64(pf.gridW)-math.Ceil(halfSize) &&
				float64(ny) >= math.Floor(halfSize) && float64(ny) <= float64(pf.gridH)-math.Ceil(halfSize) {
				k := keyFrom(nx, ny, pf.gridW)
				if !visited[k] {
					visited[k] = true
					queue = append(queue, PointI{nx, ny})
				}
			}
		}
	}

	return PointI{}, fmt.Errorf("no walkable point found")
}

// findWalkablePoint finds a random, valid walkable point within the grid.
func (pf *Pathfinder) findWalkablePoint() PointI {
	const maxAttempts = 100
	for i := 0; i < maxAttempts; i++ {
		randX := rand.Intn(pf.gridW)
		randY := rand.Intn(pf.gridH)
		if pf.isWalkable(randX, randY) {
			return PointI{X: randX, Y: randY}
		}
	}
	// Fallback to a brute-force search if random attempts fail
	for y := 0; y < pf.gridH; y++ {
		for x := 0; x < pf.gridW; x++ {
			if pf.isWalkable(x, y) {
				return PointI{X: x, Y: y}
			}
		}
	}
	// Should not happen on a valid map
	return PointI{X: 0, Y: 0}
}

// isWalkable checks whether the object centered at (x,y) fits without colliding
// with any obstacle by sampling the grid cells that the object's bounding box covers.
func (pf *Pathfinder) isWalkable(x, y int) bool {
	_, reason := pf.isWalkableVerbose(x, y)
	return reason == ""
}

// isWalkableVerbose provides a detailed reason if a point is not walkable.
func (pf *Pathfinder) isWalkableVerbose(x, y int) (bool, string) {
	halfSize := pf.objectSize / 2.0
	objRect := Rectangle{
		MinX: float64(x) - halfSize,
		MinY: float64(y) - halfSize,
		MaxX: float64(x) + halfSize,
		MaxY: float64(y) + halfSize,
	}

	if objRect.MinX < 0 {
		return false, fmt.Sprintf("out of bounds (minX=%.2f < 0)", objRect.MinX)
	}
	if objRect.MaxX > float64(pf.gridW) {
		return false, fmt.Sprintf("out of bounds (maxX=%.2f > gridW=%d)", objRect.MaxX, pf.gridW)
	}
	if objRect.MinY < 0 {
		return false, fmt.Sprintf("out of bounds (minY=%.2f < 0)", objRect.MinY)
	}
	if objRect.MaxY > float64(pf.gridH) {
		return false, fmt.Sprintf("out of bounds (maxY=%.2f > gridH=%d)", objRect.MaxY, pf.gridH)
	}
	if pf.CheckCollision(objRect) {
		return false, "collides with an obstacle"
	}
	return true, ""
}

// CheckCollision checks if the given rectangle overlaps with any of the
// obstacles in the Pathfinder's map.
func (pf *Pathfinder) CheckCollision(rect Rectangle) bool {
	for _, obs := range pf.obstacles {
		if rectsOverlap(rect, obs) {
			return true
		}
	}
	return false
}

//----------------------------------------------------------------------------------------------------------------------
// 3. Helper Functions
// These functions were moved from heuristic.go to consolidate the pathfinding logic.
//----------------------------------------------------------------------------------------------------------------------

// heuristic: Euclidean distance (admissible for diagonal movement)
func heuristic(ax, ay, bx, by int) float64 {
	dx := float64(ax - bx)
	dy := float64(ay - by)
	return math.Hypot(dx, dy)
}

// rectsOverlap is a simple AABB collision detection helper.
func rectsOverlap(a, b Rectangle) bool {
	return a.MaxX > b.MinX && a.MinX < b.MaxX && a.MaxY > b.MinY && a.MinY < b.MaxY
}

// keyFrom generates a unique integer key from a 2D coordinate.
func keyFrom(x, y, width int) int { return y*width + x }

//----------------------------------------------------------------------------------------------------------------------
// 4. Data Structures & Interfaces
// These types encapsulate the data needed for AOI management.
//----------------------------------------------------------------------------------------------------------------------

// MapData represents the complete, static context of the game map.
// This is the full map state that the server would hold.
type MapData struct {
	Width, Height int
	Obstacles     [][]int // 0 for free space, 1 for obstacle
}

// InitData is the initial data sent to the client upon connection.
type InitData struct {
	GridW      int     `json:"gridW"`
	GridH      int     `json:"gridH"`
	ObjectSize float64 `json:"objectSize"`
}

// AOIUpdates represents the data sent to a single client.
// It is an abstraction that simplifies the full map context into
// a manageable update packet. This is what's 'transmitted'.
type AOIUpdates struct {
	PlayerID           string
	PlayerPos          PointI
	VisibleGridObjects map[string]string // Key is now a string
	VisiblePlayers     map[string]PointI
	Path               []PointI // The path to send to the client
	TargetPos          PointI   // The final destination
}

// Player represents a single player with a unique ID and position.
type Player struct {
	ID        string
	Position  PointI
	isOnline  bool
	targetPos PointI
	path      []PointI
}

// AOIManager is the central class for handling all AOI-related logic.
// It manages the state of the map and all active players.
type AOIManager struct {
	MapData    *MapData
	Players    map[string]*Player
	lastUpdate time.Time
}

//----------------------------------------------------------------------------------------------------------------------
// 5. Core Logic & Methods
// These methods implement the core logic for AOI data management.
//----------------------------------------------------------------------------------------------------------------------

// NewAOIManager creates and returns a new AOIManager instance.
func NewAOIManager(mapData *MapData) *AOIManager {
	return &AOIManager{
		MapData: mapData,
		Players: make(map[string]*Player),
	}
}

// AddPlayer adds a new player to the manager.
func (am *AOIManager) AddPlayer(playerID string, startPos PointI) {
	if _, exists := am.Players[playerID]; !exists {
		am.Players[playerID] = &Player{
			ID:       playerID,
			Position: startPos,
			isOnline: true,
			path:     []PointI{},
		}
	} else {
		// Player already exists, maybe just set them to online.
		am.Players[playerID].isOnline = true
		am.Players[playerID].Position = startPos
	}
}

// RemovePlayer marks a player as offline.
func (am *AOIManager) RemovePlayer(playerID string) {
	if player, exists := am.Players[playerID]; exists {
		player.isOnline = false
	}
}

// UpdatePlayerPosition changes a player's position in the world.
func (am *AOIManager) UpdatePlayerPosition(playerID string, newPos PointI) error {
	if player, exists := am.Players[playerID]; exists {
		player.Position = newPos
		return nil
	}
	return fmt.Errorf("player with ID '%s' not found", playerID)
}

// GetAOIUpdatesForPlayer generates a simplified AOI data packet for a given player.
// It only includes grid objects and other players within the specified radius.
func (am *AOIManager) GetAOIUpdatesForPlayer(playerID string, radius float64, path []PointI, targetPos PointI) (*AOIUpdates, error) {
	player, exists := am.Players[playerID]
	if !exists || !player.isOnline {
		return nil, fmt.Errorf("player with ID '%s' not found or is offline", playerID)
	}

	updates := &AOIUpdates{
		PlayerID:           playerID,
		PlayerPos:          player.Position,
		VisibleGridObjects: make(map[string]string),
		VisiblePlayers:     make(map[string]PointI),
		Path:               path,
		TargetPos:          targetPos,
	}

	// Define the AOI bounding box to optimize iteration.
	minX := int(math.Max(0, float64(player.Position.X)-radius))
	maxX := int(math.Min(float64(am.MapData.Width-1), float64(player.Position.X)+radius))
	minY := int(math.Max(0, float64(player.Position.Y)-radius))
	maxY := int(math.Min(float64(am.MapData.Height-1), float64(player.Position.Y)+radius))

	// Iterate through the grid within the AOI bounding box.
	for y := minY; y <= maxY; y++ {
		for x := minX; x <= maxX; x++ {
			// Check if the cell is actually within the circular radius.
			if math.Hypot(float64(x-player.Position.X), float64(y-player.Position.Y)) <= radius {
				if am.MapData.Obstacles[y][x] == 1 {
					// Use a string key for JSON serialization
					updates.VisibleGridObjects[fmt.Sprintf("%d,%d", x, y)] = "Obstacle"
				}
			}
		}
	}

	// Find other players within the radius.
	for otherPlayerID, otherPlayer := range am.Players {
		if otherPlayerID != playerID && otherPlayer.isOnline {
			dist := math.Hypot(float64(otherPlayer.Position.X-player.Position.X), float64(otherPlayer.Position.Y-player.Position.Y))
			if dist <= radius {
				updates.VisiblePlayers[otherPlayerID] = otherPlayer.Position
			}
		}
	}

	return updates, nil
}

//----------------------------------------------------------------------------------------------------------------------
// 6. WebSocket Server
// This is the core of the server logic, handling connections, messages, and the game loop.
//----------------------------------------------------------------------------------------------------------------------

const (
	port         = ":8080"
	gridW        = 100
	gridH        = 100
	objectSize   = 1.0 // The object now occupies one full cell
	numObstacles = 200
	aoiRadius    = 20.0
	tickRate     = 200 * time.Millisecond // Server tick rate
	moveSpeed    = 1.0                    // Cells per tick
)

var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool {
		return true
	},
}

// Client represents a single connected player.
type Client struct {
	conn     *websocket.Conn
	playerID string
	send     chan []byte
	mutex    sync.Mutex
}

// Message is the generic communication structure between client and server.
type Message struct {
	Type    string      `json:"type"`
	Payload interface{} `json:"payload"`
}

// PathRequest is a message from the client to request a new path.
type PathRequest struct {
	X int `json:"x"`
	Y int `json:"y"`
}

// GameServer holds the entire state of the game world.
type GameServer struct {
	players     map[string]*Client
	register    chan *Client
	unregister  chan *Client
	broadcast   chan *Message
	aoiManager  *AOIManager
	pathfinder  *Pathfinder
	playerMutex sync.Mutex
}

func NewGameServer() *GameServer {
	mapData := &MapData{
		Width:     gridW,
		Height:    gridH,
		Obstacles: make([][]int, gridH),
	}
	for i := range mapData.Obstacles {
		mapData.Obstacles[i] = make([]int, gridW)
	}
	pf := NewPathfinder(gridW, gridH, objectSize)

	// Use a walkable point for both start and end, to ensure they don't overlap with obstacles
	startPos := pf.findWalkablePoint()
	endPos := pf.findWalkablePoint()

	pf.GenerateMap(numObstacles, startPos, endPos)
	for y := 0; y < gridH; y++ {
		for x := 0; x < gridW; x++ {
			mapData.Obstacles[y][x] = pf.grid[y][x]
		}
	}

	return &GameServer{
		players:    make(map[string]*Client),
		register:   make(chan *Client),
		unregister: make(chan *Client),
		broadcast:  make(chan *Message),
		aoiManager: NewAOIManager(mapData),
		pathfinder: pf,
	}
}

// run handles all server-side events.
func (gs *GameServer) run() {
	go gs.gameLoop()

	for {
		select {
		case client := <-gs.register:
			log.Printf("Client connected with ID: %s", client.playerID)
			gs.playerMutex.Lock()
			gs.players[client.playerID] = client

			// Find a random walkable start point for the new player
			startPos := gs.pathfinder.findWalkablePoint()
			gs.aoiManager.AddPlayer(client.playerID, startPos)

			initPayload := InitData{
				GridW:      gs.aoiManager.MapData.Width,
				GridH:      gs.aoiManager.MapData.Height,
				ObjectSize: objectSize,
			}
			msg := Message{Type: "init_data", Payload: initPayload}
			client.writeJSON(msg)

			gs.playerMutex.Unlock()

		case client := <-gs.unregister:
			log.Printf("Client disconnected with ID: %s", client.playerID)
			gs.playerMutex.Lock()
			if _, ok := gs.players[client.playerID]; ok {
				gs.aoiManager.RemovePlayer(client.playerID)
				delete(gs.players, client.playerID)
				close(client.send)
			}
			gs.playerMutex.Unlock()

		case message := <-gs.broadcast:
			_ = message
		}
	}
}

// gameLoop runs on a fixed tick rate and handles game state updates.
func (gs *GameServer) gameLoop() {
	ticker := time.NewTicker(tickRate)
	defer ticker.Stop()

	for range ticker.C {
		gs.playerMutex.Lock()

		for _, player := range gs.aoiManager.Players {
			if player.isOnline && len(player.path) > 0 {
				nextPoint := player.path[0]
				player.Position = nextPoint
				player.path = player.path[1:]
			}
		}

		for _, client := range gs.players {
			if player, exists := gs.aoiManager.Players[client.playerID]; exists && player.isOnline {
				updates, err := gs.aoiManager.GetAOIUpdatesForPlayer(client.playerID, aoiRadius, player.path, player.targetPos)
				if err != nil {
					log.Printf("Error getting AOI updates for player %s: %v", client.playerID, err)
					continue
				}

				msg := Message{Type: "aoi_update", Payload: updates}
				if err := client.writeJSON(msg); err != nil {
					log.Printf("Error writing to client %s: %v", client.playerID, err)
					gs.unregister <- client
				}
			}
		}

		gs.playerMutex.Unlock()
	}
}

// readPump reads messages from the WebSocket connection.
func (c *Client) readPump(server *GameServer) {
	defer func() {
		server.unregister <- c
		c.conn.Close()
	}()
	c.conn.SetReadLimit(512)
	c.conn.SetReadDeadline(time.Now().Add(60 * time.Second))
	c.conn.SetPongHandler(func(string) error { c.conn.SetReadDeadline(time.Now().Add(60 * time.Second)); return nil })
	for {
		_, message, err := c.conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("WebSocket read error: %v", err)
			}
			break
		}

		var msg Message
		if err := json.Unmarshal(message, &msg); err != nil {
			log.Printf("Error unmarshaling message: %v", err)
			continue
		}

		if msg.Type == "path_request" {
			var pathReq PathRequest
			payloadBytes, err := json.Marshal(msg.Payload)
			if err != nil {
				log.Printf("Error marshalling path request payload: %v", err)
				continue
			}
			if err := json.Unmarshal(payloadBytes, &pathReq); err != nil {
				log.Printf("Error unmarshaling path request payload: %v", err)
				continue
			}

			server.playerMutex.Lock()
			player := server.aoiManager.Players[c.playerID]
			if player != nil {
				start := player.Position
				end := PointI{X: pathReq.X, Y: pathReq.Y}

				path, target, err := server.pathfinder.findPath(start, end)
				if err != nil {
					log.Printf("Error finding path for player %s: %v", c.playerID, err)
					clientMsg := Message{Type: "path_not_found", Payload: "Cannot find a path to the requested location."}
					c.writeJSON(clientMsg)
				} else {
					player.path = path
					player.targetPos = target
					log.Printf("Player %s requested new path to (%d, %d). Path has %d steps.", c.playerID, end.X, end.Y, len(path))
				}
			}
			server.playerMutex.Unlock()
		}
	}
}

// writePump writes messages to the WebSocket connection.
func (c *Client) writePump() {
	ticker := time.NewTicker(50 * time.Second)
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
			if err := c.conn.WriteMessage(websocket.TextMessage, message); err != nil {
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

// writeJSON safely marshals and writes a JSON message.
func (c *Client) writeJSON(msg Message) error {
	c.mutex.Lock()
	defer c.mutex.Unlock()
	c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
	return c.conn.WriteJSON(msg)
}

func handleConnections(server *GameServer, w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("Failed to upgrade WebSocket connection: %v", err)
		return
	}

	playerID := uuid.New().String()
	client := &Client{
		conn:     conn,
		playerID: playerID,
		send:     make(chan []byte, 256),
	}
	server.register <- client

	go client.writePump()
	go client.readPump(server)
}

func main() {
	rand.Seed(time.Now().UnixNano())

	server := NewGameServer()
	go server.run()

	http.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		handleConnections(server, w, r)
	})

	log.Println("Go WebSocket server started on :8080")
	err := http.ListenAndServe(port, nil)
	if err != nil {
		log.Fatal("ListenAndServe: ", err)
	}
}
