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

// ObjectState represents the state of a game object, now with variable dimensions.
type ObjectState struct {
	X, Y          int
	Width, Height float64
	Type          string // e.g., "player", "obstacle"
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
// It now stores the dimensions of the object it is pathfinding for.
type Pathfinder struct {
	gridW, gridH              int
	obstacles                 []Rectangle
	objectWidth, objectHeight float64
	grid                      [][]int // 0 = free, 1 = obstacle
	start, end                PointI
}

//----------------------------------------------------------------------------------------------------------------------
// 2. Core Logic & Methods
// These methods implement the A* algorithm and its supporting functions.
//----------------------------------------------------------------------------------------------------------------------

// NewPathfinder creates and initializes a new Pathfinder instance with object dimensions.
func NewPathfinder(w, h int, objW, objH float64) *Pathfinder {
	p := &Pathfinder{
		gridW:        w,
		gridH:        h,
		objectWidth:  objW,
		objectHeight: objH,
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
	halfW := pf.objectWidth / 2.0
	halfH := pf.objectHeight / 2.0

	// Clamp the point to be within the grid boundaries, considering the object's size
	clampedPoint := PointI{
		X: int(math.Max(math.Floor(halfW), math.Min(float64(point.X), float64(pf.gridW)-math.Ceil(halfW)))),
		Y: int(math.Max(math.Floor(halfH), math.Min(float64(point.Y), float64(pf.gridH)-math.Ceil(halfH)))),
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
			// Ensure the new point is within the bounds, considering the object's size
			if float64(nx) >= math.Floor(halfW) && float64(nx) <= float64(pf.gridW)-math.Ceil(halfW) &&
				float64(ny) >= math.Floor(halfH) && float64(ny) <= float64(pf.gridH)-math.Ceil(halfH) {
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
// This function is refactored to use objectWidth and objectHeight.
func (pf *Pathfinder) isWalkable(x, y int) bool {
	halfW := pf.objectWidth / 2.0
	halfH := pf.objectHeight / 2.0

	objRect := Rectangle{
		MinX: float64(x) - halfW,
		MinY: float64(y) - halfH,
		MaxX: float64(x) + halfW,
		MaxY: float64(y) + halfH,
	}

	if objRect.MinX < 0 || objRect.MaxX > float64(pf.gridW) ||
		objRect.MinY < 0 || objRect.MaxY > float64(pf.gridH) {
		return false
	}

	if pf.CheckCollision(objRect) {
		return false
	}

	return true
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
	return a.MinX < b.MaxX && a.MaxX > b.MinX &&
		a.MinY < b.MaxY && a.MaxY > b.MinY
}

// keyFrom provides a unique integer key for a grid coordinate.
func keyFrom(x, y, w int) int {
	return x + y*w
}

//----------------------------------------------------------------------------------------------------------------------
// 4. Server & Networking
// These components handle WebSocket connections, game state, and broadcasting updates.
// The GameServer now manages players with object dimensions.
//----------------------------------------------------------------------------------------------------------------------

// Message defines the structure for messages sent and received by the server.
type Message struct {
	Type    string      `json:"type"`
	Payload interface{} `json:"payload"`
}

// initData is the payload for the initial connection message.
// It now includes default object dimensions.
type initData struct {
	GridW        int     `json:"gridW"`
	GridH        int     `json:"gridH"`
	ObjectWidth  float64 `json:"objectWidth"`
	ObjectHeight float64 `json:"objectHeight"`
}

// PlayerDimensions is a new struct to correctly serialize player dimensions.
type PlayerDimensions struct {
	Width, Height float64
}

// aoiUpdate is the payload for the Area of Interest update.
// It now uses the new PlayerDimensions struct.
type aoiUpdate struct {
	PlayerID           string                 `json:"playerID"`
	PlayerPos          Point                  `json:"playerPos"`
	PlayerDimensions   PlayerDimensions       `json:"playerDimensions"`
	VisiblePlayers     map[string]ObjectState `json:"visiblePlayers"`
	VisibleGridObjects map[string]ObjectState `json:"visibleGridObjects"`
	Path               []PointI               `json:"path"`
	TargetPos          *PointI                `json:"targetPos"`
}

// playerState holds all the state for a connected client, now with object dimensions.
type playerState struct {
	ID            string
	Pos           PointI
	Path          []PointI
	Target        *PointI
	Speed         float64
	LastMoveTime  time.Time
	mu            sync.RWMutex
	Width, Height float64
}

// Client represents a single WebSocket connection.
type Client struct {
	conn     *websocket.Conn
	playerID string
	send     chan []byte
}

// GameServer manages all clients and the game state.
type GameServer struct {
	clients      map[*Client]bool
	players      map[string]*playerState
	register     chan *Client
	unregister   chan *Client
	broadcast    chan []byte
	pathfinder   *Pathfinder
	gridW, gridH int
	aoiRadius    int
	mu           sync.RWMutex
}

// NewGameServer creates and initializes a new GameServer.
func NewGameServer(w, h, aoi int, objW, objH float64) *GameServer {
	return &GameServer{
		clients:    make(map[*Client]bool),
		players:    make(map[string]*playerState),
		register:   make(chan *Client),
		unregister: make(chan *Client),
		broadcast:  make(chan []byte),
		pathfinder: NewPathfinder(w, h, objW, objH),
		gridW:      w,
		gridH:      h,
		aoiRadius:  aoi,
	}
}

// Run starts the game server's main loop.
func (s *GameServer) Run() {
	ticker := time.NewTicker(100 * time.Millisecond) // Update tick rate
	defer func() {
		ticker.Stop()
	}()

	s.pathfinder.GenerateMap(50, PointI{X: 0, Y: 0}, PointI{X: s.gridW - 1, Y: s.gridH - 1})

	for {
		select {
		case client := <-s.register:
			s.mu.Lock()
			s.clients[client] = true

			// Initialize player state with variable dimensions.
			playerPos := s.pathfinder.findWalkablePoint()
			s.players[client.playerID] = &playerState{
				ID:           client.playerID,
				Pos:          playerPos,
				Speed:        1.0,
				LastMoveTime: time.Now(),
				Width:        1.0, // Default to 1 for new players
				Height:       1.0, // Default to 1 for new players
			}
			s.mu.Unlock()

			// Send initial data to the new client.
			initMsg, _ := json.Marshal(Message{
				Type: "init_data",
				Payload: initData{
					GridW:        s.gridW,
					GridH:        s.gridH,
					ObjectWidth:  1.0,
					ObjectHeight: 1.0,
				},
			})
			client.send <- initMsg
			log.Printf("Client registered: %s", client.playerID)

		case client := <-s.unregister:
			s.mu.Lock()
			if _, ok := s.clients[client]; ok {
				delete(s.clients, client)
				delete(s.players, client.playerID)
				close(client.send)
				log.Printf("Client unregistered: %s", client.playerID)
			}
			s.mu.Unlock()

		case <-ticker.C:
			s.updatePlayers()
			s.broadcastAoiUpdates()
		}
	}
}

// updatePlayers updates the position of all players.
func (s *GameServer) updatePlayers() {
	s.mu.Lock()
	defer s.mu.Unlock()
	for _, player := range s.players {
		player.mu.Lock()
		if len(player.Path) > 0 {
			now := time.Now()
			elapsed := now.Sub(player.LastMoveTime).Seconds()
			player.LastMoveTime = now

			// Calculate a new position based on speed and elapsed time
			if elapsed > 0 {
				newPos := player.Path[0]
				player.Pos = newPos
				player.Path = player.Path[1:]
			}
		}
		player.mu.Unlock()
	}
}

// broadcastAoiUpdates sends Area of Interest updates to all clients.
func (s *GameServer) broadcastAoiUpdates() {
	s.mu.RLock()
	defer s.mu.RUnlock()

	for client := range s.clients {
		player := s.players[client.playerID]
		if player == nil {
			continue
		}

		player.mu.RLock()
		visiblePlayers := make(map[string]ObjectState)
		visibleGridObjects := make(map[string]ObjectState)

		// Find visible players
		for _, otherPlayer := range s.players {
			if otherPlayer.ID == player.ID {
				continue
			}
			if s.distance(player.Pos, otherPlayer.Pos) <= float64(s.aoiRadius) {
				otherPlayer.mu.RLock()
				visiblePlayers[otherPlayer.ID] = ObjectState{
					X: otherPlayer.Pos.X, Y: otherPlayer.Pos.Y,
					Width: otherPlayer.Width, Height: otherPlayer.Height,
					Type: "player",
				}
				otherPlayer.mu.RUnlock()
			}
		}

		// Find visible obstacles
		for _, obs := range s.pathfinder.obstacles {
			if s.distanceToRect(player.Pos, obs) <= float64(s.aoiRadius) {
				// Obstacles don't have a single PointI, so we'll approximate for the client.
				// For the client, we'll send the min/max as a single object.
				visibleGridObjects[fmt.Sprintf("%d,%d", int(obs.MinX), int(obs.MinY))] = ObjectState{
					X: int(obs.MinX), Y: int(obs.MinY),
					Width: obs.MaxX - obs.MinX + 1, Height: obs.MaxY - obs.MinY + 1,
					Type: "obstacle",
				}
			}
		}

		targetPos := player.Target

		update := aoiUpdate{
			PlayerID:           player.ID,
			PlayerPos:          Point{X: float64(player.Pos.X), Y: float64(player.Pos.Y)},
			PlayerDimensions:   PlayerDimensions{Width: player.Width, Height: player.Height},
			VisiblePlayers:     visiblePlayers,
			VisibleGridObjects: visibleGridObjects,
			Path:               player.Path,
			TargetPos:          targetPos,
		}
		player.mu.RUnlock()

		msg, err := json.Marshal(Message{
			Type:    "aoi_update",
			Payload: update,
		})
		if err != nil {
			log.Printf("Failed to marshal AOI update: %v", err)
			continue
		}

		select {
		case client.send <- msg:
		default:
			close(client.send)
			delete(s.clients, client)
		}
	}
}

// distance calculates the Euclidean distance between two grid points.
func (s *GameServer) distance(p1, p2 PointI) float64 {
	dx := float64(p1.X - p2.X)
	dy := float64(p1.Y - p2.Y)
	return math.Sqrt(dx*dx + dy*dy)
}

// distanceToRect calculates the shortest distance from a point to a rectangle.
func (s *GameServer) distanceToRect(p PointI, r Rectangle) float64 {
	dx := math.Max(0, math.Max(r.MinX-float64(p.X), float64(p.X)-r.MaxX))
	dy := math.Max(0, math.Max(r.MinY-float64(p.Y), float64(p.Y)-r.MaxY))
	return math.Sqrt(dx*dx + dy*dy)
}

// handlePathRequest handles incoming path requests.
func (s *GameServer) handlePathRequest(client *Client, payload map[string]interface{}) {
	x, okX := payload["x"].(float64)
	y, okY := payload["y"].(float64)

	if !okX || !okY {
		log.Println("Invalid path request payload.")
		return
	}

	target := PointI{X: int(x), Y: int(y)}

	s.mu.Lock()
	defer s.mu.Unlock()

	player, ok := s.players[client.playerID]
	if !ok {
		log.Printf("Player %s not found.", client.playerID)
		return
	}

	player.mu.Lock()
	defer player.mu.Unlock()

	// Re-initialize pathfinder for this player's dimensions
	s.pathfinder.objectWidth = player.Width
	s.pathfinder.objectHeight = player.Height
	path, finalTarget, err := s.pathfinder.findPath(player.Pos, target)

	if err != nil {
		log.Printf("Path not found for player %s: %v", client.playerID, err)
		msg, _ := json.Marshal(Message{
			Type:    "path_not_found",
			Payload: "No path could be found to the target location.",
		})
		client.send <- msg
		player.Path = []PointI{}
		player.Target = nil
	} else {
		player.Path = path
		player.Target = &finalTarget
		log.Printf("Path found for player %s, length %d", client.playerID, len(path))
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
				log.Printf("error: %v", err)
			}
			break
		}
		var msg Message
		if err := json.Unmarshal(message, &msg); err != nil {
			log.Printf("failed to unmarshal message: %v", err)
			continue
		}

		if msg.Type == "path_request" {
			if payload, ok := msg.Payload.(map[string]interface{}); ok {
				server.handlePathRequest(c, payload)
			}
		}
	}
}

// writePump writes messages to the WebSocket connection.
func (c *Client) writePump() {
	ticker := time.NewTicker(54 * time.Second)
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

func handleConnections(server *GameServer, w http.ResponseWriter, r *http.Request) {
	upgrader := websocket.Upgrader{
		CheckOrigin: func(r *http.Request) bool { return true },
	}
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
	const gridW, gridH = 100, 100
	const aoiRadius = 20
	const objW, objH = 1.0, 1.0

	gameServer := NewGameServer(gridW, gridH, aoiRadius, objW, objH)
	go gameServer.Run()

	http.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		handleConnections(gameServer, w, r)
	})

	log.Println("Server started on :8080")
	if err := http.ListenAndServe(":8080", nil); err != nil {
		log.Fatal("ListenAndServe: ", err)
	}
}
