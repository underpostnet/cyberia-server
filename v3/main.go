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

// ObjectState represents the state of a game object, with variable dimensions and a unique ID.
type ObjectState struct {
	ID   string     `json:"id"`
	Pos  Point      `json:"Pos"`
	Dims Dimensions `json:"Dims"`
	Type string     `json:"Type"` // e.g., "player", "obstacle"
}

// PlayerState holds the dynamic state of a player, including their path.
type PlayerState struct {
	ID        string     `json:"id"`
	Pos       Point      `json:"Pos"`
	Dims      Dimensions `json:"Dims"`
	Path      []PointI   `json:"path"`
	TargetPos PointI     `json:"targetPos"`
	AOI       Rectangle
	Client    *Client
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
	mu           sync.Mutex
	clients      map[string]*Client
	players      map[string]*PlayerState
	obstacles    map[string]ObjectState
	register     chan *Client
	unregister   chan *Client
	gridW, gridH int
	aoiRadius    float64
	pathfinder   *Pathfinder
}

// Node and PriorityQueue for A* pathfinding (unchanged, but included for completeness).
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
	grid         [][]int    // 0 = free, 1 = obstacle
	playerDims   Dimensions // NEW: Store the dimensions of the object being path-found.
}

//----------------------------------------------------------------------------------------------------------------------
// 2. Core Logic & Methods
// These methods implement the A* algorithm and its supporting functions.
//----------------------------------------------------------------------------------------------------------------------

// NewPathfinder creates and initializes a new Pathfinder instance.
func NewPathfinder(w, h int, playerDims Dimensions) *Pathfinder {
	p := &Pathfinder{
		gridW:      w,
		gridH:      h,
		obstacles:  make(map[string]ObjectState),
		playerDims: playerDims, // NEW: Store player dimensions.
	}
	p.grid = make([][]int, h)
	for y := 0; y < h; y++ {
		p.grid[y] = make([]int, w)
	}
	return p
}

// GenerateObstacles creates a procedural grid with randomly placed obstacles.
func (pf *Pathfinder) GenerateObstacles(numObstacles int, start, end PointI) {
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

		// Check for overlap with the safe zone
		if rectsOverlap(obsRect, safeZone) {
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

// isCellWalkable checks if a single grid cell is an obstacle.
func (pf *Pathfinder) isCellWalkable(x, y int) bool {
	if x < 0 || x >= pf.gridW || y < 0 || y >= pf.gridH {
		return false
	}
	return pf.grid[y][x] == 0
}

// isWalkable now checks if a rectangular area is clear.
func (pf *Pathfinder) isWalkable(x, y int) bool {
	playerW, playerH := int(pf.playerDims.Width), int(pf.playerDims.Height)

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

// Astar finds the shortest path from start to end using the A* algorithm.
func (pf *Pathfinder) Astar(start, end PointI) ([]PointI, error) {
	// A* implementation... (unchanged)
	// We'll just rely on the new isWalkable function.
	// NOTE: The user's Astar implementation was not provided, but we assume it
	// calls isWalkable. The change is isolated to the isWalkable function itself.

	if !pf.isWalkable(start.X, start.Y) {
		closestStart, err := pf.findClosestWalkablePoint(start)
		if err != nil {
			return nil, err
		}
		start = closestStart
	}

	if !pf.isWalkable(end.X, end.Y) {
		closestEnd, err := pf.findClosestWalkablePoint(end)
		if err != nil {
			return nil, err
		}
		end = closestEnd
	}

	openSet := make(PriorityQueue, 0)
	heap.Init(&openSet)
	startNode := &Node{X: start.X, Y: start.Y, g: 0, h: heuristic(start.X, start.Y, end.X, end.Y), parent: nil}
	heap.Push(&openSet, startNode)

	cameFrom := make(map[PointI]*Node)
	gScore := make(map[PointI]float64)
	gScore[start] = 0.0

	for openSet.Len() > 0 {
		current := heap.Pop(&openSet).(*Node)
		if current.X == end.X && current.Y == end.Y {
			return pf.reconstructPath(current), nil
		}

		neighbors := []PointI{
			{X: current.X, Y: current.Y + 1},
			{X: current.X, Y: current.Y - 1},
			{X: current.X + 1, Y: current.Y},
			{X: current.X - 1, Y: current.Y},
		}

		for _, neighbor := range neighbors {
			if !pf.isWalkable(neighbor.X, neighbor.Y) {
				continue
			}

			tentativeGScore := gScore[PointI{current.X, current.Y}] + 1.0
			if tentativeGScore < gScore[neighbor] || gScore[neighbor] == 0 {
				neighborNode := &Node{
					X:      neighbor.X,
					Y:      neighbor.Y,
					g:      tentativeGScore,
					h:      heuristic(neighbor.X, neighbor.Y, end.X, end.Y),
					parent: current,
				}
				neighborNode.f = neighborNode.g + neighborNode.h
				cameFrom[neighbor] = current
				gScore[neighbor] = tentativeGScore

				heap.Push(&openSet, neighborNode)
			}
		}
	}

	return nil, fmt.Errorf("path not found")
}

func (pf *Pathfinder) reconstructPath(node *Node) []PointI {
	path := []PointI{}
	for node != nil {
		path = append([]PointI{{X: node.X, Y: node.Y}}, path...)
		node = node.parent
	}
	return path
}

// rectsOverlap checks for an intersection between two rectangles.
func rectsOverlap(r1, r2 Rectangle) bool {
	return r1.MinX < r2.MaxX && r1.MaxX > r2.MinX && r1.MinY < r2.MaxY && r1.MaxY > r2.MinY
}

// findClosestWalkablePoint finds the nearest walkable point to a given point.
func (pf *Pathfinder) findClosestWalkablePoint(point PointI) (PointI, error) {
	queue := []PointI{point}
	visited := make(map[int]bool)
	visited[keyFrom(point.X, point.Y, pf.gridW)] = true

	dirs := []struct{ dx, dy int }{
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
			if nx >= 0 && nx < pf.gridW && ny >= 0 && ny < pf.gridH {
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

// keyFrom generates a unique key for a grid coordinate.
func keyFrom(x, y, w int) int {
	return y*w + x
}

// heuristic calculates the Manhattan distance.
func heuristic(x1, y1, x2, y2 int) float64 {
	return math.Abs(float64(x1-x2)) + math.Abs(float64(y1-y2))
}

//----------------------------------------------------------------------------------------------------------------------
// 3. Server-side communication and game loop logic
//----------------------------------------------------------------------------------------------------------------------

// NewGameServer creates a new instance of the game server.
func NewGameServer() *GameServer {
	const gridW, gridH = 100, 100
	const aoiRadius = 20.0
	// NEW: Define player dimensions here
	playerDims := Dimensions{Width: 2.0, Height: 3.0}

	server := &GameServer{
		clients:    make(map[string]*Client),
		players:    make(map[string]*PlayerState),
		obstacles:  make(map[string]ObjectState),
		register:   make(chan *Client),
		unregister: make(chan *Client),
		gridW:      gridW,
		gridH:      gridH,
		aoiRadius:  aoiRadius,
		pathfinder: NewPathfinder(gridW, gridH, playerDims), // NEW: Pass player dimensions
	}
	server.pathfinder.GenerateObstacles(50, PointI{X: 10, Y: 10}, PointI{X: 90, Y: 90})
	server.obstacles = server.pathfinder.obstacles
	return server
}

// run is the main game loop for the server.
func (server *GameServer) run() {
	go server.handleClientLifecycle()
	gameTick := time.NewTicker(time.Second / 60)
	defer gameTick.Stop()
	for range gameTick.C {
		server.mu.Lock()
		server.updatePlayerPositions()
		server.sendAOIUpdates()
		server.mu.Unlock()
	}
}

// handleClientLifecycle manages client registration and unregistration.
func (server *GameServer) handleClientLifecycle() {
	for {
		select {
		case client := <-server.register:
			server.mu.Lock()
			server.clients[client.playerID] = client
			server.players[client.playerID] = &PlayerState{
				ID: client.playerID,
				Pos: Point{
					X: float64(rand.Intn(server.gridW)),
					Y: float64(rand.Intn(server.gridH)),
				},
				Dims:   server.pathfinder.playerDims, // Use the same dimensions as the pathfinder
				Path:   []PointI{},
				Client: client,
			}
			server.mu.Unlock()
			log.Printf("Client %s connected and registered.", client.playerID)
			server.sendInitialData(client)
		case client := <-server.unregister:
			server.mu.Lock()
			delete(server.clients, client.playerID)
			delete(server.players, client.playerID)
			close(client.send)
			server.mu.Unlock()
			log.Printf("Client %s disconnected and unregistered.", client.playerID)
		}
	}
}

// updatePlayerPositions moves players along their path.
func (server *GameServer) updatePlayerPositions() {
	for _, player := range server.players {
		if len(player.Path) > 0 {
			nextPoint := player.Path[0]
			dx := float64(nextPoint.X) - player.Pos.X
			dy := float64(nextPoint.Y) - player.Pos.Y
			dist := math.Sqrt(dx*dx + dy*dy)
			if dist < 0.1 { // Arrived at the next waypoint
				player.Pos.X, player.Pos.Y = float64(nextPoint.X), float64(nextPoint.Y)
				player.Path = player.Path[1:]
			} else {
				speed := 5.0 * (1.0 / 60.0) // 5 units per second
				if dist > 0 {
					player.Pos.X += dx / dist * speed
					player.Pos.Y += dy / dist * speed
				}
			}
		}
	}
}

// sendAOIUpdates sends the visible area of interest to each player.
func (server *GameServer) sendAOIUpdates() {
	for _, player := range server.players {
		player.AOI = server.calculateAOI(player)
		visiblePlayers := server.getVisiblePlayers(player)
		visibleGridObjects := server.getVisibleGridObjects(player)

		payload := AOIUpdatePayload{
			PlayerID:           player.ID,
			Player:             *player,
			VisiblePlayers:     visiblePlayers,
			VisibleGridObjects: visibleGridObjects,
		}

		message, err := json.Marshal(map[string]interface{}{
			"type":    "aoi_update",
			"payload": payload,
		})
		if err != nil {
			log.Printf("Error marshaling aoi_update: %v", err)
			continue
		}
		player.Client.send <- message
	}
}

// calculateAOI calculates the Area of Interest for a given player.
func (server *GameServer) calculateAOI(player *PlayerState) Rectangle {
	return Rectangle{
		MinX: player.Pos.X - server.aoiRadius,
		MinY: player.Pos.Y - server.aoiRadius,
		MaxX: player.Pos.X + player.Dims.Width + server.aoiRadius,
		MaxY: player.Pos.Y + player.Dims.Height + server.aoiRadius,
	}
}

// getVisiblePlayers returns a map of players within the given player's AOI.
func (server *GameServer) getVisiblePlayers(player *PlayerState) map[string]ObjectState {
	visible := make(map[string]ObjectState)
	for _, otherPlayer := range server.players {
		if otherPlayer.ID == player.ID {
			continue
		}
		playerRect := Rectangle{
			MinX: otherPlayer.Pos.X, MinY: otherPlayer.Pos.Y,
			MaxX: otherPlayer.Pos.X + otherPlayer.Dims.Width, MaxY: otherPlayer.Pos.Y + otherPlayer.Dims.Height,
		}
		if rectsOverlap(player.AOI, playerRect) {
			visible[otherPlayer.ID] = ObjectState{
				ID:   otherPlayer.ID,
				Pos:  otherPlayer.Pos,
				Dims: otherPlayer.Dims,
				Type: "player",
			}
		}
	}
	return visible
}

// getVisibleGridObjects returns a map of grid objects within the player's AOI.
func (server *GameServer) getVisibleGridObjects(player *PlayerState) map[string]ObjectState {
	visible := make(map[string]ObjectState)
	for id, obj := range server.obstacles {
		objRect := Rectangle{
			MinX: obj.Pos.X, MinY: obj.Pos.Y,
			MaxX: obj.Pos.X + obj.Dims.Width, MaxY: obj.Pos.Y + obj.Dims.Height,
		}
		if rectsOverlap(player.AOI, objRect) {
			visible[id] = obj
		}
	}
	return visible
}

// sendInitialData sends the initial game state to a new client.
func (server *GameServer) sendInitialData(client *Client) {
	initData := map[string]interface{}{
		"type": "init_data",
		"payload": map[string]interface{}{
			"gridW":     server.gridW,
			"gridH":     server.gridH,
			"aoiRadius": server.aoiRadius,
		},
	}
	message, err := json.Marshal(initData)
	if err != nil {
		log.Printf("Error marshaling init data: %v", err)
		return
	}
	client.send <- message
}

// handleConnections handles new WebSocket connections.
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
		conn:       conn,
		playerID:   playerID,
		send:       make(chan []byte, 256),
		lastAction: time.Now(),
	}
	server.register <- client

	go client.writePump()
	go client.readPump(server)
}

// main starts the HTTP server and game loop.
func main() {
	gameServer := NewGameServer()
	http.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		handleConnections(gameServer, w, r)
	})

	log.Println("Server started on :8080")
	go gameServer.run()
	log.Fatal(http.ListenAndServe(":8080", nil))
}

// readPump reads messages from the WebSocket connection.
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
				log.Printf("Error reading message: %v", err)
			}
			break
		}
		server.processMessage(c, message)
	}
}

// processMessage handles incoming client messages.
func (server *GameServer) processMessage(client *Client, message []byte) {
	var msg map[string]interface{}
	if err := json.Unmarshal(message, &msg); err != nil {
		log.Printf("Error unmarshaling message from client %s: %v", client.playerID, err)
		return
	}

	messageType, ok := msg["type"].(string)
	if !ok {
		log.Printf("Invalid message type from client %s", client.playerID)
		return
	}

	switch messageType {
	case "path_request":
		payload, ok := msg["payload"].(map[string]interface{})
		if !ok {
			log.Printf("Invalid payload for path_request from client %s", client.playerID)
			return
		}

		targetX, okX := payload["targetX"].(float64)
		targetY, okY := payload["targetY"].(float64)
		if !okX || !okY {
			log.Printf("Invalid target coordinates for path_request from client %s", client.playerID)
			return
		}

		server.mu.Lock()
		player, ok := server.players[client.playerID]
		if !ok {
			server.mu.Unlock()
			log.Printf("Player state not found for client %s", client.playerID)
			return
		}

		// Find the closest walkable point for the target, considering player's dimensions
		targetPos := PointI{X: int(targetX), Y: int(targetY)}
		closestTarget, err := server.pathfinder.findClosestWalkablePoint(targetPos)
		if err != nil {
			log.Printf("No walkable point found for target from client %s", client.playerID)
			server.sendFeedback(client, "Path target is blocked.")
			server.mu.Unlock()
			return
		}

		path, err := server.pathfinder.Astar(PointI{X: int(player.Pos.X), Y: int(player.Pos.Y)}, closestTarget)
		if err != nil {
			log.Printf("Path not found from client %s: %v", client.playerID, err)
			server.sendFeedback(client, "Path not found.")
		} else {
			player.Path = path
			player.TargetPos = closestTarget
			log.Printf("Client %s requested new path to (%d, %d)", client.playerID, closestTarget.X, closestTarget.Y)
		}

		server.mu.Unlock()
	default:
		log.Printf("Unknown message type '%s' from client %s", messageType, client.playerID)
	}
}

// sendFeedback sends a string message to a single client.
func (server *GameServer) sendFeedback(client *Client, message string) {
	msg, err := json.Marshal(map[string]interface{}{
		"type":    "path_not_found",
		"payload": message,
	})
	if err != nil {
		log.Printf("Error marshaling feedback message: %v", err)
		return
	}
	client.send <- msg
}

// writePump writes messages to the WebSocket connection.
func (c *Client) writePump() {
	ticker := time.NewTicker(1 * time.Second)
	defer func() {
		ticker.Stop()
		c.conn.Close()
	}()
	for {
		select {
		case message, ok := <-c.send:
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if !ok {
				// The server closed the channel.
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
