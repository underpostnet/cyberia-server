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
	Type string     `json:"Type"` // e.g., "player", "obstacle"
}

// PlayerState holds the dynamic state of a player, including their path, direction, and mode.
type PlayerState struct {
	ID        string     `json:"id"`
	Pos       Point      `json:"Pos"`
	Dims      Dimensions `json:"Dims"`
	Path      []PointI   `json:"path"`
	TargetPos PointI     `json:"targetPos"`
	AOI       Rectangle
	Client    *Client
	Direction Direction       `json:"direction"`
	Mode      ObjectLayerMode `json:"mode"`
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
			return pf.reconstructPath(current), nil
		}

		neighbors := []PointI{
			{X: current.X, Y: current.Y + 1},
			{X: current.X, Y: current.Y - 1},
			{X: current.X + 1, Y: current.Y},
			{X: current.X - 1, Y: current.Y},
		}

		for _, neighbor := range neighbors {
			if !pf.isWalkable(neighbor.X, neighbor.Y, playerDims) {
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

// findClosestWalkablePoint finds the nearest walkable point to a given point. It now takes player dimensions.
func (pf *Pathfinder) findClosestWalkablePoint(point PointI, playerDims Dimensions) (PointI, error) {
	queue := []PointI{point}
	visited := make(map[int]bool)
	visited[keyFrom(point.X, point.Y, pf.gridW)] = true

	dirs := []struct{ dx, dy int }{
		{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
	}

	for len(queue) > 0 {
		curr := queue[0]
		queue = queue[1:]

		if pf.isWalkable(curr.X, curr.Y, playerDims) {
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

	return PointI{}, fmt.Errorf("could not find a walkable point near (%d, %d)", point.X, point.Y)
}

// keyFrom generates a unique key for a grid point.
func keyFrom(x, y, gridW int) int {
	return y*gridW + x
}

// heuristic calculates the Manhattan distance between two points.
func heuristic(x1, y1, x2, y2 int) float64 {
	return math.Abs(float64(x1-x2)) + math.Abs(float64(y1-y2))
}

//----------------------------------------------------------------------------------------------------------------------
// 3. Server Logic
// These methods manage the WebSocket connections, game loop, and state updates.
//----------------------------------------------------------------------------------------------------------------------

// NewGameServer creates a new GameServer instance.
func NewGameServer(gridW, gridH int, aoiRadius float64) *GameServer {
	server := &GameServer{
		clients:    make(map[string]*Client),
		players:    make(map[string]*PlayerState),
		obstacles:  make(map[string]ObjectState),
		register:   make(chan *Client),
		unregister: make(chan *Client),
		gridW:      gridW,
		gridH:      gridH,
		aoiRadius:  aoiRadius,
		pathfinder: NewPathfinder(gridW, gridH),
	}
	return server
}

// run is the main game loop.
func (server *GameServer) run() {
	// INCREASED TICK RATE: Changed game tick from 100ms to 50ms for smoother animation.
	gameTick := time.NewTicker(50 * time.Millisecond) // Game tick for state updates
	defer gameTick.Stop()

	// Initial obstacle generation
	startPoint := PointI{X: 1, Y: 1}
	endPoint := PointI{X: server.gridW - 2, Y: server.gridH - 2}
	server.pathfinder.GenerateObstacles(100, startPoint, endPoint)
	for _, obs := range server.pathfinder.obstacles {
		server.obstacles[obs.ID] = obs
	}

	for {
		select {
		case client := <-server.register:
			server.handleClientRegistration(client)
		case client := <-server.unregister:
			server.handleClientUnregistration(client)
		case <-gameTick.C:
			server.mu.Lock()
			server.updatePlayerPositions()
			server.sendAOIUpdates()
			server.mu.Unlock()
		}
	}
}

// handleClientRegistration handles new client connections.
func (server *GameServer) handleClientRegistration(client *Client) {
	log.Printf("Client registered: %s", client.playerID)
	server.mu.Lock()
	server.clients[client.playerID] = client

	// Player dimensions are now truly random for each player
	randomWidth := rand.Float64()*2 + 1  // 1.0 to 3.0
	randomHeight := rand.Float64()*2 + 1 // 1.0 to 3.0
	playerDims := Dimensions{Width: randomWidth, Height: randomHeight}

	// Create a new player state for the client
	playerPos, err := server.pathfinder.findRandomWalkablePoint(playerDims)
	if err != nil {
		log.Printf("Error finding walkable point for new player: %v", err)
		server.mu.Unlock()
		return
	}
	playerState := &PlayerState{
		ID:        client.playerID,
		Pos:       Point{X: float64(playerPos.X), Y: float64(playerPos.Y)},
		Dims:      playerDims,
		Path:      []PointI{},
		TargetPos: PointI{X: -1, Y: -1},
		AOI:       Rectangle{},
		Client:    client,
		Direction: NONE,
		Mode:      IDLE,
	}
	server.players[client.playerID] = playerState

	server.mu.Unlock()

	// Send initial game data
	server.sendInitialData(client)
}

// handleClientUnregistration removes a disconnected client.
func (server *GameServer) handleClientUnregistration(client *Client) {
	server.mu.Lock()
	if _, ok := server.clients[client.playerID]; ok {
		delete(server.clients, client.playerID)
		delete(server.players, client.playerID)
		close(client.send)
		log.Printf("Client unregistered: %s", client.playerID)
	}
	server.mu.Unlock()
}

// updatePlayerPositions moves players along their path and updates their direction and mode.
func (server *GameServer) updatePlayerPositions() {
	// A simple movement speed
	speed := 0.2

	for _, player := range server.players {
		if len(player.Path) > 0 {
			// Player is walking
			player.Mode = WALKING
			nextPoint := player.Path[0]
			targetX := float64(nextPoint.X)
			targetY := float64(nextPoint.Y)

			// Calculate direction
			dx := targetX - player.Pos.X
			dy := targetY - player.Pos.Y

			// Determine discrete direction
			if dx > 0 {
				if dy > 0 {
					player.Direction = DOWN_RIGHT
				} else if dy < 0 {
					player.Direction = UP_RIGHT
				} else {
					player.Direction = RIGHT
				}
			} else if dx < 0 {
				if dy > 0 {
					player.Direction = DOWN_LEFT
				} else if dy < 0 {
					player.Direction = UP_LEFT
				} else {
					player.Direction = LEFT
				}
			} else {
				if dy > 0 {
					player.Direction = DOWN
				} else if dy < 0 {
					player.Direction = UP
				} else {
					player.Direction = NONE
				}
			}

			// Normalize movement vector and apply speed
			dist := math.Sqrt(dx*dx + dy*dy)
			if dist > speed {
				player.Pos.X += (dx / dist) * speed
				player.Pos.Y += (dy / dist) * speed
			} else {
				player.Pos.X = targetX
				player.Pos.Y = targetY
				player.Path = player.Path[1:]
			}
		} else {
			// Player has no path, they are idle
			player.Mode = IDLE
			player.Direction = NONE
		}
	}
}

// sendAOIUpdates sends state updates to all clients based on their AOI.
func (server *GameServer) sendAOIUpdates() {
	for _, client := range server.clients {
		if !client.isPlayer {
			continue
		}

		playerState, ok := server.players[client.playerID]
		if !ok {
			continue
		}

		// Update the player's AOI bounding box
		playerState.AOI = Rectangle{
			MinX: playerState.Pos.X - server.aoiRadius,
			MinY: playerState.Pos.Y - server.aoiRadius,
			MaxX: playerState.Pos.X + playerState.Dims.Width + server.aoiRadius,
			MaxY: playerState.Pos.Y + playerState.Dims.Height + server.aoiRadius,
		}

		// Find visible players and grid objects
		visiblePlayers := make(map[string]ObjectState)
		for _, otherPlayer := range server.players {
			if otherPlayer.ID == client.playerID {
				continue
			}
			if rectsOverlap(playerState.AOI, Rectangle{
				MinX: otherPlayer.Pos.X, MinY: otherPlayer.Pos.Y,
				MaxX: otherPlayer.Pos.X + otherPlayer.Dims.Width, MaxY: otherPlayer.Pos.Y + otherPlayer.Dims.Height,
			}) {
				// The player's Dims, Direction, and Mode are now part of the ObjectState.
				visiblePlayers[otherPlayer.ID] = ObjectState{
					ID:   otherPlayer.ID,
					Pos:  otherPlayer.Pos,
					Dims: otherPlayer.Dims,
					Type: "player",
				}
			}
		}

		// Corrected Logic: Only send obstacles that are within the player's AOI
		visibleGridObjects := make(map[string]ObjectState)
		for _, obstacle := range server.obstacles {
			if rectsOverlap(playerState.AOI, Rectangle{
				MinX: obstacle.Pos.X, MinY: obstacle.Pos.Y,
				MaxX: obstacle.Pos.X + obstacle.Dims.Width, MaxY: obstacle.Pos.Y + obstacle.Dims.Height,
			}) {
				visibleGridObjects[obstacle.ID] = obstacle
			}
		}

		// Create payload
		payload := AOIUpdatePayload{
			PlayerID:           client.playerID,
			Player:             *playerState,
			VisiblePlayers:     visiblePlayers,
			VisibleGridObjects: visibleGridObjects,
		}
		server.sendJSON(client, "aoi_update", payload)
	}
}

// sendInitialData sends initial game setup data to a new client.
func (server *GameServer) sendInitialData(client *Client) {
	initData := map[string]interface{}{
		"gridW":     server.gridW,
		"gridH":     server.gridH,
		"aoiRadius": server.aoiRadius,
	}
	server.sendJSON(client, "init_data", initData)
	// NEW: Send an immediate AOI update so the client has initial state.
	server.sendAOIUpdates()
}

// sendJSON is a helper function to send a JSON message to a client.
func (server *GameServer) sendJSON(client *Client, messageType string, payload interface{}) {
	msg, err := json.Marshal(map[string]interface{}{
		"type":    messageType,
		"payload": payload,
	})
	if err != nil {
		log.Printf("Error marshaling JSON for client %s: %v", client.playerID, err)
		return
	}
	select {
	case client.send <- msg:
	default:
		// Client send channel is full, assume client is gone.
		server.handleClientUnregistration(client)
	}
}

// ServeWs handles websocket requests from the peer.
func ServeWs(server *GameServer, w http.ResponseWriter, r *http.Request) {
	conn, err := (&websocket.Upgrader{CheckOrigin: func(r *http.Request) bool { return true }}).Upgrade(w, r, nil)
	if err != nil {
		log.Println(err)
		return
	}

	playerID := uuid.New().String()
	client := &Client{conn: conn, playerID: playerID, send: make(chan []byte, 256), isPlayer: true, lastAction: time.Now()}
	server.register <- client

	go client.writePump()
	go client.readPump(server)
}

// readPump processes incoming messages from a client.
func (c *Client) readPump(server *GameServer) {
	defer func() {
		server.unregister <- c
		c.conn.Close()
	}()

	for {
		_, message, err := c.conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("error: %v", err)
			}
			break
		}
		c.handleMessage(server, message)
	}
}

// handleMessage parses and processes a message from a client.
func (c *Client) handleMessage(server *GameServer, message []byte) {
	var msg map[string]interface{}
	if err := json.Unmarshal(message, &msg); err != nil {
		log.Printf("Error unmarshaling message from client %s: %v", c.playerID, err)
		return
	}

	messageType, ok := msg["type"].(string)
	if !ok {
		log.Printf("Invalid message type from client %s", c.playerID)
		return
	}

	server.mu.Lock()
	player, playerExists := server.players[c.playerID]
	server.mu.Unlock()
	if !playerExists {
		log.Printf("Player %s not found", c.playerID)
		return
	}

	switch messageType {
	case "path_request":
		payload, ok := msg["payload"].(map[string]interface{})
		if !ok {
			log.Printf("Invalid payload for path_request from client %s", c.playerID)
			return
		}
		targetX, okX := payload["targetX"].(float64)
		targetY, okY := payload["targetY"].(float64)
		if !okX || !okY {
			log.Printf("Invalid target coordinates from client %s", c.playerID)
			return
		}

		server.mu.Lock()
		start := PointI{X: int(player.Pos.X), Y: int(player.Pos.Y)}
		target := PointI{X: int(targetX), Y: int(targetY)}

		// NEW: Pass player's unique dimensions to the Astar function
		path, err := server.pathfinder.Astar(start, target, player.Dims)
		if err != nil {
			log.Printf("Path not found from client %s: %v", c.playerID, err)
			server.sendFeedback(c, "Path not found.")
		} else {
			player.Path = path
			player.TargetPos = target
			log.Printf("Client %s requested new path to (%d, %d)", c.playerID, target.X, target.Y)
		}
		server.mu.Unlock()
	default:
		log.Printf("Unknown message type '%s' from client %s", messageType, c.playerID)
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
			w, err := c.conn.NextWriter(websocket.TextMessage)
			if err != nil {
				return
			}
			w.Write(message)

			// Add queued chat messages to the current websocket message.
			n := len(c.send)
			for i := 0; i < n; i++ {
				w.Write(<-c.send)
			}

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

func main() {
	rand.Seed(time.Now().UnixNano())
	// NEW: The game server no longer needs to be initialized with a single player dimension
	server := NewGameServer(100, 100, 20.0)
	go server.run()
	http.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		ServeWs(server, w, r)
	})
	log.Println("Starting server on :8080...")
	err := http.ListenAndServe(":8080", nil)
	if err != nil {
		log.Fatal("ListenAndServe: ", err)
	}
}
