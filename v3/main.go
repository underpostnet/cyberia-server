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
func (pf *Pathfinder) getNeighbors(n *Node) []*Node {
	neighbors := make([]*Node, 0)
	for dx := -1; dx <= 1; dx++ {
		for dy := -1; dy <= 1; dy++ {
			if dx == 0 && dy == 0 {
				continue
			}
			x, y := n.X+dx, n.Y+dy
			if x >= 0 && x < pf.gridW && y >= 0 && y < pf.gridH {
				neighbors = append(neighbors, &Node{X: x, Y: y})
			}
		}
	}
	return neighbors
}

// reconstructPath rebuilds the path from the end node.
func reconstructPath(n *Node) []PointI {
	path := make([]PointI, 0)
	for n != nil {
		path = append([]PointI{{X: n.X, Y: n.Y}}, path...)
		n = n.parent
	}
	return path
}

// heuristic calculates the Manhattan distance between two points.
func heuristic(x1, y1, x2, y2 int) float64 {
	return math.Abs(float64(x1-x2)) + math.Abs(float64(y1-y2))
}

// rectsOverlap checks for overlap between two rectangles.
func rectsOverlap(r1, r2 Rectangle) bool {
	return r1.MinX < r2.MaxX && r1.MaxX > r2.MinX &&
		r1.MinY < r2.MaxY && r1.MaxY > r2.MinY
}

// findClosestWalkablePoint finds the closest walkable point to a given point.
func (pf *Pathfinder) findClosestWalkablePoint(p PointI, playerDims Dimensions) (PointI, error) {
	// A simple but potentially slow approach:
	for r := 0; r < int(math.Max(float64(pf.gridW), float64(pf.gridH))); r++ {
		for x := p.X - r; x <= p.X+r; x++ {
			for y := p.Y - r; y <= p.Y+r; y++ {
				if pf.isWalkable(x, y, playerDims) {
					return PointI{X: x, Y: y}, nil
				}
			}
		}
	}
	return PointI{}, fmt.Errorf("could not find any walkable point")
}

//----------------------------------------------------------------------------------------------------------------------
// 3. Server & Client Communication
// This section handles WebSocket connections, message parsing, and game state updates.
//----------------------------------------------------------------------------------------------------------------------

// NewGameServer creates and returns a new GameServer instance.
func NewGameServer(w, h int, aoiRadius float64) *GameServer {
	return &GameServer{
		clients:    make(map[string]*Client),
		players:    make(map[string]*PlayerState),
		obstacles:  make(map[string]ObjectState),
		register:   make(chan *Client),
		unregister: make(chan *Client),
		gridW:      w,
		gridH:      h,
		aoiRadius:  aoiRadius,
		pathfinder: NewPathfinder(w, h),
	}
}

// run is the main game loop.
func (s *GameServer) run() {
	ticker := time.NewTicker(time.Millisecond * 50)
	defer ticker.Stop()
	for {
		select {
		case client := <-s.register:
			s.handleNewClient(client)
		case client := <-s.unregister:
			s.handleClientDisconnect(client)
		case <-ticker.C:
			s.updateGameState()
			s.sendAOIUpdates()
		}
	}
}

// handleNewClient registers a new client and creates a player.
func (s *GameServer) handleNewClient(client *Client) {
	log.Printf("New client connected with ID: %s", client.playerID)
	s.mu.Lock()
	s.clients[client.playerID] = client

	// Create a new player state for the client
	playerDims := Dimensions{
		Width:  1.0 + rand.Float64()*2.0,
		Height: 1.0 + rand.Float64()*2.0,
	}

	spawnPoint, err := s.pathfinder.findRandomWalkablePoint(playerDims)
	if err != nil {
		log.Printf("Failed to find spawn point for new player: %v", err)
		s.mu.Unlock()
		return
	}

	player := &PlayerState{
		ID:        client.playerID,
		Pos:       Point{X: float64(spawnPoint.X), Y: float64(spawnPoint.Y)},
		Dims:      playerDims,
		Path:      []PointI{},
		TargetPos: PointI{X: -1, Y: -1},
		AOI:       Rectangle{},
		Client:    client,
		Direction: NONE,
		Mode:      IDLE,
	}
	s.players[client.playerID] = player

	// Send initial game data
	s.sendInitData(client)

	// Send the first AOI update
	s.sendAOIUpdate(player)

	s.mu.Unlock()
}

// handleClientDisconnect unregisters a client and removes their player.
func (s *GameServer) handleClientDisconnect(client *Client) {
	s.mu.Lock()
	if _, ok := s.clients[client.playerID]; ok {
		delete(s.clients, client.playerID)
		delete(s.players, client.playerID)
		close(client.send)
		log.Printf("Client disconnected with ID: %s", client.playerID)
	}
	s.mu.Unlock()
}

// updateGameState updates player positions and other game logic.
func (s *GameServer) updateGameState() {
	s.mu.Lock()
	defer s.mu.Unlock()

	for _, player := range s.players {
		if player.Mode == WALKING && len(player.Path) > 0 {
			s.movePlayer(player)
		} else {
			player.Mode = IDLE
		}
	}
}

// movePlayer moves a player one step along their path.
func (s *GameServer) movePlayer(player *PlayerState) {
	// A simple movement logic: move to the next point in the path.
	// This can be replaced with interpolation for smoother movement.
	nextPoint := player.Path[0]
	player.Pos.X = float64(nextPoint.X)
	player.Pos.Y = float64(nextPoint.Y)

	// Calculate direction based on old and new position
	oldPoint := player.Path[0]
	if len(player.Path) > 1 {
		oldPoint = player.Path[1]
	}
	s.updatePlayerDirection(player, oldPoint, nextPoint)

	player.Path = player.Path[1:] // Remove the current point
	if len(player.Path) == 0 {
		player.Mode = IDLE
	}
}

// updatePlayerDirection sets the player's direction based on movement.
func (s *GameServer) updatePlayerDirection(player *PlayerState, old, new PointI) {
	dx := new.X - old.X
	dy := new.Y - old.Y

	if dx == 0 && dy < 0 {
		player.Direction = UP
	} else if dx > 0 && dy < 0 {
		player.Direction = UP_RIGHT
	} else if dx > 0 && dy == 0 {
		player.Direction = RIGHT
	} else if dx > 0 && dy > 0 {
		player.Direction = DOWN_RIGHT
	} else if dx == 0 && dy > 0 {
		player.Direction = DOWN
	} else if dx < 0 && dy > 0 {
		player.Direction = DOWN_LEFT
	} else if dx < 0 && dy == 0 {
		player.Direction = LEFT
	} else if dx < 0 && dy < 0 {
		player.Direction = UP_LEFT
	} else {
		player.Direction = NONE
	}
}

// calculateAOI calculates the Area of Interest for a given player.
func (s *GameServer) calculateAOI(player *PlayerState) Rectangle {
	// Simple AOI: a rectangle centered on the player.
	minX := player.Pos.X - s.aoiRadius
	minY := player.Pos.Y - s.aoiRadius
	maxX := player.Pos.X + player.Dims.Width + s.aoiRadius
	maxY := player.Pos.Y + player.Dims.Height + s.aoiRadius
	return Rectangle{MinX: minX, MinY: minY, MaxX: maxX, MaxY: maxY}
}

// getVisiblePlayers returns a map of other players within a given AOI.
func (s *GameServer) getVisiblePlayers(playerID string, aoi Rectangle) map[string]ObjectState {
	visiblePlayers := make(map[string]ObjectState)
	for id, otherPlayer := range s.players {
		if id == playerID {
			continue
		}
		// Check if the other player's bounding box overlaps with the AOI rectangle
		otherPlayerRect := Rectangle{
			MinX: otherPlayer.Pos.X,
			MinY: otherPlayer.Pos.Y,
			MaxX: otherPlayer.Pos.X + otherPlayer.Dims.Width,
			MaxY: otherPlayer.Pos.Y + otherPlayer.Dims.Height,
		}
		if rectsOverlap(aoi, otherPlayerRect) {
			visiblePlayers[id] = ObjectState{
				ID:   otherPlayer.ID,
				Pos:  otherPlayer.Pos,
				Dims: otherPlayer.Dims,
				Type: "player",
			}
		}
	}
	return visiblePlayers
}

// getVisibleGridObjects returns a map of grid objects (obstacles) within a given AOI.
func (s *GameServer) getVisibleGridObjects(aoi Rectangle) map[string]ObjectState {
	visibleObjects := make(map[string]ObjectState)
	for id, obstacle := range s.obstacles {
		obstacleRect := Rectangle{
			MinX: obstacle.Pos.X,
			MinY: obstacle.Pos.Y,
			MaxX: obstacle.Pos.X + obstacle.Dims.Width,
			MaxY: obstacle.Pos.Y + obstacle.Dims.Height,
		}
		if rectsOverlap(aoi, obstacleRect) {
			visibleObjects[id] = obstacle
		}
	}
	return visibleObjects
}

// sendInitData sends the initial game setup data to a new client.
func (s *GameServer) sendInitData(client *Client) {
	initMsg, _ := json.Marshal(map[string]interface{}{
		"type": "init_data",
		"payload": map[string]interface{}{
			"gridW":     s.gridW,
			"gridH":     s.gridH,
			"aoiRadius": s.aoiRadius,
		},
	})
	select {
	case client.send <- initMsg:
	default:
		s.handleClientDisconnect(client)
	}
}

// sendAOIUpdates sends an AOI update message to all connected players.
func (s *GameServer) sendAOIUpdates() {
	s.mu.Lock()
	defer s.mu.Unlock()

	for _, player := range s.players {
		s.sendAOIUpdate(player)
	}
}

// sendAOIUpdate sends a single AOI update to a specific player.
func (s *GameServer) sendAOIUpdate(player *PlayerState) {
	player.AOI = s.calculateAOI(player)
	visiblePlayers := s.getVisiblePlayers(player.ID, player.AOI)
	visibleObjects := s.getVisibleGridObjects(player.AOI)

	payload := AOIUpdatePayload{
		PlayerID:           player.ID,
		Player:             *player,
		VisiblePlayers:     visiblePlayers,
		VisibleGridObjects: visibleObjects,
	}

	msg, err := json.Marshal(map[string]interface{}{
		"type":    "aoi_update",
		"payload": payload,
	})
	if err != nil {
		log.Printf("Error marshaling AOI update: %v", err)
		return
	}

	select {
	case player.Client.send <- msg:
	default:
		s.handleClientDisconnect(player.Client)
	}
}

// handlePathRequest handles a pathfinding request from a client.
func (s *GameServer) handlePathRequest(playerID string, target PointI) {
	s.mu.Lock()
	defer s.mu.Unlock()

	player, ok := s.players[playerID]
	if !ok {
		log.Printf("Player %s not found for path request", playerID)
		return
	}

	start := PointI{X: int(player.Pos.X), Y: int(player.Pos.Y)}
	path, err := s.pathfinder.Astar(start, target, player.Dims)
	if err != nil {
		s.sendFeedbackToClient(player.Client, fmt.Sprintf("Failed to find path: %v", err))
		return
	}

	player.Path = path[1:] // The first point is the current location
	player.TargetPos = target
	player.Mode = WALKING
}

// sendFeedbackToClient sends a simple text message to a client.
func (s *GameServer) sendFeedbackToClient(client *Client, message string) {
	msg, err := json.Marshal(map[string]string{
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
			// FIX: Send each message separately instead of concatenating.
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

func main() {
	rand.Seed(time.Now().UnixNano())

	gameServer := NewGameServer(100, 100, 20.0)
	gameServer.pathfinder.GenerateObstacles(25, PointI{X: 5, Y: 5}, PointI{X: 95, Y: 95})

	gameServer.obstacles = gameServer.pathfinder.obstacles

	go gameServer.run()

	upgrader := websocket.Upgrader{
		CheckOrigin: func(r *http.Request) bool {
			return true
		},
	}

	http.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		conn, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Println("Upgrade error:", err)
			return
		}
		playerID := uuid.New().String()
		client := &Client{
			conn:       conn,
			playerID:   playerID,
			send:       make(chan []byte, 256),
			isPlayer:   true,
			lastAction: time.Now(),
		}
		gameServer.register <- client

		go client.writePump()
		readPump(conn, gameServer, client)
	})

	log.Println("Starting server on :8080")
	if err := http.ListenAndServe(":8080", nil); err != nil {
		log.Fatal("ListenAndServe: ", err)
	}
}

// readPump reads messages from the WebSocket connection.
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
			targetX, okX := payload["targetX"].(float64)
			targetY, okY := payload["targetY"].(float64)
			if !okX || !okY {
				log.Printf("Invalid target coordinates in path_request")
				continue
			}
			target := PointI{X: int(targetX), Y: int(targetY)}
			s.handlePathRequest(client.playerID, target)
		}
	}
}
