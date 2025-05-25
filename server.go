package main

import (
	"container/heap" // Standard library for priority queue (heap)
	"encoding/json"  // For JSON serialization and deserialization
	"fmt"            // For formatted I/O
	"log"            // For logging events
	"math"           // For mathematical operations like sqrt and abs
	"math/rand"      // For generating random numbers
	"net/http"       // For HTTP server and routing
	"sync"           // For concurrency control (mutexes)
	"time"           // For time handling and intervals

	// For warnings, similar to Python's warn
	"github.com/gorilla/websocket" // Gorilla WebSocket library for plain WebSockets
)

// --- Game Constants ---
const (
	SERVER_HOST          = "0.0.0.0"             // The server will listen on all network interfaces
	SERVER_PORT          = 5000                  // Server port
	WORLD_WIDTH          = 1600                  // Game world width
	WORLD_HEIGHT         = 1600                  // Game world height
	OBJECT_SIZE          = 50                    // Size of game objects (players, obstacles)
	MAZE_CELL_WORLD_SIZE = WORLD_WIDTH / 32      // Size of a maze cell for A* (1600 / 32 = 50)
	GAME_TICK_INTERVAL   = 50 * time.Millisecond // Game update interval (20 FPS)
)

// --- Colors (simplified for server representation) ---
// Clients will interpret these RGB values in their own rendering system.
type Color struct {
	R, G, B, A uint8
}

var (
	BLUE     = Color{0, 0, 255, 255}  // Blue color for players
	DARKGRAY = Color{80, 80, 80, 255} // Dark gray color for obstacles
)

// --- GameObject ---
// GameObject represents any entity in the game world.
type GameObject struct {
	ID         string  `json:"obj_id"`      // Unique object identifier
	X          float64 `json:"x"`           // X position in world coordinates
	Y          float64 `json:"y"`           // Y position in world coordinates
	Color      Color   `json:"color"`       // Object color
	IsObstacle bool    `json:"is_obstacle"` // Indicates if the object is an obstacle (non-movable)
	Speed      float64 `json:"speed"`       // Movement speed (pixels per second)

	// Fields related to pathfinding (path to follow)
	Path      []struct{ X, Y float64 } `json:"path"`       // The current path as a series of (X, Y) points
	PathIndex int                      `json:"path_index"` // Current index in the path
}

// UpdatePosition updates the object's position, moving it along its path.
func (obj *GameObject) UpdatePosition(deltaTime float64) {
	// If no path or path finished, reset the path.
	if len(obj.Path) == 0 || obj.PathIndex >= len(obj.Path) {
		obj.Path = []struct{ X, Y float64 }{}
		obj.PathIndex = 0
		return
	}

	targetPoint := obj.Path[obj.PathIndex] // Current target point in the path
	targetWorldX := targetPoint.X
	targetWorldY := targetPoint.Y

	// Calculate distance to the target point
	dx := targetWorldX - obj.X
	dy := targetWorldY - obj.Y
	distance := math.Sqrt(dx*dx + dy*dy)

	moveDistance := obj.Speed * deltaTime // Distance the object can move in this frame

	if distance < moveDistance {
		// If distance is less than moveDistance, it means the target point is reached or overshot.
		// Move exactly to the target point and advance to the next point in the path.
		obj.X = targetWorldX
		obj.Y = targetWorldY
		obj.PathIndex++
	} else {
		// If not yet at the target point, move towards it.
		directionX := dx / distance // Normalized X component of direction
		directionY := dy / distance // Normalized Y component of direction
		obj.X += directionX * moveDistance
		obj.Y += directionY * moveDistance
	}
}

// --- GameState ---
// GameState manages the overall game world state, including objects and the grid.
// It also handles the A* pathfinding logic.
type GameState struct {
	mu             sync.RWMutex           // Read-write mutex for safe concurrent access
	Objects        map[string]*GameObject // Map of game objects (ID -> GameObject)
	GridCellsX     int                    // Number of cells in the X axis of the conceptual grid
	GridCellsY     int                    // Number of cells in the Y axis of the conceptual grid
	Grid           [][]*GameObject        // Conceptual grid for object placement
	MazeCellsX     int                    // Number of cells in the X axis of the simplified maze (for A*)
	MazeCellsY     int                    // Number of cells in the Y axis of the simplified maze (for A*)
	SimplifiedMaze [][]int                // Simplified maze for A* (0: walkable, 1: obstacle)
	rng            *rand.Rand             // Random number generator for positions
}

// NewGameState initializes a new GameState.
func NewGameState() *GameState {
	gs := &GameState{
		Objects:    make(map[string]*GameObject),
		GridCellsX: WORLD_WIDTH / OBJECT_SIZE,
		GridCellsY: WORLD_HEIGHT / OBJECT_SIZE,
		MazeCellsX: WORLD_WIDTH / MAZE_CELL_WORLD_SIZE,
		MazeCellsY: WORLD_HEIGHT / MAZE_CELL_WORLD_SIZE,
		rng:        rand.New(rand.NewSource(time.Now().UnixNano())), // Initialize RNG with time-based seed
	}

	// Initialize the conceptual grid
	gs.Grid = make([][]*GameObject, gs.GridCellsY)
	for i := range gs.Grid {
		gs.Grid[i] = make([]*GameObject, gs.GridCellsX)
	}

	// Initialize the simplified maze for A*
	gs.SimplifiedMaze = make([][]int, gs.MazeCellsY)
	for i := range gs.SimplifiedMaze {
		gs.SimplifiedMaze[i] = make([]int, gs.MazeCellsX)
	}

	// buildSimplifiedMaze() is now called after initial obstacles are added in GameServer.addInitialObstacles()
	return gs
}

// worldToGridCoords converts world coordinates to grid cell indices.
func (gs *GameState) worldToGridCoords(worldX, worldY float64) (int, int) {
	gridX := int(worldX / OBJECT_SIZE)
	gridY := int(worldY / OBJECT_SIZE)
	return gridX, gridY
}

// worldToMazeCoords converts world coordinates to 32x32 maze coordinates.
func (gs *GameState) worldToMazeCoords(worldX, worldY float64) (int, int) {
	mazeX := int(worldX / MAZE_CELL_WORLD_SIZE)
	mazeY := int(worldY / MAZE_CELL_WORLD_SIZE)
	return mazeX, mazeY
}

// mazeToWorldCoords converts maze coordinates to world coordinates (center of the 50x50 cell).
func (gs *GameState) mazeToWorldCoords(mazeX, mazeY int) (float64, float64) {
	worldX := float64(mazeX*MAZE_CELL_WORLD_SIZE) + OBJECT_SIZE/2
	worldY := float64(mazeY*MAZE_CELL_WORLD_SIZE) + OBJECT_SIZE/2
	return worldX, worldY
}

// buildSimplifiedMaze populates the 32x32 simplified maze based on obstacle objects.
// A cell is an obstacle (1) if any part of it is occupied by an obstacle GameObject.
func (gs *GameState) buildSimplifiedMaze() {
	gs.mu.Lock() // Exclusive lock for writing to the maze
	defer gs.mu.Unlock()

	// Reset the maze
	for y := range gs.SimplifiedMaze {
		for x := range gs.SimplifiedMaze[y] {
			gs.SimplifiedMaze[y][x] = 0
		}
	}

	for _, obj := range gs.Objects {
		if obj.IsObstacle {
			// Determine the maze cells this obstacle covers
			mazeStartX, mazeStartY := gs.worldToMazeCoords(obj.X, obj.Y)
			mazeEndX, mazeEndY := gs.worldToMazeCoords(obj.X+OBJECT_SIZE-1, obj.Y+OBJECT_SIZE-1)

			// Ensure coordinates are within maze bounds before marking
			mazeStartX = int(math.Max(0, math.Min(float64(mazeStartX), float64(gs.MazeCellsX-1))))
			mazeStartY = int(math.Max(0, math.Min(float64(mazeStartY), float64(gs.MazeCellsY-1))))
			mazeEndX = int(math.Max(0, math.Min(float64(mazeEndX), float64(gs.MazeCellsX-1))))
			mazeEndY = int(math.Max(0, math.Min(float64(mazeEndY), float64(gs.MazeCellsY-1))))

			// Mark all covered maze cells as obstacles
			for y := mazeStartY; y <= mazeEndY; y++ {
				for x := mazeStartX; x <= mazeEndX; x++ {
					gs.SimplifiedMaze[y][x] = 1
				}
			}
		}
	}
	log.Println("Simplified maze rebuilt successfully.")
}

// AddObject adds an object to the game state.
// buildSimplifiedMaze() is NOT called here to avoid deadlocks during bulk additions.
// It should be explicitly called after a batch of additions or modifications.
func (gs *GameState) AddObject(obj *GameObject) {
	gs.mu.Lock()
	defer gs.mu.Unlock()

	if _, exists := gs.Objects[obj.ID]; exists {
		log.Printf("WARNING: Object with ID %s already exists.\n", obj.ID)
		return
	}
	gs.Objects[obj.ID] = obj

	gridX, gridY := gs.worldToGridCoords(obj.X, obj.Y)
	if gridX >= 0 && gridX < gs.GridCellsX && gridY >= 0 && gridY < gs.GridCellsY {
		gs.Grid[gridY][gridX] = obj
	}
	// REMOVED: gs.buildSimplifiedMaze() from here to prevent deadlock
	log.Printf("Object added: %s at (%.0f, %.0f)", obj.ID, obj.X, obj.Y)
}

// RemoveObject removes an object from the game state.
func (gs *GameState) RemoveObject(objID string) {
	gs.mu.Lock()
	defer gs.mu.Unlock()

	obj, exists := gs.Objects[objID]
	if !exists {
		log.Printf("WARNING: Attempted to remove non-existent object: %s\n", objID)
		return
	}
	delete(gs.Objects, objID)

	gridX, gridY := gs.worldToGridCoords(obj.X, obj.Y)
	if gridX >= 0 && gridX < gs.GridCellsX && gridY >= 0 && gridY < gs.GridCellsY && gs.Grid[gridY][gridX] == obj {
		gs.Grid[gridY][gridX] = nil
	}
	gs.buildSimplifiedMaze() // Rebuild maze after removal
	log.Printf("Object removed: %s\n", objID)
}

// UpdateObjectPosition updates an object's position in the game state.
func (gs *GameState) UpdateObjectPosition(objID string, newX, newY float64) {
	gs.mu.Lock()
	defer gs.mu.Unlock()

	obj, exists := gs.Objects[objID]
	if !exists {
		log.Printf("WARNING: Attempted to update position of non-existent object: %s\n", objID)
		return
	}

	// Clear old conceptual grid cell reference
	oldGridX, oldGridY := gs.worldToGridCoords(obj.X, obj.Y)
	if oldGridX >= 0 && oldGridX < gs.GridCellsX && oldGridY >= 0 && oldGridY < gs.GridCellsY && gs.Grid[oldGridY][oldGridX] == obj {
		gs.Grid[oldGridY][oldGridX] = nil
	}

	// Update object position
	obj.X = newX
	obj.Y = newY

	// Set new conceptual grid cell reference
	newGridX, newGridY := gs.worldToGridCoords(newX, newY)
	if newGridX >= 0 && newGridX < gs.GridCellsX && newGridY >= 0 && newGridY < gs.GridCellsY {
		gs.Grid[newGridY][newGridX] = obj
	}
	gs.buildSimplifiedMaze() // Rebuild maze if position changed (might affect obstacle map)
	log.Printf("Object %s updated to (%.0f, %.0f)\n", objID, newX, newY)
}

// GetRandomAvailablePosition finds a random available (non-obstacle) position for a new object.
// Returns world coordinates (top-left corner of the 50x50 cell).
func (gs *GameState) GetRandomAvailablePosition() (float64, float64, error) {
	gs.mu.RLock() // Read lock to read maze and grid
	defer gs.mu.RUnlock()

	availableCells := []struct{ X, Y float64 }{}
	for y := 0; y < gs.GridCellsY; y++ {
		for x := 0; x < gs.GridCellsX; x++ {
			if gs.Grid[y][x] == nil { // Cell is empty in the conceptual grid
				// Check if this cell is part of an obstacle in the simplified maze
				mazeX, mazeY := gs.worldToMazeCoords(float64(x*OBJECT_SIZE), float64(y*OBJECT_SIZE))
				// Ensure mazeX and mazeY are within bounds before accessing SimplifiedMaze
				if mazeX >= 0 && mazeX < gs.MazeCellsX && mazeY >= 0 && mazeY < gs.MazeCellsY && gs.SimplifiedMaze[mazeY][mazeX] == 0 { // It's a walkable maze cell
					availableCells = append(availableCells, struct{ X, Y float64 }{X: float64(x * OBJECT_SIZE), Y: float64(y * OBJECT_SIZE)})
				}
			}
		}
	}

	if len(availableCells) == 0 {
		return 0, 0, fmt.Errorf("no available positions found on the grid")
	}
	// Use the GameState's RNG instance
	randomIndex := gs.rng.Intn(len(availableCells))
	return availableCells[randomIndex].X, availableCells[randomIndex].Y, nil
}

// --- A* Pathfinding (Custom Implementation with Diagonal Movement) ---

// AStarNode represents a node in the A* algorithm.
type AStarNode struct {
	X, Y   int        // Node coordinates in the maze grid
	G      float64    // Cost from the start node to this node
	H      float64    // Heuristic cost from this node to the end node
	F      float64    // Total estimated cost (G + H)
	Parent *AStarNode // Parent node for path reconstruction
	Index  int        // Index in the priority queue (required for container/heap)
}

// PriorityQueue implements heap.Interface for AStarNode.
type PriorityQueue []*AStarNode

func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
	// We want Pop to give us the lowest F-score (highest priority)
	return pq[i].F < pq[j].F
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].Index = i
	pq[j].Index = j
}

func (pq *PriorityQueue) Push(x interface{}) {
	n := len(*pq)
	node := x.(*AStarNode)
	node.Index = n
	*pq = append(*pq, node)
}

func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	node := old[n-1]
	old[n-1] = nil  // Avoid memory leaks
	node.Index = -1 // For safety
	*pq = old[0 : n-1]
	return node
}

// heuristic calculates the squared Euclidean distance for A*.
// This heuristic is admissible and consistent for grid-based movement with varying costs.
func heuristic(a, b *AStarNode) float64 {
	dx := float64(a.X - b.X)
	dy := float64(a.Y - b.Y)
	return (dx * dx) + (dy * dy) // Squared Euclidean distance
}

// FindPath calculates an A* path between two world coordinates using a custom implementation.
// It supports diagonal movement.
func (gs *GameState) FindPath(startWorldX, startWorldY, endWorldX, endWorldY float64) ([]struct{ X, Y float64 }, error) {
	gs.mu.RLock() // Read lock to read the maze
	defer gs.mu.RUnlock()

	startMazeX, startMazeY := gs.worldToMazeCoords(startWorldX, startWorldY)
	endMazeX, endMazeY := gs.worldToMazeCoords(endWorldX, endWorldY)

	// Ensure start and end are within maze bounds and are walkable.
	if !(startMazeX >= 0 && startMazeX < gs.MazeCellsX &&
		startMazeY >= 0 && startMazeY < gs.MazeCellsY &&
		gs.SimplifiedMaze[startMazeY][startMazeX] == 0) {
		return nil, fmt.Errorf("start position (maze: %d,%d) is invalid or an obstacle", startMazeX, startMazeY)
	}
	if !(endMazeX >= 0 && endMazeX < gs.MazeCellsX &&
		endMazeY >= 0 && endMazeY < gs.MazeCellsY &&
		gs.SimplifiedMaze[endMazeY][endMazeX] == 0) {
		return nil, fmt.Errorf("end position (maze: %d,%d) is invalid or an obstacle", endMazeX, endMazeY)
	}

	log.Printf("Finding path from maze (%d,%d) to (%d,%d) with custom A*...\n", startMazeX, startMazeY, endMazeX, endMazeY)

	startNode := &AStarNode{X: startMazeX, Y: startMazeY}
	endNode := &AStarNode{X: endMazeX, Y: endMazeY}

	// Initialize start node's heuristic and total cost
	startNode.H = heuristic(startNode, endNode)
	startNode.F = startNode.G + startNode.H

	openSet := make(PriorityQueue, 0)
	heap.Push(&openSet, startNode)

	// gScore: cost from start to current node
	// cameFrom: for path reconstruction
	gScore := make(map[string]float64)
	gScore[fmt.Sprintf("%d,%d", startNode.X, startNode.Y)] = 0

	cameFrom := make(map[string]*AStarNode)

	// Directions for 8-way movement (including diagonals)
	// Costs: 1.0 for cardinal, math.Sqrt2 for diagonal
	neighborsDirs := []struct {
		dx, dy int
		cost   float64
	}{
		{0, 1, 1.0},          // Down
		{0, -1, 1.0},         // Up
		{1, 0, 1.0},          // Right
		{-1, 0, 1.0},         // Left
		{1, 1, math.Sqrt2},   // Down-Right
		{1, -1, math.Sqrt2},  // Up-Right
		{-1, 1, math.Sqrt2},  // Down-Left
		{-1, -1, math.Sqrt2}, // Up-Left
	}

	// Safety break: max iterations to prevent infinite loops on complex/unpathable mazes
	maxIterations := (gs.MazeCellsX * gs.MazeCellsY * 2) // A heuristic for max iterations
	outerIterations := 0

	for openSet.Len() > 0 {
		outerIterations++
		if outerIterations > maxIterations {
			warn("Pathfinding: giving up, too many iterations.")
			// Return the path found so far to the closest node, if any
			// For simplicity, returning nil here if we can't reach the end within limits.
			// A more advanced implementation might return a partial path to the closest reachable node.
			return nil, fmt.Errorf("pathfinding aborted: too many iterations")
		}

		current := heap.Pop(&openSet).(*AStarNode)
		currentKey := fmt.Sprintf("%d,%d", current.X, current.Y)

		// Found the goal
		if current.X == endMazeX && current.Y == endMazeY {
			// Reconstruct path
			pathMaze := []struct{ X, Y int }{}
			for current != nil {
				pathMaze = append([]struct{ X, Y int }{{X: current.X, Y: current.Y}}, pathMaze...)
				current = cameFrom[fmt.Sprintf("%d,%d", current.X, current.Y)]
			}

			// Convert maze path back to world coordinates (center of cells for smoother movement)
			pathWorld := make([]struct{ X, Y float64 }, len(pathMaze))
			for i, p := range pathMaze {
				pathWorld[i].X, pathWorld[i].Y = gs.mazeToWorldCoords(p.X, p.Y)
			}
			log.Printf("Path found with %d steps.\n", len(pathWorld))
			return pathWorld, nil
		}

		for _, dir := range neighborsDirs {
			neighborX, neighborY := current.X+dir.dx, current.Y+dir.dy
			neighborKey := fmt.Sprintf("%d,%d", neighborX, neighborY)

			// Check bounds and if it's an obstacle
			if neighborX < 0 || neighborX >= gs.MazeCellsX ||
				neighborY < 0 || neighborY >= gs.MazeCellsY ||
				gs.SimplifiedMaze[neighborY][neighborX] == 1 {
				continue // Invalid or obstacle
			}

			tentativeGScore := gScore[currentKey] + dir.cost

			// If this path to neighbor is better than any previous one.
			if val, ok := gScore[neighborKey]; !ok || tentativeGScore < val {
				newNode := &AStarNode{X: neighborX, Y: neighborY}
				cameFrom[neighborKey] = current
				gScore[neighborKey] = tentativeGScore
				newNode.G = tentativeGScore
				newNode.H = heuristic(newNode, endNode) // Use endNode directly
				newNode.F = newNode.G + newNode.H

				// Check if the neighbor is already in the open set; if so, update its priority
				foundInOpenSet := false
				for i, node := range openSet { // Iterate over the priority queue to find the node
					if node.X == newNode.X && node.Y == newNode.Y {
						if newNode.F < node.F {
							openSet[i] = newNode
							heap.Fix(&openSet, i) // Update priority in the heap
						}
						foundInOpenSet = true
						break
					}
				}
				if !foundInOpenSet {
					heap.Push(&openSet, newNode)
				}
			}
		}
	}

	warn("Pathfinding: Couldn't get a path to destination.")
	return nil, fmt.Errorf("no path found")
}

// warn is a simple helper function for warnings, similar to Python's warnings.warn
func warn(message string) {
	log.Printf("WARNING: %s\n", message)
}

// --- WebSocketClient ---
// WebSocketClient represents a single client connected via WebSocket.
type WebSocketClient struct {
	conn     *websocket.Conn // The raw WebSocket connection
	send     chan []byte     // Channel for outgoing messages to this client
	playerID string          // The player ID associated with this client
}

// readPump pumps messages from the websocket connection to the hub.
func (c *WebSocketClient) readPump(server *GameServer) {
	defer func() {
		server.unregisterClient(c)
		c.conn.Close()
	}()
	for {
		_, message, err := c.conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseNormalClosure) {
				log.Printf("Error reading message from client %s: %v", c.playerID, err)
			}
			break
		}
		// Handle incoming JSON messages from the client
		server.handleClientMessage(c, message)
	}
}

// writePump pumps messages from the hub to the websocket connection.
func (c *WebSocketClient) writePump() {
	defer func() {
		c.conn.Close()
	}()
	for message := range c.send {
		err := c.conn.WriteMessage(websocket.TextMessage, message)
		if err != nil {
			log.Printf("Error writing message to client %s: %v", c.playerID, err)
			return
		}
	}
}

// --- GameServer ---
// GameServer encapsulates the WebSocket server logic and game state management.
type GameServer struct {
	upgrader       websocket.Upgrader        // WebSocket upgrader for HTTP connections
	gameState      *GameState                // Current game state
	clients        map[*WebSocketClient]bool // Registered clients
	register       chan *WebSocketClient     // Channel for registering new clients
	unregister     chan *WebSocketClient     // Channel for unregistering clients
	playerCounter  int                       // Counter for assigning unique player IDs
	lastUpdateTime time.Time                 // Last time the game state was updated
	clientsMutex   sync.RWMutex              // Mutex to protect the clients map
}

// NewGameServer initializes a new GameServer.
func NewGameServer() *GameServer {
	server := &GameServer{
		upgrader: websocket.Upgrader{
			ReadBufferSize:  1024,
			WriteBufferSize: 1024,
			CheckOrigin: func(r *http.Request) bool {
				// Allow all origins for development. Restrict this in production.
				return true
			},
		},
		gameState:     NewGameState(),
		clients:       make(map[*WebSocketClient]bool),
		register:      make(chan *WebSocketClient),
		unregister:    make(chan *WebSocketClient),
		playerCounter: 0,
		clientsMutex:  sync.RWMutex{},
	}

	server.addInitialObstacles() // Add some initial obstacles to the world
	return server
}

// addInitialObstacles adds some static obstacles to the game world.
func (gs *GameServer) addInitialObstacles() {
	obstaclePositions := []struct{ X, Y float64 }{
		{200, 200}, {250, 200}, {300, 200},
		{200, 250}, {300, 250},
		{200, 300}, {250, 300}, {300, 300},
		{700, 700}, {750, 700}, {800, 700},
		{700, 750}, {800, 750},
		{700, 800}, {750, 800}, {800, 800},
		{100, 500}, {150, 500}, {200, 500},
		{500, 100}, {500, 150}, {500, 200},
		{1000, 1000}, {1050, 1000}, {1100, 1000},
		{1000, 1050}, {1100, 1050},
		{1000, 1100}, {1050, 1100}, {1100, 1100},
	}
	for i, pos := range obstaclePositions {
		obstacleID := fmt.Sprintf("obstacle_%d", i)
		obstacle := &GameObject{
			ID:         obstacleID,
			X:          pos.X,
			Y:          pos.Y,
			Color:      DARKGRAY,
			IsObstacle: true,
			Speed:      0, // Obstacles do not move
		}
		gs.gameState.AddObject(obstacle)
		log.Printf("Object added: %s at (%.0f, %.0f)", obstacleID, pos.X, pos.Y)
	}
	log.Printf("Added %d initial obstacles.\n", len(obstaclePositions))
	gs.gameState.buildSimplifiedMaze() // Build the maze once after adding all initial obstacles
}

// handleWebSocketConnection handles new WebSocket upgrade requests.
func (gs *GameServer) handleWebSocketConnection(w http.ResponseWriter, r *http.Request) {
	conn, err := gs.upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("WebSocket upgrade failed: %v\n", err)
		return
	}

	gs.clientsMutex.Lock()
	gs.playerCounter++
	playerID := fmt.Sprintf("player_%d", gs.playerCounter)
	gs.clientsMutex.Unlock()

	client := &WebSocketClient{conn: conn, send: make(chan []byte, 256), playerID: playerID}
	gs.register <- client // Register the new client

	// Assign player object and send initial state
	posX, posY, err := gs.gameState.GetRandomAvailablePosition()
	if err != nil {
		log.Printf("ERROR: Could not find available position for new player %s. Closing client connection. Error: %v\n", playerID, err)
		conn.Close()
		return
	}

	playerObj := &GameObject{
		ID:         playerID,
		X:          posX,
		Y:          posY,
		Color:      BLUE,
		IsObstacle: false,
		Speed:      200, // Player speed
	}
	gs.gameState.AddObject(playerObj) // Add the player object to the game state
	// Note: buildSimplifiedMaze() is NOT called here, as players are not obstacles.

	// Send player_assigned event
	playerAssignedMsg, _ := json.Marshal(map[string]interface{}{
		"type":      "player_assigned",
		"player_id": playerID,
		"x":         posX,
		"y":         posY,
	})
	client.send <- playerAssignedMsg
	log.Printf("Player %s assigned to new client.\n", playerID)

	// Start read and write "pumps" for the client
	go client.writePump()
	go client.readPump(gs) // Pass the server to readPump to handle messages
}

// registerClient adds a new client to the server's active client list.
func (gs *GameServer) registerClient(client *WebSocketClient) {
	gs.clientsMutex.Lock()
	gs.clients[client] = true
	gs.clientsMutex.Unlock()
	log.Printf("Client %s registered.\n", client.playerID)
	gs.sendFullGameState() // Send full game state to all clients after a new one connects
}

// unregisterClient removes a client from the server's active client list.
func (gs *GameServer) unregisterClient(client *WebSocketClient) {
	gs.clientsMutex.Lock()
	if _, ok := gs.clients[client]; ok {
		delete(gs.clients, client)
		close(client.send)
		gs.clientsMutex.Unlock() // Unlock before calling RemoveObject
		gs.gameState.RemoveObject(client.playerID)
		gs.sendFullGameState() // Send full game state to all clients after a client disconnects
	} else {
		gs.clientsMutex.Unlock()
	}
	log.Printf("Client %s unregistered.\n", client.playerID)
}

// clientMessage represents the generic structure of messages coming from the client.
type clientMessage struct {
	Type string `json:"type"` // "event_name"
	// The Data field will be deserialized based on Type
	Data json.RawMessage `json:"data"`
}

// clientMoveRequestData represents the specific data for a "client_move_request" message.
type clientMoveRequestData struct {
	TargetX float64 `json:"target_x"`
	TargetY float64 `json:"target_y"`
}

// handleClientMessage processes incoming JSON messages from a client.
func (gs *GameServer) handleClientMessage(client *WebSocketClient, message []byte) {
	var msg clientMessage
	if err := json.Unmarshal(message, &msg); err != nil {
		log.Printf("ERROR: Failed to unmarshal incoming message from client %s: %v\n", client.playerID, err)
		return
	}

	switch msg.Type {
	case "client_move_request":
		var moveData clientMoveRequestData
		if err := json.Unmarshal(msg.Data, &moveData); err != nil {
			log.Printf("ERROR: Failed to unmarshal move request data from client %s: %v\n", client.playerID, err)
			return
		}
		gs.processClientMoveRequest(client, moveData)
	default:
		log.Printf("WARNING: Unknown message type '%s' from client %s\n", msg.Type, client.playerID)
	}
}

// processClientMoveRequest handles the logic for a client's move request.
func (gs *GameServer) processClientMoveRequest(client *WebSocketClient, data clientMoveRequestData) {
	gs.gameState.mu.RLock() // Read lock to get player object
	playerObj, objExists := gs.gameState.Objects[client.playerID]
	gs.gameState.mu.RUnlock()

	if !objExists {
		log.Printf("ERROR: Player object %s not found for move request.\n", client.playerID)
		// Optionally, send an error message back to the client
		return
	}

	// Find the path using A*
	path, err := gs.gameState.FindPath(playerObj.X, playerObj.Y, data.TargetX, data.TargetY)
	if err != nil {
		log.Printf("WARNING: No path found for player %s from (%.0f,%.0f) to (%.0f,%.0f). Error: %v\n", client.playerID, playerObj.X, playerObj.Y, data.TargetX, data.TargetY, err)
		// Send an error message back to the client
		errMsg, _ := json.Marshal(map[string]string{"type": "message", "text": "No path found to that location!"})
		client.send <- errMsg
		return
	}

	gs.gameState.mu.Lock() // Write lock to update player object's path
	playerObj.Path = path
	playerObj.PathIndex = 0
	gs.gameState.mu.Unlock()
	log.Printf("Calculated path for %s: %d steps.\n", client.playerID, len(path))

	// Send player_path_update event to the specific client
	pathUpdateMsg, _ := json.Marshal(map[string]interface{}{
		"type":      "player_path_update",
		"player_id": client.playerID,
		"path":      path,
	})
	client.send <- pathUpdateMsg
}

// broadcastGameStateLoop periodically broadcasts the full game state to all connected clients.
func (gs *GameServer) broadcastGameStateLoop() {
	gs.lastUpdateTime = time.Now()
	ticker := time.NewTicker(GAME_TICK_INTERVAL) // Create a ticker for the update interval
	defer ticker.Stop()                          // Ensure the ticker stops when the function exits

	for range ticker.C { // Loop that runs on each game "tick"
		deltaTime := time.Since(gs.lastUpdateTime).Seconds() // Calculate elapsed time since last update
		gs.lastUpdateTime = time.Now()

		gs.gameState.mu.Lock() // Lock game state to update objects
		for _, obj := range gs.gameState.Objects {
			if !obj.IsObstacle { // Only update non-obstacle objects (players)
				obj.UpdatePosition(deltaTime)
				// If the player reached the end of their path, clear it
				if len(obj.Path) > 0 && obj.PathIndex >= len(obj.Path) {
					obj.Path = []struct{ X, Y float64 }{}
					obj.PathIndex = 0
					log.Printf("Player %s reached destination.\n", obj.ID)
				}
			}
		}
		gs.gameState.mu.Unlock() // Unlock game state

		gs.sendFullGameState() // Send the full game state to all clients
	}
}

// sendFullGameState sends the current game state to all connected clients.
func (gs *GameServer) sendFullGameState() { // Moved to GameServer
	gs.gameState.mu.RLock() // Read lock to read game state
	defer gs.gameState.mu.RUnlock()

	// Create a serializable representation of game objects
	serializableObjects := make(map[string]GameObject)
	for id, obj := range gs.gameState.Objects {
		serializableObjects[id] = *obj // Dereference to copy the struct
	}

	// Prepare message for broadcast
	broadcastMsg, err := json.Marshal(map[string]interface{}{
		"type":    "game_state_update",
		"objects": serializableObjects,
	})
	if err != nil {
		log.Printf("ERROR: Failed to marshal game state for broadcast: %v\n", err)
		return
	}

	gs.clientsMutex.RLock() // Read lock to iterate over clients
	for client := range gs.clients {
		select {
		case client.send <- broadcastMsg:
			// Message sent successfully
		default:
			// Client's send buffer is full, or client is unresponsive
			log.Printf("WARNING: Client %s send buffer full, unregistering.\n", client.playerID)
			gs.unregisterClient(client) // Unregister the problematic client
		}
	}
	gs.clientsMutex.RUnlock()
}

// Run starts the WebSocket server and the main game loop.
func (gs *GameServer) Run() {
	// Start the client management goroutine
	go func() {
		for {
			select {
			case client := <-gs.register:
				gs.registerClient(client)
			case client := <-gs.unregister:
				gs.unregisterClient(client)
			}
		}
	}()

	go gs.broadcastGameStateLoop() // Start the game state broadcast in a goroutine

	http.HandleFunc("/ws", gs.handleWebSocketConnection) // WebSocket endpoint
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Write([]byte("Go Plain WebSocket Game Server Running. Connect to /ws for game."))
	})

	log.Printf("Starting Plain WebSocket server on http://%s:%d\n", SERVER_HOST, SERVER_PORT)
	// Start the HTTP server and block main execution until a fatal error occurs
	log.Fatal(http.ListenAndServe(fmt.Sprintf("%s:%d", SERVER_HOST, SERVER_PORT), nil))
}

func main() {
	server := NewGameServer() // Create a new server instance
	server.Run()              // Start the server
}
