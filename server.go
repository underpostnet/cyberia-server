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

	"github.com/gorilla/websocket" // Gorilla WebSocket library for plain WebSockets
)

// --- Game Constants ---
const (
	SERVER_HOST          = "0.0.0.0"             // Server listens on all network interfaces
	SERVER_PORT          = 5000                  // Default server port
	WORLD_WIDTH          = 1600                  // Game world width in pixels
	WORLD_HEIGHT         = 1600                  // Game world height in pixels
	OBJECT_SIZE          = 50                    // Size of game objects (players, obstacles) in pixels
	MAZE_CELL_WORLD_SIZE = WORLD_WIDTH / 32      // Size of a maze cell for A* pathfinding (50x50 pixels)
	GAME_TICK_INTERVAL   = 50 * time.Millisecond // Game state update interval (20 frames per second)

	// WebSocket heartbeat settings to detect disconnected clients
	PING_INTERVAL = 10 * time.Second // Frequency of sending ping messages
	PONG_WAIT     = 60 * time.Second // Time to wait for a pong response before considering client disconnected
)

// --- Colors (simplified RGB representation for server) ---
// Clients interpret these values for rendering.
type Color struct {
	R, G, B, A uint8
}

var (
	BLUE     = Color{0, 0, 255, 255}  // Color for player objects
	DARKGRAY = Color{80, 80, 80, 255} // Color for obstacle objects
)

// --- GameObject (Represents any entity in the game world) ---
type GameObject struct {
	ID         string  `json:"obj_id"`      // Unique identifier for the object
	X          float64 `json:"x"`           // X-coordinate in world space
	Y          float64 `json:"y"`           // Y-coordinate in world space
	Color      Color   `json:"color"`       // Visual color of the object
	IsObstacle bool    `json:"is_obstacle"` // True if the object blocks movement (e.g., a wall)
	Speed      float64 `json:"speed"`       // Movement speed in pixels per second

	// Pathfinding related fields
	Path      []struct{ X, Y float64 } `json:"path"`       // Current path as a slice of world coordinates
	PathIndex int                      `json:"path_index"` // Current index in the path being followed
}

// UpdatePosition moves the object along its current path based on elapsed time.
func (obj *GameObject) UpdatePosition(deltaTime float64) {
	// If path is empty or finished, reset path and stop movement.
	if len(obj.Path) == 0 || obj.PathIndex >= len(obj.Path) {
		obj.Path = []struct{ X, Y float64 }{}
		obj.PathIndex = 0
		return
	}

	targetPoint := obj.Path[obj.PathIndex] // The current target point in the path
	dx := targetPoint.X - obj.X
	dy := targetPoint.Y - obj.Y
	distance := math.Sqrt(dx*dx + dy*dy) // Distance to the target point

	moveDistance := obj.Speed * deltaTime // Distance object can cover in this frame

	if distance < moveDistance {
		// If close enough, snap to the target point and advance to the next path segment.
		obj.X = targetPoint.X
		obj.Y = targetPoint.Y
		obj.PathIndex++
	} else {
		// Move towards the target point.
		directionX := dx / distance // Normalized X direction
		directionY := dy / distance // Normalized Y direction
		obj.X += directionX * moveDistance
		obj.Y += directionY * moveDistance
	}
}

// --- GameState (Manages the overall game world state) ---
type GameState struct {
	mu             sync.RWMutex           // Mutex for concurrent access to game objects and maze
	Objects        map[string]*GameObject // All game objects, indexed by ID
	GridCellsX     int                    // Number of conceptual grid cells horizontally
	GridCellsY     int                    // Number of conceptual grid cells vertically
	Grid           [][]*GameObject        // Conceptual grid for quick object lookup by position
	MazeCellsX     int                    // Number of A* maze cells horizontally
	MazeCellsY     int                    // Number of A* maze cells vertically
	SimplifiedMaze [][]int                // Simplified maze (0: walkable, 1: obstacle) for A*
	rng            *rand.Rand             // Random number generator for positions
}

// NewGameState initializes a new GameState with default dimensions and an empty world.
func NewGameState() *GameState {
	gs := &GameState{
		Objects:    make(map[string]*GameObject),
		GridCellsX: WORLD_WIDTH / OBJECT_SIZE,
		GridCellsY: WORLD_HEIGHT / OBJECT_SIZE,
		MazeCellsX: WORLD_WIDTH / MAZE_CELL_WORLD_SIZE,
		MazeCellsY: WORLD_HEIGHT / MAZE_CELL_WORLD_SIZE,
		rng:        rand.New(rand.NewSource(time.Now().UnixNano())), // Seed RNG with current time
	}

	// Initialize the conceptual grid for object placement.
	gs.Grid = make([][]*GameObject, gs.GridCellsY)
	for i := range gs.Grid {
		gs.Grid[i] = make([]*GameObject, gs.GridCellsX)
	}

	// Initialize the simplified maze for A* pathfinding.
	gs.SimplifiedMaze = make([][]int, gs.MazeCellsY)
	for i := range gs.SimplifiedMaze {
		gs.SimplifiedMaze[i] = make([]int, gs.MazeCellsX)
	}

	// The maze is built after initial obstacles are added in GameServer.addInitialObstacles().
	return gs
}

// worldToGridCoords converts world coordinates to conceptual grid indices.
func (gs *GameState) worldToGridCoords(worldX, worldY float64) (int, int) {
	return int(worldX / OBJECT_SIZE), int(worldY / OBJECT_SIZE)
}

// worldToMazeCoords converts world coordinates to A* maze indices.
func (gs *GameState) worldToMazeCoords(worldX, worldY float64) (int, int) {
	return int(worldX / MAZE_CELL_WORLD_SIZE), int(worldY / MAZE_CELL_WORLD_SIZE)
}

// mazeToWorldCoords converts A* maze indices to world coordinates (center of the cell).
func (gs *GameState) mazeToWorldCoords(mazeX, mazeY int) (float64, float64) {
	return float64(mazeX*MAZE_CELL_WORLD_SIZE) + OBJECT_SIZE/2,
		float64(mazeY*MAZE_CELL_WORLD_SIZE) + OBJECT_SIZE/2
}

// buildSimplifiedMaze populates the A* maze based on current obstacle GameObjects.
// This should only be called when static obstacles are added, removed, or modified.
func (gs *GameState) buildSimplifiedMaze() {
	gs.mu.Lock() // Acquire write lock for maze modification
	defer gs.mu.Unlock()

	// Reset all maze cells to walkable (0).
	for y := range gs.SimplifiedMaze {
		for x := range gs.SimplifiedMaze[y] {
			gs.SimplifiedMaze[y][x] = 0
		}
	}

	// Mark cells occupied by obstacles as unwalkable (1).
	for _, obj := range gs.Objects {
		if obj.IsObstacle {
			// Determine the range of maze cells this obstacle covers.
			mazeStartX, mazeStartY := gs.worldToMazeCoords(obj.X, obj.Y)
			mazeEndX, mazeEndY := gs.worldToMazeCoords(obj.X+OBJECT_SIZE-1, obj.Y+OBJECT_SIZE-1)

			// Clamp coordinates to ensure they are within maze bounds.
			mazeStartX = int(math.Max(0, math.Min(float64(mazeStartX), float64(gs.MazeCellsX-1))))
			mazeStartY = int(math.Max(0, math.Min(float64(mazeStartY), float64(gs.MazeCellsY-1))))
			mazeEndX = int(math.Max(0, math.Min(float64(mazeEndX), float64(gs.MazeCellsX-1))))
			mazeEndY = int(math.Max(0, math.Min(float64(mazeEndY), float64(gs.MazeCellsY-1))))

			// Mark all covered cells as obstacles.
			for y := mazeStartY; y <= mazeEndY; y++ {
				for x := mazeStartX; x <= mazeEndX; x++ {
					gs.SimplifiedMaze[y][x] = 1
				}
			}
		}
	}
	log.Println("Simplified maze rebuilt successfully.")
}

// AddObject adds a GameObject to the game state.
func (gs *GameState) AddObject(obj *GameObject) {
	gs.mu.Lock() // Acquire write lock for Objects map and Grid
	defer gs.mu.Unlock()

	if _, exists := gs.Objects[obj.ID]; exists {
		log.Printf("WARNING: Object with ID %s already exists, cannot add.\n", obj.ID)
		return
	}
	gs.Objects[obj.ID] = obj

	// Update conceptual grid reference for the new object.
	gridX, gridY := gs.worldToGridCoords(obj.X, obj.Y)
	if gridX >= 0 && gridX < gs.GridCellsX && gridY >= 0 && gridY < gs.GridCellsY {
		gs.Grid[gridY][gridX] = obj
	}
	// Note: buildSimplifiedMaze() is NOT called here to prevent deadlocks and
	// because player objects (IsObstacle=false) do not affect the maze.
	log.Printf("Object added: %s at (%.0f, %.0f)", obj.ID, obj.X, obj.Y)
}

// RemoveObject removes a GameObject from the game state by its ID.
func (gs *GameState) RemoveObject(objID string) {
	gs.mu.Lock() // Acquire write lock for Objects map and Grid
	defer gs.mu.Unlock()

	obj, exists := gs.Objects[objID]
	if !exists {
		log.Printf("WARNING: Attempted to remove non-existent object: %s\n", objID)
		return
	}
	delete(gs.Objects, objID)

	// Clear old conceptual grid reference.
	gridX, gridY := gs.worldToGridCoords(obj.X, obj.Y)
	if gridX >= 0 && gridX < gs.GridCellsX && gridY >= 0 && gridY < gs.GridCellsY && gs.Grid[gridY][gridX] == obj {
		gs.Grid[gridY][gridX] = nil
	}
	// Note: buildSimplifiedMaze() is NOT called here for the same reasons as AddObject.
	log.Printf("Object removed: %s\n", objID)
}

// UpdateObjectPosition updates a GameObject's position in the game state.
func (gs *GameState) UpdateObjectPosition(objID string, newX, newY float64) {
	gs.mu.Lock() // Acquire write lock for Objects map and Grid
	defer gs.mu.Unlock()

	obj, exists := gs.Objects[objID]
	if !exists {
		log.Printf("WARNING: Attempted to update position of non-existent object: %s\n", objID)
		return
	}

	// Clear old conceptual grid cell reference.
	oldGridX, oldGridY := gs.worldToGridCoords(obj.X, obj.Y)
	if oldGridX >= 0 && oldGridX < gs.GridCellsX && oldGridY >= 0 && oldGridY < gs.GridCellsY && gs.Grid[oldGridY][oldGridX] == obj {
		gs.Grid[oldGridY][oldGridX] = nil
	}

	// Update object's position.
	obj.X = newX
	obj.Y = newY

	// Set new conceptual grid cell reference.
	newGridX, newGridY := gs.worldToGridCoords(newX, newY)
	if newGridX >= 0 && newGridX < gs.GridCellsX && newGridY >= 0 && newGridY < gs.GridCellsY {
		gs.Grid[newGridY][newGridX] = obj
	}
	// Note: buildSimplifiedMaze() is NOT called here for the same reasons as AddObject.
	log.Printf("Object %s updated to (%.0f, %.0f)\n", objID, newX, newY)
}

// GetRandomAvailablePosition finds a random empty and walkable position for a new object.
func (gs *GameState) GetRandomAvailablePosition() (float64, float64, error) {
	gs.mu.RLock() // Acquire read lock for maze and grid
	defer gs.mu.RUnlock()

	availableCells := []struct{ X, Y float64 }{}
	for y := 0; y < gs.GridCellsY; y++ {
		for x := 0; x < gs.GridCellsX; x++ {
			// Check if conceptual grid cell is empty AND corresponding maze cell is walkable.
			if gs.Grid[y][x] == nil {
				mazeX, mazeY := gs.worldToMazeCoords(float64(x*OBJECT_SIZE), float64(y*OBJECT_SIZE))
				// Ensure maze coordinates are within bounds before checking SimplifiedMaze.
				if mazeX >= 0 && mazeX < gs.MazeCellsX && mazeY >= 0 && mazeY < gs.MazeCellsY && gs.SimplifiedMaze[mazeY][mazeX] == 0 {
					availableCells = append(availableCells, struct{ X, Y float64 }{X: float64(x * OBJECT_SIZE), Y: float64(y * OBJECT_SIZE)})
				}
			}
		}
	}

	if len(availableCells) == 0 {
		return 0, 0, fmt.Errorf("no available positions found on the grid")
	}
	// Select a random available cell using the GameState's RNG.
	randomIndex := gs.rng.Intn(len(availableCells))
	return availableCells[randomIndex].X, availableCells[randomIndex].Y, nil
}

// --- A* Pathfinding (Custom Implementation) ---

// AStarNode represents a node in the A* algorithm's search space.
type AStarNode struct {
	X, Y   int        // Coordinates in the maze grid
	G      float64    // Cost from start node to this node
	H      float64    // Heuristic cost from this node to the end node
	F      float64    // Total estimated cost (G + H)
	Parent *AStarNode // Reference to the parent node for path reconstruction
	Index  int        // Index in the priority queue (required by container/heap)
}

// PriorityQueue implements heap.Interface for AStarNode to manage the open set.
type PriorityQueue []*AStarNode

func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
	// Lower F-score means higher priority (min-heap).
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
	node.Index = -1 // Mark as removed
	*pq = old[0 : n-1]
	return node
}

// heuristic calculates the squared Euclidean distance between two AStarNodes.
// This is an admissible and consistent heuristic for grid-based movement.
func heuristic(a, b *AStarNode) float64 {
	dx := float64(a.X - b.X)
	dy := float64(a.Y - b.Y)
	return (dx * dx) + (dy * dy)
}

// FindPath calculates an A* path between two world coordinates.
// It converts world coordinates to maze coordinates, performs A* search,
// and converts the resulting path back to world coordinates.
func (gs *GameState) FindPath(startWorldX, startWorldY, endWorldX, endWorldY float64) ([]struct{ X, Y float64 }, error) {
	gs.mu.RLock() // Acquire read lock for maze access during pathfinding
	defer gs.mu.RUnlock()

	startMazeX, startMazeY := gs.worldToMazeCoords(startWorldX, startWorldY)
	endMazeX, endMazeY := gs.worldToMazeCoords(endWorldX, endWorldY)

	// Validate start and end positions: must be within maze bounds and walkable.
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

	log.Printf("Pathfinding: from maze (%d,%d) to (%d,%d)...\n", startMazeX, startMazeY, endMazeX, endMazeY)

	startNode := &AStarNode{X: startMazeX, Y: startMazeY, G: 0}
	endNode := &AStarNode{X: endMazeX, Y: endMazeY}

	startNode.H = heuristic(startNode, endNode)
	startNode.F = startNode.G + startNode.H

	openSet := make(PriorityQueue, 0)
	heap.Push(&openSet, startNode)

	// gScore stores the cost from start to a node.
	gScore := make(map[string]float64)
	gScore[fmt.Sprintf("%d,%d", startNode.X, startNode.Y)] = 0

	// cameFrom stores the path reconstruction chain.
	cameFrom := make(map[string]*AStarNode)

	// Directions for 8-way movement (cardinal and diagonal).
	// Costs: 1.0 for cardinal, sqrt(2) for diagonal.
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

	// Safety break to prevent infinite loops in unpathable or complex mazes.
	maxIterations := (gs.MazeCellsX * gs.MazeCellsY * 2)
	iterations := 0

	for openSet.Len() > 0 {
		iterations++
		if iterations > maxIterations {
			log.Printf("WARNING: Pathfinding aborted due to too many iterations. No path found within limits.")
			return nil, fmt.Errorf("pathfinding aborted: too many iterations")
		}

		current := heap.Pop(&openSet).(*AStarNode)
		currentKey := fmt.Sprintf("%d,%d", current.X, current.Y)

		// If the goal is reached, reconstruct and return the path.
		if current.X == endMazeX && current.Y == endMazeY {
			pathMaze := []struct{ X, Y int }{}
			for current != nil {
				pathMaze = append([]struct{ X, Y int }{{X: current.X, Y: current.Y}}, pathMaze...)
				current = cameFrom[fmt.Sprintf("%d,%d", current.X, current.Y)]
			}

			// Convert maze coordinates path to world coordinates.
			pathWorld := make([]struct{ X, Y float64 }, len(pathMaze))
			for i, p := range pathMaze {
				pathWorld[i].X, pathWorld[i].Y = gs.mazeToWorldCoords(p.X, p.Y)
			}
			log.Printf("Pathfinding: Path found with %d steps.\n", len(pathWorld))
			return pathWorld, nil
		}

		// Explore neighbors of the current node.
		for _, dir := range neighborsDirs {
			neighborX, neighborY := current.X+dir.dx, current.Y+dir.dy
			neighborKey := fmt.Sprintf("%d,%d", neighborX, neighborY)

			// Check if neighbor is within bounds and not an obstacle.
			if neighborX < 0 || neighborX >= gs.MazeCellsX ||
				neighborY < 0 || neighborY >= gs.MazeCellsY ||
				gs.SimplifiedMaze[neighborY][neighborX] == 1 {
				continue // Skip invalid or obstructed neighbors
			}

			tentativeGScore := gScore[currentKey] + dir.cost

			// If this path to neighbor is better than any previous one, update it.
			if val, ok := gScore[neighborKey]; !ok || tentativeGScore < val {
				newNode := &AStarNode{X: neighborX, Y: neighborY}
				cameFrom[neighborKey] = current
				gScore[neighborKey] = tentativeGScore
				newNode.G = tentativeGScore
				newNode.H = heuristic(newNode, endNode)
				newNode.F = newNode.G + newNode.H

				// Add or update neighbor in the open set.
				foundInOpenSet := false
				for i, node := range openSet {
					if node.X == newNode.X && node.Y == newNode.Y {
						if newNode.F < node.F { // If new path is better, update priority
							openSet[i] = newNode
							heap.Fix(&openSet, i) // Re-heapify after update
						}
						foundInOpenSet = true
						break
					}
				}
				if !foundInOpenSet {
					heap.Push(&openSet, newNode) // Add new node to open set
				}
			}
		}
	}

	log.Printf("Pathfinding: No path found to destination.")
	return nil, fmt.Errorf("no path found")
}

// --- WebSocketClient (Represents a single connected client) ---
type WebSocketClient struct {
	conn     *websocket.Conn // The raw WebSocket connection
	send     chan []byte     // Channel for outgoing messages to this client
	playerID string          // The unique player ID associated with this client
	done     chan struct{}   // Signal channel for goroutine termination
}

// readPump continuously reads messages from the WebSocket connection.
// It handles disconnection detection and signals the writePump to terminate.
func (c *WebSocketClient) readPump(server *GameServer) {
	// Ensure connection is closed and client unregistered when this goroutine exits.
	defer func() {
		server.unregisterClient(c) // Unregister the client from the server's active list
		close(c.done)              // Signal the writePump to terminate
		c.conn.Close()             // Close the underlying WebSocket connection
	}()

	// Set a read deadline and a pong handler for heartbeat.
	c.conn.SetReadDeadline(time.Now().Add(PONG_WAIT))
	c.conn.SetPongHandler(func(string) error {
		c.conn.SetReadDeadline(time.Now().Add(PONG_WAIT)) // Extend deadline on pong
		return nil
	})

	for {
		_, message, err := c.conn.ReadMessage() // Read messages
		if err != nil {
			// Log unexpected close errors, indicating a client disconnection.
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseNormalClosure) {
				log.Printf("Client %s: Unexpected WebSocket close error: %v", c.playerID, err)
			} else {
				log.Printf("Client %s: WebSocket read error (non-unexpected close): %v", c.playerID, err)
			}
			break // Exit loop on any read error, triggering defer
		}
		// Pass the received message to the server's message handler for processing.
		server.handleClientMessage(c, message)
	}
}

// writePump continuously sends messages from the 'send' channel to the WebSocket connection.
// It also sends periodic pings for heartbeat and terminates gracefully on signal.
func (c *WebSocketClient) writePump() {
	ticker := time.NewTicker(PING_INTERVAL) // Ticker for sending periodic pings
	defer func() {
		ticker.Stop()  // Stop the ticker on exit
		c.conn.Close() // Ensure connection is closed on exit
	}()

	for {
		select {
		case message, ok := <-c.send:
			// Attempt to send a message from the 'send' channel.
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second)) // Set write deadline
			if !ok {
				// The 'send' channel was closed, indicating client unregistration.
				// Send a normal close message to the client before exiting.
				c.conn.WriteMessage(websocket.CloseMessage, []byte{})
				return // Terminate goroutine
			}

			// Get a new writer for the WebSocket message.
			w, err := c.conn.NextWriter(websocket.TextMessage)
			if err != nil {
				log.Printf("Client %s: Error getting WebSocket writer: %v", c.playerID, err)
				return // Terminate goroutine on writer error
			}
			w.Write(message) // Write the message

			// If there are more messages in the buffer, write them immediately.
			n := len(c.send)
			for i := 0; i < n; i++ {
				w.Write(<-c.send)
			}

			// Close the writer to flush the message.
			if err := w.Close(); err != nil {
				log.Printf("Client %s: Error closing WebSocket writer: %v", c.playerID, err)
				return // Terminate goroutine on close error
			}
		case <-ticker.C:
			// Send a ping message on ticker tick.
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second)) // Set write deadline for ping
			if err := c.conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				log.Printf("Client %s: Error sending ping: %v", c.playerID, err)
				return // Terminate goroutine on ping error
			}
		case <-c.done:
			// Received termination signal from readPump.
			log.Printf("Client %s: writePump received done signal, terminating.", c.playerID)
			// Attempt to send a normal close message before exiting.
			err := c.conn.WriteMessage(websocket.CloseMessage, websocket.FormatCloseMessage(websocket.CloseNormalClosure, ""))
			if err != nil {
				log.Printf("Client %s: Error sending final close message: %v", c.playerID, err)
			}
			return // Terminate goroutine
		}
	}
}

// --- GameServer (Manages WebSocket connections and game state) ---
type GameServer struct {
	upgrader         websocket.Upgrader          // WebSocket upgrader for HTTP requests
	gameState        *GameState                  // The central game state
	clients          map[*WebSocketClient]bool   // Set of active WebSocketClient connections
	playerIDToClient map[string]*WebSocketClient // Map from player ID to WebSocketClient for quick lookup
	register         chan *WebSocketClient       // Channel for new client registrations
	unregister       chan *WebSocketClient       // Channel for client unregistrations
	playerCounter    int                         // Counter for assigning unique player IDs
	lastUpdateTime   time.Time                   // Timestamp of the last game state update
	clientsMutex     sync.RWMutex                // Mutex protecting 'clients' and 'playerIDToClient' maps
}

// NewGameServer initializes a new GameServer instance.
func NewGameServer() *GameServer {
	server := &GameServer{
		upgrader: websocket.Upgrader{
			ReadBufferSize:  1024,
			WriteBufferSize: 1024,
			CheckOrigin: func(r *http.Request) bool {
				// Allow all origins for development. RESTRICT THIS IN PRODUCTION!
				return true
			},
		},
		gameState:        NewGameState(),
		clients:          make(map[*WebSocketClient]bool),
		playerIDToClient: make(map[string]*WebSocketClient), // Initialize the new map
		register:         make(chan *WebSocketClient),
		unregister:       make(chan *WebSocketClient),
		playerCounter:    0,
		clientsMutex:     sync.RWMutex{},
	}

	server.addInitialObstacles() // Populate the world with static obstacles
	return server
}

// addInitialObstacles places predefined static obstacles in the game world.
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
		log.Printf("Added obstacle: %s at (%.0f, %.0f)", obstacleID, pos.X, pos.Y)
	}
	log.Printf("Added %d initial obstacles.", len(obstaclePositions))
	gs.gameState.buildSimplifiedMaze() // Build the A* maze once after all static obstacles are added.
}

// handleWebSocketConnection upgrades HTTP requests to WebSocket connections.
func (gs *GameServer) handleWebSocketConnection(w http.ResponseWriter, r *http.Request) {
	conn, err := gs.upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("WebSocket upgrade failed: %v", err)
		return
	}

	// Assign a unique player ID and create a new WebSocketClient.
	gs.clientsMutex.Lock()
	gs.playerCounter++
	playerID := fmt.Sprintf("player_%d", gs.playerCounter)
	client := &WebSocketClient{conn: conn, send: make(chan []byte, 256), playerID: playerID, done: make(chan struct{})}
	gs.clients[client] = true              // Add to set of active clients
	gs.playerIDToClient[playerID] = client // Map player ID to client
	gs.clientsMutex.Unlock()

	gs.register <- client // Signal the GameServer's main loop to register this client

	// Find a random available position for the new player.
	posX, posY, err := gs.gameState.GetRandomAvailablePosition()
	if err != nil {
		log.Printf("ERROR: No available position for new player %s. Closing connection. Error: %v", playerID, err)
		conn.Close() // Close connection if no position is found
		return
	}

	// Create and add the player's GameObject to the game state.
	playerObj := &GameObject{
		ID:         playerID,
		X:          posX,
		Y:          posY,
		Color:      BLUE,
		IsObstacle: false,
		Speed:      200, // Player movement speed
	}
	gs.gameState.AddObject(playerObj)

	// Verify the player object was successfully added to the game state.
	gs.gameState.mu.RLock()
	_, objAdded := gs.gameState.Objects[playerID]
	gs.gameState.mu.RUnlock()
	if !objAdded {
		log.Printf("CRITICAL ERROR: Player object %s was not added to game state. Disconnecting client %s.", playerID, conn.RemoteAddr().String())
		conn.Close()
		return
	}

	// Send the 'player_assigned' message directly and synchronously to the client.
	// This ensures the client receives its ID before any other async updates.
	playerAssignedMsg, _ := json.Marshal(map[string]interface{}{
		"type":      "player_assigned",
		"player_id": playerID,
		"x":         posX,
		"y":         posY,
	})
	err = conn.WriteMessage(websocket.TextMessage, playerAssignedMsg)
	if err != nil {
		log.Printf("ERROR: Failed to send player_assigned message to client %s: %v. Disconnecting.", playerID, err)
		conn.Close()
		return
	}
	log.Printf("Player %s assigned to client %s.", playerID, conn.RemoteAddr().String())

	// Start read and write goroutines for the client.
	go client.writePump()
	go client.readPump(gs) // Pass GameServer to readPump for message handling
}

// registerClient adds a new WebSocketClient to the server's active client lists.
func (gs *GameServer) registerClient(client *WebSocketClient) {
	// The client is already added to maps in handleWebSocketConnection under mutex.
	// This channel is primarily for signaling the main server loop about new clients.
	log.Printf("Client %s (ID: %s) fully registered.", client.conn.RemoteAddr().String(), client.playerID)
	gs.sendFullGameState() // Broadcast full state to all clients, including the new one.
}

// unregisterClient removes a WebSocketClient from the server's active client lists.
// It also cleans up the associated GameObject.
func (gs *GameServer) unregisterClient(client *WebSocketClient) {
	gs.clientsMutex.Lock() // Acquire lock for client maps
	if _, ok := gs.clients[client]; ok {
		delete(gs.clients, client)                   // Remove from active clients set
		delete(gs.playerIDToClient, client.playerID) // Remove from player ID map
		close(client.send)                           // Close the client's send channel to signal writePump termination
	}
	gs.clientsMutex.Unlock() // Release lock

	// Remove the player's GameObject from the game state.
	gs.gameState.RemoveObject(client.playerID)
	log.Printf("Client %s (ID: %s) unregistered and player object removed.", client.conn.RemoteAddr().String(), client.playerID)
	gs.sendFullGameState() // Broadcast updated state to remaining clients.
}

// clientMessage represents the generic structure of messages from the client.
type clientMessage struct {
	Type string          `json:"type"` // Type of message (e.g., "client_move_request")
	Data json.RawMessage `json:"data"` // Raw JSON payload, to be unmarshaled based on Type
}

// clientMoveRequestData represents the specific data for a "client_move_request" message.
type clientMoveRequestData struct {
	TargetX float64 `json:"target_x"` // Target X-coordinate for movement
	TargetY float64 `json:"target_y"` // Target Y-coordinate for movement
}

// handleClientMessage processes incoming JSON messages from a specific client.
func (gs *GameServer) handleClientMessage(client *WebSocketClient, message []byte) {
	var msg clientMessage
	if err := json.Unmarshal(message, &msg); err != nil {
		log.Printf("Client %s: ERROR unmarshaling incoming message: %v", client.playerID, err)
		return
	}

	switch msg.Type {
	case "client_move_request":
		var moveData clientMoveRequestData
		if err := json.Unmarshal(msg.Data, &moveData); err != nil {
			log.Printf("Client %s: ERROR unmarshaling move request data: %v", client.playerID, err)
			return
		}
		gs.processClientMoveRequest(client, moveData)
	default:
		log.Printf("Client %s: WARNING unknown message type '%s'.", msg.Type, client.playerID)
	}
}

// processClientMoveRequest handles the logic for a client's movement request.
func (gs *GameServer) processClientMoveRequest(client *WebSocketClient, data clientMoveRequestData) {
	gs.gameState.mu.RLock() // Acquire read lock to access player object safely
	playerObj, objExists := gs.gameState.Objects[client.playerID]
	gs.gameState.mu.RUnlock() // Release read lock

	if !objExists {
		log.Printf("ERROR: Player object %s not found for move request.", client.playerID)
		errMsg, _ := json.Marshal(map[string]string{"type": "message", "text": "Your player object was not found on the server."})
		select {
		case client.send <- errMsg: // Attempt to send error message back to client
		default:
			log.Printf("WARNING: Failed to send error message to client %s (send channel full).", client.playerID)
		}
		return
	}

	// Calculate path using A* pathfinding.
	path, err := gs.gameState.FindPath(playerObj.X, playerObj.Y, data.TargetX, data.TargetY)
	if err != nil {
		log.Printf("WARNING: No path found for player %s from (%.0f,%.0f) to (%.0f,%.0f). Error: %v", client.playerID, playerObj.X, playerObj.Y, data.TargetX, data.TargetY, err)
		errMsg, _ := json.Marshal(map[string]string{"type": "message", "text": "No path found to that location!"})
		select {
		case client.send <- errMsg: // Attempt to send error message back to client
		default:
			log.Printf("WARNING: Failed to send error message to client %s (send channel full).", client.playerID)
		}
		return
	}

	// Update player object's path in the game state.
	gs.gameState.mu.Lock() // Acquire write lock for player object modification
	playerObj.Path = path
	playerObj.PathIndex = 0
	gs.gameState.mu.Unlock() // Release write lock
	log.Printf("Calculated path for %s: %d steps.", client.playerID, len(path))

	// Send 'player_path_update' event to the specific client.
	pathUpdateMsg, _ := json.Marshal(map[string]interface{}{
		"type":      "player_path_update",
		"player_id": client.playerID,
		"path":      path,
	})
	select {
	case client.send <- pathUpdateMsg:
	default:
		log.Printf("WARNING: Failed to send player_path_update to client %s (send channel full).", client.playerID)
	}
}

// broadcastGameStateLoop periodically updates game object positions and broadcasts the full game state.
func (gs *GameServer) broadcastGameStateLoop() {
	gs.lastUpdateTime = time.Now()
	ticker := time.NewTicker(GAME_TICK_INTERVAL) // Set up game tick interval
	defer ticker.Stop()                          // Ensure ticker is stopped when goroutine exits

	for range ticker.C { // Loop runs on each game tick
		deltaTime := time.Since(gs.lastUpdateTime).Seconds() // Calculate time since last update
		gs.lastUpdateTime = time.Now()

		gs.gameState.mu.Lock() // Acquire write lock for game state objects
		for _, obj := range gs.gameState.Objects {
			if !obj.IsObstacle { // Only update positions of non-obstacle objects (players)
				obj.UpdatePosition(deltaTime)
				// If player reached end of path, clear it.
				if len(obj.Path) > 0 && obj.PathIndex >= len(obj.Path) {
					obj.Path = []struct{ X, Y float64 }{}
					obj.PathIndex = 0
					log.Printf("Player %s reached destination.", obj.ID)
				}
			}
		}
		gs.gameState.mu.Unlock() // Release write lock

		gs.sendFullGameState() // Broadcast the updated state to all clients
	}
}

// sendFullGameState marshals the current game state and sends it to all connected clients.
func (gs *GameServer) sendFullGameState() {
	gs.gameState.mu.RLock() // Acquire read lock to read game state objects
	defer gs.gameState.mu.RUnlock()

	// Create a serializable copy of game objects.
	serializableObjects := make(map[string]GameObject)
	for id, obj := range gs.gameState.Objects {
		serializableObjects[id] = *obj // Dereference to copy the struct value
	}

	// Prepare the 'game_state_update' message.
	broadcastMsg, err := json.Marshal(map[string]interface{}{
		"type":    "game_state_update",
		"objects": serializableObjects,
	})
	if err != nil {
		log.Printf("ERROR: Failed to marshal game state for broadcast: %v", err)
		return
	}

	gs.clientsMutex.RLock() // Acquire read lock for clients map
	// Create a temporary slice of clients to iterate over, preventing map modification during iteration issues.
	clientsToBroadcast := make([]*WebSocketClient, 0, len(gs.clients))
	for client := range gs.clients {
		clientsToBroadcast = append(clientsToBroadcast, client)
	}
	gs.clientsMutex.RUnlock() // Release read lock

	// Send the broadcast message to each client.
	for _, client := range clientsToBroadcast {
		select {
		case client.send <- broadcastMsg:
			// Message sent successfully to client's send channel.
		default:
			// Client's send buffer is full, indicating a potential issue or slow client.
			log.Printf("WARNING: Client %s send buffer full, attempting to unregister.", client.playerID)
			gs.unregister <- client // Signal unregistration to main server loop
		}
	}
}

// Run starts the WebSocket server and the main game loop goroutines.
func (gs *GameServer) Run() {
	// Start a goroutine to handle client registration and unregistration requests.
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

	// Start the game state broadcast loop in a separate goroutine.
	go gs.broadcastGameStateLoop()

	// Register the WebSocket endpoint.
	http.HandleFunc("/ws", gs.handleWebSocketConnection)
	// Simple HTTP handler for the root path.
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Write([]byte("Go Plain WebSocket Game Server Running. Connect to /ws for game."))
	})

	log.Printf("Starting Plain WebSocket server on http://%s:%d", SERVER_HOST, SERVER_PORT)
	// Start the HTTP server. This call blocks until a fatal error occurs.
	log.Fatal(http.ListenAndServe(fmt.Sprintf("%s:%d", SERVER_HOST, SERVER_PORT), nil))
}

func main() {
	server := NewGameServer() // Create a new instance of the game server
	server.Run()              // Start the server
}
