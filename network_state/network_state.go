package network_state

import (
	"fmt"
	"log"
	"math"
	"math/rand"
	"sync"
	"time"
)

// NetworkState manages the overall network state world.
type NetworkState struct {
	Mu             sync.RWMutex              // Mutex for concurrent access to network objects and maze
	NetworkObjects map[string]*NetworkObject // All network objects, indexed by ID
	GridCellsX     int                       // Number of conceptual grid cells horizontally
	GridCellsY     int                       // Number of conceptual grid cells vertically
	Grid           [][]*NetworkObject        // Conceptual grid for quick object lookup by position
	MazeCellsX     int                       // Number of A* maze cells horizontally
	MazeCellsY     int                       // Number of A* maze cells vertically
	SimplifiedMaze [][]int                   // Simplified maze (0: walkable, 1: obstacle) for A*
	rng            *rand.Rand                // Random number generator for positions
}

// NewNetworkState initializes a new NetworkState with default dimensions and an empty world.
func NewNetworkState() *NetworkState {
	ns := &NetworkState{
		NetworkObjects: make(map[string]*NetworkObject),
		GridCellsX:     WORLD_WIDTH / NETWORK_OBJECT_SIZE,
		GridCellsY:     WORLD_HEIGHT / NETWORK_OBJECT_SIZE,
		MazeCellsX:     WORLD_WIDTH / MAZE_CELL_WORLD_SIZE,
		MazeCellsY:     WORLD_HEIGHT / MAZE_CELL_WORLD_SIZE, // Corrected constant name
		rng:            rand.New(rand.NewSource(time.Now().UnixNano())),
	}

	ns.Grid = make([][]*NetworkObject, ns.GridCellsY)
	for i := range ns.Grid {
		ns.Grid[i] = make([]*NetworkObject, ns.GridCellsX)
	}

	ns.SimplifiedMaze = make([][]int, ns.MazeCellsY)
	for i := range ns.SimplifiedMaze {
		ns.SimplifiedMaze[i] = make([]int, ns.MazeCellsX)
	}
	return ns
}

// WorldToGridCoords converts world coordinates to conceptual grid indices.
func (ns *NetworkState) WorldToGridCoords(worldX, worldY float64) (int, int) {
	return int(worldX / NETWORK_OBJECT_SIZE), int(worldY / NETWORK_OBJECT_SIZE)
}

// WorldToMazeCoords converts world coordinates to A* maze indices.
func (ns *NetworkState) WorldToMazeCoords(worldX, worldY float64) (int, int) {
	return int(worldX / MAZE_CELL_WORLD_SIZE), int(worldY / MAZE_CELL_WORLD_SIZE)
}

// MazeToWorldCoords converts A* maze indices to world coordinates (center of the cell).
func (ns *NetworkState) MazeToWorldCoords(mazeX, mazeY int) (float64, float64) {
	return float64(mazeX*MAZE_CELL_WORLD_SIZE) + NETWORK_OBJECT_SIZE/2,
		float64(mazeY*MAZE_CELL_WORLD_SIZE) + NETWORK_OBJECT_SIZE/2
}

// BuildSimplifiedMaze populates the A* maze based on current obstacle NetworkObjects.
func (ns *NetworkState) BuildSimplifiedMaze() {
	ns.Mu.Lock()
	defer ns.Mu.Unlock()

	for y := range ns.SimplifiedMaze {
		for x := range ns.SimplifiedMaze[y] {
			ns.SimplifiedMaze[y][x] = 0
		}
	}

	for _, obj := range ns.NetworkObjects {
		if obj.IsObstacle {
			mazeStartX, mazeStartY := ns.WorldToMazeCoords(obj.X, obj.Y)
			mazeEndX, mazeEndY := ns.WorldToMazeCoords(obj.X+float64(NETWORK_OBJECT_SIZE-1), obj.Y+float64(NETWORK_OBJECT_SIZE-1)) // Ensure float arithmetic

			mazeStartX = int(math.Max(0, math.Min(float64(mazeStartX), float64(ns.MazeCellsX-1))))
			mazeStartY = int(math.Max(0, math.Min(float64(mazeStartY), float64(ns.MazeCellsY-1))))
			mazeEndX = int(math.Max(0, math.Min(float64(mazeEndX), float64(ns.MazeCellsX-1))))
			mazeEndY = int(math.Max(0, math.Min(float64(mazeEndY), float64(ns.MazeCellsY-1))))

			for y := mazeStartY; y <= mazeEndY; y++ {
				for x := mazeStartX; x <= mazeEndX; x++ {
					ns.SimplifiedMaze[y][x] = 1
				}
			}
		}
	}
	log.Println("Simplified maze rebuilt successfully.")
}

// InitializeWithObjects populates the network state with a given set of objects and builds the maze.
func (ns *NetworkState) InitializeWithObjects(objects map[string]*NetworkObject) {
	ns.Mu.Lock()
	defer ns.Mu.Unlock()
	ns.NetworkObjects = objects
	// Populate grid (simplified, assumes objects don't overlap in initial state for grid assignment)
	for _, obj := range ns.NetworkObjects {
		gridX, gridY := ns.WorldToGridCoords(obj.X, obj.Y)
		if gridX >= 0 && gridX < ns.GridCellsX && gridY >= 0 && gridY < ns.GridCellsY {
			ns.Grid[gridY][gridX] = obj // Note: This simple grid doesn't handle multiple objects per cell
		}
	}
	// BuildSimplifiedMaze should be called after this by the caller (e.g., Channel)
}

// AddNetworkObject adds a NetworkObject to the network state.
func (ns *NetworkState) AddNetworkObject(obj *NetworkObject) {
	ns.Mu.Lock()
	defer ns.Mu.Unlock()

	if _, exists := ns.NetworkObjects[obj.ID]; exists {
		log.Printf("WARNING: Network object with ID %s already exists, cannot add.\n", obj.ID)
		return
	}
	ns.NetworkObjects[obj.ID] = obj

	gridX, gridY := ns.WorldToGridCoords(obj.X, obj.Y)
	if gridX >= 0 && gridX < ns.GridCellsX && gridY >= 0 && gridY < ns.GridCellsY {
		ns.Grid[gridY][gridX] = obj
	}
	log.Printf("Network object added: %s at (%.0f, %.0f)", obj.ID, obj.X, obj.Y)
}

// RemoveNetworkObject removes a NetworkObject from the network state by its ID.
func (ns *NetworkState) RemoveNetworkObject(objID string) {
	ns.Mu.Lock()
	defer ns.Mu.Unlock()

	obj, exists := ns.NetworkObjects[objID]
	if !exists {
		log.Printf("WARNING: Attempted to remove non-existent network object: %s\n", objID)
		return
	}
	delete(ns.NetworkObjects, objID)

	oldGridX, oldGridY := ns.WorldToGridCoords(obj.X, obj.Y)
	if oldGridX >= 0 && oldGridX < ns.GridCellsX && oldGridY >= 0 && oldGridY < ns.GridCellsY && ns.Grid[oldGridY][oldGridX] == obj {
		ns.Grid[oldGridY][oldGridX] = nil
	}
	log.Printf("Network object removed: %s\n", objID)
}

// UpdateNetworkObjectPosition updates a NetworkObject's position in the network state.
func (ns *NetworkState) UpdateNetworkObjectPosition(objID string, newX, newY float64) {
	ns.Mu.Lock()
	defer ns.Mu.Unlock()

	obj, exists := ns.NetworkObjects[objID]
	if !exists {
		log.Printf("WARNING: Attempted to update position of non-existent network object: %s\n", objID)
		return
	}

	oldGridX, oldGridY := ns.WorldToGridCoords(obj.X, obj.Y)
	if oldGridX >= 0 && oldGridX < ns.GridCellsX && oldGridY >= 0 && oldGridY < ns.GridCellsY && ns.Grid[oldGridY][oldGridX] == obj {
		ns.Grid[oldGridY][oldGridX] = nil
	}

	obj.X = newX
	obj.Y = newY

	newGridX, newGridY := ns.WorldToGridCoords(newX, newY)
	if newGridX >= 0 && newGridX < ns.GridCellsX && newGridY >= 0 && newGridY < ns.GridCellsY {
		ns.Grid[newGridY][newGridX] = obj
	}
	log.Printf("Network object %s updated to (%.0f, %.0f)\n", objID, newX, newY)
}

// GetRandomAvailablePosition finds a random empty and walkable position for a new network object.
func (ns *NetworkState) GetRandomAvailablePosition() (float64, float64, error) {
	ns.Mu.RLock()
	defer ns.Mu.RUnlock()

	availableCells := []struct{ X, Y float64 }{}
	for y := 0; y < ns.GridCellsY; y++ {
		for x := 0; x < ns.GridCellsX; x++ {
			if ns.Grid[y][x] == nil {
				mazeX, mazeY := ns.WorldToMazeCoords(float64(x*NETWORK_OBJECT_SIZE), float64(y*NETWORK_OBJECT_SIZE))
				if mazeX >= 0 && mazeX < ns.MazeCellsX && mazeY >= 0 && mazeY < ns.MazeCellsY && ns.SimplifiedMaze[mazeY][mazeX] == 0 {
					availableCells = append(availableCells, struct{ X, Y float64 }{X: float64(x * NETWORK_OBJECT_SIZE), Y: float64(y * NETWORK_OBJECT_SIZE)})
				}
			}
		}
	}

	if len(availableCells) == 0 {
		return 0, 0, fmt.Errorf("no available positions found on the grid")
	}
	randomIndex := ns.rng.Intn(len(availableCells))
	return availableCells[randomIndex].X, availableCells[randomIndex].Y, nil
}
