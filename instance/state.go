package instance

import (
	"fmt"
	"log"
	"math"
	"math/rand"
	"sync"
	"time"
)

// InstanceState manages the overall instance world state.
type InstanceState struct {
	Mu             sync.RWMutex               // Mutex for concurrent access to instance objects and maze (exported)
	Objects        map[string]*InstanceObject // All instance objects, indexed by ID
	GridCellsX     int                        // Number of conceptual grid cells horizontally
	GridCellsY     int                        // Number of conceptual grid cells vertically
	Grid           [][]*InstanceObject        // Conceptual grid for quick object lookup by position
	MazeCellsX     int                        // Number of A* maze cells horizontally
	MazeCellsY     int                        // Number of A* maze cells vertically
	SimplifiedMaze [][]int                    // Simplified maze (0: walkable, 1: obstacle) for A*
	rng            *rand.Rand                 // Random number generator for positions
}

// NewInstanceState initializes a new InstanceState with default dimensions and an empty world.
func NewInstanceState() *InstanceState {
	is := &InstanceState{
		Objects:    make(map[string]*InstanceObject),
		GridCellsX: WORLD_WIDTH / OBJECT_SIZE,
		GridCellsY: WORLD_HEIGHT / OBJECT_SIZE,
		MazeCellsX: WORLD_WIDTH / MAZE_CELL_WORLD_SIZE,
		MazeCellsY: WORLD_HEIGHT / MAZE_CELL_WORLD_SIZE,
		rng:        rand.New(rand.NewSource(time.Now().UnixNano())), // Seed RNG with current time
	}

	// Initialize the conceptual grid for object placement.
	is.Grid = make([][]*InstanceObject, is.GridCellsY)
	for i := range is.Grid {
		is.Grid[i] = make([]*InstanceObject, is.GridCellsX)
	}

	// Initialize the simplified maze for A* pathfinding.
	is.SimplifiedMaze = make([][]int, is.MazeCellsY)
	for i := range is.SimplifiedMaze {
		is.SimplifiedMaze[i] = make([]int, is.MazeCellsX)
	}

	// The maze is built after initial obstacles are added in InstanceServer.addInitialObstacles().
	return is
}

// WorldToGridCoords converts world coordinates to conceptual grid indices.
func (is *InstanceState) WorldToGridCoords(worldX, worldY float64) (int, int) {
	return int(worldX / OBJECT_SIZE), int(worldY / OBJECT_SIZE)
}

// WorldToMazeCoords converts world coordinates to A* maze indices.
func (is *InstanceState) WorldToMazeCoords(worldX, worldY float64) (int, int) {
	return int(worldX / MAZE_CELL_WORLD_SIZE), int(worldY / MAZE_CELL_WORLD_SIZE)
}

// MazeToWorldCoords converts A* maze indices to world coordinates (center of the cell).
func (is *InstanceState) MazeToWorldCoords(mazeX, mazeY int) (float64, float64) {
	return float64(mazeX*MAZE_CELL_WORLD_SIZE) + OBJECT_SIZE/2,
		float64(mazeY*MAZE_CELL_WORLD_SIZE) + OBJECT_SIZE/2
}

// BuildSimplifiedMaze populates the A* maze based on current obstacle InstanceObjects.
// This should only be called when static obstacles are added, removed, or modified.
func (is *InstanceState) BuildSimplifiedMaze() {
	is.Mu.Lock() // Acquire write lock for maze modification
	defer is.Mu.Unlock()

	// Reset all maze cells to walkable (0).
	for y := range is.SimplifiedMaze {
		for x := range is.SimplifiedMaze[y] {
			is.SimplifiedMaze[y][x] = 0
		}
	}

	// Mark cells occupied by obstacles as unwalkable (1).
	for _, obj := range is.Objects {
		if obj.IsObstacle {
			// Determine the range of maze cells this obstacle covers.
			mazeStartX, mazeStartY := is.WorldToMazeCoords(obj.X, obj.Y)
			mazeEndX, mazeEndY := is.WorldToMazeCoords(obj.X+OBJECT_SIZE-1, obj.Y+OBJECT_SIZE-1)

			// Clamp coordinates to ensure they are within maze bounds.
			mazeStartX = int(math.Max(0, math.Min(float64(mazeStartX), float64(is.MazeCellsX-1))))
			mazeStartY = int(math.Max(0, math.Min(float64(mazeStartY), float64(is.MazeCellsY-1))))
			mazeEndX = int(math.Max(0, math.Min(float64(mazeEndX), float64(is.MazeCellsX-1))))
			mazeEndY = int(math.Max(0, math.Min(float64(mazeEndY), float64(is.MazeCellsY-1))))

			// Mark all covered cells as obstacles.
			for y := mazeStartY; y <= mazeEndY; y++ {
				for x := mazeStartX; x <= mazeEndX; x++ {
					is.SimplifiedMaze[y][x] = 1
				}
			}
		}
	}
	log.Println("Simplified maze rebuilt successfully.")
}

// AddObject adds an InstanceObject to the instance state.
func (is *InstanceState) AddObject(obj *InstanceObject) {
	is.Mu.Lock() // Acquire write lock for Objects map and Grid
	defer is.Mu.Unlock()

	if _, exists := is.Objects[obj.ID]; exists {
		log.Printf("WARNING: Object with ID %s already exists, cannot add.\n", obj.ID)
		return
	}
	is.Objects[obj.ID] = obj

	// Update conceptual grid reference for the new object.
	gridX, gridY := is.WorldToGridCoords(obj.X, obj.Y)
	if gridX >= 0 && gridX < is.GridCellsX && gridY >= 0 && gridY < is.GridCellsY {
		is.Grid[gridY][gridX] = obj
	}
	// Note: BuildSimplifiedMaze() is NOT called here to prevent deadlocks and
	// because player objects (IsObstacle=false) do not affect the maze.
	log.Printf("Object added: %s at (%.0f, %.0f)", obj.ID, obj.X, obj.Y)
}

// RemoveObject removes an InstanceObject from the instance state by its ID.
func (is *InstanceState) RemoveObject(objID string) {
	is.Mu.Lock() // Acquire write lock for Objects map and Grid
	defer is.Mu.Unlock()

	obj, exists := is.Objects[objID]
	if !exists {
		log.Printf("WARNING: Attempted to remove non-existent object: %s\n", objID)
		return
	}
	delete(is.Objects, objID)

	// Clear old conceptual grid reference.
	gridX, gridY := is.WorldToGridCoords(obj.X, obj.Y)
	if gridX >= 0 && gridX < is.GridCellsX && gridY >= 0 && gridY < is.GridCellsY && is.Grid[gridY][gridX] == obj {
		is.Grid[gridY][gridX] = nil
	}
	// Note: BuildSimplifiedMaze() is NOT called here for the same reasons as AddObject.
	log.Printf("Object removed: %s\n", objID)
}

// UpdateObjectPosition updates an InstanceObject's position in the instance state.
func (is *InstanceState) UpdateObjectPosition(objID string, newX, newY float64) {
	is.Mu.Lock() // Acquire write lock for Objects map and Grid
	defer is.Mu.Unlock()

	obj, exists := is.Objects[objID]
	if !exists {
		log.Printf("WARNING: Attempted to update position of non-existent object: %s\n", objID)
		return
	}

	// Clear old conceptual grid cell reference.
	oldGridX, oldGridY := is.WorldToGridCoords(obj.X, obj.Y)
	if oldGridX >= 0 && oldGridX < is.GridCellsX && oldGridY >= 0 && oldGridY < is.GridCellsY && is.Grid[oldGridY][oldGridX] == obj {
		is.Grid[oldGridY][oldGridX] = nil
	}

	// Update object's position.
	obj.X = newX
	obj.Y = newY

	// Set new conceptual grid cell reference.
	newGridX, newGridY := is.WorldToGridCoords(newX, newY)
	if newGridX >= 0 && newGridX < is.GridCellsX && newGridY >= 0 && newGridY < is.GridCellsY {
		is.Grid[newGridY][newGridX] = obj
	}
	// Note: BuildSimplifiedMaze() is NOT called here for the same reasons as AddObject.
	log.Printf("Object %s updated to (%.0f, %.0f)\n", objID, newX, newY)
}

// GetRandomAvailablePosition finds a random empty and walkable position for a new object.
func (is *InstanceState) GetRandomAvailablePosition() (float64, float64, error) {
	is.Mu.RLock() // Acquire read lock for maze and grid
	defer is.Mu.RUnlock()

	availableCells := []struct{ X, Y float64 }{}
	for y := 0; y < is.GridCellsY; y++ {
		for x := 0; x < is.GridCellsX; x++ {
			// Check if conceptual grid cell is empty AND corresponding maze cell is walkable.
			if is.Grid[y][x] == nil {
				mazeX, mazeY := is.WorldToMazeCoords(float64(x*OBJECT_SIZE), float64(y*OBJECT_SIZE))
				// Ensure maze coordinates are within bounds before checking SimplifiedMaze.
				if mazeX >= 0 && mazeX < is.MazeCellsX && mazeY >= 0 && mazeY < is.MazeCellsY && is.SimplifiedMaze[mazeY][mazeX] == 0 {
					availableCells = append(availableCells, struct{ X, Y float64 }{X: float64(x * OBJECT_SIZE), Y: float64(y * OBJECT_SIZE)})
				}
			}
		}
	}

	if len(availableCells) == 0 {
		return 0, 0, fmt.Errorf("no available positions found on the grid")
	}
	// Select a random available cell using the InstanceState's RNG.
	randomIndex := is.rng.Intn(len(availableCells))
	return availableCells[randomIndex].X, availableCells[randomIndex].Y, nil
}
