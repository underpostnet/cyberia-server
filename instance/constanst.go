package instance

import "time"

// --- Instance Constants ---
const (
	WORLD_WIDTH            = 1600                  // Instance world width in pixels
	WORLD_HEIGHT           = 1600                  // Instance world height in pixels
	OBJECT_SIZE            = 50                    // Size of instance objects (players, obstacles) in pixels
	MAZE_CELL_WORLD_SIZE   = WORLD_WIDTH / 32      // Size of a maze cell for A* pathfinding (50x50 pixels)
	INSTANCE_TICK_INTERVAL = 50 * time.Millisecond // Instance state update interval (20 frames per second)
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
