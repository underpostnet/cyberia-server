package network_state

import "time"

// Network State Constants
const (
	WORLD_WIDTH                 = 3000                  // Network state world width in pixels
	WORLD_HEIGHT                = 3000                  // Network state world height in pixels
	NETWORK_OBJECT_SIZE         = 50                    // Size of network objects (players, obstacles) in pixels
	MAZE_CELL_WORLD_SIZE        = NETWORK_OBJECT_SIZE   // Size of a maze cell for A* pathfinding, typically same as object size
	NETWORK_STATE_TICK_INTERVAL = 50 * time.Millisecond // Network state update interval (20 frames per second)
)

// Color represents a simplified RGB representation for server.
type Color struct {
	R, G, B, A uint8
}

var (
	PlayerDefaultColor = Color{0, 121, 241, 255}
	WallColor          = Color{100, 100, 100, 255}
	// BLUE     = Color{0, 0, 255, 255} // Example, can be removed if PlayerDefaultColor is used
	// DARKGRAY = Color{80, 80, 80, 255}
	// RED      = Color{255, 0, 0, 255} // Example, specific colors can be defined in config
	// YELLOW   = Color{255, 255, 0, 255}
)
