package network_state

import "time"

// Network State Constants
const (
	WORLD_WIDTH                 = 1600                  // Network state world width in pixels
	WORLD_HEIGHT                = 1600                  // Network state world height in pixels
	NETWORK_OBJECT_SIZE         = 50                    // Size of network objects (players, obstacles) in pixels
	MAZE_CELL_WORLD_SIZE        = 50                    // Size of a maze cell for A* pathfinding (50x50 pixels)
	NETWORK_STATE_TICK_INTERVAL = 50 * time.Millisecond // Network state update interval (20 frames per second)
)

// Color represents a simplified RGB representation for server.
type Color struct {
	R, G, B, A uint8
}

var (
	BLUE     = Color{0, 0, 255, 255}
	DARKGRAY = Color{80, 80, 80, 255}
	RED      = Color{255, 0, 0, 255}
	YELLOW   = Color{255, 255, 0, 255}
)
