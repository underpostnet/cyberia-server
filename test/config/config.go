package config

import "time"

// Game World Dimensions and Object Sizes
const (
	WORLD_WIDTH          = 3000                // Network state world width in pixels
	WORLD_HEIGHT         = 3000                // Network state world height in pixels
	NETWORK_OBJECT_SIZE  = 50                  // Size of network objects (players, obstacles) in pixels
	MAZE_CELL_WORLD_SIZE = NETWORK_OBJECT_SIZE // Size of a maze cell for A* pathfinding, typically same as object size
)

// Game Speeds
const (
	DefaultPlayerSpeed = 200.0
	DefaultBotSpeed    = 150.0
)

// Network State Update Interval
const NETWORK_STATE_TICK_INTERVAL = 50 * time.Millisecond // Network state update interval (20 frames per second)

// Channel IDs
const (
	ChannelAlphaID = "channel_alpha"
	ChannelBetaID  = "channel_beta"
)

// DefaultChannelID is the channel clients are assigned to by default.
const DefaultChannelID = ChannelAlphaID

// AvailableChannels lists all channels the server can run.
var AvailableChannels = []string{ChannelAlphaID, ChannelBetaID}

// DefaultObjectLayerIDs maps network object types to their default layer IDs.
var DefaultObjectLayerIDs = map[string][]string{
	"PLAYER":             {"ACTION_AREA", "ANON"},
	"WALL":               {"WALL"},
	"BOT-QUEST-PROVIDER": {"RAVE_0"},
}

// ChannelPlayerSpawns defines player spawn points (world coordinates) for each channel.
// Format: map[channelID][][2]float64 where [2]float64 is {X, Y}
var ChannelPlayerSpawns = map[string][][2]float64{
	ChannelAlphaID: {{1650, 1475}, {1350, 1525}, {1500, 1325}},
	ChannelBetaID: {
		{750, 750},
		{2200, 750},
		{1500, 2225},
	},
}

// Predefined Clean Background Colors
var ChannelAlphaCleanBackgroundColor = []int{110, 29, 134, 255}
var ChannelBetaCleanBackgroundColor = []int{74, 163, 151, 255}

// DefaultPlayerSpawn is a fallback spawn point.
var DefaultPlayerSpawn = [2]float64{
	1475, // 3000/2 - 50/2
	1475, // 3000/2 - 50/2
}

// Color represents a simplified RGB representation.
type Color struct {
	R, G, B, A uint8
}

// Predefined Colors
var (
	PlayerDefaultColor = Color{0, 121, 241, 255}
	WallColor          = Color{100, 100, 100, 255}
	Magenta            = Color{R: 255, G: 0, B: 255, A: 255}
	Cyan               = Color{R: 0, G: 255, B: 255, A: 255}
	Orange             = Color{R: 255, G: 165, B: 0, A: 255}
	Purple             = Color{R: 128, G: 0, B: 128, A: 255}
)

// BotQuestProviderConfig defines properties for a bot.
type BotQuestProviderConfig struct {
	ObjectLayerIDs []string
	Color          Color // Use local Color type
	InitialXOffset float64
	InitialYOffset float64
}
