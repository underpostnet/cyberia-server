package config

const (
	// DefaultPlayerSpeed is the default speed for player objects.
	DefaultPlayerSpeed = 200.0
	// DefaultBotSpeed is the default speed for bot objects.
	DefaultBotSpeed = 150.0
)

// DefaultObjectLayerIDs maps network object types to their default layer IDs.
var DefaultObjectLayerIDs = map[string][]string{
	"PLAYER":             {"ACTION_AREA", "ANON"},
	"WALL":               {"WALL"},
	"BOT-QUEST-PROVIDER": {"RAVE_0"},
}

// ChannelPlayerSpawns defines player spawn points (world coordinates) for each channel.
// Format: map[channelID][][2]float64 where [2]float64 is {X, Y}
var ChannelPlayerSpawns = map[string][][2]float64{
	ChannelAlphaID: {{1650, 1475}, {1350, 1525}, {1500, 1325}}, // Assuming WORLD_WIDTH=3000, WORLD_HEIGHT=3000, NETWORK_OBJECT_SIZE=50
	ChannelBetaID: {
		{750, 750},
		{2200, 750},  // 3000 * 3/4 - 50
		{1500, 2225}, // 3000 * 3/4 - 25
	},
	// Note: Hardcoded values derived from network_state constants to break import cycle.
	// If network_state constants change, these values must be manually updated.
	// WORLD_WIDTH=3000, WORLD_HEIGHT=3000, NETWORK_OBJECT_SIZE=50
}

// DefaultPlayerSpawn is a fallback spawn point.
var DefaultPlayerSpawn = [2]float64{
	1475, // 3000/2 - 50/2
	1475, // 3000/2 - 50/2
}

// Channel IDs
const (
	ChannelAlphaID = "channel_alpha"
	ChannelBetaID  = "channel_beta"
)

// DefaultChannelID is the channel clients are assigned to by default.
const DefaultChannelID = ChannelAlphaID

// AvailableChannels lists all channels the server can run.
var AvailableChannels = []string{ChannelAlphaID, ChannelBetaID}

// BotQuestProviderConfig defines properties for a bot.
type BotQuestProviderConfig struct {
	ObjectLayerIDs []string
	Color          Color // Use local Color type
	InitialXOffset float64
	InitialYOffset float64
}

// Color represents a simplified RGB representation.
// Duplicated from network_state to break import cycle.
type Color struct {
	R, G, B, A uint8
}

// Define colors locally to break import cycle
var Magenta = Color{R: 255, G: 0, B: 255, A: 255}
var Cyan = Color{R: 0, G: 255, B: 255, A: 255}
var Orange = Color{R: 255, G: 165, B: 0, A: 255}
var Purple = Color{R: 128, G: 0, B: 128, A: 255}
