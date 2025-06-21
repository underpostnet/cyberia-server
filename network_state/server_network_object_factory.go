package network_state

import (
	"cyberia-server/config"
	"log"
	"math/rand"

	"github.com/google/uuid" // Import the uuid package
)

// ServerNetworkObjectFactory is responsible for creating initial sets of NetworkObjects for channels.
type ServerNetworkObjectFactory struct {
	rng *rand.Rand
}

// NewServerNetworkObjectFactory creates a new factory instance.
func NewServerNetworkObjectFactory(seed int64) *ServerNetworkObjectFactory {
	return &ServerNetworkObjectFactory{
		rng: rand.New(rand.NewSource(seed)),
	}
}

func (f *ServerNetworkObjectFactory) worldToGridCoords(worldX, worldY float64) (int, int) {
	return int(worldX / config.NETWORK_OBJECT_SIZE), int(worldY / config.NETWORK_OBJECT_SIZE)
}

func (f *ServerNetworkObjectFactory) gridToWorldCoords(gridX, gridY int) (float64, float64) {
	return float64(gridX*config.NETWORK_OBJECT_SIZE) + config.NETWORK_OBJECT_SIZE/2,
		float64(gridY*config.NETWORK_OBJECT_SIZE) + config.NETWORK_OBJECT_SIZE/2
}

func (f *ServerNetworkObjectFactory) findValidSpawnLocationInGrid(
	gridMaze [][]int,
	preferredGridX, preferredGridY int,
	searchRadiusGridCells int,
	maxAttempts int,
) (int, int, bool) {
	gridCellsY := len(gridMaze)
	gridCellsX := 0
	if gridCellsY > 0 {
		gridCellsX = len(gridMaze[0])
	}

	if preferredGridY >= 0 && preferredGridY < gridCellsY &&
		preferredGridX >= 0 && preferredGridX < gridCellsX &&
		gridMaze[preferredGridY][preferredGridX] == 0 {
		return preferredGridX, preferredGridY, true
	}

	for i := 0; i < maxAttempts; i++ {
		offsetX := f.rng.Intn(2*searchRadiusGridCells+1) - searchRadiusGridCells
		offsetY := f.rng.Intn(2*searchRadiusGridCells+1) - searchRadiusGridCells
		testX := preferredGridX + offsetX
		testY := preferredGridY + offsetY

		if testY >= 0 && testY < gridCellsY &&
			testX >= 0 && testX < gridCellsX &&
			gridMaze[testY][testX] == 0 {
			return testX, testY, true
		}
	}
	log.Printf("Could not find a valid spawn location near (%d,%d) after %d attempts.", preferredGridX, preferredGridY, maxAttempts)
	return preferredGridX, preferredGridY, false // Fallback to preferred, might be on obstacle
}

// GenerateInitialState creates the initial set of network objects for a given channel ID.
func (f *ServerNetworkObjectFactory) GenerateInitialState(channelID string, playerID string) (map[string]*NetworkObject, error) {
	initialObjects := make(map[string]*NetworkObject)

	// 1. Generate Walls based on channelID
	wallCoords := f.getChannelSpecificWallCoordinates(channelID)
	tempMazeForBots := make([][]int, config.WORLD_HEIGHT/config.NETWORK_OBJECT_SIZE)
	for i := range tempMazeForBots { // Fix: Use config.WORLD_WIDTH and config.NETWORK_OBJECT_SIZE
		tempMazeForBots[i] = make([]int, config.WORLD_WIDTH/config.NETWORK_OBJECT_SIZE)
	}

	for _, wc := range wallCoords {
		wallObjID := uuid.New().String()
		initialObjects[wallObjID] = NewNetworkObject(
			wallObjID,
			wc[0], wc[1], // X, Y
			config.WallColor, // Use WallColor from config
			true, 0, "WALL",
			config.DefaultObjectLayerIDs["WALL"],
			true,
		)
		gx, gy := f.worldToGridCoords(wc[0], wc[1])
		if gx >= 0 && gx < len(tempMazeForBots[0]) && gy >= 0 && gy < len(tempMazeForBots) {
			tempMazeForBots[gy][gx] = 1
		}
	}

	// 2. Generate Player
	spawnPoints, ok := config.ChannelPlayerSpawns[channelID]
	if !ok || len(spawnPoints) == 0 {
		spawnPoints = [][2]float64{config.DefaultPlayerSpawn} // Fallback
	}
	chosenSpawn := spawnPoints[f.rng.Intn(len(spawnPoints))]
	initialObjects[playerID] = NewNetworkObject(
		playerID,
		chosenSpawn[0], chosenSpawn[1],
		config.PlayerDefaultColor,
		false, config.DefaultPlayerSpeed, "PLAYER",
		config.DefaultObjectLayerIDs["PLAYER"],
		true,
	)

	// 3. Generate Bots based on channelID
	botConfigs := f.getChannelSpecificBotConfigs(channelID)
	for _, botCfg := range botConfigs {
		botObjID := uuid.New().String()
		preferredWorldX := config.WORLD_WIDTH/2 + botCfg.InitialXOffset
		preferredWorldY := config.WORLD_HEIGHT/2 + botCfg.InitialYOffset
		prefGridX, prefGridY := f.worldToGridCoords(preferredWorldX, preferredWorldY)

		botGridX, botGridY, found := f.findValidSpawnLocationInGrid(tempMazeForBots, prefGridX, prefGridY, 15, 50)
		botSpawnX, botSpawnY := f.gridToWorldCoords(botGridX, botGridY)
		if !found {
			log.Printf("Warning: Bot %s for channel %s could not find clear spawn, using preferred offset.", botObjID, channelID)
			botSpawnX, botSpawnY = preferredWorldX, preferredWorldY
		}

		initialObjects[botObjID] = NewNetworkObject(
			botObjID,
			botSpawnX, botSpawnY,
			botCfg.Color, // Now directly config.Color
			false, config.DefaultBotSpeed, "BOT-QUEST-PROVIDER",
			botCfg.ObjectLayerIDs, // Use specific layer IDs from config
			true,
		)
	}

	return initialObjects, nil
}

func (f *ServerNetworkObjectFactory) getChannelSpecificWallCoordinates(channelID string) [][2]float64 {
	// This logic should mirror the client's NetworkObjectFactory._get_channel_specific_obstacle_mounds
	// For brevity, returning a simplified version. You'd implement the full mound logic here.
	var obstacleMoundsGrid [][4]int // {startX, startY, width, height} in grid coords

	gridCellsX := config.WORLD_WIDTH / config.NETWORK_OBJECT_SIZE
	gridCellsY := config.WORLD_HEIGHT / config.NETWORK_OBJECT_SIZE
	gridCenterX := gridCellsX / 2
	gridCenterY := gridCellsY / 2

	if channelID == config.ChannelBetaID {
		obstacleMoundsGrid = [][4]int{
			{gridCenterX - 10, gridCenterY - 15, 8, 6},
			{gridCenterX - 5, gridCenterY - 5, 10, 10},
		}
	} else { // Default to Alpha
		obstacleMoundsGrid = [][4]int{
			{gridCenterX - 15, gridCenterY - 10, 6, 8},
			{gridCenterX + 9, gridCenterY - 10, 6, 8},
		}
	}

	wallCoords := [][2]float64{}
	seen := make(map[[2]float64]bool)
	for _, mound := range obstacleMoundsGrid {
		for yOff := 0; yOff < mound[3]; yOff++ {
			for xOff := 0; xOff < mound[2]; xOff++ {
				gx, gy := mound[0]+xOff, mound[1]+yOff
				if gx >= 0 && gx < gridCellsX && gy >= 0 && gy < gridCellsY {
					wx, wy := float64(gx*config.NETWORK_OBJECT_SIZE), float64(gy*config.NETWORK_OBJECT_SIZE)
					coord := [2]float64{wx, wy}
					if !seen[coord] {
						wallCoords = append(wallCoords, coord)
						seen[coord] = true
					}
				}
			}
		}
	}
	return wallCoords
}

func (f *ServerNetworkObjectFactory) getChannelSpecificBotConfigs(channelID string) []config.BotQuestProviderConfig {
	if channelID == config.ChannelBetaID {
		return []config.BotQuestProviderConfig{
			{ObjectLayerIDs: []string{"RAVE_2"}, Color: config.Orange, InitialXOffset: -300, InitialYOffset: 100},
			{ObjectLayerIDs: []string{"RAVE_3"}, Color: config.Purple, InitialXOffset: 150, InitialYOffset: -250},
		}
	}
	// Default (Alpha)
	return []config.BotQuestProviderConfig{
		{ObjectLayerIDs: config.DefaultObjectLayerIDs["BOT-QUEST-PROVIDER"], Color: config.Magenta, InitialXOffset: -200, InitialYOffset: -200},
		{ObjectLayerIDs: []string{"PUNK"}, Color: config.Cyan, InitialXOffset: 200, InitialYOffset: 200},
	}
}
