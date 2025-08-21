package game

import (
	"fmt"
	"log"
	"math"
	"math/rand"

	"github.com/google/uuid"
)

// Global variable to hold portal states for cross-map linking
var allMapsPortals [][]*PortalState

// createMaps initializes maps, portals, obstacles, foregrounds, and bots.
func (s *GameServer) createMaps() {
	log.Println("Creating game maps...")
	const (
		numMaps    = 3
		gridSizeW  = 100
		gridSizeH  = 100
		numObs     = 100
		numPortals = 2
	)

	portalConfigs := [numMaps][]PortalConfig{
		{
			{DestMapID: 1, DestPortalIndex: 0, SpawnRadius: 2.0},
			{DestMapID: 0, DestPortalIndex: 1, SpawnRadius: 2.0},
		},
		{
			{DestMapID: 0, DestPortalIndex: 0, SpawnRadius: 2.0},
			{DestMapID: 2, DestPortalIndex: 1, SpawnRadius: 2.0},
		},
		{
			{DestMapID: 1, DestPortalIndex: 1, SpawnRadius: 2.0},
			{DestMapID: 2, DestPortalIndex: 0, SpawnRadius: 2.0},
		},
	}

	allMapsPortals = make([][]*PortalState, numMaps)

	for i := 0; i < numMaps; i++ {
		ms := &MapState{
			gridW:       gridSizeW,
			gridH:       gridSizeH,
			players:     make(map[string]*PlayerState),
			portals:     make(map[string]*PortalState),
			obstacles:   make(map[string]ObjectState),
			foregrounds: make(map[string]ObjectState),
			pathfinder:  NewPathfinder(gridSizeW, gridSizeH),
			bots:        make(map[string]*BotState),
		}

		portals := ms.generatePortals(numPortals)
		allMapsPortals[i] = portals

		for _, p := range portals {
			ms.portals[p.ID] = p
		}

		portalRects := make([]Rectangle, len(portals))
		for idx, p := range portals {
			portalRects[idx] = Rectangle{MinX: p.Pos.X, MinY: p.Pos.Y, MaxX: p.Pos.X + p.Dims.Width, MaxY: p.Pos.Y + p.Dims.Height}
		}

		ms.pathfinder.GenerateObstacles(numObs, portalRects)
		ms.obstacles = ms.pathfinder.obstacles

		for _, obs := range ms.obstacles {
			fg := ObjectState{
				ID:   uuid.New().String(),
				Pos:  Point{X: obs.Pos.X, Y: obs.Pos.Y - obs.Dims.Height},
				Dims: obs.Dims,
				Type: "foreground",
			}
			ms.foregrounds[fg.ID] = fg
		}

		// instantiate bots for this map
		s.instantiateBots(ms, i)

		s.maps[i] = ms
	}

	for mapID, portals := range allMapsPortals {
		for portalIndex, portal := range portals {
			portal.PortalConfig = &portalConfigs[mapID][portalIndex]
			destPortal := allMapsPortals[portal.PortalConfig.DestMapID][portal.PortalConfig.DestPortalIndex]
			portal.Label = fmt.Sprintf("Map %d, Pos: (%d, %d)",
				portal.PortalConfig.DestMapID,
				int(destPortal.Pos.X),
				int(destPortal.Pos.Y),
			)
		}
	}

	log.Println("Maps created and portals linked.")
}

// generatePortals creates and places portals on a map.
func (ms *MapState) generatePortals(numPortals int) []*PortalState {
	portals := make([]*PortalState, 0, numPortals)
	for i := 0; i < numPortals; i++ {
		dims := Dimensions{Width: float64(rand.Intn(3) + 2), Height: float64(rand.Intn(3) + 2)}
		posI, err := ms.pathfinder.findRandomWalkablePoint(dims)
		if err != nil {
			log.Printf("Error placing portal: %v", err)
			continue
		}
		portal := &PortalState{
			ID:  uuid.New().String(),
			Pos: Point{X: float64(posI.X), Y: float64(posI.Y)},
			Dims: dims,
		}
		portals = append(portals, portal)
		for y := posI.Y; y < posI.Y+int(dims.Height); y++ {
			for x := posI.X; x < posI.X+int(dims.Width); x++ {
				if x >= 0 && x < ms.gridW && y >= 0 && y < ms.gridH {
					ms.pathfinder.grid[y][x] = 1
				}
			}
		}
	}
	return portals
}

// instantiateBots creates bots for a map with random passive/hostile distribution.
func (s *GameServer) instantiateBots(ms *MapState, mapID int) {
	for i := 0; i < s.botsPerMap; i++ {
		// random dims and spawn point
		dims := Dimensions{Width: float64(rand.Intn(4) + 1), Height: float64(rand.Intn(4) + 1)}
		startPosI, err := ms.pathfinder.findRandomWalkablePoint(dims)
		if err != nil {
			continue
		}
		startPos := Point{X: float64(startPosI.X), Y: float64(startPosI.Y)}
		spawnRadius := rand.Float64()*6.0 + 3.0 // 3..9
		behavior := "passive"
		if rand.Intn(2) == 0 {
			behavior = "hostile"
		}

		bot := &BotState{
			ID:          uuid.New().String(),
			MapID:       mapID,
			Pos:         startPos,
			Dims:        dims,
			Path:        []PointI{},
			TargetPos:   PointI{-1, -1},
			Direction:   NONE,
			Mode:        IDLE,
			Behavior:    behavior,
			SpawnCenter: startPos,
			SpawnRadius: spawnRadius,
			AggroRange:  s.botAggroRange,
		}

		// initial wandering path: random point within spawn radius
		target := s.randomPointWithinRadius(ms, bot.SpawnCenter, bot.SpawnRadius, bot.Dims)
		if target.X >= 0 {
			if pth, err := ms.pathfinder.Astar(PointI{X: int(math.Round(bot.Pos.X)), Y: int(math.Round(bot.Pos.Y))}, target, bot.Dims); err == nil && len(pth) > 0 {
				bot.Path = pth
				bot.TargetPos = target
				bot.Mode = WALKING
			}
		}
		ms.bots[bot.ID] = bot
	}
}
