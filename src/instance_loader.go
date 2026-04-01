// Package game — instance_loader.go
//
// Builds the Go game server's in-memory world from gRPC data.
// Replaces the old hardcoded createMaps() approach: the Go server
// no longer creates its own maps, entities, or portals.
// Everything is fetched from the Engine-Cyberia via GetFullInstance.
package game

import (
	"fmt"
	"log"
	"math"
	"math/rand"

	pb "cyberia-server/proto"

	"github.com/google/uuid"
)

// portalEntry tracks a portal entity for cross-map linking.
type portalEntry struct {
	portal  *PortalState
	cellX   int
	cellY   int
	mapCode string
}

// BuildWorldFromInstance reconstructs the entire game world from gRPC
// GetFullInstance response data. This is called by WorldBuilder.
func (s *GameServer) BuildWorldFromInstance(
	instance *pb.InstanceMessage,
	mapMsgs []*pb.MapDataMessage,
	olMsgs []*pb.ObjectLayerMessage,
) {
	s.mu.Lock()
	defer s.mu.Unlock()

	log.Printf("[InstanceLoader] Building world for instance %q (%d maps, %d object layers)",
		instance.GetCode(), len(mapMsgs), len(olMsgs))

	// Build map-code → MapDataMessage lookup
	mapMsgByCode := make(map[string]*pb.MapDataMessage)
	for _, m := range mapMsgs {
		mapMsgByCode[m.GetCode()] = m
	}

	// Reset maps (keyed by map code)
	s.maps = make(map[string]*MapState)

	allPortals := make(map[string][]portalEntry) // mapCode → portals

	// Phase 1: Build each map
	for _, mapCode := range instance.GetMapCodes() {
		mapMsg, ok := mapMsgByCode[mapCode]
		if !ok {
			log.Printf("[InstanceLoader] WARNING: map code %q in instance but not in gRPC response", mapCode)
			continue
		}

		gridW := int(mapMsg.GetGridX())
		gridH := int(mapMsg.GetGridY())
		if gridW <= 0 {
			gridW = 100
		}
		if gridH <= 0 {
			gridH = 100
		}

		ms := &MapState{
			gridW:       gridW,
			gridH:       gridH,
			players:     make(map[string]*PlayerState),
			portals:     make(map[string]*PortalState),
			obstacles:   make(map[string]ObjectState),
			foregrounds: make(map[string]ObjectState),
			floors:      make(map[string]*FloorState),
			pathfinder:  NewPathfinder(gridW, gridH),
			bots:        make(map[string]*BotState),
		}

		// Process entities from the map
		for _, ent := range mapMsg.GetEntities() {
			switch ent.GetEntityType() {
			case "floor":
				s.buildFloor(ms, ent)
			case "bot":
				s.buildBot(ms, mapCode, ent)
			case "obstacle":
				s.buildObstacle(ms, ent)
			case "foreground":
				s.buildForeground(ms, ent)
			case "portal":
				portal := s.buildPortal(ms, ent)
				if portal != nil {
					allPortals[mapCode] = append(allPortals[mapCode], portalEntry{
						portal:  portal,
						cellX:   int(ent.GetInitCellX()),
						cellY:   int(ent.GetInitCellY()),
						mapCode: mapCode,
					})
				}
			default:
				log.Printf("[InstanceLoader] Unknown entity type %q in map %q", ent.GetEntityType(), mapCode)
			}
		}

		s.maps[mapCode] = ms
	}

	// Phase 2: Link portals using instance portal edges
	for _, edge := range instance.GetPortals() {
		srcMapCode := edge.GetSourceMapCode()
		dstMapCode := edge.GetTargetMapCode()

		_, srcOk := s.maps[srcMapCode]
		_, dstOk := s.maps[dstMapCode]
		if !srcOk || !dstOk {
			log.Printf("[InstanceLoader] WARNING: portal edge references unknown map: %q → %q", srcMapCode, dstMapCode)
			continue
		}

		// Find the source portal by matching cell coordinates
		srcPortal := findPortalByCell(allPortals[srcMapCode], int(edge.GetSourceCellX()), int(edge.GetSourceCellY()))
		dstPortal := findPortalByCell(allPortals[dstMapCode], int(edge.GetTargetCellX()), int(edge.GetTargetCellY()))

		if srcPortal == nil {
			log.Printf("[InstanceLoader] WARNING: no portal entity at (%d,%d) on map %q",
				edge.GetSourceCellX(), edge.GetSourceCellY(), srcMapCode)
			continue
		}

		srcPortal.PortalConfig = &PortalConfig{
			DestMapCode: dstMapCode,
			SpawnRadius: s.portalSpawnRadius,
		}

		if dstPortal != nil {
			srcPortal.Label = fmt.Sprintf("%s, Pos: (%d, %d)",
				dstMapCode, int(dstPortal.Pos.X), int(dstPortal.Pos.Y))
		} else {
			srcPortal.Label = dstMapCode
		}
	}

	log.Printf("[InstanceLoader] World built: %d maps loaded.", len(s.maps))

	// Log entity counts per map
	for code, ms := range s.maps {
		log.Printf("[InstanceLoader]   Map %q: %d floors, %d obstacles, %d foregrounds, %d portals, %d bots",
			code, len(ms.floors), len(ms.obstacles), len(ms.foregrounds), len(ms.portals), len(ms.bots))
	}
}

// findPortalByCell finds a portal entry by cell coordinates.
func findPortalByCell(entries []portalEntry, cellX, cellY int) *PortalState {
	for _, e := range entries {
		if e.cellX == cellX && e.cellY == cellY {
			return e.portal
		}
	}
	return nil
}

// ── Entity builders ──────────────────────────────────────────────

func (s *GameServer) buildFloor(ms *MapState, ent *pb.EntityMessage) {
	floor := &FloorState{
		ID:   uuid.New().String(),
		Pos:  Point{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())},
		Dims: Dimensions{Width: float64(ent.GetDimX()), Height: float64(ent.GetDimY())},
		Type: "floor",
	}
	for _, itemID := range ent.GetObjectLayerItemIds() {
		floor.ObjectLayers = append(floor.ObjectLayers, ObjectLayerState{
			ItemID: itemID, Active: true, Quantity: 1,
		})
	}
	if len(floor.ObjectLayers) == 0 {
		if d, ok := s.entityDefaults["floor"]; ok && d.LiveItemID != "" {
			floor.ObjectLayers = []ObjectLayerState{{ItemID: d.LiveItemID, Active: true, Quantity: 1}}
		}
	}
	ms.floors[floor.ID] = floor
}

func (s *GameServer) buildBot(ms *MapState, mapCode string, ent *pb.EntityMessage) {
	dims := Dimensions{Width: float64(ent.GetDimX()), Height: float64(ent.GetDimY())}
	if dims.Width <= 0 {
		dims.Width = float64(rand.Intn(4) + 1)
	}
	if dims.Height <= 0 {
		dims.Height = float64(rand.Intn(4) + 1)
	}

	startPos := Point{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())}

	// Determine max life
	maxLife := s.entityBaseMaxLife
	if ent.GetMaxLife() > 0 {
		maxLife = ent.GetMaxLife()
	}

	lifeRegen := s.playerBaseLifeRegenMin + rand.Float64()*(s.playerBaseLifeRegenMax-s.playerBaseLifeRegenMin)
	if ent.GetLifeRegen() > 0 {
		lifeRegen = ent.GetLifeRegen()
	}

	spawnRadius := rand.Float64()*6.0 + 3.0
	if ent.GetSpawnRadius() > 0 {
		spawnRadius = ent.GetSpawnRadius()
	}

	aggroRange := s.botAggroRange
	if ent.GetAggroRange() > 0 {
		aggroRange = ent.GetAggroRange()
	}

	// Build object layers
	var objectLayers []ObjectLayerState
	for _, itemID := range ent.GetObjectLayerItemIds() {
		objectLayers = append(objectLayers, ObjectLayerState{
			ItemID: itemID, Active: true, Quantity: 1,
		})
	}
	// If no items are assigned in the map definition use the instance-level
	// bot default visual (atlas if present, solid BOT colour otherwise).
	if len(objectLayers) == 0 {
		if d, ok := s.entityDefaults["bot"]; ok && d.LiveItemID != "" {
			objectLayers = []ObjectLayerState{{ItemID: d.LiveItemID, Active: true, Quantity: 1}}
		}
	}

	// Determine behavior from object layers:
	// If the bot has a weapon-type item → hostile, otherwise passive
	behavior := "passive"
	for _, ol := range objectLayers {
		if data, ok := s.GetObjectLayerData(ol.ItemID); ok {
			if data.Data.Item.Type == "weapon" {
				behavior = "hostile"
				break
			}
		}
	}

	bot := &BotState{
		ID:           uuid.New().String(),
		MapCode:      mapCode,
		Pos:          startPos,
		Dims:         dims,
		Path:         []PointI{},
		TargetPos:    PointI{-1, -1},
		Direction:    NONE,
		Mode:         IDLE,
		Behavior:     behavior,
		SpawnCenter:  startPos,
		SpawnRadius:  spawnRadius,
		AggroRange:   aggroRange,
		MaxLife:      maxLife,
		Life:         maxLife * s.initialLifeFraction,
		LifeRegen:    lifeRegen,
		ObjectLayers: objectLayers,
		Color:        s.colors["BOT"],
	}

	// Apply initial stats
	s.ApplyResistanceStat(bot, ms)
	bot.Life = bot.MaxLife * s.initialLifeFraction

	// Initial wandering path
	target := s.randomPointWithinRadius(ms, bot.SpawnCenter, bot.SpawnRadius, bot.Dims)
	if target.X >= 0 {
		if pth, err := ms.pathfinder.Astar(
			PointI{X: int(math.Round(bot.Pos.X)), Y: int(math.Round(bot.Pos.Y))},
			target, bot.Dims,
		); err == nil && len(pth) > 0 {
			bot.Path = pth
			bot.TargetPos = target
			bot.Mode = WALKING
		}
	}

	ms.bots[bot.ID] = bot
}

func (s *GameServer) buildObstacle(ms *MapState, ent *pb.EntityMessage) {
	dims := Dimensions{Width: float64(ent.GetDimX()), Height: float64(ent.GetDimY())}
	pos := Point{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())}

	obs := ObjectState{
		ID:   uuid.New().String(),
		Pos:  pos,
		Dims: dims,
		Type: "obstacle",
	}
	ms.obstacles[obs.ID] = obs

	// Mark grid cells as blocked
	for y := int(pos.Y); y < int(pos.Y+dims.Height); y++ {
		for x := int(pos.X); x < int(pos.X+dims.Width); x++ {
			if x >= 0 && x < ms.gridW && y >= 0 && y < ms.gridH {
				ms.pathfinder.grid[y][x] = 1
			}
		}
	}
}

func (s *GameServer) buildForeground(ms *MapState, ent *pb.EntityMessage) {
	fg := ObjectState{
		ID:   uuid.New().String(),
		Pos:  Point{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())},
		Dims: Dimensions{Width: float64(ent.GetDimX()), Height: float64(ent.GetDimY())},
		Type: "foreground",
	}
	ms.foregrounds[fg.ID] = fg
}

func (s *GameServer) buildPortal(ms *MapState, ent *pb.EntityMessage) *PortalState {
	dims := Dimensions{Width: float64(ent.GetDimX()), Height: float64(ent.GetDimY())}
	if dims.Width <= 0 {
		dims.Width = 3
	}
	if dims.Height <= 0 {
		dims.Height = 3
	}

	portal := &PortalState{
		ID:   uuid.New().String(),
		Pos:  Point{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())},
		Dims: dims,
		Type: "portal",
	}
	ms.portals[portal.ID] = portal

	// Mark portal cells in pathfinder grid
	for y := int(portal.Pos.Y); y < int(portal.Pos.Y+dims.Height); y++ {
		for x := int(portal.Pos.X); x < int(portal.Pos.X+dims.Width); x++ {
			if x >= 0 && x < ms.gridW && y >= 0 && y < ms.gridH {
				ms.pathfinder.grid[y][x] = 1
			}
		}
	}

	return portal
}

// generateFloors creates a default floor grid if no floors are in the DB.
func (ms *MapState) generateFloors(rows, cols int, floorItemID string) {
	if rows <= 0 || cols <= 0 {
		return
	}
	tileWidth := float64(ms.gridW) / float64(cols)
	tileHeight := float64(ms.gridH) / float64(rows)

	for r := 0; r < rows; r++ {
		for c := 0; c < cols; c++ {
			floor := &FloorState{
				ID:   uuid.New().String(),
				Pos:  Point{X: float64(c) * tileWidth, Y: float64(r) * tileHeight},
				Dims: Dimensions{Width: tileWidth, Height: tileHeight},
				Type: "floor",
			}
			if floorItemID != "" {
				floor.ObjectLayers = []ObjectLayerState{
					{ItemID: floorItemID, Active: true, Quantity: 1},
				}
			}
			ms.floors[floor.ID] = floor
		}
	}
}
