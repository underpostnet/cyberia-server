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

	s.buildMapsFromInstance(instance, mapMsgs)

	log.Printf("[InstanceLoader] World built: %d maps loaded.", len(s.maps))

	// Log entity counts per map
	for code, ms := range s.maps {
		log.Printf("[InstanceLoader]   Map %q: %d floors, %d obstacles, %d foregrounds, %d portals, %d bots",
			code, len(ms.floors), len(ms.obstacles), len(ms.foregrounds), len(ms.portals), len(ms.bots))
	}

	// Print ASCII graph of portal topology
	printInstanceGraph(instance, s.maps)
}

// RebuildWorld re-fetches instance data and rebuilds all map entities
// (floors, obstacles, foregrounds, portals, bots) while preserving
// connected players.  Called by the hot-reload loop so that map edits
// made in the admin UI are reflected without a full server restart.
func (s *GameServer) RebuildWorld(
	instance *pb.InstanceMessage,
	mapMsgs []*pb.MapDataMessage,
	olMsgs []*pb.ObjectLayerMessage,
) {
	s.mu.Lock()
	defer s.mu.Unlock()

	log.Printf("[InstanceLoader] RebuildWorld for instance %q (%d maps)", instance.GetCode(), len(mapMsgs))

	// Snapshot existing players per map so they survive the rebuild.
	savedPlayers := make(map[string]map[string]*PlayerState)
	for code, ms := range s.maps {
		ms.mu.RLock()
		if len(ms.players) > 0 {
			cp := make(map[string]*PlayerState, len(ms.players))
			for id, p := range ms.players {
				cp[id] = p
			}
			savedPlayers[code] = cp
		}
		ms.mu.RUnlock()
	}

	// Rebuild via the same path as initial load.
	s.buildMapsFromInstance(instance, mapMsgs)

	// Restore players into the (possibly new) map states.
	for code, players := range savedPlayers {
		ms, ok := s.maps[code]
		if !ok {
			// Map was removed — players will be orphaned and eventually
			// time out or reconnect.  Log a warning.
			log.Printf("[InstanceLoader] WARNING: map %q was removed during rebuild; %d players orphaned", code, len(players))
			continue
		}
		ms.mu.Lock()
		for id, p := range players {
			ms.players[id] = p
		}
		ms.mu.Unlock()
	}

	log.Printf("[InstanceLoader] RebuildWorld complete: %d maps.", len(s.maps))
	for code, ms := range s.maps {
		log.Printf("[InstanceLoader]   Map %q: %d floors, %d obstacles, %d foregrounds, %d portals, %d bots, %d players",
			code, len(ms.floors), len(ms.obstacles), len(ms.foregrounds), len(ms.portals), len(ms.bots), len(ms.players))
	}
	printInstanceGraph(instance, s.maps)
}

// buildMapsFromInstance is the shared implementation for BuildWorldFromInstance
// and RebuildWorld.  Caller MUST hold s.mu.
func (s *GameServer) buildMapsFromInstance(
	instance *pb.InstanceMessage,
	mapMsgs []*pb.MapDataMessage,
) {
	mapMsgByCode := make(map[string]*pb.MapDataMessage)
	for _, m := range mapMsgs {
		mapMsgByCode[m.GetCode()] = m
	}

	s.maps = make(map[string]*MapState)
	allPortals := make(map[string][]portalEntry)

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

	// Link portals
	for _, edge := range instance.GetPortals() {
		srcMapCode := edge.GetSourceMapCode()
		dstMapCode := edge.GetTargetMapCode()

		_, srcOk := s.maps[srcMapCode]
		_, dstOk := s.maps[dstMapCode]
		if !srcOk || !dstOk {
			log.Printf("[InstanceLoader] WARNING: portal edge references unknown map: %q → %q", srcMapCode, dstMapCode)
			continue
		}

		srcPortal := findPortalByCell(allPortals[srcMapCode], int(edge.GetSourceCellX()), int(edge.GetSourceCellY()))
		dstPortal := findPortalByCell(allPortals[dstMapCode], int(edge.GetTargetCellX()), int(edge.GetTargetCellY()))

		if srcPortal == nil {
			log.Printf("[InstanceLoader] WARNING: no portal entity at (%d,%d) on map %q",
				edge.GetSourceCellX(), edge.GetSourceCellY(), srcMapCode)
			continue
		}

		portalMode := edge.GetPortalMode()
		if portalMode == "" {
			portalMode = "inter-portal"
		}

		srcPortal.PortalConfig = &PortalConfig{
			DestMapCode: dstMapCode,
			SpawnRadius: s.portalSpawnRadius,
			PortalMode:  portalMode,
			DestCellX:   float64(edge.GetTargetCellX()),
			DestCellY:   float64(edge.GetTargetCellY()),
		}

		modeTag := fmt.Sprintf("[%s]", portalMode)
		if dstPortal != nil {
			srcPortal.Label = fmt.Sprintf("%s %s, Pos: (%d, %d)",
				modeTag, dstMapCode, int(dstPortal.Pos.X), int(dstPortal.Pos.Y))
		} else {
			srcPortal.Label = fmt.Sprintf("%s %s", modeTag, dstMapCode)
		}
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
		ID:    uuid.New().String(),
		Pos:   Point{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())},
		Dims:  Dimensions{Width: float64(ent.GetDimX()), Height: float64(ent.GetDimY())},
		Type:  "floor",
		Color: s.resolveEntityColor(ent, "FLOOR"),
	}
	for _, itemID := range ent.GetObjectLayerItemIds() {
		floor.ObjectLayers = append(floor.ObjectLayers, ObjectLayerState{
			ItemID: itemID, Active: true, Quantity: 1,
		})
	}
	if len(floor.ObjectLayers) == 0 && ent.GetColorA() == 0 {
		// Apply instance-level default OLs only when the entity has no
		// explicit DB colour.  A non-zero ColorA means the map creator
		// intentionally placed a solid-colour floor without sprites.
		if d, ok := s.entityDefaults["floor"]; ok && len(d.LiveItemIDs) > 0 {
			for _, itemID := range d.LiveItemIDs {
				floor.ObjectLayers = append(floor.ObjectLayers, ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
			}
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
	// bot default visual (sprite if present, solid BOT colour otherwise).
	if len(objectLayers) == 0 {
		if d, ok := s.entityDefaults["bot"]; ok && len(d.LiveItemIDs) > 0 {
			for _, itemID := range d.LiveItemIDs {
				objectLayers = append(objectLayers, ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
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
		Behavior:     s.DetermineBotBehavior(objectLayers),
		SpawnCenter:  startPos,
		SpawnRadius:  spawnRadius,
		AggroRange:   aggroRange,
		MaxLife:      maxLife,
		Life:         maxLife * s.initialLifeFraction,
		LifeRegen:    lifeRegen,
		ObjectLayers: objectLayers,
		Color:        s.resolveEntityColor(ent, "BOT"),
	}

	// Apply initial stats
	s.ApplyResistanceStat(bot, ms)
	bot.Life = bot.MaxLife * s.initialLifeFraction

	// Fountain: credit the bot's starting coin supply (same path as respawn).
	s.FountainInitBot(bot)

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
		ID:    uuid.New().String(),
		Pos:   pos,
		Dims:  dims,
		Type:  "obstacle",
		Color: s.resolveEntityColor(ent, "OBSTACLE"),
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
		ID:    uuid.New().String(),
		Pos:   Point{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())},
		Dims:  Dimensions{Width: float64(ent.GetDimX()), Height: float64(ent.GetDimY())},
		Type:  "foreground",
		Color: s.resolveEntityColor(ent, "FOREGROUND"),
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

	subtype := ent.GetPortalSubtype()
	if subtype == "" {
		subtype = "inter-portal"
	}

	// Resolve colour: per-entity DB colour overrides palette default.
	color := s.resolveEntityColor(ent, portalSubtypeColorKey(subtype))
	if color == (ColorRGBA{}) {
		color = s.colors["PORTAL"]
	}

	portal := &PortalState{
		ID:      uuid.New().String(),
		Pos:     Point{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())},
		Dims:    dims,
		Type:    "portal",
		Subtype: subtype,
		Color:   color,
	}
	ms.portals[portal.ID] = portal

	// Portals are walkable — no grid cells are blocked.
	// The player must stand on the portal for portalHoldTime to trigger transport.

	return portal
}

// resolveEntityColor returns the per-entity DB colour when it has non-zero
// alpha, falling back to the named palette colour identified by paletteKey.
// This ensures that map-placed entities with a custom colour in MongoDB
// override the instance-wide palette default (fallback for no ObjectLayers).
func (s *GameServer) resolveEntityColor(ent *pb.EntityMessage, paletteKey string) ColorRGBA {
	if ent.GetColorA() > 0 {
		return ColorRGBA{
			R: int(ent.GetColorR()),
			G: int(ent.GetColorG()),
			B: int(ent.GetColorB()),
			A: int(ent.GetColorA()),
		}
	}
	return s.colors[paletteKey]
}

// portalSubtypeColorKey maps a portal subtype string to its palette colour key.
func portalSubtypeColorKey(subtype string) string {
	switch subtype {
	case "inter-portal":
		return "PORTAL_INTER_PORTAL"
	case "inter-random":
		return "PORTAL_INTER_RANDOM"
	case "intra-random":
		return "PORTAL_INTRA_RANDOM"
	case "intra-portal":
		return "PORTAL_INTRA_PORTAL"
	default:
		return "PORTAL"
	}
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

// printInstanceGraph prints an ASCII representation of the instance's map/portal graph.
func printInstanceGraph(instance *pb.InstanceMessage, maps map[string]*MapState) {
	mapCodes := instance.GetMapCodes()
	edges := instance.GetPortals()

	if len(mapCodes) == 0 {
		log.Println("[InstanceGraph] (empty graph — no maps)")
		return
	}

	// Collect adjacency: source → []target (with cell coords and mode)
	type edgeLabel struct {
		target string
		srcX   int32
		srcY   int32
		dstX   int32
		dstY   int32
		mode   string
	}
	adj := make(map[string][]edgeLabel)
	for _, e := range edges {
		adj[e.GetSourceMapCode()] = append(adj[e.GetSourceMapCode()], edgeLabel{
			target: e.GetTargetMapCode(),
			srcX:   e.GetSourceCellX(),
			srcY:   e.GetSourceCellY(),
			dstX:   e.GetTargetCellX(),
			dstY:   e.GetTargetCellY(),
			mode:   e.GetPortalMode(),
		})
	}

	// Build all content lines first to measure widths
	var content []string

	title := fmt.Sprintf("  Instance Graph: %s  ", instance.GetCode())
	summary := fmt.Sprintf("  Maps: %d   Edges: %d", len(mapCodes), len(edges))
	content = append(content, title)
	content = append(content, summary)
	content = append(content, "") // separator
	content = append(content, "  Nodes (maps):")

	for _, code := range mapCodes {
		line := fmt.Sprintf("    [%s]", code)
		if ms, ok := maps[code]; ok {
			line += fmt.Sprintf("  [F:%d O:%d B:%d P:%d]",
				len(ms.floors), len(ms.obstacles), len(ms.bots), len(ms.portals))
		}
		content = append(content, line)
	}

	content = append(content, "")
	content = append(content, "  Edges (portals):")

	if len(edges) == 0 {
		content = append(content, "    (no portal edges)")
	} else {
		for _, code := range mapCodes {
			for _, t := range adj[code] {
				var dstPos string
				if t.mode == "inter-random" || t.mode == "intra-random" {
					dstPos = "(random)"
				} else {
					dstPos = fmt.Sprintf("(%d,%d)", t.dstX, t.dstY)
				}
				content = append(content, fmt.Sprintf("    [%s] (%d,%d) ──▶ [%s] %s {%s}",
					code, t.srcX, t.srcY, t.target, dstPos, t.mode))
			}
		}
	}

	// Find max content width
	maxW := 0
	for _, l := range content {
		if len([]rune(l)) > maxW {
			maxW = len([]rune(l))
		}
	}
	// Account for the arrow character width (3-byte rune counted as 1)
	boxW := maxW + 2 // padding inside box

	pad := func(s string, width int) string {
		runeLen := len([]rune(s))
		if runeLen >= width {
			return s
		}
		p := make([]byte, width-runeLen)
		for i := range p {
			p[i] = ' '
		}
		return s + string(p)
	}

	hLine := ""
	for i := 0; i < boxW; i++ {
		hLine += "═"
	}

	log.Printf("[InstanceGraph]")
	log.Printf("[InstanceGraph] ╔%s╗", hLine)

	for i, l := range content {
		if i == 2 {
			log.Printf("[InstanceGraph] ╠%s╣", hLine)
		} else {
			log.Printf("[InstanceGraph] ║%s║", pad(l, boxW))
		}
	}

	log.Printf("[InstanceGraph] ╚%s╝", hLine)
	log.Printf("[InstanceGraph]")
}
