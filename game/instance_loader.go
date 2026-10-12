// Package game — instance_loader.go
//
// Builds the Go game server's in-memory world from gRPC data.
// Replaces the old hardcoded createMaps() approach: the Go server
// no longer creates its own maps, entities, or portals.
// Everything is fetched from the Engine-Cyberia via GetFullInstance.
package game

import (
	"cyberia-server/logx"
	"fmt"
	"log"
	"math"
	"math/rand"
	"time"

	pb "cyberia-server/gen/proto"
)

// portalEntry tracks a portal entity for cross-map linking.
type portalEntry struct {
	portal  EntityID
	cellX   int
	cellY   int
	mapCode string
}

// BuildWorldFromInstance reconstructs the entire game world from gRPC
// GetFullInstance response data. This is called by WorldBuilder.
// playerSpawnFromInstance decodes the authoritative initial spawn from the
// instance message, defaulting to a zero (random-fallback) config when unset.
func playerSpawnFromInstance(instance *pb.InstanceMessage) PlayerSpawnConfig {
	ps := instance.GetPlayerSpawn()
	if ps == nil {
		return PlayerSpawnConfig{}
	}
	return PlayerSpawnConfig{
		MapCode: ps.GetSourceMapCode(),
		CellX:   int(ps.GetSourceCellX()),
		CellY:   int(ps.GetSourceCellY()),
		Random:  ps.GetRandom(),
	}
}

func (s *GameServer) BuildWorldFromInstance(
	instance *pb.InstanceMessage,
	mapMsgs []*pb.MapDataMessage,
	olMsgs []*pb.ObjectLayerMessage,
	actions []*pb.CyberiaActionMessage,
	quests []*pb.CyberiaQuestMessage,
) {
	s.mu.Lock()
	defer s.mu.Unlock()

	logx.Infof("[InstanceLoader] Building world for instance %q (%d maps, %d object layers)",
		instance.GetCode(), len(mapMsgs), len(olMsgs))

	s.buildMapsFromInstance(instance, mapMsgs)
	s.playerSpawn = playerSpawnFromInstance(instance)

	// Bind the CyberiaAction / CyberiaQuest content the engine delivered with
	// the world to the freshly built entities. No separate fetch.
	s.bindQuests(instance.GetMapCodes(), quests)
	s.bindActions(instance.GetMapCodes(), actions)

	logx.Infof("[InstanceLoader] World built: %d maps loaded.", len(s.maps))

	// Log entity counts per map
	for code, ms := range s.maps {
		logx.Debugf("[InstanceLoader]   Map %q: %d floors, %d obstacles, %d foregrounds, %d portals, %d bots",
			code, ms.sceneryCount("floor"), ms.sceneryCount("obstacle"), ms.sceneryCount("foreground"), ms.count(EntityPortal), len(ms.brains))
	}

	// Print ASCII graph of portal topology
	printInstanceGraph(instance, s.maps)

	// List the entities bound to a CyberiaAction and the quests each cell offers.
	s.logActionProviders()
	s.logQuestProviders()
}

// SetInstanceCode records the INSTANCE_CODE the world was loaded for. When it is
// "TEST", new players always spawn on the first declared map (deterministic for
// testing) instead of a random one.
func (s *GameServer) SetInstanceCode(code string) {
	s.mu.Lock()
	s.instanceCode = code
	s.mu.Unlock()
}

// RebuildWorld re-fetches instance data and rebuilds all map entities
// (floors, obstacles, foregrounds, portals, bots) while preserving
// connected players.  Called by the hot-reload loop so that map edits
// made in the admin UI are reflected without a full server restart.
func (s *GameServer) RebuildWorld(
	instance *pb.InstanceMessage,
	mapMsgs []*pb.MapDataMessage,
	olMsgs []*pb.ObjectLayerMessage,
	actions []*pb.CyberiaActionMessage,
	quests []*pb.CyberiaQuestMessage,
) {
	s.mu.Lock()
	defer s.mu.Unlock()

	logx.Infof("[InstanceLoader] RebuildWorld for instance %q (%d maps)", instance.GetCode(), len(mapMsgs))

	// Snapshot existing players per map so they survive the rebuild.
	type savedPlayer struct {
		id  EntityID
		row entityRow
	}
	savedPlayers := make(map[string][]savedPlayer)
	for code, ms := range s.maps {
		ms.mu.RLock()
		for id := range ms.ofKind(EntityPlayer) {
			savedPlayers[code] = append(savedPlayers[code], savedPlayer{id, ms.take(id)})
		}
		ms.mu.RUnlock()
	}

	// Rebuild via the same path as initial load.
	s.buildMapsFromInstance(instance, mapMsgs)
	s.playerSpawn = playerSpawnFromInstance(instance)
	s.bindQuests(instance.GetMapCodes(), quests)
	s.bindActions(instance.GetMapCodes(), actions)

	// Restore players into the (possibly new) map states.
	for code, players := range savedPlayers {
		ms, ok := s.maps[code]
		if !ok {
			// Map was removed — players will be orphaned and eventually
			// time out or reconnect.  Log a warning.
			logx.Warnf("[InstanceLoader] map %q was removed during rebuild; %d players orphaned", code, len(players))
			continue
		}
		ms.mu.Lock()
		for _, p := range players {
			ms.put(p.id, p.row)
		}
		ms.mu.Unlock()
	}

	logx.Infof("[InstanceLoader] RebuildWorld complete: %d maps.", len(s.maps))
	for code, ms := range s.maps {
		logx.Debugf("[InstanceLoader]   Map %q: %d floors, %d obstacles, %d foregrounds, %d portals, %d bots, %d resources, %d statics, %d players",
			code, ms.sceneryCount("floor"), ms.sceneryCount("obstacle"), ms.sceneryCount("foreground"), ms.count(EntityPortal), len(ms.brains), ms.count(EntityResource), ms.sceneryCount("static"), ms.count(EntityPlayer))
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
			logx.Warnf("[InstanceLoader] map code %q in instance but not in gRPC response", mapCode)
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
			code:       mapCode,
			gridW:      gridW,
			gridH:      gridH,
			pathfinder: NewPathfinder(gridW, gridH),
		}
		// Register the map first: a builder reads its components through s.maps.
		s.maps[mapCode] = ms

		for _, ent := range mapMsg.GetEntities() {
			switch ent.GetEntityType() {
			case "floor":
				s.buildFloor(ms, ent)
			case "bot":
				s.buildBot(ms, ent)
			case "resource":
				s.buildResource(ms, ent)
			case "static":
				s.buildStatic(ms, ent)
			case "obstacle":
				s.buildObstacle(ms, ent)
			case "foreground":
				s.buildForeground(ms, ent)
			case "loot":
				s.buildLoot(ms, ent)
			case "portal":
				allPortals[mapCode] = append(allPortals[mapCode], portalEntry{
					portal:  s.buildPortal(ms, ent),
					cellX:   int(ent.GetInitCellX()),
					cellY:   int(ent.GetInitCellY()),
					mapCode: mapCode,
				})
			default:
				logx.Warnf("[InstanceLoader] Unknown entity type %q in map %q", ent.GetEntityType(), mapCode)
			}
		}
	}

	// Link portals
	for _, edge := range instance.GetPortals() {
		srcMapCode := edge.GetSourceMapCode()
		dstMapCode := edge.GetTargetMapCode()

		_, srcOk := s.maps[srcMapCode]
		_, dstOk := s.maps[dstMapCode]
		if !srcOk || !dstOk {
			logx.Warnf("[InstanceLoader] portal edge references unknown map: %q → %q", srcMapCode, dstMapCode)
			continue
		}

		srcPortal := findPortalByCell(allPortals[srcMapCode], int(edge.GetSourceCellX()), int(edge.GetSourceCellY()))

		if srcPortal == 0 {
			logx.Warnf("[InstanceLoader] no portal entity at (%d,%d) on map %q",
				edge.GetSourceCellX(), edge.GetSourceCellY(), srcMapCode)
			continue
		}

		portalMode := edge.GetPortalMode()
		if portalMode == "" {
			portalMode = "inter-portal"
		}

		// A random mode spawns anywhere on the target map: a negative target
		// cell says so to the teleport and to the client label.
		targetCell := Cell{X: int(edge.GetTargetCellX()), Y: int(edge.GetTargetCellY())}
		if portalMode == "inter-random" || portalMode == "intra-random" {
			targetCell = Cell{X: -1, Y: -1}
		}
		s.maps[srcMapCode].addPortal(srcPortal, Portal{TargetMap: dstMapCode, TargetCell: targetCell})
	}
}

// findPortalByCell finds a portal entry by cell coordinates.
func findPortalByCell(entries []portalEntry, cellX, cellY int) EntityID {
	for _, e := range entries {
		if e.cellX == cellX && e.cellY == cellY {
			return e.portal
		}
	}
	return 0
}

// ── Entity builders ──────────────────────────────────────────────

// entSize is the size the map editor gave the entity.
func entSize(ent *pb.EntityMessage) Vec2 {
	return Vec2{X: float64(ent.GetDimX()), Y: float64(ent.GetDimY())}
}

// entTransform places an entity of `size` on its initial cell.
func entTransform(ent *pb.EntityMessage, size Vec2) Transform {
	return Transform{Pos: Vec2{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())}, Size: size}
}

func (s *GameServer) buildFloor(ms *MapState, ent *pb.EntityMessage) {
	var layers []ObjectLayerState
	for _, itemID := range ent.GetObjectLayerItemIds() {
		layers = append(layers, ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
	}
	if len(layers) == 0 && ent.GetColorA() == 0 {
		// Apply instance-level default OLs only when the entity has no
		// explicit DB colour.  A non-zero ColorA means the map creator
		// intentionally placed a solid-colour floor without sprites.
		if d, ok := s.entityDefaults["floor"]; ok && len(d.LiveItemIDs) > 0 {
			for _, itemID := range d.LiveItemIDs {
				layers = append(layers, ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
			}
		}
	}
	ms.spawn(s.newEntityID(), EntityData{Kind: EntityScenery, DrawOrder: DrawFloor},
		entTransform(ent, entSize(ent)), Appearance{Layers: layers})
}

func (s *GameServer) buildBot(ms *MapState, ent *pb.EntityMessage) {
	dims := Vec2{X: float64(ent.GetDimX()), Y: float64(ent.GetDimY())}
	if dims.X <= 0 {
		dims.X = float64(rand.Intn(4) + 1)
	}
	if dims.Y <= 0 {
		dims.Y = float64(rand.Intn(4) + 1)
	}

	startPos := Vec2{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())}

	// Determine max life
	maxLife := s.placedMaxLife(ent)

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

	// A non-zero ColorA means the map creator intentionally placed a solid-colour bot without
	// sprites, so it carries no default layers.
	objectLayers := s.spawnObjectLayers("bot", ent.GetObjectLayerItemIds(), ent.GetColorA() == 0)

	id := s.newEntityID()
	bot := &Brain{
		Path:        []Cell{},
		TargetPos:   Cell{-1, -1},
		SpawnCenter: startPos,
		SpawnRadius: spawnRadius,
		AggroRange:  aggroRange,
	}
	ms.addBrain(id, bot)
	ms.spawn(id, EntityData{Kind: EntityBot}, Transform{Pos: startPos, Size: dims}, Appearance{Layers: objectLayers})
	ms.addVitals(id, Vitals{MaxLife: maxLife, Life: maxLife * s.initialLifeFraction})
	ms.addMotion(id, Motion{Direction: NONE, Mode: IDLE})
	ms.addAgent(id, Agent{Behavior: s.resolveEntityBehavior("bot", objectLayers)})
	ms.addMortal(id, &Mortal{
		BaseMaxLife:     maxLife,
		Progression:     NewEntityProgression(int(ent.GetLevel()), true, s.progressionConfig()),
		LifeRegen:       lifeRegen,
		RespawnDuration: placedRespawnDuration(ent),
	})
	s.ApplyResistanceStat(id, ms)
	ms.vt(id).Life = ms.vt(id).MaxLife * s.initialLifeFraction

	// Fountain: credit the bot's starting coin supply (same path as respawn).
	s.FountainInitBot(id, ms)

	// Initial wandering path — skipped for providers, which stay at their spawn
	// (provider-static never moves; provider only takes sporadic short steps).
	if !behaviorIsProvider(ms.ag(id).Behavior) {
		target := s.randomPointWithinRadius(ms, bot.SpawnCenter, bot.SpawnRadius, dims)
		if target.X >= 0 {
			if pth, err := ms.pathfinder.Astar(
				Cell{X: int(math.Round(startPos.X)), Y: int(math.Round(startPos.Y))},
				target, dims,
			); err == nil && len(pth) > 0 {
				bot.Path = pth
				bot.TargetPos = target
				ms.mo(id).Mode = WALKING
			}
		}
	}
}

func (s *GameServer) buildResource(ms *MapState, ent *pb.EntityMessage) {
	dims := Vec2{X: float64(ent.GetDimX()), Y: float64(ent.GetDimY())}
	if dims.X <= 0 {
		dims.X = 2
	}
	if dims.Y <= 0 {
		dims.Y = 2
	}

	startPos := Vec2{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())}

	maxLife := s.placedMaxLife(ent)

	objectLayers := s.spawnObjectLayers("resource", ent.GetObjectLayerItemIds(), ent.GetColorA() == 0)

	res := s.newEntityID()
	ms.spawn(res, EntityData{Kind: EntityResource}, Transform{Pos: startPos, Size: dims}, Appearance{Layers: objectLayers})
	ms.addVitals(res, Vitals{MaxLife: maxLife, Life: maxLife})
	ms.addMortal(res, &Mortal{
		BaseMaxLife:     maxLife,
		Progression:     NewEntityProgression(int(ent.GetLevel()), true, s.progressionConfig()),
		RespawnDuration: placedRespawnDuration(ent),
	})

	// Apply stats (resistance may increase MaxLife)
	s.ApplyResistanceStat(res, ms)
	ms.vt(res).Life = ms.vt(res).MaxLife
}

// placedMaxLife is the max life a map entity names, or the instance base when it
// names none.
func (s *GameServer) placedMaxLife(ent *pb.EntityMessage) float64 {
	if ent.GetMaxLife() > 0 {
		return ent.GetMaxLife()
	}
	return s.entityBaseMaxLife
}

// placedRespawnDuration is the respawn time a map entity names; 0 takes the default.
func placedRespawnDuration(ent *pb.EntityMessage) time.Duration {
	return time.Duration(ent.GetRespawnDurationMs()) * time.Millisecond
}

// buildObstacle places an obstacle and blocks its cells. The destructible
// behavior binds only on a live-item match: an obstacle that falls back to the
// first default of its type stays ordinary.
func (s *GameServer) buildObstacle(ms *MapState, ent *pb.EntityMessage) {
	dims := Vec2{X: float64(ent.GetDimX()), Y: float64(ent.GetDimY())}
	pos := Vec2{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())}
	objectLayers := s.spawnObjectLayers("obstacle", ent.GetObjectLayerItemIds(), ent.GetColorA() == 0)
	id := s.newEntityID()
	ms.pathfinder.occupy(pos, dims, 1)
	ms.spawn(id, EntityData{Kind: EntityScenery, Flags: FlagBlocks},
		Transform{Pos: pos, Size: dims}, Appearance{Layers: objectLayers})

	activeIDs := activeObjectLayerItemIDs(objectLayers)
	build, _ := s.resolveEntityDefaultBuild("obstacle", activeIDs)
	if build.Behavior != BehaviorDestructible || !matchesLiveItems(build, activeIDs) {
		return
	}
	maxLife := s.placedMaxLife(ent)
	ms.addVitals(id, Vitals{Life: maxLife, MaxLife: maxLife})
	ms.addMortal(id, &Mortal{BaseMaxLife: maxLife, RespawnDuration: placedRespawnDuration(ent)})
}

// entAppearance gives an entity of `kind` its layers. One that carries neither
// object layers nor a colour falls back to the kind's default live items.
func (s *GameServer) entAppearance(ent *pb.EntityMessage, kind string) Appearance {
	return Appearance{Layers: s.spawnObjectLayers(kind, ent.GetObjectLayerItemIds(), ent.GetColorA() == 0)}
}

func (s *GameServer) buildForeground(ms *MapState, ent *pb.EntityMessage) {
	ap := s.entAppearance(ent, "foreground")
	data := EntityData{Kind: EntityScenery, DrawOrder: DrawForeground}
	if s.boundBehavior("foreground", ap.Layers) == BehaviorOverheadOcclusion {
		data.Flags |= FlagOverheadOcclusion
	}
	ms.spawn(s.newEntityID(), data, entTransform(ent, entSize(ent)), ap)
}

func (s *GameServer) buildStatic(ms *MapState, ent *pb.EntityMessage) {
	// Statics are passable: no pathfinder cells are blocked.
	ms.spawn(s.newEntityID(), EntityData{Kind: EntityScenery},
		entTransform(ent, entSize(ent)), s.entAppearance(ent, "static"))
}

func (s *GameServer) buildPortal(ms *MapState, ent *pb.EntityMessage) EntityID {
	dims := Vec2{X: float64(ent.GetDimX()), Y: float64(ent.GetDimY())}
	if dims.X <= 0 {
		dims.X = 3
	}
	if dims.Y <= 0 {
		dims.Y = 3
	}

	objectLayers := s.spawnObjectLayers("portal", ent.GetObjectLayerItemIds(), ent.GetColorA() == 0)

	portal := s.newEntityID()
	ms.spawn(portal, EntityData{Kind: EntityPortal}, entTransform(ent, dims), Appearance{Layers: objectLayers})

	// Portals are walkable — no grid cells are blocked.
	// The player must stand on the portal for portalHoldTime to trigger transport.

	return portal
}

// printInstanceGraph prints an ASCII representation of the instance's map/portal graph.
func printInstanceGraph(instance *pb.InstanceMessage, maps map[string]*MapState) {
	mapCodes := instance.GetMapCodes()
	edges := instance.GetPortals()

	if len(mapCodes) == 0 {
		logx.Debugf("[InstanceGraph] (empty graph — no maps)")
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
				ms.sceneryCount("floor"), ms.sceneryCount("obstacle"), len(ms.brains), ms.count(EntityPortal))
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
