package game

import (
	"fmt"
	"math"
	"math/rand"
	"time"

	pb "cyberia-server/gen/proto"
	"cyberia-server/logx"
)

// NewGameServer creates an empty game server with no hardcoded defaults.
// All configuration must be applied via ApplyInstanceConfig from gRPC data.
func NewGameServer() *GameServer {
	gs := &GameServer{
		maps:    make(map[string]*MapState),
		clients: make(map[string]*Client),
		// Buffer register/unregister so HandleConnections never blocks the
		// HTTP handler goroutine waiting for listenForClients to win the
		// world lock against gameLoop. 64 outstanding connect/disconnect
		// events is well beyond any realistic burst.
		register:             make(chan *Client, 64),
		unregister:           make(chan *Client, 64),
		objectLayerDataCache: make(map[string]*ObjectLayer),
		skillConfig:          make(map[string][]SkillDefinition),
	}
	gs.SetConnectionLimits(DefaultConnectionLimits())
	return gs
}

// SetConnectionLimits replaces the admission and rate thresholds. Call it
// before the server accepts connections.
func (s *GameServer) SetConnectionLimits(limits ConnectionLimits) {
	s.limits = limits
	s.guard = newConnectionGuard(limits)
}

// ApplyInstanceConfig applies the gRPC InstanceConfig to the game server.
// This replaces all hardcoded defaults — called during LoadAll and hot-reload.
func (s *GameServer) ApplyInstanceConfig(cfg *pb.InstanceConfig) error {
	if cfg == nil {
		return fmt.Errorf("instance config is required")
	}

	rules := progressionRulesFromProto(cfg.GetProgressionRules())
	if err := rules.Validate(); err != nil {
		return err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	s.progressionRules = rules

	// ── Simulation cadence ─────────────────────────────────────────────────
	// Presentation defaults (cell-pixel size, object default dims, camera,
	// palette, interpolation) are deliberately absent — they are not part
	// of the simulation contract. The cyberia-client fetches them from
	// /api/cyberia-client-hints using its own CYBERIA_CLIENT_HINTS_CODE.
	s.tickRate = int(cfg.GetTickRate())
	if s.tickRate <= 0 {
		s.tickRate = DefaultTickRate
	}
	s.snapshotRate = int(cfg.GetSnapshotRate())
	if s.snapshotRate <= 0 {
		s.snapshotRate = DefaultSnapshotRate
	}
	if s.snapshotRate > s.tickRate {
		s.snapshotRate = s.tickRate
	}
	s.tickDuration = computeTickDuration(s.tickRate)

	// World / AOI
	s.aoiRadius = cfg.GetAoiRadius()
	s.portalHoldTime = time.Duration(cfg.GetPortalHoldTimeMs()) * time.Millisecond
	s.portalSpawnRadius = cfg.GetPortalSpawnRadius()

	// Entity base stats
	s.entityBaseSpeed = cfg.GetEntityBaseSpeed()
	s.playerBaseSpeed = cfg.GetPlayerBaseSpeed()
	s.entityBaseMaxLife = cfg.GetEntityBaseMaxLife()
	s.entityBaseActionCooldown = time.Duration(cfg.GetEntityBaseActionCooldownMs()) * time.Millisecond
	s.entityBaseMinActionCooldown = time.Duration(cfg.GetEntityBaseMinActionCooldownMs()) * time.Millisecond

	// Bot defaults
	s.botAggroRange = cfg.GetBotAggroRange()

	// Player defaults
	s.defaultPlayerWidth = cfg.GetDefaultPlayerWidth()
	s.defaultPlayerHeight = cfg.GetDefaultPlayerHeight()
	s.playerBaseLifeRegenMin = cfg.GetPlayerBaseLifeRegenMin()
	s.playerBaseLifeRegenMax = cfg.GetPlayerBaseLifeRegenMax()
	s.maxActiveLayers = int(cfg.GetMaxActiveLayers())
	s.initialLifeFraction = cfg.GetInitialLifeFraction()

	// Combat / death
	s.respawnDuration = time.Duration(cfg.GetRespawnDurationMs()) * time.Millisecond
	s.collisionLifeLoss = cfg.GetCollisionLifeLoss()

	// Economy — Fountain & Sink model (nested EconomyRules proto message).
	er := cfg.GetEconomyRules()
	if er == nil {
		er = &pb.EconomyRules{}
	}
	s.botSpawnCoins = int(er.GetBotSpawnCoins())
	s.playerSpawnCoins = int(er.GetPlayerSpawnCoins())
	s.coinKillPercentVsBot = er.GetCoinKillPercentVsBot()
	s.coinKillPercentVsPlayer = er.GetCoinKillPercentVsPlayer()
	if s.coinKillPercentVsPlayer == 0 {
		// Default PvP to half the PvE rate if not explicitly set.
		s.coinKillPercentVsPlayer = s.coinKillPercentVsBot * 0.5
	}
	s.coinKillMinAmount = int(er.GetCoinKillMinAmount())
	s.respawnCostPercent = er.GetRespawnCostPercent()
	s.portalFee = int(er.GetPortalFee())
	s.craftingFeePercent = er.GetCraftingFeePercent()
	s.lifeRegenChance = cfg.GetLifeRegenChance()
	s.maxChance = cfg.GetMaxChance()

	// Skill params (nested under SkillRules in proto)
	sr := cfg.GetSkillRules()
	if sr == nil {
		sr = &pb.SkillRules{}
	}
	s.projectileSpawnChance = sr.GetProjectileSpawnChance()
	s.projectileLifetimeMs = int(sr.GetProjectileLifetimeMs())
	s.projectileWidth = sr.GetProjectileWidth()
	s.projectileHeight = sr.GetProjectileHeight()
	s.projectileSpeedMultiplier = sr.GetProjectileSpeedMultiplier()
	s.doppelgangerSpawnChance = sr.GetDoppelgangerSpawnChance()
	s.doppelgangerLifetimeMs = int(sr.GetDoppelgangerLifetimeMs())
	s.doppelgangerSpawnRadius = sr.GetDoppelgangerSpawnRadius()
	s.doppelgangerInitialLifeFraction = sr.GetDoppelgangerInitialLifeFraction()

	// Equipment rules — governs which item types can be active.
	eqr := cfg.GetEquipmentRules()
	if eqr != nil {
		s.equipmentRules.ActiveItemTypes = make(map[string]bool, len(eqr.GetActiveItemTypes()))
		for _, t := range eqr.GetActiveItemTypes() {
			s.equipmentRules.ActiveItemTypes[t] = true
		}
		s.equipmentRules.OnePerType = eqr.GetOnePerType()
		s.equipmentRules.RequireSkin = eqr.GetRequireSkin()
	} else {
		// Sensible defaults when no equipment rules are provided.
		s.equipmentRules.ActiveItemTypes = map[string]bool{
			"skin": true, "breastplate": true, "weapon": true, "skill": true,
		}
		s.equipmentRules.OnePerType = true
		s.equipmentRules.RequireSkin = true
	}

	// Per-entity-type visual defaults — keep both the ordered build list and
	// the last-seen per-type lookup for unique entity types.
	s.entityDefaults = make(map[string]EntityTypeDefaultConfig, len(cfg.GetEntityDefaults()))
	s.entityDefaultBuilds = make([]EntityTypeDefaultConfig, 0, len(cfg.GetEntityDefaults()))
	for _, etd := range cfg.GetEntityDefaults() {
		var dols []ObjectLayerState
		dropChances := make(map[string]float64)
		for _, dol := range etd.GetDefaultObjectLayers() {
			dols = append(dols, ObjectLayerState{
				ItemID:   dol.GetItemId(),
				Active:   dol.GetActive(),
				Quantity: int(dol.GetQuantity()),
			})
			// Only a row that states a chance is recorded. The field is optional precisely so a
			// payload from before it existed reads as absent rather than as a deliberate 0, and
			// dropChance answers 1 for anything unrecorded.
			if dol.DropChance != nil {
				dropChances[dol.GetItemId()] = dol.GetDropChance()
			}
		}
		defaultBuild := EntityTypeDefaultConfig{
			EntityType:          etd.GetEntityType(),
			LiveItemIDs:         etd.GetLiveItemIds(),
			DeadItemIDs:         etd.GetDeadItemIds(),
			DropItemIDs:         etd.GetDropItemIds(),
			DefaultObjectLayers: dols,
			DropChances:         dropChances,
			Behavior:            etd.GetBehavior(),
		}
		s.entityDefaultBuilds = append(s.entityDefaultBuilds, defaultBuild)
		s.entityDefaults[defaultBuild.EntityType] = defaultBuild
	}
	if d, ok := s.entityDefaults["player"]; ok && len(d.DeadItemIDs) > 0 {
		s.ghostItemID = d.DeadItemIDs[0]
	} else {
		s.ghostItemID = ""
	}
	s.deadItemIDs = make(map[string]bool)
	for _, build := range s.entityDefaultBuilds {
		for _, id := range build.DeadItemIDs {
			if id != "" {
				s.deadItemIDs[id] = true
			}
		}
	}
	if d, ok := s.entityDefaults["coin"]; ok && len(d.LiveItemIDs) > 0 {
		s.coinItemID = d.LiveItemIDs[0]
	}
	if d, ok := s.entityDefaults["floor"]; ok && len(d.LiveItemIDs) > 0 {
		s.defaultFloorItemID = d.LiveItemIDs[0]
	}

	// Status-icon visuals (iconId + borderColor) are not on the server.
	// The simulation writes the numeric status_icon u8 into the AOI encoder;
	// visual resolution happens on the client via domain/presentation_defaults
	// and the optional /api/cyberia-client-hints override.

	// Skill map
	s.skillConfig = make(map[string][]SkillDefinition, len(cfg.GetSkillConfig()))
	for _, sc := range cfg.GetSkillConfig() {
		triggerID := sc.GetTriggerItemId()
		for _, sk := range sc.GetSkills() {
			s.skillConfig[triggerID] = append(s.skillConfig[triggerID], SkillDefinition{
				LogicEventID:         sk.GetLogicEventId(),
				Name:                 sk.GetName(),
				Description:          sk.GetDescription(),
				SummonedEntityItemID: sk.GetSummonedEntityItemId(),
			})
		}
	}

	// Register built-in skill handlers now that skillConfig is populated.
	s.InitSkills()

	logx.Infof("[GameServer] Instance config applied: tickRate=%dHz, snapshotRate=%dHz, tickDuration=%v, aoiRadius=%.1f, entityBaseSpeed=%.1f, entityBaseMaxLife=%.1f, %d skills, %d entityDefaultTypes, %d entityDefaultBuilds, floorItem=%q, ghostItem=%q, coinItem=%q",
		s.tickRate, s.snapshotRate, s.tickDuration, s.aoiRadius, s.entityBaseSpeed, s.entityBaseMaxLife, len(s.skillConfig), len(s.entityDefaults), len(s.entityDefaultBuilds), s.defaultFloorItemID, s.ghostItemID, s.coinItemID)
	return nil
}

// ReplaceObjectLayerCache atomically replaces the entire cache.
// Used by WorldBuilder for initial full load via gRPC.
func (s *GameServer) ReplaceObjectLayerCache(cache map[string]*ObjectLayer) error {
	if err := validateObjectLayers(cache); err != nil {
		return err
	}
	s.olMu.Lock()
	defer s.olMu.Unlock()
	s.objectLayerDataCache = cache

	typeCounts := make(map[string]int)
	for _, layer := range cache {
		itemType := layer.Data.Item.Type
		if itemType == "" {
			itemType = "(unknown)"
		}
		typeCounts[itemType]++
	}
	logx.Infof("Object layer cache replaced with %d items.", len(cache))
	for itemType, count := range typeCounts {
		logx.Debugf("  %-20s %d", itemType, count)
	}
	return nil
}

// PatchObjectLayerCache applies incremental updates and deletions.
// Used by WorldBuilder for hot-reload via gRPC manifest diffing.
func (s *GameServer) PatchObjectLayerCache(updates map[string]*ObjectLayer, deletions []string) error {
	if err := validateObjectLayers(updates); err != nil {
		return err
	}
	s.olMu.Lock()
	defer s.olMu.Unlock()
	if s.objectLayerDataCache == nil {
		s.objectLayerDataCache = make(map[string]*ObjectLayer)
	}
	for itemID, ol := range updates {
		s.objectLayerDataCache[itemID] = ol
	}
	for _, itemID := range deletions {
		delete(s.objectLayerDataCache, itemID)
	}
	return nil
}

// GetObjectLayerData returns an ObjectLayer by item ID (read-locked).
func (s *GameServer) GetObjectLayerData(itemID string) (*ObjectLayer, bool) {
	s.olMu.RLock()
	defer s.olMu.RUnlock()
	ol, ok := s.objectLayerDataCache[itemID]
	return ol, ok
}

// SetDataServerURL sets the Data Server origin used for server-to-server
// content-authority calls. Never forwarded to clients.
func (s *GameServer) SetDataServerURL(url string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.dataServerURL = url
	logx.Infof("Data Server URL set to: %s", url)
}

func (s *GameServer) Run() {
	go s.listenForClients()
	go s.gameLoop()
}

func (s *GameServer) listenForClients() {
	defer func() {
		if r := recover(); r != nil {
			logx.Errorf("[listenForClients] PANIC: %v — restarting goroutine", r)
			go s.listenForClients()
		}
	}()
	logx.Infof("Starting client listener...")
	for {
		select {
		case client := <-s.register:
			s.mu.Lock()
			// A disconnect already handled for this client wins: re-adding it
			// here would leave an entry nothing will ever remove.
			if !client.detached {
				s.clients[client.playerID] = client
			}
			s.mu.Unlock()
			logx.Debugf("[listenForClients] registered player=%s", client.playerID)
		case client := <-s.unregister:
			s.detachClient(client)
			logx.Debugf("[listenForClients] unregistered player=%s", client.playerID)
		}
	}
}

// gameLoop runs two independent tickers:
//
//   - simTicker  @ tickRate  Hz  — advances the authoritative world by exactly
//     one Tick. Calls the named simulation phases in a fixed order. The only
//     function permitted to mutate world state lives downstream of this path.
//
//   - snapTicker @ snapshotRate Hz — produces AOI snapshots (replication).
//     Decoupled from sim cadence so bandwidth scales independently of
//     simulation fidelity.
//
// Update ordering inside one simulation tick (phase functions live in
// simulation_phases.go):
//
//	phaseInput      drain queued InputCommand per player
//	phaseLifecycle  respawn timers, despawn expirations
//	phaseSkills     skill projectile collisions
//	phaseAI         bot behaviour decisions
//	phaseMovement   integrate positions using s.tickDuration (NOT 1/fps)
//	phaseLoot       player↔drop-token collection + priority validation
//	phasePortals    portal entry / teleport
//
// Neither phase reads PresentationHints. Replication runs on its own
// goroutine ticker — it does not block the simulation.
func (s *GameServer) gameLoop() {
	if s.tickRate <= 0 {
		s.tickRate = DefaultTickRate
	}
	if s.snapshotRate <= 0 || s.snapshotRate > s.tickRate {
		s.snapshotRate = s.tickRate
	}
	s.tickDuration = computeTickDuration(s.tickRate)

	simTicker := time.NewTicker(s.tickDuration)
	defer simTicker.Stop()
	snapTicker := time.NewTicker(time.Second / time.Duration(s.snapshotRate))
	defer snapTicker.Stop()

	logx.Infof("[GameServer] simulation @ %d Hz (tick %v), replication @ %d Hz",
		s.tickRate, s.tickDuration, s.snapshotRate)

	for {
		select {
		case <-simTicker.C:
			func() {
				defer func() {
					if r := recover(); r != nil {
						logx.Errorf("[GameServer] PANIC in simulation tick: %v", r)
					}
				}()
				s.mu.Lock()
				defer s.mu.Unlock()
				tick := s.currentTick
				for _, mapState := range s.maps {
					s.phaseInput(tick, mapState)
					s.phaseLifecycle(tick, mapState)
					s.phaseSkills(tick, mapState)
					s.phaseAI(tick, mapState)
					s.phaseMovement(tick, mapState)
					s.phaseLoot(tick, mapState)
					s.phasePortals(tick, mapState)
				}
				s.currentTick++
			}()
		case <-snapTicker.C:
			func() {
				defer func() {
					if r := recover(); r != nil {
						logx.Errorf("[GameServer] PANIC in snapshot tick: %v", r)
					}
				}()
				s.mu.Lock()
				defer s.mu.Unlock()
				tick := s.currentTick
				for _, mapState := range s.maps {
					s.phaseReplication(tick, mapState)
				}
			}()
		}
	}
}

// ---------- Movement and direction ----------

// directionFromVector maps a movement vector onto one of the eight facings.
func directionFromVector(dirX, dirY float64) Direction {
	angle := math.Atan2(dirY, dirX)
	if angle < 0 {
		angle += 2 * math.Pi
	}
	return Direction((int(math.Round(angle/(math.Pi/4))) + 2) % 8)
}

// stepAlongPath advances pos toward the head of path by speed*dt cells.
//
// dt-based integration: speed is cells/second and dt is the authoritative
// simulation step (1 / tickRate). A frame count instead of dt silently breaks
// movement whenever the loop runs slow or the tickRate config changes.
//
// It returns the new position, the path left to walk, and the facing for this
// step. The bool is false when the step produced no heading — an empty path
// or a zero-length hop — so the caller decides what an arriving entity faces.
func stepAlongPath(pos Point, path []PointI, speed, dt float64) (Point, []PointI, Direction, bool) {
	if len(path) == 0 {
		return pos, path, NONE, false
	}

	targetNode := path[0]
	dx := float64(targetNode.X) - pos.X
	dy := float64(targetNode.Y) - pos.Y
	dist := math.Sqrt(dx*dx + dy*dy)
	step := speed * dt // cells per simulation tick

	if dist >= step {
		dirX, dirY := dx/dist, dy/dist
		pos.X += dirX * step
		pos.Y += dirY * step
		return pos, path, directionFromVector(dirX, dirY), true
	}

	// Node reached. Face the next leg, if there is one.
	pos = Point{X: float64(targetNode.X), Y: float64(targetNode.Y)}
	path = path[1:]
	if len(path) == 0 {
		return pos, path, NONE, false
	}

	next := path[0]
	dirX := float64(next.X) - pos.X
	dirY := float64(next.Y) - pos.Y
	norm := math.Sqrt(dirX*dirX + dirY*dirY)
	if norm == 0 {
		return pos, path, NONE, false
	}
	return pos, path, directionFromVector(dirX/norm, dirY/norm), true
}

func (s *GameServer) updatePlayerPosition(player *PlayerState, mapState *MapState) {
	// Dead players can't move.
	if player.IsGhost() {
		return
	}

	playerStats := s.CalculateStats(player, mapState)
	speed := s.CalculatePlayerMovementSpeed(playerStats)

	if player.Mode != WALKING {
		return
	}

	pos, path, dir, facing := stepAlongPath(player.Pos, player.Path, speed, s.tickDuration.Seconds())
	player.Pos, player.Path = pos, path
	if facing {
		player.Direction = dir
	}
	if len(player.Path) == 0 {
		player.Mode = IDLE
		player.Direction = NONE
	}
}

// ---------- Portals ----------
func (s *GameServer) checkPortal(player *PlayerState, mapState *MapState) {
	// Teleport is only possible while alive. A dead player standing on a portal
	// neither accrues hold progress nor teleports; entering ghost state cancels
	// any in-progress hold.
	if player.IsGhost() {
		player.OnPortal = false
		player.ActivePortalID = ""
		return
	}

	onPortal := false
	var activePortal *PortalState
	for _, portal := range mapState.portals {
		portalRect := Rectangle{MinX: portal.Pos.X, MinY: portal.Pos.Y, MaxX: portal.Pos.X + portal.Dims.Width, MaxY: portal.Pos.Y + portal.Dims.Height}
		playerRect := Rectangle{MinX: player.Pos.X, MinY: player.Pos.Y, MaxX: player.Pos.X + player.Dims.Width, MaxY: player.Pos.Y + player.Dims.Height}
		if rectsOverlap(playerRect, portalRect) {
			onPortal = true
			activePortal = portal
			break
		}
	}
	if onPortal {
		if !player.OnPortal {
			player.OnPortal = true
			player.TimeOnPortal = time.Now()
			player.ActivePortalID = activePortal.ID
		} else if time.Since(player.TimeOnPortal) > s.portalHoldTime {
			s.teleportPlayer(player, activePortal)
		}
	} else {
		player.OnPortal = false
		player.ActivePortalID = ""
	}
}

func (s *GameServer) teleportPlayer(player *PlayerState, portal *PortalState) {
	// s.mu is held by the caller (phasePortals → simTicker).
	// Do NOT release it here — map mutations (delete/insert mapState.players)
	// must be serialised against snapTicker's phaseReplication reads.
	// Releasing s.mu and re-acquiring via defer caused a fatal
	// "concurrent map read and map write" crash: snapTicker could acquire s.mu
	// and iterate mapState.players while we were still writing to it.

	if portal.PortalConfig == nil {
		logx.Warnf("Teleportation failed: portal config is nil.")
		return
	}
	destMapCode := portal.PortalConfig.DestMapCode
	destMapState, ok := s.maps[destMapCode]
	if !ok {
		logx.Warnf("Teleportation failed: destination map %q not found.", destMapCode)
		return
	}

	// Determine spawn position based on portal mode
	var spawnX, spawnY float64
	mode := portal.PortalConfig.PortalMode
	if mode == "" {
		mode = "inter-portal"
	}

	switch mode {
	case "inter-portal", "intra-portal":
		// Spawn at the destination portal's position
		spawnX = portal.PortalConfig.DestCellX
		spawnY = portal.PortalConfig.DestCellY
	case "inter-random", "intra-random":
		// Spawn at a random position within the destination map
		spawnX = rand.Float64() * float64(destMapState.gridW)
		spawnY = rand.Float64() * float64(destMapState.gridH)
	default:
		spawnX = float64(destMapState.gridW) / 2.0
		spawnY = float64(destMapState.gridH) / 2.0
	}

	// Apply spawn radius jitter for portal-to-portal modes
	if mode == "inter-portal" || mode == "intra-portal" {
		spawnX += rand.Float64()*portal.PortalConfig.SpawnRadius*2 - portal.PortalConfig.SpawnRadius
		spawnY += rand.Float64()*portal.PortalConfig.SpawnRadius*2 - portal.PortalConfig.SpawnRadius
	}

	destPosI, err := destMapState.pathfinder.findClosestWalkablePoint(PointI{X: int(spawnX), Y: int(spawnY)}, player.Dims)
	if err != nil {
		logx.Warnf("Could not find a walkable spawn point: %v", err)
		return
	}
	currentMapState, ok := s.maps[player.MapCode]
	if ok {
		delete(currentMapState.players, player.ID)
	}
	player.MapCode = destMapCode
	player.Pos = Point{X: float64(destPosI.X), Y: float64(destPosI.Y)}
	player.TargetPos = PointI{}
	player.Path = []PointI{}
	player.Mode = TELEPORTING
	player.OnPortal = false
	destMapState.players[player.ID] = player
	s.sendAOI(player)
}

// ---------- AOI snapshot send ----------
//
// sendAOI is the one-shot snapshot dispatcher. Two call sites use it:
//
//   - phaseReplication (the snapshot-rate ticker) — the canonical path.
//   - teleportPlayer — pushes an immediate snapshot after a portal so the
//     client doesn't see a frame of stale geometry; semantically still part
//     of the replication contract, just out-of-band w.r.t. the tick.
//
// Outside these two call sites, simulation code MUST NOT send a snapshot —
// replication is the only path that may produce AOI frames.
func (s *GameServer) sendAOI(player *PlayerState) {
	mapState, ok := s.maps[player.MapCode]
	if !ok {
		logx.Warnf("Map %q not found for player %s.", player.MapCode, player.ID)
		return
	}

	sendMessage(player, "snapshot", s.buildSnapshot(player, mapState))
}

// GetConnectedClientsCount returns the current connection count.
func (s *GameServer) GetConnectedClientsCount() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	return len(s.clients)
}
