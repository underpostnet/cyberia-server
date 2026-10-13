package game

import (
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	pb "cyberia-server/gen/proto"
	"cyberia-server/logx"
	"cyberia-server/socket"
)

// NewGameServer creates an empty game server with no hardcoded defaults.
// All configuration must be applied via ApplyInstanceConfig from gRPC data.
func NewGameServer() *GameServer {
	gs := &GameServer{
		maps:     make(map[string]*MapState),
		clients:  make(map[EntityID]*Client),
		sessions: make(map[EntityID]*Session),
		// Buffer register/unregister so HandleConnections never blocks the
		// HTTP handler goroutine waiting for listenForClients to win the
		// world lock against gameLoop. 64 outstanding connect/disconnect
		// events is well beyond any realistic burst.
		register:             make(chan *Client, 64),
		unregister:           make(chan *Client, 64),
		objectLayerDataCache: make(map[string]*ObjectLayer),
		skillConfig:          make(map[string][]SkillDefinition),
		persistQueue:         make(chan persistJob, 256),
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
	// /api/v1/cyberia-client-hints using its own CYBERIA_CLIENT_HINTS_CODE.
	// The instance config tick_rate is ignored. The client fixes its step
	// at compile time, and a replay needs the same dt on both sides.
	s.tickRate = DefaultTickRate
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
	// and the optional /api/v1/cyberia-client-hints override.

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

	logx.Infof("[GameServer] Instance config applied: tickRate=%dHz, tickDuration=%v, aoiRadius=%.1f, entityBaseSpeed=%.1f, entityBaseMaxLife=%.1f, %d skills, %d entityDefaultTypes, %d entityDefaultBuilds, floorItem=%q, ghostItem=%q, coinItem=%q",
		s.tickRate, s.tickDuration, s.aoiRadius, s.entityBaseSpeed, s.entityBaseMaxLife, len(s.skillConfig), len(s.entityDefaults), len(s.entityDefaultBuilds), s.defaultFloorItemID, s.ghostItemID, s.coinItemID)
	return nil
}

// SetDataServerURL sets the Data Server origin used for server-to-server
// content-authority calls, and the key those calls carry. Never forwarded to
// clients.
func (s *GameServer) SetDataServerURL(url, apiKey string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.dataServerURL = url
	s.dataServerAPIKey = apiKey
	logx.Infof("Data Server URL set to: %s", url)
}

func (s *GameServer) Run() {
	go s.listenForClients()
	go s.gameLoop()
	go s.persistLoop()
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

// gameLoop advances the authoritative world by exactly one Tick per
// tickDuration. It calls the named simulation phases in a fixed order. The
// only function permitted to mutate world state lives downstream of this path.
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
//	phaseReplication one snapshot per player, after every map has stepped
//
// No phase reads PresentationHints.
func (s *GameServer) gameLoop() {
	if s.tickRate <= 0 {
		s.tickRate = DefaultTickRate
	}
	s.tickDuration = computeTickDuration(s.tickRate)

	simTicker := time.NewTicker(s.tickDuration)
	defer simTicker.Stop()

	logx.Infof("[GameServer] simulation and replication @ %d Hz (tick %v)",
		s.tickRate, s.tickDuration)

	for range simTicker.C {
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
				for _, player := range s.teleported {
					s.sendAOI(player)
				}
				s.teleported = s.teleported[:0]
			}
			for _, mapState := range s.maps {
				s.phaseReplication(mapState)
			}
			s.currentTick++
		}()
	}
}

// ---------- Movement and direction ----------

// ---------- AOI snapshot send ----------
//
// sendAOI is the one-shot snapshot dispatcher. Two call sites use it:
//
//   - phaseReplication (the snapshot-rate ticker) — the canonical path.
//   - gameLoop after phasePortals — an immediate snapshot for each player in
//     s.teleported, so the client doesn't see a frame of stale geometry;
//     semantically still part of the replication contract, just out-of-band
//     w.r.t. the tick.
//
// Outside these two call sites, simulation code MUST NOT send a snapshot —
// replication is the only path that may produce AOI frames.
func (s *GameServer) sendAOI(player *Session) {
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

type Client struct {
	sock        *socket.Socket
	playerID    EntityID
	playerState *Session

	// ip is the address the guard counts this connection against.
	ip string
	// limiter is the inbound message budget. Only the read goroutine for this
	// connection touches it.
	limiter *inputLimiter
	// released guards the guard slot so every disconnect path frees it once.
	released sync.Once
	// detached marks a torn-down client. register and unregister are separate
	// channels, so a fast disconnect can be handled before the matching
	// connect; the flag stops that from re-adding a dead client. Read and
	// written under GameServer.mu.
	detached bool
}

type GameServer struct {
	progressionRules ProgressionRules
	mu               sync.Mutex
	maps             map[string]*MapState
	instanceCode     string               // INSTANCE_CODE — selects which instance to load
	playerSpawn      PlayerSpawnConfig    // authoritative initial spawn for new players
	questsByCell     map[cellKey][]string // quest codes bound to each action cell
	clients          map[EntityID]*Client
	lastEntityID     EntityID // only goes up, so a freed id never comes back
	sessions         map[EntityID]*Session
	register         chan *Client
	unregister       chan *Client
	aoiRadius        float64
	portalHoldTime   time.Duration

	// ── Tick model (authoritative simulation cadence) ────────────────────────
	// The server advances one logical Tick per tickDuration and sends one
	// snapshot per player each tick.
	tickRate     int           // simulation Hz (e.g. 30)
	tickDuration time.Duration // 1 / tickRate
	currentTick  uint32        // monotonic; advanced once per simulation tick
	// teleported holds the players a portal moved this map step. gameLoop
	// sends their snapshots after phasePortals and clears it.
	teleported []*Session

	// bot related defaults
	botAggroRange float64

	// Caches (protected by olMu)
	olMu                 sync.RWMutex
	objectLayerDataCache map[string]*ObjectLayer

	// dataServerURL is the Data Server origin for server-to-server
	// content-authority calls (quest persistence). Never forwarded to clients:
	// each client learns its own Data Server URL from its command line.
	dataServerURL string
	// dataServerAPIKey is CYBERIA_SERVER_API_KEY, sent with every server-to-server
	// write. An empty key disables them.
	dataServerAPIKey string
	// persistQueue holds engine POSTs for persistLoop, so no HTTP call runs
	// under s.mu.
	persistQueue chan persistJob

	entityBaseSpeed float64
	// playerBaseSpeed applies to the player entity only. 0 means unset, and
	// CalculatePlayerMovementSpeed then falls back to entityBaseSpeed.
	playerBaseSpeed             float64
	entityBaseMaxLife           float64
	entityBaseActionCooldown    time.Duration
	entityBaseMinActionCooldown time.Duration

	// Player defaults
	defaultPlayerWidth     float64
	defaultPlayerHeight    float64
	playerBaseLifeRegenMin float64
	playerBaseLifeRegenMax float64
	maxActiveLayers        int
	initialLifeFraction    float64

	// Combat / death
	respawnDuration   time.Duration
	ghostItemID       string
	collisionLifeLoss float64
	// Every configured dead-item id — excluded from the player inventory wire
	// and from item activation. Populated exclusively from gRPC
	// entity_defaults.dead_item_ids (no hardcoded fallback).
	deadItemIDs map[string]bool

	// Economy — Fountain & Sink model
	// Fountains (coin injection)
	coinItemID       string
	botSpawnCoins    int // coins every bot carries on spawn (infinite mint)
	playerSpawnCoins int // coins given to player on first connect (guest wallet)
	// Kill Transfer (redistribution)
	coinKillPercentVsBot    float64 // fraction of victim coins transferred: PvE
	coinKillPercentVsPlayer float64 // fraction of victim coins transferred: PvP (gentler)
	coinKillMinAmount       int     // floor — every kill always pays this
	// Sinks (coin removal; all default to 0 = disabled in alpha)
	respawnCostPercent float64 // fraction burned on player death
	portalFee          int     // flat fee per portal use
	craftingFeePercent float64 // fraction of item value burned on crafting

	// Regen
	lifeRegenChance float64
	maxChance       float64

	// Skill config
	projectileSpawnChance           float64
	projectileLifetimeMs            int
	projectileWidth                 float64
	projectileHeight                float64
	projectileSpeedMultiplier       float64
	doppelgangerSpawnChance         float64
	doppelgangerLifetimeMs          int
	doppelgangerSpawnRadius         float64
	doppelgangerInitialLifeFraction float64

	// Floor defaults
	defaultFloorItemID string

	// Per-entity-type visual defaults.
	// entityDefaults keeps the last build per type for unique lookups.
	// entityDefaultBuilds preserves the full ordered config, including duplicates.
	entityDefaults      map[string]EntityTypeDefaultConfig
	entityDefaultBuilds []EntityTypeDefaultConfig

	// Portal defaults
	portalSpawnRadius float64

	// Skill map (runtime): trigger item ID → []SkillDefinition
	skillConfig map[string][]SkillDefinition

	// Action / quest content fetched from engine-cyberia REST at instance
	// init (actions and quests are not part of the gRPC world payload).
	//   actionCache  — entityId → bound CyberiaAction (resolved by the server
	//                  on every talk_done; the client never declares it).
	//   questDefs    — questCode → CyberiaQuest definition (steps/rewards).
	// Both are rebuilt on every world (re)build; access is under s.mu.
	actionCache map[EntityID]*CyberiaAction
	// storage — per (player, action code) vault contents. Session state
	// only; the slot shape is the future persistence document.
	storage   map[storageKey][]StorageSlot
	questDefs map[string]*CyberiaQuest
	// progression is the level and XP of each mortal entity. despawn deletes
	// the row.
	progression map[EntityID]*EntityProgression
	// quests is the quest progress of each player, keyed by quest code.
	// Authoritative for the session; best-effort mirrored to engine-cyberia
	// quest-progress REST for persistence.
	quests map[EntityID]map[string]*QuestProgress
	// crafts is the assembly running for each player. A craft consumes its
	// ingredients up front and pays out only when the timer elapses, so the
	// row is also the refund receipt a cancel needs. One at a time:
	// assembling is a modal operation.
	crafts map[EntityID]*ActiveCraft

	// Equipment rules — governs activation constraints.
	equipmentRules EquipmentRulesConfig

	// Lock-free observability counters. See metrics_counters.go.
	counters runtimeCounters

	// guard holds the connection admission limits. See connection_guard.go.
	guard  *connectionGuard
	limits ConnectionLimits

	// draining refuses new sessions while connected players stay. See drain.go.
	draining atomic.Bool
}

// SkillMapEntry describes one skill associated with a trigger item,
// for the init_data JSON payload sent to C clients.
type SkillMapEntry struct {
	LogicEventID         string `json:"logicEventId"`
	Name                 string `json:"name"`
	Description          string `json:"description"`
	SummonedEntityItemID string `json:"summonedEntityItemId"`
	// Held marks a skill the simulation runs by itself on the units an entity holds; a tap never fires it.
	Held bool `json:"held"`
}

// InitPayload — bootstrap message sent once on WebSocket connect.
// Strictly simulation and protocol; no presentation fields.
//
// Presentation values (cell-pixel size, camera tunings, palette,
// interpolation window, status-icon visuals) are NOT sent here. The C
// client resolves all of those through /api/v1/cyberia-client-hints using
// its own CYBERIA_CLIENT_HINTS_CODE.
type InitPayload struct {
	// EngineID is the own player id for the client's engine REST calls.
	EngineID       string                     `json:"engineId"`
	GridW          int                        `json:"gridW"`
	GridH          int                        `json:"gridH"`
	AoiRadius      float64                    `json:"aoiRadius"`
	ObjectLayers   []ObjectLayerState         `json:"objectLayers"`
	SkillMap       map[string][]SkillMapEntry `json:"skillMap"`
	EntityDefaults []EntityTypeDefaultConfig  `json:"entityDefaults"`
	// Resolved dead-state (Fragmentation) ids — the client labels these in
	// the inventory and disables their equip UI.
	DeadItemIds []string `json:"deadItemIds"`
	// Quests is the player's active/completed quest snapshot on connect.
	// Empty for a fresh guest. The C client seeds its local quest_store from
	// this and keeps it live via dialog_ack events (see Quest Journal).
	Quests []QuestSnapshotEntry `json:"quests"`
}

// phaseReplication runs last in each tick. Iterates connected players for
// this map, computes their AOI, encodes a snapshot, and sends it. This path
// is the only one that produces server→client AOI frames at steady state
// (the bootstrap full snapshot on connect is sent from handlers.go).
func (s *GameServer) phaseReplication(mapState *MapState) {
	for player := range s.playersOn(mapState) {
		player.AOI = Rectangle{
			MinX: mapState.tf(player.ID).Pos.X - s.aoiRadius,
			MinY: mapState.tf(player.ID).Pos.Y - s.aoiRadius,
			MaxX: mapState.tf(player.ID).Pos.X + mapState.tf(player.ID).Size.X + s.aoiRadius,
			MaxY: mapState.tf(player.ID).Pos.Y + mapState.tf(player.ID).Size.Y + s.aoiRadius,
		}
		s.sendAOI(player)
	}
}
