package net

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/persist"
	"cyberia-server/game/sim"
	"cyberia-server/game/social"
	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
	"cyberia-server/logx"

	"github.com/gorilla/websocket"
)

// captureEvery is how many ticks pass between two captures of the durable
// state of the account players.
const captureEvery = 6

// NewGameServer creates an empty game server with no hardcoded defaults.
// All configuration must be applied via ApplyInstanceConfig from gRPC data.
func NewGameServer() *GameServer {
	w := &world.World{
		Maps:     make(map[string]*world.MapState),
		Sessions: make(map[world.EntityID]*world.Session),
	}
	gs := &GameServer{
		Sim:     &sim.Sim{Content: content.NewContent(w), Social: social.New()},
		clients: make(map[world.EntityID]*Client),
		// Buffer register/unregister so HandleConnections never blocks the
		// HTTP handler goroutine waiting for listenForClients to win the
		// world lock against gameLoop. 64 outstanding connect/disconnect
		// events is well beyond any realistic burst.
		register:    make(chan *Client, 64),
		unregister:  make(chan *Client, 64),
		mode:        SessionAnonymous,
		upgrader:    newUpgrader(nil),
		authTimeout: 10 * time.Second,
	}
	gs.SetConnectionLimits(DefaultConnectionLimits())
	return gs
}

// SetAllowedOrigins sets the browser origins that may open the game socket.
// Call it before the server accepts connections.
func (s *GameServer) SetAllowedOrigins(origins []string) {
	s.upgrader = newUpgrader(origins)
}

// SetAuthTimeout bounds how long a new connection may take to send its auth
// message. Call it before the server accepts connections.
func (s *GameServer) SetAuthTimeout(timeout time.Duration) {
	if timeout > 0 {
		s.authTimeout = timeout
	}
}

// EnableAccounts switches the server to account mode: a connection needs a
// launch ticket, and store commits the durable state of every player. Call it
// before Run.
// SetBuild names the binary of this process for the clients. Call it before Run.
func (s *GameServer) SetBuild(build string) { s.build = build }

func (s *GameServer) EnableAccounts(accounts Accounts, store persist.Store, serverID string) {
	s.mode = SessionAccount
	s.accounts = accounts
	s.serverID = serverID
	s.durable = persist.New(store)
	s.Durability = s.durable
	s.Friendships = s
	s.Markets = s
}

// RestoreWorldDrops puts back the public world drops of this world. Account
// mode only; call it once the world is built.
func (s *GameServer) RestoreWorldDrops(ctx context.Context) error {
	if s.accounts == nil {
		return nil
	}
	drops, err := s.accounts.ListWorldDrops(ctx, s.InstanceCode)
	if err != nil {
		return err
	}
	s.Mu.Lock()
	defer s.Mu.Unlock()
	restored := s.Content.RestoreWorldDrops(drops, time.Now())
	logx.Infof("[GameServer] world drops restored: %d of %d", restored, len(drops))
	return nil
}

// Mode is the session mode of the server.
func (s *GameServer) Mode() SessionMode { return s.mode }

// Persistence reads the counters and the queue of the persist pipeline; false
// in anonymous mode.
func (s *GameServer) Persistence() (persist.Snapshot, bool) {
	if s.durable == nil {
		return persist.Snapshot{}, false
	}
	return s.durable.Snapshot(), true
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

	rules := content.ProgressionRulesFromProto(cfg.GetProgressionRules())
	if err := rules.Validate(); err != nil {
		return err
	}
	if err := content.EconomyRulesFromProto(cfg.GetEconomyRules()).Validate(); err != nil {
		return err
	}
	s.Mu.Lock()
	defer s.Mu.Unlock()

	// ── Simulation cadence ─────────────────────────────────────────────────
	// Presentation defaults (cell-pixel size, object default dims, camera,
	// palette, interpolation) are deliberately absent — they are not part
	// of the simulation contract. The cyberia-client fetches them from
	// /api/v1/cyberia-client-hints using its own CYBERIA_CLIENT_HINTS_CODE.
	// The instance config tick_rate is ignored. The client fixes its step
	// at compile time, and a replay needs the same dt on both sides.

	// World / AOI
	s.aoiRadius = cfg.GetAoiRadius()
	s.Sim.ApplyConfig(cfg, rules)

	logx.Infof("[GameServer] Instance config applied: tickRate=%dHz, tickDuration=%v, aoiRadius=%.1f, entityBaseSpeed=%.1f, entityBaseMaxLife=%.1f, %d skills, %d entityDefaultTypes, %d entityDefaultBuilds, floorItem=%q, ghostItem=%q, coinItem=%q",
		sim.DefaultTickRate, sim.TickDuration, s.aoiRadius, s.EntityBaseSpeed, s.EntityBaseMaxLife, len(s.SkillConfig), len(s.EntityDefaults), len(s.EntityDefaultBuilds), s.DefaultFloorItemID, s.GhostItemID, s.CoinItemID)
	return nil
}

func (s *GameServer) Run() {
	go s.listenForClients()
	go s.gameLoop()
	if s.resolver != nil {
		go s.RunContentSweep(context.Background())
	}
	if s.durable != nil {
		go s.durable.Run(context.Background())
		go s.durable.RunCheckpoints(context.Background())
		go s.RunResumeTickets(context.Background())
	}
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
			s.Mu.Lock()
			// A disconnect already handled for this client wins: re-adding it
			// here would leave an entry nothing will ever remove.
			if !client.detached {
				s.clients[client.playerID] = client
			}
			s.Mu.Unlock()
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
//	phaseMovement   integrate positions using tickDuration (NOT 1/fps)
//	phaseLoot       player↔drop-token collection + priority validation
//	phasePortals    portal entry / teleport
//	phaseReplication one snapshot per player, after every map has stepped
//
// No phase reads PresentationHints.
func (s *GameServer) gameLoop() {
	simTicker := time.NewTicker(sim.TickDuration)
	defer simTicker.Stop()

	logx.Infof("[GameServer] simulation and replication @ %d Hz (tick %v)",
		sim.DefaultTickRate, sim.TickDuration)

	for range simTicker.C {
		started := time.Now()
		func() {
			defer func() {
				if r := recover(); r != nil {
					logx.Errorf("[GameServer] PANIC in simulation tick: %v", r)
				}
			}()
			s.Mu.Lock()
			defer s.Mu.Unlock()
			tick := s.currentTick
			for _, mapState := range s.Maps {
				s.PhaseInput(tick, mapState)
				s.PhaseLifecycle(tick, mapState)
				s.PhaseSkills(tick, mapState)
				s.PhaseAI(tick, mapState)
				s.PhaseMovement(tick, mapState)
				s.PhaseLoot(tick, mapState)
				s.PhasePortals(tick, mapState)
				for _, player := range s.Teleported {
					s.sendAOI(player)
					s.CaptureNow(player, true)
				}
				s.Teleported = s.Teleported[:0]
			}
			if s.durable != nil {
				s.SettleLinkedOps(time.Now())
			}
			for _, mapState := range s.Maps {
				s.phaseReplication(mapState)
			}
			if s.durable != nil && tick%captureEvery == 0 {
				s.captureDurable()
			}
			s.currentTick++
		}()
		if s.durable != nil {
			s.endSessions(s.durable.TakeEnded())
		}
		s.counters.recordTick(time.Since(started))
	}
}

// captureDurable records the durable state of every account player and the
// health of its commits. Caller holds s.Mu.
func (s *GameServer) captureDurable() {
	for _, player := range s.Sessions {
		if player.AccountID == "" {
			continue
		}
		id := string(player.PlayerID)
		s.durable.Capture(id, s.DurableState(player), false)
		player.Durable = s.durable.Health(id)
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
func (s *GameServer) sendAOI(player *world.Session) {
	mapState, ok := s.Maps[player.MapCode]
	if !ok {
		logx.Warnf("Map %q not found for player %s.", player.MapCode, player.ID)
		return
	}

	s.sendObjectLayerChanges(player)
	s.sendEconomyChange(player)
	s.sendWorldChange(player)
	world.SendMessage(player, "snapshot", s.buildSnapshot(player, mapState))
}

// GetConnectedClientsCount returns the current connection count.
func (s *GameServer) GetConnectedClientsCount() int {
	s.Mu.Lock()
	defer s.Mu.Unlock()
	return len(s.clients)
}

type Client struct {
	sock        *Socket
	playerID    world.EntityID
	playerState *world.Session

	// ip is the address the guard counts this connection against.
	ip string
	// limiter is the inbound message budget. Only the read goroutine for this
	// connection touches it.
	limiter *inputLimiter
	// queuedThrough is the last event sequence queued for the simulation;
	// dropped marks a message the rate limit dropped since, the one cause of a
	// gap; resendFrom, resendAt name the last input_resend asked, and gapSince
	// when the server first asked for that gap. Only the read goroutine
	// touches them.
	queuedThrough uint32
	dropped       bool
	resendFrom    uint32
	resendAt      time.Time
	gapSince      time.Time
	// released guards the guard slot so every disconnect path frees it once.
	released sync.Once
	// detached marks a torn-down client. register and unregister are separate
	// channels, so a fast disconnect can be handled before the matching
	// connect; the flag stops that from re-adding a dead client. Read and
	// written under World.Mu.
	detached bool
	// resumeExpiresAt is when the resume ticket the client holds expires; zero
	// before the first. Read and written under World.Mu.
	resumeExpiresAt time.Time
	// olSeen is the definition revision the client holds. Written at placement,
	// then under World.Mu.
	olSeen uint64
	// economySeen names the economy snapshot the client holds, written like olSeen.
	economySeen string
	// worldSeen is the world revision the client holds, written like olSeen.
	worldSeen uint64
}

// GameServer is the network side: the client links, the tick loop and the
// replication. Sim runs the simulation.
type GameServer struct {
	*sim.Sim
	clients    map[world.EntityID]*Client
	register   chan *Client
	unregister chan *Client
	aoiRadius  float64

	// ── Tick model (authoritative simulation cadence) ────────────────────────
	// The server advances one logical Tick per tickDuration and sends one
	// snapshot per player each tick.
	currentTick uint32 // monotonic; advanced once per simulation tick

	// Lock-free observability counters. See metrics_counters.go.
	counters runtimeCounters

	// guard holds the connection admission limits. See connection_guard.go.
	guard  *connectionGuard
	limits ConnectionLimits

	// draining refuses new sessions while connected players stay. See drain.go.
	draining atomic.Bool

	// ── Sessions (accounts.go, sessions.go) ──────────────────────────────────
	mode     SessionMode
	serverID string
	// build names the binary of this process, sent to each client at join.
	build       string
	accounts    Accounts
	durable     *persist.Pipeline
	upgrader    websocket.Upgrader
	authTimeout time.Duration

	// resolver fetches the content a player brings from another world, and
	// release names the content release of the world. See content_resolve.go.
	resolver *contentResolver
	release  func() string
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
	// Session is the principal of the session: "account", whose state
	// persists, or "guest", whose state ends with it.
	Session string `json:"session"`
	// MaxLevel is the highest level of this instance; a container slot that
	// opens past it never opens.
	MaxLevel       int                               `json:"maxLevel"`
	GridW          int                               `json:"gridW"`
	GridH          int                               `json:"gridH"`
	AoiRadius      float64                           `json:"aoiRadius"`
	ObjectLayers   []content.InventoryStack          `json:"objectLayers"`
	SkillMap       map[string][]SkillMapEntry        `json:"skillMap"`
	EntityDefaults []content.EntityTypeDefaultConfig `json:"entityDefaults"`
	// Resolved dead-state (Fragmentation) ids — the client labels these in
	// the inventory and disables their equip UI.
	DeadItemIds []string `json:"deadItemIds"`
	// Quests is the player's active/completed quest snapshot on connect.
	// Empty for a fresh guest. The C client seeds its local quest_store from
	// this and keeps it live via dialog_ack events (see Quest Journal).
	Quests []content.QuestSnapshotEntry `json:"quests"`
	// Build names the binary of this server. A client that plays here plays
	// on every server of the same build.
	Build string `json:"build,omitempty"`
	// Adopted marks an account session that plays the state its closed
	// session on this server had not saved yet; that state saves next.
	Adopted bool `json:"adopted,omitempty"`
}

// phaseReplication runs last in each tick. Iterates connected players for
// this map, computes their AOI, encodes a snapshot, and sends it. This path
// is the only one that produces server→client AOI frames at steady state
// (the bootstrap full snapshot on connect is sent from handlers.go).
func (s *GameServer) phaseReplication(mapState *world.MapState) {
	for player := range s.PlayersOn(mapState) {
		player.AOI = world.Rectangle{
			MinX: mapState.Tf(player.ID).Pos.X - s.aoiRadius,
			MinY: mapState.Tf(player.ID).Pos.Y - s.aoiRadius,
			MaxX: mapState.Tf(player.ID).Pos.X + mapState.Tf(player.ID).Size.X + s.aoiRadius,
			MaxY: mapState.Tf(player.ID).Pos.Y + mapState.Tf(player.ID).Size.Y + s.aoiRadius,
		}
		s.sendAOI(player)
	}
}
