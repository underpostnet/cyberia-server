package net

import (
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/sim"
	"cyberia-server/game/social"
	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
	"cyberia-server/logx"
)

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
		register:   make(chan *Client, 64),
		unregister: make(chan *Client, 64),
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

// SetDataServerURL sets the Data Server origin used for server-to-server
// content-authority calls, and the key those calls carry. Never forwarded to
// clients.
func (s *GameServer) SetDataServerURL(url, apiKey string) {
	s.Mu.Lock()
	defer s.Mu.Unlock()
	s.DataServerURL = url
	s.DataServerAPIKey = apiKey
	logx.Infof("Data Server URL set to: %s", url)
}

func (s *GameServer) Run() {
	go s.listenForClients()
	go s.gameLoop()
	go s.PersistLoop()
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
				}
				s.Teleported = s.Teleported[:0]
			}
			for _, mapState := range s.Maps {
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
func (s *GameServer) sendAOI(player *world.Session) {
	mapState, ok := s.Maps[player.MapCode]
	if !ok {
		logx.Warnf("Map %q not found for player %s.", player.MapCode, player.ID)
		return
	}

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
	// released guards the guard slot so every disconnect path frees it once.
	released sync.Once
	// detached marks a torn-down client. register and unregister are separate
	// channels, so a fast disconnect can be handled before the matching
	// connect; the flag stops that from re-adding a dead client. Read and
	// written under World.Mu.
	detached bool
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
	// PlayerID is the own player id for the client's engine REST calls.
	PlayerID world.PlayerID `json:"playerId"`
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
