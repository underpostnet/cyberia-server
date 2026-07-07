package game

import (
	"sync"
	"time"

	"github.com/gorilla/websocket"
)

// 1. Data Structures & Interfaces

type Point struct {
	X float64 `json:"X"`
	Y float64 `json:"Y"`
}

type PointI struct {
	X, Y int
}

// cellKey identifies a map cell. Both cyberia-action and cyberia-quest bind to
// entities by this cell, so a bot on a quest's cell can offer it.
type cellKey struct {
	mapCode string
	cellX   int
	cellY   int
}

type Rectangle struct {
	MinX, MinY, MaxX, MaxY float64
}

type Dimensions struct {
	Width  float64 `json:"Width"`
	Height float64 `json:"Height"`
}

type Direction int

const (
	UP Direction = iota
	UP_RIGHT
	RIGHT
	DOWN_RIGHT
	DOWN
	DOWN_LEFT
	LEFT
	UP_LEFT
	NONE
)

type ObjectLayerMode int

const (
	IDLE ObjectLayerMode = iota
	WALKING
	TELEPORTING
)

type ObjectLayerState struct {
	ItemID   string `json:"itemId"`
	Active   bool   `json:"active"`
	Quantity int    `json:"quantity"`
}

// EntityBase — identity + spatial fields shared by every world entity.
// Anonymously embedded so field access (e.g. player.Pos) and JSON keys are
// promoted unchanged.
type EntityBase struct {
	ID           string             `json:"id"`
	Pos          Point              `json:"Pos"`
	Dims         Dimensions         `json:"Dims"`
	ObjectLayers []ObjectLayerState `json:"objectLayers"`
}

// Mortal — life/respawn block for entities that can die (Player, Bot, Resource).
type Mortal struct {
	MaxLife                float64            `json:"maxLife"`
	Life                   float64            `json:"life"`
	RespawnTime            time.Time          `json:"-"`
	PreRespawnObjectLayers []ObjectLayerState `json:"-"`
	StatsDirty             bool               `json:"-"` // Set true when ObjectLayers change; cleared by CalculateStats cache.
}

// IsGhost reports whether the entity is dead and waiting to respawn.
func (m *Mortal) IsGhost() bool { return !m.RespawnTime.IsZero() }

type ObjectState struct {
	EntityBase
	Type string `json:"Type"`
}

type PlayerState struct {
	EntityBase
	Mortal
	MapCode        string          `json:"MapCode"`
	Path           []PointI        `json:"path"`
	TargetPos      PointI          `json:"targetPos"`
	AOI            Rectangle       `json:"AOI"`
	Client         *Client         `json:"-"`
	Direction      Direction       `json:"direction"`
	Mode           ObjectLayerMode `json:"mode"`
	OnPortal       bool            `json:"onPortal"`
	TimeOnPortal   time.Time       `json:"-"`
	ActivePortalID string          `json:"activePortalID"`
	SumStatsLimit  int             `json:"sumStatsLimit"`
	LifeRegen      float64         `json:"lifeRegen"`
	// Coins is the canonical flat coin balance — the single source of truth
	// for all economy operations.  O(1) read/write, never iterates ObjectLayers.
	// The coin ObjectLayer slot (coinItemID) is kept in sync for inventory
	// visualization only and is always Active: false.
	Coins uint32 `json:"-"`
	// FrozenInteractionState — general-purpose modal protection.
	// While Frozen, the player cannot deal or receive damage, send or
	// receive events, or move.  The rest of the world continues.
	// Managed exclusively by FreezePlayer / ThawPlayer (frozen_state.go).
	Frozen       bool      `json:"-"`
	FreezeReason string    `json:"-"` // e.g. "dialogue", "inventory"
	FreezeStart  time.Time `json:"-"`

	// ── Dialogue interaction context ────────────────────────────────────────
	// ActiveDialogueEntityID is the entity the player currently has an open
	// dialogue with (set on dlg_start, cleared on dlg_complete / dlg_cancel).
	// It is the cross-process contract that lets dlg_complete validate that
	// the client is acknowledging the dialogue it actually opened — a stale
	// or spoofed entityId is dropped. The client never declares the action
	// type or quest code; the server resolves those from actionCache.
	ActiveDialogueEntityID string `json:"-"`
	// ActiveDialogueSkin freezes the provider's NPC skin at dlg_start, when the bot
	// is in range and the modal opens. It is the authority dlg_complete validates
	// the talk objective against, so completion resolves from the snapshot taken on
	// open — independent of whether the bot later dies, wanders off, or leaves the
	// player's AOI before the dialogue is finished.
	ActiveDialogueSkin string `json:"-"`
	// Quests is the player's per-session quest progress, keyed by quest code.
	// Authoritative for the session; best-effort mirrored to engine-cyberia
	// quest-progress REST for persistence.
	Quests map[string]*QuestProgress `json:"-"`

	// ── Tick / replication metadata ─────────────────────────────────────────
	// LastSnapshotTick is stamped by phaseReplication just before the AOI
	// encoder runs for this player. It is embedded in the snapshot header.
	// LastAckedInputSequence is the highest InputCommand.Sequence accepted by
	// the simulation for this player; the snapshot encoder echoes it so the
	// client can drop acknowledged commands from its prediction buffer.
	LastSnapshotTick       uint32 `json:"-"`
	LastAckedInputSequence uint32 `json:"-"`
	// InputQueue holds InputCommand frames received from the client between
	// simulation ticks. Drained in fixed order by phaseInput. nil-safe: the
	// queue is allocated lazily when the first command is enqueued.
	InputQueue []InputCommand `json:"-"`
}

type FloorState struct {
	EntityBase
	Type string `json:"Type"`
}

type BotState struct {
	EntityBase
	Mortal
	MapCode              string          `json:"MapCode"`
	Path                 []PointI        `json:"path"`
	TargetPos            PointI          `json:"targetPos"`
	Direction            Direction       `json:"direction"`
	Mode                 ObjectLayerMode `json:"mode"`
	Behavior             string          `json:"behavior"` // canonical entity behavior (see behavior.go)
	SpawnCenter          Point           `json:"spawnCenter"`
	SpawnRadius          float64         `json:"spawnRadius"`
	AggroRange           float64         `json:"aggroRange"`
	CurrentTargetPlayer  string          `json:"-"` // player ID currently being pursued (if any)
	lastPursuitTargetPos PointI          `json:"-"` // cached player's last cell to know when to re-path
	lastAction           time.Time       `json:"-"`
	ExpiresAt            time.Time       `json:"-"` // bots with a lifetime will be removed after this time
	LifeRegen            float64         `json:"lifeRegen"`
	CasterID             string          `json:"-"` // ID of the player or bot that created this bot
	// Coins is the canonical flat coin balance — the single source of truth
	// for all economy operations.  O(1) read/write, never iterates ObjectLayers.
	// The coin ObjectLayer slot (coinItemID) is kept in sync for inventory
	// visualization only and is always Active: false.
	Coins uint32 `json:"-"`
	// ActionCode binds this entity to a cached CyberiaAction (see actionCache).
	// Non-empty marks the bot as an action-provider (ESI 8 overhead icon).
	ActionCode string `json:"-"`

	// DamageLedger is the per-victim threat/contribution ledger:
	// attacking player ID → cumulative damage dealt to this bot. Populated by
	// the skill-collision path (players only; bot-vs-bot damage is not tracked
	// for loot). Read once on death to resolve the top contributor, then reset.
	// Allocated lazily on first hit. Never sent on the wire.
	DamageLedger map[string]float64 `json:"-"`

	// ── Transient world-item drop fields (BehaviorDrop bots only) ────────────
	// A drop is a collectible token scattered on the grid when a bot dies. It
	// carries exactly one item and is picked up by walking over it.
	// DropItemID is the item this token grants on collection.
	DropItemID string `json:"-"`
	// LootContributors is the set of player IDs that dealt damage to the entity
	// this token dropped from. Once the token settles, collection is a race: any
	// contributor that collides may pick it up. Empty = no player dealt damage,
	// so anyone may collect.
	LootContributors map[string]struct{} `json:"-"`
	// CollectableAt is the instant the spawn-launch settle window ends. Until
	// then the token is mid-flight from the corpse to its cell and registers no
	// collision — collection is disabled regardless of loot priority. Matches
	// the client launch-animation duration sent in MsgTypeDropSpawn.
	CollectableAt time.Time `json:"-"`
}

type PortalConfig struct {
	DestMapCode string
	SpawnRadius float64
	PortalMode  string  // inter-portal | inter-random | intra-random | intra-portal
	DestCellX   float64 // target cell position (used by portal-to-portal modes)
	DestCellY   float64
}

// ResourceState represents a static exploitable entity on the map
// (wood, minerals, organic matter, etc.). Resources have life, can be
// destroyed by projectile impact, transfer configured drop items to the
// extractor on death, switch to dead/extracted visuals, and respawn later.
// They never move.
type ResourceState struct {
	EntityBase
	Mortal
	MapCode string `json:"MapCode"`
	// DamageLedger is the per-resource extraction ledger: attacking player ID →
	// cumulative damage dealt. Mirrors BotState.DamageLedger — read once on
	// death to grant the top contributor loot priority, then reset. Never wired.
	DamageLedger map[string]float64 `json:"-"`
}

// StaticState represents a non-moving, passable decorator entity (rocks,
// bushes, signage, etc.). It has a position, dimensions, and object layers,
// but no life, no AI, and no collision: it never blocks movement and is never
// exploitable. The client depth-sorts it with bots/players (Y-axis ordering)
// so the player can pass behind or in front of it.
type StaticState struct {
	EntityBase
	MapCode string `json:"MapCode"`
}

type PortalState struct {
	EntityBase
	Type         string        `json:"Type"`
	Subtype      string        `json:"Subtype"` // inter-portal | inter-random | intra-random | intra-portal
	PortalConfig *PortalConfig `json:"-"`
}

type MapState struct {
	mu           sync.RWMutex
	pathfinder   *Pathfinder
	obstacles    map[string]ObjectState
	foregrounds  map[string]ObjectState
	portals      map[string]*PortalState
	floors       map[string]*FloorState
	players      map[string]*PlayerState
	bots         map[string]*BotState
	resources    map[string]*ResourceState
	statics      map[string]*StaticState
	gridW, gridH int
}

type Client struct {
	conn        *websocket.Conn
	playerID    string
	send        chan []byte
	lastAction  time.Time
	playerState *PlayerState
}

type GameServer struct {
	mu             sync.Mutex
	maps           map[string]*MapState
	instanceCode   string               // INSTANCE_CODE — selects which instance to load
	playerSpawn    PlayerSpawnConfig    // authoritative initial spawn for new players
	questsByCell   map[cellKey][]string // quest codes bound to each action cell
	clients        map[string]*Client
	register       chan *Client
	unregister     chan *Client
	aoiRadius      float64
	portalHoldTime time.Duration

	// ── Tick model (authoritative simulation cadence) ────────────────────────
	// The server advances one logical Tick per tickDuration. snapshotRate
	// is the AOI replication Hz; ≤ tickRate.
	tickRate     int           // simulation Hz (e.g. 30)
	snapshotRate int           // replication Hz (e.g. 20)
	tickDuration time.Duration // 1 / tickRate
	currentTick  uint32        // monotonic; advanced once per simulation tick

	// bot related defaults
	botAggroRange float64

	// Caches (protected by olMu)
	olMu                 sync.RWMutex
	objectLayerDataCache map[string]*ObjectLayer

	// engineApiBaseUrl is the internal engine-cyberia origin for
	// server-to-server content-authority calls (quest persistence). Never
	// forwarded to clients.
	engineApiBaseUrl string

	// enginePublicURL is the client-visible Content Authority origin forwarded
	// to clients for every content/asset/metadata request.
	enginePublicURL string

	entityBaseSpeed             float64
	entityBaseMaxLife           float64
	entityBaseActionCooldown    time.Duration
	entityBaseMinActionCooldown time.Duration

	// Player defaults
	defaultPlayerWidth     float64
	defaultPlayerHeight    float64
	playerBaseLifeRegenMin float64
	playerBaseLifeRegenMax float64
	sumStatsLimit          int
	maxActiveLayers        int
	initialLifeFraction    float64

	// Combat / death
	respawnDuration   time.Duration
	ghostItemID       string
	collisionLifeLoss float64

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
	//                  on every dlg_complete; the client never declares it).
	//   questDefs    — questCode → CyberiaQuest definition (steps/rewards).
	// Both are rebuilt on every world (re)build; access is under s.mu.
	actionCache map[string]*CyberiaAction
	questDefs   map[string]*CyberiaQuest

	// Stats cache: entityID → cached entry with TTL. Invalidated when StatsDirty is set
	// or when the TTL (statsCacheTTL) expires.
	statsCache    map[string]statsCacheEntry
	statsCacheTTL time.Duration

	// Equipment rules — governs activation constraints.
	equipmentRules EquipmentRulesConfig

	// Lock-free observability counters. See metrics_counters.go.
	counters runtimeCounters
}

// PlayerSpawnConfig — authoritative initial spawn for new players. When Random
// is false and MapCode names a loaded map, players spawn at (CellX, CellY) on it;
// otherwise spawn is a random walkable cell on a random map.
type PlayerSpawnConfig struct {
	MapCode string
	CellX   int
	CellY   int
	Random  bool
}

// EntityTypeDefaultConfig — gameplay defaults for one entity type.
type EntityTypeDefaultConfig struct {
	EntityType          string             `json:"entityType"`
	LiveItemIDs         []string           `json:"liveItemIds"`
	DeadItemIDs         []string           `json:"deadItemIds"`
	DropItemIDs         []string           `json:"dropItemIds"`
	DefaultObjectLayers []ObjectLayerState `json:"defaultObjectLayers,omitempty"`
	// Canonical entity behavior bound to matched entities (see behavior.go).
	// Empty = derive from layers (armed → hostile, else passive).
	Behavior string `json:"behavior,omitempty"`
}

// EquipmentRulesConfig governs which item types can be simultaneously active
// and enforces the one-active-per-type constraint.  Loaded from the proto
// EquipmentRules message via ApplyInstanceConfig.
type EquipmentRulesConfig struct {
	ActiveItemTypes map[string]bool `json:"activeItemTypes"` // set of activable item types
	OnePerType      bool            `json:"onePerType"`      // enforce one active per type
	RequireSkin     bool            `json:"requireSkin"`     // require ≥ 1 active skin
}

// SkillMapEntry describes one skill associated with a trigger item,
// for the init_data JSON payload sent to C clients.
type SkillMapEntry struct {
	LogicEventID         string `json:"logicEventId"`
	Name                 string `json:"name"`
	Description          string `json:"description"`
	SummonedEntityItemID string `json:"summonedEntityItemId"`
}

// InitPayload — bootstrap message sent once on WebSocket connect.
// Strictly simulation and protocol; no presentation fields.
//
// Presentation values (cell-pixel size, camera tunings, palette,
// interpolation window, status-icon visuals) are NOT sent here. The C
// client resolves all of those through /api/cyberia-client-hints using
// its own CYBERIA_CLIENT_HINTS_CODE.
type InitPayload struct {
	GridW          int                        `json:"gridW"`
	GridH          int                        `json:"gridH"`
	TickRate       int                        `json:"tickRate"`
	SnapshotRate   int                        `json:"snapshotRate"`
	AoiRadius      float64                    `json:"aoiRadius"`
	SumStatsLimit  int                        `json:"sumStatsLimit"`
	ObjectLayers   []ObjectLayerState         `json:"objectLayers"`
	SkillMap       map[string][]SkillMapEntry `json:"skillMap"`
	EntityDefaults []EntityTypeDefaultConfig  `json:"entityDefaults"`
	// Quests is the player's active/completed quest snapshot on connect.
	// Empty for a fresh guest. The C client seeds its local quest_store from
	// this and keeps it live via dlg_ack events (see Quest Journal).
	Quests []QuestSnapshotEntry `json:"quests"`
}
