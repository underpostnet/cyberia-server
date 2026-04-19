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

type ObjectState struct {
	ID    string     `json:"id"`
	Pos   Point      `json:"Pos"`
	Dims  Dimensions `json:"Dims"`
	Type  string     `json:"Type"`
	Color ColorRGBA  `json:"color"`
}

type PlayerState struct {
	ID                     string             `json:"id"`
	MapCode                string             `json:"MapCode"`
	Pos                    Point              `json:"Pos"`
	Dims                   Dimensions         `json:"Dims"`
	Color                  ColorRGBA          `json:"color"`
	Path                   []PointI           `json:"path"`
	TargetPos              PointI             `json:"targetPos"`
	AOI                    Rectangle          `json:"AOI"`
	Client                 *Client            `json:"-"`
	Direction              Direction          `json:"direction"`
	Mode                   ObjectLayerMode    `json:"mode"`
	OnPortal               bool               `json:"onPortal"`
	TimeOnPortal           time.Time          `json:"-"`
	ActivePortalID         string             `json:"activePortalID"`
	SumStatsLimit          int                `json:"sumStatsLimit"`
	ObjectLayers           []ObjectLayerState `json:"objectLayers"`
	MaxLife                float64            `json:"maxLife"`
	Life                   float64            `json:"life"`
	LifeRegen              float64            `json:"lifeRegen"`
	RespawnTime            time.Time          `json:"-"`
        PreRespawnObjectLayers []ObjectLayerState `json:"-"`
        // Coins is the canonical flat coin balance — the single source of truth
        // for all economy operations.  O(1) read/write, never iterates ObjectLayers.
        // The coin ObjectLayer slot (coinItemID) is kept in sync for inventory
        // visualization only and is always Active: false.
        Coins     uint32 `json:"-"`
        // FrozenInteractionState — general-purpose modal protection.
        // While Frozen, the player cannot deal or receive damage, send or
        // receive events, or move.  The rest of the world continues.
        // Managed exclusively by FreezePlayer / ThawPlayer (frozen_state.go).
        Frozen       bool      `json:"-"`
        FreezeReason string    `json:"-"` // e.g. "dialogue", "inventory"
        FreezeStart  time.Time `json:"-"`
        StatsDirty             bool               `json:"-"` // Set true when ObjectLayers change; cleared by CalculateStats cache.
}

type FloorState struct {
	ID           string             `json:"id"`
	Pos          Point              `json:"Pos"`
	Dims         Dimensions         `json:"Dims"`
	Type         string             `json:"Type"`
	ObjectLayers []ObjectLayerState `json:"objectLayers"`
	Color        ColorRGBA          `json:"color"`
}

type BotState struct {
	ID                     string             `json:"id"`
	MapCode                string             `json:"MapCode"`
	Pos                    Point              `json:"Pos"`
	Dims                   Dimensions         `json:"Dims"`
	Color                  ColorRGBA          `json:"color"`
	Path                   []PointI           `json:"path"`
	TargetPos              PointI             `json:"targetPos"`
	Direction              Direction          `json:"direction"`
	Mode                   ObjectLayerMode    `json:"mode"`
	Behavior               string             `json:"behavior"` // "hostile" or "passive"
	SpawnCenter            Point              `json:"spawnCenter"`
	SpawnRadius            float64            `json:"spawnRadius"`
	AggroRange             float64            `json:"aggroRange"`
	CurrentTargetPlayer    string             `json:"-"` // player ID currently being pursued (if any)
	lastPursuitTargetPos   PointI             `json:"-"` // cached player's last cell to know when to re-path
	lastAction             time.Time          `json:"-"`
	ObjectLayers           []ObjectLayerState `json:"objectLayers"`
	ExpiresAt              time.Time          `json:"-"` // bots with a lifetime will be removed after this time
	MaxLife                float64            `json:"maxLife"`
	Life                   float64            `json:"life"`
	LifeRegen              float64            `json:"lifeRegen"`
	RespawnTime            time.Time          `json:"-"`
	PreRespawnObjectLayers []ObjectLayerState `json:"-"`
	CasterID               string             `json:"-"` // ID of the player or bot that created this bot
        // Coins is the canonical flat coin balance — the single source of truth
        // for all economy operations.  O(1) read/write, never iterates ObjectLayers.
        // The coin ObjectLayer slot (coinItemID) is kept in sync for inventory
        // visualization only and is always Active: false.
        Coins      uint32 `json:"-"`
        StatsDirty             bool               `json:"-"` // Set true when ObjectLayers change; cleared by CalculateStats cache.
}

type PortalConfig struct {
	DestMapCode string
	SpawnRadius float64
	PortalMode  string  // inter-portal | inter-random | intra-random | intra-portal
	DestCellX   float64 // target cell position (used by portal-to-portal modes)
	DestCellY   float64
}

// ResourceState represents a static exploitable entity on the map
// (wood, minerals, organic matter, etc.).  Resources have life, can be
// destroyed by projectile impact, transfer their OLs to the killer on death,
// and respawn after a timer.  They never move.
type ResourceState struct {
	ID                     string             `json:"id"`
	MapCode                string             `json:"MapCode"`
	Pos                    Point              `json:"Pos"`
	Dims                   Dimensions         `json:"Dims"`
	Color                  ColorRGBA          `json:"color"`
	ObjectLayers           []ObjectLayerState `json:"objectLayers"`
	MaxLife                float64            `json:"maxLife"`
	Life                   float64            `json:"life"`
	RespawnTime            time.Time          `json:"-"`
	PreRespawnObjectLayers []ObjectLayerState `json:"-"`
	StatsDirty             bool               `json:"-"`
}

// IsGhost checks if a resource is in a destroyed state (waiting to respawn).
func (r *ResourceState) IsGhost() bool {
	return !r.RespawnTime.IsZero()
}

type PortalState struct {
	ID           string        `json:"id"`
	Pos          Point         `json:"Pos"`
	Dims         Dimensions    `json:"Dims"`
	Type         string        `json:"Type"`
	Subtype      string        `json:"Subtype"` // inter-portal | inter-random | intra-random | intra-portal
	PortalConfig *PortalConfig `json:"-"`
	Label        string        `json:"PortalLabel"`
	Color        ColorRGBA     `json:"color"`
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
	clients        map[string]*Client
	register       chan *Client
	unregister     chan *Client
	aoiRadius      float64
	portalHoldTime time.Duration

	cellSize         float64
	fps              int
	interpolationMs  int
	defaultObjWidth  float64
	defaultObjHeight float64
	colors           map[string]ColorRGBA

	cameraSmoothing float64
	cameraZoom      float64

	defaultWidthScreenFactor  float64
	defaultHeightScreenFactor float64

	devUi bool

	// bot related defaults
	botAggroRange float64

	// Caches (protected by olMu)
	olMu                 sync.RWMutex
	objectLayerDataCache map[string]*ObjectLayer

	// Engine API base URL — forwarded to clients for binary blob fetches.
	engineApiBaseUrl string

	entityBaseSpeed             float64
	entityBaseMaxLife           float64
	entityBaseActionCooldown    time.Duration
	entityBaseMinActionCooldown time.Duration

	// Player defaults
	defaultPlayerWidth    float64
	defaultPlayerHeight   float64
	playerBaseLifeRegenMin float64
	playerBaseLifeRegenMax float64
	sumStatsLimit         int
	maxActiveLayers       int
	initialLifeFraction   float64

	// Combat / death
	respawnDuration  time.Duration
	ghostItemID      string
	collisionLifeLoss float64

	// Economy — Fountain & Sink model
	// Fountains (coin injection)
	coinItemID        string
	botSpawnCoins     int     // coins every bot carries on spawn (infinite mint)
	playerSpawnCoins  int     // coins given to player on first connect (guest wallet)
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
	doppelgangerSpawnChance     float64
	doppelgangerLifetimeMs      int
	doppelgangerSpawnRadius     float64
	doppelgangerInitialLifeFraction float64

	// Floor defaults
	defaultFloorItemID string

	// Per-entity-type visual defaults (live items, dead items, color key).
	entityDefaults map[string]EntityTypeDefaultConfig

	// Portal defaults
	portalSpawnRadius float64

	// Skill map (runtime): trigger item ID → []SkillDefinition
	skillConfig map[string][]SkillDefinition

	// Stats cache: entityID → cached entry with TTL. Invalidated when StatsDirty is set
	// or when the TTL (statsCacheTTL) expires.
	statsCache    map[string]statsCacheEntry
	statsCacheTTL time.Duration

	// Equipment rules — governs activation constraints.
	equipmentRules EquipmentRulesConfig

	// Status icon mapping — u8 ID → icon filename stem (from gRPC config).
	statusIcons []StatusIconConfig
}

type EntityTypeDefaultConfig struct {
        EntityType          string             `json:"entityType"`
        LiveItemIDs         []string           `json:"liveItemIds"`
        DeadItemIDs         []string           `json:"deadItemIds"`
        ColorKey            string             `json:"colorKey"`
        DefaultObjectLayers []ObjectLayerState `json:"defaultObjectLayers,omitempty"`
}

// StatusIconConfig maps a u8 status icon ID to a ui-icon filename stem and an
// interaction-bubble border colour.  Sent to clients in InitPayload.
type StatusIconConfig struct {
	ID          int       `json:"id"`
	IconID      string    `json:"iconId"`
	BorderColor ColorRGBA `json:"borderColor"`
}

// EquipmentRulesConfig governs which item types can be simultaneously active
// and enforces the one-active-per-type constraint.  Loaded from the proto
// EquipmentRules message via ApplyInstanceConfig.
type EquipmentRulesConfig struct {
	ActiveItemTypes map[string]bool `json:"activeItemTypes"` // set of activable item types
	OnePerType      bool            `json:"onePerType"`      // enforce one active per type
	RequireSkin     bool            `json:"requireSkin"`     // require ≥ 1 active skin
}

type ColorRGBA struct {
	R int `json:"r"`
	G int `json:"g"`
	B int `json:"b"`
	A int `json:"a"`
}

type InitPayload struct {
	GridW                     int                  `json:"gridW"`
	GridH                     int                  `json:"gridH"`
	DefaultObjectWidth        float64              `json:"defaultObjectWidth"`
	DefaultObjectHeight       float64              `json:"defaultObjectHeight"`
	CellSize                  float64              `json:"cellSize"`
	Fps                       int                  `json:"fps"`
	InterpolationMs           int                  `json:"interpolationMs"`
	AoiRadius                 float64              `json:"aoiRadius"`
	Colors          map[string]ColorRGBA      `json:"colors"`
	EntityDefaults  []EntityTypeDefaultConfig `json:"entityDefaults"`
	CameraSmoothing float64                   `json:"cameraSmoothing"`
	CameraZoom                float64              `json:"cameraZoom"`
	DefaultWidthScreenFactor  float64              `json:"defaultWidthScreenFactor"`
	DefaultHeightScreenFactor float64              `json:"defaultHeightScreenFactor"`
	DevUi                     bool                 `json:"devUi"`
	SumStatsLimit             int                  `json:"sumStatsLimit"`
	ObjectLayers              []ObjectLayerState   `json:"objectLayers"`
	Color                     ColorRGBA            `json:"color"`
	SkillMap                  map[string][]string  `json:"skillMap"`
	StatusIcons               []StatusIconConfig   `json:"statusIcons"`
}
