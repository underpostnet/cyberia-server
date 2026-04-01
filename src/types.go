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
	ID   string     `json:"id"`
	Pos  Point      `json:"Pos"`
	Dims Dimensions `json:"Dims"`
	Type string     `json:"Type"`
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
	StatsDirty             bool               `json:"-"` // Set true when ObjectLayers change; cleared by CalculateStats cache.
}

type FloorState struct {
	ID           string             `json:"id"`
	Pos          Point              `json:"Pos"`
	Dims         Dimensions         `json:"Dims"`
	Type         string             `json:"Type"`
	ObjectLayers []ObjectLayerState `json:"objectLayers"`
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
	StatsDirty             bool               `json:"-"` // Set true when ObjectLayers change; cleared by CalculateStats cache.
}

type PortalConfig struct {
	DestMapCode string
	SpawnRadius float64
}

type PortalState struct {
	ID           string        `json:"id"`
	Pos          Point         `json:"Pos"`
	Dims         Dimensions    `json:"Dims"`
	Type         string        `json:"Type"`
	PortalConfig *PortalConfig `json:"-"`
	Label        string        `json:"PortalLabel"`
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
	atlasDataCache       map[string]*AtlasData // keyed by item key

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
	defaultPlayerObjectLayers []ObjectLayerState

	// Combat / death
	respawnDuration  time.Duration
	ghostItemID      string
	collisionLifeLoss float64

	// Economy
	coinItemID          string
	defaultCoinQuantity int

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
}

type EntityTypeDefaultConfig struct {
        EntityType  string   `json:"entityType"`
        LiveItemIDs []string `json:"liveItemIds"`
        DeadItemIDs []string `json:"deadItemIds"`
        ColorKey    string   `json:"colorKey"`
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
}
