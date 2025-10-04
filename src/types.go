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
	ID          string     `json:"id"`
	Pos         Point      `json:"Pos"`
	Dims        Dimensions `json:"Dims"`
	Type        string     `json:"Type"`
	PortalLabel string     `json:"PortalLabel,omitempty"`
}

type PlayerState struct {
	ID                     string             `json:"id"`
	MapID                  int                `json:"MapID"`
	Pos                    Point              `json:"Pos"`
	Dims                   Dimensions         `json:"Dims"`
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
	MapID                  int                `json:"MapID"`
	Pos                    Point              `json:"Pos"`
	Dims                   Dimensions         `json:"Dims"`
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
}

type PortalConfig struct {
	DestMapID       int
	DestPortalIndex int
	SpawnRadius     float64
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
	pathfinder   *Pathfinder
	obstacles    map[string]ObjectState
	foregrounds  map[string]ObjectState
	portals      map[string]*PortalState
	floors       map[string]*FloorState
	players      map[string]*PlayerState
	bots         map[string]*BotState
	gridW, gridH int
}

// AOI (Area of Interest) update payload structures
// These structs ensure correct JSON marshaling with field tags.

type VisiblePlayer struct {
	ID           string             `json:"id"`
	Pos          Point              `json:"Pos"`
	Dims         Dimensions         `json:"Dims"`
	Type         string             `json:"Type"`
	Direction    Direction          `json:"direction"`
	Mode         ObjectLayerMode    `json:"mode"`
	ObjectLayers []ObjectLayerState `json:"objectLayers"`
	Life         float64            `json:"life"`
	MaxLife      float64            `json:"maxLife"`
	RespawnIn    *float64           `json:"respawnIn,omitempty"`
}

type VisibleBot struct {
	ID           string             `json:"id"`
	Pos          Point              `json:"Pos"`
	Dims         Dimensions         `json:"Dims"`
	Type         string             `json:"Type"`
	Behavior     string             `json:"behavior"`
	Direction    Direction          `json:"direction"`
	Mode         ObjectLayerMode    `json:"mode"`
	Life         float64            `json:"life"`
	MaxLife      float64            `json:"maxLife"`
	ObjectLayers []ObjectLayerState `json:"objectLayers"`
	RespawnIn    *float64           `json:"respawnIn,omitempty"`
}

type VisibleFloor struct {
	ID           string             `json:"id"`
	Pos          Point              `json:"Pos"`
	Dims         Dimensions         `json:"Dims"`
	Type         string             `json:"Type"`
	ObjectLayers []ObjectLayerState `json:"objectLayers"`
}

type PlayerObject struct {
	ID             string             `json:"id"`
	MapID          int                `json:"MapID"`
	Pos            Point              `json:"Pos"`
	Dims           Dimensions         `json:"Dims"`
	Path           []PointI           `json:"path"`
	TargetPos      PointI             `json:"targetPos"`
	AOI            Rectangle          `json:"AOI"`
	Direction      Direction          `json:"direction"`
	Mode           ObjectLayerMode    `json:"mode"`
	OnPortal       bool               `json:"onPortal"`
	ActivePortalID string             `json:"activePortalID"`
	Life           float64            `json:"life"`
	MaxLife        float64            `json:"maxLife"`
	SumStatsLimit  int                `json:"sumStatsLimit"`
	ObjectLayers   []ObjectLayerState `json:"objectLayers"`
	RespawnIn      *float64           `json:"respawnIn,omitempty"`
}

type AOIUpdatePayload struct {
	PlayerID           string                   `json:"playerID"`
	Player             PlayerObject             `json:"player"`
	VisiblePlayers     map[string]VisiblePlayer `json:"visiblePlayers"`
	VisibleGridObjects map[string]interface{}   `json:"visibleGridObjects"`
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
	maps           map[int]*MapState
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
	botsPerMap    int
	botAggroRange float64

	// Caches
	objectLayerDataCache map[string]*ObjectLayer

	entityBaseSpeed             float64
	entityBaseMaxLife           float64
	entityBaseActionCooldown    time.Duration
	entityBaseMinActionCooldown time.Duration
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
	Colors                    map[string]ColorRGBA `json:"colors"`
	CameraSmoothing           float64              `json:"cameraSmoothing"`
	CameraZoom                float64              `json:"cameraZoom"`
	DefaultWidthScreenFactor  float64              `json:"defaultWidthScreenFactor"`
	DefaultHeightScreenFactor float64              `json:"defaultHeightScreenFactor"`
	DevUi                     bool                 `json:"devUi"`
	SumStatsLimit             int                  `json:"sumStatsLimit"`
	ObjectLayers              []ObjectLayerState   `json:"objectLayers"`
}
