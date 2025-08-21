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

type ObjectState struct {
	ID          string     `json:"id"`
	Pos         Point      `json:"Pos"`
	Dims        Dimensions `json:"Dims"`
	Type        string     `json:"Type"`
	PortalLabel string     `json:"PortalLabel,omitempty"`
}

type PlayerState struct {
	ID             string          `json:"id"`
	MapID          int             `json:"MapID"`
	Pos            Point           `json:"Pos"`
	Dims           Dimensions      `json:"Dims"`
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
}

type BotState struct {
	ID                   string          `json:"id"`
	MapID                int             `json:"MapID"`
	Pos                  Point           `json:"Pos"`
	Dims                 Dimensions      `json:"Dims"`
	Path                 []PointI        `json:"path"`
	TargetPos            PointI          `json:"targetPos"`
	Direction            Direction       `json:"direction"`
	Mode                 ObjectLayerMode `json:"mode"`
	Behavior             string          `json:"behavior"` // "hostile" or "passive"
	SpawnCenter          Point           `json:"spawnCenter"`
	SpawnRadius          float64         `json:"spawnRadius"`
	AggroRange           float64         `json:"aggroRange"`
	CurrentTargetPlayer  string          `json:"-"` // player ID currently being pursued (if any)
	lastPursuitTargetPos PointI          `json:"-"` // cached player's last cell to know when to re-path
}

type PortalConfig struct {
	DestMapID       int
	DestPortalIndex int
	SpawnRadius     float64
}

type PortalState struct {
	ID           string
	Pos          Point
	Dims         Dimensions
	PortalConfig *PortalConfig
	Label        string `json:"Label"`
}

type MapState struct {
	pathfinder   *Pathfinder
	obstacles    map[string]ObjectState
	foregrounds  map[string]ObjectState
	portals      map[string]*PortalState
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
	maps           map[int]*MapState
	clients        map[string]*Client
	register       chan *Client
	unregister     chan *Client
	aoiRadius      float64
	portalHoldTime time.Duration
	playerSpeed    float64

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
}

type ColorRGBA struct {
	R int `json:"r"`
	G int `json:"g"`
	B int `json:"b"`
	A int `json:"a"`
}
