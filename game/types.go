package game

import (
	"sync"
	"time"
)

// 1. Data Structures & Interfaces

// Vec2 is a 2D quantity in grid units: a position or a size.
type Vec2 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// Transform is the transform component: the top-left and the size of an
// entity, in grid units.
type Transform struct {
	Pos  Vec2 `json:"pos"`
	Size Vec2 `json:"size"`
}

// Appearance is the appearance component: every layer the entity holds. The
// encoder sends the active ones.
type Appearance struct {
	Layers []ObjectLayerState
}

// EntityData is the data component: the kind and the small flags that do not
// earn an array of their own.
type EntityData struct {
	Kind      string
	DrawOrder DrawOrder
	Flags     EntityFlags
}

// DrawOrder places an entity in the client draw passes.
type DrawOrder uint8

const (
	DrawSorted     DrawOrder = iota // depth-sorted with the other entities
	DrawFloor                       // under every entity
	DrawForeground                  // over every entity
)

// EntityFlags are the yes/no facts of an entity.
type EntityFlags uint8

const (
	FlagBlocks            EntityFlags = 1 << iota // the pathfinder cannot cross it
	FlagOverheadOcclusion                         // fades while the own player is under it
)

// Component bits of MapState.mask, in the §4 order.
const (
	maskData       ComponentMask = 1 << 0
	maskTransform  ComponentMask = 1 << 1
	maskAppearance ComponentMask = 1 << 2
	maskVitals     ComponentMask = 1 << 3
	maskMotion     ComponentMask = 1 << 5
	maskAgent      ComponentMask = 1 << 6
	maskPortal     ComponentMask = 1 << 7
	maskPlayer     ComponentMask = 1 << 8 // a client controls the entity; no array yet
)

type Cell struct {
	X int `json:"x"`
	Y int `json:"y"`
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

// EntityID is the id of an entity on the server, on the wire and on the
// client. 0 is no entity.
type EntityID uint32

// ComponentMask holds one bit per component an entity has.
type ComponentMask uint64

// Mortal is the server-only side table row of an entity that can die (player,
// bot, resource). Life itself is in MapState.vitals.
type Mortal struct {
	NextSkillAt            time.Time
	NextConsumeAt          time.Time // ends the cooldown a consumed held item starts (consumable.go)
	BaseMaxLife            float64
	PreRespawnObjectLayers []ObjectLayerState
	LifeRegen              float64
	// RespawnDuration is the wait before the entity comes back; 0 takes respawnDurationDefaultMs.
	RespawnDuration time.Duration
	// DamageLedger is the per-victim threat/contribution ledger: attacker id →
	// cumulative damage dealt to this entity. Populated by the skill-collision
	// path for every attacker. Consumed on death to build the loot contributor
	// set and the XP shares, then reset. Allocated lazily on first hit.
	DamageLedger map[EntityID]float64
}

// Vitals is the vitals component. The server keeps the respawn instant; the
// encoder sends the seconds left.
type Vitals struct {
	Life      float64
	MaxLife   float64
	RespawnAt time.Time // zero while alive
}

// Motion is the motion component: the facing and the walk mode.
type Motion struct {
	Direction Direction
	Mode      ObjectLayerMode
}

// Agent is the agent component of a bot: what drives it and who made it.
type Agent struct {
	Behavior   string   // canonical bot behavior (see behavior.go)
	CasterID   EntityID // the player or bot that made this bot; 0 for none
	ActionCode string   // the bound CyberiaAction (see actionCache); "" for none
}

// Portal is the portal component: where a linked portal sends the player. A
// negative TargetCell marks a random spawn on the target map.
type Portal struct {
	TargetMap  string
	TargetCell Cell
}

// ghost reports whether the entity is dead and waiting to respawn.
func (v *Vitals) ghost() bool { return !v.RespawnAt.IsZero() }

// respawnDuration is the wait before the entity comes back.
func (m *Mortal) respawnDuration() time.Duration {
	if m.RespawnDuration > 0 {
		return m.RespawnDuration
	}
	return respawnDurationDefaultMs * time.Millisecond
}

// Session is the server-only side table row of a connected player: its link,
// its map, its economy, its quests and its input stream.
type Session struct {
	ID EntityID
	// EngineID is the player id the engine knows. It never goes in a snapshot.
	EngineID     string    `json:"-"`
	MapCode      string    `json:"MapCode"`
	Path         []Cell    `json:"path"`
	TargetPos    Cell      `json:"targetPos"`
	AOI          Rectangle `json:"AOI"`
	Client       Conn      `json:"-"`
	OnPortal     bool      `json:"onPortal"`
	TimeOnPortal time.Time `json:"-"`
	// Coins is the canonical flat coin balance — the single source of truth
	// for all economy operations.  O(1) read/write, never iterates ObjectLayers.
	// The coin ObjectLayer slot (coinItemID) is kept in sync for inventory
	// visualization only and is always Active: false.
	Coins uint32 `json:"-"`
	// DeadLoadoutItemIDs remembers the dead-state items active during the
	// player's last Fragmented State; reapplied on the next death. Never sent
	// on the wire.
	DeadLoadoutItemIDs []string `json:"-"`
	// FrozenInteractionState — general-purpose modal protection.
	// While Frozen, the player cannot deal or receive damage, send or
	// receive events, or move.  The rest of the world continues.
	// Managed exclusively by FreezePlayer / ThawPlayer (frozen_state.go).
	Frozen      bool      `json:"-"`
	FreezeStart time.Time `json:"-"` // [FREEZE] duration log only

	// ── Tick / replication metadata ─────────────────────────────────────────
	// InputConsumedThrough is the highest InputCommand.Sequence phaseInput has
	// applied for this player; the snapshot echoes it so the client can drop
	// consumed events from its replay list.
	InputConsumedThrough uint32 `json:"-"`
	// FrameOffset is the client step minus the server tick, from the newest
	// consumed event.
	FrameOffset int64 `json:"-"`
	// PendingMove is the destination of the newest PLAYER_ACTION drained this
	// tick. phaseInput re-plans from it once, after the whole queue is applied,
	// so a burst of taps costs one A* instead of one per tap. Cleared by the
	// flush; never persisted across ticks.
	PendingMove    Cell `json:"-"`
	HasPendingMove bool `json:"-"`
	// InputQueue holds InputCommand frames received from the client between
	// simulation ticks. Drained in fixed order by phaseInput. nil-safe: the
	// queue is allocated lazily when the first command is enqueued.
	InputQueue []InputCommand `json:"-"`
}

// Brain is the server-only side table row of a bot: its AI, its lifetime and,
// for a drop, its loot.
type Brain struct {
	Path                 []Cell    `json:"path"`
	TargetPos            Cell      `json:"targetPos"`
	SpawnCenter          Vec2      `json:"spawnCenter"`
	SpawnRadius          float64   `json:"spawnRadius"`
	AggroRange           float64   `json:"aggroRange"`
	CurrentTargetPlayer  EntityID  `json:"-"` // player ID currently being pursued (if any)
	lastPursuitTargetPos Cell      `json:"-"` // cached player's last cell to know when to re-path
	lastAction           time.Time `json:"-"`
	ExpiresAt            time.Time `json:"-"` // bots with a lifetime will be removed after this time
	// Coins is the canonical flat coin balance — the single source of truth
	// for all economy operations.  O(1) read/write, never iterates ObjectLayers.
	// The coin ObjectLayer slot (coinItemID) is kept in sync for inventory
	// visualization only and is always Active: false.
	Coins uint32 `json:"-"`

	// ── Transient world-item drop fields (BehaviorDrop bots only) ────────────
	// A drop is a collectible token: scattered on the grid when an entity dies, or
	// placed by the world build. It shows one item and is picked up by walking over it.
	// DropItemID is the item this token grants on collection.
	DropItemID string `json:"-"`
	// DropQuantity is how many units this token grants on collection. 1 for a
	// scattered inventory item; the coin amount for a coin drop (DropItemID ==
	// coinItemID). Mirrored into the token's active ObjectLayer.Quantity so the
	// client can render the on-grid counter.
	DropQuantity int `json:"-"`
	// LootContributors is the set of IDs — players and bots — that dealt damage to
	// the entity this token dropped from, tracked uniformly for every victim type
	// (player, bot, resource). Once the token settles, collection is a race among
	// contributors: a contributing player collects on collision, and a contributing
	// bot has the loot transferred to it on collision. A death drop always has ≥1
	// contributor (a contributor-less death drops nothing — see spawnDrops); a
	// world token has none.
	LootContributors map[EntityID]struct{} `json:"-"`
	// CollectableAt is the instant the spawn-launch settle window ends. Until
	// then the token is mid-flight from the corpse to its cell and registers no
	// collision — collection is disabled regardless of loot priority. Matches
	// the client launch-animation duration sent in drop_spawn.
	CollectableAt time.Time `json:"-"`
	// LootOrigin, LootCollection and LootOutcome are fixed when the token spawns (see loot.go).
	LootOrigin     LootOrigin     `json:"-"`
	LootCollection LootCollection `json:"-"`
	LootOutcome    LootOutcome    `json:"-"`
}

type MapState struct {
	code         string // the map code; the key of this map in World.maps
	pathfinder   *Pathfinder
	gridW, gridH int

	ids        []EntityID // entity id at slot i. 0 = free slot
	index      map[EntityID]int
	mask       []ComponentMask
	data       []EntityData
	transform  []Transform
	appearance []Appearance
	vitals     []Vitals
	motion     []Motion
	agent      []Agent
	portal     []Portal
	cursor     int // next slot the allocator tests

	mortals map[EntityID]*Mortal
	brains  map[EntityID]*Brain
}

// Conn is the link of a session to its client.
type Conn interface {
	Send(msgType string, payload any)
}

// Entity kinds on the wire.
const (
	EntityPlayer   = "player"
	EntityBot      = "bot"
	EntityPortal   = "portal"
	EntityResource = "resource"
	EntityScenery  = "scenery"
)

// Floating combat text kinds.
const (
	FCTDamage = "damage" // life loss — red "-N"
	FCTRegen  = "regen"  // life gain — green "+N"
	FCTXp     = "xp"     // XP award — gold "+N XP", sent to the earner only
)

// Audio logic ids the server raises. The client binds them to assets per map,
// so an id here must exist in engine-cyberia's AUDIO_LOGIC_IDS.
const (
	AudioEventDeath   = "death"
	AudioEventLevelUp = "level-up"
)

// AudioEvent is the `audio_event` message payload: a world event every viewer
// in reach hears the same way.
type AudioEvent struct {
	LogicEventID string  `json:"logicEventId"`
	WorldX       float64 `json:"worldX"`
	WorldY       float64 `json:"worldY"`
}

// CombatText is the `combat_text` message payload.
type CombatText struct {
	Kind   string  `json:"kind"`
	WorldX float64 `json:"worldX"`
	WorldY float64 `json:"worldY"`
	Value  int     `json:"value"` // always positive; the kind implies the sign
}

// DropSpawn is the `drop_spawn` message payload — a loot token leaves a corpse.
type DropSpawn struct {
	DropID   EntityID `json:"dropId"`
	OriginX  float64  `json:"originX"`
	OriginY  float64  `json:"originY"`
	LandingX float64  `json:"landingX"`
	LandingY float64  `json:"landingY"`
	LaunchMs int      `json:"launchMs"` // settle window; client animation length
	ItemID   string   `json:"itemId"`
}

// DropCollect is the `drop_collect` message payload — a token was collected.
type DropCollect struct {
	DropID      EntityID `json:"dropId"`
	CollectorID EntityID `json:"collectorId"`
	WorldX      float64  `json:"worldX"`
	WorldY      float64  `json:"worldY"`
	ItemID      string   `json:"itemId"`
	Granted     bool     `json:"granted"` // the collector received the item; false for an effect such as a heal
}

// ItemConsume tells viewers an entity consumed an item; Behavior is the loot behavior of the item.
type ItemConsume struct {
	EntityID EntityID `json:"entityId"`
	ItemID   string   `json:"itemId"`
	Behavior string   `json:"behavior"`
}

// sendMessage packs a message and queues it for the player. The send never
// blocks: a full queue drops the message.
func sendMessage(player *Session, msgType string, payload any) {
	if player != nil && player.Client != nil {
		player.Client.Send(msgType, payload)
	}
}

// sendAck answers a request with the {ok, reason} pair every ack carries, plus
// the caller's own fields. An empty reason means success.
func sendAck(player *Session, msgType, reason string, fields map[string]interface{}) {
	fields["ok"] = reason == ""
	fields["reason"] = reason
	sendMessage(player, msgType, fields)
}

// frameAt is the client step that tick stands for. 0 before the first consumed event.
func (p *Session) frameAt(tick uint32) uint32 {
	if p.InputConsumedThrough == 0 {
		return 0
	}
	return uint32(int64(tick) + p.FrameOffset)
}

// World is the entity state: every map and the session of every player.
type World struct {
	mu           sync.Mutex // guards maps and sessions
	maps         map[string]*MapState
	lastEntityID EntityID // only goes up, so a freed id never comes back
	sessions     map[EntityID]*Session
}
