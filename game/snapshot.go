// Package game — snapshot.go
//
// The AOI snapshot: what one player sees this tick. buildSnapshot walks the
// map and fills structs; package serial turns them into JSON bytes.
//
// One SnapshotEntity shape covers all eight entity types. JSON names each
// key, so a type that does not use a field ships it as zero. No omitempty
// anywhere: the client must read `mode: 0`, `life: 0` and `posX: 0` as
// values, not as missing data.
package game

import (
	"math"
	"sort"
	"time"
)

// Entity type names on the wire.
const (
	EntityPlayer     = "player"
	EntityBot        = "bot"
	EntityFloor      = "floor"
	EntityObstacle   = "obstacle"
	EntityPortal     = "portal"
	EntityForeground = "foreground"
	EntityResource   = "resource"
	EntityStatic     = "static"
)

// Floating combat text kinds.
const (
	FCTDamage = "damage" // life loss — red "-N"
	FCTRegen  = "regen"  // life gain — green "+N"
)

// SnapshotEntity is one entity inside a player's area of interest.
type SnapshotEntity struct {
	Type      string  `json:"type"`
	ID        string  `json:"id"`
	PosX      float64 `json:"posX"`
	PosY      float64 `json:"posY"`
	DimW      float64 `json:"dimW"`
	DimH      float64 `json:"dimH"`
	Direction int     `json:"direction"`
	Mode      int     `json:"mode"`

	Life      float64 `json:"life"`
	MaxLife   float64 `json:"maxLife"`
	RespawnIn float64 `json:"respawnIn"` // seconds, 0 when alive

	StatsSum   int   `json:"statsSum"`
	StatusIcon uint8 `json:"statusIcon"`

	// Bot fields.
	Behavior string `json:"behavior"`
	CasterID string `json:"casterId"`
	// InteractionFlags resolve per viewing player. Each bit enables an
	// overlay icon and its interact-modal tab.
	InteractionFlags uint8 `json:"interactionFlags"`
	// ActionCode is "" for ordinary bots. The client fetches the action
	// metadata by this code through REST.
	ActionCode string `json:"actionCode"`
	// QuestCodes are the quests this NPC offers the viewing player.
	QuestCodes []string `json:"questCodes"`
	// QuestTalkDialogCodes is parallel to QuestCodes: entry i is non-empty
	// when QuestCodes[i] has an open talk objective this NPC answers.
	QuestTalkDialogCodes []string `json:"questTalkDialogCodes"`

	// Portal fields. A negative target cell marks a random portal.
	TargetMapCode string `json:"targetMapCode"`
	TargetCellX   int    `json:"targetCellX"`
	TargetCellY   int    `json:"targetCellY"`

	// ObjectLayers carries the active layers only.
	ObjectLayers []ObjectLayerState `json:"objectLayers"`
}

// SnapshotSelf is the viewing player's own block. It adds the fields only
// the owner may read.
type SnapshotSelf struct {
	SnapshotEntity

	AoiMinX float64 `json:"aoiMinX"`
	AoiMinY float64 `json:"aoiMinY"`
	AoiMaxX float64 `json:"aoiMaxX"`
	AoiMaxY float64 `json:"aoiMaxY"`

	OnPortal       bool `json:"onPortal"`
	SumStatsLimit  int  `json:"sumStatsLimit"`
	ActiveStatsSum int  `json:"activeStatsSum"`

	MapCode        string   `json:"mapCode"`
	Path           []PointI `json:"path"`
	TargetPosX     int      `json:"targetPosX"`
	TargetPosY     int      `json:"targetPosY"`
	ActivePortalID string   `json:"activePortalId"`

	CoinBalance uint32 `json:"coinBalance"`
	// Inventory carries every visible layer, active and inactive.
	Inventory []ObjectLayerState `json:"inventory"`

	Frozen bool `json:"frozen"`
	// MoveSpeed is cells per second. The client prediction integrator uses
	// it directly, so its step formula matches phaseMovement.
	MoveSpeed float64 `json:"moveSpeed"`
	// ActionCooldownMs is the effective action cooldown for this player. It
	// does not gate movement — that re-plans every tick — but every tap fires
	// skills, so it is the cadence at which repeating an input is worth
	// anything. Keyboard steering paces its refresh by it; sending it keeps
	// the Utility stat reduction in one place.
	ActionCooldownMs int `json:"actionCooldownMs"`
	// PortalHoldProgress is the authoritative teleport charge, 0..1.
	PortalHoldProgress float64 `json:"portalHoldProgress"`
}

// Snapshot is the `snapshot` message payload.
type Snapshot struct {
	Tick uint32 `json:"tick"`
	// Ack is the highest input sequence received for this player. The client
	// drops acknowledged commands from its prediction buffer.
	Ack uint32 `json:"ack"`
	// MoveAck is the highest PLAYER_ACTION sequence that re-planned movement.
	// Ack only proves arrival: taps landing in the same tick are all
	// acknowledged, but only the newest is planned, so self.path and
	// self.targetPos can still describe an earlier command. The client adopts
	// the authoritative route only once MoveAck covers its newest command, so a
	// rapid change of direction can never pull prediction onto a stale route.
	MoveAck  uint32           `json:"moveAck"`
	Entities []SnapshotEntity `json:"entities"`
	Self     SnapshotSelf     `json:"self"`
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
	DropID   string  `json:"dropId"`
	OriginX  float64 `json:"originX"`
	OriginY  float64 `json:"originY"`
	LandingX float64 `json:"landingX"`
	LandingY float64 `json:"landingY"`
	LaunchMs int     `json:"launchMs"` // settle window; client animation length
	ItemID   string  `json:"itemId"`
}

// DropCollect is the `drop_collect` message payload — a token was collected.
type DropCollect struct {
	DropID      string  `json:"dropId"`
	CollectorID string  `json:"collectorId"`
	WorldX      float64 `json:"worldX"`
	WorldY      float64 `json:"worldY"`
	ItemID      string  `json:"itemId"`
}

// activeLayers returns the active object layers — what the client draws.
func activeLayers(layers []ObjectLayerState) []ObjectLayerState {
	out := make([]ObjectLayerState, 0, len(layers))
	for _, l := range layers {
		if l.Active {
			out = append(out, l)
		}
	}
	return out
}

// respawnSeconds returns the whole seconds left of a ghost's respawn timer,
// or 0 when the entity is alive.
func respawnSeconds(ghost bool, respawnTime time.Time) float64 {
	if !ghost {
		return 0
	}
	remaining := math.Ceil(time.Until(respawnTime).Seconds())
	if remaining < 0 {
		return 0
	}
	return remaining
}

// baseEntity fills the fields every entity type carries.
func baseEntity(entityType, id string, pos Point, dims Dimensions,
	dir Direction, mode ObjectLayerMode, layers []ObjectLayerState) SnapshotEntity {
	return SnapshotEntity{
		Type:         entityType,
		ID:           id,
		PosX:         pos.X,
		PosY:         pos.Y,
		DimW:         dims.Width,
		DimH:         dims.Height,
		Direction:    int(dir),
		Mode:         int(mode),
		ObjectLayers: activeLayers(layers),
	}
}

// ═══════════════════════════════════════════════════════════════════
// Stats sum helper
// ═══════════════════════════════════════════════════════════════════

// statsSum returns the clamped sum of all stat fields for any entity
// (PlayerState, BotState, or ResourceState). Players clamp to their own
// SumStatsLimit; other entities use the server-level cap.
func (s *GameServer) statsSum(entity interface{}, mapState *MapState) int {
	cs := s.CalculateStats(entity, mapState)
	sum := int(cs.Effect + cs.Resistance + cs.Agility + cs.Range + cs.Intelligence + cs.Utility)
	limit := s.sumStatsLimit
	if p, ok := entity.(*PlayerState); ok {
		limit = p.SumStatsLimit
	}
	if sum > limit {
		sum = limit
	}
	return sum
}

// ═══════════════════════════════════════════════════════════════════
// Snapshot builder
// ═══════════════════════════════════════════════════════════════════

// buildSnapshot collects everything inside the player's area of interest.
func (s *GameServer) buildSnapshot(player *PlayerState, mapState *MapState) Snapshot {
	snap := Snapshot{
		Tick:     player.LastSnapshotTick,
		Ack:      player.LastAckedInputSequence,
		MoveAck:  player.LastMovementSequence,
		Entities: make([]SnapshotEntity, 0, 64),
	}

	inAOI := func(pos Point, dims Dimensions) bool {
		return rectsOverlap(player.AOI, Rectangle{
			MinX: pos.X, MinY: pos.Y,
			MaxX: pos.X + dims.Width, MaxY: pos.Y + dims.Height,
		})
	}

	// Players
	for _, op := range mapState.players {
		if op.ID == player.ID || !inAOI(op.Pos, op.Dims) {
			continue
		}
		e := baseEntity(EntityPlayer, op.ID, op.Pos, op.Dims, op.Direction, op.Mode, op.ObjectLayers)
		e.Life = op.Life
		e.MaxLife = op.MaxLife
		e.RespawnIn = respawnSeconds(op.IsGhost(), op.RespawnTime)
		e.StatsSum = s.statsSum(op, mapState)
		e.StatusIcon = PlayerStatusIcon(op)
		snap.Entities = append(snap.Entities, e)
	}

	// Obstacles
	for _, o := range mapState.obstacles {
		if inAOI(o.Pos, o.Dims) {
			snap.Entities = append(snap.Entities,
				baseEntity(EntityObstacle, o.ID, o.Pos, o.Dims, NONE, IDLE, o.ObjectLayers))
		}
	}

	// Floors — sorted: layered floors first (background), then solid-colour
	// floors on top. Within each group sort by Y→X→ID for stability. Go maps
	// iterate at random, and overlapping floors would flicker without this.
	var aoiFloors []*FloorState
	for _, f := range mapState.floors {
		if inAOI(f.Pos, f.Dims) {
			aoiFloors = append(aoiFloors, f)
		}
	}
	sort.Slice(aoiFloors, func(i, j int) bool {
		iOL := len(aoiFloors[i].ObjectLayers) > 0
		jOL := len(aoiFloors[j].ObjectLayers) > 0
		if iOL != jOL {
			return iOL // layered floors drawn first (underneath)
		}
		if aoiFloors[i].Pos.Y != aoiFloors[j].Pos.Y {
			return aoiFloors[i].Pos.Y < aoiFloors[j].Pos.Y
		}
		if aoiFloors[i].Pos.X != aoiFloors[j].Pos.X {
			return aoiFloors[i].Pos.X < aoiFloors[j].Pos.X
		}
		return aoiFloors[i].ID < aoiFloors[j].ID
	})
	for _, f := range aoiFloors {
		snap.Entities = append(snap.Entities,
			baseEntity(EntityFloor, f.ID, f.Pos, f.Dims, NONE, IDLE, f.ObjectLayers))
	}

	// Portals
	for _, p := range mapState.portals {
		if !inAOI(p.Pos, p.Dims) {
			continue
		}
		e := baseEntity(EntityPortal, p.ID, p.Pos, p.Dims, NONE, IDLE, p.ObjectLayers)
		if p.PortalConfig != nil {
			e.TargetMapCode = p.PortalConfig.DestMapCode
			e.TargetCellX = int(p.PortalConfig.DestCellX)
			e.TargetCellY = int(p.PortalConfig.DestCellY)
		}
		// A negative target cell marks a random portal: it carries the
		// 'portal-random' icon and the client omits the cell from the label.
		e.StatusIcon = StatusPortal
		if e.TargetCellX < 0 || e.TargetCellY < 0 {
			e.StatusIcon = StatusPortalRandom
		}
		snap.Entities = append(snap.Entities, e)
	}

	// Foregrounds
	for _, fg := range mapState.foregrounds {
		if inAOI(fg.Pos, fg.Dims) {
			snap.Entities = append(snap.Entities,
				baseEntity(EntityForeground, fg.ID, fg.Pos, fg.Dims, NONE, IDLE, fg.ObjectLayers))
		}
	}

	// Resources — static exploitable entities
	for _, r := range mapState.resources {
		if !inAOI(r.Pos, r.Dims) {
			continue
		}
		e := baseEntity(EntityResource, r.ID, r.Pos, r.Dims, NONE, IDLE, r.ObjectLayers)
		e.Life = r.Life
		e.MaxLife = r.MaxLife
		e.RespawnIn = respawnSeconds(r.IsGhost(), r.RespawnTime)
		e.StatsSum = s.statsSum(r, mapState)
		e.StatusIcon = ResourceStatusIcon(r)
		snap.Entities = append(snap.Entities, e)
	}

	// Statics — non-moving, passable decorators
	for _, st := range mapState.statics {
		if inAOI(st.Pos, st.Dims) {
			snap.Entities = append(snap.Entities,
				baseEntity(EntityStatic, st.ID, st.Pos, st.Dims, NONE, IDLE, st.ObjectLayers))
		}
	}

	// Bots
	for _, b := range mapState.bots {
		if !inAOI(b.Pos, b.Dims) {
			continue
		}
		e := baseEntity(EntityBot, b.ID, b.Pos, b.Dims, b.Direction, b.Mode, b.ObjectLayers)
		e.Life = b.Life
		e.MaxLife = b.MaxLife
		e.RespawnIn = respawnSeconds(b.IsGhost(), b.RespawnTime)
		e.StatsSum = s.statsSum(b, mapState)
		e.StatusIcon = BotStatusIcon(b)
		e.Behavior = b.Behavior
		e.CasterID = b.CasterID
		// The action code is location-scoped and position-independent. The
		// client fetches the metadata by code; quests come from the quest API
		// by the bot's binding cell.
		e.ActionCode = b.ActionCode
		// Quests resolve per viewing player and per cell. QuestCodes carries
		// every surfaced quest for the interact modal; the capability bits
		// light only for actionable content.
		e.QuestCodes = s.botQuestCodes(player, b)
		e.QuestTalkDialogCodes = s.pendingActionTalkDialogs(player, b, e.QuestCodes)
		hasPendingTalk := false
		for _, d := range e.QuestTalkDialogCodes {
			if d != "" {
				hasPendingTalk = true
				break
			}
		}
		e.InteractionFlags = s.botInteractionFlags(s.botHasActionableQuest(player, b),
			hasPendingTalk, s.botHasUsableAction(b))
		// Loot eligibility is personal: only damage contributors may collect a
		// drop token, and the client colours the particles by this bit.
		if b.Behavior == BehaviorDrop {
			if _, ok := b.LootContributors[player.ID]; ok {
				e.InteractionFlags |= InteractionFlagLootEligible
			}
		}
		snap.Entities = append(snap.Entities, e)
	}

	snap.Self = s.buildSnapshotSelf(player, mapState)
	return snap
}

// buildSnapshotSelf fills the viewing player's own block.
func (s *GameServer) buildSnapshotSelf(player *PlayerState, mapState *MapState) SnapshotSelf {
	stats := s.CalculateStats(player, mapState)
	activeStatsSum := int(stats.Effect + stats.Resistance + stats.Agility +
		stats.Range + stats.Intelligence + stats.Utility)

	// Portal hold progress — the fraction of portalHoldTime elapsed while the
	// player stands on a portal. checkPortal clears OnPortal for a ghost, so
	// this is non-zero only for a live player charging a teleport.
	portalHold := 0.0
	if player.OnPortal && s.portalHoldTime > 0 {
		portalHold = time.Since(player.TimeOnPortal).Seconds() / s.portalHoldTime.Seconds()
		if portalHold > 1.0 {
			portalHold = 1.0
		} else if portalHold < 0.0 {
			portalHold = 0.0
		}
	}

	self := SnapshotSelf{
		SnapshotEntity: baseEntity(EntityPlayer, player.ID, player.Pos, player.Dims,
			player.Direction, player.Mode, player.ObjectLayers),
		AoiMinX:        player.AOI.MinX,
		AoiMinY:        player.AOI.MinY,
		AoiMaxX:        player.AOI.MaxX,
		AoiMaxY:        player.AOI.MaxY,
		OnPortal:       player.OnPortal,
		SumStatsLimit:  player.SumStatsLimit,
		ActiveStatsSum: activeStatsSum,
		MapCode:        player.MapCode,
		Path:           player.Path,
		TargetPosX:     player.TargetPos.X,
		TargetPosY:     player.TargetPos.Y,
		ActivePortalID: player.ActivePortalID,
		// player.Coins is the canonical flat balance — read it directly.
		CoinBalance:        player.Coins,
		Inventory:          s.visibleInventory(player.ObjectLayers),
		Frozen:             player.Frozen,
		MoveSpeed:          s.CalculatePlayerMovementSpeed(stats),
		ActionCooldownMs:   int(s.CalculateActionCooldown(stats) / time.Millisecond),
		PortalHoldProgress: portalHold,
	}
	self.Life = player.Life
	self.MaxLife = player.MaxLife
	self.RespawnIn = respawnSeconds(player.IsGhost(), player.RespawnTime)
	self.StatsSum = activeStatsSum
	if self.StatsSum > player.SumStatsLimit {
		self.StatsSum = player.SumStatsLimit
	}
	self.StatusIcon = PlayerStatusIcon(player)
	return self
}
