// Snapshots carry authoritative state. Passive objects omit progression fields.
package game

import (
	"math"
	"sort"
	"time"
)

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

// SnapshotEntity is one entity inside a player's area of interest.
type SnapshotEntity struct {
	ID        EntityID    `json:"id"`
	Kind      string      `json:"kind"`
	DrawOrder DrawOrder   `json:"drawOrder"`
	Flags     EntityFlags `json:"flags"`
	Transform
	Direction int `json:"direction"`
	Mode      int `json:"mode"`

	Life      int `json:"life"`
	MaxLife   int `json:"maxLife"`
	RespawnIn int `json:"respawnIn"` // whole seconds, 0 when alive

	Level      int             `json:"level,omitempty"`
	Stats      *[StatCount]int `json:"stats,omitempty"`
	StatsSum   int             `json:"statsSum"`
	StatusIcon uint8           `json:"statusIcon"`

	// Player fields: the teleport charge, 0..1, while the player stands on a portal.
	OnPortal           bool    `json:"onPortal,omitempty"`
	PortalHoldProgress float64 `json:"portalHoldProgress,omitempty"`

	// Behavior is set on bots.
	Behavior string `json:"behavior"`

	// Bot fields.
	CasterID EntityID `json:"casterId"`
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

	// Layers carries the active layers only.
	Layers []ObjectLayerState `json:"layers"`
}

// SnapshotSelf is the viewing player's own block. It adds the fields only
// the owner may read.
type SnapshotSelf struct {
	SnapshotEntity

	AoiMinX float64 `json:"aoiMinX"`
	AoiMinY float64 `json:"aoiMinY"`
	AoiMaxX float64 `json:"aoiMaxX"`
	AoiMaxY float64 `json:"aoiMaxY"`

	// XP is total. LevelXP and NextLevelXP are the thresholds that bound the
	// current level, so the client can draw in-level progress without the curve.
	XP             uint64         `json:"xp"`
	LevelXP        uint64         `json:"levelXp"`
	NextLevelXP    uint64         `json:"nextLevelXp"`
	BaseStats      [StatCount]int `json:"baseStats"`
	LayerStats     [StatCount]int `json:"layerStats"`
	TemporaryStats [StatCount]int `json:"temporaryStats"`

	MapCode string `json:"mapCode"`
	Path    []Cell `json:"path"`

	CoinBalance uint32 `json:"coinBalance"`
	// Inventory carries every visible layer, active and inactive.
	Inventory []ObjectLayerState `json:"inventory"`

	Frozen bool `json:"frozen"`
	// MoveSpeed is cells per second. The client prediction integrator uses
	// it directly, so its step formula matches phaseMovement.
	MoveSpeed float64 `json:"moveSpeed"`
	// ActionCooldownMs gates skills and regeneration, independently of movement.
	ActionCooldownMs int `json:"actionCooldownMs"`
	// ActionReadyInMs is the time left on that cooldown, 0 when the next tap fires skills.
	ActionReadyInMs int `json:"actionReadyInMs"`
	// ConsumeReadyInMs is the time left on the consume cooldown, 0 when a held item may be consumed.
	ConsumeReadyInMs int `json:"consumeReadyInMs"`
}

// Snapshot is the `snapshot` message payload.
type Snapshot struct {
	// Frame is the client step this state stands for. 0 before the first
	// consumed event.
	Frame uint32 `json:"frame"`
	// InputConsumedThrough is the highest input sequence a tick has applied
	// for this player. The client drops consumed events from its replay list.
	InputConsumedThrough uint32           `json:"inputConsumedThrough"`
	Entities             []SnapshotEntity `json:"entities"`
	Self                 SnapshotSelf     `json:"self"`
}

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

// setVitals writes v in wire form: whole numbers rounded up, so 0.4 life reads
// alive and a respawn 0.4 s away has not happened. RespawnIn is 0 while alive.
func (e *SnapshotEntity) setVitals(v *Vitals) {
	e.Life = int(math.Ceil(v.Life))
	e.MaxLife = int(math.Ceil(v.MaxLife))
	e.RespawnIn = 0
	if v.ghost() {
		e.RespawnIn = max(0, int(math.Ceil(time.Until(v.RespawnAt).Seconds())))
	}
}

// msUntil returns the whole milliseconds left until t, rounded up, or 0 once
// t has passed.
func msUntil(t time.Time) int {
	left := time.Until(t)
	if left <= 0 {
		return 0
	}
	return int((left + time.Millisecond - 1) / time.Millisecond)
}

// baseEntity fills the fields every entity carries: its data, transform,
// motion and active layers. An entity with no motion faces NONE and idles.
func baseEntity(ms *MapState, id EntityID) SnapshotEntity {
	slot := ms.slot(id)
	data := ms.data[slot]
	motion := Motion{Direction: NONE, Mode: IDLE}
	if ms.mask[slot]&maskMotion != 0 {
		motion = ms.motion[slot]
	}
	return SnapshotEntity{
		ID:        id,
		Kind:      data.Kind,
		DrawOrder: data.DrawOrder,
		Flags:     data.Flags,
		Transform: ms.transform[slot],
		Direction: int(motion.Direction),
		Mode:      int(motion.Mode),
		Layers:    activeLayers(ms.appearance[slot].Layers),
	}
}

// ═══════════════════════════════════════════════════════════════════
// Stats sum helper
// ═══════════════════════════════════════════════════════════════════

func sumStats(cs ComputedStats) int {
	total := 0.0
	for _, value := range cs.Values() {
		total += value
	}
	return int(total)
}

// wholeStats is the wire form of stats: every stat source is whole, so this
// only drops the float type.
func wholeStats(cs ComputedStats) [StatCount]int {
	var out [StatCount]int
	for i, v := range cs.Values() {
		out[i] = int(math.Round(v))
	}
	return out
}

func (s *GameServer) snapshotStats(snapshot *SnapshotEntity, entity statSource, ms *MapState) {
	stats := s.CalculateStats(entity, ms)
	values := wholeStats(stats)
	snapshot.Level = s.entityProgression(entity).Level
	snapshot.Stats = &values
	snapshot.StatsSum = sumStats(stats)
}

// snapshotPortalHold sets the teleport charge of a player on a portal: the fraction of
// portalHoldTime elapsed. checkPortal clears OnPortal for a ghost.
func (s *GameServer) snapshotPortalHold(e *SnapshotEntity, player *PlayerState) {
	e.OnPortal = player.OnPortal
	if player.OnPortal && s.portalHoldTime > 0 {
		e.PortalHoldProgress = min(max(time.Since(player.TimeOnPortal).Seconds()/s.portalHoldTime.Seconds(), 0), 1)
	}
}

// ═══════════════════════════════════════════════════════════════════
// Snapshot builder
// ═══════════════════════════════════════════════════════════════════

// buildSnapshot collects everything inside the player's area of interest.
func (s *GameServer) buildSnapshot(player *PlayerState, mapState *MapState) Snapshot {
	snap := Snapshot{
		Frame:                player.frameAt(s.currentTick),
		InputConsumedThrough: player.InputConsumedThrough,
		Entities:             make([]SnapshotEntity, 0, 64),
	}

	inAOI := func(id EntityID) bool {
		tf := mapState.tf(id)
		return rectsOverlap(player.AOI, Rectangle{
			MinX: tf.Pos.X, MinY: tf.Pos.Y,
			MaxX: tf.Pos.X + tf.Size.X, MaxY: tf.Pos.Y + tf.Size.Y,
		})
	}

	// Players
	for _, op := range mapState.players {
		if op.ID == player.ID || !inAOI(op.ID) {
			continue
		}
		e := baseEntity(mapState, op.ID)
		e.setVitals(mapState.vt(op.ID))
		s.snapshotStats(&e, op, mapState)
		s.snapshotPortalHold(&e, op)
		e.StatusIcon = PlayerStatusIcon(op, mapState.vt(op.ID))
		snap.Entities = append(snap.Entities, e)
	}

	// Scenery. The client draws floors in wire order, so floors are sorted:
	// layered floors first (background), then solid-colour floors on top, then
	// Y→X→ID for stability.
	var floors []int
	for i, id := range mapState.ids {
		if id == 0 || mapState.data[i].Kind != EntityScenery || !inAOI(id) {
			continue
		}
		if mapState.data[i].DrawOrder == DrawFloor {
			floors = append(floors, i)
			continue
		}
		e := baseEntity(mapState, id)
		// A destructible obstacle adds its vitals; an ordinary obstacle sends maxLife 0.
		if mapState.mask[i]&maskVitals != 0 {
			e.setVitals(&mapState.vitals[i])
		}
		snap.Entities = append(snap.Entities, e)
	}
	sort.Slice(floors, func(a, b int) bool {
		i, j := floors[a], floors[b]
		iOL := len(mapState.appearance[i].Layers) > 0
		jOL := len(mapState.appearance[j].Layers) > 0
		if iOL != jOL {
			return iOL // layered floors drawn first (underneath)
		}
		pi, pj := mapState.transform[i].Pos, mapState.transform[j].Pos
		if pi.Y != pj.Y {
			return pi.Y < pj.Y
		}
		if pi.X != pj.X {
			return pi.X < pj.X
		}
		return mapState.ids[i] < mapState.ids[j]
	})
	for _, i := range floors {
		snap.Entities = append(snap.Entities, baseEntity(mapState, mapState.ids[i]))
	}

	// Portals
	for _, p := range mapState.portals {
		if !inAOI(p.ID) {
			continue
		}
		e := baseEntity(mapState, p.ID)
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

	// Resources — static exploitable entities
	for _, r := range mapState.resources {
		if !inAOI(r.ID) {
			continue
		}
		e := baseEntity(mapState, r.ID)
		e.setVitals(mapState.vt(r.ID))
		s.snapshotStats(&e, r, mapState)
		e.StatusIcon = ResourceStatusIcon(mapState.vt(r.ID))
		snap.Entities = append(snap.Entities, e)
	}

	// Bots
	for _, b := range mapState.bots {
		if !inAOI(b.ID) {
			continue
		}
		e := baseEntity(mapState, b.ID)
		e.setVitals(mapState.vt(b.ID))
		s.snapshotStats(&e, b, mapState)
		e.StatusIcon = BotStatusIcon(mapState.ag(b.ID), mapState.vt(b.ID))
		e.Behavior = mapState.ag(b.ID).Behavior
		e.CasterID = mapState.ag(b.ID).CasterID
		// The action code is location-scoped and position-independent. The
		// client fetches the metadata by code; quests come from the quest API
		// by the bot's binding cell.
		e.ActionCode = mapState.ag(b.ID).ActionCode
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
		// Loot eligibility is personal: the token's collection policy decides who
		// may collect it, and the client colours the particles by this bit.
		if mapState.ag(b.ID).Behavior == BehaviorDrop && lootAdmits(b, player.ID) {
			e.InteractionFlags |= InteractionFlagLootEligible
		}
		snap.Entities = append(snap.Entities, e)
	}

	snap.Self = s.buildSnapshotSelf(player, mapState)
	return snap
}

// buildSnapshotSelf fills the viewing player's own block.
func (s *GameServer) buildSnapshotSelf(player *PlayerState, mapState *MapState) SnapshotSelf {
	stats := s.CalculateStats(player, mapState)
	breakdown := s.statBreakdown(player, mapState, time.Now(), make(map[EntityID]bool))
	progression := s.entityProgression(player)

	self := SnapshotSelf{
		SnapshotEntity: baseEntity(mapState, player.ID),
		AoiMinX:        player.AOI.MinX,
		AoiMinY:        player.AOI.MinY,
		AoiMaxX:        player.AOI.MaxX,
		AoiMaxY:        player.AOI.MaxY,
		XP:             progression.XP,
		LevelXP:        s.progressionConfig().Threshold(progression.Level),
		NextLevelXP:    s.progressionConfig().Threshold(progression.Level + 1),
		BaseStats:      wholeStats(breakdown.Base),
		LayerStats:     wholeStats(breakdown.Layers),
		TemporaryStats: wholeStats(breakdown.Temporary),

		MapCode: player.MapCode,
		Path:    player.Path,
		// player.Coins is the canonical flat balance — read it directly.
		CoinBalance:      player.Coins,
		Inventory:        s.visibleInventory(mapState.ap(player.ID).Layers),
		Frozen:           player.Frozen,
		MoveSpeed:        s.CalculatePlayerMovementSpeed(stats),
		ActionCooldownMs: int(s.CalculateActionCooldown(stats) / time.Millisecond),
		ActionReadyInMs:  msUntil(player.NextSkillAt),
		ConsumeReadyInMs: msUntil(player.NextConsumeAt),
	}
	self.setVitals(mapState.vt(player.ID))
	s.snapshotStats(&self.SnapshotEntity, player, mapState)
	s.snapshotPortalHold(&self.SnapshotEntity, player)
	self.StatusIcon = PlayerStatusIcon(player, mapState.vt(player.ID))
	return self
}
