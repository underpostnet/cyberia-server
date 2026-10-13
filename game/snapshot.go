// Snapshots carry authoritative state. Passive objects omit progression fields.
package game

import (
	"math"
	"sort"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// SnapshotEntity is one entity inside a player's area of interest.
type SnapshotEntity struct {
	ID        world.EntityID    `json:"id"`
	Kind      string            `json:"kind"`
	DrawOrder world.DrawOrder   `json:"drawOrder"`
	Flags     world.EntityFlags `json:"flags"`
	world.Transform
	Direction int `json:"direction"`
	Mode      int `json:"mode"`

	Life      int `json:"life"`
	MaxLife   int `json:"maxLife"`
	RespawnIn int `json:"respawnIn"` // whole seconds, 0 when alive

	Level      int                     `json:"level,omitempty"`
	Stats      *[content.StatCount]int `json:"stats,omitempty"`
	StatsSum   int                     `json:"statsSum"`
	StatusIcon uint8                   `json:"statusIcon"`

	// Player fields: the teleport charge, 0..1, while the player stands on a portal.
	OnPortal           bool    `json:"onPortal,omitempty"`
	PortalHoldProgress float64 `json:"portalHoldProgress,omitempty"`

	// Behavior is set on bots.
	Behavior string `json:"behavior"`

	// Bot fields.
	CasterID world.EntityID `json:"casterId"`
	// ActionCode is "" for ordinary bots. The client fetches the action
	// metadata by this code through REST.
	ActionCode string `json:"actionCode"`

	// Portal fields. A negative target cell marks a random portal.
	TargetMap  string     `json:"targetMap"`
	TargetCell world.Cell `json:"targetCell"`

	// Layers carries the active layers only.
	Layers []world.ObjectLayerState `json:"layers"`
}

// SnapshotInteraction is what one bot in the AOI offers the viewing player.
type SnapshotInteraction struct {
	ID world.EntityID `json:"id"`
	// Flags enable the overlay icons and the interact-modal tabs.
	Flags uint8 `json:"flags"`
	// QuestCodes are the quests this NPC offers the viewing player.
	QuestCodes []string `json:"questCodes"`
	// QuestTalkDialogCodes is parallel to QuestCodes: entry i is non-empty
	// when QuestCodes[i] has an open talk objective this NPC answers.
	QuestTalkDialogCodes []string `json:"questTalkDialogCodes"`
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
	XP             uint64                 `json:"xp"`
	LevelXP        uint64                 `json:"levelXp"`
	NextLevelXP    uint64                 `json:"nextLevelXp"`
	BaseStats      [content.StatCount]int `json:"baseStats"`
	LayerStats     [content.StatCount]int `json:"layerStats"`
	TemporaryStats [content.StatCount]int `json:"temporaryStats"`

	MapCode string       `json:"mapCode"`
	Path    []world.Cell `json:"path"`

	CoinBalance uint32 `json:"coinBalance"`
	// Inventory carries every visible layer, active and inactive.
	Inventory []world.ObjectLayerState `json:"inventory"`
	// Vaults carries every vault of the player.
	Vaults []content.SnapshotVault `json:"vaults"`
	// Craft is the running assembly, absent when none runs.
	Craft *content.SnapshotCraft `json:"craft,omitempty"`
	// Interaction has one row per bot in the AOI that offers something.
	Interaction []SnapshotInteraction `json:"interaction"`

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

// activeLayers returns the active object layers — what the client draws.
func activeLayers(layers []world.ObjectLayerState) []world.ObjectLayerState {
	out := make([]world.ObjectLayerState, 0, len(layers))
	for _, l := range layers {
		if l.Active {
			out = append(out, l)
		}
	}
	return out
}

// setVitals writes v in wire form: whole numbers rounded up, so 0.4 life reads
// alive and a respawn 0.4 s away has not happened. RespawnIn is 0 while alive.
func (e *SnapshotEntity) setVitals(v *world.Vitals) {
	e.Life = int(math.Ceil(v.Life))
	e.MaxLife = int(math.Ceil(v.MaxLife))
	e.RespawnIn = 0
	if v.Ghost() {
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
func baseEntity(ms *world.MapState, id world.EntityID) SnapshotEntity {
	row := ms.Row(id)
	motion := world.Motion{Direction: world.NONE, Mode: world.IDLE}
	if row.Mask&world.MaskMotion != 0 {
		motion = row.Motion
	}
	return SnapshotEntity{
		ID:        id,
		Kind:      row.Data.Kind,
		DrawOrder: row.Data.DrawOrder,
		Flags:     row.Data.Flags,
		Transform: row.Transform,
		Direction: int(motion.Direction),
		Mode:      int(motion.Mode),
		Layers:    activeLayers(row.Appearance.Layers),
	}
}

// ═══════════════════════════════════════════════════════════════════
// Stats sum helper
// ═══════════════════════════════════════════════════════════════════

func sumStats(cs content.ComputedStats) int {
	total := 0.0
	for _, value := range cs.Values() {
		total += value
	}
	return int(total)
}

// wholeStats is the wire form of stats: every stat source is whole, so this
// only drops the float type.
func wholeStats(cs content.ComputedStats) [content.StatCount]int {
	var out [content.StatCount]int
	for i, v := range cs.Values() {
		out[i] = int(math.Round(v))
	}
	return out
}

func (s *GameServer) snapshotStats(snapshot *SnapshotEntity, ms *world.MapState) {
	stats := s.CalculateStats(snapshot.ID, ms)
	values := wholeStats(stats)
	snapshot.Level = s.EntityProgression(snapshot.ID, ms).Level
	snapshot.Stats = &values
	snapshot.StatsSum = sumStats(stats)
}

// snapshotPortalHold sets the teleport charge of a player on a portal: the fraction of
// portalHoldTime elapsed. checkPortal clears OnPortal for a ghost.
func (s *GameServer) snapshotPortalHold(e *SnapshotEntity, player *world.Session) {
	e.OnPortal = player.OnPortal
	if player.OnPortal && s.portalHoldTime > 0 {
		e.PortalHoldProgress = min(max(time.Since(player.TimeOnPortal).Seconds()/s.portalHoldTime.Seconds(), 0), 1)
	}
}

// ═══════════════════════════════════════════════════════════════════
// Snapshot builder
// ═══════════════════════════════════════════════════════════════════

// buildSnapshot collects everything inside the player's area of interest.
func (s *GameServer) buildSnapshot(player *world.Session, mapState *world.MapState) Snapshot {
	snap := Snapshot{
		Frame:                player.FrameAt(s.currentTick),
		InputConsumedThrough: player.InputConsumedThrough,
		Entities:             make([]SnapshotEntity, 0, 64),
	}

	inAOI := func(id world.EntityID) bool {
		tf := mapState.Tf(id)
		return world.RectsOverlap(player.AOI, world.Rectangle{
			MinX: tf.Pos.X, MinY: tf.Pos.Y,
			MaxX: tf.Pos.X + tf.Size.X, MaxY: tf.Pos.Y + tf.Size.Y,
		})
	}

	// Players
	for op := range s.PlayersOn(mapState) {
		if op.ID == player.ID || !inAOI(op.ID) {
			continue
		}
		e := baseEntity(mapState, op.ID)
		e.setVitals(mapState.Vt(op.ID))
		s.snapshotStats(&e, mapState)
		s.snapshotPortalHold(&e, op)
		e.StatusIcon = content.PlayerStatusIcon(op, mapState.Vt(op.ID))
		snap.Entities = append(snap.Entities, e)
	}

	// Scenery. The client draws floors in wire order, so floors are sorted:
	// layered floors first (background), then solid-colour floors on top, then
	// Y→X→ID for stability.
	type floor struct {
		e       SnapshotEntity
		layered bool // holds a layer, active or not
	}
	var floors []floor
	for id := range mapState.OfKind(world.EntityScenery) {
		if !inAOI(id) {
			continue
		}
		e := baseEntity(mapState, id)
		if e.DrawOrder == world.DrawFloor {
			floors = append(floors, floor{e, len(mapState.Ap(id).Layers) > 0})
			continue
		}
		// A destructible obstacle adds its vitals; an ordinary obstacle sends maxLife 0.
		if mapState.Has(id, world.MaskVitals) {
			e.setVitals(mapState.Vt(id))
		}
		snap.Entities = append(snap.Entities, e)
	}
	sort.Slice(floors, func(a, b int) bool {
		i, j := &floors[a], &floors[b]
		if i.layered != j.layered {
			return i.layered // layered floors drawn first (underneath)
		}
		pi, pj := i.e.Pos, j.e.Pos
		if pi.Y != pj.Y {
			return pi.Y < pj.Y
		}
		if pi.X != pj.X {
			return pi.X < pj.X
		}
		return i.e.ID < j.e.ID
	})
	for _, f := range floors {
		snap.Entities = append(snap.Entities, f.e)
	}

	// Portals
	for p := range mapState.OfKind(world.EntityPortal) {
		if !inAOI(p) {
			continue
		}
		e := baseEntity(mapState, p)
		if target, linked := mapState.Pt(p); linked {
			e.TargetMap = target.TargetMap
			e.TargetCell = target.TargetCell
		}
		// A negative target cell marks a random portal: it carries the
		// 'portal-random' icon and the client omits the cell from the label.
		e.StatusIcon = content.StatusPortal
		if e.TargetCell.X < 0 || e.TargetCell.Y < 0 {
			e.StatusIcon = content.StatusPortalRandom
		}
		snap.Entities = append(snap.Entities, e)
	}

	// Resources — static exploitable entities
	for r := range mapState.OfKind(world.EntityResource) {
		if !inAOI(r) {
			continue
		}
		e := baseEntity(mapState, r)
		e.setVitals(mapState.Vt(r))
		s.snapshotStats(&e, mapState)
		e.StatusIcon = content.ResourceStatusIcon(mapState.Vt(r))
		snap.Entities = append(snap.Entities, e)
	}

	// Bots
	var interaction []SnapshotInteraction
	for b := range mapState.OfKind(world.EntityBot) {
		if !inAOI(b) {
			continue
		}
		brain := mapState.Br(b)
		e := baseEntity(mapState, b)
		e.setVitals(mapState.Vt(b))
		s.snapshotStats(&e, mapState)
		e.StatusIcon = content.BotStatusIcon(mapState.Ag(b), mapState.Vt(b))
		e.Behavior = mapState.Ag(b).Behavior
		e.CasterID = mapState.Ag(b).CasterID
		// The action code is location-scoped and position-independent. The
		// client fetches the metadata by code; quests come from the quest API
		// by the bot's binding cell.
		e.ActionCode = mapState.Ag(b).ActionCode
		// Quests resolve per viewing player and per cell. QuestCodes carries
		// every surfaced quest for the interact modal; the capability bits
		// light only for actionable content.
		row := SnapshotInteraction{ID: b, QuestCodes: s.BotQuestCodes(player, b, mapState)}
		row.QuestTalkDialogCodes = s.PendingActionTalkDialogs(player, b, mapState, row.QuestCodes)
		hasPendingTalk := false
		for _, d := range row.QuestTalkDialogCodes {
			if d != "" {
				hasPendingTalk = true
				break
			}
		}
		row.Flags = s.BotInteractionFlags(s.BotHasActionableQuest(player, b, mapState),
			hasPendingTalk, s.BotHasUsableAction(b, mapState))
		// Loot eligibility is personal: the token's collection policy decides who
		// may collect it, and the client colours the particles by this bit.
		if mapState.Ag(b).Behavior == content.BehaviorDrop && content.LootAdmits(brain, player.ID) {
			row.Flags |= content.InteractionFlagLootEligible
		}
		if row.Flags != 0 || len(row.QuestCodes) > 0 {
			interaction = append(interaction, row)
		}
		snap.Entities = append(snap.Entities, e)
	}

	snap.Self = s.buildSnapshotSelf(player, mapState)
	snap.Self.Interaction = interaction
	return snap
}

// buildSnapshotSelf fills the viewing player's own block.
func (s *GameServer) buildSnapshotSelf(player *world.Session, mapState *world.MapState) SnapshotSelf {
	stats := s.CalculateStats(player.ID, mapState)
	breakdown := s.StatBreakdown(player.ID, mapState, make(map[world.EntityID]bool))
	progression := s.EntityProgression(player.ID, mapState)

	self := SnapshotSelf{
		SnapshotEntity: baseEntity(mapState, player.ID),
		AoiMinX:        player.AOI.MinX,
		AoiMinY:        player.AOI.MinY,
		AoiMaxX:        player.AOI.MaxX,
		AoiMaxY:        player.AOI.MaxY,
		XP:             progression.XP,
		LevelXP:        s.ProgressionConfig().Threshold(progression.Level),
		NextLevelXP:    s.ProgressionConfig().Threshold(progression.Level + 1),
		BaseStats:      wholeStats(breakdown.Base),
		LayerStats:     wholeStats(breakdown.Layers),
		TemporaryStats: wholeStats(breakdown.Temporary),

		MapCode: player.MapCode,
		Path:    player.Path,
		// player.Coins is the canonical flat balance — read it directly.
		CoinBalance:      player.Coins,
		Vaults:           s.PlayerVaults(player),
		Craft:            s.SnapshotCraft(player),
		Inventory:        s.VisibleInventory(mapState.Ap(player.ID).Layers),
		Frozen:           player.Frozen,
		MoveSpeed:        s.CalculatePlayerMovementSpeed(stats),
		ActionCooldownMs: int(s.CalculateActionCooldown(stats) / time.Millisecond),
		ActionReadyInMs:  msUntil(mapState.Mt(player.ID).NextSkillAt),
		ConsumeReadyInMs: msUntil(mapState.Mt(player.ID).NextConsumeAt),
	}
	self.setVitals(mapState.Vt(player.ID))
	s.snapshotStats(&self.SnapshotEntity, mapState)
	s.snapshotPortalHold(&self.SnapshotEntity, player)
	self.StatusIcon = content.PlayerStatusIcon(player, mapState.Vt(player.ID))
	return self
}
