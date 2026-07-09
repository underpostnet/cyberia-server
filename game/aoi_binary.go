// Package game — aoi_binary.go
//
// Minimal binary AOI protocol for WebSocket communication with the
// C/WASM client. Only render-essential data is sent: positions,
// directions, modes, life values, and item-ID stacks. The client
// fetches atlas binaries from Engine-Cyberia REST independently.
//
// Wire format (little-endian).
//
// Header (v2 — used by 0x01 aoi_update and 0x03 full_aoi):
//
//	[0]        u8   msgType  (0x01 = aoi_update, 0x02 = init_data, 0x03 = full_aoi)
//	[1..4]     u32  tick                 — simulation tick when the snapshot was produced
//	[5..8]     u32  lastAckedSequence    — highest InputCommand.Sequence applied for the
//	                                       receiving player; the client drops InputCommand
//	                                       entries with sequence ≤ this from its prediction
//	                                       buffer.
//	[9..10]    u16  entityCount (number of entity blocks that follow)
//
// 0x02 (init_data) and 0x04/0x05 (FCT) keep their pre-v2 layouts — they are
// not part of the per-tick replication stream and have no tick semantics.
//
//	Per-entity block (variable length):
//	  [0]       u8   flags
//	                  bits 0-2: type (0=player, 1=bot, 2=floor, 3=obstacle, 4=portal, 5=foreground, 6=resource, 7=static)
//	                  bit 3:    removed (entity left AOI — only type+ID follows)
//	                  bit 4:    has life data
//	                  bit 5:    has respawn timer
//	                  bit 6:    has behavior string (bots)
//	                  bit 7:    has color data (4 bytes RGBA)
//	  [1..36]   36B  id (UUID string, zero-padded)
//	  -- if removed, stop here --
//	  [37..40]  f32  posX
//	  [41..44]  f32  posY
//	  [45..48]  f32  dimW
//	  [49..52]  f32  dimH
//	  [53]      u8   direction (0-8)
//	  [54]      u8   mode (0=idle, 1=walking, 2=teleporting)
//	  -- if bit 4 (has life):
//	    f32  life
//	    f32  maxLife
//	  -- if bit 5 (has respawn):
//	    f32  respawnIn (seconds)
//	  -- if bit 6 (has behavior):
//	    u8   behaviorLen
//	    str  behavior
//	  -- if bit 7 (has color):
//	    u8   r
//	    u8   g
//	    u8   b
//	    u8   a
//	  -- item ID stack (IDs only — no active/quantity):
//	    u8   itemIdCount
//	    per item:
//	      u8   itemIdLen
//	      str  itemId
//
//	Self-player section (appended after entity blocks):
//	  Same entity block format, then:
//	    f32  aoiMinX, aoiMinY, aoiMaxX, aoiMaxY
//	    u8   onPortal (0/1)
//	    u16  sumStatsLimit
//	    u16  activeStatsSum
//	    u8+str mapCode (length-prefixed map code string)
//	    u8   pathLen
//	    per path point: i16 x, i16 y
//	    i16  targetPosX, targetPosY
//	    u8+str activePortalID
//	    u32  coinBalance
//	    full inventory (writeFullInventory)
//	    u8   frozen (0=normal, 1=frozen — FrozenInteractionState)
//	    u8   statusIcon
//	    f32  moveSpeed (cells/second — fed to client prediction integrator)
//	    f32  portalHoldProgress (0..1 — authoritative teleport charge, local player)
package game

import (
	"encoding/binary"
	"math"
	"sort"
	"sync"
	"time"
)

const (
	MsgTypeAOIUpdate byte = 0x01
	MsgTypeInitData  byte = 0x02
	MsgTypeFullAOI   byte = 0x03
	// MsgTypeFCT — Floating Combat Text event (14 bytes, little-endian).
	// Wire format:
	//   [0]      u8   0x04
	//   [1]      u8   FCTType* (see constants below)
	//   [2..5]   f32  worldX
	//   [6..9]   f32  worldY
	//   [10..13] u32  value (always positive; sign implied by type)
	MsgTypeFCT byte = 0x04
	// MsgTypeItemFCT — reserved item-quantity FCT wire slot. No longer emitted:
	// item pickups surface in the loot grid, not as floating text.
	MsgTypeItemFCT byte = 0x05
	// MsgTypeDropCollect — a scattered loot token was collected by a player.
	// Drives the client parabolic pickup animation. See loot.go
	// buildDropCollectMsg for the wire format.
	MsgTypeDropCollect byte = 0x06
	// MsgTypeDropSpawn — a scattered loot token was launched from a corpse.
	// Carries the launch origin, landing cell, and settle duration so the client
	// plays the bot→ground parabolic launch. See loot.go buildDropSpawnMsg.
	MsgTypeDropSpawn byte = 0x07

	// FCT event sub-types — MUST stay in sync with C constants in
	// floating_combat_text.h and binary_aoi_decoder.h.
	FCTTypeDamage byte = 0x00 // life loss            — red    "-N"
	FCTTypeRegen  byte = 0x01 // life gain / regen    — green  "+N"
	// Gain sub-types (0x02 CoinGain, 0x04 ItemGain) are retired: gains show in
	// the loot grid, not as floating text. Loss/damage/regen still emit.
	FCTTypeCoinLoss byte = 0x03 // coins lost / sink    — yellow "-N"

	EntityTypePlayer     byte = 0
	EntityTypeBot        byte = 1
	EntityTypeFloor      byte = 2
	EntityTypeObstacle   byte = 3
	EntityTypePortal     byte = 4
	EntityTypeForeground byte = 5
	EntityTypeResource   byte = 6
	EntityTypeStatic     byte = 7

	FlagRemoved     byte = 0x08 // bit 3
	FlagHasLife     byte = 0x10 // bit 4
	FlagHasRespawn  byte = 0x20 // bit 5
	FlagHasBehavior byte = 0x40 // bit 6
	// Bit 7 reserved (was FlagHasColor; per-entity RGBA no longer ships on
	// the wire — the client resolves colour from its presentation palette).

	maxBinaryBufSize = 64 * 1024 // 64 KB per AOI message
)

// BinaryAOIEncoder builds compact binary AOI messages.
type BinaryAOIEncoder struct {
	buf    []byte
	pos    int
	bufPtr *[]byte // non-nil when buf came from aoiBufPool; returned on release
}

// aoiBufPool recycles the maxBinaryBufSize scratch buffers used to encode AOI
// snapshots. Without it every player×snapshot-tick allocated and discarded a
// fresh 64 KiB buffer (at snapshotRate Hz × N players this dominated GC
// pressure and drove the heap-size spikes). Pointers to slices are pooled to
// avoid boxing the slice header on every Get/Put.
var aoiBufPool = sync.Pool{
	New: func() any {
		b := make([]byte, maxBinaryBufSize)
		return &b
	},
}

// NewBinaryAOIEncoder returns an encoder backed by a freshly allocated buffer.
// Prefer acquireEncoder for the per-snapshot hot path; this constructor is for
// callers that keep an encoder around.
func NewBinaryAOIEncoder() *BinaryAOIEncoder {
	return &BinaryAOIEncoder{buf: make([]byte, maxBinaryBufSize)}
}

// acquireEncoder borrows a pooled scratch buffer. The caller MUST call
// release() (typically deferred) once the encoded bytes have been copied out.
func acquireEncoder() *BinaryAOIEncoder {
	bp := aoiBufPool.Get().(*[]byte)
	return &BinaryAOIEncoder{buf: *bp, pos: 0, bufPtr: bp}
}

// release returns a pooled buffer. Idempotent; safe to defer.
func (e *BinaryAOIEncoder) release() {
	if e.bufPtr != nil {
		aoiBufPool.Put(e.bufPtr)
		e.bufPtr = nil
		e.buf = nil
	}
}

func (e *BinaryAOIEncoder) putU8(v byte) {
	e.buf[e.pos] = v
	e.pos++
}

func (e *BinaryAOIEncoder) putU16(v uint16) {
	binary.LittleEndian.PutUint16(e.buf[e.pos:], v)
	e.pos += 2
}

func (e *BinaryAOIEncoder) putU32(v uint32) {
	binary.LittleEndian.PutUint32(e.buf[e.pos:], v)
	e.pos += 4
}

func (e *BinaryAOIEncoder) putI16(v int16) {
	binary.LittleEndian.PutUint16(e.buf[e.pos:], uint16(v))
	e.pos += 2
}

func (e *BinaryAOIEncoder) putF32(v float64) {
	binary.LittleEndian.PutUint32(e.buf[e.pos:], math.Float32bits(float32(v)))
	e.pos += 4
}

func (e *BinaryAOIEncoder) putString(s string) {
	n := len(s)
	if n > 255 {
		n = 255
	}
	e.putU8(byte(n))
	copy(e.buf[e.pos:], s[:n])
	e.pos += n
}

// putStringList writes a u8 count followed by each length-prefixed string.
func (e *BinaryAOIEncoder) putStringList(items []string) {
	n := len(items)
	if n > 255 {
		n = 255
	}
	e.putU8(byte(n))
	for i := 0; i < n; i++ {
		e.putString(items[i])
	}
}

func (e *BinaryAOIEncoder) putID(id string) {
	n := len(id)
	if n > 36 {
		n = 36
	}
	copy(e.buf[e.pos:], id[:n])
	for i := n; i < 36; i++ {
		e.buf[e.pos+i] = 0
	}
	e.pos += 36
}

// writeItemIDs writes active layer item IDs with their quantities.
//
// Wire format (per active layer):
//
//	[u8-len string] itemId
//	[u16]           quantity (0 = non-stackable / not tracked)
func (e *BinaryAOIEncoder) writeItemIDs(layers []ObjectLayerState) {
	activeCount := 0
	for i := range layers {
		if layers[i].Active {
			activeCount++
		}
	}
	if activeCount > 255 {
		activeCount = 255
	}
	e.putU8(byte(activeCount))
	written := 0
	for i := range layers {
		if layers[i].Active && written < activeCount {
			e.putString(layers[i].ItemID)
			qty := layers[i].Quantity
			if qty < 0 {
				qty = 0
			}
			if qty > 65535 {
				qty = 65535
			}
			e.putU16(uint16(qty))
			written++
		}
	}
}

// writeFullInventory writes ALL ObjectLayers (active and inactive) for the
// self-player so the client can render the complete inventory bottom bar.
//
// Wire format:
//
//	[u8]            total count (all layers, active + inactive)
//	[per layer]:
//	  [u8-len str]  itemId
//	  [u8]          active flag (0 = inactive, 1 = active)
//	  [u16]         quantity
func (e *BinaryAOIEncoder) writeFullInventory(layers []ObjectLayerState) {
	n := len(layers)
	if n > 255 {
		n = 255
	}
	e.putU8(byte(n))
	for i := 0; i < n; i++ {
		e.putString(layers[i].ItemID)
		if layers[i].Active {
			e.putU8(1)
		} else {
			e.putU8(0)
		}
		qty := layers[i].Quantity
		if qty < 0 {
			qty = 0
		}
		if qty > 65535 {
			qty = 65535
		}
		e.putU16(uint16(qty))
	}
}

// writeEntityBase writes the common per-entity fields.
func (e *BinaryAOIEncoder) writeEntityBase(flags byte, id string, pos Point, dims Dimensions, dir Direction, mode ObjectLayerMode) {
	e.putU8(flags)
	e.putID(id)
	e.putF32(pos.X)
	e.putF32(pos.Y)
	e.putF32(dims.Width)
	e.putF32(dims.Height)
	e.putU8(byte(dir))
	e.putU8(byte(mode))
}

// ═══════════════════════════════════════════════════════════════════
// Entity writers
// ═══════════════════════════════════════════════════════════════════

func (e *BinaryAOIEncoder) WritePlayer(p *PlayerState, respawnIn *float64, statsSum int) {
	flags := EntityTypePlayer | FlagHasLife
	if respawnIn != nil {
		flags |= FlagHasRespawn
	}
	e.writeEntityBase(flags, p.ID, p.Pos, p.Dims, p.Direction, p.Mode)
	e.putF32(p.Life)
	e.putF32(p.MaxLife)
	if respawnIn != nil {
		e.putF32(*respawnIn)
	}
	e.writeItemIDs(p.ObjectLayers)
	e.putU16(uint16(statsSum))
	// Entity Status Indicator — u8 overhead icon ID (see entity_status.go).
	e.putU8(PlayerStatusIcon(p))
}

func (e *BinaryAOIEncoder) WriteBot(b *BotState, respawnIn *float64, statsSum int, actionCode string, statusIcon uint8, interactionFlags uint8, questCodes []string, actionDialogCode string) {
	flags := EntityTypeBot | FlagHasLife | FlagHasBehavior
	if respawnIn != nil {
		flags |= FlagHasRespawn
	}
	e.writeEntityBase(flags, b.ID, b.Pos, b.Dims, b.Direction, b.Mode)
	e.putF32(b.Life)
	e.putF32(b.MaxLife)
	if respawnIn != nil {
		e.putF32(*respawnIn)
	}
	e.putString(b.Behavior)
	e.writeItemIDs(b.ObjectLayers)
	e.putString(b.CasterID)
	e.putU16(uint16(statsSum))
	// Presence status — u8 lifecycle icon ID (entity_status.go).
	e.putU8(statusIcon)
	// Interaction capability bitmask — u8, resolved PER VIEWING PLAYER. Each set
	// bit (action/quest) enables an overlay icon and its interact-modal tab.
	e.putU8(interactionFlags)
	// The bot's action code, "" for ordinary bots. The client fetches the action
	// metadata (label, dialogue map) by this code via REST.
	e.putString(actionCode)
	// Authoritative quest codes this NPC provides to the viewing player; the
	// client fetches quest metadata by code only when not already cached.
	e.putStringList(questCodes)
	// Pending action-talk-quest dialogue code, "" when none. Non-empty means the
	// player has an active talk step this NPC's action maps to a dialogue; the
	// client shows that dialogue (quest-framed) in place of the default greeting.
	e.putString(actionDialogCode)
}

func (e *BinaryAOIEncoder) WriteFloor(f *FloorState) {
	e.writeEntityBase(EntityTypeFloor, f.ID, f.Pos, f.Dims, NONE, IDLE)
	e.writeItemIDs(f.ObjectLayers)
}

func (e *BinaryAOIEncoder) WriteObstacle(o ObjectState) {
	e.writeEntityBase(EntityTypeObstacle, o.ID, o.Pos, o.Dims, NONE, IDLE)
	e.writeItemIDs(o.ObjectLayers)
}

func (e *BinaryAOIEncoder) WritePortal(p *PortalState) {
	e.writeEntityBase(EntityTypePortal, p.ID, p.Pos, p.Dims, NONE, IDLE)
	// Destination for the client nameplate. A negative target cell marks a random
	// portal (inter/intra-random): it carries the 'portal-random' presence icon
	// and the client omits the cell from the nameplate (the target is random).
	destMapCode := ""
	destCellX, destCellY := 0, 0
	if p.PortalConfig != nil {
		destMapCode = p.PortalConfig.DestMapCode
		destCellX = int(p.PortalConfig.DestCellX)
		destCellY = int(p.PortalConfig.DestCellY)
	}
	statusIcon := StatusPortal
	if destCellX < 0 || destCellY < 0 {
		statusIcon = StatusPortalRandom
	}
	e.putU8(statusIcon)
	e.putString(destMapCode)
	e.putI16(int16(destCellX))
	e.putI16(int16(destCellY))
	e.writeItemIDs(p.ObjectLayers)
}

func (e *BinaryAOIEncoder) WriteForeground(fg ObjectState) {
	e.writeEntityBase(EntityTypeForeground, fg.ID, fg.Pos, fg.Dims, NONE, IDLE)
	e.writeItemIDs(fg.ObjectLayers)
}

// WriteStatic writes a static decorator block: entity base + active item IDs.
// No life, respawn, behavior, or status icon — statics are inert decorations
// the client depth-sorts with entities but never collides with.
func (e *BinaryAOIEncoder) WriteStatic(st *StaticState) {
	e.writeEntityBase(EntityTypeStatic, st.ID, st.Pos, st.Dims, NONE, IDLE)
	e.writeItemIDs(st.ObjectLayers)
}

func (e *BinaryAOIEncoder) WriteResource(r *ResourceState, respawnIn *float64, statsSum int) {
	flags := EntityTypeResource | FlagHasLife
	if respawnIn != nil {
		flags |= FlagHasRespawn
	}
	e.writeEntityBase(flags, r.ID, r.Pos, r.Dims, NONE, IDLE)
	e.putF32(r.Life)
	e.putF32(r.MaxLife)
	if respawnIn != nil {
		e.putF32(*respawnIn)
	}
	e.writeItemIDs(r.ObjectLayers)
	// Stats sum (sum of active-layer stats) — drives the overhead Σ-stats
	// capability value, same as players and bots.
	e.putU16(uint16(statsSum))
	// Entity Status Indicator — u8 overhead icon ID (see entity_status.go).
	e.putU8(ResourceStatusIcon(r))
}

// WriteSelfPlayer writes the self-player block. The final two fields
// (frozen + status icon) are followed by the authoritative MoveSpeed in
// grid-units per second; the C/WASM client feeds this directly into its
// prediction integrator so the byte-identical step formula matches
// phaseMovement on every Agility / buff / debuff change.
func (e *BinaryAOIEncoder) WriteSelfPlayer(p *PlayerState, activeStatsSum int, coinBalance uint32, moveSpeed float64, portalHoldProgress float64) {
	flags := EntityTypePlayer | FlagHasLife
	var respawnIn *float64
	if p.IsGhost() {
		remaining := math.Ceil(time.Until(p.RespawnTime).Seconds())
		if remaining > 0 {
			flags |= FlagHasRespawn
			respawnIn = &remaining
		}
	}
	e.writeEntityBase(flags, p.ID, p.Pos, p.Dims, p.Direction, p.Mode)
	e.putF32(p.Life)
	e.putF32(p.MaxLife)
	if respawnIn != nil {
		e.putF32(*respawnIn)
	}
	e.writeItemIDs(p.ObjectLayers)

	// Extended self-player fields
	e.putF32(p.AOI.MinX)
	e.putF32(p.AOI.MinY)
	e.putF32(p.AOI.MaxX)
	e.putF32(p.AOI.MaxY)
	if p.OnPortal {
		e.putU8(1)
	} else {
		e.putU8(0)
	}
	e.putU16(uint16(p.SumStatsLimit))
	e.putU16(uint16(activeStatsSum))
	e.putString(p.MapCode)

	pathLen := len(p.Path)
	if pathLen > 255 {
		pathLen = 255
	}
	e.putU8(byte(pathLen))
	for i := 0; i < pathLen; i++ {
		e.putI16(int16(p.Path[i].X))
		e.putI16(int16(p.Path[i].Y))
	}
	e.putI16(int16(p.TargetPos.X))
	e.putI16(int16(p.TargetPos.Y))
	e.putString(p.ActivePortalID)
	// Coin balance — u32, always present (0 when no coins).
	e.putU32(coinBalance)

	// Full inventory — ALL ObjectLayers (active + inactive) with quantities.
	// Powers the client-side inventory bottom bar.
	e.writeFullInventory(p.ObjectLayers)

	// FrozenInteractionState — u8 (0 = normal, 1 = frozen).
	// Client uses this as the authoritative frozen flag for visual feedback
	// and to block outgoing actions.
	if p.Frozen {
		e.putU8(1)
	} else {
		e.putU8(0)
	}

	// Entity Status Indicator — u8 overhead icon ID (see entity_status.go).
	// Self-player also gets the status icon so the client can render the
	// frozen/dead indicator above its own entity.
	e.putU8(PlayerStatusIcon(p))

	// Authoritative move speed (cells/second). Fed to the client prediction
	// integrator so client-side step = moveSpeed * tickDuration matches the
	// server byte-for-byte.
	e.putF32(moveSpeed)

	// Portal hold progress (0..1) — authoritative teleport charge for the local
	// player only. Non-zero exclusively while alive and standing on a portal;
	// the client renders it as a hold bar and never derives it locally.
	e.putF32(portalHoldProgress)
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
// Full AOI encoder — replaces JSON sendAOI
// ═══════════════════════════════════════════════════════════════════

func (s *GameServer) EncodeBinaryAOI(player *PlayerState, mapState *MapState) []byte {
	enc := acquireEncoder()
	defer enc.release()

	headerPos := enc.pos
	// AOI v2 header — tick + lastAckedSequence carried in every snapshot.
	// See file-top comment for layout. The entityCount u16 is back-patched
	// at the end once we know how many blocks were written.
	enc.putU8(MsgTypeFullAOI)
	enc.putU32(player.LastSnapshotTick)
	enc.putU32(player.LastAckedInputSequence)
	enc.putU16(0) // placeholder entity count — patched below

	entityCount := 0

	// Players
	for _, op := range mapState.players {
		if op.ID == player.ID {
			continue
		}
		oRect := Rectangle{MinX: op.Pos.X, MinY: op.Pos.Y, MaxX: op.Pos.X + op.Dims.Width, MaxY: op.Pos.Y + op.Dims.Height}
		if !rectsOverlap(player.AOI, oRect) {
			continue
		}
		var respawnIn *float64
		if op.IsGhost() {
			r := math.Ceil(time.Until(op.RespawnTime).Seconds())
			if r > 0 {
				respawnIn = &r
			}
		}
		enc.WritePlayer(op, respawnIn, s.statsSum(op, mapState))
		entityCount++
	}

	// Obstacles
	for _, o := range mapState.obstacles {
		oRect := Rectangle{MinX: o.Pos.X, MinY: o.Pos.Y, MaxX: o.Pos.X + o.Dims.Width, MaxY: o.Pos.Y + o.Dims.Height}
		if rectsOverlap(player.AOI, oRect) {
			enc.WriteObstacle(o)
			entityCount++
		}
	}

	// Floors — sorted: OL floors first (background), then solid-colour
	// floors on top.  Within each group sort by Y→X→ID for stability.
	// This prevents flicker caused by Go map random iteration order when
	// overlapping floors exist (e.g. a solid-colour floor on top of a
	// larger textured floor).
	var aoiFloors []*FloorState
	for _, f := range mapState.floors {
		fRect := Rectangle{MinX: f.Pos.X, MinY: f.Pos.Y, MaxX: f.Pos.X + f.Dims.Width, MaxY: f.Pos.Y + f.Dims.Height}
		if rectsOverlap(player.AOI, fRect) {
			aoiFloors = append(aoiFloors, f)
		}
	}
	sort.Slice(aoiFloors, func(i, j int) bool {
		iOL := len(aoiFloors[i].ObjectLayers) > 0
		jOL := len(aoiFloors[j].ObjectLayers) > 0
		if iOL != jOL {
			return iOL // OL floors drawn first (underneath)
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
		enc.WriteFloor(f)
		entityCount++
	}

	// Portals
	for _, p := range mapState.portals {
		pRect := Rectangle{MinX: p.Pos.X, MinY: p.Pos.Y, MaxX: p.Pos.X + p.Dims.Width, MaxY: p.Pos.Y + p.Dims.Height}
		if rectsOverlap(player.AOI, pRect) {
			enc.WritePortal(p)
			entityCount++
		}
	}

	// Foregrounds
	for _, fg := range mapState.foregrounds {
		fgRect := Rectangle{MinX: fg.Pos.X, MinY: fg.Pos.Y, MaxX: fg.Pos.X + fg.Dims.Width, MaxY: fg.Pos.Y + fg.Dims.Height}
		if rectsOverlap(player.AOI, fgRect) {
			enc.WriteForeground(fg)
			entityCount++
		}
	}

	// Resources — static exploitable entities (depth-sorted with bots by client)
	for _, r := range mapState.resources {
		rRect := Rectangle{MinX: r.Pos.X, MinY: r.Pos.Y, MaxX: r.Pos.X + r.Dims.Width, MaxY: r.Pos.Y + r.Dims.Height}
		if !rectsOverlap(player.AOI, rRect) {
			continue
		}
		var respawnIn *float64
		if r.IsGhost() {
			remaining := math.Ceil(time.Until(r.RespawnTime).Seconds())
			if remaining > 0 {
				respawnIn = &remaining
			}
		}
		enc.WriteResource(r, respawnIn, s.statsSum(r, mapState))
		entityCount++
	}

	// Statics — non-moving, passable decorators (depth-sorted with bots by client)
	for _, st := range mapState.statics {
		stRect := Rectangle{MinX: st.Pos.X, MinY: st.Pos.Y, MaxX: st.Pos.X + st.Dims.Width, MaxY: st.Pos.Y + st.Dims.Height}
		if rectsOverlap(player.AOI, stRect) {
			enc.WriteStatic(st)
			entityCount++
		}
	}

	// Bots
	for _, b := range mapState.bots {
		bRect := Rectangle{MinX: b.Pos.X, MinY: b.Pos.Y, MaxX: b.Pos.X + b.Dims.Width, MaxY: b.Pos.Y + b.Dims.Height}
		if !rectsOverlap(player.AOI, bRect) {
			continue
		}
		var respawnIn *float64
		if b.IsGhost() {
			r := math.Ceil(time.Until(b.RespawnTime).Seconds())
			if r > 0 {
				respawnIn = &r
			}
		}
		// The bot's action code (location-scoped, position-independent). The
		// client fetches the action metadata (label, dialogue map) by code via
		// REST; offered quests are located via the cyberia-quest API by the bot's
		// binding cell, decoupled from CyberiaAction.
		actionCode := b.ActionCode
		statusIcon := BotStatusIcon(b)
		// Interaction capabilities + provided quests are resolved PER VIEWING
		// PLAYER and per cell — independent of any cyberia-action — so the quest
		// bit/codes appear whenever this NPC offers or advances a quest right now.
		questCodes := s.botQuestCodes(player, b)
		actionDialog := s.pendingActionTalkDialog(player, b)
		interactionFlags := s.botInteractionFlags(len(questCodes) > 0, actionDialog != "")
		enc.WriteBot(b, respawnIn, s.statsSum(b, mapState), actionCode, statusIcon, interactionFlags, questCodes, actionDialog)
		entityCount++
	}

	// Self-player (always last)
	playerStats := s.CalculateStats(player, mapState)
	activeStatsSum := int(playerStats.Effect + playerStats.Resistance + playerStats.Agility +
		playerStats.Range + playerStats.Intelligence + playerStats.Utility)
	moveSpeed := s.CalculateMovementSpeed(playerStats)
	// Portal hold progress — fraction of portalHoldTime elapsed while the player
	// stands on a portal. checkPortal already clears OnPortal when the player is a
	// ghost, so this is non-zero only for a live player charging a teleport.
	portalHoldProgress := 0.0
	if player.OnPortal && s.portalHoldTime > 0 {
		portalHoldProgress = time.Since(player.TimeOnPortal).Seconds() / s.portalHoldTime.Seconds()
		if portalHoldProgress > 1.0 {
			portalHoldProgress = 1.0
		} else if portalHoldProgress < 0.0 {
			portalHoldProgress = 0.0
		}
	}
	// player.Coins is the canonical flat balance — read directly (O(1), no OL traversal).
	enc.WriteSelfPlayer(player, activeStatsSum, player.Coins, moveSpeed, portalHoldProgress)

	// Patch entity count in header.
	// v2 header layout: msgType(1) + tick(4) + lastAckedSequence(4) + entityCount(2)
	// → entityCount starts at byte 9 inside the header block.
	binary.LittleEndian.PutUint16(enc.buf[headerPos+9:], uint16(entityCount))

	result := make([]byte, enc.pos)
	copy(result, enc.buf[:enc.pos])
	return result
}
