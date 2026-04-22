// Package game — aoi_binary.go
//
// Minimal binary AOI protocol for WebSocket communication with the
// C/WASM client. Only render-essential data is sent: positions,
// directions, modes, life values, and item-ID stacks. The client
// fetches atlas binaries from Engine-Cyberia REST independently.
//
// Wire format (little-endian):
//
//	Header (5 bytes):
//	  [0]       u8   msgType  (0x01 = aoi_update, 0x02 = init_data, 0x03 = full_aoi)
//	  [1..2]    u16  reserved (always 0)
//	  [3..4]    u16  entityCount (number of entity blocks that follow)
//
//	Per-entity block (variable length):
//	  [0]       u8   flags
//	                  bits 0-2: type (0=player, 1=bot, 2=floor, 3=obstacle, 4=portal, 5=foreground)
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
package game

import (
	"encoding/binary"
	"math"
	"sort"
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
	MsgTypeFCT     byte = 0x04
	// MsgTypeItemFCT — Item quantity Floating Combat Text event (≥15 bytes).
	// See economy.go buildItemFCTMsg for wire format.
	MsgTypeItemFCT byte = 0x05

	// FCT event sub-types — MUST stay in sync with C constants in
	// floating_combat_text.h and binary_aoi_decoder.h.
	FCTTypeDamage   byte = 0x00 // life loss            — red    "-N"
	FCTTypeRegen    byte = 0x01 // life gain / regen    — green  "+N"
	FCTTypeCoinGain byte = 0x02 // coins received       — yellow "+N"
	FCTTypeCoinLoss byte = 0x03 // coins lost / sink    — yellow "-N"
	FCTTypeItemGain byte = 0x04 // generic item gain    — cyan   "+N ItemID"
	FCTTypeItemLoss byte = 0x05 // generic item loss    — purple "-N ItemID"

	EntityTypePlayer     byte = 0
	EntityTypeBot        byte = 1
	EntityTypeFloor      byte = 2
	EntityTypeObstacle   byte = 3
	EntityTypePortal     byte = 4
	EntityTypeForeground byte = 5
	EntityTypeResource   byte = 6

	FlagRemoved     byte = 0x08 // bit 3
	FlagHasLife     byte = 0x10 // bit 4
	FlagHasRespawn  byte = 0x20 // bit 5
	FlagHasBehavior byte = 0x40 // bit 6
	FlagHasColor    byte = 0x80 // bit 7

	maxBinaryBufSize = 64 * 1024 // 64 KB per AOI message
)

// BinaryAOIEncoder builds compact binary AOI messages.
type BinaryAOIEncoder struct {
	buf []byte
	pos int
}

func NewBinaryAOIEncoder() *BinaryAOIEncoder {
	return &BinaryAOIEncoder{buf: make([]byte, maxBinaryBufSize)}
}

func (e *BinaryAOIEncoder) Reset()       { e.pos = 0 }
func (e *BinaryAOIEncoder) Bytes() []byte { return e.buf[:e.pos] }

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

func (e *BinaryAOIEncoder) WritePlayer(p *PlayerState, respawnIn *float64, effectiveLevel int) {
	flags := EntityTypePlayer | FlagHasLife | FlagHasColor
	if respawnIn != nil {
		flags |= FlagHasRespawn
	}
	e.writeEntityBase(flags, p.ID, p.Pos, p.Dims, p.Direction, p.Mode)
	e.putF32(p.Life)
	e.putF32(p.MaxLife)
	if respawnIn != nil {
		e.putF32(*respawnIn)
	}
	e.putU8(byte(p.Color.R))
	e.putU8(byte(p.Color.G))
	e.putU8(byte(p.Color.B))
	e.putU8(byte(p.Color.A))
	e.writeItemIDs(p.ObjectLayers)
	e.putU16(uint16(effectiveLevel))
	// Entity Status Indicator — u8 overhead icon ID (see entity_status.go).
	e.putU8(PlayerStatusIcon(p))
}

func (e *BinaryAOIEncoder) WriteBot(b *BotState, respawnIn *float64, effectiveLevel int) {
	flags := EntityTypeBot | FlagHasLife | FlagHasBehavior | FlagHasColor
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
	e.putU8(byte(b.Color.R))
	e.putU8(byte(b.Color.G))
	e.putU8(byte(b.Color.B))
	e.putU8(byte(b.Color.A))
	e.writeItemIDs(b.ObjectLayers)
	e.putString(b.CasterID)
	e.putU16(uint16(effectiveLevel))
	// Entity Status Indicator — u8 overhead icon ID (see entity_status.go).
	e.putU8(BotStatusIcon(b))
}

func (e *BinaryAOIEncoder) WriteFloor(f *FloorState) {
	flags := EntityTypeFloor
	if f.Color.A > 0 {
		flags |= FlagHasColor
	}
	e.writeEntityBase(flags, f.ID, f.Pos, f.Dims, NONE, IDLE)
	if f.Color.A > 0 {
		e.putU8(byte(f.Color.R))
		e.putU8(byte(f.Color.G))
		e.putU8(byte(f.Color.B))
		e.putU8(byte(f.Color.A))
	}
	e.writeItemIDs(f.ObjectLayers)
}

func (e *BinaryAOIEncoder) WriteObstacle(o ObjectState) {
	flags := EntityTypeObstacle
	if o.Color.A > 0 {
		flags |= FlagHasColor
	}
	e.writeEntityBase(flags, o.ID, o.Pos, o.Dims, NONE, IDLE)
	if o.Color.A > 0 {
		e.putU8(byte(o.Color.R))
		e.putU8(byte(o.Color.G))
		e.putU8(byte(o.Color.B))
		e.putU8(byte(o.Color.A))
	}
	e.writeItemIDs(o.ObjectLayers)
}

func (e *BinaryAOIEncoder) WritePortal(p *PortalState) {
	e.writeEntityBase(EntityTypePortal|FlagHasColor, p.ID, p.Pos, p.Dims, NONE, IDLE)
	e.putString(p.Label)
	e.putU8(byte(p.Color.R))
	e.putU8(byte(p.Color.G))
	e.putU8(byte(p.Color.B))
	e.putU8(byte(p.Color.A))
}

func (e *BinaryAOIEncoder) WriteForeground(fg ObjectState) {
	flags := EntityTypeForeground
	if fg.Color.A > 0 {
		flags |= FlagHasColor
	}
	e.writeEntityBase(flags, fg.ID, fg.Pos, fg.Dims, NONE, IDLE)
	if fg.Color.A > 0 {
		e.putU8(byte(fg.Color.R))
		e.putU8(byte(fg.Color.G))
		e.putU8(byte(fg.Color.B))
		e.putU8(byte(fg.Color.A))
	}
	e.writeItemIDs(fg.ObjectLayers)
}

func (e *BinaryAOIEncoder) WriteResource(r *ResourceState, respawnIn *float64) {
	flags := EntityTypeResource | FlagHasLife | FlagHasColor
	if respawnIn != nil {
		flags |= FlagHasRespawn
	}
	e.writeEntityBase(flags, r.ID, r.Pos, r.Dims, NONE, IDLE)
	e.putF32(r.Life)
	e.putF32(r.MaxLife)
	if respawnIn != nil {
		e.putF32(*respawnIn)
	}
	e.putU8(byte(r.Color.R))
	e.putU8(byte(r.Color.G))
	e.putU8(byte(r.Color.B))
	e.putU8(byte(r.Color.A))
	e.writeItemIDs(r.ObjectLayers)
	// Entity Status Indicator — u8 overhead icon ID (see entity_status.go).
	e.putU8(ResourceStatusIcon(r))
}

func (e *BinaryAOIEncoder) WriteSelfPlayer(p *PlayerState, activeStatsSum int, coinBalance uint32) {
	flags := EntityTypePlayer | FlagHasLife
	var respawnIn *float64
	if p.IsGhost() {
		flags |= FlagHasRespawn
		remaining := math.Ceil(time.Until(p.RespawnTime).Seconds())
		if remaining > 0 {
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
}

// ═══════════════════════════════════════════════════════════════════
// Effective Level helper
// ═══════════════════════════════════════════════════════════════════

// effLevel returns the clamped effective level (sum of all stat fields)
// for any entity (PlayerState or BotState). Players use their own
// SumStatsLimit; bots use the server-level cap.
func (s *GameServer) effLevel(entity interface{}, mapState *MapState) int {
	cs := s.CalculateStats(entity, mapState)
	level := int(cs.Effect + cs.Resistance + cs.Agility + cs.Range + cs.Intelligence + cs.Utility)
	cap := s.sumStatsLimit
	if p, ok := entity.(*PlayerState); ok {
		cap = p.SumStatsLimit
	}
	if level > cap {
		level = cap
	}
	return level
}

// ═══════════════════════════════════════════════════════════════════
// Full AOI encoder — replaces JSON sendAOI
// ═══════════════════════════════════════════════════════════════════

func (s *GameServer) EncodeBinaryAOI(player *PlayerState, mapState *MapState) []byte {
	enc := NewBinaryAOIEncoder()

	headerPos := enc.pos
	enc.putU8(MsgTypeFullAOI)
	enc.putU16(0) // reserved
	enc.putU16(0) // placeholder entity count

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
		enc.WritePlayer(op, respawnIn, s.effLevel(op, mapState))
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
		enc.WriteResource(r, respawnIn)
		entityCount++
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
		enc.WriteBot(b, respawnIn, s.effLevel(b, mapState))
		entityCount++
	}

	// Self-player (always last)
	playerStats := s.CalculateStats(player, mapState)
	activeStatsSum := int(playerStats.Effect + playerStats.Resistance + playerStats.Agility +
		playerStats.Range + playerStats.Intelligence + playerStats.Utility)
	// player.Coins is the canonical flat balance — read directly (O(1), no OL traversal).
	enc.WriteSelfPlayer(player, activeStatsSum, player.Coins)

	// Patch entity count in header
	binary.LittleEndian.PutUint16(enc.buf[headerPos+3:], uint16(entityCount))

	result := make([]byte, enc.pos)
	copy(result, enc.buf[:enc.pos])
	return result
}
