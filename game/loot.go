package game

// loot.go — Damage-contribution ledger, scattered world drops, and
// authoritative collection.
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │  DROP LIFECYCLE                                                          │
// │                                                                         │
// │  1. Contribution  — every projectile hit on a bot/resource records the  │
// │     firing player into the victim's DamageLedger (skill collision path). │
// │                                                                         │
// │  2. Death scatter — the death handler calls spawnDrops: each configured │
// │     drop item becomes a transient BehaviorDrop token flung a random     │
// │     distance from the corpse along a random direction vector, carrying  │
// │     the set of damage contributors eligible to collect it.              │
// │                                                                         │
// │  3. Collection   — phaseLoot runs each tick after movement. Once a token │
// │     settles, collection is a race: the first contributor to overlap it  │
// │     wins (anyone may, if no player dealt damage). Collection grants the  │
// │     item, fires the pickup animation event (MsgTypeDropCollect) to every │
// │     viewer, and removes the token.                                      │
// │                                                                         │
// │  4. Expiry       — uncollected tokens despawn via ExpiresAt (updateBots)│
// └─────────────────────────────────────────────────────────────────────────┘

import (
	"encoding/binary"
	"math"
	"math/rand"
	"time"

	"github.com/google/uuid"

	"cyberia-server/logx"
)

// ── Tunables ──────────────────────────────────────────────────────────────

const (
	// lootLandingWindow is the settle window: while the token flies from the
	// corpse to its cell it registers no collision. Must match the client launch
	// animation length (sent as launchMs in MsgTypeDropSpawn).
	lootLandingWindow = 420 * time.Millisecond
	// lootLifetime bounds an uncollected token's existence on the grid.
	lootLifetime = 45 * time.Second
	// lootScatterMinCells / lootScatterMaxCells bound the scatter distance from
	// the corpse (grid cells). A wide range keeps drops from clustering: most
	// land nearby, some fling out to the far edge.
	lootScatterMinCells = 1.0
	lootScatterMaxCells = 5.0
	// lootTokenSize is the token's square side in grid cells — the size of the
	// dropped object layer resting on the grid.
	lootTokenSize = 1.6
)

// ── Contribution ledger ──────────────────────────────────────────────────

// recordDamage records that a player dealt damage to a mortal entity, so every
// contributor can race for its drops on death. Only player casters are tracked:
// bot-cast damage grants no claim. The ledger is allocated lazily via the
// pointer. Caller holds s.mu.
func recordDamage(ledger *map[string]float64, mapState *MapState, casterID string, amount float64) {
	if ledger == nil || casterID == "" || amount <= 0 {
		return
	}
	if _, ok := mapState.players[casterID]; !ok {
		return // caster is not a player (e.g. bot-cast projectile) — no claim
	}
	if *ledger == nil {
		*ledger = make(map[string]float64, 4)
	}
	(*ledger)[casterID] += amount
}

// contributorSet copies the ledger's player IDs into a membership set — the
// eligibility list a drop carries for its collection race. Returns nil when no
// player dealt damage (anyone may then collect).
func contributorSet(ledger map[string]float64) map[string]struct{} {
	if len(ledger) == 0 {
		return nil
	}
	set := make(map[string]struct{}, len(ledger))
	for id := range ledger {
		set[id] = struct{}{}
	}
	return set
}

// ── Death scatter ──────────────────────────────────────────────────────────

// spawnDrops scatters each drop item as a transient BehaviorDrop token onto a
// walkable cell a random distance from `center`, along an independent direction
// vector. Every player in `ledger` may race to collect the tokens once they
// settle. Shared by every dying content-authority entity (bots, resources).
// No-op without drops. Caller holds s.mu.
func (s *GameServer) spawnDrops(mapState *MapState, mapCode string, center Point, dropItemIDs []string, ledger map[string]float64) {
	if len(dropItemIDs) == 0 {
		return
	}

	now := time.Now()
	contributors := contributorSet(ledger)
	collectableAt := now.Add(lootLandingWindow)

	tokenDims := Dimensions{Width: lootTokenSize, Height: lootTokenSize}

	for _, itemID := range dropItemIDs {
		if itemID == "" {
			continue
		}

		angle := rand.Float64() * 2 * math.Pi
		// Squared roll biases toward the near end with a long tail to the far
		// edge, so drops feel randomly flung rather than evenly ringed.
		roll := rand.Float64()
		dist := lootScatterMinCells + roll*roll*(lootScatterMaxCells-lootScatterMinCells)
		targetCell := PointI{
			X: int(math.Round(center.X + math.Cos(angle)*dist)),
			Y: int(math.Round(center.Y + math.Sin(angle)*dist)),
		}

		cell := s.clampDropCell(mapState, targetCell, tokenDims)
		// Center the token on the resolved cell.
		pos := Point{
			X: float64(cell.X) - tokenDims.Width*0.5,
			Y: float64(cell.Y) - tokenDims.Height*0.5,
		}

		drop := &BotState{
			EntityBase: EntityBase{
				ID:   uuid.New().String(),
				Pos:  pos,
				Dims: tokenDims,
				ObjectLayers: []ObjectLayerState{
					{ItemID: itemID, Active: true, Quantity: 1},
				},
			},
			Mortal: Mortal{
				MaxLife: s.entityBaseMaxLife,
				Life:    s.entityBaseMaxLife,
			},
			MapCode:       mapCode,
			Behavior:      BehaviorDrop,
			Direction:     NONE,
			Mode:          IDLE,
			DropItemID:       itemID,
			LootContributors: contributors,
			CollectableAt:    collectableAt,
			ExpiresAt:        now.Add(lootLifetime),
		}
		mapState.bots[drop.ID] = drop

		// Announce the launch so viewers play the corpse→cell parabola. The
		// landing point is the token center.
		landing := Point{X: pos.X + tokenDims.Width*0.5, Y: pos.Y + tokenDims.Height*0.5}
		s.broadcastDropSpawn(mapState, drop, center, landing)
	}

	logx.Debugf("[LOOT] scattered %d drop(s) at (%.1f,%.1f) on %s (contributors=%d)",
		len(dropItemIDs), center.X, center.Y, mapCode, len(contributors))
}

// clampDropCell keeps a scatter target inside the grid and on a walkable cell,
// falling back to the closest walkable point (then the raw cell) so a drop is
// never lost to an obstacle.
func (s *GameServer) clampDropCell(mapState *MapState, cell PointI, dims Dimensions) PointI {
	if cell.X < 0 {
		cell.X = 0
	} else if cell.X >= mapState.gridW {
		cell.X = mapState.gridW - 1
	}
	if cell.Y < 0 {
		cell.Y = 0
	} else if cell.Y >= mapState.gridH {
		cell.Y = mapState.gridH - 1
	}
	if mapState.pathfinder != nil {
		if mapState.pathfinder.isWalkable(cell.X, cell.Y, dims) {
			return cell
		}
		if c, err := mapState.pathfinder.findClosestWalkablePoint(cell, dims); err == nil {
			return c
		}
	}
	return cell
}

// ── Authoritative collection ─────────────────────────────────────────────

// phaseLoot resolves player↔drop-token collisions for one tick. A live player
// overlapping a token collects it when the priority lock has expired or the
// player is the lock owner. Collection grants the item, broadcasts the pickup
// animation event to every viewer, and removes the token. Caller holds s.mu.
func (s *GameServer) phaseLoot(tick uint32, mapState *MapState) {
	_ = tick
	now := time.Now()

	for dropID, drop := range mapState.bots {
		if !behaviorIsInert(drop.Behavior) {
			continue
		}
		// Mid-launch tokens register no collision until they settle on the cell.
		if now.Before(drop.CollectableAt) {
			continue
		}

		collector := s.resolveDropCollector(mapState, drop)
		if collector == nil {
			continue
		}

		cx := drop.Pos.X + drop.Dims.Width*0.5
		cy := drop.Pos.Y + drop.Dims.Height*0.5

		s.grantDropItemsToPlayer(collector, []string{drop.DropItemID}, cx, cy)
		s.advancePlayerQuestsOnGain(collector)
		s.broadcastDropCollect(mapState, drop, collector, cx, cy)

		delete(mapState.bots, dropID)
	}
}

// resolveDropCollector returns the first live, non-frozen player overlapping the
// token that is eligible to collect it, or nil when none is. Eligibility is the
// collection race: any damage contributor may grab the drop; when no player
// contributed (empty set) it is free for anyone. Go's random map iteration
// order fairly breaks ties between simultaneous colliders.
func (s *GameServer) resolveDropCollector(mapState *MapState, drop *BotState) *PlayerState {
	for _, player := range mapState.players {
		if player.IsGhost() || player.Frozen {
			continue
		}
		if !checkAABBCollision(drop.Pos, drop.Dims, player.Pos, player.Dims) {
			continue
		}
		if len(drop.LootContributors) == 0 {
			return player // no player dealt damage — free for all
		}
		if _, ok := drop.LootContributors[player.ID]; ok {
			return player
		}
	}
	return nil
}

// ── Pickup animation event ──────────────────────────────────────────────────

// buildDropCollectMsg encodes a MsgTypeDropCollect event.
//
// Wire format (little-endian):
//
//	[0]       u8   MsgTypeDropCollect (0x06)
//	[1..36]   36B  dropId       (token entity UUID, zero-padded)
//	[37..72]  36B  collectorId  (collecting player UUID, zero-padded)
//	[73..76]  f32  worldX       (token origin — animation start)
//	[77..80]  f32  worldY
//	[81]      u8   itemId length (0–63)
//	[82..]    str  itemId bytes
func buildDropCollectMsg(dropID, collectorID string, worldX, worldY float64, itemID string) []byte {
	idLen := len(itemID)
	if idLen > 63 {
		idLen = 63
	}
	buf := make([]byte, 82+idLen)
	buf[0] = MsgTypeDropCollect
	copyPaddedID(buf[1:37], dropID)
	copyPaddedID(buf[37:73], collectorID)
	binary.LittleEndian.PutUint32(buf[73:], math.Float32bits(float32(worldX)))
	binary.LittleEndian.PutUint32(buf[77:], math.Float32bits(float32(worldY)))
	buf[81] = byte(idLen)
	copy(buf[82:], itemID[:idLen])
	return buf
}

// copyPaddedID writes a UUID string into a fixed 36-byte field, zero-padded.
func copyPaddedID(dst []byte, id string) {
	n := len(id)
	if n > len(dst) {
		n = len(dst)
	}
	copy(dst, id[:n])
	for i := n; i < len(dst); i++ {
		dst[i] = 0
	}
}

// buildDropSpawnMsg encodes a MsgTypeDropSpawn event.
//
// Wire format (little-endian):
//
//	[0]       u8   MsgTypeDropSpawn (0x07)
//	[1..36]   36B  dropId (token entity UUID, zero-padded)
//	[37..40]  f32  originX  (corpse center — launch start)
//	[41..44]  f32  originY
//	[45..48]  f32  landingX (token center — launch end / rest cell)
//	[49..52]  f32  landingY
//	[53..54]  u16  launchMs (settle-window duration; client animation length)
//	[55]      u8   itemId length (0–63)
//	[56..]    str  itemId bytes
func buildDropSpawnMsg(dropID string, origin, landing Point, launchMs int, itemID string) []byte {
	idLen := len(itemID)
	if idLen > 63 {
		idLen = 63
	}
	if launchMs < 0 {
		launchMs = 0
	} else if launchMs > 65535 {
		launchMs = 65535
	}
	buf := make([]byte, 56+idLen)
	buf[0] = MsgTypeDropSpawn
	copyPaddedID(buf[1:37], dropID)
	binary.LittleEndian.PutUint32(buf[37:], math.Float32bits(float32(origin.X)))
	binary.LittleEndian.PutUint32(buf[41:], math.Float32bits(float32(origin.Y)))
	binary.LittleEndian.PutUint32(buf[45:], math.Float32bits(float32(landing.X)))
	binary.LittleEndian.PutUint32(buf[49:], math.Float32bits(float32(landing.Y)))
	binary.LittleEndian.PutUint16(buf[53:], uint16(launchMs))
	buf[55] = byte(idLen)
	copy(buf[56:], itemID[:idLen])
	return buf
}

// broadcastDropSpawn delivers the launch event to every player watching the
// corpse (AOI covers the launch origin), so all viewers see the token arc out
// of the dead bot. Non-blocking (cosmetic).
func (s *GameServer) broadcastDropSpawn(mapState *MapState, drop *BotState, origin, landing Point) {
	msg := buildDropSpawnMsg(drop.ID, origin, landing,
		int(lootLandingWindow/time.Millisecond), drop.DropItemID)
	originRect := Rectangle{MinX: origin.X, MinY: origin.Y, MaxX: origin.X, MaxY: origin.Y}
	for _, player := range mapState.players {
		if player.Client == nil {
			continue
		}
		if !rectsOverlap(player.AOI, originRect) {
			continue
		}
		select {
		case player.Client.send <- msg:
		default:
		}
	}
}

// broadcastDropCollect delivers the pickup animation event to every player whose
// AOI covers the token, so all viewers see the token fly to the collector. The
// send is non-blocking (cosmetic).
func (s *GameServer) broadcastDropCollect(mapState *MapState, drop *BotState, collector *PlayerState, cx, cy float64) {
	msg := buildDropCollectMsg(drop.ID, collector.ID, cx, cy, drop.DropItemID)
	dropRect := Rectangle{
		MinX: drop.Pos.X, MinY: drop.Pos.Y,
		MaxX: drop.Pos.X + drop.Dims.Width, MaxY: drop.Pos.Y + drop.Dims.Height,
	}
	for _, player := range mapState.players {
		if player.Client == nil {
			continue
		}
		if player.ID != collector.ID && !rectsOverlap(player.AOI, dropRect) {
			continue
		}
		select {
		case player.Client.send <- msg:
		default:
		}
	}
}
