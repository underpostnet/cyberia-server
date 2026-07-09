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
// │     wins. Collection grants the item, fires the pickup animation event   │
// │     (MsgTypeDropCollect) to every viewer, and removes the token. A kill  │
// │     with no player contributor drops nothing at all.                    │
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
	// coinTokenSize is the smaller square side used for coin tokens, so currency
	// reads as more minor loot than item drops.
	coinTokenSize = 1.0
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
// player dealt damage, which suppresses the drop entirely (see spawnDrops).
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

// spawnDrops scatters each configured drop item (quantity 1) as a BehaviorDrop
// token around `center`, and — when the dying entity carried coins — one coin
// token whose quantity is the looted amount. Every player in `ledger` may race
// to collect the tokens once they settle. Shared by every dying
// content-authority entity (bots, resources). Caller holds s.mu.
func (s *GameServer) spawnDrops(mapState *MapState, mapCode string, center Point, dropItemIDs []string, coinAmount int, ledger map[string]float64) {
	contributors := contributorSet(ledger)
	// At least one player must have dealt damage for loot to drop. Bot-only
	// kills (e.g. bot-vs-bot) leave nothing behind.
	if len(contributors) == 0 {
		return
	}

	for _, itemID := range dropItemIDs {
		if itemID == "" {
			continue
		}
		s.spawnDropToken(mapState, mapCode, center, itemID, 1, contributors)
	}

	// Coins scatter as a single quantity-bearing token rather than crediting the
	// killer directly (see economy.go).
	if coinAmount > 0 && s.coinItemID != "" {
		s.spawnDropToken(mapState, mapCode, center, s.coinItemID, coinAmount, contributors)
	}

	logx.Debugf("[LOOT] scattered %d item drop(s) + %d coins at (%.1f,%.1f) on %s (contributors=%d)",
		len(dropItemIDs), coinAmount, center.X, center.Y, mapCode, len(contributors))
}

// spawnDropToken creates one BehaviorDrop token carrying (itemID × quantity),
// flings it to a walkable cell a random distance from `center` along a random
// direction, mirrors the quantity into its active ObjectLayer (for the on-grid
// counter), and announces the launch. Caller holds s.mu.
func (s *GameServer) spawnDropToken(mapState *MapState, mapCode string, center Point, itemID string, quantity int, contributors map[string]struct{}) {
	if itemID == "" || quantity <= 0 {
		return
	}
	now := time.Now()
	size := lootTokenSize
	if itemID == s.coinItemID {
		size = coinTokenSize
	}
	tokenDims := Dimensions{Width: size, Height: size}

	angle := rand.Float64() * 2 * math.Pi
	// Squared roll biases toward the near end with a long tail to the far edge,
	// so drops feel randomly flung rather than evenly ringed.
	roll := rand.Float64()
	dist := lootScatterMinCells + roll*roll*(lootScatterMaxCells-lootScatterMinCells)
	targetCell := PointI{
		X: int(math.Round(center.X + math.Cos(angle)*dist)),
		Y: int(math.Round(center.Y + math.Sin(angle)*dist)),
	}

	cell := s.clampDropCell(mapState, targetCell, tokenDims)
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
				{ItemID: itemID, Active: true, Quantity: quantity},
			},
		},
		Mortal: Mortal{
			MaxLife: s.entityBaseMaxLife,
			Life:    s.entityBaseMaxLife,
		},
		MapCode:          mapCode,
		Behavior:         BehaviorDrop,
		Direction:        NONE,
		Mode:             IDLE,
		DropItemID:       itemID,
		DropQuantity:     quantity,
		LootContributors: contributors,
		CollectableAt:    now.Add(lootLandingWindow),
		ExpiresAt:        now.Add(lootLifetime),
	}
	mapState.bots[drop.ID] = drop

	// Announce the launch so viewers play the corpse→cell parabola. The landing
	// point is the token center.
	landing := Point{X: pos.X + tokenDims.Width*0.5, Y: pos.Y + tokenDims.Height*0.5}
	s.broadcastDropSpawn(mapState, drop, center, landing)
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

// phaseLoot resolves player↔drop-token collisions for one tick. Once a token has
// settled, the first eligible overlapping player (the contribution race, see
// resolveDropCollector) collects it: coins credit the wallet, items enter the
// inventory — both by the token's quantity. The pickup animation event is
// broadcast to every viewer and the token removed. Caller holds s.mu.
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

		if drop.DropItemID == s.coinItemID {
			// No coin-gain FCT — the amount is shown on the grid loot counter.
			s.addCoins(collector, drop.DropQuantity)
			s.InvalidateStats(collector)
		} else {
			s.grantItemToPlayer(collector, drop.DropItemID, drop.DropQuantity)
			s.advancePlayerQuestsOnGain(collector)
		}
		s.broadcastDropCollect(mapState, drop, collector, cx, cy)

		delete(mapState.bots, dropID)
	}
}

// resolveDropCollector returns the first live, non-frozen player overlapping the
// token that is eligible to collect it, or nil when none is. Eligibility is the
// collection race: only a damage contributor may grab the drop (a token always
// has ≥1 contributor — see spawnDrops). Go's random map iteration order fairly
// breaks ties between simultaneous colliders.
func (s *GameServer) resolveDropCollector(mapState *MapState, drop *BotState) *PlayerState {
	for _, player := range mapState.players {
		if player.IsGhost() || player.Frozen {
			continue
		}
		if !checkAABBCollision(drop.Pos, drop.Dims, player.Pos, player.Dims) {
			continue
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
