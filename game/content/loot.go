package content

// loot.go — Damage-contribution ledger, scattered and world drops, and
// authoritative collection.
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │  DROP LIFECYCLE                                                         │
// │                                                                         │
// │  1. Contribution  — every projectile hit on a bot, resource or          │
// │     destructible obstacle records the firing player into the victim's   │
// │     DamageLedger (skill collision path).                                │
// │                                                                         │
// │  2. Death scatter — the death handler calls spawnDrops: each configured │
// │     drop item becomes a transient BehaviorDrop token flung a random     │
// │     distance from the corpse along a random direction vector, carrying  │
// │     the set of damage contributors eligible to collect it.              │
// │                                                                         │
// │  3. World loot    — every world build places each map's `loot` entities │
// │     on their cells. A map holds only the tokens it places.              │
// │                                                                         │
// │  4. Collection    — phaseLoot runs each tick after movement. The token's│
// │     collection policy picks the collector: the first contributor on a   │
// │     settled death drop, the first living entity on a world token. Its   │
// │     outcome grants the item, credits the coins or consumes the item; a  │
// │     collector at full life keeps a consuming token's item instead. Then │
// │     drop_collect plays the pickup animation for every viewer and the    │
// │     token is removed. A kill with no contributor drops nothing.         │
// │                                                                         │
// │  5. Expiry        — uncollected death drops despawn via ExpiresAt       │
// │     (updateBots). World tokens stay until collected; a consuming one    │
// │     comes back on its cell after the respawn time of its entity.        │
// └─────────────────────────────────────────────────────────────────────────┘

import (
	"cyberia-server/game/world"
	"math"
	"math/rand"
	"time"

	pb "cyberia-server/gen/proto"
	"cyberia-server/logx"
)

// ── Tunables ──────────────────────────────────────────────────────────────

const (
	// lootLandingWindow is the settle window: while the token flies from the
	// corpse to its cell it registers no collision. Must match the client launch
	// animation length (sent as launchMs in drop_spawn).
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

// worldTokenDims is the footprint of a world token.
var worldTokenDims = world.Vec2{X: lootTokenSize, Y: lootTokenSize}

// LootRespawn is a consumed world token waiting to come back on its cell.
type LootRespawn struct {
	ItemID  string
	Pos     world.Vec2
	Respawn time.Duration
	At      time.Time
}

// LootRespawns returns the pending respawns of the map code. A caller may
// change a row in place.
func (s *Content) LootRespawns(code string) []LootRespawn { return s.lootRespawns[code] }

// RespawnWait is the wait before a dead entity comes back.
func RespawnWait(m *world.Mortal) time.Duration {
	if m.RespawnDuration > 0 {
		return m.RespawnDuration
	}
	return respawnDurationDefaultMs * time.Millisecond
}

// PlacedRespawnDuration is the respawn time a map entity names; 0 takes the default.
func PlacedRespawnDuration(ent *pb.EntityMessage) time.Duration {
	return time.Duration(ent.GetRespawnDurationMs()) * time.Millisecond
}

// ── Contribution ledger ──────────────────────────────────────────────────

// RecordDamage records that a player or bot dealt damage to a mortal entity, so
// every contributor can race for its drops on death — uniformly, regardless of
// the victim's entity type (player, bot, or resource). Players collect on
// collision; contributing bots have the loot transferred to them on collision
// (see phaseLoot). Unknown casters (e.g. an already-expired projectile) grant no
// claim. The ledger is allocated lazily via the pointer. Caller holds s.mu.
func RecordDamage(ledger *map[world.EntityID]float64, mapState *world.MapState, casterID world.EntityID, amount float64) {
	if ledger == nil || casterID == 0 || amount <= 0 {
		return
	}
	if !mapState.IsKind(casterID, world.EntityPlayer) && !mapState.IsKind(casterID, world.EntityBot) {
		return // unknown caster — no claim
	}
	if *ledger == nil {
		*ledger = make(map[world.EntityID]float64, 4)
	}
	(*ledger)[casterID] += amount
}

// ContributorSet copies the ledger's contributor IDs into a membership set — the
// eligibility list a drop carries for its collection race. Returns nil when no
// one dealt damage, which suppresses the drop entirely (see spawnDrops).
func ContributorSet(ledger map[world.EntityID]float64) map[world.EntityID]struct{} {
	if len(ledger) == 0 {
		return nil
	}
	set := make(map[world.EntityID]struct{}, len(ledger))
	for id := range ledger {
		set[id] = struct{}{}
	}
	return set
}

// ── Death scatter ──────────────────────────────────────────────────────────

// SpawnDrops scatters each configured drop item (quantity 1) as a BehaviorDrop
// token around `center`, and — when the dying entity carried coins — one coin
// token whose quantity is the looted amount. Every player in `ledger` may race
// to collect the tokens once they settle. Shared by every dying
// content-authority entity (bots, resources, destructible obstacles). Caller
// holds s.mu.
func (s *Content) SpawnDrops(mapState *world.MapState, center world.Vec2, build EntityTypeDefaultConfig, coinAmount int, ledger map[world.EntityID]float64) {
	contributors := ContributorSet(ledger)
	// At least one contributor (player or bot) must have dealt damage for loot to
	// drop. A death nobody contributed to leaves nothing behind.
	if len(contributors) == 0 {
		return
	}

	scattered := 0
	for _, itemID := range build.DropItemIDs {
		if itemID == "" {
			continue
		}
		// Each id is rolled on its own, so a build can pair a common drop with a rare one and
		// have both decided independently. An id the build says nothing about always scatters.
		if chance := dropChance(build, itemID); chance < 1 && rand.Float64() >= chance {
			continue
		}
		s.spawnDropToken(mapState, center, itemID, stackQuantity(build, itemID), contributors)
		scattered++
	}

	// Coins scatter as a single quantity-bearing token rather than crediting the
	// killer directly (see economy.go).
	if coinAmount > 0 && s.CoinItemID != "" {
		s.spawnDropToken(mapState, center, s.CoinItemID, coinAmount, contributors)
	}

	logx.Debugf("[LOOT] scattered %d/%d item drop(s) + %d coins at (%.1f,%.1f) on %s (contributors=%d)",
		scattered, len(build.DropItemIDs), coinAmount, center.X, center.Y, mapState.Code(), len(contributors))
}

// spawnLootToken spawns one BehaviorDrop token showing itemID × quantity at
// pos with its brain. Its one active layer carries the quantity for the
// on-grid counter. Caller holds s.mu.
func (s *Content) spawnLootToken(mapState *world.MapState, pos, size world.Vec2, brain *world.Brain, respawn time.Duration) world.EntityID {
	id := s.NewEntityID()
	mapState.AddBrain(id, brain)
	mapState.Spawn(id, world.EntityData{Kind: world.EntityBot}, world.Transform{Pos: pos, Size: size},
		world.Appearance{Layers: []world.ObjectLayerState{{ItemID: brain.DropItemID, Active: true, Quantity: brain.DropQuantity}}})
	mapState.AddVitals(id, world.Vitals{MaxLife: s.EntityBaseMaxLife, Life: s.EntityBaseMaxLife})
	mapState.AddMotion(id, world.Motion{Direction: world.NONE, Mode: world.IDLE})
	mapState.AddAgent(id, world.Agent{Behavior: BehaviorDrop})
	mapState.AddMortal(id, &world.Mortal{RespawnDuration: respawn})
	return id
}

// spawnDropToken creates one death-drop token carrying (itemID × quantity),
// flings it to a walkable cell a random distance from `center` along a random
// direction, and announces the launch. Caller holds s.mu.
func (s *Content) spawnDropToken(mapState *world.MapState, center world.Vec2, itemID string, quantity int, contributors map[world.EntityID]struct{}) {
	if itemID == "" || quantity <= 0 {
		return
	}
	now := time.Now()
	size, outcome := lootTokenSize, world.LootOutcomeItem
	if itemID == s.CoinItemID {
		size, outcome = coinTokenSize, world.LootOutcomeCoin
	}
	tokenDims := world.Vec2{X: size, Y: size}

	angle := rand.Float64() * 2 * math.Pi
	// Squared roll biases toward the near end with a long tail to the far edge,
	// so drops feel randomly flung rather than evenly ringed.
	roll := rand.Float64()
	dist := lootScatterMinCells + roll*roll*(lootScatterMaxCells-lootScatterMinCells)
	targetCell := world.Cell{
		X: int(math.Round(center.X + math.Cos(angle)*dist)),
		Y: int(math.Round(center.Y + math.Sin(angle)*dist)),
	}

	cell := s.clampDropCell(mapState, targetCell, tokenDims)
	pos := world.Vec2{
		X: float64(cell.X) - tokenDims.X*0.5,
		Y: float64(cell.Y) - tokenDims.Y*0.5,
	}

	dropID := s.spawnLootToken(mapState, pos, tokenDims, &world.Brain{
		DropItemID:       itemID,
		DropQuantity:     quantity,
		LootOrigin:       world.LootOriginDeath,
		LootCollection:   world.LootCollectContributors,
		LootOutcome:      outcome,
		LootContributors: contributors,
		CollectableAt:    now.Add(lootLandingWindow),
		ExpiresAt:        now.Add(lootLifetime),
	}, 0)

	// Announce the launch so viewers play the corpse→cell parabola. The landing
	// point is the token center.
	landing := world.Vec2{X: pos.X + tokenDims.X*0.5, Y: pos.Y + tokenDims.Y*0.5}
	s.broadcastDropSpawn(mapState, dropID, center, landing)
}

// clampDropCell keeps a scatter target inside the grid and on a walkable cell,
// falling back to the closest walkable point (then the raw cell) so a drop is
// never lost to an obstacle.
func (s *Content) clampDropCell(mapState *world.MapState, cell world.Cell, dims world.Vec2) world.Cell {
	if cell.X < 0 {
		cell.X = 0
	} else if cell.X >= mapState.GridW() {
		cell.X = mapState.GridW() - 1
	}
	if cell.Y < 0 {
		cell.Y = 0
	} else if cell.Y >= mapState.GridH() {
		cell.Y = mapState.GridH() - 1
	}
	if mapState.Pathfinder() != nil {
		if mapState.Pathfinder().IsWalkable(cell.X, cell.Y, dims) {
			return cell
		}
		if c, err := mapState.Pathfinder().FindClosestWalkablePoint(cell, dims); err == nil {
			return c
		}
	}
	return cell
}

// ── World loot ─────────────────────────────────────────────────────────────

// lootBuild is the loot default whose live items itemID matches, or an empty
// build and false.
func (s *Content) lootBuild(itemID string) (EntityTypeDefaultConfig, bool) {
	items := []string{itemID}
	if build, _ := s.ResolveEntityDefaultBuild("loot", items); MatchesLiveItems(build, items) {
		return build, true
	}
	return EntityTypeDefaultConfig{}, false
}

// placeWorldLoot puts one world token of itemID at pos: collectable at once by
// the first living entity it benefits, never expiring, blocking nothing. The
// loot default its item matches makes it consume on pickup; else it grants the
// item. Caller holds s.mu.
func (s *Content) placeWorldLoot(mapState *world.MapState, pos world.Vec2, itemID string, respawn time.Duration) {
	brain := &world.Brain{
		DropItemID:     itemID,
		DropQuantity:   1,
		LootOrigin:     world.LootOriginWorld,
		LootCollection: world.LootCollectFirstContact,
		LootOutcome:    world.LootOutcomeItem,
	}
	if build, ok := s.lootBuild(itemID); ok && build.Behavior == BehaviorConsumeOnPickupHeal {
		brain.LootOutcome = world.LootOutcomeConsume
	}
	s.spawnLootToken(mapState, pos, worldTokenDims, brain, respawn)
}

// ScheduleLootRespawn brings a consumed world token back on its cell after its
// respawn time. Call it while the token still has its slot. Caller holds s.mu.
func (s *Content) ScheduleLootRespawn(mapState *world.MapState, dropID world.EntityID, now time.Time) {
	mortal := mapState.Mt(dropID)
	if s.lootRespawns == nil {
		s.lootRespawns = make(map[string][]LootRespawn)
	}
	code := mapState.Code()
	s.lootRespawns[code] = append(s.lootRespawns[code], LootRespawn{
		ItemID:  mapState.Br(dropID).DropItemID,
		Pos:     mapState.Tf(dropID).Pos,
		Respawn: mortal.RespawnDuration,
		At:      now.Add(RespawnWait(mortal)),
	})
}

// ForgetLootRespawns drops every pending world-token respawn. A world build
// replaces every map.
func (s *Content) ForgetLootRespawns() { s.lootRespawns = nil }

// RespawnWorldLoot places every consumed world token whose wait ran out.
func (s *Content) RespawnWorldLoot(mapState *world.MapState, now time.Time) {
	code := mapState.Code()
	waiting := s.lootRespawns[code][:0]
	for _, respawn := range s.lootRespawns[code] {
		if now.Before(respawn.At) {
			waiting = append(waiting, respawn)
			continue
		}
		s.placeWorldLoot(mapState, respawn.Pos, respawn.ItemID, respawn.Respawn)
	}
	if len(waiting) > 0 {
		s.lootRespawns[code] = waiting
	} else {
		delete(s.lootRespawns, code)
	}
}

// BuildLoot places a map-authored `loot` entity on its cell. It shows one item.
func (s *Content) BuildLoot(mapState *world.MapState, ent *pb.EntityMessage) {
	itemIDs := ent.GetObjectLayerItemIds()
	if len(itemIDs) != 1 {
		logx.Warnf("[LOOT] skip loot at (%d,%d) on %q: it shows %d items, not one",
			ent.GetInitCellX(), ent.GetInitCellY(), mapState.Code(), len(itemIDs))
		return
	}
	pos := world.Vec2{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())}
	s.placeWorldLoot(mapState, pos, itemIDs[0], PlacedRespawnDuration(ent))
}

// ── Authoritative collection ─────────────────────────────────────────────

// LootConsumes reports whether the collector consumes the token at once: a
// consuming token heals a collector that misses life; one at full life keeps the item.
func LootConsumes(drop *world.Brain, collector *world.Vitals) bool {
	return drop.LootOutcome == world.LootOutcomeConsume && canConsume(collector)
}

// ApplyLootToPlayer applies the token's outcome to the collecting player and
// reports whether the player received what it carries.
func (s *Content) ApplyLootToPlayer(mapState *world.MapState, drop *world.Brain, player *world.Session) bool {
	logx.Debugf("[LOOT] %s token %s taken by player %s", drop.LootOrigin, drop.DropItemID, player.ID)
	switch {
	case drop.LootOutcome == world.LootOutcomeCoin:
		// No coin-gain FCT — the amount is shown on the grid loot counter.
		s.AddCoins(player.ID, mapState, drop.DropQuantity)
	case LootConsumes(drop, mapState.Vt(player.ID)):
		s.ConsumeItem(mapState, player.ID, drop.DropItemID)
		return false
	default:
		if !s.ExchangeItems(player, nil, []Stack{{ItemID: drop.DropItemID, Qty: drop.DropQuantity}}) {
			return false
		}
		s.AdvancePlayerQuestsOnGain(player)
	}
	return true
}

// LootFits reports whether the player has room for what the token gives it:
// coins and a consumption take no slot, an item needs one.
func (s *Content) LootFits(mapState *world.MapState, drop *world.Brain, player *world.Session) bool {
	if drop.LootOutcome == world.LootOutcomeCoin || LootConsumes(drop, mapState.Vt(player.ID)) {
		return true
	}
	return s.InventoryFits(player, nil, []Stack{{ItemID: drop.DropItemID, Qty: drop.DropQuantity}})
}

// ApplyLootToBot applies the token's outcome to the collecting bot and reports
// whether the bot received what it carries.
func (s *Content) ApplyLootToBot(mapState *world.MapState, drop *world.Brain, bot world.EntityID) bool {
	logx.Debugf("[LOOT] %s token %s taken by bot %s", drop.LootOrigin, drop.DropItemID, bot)
	switch {
	case drop.LootOutcome == world.LootOutcomeCoin:
		s.AddCoins(bot, mapState, drop.DropQuantity)
	case LootConsumes(drop, mapState.Vt(bot)):
		s.ConsumeItem(mapState, bot, drop.DropItemID)
		return false
	default:
		s.GrantItemToBot(bot, mapState, drop.DropItemID, drop.DropQuantity)
	}
	return true
}

// LootAdmits reports whether the token's collection policy admits collectorID.
func LootAdmits(drop *world.Brain, collectorID world.EntityID) bool {
	if drop.LootCollection == world.LootCollectFirstContact {
		return true
	}
	_, ok := drop.LootContributors[collectorID]
	return ok
}

// ResolveDropCollector returns the first live, non-frozen player overlapping the
// token that its policy admits and that has room for it, or nil. Go's random
// map iteration order fairly breaks ties between simultaneous colliders.
func (s *Content) ResolveDropCollector(mapState *world.MapState, dropID world.EntityID) *world.Session {
	drop := mapState.Br(dropID)
	for player := range s.PlayersOn(mapState) {
		if mapState.Vt(player.ID).Ghost() || player.Frozen {
			continue
		}
		if !world.CheckAABBCollision(mapState.Tf(dropID).Pos, mapState.Tf(dropID).Size, mapState.Tf(player.ID).Pos, mapState.Tf(player.ID).Size) {
			continue
		}
		if LootAdmits(drop, player.ID) && s.LootFits(mapState, drop, player) {
			return player
		}
	}
	return nil
}

// ResolveBotDropCollector returns the first live loot-collecting bot overlapping
// the token that its policy admits, or 0.
func (s *Content) ResolveBotDropCollector(mapState *world.MapState, dropID world.EntityID) world.EntityID {
	drop := mapState.Br(dropID)
	for id := range mapState.OfKind(world.EntityBot) {
		if id == dropID || mapState.Vt(id).Ghost() || !BehaviorCollectsLoot(mapState.Ag(id).Behavior) {
			continue
		}
		if !LootAdmits(drop, id) {
			continue
		}
		if world.CheckAABBCollision(mapState.Tf(dropID).Pos, mapState.Tf(dropID).Size, mapState.Tf(id).Pos, mapState.Tf(id).Size) {
			return id
		}
	}
	return 0
}

// ── Pickup animation event ──────────────────────────────────────────────────

// broadcastDropSpawn delivers the launch event to every player watching the
// corpse (AOI covers the launch origin), so all viewers see the token arc out
// of the dead bot.
func (s *Content) broadcastDropSpawn(mapState *world.MapState, dropID world.EntityID, origin, landing world.Vec2) {
	drop := mapState.Br(dropID)
	payload := world.DropSpawn{
		DropID:   dropID,
		OriginX:  origin.X,
		OriginY:  origin.Y,
		LandingX: landing.X,
		LandingY: landing.Y,
		LaunchMs: int(lootLandingWindow / time.Millisecond),
		ItemID:   drop.DropItemID,
	}
	originRect := world.Rectangle{MinX: origin.X, MinY: origin.Y, MaxX: origin.X, MaxY: origin.Y}
	for player := range s.PlayersOn(mapState) {
		if world.RectsOverlap(player.AOI, originRect) {
			world.SendMessage(player, "drop_spawn", payload)
		}
	}
}

// BroadcastDropCollect delivers the pickup animation event to every player whose
// AOI covers the token, so all viewers see the token fly to the collector (a
// player or a bot).
func (s *Content) BroadcastDropCollect(mapState *world.MapState, dropID world.EntityID, collectorID world.EntityID, cx, cy float64, granted bool) {
	drop := mapState.Br(dropID)
	payload := world.DropCollect{
		DropID:      dropID,
		CollectorID: collectorID,
		WorldX:      cx,
		WorldY:      cy,
		ItemID:      drop.DropItemID,
		Granted:     granted,
	}
	dropRect := world.Rectangle{
		MinX: mapState.Tf(dropID).Pos.X, MinY: mapState.Tf(dropID).Pos.Y,
		MaxX: mapState.Tf(dropID).Pos.X + mapState.Tf(dropID).Size.X, MaxY: mapState.Tf(dropID).Pos.Y + mapState.Tf(dropID).Size.Y,
	}
	for player := range s.PlayersOn(mapState) {
		if player.ID == collectorID || world.RectsOverlap(player.AOI, dropRect) {
			world.SendMessage(player, "drop_collect", payload)
		}
	}
}

// GrantItemToBot transfers `qty` of a non-coin item into a contributing bot's
// inventory when it collects a scattered drop (appended inactive if new).
func (s *Content) GrantItemToBot(id world.EntityID, ms *world.MapState, itemID string, qty int) {
	if itemID == "" || qty <= 0 {
		return
	}
	ol := getOrCreateItemOL(&ms.Ap(id).Layers, itemID)
	ol.Quantity += qty
}
