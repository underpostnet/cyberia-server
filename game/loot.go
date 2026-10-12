package game

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
var worldTokenDims = Vec2{X: lootTokenSize, Y: lootTokenSize}

// lootRespawn is a consumed world token waiting to come back on its cell.
type lootRespawn struct {
	itemID  string
	pos     Vec2
	respawn time.Duration
	at      time.Time
}

// ── Token semantics ───────────────────────────────────────────────────────

// LootOrigin is where a token comes from.
type LootOrigin uint8

const (
	LootOriginDeath LootOrigin = iota // scattered from a dying entity
	LootOriginWorld                   // placed on the map by the world build
)

func (o LootOrigin) String() string {
	if o == LootOriginWorld {
		return "world"
	}
	return "death"
}

// LootCollection is who may collect a token.
type LootCollection uint8

const (
	LootCollectContributors LootCollection = iota // a damage contributor only
	LootCollectFirstContact                       // any living player or loot-collecting bot
)

// LootOutcome is what a collection does.
type LootOutcome uint8

const (
	LootOutcomeItem    LootOutcome = iota // the item enters the collector's inventory
	LootOutcomeCoin                       // the quantity credits the collector's wallet
	LootOutcomeConsume                    // a collector that misses life consumes the item at once (consumable.go)
)

// ── Contribution ledger ──────────────────────────────────────────────────

// recordDamage records that a player or bot dealt damage to a mortal entity, so
// every contributor can race for its drops on death — uniformly, regardless of
// the victim's entity type (player, bot, or resource). Players collect on
// collision; contributing bots have the loot transferred to them on collision
// (see phaseLoot). Unknown casters (e.g. an already-expired projectile) grant no
// claim. The ledger is allocated lazily via the pointer. Caller holds s.mu.
func recordDamage(ledger *map[EntityID]float64, mapState *MapState, casterID EntityID, amount float64) {
	if ledger == nil || casterID == 0 || amount <= 0 {
		return
	}
	_, isPlayer := mapState.players[casterID]
	_, isBot := mapState.brains[casterID]
	if !isPlayer && !isBot {
		return // unknown caster — no claim
	}
	if *ledger == nil {
		*ledger = make(map[EntityID]float64, 4)
	}
	(*ledger)[casterID] += amount
}

// contributorSet copies the ledger's contributor IDs into a membership set — the
// eligibility list a drop carries for its collection race. Returns nil when no
// one dealt damage, which suppresses the drop entirely (see spawnDrops).
func contributorSet(ledger map[EntityID]float64) map[EntityID]struct{} {
	if len(ledger) == 0 {
		return nil
	}
	set := make(map[EntityID]struct{}, len(ledger))
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
// content-authority entity (bots, resources, destructible obstacles). Caller
// holds s.mu.
func (s *GameServer) spawnDrops(mapState *MapState, center Vec2, build EntityTypeDefaultConfig, coinAmount int, ledger map[EntityID]float64) {
	contributors := contributorSet(ledger)
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
	if coinAmount > 0 && s.coinItemID != "" {
		s.spawnDropToken(mapState, center, s.coinItemID, coinAmount, contributors)
	}

	logx.Debugf("[LOOT] scattered %d/%d item drop(s) + %d coins at (%.1f,%.1f) on %s (contributors=%d)",
		scattered, len(build.DropItemIDs), coinAmount, center.X, center.Y, mapState.code, len(contributors))
}

// spawnLootToken spawns one BehaviorDrop token showing itemID × quantity at
// pos with its brain. Its one active layer carries the quantity for the
// on-grid counter. Caller holds s.mu.
func (s *GameServer) spawnLootToken(mapState *MapState, pos, size Vec2, brain *Brain, respawn time.Duration) EntityID {
	id := s.newEntityID()
	mapState.addBrain(id, brain)
	mapState.spawn(id, EntityData{Kind: EntityBot}, Transform{Pos: pos, Size: size},
		Appearance{Layers: []ObjectLayerState{{ItemID: brain.DropItemID, Active: true, Quantity: brain.DropQuantity}}})
	mapState.addVitals(id, Vitals{MaxLife: s.entityBaseMaxLife, Life: s.entityBaseMaxLife})
	mapState.addMotion(id, Motion{Direction: NONE, Mode: IDLE})
	mapState.addAgent(id, Agent{Behavior: BehaviorDrop})
	mapState.addMortal(id, &Mortal{RespawnDuration: respawn})
	return id
}

// spawnDropToken creates one death-drop token carrying (itemID × quantity),
// flings it to a walkable cell a random distance from `center` along a random
// direction, and announces the launch. Caller holds s.mu.
func (s *GameServer) spawnDropToken(mapState *MapState, center Vec2, itemID string, quantity int, contributors map[EntityID]struct{}) {
	if itemID == "" || quantity <= 0 {
		return
	}
	now := time.Now()
	size, outcome := lootTokenSize, LootOutcomeItem
	if itemID == s.coinItemID {
		size, outcome = coinTokenSize, LootOutcomeCoin
	}
	tokenDims := Vec2{X: size, Y: size}

	angle := rand.Float64() * 2 * math.Pi
	// Squared roll biases toward the near end with a long tail to the far edge,
	// so drops feel randomly flung rather than evenly ringed.
	roll := rand.Float64()
	dist := lootScatterMinCells + roll*roll*(lootScatterMaxCells-lootScatterMinCells)
	targetCell := Cell{
		X: int(math.Round(center.X + math.Cos(angle)*dist)),
		Y: int(math.Round(center.Y + math.Sin(angle)*dist)),
	}

	cell := s.clampDropCell(mapState, targetCell, tokenDims)
	pos := Vec2{
		X: float64(cell.X) - tokenDims.X*0.5,
		Y: float64(cell.Y) - tokenDims.Y*0.5,
	}

	dropID := s.spawnLootToken(mapState, pos, tokenDims, &Brain{
		DropItemID:       itemID,
		DropQuantity:     quantity,
		LootOrigin:       LootOriginDeath,
		LootCollection:   LootCollectContributors,
		LootOutcome:      outcome,
		LootContributors: contributors,
		CollectableAt:    now.Add(lootLandingWindow),
		ExpiresAt:        now.Add(lootLifetime),
	}, 0)

	// Announce the launch so viewers play the corpse→cell parabola. The landing
	// point is the token center.
	landing := Vec2{X: pos.X + tokenDims.X*0.5, Y: pos.Y + tokenDims.Y*0.5}
	s.broadcastDropSpawn(mapState, dropID, center, landing)
}

// clampDropCell keeps a scatter target inside the grid and on a walkable cell,
// falling back to the closest walkable point (then the raw cell) so a drop is
// never lost to an obstacle.
func (s *GameServer) clampDropCell(mapState *MapState, cell Cell, dims Vec2) Cell {
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

// ── World loot ─────────────────────────────────────────────────────────────

// lootBuild is the loot default whose live items itemID matches, or an empty
// build and false.
func (s *GameServer) lootBuild(itemID string) (EntityTypeDefaultConfig, bool) {
	items := []string{itemID}
	if build, _ := s.resolveEntityDefaultBuild("loot", items); matchesLiveItems(build, items) {
		return build, true
	}
	return EntityTypeDefaultConfig{}, false
}

// placeWorldLoot puts one world token of itemID at pos: collectable at once by
// the first living entity it benefits, never expiring, blocking nothing. The
// loot default its item matches makes it consume on pickup; else it grants the
// item. Caller holds s.mu.
func (s *GameServer) placeWorldLoot(mapState *MapState, pos Vec2, itemID string, respawn time.Duration) {
	brain := &Brain{
		DropItemID:     itemID,
		DropQuantity:   1,
		LootOrigin:     LootOriginWorld,
		LootCollection: LootCollectFirstContact,
		LootOutcome:    LootOutcomeItem,
	}
	if build, ok := s.lootBuild(itemID); ok && build.Behavior == BehaviorConsumeOnPickupHeal {
		brain.LootOutcome = LootOutcomeConsume
	}
	s.spawnLootToken(mapState, pos, worldTokenDims, brain, respawn)
}

// scheduleLootRespawn brings a consumed world token back on its cell after its
// respawn time. Call it while the token still has its slot. Caller holds s.mu.
func scheduleLootRespawn(mapState *MapState, dropID EntityID, now time.Time) {
	mortal := mapState.mt(dropID)
	mapState.lootRespawns = append(mapState.lootRespawns, lootRespawn{
		itemID:  mapState.br(dropID).DropItemID,
		pos:     mapState.tf(dropID).Pos,
		respawn: mortal.RespawnDuration,
		at:      now.Add(mortal.respawnDuration()),
	})
}

// respawnWorldLoot places every consumed world token whose wait ran out.
func (s *GameServer) respawnWorldLoot(mapState *MapState, now time.Time) {
	waiting := mapState.lootRespawns[:0]
	for _, respawn := range mapState.lootRespawns {
		if now.Before(respawn.at) {
			waiting = append(waiting, respawn)
			continue
		}
		s.placeWorldLoot(mapState, respawn.pos, respawn.itemID, respawn.respawn)
	}
	mapState.lootRespawns = waiting
}

// buildLoot places a map-authored `loot` entity on its cell. It shows one item.
func (s *GameServer) buildLoot(mapState *MapState, ent *pb.EntityMessage) {
	itemIDs := ent.GetObjectLayerItemIds()
	if len(itemIDs) != 1 {
		logx.Warnf("[LOOT] skip loot at (%d,%d) on %q: it shows %d items, not one",
			ent.GetInitCellX(), ent.GetInitCellY(), mapState.code, len(itemIDs))
		return
	}
	pos := Vec2{X: float64(ent.GetInitCellX()), Y: float64(ent.GetInitCellY())}
	s.placeWorldLoot(mapState, pos, itemIDs[0], placedRespawnDuration(ent))
}

// ── Authoritative collection ─────────────────────────────────────────────

// phaseLoot resolves token collisions for one tick. The first collector the
// token's collection policy admits takes it, players before bots: its outcome
// applies, the pickup animation event goes to every viewer and the token is
// removed in the same step, so one token applies once. Caller holds s.mu.
func (s *GameServer) phaseLoot(tick uint32, mapState *MapState) {
	_ = tick
	now := time.Now()
	s.respawnWorldLoot(mapState, now)

	for dropID, drop := range mapState.brains {
		if !behaviorIsInert(mapState.ag(dropID).Behavior) {
			continue
		}
		// Mid-launch tokens register no collision until they settle on the cell.
		if now.Before(drop.CollectableAt) {
			continue
		}

		cx := mapState.tf(dropID).Pos.X + mapState.tf(dropID).Size.X*0.5
		cy := mapState.tf(dropID).Pos.Y + mapState.tf(dropID).Size.Y*0.5

		var collectorID EntityID
		var granted bool
		if player := s.resolveDropCollector(mapState, dropID); player != nil {
			granted, collectorID = s.applyLootToPlayer(mapState, drop, player), player.ID
		} else if bot := s.resolveBotDropCollector(mapState, dropID); bot != 0 {
			granted, collectorID = s.applyLootToBot(mapState, drop, bot), bot
		} else {
			continue
		}
		s.broadcastDropCollect(mapState, dropID, collectorID, cx, cy, granted)
		if drop.LootOrigin == LootOriginWorld && drop.LootOutcome == LootOutcomeConsume {
			scheduleLootRespawn(mapState, dropID, now)
		}
		mapState.despawn(dropID)
	}
}

// lootConsumes reports whether the collector consumes the token at once: a
// consuming token heals a collector that misses life; one at full life keeps the item.
func lootConsumes(drop *Brain, collector *Vitals) bool {
	return drop.LootOutcome == LootOutcomeConsume && canConsume(collector)
}

// applyLootToPlayer applies the token's outcome to the collecting player and
// reports whether the player received what it carries.
func (s *GameServer) applyLootToPlayer(mapState *MapState, drop *Brain, player *PlayerState) bool {
	logx.Debugf("[LOOT] %s token %s taken by player %s", drop.LootOrigin, drop.DropItemID, player.ID)
	switch {
	case drop.LootOutcome == LootOutcomeCoin:
		// No coin-gain FCT — the amount is shown on the grid loot counter.
		s.addCoins(player.ID, mapState, drop.DropQuantity)
	case lootConsumes(drop, mapState.vt(player.ID)):
		s.consumeItem(mapState, player.ID, drop.DropItemID)
		return false
	default:
		s.addPlayerItem(player, drop.DropItemID, drop.DropQuantity)
		s.advancePlayerQuestsOnGain(player)
	}
	return true
}

// applyLootToBot applies the token's outcome to the collecting bot and reports
// whether the bot received what it carries.
func (s *GameServer) applyLootToBot(mapState *MapState, drop *Brain, bot EntityID) bool {
	logx.Debugf("[LOOT] %s token %s taken by bot %s", drop.LootOrigin, drop.DropItemID, bot)
	switch {
	case drop.LootOutcome == LootOutcomeCoin:
		s.addCoins(bot, mapState, drop.DropQuantity)
	case lootConsumes(drop, mapState.vt(bot)):
		s.consumeItem(mapState, bot, drop.DropItemID)
		return false
	default:
		s.grantItemToBot(bot, mapState, drop.DropItemID, drop.DropQuantity)
	}
	return true
}

// lootAdmits reports whether the token's collection policy admits collectorID.
func lootAdmits(drop *Brain, collectorID EntityID) bool {
	if drop.LootCollection == LootCollectFirstContact {
		return true
	}
	_, ok := drop.LootContributors[collectorID]
	return ok
}

// resolveDropCollector returns the first live, non-frozen player overlapping the
// token that its policy admits, or nil. Go's random map
// iteration order fairly breaks ties between simultaneous colliders.
func (s *GameServer) resolveDropCollector(mapState *MapState, dropID EntityID) *PlayerState {
	drop := mapState.br(dropID)
	for _, player := range mapState.players {
		if mapState.vt(player.ID).ghost() || player.Frozen {
			continue
		}
		if !checkAABBCollision(mapState.tf(dropID).Pos, mapState.tf(dropID).Size, mapState.tf(player.ID).Pos, mapState.tf(player.ID).Size) {
			continue
		}
		if lootAdmits(drop, player.ID) {
			return player
		}
	}
	return nil
}

// resolveBotDropCollector returns the first live loot-collecting bot overlapping
// the token that its policy admits, or 0.
func (s *GameServer) resolveBotDropCollector(mapState *MapState, dropID EntityID) EntityID {
	drop := mapState.br(dropID)
	for id := range mapState.brains {
		if id == dropID || mapState.vt(id).ghost() || !behaviorCollectsLoot(mapState.ag(id).Behavior) {
			continue
		}
		if !lootAdmits(drop, id) {
			continue
		}
		if checkAABBCollision(mapState.tf(dropID).Pos, mapState.tf(dropID).Size, mapState.tf(id).Pos, mapState.tf(id).Size) {
			return id
		}
	}
	return 0
}

// ── Pickup animation event ──────────────────────────────────────────────────

// broadcastDropSpawn delivers the launch event to every player watching the
// corpse (AOI covers the launch origin), so all viewers see the token arc out
// of the dead bot.
func (s *GameServer) broadcastDropSpawn(mapState *MapState, dropID EntityID, origin, landing Vec2) {
	drop := mapState.br(dropID)
	payload := DropSpawn{
		DropID:   dropID,
		OriginX:  origin.X,
		OriginY:  origin.Y,
		LandingX: landing.X,
		LandingY: landing.Y,
		LaunchMs: int(lootLandingWindow / time.Millisecond),
		ItemID:   drop.DropItemID,
	}
	originRect := Rectangle{MinX: origin.X, MinY: origin.Y, MaxX: origin.X, MaxY: origin.Y}
	for _, player := range mapState.players {
		if rectsOverlap(player.AOI, originRect) {
			sendMessage(player, "drop_spawn", payload)
		}
	}
}

// broadcastDropCollect delivers the pickup animation event to every player whose
// AOI covers the token, so all viewers see the token fly to the collector (a
// player or a bot).
func (s *GameServer) broadcastDropCollect(mapState *MapState, dropID EntityID, collectorID EntityID, cx, cy float64, granted bool) {
	drop := mapState.br(dropID)
	payload := DropCollect{
		DropID:      dropID,
		CollectorID: collectorID,
		WorldX:      cx,
		WorldY:      cy,
		ItemID:      drop.DropItemID,
		Granted:     granted,
	}
	dropRect := Rectangle{
		MinX: mapState.tf(dropID).Pos.X, MinY: mapState.tf(dropID).Pos.Y,
		MaxX: mapState.tf(dropID).Pos.X + mapState.tf(dropID).Size.X, MaxY: mapState.tf(dropID).Pos.Y + mapState.tf(dropID).Size.Y,
	}
	for _, player := range mapState.players {
		if player.ID == collectorID || rectsOverlap(player.AOI, dropRect) {
			sendMessage(player, "drop_collect", payload)
		}
	}
}
