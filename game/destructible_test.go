package game

import (
	"fmt"
	"iter"
	"math"
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

// destructibleFixture is the movement map with a damaging bullet item and the
// obstacle defaults a test resolves against.
func destructibleFixture(t *testing.T, builds ...EntityTypeDefaultConfig) (*GameServer, *MapState, *Session) {
	t.Helper()
	s, ms, player := movementFixture(t)
	s.entityDefaults = map[string]EntityTypeDefaultConfig{}
	s.entityDefaultBuilds = builds
	s.entityBaseMaxLife = 10
	s.lastEntityID = player.ID // built entities take ids after the player
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{
		"bullet": {Data: ObjectLayerData{Stats: Stats{Effect: 4}, Item: Item{Type: "skill"}}},
	}); err != nil {
		t.Fatal(err)
	}
	return s, ms, player
}

// placeObstacle builds one 2×2 obstacle at (10, 10) and returns its id.
func placeObstacle(s *GameServer, ms *MapState, maxLife float64, itemIDs ...string) EntityID {
	s.buildObstacle(ms, &pb.EntityMessage{ObjectLayerItemIds: itemIDs, InitCellX: 10, InitCellY: 10, DimX: 2, DimY: 2, MaxLife: maxLife})
	return s.lastEntityID
}

// shot is one bullet of the caster's at the point, with a slot in the map.
func shot(ms *MapState, caster *Session, at Vec2) EntityID {
	const bullet EntityID = 900
	ms.addBrain(bullet, &Brain{})
	ms.spawn(bullet, EntityData{Kind: EntityBot}, Transform{Pos: at, Size: Vec2{X: 1, Y: 1}},
		Appearance{Layers: []ObjectLayerState{{ItemID: "bullet", Active: true, Quantity: 1}}})
	ms.addVitals(bullet, Vitals{MaxLife: 100, Life: 100})
	ms.addMotion(bullet, Motion{})
	ms.addAgent(bullet, Agent{Behavior: BehaviorSkill, CasterID: caster.ID})
	ms.addMortal(bullet, &Mortal{})
	return bullet
}

// fire lands one bullet of the caster's at the point, then removes it.
func fire(s *GameServer, ms *MapState, caster *Session, at Vec2) {
	bullet := shot(ms, caster, at)
	s.handleSkillCollisions(ms)
	ms.despawn(bullet)
}

// addBot gives bot id a slot at tf with the brain, walking in the motion.
func addBot(ms *MapState, id EntityID, tf Transform, brain *Brain, motion Motion, behavior string) *Brain {
	ms.addBrain(id, brain)
	ms.spawn(id, EntityData{Kind: EntityBot}, tf, Appearance{})
	ms.addVitals(id, Vitals{Life: 100, MaxLife: 100})
	ms.addMotion(id, motion)
	ms.addAgent(id, Agent{Behavior: behavior})
	ms.addMortal(id, &Mortal{})
	return brain
}

func drops(ms *MapState) []EntityID {
	var out []EntityID
	for id := range ms.brains {
		if ms.ag(id).Behavior == BehaviorDrop {
			out = append(out, id)
		}
	}
	return out
}

func countOf(seq iter.Seq[EntityID]) int {
	n := 0
	for range seq {
		n++
	}
	return n
}

var crystalRock = EntityTypeDefaultConfig{
	EntityType:          "obstacle",
	LiveItemIDs:         []string{"crystal"},
	DeadItemIDs:         []string{"rubble"},
	DropItemIDs:         []string{"shard"},
	DefaultObjectLayers: []ObjectLayerState{{ItemID: "shard", Quantity: 3}},
	Behavior:            BehaviorDestructible,
}

// An obstacle no default matches stays ordinary, even when the first obstacle
// default of the world is destructible.
func TestOrdinaryObstacleBlocksAndTakesNoDamage(t *testing.T) {
	s, ms, player := destructibleFixture(t, crystalRock)
	id := placeObstacle(s, ms, 0, "wall")

	if ms.data[ms.slot(id)].Flags&FlagBlocks == 0 || countOf(ms.destructibles()) != 0 {
		t.Fatal("an unmatched obstacle became destructible")
	}
	fire(s, ms, player, Vec2{X: 10, Y: 10})
	if ms.pathfinder.isWalkable(10, 10, Vec2{X: 1, Y: 1}) || len(drops(ms)) != 0 {
		t.Fatal("an ordinary obstacle stopped blocking or dropped loot")
	}
}

// Behavior and lifecycle come from the same most specific default.
func TestTheMostSpecificObstacleDefaultDecidesDestructibility(t *testing.T) {
	mossy := EntityTypeDefaultConfig{EntityType: "obstacle", LiveItemIDs: []string{"crystal", "moss"}}
	s, ms, _ := destructibleFixture(t, crystalRock, mossy)

	if !ms.has(placeObstacle(s, ms, 0, "crystal"), maskVitals) {
		t.Fatal("the crystal default binds destructible")
	}
	if ms.has(placeObstacle(s, ms, 0, "crystal", "moss"), maskVitals) {
		t.Fatal("the more specific mossy default binds no behavior")
	}
}

// A coloured placement keeps its live item, so it binds the behavior whether or
// not an object layer for the item is cached.
func TestAColouredObstacleCarryingItsLiveItemIsDestructible(t *testing.T) {
	s, ms, _ := destructibleFixture(t, crystalRock)
	s.buildObstacle(ms, &pb.EntityMessage{
		ObjectLayerItemIds: []string{"crystal"}, InitCellX: 4, InitCellY: 4, DimX: 2, DimY: 2, ColorA: 255,
	})
	if countOf(ms.destructibles()) != 1 {
		t.Fatal("a coloured crystal obstacle must bind destructible")
	}
}

func TestDestructibleObstacleDepletesOnceAndRespawnsOverWhoStandsInIt(t *testing.T) {
	s, ms, player := destructibleFixture(t, crystalRock)
	probe := shot(ms, player, Vec2{})
	maxLife := 2.5 * s.CalculateStats(probe, ms).Effect
	ms.despawn(probe)
	obs := placeObstacle(s, ms, maxLife, "crystal")
	if !ms.has(obs, maskVitals) || ms.vt(obs).Life != maxLife || ms.vt(obs).MaxLife != maxLife {
		t.Fatalf("placed life: %+v", obs)
	}
	cell := Vec2{X: 1, Y: 1}
	if ms.pathfinder.isWalkable(11, 11, cell) {
		t.Fatal("a live destructible obstacle blocks its cells")
	}

	fire(s, ms, player, Vec2{X: 10, Y: 10})
	if ms.vt(obs).Life >= maxLife || ms.vt(obs).ghost() || ms.pathfinder.isWalkable(11, 11, cell) {
		t.Fatalf("a hit damages it and it keeps blocking: %+v", ms.vt(obs))
	}
	for hits := 1; !ms.vt(obs).ghost(); hits++ {
		if hits > 3 {
			t.Fatal("three hits deplete the obstacle")
		}
		fire(s, ms, player, Vec2{X: 10, Y: 10})
	}
	if !ms.vt(obs).ghost() || ms.vt(obs).Life != 0 || !ms.pathfinder.isWalkable(10, 10, Vec2{X: 2, Y: 2}) {
		t.Fatalf("depletion frees the cells: %+v", ms.vt(obs))
	}
	if active := activeObjectLayerItemIDs(ms.ap(obs).Layers); len(active) != 1 || active[0] != "rubble" {
		t.Fatalf("depleted visual: %v", active)
	}
	scattered := drops(ms)
	if len(scattered) != 1 || ms.ap(scattered[0]).Layers[0].ItemID != "shard" || ms.ap(scattered[0]).Layers[0].Quantity != 3 {
		t.Fatalf("drops: %v", scattered)
	}
	if _, ok := ms.br(scattered[0]).LootContributors[player.ID]; !ok {
		t.Fatal("the damage contributor may collect the drop")
	}
	if ms.mt(player.ID).Progression.XP != 0 {
		t.Fatal("an obstacle awards no XP")
	}

	fire(s, ms, player, Vec2{X: 10, Y: 10})
	if len(drops(ms)) != 1 || ms.vt(obs).Life != 0 {
		t.Fatal("a depleted obstacle takes no hit and drops nothing again")
	}

	ms.vt(obs).RespawnAt = time.Now().Add(-time.Second)
	ptf := ms.tf(player.ID)
	ptf.Pos, player.Path, ms.mo(player.ID).Mode = Vec2{X: 10, Y: 10}, []Cell{{X: 12, Y: 12}}, WALKING
	const botID EntityID = 901
	bot := addBot(ms, botID, Transform{Pos: Vec2{X: 10.5, Y: 10.5}, Size: cell},
		&Brain{Path: []Cell{{X: 11, Y: 11}}, CurrentTargetPlayer: player.ID}, Motion{Mode: WALKING}, BehaviorHostile)
	projectile := shot(ms, player, Vec2{X: 10, Y: 10})
	ptf = ms.tf(player.ID) // a spawn may move the transform array
	s.handleRespawns(ms)
	if ms.vt(obs).ghost() || ms.vt(obs).Life != ms.vt(obs).MaxLife || ms.pathfinder.isWalkable(10, 10, cell) {
		t.Fatalf("the obstacle respawns on its timer and blocks again: %+v", ms.vt(obs))
	}
	otf := ms.tf(obs)
	if checkAABBCollision(otf.Pos, otf.Size, ptf.Pos, ptf.Size) ||
		!ms.pathfinder.isWalkable(int(ptf.Pos.X), int(ptf.Pos.Y), ptf.Size) ||
		math.Hypot(ptf.Pos.X-11, ptf.Pos.Y-11) > 4 || ms.mo(player.ID).Mode != IDLE || len(player.Path) != 0 {
		t.Fatalf("the player moves to a free spot near the obstacle and stops: %+v", ptf.Pos)
	}
	for other := range ms.brains {
		btf := ms.tf(other)
		if other != projectile && checkAABBCollision(otf.Pos, otf.Size, btf.Pos, btf.Size) {
			t.Fatalf("bot %s still stands in the obstacle", other)
		}
	}
	if ms.tf(projectile).Pos != (Vec2{X: 10, Y: 10}) {
		t.Fatal("a projectile keeps its course")
	}
	btf := ms.tf(botID)
	if ms.mo(botID).Mode != IDLE || len(bot.Path) != 0 || bot.CurrentTargetPlayer != 0 ||
		bot.lastPursuitTargetPos != (Cell{-1, -1}) ||
		!ms.pathfinder.isWalkable(int(btf.Pos.X), int(btf.Pos.Y), btf.Size) {
		t.Fatalf("the displaced bot clears its movement and pursuit: %+v", bot)
	}
	if active := activeObjectLayerItemIDs(ms.ap(obs).Layers); len(active) != 1 || active[0] != "crystal" {
		t.Fatalf("respawned visual: %v", active)
	}
	s.handleRespawns(ms)
	if ms.pathfinder.grid[10][10] != 1 {
		t.Fatal("a live obstacle blocks its cells once")
	}
}

func TestRestoredObstacleReplansCrossingPaths(t *testing.T) {
	s, ms, player := destructibleFixture(t, crystalRock)
	obs := placeObstacle(s, ms, 1, "crystal")
	otf := *ms.tf(obs)
	ptf := ms.tf(player.ID)
	fire(s, ms, player, otf.Pos)
	ms.vt(obs).RespawnAt = time.Now().Add(time.Minute)
	tapTick(s, ms, player, tap(7, 15, 15))
	if !pathCrossesObstacle(player.Path, ptf.Size, otf) {
		t.Fatal("the initial route crosses the depleted obstacle")
	}
	const botID EntityID = 902
	bot := addBot(ms, botID, Transform{Pos: Vec2{X: 9, Y: 10}, Size: ptf.Size},
		&Brain{Path: []Cell{{X: 10, Y: 10}, {X: 12, Y: 10}}, CurrentTargetPlayer: player.ID}, Motion{Mode: WALKING}, BehaviorHostile)
	ptf = ms.tf(player.ID) // a spawn may move the transform array
	s.handleRespawns(ms)
	if !ms.vt(obs).ghost() || !pathCrossesObstacle(player.Path, ptf.Size, otf) {
		t.Fatal("the obstacle stays passable until its timer ends")
	}
	ms.vt(obs).RespawnAt = time.Now().Add(-time.Second)
	before := ptf.Pos
	s.handleRespawns(ms)
	if ptf.Pos != before || ms.mo(player.ID).Mode != WALKING || player.TargetPos != (Cell{X: 15, Y: 15}) ||
		len(player.Path) == 0 {
		t.Fatalf("the player keeps its destination: %+v", player)
	}
	for _, cell := range player.Path {
		if !ms.pathfinder.isWalkable(cell.X, cell.Y, ptf.Size) {
			t.Fatalf("the new path crosses blocked cell %v", cell)
		}
	}
	if ms.mo(botID).Mode != IDLE || len(bot.Path) != 0 || bot.CurrentTargetPlayer != 0 {
		t.Fatal("the bot clears its blocked route before its next AI tick")
	}
}

func TestObstacleClearanceUsesEntityDimensionsAtMapEdges(t *testing.T) {
	for _, pos := range []Vec2{{X: 0, Y: 0}, {X: 18, Y: 18}} {
		t.Run(fmt.Sprintf("%g-%g", pos.X, pos.Y), func(t *testing.T) {
			s, ms, player := movementFixture(t)
			obs := Transform{Pos: pos, Size: Vec2{X: 2, Y: 2}}
			ms.pathfinder.occupy(obs.Pos, obs.Size, 1)
			ptf := ms.tf(player.ID)
			ptf.Size = Vec2{X: 3, Y: 2}
			ptf.Pos = Vec2{X: math.Min(pos.X, 17), Y: pos.Y}
			player.Path, ms.mo(player.ID).Mode = []Cell{{X: 10, Y: 10}}, WALKING
			s.clearObstacleCells(ms, obs)
			if checkAABBCollision(obs.Pos, obs.Size, ptf.Pos, ptf.Size) ||
				!ms.pathfinder.isWalkable(int(ptf.Pos.X), int(ptf.Pos.Y), ptf.Size) ||
				ms.mo(player.ID).Mode != IDLE || len(player.Path) != 0 {
				t.Fatalf("the full player footprint clears the obstacle at %v: %v", pos, ptf.Pos)
			}
		})
	}
}

func TestObstacleRespawnPushesEntitiesOutward(t *testing.T) {
	for _, tc := range []struct {
		name string
		pos  Vec2
		want Vec2
	}{
		{"north", Vec2{11, 10}, Vec2{11, 8}},
		{"north-east", Vec2{12, 10}, Vec2{14, 8}},
		{"east", Vec2{12, 11}, Vec2{14, 11}},
		{"south-east", Vec2{12, 12}, Vec2{14, 14}},
		{"south", Vec2{11, 12}, Vec2{11, 14}},
		{"south-west", Vec2{10, 12}, Vec2{8, 14}},
		{"west", Vec2{10, 11}, Vec2{8, 11}},
		{"north-west", Vec2{10, 10}, Vec2{8, 8}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, ms, player := movementFixture(t)
			obs := Transform{Pos: Vec2{10, 10}, Size: Vec2{4, 4}}
			ms.pathfinder.occupy(obs.Pos, obs.Size, 1)
			ptf := ms.tf(player.ID)
			ptf.Pos, ptf.Size, ms.mo(player.ID).Direction = tc.pos, Vec2{2, 2}, DOWN
			const botID EntityID = 903
			addBot(ms, botID, Transform{Pos: tc.pos, Size: ptf.Size}, &Brain{}, Motion{Direction: UP}, "")
			ptf = ms.tf(player.ID) // a spawn may move the transform array
			s.clearObstacleCells(ms, obs)
			if ptf.Pos != tc.want || ms.tf(botID).Pos != tc.want {
				t.Fatalf("outward push: player %v, bot %v, want %v", ptf.Pos, ms.tf(botID).Pos, tc.want)
			}
			if !ms.pathfinder.isWalkable(int(ptf.Pos.X), int(ptf.Pos.Y), ptf.Size) {
				t.Fatal("the full footprint must land on free cells")
			}
		})
	}
}

func TestObstaclePushAtCenterUsesFacing(t *testing.T) {
	for _, tc := range []struct {
		direction Direction
		want      Cell
	}{
		{RIGHT, Cell{14, 11}},
		{LEFT, Cell{8, 11}},
		{UP, Cell{11, 8}},
		{DOWN, Cell{11, 14}},
		{NONE, Cell{11, 14}},
	} {
		_, ms, _ := movementFixture(t)
		obs := Transform{Pos: Vec2{10, 10}, Size: Vec2{4, 4}}
		ms.pathfinder.occupy(obs.Pos, obs.Size, 1)
		entity := Transform{Pos: Vec2{11, 11}, Size: Vec2{2, 2}}
		cell, err := obstaclePushCell(ms, obs, entity, tc.direction)
		if err != nil || cell != tc.want {
			t.Fatalf("center push for direction %v: %v, %v; want %v", tc.direction, cell, err, tc.want)
		}
	}
}

func TestObstaclePushFindsFreeCellsBesideBlockedExit(t *testing.T) {
	_, ms, player := movementFixture(t)
	obs := Transform{Pos: Vec2{10, 10}, Size: Vec2{2, 2}}
	ms.pathfinder.occupy(obs.Pos, obs.Size, 1)
	ms.pathfinder.occupy(Vec2{12, 10}, Vec2{1, 2}, 1)
	ptf := ms.tf(player.ID)
	ptf.Pos = Vec2{11, 10.5}
	cell, err := obstaclePushCell(ms, obs, *ptf, NONE)
	if err != nil || cell.X != 13 || !ms.pathfinder.isWalkable(cell.X, cell.Y, ptf.Size) {
		t.Fatalf("a blocked exit keeps the push on the outward side: %v, %v", cell, err)
	}
}

// A resource depleted by the same hits still earns its objective XP.
func TestResourceDepletionKeepsItsObjectiveXP(t *testing.T) {
	s, ms, player := destructibleFixture(t)
	s.buildResource(ms, &pb.EntityMessage{InitCellX: 10, InitCellY: 10, DimX: 2, DimY: 2, MaxLife: 1, ColorA: 255})
	for res := range ms.ofKind(EntityResource) {
		for hits := 0; !ms.vt(res).ghost(); hits++ {
			if hits == 100 {
				t.Fatal("the resource never depletes")
			}
			fire(s, ms, player, Vec2{X: 10, Y: 10})
		}
	}
	if ms.mt(player.ID).Progression.XP == 0 {
		t.Fatal("a depleted resource awards objective XP")
	}
}

func TestDestructibleObstacleDropsOnlyWhatItsDefaultRolls(t *testing.T) {
	bare := EntityTypeDefaultConfig{EntityType: "obstacle", LiveItemIDs: []string{"crate"}, Behavior: BehaviorDestructible}
	never := EntityTypeDefaultConfig{
		EntityType: "obstacle", LiveItemIDs: []string{"urn"}, DropItemIDs: []string{"shard"},
		DropChances: map[string]float64{"shard": 0}, Behavior: BehaviorDestructible,
	}
	for _, build := range []EntityTypeDefaultConfig{bare, never} {
		s, ms, player := destructibleFixture(t, build)
		obs := placeObstacle(s, ms, 1, build.LiveItemIDs[0])
		fire(s, ms, player, Vec2{X: 10, Y: 10})
		if !ms.vt(obs).ghost() || len(drops(ms)) != 0 {
			t.Fatalf("%v: depleted with drops %v", build.LiveItemIDs, drops(ms))
		}
	}
}

func TestSnapshotCarriesOnlyTheDestructibleLife(t *testing.T) {
	s, ms, player := destructibleFixture(t, crystalRock)
	player.AOI = Rectangle{MaxX: 20, MaxY: 20}
	wall := placeObstacle(s, ms, 0, "wall")
	crystal := placeObstacle(s, ms, 9, "crystal")

	life := map[EntityID]int{}
	for _, e := range s.buildSnapshot(player, ms).Entities {
		if e.Kind == EntityScenery {
			life[e.ID] = e.MaxLife
		}
	}
	if len(life) != 2 || life[wall] != 0 || life[crystal] != 9 {
		t.Fatalf("obstacle max life on the wire: %v", life)
	}
}
