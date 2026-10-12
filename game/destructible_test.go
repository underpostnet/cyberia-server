package game

import (
	"fmt"
	"math"
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

// destructibleFixture is the movement map with a damaging bullet item and the
// obstacle defaults a test resolves against.
func destructibleFixture(t *testing.T, builds ...EntityTypeDefaultConfig) (*GameServer, *MapState, *PlayerState) {
	t.Helper()
	s, ms, player := movementFixture(t)
	s.entityDefaults = map[string]EntityTypeDefaultConfig{}
	s.entityDefaultBuilds = builds
	s.entityBaseMaxLife = 10
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{
		"bullet": {Data: ObjectLayerData{Stats: Stats{Effect: 4}, Item: Item{Type: "skill"}}},
	}); err != nil {
		t.Fatal(err)
	}
	ms.obstacles = map[string]ObjectState{}
	ms.destructibles = map[string]*DestructibleState{}
	ms.resources = map[string]*ResourceState{}
	return s, ms, player
}

// placeObstacle builds one 2×2 obstacle at (10, 10) and returns its id.
func placeObstacle(s *GameServer, ms *MapState, maxLife float64, itemIDs ...string) string {
	before := map[string]bool{}
	for id := range ms.obstacles {
		before[id] = true
	}
	for id := range ms.destructibles {
		before[id] = true
	}
	s.buildObstacle(ms, "test", &pb.EntityMessage{ObjectLayerItemIds: itemIDs, InitCellX: 10, InitCellY: 10, DimX: 2, DimY: 2, MaxLife: maxLife})
	for id := range ms.obstacles {
		if !before[id] {
			return id
		}
	}
	for id := range ms.destructibles {
		if !before[id] {
			return id
		}
	}
	return ""
}

// shot is one bullet of the caster's at the point.
func shot(caster *PlayerState, at Point) *BotState {
	return &BotState{
		EntityBase: EntityBase{ID: "shot", Pos: at, Dims: Dimensions{Width: 1, Height: 1},
			ObjectLayers: []ObjectLayerState{{ItemID: "bullet", Active: true, Quantity: 1}}},
		Mortal:   Mortal{MaxLife: 100, Life: 100},
		Behavior: BehaviorSkill,
		CasterID: caster.ID,
	}
}

// fire lands one bullet of the caster's at the point, then removes it.
func fire(s *GameServer, ms *MapState, caster *PlayerState, at Point) {
	bullet := shot(caster, at)
	ms.bots[bullet.ID] = bullet
	s.handleSkillCollisions(ms)
	delete(ms.bots, bullet.ID)
}

func drops(ms *MapState) []*BotState {
	var out []*BotState
	for _, bot := range ms.bots {
		if bot.Behavior == BehaviorDrop {
			out = append(out, bot)
		}
	}
	return out
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

	if _, ok := ms.obstacles[id]; !ok || len(ms.destructibles) != 0 {
		t.Fatal("an unmatched obstacle became destructible")
	}
	fire(s, ms, player, Point{X: 10, Y: 10})
	if ms.pathfinder.isWalkable(10, 10, Dimensions{Width: 1, Height: 1}) || len(drops(ms)) != 0 {
		t.Fatal("an ordinary obstacle stopped blocking or dropped loot")
	}
}

// Behavior and lifecycle come from the same most specific default.
func TestTheMostSpecificObstacleDefaultDecidesDestructibility(t *testing.T) {
	mossy := EntityTypeDefaultConfig{EntityType: "obstacle", LiveItemIDs: []string{"crystal", "moss"}}
	s, ms, _ := destructibleFixture(t, crystalRock, mossy)

	if _, ok := ms.destructibles[placeObstacle(s, ms, 0, "crystal")]; !ok {
		t.Fatal("the crystal default binds destructible")
	}
	if _, ok := ms.obstacles[placeObstacle(s, ms, 0, "crystal", "moss")]; !ok {
		t.Fatal("the more specific mossy default binds no behavior")
	}
}

// A coloured placement keeps its live item, so it binds the behavior whether or
// not an object layer for the item is cached.
func TestAColouredObstacleCarryingItsLiveItemIsDestructible(t *testing.T) {
	s, ms, _ := destructibleFixture(t, crystalRock)
	s.buildObstacle(ms, "test", &pb.EntityMessage{
		ObjectLayerItemIds: []string{"crystal"}, InitCellX: 4, InitCellY: 4, DimX: 2, DimY: 2, ColorA: 255,
	})
	if len(ms.destructibles) != 1 || len(ms.obstacles) != 0 {
		t.Fatal("a coloured crystal obstacle must bind destructible")
	}
}

func TestDestructibleObstacleDepletesOnceAndRespawnsOverWhoStandsInIt(t *testing.T) {
	s, ms, player := destructibleFixture(t, crystalRock)
	maxLife := 2.5 * s.CalculateStats(shot(player, Point{}), ms).Effect
	obs := ms.destructibles[placeObstacle(s, ms, maxLife, "crystal")]
	if obs == nil || obs.Life != maxLife || obs.MaxLife != maxLife {
		t.Fatalf("placed life: %+v", obs)
	}
	cell := Dimensions{Width: 1, Height: 1}
	if ms.pathfinder.isWalkable(11, 11, cell) {
		t.Fatal("a live destructible obstacle blocks its cells")
	}

	fire(s, ms, player, Point{X: 10, Y: 10})
	if obs.Life >= maxLife || obs.IsGhost() || ms.pathfinder.isWalkable(11, 11, cell) {
		t.Fatalf("a hit damages it and it keeps blocking: %+v", obs.Mortal)
	}
	for hits := 1; !obs.IsGhost(); hits++ {
		if hits > 3 {
			t.Fatal("three hits deplete the obstacle")
		}
		fire(s, ms, player, Point{X: 10, Y: 10})
	}
	if !obs.IsGhost() || obs.Life != 0 || !ms.pathfinder.isWalkable(10, 10, Dimensions{Width: 2, Height: 2}) {
		t.Fatalf("depletion frees the cells: %+v", obs.Mortal)
	}
	if active := activeObjectLayerItemIDs(obs.ObjectLayers); len(active) != 1 || active[0] != "rubble" {
		t.Fatalf("depleted visual: %v", active)
	}
	scattered := drops(ms)
	if len(scattered) != 1 || scattered[0].ObjectLayers[0].ItemID != "shard" || scattered[0].ObjectLayers[0].Quantity != 3 {
		t.Fatalf("drops: %v", scattered)
	}
	if _, ok := scattered[0].LootContributors[player.ID]; !ok {
		t.Fatal("the damage contributor may collect the drop")
	}
	if player.Progression.XP != 0 {
		t.Fatal("an obstacle awards no XP")
	}

	fire(s, ms, player, Point{X: 10, Y: 10})
	if len(drops(ms)) != 1 || obs.Life != 0 {
		t.Fatal("a depleted obstacle takes no hit and drops nothing again")
	}

	obs.RespawnTime = time.Now().Add(-time.Second)
	player.Pos, player.Path, player.Mode = Point{X: 10, Y: 10}, []PointI{{X: 12, Y: 12}}, WALKING
	bot := &BotState{
		EntityBase: EntityBase{ID: "standing-bot", Pos: Point{X: 10.5, Y: 10.5}, Dims: cell},
		Mortal:     Mortal{Life: 100, MaxLife: 100}, Behavior: BehaviorHostile,
		Path: []PointI{{X: 11, Y: 11}}, Mode: WALKING, CurrentTargetPlayer: player.ID,
	}
	ms.bots[bot.ID] = bot
	projectile := shot(player, Point{X: 10, Y: 10})
	ms.bots[projectile.ID] = projectile
	s.handleRespawns(ms)
	if obs.IsGhost() || obs.Life != obs.MaxLife || ms.pathfinder.isWalkable(10, 10, cell) {
		t.Fatalf("the obstacle respawns on its timer and blocks again: %+v", obs.Mortal)
	}
	if checkAABBCollision(obs.Pos, obs.Dims, player.Pos, player.Dims) ||
		!ms.pathfinder.isWalkable(int(player.Pos.X), int(player.Pos.Y), player.Dims) ||
		math.Hypot(player.Pos.X-11, player.Pos.Y-11) > 4 || player.Mode != IDLE || len(player.Path) != 0 {
		t.Fatalf("the player moves to a free spot near the obstacle and stops: %+v", player.Pos)
	}
	for _, bot := range ms.bots {
		if bot != projectile && checkAABBCollision(obs.Pos, obs.Dims, bot.Pos, bot.Dims) {
			t.Fatalf("bot %s still stands in the obstacle", bot.ID)
		}
	}
	if projectile.Pos != (Point{X: 10, Y: 10}) {
		t.Fatal("a projectile keeps its course")
	}
	if bot.Mode != IDLE || len(bot.Path) != 0 || bot.CurrentTargetPlayer != "" ||
		bot.lastPursuitTargetPos != (PointI{-1, -1}) ||
		!ms.pathfinder.isWalkable(int(bot.Pos.X), int(bot.Pos.Y), bot.Dims) {
		t.Fatalf("the displaced bot clears its movement and pursuit: %+v", bot)
	}
	if active := activeObjectLayerItemIDs(obs.ObjectLayers); len(active) != 1 || active[0] != "crystal" {
		t.Fatalf("respawned visual: %v", active)
	}
	s.handleRespawns(ms)
	if ms.pathfinder.grid[10][10] != 1 {
		t.Fatal("a live obstacle blocks its cells once")
	}
}

func TestRestoredObstacleReplansCrossingPaths(t *testing.T) {
	s, ms, player := destructibleFixture(t, crystalRock)
	obs := ms.destructibles[placeObstacle(s, ms, 1, "crystal")]
	fire(s, ms, player, obs.Pos)
	obs.RespawnTime = time.Now().Add(time.Minute)
	tapTick(s, ms, player, tap(7, 15, 15))
	if !pathCrossesObstacle(player.Path, player.Dims, obs.EntityBase) {
		t.Fatal("the initial route crosses the depleted obstacle")
	}
	bot := &BotState{
		EntityBase: EntityBase{ID: "walking-bot", Pos: Point{X: 9, Y: 10}, Dims: player.Dims},
		Mortal:     Mortal{Life: 100, MaxLife: 100}, Behavior: BehaviorHostile,
		Path: []PointI{{X: 10, Y: 10}, {X: 12, Y: 10}}, Mode: WALKING,
		CurrentTargetPlayer: player.ID,
	}
	ms.bots[bot.ID] = bot
	s.handleRespawns(ms)
	if !obs.IsGhost() || !pathCrossesObstacle(player.Path, player.Dims, obs.EntityBase) {
		t.Fatal("the obstacle stays passable until its timer ends")
	}
	obs.RespawnTime = time.Now().Add(-time.Second)
	before := player.Pos
	s.handleRespawns(ms)
	if player.Pos != before || player.Mode != WALKING || player.TargetPos != (PointI{X: 15, Y: 15}) ||
		player.LastMovementSequence != 7 || len(player.Path) == 0 {
		t.Fatalf("the player keeps its destination and movement acknowledgement: %+v", player)
	}
	for _, cell := range player.Path {
		if !ms.pathfinder.isWalkable(cell.X, cell.Y, player.Dims) {
			t.Fatalf("the new path crosses blocked cell %v", cell)
		}
	}
	if bot.Mode != IDLE || len(bot.Path) != 0 || bot.CurrentTargetPlayer != "" {
		t.Fatal("the bot clears its blocked route before its next AI tick")
	}
}

func TestObstacleClearanceUsesEntityDimensionsAtMapEdges(t *testing.T) {
	for _, pos := range []Point{{X: 0, Y: 0}, {X: 18, Y: 18}} {
		t.Run(fmt.Sprintf("%g-%g", pos.X, pos.Y), func(t *testing.T) {
			s, ms, player := movementFixture(t)
			obs := EntityBase{Pos: pos, Dims: Dimensions{Width: 2, Height: 2}}
			ms.pathfinder.occupy(obs.Pos, obs.Dims, 1)
			player.Dims = Dimensions{Width: 3, Height: 2}
			player.Pos = Point{X: math.Min(pos.X, 17), Y: pos.Y}
			player.Path, player.Mode = []PointI{{X: 10, Y: 10}}, WALKING
			s.clearObstacleCells(ms, obs)
			if checkAABBCollision(obs.Pos, obs.Dims, player.Pos, player.Dims) ||
				!ms.pathfinder.isWalkable(int(player.Pos.X), int(player.Pos.Y), player.Dims) ||
				player.Mode != IDLE || len(player.Path) != 0 {
				t.Fatalf("the full player footprint clears the obstacle at %v: %v", pos, player.Pos)
			}
		})
	}
}

func TestObstacleRespawnPushesEntitiesOutward(t *testing.T) {
	for _, tc := range []struct {
		name string
		pos  Point
		want Point
	}{
		{"north", Point{11, 10}, Point{11, 8}},
		{"north-east", Point{12, 10}, Point{14, 8}},
		{"east", Point{12, 11}, Point{14, 11}},
		{"south-east", Point{12, 12}, Point{14, 14}},
		{"south", Point{11, 12}, Point{11, 14}},
		{"south-west", Point{10, 12}, Point{8, 14}},
		{"west", Point{10, 11}, Point{8, 11}},
		{"north-west", Point{10, 10}, Point{8, 8}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, ms, player := movementFixture(t)
			obs := EntityBase{Pos: Point{10, 10}, Dims: Dimensions{4, 4}}
			ms.pathfinder.occupy(obs.Pos, obs.Dims, 1)
			player.Pos, player.Dims, player.Direction = tc.pos, Dimensions{2, 2}, DOWN
			bot := &BotState{EntityBase: EntityBase{ID: "bot", Pos: tc.pos, Dims: player.Dims}, Direction: UP}
			ms.bots[bot.ID] = bot
			s.clearObstacleCells(ms, obs)
			if player.Pos != tc.want || bot.Pos != tc.want {
				t.Fatalf("outward push: player %v, bot %v, want %v", player.Pos, bot.Pos, tc.want)
			}
			if !ms.pathfinder.isWalkable(int(player.Pos.X), int(player.Pos.Y), player.Dims) {
				t.Fatal("the full footprint must land on free cells")
			}
		})
	}
}

func TestObstaclePushAtCenterUsesFacing(t *testing.T) {
	for _, tc := range []struct {
		direction Direction
		want      PointI
	}{
		{RIGHT, PointI{14, 11}},
		{LEFT, PointI{8, 11}},
		{UP, PointI{11, 8}},
		{DOWN, PointI{11, 14}},
		{NONE, PointI{11, 14}},
	} {
		_, ms, player := movementFixture(t)
		obs := EntityBase{Pos: Point{10, 10}, Dims: Dimensions{4, 4}}
		ms.pathfinder.occupy(obs.Pos, obs.Dims, 1)
		player.Pos, player.Dims = Point{11, 11}, Dimensions{2, 2}
		cell, err := obstaclePushCell(ms, obs, player.EntityBase, tc.direction)
		if err != nil || cell != tc.want {
			t.Fatalf("center push for direction %v: %v, %v; want %v", tc.direction, cell, err, tc.want)
		}
	}
}

func TestObstaclePushFindsFreeCellsBesideBlockedExit(t *testing.T) {
	_, ms, player := movementFixture(t)
	obs := EntityBase{Pos: Point{10, 10}, Dims: Dimensions{2, 2}}
	ms.pathfinder.occupy(obs.Pos, obs.Dims, 1)
	ms.pathfinder.occupy(Point{12, 10}, Dimensions{1, 2}, 1)
	player.Pos = Point{11, 10.5}
	cell, err := obstaclePushCell(ms, obs, player.EntityBase, NONE)
	if err != nil || cell.X != 13 || !ms.pathfinder.isWalkable(cell.X, cell.Y, player.Dims) {
		t.Fatalf("a blocked exit keeps the push on the outward side: %v, %v", cell, err)
	}
}

// A resource depleted by the same hits still earns its objective XP.
func TestResourceDepletionKeepsItsObjectiveXP(t *testing.T) {
	s, ms, player := destructibleFixture(t)
	s.buildResource(ms, "test", &pb.EntityMessage{InitCellX: 10, InitCellY: 10, DimX: 2, DimY: 2, MaxLife: 1, ColorA: 255})
	for _, res := range ms.resources {
		for hits := 0; !res.IsGhost(); hits++ {
			if hits == 100 {
				t.Fatal("the resource never depletes")
			}
			fire(s, ms, player, Point{X: 10, Y: 10})
		}
	}
	if player.Progression.XP == 0 {
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
		obs := ms.destructibles[placeObstacle(s, ms, 1, build.LiveItemIDs[0])]
		fire(s, ms, player, Point{X: 10, Y: 10})
		if !obs.IsGhost() || len(drops(ms)) != 0 {
			t.Fatalf("%v: depleted with drops %v", build.LiveItemIDs, drops(ms))
		}
	}
}

func TestSnapshotCarriesOnlyTheDestructibleLife(t *testing.T) {
	s, ms, player := destructibleFixture(t, crystalRock)
	player.AOI = Rectangle{MaxX: 20, MaxY: 20}
	wall := placeObstacle(s, ms, 0, "wall")
	crystal := placeObstacle(s, ms, 9, "crystal")

	life := map[string]float64{}
	for _, e := range s.buildSnapshot(player, ms).Entities {
		if e.Type == EntityObstacle {
			life[e.ID] = e.MaxLife
		}
	}
	if len(life) != 2 || life[wall] != 0 || life[crystal] != 9 {
		t.Fatalf("obstacle max life on the wire: %v", life)
	}
}
