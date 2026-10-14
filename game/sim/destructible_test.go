package sim

import (
	"cyberia-server/game/content"
	"cyberia-server/game/world"
	"fmt"
	"iter"
	"math"
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

// destructibleFixture is the movement map with a damaging bullet item and the
// obstacle defaults a test resolves against.
func destructibleFixture(t *testing.T, builds ...content.EntityTypeDefaultConfig) (*Sim, *world.MapState, *world.Session) {
	t.Helper()
	s, ms, player := movementFixture(t)
	s.EntityDefaults = map[string]content.EntityTypeDefaultConfig{}
	s.EntityDefaultBuilds = builds
	s.EntityBaseMaxLife = 10
	s.NewEntityID() // built entities take ids after the player
	if err := s.ReplaceObjectLayerCache(map[string]*content.ObjectLayer{
		"bullet": {Data: content.ObjectLayerData{Stats: content.Stats{Effect: 4}, Item: content.Item{Type: "skill"}}},
	}); err != nil {
		t.Fatal(err)
	}
	return s, ms, player
}

// placeObstacle builds one 2×2 obstacle at (10, 10) and returns its id.
func placeObstacle(s *Sim, ms *world.MapState, maxLife float64, itemIDs ...string) world.EntityID {
	before := map[world.EntityID]bool{}
	for id := range ms.OfKind(world.EntityScenery) {
		before[id] = true
	}
	s.buildObstacle(ms, &pb.EntityMessage{ObjectLayerItemIds: itemIDs, InitCellX: 10, InitCellY: 10, DimX: 2, DimY: 2, MaxLife: maxLife})
	for id := range ms.OfKind(world.EntityScenery) {
		if !before[id] {
			return id
		}
	}
	return 0
}

// shot is one bullet of the caster's at the point, with a slot in the map.
func shot(ms *world.MapState, caster *world.Session, at world.Vec2) world.EntityID {
	const bullet world.EntityID = 900
	ms.AddBrain(bullet, &world.Brain{})
	ms.Spawn(bullet, world.EntityData{Kind: world.EntityBot}, world.Transform{Pos: at, Size: world.Vec2{X: 1, Y: 1}},
		world.Appearance{Layers: []world.ObjectLayerState{{ItemID: "bullet", Active: true, Quantity: 1}}})
	ms.AddVitals(bullet, world.Vitals{MaxLife: 100, Life: 100})
	ms.AddMotion(bullet, world.Motion{})
	ms.AddAgent(bullet, world.Agent{Behavior: content.BehaviorSkill, CasterID: caster.ID, OwnerID: caster.ID})
	ms.AddMortal(bullet, &world.Mortal{})
	return bullet
}

// fire lands one bullet of the caster's at the point, then removes it.
func fire(s *Sim, ms *world.MapState, caster *world.Session, at world.Vec2) {
	bullet := shot(ms, caster, at)
	s.handleSkillCollisions(ms)
	s.Despawn(ms, bullet)
}

// addBot gives bot id a slot at tf with the brain, walking in the motion.
func addBot(ms *world.MapState, id world.EntityID, tf world.Transform, brain *world.Brain, motion world.Motion, behavior string) *world.Brain {
	ms.AddBrain(id, brain)
	ms.Spawn(id, world.EntityData{Kind: world.EntityBot}, tf, world.Appearance{})
	ms.AddVitals(id, world.Vitals{Life: 100, MaxLife: 100})
	ms.AddMotion(id, motion)
	ms.AddAgent(id, world.Agent{Behavior: behavior})
	ms.AddMortal(id, &world.Mortal{})
	return brain
}

func drops(ms *world.MapState) []world.EntityID {
	var out []world.EntityID
	for id := range ms.OfKind(world.EntityBot) {
		if ms.Ag(id).Behavior == content.BehaviorDrop {
			out = append(out, id)
		}
	}
	return out
}

func countOf(seq iter.Seq[world.EntityID]) int {
	n := 0
	for range seq {
		n++
	}
	return n
}

var crystalRock = content.EntityTypeDefaultConfig{
	EntityType:          "obstacle",
	LiveItemIDs:         []string{"crystal"},
	DeadItemIDs:         []string{"rubble"},
	DropItemIDs:         []string{"shard"},
	DefaultObjectLayers: []world.ObjectLayerState{{ItemID: "shard", Quantity: 3}},
	Behavior:            content.BehaviorDestructible,
}

// An obstacle no default matches stays ordinary, even when the first obstacle
// default of the world is destructible.
func TestOrdinaryObstacleBlocksAndTakesNoDamage(t *testing.T) {
	s, ms, player := destructibleFixture(t, crystalRock)
	id := placeObstacle(s, ms, 0, "wall")

	if ms.Row(id).Data.Flags&world.FlagBlocks == 0 || countOf(ms.Destructibles()) != 0 {
		t.Fatal("an unmatched obstacle became destructible")
	}
	fire(s, ms, player, world.Vec2{X: 10, Y: 10})
	if ms.Pathfinder().IsWalkable(10, 10, world.Vec2{X: 1, Y: 1}) || len(drops(ms)) != 0 {
		t.Fatal("an ordinary obstacle stopped blocking or dropped loot")
	}
}

// Behavior and lifecycle come from the same most specific default.
func TestTheMostSpecificObstacleDefaultDecidesDestructibility(t *testing.T) {
	mossy := content.EntityTypeDefaultConfig{EntityType: "obstacle", LiveItemIDs: []string{"crystal", "moss"}}
	s, ms, _ := destructibleFixture(t, crystalRock, mossy)

	if !ms.Has(placeObstacle(s, ms, 0, "crystal"), world.MaskVitals) {
		t.Fatal("the crystal default binds destructible")
	}
	if ms.Has(placeObstacle(s, ms, 0, "crystal", "moss"), world.MaskVitals) {
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
	if countOf(ms.Destructibles()) != 1 {
		t.Fatal("a coloured crystal obstacle must bind destructible")
	}
}

func TestDestructibleObstacleDepletesOnceAndRespawnsOverWhoStandsInIt(t *testing.T) {
	s, ms, player := destructibleFixture(t, crystalRock)
	probe := shot(ms, player, world.Vec2{})
	maxLife := 2.5 * s.CalculateStats(probe, ms).Effect
	ms.Despawn(probe)
	obs := placeObstacle(s, ms, maxLife, "crystal")
	if !ms.Has(obs, world.MaskVitals) || ms.Vt(obs).Life != maxLife || ms.Vt(obs).MaxLife != maxLife {
		t.Fatalf("placed life: %+v", obs)
	}
	cell := world.Vec2{X: 1, Y: 1}
	if ms.Pathfinder().IsWalkable(11, 11, cell) {
		t.Fatal("a live destructible obstacle blocks its cells")
	}

	fire(s, ms, player, world.Vec2{X: 10, Y: 10})
	if ms.Vt(obs).Life >= maxLife || ms.Vt(obs).Ghost() || ms.Pathfinder().IsWalkable(11, 11, cell) {
		t.Fatalf("a hit damages it and it keeps blocking: %+v", ms.Vt(obs))
	}
	for hits := 1; !ms.Vt(obs).Ghost(); hits++ {
		if hits > 3 {
			t.Fatal("three hits deplete the obstacle")
		}
		fire(s, ms, player, world.Vec2{X: 10, Y: 10})
	}
	if !ms.Vt(obs).Ghost() || ms.Vt(obs).Life != 0 || !ms.Pathfinder().IsWalkable(10, 10, world.Vec2{X: 2, Y: 2}) {
		t.Fatalf("depletion frees the cells: %+v", ms.Vt(obs))
	}
	if active := content.ActiveObjectLayerItemIDs(ms.Ap(obs).Layers); len(active) != 1 || active[0] != "rubble" {
		t.Fatalf("depleted visual: %v", active)
	}
	scattered := drops(ms)
	if len(scattered) != 1 || ms.Ap(scattered[0]).Layers[0].ItemID != "shard" || ms.Ap(scattered[0]).Layers[0].Quantity != 3 {
		t.Fatalf("drops: %v", scattered)
	}
	if _, ok := ms.Br(scattered[0]).LootContributors[player.ID]; !ok {
		t.Fatal("the damage contributor may collect the drop")
	}
	if s.EntityProgression(player.ID, ms).XP != 0 {
		t.Fatal("an obstacle awards no XP")
	}

	fire(s, ms, player, world.Vec2{X: 10, Y: 10})
	if len(drops(ms)) != 1 || ms.Vt(obs).Life != 0 {
		t.Fatal("a depleted obstacle takes no hit and drops nothing again")
	}

	ms.Vt(obs).RespawnAt = time.Now().Add(-time.Second)
	ptf := ms.Tf(player.ID)
	ptf.Pos, player.Path, ms.Mo(player.ID).Mode = world.Vec2{X: 10, Y: 10}, []world.Cell{{X: 12, Y: 12}}, world.WALKING
	const botID world.EntityID = 901
	bot := addBot(ms, botID, world.Transform{Pos: world.Vec2{X: 10.5, Y: 10.5}, Size: cell},
		&world.Brain{Path: []world.Cell{{X: 11, Y: 11}}, CurrentTargetPlayer: player.ID}, world.Motion{Mode: world.WALKING}, content.BehaviorHostile)
	projectile := shot(ms, player, world.Vec2{X: 10, Y: 10})
	ptf = ms.Tf(player.ID) // a spawn may move the transform array
	s.handleRespawns(ms)
	if ms.Vt(obs).Ghost() || ms.Vt(obs).Life != ms.Vt(obs).MaxLife || ms.Pathfinder().IsWalkable(10, 10, cell) {
		t.Fatalf("the obstacle respawns on its timer and blocks again: %+v", ms.Vt(obs))
	}
	otf := ms.Tf(obs)
	if world.CheckAABBCollision(otf.Pos, otf.Size, ptf.Pos, ptf.Size) ||
		!ms.Pathfinder().IsWalkable(int(ptf.Pos.X), int(ptf.Pos.Y), ptf.Size) ||
		math.Hypot(ptf.Pos.X-11, ptf.Pos.Y-11) > 4 || ms.Mo(player.ID).Mode != world.IDLE || len(player.Path) != 0 {
		t.Fatalf("the player moves to a free spot near the obstacle and stops: %+v", ptf.Pos)
	}
	for other := range ms.OfKind(world.EntityBot) {
		btf := ms.Tf(other)
		if other != projectile && world.CheckAABBCollision(otf.Pos, otf.Size, btf.Pos, btf.Size) {
			t.Fatalf("bot %s still stands in the obstacle", other)
		}
	}
	if ms.Tf(projectile).Pos != (world.Vec2{X: 10, Y: 10}) {
		t.Fatal("a projectile keeps its course")
	}
	btf := ms.Tf(botID)
	if ms.Mo(botID).Mode != world.IDLE || len(bot.Path) != 0 || bot.CurrentTargetPlayer != 0 ||
		bot.LastPursuitTargetPos != (world.Cell{X: -1, Y: -1}) ||
		!ms.Pathfinder().IsWalkable(int(btf.Pos.X), int(btf.Pos.Y), btf.Size) {
		t.Fatalf("the displaced bot clears its movement and pursuit: %+v", bot)
	}
	if active := content.ActiveObjectLayerItemIDs(ms.Ap(obs).Layers); len(active) != 1 || active[0] != "crystal" {
		t.Fatalf("respawned visual: %v", active)
	}
	s.handleRespawns(ms)
	ms.Pathfinder().Occupy(otf.Pos, otf.Size, -1)
	if !ms.Pathfinder().IsWalkable(10, 10, cell) {
		t.Fatal("a live obstacle blocks its cells once")
	}
}

func TestRestoredObstacleReplansCrossingPaths(t *testing.T) {
	s, ms, player := destructibleFixture(t, crystalRock)
	obs := placeObstacle(s, ms, 1, "crystal")
	otf := *ms.Tf(obs)
	ptf := ms.Tf(player.ID)
	fire(s, ms, player, otf.Pos)
	ms.Vt(obs).RespawnAt = time.Now().Add(time.Minute)
	tapTick(s, ms, player, tap(7, 15, 15))
	if !pathCrossesObstacle(player.Path, ptf.Size, otf) {
		t.Fatal("the initial route crosses the depleted obstacle")
	}
	const botID world.EntityID = 902
	bot := addBot(ms, botID, world.Transform{Pos: world.Vec2{X: 9, Y: 10}, Size: ptf.Size},
		&world.Brain{Path: []world.Cell{{X: 10, Y: 10}, {X: 12, Y: 10}}, CurrentTargetPlayer: player.ID}, world.Motion{Mode: world.WALKING}, content.BehaviorHostile)
	ptf = ms.Tf(player.ID) // a spawn may move the transform array
	s.handleRespawns(ms)
	if !ms.Vt(obs).Ghost() || !pathCrossesObstacle(player.Path, ptf.Size, otf) {
		t.Fatal("the obstacle stays passable until its timer ends")
	}
	ms.Vt(obs).RespawnAt = time.Now().Add(-time.Second)
	before := ptf.Pos
	s.handleRespawns(ms)
	if ptf.Pos != before || ms.Mo(player.ID).Mode != world.WALKING || player.TargetPos != (world.Cell{X: 15, Y: 15}) ||
		len(player.Path) == 0 {
		t.Fatalf("the player keeps its destination: %+v", player)
	}
	for _, cell := range player.Path {
		if !ms.Pathfinder().IsWalkable(cell.X, cell.Y, ptf.Size) {
			t.Fatalf("the new path crosses blocked cell %v", cell)
		}
	}
	if ms.Mo(botID).Mode != world.IDLE || len(bot.Path) != 0 || bot.CurrentTargetPlayer != 0 {
		t.Fatal("the bot clears its blocked route before its next AI tick")
	}
}

func TestObstacleClearanceUsesEntityDimensionsAtMapEdges(t *testing.T) {
	for _, pos := range []world.Vec2{{X: 0, Y: 0}, {X: 18, Y: 18}} {
		t.Run(fmt.Sprintf("%g-%g", pos.X, pos.Y), func(t *testing.T) {
			s, ms, player := movementFixture(t)
			obs := world.Transform{Pos: pos, Size: world.Vec2{X: 2, Y: 2}}
			ms.Pathfinder().Occupy(obs.Pos, obs.Size, 1)
			ptf := ms.Tf(player.ID)
			ptf.Size = world.Vec2{X: 3, Y: 2}
			ptf.Pos = world.Vec2{X: math.Min(pos.X, 17), Y: pos.Y}
			player.Path, ms.Mo(player.ID).Mode = []world.Cell{{X: 10, Y: 10}}, world.WALKING
			s.clearObstacleCells(ms, obs)
			if world.CheckAABBCollision(obs.Pos, obs.Size, ptf.Pos, ptf.Size) ||
				!ms.Pathfinder().IsWalkable(int(ptf.Pos.X), int(ptf.Pos.Y), ptf.Size) ||
				ms.Mo(player.ID).Mode != world.IDLE || len(player.Path) != 0 {
				t.Fatalf("the full player footprint clears the obstacle at %v: %v", pos, ptf.Pos)
			}
		})
	}
}

func TestObstacleRespawnPushesEntitiesOutward(t *testing.T) {
	for _, tc := range []struct {
		name string
		pos  world.Vec2
		want world.Vec2
	}{
		{"north", world.Vec2{X: 11, Y: 10}, world.Vec2{X: 11, Y: 8}},
		{"north-east", world.Vec2{X: 12, Y: 10}, world.Vec2{X: 14, Y: 8}},
		{"east", world.Vec2{X: 12, Y: 11}, world.Vec2{X: 14, Y: 11}},
		{"south-east", world.Vec2{X: 12, Y: 12}, world.Vec2{X: 14, Y: 14}},
		{"south", world.Vec2{X: 11, Y: 12}, world.Vec2{X: 11, Y: 14}},
		{"south-west", world.Vec2{X: 10, Y: 12}, world.Vec2{X: 8, Y: 14}},
		{"west", world.Vec2{X: 10, Y: 11}, world.Vec2{X: 8, Y: 11}},
		{"north-west", world.Vec2{X: 10, Y: 10}, world.Vec2{X: 8, Y: 8}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, ms, player := movementFixture(t)
			obs := world.Transform{Pos: world.Vec2{X: 10, Y: 10}, Size: world.Vec2{X: 4, Y: 4}}
			ms.Pathfinder().Occupy(obs.Pos, obs.Size, 1)
			ptf := ms.Tf(player.ID)
			ptf.Pos, ptf.Size, ms.Mo(player.ID).Direction = tc.pos, world.Vec2{X: 2, Y: 2}, world.DOWN
			const botID world.EntityID = 903
			addBot(ms, botID, world.Transform{Pos: tc.pos, Size: ptf.Size}, &world.Brain{}, world.Motion{Direction: world.UP}, "")
			ptf = ms.Tf(player.ID) // a spawn may move the transform array
			s.clearObstacleCells(ms, obs)
			if ptf.Pos != tc.want || ms.Tf(botID).Pos != tc.want {
				t.Fatalf("outward push: player %v, bot %v, want %v", ptf.Pos, ms.Tf(botID).Pos, tc.want)
			}
			if !ms.Pathfinder().IsWalkable(int(ptf.Pos.X), int(ptf.Pos.Y), ptf.Size) {
				t.Fatal("the full footprint must land on free cells")
			}
		})
	}
}

func TestObstaclePushAtCenterUsesFacing(t *testing.T) {
	for _, tc := range []struct {
		direction world.Direction
		want      world.Cell
	}{
		{world.RIGHT, world.Cell{X: 14, Y: 11}},
		{world.LEFT, world.Cell{X: 8, Y: 11}},
		{world.UP, world.Cell{X: 11, Y: 8}},
		{world.DOWN, world.Cell{X: 11, Y: 14}},
		{world.NONE, world.Cell{X: 11, Y: 14}},
	} {
		_, ms, _ := movementFixture(t)
		obs := world.Transform{Pos: world.Vec2{X: 10, Y: 10}, Size: world.Vec2{X: 4, Y: 4}}
		ms.Pathfinder().Occupy(obs.Pos, obs.Size, 1)
		entity := world.Transform{Pos: world.Vec2{X: 11, Y: 11}, Size: world.Vec2{X: 2, Y: 2}}
		cell, err := obstaclePushCell(ms, obs, entity, tc.direction)
		if err != nil || cell != tc.want {
			t.Fatalf("center push for direction %v: %v, %v; want %v", tc.direction, cell, err, tc.want)
		}
	}
}

func TestObstaclePushFindsFreeCellsBesideBlockedExit(t *testing.T) {
	_, ms, player := movementFixture(t)
	obs := world.Transform{Pos: world.Vec2{X: 10, Y: 10}, Size: world.Vec2{X: 2, Y: 2}}
	ms.Pathfinder().Occupy(obs.Pos, obs.Size, 1)
	ms.Pathfinder().Occupy(world.Vec2{X: 12, Y: 10}, world.Vec2{X: 1, Y: 2}, 1)
	ptf := ms.Tf(player.ID)
	ptf.Pos = world.Vec2{X: 11, Y: 10.5}
	cell, err := obstaclePushCell(ms, obs, *ptf, world.NONE)
	if err != nil || cell.X != 13 || !ms.Pathfinder().IsWalkable(cell.X, cell.Y, ptf.Size) {
		t.Fatalf("a blocked exit keeps the push on the outward side: %v, %v", cell, err)
	}
}

// A resource depleted by the same hits still earns its objective XP.
func TestResourceDepletionKeepsItsObjectiveXP(t *testing.T) {
	s, ms, player := destructibleFixture(t)
	s.BuildResource(ms, &pb.EntityMessage{InitCellX: 10, InitCellY: 10, DimX: 2, DimY: 2, MaxLife: 1, ColorA: 255})
	for res := range ms.OfKind(world.EntityResource) {
		for hits := 0; !ms.Vt(res).Ghost(); hits++ {
			if hits == 100 {
				t.Fatal("the resource never depletes")
			}
			fire(s, ms, player, world.Vec2{X: 10, Y: 10})
		}
	}
	if s.EntityProgression(player.ID, ms).XP == 0 {
		t.Fatal("a depleted resource awards objective XP")
	}
}

func TestDestructibleObstacleDropsOnlyWhatItsDefaultRolls(t *testing.T) {
	bare := content.EntityTypeDefaultConfig{EntityType: "obstacle", LiveItemIDs: []string{"crate"}, Behavior: content.BehaviorDestructible}
	never := content.EntityTypeDefaultConfig{
		EntityType: "obstacle", LiveItemIDs: []string{"urn"}, DropItemIDs: []string{"shard"},
		DropChances: map[string]float64{"shard": 0}, Behavior: content.BehaviorDestructible,
	}
	for _, build := range []content.EntityTypeDefaultConfig{bare, never} {
		s, ms, player := destructibleFixture(t, build)
		obs := placeObstacle(s, ms, 1, build.LiveItemIDs[0])
		fire(s, ms, player, world.Vec2{X: 10, Y: 10})
		if !ms.Vt(obs).Ghost() || len(drops(ms)) != 0 {
			t.Fatalf("%v: depleted with drops %v", build.LiveItemIDs, drops(ms))
		}
	}
}
