package game

import (
	"encoding/json"
	"slices"
	"strings"
	"testing"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
	"cyberia-server/game/world/worldtest"
	pb "cyberia-server/gen/proto"
)

func TestStatValidationRejectsCacheMutation(t *testing.T) {
	s := NewGameServer()
	good := map[string]*content.ObjectLayer{"good": {Data: content.ObjectLayerData{Stats: content.Stats{Effect: -100}}}}
	if err := s.ReplaceObjectLayerCache(good); err != nil {
		t.Fatal(err)
	}
	bad := map[string]*content.ObjectLayer{"bad": {Data: content.ObjectLayerData{Stats: content.Stats{Effect: 101}}}}
	if s.ReplaceObjectLayerCache(bad) == nil || s.PatchObjectLayerCache(bad, []string{"good"}) == nil {
		t.Fatal("accepted invalid cache")
	}
	if _, ok := s.GetObjectLayerData("good"); !ok {
		t.Fatal("invalid update changed cache")
	}
	if err := s.ApplyInstanceConfig(&pb.InstanceConfig{ProgressionRules: &pb.ProgressionRules{MaxLevel: -1}}); err == nil {
		t.Fatal("accepted invalid curve")
	}
}

func TestSnapshotCarriesProgressionAndSignedBreakdown(t *testing.T) {
	s := NewGameServer()
	s.EntityBaseSpeed = 5
	p := worldtest.PlacePlayer(s.World, 1, []world.ObjectLayerState{{ItemID: "nerf", Active: true, Quantity: 1}})
	if err := s.ReplaceObjectLayerCache(map[string]*content.ObjectLayer{"nerf": {Data: content.ObjectLayerData{Stats: content.Stats{Agility: -100}}}}); err != nil {
		t.Fatal(err)
	}
	s.SetProgression(p.ID, content.NewEntityProgression(2, false, s.ProgressionConfig()))
	snap := s.buildSnapshotSelf(p, worldtest.Map(s.World))
	if snap.Level != 2 || snap.XP != 100 || snap.LevelXP != 100 || snap.NextLevelXP != 400 || snap.LayerStats[2] != -100 || snap.Stats[2] != -90 {
		t.Fatalf("snapshot: %+v", snap)
	}
	if snap.MoveSpeed <= 0 {
		t.Fatal("movement is not playable")
	}
	encoded, err := json.Marshal(snap)
	if err != nil {
		t.Fatal(err)
	}
	if strings.Contains(string(encoded), "sumStatsLimit") {
		t.Fatal("obsolete limit in snapshot")
	}
}

// The snapshot carries the time left on the shared action cooldown. A tap
// starts it; it reads 0 once the next tap fires skills again.
func TestSnapshotActionReadyInFollowsTheSkillGate(t *testing.T) {
	s := NewGameServer()
	s.EntityBaseSpeed = 5
	s.EntityBaseActionCooldown = 500 * time.Millisecond
	s.EntityBaseMinActionCooldown = 100 * time.Millisecond
	s.Maps = map[string]*world.MapState{"test": world.NewMapState("test", 20, 20)}
	mapState := s.Maps["test"]
	player := worldtest.PlacePlayer(s.World, 1, nil)
	*mapState.Tf(player.ID) = world.Transform{Pos: world.Vec2{X: 1, Y: 1}, Size: world.Vec2{X: 1, Y: 1}}
	mapState.Vt(player.ID).Life, mapState.Vt(player.ID).MaxLife = 100, 100
	mapState.AddMotion(player.ID, world.Motion{Direction: world.NONE, Mode: world.IDLE})

	if ready := s.buildSnapshotSelf(player, mapState).ActionReadyInMs; ready != 0 {
		t.Fatalf("actionReadyInMs = %d before any tap, want 0", ready)
	}
	s.HandlePlayerTapAction(player, mapState, world.Vec2{X: 2, Y: 2})
	if ready := s.buildSnapshotSelf(player, mapState).ActionReadyInMs; ready <= 0 || ready > 500 {
		t.Fatalf("actionReadyInMs = %d after a tap, want 1..500", ready)
	}
	mapState.Mt(player.ID).NextSkillAt = time.Now().Add(-time.Millisecond)
	if ready := s.buildSnapshotSelf(player, mapState).ActionReadyInMs; ready != 0 {
		t.Fatalf("actionReadyInMs = %d after the cooldown, want 0", ready)
	}
}

func TestSnapshotCarriesOnlyTheDestructibleLife(t *testing.T) {
	s := NewGameServer()
	if err := s.ApplyInstanceConfig(&pb.InstanceConfig{EntityBaseMaxLife: 10, EntityDefaults: []*pb.EntityTypeDefault{
		{EntityType: "obstacle", LiveItemIds: []string{"crystal"}, Behavior: content.BehaviorDestructible},
	}}); err != nil {
		t.Fatal(err)
	}
	s.BuildWorldFromInstance(&pb.InstanceMessage{Code: "t", MapCodes: []string{"test"}}, []*pb.MapDataMessage{{
		Code: "test", GridX: 20, GridY: 20, Entities: []*pb.EntityMessage{
			{EntityType: "obstacle", ObjectLayerItemIds: []string{"wall"}, InitCellX: 2, InitCellY: 2, DimX: 2, DimY: 2},
			{EntityType: "obstacle", ObjectLayerItemIds: []string{"crystal"}, InitCellX: 10, InitCellY: 10, DimX: 2, DimY: 2, MaxLife: 9},
		},
	}}, nil, nil, nil)
	player := worldtest.PlacePlayer(s.World, 1000, nil)
	player.AOI = world.Rectangle{MaxX: 20, MaxY: 20}

	var lives []int
	for _, e := range s.buildSnapshot(player, s.Maps["test"]).Entities {
		if e.Kind == world.EntityScenery {
			lives = append(lives, e.MaxLife)
		}
	}
	slices.Sort(lives)
	if !slices.Equal(lives, []int{0, 9}) {
		t.Fatalf("obstacle max life on the wire: %v", lives)
	}
}
