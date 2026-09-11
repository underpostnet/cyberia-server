package game

import (
	"encoding/json"
	"math"
	"strings"
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

func TestSignedStatContract(t *testing.T) {
	for _, value := range []int{-100, 0, 100} {
		if err := (Stats{Effect: value, Resistance: value, Agility: value, Range: value, Intelligence: value, Utility: value}).Validate(); err != nil {
			t.Fatal(err)
		}
	}
	for _, value := range []int{-101, 101} {
		for i := 0; i < StatCount; i++ {
			var block [StatCount]float64
			block[i] = float64(value)
			data, _ := json.Marshal(statsFromValues(block))
			var stats Stats
			if err := json.Unmarshal(data, &stats); err != nil {
				t.Fatal(err)
			}
			if stats.Validate() == nil {
				t.Fatalf("accepted stat %d = %d", i, value)
			}
		}
	}
}

func TestSignedAggregationAcrossEntities(t *testing.T) {
	s := &GameServer{}
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{
		"nerf": {Data: ObjectLayerData{Stats: Stats{Effect: -100, Resistance: -100, Agility: -100}}},
		"buff": {Data: ObjectLayerData{Stats: Stats{Effect: 100, Resistance: 100, Agility: 100}}},
	}); err != nil {
		t.Fatal(err)
	}
	layers := []ObjectLayerState{{ItemID: "nerf", Active: true, Quantity: 20}, {ItemID: "buff", Active: true, Quantity: 1}}
	entities := []statSource{
		&PlayerState{EntityBase: EntityBase{ID: "p", ObjectLayers: layers}},
		&BotState{EntityBase: EntityBase{ID: "b", ObjectLayers: layers}, Behavior: "provider"},
		&ResourceState{EntityBase: EntityBase{ID: "r", ObjectLayers: layers}},
	}
	for _, entity := range entities {
		entity.StatState().Progression = NewEntityProgression(4, true, s.progressionConfig())
		expected := s.progressionConfig().BaseAtLevel(4)
		if got := s.CalculateStats(entity, nil); got != expected {
			t.Fatalf("%T: %v != %v", entity, got, expected)
		}
	}
	layers[1].Active = false
	for _, entity := range entities {
		got := s.CalculateStats(entity, nil)
		if got.Effect != 1 || got.Resistance != 0 || got.Agility != -90 {
			t.Fatalf("floors: %v", got)
		}
	}
}

func TestTemporaryStatsAndExpiry(t *testing.T) {
	s := &GameServer{entityBaseMaxLife: 100}
	p := &PlayerState{EntityBase: EntityBase{ID: "p"}}
	now := time.Now()
	modifier := TemporaryModifier{ID: "buff", Stats: ComputedStats{Effect: 10, Resistance: 20}, ExpiresAt: now.Add(time.Second)}
	if err := s.SetTemporaryModifier(p, modifier, now); err != nil {
		t.Fatal(err)
	}
	if err := s.SetTemporaryModifier(p, modifier, now); err != nil {
		t.Fatal(err)
	}
	s.refreshEntityStats(p, nil, now)
	if p.MaxLife != 130 || len(p.TemporaryModifiers) != 1 {
		t.Fatalf("temporary stats: %+v", p.Mortal)
	}
	if got := s.statBreakdown(p, nil, now.Add(time.Second), map[string]bool{}).Effective.Effect; got != 5 {
		t.Fatalf("expired effect = %v", got)
	}
	s.refreshEntityStats(p, nil, now.Add(2*time.Second))
	if p.MaxLife != 110 || len(p.TemporaryModifiers) != 0 {
		t.Fatal("expiry did not refresh life")
	}
	modifier.Stats.Effect = math.NaN()
	if s.SetTemporaryModifier(p, modifier, now) == nil {
		t.Fatal("accepted NaN")
	}
}

func TestProgressionThresholdsAndFrozenEntities(t *testing.T) {
	rules := defaultProgressionRules()
	if rules.Threshold(1) != 0 || rules.Threshold(2) != 100 || rules.Threshold(3) != 400 {
		t.Fatal("wrong thresholds")
	}
	player := NewEntityProgression(1, false, rules)
	now := time.Unix(1000, 0)
	if amount := player.award(rules, xpQuest, "quest", 1, now); amount != 100 || player.Level != 2 || player.XP != 100 {
		t.Fatalf("level up: %+v", player)
	}
	if player.award(rules, xpQuest, "quest", 1, now.Add(time.Minute)) != 0 {
		t.Fatal("quest replay awards XP")
	}
	rules.QuestXp = 10000
	if player.award(rules, xpQuest, "large-quest", 1, now.Add(2*time.Minute)) != 10000 || player.Level != 11 {
		t.Fatal("multiple level transition failed")
	}
	for _, kind := range []string{"bot", "npc", "resource"} {
		entity := NewEntityProgression(7, true, rules)
		before := entity.XP
		if entity.award(rules, xpKill, kind, 7, now) != 0 || entity.Level != 7 || entity.XP != before {
			t.Fatal("frozen progression changed")
		}
	}
	player = NewEntityProgression(rules.MaxLevel, false, rules)
	if player.award(rules, xpQuest, "cap", 1, now) != 0 {
		t.Fatal("maximum level changed")
	}
}

func TestProgressionRateAndRepeatLimits(t *testing.T) {
	rules := defaultProgressionRules()
	p := NewEntityProgression(1, false, rules)
	now := time.Unix(1000, 0)
	if p.award(rules, 0, "tap", 1, now) != 0 {
		t.Fatal("input event awards XP")
	}
	first := p.award(rules, xpKill, "target", 1, now)
	if first != 25 {
		t.Fatalf("first reward %d", first)
	}
	if p.award(rules, xpKill, "other", 1, now) != 0 {
		t.Fatal("cooldown bypass")
	}
	second := p.award(rules, xpKill, "target", 1, now.Add(time.Second))
	if second != 12 {
		t.Fatalf("repeated reward %d", second)
	}
	for i := 2; i < rules.MaxRepeatAwards; i++ {
		p.award(rules, xpKill, "target", 1, now.Add(time.Duration(i)*time.Second))
	}
	if p.award(rules, xpKill, "target", 1, now.Add(10*time.Second)) != 0 {
		t.Fatal("repeat limit bypass")
	}
	p = NewEntityProgression(1, false, rules)
	rules.MaxAwardsPerWindow = 2
	for i, key := range []string{"a", "b", "c"} {
		got := p.award(rules, xpKill, key, 1, now.Add(time.Duration(i)*time.Second))
		if (i < 2) != (got > 0) {
			t.Fatalf("window reward %d: %d", i, got)
		}
	}
	if p.award(rules, xpKill, "d", 1, now.Add(time.Minute)) == 0 {
		t.Fatal("window did not reset")
	}
}

func TestRawTapSpamDoesNotAwardXP(t *testing.T) {
	s, ms, p := movementFixture(t)
	p.Progression = NewEntityProgression(1, false, s.progressionConfig())
	for i := uint32(1); i <= 1000; i++ {
		s.HandlePlayerTapAction(p, ms, Point{X: 2, Y: 2})
	}
	if p.Progression.XP != 0 || p.Progression.Level != 1 {
		t.Fatal("raw taps award XP")
	}
	if p.NextSkillAt.IsZero() {
		t.Fatal("skills have no cooldown")
	}
	if s.acceptSkillAction(p, ms, p.NextSkillAt.Add(-time.Nanosecond)) {
		t.Fatal("early skill accepted")
	}
	if !s.acceptSkillAction(p, ms, p.NextSkillAt) {
		t.Fatal("ready skill rejected")
	}
}

func TestDefeatAwardsOnlyEligibleContributors(t *testing.T) {
	s, ms, p := movementFixture(t)
	victim := &BotState{EntityBase: EntityBase{ID: "victim"}, Behavior: "hostile"}
	victim.Progression = NewEntityProgression(1, true, s.progressionConfig())
	ms.bots[victim.ID] = victim
	s.awardDefeat(&victim.EntityBase, &victim.Mortal, "bot", map[string]float64{p.ID: 100}, ms)
	if p.Progression.XP != 25 {
		t.Fatalf("kill XP = %d", p.Progression.XP)
	}
	p.Progression = NewEntityProgression(1, false, s.progressionConfig())
	victim.CasterID = p.ID
	s.awardDefeat(&victim.EntityBase, &victim.Mortal, "bot", map[string]float64{p.ID: 100}, ms)
	if p.Progression.XP != 0 {
		t.Fatal("summon farming awards XP")
	}
	victim.CasterID = ""
	s.awardDefeat(&victim.EntityBase, &victim.Mortal, "bot", map[string]float64{p.ID: 1, "other": 100}, ms)
	if p.Progression.XP != 0 {
		t.Fatal("low contribution awards XP")
	}
}

func TestStatValidationRejectsCacheMutation(t *testing.T) {
	s := &GameServer{}
	good := map[string]*ObjectLayer{"good": {Data: ObjectLayerData{Stats: Stats{Effect: -100}}}}
	if err := s.ReplaceObjectLayerCache(good); err != nil {
		t.Fatal(err)
	}
	bad := map[string]*ObjectLayer{"bad": {Data: ObjectLayerData{Stats: Stats{Effect: 101}}}}
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
	s, ms, p := movementFixture(t)
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{"nerf": {Data: ObjectLayerData{Stats: Stats{Agility: -100}}}}); err != nil {
		t.Fatal(err)
	}
	p.ObjectLayers = []ObjectLayerState{{ItemID: "nerf", Active: true, Quantity: 1}}
	p.Progression = NewEntityProgression(2, false, s.progressionConfig())
	snap := s.buildSnapshotSelf(p, ms)
	if snap.Level != 2 || snap.XP != 100 || snap.LevelXP != 100 || snap.NextLevelXP != 400 || snap.LayerStats[2] != -100 || snap.EffectiveStats[2] != -90 {
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

func TestSummonFloorsAfterAllContributions(t *testing.T) {
	s, ms, p := movementFixture(t)
	p.Progression = NewEntityProgression(1, false, s.progressionConfig())
	s.ReplaceObjectLayerCache(map[string]*ObjectLayer{
		"nerf": {Data: ObjectLayerData{Stats: Stats{Effect: -100}}},
		"buff": {Data: ObjectLayerData{Stats: Stats{Effect: 100}}},
	})
	p.ObjectLayers = []ObjectLayerState{{ItemID: "nerf", Active: true, Quantity: 1}}
	summon := &BotState{EntityBase: EntityBase{ID: "summon", ObjectLayers: []ObjectLayerState{{ItemID: "buff", Active: true, Quantity: 1}}}, CasterID: p.ID}
	if got := s.CalculateStats(summon, ms).Effect; got != 5 {
		t.Fatalf("floor applied before full sum: %v", got)
	}
	summon.CasterID = summon.ID
	ms.bots[summon.ID] = summon
	if got := s.CalculateStats(summon, ms).Effect; math.IsNaN(got) {
		t.Fatal("caster cycle produces invalid stats")
	}
}

func TestProgressionNormalizesAfterCurveChange(t *testing.T) {
	s := &GameServer{progressionRules: defaultProgressionRules()}
	p := &PlayerState{EntityBase: EntityBase{ID: "player"}}
	p.Progression = NewEntityProgression(3, false, s.progressionConfig())
	s.progressionRules.XpPerLevel = 200
	if got := s.entityProgression(p); got.Level != 2 || got.XP != 400 {
		t.Fatalf("curve normalization: %+v", got)
	}
	s.progressionRules.MaxLevel = 1
	if got := s.entityProgression(p); got.Level != 1 || got.XP != 0 {
		t.Fatalf("level cap normalization: %+v", got)
	}
}

func TestQuestCompletionAwardsAfterKillAndRejectsReplay(t *testing.T) {
	s, _, p := movementFixture(t)
	s.awardProgression(p, xpKill, "victim", 1, time.Now())
	progress := &QuestProgress{QuestCode: "quest", Status: "active"}
	affected := []QuestSnapshotEntry{}
	s.completeQuest(p, progress, &affected)
	if p.Progression.XP != 125 || p.Progression.Level != 2 {
		t.Fatalf("quest result: %+v", p.Progression)
	}
	s.completeQuest(p, progress, &affected)
	if p.Progression.XP != 125 || len(affected) != 1 {
		t.Fatal("quest completion replay")
	}
}

func TestAuthoredLevelsUseCommonBaseCurve(t *testing.T) {
	s, ms, _ := movementFixture(t)
	ms.resources = make(map[string]*ResourceState)
	s.buildResource(ms, "test", &pb.EntityMessage{Level: 7, MaxLife: 50, ColorA: 255})
	for _, resource := range ms.resources {
		if !resource.Progression.Frozen || resource.Progression.Level != 7 || resource.MaxLife != 90 {
			t.Fatalf("resource curve: %+v", resource.Mortal)
		}
	}
	rules := s.progressionConfig()
	for level := 1; level <= rules.MaxLevel; level++ {
		if got := rules.LevelAtXP(rules.Threshold(level)); got != level {
			t.Fatalf("level %d -> %d", level, got)
		}
		if level > 1 && rules.LevelAtXP(rules.Threshold(level)-1) != level-1 {
			t.Fatal("early level transition")
		}
	}
	if err := ValidateWorldProgression(&pb.InstanceConfig{}, []*pb.MapDataMessage{{Entities: []*pb.EntityMessage{{Level: 101}}}}); err == nil {
		t.Fatal("invalid authored level")
	}
}
