package game

import (
	"encoding/json"
	"math"
	"strings"
	"testing"
	"time"

	"cyberia-server/game/world"
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
	s := &Content{}
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{
		"nerf": {Data: ObjectLayerData{Stats: Stats{Effect: -100, Resistance: -100, Agility: -100}}},
		"buff": {Data: ObjectLayerData{Stats: Stats{Effect: 100, Resistance: 100, Agility: 100}}},
	}); err != nil {
		t.Fatal(err)
	}
	layers := []world.ObjectLayerState{{ItemID: "nerf", Active: true, Quantity: 20}, {ItemID: "buff", Active: true, Quantity: 1}}
	entities := []world.EntityID{1, 2, 3}
	ms := &world.MapState{}
	for _, id := range entities {
		ms.Spawn(id, world.EntityData{}, world.Transform{}, world.Appearance{Layers: layers})
		ms.AddAgent(id, world.Agent{Behavior: "provider"})
		ms.AddMortal(id, &world.Mortal{})
	}
	for _, id := range entities {
		s.setProgression(id, NewEntityProgression(4, true, s.progressionConfig()))
		expected := s.progressionConfig().BaseAtLevel(4)
		if got := s.CalculateStats(id, ms); got != expected {
			t.Fatalf("%d: %v != %v", id, got, expected)
		}
	}
	layers[1].Active = false
	for _, id := range entities {
		got := s.CalculateStats(id, ms)
		if got.Effect != 1 || got.Resistance != 0 || got.Agility != -90 {
			t.Fatalf("floors: %v", got)
		}
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
	m := ms.Mt(p.ID)
	progression := s.setProgression(p.ID, NewEntityProgression(1, false, s.progressionConfig()))
	for i := uint32(1); i <= 1000; i++ {
		s.HandlePlayerTapAction(p, ms, world.Vec2{X: 2, Y: 2})
	}
	if progression.XP != 0 || progression.Level != 1 {
		t.Fatal("raw taps award XP")
	}
	if m.NextSkillAt.IsZero() {
		t.Fatal("skills have no cooldown")
	}
	if s.acceptSkillAction(p.ID, ms, m.NextSkillAt.Add(-time.Nanosecond)) {
		t.Fatal("early skill accepted")
	}
	if !s.acceptSkillAction(p.ID, ms, m.NextSkillAt) {
		t.Fatal("ready skill rejected")
	}
}

func TestDefeatAwardsOnlyEligibleContributors(t *testing.T) {
	s, ms, p := movementFixture(t)
	victim := world.EntityID(10)
	ms.Spawn(victim, world.EntityData{Kind: world.EntityBot}, world.Transform{}, world.Appearance{})
	ms.AddVitals(victim, world.Vitals{})
	ms.AddAgent(victim, world.Agent{Behavior: "hostile"})
	ms.AddMortal(victim, &world.Mortal{})
	s.setProgression(victim, NewEntityProgression(1, true, s.progressionConfig()))
	ms.Mt(victim).DamageLedger = map[world.EntityID]float64{p.ID: 100}
	s.awardDefeat(victim, "bot", ms)
	if xp := s.progression[p.ID].XP; xp != 25 {
		t.Fatalf("kill XP = %d", xp)
	}
	s.setProgression(p.ID, NewEntityProgression(1, false, s.progressionConfig()))
	ms.Ag(victim).CasterID = p.ID
	s.awardDefeat(victim, "bot", ms)
	if s.progression[p.ID].XP != 0 {
		t.Fatal("summon farming awards XP")
	}
	ms.Ag(victim).CasterID = 0
	ms.Mt(victim).DamageLedger = map[world.EntityID]float64{p.ID: 1, 999: 100}
	s.awardDefeat(victim, "bot", ms)
	if s.progression[p.ID].XP != 0 {
		t.Fatal("low contribution awards XP")
	}
}

func TestStatValidationRejectsCacheMutation(t *testing.T) {
	s := NewGameServer()
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
	ms.Ap(p.ID).Layers = []world.ObjectLayerState{{ItemID: "nerf", Active: true, Quantity: 1}}
	s.setProgression(p.ID, NewEntityProgression(2, false, s.progressionConfig()))
	snap := (&GameServer{Sim: s}).buildSnapshotSelf(p, ms)
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

func TestSummonFloorsAfterAllContributions(t *testing.T) {
	s, ms, p := movementFixture(t)
	s.setProgression(p.ID, NewEntityProgression(1, false, s.progressionConfig()))
	s.ReplaceObjectLayerCache(map[string]*ObjectLayer{
		"nerf": {Data: ObjectLayerData{Stats: Stats{Effect: -100}}},
		"buff": {Data: ObjectLayerData{Stats: Stats{Effect: 100}}},
	})
	ms.Ap(p.ID).Layers = []world.ObjectLayerState{{ItemID: "nerf", Active: true, Quantity: 1}}
	summon := world.EntityID(11)
	ms.Spawn(summon, world.EntityData{Kind: world.EntityBot}, world.Transform{}, world.Appearance{Layers: []world.ObjectLayerState{{ItemID: "buff", Active: true, Quantity: 1}}})
	ms.AddAgent(summon, world.Agent{CasterID: p.ID})
	ms.AddMortal(summon, &world.Mortal{})
	if got := s.CalculateStats(summon, ms).Effect; got != 5 {
		t.Fatalf("floor applied before full sum: %v", got)
	}
	ms.Ag(summon).CasterID = summon
	if got := s.CalculateStats(summon, ms).Effect; math.IsNaN(got) {
		t.Fatal("caster cycle produces invalid stats")
	}
}

func TestProgressionNormalizesAfterCurveChange(t *testing.T) {
	s := &Content{progressionRules: defaultProgressionRules()}
	ms := &world.MapState{}
	ms.Spawn(1, world.EntityData{Kind: world.EntityPlayer}, world.Transform{}, world.Appearance{})
	ms.AddMortal(1, &world.Mortal{})
	s.setProgression(1, NewEntityProgression(3, false, s.progressionConfig()))
	s.progressionRules.XpPerLevel = 200
	if got := s.entityProgression(1, ms); got.Level != 2 || got.XP != 400 {
		t.Fatalf("curve normalization: %+v", got)
	}
	s.progressionRules.MaxLevel = 1
	if got := s.entityProgression(1, ms); got.Level != 1 || got.XP != 0 {
		t.Fatalf("level cap normalization: %+v", got)
	}
}

func TestQuestCompletionAwardsAfterKillAndRejectsReplay(t *testing.T) {
	s, _, p := movementFixture(t)
	s.awardProgression(p, xpKill, "victim", 1, time.Now())
	progress := &QuestProgress{QuestCode: "quest", Status: "active"}
	affected := []QuestSnapshotEntry{}
	s.completeQuest(p, progress, &affected)
	m := s.progression[p.ID]
	if m.XP != 125 || m.Level != 2 {
		t.Fatalf("quest result: %+v", m)
	}
	s.completeQuest(p, progress, &affected)
	if m.XP != 125 || len(affected) != 1 {
		t.Fatal("quest completion replay")
	}
}

func TestAuthoredLevelsUseCommonBaseCurve(t *testing.T) {
	s, ms, _ := movementFixture(t)
	s.buildResource(ms, &pb.EntityMessage{Level: 7, MaxLife: 50, ColorA: 255})
	for resource := range ms.OfKind(world.EntityResource) {
		if p := s.progression[resource]; !p.Frozen || p.Level != 7 || ms.Vt(resource).MaxLife != 90 {
			t.Fatalf("resource curve: %+v", p)
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
