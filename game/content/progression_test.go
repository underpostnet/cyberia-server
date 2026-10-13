package content

import (
	"encoding/json"
	"testing"
	"time"

	"cyberia-server/game/world"
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
		s.SetProgression(id, NewEntityProgression(4, true, s.ProgressionConfig()))
		expected := s.ProgressionConfig().BaseAtLevel(4)
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
		if entity.award(rules, XpKill, kind, 7, now) != 0 || entity.Level != 7 || entity.XP != before {
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
	first := p.award(rules, XpKill, "target", 1, now)
	if first != 25 {
		t.Fatalf("first reward %d", first)
	}
	if p.award(rules, XpKill, "other", 1, now) != 0 {
		t.Fatal("cooldown bypass")
	}
	second := p.award(rules, XpKill, "target", 1, now.Add(time.Second))
	if second != 12 {
		t.Fatalf("repeated reward %d", second)
	}
	for i := 2; i < rules.MaxRepeatAwards; i++ {
		p.award(rules, XpKill, "target", 1, now.Add(time.Duration(i)*time.Second))
	}
	if p.award(rules, XpKill, "target", 1, now.Add(10*time.Second)) != 0 {
		t.Fatal("repeat limit bypass")
	}
	p = NewEntityProgression(1, false, rules)
	rules.MaxAwardsPerWindow = 2
	for i, key := range []string{"a", "b", "c"} {
		got := p.award(rules, XpKill, key, 1, now.Add(time.Duration(i)*time.Second))
		if (i < 2) != (got > 0) {
			t.Fatalf("window reward %d: %d", i, got)
		}
	}
	if p.award(rules, XpKill, "d", 1, now.Add(time.Minute)) == 0 {
		t.Fatal("window did not reset")
	}
}

func TestProgressionNormalizesAfterCurveChange(t *testing.T) {
	s := &Content{progressionRules: defaultProgressionRules()}
	ms := &world.MapState{}
	ms.Spawn(1, world.EntityData{Kind: world.EntityPlayer}, world.Transform{}, world.Appearance{})
	ms.AddMortal(1, &world.Mortal{})
	s.SetProgression(1, NewEntityProgression(3, false, s.ProgressionConfig()))
	s.progressionRules.XpPerLevel = 200
	if got := s.EntityProgression(1, ms); got.Level != 2 || got.XP != 400 {
		t.Fatalf("curve normalization: %+v", got)
	}
	s.progressionRules.MaxLevel = 1
	if got := s.EntityProgression(1, ms); got.Level != 1 || got.XP != 0 {
		t.Fatalf("level cap normalization: %+v", got)
	}
}
