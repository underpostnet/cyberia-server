package sim

import (
	"math"
	"testing"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
)

func TestRawTapSpamDoesNotAwardXP(t *testing.T) {
	s, ms, p := movementFixture(t)
	m := ms.Mt(p.ID)
	progression := s.SetProgression(p.ID, content.NewEntityProgression(1, false, s.ProgressionConfig()))
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
	s.SetProgression(victim, content.NewEntityProgression(1, true, s.ProgressionConfig()))
	ms.Mt(victim).DamageLedger = map[world.EntityID]float64{p.ID: 100}
	s.AwardDefeat(victim, "bot", ms)
	if xp := s.Progression[p.ID].XP; xp != 25 {
		t.Fatalf("kill XP = %d", xp)
	}
	s.SetProgression(p.ID, content.NewEntityProgression(1, false, s.ProgressionConfig()))
	ms.Ag(victim).CasterID = p.ID
	s.AwardDefeat(victim, "bot", ms)
	if s.Progression[p.ID].XP != 0 {
		t.Fatal("summon farming awards XP")
	}
	ms.Ag(victim).CasterID = 0
	ms.Mt(victim).DamageLedger = map[world.EntityID]float64{p.ID: 1, 999: 100}
	s.AwardDefeat(victim, "bot", ms)
	if s.Progression[p.ID].XP != 0 {
		t.Fatal("low contribution awards XP")
	}
}

func TestSummonFloorsAfterAllContributions(t *testing.T) {
	s, ms, p := movementFixture(t)
	s.SetProgression(p.ID, content.NewEntityProgression(1, false, s.ProgressionConfig()))
	s.ReplaceObjectLayerCache(map[string]*content.ObjectLayer{
		"nerf": {Data: content.ObjectLayerData{Stats: content.Stats{Effect: -100}}},
		"buff": {Data: content.ObjectLayerData{Stats: content.Stats{Effect: 100}}},
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

func TestQuestCompletionAwardsAfterKillAndRejectsReplay(t *testing.T) {
	s, _, p := movementFixture(t)
	s.AwardProgression(p, content.XpKill, "victim", 1, time.Now())
	progress := &content.QuestProgress{QuestCode: "quest", Status: "active"}
	affected := []content.QuestSnapshotEntry{}
	s.CompleteQuest(p, progress, &affected)
	m := s.Progression[p.ID]
	if m.XP != 125 || m.Level != 2 {
		t.Fatalf("quest result: %+v", m)
	}
	s.CompleteQuest(p, progress, &affected)
	if m.XP != 125 || len(affected) != 1 {
		t.Fatal("quest completion replay")
	}
}

func TestAuthoredLevelsUseCommonBaseCurve(t *testing.T) {
	s, ms, _ := movementFixture(t)
	s.BuildResource(ms, &pb.EntityMessage{Level: 7, MaxLife: 50, ColorA: 255})
	for resource := range ms.OfKind(world.EntityResource) {
		if p := s.Progression[resource]; !p.Frozen || p.Level != 7 || ms.Vt(resource).MaxLife != 90 {
			t.Fatalf("resource curve: %+v", p)
		}
	}
	rules := s.ProgressionConfig()
	for level := 1; level <= rules.MaxLevel; level++ {
		if got := rules.LevelAtXP(rules.Threshold(level)); got != level {
			t.Fatalf("level %d -> %d", level, got)
		}
		if level > 1 && rules.LevelAtXP(rules.Threshold(level)-1) != level-1 {
			t.Fatal("early level transition")
		}
	}
	if err := content.ValidateWorldProgression(&pb.InstanceConfig{}, []*pb.MapDataMessage{{Entities: []*pb.EntityMessage{{Level: 101}}}}); err == nil {
		t.Fatal("invalid authored level")
	}
}
