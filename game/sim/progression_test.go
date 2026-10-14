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
	s.AwardDefeat(victim, "bot", ms, s.Social.PartyMembers)
	if xp := s.Progression[p.ID].XP; xp != 25 {
		t.Fatalf("kill XP = %d", xp)
	}
	s.SetProgression(p.ID, content.NewEntityProgression(1, false, s.ProgressionConfig()))
	ms.Ag(victim).CasterID = p.ID
	s.AwardDefeat(victim, "bot", ms, s.Social.PartyMembers)
	if s.Progression[p.ID].XP != 0 {
		t.Fatal("summon farming awards XP")
	}
	ms.Ag(victim).CasterID = 0
	ms.Mt(victim).DamageLedger = map[world.EntityID]float64{p.ID: 1, 999: 100}
	s.AwardDefeat(victim, "bot", ms, s.Social.PartyMembers)
	if s.Progression[p.ID].XP != 0 {
		t.Fatal("low contribution awards XP")
	}
}

// xpFixture is the movement map with the player a online at (11, 11) and a
// level-1 bot victim at (10, 10).
func xpFixture(t *testing.T) (*Sim, *world.MapState, *world.Session, world.EntityID) {
	t.Helper()
	s, ms, a := movementFixture(t)
	s.NewEntityID() // new players take ids after a
	a.PlayerID = "a"
	ms.Tf(a.ID).Pos = world.Vec2{X: 11, Y: 11}
	s.Social.Join(a)
	victim := world.EntityID(50)
	ms.Spawn(victim, world.EntityData{Kind: world.EntityBot},
		world.Transform{Pos: world.Vec2{X: 10, Y: 10}, Size: world.Vec2{X: 1, Y: 1}}, world.Appearance{})
	ms.AddVitals(victim, world.Vitals{})
	ms.AddAgent(victim, world.Agent{Behavior: "hostile"})
	ms.AddMortal(victim, &world.Mortal{})
	s.SetProgression(victim, content.NewEntityProgression(1, true, s.ProgressionConfig()))
	return s, ms, a, victim
}

func xpOf(s *Sim, p *world.Session) uint64 {
	if progression := s.Progression[p.ID]; progression != nil {
		return progression.XP
	}
	return 0
}

func defeat(s *Sim, ms *world.MapState, victim world.EntityID, kind string, ledger map[world.EntityID]float64) {
	ms.Mt(victim).DamageLedger = ledger
	s.AwardDefeat(victim, kind, ms, s.Social.PartyMembers)
}

// A party earns one bot kill once. Its pool splits in integer shares in party
// order, the first members taking the remainder, so the shares add up to the
// kill XP of one player.
func TestPartySplitsOneBotKillAmongNearMembers(t *testing.T) {
	s, ms, a, victim := xpFixture(t)
	b := onlinePlayer(s, ms, "b", world.Vec2{X: 9, Y: 9})
	c := onlinePlayer(s, ms, "c", world.Vec2{X: 12, Y: 10})
	group(t, s, a, b)
	group(t, s, a, c)

	defeat(s, ms, victim, "bot", map[world.EntityID]float64{a.ID: 100})
	if xpOf(s, a) != 9 || xpOf(s, b) != 8 || xpOf(s, c) != 8 {
		t.Fatalf("shares = %d %d %d, want 9 8 8", xpOf(s, a), xpOf(s, b), xpOf(s, c))
	}
}

// Only members near the bot, alive and online at its death share the kill. A
// member far away earns nothing even when it dealt damage, and one member
// alone near the bot takes the whole pool.
func TestOnlyNearLivingMembersShareTheKill(t *testing.T) {
	s, ms, a, victim := xpFixture(t)
	far := onlinePlayer(s, ms, "far", world.Vec2{X: 0, Y: 0})
	dead := onlinePlayer(s, ms, "dead", world.Vec2{X: 10, Y: 11})
	gone := onlinePlayer(s, ms, "gone", world.Vec2{X: 11, Y: 10})
	for _, member := range []*world.Session{far, dead, gone} {
		group(t, s, a, member)
	}
	ms.Vt(dead.ID).RespawnAt = time.Now().Add(time.Minute)
	s.Social.Leave(gone)

	defeat(s, ms, victim, "bot", map[world.EntityID]float64{a.ID: 60, far.ID: 40})
	if xpOf(s, a) != 25 || xpOf(s, far) != 0 || xpOf(s, dead) != 0 || xpOf(s, gone) != 0 {
		t.Fatalf("xp = %d %d %d %d, want 25 0 0 0", xpOf(s, a), xpOf(s, far), xpOf(s, dead), xpOf(s, gone))
	}
}

// A party qualifies through a member that is near the bot. When only a far
// member dealt the damage, nobody earns XP.
func TestAPartyWithNoNearContributorEarnsNothing(t *testing.T) {
	s, ms, a, victim := xpFixture(t)
	far := onlinePlayer(s, ms, "far", world.Vec2{X: 0, Y: 0})
	group(t, s, a, far)

	defeat(s, ms, victim, "bot", map[world.EntityID]float64{far.ID: 100})
	if xpOf(s, a) != 0 || xpOf(s, far) != 0 {
		t.Fatalf("xp = %d %d, want 0 0", xpOf(s, a), xpOf(s, far))
	}
}

// Membership counts at the death. A member who left during the fight earns
// alone, and a player who joined shares with the party.
func TestMembershipAtTheDeathDecidesTheShares(t *testing.T) {
	s, ms, a, victim := xpFixture(t)
	b := onlinePlayer(s, ms, "b", world.Vec2{X: 9, Y: 9})
	c := onlinePlayer(s, ms, "c", world.Vec2{X: 12, Y: 10})
	group(t, s, a, b)
	if reason := s.Social.LeaveParty(b); reason != "" {
		t.Fatalf("leave: %s", reason)
	}
	group(t, s, a, c)

	defeat(s, ms, victim, "bot", map[world.EntityID]float64{a.ID: 50, b.ID: 50})
	if xpOf(s, a) != 13 || xpOf(s, c) != 12 || xpOf(s, b) != 25 {
		t.Fatalf("xp = %d %d %d, want 13 12 25", xpOf(s, a), xpOf(s, c), xpOf(s, b))
	}
}

// The anti-farming rules cut each share for its own earner. A cut share stays
// unpaid, so the party never earns more than one kill.
func TestAntiFarmingCutsEachShareForItsEarner(t *testing.T) {
	s, ms, a, victim := xpFixture(t)
	b := onlinePlayer(s, ms, "b", world.Vec2{X: 9, Y: 9})
	group(t, s, a, b)
	s.AwardProgression(b, content.XpKill, s.ProgressionConfig().KillXp, "earlier", 1, time.Now())
	before := xpOf(s, b)

	defeat(s, ms, victim, "bot", map[world.EntityID]float64{a.ID: 100})
	if xpOf(s, a) != 13 || xpOf(s, b) != before {
		t.Fatalf("xp = %d, +%d; want 13, +0 inside the award interval", xpOf(s, a), xpOf(s, b)-before)
	}
}

// A player kill keeps its rule: each contributor earns the whole kill XP,
// party or not.
func TestAPlayerKillPaysEachContributor(t *testing.T) {
	s, ms, a, _ := xpFixture(t)
	b := onlinePlayer(s, ms, "b", world.Vec2{X: 9, Y: 9})
	enemy := onlinePlayer(s, ms, "enemy", world.Vec2{X: 10, Y: 10})
	group(t, s, a, b)

	defeat(s, ms, enemy.ID, "player", map[world.EntityID]float64{a.ID: 50, b.ID: 50})
	if xpOf(s, a) != 25 || xpOf(s, b) != 25 {
		t.Fatalf("xp = %d %d, want 25 25", xpOf(s, a), xpOf(s, b))
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
	s.AwardProgression(p, content.XpKill, s.ProgressionConfig().KillXp, "victim", 1, time.Now())
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
