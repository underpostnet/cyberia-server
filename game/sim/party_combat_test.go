package sim

import (
	"testing"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// onlinePlayer places an online player named name at the point.
func onlinePlayer(s *Sim, ms *world.MapState, name world.PlayerID, at world.Vec2) *world.Session {
	p := &world.Session{ID: s.NewEntityID(), MapCode: "test", PlayerID: name, Client: nopConn{}}
	s.Sessions[p.ID] = p
	ms.Spawn(p.ID, world.EntityData{Kind: world.EntityPlayer},
		world.Transform{Pos: at, Size: world.Vec2{X: 1, Y: 1}}, world.Appearance{})
	ms.AddVitals(p.ID, world.Vitals{Life: 100, MaxLife: 100})
	ms.AddMotion(p.ID, world.Motion{})
	ms.AddMortal(p.ID, &world.Mortal{})
	ms.AddPlayer(p.ID)
	s.Social.Join(p)
	return p
}

// partyFixture is the bullet map with a second player, b, at (10, 10). Both
// players are online and in no party.
func partyFixture(t *testing.T) (*Sim, *world.MapState, *world.Session, *world.Session) {
	t.Helper()
	s, ms, a := destructibleFixture(t)
	a.PlayerID = "a"
	s.Social.Join(a)
	return s, ms, a, onlinePlayer(s, ms, "b", world.Vec2{X: 10, Y: 10})
}

func group(t *testing.T, s *Sim, leader, member *world.Session) {
	t.Helper()
	now := time.Now()
	if reason := s.Social.Invite(leader, member, now); reason != "" {
		t.Fatalf("invite: %s", reason)
	}
	if reason := s.Social.Reply(member, leader, true, now); reason != "" {
		t.Fatalf("reply: %s", reason)
	}
}

// bullet spawns a damaging projectile of caster at the point, with the agent a
// skill handler gives it.
func bullet(s *Sim, ms *world.MapState, id, caster world.EntityID, at world.Vec2) {
	ms.AddBrain(id, &world.Brain{})
	ms.Spawn(id, world.EntityData{Kind: world.EntityBot}, world.Transform{Pos: at, Size: world.Vec2{X: 1, Y: 1}},
		world.Appearance{Layers: []world.ObjectLayerState{{ItemID: "bullet", Active: true, Quantity: 1}}})
	ms.AddVitals(id, world.Vitals{MaxLife: 100, Life: 100})
	ms.AddMotion(id, world.Motion{})
	ms.AddAgent(id, s.summonAgent(ms, content.BehaviorSkill, caster))
	ms.AddMortal(id, &world.Mortal{})
}

// summon spawns a doppelganger of caster at the point.
func summon(s *Sim, ms *world.MapState, id, caster world.EntityID, at world.Vec2) {
	ms.AddBrain(id, &world.Brain{})
	ms.Spawn(id, world.EntityData{Kind: world.EntityBot}, world.Transform{Pos: at, Size: world.Vec2{X: 1, Y: 1}}, world.Appearance{})
	ms.AddVitals(id, world.Vitals{MaxLife: 100, Life: 100})
	ms.AddMotion(id, world.Motion{})
	ms.AddAgent(id, s.summonAgent(ms, content.BehaviorPassive, caster))
	ms.AddMortal(id, &world.Mortal{})
}

func lifeOf(ms *world.MapState, id world.EntityID) float64 { return ms.Vt(id).Life }

// A hit between two members of one party lands nothing: no damage, no ledger
// entry, no thorns. Membership counts at the hit, so a member who leaves takes
// damage again.
func TestPartyMembersTakeNoDirectHit(t *testing.T) {
	s, ms, a, b := partyFixture(t)
	group(t, s, a, b)
	at := world.Vec2{X: 10, Y: 10}

	bullet(s, ms, 900, a.ID, at)
	s.handleSkillCollisions(ms)
	if lifeOf(ms, b.ID) != 100 || len(ms.Mt(b.ID).DamageLedger) != 0 || lifeOf(ms, 900) != 100 {
		t.Fatalf("life = %v, ledger = %v, bullet = %v: a party hit must land nothing",
			lifeOf(ms, b.ID), ms.Mt(b.ID).DamageLedger, lifeOf(ms, 900))
	}
	s.Despawn(ms, 900)

	if reason := s.Social.LeaveParty(b); reason != "" {
		t.Fatalf("leave: %s", reason)
	}
	fire(s, ms, a, at)
	if lifeOf(ms, b.ID) >= 100 || ms.Mt(b.ID).DamageLedger[a.ID] != 100-lifeOf(ms, b.ID) {
		t.Fatalf("life = %v, ledger = %v: a player outside the party takes the hit",
			lifeOf(ms, b.ID), ms.Mt(b.ID).DamageLedger)
	}
}

// A projectile fired before the party formed meets the rule at its hit.
func TestAProjectileInFlightMeetsTheRuleAtItsHit(t *testing.T) {
	s, ms, a, b := partyFixture(t)
	bullet(s, ms, 900, a.ID, world.Vec2{X: 10, Y: 10})
	group(t, s, a, b)

	s.handleSkillCollisions(ms)
	if lifeOf(ms, b.ID) != 100 {
		t.Fatalf("life = %v: the bullet hit after the party formed", lifeOf(ms, b.ID))
	}
}

// Summons stand with their owner: the attack of a member's summon spares the
// other members, a member's attack spares their summons, and the rule holds
// after the summon that fired is gone. A hostile bot still hits a member.
func TestSummonsAndTheirAttacksSpareTheParty(t *testing.T) {
	s, ms, a, b := partyFixture(t)
	group(t, s, a, b)

	summon(s, ms, 800, a.ID, world.Vec2{X: 3, Y: 3})
	bullet(s, ms, 900, 800, world.Vec2{X: 10, Y: 10})
	s.Despawn(ms, 800)
	s.handleSkillCollisions(ms)
	if lifeOf(ms, b.ID) != 100 {
		t.Fatalf("life = %v: the summon of a member hit another member", lifeOf(ms, b.ID))
	}
	s.Despawn(ms, 900)

	summon(s, ms, 801, b.ID, world.Vec2{X: 15, Y: 15})
	bullet(s, ms, 901, a.ID, world.Vec2{X: 15, Y: 15})
	s.handleSkillCollisions(ms)
	if lifeOf(ms, 801) != 100 {
		t.Fatalf("life = %v: a member hit the summon of another member", lifeOf(ms, 801))
	}
	s.Despawn(ms, 901)

	addBot(ms, 700, world.Transform{Pos: world.Vec2{X: 5, Y: 5}, Size: world.Vec2{X: 1, Y: 1}},
		&world.Brain{}, world.Motion{}, content.BehaviorHostile)
	bullet(s, ms, 902, 700, world.Vec2{X: 10, Y: 10})
	s.handleSkillCollisions(ms)
	if lifeOf(ms, b.ID) >= 100 {
		t.Fatalf("life = %v: a hostile bot must still hit a party member", lifeOf(ms, b.ID))
	}
}

func TestAPartyInviteReachesAFriendOnAnotherMap(t *testing.T) {
	s, _, a, b := partyFixture(t)
	a.AccountID, b.AccountID = "acc-a", "acc-b"
	s.Social.Join(a)
	s.Social.Join(b)
	b.MapCode = "elsewhere"
	invite := func() {
		s.handlePartyInput(a, &world.InputCommand{Kind: world.InputKindPartyInvite, EntityID: b.ID})
	}

	invite()
	if s.Social.Invited(a.PlayerID, b.PlayerID, time.Now()) {
		t.Fatal("a player out of view who is no friend is out of reach")
	}
	s.Social.SetFriendship("acc-a", "acc-b", true)
	invite()
	if !s.Social.Invited(a.PlayerID, b.PlayerID, time.Now()) {
		t.Fatal("an invite reaches a friend on another map of this server")
	}
	if reason := s.Social.Reply(b, a, true, time.Now()); reason != "" {
		t.Fatalf("the friend accepts from another map: %s", reason)
	}
}
