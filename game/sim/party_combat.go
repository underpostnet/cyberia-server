package sim

import (
	"cyberia-server/game/world"
)

// ownerOf is the player behind id: id itself when it is a player, else the
// owner its spawn recorded. 0 for none.
func (s *Sim) ownerOf(ms *world.MapState, id world.EntityID) world.EntityID {
	if s.Sessions[id] != nil {
		return id
	}
	if ms.Has(id, world.MaskAgent) {
		return ms.Ag(id).OwnerID
	}
	return 0
}

// summonAgent is the agent of a bot that casterID makes. It records the player
// behind the caster, so the bot keeps its owner after the caster is gone.
func (s *Sim) summonAgent(ms *world.MapState, behavior string, casterID world.EntityID) world.Agent {
	return world.Agent{Behavior: behavior, CasterID: casterID, OwnerID: s.ownerOf(ms, casterID)}
}

// friendlyHit reports whether a hit of attacker on target falls between two
// players of one party, their summons and projectiles included. Membership
// counts at the moment of the hit.
func (s *Sim) friendlyHit(ms *world.MapState, attacker, target world.EntityID) bool {
	a, b := s.Sessions[s.ownerOf(ms, attacker)], s.Sessions[s.ownerOf(ms, target)]
	return a != nil && b != nil && a != b && s.Social.SameParty(a.PlayerID, b.PlayerID)
}
