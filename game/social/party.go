// Package social — party.go
//
// Party lifecycle. A player who invites while in no party founds one when the
// first invite is accepted. The leader is the first member; when the leader
// goes, the next member in join order leads. A party of one ends.
//
// Cross-process contract:
//
//	party_invite {toId}            — invite a visible player
//	party_reply  {fromId, accept}  — answer an invite
//	party_leave, party_kick {toId}, party_disband
//	party_ack    {ok, reason, action} — sent on a refusal only
//	snapshot self.party / self.partyInvites / self.partyInvitesSent — the whole state, every tick
package social

import (
	"slices"
	"strings"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// InviteTTL is how long an invite waits for its answer.
const InviteTTL = 60 * time.Second

// Party is one group of players. Members keeps join order and starts with
// the leader.
type Party struct {
	Members []world.PlayerID
}

// Leader is the player who leads the party.
func (p *Party) Leader() world.PlayerID { return p.Members[0] }

type invite struct {
	from, to world.PlayerID
}

// Invite is one pending invite between two players.
type Invite struct {
	From, To  world.PlayerID
	ExpiresAt time.Time
}

// PartyOf is the party of id, or nil.
func (s *Social) PartyOf(id world.PlayerID) *Party { return s.parties[id] }

// SameParty reports whether a and b are members of one party.
func (s *Social) SameParty(a, b world.PlayerID) bool {
	party := s.parties[a]
	return party != nil && party == s.parties[b]
}

// PartyMembers is the online sessions of id's party in join order, or nil.
func (s *Social) PartyMembers(id world.PlayerID) []*world.Session {
	party := s.parties[id]
	if party == nil {
		return nil
	}
	members := make([]*world.Session, 0, len(party.Members))
	for _, member := range party.Members {
		if p := s.online[member]; p != nil {
			members = append(members, p)
		}
	}
	return members
}

// inviteRefusal is why from cannot invite to now, or "".
func (s *Social) inviteRefusal(from, to *world.Session) string {
	if to == nil || from.PlayerID == to.PlayerID {
		return ReasonUnavailable
	}
	if s.parties[to.PlayerID] != nil {
		return ReasonInParty
	}
	if party := s.parties[from.PlayerID]; party != nil {
		if party.Leader() != from.PlayerID {
			return ReasonNotLeader
		}
		if len(party.Members) >= content.PartyMaxMembers {
			return ReasonPartyFull
		}
	}
	return ""
}

// CanInvite reports whether from may invite to now.
func (s *Social) CanInvite(from, to *world.Session) bool { return s.inviteRefusal(from, to) == "" }

// Invited reports whether from has an invite to to that waits for an answer.
func (s *Social) Invited(from, to world.PlayerID, now time.Time) bool {
	expiresAt, ok := s.invites[invite{from, to}]
	return ok && now.Before(expiresAt)
}

// Invite records an invite from from to to. A repeat renews it. It returns ""
// or the reason it refused.
func (s *Social) Invite(from, to *world.Session, now time.Time) string {
	if reason := s.inviteRefusal(from, to); reason != "" {
		return reason
	}
	s.dropInvites(func(i invite) bool { return !now.Before(s.invites[i]) })
	key := invite{from.PlayerID, to.PlayerID}
	if _, renew := s.invites[key]; !renew {
		if len(s.InvitesTo(to.PlayerID, now)) >= content.PartyInvitesMax {
			return ReasonBusy
		}
		if len(s.InvitesFrom(from.PlayerID, now)) >= content.PartyInvitesMax {
			return ReasonInvitesFull
		}
	}
	s.invites[key] = now.Add(InviteTTL)
	return ""
}

// InvitesTo lists the pending invites to id, the one that expires first first.
func (s *Social) InvitesTo(id world.PlayerID, now time.Time) []Invite {
	return s.pendingInvites(now, func(i invite) bool { return i.to == id })
}

// InvitesFrom lists the pending invites id sent, the one that expires first first.
func (s *Social) InvitesFrom(id world.PlayerID, now time.Time) []Invite {
	return s.pendingInvites(now, func(i invite) bool { return i.from == id })
}

func (s *Social) pendingInvites(now time.Time, match func(invite) bool) []Invite {
	var out []Invite
	for key, expiresAt := range s.invites {
		if match(key) && now.Before(expiresAt) {
			out = append(out, Invite{From: key.from, To: key.to, ExpiresAt: expiresAt})
		}
	}
	slices.SortFunc(out, func(a, b Invite) int {
		if c := a.ExpiresAt.Compare(b.ExpiresAt); c != 0 {
			return c
		}
		if c := strings.Compare(string(a.From), string(b.From)); c != 0 {
			return c
		}
		return strings.Compare(string(a.To), string(b.To))
	})
	return out
}

// Reply answers the invite from from to to. Accepting seats to in the party
// of from, which a first accept founds. It returns "" or the reason it refused.
func (s *Social) Reply(to, from *world.Session, accept bool, now time.Time) string {
	if from == nil || !s.Invited(from.PlayerID, to.PlayerID, now) {
		return ReasonExpired
	}
	delete(s.invites, invite{from.PlayerID, to.PlayerID})
	if !accept {
		return ""
	}
	if reason := s.inviteRefusal(from, to); reason != "" {
		return reason
	}
	party := s.parties[from.PlayerID]
	if party == nil {
		party = &Party{Members: []world.PlayerID{from.PlayerID}}
		s.parties[from.PlayerID] = party
	}
	party.Members = append(party.Members, to.PlayerID)
	s.parties[to.PlayerID] = party
	// A seated member answers no other invite and leads no invite of its own.
	s.dropInvites(func(i invite) bool { return i.to == to.PlayerID || i.from == to.PlayerID })
	if len(party.Members) >= content.PartyMaxMembers {
		s.dropInvites(func(i invite) bool { return i.from == party.Leader() })
	}
	return ""
}

// LeaveParty takes p out of its party. It returns "" or the reason it refused.
func (s *Social) LeaveParty(p *world.Session) string {
	if !s.leaveParty(p.PlayerID) {
		return ReasonNoParty
	}
	return ""
}

// Kick lets the leader take member out of the party.
func (s *Social) Kick(leader, member *world.Session) string {
	party := s.parties[leader.PlayerID]
	switch {
	case party == nil:
		return ReasonNoParty
	case party.Leader() != leader.PlayerID:
		return ReasonNotLeader
	case member == nil || member.PlayerID == leader.PlayerID || s.parties[member.PlayerID] != party:
		return ReasonNotMember
	}
	s.leaveParty(member.PlayerID)
	return ""
}

// Disband lets the leader end the party.
func (s *Social) Disband(leader *world.Session) string {
	party := s.parties[leader.PlayerID]
	switch {
	case party == nil:
		return ReasonNoParty
	case party.Leader() != leader.PlayerID:
		return ReasonNotLeader
	}
	for _, id := range party.Members {
		delete(s.parties, id)
	}
	s.dropInvites(func(i invite) bool { return i.from == leader.PlayerID })
	return ""
}

// leaveParty takes id out of its party and reports whether it was in one. A
// leaver's invites lapse; a party of one ends.
func (s *Social) leaveParty(id world.PlayerID) bool {
	party := s.parties[id]
	if party == nil {
		return false
	}
	delete(s.parties, id)
	party.Members = slices.DeleteFunc(party.Members, func(m world.PlayerID) bool { return m == id })
	s.dropInvites(func(i invite) bool { return i.from == id })
	if len(party.Members) == 1 {
		delete(s.parties, party.Members[0])
	}
	return true
}

func (s *Social) dropInvites(match func(invite) bool) {
	for key := range s.invites {
		if match(key) {
			delete(s.invites, key)
		}
	}
}
