// Package social is the chat, the parties and the friends of the players on
// one game server.
//
// World chat reaches every player this server holds: one process runs one
// instance. A party is a runtime group of up to content.PartyMaxMembers
// players, led by its first member. Chat and parties end with the sessions.
// Friendships persist in the engine by account; this package keeps the
// friends of each online account player to route direct lines and markers.
//
// Membership and invites key by world.PlayerID. The wire names a player by
// its entity id, and the caller resolves the session first.
//
// Caller MUST hold the world mutex for every method.
package social

import (
	"strings"
	"time"

	"cyberia-server/game/world"
)

// Refusal reasons, echoed on chat_ack and party_ack.
const (
	ReasonInvalid     = "invalid"      // the request names nothing usable
	ReasonUnavailable = "unavailable"  // the other player is gone or out of reach
	ReasonRateLimited = "rate_limited" // the sender spent its chat budget
	ReasonNoParty     = "no_party"     // the player is in no party
	ReasonInParty     = "in_party"     // the invited player is already in a party
	ReasonNotLeader   = "not_leader"   // only the leader may do this
	ReasonPartyFull   = "party_full"   // the party holds PartyMaxMembers
	ReasonBusy        = "busy"         // the invited player holds too many invites
	ReasonInvitesFull = "invites_full" // the inviter holds too many invites
	ReasonExpired     = "expired"      // the invite was answered, withdrawn or timed out
	ReasonNotMember   = "not_member"   // the player is not in the leader's party
	ReasonGuest       = "guest"        // friends need an account on both sides
)

// Social holds the online players, the parties, the pending invites, the
// friends of the account players and the chat and friend budgets.
type Social struct {
	online   map[world.PlayerID]*world.Session
	accounts map[string]*world.Session // online account players by account id
	parties  map[world.PlayerID]*Party
	invites  map[invite]time.Time // pending invite → expiry
	budgets  map[world.PlayerID]*chatBudget
	friends  map[world.PlayerID]map[string]bool // accepted friend accounts
	requests map[world.PlayerID]*chatBudget     // friend request budgets
}

// New returns an empty registry.
func New() *Social {
	return &Social{
		online:   make(map[world.PlayerID]*world.Session),
		accounts: make(map[string]*world.Session),
		parties:  make(map[world.PlayerID]*Party),
		invites:  make(map[invite]time.Time),
		budgets:  make(map[world.PlayerID]*chatBudget),
		friends:  make(map[world.PlayerID]map[string]bool),
		requests: make(map[world.PlayerID]*chatBudget),
	}
}

// Join registers a connected player. A guest gets a name that no other online
// player holds; an account player keeps its account name.
func (s *Social) Join(p *world.Session) {
	if p.AccountID != "" {
		s.accounts[p.AccountID] = p
	} else {
		p.DisplayName = s.guestName(p.PlayerID)
	}
	s.online[p.PlayerID] = p
}

// Leave removes a player whose session ended: it leaves its party, and every
// invite to or from it lapses.
func (s *Social) Leave(p *world.Session) {
	s.leaveParty(p.PlayerID)
	s.dropInvites(func(i invite) bool { return i.from == p.PlayerID || i.to == p.PlayerID })
	delete(s.budgets, p.PlayerID)
	delete(s.requests, p.PlayerID)
	delete(s.friends, p.PlayerID)
	if s.accounts[p.AccountID] == p {
		delete(s.accounts, p.AccountID)
	}
	delete(s.online, p.PlayerID)
}

// Account is the online session of an account, or nil.
func (s *Social) Account(accountID string) *world.Session { return s.accounts[accountID] }

// Player is the online session of id, or nil.
func (s *Social) Player(id world.PlayerID) *world.Session { return s.online[id] }

// guestName derives a short name from the player id, longer only when a
// shorter one is taken.
func (s *Social) guestName(id world.PlayerID) string {
	code := strings.ToUpper(strings.ReplaceAll(string(id), "-", ""))
	for n := min(4, len(code)); n < len(code); n += 2 {
		if name := "Guest-" + code[:n]; !s.nameTaken(name) {
			return name
		}
	}
	return "Guest-" + code
}

func (s *Social) nameTaken(name string) bool {
	for _, p := range s.online {
		if p.DisplayName == name {
			return true
		}
	}
	return false
}
