// Package social — friends.go
//
// The friends of the online account players. The engine owns the
// friendships; the game server keeps the accepted friend accounts of each
// online player, so a direct line and a marker need no engine call.
//
// Cross-process contract:
//
//	friends_list {}                      — ask for the friend list
//	friend_request {toId}                — ask a visible account player
//	friend_reply {friendshipId, accept}  — answer a request; decline also withdraws one
//	friend_remove {friendshipId}         — end a friendship
//	friends (server→client)              — the whole list, after each change
//	friend_ack {ok, reason, action}      — sent on a refusal only
package social

import (
	"time"

	"cyberia-server/game/world"
)

// A player spends one friend action of its budget per request; the budget
// holds friendBurst and gets one back every friendRefill.
const (
	friendBurst  = 3
	friendRefill = 2 * time.Second
)

// Friend is one row of the friend list a client receives. ID is the
// friendship id; EntityID and Skin, the item that draws its identity, are set
// while the friend plays on this server.
type Friend struct {
	ID          string         `json:"id"`
	Name        string         `json:"name"`
	Skin        string         `json:"skin,omitempty"`
	Status      string         `json:"status"` // friend | incoming | outgoing
	Online      bool           `json:"online"`
	World       string         `json:"world,omitempty"`
	MapCode     string         `json:"mapCode,omitempty"`
	EntityID    world.EntityID `json:"entityId,omitempty"`
	ExpiresInMs int64          `json:"expiresInMs,omitempty"`
}

// SetFriends replaces the accepted friend accounts of an online player.
func (s *Social) SetFriends(id world.PlayerID, accounts []string) {
	if s.online[id] == nil {
		return
	}
	set := make(map[string]bool, len(accounts))
	for _, account := range accounts {
		set[account] = true
	}
	s.friends[id] = set
}

// SetFriendship records that two accounts are friends now, or no more, for
// whichever of them plays on this server.
func (s *Social) SetFriendship(a, b string, friends bool) {
	for _, pair := range [][2]string{{a, b}, {b, a}} {
		p := s.accounts[pair[0]]
		if p == nil {
			continue
		}
		if s.friends[p.PlayerID] == nil {
			s.friends[p.PlayerID] = make(map[string]bool)
		}
		if friends {
			s.friends[p.PlayerID][pair[1]] = true
		} else {
			delete(s.friends[p.PlayerID], pair[1])
		}
	}
}

// Friends reports whether two players are accepted friends.
func (s *Social) Friends(a, b *world.Session) bool {
	return a != nil && b != nil && b.AccountID != "" && s.friends[a.PlayerID][b.AccountID]
}

// SpendFriendAction takes one action from the friend budget of a player and
// returns "" or the reason it refused.
func (s *Social) SpendFriendAction(p *world.Session, now time.Time) string {
	if p.AccountID == "" {
		return ReasonGuest
	}
	b := s.requests[p.PlayerID]
	if b == nil {
		b = &chatBudget{lines: friendBurst, at: now}
		s.requests[p.PlayerID] = b
	}
	b.lines = min(friendBurst, b.lines+now.Sub(b.at).Seconds()/friendRefill.Seconds())
	b.at = now
	if b.lines < 1 {
		return ReasonRateLimited
	}
	b.lines--
	return ""
}
