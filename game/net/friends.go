// Package net — friends.go
//
// The friend broker: it runs friend actions against the engine off the tick,
// updates the friend cache of the players on this server, and sends each
// player concerned a fresh friend list.
package net

import (
	"context"
	"time"

	"cyberia-server/game/sim"
	"cyberia-server/game/social"
	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// friendCallTimeout bounds one friend call to the engine.
const friendCallTimeout = 5 * time.Second

// RunFriendOp runs one friend action of a player, off the tick.
func (s *GameServer) RunFriendOp(op sim.FriendOp) {
	go s.runFriendOp(op)
}

func (s *GameServer) runFriendOp(op sim.FriendOp) {
	if op.Action != "list" {
		ctx, cancel := context.WithTimeout(context.Background(), friendCallTimeout)
		refusal, friend, err := s.accounts.UpdateFriendship(ctx, op.AccountID, op.Action, op.TargetAccountID, op.FriendshipID)
		cancel()
		if err != nil {
			logx.Warnf("[Friends] %s by account=%s failed: %v", op.Action, op.AccountID, err)
			refusal = EndUnavailable
		}
		if refusal != "" {
			s.friendAck(op.PlayerID, refusal, op.Action)
			return
		}
		if friend != nil {
			s.Mu.Lock()
			switch {
			case op.Action == "remove":
				s.Social.SetFriendship(op.AccountID, friend.AccountID, false)
			case friend.Status == "friend":
				s.Social.SetFriendship(op.AccountID, friend.AccountID, true)
			}
			other := s.Social.Account(friend.AccountID)
			s.Mu.Unlock()
			if other != nil {
				s.pushFriends(other.PlayerID)
			}
		}
	}
	s.pushFriends(op.PlayerID)
}

// friendAck answers a refused friend action.
func (s *GameServer) friendAck(id world.PlayerID, reason, action string) {
	s.Mu.Lock()
	player := s.Social.Player(id)
	s.Mu.Unlock()
	if player != nil {
		world.SendAck(player, "friend_ack", reason, map[string]interface{}{"action": action})
	}
}

// announceArrival sends a player that just joined its friend list, and every
// friend online on this server a fresh one, which shows the new presence.
func (s *GameServer) announceArrival(id world.PlayerID) {
	s.pushFriends(id)
	s.Mu.Lock()
	player := s.Social.Player(id)
	var friends []world.PlayerID
	for _, other := range s.Sessions {
		if other != player && s.Social.Friends(player, other) {
			friends = append(friends, other.PlayerID)
		}
	}
	s.Mu.Unlock()
	for _, friend := range friends {
		s.pushFriends(friend)
	}
}

// pushFriends reads the friend list of an online account player, refreshes
// its friend cache and sends it the list.
func (s *GameServer) pushFriends(id world.PlayerID) {
	s.Mu.Lock()
	player := s.Social.Player(id)
	s.Mu.Unlock()
	if player == nil || player.AccountID == "" {
		return
	}
	ctx, cancel := context.WithTimeout(context.Background(), friendCallTimeout)
	records, err := s.accounts.ListFriends(ctx, player.AccountID)
	cancel()
	if err != nil {
		logx.Warnf("[Friends] list of account=%s failed: %v", player.AccountID, err)
		s.friendAck(id, EndUnavailable, "list")
		return
	}

	now := time.Now()
	s.Mu.Lock()
	defer s.Mu.Unlock()
	if s.Social.Player(id) != player {
		return
	}
	rows := make([]social.Friend, 0, len(records))
	var accepted []string
	for _, r := range records {
		row := social.Friend{ID: r.FriendshipID, Name: r.Name, Status: r.Status}
		// The entity id lets the client match a row to a player it sees; the
		// presence goes to friends only.
		here := s.Social.Account(r.AccountID)
		if here != nil {
			row.EntityID, row.Skin = here.ID, s.PlayerSkin(here)
		}
		switch {
		case r.Status == "friend" && here != nil:
			accepted = append(accepted, r.AccountID)
			row.Online, row.World, row.MapCode = true, s.InstanceCode, here.MapCode
		case r.Status == "friend":
			accepted = append(accepted, r.AccountID)
			row.Online, row.World, row.MapCode = r.Online, r.InstanceCode, r.MapCode
		case !r.ExpiresAt.IsZero():
			row.ExpiresInMs = max(0, r.ExpiresAt.Sub(now).Milliseconds())
		}
		rows = append(rows, row)
	}
	s.Social.SetFriends(id, accepted)
	world.SendMessage(player, "friends", map[string]interface{}{"friends": rows})
}
