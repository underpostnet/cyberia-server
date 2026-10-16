// Package sim — friends.go
//
// Friend actions. The tick validates each one against the live world and the
// friend budget, then hands it to the broker, which runs it against the
// engine off the tick and answers with a fresh friend list.
package sim

import (
	"time"

	"cyberia-server/game/social"
	"cyberia-server/game/world"
)

// FriendOp is one friend action of an account player. TargetAccountID names
// the player a request goes to; FriendshipID the friendship an answer or a
// removal names.
type FriendOp struct {
	PlayerID        world.PlayerID
	AccountID       string
	Action          string // list | request | accept | decline | remove
	TargetAccountID string
	FriendshipID    string
}

// FriendBroker runs friend actions against the engine, off the tick.
type FriendBroker interface {
	RunFriendOp(op FriendOp)
}

// reachablePlayer is a player in the viewer's view, or an accepted friend of
// the viewer on any map of this server; nil otherwise.
func (s *Sim) reachablePlayer(viewer *world.Session, id world.EntityID) *world.Session {
	if target := s.VisiblePlayer(viewer, id); target != nil {
		return target
	}
	if target := s.Sessions[id]; s.Social.Friends(viewer, target) {
		return target
	}
	return nil
}

// handleFriendInput validates one friend action and hands it to the broker. A
// refusal answers with friend_ack.
func (s *Sim) handleFriendInput(player *world.Session, cmd *world.InputCommand) {
	op := FriendOp{PlayerID: player.PlayerID, AccountID: player.AccountID, FriendshipID: cmd.FriendshipID}
	var reason string
	switch cmd.Kind {
	case world.InputKindFriendList:
		op.Action = "list"
	case world.InputKindFriendRequest:
		op.Action = "request"
		switch target := s.VisiblePlayer(player, cmd.EntityID); {
		case target == nil || target == player:
			reason = social.ReasonUnavailable
		case target.AccountID == "":
			reason = social.ReasonGuest
		default:
			op.TargetAccountID = target.AccountID
		}
	case world.InputKindFriendReply:
		op.Action = "decline"
		if cmd.Active {
			op.Action = "accept"
		}
	case world.InputKindFriendRemove:
		op.Action = "remove"
	}
	if reason == "" && s.Friendships == nil {
		reason = social.ReasonGuest
	}
	if reason == "" {
		reason = s.Social.SpendFriendAction(player, time.Now())
	}
	if reason != "" {
		world.SendAck(player, "friend_ack", reason, map[string]interface{}{"action": op.Action})
		return
	}
	s.Friendships.RunFriendOp(op)
}
