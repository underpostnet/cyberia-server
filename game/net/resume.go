// Package net — resume.go
//
// Resume tickets. A live account session holds the next launch ticket of its
// own world, bound to its profile epoch, so a client whose connection drops
// rejoins without the portal. A newer launch of the account voids it. The
// engine issues it off the tick; the client keeps it in memory only.
//
//	resume (server→client) {ticket}
package net

import (
	"context"
	"time"

	"cyberia-server/game/world"
	"cyberia-server/logx"
)

const (
	// resumeRenewAhead is how long before its expiry a resume ticket is renewed.
	resumeRenewAhead = 2 * time.Minute
	// resumeSweep is how often the renewal looks for tickets near expiry.
	resumeSweep = 30 * time.Second
	// resumeCallTimeout bounds one resume call to the engine.
	resumeCallTimeout = 5 * time.Second
)

type resumePayload struct {
	Ticket string `json:"ticket"`
}

// issueResume asks the engine for the next resume ticket of a live account
// player and sends it to its client. A refusal or a failure leaves the older
// ticket in place; the next sweep asks again.
func (s *GameServer) issueResume(id world.PlayerID) {
	lease, live := s.durable.Lease(string(id))
	if !live {
		return
	}
	ctx, cancel := context.WithTimeout(context.Background(), resumeCallTimeout)
	ticket, err := s.accounts.IssueResume(ctx, lease, s.InstanceCode)
	cancel()
	if err != nil || ticket.Refusal != "" {
		logx.Debugf("[Resume] player=%s: no ticket (refusal=%q err=%v)", id, ticket.Refusal, err)
		return
	}
	s.Mu.Lock()
	defer s.Mu.Unlock()
	player := s.Social.Player(id)
	if player == nil {
		return
	}
	if client, ok := player.Client.(*Client); ok {
		client.resumeExpiresAt = ticket.ExpiresAt
		world.SendMessage(player, "resume", resumePayload{Ticket: ticket.Ticket})
	}
}

// RunResumeTickets renews the resume ticket of every account client whose
// ticket expires within resumeRenewAhead, until ctx ends.
func (s *GameServer) RunResumeTickets(ctx context.Context) {
	ticker := time.NewTicker(resumeSweep)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
		}
		now := time.Now()
		var due []world.PlayerID
		s.Mu.Lock()
		for _, client := range s.clients {
			if client.playerState.AccountID != "" && client.resumeExpiresAt.Sub(now) < resumeRenewAhead {
				due = append(due, client.playerState.PlayerID)
			}
		}
		s.Mu.Unlock()
		for _, id := range due {
			s.issueResume(id)
		}
	}
}
