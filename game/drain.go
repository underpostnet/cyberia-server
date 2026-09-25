package game

import (
	"context"
	"time"
)

// BeginDrain stops admitting sessions. Connected players keep playing until
// they leave or the drain deadline closes the server. Idempotent.
func (s *GameServer) BeginDrain() {
	s.draining.Store(true)
}

// IsDraining reports whether the server refuses new sessions.
func (s *GameServer) IsDraining() bool {
	return s.draining.Load()
}

// WaitDrained returns when no client is connected, or when ctx ends. It polls
// the client count every interval.
func (s *GameServer) WaitDrained(ctx context.Context, interval time.Duration) int {
	ticker := time.NewTicker(interval)
	defer ticker.Stop()
	for {
		remaining := s.GetConnectedClientsCount()
		if remaining == 0 {
			return 0
		}
		select {
		case <-ctx.Done():
			return remaining
		case <-ticker.C:
		}
	}
}
