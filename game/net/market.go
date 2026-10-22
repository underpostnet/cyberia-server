// Package net — market.go
//
// The market broker: it reads market views from the engine off the tick and
// hands each one to the simulation, which keeps it and sends it.
package net

import (
	"context"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/logx"
)

// marketCallTimeout bounds one market view call to the engine.
const marketCallTimeout = 5 * time.Second

// ViewMarket implements content.MarketBroker.
func (s *GameServer) ViewMarket(q content.MarketQuery) {
	go func() {
		ctx, cancel := context.WithTimeout(context.Background(), marketCallTimeout)
		view, err := s.accounts.ListMarket(ctx, s.InstanceCode, q)
		cancel()
		s.Mu.Lock()
		defer s.Mu.Unlock()
		if err != nil {
			logx.Warnf("[Market] view of %s for player=%s failed: %v", q.Code, q.Profile, err)
			s.MarketUnavailable(q)
			return
		}
		s.ShowMarket(q, view, time.Now())
	}()
}
