package hotreload

import (
	"context"
	"log"
	"time"
)

// Passive reload scheduling lives here so every reload path — the interval
// poll, the gRPC control service, and the REST fallback — funnels through the
// same Service.run() single-flight core. The world-rebuild work itself stays
// in engine_client (WorldBuilder.HotReload / ReloadWorld); this file owns only
// the schedule.

// LoopTimeout bounds one scheduled reload so a stalled Engine cannot wedge the
// ticker goroutine forever.
const LoopTimeout = 2 * time.Minute

// Loop polls the Engine on a fixed interval, patching whatever changed.
type Loop struct {
	svc      *Service
	interval time.Duration
	stopCh   chan struct{}
}

// StartLoop begins passive reload polling every `interval`
// (ENGINE_GRPC_RELOAD_INTERVAL_SEC). A non-positive interval disables polling
// and returns a Loop whose Stop is a no-op, so callers need no branch.
//
// Polling uses ModeIncremental: it diffs the ObjectLayer manifest and patches
// only what changed. Full world rebuilds stay on-demand (the control trigger),
// since they churn every map entity.
func StartLoop(svc *Service, interval time.Duration) *Loop {
	l := &Loop{svc: svc, interval: interval, stopCh: make(chan struct{})}
	if svc == nil || interval <= 0 {
		log.Println("[HotReload] Passive reload loop disabled (interval <= 0).")
		return l
	}

	log.Printf("[HotReload] Passive reload loop every %s", interval)
	go func() {
		ticker := time.NewTicker(interval)
		defer ticker.Stop()
		for {
			select {
			case <-l.stopCh:
				log.Println("[HotReload] Passive reload loop stopped.")
				return
			case <-ticker.C:
				l.tick()
			}
		}
	}()
	return l
}

func (l *Loop) tick() {
	ctx, cancel := context.WithTimeout(context.Background(), LoopTimeout)
	defer cancel()

	// ErrBusy is expected whenever a triggered reload is mid-flight; the next
	// tick picks the work up, so it is not an error worth surfacing.
	if _, err := l.svc.run(ctx, ModeIncremental, "poll"); err != nil && err != ErrBusy {
		log.Printf("[HotReload] Passive reload error: %v", err)
	}
}

// Stop terminates the loop. Safe to call twice and on a disabled loop.
func (l *Loop) Stop() {
	if l == nil {
		return
	}
	select {
	case <-l.stopCh:
		// already stopped
	default:
		close(l.stopCh)
	}
}
