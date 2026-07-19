// Package hotreload owns WHEN the world reloads. Every path funnels through
// one single-flight core (Service.run):
//
//	loop.go — passive polling on ENGINE_GRPC_RELOAD_INTERVAL_SEC.
//	grpc.go — on-demand control service on its own port. Preferred trigger.
//	rest.go — POST /api/v1/hot-reload. Fallback when gRPC is unreachable.
//
// It owns no transport or rebuild logic of its own: the Engine fetches and the
// world rebuild live in engine_client (WorldBuilder), reached through the
// Reloader interface. Keeping the two apart means engine_client stays "how to
// talk to the Engine" and this package stays "when to reload".
//
// AUTHORIZATION
// -------------
// The trigger is INTERNAL: only engine-cyberia may call it. Both transports
// require the shared secret CYBERIA_SERVER_API_KEY, compared in constant time.
// An unset key disables the trigger entirely (fail closed) — a deploy that
// forgets to set it cannot accidentally expose a public reload endpoint.
package hotreload

import (
	"context"
	"crypto/subtle"
	"errors"
	"log"
	"sync"
	"time"
)

// Reloader is the world-rebuild surface Trigger drives — implemented by
// engine_client.WorldBuilder.
type Reloader interface {
	// ReloadWorld re-fetches the full instance and rebuilds when its version
	// changed, preserving connected players.
	ReloadWorld(ctx context.Context) error
	// HotReload diffs the ObjectLayer manifest and patches only what changed.
	HotReload(ctx context.Context) error
}

// Mode selects how much of the world a trigger rebuilds.
type Mode string

const (
	// ModeFull re-fetches the instance and rebuilds maps/entities when the
	// instance version changed.
	ModeFull Mode = "full"
	// ModeIncremental only diffs and patches the ObjectLayer cache.
	ModeIncremental Mode = "incremental"
)

// ErrUnauthorized is returned when the caller's key does not match, or when
// no key is configured (the trigger is disabled).
var ErrUnauthorized = errors.New("unauthorized hot-reload trigger")

// ErrBusy is returned when a reload is already running.
var ErrBusy = errors.New("hot reload already in progress")

// Result reports what a trigger did; both transports serialize it verbatim.
type Result struct {
	OK           bool   `json:"ok"`
	Mode         string `json:"mode"`
	InstanceCode string `json:"instanceCode"`
	DurationMS   int64  `json:"durationMs"`
	Message      string `json:"message"`
}

// Service authorizes and serializes hot-reload triggers.
type Service struct {
	reloader     Reloader
	apiKey       string
	instanceCode string

	// One reload at a time: concurrent rebuilds would race on world state.
	mu      sync.Mutex
	running bool
}

// NewService builds the trigger core. An empty apiKey leaves the trigger
// disabled — Authorize then rejects every call.
func NewService(reloader Reloader, apiKey, instanceCode string) *Service {
	if apiKey == "" {
		log.Println("[HotReload] CYBERIA_SERVER_API_KEY unset — trigger disabled (fail closed).")
	}
	return &Service{reloader: reloader, apiKey: apiKey, instanceCode: instanceCode}
}

// Enabled reports whether a key is configured.
func (s *Service) Enabled() bool { return s != nil && s.apiKey != "" }

// Authorize compares the presented key in constant time.
func (s *Service) Authorize(key string) error {
	if !s.Enabled() {
		return ErrUnauthorized
	}
	if 1 != subtle.ConstantTimeCompare([]byte(key), []byte(s.apiKey)) {
		return ErrUnauthorized
	}
	return nil
}

// InstanceCode is the instance this server serves; triggers naming a different
// instance are rejected by Trigger.
func (s *Service) InstanceCode() string { return s.instanceCode }

// Trigger authorizes, then runs the reload for `mode`. `instanceCode` may be
// empty (defaults to this server's instance); a mismatch is an error so a
// misrouted trigger never silently rebuilds the wrong world.
//
// Entry point for the EXTERNAL transports (gRPC control service, REST). The
// passive interval loop calls run() directly — it is already internal, so it
// carries no key, but it shares run()'s single-flight guard so a scheduled
// reload can never race a triggered one.
func (s *Service) Trigger(ctx context.Context, key string, mode Mode, instanceCode string) (Result, error) {
	if err := s.Authorize(key); err != nil {
		return Result{}, err
	}
	if instanceCode != "" && s.instanceCode != "" && instanceCode != s.instanceCode {
		return Result{}, errors.New("instance code mismatch: this server serves " + s.instanceCode)
	}
	return s.run(ctx, mode, "trigger")
}

// run executes one reload. Single-flight: concurrent rebuilds would race on
// world state, so a second caller gets ErrBusy rather than queueing.
func (s *Service) run(ctx context.Context, mode Mode, source string) (Result, error) {
	if mode != ModeIncremental {
		mode = ModeFull
	}

	s.mu.Lock()
	if s.running {
		s.mu.Unlock()
		return Result{}, ErrBusy
	}
	s.running = true
	s.mu.Unlock()
	defer func() {
		s.mu.Lock()
		s.running = false
		s.mu.Unlock()
	}()

	log.Printf("[HotReload] %s accepted (mode=%s instance=%s)", source, mode, s.instanceCode)
	start := time.Now()

	var err error
	if mode == ModeIncremental {
		err = s.reloader.HotReload(ctx)
	} else {
		err = s.reloader.ReloadWorld(ctx)
	}
	elapsed := time.Since(start)

	if err != nil {
		log.Printf("[HotReload] %s failed after %s: %v", source, elapsed, err)
		return Result{
			Mode:         string(mode),
			InstanceCode: s.instanceCode,
			DurationMS:   elapsed.Milliseconds(),
			Message:      err.Error(),
		}, err
	}

	log.Printf("[HotReload] %s complete in %s (mode=%s)", source, elapsed, mode)
	return Result{
		OK:           true,
		Mode:         string(mode),
		InstanceCode: s.instanceCode,
		DurationMS:   elapsed.Milliseconds(),
		Message:      "world reloaded",
	}, nil
}
