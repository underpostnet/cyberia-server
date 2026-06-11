package api

import (
	"net/http"
	"time"

	"cyberia-server/config"
	game "cyberia-server/game"
	"cyberia-server/httpserver"
	"cyberia-server/httpserver/problem"

	"github.com/go-chi/chi/v5"
)

// RouterOptions configures the top-level /api router.
//
// Zero values yield sane defaults: CORS open to localhost during dev,
// 30-second request timeout, no instance metadata. Production callers
// pass an InstanceCode and an explicit AllowedOrigins list (typically
// from CYBERIA_CORS_ALLOWED_ORIGINS).
type RouterOptions struct {
	GameServer     *game.GameServer
	InstanceCode   string
	AllowedOrigins []string
	RequestTimeout time.Duration
}

// NewAPIRouter builds the /api router: the generic transport stack from
// httpserver (middleware, CORS, RFC 9457 404/405) plus the domain /v1
// subtree (health, readiness, metrics, docs). Mount the result at /api.
func NewAPIRouter(opts RouterOptions) chi.Router {
	allowedOrigins := opts.AllowedOrigins
	if len(allowedOrigins) == 0 {
		allowedOrigins = config.DefaultCORSOrigins()
	}

	r := httpserver.NewRouter(httpserver.RouterOptions{
		AllowedOrigins: allowedOrigins,
		RequestTimeout: opts.RequestTimeout,
	})

	mh := NewMetricsHandler(opts.GameServer, opts.InstanceCode)
	dh := NewDocsHandler()

	r.Route("/v1", func(sub chi.Router) {
		// Liveness — answers as long as the HTTP server is up. Distinct
		// from /metrics/health, which encodes domain health.
		sub.Get("/health", livenessHandler)
		// Readiness — fails until the GameServer has loaded its world.
		sub.Get("/health/ready", readinessHandlerFor(opts.GameServer))

		mh.Routes(sub)
		dh.Routes(sub)
	})

	return r
}

// Mount attaches the /api subtree to r. The prefix lives here so callers
// don't hardcode it.
func Mount(r chi.Router, opts RouterOptions) {
	r.Mount("/api", NewAPIRouter(opts))
}

// livenessHandler — minimal "I can serve traffic" response, kept decoupled
// from world state so it stays green during gRPC reload bursts.
func livenessHandler(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, map[string]any{
		"status":    "ok",
		"timestamp": time.Now().UTC(),
	})
}

// readinessHandlerFor reports 503 with a problem+json envelope until the
// GameServer has at least one map loaded.
func readinessHandlerFor(gs *game.GameServer) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		if gs == nil || !gs.HasMaps() {
			problem.Write(w, r, problem.ServiceUnavailable("game world not yet loaded"))
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{
			"status":    "ready",
			"timestamp": time.Now().UTC(),
		})
	}
}
