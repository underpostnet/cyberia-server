package api

import (
	"net/http"
	"time"

	"cyberia-server/api/problem"
	"cyberia-server/config"
	game "cyberia-server/game"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
	"github.com/go-chi/cors"
)

// RouterOptions configures the top-level /api router.
//
// Zero values yield sane defaults: CORS open to localhost during dev,
// 30-second request timeout, no instance metadata. Production callers
// should pass an InstanceCode and an explicit AllowedOrigins list
// (typically derived from `CYBERIA_CORS_ALLOWED_ORIGINS`).
type RouterOptions struct {
	GameServer     *game.GameServer
	InstanceCode   string
	AllowedOrigins []string
	RequestTimeout time.Duration
}

// NewAPIRouter builds the /api router. It owns:
//
//   - request-id / real-ip / structured logger / panic-recover middleware
//   - CORS (env-driven)
//   - 404 + 405 handlers that emit RFC 9457 problem+json
//   - the /v1 subtree (health, metrics, openapi, docs, postman)
//
// Callers mount the returned router at /api on their top-level chi.Router.
func NewAPIRouter(opts RouterOptions) chi.Router {
	if opts.RequestTimeout <= 0 {
		opts.RequestTimeout = 30 * time.Second
	}
	if len(opts.AllowedOrigins) == 0 {
		opts.AllowedOrigins = config.DefaultCORSOrigins()
	}

	r := chi.NewRouter()

	// Order matters: RequestID first so every downstream log line carries it;
	// Recoverer last so it sees the request id in the log line it emits.
	r.Use(middleware.RequestID)
	r.Use(middleware.RealIP)
	r.Use(middleware.Logger)
	r.Use(problemRecoverer)
	r.Use(middleware.Timeout(opts.RequestTimeout))

	r.Use(cors.Handler(cors.Options{
		AllowedOrigins:   opts.AllowedOrigins,
		AllowedMethods:   []string{http.MethodGet, http.MethodHead, http.MethodOptions},
		AllowedHeaders:   []string{"Accept", "Authorization", "Content-Type", "X-Request-Id"},
		ExposedHeaders:   []string{"Link", "X-Request-Id"},
		AllowCredentials: false,
		MaxAge:           300,
	}))

	// Global 404 / 405 — must speak problem+json so clients see a
	// consistent error shape regardless of which layer rejected them.
	r.NotFound(notFoundHandler)
	r.MethodNotAllowed(methodNotAllowedHandler)

	mh := NewMetricsHandler(opts.GameServer, opts.InstanceCode)
	dh := NewDocsHandler()

	r.Route("/v1", func(sub chi.Router) {
		// Liveness — answers as long as the HTTP server is up.
		// Distinct from /metrics/health which encodes domain health.
		sub.Get("/health", livenessHandler)
		// Readiness — fails until the GameServer has loaded its world.
		sub.Get("/health/ready", readinessHandlerFor(opts.GameServer))

		mh.Routes(sub)
		dh.Routes(sub)
	})

	return r
}

// livenessHandler — minimal, JSON-shaped "I can serve traffic" response.
// Kept deliberately decoupled from world state so it stays green during
// gRPC reload bursts.
func livenessHandler(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, map[string]any{
		"status":    "ok",
		"timestamp": time.Now().UTC(),
	})
}

// readinessHandlerFor reports 503 with a problem+json envelope until the
// GameServer has at least one map loaded (the post-gRPC LoadAll signal).
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

// notFoundHandler emits the RFC 9457 envelope for unknown routes under /api.
func notFoundHandler(w http.ResponseWriter, r *http.Request) {
	problem.Write(w, r, problem.NotFound("no route matches "+r.Method+" "+r.URL.Path))
}

// methodNotAllowedHandler — chi sets the Allow header before invoking
// this; we forward whatever it computed so the response is RFC-correct.
func methodNotAllowedHandler(w http.ResponseWriter, r *http.Request) {
	problem.Write(w, r, problem.MethodNotAllowed(r.Method+" is not supported on "+r.URL.Path))
}

// problemRecoverer is a panic-safety net that emits a 500 problem+json
// instead of letting middleware.Recoverer write a plain-text "500
// Internal Server Error". Defers to middleware.Recoverer-style logging
// by re-panicking after writing the response would be unsafe — the
// response is already committed — so we log inline.
func problemRecoverer(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		defer func() {
			if rec := recover(); rec != nil {
				// Best-effort detail. The full stack is logged by
				// chi's standard recoverer in production; we don't
				// repeat it in the response body.
				problem.Write(w, r, problem.InternalServerError("an unexpected error occurred"))
			}
		}()
		next.ServeHTTP(w, r)
	})
}
