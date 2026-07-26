package httpserver

import (
	"net/http"
	"time"

	"cyberia-server/httpserver/problem"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
	"github.com/go-chi/cors"
)

// RouterOptions configures the generic transport router built by NewRouter.
type RouterOptions struct {
	// AllowedOrigins is the CORS allow-list. This package applies no
	// default — callers resolve their own.
	AllowedOrigins []string
	// RequestTimeout caps handler duration; ≤0 selects 30s.
	RequestTimeout time.Duration
}

// NewRouter returns a chi.Router pre-wired with the transport middleware
// stack (request-id, real-ip, structured logging, panic→problem+json,
// timeout), CORS, and a RFC 9457 405 handler. Callers mount their domain
// routes onto the returned router. Unmatched-route presentation is owned by
// Envoy, not by the application.
func NewRouter(opts RouterOptions) chi.Router {
	if opts.RequestTimeout <= 0 {
		opts.RequestTimeout = 30 * time.Second
	}

	r := chi.NewRouter()

	// Order matters: RequestID first so every downstream log line carries
	// it; the panic net sits before Timeout so it wraps handler execution.
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

	// Do not install an application-level 404 handler. Envoy is responsible for
	// custom status pages and catches unmatched public routes at the edge.
	r.MethodNotAllowed(methodNotAllowedHandler)

	return r
}

// methodNotAllowedHandler forwards chi's computed Allow header in an RFC
// 9457 envelope.
func methodNotAllowedHandler(w http.ResponseWriter, r *http.Request) {
	problem.Write(w, r, problem.MethodNotAllowed(r.Method+" is not supported on "+r.URL.Path))
}

// problemRecoverer converts a panic in a downstream handler into a 500
// problem+json response. The response is committed once written, so the
// panic is swallowed rather than re-raised.
func problemRecoverer(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		defer func() {
			if rec := recover(); rec != nil {
				problem.Write(w, r, problem.InternalServerError("an unexpected error occurred"))
			}
		}()
		next.ServeHTTP(w, r)
	})
}
