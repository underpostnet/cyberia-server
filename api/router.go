package api

import (
	"net/http"

	game "cyberia-server/src"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
)

// NewAPIRouter builds the /api router with middlewares and routes.
func NewAPIRouter(cfg Config, db *DB, gameServer *game.GameServer) chi.Router {
	r := chi.NewRouter()

	// Middlewares
	r.Use(middleware.RequestID)
	r.Use(middleware.RealIP)
	r.Use(middleware.Logger)
	r.Use(middleware.Recoverer)

	// Object layers, users, and metrics
	ol := NewObjectLayerHandler(cfg, db)
	uh := NewUserHandler(cfg, db)
	mh := NewMetricsHandler(cfg, db, gameServer)
	r.Route("/v1", func(sub chi.Router) {
		// Health
		sub.Get("/health", func(w http.ResponseWriter, r *http.Request) {
			w.Header().Set("Content-Type", "application/json")
			w.WriteHeader(http.StatusOK)
			w.Write([]byte(`{"status":"ok"}`))
		})
		ol.Routes(sub)
		uh.Routes(sub)
		mh.Routes(sub)
	})

	return r
}
