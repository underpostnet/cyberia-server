package api

import (
	"net/http"

	game "cyberia-server/src"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
	"github.com/go-chi/cors"
)

// NewAPIRouter builds the /api router with middlewares and routes.
func NewAPIRouter(cfg Config, db *DB, gameServer *game.GameServer) chi.Router {
	r := chi.NewRouter()

	// Middlewares
	r.Use(middleware.RequestID)
	r.Use(middleware.RealIP)
	r.Use(middleware.Logger)
	r.Use(middleware.Recoverer)

	// CORS middleware
	r.Use(cors.Handler(cors.Options{
		AllowedOrigins:   []string{"http://localhost:8081", "http://localhost:*"},
		AllowedMethods:   []string{"GET", "POST", "PUT", "DELETE", "OPTIONS"},
		AllowedHeaders:   []string{"Accept", "Authorization", "Content-Type", "X-CSRF-Token"},
		ExposedHeaders:   []string{"Link"},
		AllowCredentials: true,
		MaxAge:           300,
	}))

	// Object layers, users, and metrics
	ol := NewObjectLayerHandler(cfg, db, gameServer)
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
