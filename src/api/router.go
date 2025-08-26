package api

import (
	"net/http"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
)

// NewAPIRouter builds the /api router with middlewares and routes.
func NewAPIRouter(cfg Config, db *DB) chi.Router {
	r := chi.NewRouter()

	// Middlewares
	r.Use(middleware.RequestID)
	r.Use(middleware.RealIP)
	r.Use(middleware.Logger)
	r.Use(middleware.Recoverer)

	// Health
	r.Get("/health", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(http.StatusOK)
		w.Write([]byte(`{"status":"ok"}`))
	})

	// Object layers and users
	ol := NewObjectLayerHandler(cfg, db)
	uh := NewUserHandler(cfg, db)
	r.Route("/v1", func(sub chi.Router) {
		ol.Routes(sub)
		uh.Routes(sub)
	})

	return r
}
