package main

import (
	"context"
	"log"
	"math/rand"
	"net/http"
	"os"
	"time"

	game "cyberia-server/src"
	api "cyberia-server/src/api"

	"github.com/go-chi/chi/v5"
)

func main() {
	rand.Seed(time.Now().UnixNano())

	// Core game server
	s := game.NewGameServer()
	go s.Run()

	// API setup
	cfg := api.LoadConfig()
	db, err := api.ConnectMongo(context.Background(), cfg)
	if err != nil {
		log.Fatalf("mongo connect error: %v", err)
	}
	// Seed default data
	if err := api.SeedObjectLayers(context.Background(), db); err != nil {
		log.Printf("object layers seed error: %v", err)
	}
	if err := api.SeedDefaultAdmin(context.Background(), cfg, db); err != nil {
		log.Printf("admin seed error: %v", err)
	}

	r := chi.NewRouter()

	// Use environment variable `STATIC_DIR` or default to `./` for static files.
	// This serves the static frontend application.
	staticDir := os.Getenv("STATIC_DIR")
	if staticDir == "" {
		staticDir = "/home/dd/engine/src/client/public/cyberia"
	}
	r.Handle("/*", game.StaticFileServer(staticDir, "/index.html"))

	// Mount REST API under /api
	r.Mount("/api", api.NewAPIRouter(cfg, db))
	// Keep websocket endpoint
	r.HandleFunc("/ws", s.HandleConnections)

	srv := &http.Server{
		Addr:         ":8080",
		Handler:      r,
		ReadTimeout:  cfg.ReadTimeout,
		WriteTimeout: cfg.WriteTimeout,
	}

	log.Println("Server started on :8080")
	if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		log.Fatal("ListenAndServe:", err)
	}
}
