package main

import (
	"log"
	"math/rand"
	"net/http"
	"os"
	"time"

	api "cyberia-server/api"
	game "cyberia-server/src"

	"github.com/go-chi/chi/v5"
	"github.com/joho/godotenv"
)

func main() {
	rand.Seed(time.Now().UnixNano())

	// Load .env file if present (does not override already-set env vars)
	if err := godotenv.Load(); err != nil {
		log.Println("No .env file found or failed to load, relying on environment variables.")
	}

	// Core game server
	s := game.NewGameServer()

	// Authenticate with www.cyberiaonline.com and fetch object layer metadata into cache.
	// Requires CYBERIA_API_EMAIL and CYBERIA_API_PASSWORD env vars.
	s.SetObjectLayerCache()

	go s.Run()

	r := chi.NewRouter()

	// Use environment variable `STATIC_DIR` or default to `./` for static files.
	// This serves the static frontend application.
	staticDir := os.Getenv("STATIC_DIR")
	if staticDir == "" {
		staticDir = "/home/dd/cyberia-server/public"
	}
	r.Handle("/*", game.StaticFileServer(staticDir, "/index.html"))

	// Mount REST API under /api
	r.Mount("/api", api.NewAPIRouter(s))
	// Keep websocket endpoint
	r.HandleFunc("/ws", s.HandleConnections)

	srv := &http.Server{
		Addr:    ":8080",
		Handler: r,
	}

	log.Println("Server started on :8080")
	if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		log.Fatal("ListenAndServe:", err)
	}
}
