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
)

func main() {
	rand.Seed(time.Now().UnixNano())

	// Core game server
	s := game.NewGameServer()

	// TODO: www.cyberiaonline.com object layer api integration
	// s.SetObjectLayerCache()

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
