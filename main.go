package main

import (
    "context"
    "log"
    "math/rand"
    "net/http"
    "time"

    api "cyberia-server/src/api"
    game "cyberia-server/src"

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
    if err := api.SeedDefaultAdmin(context.Background(), cfg, db); err != nil {
        log.Printf("admin seed error: %v", err)
    }

    r := chi.NewRouter()
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
