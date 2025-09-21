package main

import (
	"context"
	"fmt"
	"log"
	"math/rand"
	"net/http"
	"os"
	"time"

	api "cyberia-server/api"
	game "cyberia-server/src"

	"github.com/go-chi/chi/v5"
	"go.mongodb.org/mongo-driver/bson"
)

func loadObjectLayers(db *api.DB) (map[string]*game.ObjectLayer, error) {
	cache := make(map[string]*game.ObjectLayer)
	col := db.Collection("objectlayers")
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()

	cursor, err := col.Find(ctx, bson.M{})
	if err != nil {
		return nil, fmt.Errorf("failed to query object layers from MongoDB: %w", err)
	}
	defer cursor.Close(ctx)

	for cursor.Next(ctx) {
		var layer game.ObjectLayer
		if err := cursor.Decode(&layer); err != nil {
			log.Printf("Error decoding object layer: %v", err)
			continue
		}
		cache[layer.Data.Item.ID] = &layer
	}

	if err := cursor.Err(); err != nil {
		return nil, fmt.Errorf("cursor error while loading object layers: %w", err)
	}

	return cache, nil
}

func main() {
	rand.Seed(time.Now().UnixNano())

	// API setup
	cfg := api.LoadConfig()
	db, err := api.ConnectMongo(context.Background(), cfg)
	if err != nil {
		log.Fatalf("mongo connect error: %v", err)
	}
	itemTypes := []string{"skin", "floor", "weapon", "skill", "coin"}

	// Load all object layers into a cache
	objectLayerCache, err := loadObjectLayers(db)
	if err != nil {
		log.Fatalf("Failed to load object layers: %v", err)
	}

	// Core game server
	s := game.NewGameServer()
	s.SetObjectLayerCache(objectLayerCache)
	go s.Run()

	// Check object layers id by type
	for _, itemType := range itemTypes {
		if err := api.CheckObjectLayersByType(context.Background(), db, itemType); err != nil {
			log.Printf("%s object layers check error: %v", itemType, err)
		}
	}

	// Set default admin
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
