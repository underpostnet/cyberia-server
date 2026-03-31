// Package grpcclient — world_builder.go
//
// WorldBuilder uses the gRPC client to populate the GameServer's
// objectLayerDataCache at startup and supports incremental hot-reload
// by diffing sha256 manifests and surgically replacing stale entries.
package grpcclient

import (
	"context"
	"log"
	"sync"
	"time"

	game "cyberia-server/src"
)

// WorldBuilder orchestrates data loading from the Engine gRPC server
// into the GameServer's in-memory cache.
type WorldBuilder struct {
	mu     sync.Mutex
	client *Client
	server *game.GameServer

	// InstanceCode is the INSTANCE_CODE to query from the Engine.
	InstanceCode string

	// manifest tracks the last-known sha256 per item ID for diffing.
	manifest map[string]string

	// ReloadInterval is the polling interval for hot-reload (0 = disabled).
	ReloadInterval time.Duration

	stopCh chan struct{}
}

// NewWorldBuilder creates a WorldBuilder bound to the given client + server.
func NewWorldBuilder(client *Client, server *game.GameServer) *WorldBuilder {
	return &WorldBuilder{
		client:         client,
		server:         server,
		manifest:       make(map[string]string),
		ReloadInterval: 0,
		stopCh:         make(chan struct{}),
	}
}

// LoadAll performs the initial full load via gRPC GetFullInstance.
// It fetches the instance graph, all maps, entities, and object layers,
// then builds the game world and populates the object layer cache.
func (wb *WorldBuilder) LoadAll(ctx context.Context) error {
	if wb.InstanceCode == "" {
		log.Println("[WorldBuilder] No INSTANCE_CODE set, falling back to ObjectLayerBatch-only load.")
		return wb.loadObjectLayersOnly(ctx)
	}

	log.Printf("[WorldBuilder] Loading full instance %q via gRPC...", wb.InstanceCode)

	resp, err := wb.client.FetchFullInstance(ctx, wb.InstanceCode)
	if err != nil {
		return err
	}

	// Apply instance config to GameServer (must happen before building world)
	if cfg := resp.GetConfig(); cfg != nil {
		wb.server.ApplyInstanceConfig(cfg)
		log.Println("[WorldBuilder] Applied instance config from gRPC.")
	} else {
		log.Println("[WorldBuilder] WARNING: no instance config in gRPC response.")
	}

	// Build object layer cache from the response
	cache := make(map[string]*game.ObjectLayer, len(resp.GetObjectLayers()))
	for _, olMsg := range resp.GetObjectLayers() {
		ol := protoToObjectLayer(olMsg)
		if ol.Data.Item.ID != "" {
			cache[ol.Data.Item.ID] = ol
		}
	}

	// Also fetch all remaining ObjectLayers (items not referenced by this instance
	// but still needed for stats calculation, e.g. weapon items on players)
	allCache, err := wb.client.FetchObjectLayerBatch(ctx, "")
	if err != nil {
		log.Printf("[WorldBuilder] WARNING: ObjectLayerBatch failed: %v — using instance OLs only.", err)
	} else {
		for itemID, ol := range allCache {
			cache[itemID] = ol
		}
	}

	// Push OL cache into GameServer FIRST (needed for bot behavior detection)
	wb.server.ReplaceObjectLayerCache(cache)

	// Build manifest
	wb.mu.Lock()
	wb.manifest = make(map[string]string, len(cache))
	for itemID, ol := range cache {
		wb.manifest[itemID] = ol.Sha256
	}
	wb.mu.Unlock()

	// Build the game world from instance data
	wb.server.BuildWorldFromInstance(resp.GetInstance(), resp.GetMaps(), resp.GetObjectLayers())

	// Fetch all atlas sprite sheets via gRPC batch
	atlasCache, err := wb.client.FetchAtlasSpriteSheetBatch(ctx)
	if err != nil {
		log.Printf("[WorldBuilder] WARNING: AtlasSpriteSheetBatch failed: %v — atlas metadata unavailable.", err)
	} else {
		wb.server.ReplaceAtlasCache(atlasCache)
	}

	log.Printf("[WorldBuilder] Full instance load complete: %d ObjectLayers, %d AtlasSheets cached.",
		len(cache), len(atlasCache))
	return nil
}

// loadObjectLayersOnly is the fallback when no INSTANCE_CODE is set.
// It only loads the object layer cache without building maps.
func (wb *WorldBuilder) loadObjectLayersOnly(ctx context.Context) error {
	log.Println("[WorldBuilder] Starting ObjectLayer-only load via gRPC...")

	cache, err := wb.client.FetchObjectLayerBatch(ctx, "")
	if err != nil {
		return err
	}

	wb.mu.Lock()
	wb.manifest = make(map[string]string, len(cache))
	for itemID, ol := range cache {
		wb.manifest[itemID] = ol.Sha256
	}
	wb.mu.Unlock()

	wb.server.ReplaceObjectLayerCache(cache)

	// Also fetch atlas sprite sheets
	atlasCache, err := wb.client.FetchAtlasSpriteSheetBatch(ctx)
	if err != nil {
		log.Printf("[WorldBuilder] WARNING: AtlasSpriteSheetBatch failed: %v", err)
	} else {
		wb.server.ReplaceAtlasCache(atlasCache)
	}

	log.Printf("[WorldBuilder] ObjectLayer-only load complete: %d OLs, %d atlases cached.",
		len(cache), len(atlasCache))
	return nil
}

// HotReload fetches the manifest from Engine, diffs against the local
// copy, and surgically re-fetches only the changed/new ObjectLayers.
// Deleted items are removed from cache.
func (wb *WorldBuilder) HotReload(ctx context.Context) error {
	// 1. Ping Engine to confirm liveness
	_, err := wb.client.Ping(ctx)
	if err != nil {
		return err
	}

	// 2. Fetch remote manifest
	remoteEntries, err := wb.client.FetchObjectLayerManifest(ctx)
	if err != nil {
		return err
	}

	remoteMap := make(map[string]string, len(remoteEntries))
	for _, e := range remoteEntries {
		remoteMap[e.ItemID] = e.Sha256
	}

	wb.mu.Lock()
	localManifest := wb.manifest
	wb.mu.Unlock()

	// 3. Diff: find new/changed items
	var toFetch []string
	for itemID, remoteSha := range remoteMap {
		localSha, exists := localManifest[itemID]
		if !exists || localSha != remoteSha {
			toFetch = append(toFetch, itemID)
		}
	}

	// 4. Diff: find deleted items
	var toDelete []string
	for itemID := range localManifest {
		if _, exists := remoteMap[itemID]; !exists {
			toDelete = append(toDelete, itemID)
		}
	}

	if len(toFetch) == 0 && len(toDelete) == 0 {
		log.Println("[WorldBuilder] Hot-reload: no changes detected.")
		return nil
	}

	log.Printf("[WorldBuilder] Hot-reload: %d changed/new, %d deleted.", len(toFetch), len(toDelete))

	// 5. Fetch changed ObjectLayers one by one
	updates := make(map[string]*game.ObjectLayer, len(toFetch))
	fetchErrors := 0
	for _, itemID := range toFetch {
		ol, err := wb.client.FetchObjectLayer(ctx, itemID)
		if err != nil {
			log.Printf("[WorldBuilder] Warning: failed to fetch ObjectLayer %s: %v", itemID, err)
			fetchErrors++
			continue
		}
		updates[itemID] = ol
	}

	// 6. Apply changes atomically to GameServer
	wb.server.PatchObjectLayerCache(updates, toDelete)

	// 7. Update local manifest
	wb.mu.Lock()
	for itemID, ol := range updates {
		wb.manifest[itemID] = ol.Sha256
	}
	for _, itemID := range toDelete {
		delete(wb.manifest, itemID)
	}
	wb.mu.Unlock()

	log.Printf("[WorldBuilder] Hot-reload applied: %d updated, %d deleted, %d errors.",
		len(updates), len(toDelete), fetchErrors)
	return nil
}

// StartReloadLoop runs HotReload on the configured interval in a goroutine.
// Call Stop() to terminate.
func (wb *WorldBuilder) StartReloadLoop() {
	if wb.ReloadInterval <= 0 {
		log.Println("[WorldBuilder] Hot-reload loop disabled (ReloadInterval <= 0).")
		return
	}
	log.Printf("[WorldBuilder] Starting hot-reload loop every %s", wb.ReloadInterval)
	go func() {
		ticker := time.NewTicker(wb.ReloadInterval)
		defer ticker.Stop()
		for {
			select {
			case <-wb.stopCh:
				log.Println("[WorldBuilder] Hot-reload loop stopped.")
				return
			case <-ticker.C:
				ctx, cancel := context.WithTimeout(context.Background(), 2*time.Minute)
				if err := wb.HotReload(ctx); err != nil {
					log.Printf("[WorldBuilder] Hot-reload error: %v", err)
				}
				cancel()
			}
		}
	}()
}

// Stop terminates the hot-reload loop.
func (wb *WorldBuilder) Stop() {
	select {
	case <-wb.stopCh:
		// already closed
	default:
		close(wb.stopCh)
	}
}
