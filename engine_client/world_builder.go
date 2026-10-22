// Package engine_client — world_builder.go
//
// WorldBuilder uses an engine DataSource (gRPC primary or REST boot fallback)
// to warm the GameServer object layer cache with the definitions its world
// names. A reload compares the cached labels with the engine catalog and
// replaces only the stale ones; it never adds a label the world does not name.
package engine_client

import (
	"context"
	"fmt"
	"sync"

	"cyberia-server/game/content"
	"cyberia-server/logx"

	"cyberia-server/game/net"
	pb "cyberia-server/gen/proto"
)

// WorldBuilder orchestrates data loading from an engine DataSource
// into the GameServer's in-memory cache.
type WorldBuilder struct {
	mu     sync.Mutex
	client DataSource
	server *net.GameServer

	// InstanceCode is the INSTANCE_CODE to query from the Engine.
	InstanceCode string

	// instanceName is the name of the loaded instance.
	instanceName string

	// lastInstanceVersion is the opaque version string returned by
	// GetFullInstance.  ReloadWorld compares it to skip full rebuilds
	// when the instance, its maps, and its config have not changed.
	lastInstanceVersion string

	// releaseID is the content release the engine served the last full load from.
	releaseID string
}

// NewWorldBuilder creates a WorldBuilder bound to the given data source + server.
func NewWorldBuilder(client DataSource, server *net.GameServer) *WorldBuilder {
	return &WorldBuilder{client: client, server: server}
}

// LoadAll performs the initial full load via GetFullInstance.
// It fetches the instance graph, all maps, entities, and object layers,
// then builds the game world and populates the object layer cache.
// INSTANCE_CODE is required — there is no fallback instance.
func (wb *WorldBuilder) LoadAll(ctx context.Context) error {
	if wb.InstanceCode == "" {
		return fmt.Errorf("INSTANCE_CODE required: set it before LoadAll")
	}

	logx.Infof("[WorldBuilder] Loading full instance %q...", wb.InstanceCode)
	wb.server.SetInstanceCode(wb.InstanceCode)

	resp, err := wb.client.FetchFullInstance(ctx, wb.InstanceCode)
	if err != nil {
		return err
	}

	if err := validateWorldProgression(resp); err != nil {
		return err
	}

	// Apply instance config to GameServer (must happen before building world).
	// Config is required — a world without it is misconfigured.
	cfg := resp.GetConfig()
	if cfg == nil {
		return fmt.Errorf("instance %q returned no config", wb.InstanceCode)
	}
	if err := wb.server.ApplyInstanceConfig(cfg); err != nil {
		return err
	}
	logx.Infof("[WorldBuilder] Applied instance config.")

	// Build object layer cache exclusively from the instance response.
	// getFullInstance already includes OLs for all map entity items.
	cache := make(map[string]*content.ObjectLayer, len(resp.GetObjectLayers()))
	for _, olMsg := range resp.GetObjectLayers() {
		ol := protoToObjectLayer(olMsg)
		if ol.Data.Item.ID != "" {
			cache[ol.Data.Item.ID] = ol
		}
	}

	// Push OL cache into GameServer FIRST (needed for bot behavior detection)
	if err := wb.server.ReplaceObjectLayerCache(cache); err != nil {
		return err
	}

	// Save initial version for change-detection.
	wb.mu.Lock()
	wb.lastInstanceVersion = resp.GetVersion()
	wb.instanceName = resp.GetInstance().GetName()
	wb.releaseID = resp.GetReleaseId()
	wb.mu.Unlock()

	// Build the game world from instance data
	wb.server.BuildWorldFromInstance(resp.GetInstance(), resp.GetMaps(), resp.GetObjectLayers(),
		resp.GetActions(), resp.GetQuests(), resp.GetMarkets())

	logx.Infof("[WorldBuilder] Full instance load complete: %d ObjectLayers cached.", len(cache))
	return nil
}

// ReleaseID is the content release the engine served the last full load
// from, "" for the workspace.
func (wb *WorldBuilder) ReleaseID() string {
	wb.mu.Lock()
	defer wb.mu.Unlock()
	return wb.releaseID
}

// InstanceName is the name of the loaded instance, or its code when it has none.
func (wb *WorldBuilder) InstanceName() string {
	wb.mu.Lock()
	defer wb.mu.Unlock()
	if wb.instanceName == "" {
		return wb.InstanceCode
	}
	return wb.instanceName
}

// refreshLabels compares the cached labels with the engine catalog: a label
// bound to another definition is fetched again, an unbound one leaves the
// cache. It never adds a label the world does not hold.
func (wb *WorldBuilder) refreshLabels(ctx context.Context) error {
	remoteEntries, err := wb.client.FetchObjectLayerManifest(ctx)
	if err != nil {
		return err
	}
	remote := make(map[string]string, len(remoteEntries))
	for _, e := range remoteEntries {
		remote[e.ItemID] = e.Cid
	}

	var stale, unbound []string
	for itemID, cid := range wb.server.ObjectLayerManifest() {
		switch remoteCid, bound := remote[itemID]; {
		case !bound:
			unbound = append(unbound, itemID)
		case remoteCid != cid:
			stale = append(stale, itemID)
		}
	}
	if len(stale) == 0 && len(unbound) == 0 {
		return nil
	}
	logx.Infof("[WorldBuilder] Hot-reload: %d rebound, %d unbound.", len(stale), len(unbound))
	updates := make(map[string]*content.ObjectLayer, len(stale))
	fetchErrors := 0
	for _, itemID := range stale {
		ol, err := wb.client.FetchObjectLayer(ctx, itemID)
		if err != nil {
			logx.Warnf("[WorldBuilder] failed to fetch ObjectLayer %s: %v", itemID, err)
			fetchErrors++
			continue
		}
		updates[itemID] = ol
	}
	if err := wb.server.PatchObjectLayerCache(updates, unbound); err != nil {
		return err
	}
	logx.Infof("[WorldBuilder] Hot-reload applied: %d updated, %d deleted, %d errors.",
		len(updates), len(unbound), fetchErrors)
	return nil
}

// HotReload refreshes the cached definitions against the engine catalog and
// the resolved content, then evicts the resolved content nothing holds. The
// world stays as it is.
func (wb *WorldBuilder) HotReload(ctx context.Context) error {
	if _, err := wb.client.Ping(ctx); err != nil {
		return err
	}
	if err := wb.refreshLabels(ctx); err != nil {
		return err
	}
	wb.server.RefreshResolvedContent(ctx)
	wb.server.EvictResolvedContent()
	return nil
}

// ReloadWorld refreshes the cached definitions, re-fetches the full instance
// and, when the version string has changed, rebuilds all map entities while
// preserving connected players. When nothing changed the rebuild is skipped
// to avoid churn. Then it resolves the resolved content again from the release
// the engine serves, and evicts the resolved content nothing holds.
func (wb *WorldBuilder) ReloadWorld(ctx context.Context) error {
	if err := wb.refreshLabels(ctx); err != nil {
		return err
	}
	defer wb.server.EvictResolvedContent()
	resp, err := wb.client.FetchFullInstance(ctx, wb.InstanceCode)
	if err != nil {
		return err
	}

	if err := validateWorldProgression(resp); err != nil {
		return err
	}

	// Compare version — skip rebuild when instance+maps are unchanged.
	newVersion := resp.GetVersion()
	wb.mu.Lock()
	same := newVersion != "" && newVersion == wb.lastInstanceVersion
	wb.releaseID = resp.GetReleaseId()
	wb.mu.Unlock()

	if same {
		wb.server.RefreshResolvedContent(ctx)
		return nil
	}

	logx.Infof("[WorldBuilder] Instance version changed — rebuilding world...")

	// Re-apply config in case it changed.
	if cfg := resp.GetConfig(); cfg != nil {
		if err := wb.server.ApplyInstanceConfig(cfg); err != nil {
			return err
		}
	}

	warm := make(map[string]*content.ObjectLayer, len(resp.GetObjectLayers()))
	for _, msg := range resp.GetObjectLayers() {
		ol := protoToObjectLayer(msg)
		if ol.Data.Item.ID != "" {
			warm[ol.Data.Item.ID] = ol
		}
	}
	if err := wb.server.ReplaceWarmObjectLayers(warm); err != nil {
		return err
	}

	// Rebuild world preserving players.
	wb.server.RebuildWorld(resp.GetInstance(), resp.GetMaps(), resp.GetObjectLayers(),
		resp.GetActions(), resp.GetQuests(), resp.GetMarkets())

	wb.mu.Lock()
	wb.lastInstanceVersion = newVersion
	wb.instanceName = resp.GetInstance().GetName()
	wb.mu.Unlock()
	wb.server.RefreshResolvedContent(ctx)

	logx.Infof("[WorldBuilder] ReloadWorld complete — maps and entities refreshed.")
	return nil
}

func validateWorldProgression(response *pb.GetFullInstanceResponse) error {
	if err := content.ValidateWorldProgression(response.GetConfig(), response.GetMaps()); err != nil {
		return err
	}
	for _, message := range response.GetObjectLayers() {
		if err := protoToObjectLayer(message).Data.Stats.Validate(); err != nil {
			return err
		}
	}
	return nil
}
