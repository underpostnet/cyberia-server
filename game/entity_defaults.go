package game

import "cyberia-server/logx"

func (s *GameServer) buildEntityDefaultsSlice() []EntityTypeDefaultConfig {
	result := make([]EntityTypeDefaultConfig, len(s.entityDefaultBuilds))
	copy(result, s.entityDefaultBuilds)
	return result
}

// resolveEntityDefaultBuild picks the entity-type default for an entity from its
// active item IDs. A default matches only when ALL of its liveItemIds are present
// on the entity (subset containment), and the MOST SPECIFIC match wins — the one
// requiring the largest item set. This lets the same skin carry different
// defaults by its full active set, e.g. {purple, atlas_pistol_mk2} → hostile but
// {purple} → passive. With no containing match it falls back to the first build
// of the type, then the per-type lookup.
func (s *GameServer) resolveEntityDefaultBuild(entityType string, itemIDs []string) (EntityTypeDefaultConfig, bool) {
	var firstTypeBuild *EntityTypeDefaultConfig
	bestIndex := -1
	bestSize := 0

	itemSet := make(map[string]struct{}, len(itemIDs))
	for _, itemID := range itemIDs {
		if itemID == "" {
			continue
		}
		itemSet[itemID] = struct{}{}
	}

	for i := range s.entityDefaultBuilds {
		build := s.entityDefaultBuilds[i]
		if build.EntityType != entityType {
			continue
		}
		if firstTypeBuild == nil {
			firstTypeBuild = &s.entityDefaultBuilds[i]
		}
		if len(itemSet) == 0 || len(build.LiveItemIDs) == 0 {
			continue
		}

		contained := true
		for _, itemID := range build.LiveItemIDs {
			if _, ok := itemSet[itemID]; !ok {
				contained = false
				break
			}
		}
		if !contained {
			continue
		}
		// Most specific wins: the largest fully-contained liveItemIds set. Ties
		// keep the earlier build (deterministic by config order).
		if len(build.LiveItemIDs) > bestSize {
			bestIndex = i
			bestSize = len(build.LiveItemIDs)
		}
	}

	if bestIndex >= 0 {
		return s.entityDefaultBuilds[bestIndex], true
	}
	if firstTypeBuild != nil {
		return *firstTypeBuild, true
	}
	build, ok := s.entityDefaults[entityType]
	return build, ok
}

// stackQuantity is the stack size a build carries for itemID, taken from the
// seed inventory engine-cyberia derives for it (overrideItemsIdsState sizes the
// stack there). An id the build carries no row for is a single token.
func stackQuantity(build EntityTypeDefaultConfig, itemID string) int {
	for _, layer := range build.DefaultObjectLayers {
		if layer.ItemID == itemID && layer.Quantity > 0 {
			return layer.Quantity
		}
	}
	return 1
}

// dropChance is how often an id actually scatters when the entity dies, in [0,1].
//
// A build that says nothing about an id drops it every time, which is what every world did before
// the field existed. Values are clamped rather than rejected: engine-cyberia already constrains
// the authored range, and a payload that slipped past it should bias toward the old behaviour
// instead of silently dropping nothing.
func dropChance(build EntityTypeDefaultConfig, itemID string) float64 {
	chance, ok := build.DropChances[itemID]
	if !ok {
		return 1
	}
	if chance < 0 {
		return 0
	}
	if chance > 1 {
		return 1
	}
	return chance
}

// buildLayer is a placed id as the build states it: its row when the build carries one, and a
// plainly worn single item when it does not.
func buildLayer(build EntityTypeDefaultConfig, itemID string) ObjectLayerState {
	for _, layer := range build.DefaultObjectLayers {
		if layer.ItemID == itemID {
			return layer
		}
	}
	return ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1}
}

// spawnObjectLayers is the one way every entity type reads its object-layer stack.
//
// A map definition names what an entity *is* — the skin and visuals its author placed on it — and
// the entity-type default names everything it carries: the lifecycle discriminators (live, dead,
// drop), the inventory-only extras, and whatever an override adds to them. Both are seeded, each
// layer keeping the active flag and stack size engine-cyberia derived for it
// (resolveEntityInventory). Seeding the whole set is what lets a lifecycle change activate a slot
// that is already there (activateOrAppendLayer) instead of appending one, and what puts an
// overridden item on a live entity rather than leaving it in a list nothing reads.
//
// A placed id is active by definition: it is what the author put there. seedDefaults is false for
// a solid-colour entity placed without sprites, which carries no default layers at all.
func (s *GameServer) spawnObjectLayers(entityType string, placedItemIDs []string, seedDefaults bool) []ObjectLayerState {
	build, hasBuild := s.resolveEntityDefaultBuild(entityType, placedItemIDs)
	layers := make([]ObjectLayerState, 0, len(placedItemIDs))
	seen := make(map[string]struct{}, len(placedItemIDs))
	appendLayer := func(layer ObjectLayerState) {
		if layer.ItemID == "" {
			return
		}
		if _, ok := seen[layer.ItemID]; ok {
			return
		}
		seen[layer.ItemID] = struct{}{}
		layers = append(layers, layer)
	}

	// Placement decides what an entity has; the build decides what it wears. A placed id the build
	// carries a row for takes that row's state — which is how an override that names a second weapon
	// reaches the entity with the same answer engine-cyberia already wrote into the map.
	for _, itemID := range placedItemIDs {
		appendLayer(buildLayer(build, itemID))
	}
	if !seedDefaults || !hasBuild {
		return layers
	}
	for _, layer := range build.DefaultObjectLayers {
		appendLayer(layer)
	}
	// One line per spawned entity, at debug: what a world actually seeds is the fact that settles
	// "the item is in the config but not on screen", without reading a snapshot off the wire.
	logx.Debugf("spawn %s placed=%v seeded=%d from build live=%v", entityType, placedItemIDs, len(layers), build.LiveItemIDs)
	// A build seeded before engine-cyberia derived its inventory still names its live ids.
	if len(layers) == 0 {
		for _, itemID := range build.LiveItemIDs {
			appendLayer(ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
		}
	}
	return s.normalizeLoadout(layers)
}

// normalizeLoadout settles what may be active on an entity at once, the same way
// normalizeDeadLoadout settles a dead one: one active item per governed type, first in stack order
// keeps the slot, and the maxActiveLayers cap applies to what is left active.
//
// A contested slot is normally already settled by engine-cyberia, which knows which item an
// override named; this is the rule holding at the boundary the simulation owns, for a build that
// reached it any other way. A type the rules do not govern — a resource visual, a coin — passes
// through, and so does an item whose type this server has not cached, exactly as an activation
// request does.
func (s *GameServer) normalizeLoadout(layers []ObjectLayerState) []ObjectLayerState {
	claimed := make(map[string]bool, len(layers))
	active := 0
	out := make([]ObjectLayerState, len(layers))
	copy(out, layers)
	for i := range out {
		if !out[i].Active {
			continue
		}
		if itemType := s.itemType(out[i].ItemID); itemType != "" && s.equipmentRules.ActiveItemTypes[itemType] {
			if s.equipmentRules.OnePerType && claimed[itemType] {
				// Taken off, not taken away: the entity still carries the stack.
				out[i].Active = false
				continue
			}
			claimed[itemType] = true
		}
		active++
		if s.maxActiveLayers > 0 && active > s.maxActiveLayers {
			out[i].Active = false
			active--
		}
	}
	return out
}

func activeObjectLayerItemIDs(objectLayers []ObjectLayerState) []string {
	itemIDs := make([]string, 0, len(objectLayers))
	for _, objectLayer := range objectLayers {
		if !objectLayer.Active || objectLayer.ItemID == "" {
			continue
		}
		itemIDs = append(itemIDs, objectLayer.ItemID)
	}
	return itemIDs
}
