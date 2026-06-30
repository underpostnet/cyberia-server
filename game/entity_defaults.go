package game

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
