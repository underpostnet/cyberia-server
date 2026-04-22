package game

func (s *GameServer) buildEntityDefaultsSlice() []EntityTypeDefaultConfig {
	result := make([]EntityTypeDefaultConfig, len(s.entityDefaultBuilds))
	copy(result, s.entityDefaultBuilds)
	return result
}

func (s *GameServer) resolveEntityDefaultBuild(entityType string, itemIDs []string) (EntityTypeDefaultConfig, bool) {
	var firstTypeBuild *EntityTypeDefaultConfig
	bestIndex := -1
	bestOverlap := 0
	bestLiveItemCount := 0

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

		overlap := 0
		for _, itemID := range build.LiveItemIDs {
			if _, ok := itemSet[itemID]; ok {
				overlap++
			}
		}
		if overlap == 0 {
			continue
		}
		if bestIndex == -1 || overlap > bestOverlap || (overlap == bestOverlap && len(build.LiveItemIDs) < bestLiveItemCount) {
			bestIndex = i
			bestOverlap = overlap
			bestLiveItemCount = len(build.LiveItemIDs)
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
