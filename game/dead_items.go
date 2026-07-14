package game

import "sort"

// resolveDeadItemIDs returns the build's dead visuals. Every entity type that
// needs a dead state must declare deadItemIds in its engine-cyberia config.
// There is no hardcoded fallback — the gRPC boot is the single authority.
func (s *GameServer) resolveDeadItemIDs(build EntityTypeDefaultConfig) []string {
	return build.DeadItemIDs
}

// visibleInventory filters all dead-state items out of an inventory view.
// Other dead-state ids stay visible as Fragmentation items — equippable only
// while the player is dead (handleItemActivationInput gates by state).
// The dead-item set comes exclusively from s.deadItemIDs (gRPC boot).
func (s *GameServer) visibleInventory(layers []ObjectLayerState) []ObjectLayerState {
	out := make([]ObjectLayerState, 0, len(layers))
	for _, l := range layers {
		if _, ok := s.deadItemIDs[l.ItemID]; ok {
			continue
		}
		out = append(out, l)
	}
	return out
}

// activateOrAppendLayer activates the layer holding itemID, appending a new
// row when the inventory has none — same append semantics as applyDeadItems.
func activateOrAppendLayer(layers *[]ObjectLayerState, itemID string) {
	for i := range *layers {
		if (*layers)[i].ItemID == itemID {
			(*layers)[i].Active = true
			(*layers)[i].Quantity = 1
			return
		}
	}
	*layers = append(*layers, ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
}

// deadItemIDList returns the resolved dead-item ids (all from gRPC config),
// sorted for a deterministic wire order. Shipped in init_data so the client
// can label Fragmentation items and gate their equip UI.
func (s *GameServer) deadItemIDList() []string {
	ids := make([]string, 0, len(s.deadItemIDs))
	for id := range s.deadItemIDs {
		ids = append(ids, id)
	}
	sort.Strings(ids)
	return ids
}
