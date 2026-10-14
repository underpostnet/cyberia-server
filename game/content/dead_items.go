package content

import (
	"sort"

	"cyberia-server/game/world"
)

// ResolveDeadItemIDs returns the build's dead visuals. Every entity type that
// needs a dead state must declare deadItemIds in its engine-cyberia config.
// There is no hardcoded fallback — the gRPC boot is the single authority.
func (s *Content) ResolveDeadItemIDs(build EntityTypeDefaultConfig) []string {
	return build.DeadItemIDs
}

// NormalizeDeadLoadout applies the equipment rules to a dead loadout in id
// order: one active per type (first wins, so the configured default leads)
// and the maxActiveLayers cap. Unknown types pass through, matching the live
// normalization.
func (s *Content) NormalizeDeadLoadout(ids []string) []string {
	out := make([]string, 0, len(ids))
	seenID := make(map[string]bool)
	seenType := make(map[string]bool)
	for _, id := range ids {
		if id == "" || seenID[id] {
			continue
		}
		seenID[id] = true
		if s.EquipmentRules.OnePerType {
			if t := s.ItemType(id); t != "" {
				if seenType[t] {
					continue
				}
				seenType[t] = true
			}
		}
		out = append(out, id)
		if s.MaxActiveLayers > 0 && len(out) >= s.MaxActiveLayers {
			break
		}
	}
	return out
}

// ActivateOrAppendLayer activates the layer holding itemID, appending a new
// row when the inventory has none — same append semantics as applyDeadItems.
func ActivateOrAppendLayer(layers *[]world.ObjectLayerState, itemID string) {
	for i := range *layers {
		if (*layers)[i].ItemID == itemID {
			(*layers)[i].Active = true
			(*layers)[i].Quantity = 1
			return
		}
	}
	*layers = append(*layers, world.ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
}

// DeadItemIDList returns the resolved dead-item ids (all from gRPC config),
// sorted for a deterministic wire order. Shipped in init_data so the client
// can label Fragmentation items and gate their equip UI.
func (s *Content) DeadItemIDList() []string {
	ids := make([]string, 0, len(s.DeadItemIDs))
	for id := range s.DeadItemIDs {
		ids = append(ids, id)
	}
	sort.Strings(ids)
	return ids
}
