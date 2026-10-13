package content

import (
	"cyberia-server/game/world"
	"slices"

	"cyberia-server/logx"
)

func (s *Content) BuildEntityDefaultsSlice() []EntityTypeDefaultConfig {
	result := make([]EntityTypeDefaultConfig, len(s.EntityDefaultBuilds))
	copy(result, s.EntityDefaultBuilds)
	return result
}

// ResolveEntityDefaultBuild picks the entity-type default for an entity from its
// active item IDs. A default matches only when ALL of its liveItemIds are present
// on the entity (subset containment), and the MOST SPECIFIC match wins — the one
// requiring the largest item set. This lets the same skin carry different
// defaults by its full active set, e.g. {purple, atlas_pistol_mk2} → hostile but
// {purple} → passive. With no containing match it falls back to the first build
// of the type, then the per-type lookup.
func (s *Content) ResolveEntityDefaultBuild(entityType string, itemIDs []string) (EntityTypeDefaultConfig, bool) {
	var firstTypeBuild *EntityTypeDefaultConfig
	bestIndex := -1
	bestSize := 0

	for i := range s.EntityDefaultBuilds {
		build := s.EntityDefaultBuilds[i]
		if build.EntityType != entityType {
			continue
		}
		if firstTypeBuild == nil {
			firstTypeBuild = &s.EntityDefaultBuilds[i]
		}
		if !MatchesLiveItems(build, itemIDs) {
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
		return s.EntityDefaultBuilds[bestIndex], true
	}
	if firstTypeBuild != nil {
		return *firstTypeBuild, true
	}
	build, ok := s.EntityDefaults[entityType]
	return build, ok
}

// MatchesLiveItems reports whether every live item of the build is among
// itemIDs. A build with no live items matches nothing; it only stands in as
// the first build of its type.
func MatchesLiveItems(build EntityTypeDefaultConfig, itemIDs []string) bool {
	if len(build.LiveItemIDs) == 0 {
		return false
	}
	for _, itemID := range build.LiveItemIDs {
		if !slices.Contains(itemIDs, itemID) {
			return false
		}
	}
	return true
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
func buildLayer(build EntityTypeDefaultConfig, itemID string) world.ObjectLayerState {
	for _, layer := range build.DefaultObjectLayers {
		if layer.ItemID == itemID {
			return layer
		}
	}
	return world.ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1}
}

// SpawnObjectLayers is the one way every entity type reads its object-layer stack.
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
func (s *Content) SpawnObjectLayers(entityType string, placedItemIDs []string, seedDefaults bool) []world.ObjectLayerState {
	build, hasBuild := s.ResolveEntityDefaultBuild(entityType, placedItemIDs)
	layers := make([]world.ObjectLayerState, 0, len(placedItemIDs))
	seen := make(map[string]struct{}, len(placedItemIDs))
	appendLayer := func(layer world.ObjectLayerState) {
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
			appendLayer(world.ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
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
func (s *Content) normalizeLoadout(layers []world.ObjectLayerState) []world.ObjectLayerState {
	claimed := make(map[string]bool, len(layers))
	active := 0
	out := make([]world.ObjectLayerState, len(layers))
	copy(out, layers)
	for i := range out {
		if !out[i].Active {
			continue
		}
		if itemType := s.ItemType(out[i].ItemID); itemType != "" && s.EquipmentRules.ActiveItemTypes[itemType] {
			if s.EquipmentRules.OnePerType && claimed[itemType] {
				// Taken off, not taken away: the entity still carries the stack.
				out[i].Active = false
				continue
			}
			claimed[itemType] = true
		}
		active++
		if s.MaxActiveLayers > 0 && active > s.MaxActiveLayers {
			out[i].Active = false
			active--
		}
	}
	return out
}

func ActiveObjectLayerItemIDs(objectLayers []world.ObjectLayerState) []string {
	itemIDs := make([]string, 0, len(objectLayers))
	for _, objectLayer := range objectLayers {
		if !objectLayer.Active || objectLayer.ItemID == "" {
			continue
		}
		itemIDs = append(itemIDs, objectLayer.ItemID)
	}
	return itemIDs
}

// EntityTypeDefaultConfig — gameplay defaults for one entity type.
type EntityTypeDefaultConfig struct {
	EntityType          string                   `json:"entityType"`
	LiveItemIDs         []string                 `json:"liveItemIds"`
	DeadItemIDs         []string                 `json:"deadItemIds"`
	DropItemIDs         []string                 `json:"dropItemIds"`
	DefaultObjectLayers []world.ObjectLayerState `json:"defaultObjectLayers,omitempty"`
	// DropChances is itemID → probability in [0,1] that the id scatters on death. It lives on the
	// build rather than on ObjectLayerState because it describes what an entity type drops, not
	// what one entity is currently wearing — and ObjectLayerState rides every snapshot, where a
	// per-layer probability would be dead weight on the wire. A missing id means 1.
	DropChances map[string]float64 `json:"dropChances,omitempty"`
	// Canonical entity behavior bound to matched entities (see behavior.go).
	// Empty: a bot derives it from layers (armed → hostile, else passive); a foreground stays normal.
	Behavior string `json:"behavior,omitempty"`
}

// EquipmentRulesConfig governs which item types can be simultaneously active
// and enforces the one-active-per-type constraint.  Loaded from the proto
// EquipmentRules message via ApplyInstanceConfig.
type EquipmentRulesConfig struct {
	ActiveItemTypes map[string]bool `json:"activeItemTypes"` // set of activable item types
	OnePerType      bool            `json:"onePerType"`      // enforce one active per type
	RequireSkin     bool            `json:"requireSkin"`     // require ≥ 1 active skin
}

// SkillDefinition defines the properties of a single skill logic event
// associated with a trigger item.
type SkillDefinition struct {
	LogicEventID         string
	Name                 string
	Description          string
	SummonedEntityItemID string
}

// ItemType returns the ObjectLayer item type for an itemID, or "" if the
// ObjectLayer cache has no entry. Pure helper — no state mutation.
func (s *Content) ItemType(itemID string) string {
	if data, ok := s.GetObjectLayerData(itemID); ok {
		return data.Data.Item.Type
	}
	return ""
}
