package content

import (
	"errors"
	"maps"
	"slices"

	"cyberia-server/logx"
)

// ErrUnbound is the answer of the engine for a label it binds to no definition.
var ErrUnbound = errors.New("the label has no bound definition")

// Item describes a generic item.
type Item struct {
	ID          string `json:"id"`
	Type        string `json:"type"`
	Description string `json:"description"`
	Activable   bool   `json:"activable"`
}

// Ledger is the ItemLedger binding of a definition: a projection of chain state.
// Empty when the definition is not registered.
type Ledger struct {
	Standard        string `json:"standard,omitempty"` // ERC1155
	ChainID         uint64 `json:"chainId,omitempty"`
	ContractAddress string `json:"contractAddress,omitempty"`
	TokenID         string `json:"tokenId,omitempty"` // uint256, decimal
}

// Render is the render contract of a definition. Both CIDs are empty when it names no render.
type Render struct {
	Cid         string `json:"cid,omitempty"`         // canonical render CID: the primary render PNG
	MetadataCid string `json:"metadataCid,omitempty"` // canonical metadata CID: the layout of the primary render
}

// ObjectLayerData groups the data for an ObjectLayer.
type ObjectLayerData struct {
	Stats  Stats   `json:"stats"`
	Item   Item    `json:"item"`
	Ledger *Ledger `json:"ledger,omitempty"`
	Render *Render `json:"render,omitempty"`
}

// ObjectLayer is one immutable Object Layer definition. Cid is its canonical identity;
// Data.Item.ID is the Cyberia label the catalog binds to it.
type ObjectLayer struct {
	Data ObjectLayerData `json:"data"`
	Cid  string          `json:"cid"`
}

// ReplaceObjectLayerCache atomically replaces the entire cache with the
// definitions the world names. Used by WorldBuilder for the initial full load.
func (s *Content) ReplaceObjectLayerCache(cache map[string]*ObjectLayer) error {
	if err := validateObjectLayers(cache); err != nil {
		return err
	}
	s.OlMu.Lock()
	defer s.OlMu.Unlock()
	s.ObjectLayerDataCache = make(map[string]*ObjectLayer, len(cache))
	s.olChanged = make(map[string]uint64, len(cache))
	for itemID, ol := range cache {
		s.store(itemID, ol)
	}
	s.warmLabels = labelSet(cache)

	typeCounts := make(map[string]int)
	for _, layer := range cache {
		itemType := layer.Data.Item.Type
		if itemType == "" {
			itemType = "(unknown)"
		}
		typeCounts[itemType]++
	}
	logx.Infof("Object layer cache replaced with %d items.", len(cache))
	for itemType, count := range typeCounts {
		logx.Debugf("  %-20s %d", itemType, count)
	}
	return nil
}

// ReplaceWarmObjectLayers caches the definitions the rebuilt world names and
// makes them its warm set. A label the world no longer names stays until
// EvictResolvedObjectLayers finds nothing that holds it.
func (s *Content) ReplaceWarmObjectLayers(world map[string]*ObjectLayer) error {
	if err := s.PatchObjectLayerCache(world, nil); err != nil {
		return err
	}
	s.OlMu.Lock()
	s.warmLabels = labelSet(world)
	s.OlMu.Unlock()
	return nil
}

// PatchObjectLayerCache applies incremental updates and deletions.
// Used by WorldBuilder for hot-reload via gRPC manifest diffing.
func (s *Content) PatchObjectLayerCache(updates map[string]*ObjectLayer, deletions []string) error {
	if err := validateObjectLayers(updates); err != nil {
		return err
	}
	s.OlMu.Lock()
	defer s.OlMu.Unlock()
	for itemID, ol := range updates {
		s.store(itemID, ol)
	}
	for _, itemID := range deletions {
		s.drop(itemID)
	}
	return nil
}

// store caches a definition under its label. A new label or a new definition
// is a change the clients receive. Caller holds OlMu.
func (s *Content) store(itemID string, ol *ObjectLayer) {
	if s.ObjectLayerDataCache == nil {
		s.ObjectLayerDataCache = make(map[string]*ObjectLayer)
	}
	if s.olChanged == nil {
		s.olChanged = make(map[string]uint64)
	}
	if cached, ok := s.ObjectLayerDataCache[itemID]; ok && cached.Cid == ol.Cid {
		s.ObjectLayerDataCache[itemID] = ol
		return
	}
	s.olRevision++
	s.ObjectLayerDataCache[itemID] = ol
	s.olChanged[itemID] = s.olRevision
}

// drop removes a label from the cache. Caller holds OlMu.
func (s *Content) drop(itemID string) {
	delete(s.ObjectLayerDataCache, itemID)
	delete(s.olChanged, itemID)
}

// ObjectLayerRevision is the revision of the last definition change.
func (s *Content) ObjectLayerRevision() uint64 {
	s.OlMu.RLock()
	defer s.OlMu.RUnlock()
	return s.olRevision
}

// ObjectLayers is every cached definition and the current revision: what a
// joining client receives.
func (s *Content) ObjectLayers() (map[string]*ObjectLayer, uint64) {
	s.OlMu.RLock()
	defer s.OlMu.RUnlock()
	return maps.Clone(s.ObjectLayerDataCache), s.olRevision
}

// ObjectLayersSince is every cached definition that changed after revision,
// and the current revision.
func (s *Content) ObjectLayersSince(revision uint64) (map[string]*ObjectLayer, uint64) {
	s.OlMu.RLock()
	defer s.OlMu.RUnlock()
	changed := make(map[string]*ObjectLayer)
	for itemID, at := range s.olChanged {
		if at > revision {
			changed[itemID] = s.ObjectLayerDataCache[itemID]
		}
	}
	return changed, s.olRevision
}

// GetObjectLayerData returns an ObjectLayer by item ID (read-locked).
func (s *Content) GetObjectLayerData(itemID string) (*ObjectLayer, bool) {
	s.OlMu.RLock()
	defer s.OlMu.RUnlock()
	ol, ok := s.ObjectLayerDataCache[itemID]
	return ol, ok
}

// ObjectLayerManifest is the label and the definition of every cached
// definition: what a hot reload compares with the engine.
func (s *Content) ObjectLayerManifest() map[string]string {
	s.OlMu.RLock()
	defer s.OlMu.RUnlock()
	manifest := make(map[string]string, len(s.ObjectLayerDataCache))
	for itemID, ol := range s.ObjectLayerDataCache {
		manifest[itemID] = ol.Cid
	}
	return manifest
}

// MissingObjectLayers is each label of itemIDs the cache does not hold, once.
func (s *Content) MissingObjectLayers(itemIDs []string) []string {
	s.OlMu.RLock()
	defer s.OlMu.RUnlock()
	var missing []string
	for _, itemID := range itemIDs {
		if _, ok := s.ObjectLayerDataCache[itemID]; !ok && itemID != "" && !slices.Contains(missing, itemID) {
			missing = append(missing, itemID)
		}
	}
	return missing
}

// AddResolvedObjectLayers caches definitions the engine resolved for a player
// outside the warm set. A label the cache holds keeps its definition, and an
// invalid definition is refused with the label. Caller holds s.mu, so no sweep
// runs before the player holds them.
func (s *Content) AddResolvedObjectLayers(layers map[string]*ObjectLayer) {
	s.OlMu.Lock()
	defer s.OlMu.Unlock()
	for itemID, ol := range layers {
		if _, ok := s.ObjectLayerDataCache[itemID]; ok {
			continue
		}
		if err := validateObjectLayers(map[string]*ObjectLayer{itemID: ol}); err != nil {
			logx.Warnf("[Content] resolved %s is not runnable here: %v", itemID, err)
			continue
		}
		s.store(itemID, ol)
	}
}

// EvictResolvedObjectLayers removes the cached definitions outside the warm
// set that no entity, personal storage or vault holds, nor the skill of a held
// label summons, with their resolved skills, and returns their labels. Caller
// holds s.mu.
func (s *Content) EvictResolvedObjectLayers() []string {
	held := make(map[string]bool)
	for _, ms := range s.Maps {
		for id := range ms.IDs() {
			for _, layer := range ms.Ap(id).Layers {
				held[layer.ItemID] = true
			}
		}
	}
	for _, stacks := range s.PersonalStorage {
		holdStacks(held, stacks)
	}
	for _, vaults := range s.Vaults {
		for _, stacks := range vaults {
			holdStacks(held, stacks)
		}
	}
	s.keepResolved(held)
	s.OlMu.Lock()
	var evicted []string
	for itemID := range s.ObjectLayerDataCache {
		if !s.warmLabels[itemID] && !held[itemID] {
			s.drop(itemID)
			evicted = append(evicted, itemID)
		}
	}
	s.OlMu.Unlock()
	for _, itemID := range evicted {
		delete(s.resolvedSkills, itemID)
		delete(s.requires, itemID)
	}
	if len(evicted) > 0 {
		s.composeSkills()
	}
	return evicted
}

// ResolvedObjectLayerCount is the number of cached definitions outside the warm set.
func (s *Content) ResolvedObjectLayerCount() int {
	s.OlMu.RLock()
	defer s.OlMu.RUnlock()
	count := 0
	for itemID := range s.ObjectLayerDataCache {
		if !s.warmLabels[itemID] {
			count++
		}
	}
	return count
}

func holdStacks(held map[string]bool, stacks []Stack) {
	for _, st := range stacks {
		held[st.ItemID] = true
	}
}

func labelSet(cache map[string]*ObjectLayer) map[string]bool {
	labels := make(map[string]bool, len(cache))
	for itemID := range cache {
		labels[itemID] = true
	}
	return labels
}
