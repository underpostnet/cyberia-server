package content

import (
	"cyberia-server/logx"
)

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

// ReplaceObjectLayerCache atomically replaces the entire cache.
// Used by WorldBuilder for initial full load via gRPC.
func (s *Content) ReplaceObjectLayerCache(cache map[string]*ObjectLayer) error {
	if err := validateObjectLayers(cache); err != nil {
		return err
	}
	s.OlMu.Lock()
	defer s.OlMu.Unlock()
	s.ObjectLayerDataCache = cache

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

// PatchObjectLayerCache applies incremental updates and deletions.
// Used by WorldBuilder for hot-reload via gRPC manifest diffing.
func (s *Content) PatchObjectLayerCache(updates map[string]*ObjectLayer, deletions []string) error {
	if err := validateObjectLayers(updates); err != nil {
		return err
	}
	s.OlMu.Lock()
	defer s.OlMu.Unlock()
	if s.ObjectLayerDataCache == nil {
		s.ObjectLayerDataCache = make(map[string]*ObjectLayer)
	}
	for itemID, ol := range updates {
		s.ObjectLayerDataCache[itemID] = ol
	}
	for _, itemID := range deletions {
		delete(s.ObjectLayerDataCache, itemID)
	}
	return nil
}

// GetObjectLayerData returns an ObjectLayer by item ID (read-locked).
func (s *Content) GetObjectLayerData(itemID string) (*ObjectLayer, bool) {
	s.OlMu.RLock()
	defer s.OlMu.RUnlock()
	ol, ok := s.ObjectLayerDataCache[itemID]
	return ol, ok
}
