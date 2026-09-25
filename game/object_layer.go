package game

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
