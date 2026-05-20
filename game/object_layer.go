package game

import "time"

// Stats describes the attribute distribution for an object layer.
type Stats struct {
	Effect       int `json:"effect"`
	Resistance   int `json:"resistance"`
	Agility      int `json:"agility"`
	Range        int `json:"range"`
	Intelligence int `json:"intelligence"`
	Utility      int `json:"utility"`
}

// Item describes a generic item.
type Item struct {
	ID          string `json:"id"`
	Type        string `json:"type"`
	Description string `json:"description"`
	Activable   bool   `json:"activable"`
}

// Ledger holds blockchain protocol metadata linking the visual object-layer
// prefab to its economic reality (token standard + smart-contract address).
type Ledger struct {
	Type    string `json:"type"`              // ERC20, ERC721, OFF_CHAIN
	Address string `json:"address,omitempty"` // Solidity contract address
}

// Render holds IPFS content identifiers for the consolidated atlas sprite sheet.
type Render struct {
	Cid         string `json:"cid,omitempty"`         // IPFS CID for the atlas PNG
	MetadataCid string `json:"metadataCid,omitempty"` // IPFS CID for the atlas metadata JSON
}

// ObjectLayerData groups the data for an ObjectLayer.
type ObjectLayerData struct {
	Stats  Stats   `json:"stats"`
	Item   Item    `json:"item"`
	Ledger *Ledger `json:"ledger,omitempty"`
	Render *Render `json:"render,omitempty"`
}

// ObjectLayer is the top level schema for an object layer.
type ObjectLayer struct {
	ID                        string          `json:"_id,omitempty"`
	Data                      ObjectLayerData `json:"data"`
	Cid                       string          `json:"cid,omitempty"`
	ObjectLayerRenderFramesId interface{}     `json:"objectLayerRenderFramesId,omitempty"`
	Sha256                    string          `json:"sha256"`
	CreatedAt                 time.Time       `json:"createdAt,omitempty"`
	UpdatedAt                 time.Time       `json:"updatedAt,omitempty"`
}


