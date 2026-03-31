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
	AtlasSpriteSheetId        interface{}     `json:"atlasSpriteSheetId,omitempty"`
	Sha256                    string          `json:"sha256"`
	FrameDuration             int             `json:"frame_duration"`
	IsStateless               bool            `json:"is_stateless"`
	CreatedAt                 time.Time       `json:"createdAt,omitempty"`
	UpdatedAt                 time.Time       `json:"updatedAt,omitempty"`
}

// ═══════════════════════════════════════════════════════════════════
// Atlas Sprite Sheet types
// ═══════════════════════════════════════════════════════════════════

// FrameMeta describes one frame's position in the atlas PNG.
type FrameMeta struct {
	X          int `json:"x"`
	Y          int `json:"y"`
	Width      int `json:"width"`
	Height     int `json:"height"`
	FrameIndex int `json:"frameIndex"`
}

// DirFrames holds frames for each animation direction.
type DirFrames struct {
	UpIdle           []FrameMeta `json:"up_idle"`
	DownIdle         []FrameMeta `json:"down_idle"`
	RightIdle        []FrameMeta `json:"right_idle"`
	LeftIdle         []FrameMeta `json:"left_idle"`
	UpRightIdle      []FrameMeta `json:"up_right_idle"`
	DownRightIdle    []FrameMeta `json:"down_right_idle"`
	UpLeftIdle       []FrameMeta `json:"up_left_idle"`
	DownLeftIdle     []FrameMeta `json:"down_left_idle"`
	DefaultIdle      []FrameMeta `json:"default_idle"`
	UpWalking        []FrameMeta `json:"up_walking"`
	DownWalking      []FrameMeta `json:"down_walking"`
	RightWalking     []FrameMeta `json:"right_walking"`
	LeftWalking      []FrameMeta `json:"left_walking"`
	UpRightWalking   []FrameMeta `json:"up_right_walking"`
	DownRightWalking []FrameMeta `json:"down_right_walking"`
	UpLeftWalking    []FrameMeta `json:"up_left_walking"`
	DownLeftWalking  []FrameMeta `json:"down_left_walking"`
	NoneIdle         []FrameMeta `json:"none_idle"`
}

// AtlasData holds the metadata for a single atlas sprite sheet.
// The actual PNG binary is served via REST (/api/file/blob/:FileID).
type AtlasData struct {
	FileID       string    `json:"fileId"`
	ItemKey      string    `json:"itemKey"`
	AtlasWidth   int       `json:"atlasWidth"`
	AtlasHeight  int       `json:"atlasHeight"`
	CellPixelDim int       `json:"cellPixelDim"`
	Frames       DirFrames `json:"frames"`
}
