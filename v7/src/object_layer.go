package game

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
)

// Stats describes the attribute distribution for an object layer.
type Stats struct {
	Effect       int `json:"effect"`
	Resistance   int `json:"resistance"`
	Agility      int `json:"agility"`
	Range        int `json:"range"`
	Intelligence int `json:"intelligence"`
	Utility      int `json:"utility"`
}

// RenderFrames holds named animation frames as 3D int matrices.
type RenderFrames struct {
	UpIdle         [][][]int `json:"up_idle"`
	DownIdle       [][][]int `json:"down_idle"`
	RightIdle      [][][]int `json:"right_idle"`
	LeftIdle       [][][]int `json:"left_idle"`
	UpRightIdle    [][][]int `json:"up_right_idle"`
	DownRightIdle  [][][]int `json:"down_right_idle"`
	UpLeftIdle     [][][]int `json:"up_left_idle"`
	DownLeftIdle   [][][]int `json:"down_left_idle"`
	DefaultIdle    [][][]int `json:"default_idle"`
	UpWalking      [][][]int `json:"up_walking"`
	DownWalking    [][][]int `json:"down_walking"`
	RightWalking   [][][]int `json:"right_walking"`
	LeftWalking    [][][]int `json:"left_walking"`
	UpRightWalking [][][]int `json:"up_right_walking"`
	DownRightWalking [][][]int `json:"down_right_walking"`
	UpLeftWalking  [][][]int `json:"up_left_walking"`
	DownLeftWalking [][][]int `json:"down_left_walking"`
	NoneIdle       [][][]int `json:"none_idle"`
}

// Render holds the frames, palette/colors and timing metadata.
type Render struct {
	Frames        RenderFrames `json:"frames"`
	Colors        [][]int      `json:"colors"`
	FrameDuration int          `json:"frame_duration"`
	IsStateless   bool         `json:"is_stateless"`
}

// Item describes the item this layer represents.
type Item struct {
	ID          string `json:"id"`
	Type        string `json:"type"`
	Description string `json:"description"`
	Activable   bool   `json:"activable"`
}

// ObjectLayer encapsulates stats, render, item and a content hash.
type ObjectLayer struct {
	Stats  Stats  `json:"stats"`
	Render Render `json:"render"`
	Item   Item   `json:"item"`
	Sha256 string `json:"sha256"`
}

// ComputeHash computes a SHA-256 hash of the canonical JSON encoding of the
// ObjectLayer core content (stats, render, item) and returns it as hex.
func (ol *ObjectLayer) ComputeHash() (string, error) {
	// Hash only the encapsulated content, not the hash field itself
	type payload struct {
		Stats  Stats  `json:"stats"`
		Render Render `json:"render"`
		Item   Item   `json:"item"`
	}
	p := payload{Stats: ol.Stats, Render: ol.Render, Item: ol.Item}
	b, err := json.Marshal(p)
	if err != nil {
		return "", err
	}
	sum := sha256.Sum256(b)
	return hex.EncodeToString(sum[:]), nil
}

// UpdateHash updates the Sha256 field to reflect current content.
func (ol *ObjectLayer) UpdateHash() error {
	h, err := ol.ComputeHash()
	if err != nil {
		return err
	}
	ol.Sha256 = h
	return nil
}

// NewDefaultSkinObjectLayer builds a minimal skin layer with given item ID.
func NewDefaultSkinObjectLayer(id string) ObjectLayer {
	layer := ObjectLayer{
		Stats: Stats{},
		Render: Render{
			Frames:        RenderFrames{},
			Colors:        nil,
			FrameDuration: 0,
			IsStateless:   true,
		},
		Item: Item{
			ID:          id,
			Type:        "skin",
			Description: "",
			Activable:   false,
		},
	}
	_ = layer.UpdateHash()
	return layer
}
