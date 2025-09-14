package game

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"math/rand"
	"os"
	"path/filepath"
	"strings"
	"time"

	"go.mongodb.org/mongo-driver/bson/primitive"
)

// Stats describes the attribute distribution for an object layer.
type Stats struct {
	Effect       int `json:"effect" bson:"effect"`
	Resistance   int `json:"resistance" bson:"resistance"`
	Agility      int `json:"agility" bson:"agility"`
	Range        int `json:"range" bson:"range"`
	Intelligence int `json:"intelligence" bson:"intelligence"`
	Utility      int `json:"utility" bson:"utility"`
}

// RenderFrames holds named animation frames as 3D int matrices.
type RenderFrames struct {
	UpIdle           [][][]int `json:"up_idle" bson:"up_idle"`
	DownIdle         [][][]int `json:"down_idle" bson:"down_idle"`
	RightIdle        [][][]int `json:"right_idle" bson:"right_idle"`
	LeftIdle         [][][]int `json:"left_idle" bson:"left_idle"`
	UpRightIdle      [][][]int `json:"up_right_idle" bson:"up_right_idle"`
	DownRightIdle    [][][]int `json:"down_right_idle" bson:"down_right_idle"`
	UpLeftIdle       [][][]int `json:"up_left_idle" bson:"up_left_idle"`
	DownLeftIdle     [][][]int `json:"down_left_idle" bson:"down_left_idle"`
	DefaultIdle      [][][]int `json:"default_idle" bson:"default_idle"`
	UpWalking        [][][]int `json:"up_walking" bson:"up_walking"`
	DownWalking      [][][]int `json:"down_walking" bson:"down_walking"`
	RightWalking     [][][]int `json:"right_walking" bson:"right_walking"`
	LeftWalking      [][][]int `json:"left_walking" bson:"left_walking"`
	UpRightWalking   [][][]int `json:"up_right_walking" bson:"up_right_walking"`
	DownRightWalking [][][]int `json:"down_right_walking" bson:"down_right_walking"`
	UpLeftWalking    [][][]int `json:"up_left_walking" bson:"up_left_walking"`
	DownLeftWalking  [][][]int `json:"down_left_walking" bson:"down_left_walking"`
	NoneIdle         [][][]int `json:"none_idle" bson:"none_idle"`
}

// Render describes how to display an object layer.
type Render struct {
	Frames        RenderFrames `json:"frames" bson:"frames"`
	Colors        [][]int      `json:"colors" bson:"colors"`
	FrameDuration int          `json:"frame_duration" bson:"frame_duration"`
	IsStateless   bool         `json:"is_stateless" bson:"is_stateless"`
}

// Item describes a generic item.
type Item struct {
	ID          string `json:"id" bson:"id"`
	Type        string `json:"type" bson:"type"`
	Description string `json:"description" bson:"description"`
	Activable   bool   `json:"activable" bson:"activable"`
}

// ObjectLayerData groups the data for an ObjectLayer.
type ObjectLayerData struct {
	Stats  Stats  `json:"stats" bson:"stats"`
	Render Render `json:"render" bson:"render"`
	Item   Item   `json:"item" bson:"item"`
}

// ObjectLayer is the top level schema for an object layer.
type ObjectLayer struct {
	ID        primitive.ObjectID `json:"_id,omitempty" bson:"_id,omitempty"`
	Data      ObjectLayerData    `json:"data" bson:"data"`
	Sha256    string             `json:"sha256" bson:"sha256"`
	CreatedAt time.Time          `json:"createdAt,omitempty" bson:"createdAt,omitempty"`
	UpdatedAt time.Time          `json:"updatedAt,omitempty" bson:"updatedAt,omitempty"`
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
	p := payload{Stats: ol.Data.Stats, Render: ol.Data.Render, Item: ol.Data.Item}
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
	layer := BuildRandomObjectLayer()
	layer.Data.Item.ID = id
	layer.Data.Item.Type = "skin"
	layer.Data.Item.Description = ""
	layer.Data.Item.Activable = false
	_ = layer.UpdateHash()
	return layer
}

// randomInt returns a random int in [min, max].
func randomInt(min, max int) int {
	if max < min {
		min, max = max, min
	}
	return rand.Intn(max-min+1) + min
}

// randomBool returns a random boolean.
func randomBool() bool { return rand.Intn(2) == 0 }

// randomID creates a lightweight pseudo-random identifier with a given prefix.
func randomID(prefix string) string {
	alphabet := "abcdefghijklmnopqrstuvwxyz0123456789"
	b := make([]byte, 8)
	for i := range b {
		b[i] = alphabet[rand.Intn(len(alphabet))]
	}
	return fmt.Sprintf("%s-%s", prefix, string(b))
}

// random3D builds a WxHxD cube of small ints for demo frames.
func random3D(w, h, d, minVal, maxVal int) [][][]int {
	cube := make([][][]int, d)
	for z := 0; z < d; z++ {
		plane := make([][]int, h)
		for y := 0; y < h; y++ {
			row := make([]int, w)
			for x := 0; x < w; x++ {
				row[x] = randomInt(minVal, maxVal)
			}
			plane[y] = row
		}
		cube[z] = plane
	}
	return cube
}

// randomPalette returns N RGB colors with 0-255 ints.
func randomPalette(n int) [][]int {
	colors := make([][]int, n)
	for i := 0; i < n; i++ {
		colors[i] = []int{randomInt(0, 255), randomInt(0, 255), randomInt(0, 255)}
	}
	return colors
}

// BuildRandomObjectLayer constructs a small randomized ObjectLayer.
func BuildRandomObjectLayer() ObjectLayer {
	// Keep values modest so the file remains small.
	stats := Stats{
		Effect:       randomInt(0, 10),
		Resistance:   randomInt(0, 10),
		Agility:      randomInt(0, 10),
		Range:        randomInt(0, 10),
		Intelligence: randomInt(0, 10),
		Utility:      randomInt(0, 10),
	}

	frames := RenderFrames{
		UpIdle:           random3D(3, 3, 1, 0, 1),
		DownIdle:         random3D(3, 3, 1, 0, 1),
		RightIdle:        random3D(3, 3, 1, 0, 1),
		LeftIdle:         random3D(3, 3, 1, 0, 1),
		UpRightIdle:      random3D(3, 3, 1, 0, 1),
		DownRightIdle:    random3D(3, 3, 1, 0, 1),
		UpLeftIdle:       random3D(3, 3, 1, 0, 1),
		DownLeftIdle:     random3D(3, 3, 1, 0, 1),
		DefaultIdle:      random3D(3, 3, 1, 0, 1),
		UpWalking:        random3D(3, 3, 2, 0, 1),
		DownWalking:      random3D(3, 3, 2, 0, 1),
		RightWalking:     random3D(3, 3, 2, 0, 1),
		LeftWalking:      random3D(3, 3, 2, 0, 1),
		UpRightWalking:   random3D(3, 3, 2, 0, 1),
		DownRightWalking: random3D(3, 3, 2, 0, 1),
		UpLeftWalking:    random3D(3, 3, 2, 0, 1),
		DownLeftWalking:  random3D(3, 3, 2, 0, 1),
		NoneIdle:         random3D(3, 3, 1, 0, 1),
	}

	render := Render{
		Frames:        frames,
		Colors:        randomPalette(4),
		FrameDuration: randomInt(60, 180), // ms
		IsStateless:   randomBool(),
	}

	itemTypes := []string{"skin", "weapon", "armor", "artifact"}
	it := itemTypes[rand.Intn(len(itemTypes))]
	item := Item{
		ID:          randomID("item"),
		Type:        it,
		Description: fmt.Sprintf("Random %s generated by object_layer_create", it),
		Activable:   randomBool(),
	}

	ol := ObjectLayer{Data: ObjectLayerData{Stats: stats, Render: render, Item: item}}
	_ = ol.UpdateHash() // ignore err here; data is valid and local
	return ol
}

// DefaultOutPath returns the default output path for a random object layer.
func DefaultOutPath() string {
	return "./object_layer_random.json"
}

// NormalizePath normalizes a given path to an absolute path.
func NormalizePath(p string) string {
	p = strings.TrimSpace(p)
	if p == "" {
		return DefaultOutPath()
	}
	// Expand to absolute path for logging clarity but write using provided path.
	abs, err := filepath.Abs(p)
	if err == nil {
		return abs
	}
	return p
}

// WriteJSON writes a JSON object to a file.
func WriteJSON(path string, v any) error {
	b, err := json.MarshalIndent(v, "", "  ")
	if err != nil {
		return err
	}
	// Ensure directory exists if a nested path is provided.
	dir := filepath.Dir(path)
	if dir != "." && dir != "" {
		if err := os.MkdirAll(dir, 0o755); err != nil {
			return err
		}
	}
	return os.WriteFile(path, b, 0o644)
}
