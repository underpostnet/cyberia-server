package game

import (
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
