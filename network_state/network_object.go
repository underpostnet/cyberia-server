package network_state

import (
	"math"
)

// NetworkObject represents any entity in the network state world.
type NetworkObject struct {
	ID                string                   `json:"obj_id"`
	X                 float64                  `json:"x"`
	Y                 float64                  `json:"y"`
	Color             Color                    `json:"color"`
	IsObstacle        bool                     `json:"is_obstacle"`
	Speed             float64                  `json:"speed"`
	Path              []struct{ X, Y float64 } `json:"path"`
	PathIndex         int                      `json:"path_index"`
	NetworkObjectType string                   `json:"network_object_type"`
	ObjectLayerIDs    []string                 `json:"object_layer_ids"` // New field
	IsPersistent      bool                     `json:"is_persistent"`    // New field, defaults to true for most server objects
}

// NewNetworkObject is a factory function to create new NetworkObject instances.
func NewNetworkObject(id string, x, y float64, color Color, isObstacle bool, speed float64, objType string, objectLayerIDs []string, isPersistent bool) *NetworkObject {
	return &NetworkObject{
		ID:                id,
		X:                 x,
		Y:                 y,
		Color:             color,
		IsObstacle:        isObstacle,
		Speed:             speed,
		Path:              []struct{ X, Y float64 }{},
		PathIndex:         0,
		NetworkObjectType: objType,
		ObjectLayerIDs:    objectLayerIDs,
		IsPersistent:      isPersistent,
	}
}

// UpdatePosition moves the object along its current path based on elapsed time.
func (obj *NetworkObject) UpdatePosition(deltaTime float64) {
	if len(obj.Path) == 0 || obj.PathIndex >= len(obj.Path) {
		obj.Path = []struct{ X, Y float64 }{}
		obj.PathIndex = 0
		return
	}

	targetPoint := obj.Path[obj.PathIndex]
	dx := targetPoint.X - obj.X
	dy := targetPoint.Y - obj.Y
	distance := math.Sqrt(dx*dx + dy*dy)

	moveDistance := obj.Speed * deltaTime

	if distance < moveDistance {
		obj.X = targetPoint.X
		obj.Y = targetPoint.Y
		obj.PathIndex++
	} else {
		directionX := dx / distance
		directionY := dy / distance
		obj.X += directionX * moveDistance
		obj.Y += directionY * moveDistance
	}
}
