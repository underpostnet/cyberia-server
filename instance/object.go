package instance

import (
	"math"
)

// InstanceObject represents any entity in the instance world.
type InstanceObject struct {
	ID         string  `json:"obj_id"`      // Unique identifier for the object
	X          float64 `json:"x"`           // X-coordinate in world space
	Y          float64 `json:"y"`           // Y-coordinate in world space
	Color      Color   `json:"color"`       // Visual color of the object
	IsObstacle bool    `json:"is_obstacle"` // True if the object blocks movement (e.g., a wall)
	Speed      float64 `json:"speed"`       // Movement speed in pixels per second

	// Pathfinding related fields
	Path      []struct{ X, Y float64 } `json:"path"`       // Current path as a slice of world coordinates
	PathIndex int                      `json:"path_index"` // Current index in the path being followed
}

// UpdatePosition moves the object along its current path based on elapsed time.
func (obj *InstanceObject) UpdatePosition(deltaTime float64) {
	// If path is empty or finished, reset path and stop movement.
	if len(obj.Path) == 0 || obj.PathIndex >= len(obj.Path) {
		obj.Path = []struct{ X, Y float64 }{}
		obj.PathIndex = 0
		return
	}

	targetPoint := obj.Path[obj.PathIndex] // The current target point in the path
	dx := targetPoint.X - obj.X
	dy := targetPoint.Y - obj.Y
	distance := math.Sqrt(dx*dx + dy*dy) // Distance to the target point

	moveDistance := obj.Speed * deltaTime // Distance object can cover in this frame

	if distance < moveDistance {
		// If close enough, snap to the target point and advance to the next path segment.
		obj.X = targetPoint.X
		obj.Y = targetPoint.Y
		obj.PathIndex++
	} else {
		// Move towards the target point.
		directionX := dx / distance // Normalized X direction
		directionY := dy / distance // Normalized Y direction
		obj.X += directionX * moveDistance
		obj.Y += directionY * moveDistance
	}
}
