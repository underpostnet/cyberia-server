package main

import (
	"fmt"
	"math"
	"time"
)

//----------------------------------------------------------------------------------------------------------------------
// 1. Data Structures & Interfaces
// These types encapsulate the data needed for AOI management.
//----------------------------------------------------------------------------------------------------------------------

// MapData represents the complete, static context of the game map.
// This is the full map state that the server would hold.
type MapData struct {
	Width, Height int
	Obstacles     [][]int // 0 for free space, 1 for obstacle
	// We can add more here, like resources, points of interest, etc.
}

// AOIUpdates represents the data sent to a single client.
// It is an abstraction that simplifies the full map context into
// a manageable update packet. This is what's 'transmitted'.
type AOIUpdates struct {
	PlayerID  string
	PlayerPos PointI
	// We use a map here for efficient lookup of grid objects within the AOI.
	// The key is a PointI, and the value could be an object ID or type.
	VisibleGridObjects map[PointI]string
	// For a real MMO, this would also contain other player updates, NPC positions, etc.
	VisiblePlayers map[string]PointI
}

// Player represents a single player with a unique ID and position.
type Player struct {
	ID       string
	Position PointI
	isOnline bool
}

// AOIManager is the central class for handling all AOI-related logic.
// It manages the state of the map and all active players.
type AOIManager struct {
	MapData    *MapData
	Players    map[string]*Player
	lastUpdate time.Time
}

//----------------------------------------------------------------------------------------------------------------------
// 2. Core Logic & Methods
// These methods implement the core logic for AOI data management.
//----------------------------------------------------------------------------------------------------------------------

// NewAOIManager creates and returns a new AOIManager instance.
func NewAOIManager(mapData *MapData) *AOIManager {
	return &AOIManager{
		MapData: mapData,
		Players: make(map[string]*Player),
	}
}

// AddPlayer adds a new player to the manager.
func (am *AOIManager) AddPlayer(playerID string, startPos PointI) {
	if _, exists := am.Players[playerID]; !exists {
		am.Players[playerID] = &Player{
			ID:       playerID,
			Position: startPos,
			isOnline: true,
		}
	} else {
		// Player already exists, maybe just set them to online.
		am.Players[playerID].isOnline = true
		am.Players[playerID].Position = startPos
	}
}

// RemovePlayer marks a player as offline.
func (am *AOIManager) RemovePlayer(playerID string) {
	if player, exists := am.Players[playerID]; exists {
		player.isOnline = false
	}
}

// UpdatePlayerPosition changes a player's position in the world.
func (am *AOIManager) UpdatePlayerPosition(playerID string, newPos PointI) error {
	if player, exists := am.Players[playerID]; exists {
		// A more robust system would validate the position here (e.g., is it walkable?)
		player.Position = newPos
		return nil
	}
	return fmt.Errorf("player with ID '%s' not found", playerID)
}

// GetAOIUpdatesForPlayer generates a simplified AOI data packet for a given player.
// It only includes grid objects and other players within the specified radius.
func (am *AOIManager) GetAOIUpdatesForPlayer(playerID string, radius float64) (*AOIUpdates, error) {
	player, exists := am.Players[playerID]
	if !exists || !player.isOnline {
		return nil, fmt.Errorf("player with ID '%s' not found or is offline", playerID)
	}

	updates := &AOIUpdates{
		PlayerID:           playerID,
		PlayerPos:          player.Position,
		VisibleGridObjects: make(map[PointI]string),
		VisiblePlayers:     make(map[string]PointI),
	}

	// Define the AOI bounding box to optimize iteration.
	minX := int(math.Max(0, float64(player.Position.X)-radius))
	maxX := int(math.Min(float64(am.MapData.Width-1), float64(player.Position.X)+radius))
	minY := int(math.Max(0, float64(player.Position.Y)-radius))
	maxY := int(math.Min(float64(am.MapData.Height-1), float64(player.Position.Y)+radius))

	// Iterate through the grid within the AOI bounding box.
	for y := minY; y <= maxY; y++ {
		for x := minX; x <= maxX; x++ {
			// Check if the cell is actually within the circular radius.
			if math.Hypot(float64(x-player.Position.X), float64(y-player.Position.Y)) <= radius {
				if am.MapData.Obstacles[y][x] == 1 {
					updates.VisibleGridObjects[PointI{X: x, Y: y}] = "Obstacle"
				}
			}
		}
	}

	// Find other players within the radius.
	for otherPlayerID, otherPlayer := range am.Players {
		if otherPlayerID != playerID && otherPlayer.isOnline {
			dist := math.Hypot(float64(otherPlayer.Position.X-player.Position.X), float64(otherPlayer.Position.Y-player.Position.Y))
			if dist <= radius {
				updates.VisiblePlayers[otherPlayerID] = otherPlayer.Position
			}
		}
	}

	return updates, nil
}
