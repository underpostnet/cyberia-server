package game

import (
	"math"
	"math/rand"
	"time"

	"github.com/google/uuid"
)

// executePlayerBulletSkill contains the specific logic for the "atlas_pistol_mk2_bullet" skill.
// It has a chance to spawn a temporary, fast-moving "bullet" bot in the player's current direction.
func (s *GameServer) executePlayerBulletSkill(player *PlayerState, mapState *MapState, itemID string, target Point) {
	const bulletSpawnChance = 0.25 // 25% chance
	if rand.Float64() >= bulletSpawnChance {
		return // Skill did not activate on this action
	}

	// Calculate direction from player to target, as player.Direction might be NONE if idle.
	dx := target.X - player.Pos.X
	dy := target.Y - player.Pos.Y
	dist := math.Sqrt(dx*dx + dy*dy)

	var bulletDirection Direction
	if dist > 0 {
		dirX, dirY := dx/dist, dy/dist
		angle := math.Atan2(dirY, dirX)
		if angle < 0 {
			angle += 2 * math.Pi
		}
		directionIndex := (int(math.Round(angle/(math.Pi/4))) + 2) % 8
		bulletDirection = Direction(directionIndex)
	} else {
		// If target is same as player pos, use player's current direction as a fallback.
		bulletDirection = player.Direction
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	bulletLifetime := 5 * time.Second
	bulletDims := Dimensions{Width: 2, Height: 2}
	const bulletLife = 100.0

	// Pre-compensate for the first tick's movement.
	// The bullet is created and then moved in the same game loop before being sent to the client.
	// By spawning it one "step" behind its intended origin, it will appear at the correct
	// location on the first frame it's rendered.
	bulletStep := (s.playerSpeed * 2.0) / 10.0
	dirX, dirY := getDirectionVector(bulletDirection)

	// The desired position on the first rendered frame is the player's position.
	// The actual spawn position is one step behind this.
	spawnX := player.Pos.X - (dirX * bulletStep)
	spawnY := player.Pos.Y - (dirY * bulletStep)

	bulletBot := &BotState{
		ID:           uuid.New().String(),
		MapID:        player.MapID,
		Pos:          Point{X: spawnX, Y: spawnY},
		Dims:         bulletDims,
		Behavior:     "bullet",        // New behavior type for straight-line movement
		Direction:    bulletDirection, // Use the calculated direction to the target
		ExpiresAt:    time.Now().Add(bulletLifetime),
		ObjectLayers: []ObjectLayerState{{ItemID: "atlas_pistol_mk2_bullet", Active: true, Quantity: 1}},
		CasterID:     player.ID,
		MaxLife:      bulletLife,
		Life:         bulletLife,
	}

	mapState.bots[bulletBot.ID] = bulletBot
	// log.Printf("Player %s triggered skill 'atlas_pistol_mk2_bullet', spawning bullet bot %s", player.ID, bulletBot.ID)
}

// executeBotBulletSkill contains the specific logic for the "atlas_pistol_mk2_bullet" skill, triggered by a bot.
// It has a chance to spawn a temporary, fast-moving "bullet" bot towards the bot's target.
// NOTE: This is called from within updateBots, which is already under a server-wide mutex.
func (s *GameServer) executeBotBulletSkill(bot *BotState, mapState *MapState, itemID string, target Point) {
	const bulletSpawnChance = 0.25 // 25% chance
	if rand.Float64() >= bulletSpawnChance {
		return // Skill did not activate on this action
	}

	// Calculate direction from bot to its target.
	dx := target.X - bot.Pos.X
	dy := target.Y - bot.Pos.Y
	dist := math.Sqrt(dx*dx + dy*dy)

	var bulletDirection Direction
	if dist > 0 {
		dirX, dirY := dx/dist, dy/dist
		angle := math.Atan2(dirY, dirX)
		if angle < 0 {
			angle += 2 * math.Pi
		}
		directionIndex := (int(math.Round(angle/(math.Pi/4))) + 2) % 8
		bulletDirection = Direction(directionIndex)
	} else {
		// If target is same as bot pos, use bot's current direction as a fallback.
		bulletDirection = bot.Direction
	}

	bulletLifetime := 5 * time.Second
	bulletDims := Dimensions{Width: 2, Height: 2}
	const bulletLife = 100.0

	// Pre-compensate for the first tick's movement.
	// The bullet is created and then moved in the same game loop before being sent to the client.
	// By spawning it one "step" behind its intended origin, it will appear at the correct
	// location on the first frame it's rendered.
	bulletStep := (s.playerSpeed * 2.0) / 10.0
	dirX, dirY := getDirectionVector(bulletDirection)

	// The desired position on the first rendered frame is the bot's position.
	// The actual spawn position is one step behind this.
	spawnX := bot.Pos.X - (dirX * bulletStep)
	spawnY := bot.Pos.Y - (dirY * bulletStep)

	bulletBot := &BotState{
		ID:           uuid.New().String(),
		MapID:        bot.MapID,
		Pos:          Point{X: spawnX, Y: spawnY},
		Dims:         bulletDims,
		Behavior:     "bullet",        // New behavior type for straight-line movement
		Direction:    bulletDirection, // Use the calculated direction to the target
		ExpiresAt:    time.Now().Add(bulletLifetime),
		ObjectLayers: []ObjectLayerState{{ItemID: "atlas_pistol_mk2_bullet", Active: true, Quantity: 1}},
		CasterID:     bot.ID,
		MaxLife:      bulletLife,
		Life:         bulletLife,
	}

	mapState.bots[bulletBot.ID] = bulletBot
	// log.Printf("Bot %s triggered skill 'atlas_pistol_mk2_bullet', spawning bullet bot %s", bot.ID, bulletBot.ID)
}
