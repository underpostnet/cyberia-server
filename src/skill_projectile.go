package game

import (
	"math"
	"math/rand"
	"time"

	"github.com/google/uuid"
)

// executePlayerProjectileSkill contains the specific logic for the "atlas_pistol_mk2_logic" skill.
// It has a chance to spawn a temporary, fast-moving "skill" projectile bot in the player's current direction.
func (s *GameServer) executePlayerProjectileSkill(player *PlayerState, mapState *MapState, target Point) {
	playerStats := s.CalculateStats(player, mapState)

	projectileItemID := "atlas_pistol_mk2_bullet"
	if d, ok := s.entityDefaults["skill"]; ok && d.LiveItemID != "" {
		projectileItemID = d.LiveItemID
	}
	// Intelligence stat increases the chance of the skill activating.
	// We'll model this as a linear increase, capping the effective chance.
	if rand.Float64() >= math.Min(s.projectileSpawnChance+(playerStats.Intelligence/100.0), s.maxChance) {
		return // Skill did not activate on this action
	}

	// Calculate direction from player to target, as player.Direction might be NONE if idle.
	dx := target.X - player.Pos.X
	dy := target.Y - player.Pos.Y
	dist := math.Sqrt(dx*dx + dy*dy)

	var projectileDirection Direction
	if dist > 0 {
		dirX, dirY := dx/dist, dy/dist
		angle := math.Atan2(dirY, dirX)
		if angle < 0 {
			angle += 2 * math.Pi
		}
		directionIndex := (int(math.Round(angle/(math.Pi/4))) + 2) % 8
		projectileDirection = Direction(directionIndex)
	} else {
		// If target is same as player pos, use player's current direction as a fallback.
		projectileDirection = player.Direction
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	// Range stat increases the projectile's lifetime in milliseconds.
	projectileLifetime := (time.Duration(s.projectileLifetimeMs) * time.Millisecond) + (time.Duration(playerStats.Range) * time.Millisecond)
	projectileDims := Dimensions{Width: s.projectileWidth, Height: s.projectileHeight}
	projectileBaseLife := s.entityBaseMaxLife

	// Pre-compensate for the first tick's movement.
	// The projectile is created and then moved in the same game loop before being sent to the client.
	// By spawning it one "step" behind its intended origin, it will appear at the correct
	// location on the first frame it's rendered.
	projectileStep := (s.entityBaseSpeed * s.projectileSpeedMultiplier) / float64(s.fps)
	dirX, dirY := getDirectionVector(projectileDirection)

	// The desired position on the first rendered frame is the player's position.
	// The actual spawn position is one step behind this.
	spawnX := player.Pos.X - (dirX * projectileStep)
	spawnY := player.Pos.Y - (dirY * projectileStep)

	projectileBot := &BotState{
		ID:           uuid.New().String(),
		MapCode:      player.MapCode,
		Pos:          Point{X: spawnX, Y: spawnY},
		Dims:         projectileDims,
		Behavior:     "skill",
		Direction:    projectileDirection,
		ExpiresAt:    time.Now().Add(projectileLifetime),
		ObjectLayers: []ObjectLayerState{{ItemID: projectileItemID, Active: true, Quantity: 1}},
		CasterID:     player.ID,
		MaxLife:      projectileBaseLife,
		Life:         projectileBaseLife,
		Color:        s.colors["SKILL"],
	}

	mapState.bots[projectileBot.ID] = projectileBot
	// Apply resistance stat from caster and projectile's own items.
	s.ApplyResistanceStat(projectileBot, mapState)
	projectileBot.Life = projectileBot.MaxLife // Spawn with full life based on final MaxLife
}

// executeBotProjectileSkill contains the specific logic for the "atlas_pistol_mk2_logic" skill, triggered by a bot.
// It has a chance to spawn a temporary, fast-moving "skill" projectile bot towards the bot's target.
// NOTE: This is called from within updateBots, which is already under a server-wide mutex.
func (s *GameServer) executeBotProjectileSkill(bot *BotState, mapState *MapState, target Point) {
	botStats := s.CalculateStats(bot, mapState)

	projectileItemID := "atlas_pistol_mk2_bullet"
	if d, ok := s.entityDefaults["skill"]; ok && d.LiveItemID != "" {
		projectileItemID = d.LiveItemID
	}
	// Intelligence stat increases the chance of the skill activating.
	// We'll model this as a linear increase, capping the effective chance.
	if rand.Float64() >= math.Min(s.projectileSpawnChance+(botStats.Intelligence/100.0), s.maxChance) {
		return // Skill did not activate on this action
	}

	// Calculate direction from bot to its target.
	dx := target.X - bot.Pos.X
	dy := target.Y - bot.Pos.Y
	dist := math.Sqrt(dx*dx + dy*dy)

	var projectileDirection Direction
	if dist > 0 {
		dirX, dirY := dx/dist, dy/dist
		angle := math.Atan2(dirY, dirX)
		if angle < 0 {
			angle += 2 * math.Pi
		}
		directionIndex := (int(math.Round(angle/(math.Pi/4))) + 2) % 8
		projectileDirection = Direction(directionIndex)
	} else {
		// If target is same as bot pos, use bot's current direction as a fallback.
		projectileDirection = bot.Direction
	}

	// Range stat increases the projectile's lifetime in milliseconds.
	projectileLifetime := (time.Duration(s.projectileLifetimeMs) * time.Millisecond) + (time.Duration(botStats.Range) * time.Millisecond)
	projectileDims := Dimensions{Width: s.projectileWidth, Height: s.projectileHeight}
	projectileBaseLife := s.entityBaseMaxLife

	// Pre-compensate for the first tick's movement.
	// The projectile is created and then moved in the same game loop before being sent to the client.
	// By spawning it one "step" behind its intended origin, it will appear at the correct
	// location on the first frame it's rendered.
	projectileStep := (s.entityBaseSpeed * s.projectileSpeedMultiplier) / float64(s.fps)
	dirX, dirY := getDirectionVector(projectileDirection)

	// The desired position on the first rendered frame is the bot's position.
	// The actual spawn position is one step behind this.
	spawnX := bot.Pos.X - (dirX * projectileStep)
	spawnY := bot.Pos.Y - (dirY * projectileStep)

	projectileBot := &BotState{
		ID:           uuid.New().String(),
		MapCode:      bot.MapCode,
		Pos:          Point{X: spawnX, Y: spawnY},
		Dims:         projectileDims,
		Behavior:     "skill",
		Direction:    projectileDirection,
		ExpiresAt:    time.Now().Add(projectileLifetime),
		ObjectLayers: []ObjectLayerState{{ItemID: projectileItemID, Active: true, Quantity: 1}},
		CasterID:     bot.ID,
		MaxLife:      projectileBaseLife,
		Life:         projectileBaseLife,
		Color:        s.colors["SKILL"],
	}

	mapState.bots[projectileBot.ID] = projectileBot
	// Apply resistance stat from caster and projectile's own items.
	s.ApplyResistanceStat(projectileBot, mapState)
	projectileBot.Life = projectileBot.MaxLife // Spawn with full life based on final MaxLife
}
