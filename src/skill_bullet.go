package game

import (
	"log"
	"math"
	"math/rand"
	"time"

	"github.com/google/uuid"
)

// executePlayerBulletSkill contains the specific logic for the "atlas_pistol_mk2_logic" skill.
// It has a chance to spawn a temporary, fast-moving "bullet" bot in the player's current direction.
func (s *GameServer) executePlayerBulletSkill(player *PlayerState, mapState *MapState, skillDef SkillDefinition, target Point) {
	playerStats := s.CalculateStats(player, mapState)

	if len(skillDef.ItemIDs) == 0 {
		log.Printf("Skill 'atlas_pistol_mk2_logic' triggered for player %s but no bullet ItemID was defined in SkillConfig.", player.ID)
		return
	}
	bulletItemID := skillDef.ItemIDs[0]
	const bulletSpawnChance = 0.25 // 25% chance
	// Intelligence stat increases the chance of the skill activating.
	// We'll model this as a linear increase, capping the effective chance at 95%.
	if rand.Float64() >= math.Min(bulletSpawnChance+(playerStats.Intelligence/100.0), 0.95) {
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

	// Range stat increases the bullet's lifetime in milliseconds.
	bulletLifetime := (5 * time.Second) + (time.Duration(playerStats.Range) * time.Millisecond)
	bulletDims := Dimensions{Width: 2, Height: 2}
	bulletBaseLife := s.entityBaseMaxLife

	// Pre-compensate for the first tick's movement.
	// The bullet is created and then moved in the same game loop before being sent to the client.
	// By spawning it one "step" behind its intended origin, it will appear at the correct
	// location on the first frame it's rendered.
	bulletStep := (s.entityBaseSpeed * 2.0) / (1000.0 / 100.0)
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
		ObjectLayers: []ObjectLayerState{{ItemID: bulletItemID, Active: true, Quantity: 1}},
		CasterID:     player.ID,
		MaxLife:      bulletBaseLife, // Initial MaxLife before resistance is applied
		Life:         bulletBaseLife,
	}

	mapState.bots[bulletBot.ID] = bulletBot
	// Apply resistance stat from caster and bullet's own items.
	s.ApplyResistanceStat(bulletBot, mapState)
	bulletBot.Life = bulletBot.MaxLife // Spawn with full life based on final MaxLife
	// log.Printf("Player %s triggered skill 'atlas_pistol_mk2_logic', spawning bullet bot %s", player.ID, bulletBot.ID)
}

// executeBotBulletSkill contains the specific logic for the "atlas_pistol_mk2_logic" skill, triggered by a bot.
// It has a chance to spawn a temporary, fast-moving "bullet" bot towards the bot's target.
// NOTE: This is called from within updateBots, which is already under a server-wide mutex.
func (s *GameServer) executeBotBulletSkill(bot *BotState, mapState *MapState, skillDef SkillDefinition, target Point) {
	botStats := s.CalculateStats(bot, mapState)

	if len(skillDef.ItemIDs) == 0 {
		log.Printf("Skill 'atlas_pistol_mk2_logic' triggered for bot %s but no bullet ItemID was defined in SkillConfig.", bot.ID)
		return
	}
	bulletItemID := skillDef.ItemIDs[0]
	const bulletSpawnChance = 0.25 // 25% chance
	// Intelligence stat increases the chance of the skill activating.
	// We'll model this as a linear increase, capping the effective chance at 95%.
	if rand.Float64() >= math.Min(bulletSpawnChance+(botStats.Intelligence/100.0), 0.95) {
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

	// Range stat increases the bullet's lifetime in milliseconds.
	bulletLifetime := (5 * time.Second) + (time.Duration(botStats.Range) * time.Millisecond)
	bulletDims := Dimensions{Width: 2, Height: 2}
	bulletBaseLife := s.entityBaseMaxLife

	// Pre-compensate for the first tick's movement.
	// The bullet is created and then moved in the same game loop before being sent to the client.
	// By spawning it one "step" behind its intended origin, it will appear at the correct
	// location on the first frame it's rendered.
	bulletStep := (s.entityBaseSpeed * 2.0) / (1000.0 / 100.0)
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
		ObjectLayers: []ObjectLayerState{{ItemID: bulletItemID, Active: true, Quantity: 1}},
		CasterID:     bot.ID,
		MaxLife:      bulletBaseLife, // Initial MaxLife before resistance is applied
		Life:         bulletBaseLife,
	}

	mapState.bots[bulletBot.ID] = bulletBot
	// Apply resistance stat from caster and bullet's own items.
	s.ApplyResistanceStat(bulletBot, mapState)
	bulletBot.Life = bulletBot.MaxLife // Spawn with full life based on final MaxLife
	// log.Printf("Bot %s triggered skill 'atlas_pistol_mk2_logic', spawning bullet bot %s", bot.ID, bulletBot.ID)
}
