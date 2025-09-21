package game

import (
	"log"
	"math"
	"math/rand"
	"time"

	"github.com/google/uuid"
)

// SkillConfig maps an ItemID to a list of SkillIDs that can be triggered.
// This provides a declarative way to associate items with behaviors.
var SkillConfig = map[string][]string{
	// "anon":             {"doppelganger"},
	"atlas_pistol_mk2": {"atlas_pistol_mk2_bullet"},
	"coin":             {"coin_drop_or_transaction"},
	// "purple": {"doppelganger"},
}

// HandlePlayerActionSkills checks if a player's action triggers any skills
// based on their active items and calls the corresponding skill logic.
func (s *GameServer) HandlePlayerActionSkills(player *PlayerState, mapState *MapState, target Point) {
	for _, layer := range player.ObjectLayers {
		if !layer.Active {
			continue
		}

		// Check if the active item has any associated skills in our config
		if skillIDs, ok := SkillConfig[layer.ItemID]; ok {
			for _, skillID := range skillIDs {
				// Execute the skill logic based on its ID
				switch skillID {
				case "doppelganger":
					s.executePlayerDoppelgangerSkill(player, mapState, layer.ItemID)
				case "atlas_pistol_mk2_bullet":
					s.executePlayerBulletSkill(player, mapState, layer.ItemID, target)
				// Future skills like "teleport_burst" or "invisibility" could be added here.
				default:
					log.Printf("Unknown skillID '%s' for item '%s'", skillID, layer.ItemID)
				}
			}
		}
	}
}

// executePlayerDoppelgangerSkill contains the specific logic for the "doppelganger" skill.
// It has a chance to spawn a temporary, wandering clone of the player.
func (s *GameServer) executePlayerDoppelgangerSkill(player *PlayerState, mapState *MapState, itemID string) {
	const doppelgangerSpawnChance = 0.25 // 25% chance
	if rand.Float64() >= doppelgangerSpawnChance {
		return // Skill did not activate on this action
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	maxLife := float64(rand.Intn(151) + 50) // Random between 50 and 200
	botLifetime := 10 * time.Second

	doppelgangerBot := &BotState{
		ID:           uuid.New().String(),
		MapID:        player.MapID,
		Pos:          player.Pos,
		Dims:         player.Dims,
		Behavior:     "passive",
		SpawnCenter:  player.Pos,
		SpawnRadius:  5.0,
		ObjectLayers: []ObjectLayerState{{ItemID: itemID, Active: true, Quantity: 1}},
		ExpiresAt:    time.Now().Add(botLifetime),
		MaxLife:      maxLife,
		Life:         maxLife * 0.5, // Set life to 50% of max life
	}

	mapState.bots[doppelgangerBot.ID] = doppelgangerBot
	// log.Printf("Player %s triggered skill 'doppelganger', spawning bot %s", player.ID, doppelgangerBot.ID)
}

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

// handleBotSkills checks and executes skills for a bot when it takes an "action" (like finding a new path).
func (s *GameServer) handleBotSkills(bot *BotState, mapState *MapState, target Point) {
	for _, layer := range bot.ObjectLayers {
		if !layer.Active {
			continue
		}

		// Check if the active item has any associated skills in our config
		if skillIDs, ok := SkillConfig[layer.ItemID]; ok {
			for _, skillID := range skillIDs {
				// Execute the skill logic based on its ID
				switch skillID {
				case "doppelganger":
					s.executeBotDoppelgangerSkill(bot, mapState, layer.ItemID)
				case "atlas_pistol_mk2_bullet":
					s.executeBotBulletSkill(bot, mapState, layer.ItemID, target)
				// Future skills could be added here.
				default:
					log.Printf("Unknown skillID '%s' for item '%s' on bot %s", skillID, layer.ItemID, bot.ID)
				}
			}
		}
	}
}

// executeBotDoppelgangerSkill contains the specific logic for the "doppelganger" skill, triggered by a bot.
// It has a chance to spawn a temporary, wandering clone of the bot.
// NOTE: This is called from within updateBots, which is already under a server-wide mutex.
func (s *GameServer) executeBotDoppelgangerSkill(bot *BotState, mapState *MapState, itemID string) {
	const doppelgangerSpawnChance = 0.25 // 25% chance
	if rand.Float64() >= doppelgangerSpawnChance {
		return // Skill did not activate on this action
	}

	maxLife := float64(rand.Intn(151) + 50) // Random between 50 and 200
	botLifetime := 10 * time.Second

	doppelgangerBot := &BotState{
		ID:           uuid.New().String(),
		MapID:        bot.MapID,
		Pos:          bot.Pos,
		Dims:         bot.Dims,
		Behavior:     "passive",
		SpawnCenter:  bot.Pos,
		SpawnRadius:  5.0,
		ObjectLayers: []ObjectLayerState{{ItemID: itemID, Active: true, Quantity: 1}},
		ExpiresAt:    time.Now().Add(botLifetime),
		MaxLife:      maxLife,
		Life:         maxLife * 0.5, // Set life to 50% of max life
	}

	mapState.bots[doppelgangerBot.ID] = doppelgangerBot
	// log.Printf("Bot %s triggered skill 'doppelganger', spawning bot %s", bot.ID, doppelgangerBot.ID)
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

// HandleOnKillSkills checks for skills that trigger when an entity is killed by a bullet,
// and handles the core mechanic of coin transfer.
func (s *GameServer) HandleOnKillSkills(killerBullet *BotState, victim interface{}, mapState *MapState) { // The killer is a bullet bot. The actual killer is the caster of the bullet.
	// The killer is a bullet bot. The actual killer is the caster of the bullet.
	if killerBullet.CasterID == "" {
		return
	}

	var caster interface{}

	// Find the caster, which can be a player or a bot.
	if p, ok := mapState.players[killerBullet.CasterID]; ok {
		caster = p
	} else if b, ok := mapState.bots[killerBullet.CasterID]; ok {
		caster = b
	} else {
		return // Caster not found in the map.
	}

	// --- Core Mechanic: Coin Transfer on Kill ---
	// This is not a skill, but a fundamental rule: the victor loots the vanquished.
	s.executeCoinDropOnKill(caster, victim, s)

}

// executeCoinDropOnKill handles the logic for transferring coins from a killed entity to the killer.
// This is a core mechanic, not a conditional skill.
func (s *GameServer) executeCoinDropOnKill(caster interface{}, victim interface{}, server *GameServer) {
	// 1. Find the victim's coin layer and check if they have coins.
	var victimCoinLayer *ObjectLayerState
	var victimID string
	var victimObjectLayers []ObjectLayerState

	switch v := victim.(type) {
	case *PlayerState:
		victimID = v.ID
		victimObjectLayers = v.ObjectLayers
	case *BotState:
		victimID = v.ID
		victimObjectLayers = v.ObjectLayers
	default:
		log.Printf("executeCoinDropOnKill called with unknown victim type")
		return
	}

	for i := range victimObjectLayers {
		if victimObjectLayers[i].ItemID == "coin" {
			victimCoinLayer = &victimObjectLayers[i]
			break
		}
	}

	if victimCoinLayer == nil || victimCoinLayer.Quantity <= 0 {
		return // Victim has no coins to drop.
	}

	// 2. Find or create the caster's coin layer.
	var casterCoinLayer *ObjectLayerState
	var casterID string

	casterCoinLayer, casterID = s.findOrCreateCoinLayer(caster)
	if casterCoinLayer == nil {
		log.Printf("executeCoinDropOnKill failed to find or create coin layer for caster")
		return
	}

	// 3. Perform the transaction.
	amountToTransfer := victimCoinLayer.Quantity
	casterCoinLayer.Quantity += amountToTransfer
	victimCoinLayer.Quantity = 0

	log.Printf("Caster %s triggered 'coin_drop_or_transaction', looting %d coins from %s.", casterID, amountToTransfer, victimID)

}

// findOrCreateCoinLayer finds the coin layer for a given entity (player or bot).
// If the entity doesn't have a coin layer, it creates one.
// It returns the coin layer and the entity's ID.
func (s *GameServer) findOrCreateCoinLayer(entity interface{}) (*ObjectLayerState, string) {
	switch e := entity.(type) {
	case *PlayerState:
		for i := range e.ObjectLayers {
			if e.ObjectLayers[i].ItemID == "coin" {
				return &e.ObjectLayers[i], e.ID
			}
		}
		// If player doesn't have a coin layer, add one.
		e.ObjectLayers = append(e.ObjectLayers, ObjectLayerState{ItemID: "coin", Active: false, Quantity: 0})
		return &e.ObjectLayers[len(e.ObjectLayers)-1], e.ID
	case *BotState:
		for i := range e.ObjectLayers {
			if e.ObjectLayers[i].ItemID == "coin" {
				return &e.ObjectLayers[i], e.ID
			}
		}
		// If bot doesn't have a coin layer, add one.
		e.ObjectLayers = append(e.ObjectLayers, ObjectLayerState{ItemID: "coin", Active: false, Quantity: 0})
		return &e.ObjectLayers[len(e.ObjectLayers)-1], e.ID
	default:
		return nil, ""
	}
}
