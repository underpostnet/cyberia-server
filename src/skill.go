package game

import (
	"log"
	"math/rand"
	"time"

	"github.com/google/uuid"
)

// SkillConfig maps an ItemID to a list of SkillIDs that can be triggered.
// This provides a declarative way to associate items with behaviors.
var SkillConfig = map[string][]string{
	"anon":   {"doppelganger"},
	"purple": {"doppelganger"},
}

// HandlePlayerActionSkills checks if a player's action triggers any skills
// based on their active items and calls the corresponding skill logic.
func (s *GameServer) HandlePlayerActionSkills(player *PlayerState, mapState *MapState) {
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
		ObjectLayers: []ObjectLayerState{{ItemID: itemID, Active: true}},
		ExpiresAt:    time.Now().Add(botLifetime),
		MaxLife:      maxLife,
		Life:         maxLife * 0.5, // Set life to 50% of max life
	}

	mapState.bots[doppelgangerBot.ID] = doppelgangerBot
	// log.Printf("Player %s triggered skill 'doppelganger', spawning bot %s", player.ID, doppelgangerBot.ID)
}

// handleBotSkills checks and executes skills for a bot when it takes an "action" (like finding a new path).
func (s *GameServer) handleBotSkills(bot *BotState, mapState *MapState) {
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
		ObjectLayers: []ObjectLayerState{{ItemID: itemID, Active: true}},
		ExpiresAt:    time.Now().Add(botLifetime),
		MaxLife:      maxLife,
		Life:         maxLife * 0.5, // Set life to 50% of max life
	}

	mapState.bots[doppelgangerBot.ID] = doppelgangerBot
	// log.Printf("Bot %s triggered skill 'doppelganger', spawning bot %s", bot.ID, doppelgangerBot.ID)
}
