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
	"anon": {"doppelganger"},
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
					s.executeDoppelgangerSkill(player, mapState, layer.ItemID)
				// Future skills like "teleport_burst" or "invisibility" could be added here.
				default:
					log.Printf("Unknown skillID '%s' for item '%s'", skillID, layer.ItemID)
				}
			}
		}
	}
}

// executeDoppelgangerSkill contains the specific logic for the "doppelganger" skill.
// It has a chance to spawn a temporary, wandering clone of the player.
func (s *GameServer) executeDoppelgangerSkill(player *PlayerState, mapState *MapState, itemID string) {
	const doppelgangerSpawnChance = 0.25 // 25% chance
	if rand.Float64() >= doppelgangerSpawnChance {
		return // Skill did not activate on this action
	}

	s.mu.Lock()
	defer s.mu.Unlock()

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
	}

	mapState.bots[doppelgangerBot.ID] = doppelgangerBot
	log.Printf("Player %s triggered skill 'doppelganger', spawning bot %s", player.ID, doppelgangerBot.ID)
}
