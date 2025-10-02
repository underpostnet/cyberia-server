package game

import (
	"math/rand"
	"time"

	"github.com/google/uuid"
)

// executePlayerDoppelgangerSkill contains the specific logic for the "doppelganger" skill.
// It has a chance to spawn a temporary, wandering clone of the player.
func (s *GameServer) executePlayerDoppelgangerSkill(player *PlayerState, mapState *MapState, itemID string) {
	const doppelgangerSpawnChance = 0.25 // 25% chance
	if rand.Float64() >= doppelgangerSpawnChance {
		return // Skill did not activate on this action
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	maxLife := s.entityBaseMaxLife
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

// executeBotDoppelgangerSkill contains the specific logic for the "doppelganger" skill, triggered by a bot.
// It has a chance to spawn a temporary, wandering clone of the bot.
// NOTE: This is called from within updateBots, which is already under a server-wide mutex.
func (s *GameServer) executeBotDoppelgangerSkill(bot *BotState, mapState *MapState, itemID string) {
	const doppelgangerSpawnChance = 0.25 // 25% chance
	if rand.Float64() >= doppelgangerSpawnChance {
		return // Skill did not activate on this action
	}

	maxLife := s.entityBaseMaxLife
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
