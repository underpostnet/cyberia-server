package game

import (
	"math"
	"math/rand"
	"time"

	"github.com/google/uuid"
)

// executePlayerDoppelgangerSkill contains the specific logic for the "doppelganger" skill.
// It has a chance to spawn a temporary, wandering clone of the player.
func (s *GameServer) executePlayerDoppelgangerSkill(player *PlayerState, mapState *MapState, itemID string) {
	playerStats := s.CalculateStats(player, mapState)

	const doppelgangerSpawnChance = 0.25 // 25% chance
	// Intelligence stat increases the chance of the skill activating.
	if rand.Float64() >= math.Min(doppelgangerSpawnChance+(playerStats.Intelligence/100.0), 0.95) {
		return // Skill did not activate on this action
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	// Range stat increases the doppelganger's lifetime in milliseconds.
	botLifetime := (10 * time.Second) + (time.Duration(playerStats.Range) * time.Millisecond)

	// The doppelganger inherits the caster's object layers, but we add the skill item that triggered it.
	doppelgangerLayers := append(player.ObjectLayers, ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})

	doppelgangerBot := &BotState{
		ID:           uuid.New().String(),
		MapID:        player.MapID,
		Pos:          player.Pos,
		Dims:         player.Dims,
		Behavior:     "passive",
		SpawnCenter:  player.Pos,
		SpawnRadius:  5.0,
		ObjectLayers: doppelgangerLayers,
		ExpiresAt:    time.Now().Add(botLifetime),
		CasterID:     player.ID, // Mark the player as the caster
		MaxLife:      s.entityBaseMaxLife,
		Life:         s.entityBaseMaxLife * 0.5, // Set life to 50% of base max life
	}

	mapState.bots[doppelgangerBot.ID] = doppelgangerBot
	// Apply resistance stat from caster and the doppelganger's own items.
	s.ApplyResistanceStat(doppelgangerBot, mapState)
	doppelgangerBot.Life = doppelgangerBot.MaxLife // Spawn with full life
	// log.Printf("Player %s triggered skill 'doppelganger', spawning bot %s", player.ID, doppelgangerBot.ID)
}

// executeBotDoppelgangerSkill contains the specific logic for the "doppelganger" skill, triggered by a bot.
// It has a chance to spawn a temporary, wandering clone of the bot.
// NOTE: This is called from within updateBots, which is already under a server-wide mutex.
func (s *GameServer) executeBotDoppelgangerSkill(bot *BotState, mapState *MapState, itemID string) {
	botStats := s.CalculateStats(bot, mapState)

	const doppelgangerSpawnChance = 0.25 // 25% chance
	// Intelligence stat increases the chance of the skill activating.
	if rand.Float64() >= math.Min(doppelgangerSpawnChance+(botStats.Intelligence/100.0), 0.95) {
		return // Skill did not activate on this action
	}

	// Range stat increases the doppelganger's lifetime in milliseconds.
	botLifetime := (10 * time.Second) + (time.Duration(botStats.Range) * time.Millisecond)

	// The doppelganger inherits the caster's object layers, but we add the skill item that triggered it.
	doppelgangerLayers := append(bot.ObjectLayers, ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})

	doppelgangerBot := &BotState{
		ID:           uuid.New().String(),
		MapID:        bot.MapID,
		Pos:          bot.Pos,
		Dims:         bot.Dims,
		Behavior:     "passive",
		SpawnCenter:  bot.Pos,
		SpawnRadius:  5.0,
		ObjectLayers: doppelgangerLayers,
		ExpiresAt:    time.Now().Add(botLifetime),
		CasterID:     bot.ID, // Mark the bot as the caster
		MaxLife:      s.entityBaseMaxLife,
		Life:         s.entityBaseMaxLife * 0.5, // Set life to 50% of base max life
	}

	mapState.bots[doppelgangerBot.ID] = doppelgangerBot
	// Apply resistance stat from caster and the doppelganger's own items.
	s.ApplyResistanceStat(doppelgangerBot, mapState)
	doppelgangerBot.Life = doppelgangerBot.MaxLife // Spawn with full life
	// log.Printf("Bot %s triggered skill 'doppelganger', spawning bot %s", bot.ID, doppelgangerBot.ID)
}
