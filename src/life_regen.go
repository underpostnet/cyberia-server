package game

import (
	"time"
)

// handleLifeRegeneration processes life regeneration for all players and bots in a given map.
// It should be called within the main game loop.
func (s *GameServer) handleLifeRegeneration(mapState *MapState) {
	now := time.Now()

	// Regenerate life for players
	for _, player := range mapState.players {
		// Skip dead players or players with no regen rate
		if player.IsGhost() || player.LifeTimeRegenRate <= 0 || player.Life >= player.MaxLife {
			continue
		}

		if now.After(player.NextRegenTime) {
			player.Life += player.LifeRegen
			if player.Life > player.MaxLife {
				player.Life = player.MaxLife
			}
			// Schedule the next regeneration
			player.NextRegenTime = now.Add(time.Duration(player.LifeTimeRegenRate * float64(time.Second)))
		}
	}

	// Regenerate life for bots
	for _, bot := range mapState.bots {
		// Skip dead bots or bots with no regen rate
		if bot.IsGhost() || bot.LifeTimeRegenRate <= 0 || bot.Life >= bot.MaxLife {
			continue
		}

		if now.After(bot.NextRegenTime) {
			bot.Life += bot.LifeRegen
			if bot.Life > bot.MaxLife {
				bot.Life = bot.MaxLife
			}
			// Schedule the next regeneration
			bot.NextRegenTime = now.Add(time.Duration(bot.LifeTimeRegenRate * float64(time.Second)))
		}
	}
}
