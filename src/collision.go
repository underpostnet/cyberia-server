package game

import (
	"time"
)

const (
	respawnDuration = 10 * time.Second
	ghostItemID     = "ghost"
)

// checkAABBCollision checks for collision between two axis-aligned bounding boxes.
func checkAABBCollision(pos1 Point, dims1 Dimensions, pos2 Point, dims2 Dimensions) bool {
	// Check for overlap on X axis
	if pos1.X < pos2.X+dims2.Width && pos1.X+dims1.Width > pos2.X {
		// Check for overlap on Y axis
		if pos1.Y < pos2.Y+dims2.Height && pos1.Y+dims1.Height > pos2.Y {
			return true
		}
	}
	return false
}

// handleBulletCollisions manages collisions for 'bullet' behavior bots.
// It checks for intersections with players and other bots, applies damage,
// and handles death, removing the bullet upon impact.
func (s *GameServer) handleBulletCollisions(mapState *MapState) {
	bulletsToDelete := []string{}
	const collisionLifeLoss = 25.0 // Life lost by the bullet per tick of collision.

	for bulletID, bullet := range mapState.bots {
		if bullet.Behavior != "bullet" {
			continue
		}

		// Calculate bullet damage once.
		// This assumes s.objectLayerDataCache is available on the GameServer
		// and maps item IDs to their full ObjectLayer data.
		var bulletDamage float64
		for _, layer := range bullet.ObjectLayers {
			if data, ok := s.objectLayerDataCache[layer.ItemID]; ok {
				bulletDamage += float64(data.Data.Stats.Effect)
			}
		}
		if bulletDamage <= 0 {
			continue // No damage to deal.
		}

		var isColliding bool

		// Check collision with players
		for _, player := range mapState.players {
			if player.IsGhost() { // Don't hit dead/ghost players
				continue
			}
			// Bullets should not damage their caster.
			if player.ID == bullet.CasterID {
				continue
			}
			if checkAABBCollision(bullet.Pos, bullet.Dims, player.Pos, player.Dims) {
				player.Life -= bulletDamage
				if player.Life <= 0 {
					player.Life = 0
					s.handlePlayerDeath(player)
				}
				isColliding = true
			}
		}

		// Check collision with other bots
		for otherBotID, otherBot := range mapState.bots {
			if bulletID == otherBotID || otherBot.Behavior == "bullet" || otherBot.IsGhost() {
				continue // Don't collide with self, other bullets, or dead bots.
			}
			// Bullets should not damage their caster.
			if otherBotID == bullet.CasterID {
				continue
			}
			if checkAABBCollision(bullet.Pos, bullet.Dims, otherBot.Pos, otherBot.Dims) {
				otherBot.Life -= bulletDamage
				if otherBot.Life <= 0 {
					otherBot.Life = 0
					s.handleBotDeath(otherBot)
				}
				isColliding = true
			}
		}

		// If the bullet is colliding, it loses life.
		if isColliding {
			bullet.Life -= collisionLifeLoss
			if bullet.Life <= 0 {
				// Bullet is "dead" from collision damage, mark for deletion.
				bulletsToDelete = append(bulletsToDelete, bulletID)
			}
		}
	}

	for _, id := range bulletsToDelete {
		delete(mapState.bots, id)
	}
}

// handlePlayerDeath sets a player to a dead/ghost state.
func (s *GameServer) handlePlayerDeath(player *PlayerState) {
	player.PreRespawnObjectLayers = make([]ObjectLayerState, len(player.ObjectLayers))
	copy(player.PreRespawnObjectLayers, player.ObjectLayers)

	player.ObjectLayers = []ObjectLayerState{{ItemID: ghostItemID, Active: true, Quantity: 1}}
	player.RespawnTime = time.Now().Add(respawnDuration)
	player.Mode = IDLE // Stop movement
}

// handleBotDeath sets a bot to a dead state.
func (s *GameServer) handleBotDeath(bot *BotState) {
	bot.PreRespawnObjectLayers = make([]ObjectLayerState, len(bot.ObjectLayers))
	copy(bot.PreRespawnObjectLayers, bot.ObjectLayers)

	bot.ObjectLayers = []ObjectLayerState{{ItemID: ghostItemID, Active: true, Quantity: 1}}
	bot.RespawnTime = time.Now().Add(respawnDuration)
	bot.Mode = IDLE // Stop movement
}

// handleRespawns checks and respawns dead players and bots.
func (s *GameServer) handleRespawns(mapState *MapState) {
	// Respawn players
	for _, player := range mapState.players {
		if !player.RespawnTime.IsZero() && time.Now().After(player.RespawnTime) {
			player.Life = player.MaxLife
			player.ObjectLayers = player.PreRespawnObjectLayers
			player.PreRespawnObjectLayers = nil
			player.RespawnTime = time.Time{}
		}
	}

	// Respawn bots
	for _, bot := range mapState.bots {
		if !bot.RespawnTime.IsZero() && time.Now().After(bot.RespawnTime) {
			bot.Life = bot.MaxLife
			bot.ObjectLayers = bot.PreRespawnObjectLayers
			bot.PreRespawnObjectLayers = nil
			bot.RespawnTime = time.Time{}
		}
	}
}

// IsGhost checks if a player is in a ghost state (i.e., dead and waiting to respawn).
func (p *PlayerState) IsGhost() bool {
	return !p.RespawnTime.IsZero()
}

// IsGhost checks if a bot is in a ghost state (i.e., dead and waiting to respawn).
func (b *BotState) IsGhost() bool {
	return !b.RespawnTime.IsZero()
}
