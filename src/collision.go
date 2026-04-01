package game

import (
	"time"
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

// handleSkillCollisions manages collisions for 'skill' behavior bots (projectiles).
// It checks for intersections with players and other bots, applies damage,
// and handles death, removing the projectile upon impact.
func (s *GameServer) handleSkillCollisions(mapState *MapState) {
	projectilesToDelete := []string{}

	for projectileID, projectile := range mapState.bots {
		if projectile.Behavior != "skill" {
			continue
		}

		projectileStats := s.CalculateStats(projectile, mapState)
		if projectileStats.Effect <= 0 {
			continue // No damage to deal.
		}

		var isColliding bool

		// Check collision with players
		for _, player := range mapState.players {
			if player.IsGhost() { // Don't hit dead/ghost players
				continue
			}
			// Projectiles should not damage their caster.
			if player.ID == projectile.CasterID {
				continue
			}
			if checkAABBCollision(projectile.Pos, projectile.Dims, player.Pos, player.Dims) {
				player.Life -= projectileStats.Effect
				if player.Life <= 0 {
					player.Life = 0
					s.HandleOnKillSkills(projectile, player, mapState)
					s.handlePlayerDeath(player)
				}

				// "Thorns" effect: The player's Effect stat damages the projectile.
				playerStats := s.CalculateStats(player, mapState)
				if playerStats.Effect > 0 {
					projectile.Life -= playerStats.Effect
				}

				isColliding = true
			}
		}

		// Check collision with other bots
		for otherBotID, otherBot := range mapState.bots {
			if projectileID == otherBotID || otherBot.Behavior == "skill" || otherBot.IsGhost() {
				continue // Don't collide with self, other projectiles, or dead bots.
			}
			// Projectiles should not damage their caster.
			if otherBotID == projectile.CasterID {
				continue
			}
			if checkAABBCollision(projectile.Pos, projectile.Dims, otherBot.Pos, otherBot.Dims) {
				otherBot.Life -= projectileStats.Effect
				if otherBot.Life <= 0 {
					otherBot.Life = 0
					s.HandleOnKillSkills(projectile, otherBot, mapState)
					s.handleBotDeath(otherBot)
				}

				// "Thorns" effect: The other bot's Effect stat damages the projectile.
				otherBotStats := s.CalculateStats(otherBot, mapState)
				if otherBotStats.Effect > 0 {
					projectile.Life -= otherBotStats.Effect
				}

				isColliding = true
			}
		}

		// If the projectile is colliding, it loses life.
		if isColliding {
			projectile.Life -= s.collisionLifeLoss
			if projectile.Life <= 0 {
				// Projectile is "dead" from collision damage, mark for deletion.
				projectilesToDelete = append(projectilesToDelete, projectileID)
			}
		}
	}

	for _, id := range projectilesToDelete {
		delete(mapState.bots, id)
	}
}

// handlePlayerDeath sets a player to a dead/ghost state.
func (s *GameServer) handlePlayerDeath(player *PlayerState) {
	// Make a definitive copy of the object layers *after* on-kill effects (like coin transfer) have been applied.
	// This state will be restored on respawn.
	layersToSave := make([]ObjectLayerState, len(player.ObjectLayers))
	copy(layersToSave, player.ObjectLayers)
	player.PreRespawnObjectLayers = layersToSave

	// Deactivate all existing object layers
	for i := range player.ObjectLayers {
		player.ObjectLayers[i].Active = false
	}

	// Activate all dead item IDs from entity defaults
	if d, ok := s.entityDefaults["player"]; ok {
		for _, deadID := range d.DeadItemIDs {
			found := false
			for i := range player.ObjectLayers {
				if player.ObjectLayers[i].ItemID == deadID {
					player.ObjectLayers[i].Active = true
					player.ObjectLayers[i].Quantity = 1
					found = true
					break
				}
			}
			if !found {
				player.ObjectLayers = append(player.ObjectLayers, ObjectLayerState{ItemID: deadID, Active: true, Quantity: 1})
			}
		}
	}

	player.RespawnTime = time.Now().Add(s.respawnDuration)
	// Recalculate stats for the ghost state to ensure consistency.
	// This will primarily affect MaxLife.
	s.InvalidateStats(player)
	s.ApplyResistanceStat(player, s.maps[player.MapCode]) // This assumes player.MapCode is correct and map exists.
	player.Mode = IDLE                                  // Stop movement
}

// handleBotDeath sets a bot to a dead state.
func (s *GameServer) handleBotDeath(bot *BotState) {
	// Make a definitive copy of the object layers *after* on-kill effects have been applied.
	layersToSave := make([]ObjectLayerState, len(bot.ObjectLayers))
	copy(layersToSave, bot.ObjectLayers)
	bot.PreRespawnObjectLayers = layersToSave

	// Deactivate all existing object layers
	for i := range bot.ObjectLayers {
		bot.ObjectLayers[i].Active = false
	}

	// Activate all dead item IDs from entity defaults
	if d, ok := s.entityDefaults["bot"]; ok {
		for _, deadID := range d.DeadItemIDs {
			found := false
			for i := range bot.ObjectLayers {
				if bot.ObjectLayers[i].ItemID == deadID {
					bot.ObjectLayers[i].Active = true
					bot.ObjectLayers[i].Quantity = 1
					found = true
					break
				}
			}
			if !found {
				bot.ObjectLayers = append(bot.ObjectLayers, ObjectLayerState{ItemID: deadID, Active: true, Quantity: 1})
			}
		}
	}

	bot.RespawnTime = time.Now().Add(s.respawnDuration)
	// Recalculate stats for the ghost state to ensure consistency.
	// This will primarily affect MaxLife.
	s.InvalidateStats(bot)
	s.ApplyResistanceStat(bot, s.maps[bot.MapCode])
	bot.Mode = IDLE // Stop movement
}

// handleRespawns checks and respawns dead players and bots.
func (s *GameServer) handleRespawns(mapState *MapState) {
	// Respawn players
	for _, player := range mapState.players {
		if !player.RespawnTime.IsZero() && time.Now().After(player.RespawnTime) {
			player.ObjectLayers = player.PreRespawnObjectLayers
			player.PreRespawnObjectLayers = nil
			s.InvalidateStats(player)
			s.ApplyResistanceStat(player, mapState) // Recalculate stats with restored items.
			player.Life = player.MaxLife
			player.RespawnTime = time.Time{}
		}
	}

	// Respawn bots
	for _, bot := range mapState.bots {
		if !bot.RespawnTime.IsZero() && time.Now().After(bot.RespawnTime) {
			bot.ObjectLayers = bot.PreRespawnObjectLayers
			bot.PreRespawnObjectLayers = nil
			s.InvalidateStats(bot)
			s.ApplyResistanceStat(bot, mapState) // Recalculate stats with restored items.
			bot.Life = bot.MaxLife
			bot.RespawnTime = time.Time{}

			// Reset coin quantity for respawned bots.
			var coinLayerFound bool
			for i := range bot.ObjectLayers {
				if bot.ObjectLayers[i].ItemID == s.coinItemID {
					bot.ObjectLayers[i].Quantity = s.defaultCoinQuantity
					coinLayerFound = true
					break
				}
			}

			if !coinLayerFound {
				// If the bot somehow lost its coin layer, add it back on respawn.
				bot.ObjectLayers = append(bot.ObjectLayers, ObjectLayerState{ItemID: s.coinItemID, Active: false, Quantity: s.defaultCoinQuantity})
			}
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

// isDeadItemID returns true if itemID is among the DeadItemIDs for the given entity type.
func (s *GameServer) isDeadItemID(entityType, itemID string) bool {
	if d, ok := s.entityDefaults[entityType]; ok {
		for _, id := range d.DeadItemIDs {
			if id == itemID {
				return true
			}
		}
	}
	return false
}
