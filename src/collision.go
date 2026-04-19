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
			// Frozen players (in modal interaction) cannot be damaged.
			if player.Frozen {
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
				// FCT: show the hit as a red flying number at the collision point.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					sendFCT(player, FCTTypeDamage,
						projectile.Pos.X+projectile.Dims.Width*0.5,
						projectile.Pos.Y+projectile.Dims.Height*0.5, dmg)
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

		// Check collision with resources
		for _, res := range mapState.resources {
			if res.IsGhost() {
				continue // Don't collide with destroyed/respawning resources.
			}
			if checkAABBCollision(projectile.Pos, projectile.Dims, res.Pos, res.Dims) {
				res.Life -= projectileStats.Effect
				if res.Life <= 0 {
					res.Life = 0
					s.handleResourceDeath(res, projectile, mapState)
				}
				// FCT: show the hit as a red flying number at the collision point.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					// Send FCT to the caster if they are a player.
					if p, ok := mapState.players[projectile.CasterID]; ok {
						sendFCT(p, FCTTypeDamage,
							res.Pos.X+res.Dims.Width*0.5,
							res.Pos.Y+res.Dims.Height*0.5, dmg)
					}
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
	// Economy: apply respawn-cost sink before saving layers (disabled by default, rate = 0).
	s.SinkRespawnCost(player)

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

// handleResourceDeath sets a resource to a destroyed state and transfers its
// object layers to the caster (the player or bot that fired the killing projectile).
// Resources have NO dead item IDs — they simply deactivate all OLs when destroyed.
func (s *GameServer) handleResourceDeath(res *ResourceState, killerProjectile *BotState, mapState *MapState) {
	// Save the pre-destruction object layers for restoration on respawn.
	layersToSave := make([]ObjectLayerState, len(res.ObjectLayers))
	copy(layersToSave, res.ObjectLayers)
	res.PreRespawnObjectLayers = layersToSave

	// Transfer resource OLs to the caster (player or bot).
	if killerProjectile.CasterID != "" {
		if casterPlayer, ok := mapState.players[killerProjectile.CasterID]; ok {
			s.transferResourceLayers(res, casterPlayer, mapState)
		} else if casterBot, ok := mapState.bots[killerProjectile.CasterID]; ok {
			s.transferResourceLayersToBot(res, casterBot)
		}
	}

	// Deactivate all existing object layers — no dead sprite for resources.
	for i := range res.ObjectLayers {
		res.ObjectLayers[i].Active = false
	}

	res.RespawnTime = time.Now().Add(s.respawnDuration)
	s.InvalidateStats(res)
}

// transferResourceLayers transfers the active OLs of a destroyed resource to
// a player.  Each active layer is added to the player's inventory (quantity
// incremented if already owned, otherwise appended).  FCT events are sent.
func (s *GameServer) transferResourceLayers(res *ResourceState, player *PlayerState, mapState *MapState) {
	for _, layer := range res.ObjectLayers {
		if !layer.Active || layer.Quantity <= 0 {
			continue
		}
		found := false
		for i := range player.ObjectLayers {
			if player.ObjectLayers[i].ItemID == layer.ItemID {
				player.ObjectLayers[i].Quantity += layer.Quantity
				found = true
				break
			}
		}
		if !found {
			player.ObjectLayers = append(player.ObjectLayers, ObjectLayerState{
				ItemID:   layer.ItemID,
				Active:   false, // added to inventory, not auto-equipped
				Quantity: layer.Quantity,
			})
		}
		// FCT: show item gain
		sendItemFCT(player, FCTTypeItemGain,
			res.Pos.X+res.Dims.Width*0.5,
			res.Pos.Y+res.Dims.Height*0.5,
			layer.Quantity, layer.ItemID)
	}
	s.InvalidateStats(player)
}

// transferResourceLayersToBot transfers the active OLs of a destroyed resource
// to a bot (for bot-triggered resource destruction).
func (s *GameServer) transferResourceLayersToBot(res *ResourceState, bot *BotState) {
	for _, layer := range res.ObjectLayers {
		if !layer.Active || layer.Quantity <= 0 {
			continue
		}
		found := false
		for i := range bot.ObjectLayers {
			if bot.ObjectLayers[i].ItemID == layer.ItemID {
				bot.ObjectLayers[i].Quantity += layer.Quantity
				found = true
				break
			}
		}
		if !found {
			bot.ObjectLayers = append(bot.ObjectLayers, ObjectLayerState{
				ItemID:   layer.ItemID,
				Active:   false,
				Quantity: layer.Quantity,
			})
		}
	}
	s.InvalidateStats(bot)
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

			// Reset coin supply via the Fountain — single source of truth.
			s.FountainInitBot(bot)
		}
	}

	// Respawn resources — restore original OLs and reset life.
	for _, res := range mapState.resources {
		if !res.RespawnTime.IsZero() && time.Now().After(res.RespawnTime) {
			res.ObjectLayers = res.PreRespawnObjectLayers
			res.PreRespawnObjectLayers = nil
			s.InvalidateStats(res)
			s.ApplyResistanceStat(res, mapState)
			res.Life = res.MaxLife
			res.RespawnTime = time.Time{}
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
