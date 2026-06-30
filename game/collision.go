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
		if projectile.Behavior != BehaviorSkill {
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
			if projectileID == otherBotID || otherBot.Behavior == BehaviorSkill || otherBot.IsGhost() {
				continue // Don't collide with self, other projectiles, or dead bots.
			}
			// Provider NPCs are immortal — projectiles pass through without damage.
			if behaviorIsImmortal(otherBot.Behavior) {
				continue
			}
			// Projectiles should not damage their caster.
			if otherBotID == projectile.CasterID {
				continue
			}
			if checkAABBCollision(projectile.Pos, projectile.Dims, otherBot.Pos, otherBot.Dims) {
				otherBot.Life -= projectileStats.Effect
				if otherBot.Life <= 0 {
					otherBot.Life = 0
					// Capture the victim's skin before death deactivates its
					// layers — quest `kill` objectives match on it.
					killedSkin := s.botActiveSkin(otherBot.ID)
					s.HandleOnKillSkills(projectile, otherBot, mapState)
					s.handleBotDeath(otherBot, projectile, mapState)
					if killer, ok := mapState.players[projectile.CasterID]; ok {
						s.advancePlayerQuestsOnKill(killer, killedSkin)
					}
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

// applyDeadItems deactivates every layer and (re)activates the configured dead
// item IDs, appending any that are missing. Shared by every entity death path so
// the dead/ghost visual swap is identical for players, bots, and resources.
func applyDeadItems(layers []ObjectLayerState, deadItemIDs []string) []ObjectLayerState {
	for i := range layers {
		layers[i].Active = false
	}
	for _, deadID := range deadItemIDs {
		if deadID == "" {
			continue
		}
		found := false
		for i := range layers {
			if layers[i].ItemID == deadID {
				layers[i].Active = true
				layers[i].Quantity = 1
				found = true
				break
			}
		}
		if !found {
			layers = append(layers, ObjectLayerState{ItemID: deadID, Active: true, Quantity: 1})
		}
	}
	return layers
}

// grantDropItemsToPlayer adds drop items to a player's inventory (quantity +1,
// appended inactive if new) and emits an item-gain FCT at (cx, cy).
func (s *GameServer) grantDropItemsToPlayer(player *PlayerState, dropItemIDs []string, cx, cy float64) {
	for _, itemID := range dropItemIDs {
		if itemID == "" {
			continue
		}
		found := false
		for i := range player.ObjectLayers {
			if player.ObjectLayers[i].ItemID == itemID {
				player.ObjectLayers[i].Quantity += 1
				found = true
				break
			}
		}
		if !found {
			player.ObjectLayers = append(player.ObjectLayers, ObjectLayerState{ItemID: itemID, Active: false, Quantity: 1})
		}
		sendItemFCT(player, FCTTypeItemGain, cx, cy, 1, itemID)
	}
	s.InvalidateStats(player)
}

// grantDropItemsToBot adds drop items to a bot's inventory (no FCT — bots have no
// HUD).
func (s *GameServer) grantDropItemsToBot(bot *BotState, dropItemIDs []string) {
	for _, itemID := range dropItemIDs {
		if itemID == "" {
			continue
		}
		found := false
		for i := range bot.ObjectLayers {
			if bot.ObjectLayers[i].ItemID == itemID {
				bot.ObjectLayers[i].Quantity += 1
				found = true
				break
			}
		}
		if !found {
			bot.ObjectLayers = append(bot.ObjectLayers, ObjectLayerState{ItemID: itemID, Active: false, Quantity: 1})
		}
	}
	s.InvalidateStats(bot)
}

// grantDropItemsToCaster transfers a dying entity's drop items to whoever fired
// the killing projectile: a player (inventory + item FCT at (cx, cy) + collect
// quest gain) or a bot (inventory only). No-op without drops or a caster.
func (s *GameServer) grantDropItemsToCaster(mapState *MapState, casterID string, dropItemIDs []string, cx, cy float64) {
	if len(dropItemIDs) == 0 || casterID == "" {
		return
	}
	if player, ok := mapState.players[casterID]; ok {
		s.grantDropItemsToPlayer(player, dropItemIDs, cx, cy)
		s.advancePlayerQuestsOnGain(player)
		return
	}
	if bot, ok := mapState.bots[casterID]; ok {
		s.grantDropItemsToBot(bot, dropItemIDs)
	}
}

// handlePlayerDeath sets a player to a dead/ghost state using the dead items of
// the entity-type default resolved from its pre-death active items.
func (s *GameServer) handlePlayerDeath(player *PlayerState) {
	// Economy: apply respawn-cost sink before saving layers (disabled by default, rate = 0).
	s.SinkRespawnCost(player)

	// Snapshot the pre-death layers (restored on respawn) and resolve the build
	// from them, before the death swap deactivates the live items.
	layersToSave := make([]ObjectLayerState, len(player.ObjectLayers))
	copy(layersToSave, player.ObjectLayers)
	player.PreRespawnObjectLayers = layersToSave
	build, _ := s.resolveEntityDefaultBuild("player", activeObjectLayerItemIDs(layersToSave))

	player.ObjectLayers = applyDeadItems(player.ObjectLayers, build.DeadItemIDs)

	player.RespawnTime = time.Now().Add(s.respawnDuration)
	// Recalculate stats for the ghost state to ensure consistency (mainly MaxLife).
	s.InvalidateStats(player)
	s.ApplyResistanceStat(player, s.maps[player.MapCode])
	player.Mode = IDLE // Stop movement
}

// handleBotDeath sets a bot to its dead state and transfers its drop items to the
// killer. Both the dead visuals and the drops come from the entity-type default
// resolved by the bot's pre-death active items (most-specific subset match), so a
// skin can carry different dead/drop sets depending on its full active set.
func (s *GameServer) handleBotDeath(bot *BotState, killerProjectile *BotState, mapState *MapState) {
	layersToSave := make([]ObjectLayerState, len(bot.ObjectLayers))
	copy(layersToSave, bot.ObjectLayers)
	bot.PreRespawnObjectLayers = layersToSave
	build, _ := s.resolveEntityDefaultBuild("bot", activeObjectLayerItemIDs(layersToSave))

	if killerProjectile != nil {
		s.grantDropItemsToCaster(mapState, killerProjectile.CasterID, build.DropItemIDs,
			bot.Pos.X+bot.Dims.Width*0.5, bot.Pos.Y+bot.Dims.Height*0.5)
	}

	bot.ObjectLayers = applyDeadItems(bot.ObjectLayers, build.DeadItemIDs)

	bot.RespawnTime = time.Now().Add(s.respawnDuration)
	// Recalculate stats for the ghost state to ensure consistency (mainly MaxLife).
	s.InvalidateStats(bot)
	s.ApplyResistanceStat(bot, s.maps[bot.MapCode])
	bot.Mode = IDLE // Stop movement
}

// handleResourceDeath sets a resource to a destroyed state, transfers its drop
// items to the extractor, and activates the dead/extracted visuals until respawn.
func (s *GameServer) handleResourceDeath(res *ResourceState, killerProjectile *BotState, mapState *MapState) {
	layersToSave := make([]ObjectLayerState, len(res.ObjectLayers))
	copy(layersToSave, res.ObjectLayers)
	res.PreRespawnObjectLayers = layersToSave
	build, _ := s.resolveEntityDefaultBuild("resource", activeObjectLayerItemIDs(layersToSave))

	s.grantDropItemsToCaster(mapState, killerProjectile.CasterID, build.DropItemIDs,
		res.Pos.X+res.Dims.Width*0.5, res.Pos.Y+res.Dims.Height*0.5)

	res.ObjectLayers = applyDeadItems(res.ObjectLayers, build.DeadItemIDs)

	res.RespawnTime = time.Now().Add(s.respawnDuration)
	s.InvalidateStats(res)
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
