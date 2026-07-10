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
				// Attribute damage for the PvP loot race (player casters only —
				// recordDamage filters; bot damage grants no claim). Recorded
				// before the death handler consumes the ledger.
				recordDamage(&player.DamageLedger, mapState, projectile.CasterID, projectileStats.Effect)
				if player.Life <= 0 {
					player.Life = 0
					s.handlePlayerDeath(player, mapState)
				}
				// FCT: the same red "-N" for every AOI viewer.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					broadcastFCT(mapState, FCTTypeDamage,
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
			// Inert loot tokens are non-combat: projectiles pass through them.
			if behaviorIsInert(otherBot.Behavior) {
				continue
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
				// Attribute damage to the firing player for loot priority.
				recordDamage(&otherBot.DamageLedger, mapState, projectile.CasterID, projectileStats.Effect)
				// FCT: bot amounts are public — the same red "-N" for every AOI
				// viewer, including the killing (one-shot) blow.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					broadcastFCT(mapState, FCTTypeDamage,
						otherBot.Pos.X+otherBot.Dims.Width*0.5,
						otherBot.Pos.Y+otherBot.Dims.Height*0.5, dmg)
				}
				if otherBot.Life <= 0 {
					otherBot.Life = 0
					// Capture the victim's skin before death deactivates its
					// layers — quest `kill` objectives match on it.
					killedSkin := s.botActiveSkin(otherBot.ID)
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
				// Attribute extraction damage to the firing player for loot priority.
				recordDamage(&res.DamageLedger, mapState, projectile.CasterID, projectileStats.Effect)
				if res.Life <= 0 {
					res.Life = 0
					s.handleResourceDeath(res, projectile, mapState)
				}
				// FCT: resource amounts are public — the same red "-N" for
				// every AOI viewer.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					broadcastFCT(mapState, FCTTypeDamage,
						res.Pos.X+res.Dims.Width*0.5,
						res.Pos.Y+res.Dims.Height*0.5, dmg)
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

// grantItemToPlayer adds `qty` of a non-coin item to a player's inventory
// (appended inactive if new). No gain FCT — the pickup shows in the loot grid.
func (s *GameServer) grantItemToPlayer(player *PlayerState, itemID string, qty int) {
	if itemID == "" || qty <= 0 {
		return
	}
	found := false
	for i := range player.ObjectLayers {
		if player.ObjectLayers[i].ItemID == itemID {
			player.ObjectLayers[i].Quantity += qty
			found = true
			break
		}
	}
	if !found {
		player.ObjectLayers = append(player.ObjectLayers, ObjectLayerState{ItemID: itemID, Active: false, Quantity: qty})
	}
	s.InvalidateStats(player)
}

// handlePlayerDeath sets a player to a dead/ghost state using the dead items of
// the entity-type default resolved from its pre-death active items, and scatters
// the PvP loot: the victim's coin stake (and any configured drop items) become
// grid tokens only damage contributors may race to collect — the same model as
// handleBotDeath. A death with no player contributor drops nothing.
func (s *GameServer) handlePlayerDeath(player *PlayerState, mapState *MapState) {
	// Economy: apply respawn-cost sink before saving layers (disabled by default, rate = 0).
	s.SinkRespawnCost(player)

	// Snapshot the pre-death layers (restored on respawn) and resolve the build
	// from them, before the death swap deactivates the live items.
	layersToSave := make([]ObjectLayerState, len(player.ObjectLayers))
	copy(layersToSave, player.ObjectLayers)
	player.PreRespawnObjectLayers = layersToSave
	build, _ := s.resolveEntityDefaultBuild("player", activeObjectLayerItemIDs(layersToSave))

	// No coin FCT — balance changes surface in the inventory-bar quantity FX.
	coinDrop := 0
	if len(contributorSet(player.DamageLedger)) > 0 {
		coinDrop = s.pvpCoinDropAmount(player)
		if coinDrop > 0 {
			s.addCoins(player, -coinDrop)
		}
	}
	playerCenter := Point{X: player.Pos.X + player.Dims.Width*0.5, Y: player.Pos.Y + player.Dims.Height*0.5}
	s.spawnDrops(mapState, player.MapCode, playerCenter, build.DropItemIDs, coinDrop, player.DamageLedger)
	player.DamageLedger = nil

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

	// Loot and coins are no longer injected into the killer — both scatter as
	// collectible tokens the damage contributors race to collect.
	botCenter := Point{X: bot.Pos.X + bot.Dims.Width*0.5, Y: bot.Pos.Y + bot.Dims.Height*0.5}
	s.spawnDrops(mapState, bot.MapCode, botCenter, build.DropItemIDs, s.botCoinDropAmount(bot), bot.DamageLedger)
	bot.DamageLedger = nil

	bot.ObjectLayers = applyDeadItems(bot.ObjectLayers, build.DeadItemIDs)

	bot.RespawnTime = time.Now().Add(s.respawnDuration)
	// Recalculate stats for the ghost state to ensure consistency (mainly MaxLife).
	s.InvalidateStats(bot)
	s.ApplyResistanceStat(bot, s.maps[bot.MapCode])
	bot.Mode = IDLE // Stop movement
}

// handleResourceDeath sets a resource to a destroyed state, scatters its drop
// items as collectible tokens (top extraction contributor gets loot priority),
// and activates the dead/extracted visuals until respawn.
func (s *GameServer) handleResourceDeath(res *ResourceState, killerProjectile *BotState, mapState *MapState) {
	layersToSave := make([]ObjectLayerState, len(res.ObjectLayers))
	copy(layersToSave, res.ObjectLayers)
	res.PreRespawnObjectLayers = layersToSave
	build, _ := s.resolveEntityDefaultBuild("resource", activeObjectLayerItemIDs(layersToSave))

	resCenter := Point{X: res.Pos.X + res.Dims.Width*0.5, Y: res.Pos.Y + res.Dims.Height*0.5}
	// Resources carry no coins — item drops only.
	s.spawnDrops(mapState, res.MapCode, resCenter, build.DropItemIDs, 0, res.DamageLedger)
	res.DamageLedger = nil

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
