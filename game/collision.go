package game

import (
	"math"
	"time"
)

// checkAABBCollision checks for collision between two axis-aligned bounding boxes.
func checkAABBCollision(pos1 Vec2, dims1 Vec2, pos2 Vec2, dims2 Vec2) bool {
	// Check for overlap on X axis
	if pos1.X < pos2.X+dims2.X && pos1.X+dims1.X > pos2.X {
		// Check for overlap on Y axis
		if pos1.Y < pos2.Y+dims2.Y && pos1.Y+dims1.Y > pos2.Y {
			return true
		}
	}
	return false
}

// handleSkillCollisions manages collisions for 'skill' behavior bots (projectiles).
// It checks for intersections with players and other bots, applies damage,
// and handles death, removing the projectile upon impact.
func (s *GameServer) handleSkillCollisions(mapState *MapState) {
	projectilesToDelete := []EntityID{}

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
			if mapState.vt(player.ID).ghost() { // Don't hit dead/ghost players
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
			if checkAABBCollision(mapState.tf(projectile.ID).Pos, mapState.tf(projectile.ID).Size, mapState.tf(player.ID).Pos, mapState.tf(player.ID).Size) {
				mapState.vt(player.ID).Life -= projectileStats.Effect
				// Attribute damage for the loot race — player and bot casters both
				// count, so a bot-only kill still drops the victim's coins and the
				// contributing bots can collect. Recorded before the death handler
				// consumes the ledger.
				recordDamage(&player.DamageLedger, mapState, projectile.CasterID, projectileStats.Effect)
				if mapState.vt(player.ID).Life <= 0 {
					mapState.vt(player.ID).Life = 0
					s.handlePlayerDeath(player, mapState)
				}
				// FCT: the same red "-N" for every AOI viewer.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					broadcastFCT(mapState, FCTDamage,
						mapState.tf(projectile.ID).Pos.X+mapState.tf(projectile.ID).Size.X*0.5,
						mapState.tf(projectile.ID).Pos.Y+mapState.tf(projectile.ID).Size.Y*0.5, dmg)
				}

				// "Thorns" effect: The player's Effect stat damages the projectile.
				playerStats := s.CalculateStats(player, mapState)
				if playerStats.Effect > 0 {
					mapState.vt(projectile.ID).Life -= playerStats.Effect
				}

				isColliding = true
			}
		}

		// Check collision with other bots
		for otherBotID, otherBot := range mapState.bots {
			if projectileID == otherBotID || otherBot.Behavior == BehaviorSkill || mapState.vt(otherBot.ID).ghost() {
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
			if checkAABBCollision(mapState.tf(projectile.ID).Pos, mapState.tf(projectile.ID).Size, mapState.tf(otherBot.ID).Pos, mapState.tf(otherBot.ID).Size) {
				mapState.vt(otherBot.ID).Life -= projectileStats.Effect
				// Attribute damage to the firing player or bot for the loot race.
				recordDamage(&otherBot.DamageLedger, mapState, projectile.CasterID, projectileStats.Effect)
				// FCT: bot amounts are public — the same red "-N" for every AOI
				// viewer, including the killing (one-shot) blow.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					broadcastFCT(mapState, FCTDamage,
						mapState.tf(otherBot.ID).Pos.X+mapState.tf(otherBot.ID).Size.X*0.5,
						mapState.tf(otherBot.ID).Pos.Y+mapState.tf(otherBot.ID).Size.Y*0.5, dmg)
				}
				if mapState.vt(otherBot.ID).Life <= 0 {
					mapState.vt(otherBot.ID).Life = 0
					// Capture the victim's skin before death deactivates its
					// layers — quest `kill` objectives match on it.
					killedSkin := s.botActiveSkin(otherBot.ID)
					s.handleBotDeath(otherBot, mapState)
					if killer, ok := mapState.players[projectile.CasterID]; ok {
						s.advancePlayerQuestsOnKill(killer, killedSkin)
					}
				}

				// "Thorns" effect: The other bot's Effect stat damages the projectile.
				otherBotStats := s.CalculateStats(otherBot, mapState)
				if otherBotStats.Effect > 0 {
					mapState.vt(projectile.ID).Life -= otherBotStats.Effect
				}

				isColliding = true
			}
		}

		for _, res := range mapState.resources {
			if hitStatic(mapState, projectile, projectileStats.Effect, res.ID, &res.DamageLedger,
				func() { s.handleResourceDeath(res, mapState) }) {
				isColliding = true
			}
		}
		for _, obs := range mapState.destructibles {
			if hitStatic(mapState, projectile, projectileStats.Effect, obs.ID, &obs.DamageLedger,
				func() { s.handleDestructibleDeath(obs, mapState) }) {
				isColliding = true
			}
		}

		// If the projectile is colliding, it loses life.
		if isColliding {
			mapState.vt(projectile.ID).Life -= s.collisionLifeLoss
			if mapState.vt(projectile.ID).Life <= 0 {
				// Projectile is "dead" from collision damage, mark for deletion.
				projectilesToDelete = append(projectilesToDelete, projectileID)
			}
		}
	}

	for _, id := range projectilesToDelete {
		delete(mapState.bots, id)
		mapState.despawn(id)
	}
}

// hitStatic lands a projectile hit on a static mortal entity — a resource or a
// destructible obstacle — and reports whether it hit. A ghost takes no hit. The
// damage joins the loot race ledger, and `die` runs on the lethal hit.
func hitStatic(mapState *MapState, projectile *BotState, effect float64,
	id EntityID, ledger *map[EntityID]float64, die func()) bool {
	shot, tf, v := mapState.tf(projectile.ID), mapState.tf(id), mapState.vt(id)
	if v.ghost() || !checkAABBCollision(shot.Pos, shot.Size, tf.Pos, tf.Size) {
		return false
	}
	v.Life -= effect
	recordDamage(ledger, mapState, projectile.CasterID, effect)
	if v.Life <= 0 {
		v.Life = 0
		die()
	}
	// FCT: the amount is public — the same red "-N" for every AOI viewer.
	if dmg := int(effect + 0.5); dmg > 0 {
		broadcastFCT(mapState, FCTDamage, tf.Pos.X+tf.Size.X*0.5, tf.Pos.Y+tf.Size.Y*0.5, dmg)
	}
	return true
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

// grantItemToBot transfers `qty` of a non-coin item into a contributing bot's
// inventory when it collects a scattered drop (appended inactive if new).
func (s *GameServer) grantItemToBot(bot *BotState, itemID string, qty int) {
	if itemID == "" || qty <= 0 {
		return
	}
	ol := getOrCreateItemOL(&s.ap(bot.ID, bot.MapCode).Layers, itemID)
	ol.Quantity += qty
}

// handlePlayerDeath sets a player to a dead/ghost state using the dead items of
// the entity-type default resolved from its pre-death active items, and scatters
// the loot: the victim's coin stake (and any configured drop items) become grid
// tokens the damage contributors may collect. The contributor set includes the
// bots that dealt damage, so a bot-only kill still drops the victim's coins per
// the instance economy rules; contributing bots collect on collision. A death
// nobody contributed to drops nothing.
func (s *GameServer) handlePlayerDeath(player *PlayerState, mapState *MapState) {
	s.awardDefeat(&player.EntityBase, &player.Mortal, "player", player.DamageLedger, mapState)
	// Economy: apply respawn-cost sink before saving layers (disabled by default, rate = 0).
	s.SinkRespawnCost(player)

	// Snapshot the pre-death layers (restored on respawn) and resolve the build
	// from them, before the death swap deactivates the live items.
	layersToSave := make([]ObjectLayerState, len(mapState.ap(player.ID).Layers))
	copy(layersToSave, mapState.ap(player.ID).Layers)
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
	playerCenter := Vec2{X: mapState.tf(player.ID).Pos.X + mapState.tf(player.ID).Size.X*0.5, Y: mapState.tf(player.ID).Pos.Y + mapState.tf(player.ID).Size.Y*0.5}
	s.spawnDrops(mapState, player.MapCode, playerCenter, build, coinDrop, player.DamageLedger)
	broadcastAudioEvent(mapState, AudioEventDeath, playerCenter)
	player.DamageLedger = nil

	// The Fragmented State reuses the loadout from the player's last death;
	// first death (or an emptied loadout) falls back to the build's dead items.
	// Normalized so a multi-skin deadItemIds config activates only its leading
	// id per type — the rest stay inventory the player can switch to.
	deadLoadout := player.DeadLoadoutItemIDs
	if len(deadLoadout) == 0 {
		deadLoadout = s.resolveDeadItemIDs(build)
	}
	deadLoadout = s.normalizeDeadLoadout(deadLoadout)
	mapState.ap(player.ID).Layers = applyDeadItems(mapState.ap(player.ID).Layers, deadLoadout)
	player.DeadLoadoutItemIDs = activeObjectLayerItemIDs(mapState.ap(player.ID).Layers)

	mapState.vt(player.ID).RespawnAt = time.Now().Add(player.respawnDuration())
	// Recalculate stats for the ghost state to ensure consistency (mainly MaxLife).
	s.ApplyResistanceStat(player, s.maps[player.MapCode])
	player.Mode = IDLE // Stop movement
}

// killEntity runs the death sequence shared by bots, resources and destructible
// obstacles: snapshot the live loadout for respawn, scatter the drops the damage
// contributors race to collect, swap in the dead visuals, and arm the respawn
// timer.
//
// Both the dead visuals and the drops come from the entity-type default
// resolved by the pre-death active items (most-specific subset match), so a
// skin can carry different dead/drop sets depending on its full active set.
func (s *GameServer) killEntity(base *EntityBase, mortal *Mortal, kind, mapCode string,
	ledger map[EntityID]float64, coins int, mapState *MapState) {

	s.awardDefeat(base, mortal, kind, ledger, mapState)
	layersToSave := make([]ObjectLayerState, len(mapState.ap(base.ID).Layers))
	copy(layersToSave, mapState.ap(base.ID).Layers)
	mortal.PreRespawnObjectLayers = layersToSave
	build, _ := s.resolveEntityDefaultBuild(kind, activeObjectLayerItemIDs(layersToSave))

	center := Vec2{X: mapState.tf(base.ID).Pos.X + mapState.tf(base.ID).Size.X*0.5, Y: mapState.tf(base.ID).Pos.Y + mapState.tf(base.ID).Size.Y*0.5}
	s.spawnDrops(mapState, mapCode, center, build, coins, ledger)
	broadcastAudioEvent(mapState, AudioEventDeath, center)

	mapState.ap(base.ID).Layers = applyDeadItems(mapState.ap(base.ID).Layers, s.resolveDeadItemIDs(build))
	mapState.vt(base.ID).RespawnAt = time.Now().Add(mortal.respawnDuration())
}

// handleBotDeath kills a bot. Loot and coins are not injected into the killer:
// both scatter as collectible tokens.
func (s *GameServer) handleBotDeath(bot *BotState, mapState *MapState) {
	s.killEntity(&bot.EntityBase, &bot.Mortal, "bot", bot.MapCode,
		bot.DamageLedger, s.botCoinDropAmount(bot), mapState)
	bot.DamageLedger = nil

	s.ApplyResistanceStat(bot, s.maps[bot.MapCode])
	bot.Mode = IDLE // Stop movement
}

// handleResourceDeath destroys a resource. Resources carry no coins — item
// drops only, with loot priority to the top extraction contributor.
func (s *GameServer) handleResourceDeath(res *ResourceState, mapState *MapState) {
	s.killEntity(&res.EntityBase, &res.Mortal, "resource", res.MapCode,
		res.DamageLedger, 0, mapState)
	res.DamageLedger = nil
}

// handleDestructibleDeath depletes a destructible obstacle. It carries no coins
// and earns no XP; it frees its cells until it respawns.
func (s *GameServer) handleDestructibleDeath(obs *DestructibleState, mapState *MapState) {
	s.killEntity(&obs.EntityBase, &obs.Mortal, "obstacle", obs.MapCode,
		obs.DamageLedger, 0, mapState)
	obs.DamageLedger = nil
	tf := mapState.tf(obs.ID)
	mapState.pathfinder.occupy(tf.Pos, tf.Size, -1)
}

// handleRespawns checks and respawns dead players and bots.
func (s *GameServer) handleRespawns(mapState *MapState) {
	// Respawn players
	for _, player := range mapState.players {
		if !mapState.vt(player.ID).RespawnAt.IsZero() && time.Now().After(mapState.vt(player.ID).RespawnAt) {
			mapState.ap(player.ID).Layers = player.PreRespawnObjectLayers
			player.PreRespawnObjectLayers = nil
			// The restored loadout was snapshotted before the death coin drop, so
			// its coin slot is stale. Re-sync the display-only slot to the flat
			// balance (the source of truth) — otherwise the client sees a bogus
			// +N as the coin quantity jumps back up on respawn.
			s.syncCoinOL(&mapState.ap(player.ID).Layers, player.Coins)
			s.ApplyResistanceStat(player, mapState) // Recalculate stats with restored items.
			mapState.vt(player.ID).Life = mapState.vt(player.ID).MaxLife
			mapState.vt(player.ID).RespawnAt = time.Time{}
		}
	}

	// Respawn bots
	for _, bot := range mapState.bots {
		if !mapState.vt(bot.ID).RespawnAt.IsZero() && time.Now().After(mapState.vt(bot.ID).RespawnAt) {
			mapState.ap(bot.ID).Layers = bot.PreRespawnObjectLayers
			bot.PreRespawnObjectLayers = nil
			s.ApplyResistanceStat(bot, mapState) // Recalculate stats with restored items.
			mapState.vt(bot.ID).Life = mapState.vt(bot.ID).MaxLife
			mapState.vt(bot.ID).RespawnAt = time.Time{}

			// Reset coin supply via the Fountain — single source of truth.
			s.FountainInitBot(bot)
		}
	}

	// Respawn resources — restore original OLs and reset life.
	for _, res := range mapState.resources {
		if !mapState.vt(res.ID).RespawnAt.IsZero() && time.Now().After(mapState.vt(res.ID).RespawnAt) {
			mapState.ap(res.ID).Layers = res.PreRespawnObjectLayers
			res.PreRespawnObjectLayers = nil
			s.ApplyResistanceStat(res, mapState)
			mapState.vt(res.ID).Life = mapState.vt(res.ID).MaxLife
			mapState.vt(res.ID).RespawnAt = time.Time{}
		}
	}

	// Restored obstacles block their cells before nearby entities move.
	for _, obs := range mapState.destructibles {
		v := mapState.vt(obs.ID)
		if !v.ghost() || time.Now().Before(v.RespawnAt) {
			continue
		}
		mapState.ap(obs.ID).Layers = obs.PreRespawnObjectLayers
		obs.PreRespawnObjectLayers = nil
		v.Life = v.MaxLife
		v.RespawnAt = time.Time{}
		tf := *mapState.tf(obs.ID)
		mapState.pathfinder.occupy(tf.Pos, tf.Size, 1)
		s.clearObstacleCells(mapState, tf)
	}
}

// Projectiles keep their course through restored obstacles.
func (s *GameServer) clearObstacleCells(ms *MapState, obs Transform) {
	for _, player := range ms.players {
		tf := ms.tf(player.ID)
		if !checkAABBCollision(obs.Pos, obs.Size, tf.Pos, tf.Size) {
			if pathCrossesObstacle(player.Path, tf.Size, obs) {
				player.Path, player.Mode = nil, IDLE
				player.PendingMove = player.TargetPos
				player.HasPendingMove = true
				s.flushPendingMove(player, ms)
			}
			continue
		}
		if cell, err := obstaclePushCell(ms, obs, *tf, player.Direction); err == nil {
			tf.Pos = Vec2{X: float64(cell.X), Y: float64(cell.Y)}
			player.Path, player.TargetPos, player.Mode = nil, cell, IDLE
		}
	}
	for _, bot := range ms.bots {
		if bot.Behavior == BehaviorSkill {
			continue
		}
		tf := ms.tf(bot.ID)
		if checkAABBCollision(obs.Pos, obs.Size, tf.Pos, tf.Size) {
			if cell, err := obstaclePushCell(ms, obs, *tf, bot.Direction); err == nil {
				tf.Pos = Vec2{X: float64(cell.X), Y: float64(cell.Y)}
				bot.TargetPos = cell
			}
		} else if !pathCrossesObstacle(bot.Path, tf.Size, obs) {
			continue
		}
		bot.Path, bot.Mode = nil, IDLE
		bot.CurrentTargetPlayer = 0
		bot.lastPursuitTargetPos = PointI{-1, -1}
	}
}

func obstaclePushCell(ms *MapState, obs, entity Transform, direction Direction) (PointI, error) {
	center := Vec2{X: obs.Pos.X + obs.Size.X*0.5, Y: obs.Pos.Y + obs.Size.Y*0.5}
	dx := entity.Pos.X + entity.Size.X*0.5 - center.X
	dy := entity.Pos.Y + entity.Size.Y*0.5 - center.Y
	// An entity at the center follows its facing direction.
	if dx == 0 && dy == 0 {
		dx, dy = getDirectionVector(direction)
		if dx == 0 && dy == 0 {
			dy = 1
		}
	}
	// The combined half-widths place the entire entity outside the obstacle.
	scale := math.Inf(1)
	if dx != 0 {
		scale = (obs.Size.X + entity.Size.X) * 0.5 / math.Abs(dx)
	}
	if dy != 0 {
		scale = math.Min(scale, (obs.Size.Y+entity.Size.Y)*0.5/math.Abs(dy))
	}
	x := center.X + dx*scale - entity.Size.X*0.5
	y := center.Y + dy*scale - entity.Size.Y*0.5
	cell := PointI{X: int(math.Round(x)), Y: int(math.Round(y))}
	if dx > 0 {
		cell.X = int(math.Ceil(x))
	} else if dx < 0 {
		cell.X = int(math.Floor(x))
	}
	if dy > 0 {
		cell.Y = int(math.Ceil(y))
	} else if dy < 0 {
		cell.Y = int(math.Floor(y))
	}
	cell = clampCellToGrid(cell, ms.gridW, ms.gridH)
	return ms.pathfinder.findClosestWalkablePoint(cell, entity.Size)
}

func pathCrossesObstacle(path []PointI, size Vec2, obs Transform) bool {
	for _, cell := range path {
		if checkAABBCollision(obs.Pos, obs.Size, Vec2{X: float64(cell.X), Y: float64(cell.Y)}, size) {
			return true
		}
	}
	return false
}
