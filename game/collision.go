package game

import (
	"math"
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
				// Attribute damage for the loot race — player and bot casters both
				// count, so a bot-only kill still drops the victim's coins and the
				// contributing bots can collect. Recorded before the death handler
				// consumes the ledger.
				recordDamage(&player.DamageLedger, mapState, projectile.CasterID, projectileStats.Effect)
				if player.Life <= 0 {
					player.Life = 0
					s.handlePlayerDeath(player, mapState)
				}
				// FCT: the same red "-N" for every AOI viewer.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					broadcastFCT(mapState, FCTDamage,
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
				// Attribute damage to the firing player or bot for the loot race.
				recordDamage(&otherBot.DamageLedger, mapState, projectile.CasterID, projectileStats.Effect)
				// FCT: bot amounts are public — the same red "-N" for every AOI
				// viewer, including the killing (one-shot) blow.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					broadcastFCT(mapState, FCTDamage,
						otherBot.Pos.X+otherBot.Dims.Width*0.5,
						otherBot.Pos.Y+otherBot.Dims.Height*0.5, dmg)
				}
				if otherBot.Life <= 0 {
					otherBot.Life = 0
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
					projectile.Life -= otherBotStats.Effect
				}

				isColliding = true
			}
		}

		for _, res := range mapState.resources {
			if hitStatic(mapState, projectile, projectileStats.Effect, &res.EntityBase, &res.Mortal, &res.DamageLedger,
				func() { s.handleResourceDeath(res, mapState) }) {
				isColliding = true
			}
		}
		for _, obs := range mapState.destructibles {
			if hitStatic(mapState, projectile, projectileStats.Effect, &obs.EntityBase, &obs.Mortal, &obs.DamageLedger,
				func() { s.handleDestructibleDeath(obs, mapState) }) {
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

// hitStatic lands a projectile hit on a static mortal entity — a resource or a
// destructible obstacle — and reports whether it hit. A ghost takes no hit. The
// damage joins the loot race ledger, and `die` runs on the lethal hit.
func hitStatic(mapState *MapState, projectile *BotState, effect float64,
	base *EntityBase, mortal *Mortal, ledger *map[string]float64, die func()) bool {
	if mortal.IsGhost() || !checkAABBCollision(projectile.Pos, projectile.Dims, base.Pos, base.Dims) {
		return false
	}
	mortal.Life -= effect
	recordDamage(ledger, mapState, projectile.CasterID, effect)
	if mortal.Life <= 0 {
		mortal.Life = 0
		die()
	}
	// FCT: the amount is public — the same red "-N" for every AOI viewer.
	if dmg := int(effect + 0.5); dmg > 0 {
		broadcastFCT(mapState, FCTDamage, base.Pos.X+base.Dims.Width*0.5, base.Pos.Y+base.Dims.Height*0.5, dmg)
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
	ol := getOrCreateItemOL(&bot.ObjectLayers, itemID)
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
	player.ObjectLayers = applyDeadItems(player.ObjectLayers, deadLoadout)
	player.DeadLoadoutItemIDs = activeObjectLayerItemIDs(player.ObjectLayers)

	player.RespawnTime = time.Now().Add(player.respawnDuration())
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
	ledger map[string]float64, coins int, mapState *MapState) {

	s.awardDefeat(base, mortal, kind, ledger, mapState)
	layersToSave := make([]ObjectLayerState, len(base.ObjectLayers))
	copy(layersToSave, base.ObjectLayers)
	mortal.PreRespawnObjectLayers = layersToSave
	build, _ := s.resolveEntityDefaultBuild(kind, activeObjectLayerItemIDs(layersToSave))

	center := Point{X: base.Pos.X + base.Dims.Width*0.5, Y: base.Pos.Y + base.Dims.Height*0.5}
	s.spawnDrops(mapState, mapCode, center, build, coins, ledger)
	broadcastAudioEvent(mapState, AudioEventDeath, center)

	base.ObjectLayers = applyDeadItems(base.ObjectLayers, s.resolveDeadItemIDs(build))
	mortal.RespawnTime = time.Now().Add(mortal.respawnDuration())
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
	s.killEntity(&obs.EntityBase, &obs.Mortal, EntityObstacle, obs.MapCode,
		obs.DamageLedger, 0, mapState)
	obs.DamageLedger = nil
	mapState.pathfinder.occupy(obs.Pos, obs.Dims, -1)
}

// handleRespawns checks and respawns dead players and bots.
func (s *GameServer) handleRespawns(mapState *MapState) {
	// Respawn players
	for _, player := range mapState.players {
		if !player.RespawnTime.IsZero() && time.Now().After(player.RespawnTime) {
			player.ObjectLayers = player.PreRespawnObjectLayers
			player.PreRespawnObjectLayers = nil
			// The restored loadout was snapshotted before the death coin drop, so
			// its coin slot is stale. Re-sync the display-only slot to the flat
			// balance (the source of truth) — otherwise the client sees a bogus
			// +N as the coin quantity jumps back up on respawn.
			s.syncCoinOL(&player.ObjectLayers, player.Coins)
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
			s.ApplyResistanceStat(res, mapState)
			res.Life = res.MaxLife
			res.RespawnTime = time.Time{}
		}
	}

	// Restored obstacles block their cells before nearby entities move.
	for _, obs := range mapState.destructibles {
		if !obs.IsGhost() || time.Now().Before(obs.RespawnTime) {
			continue
		}
		obs.ObjectLayers = obs.PreRespawnObjectLayers
		obs.PreRespawnObjectLayers = nil
		obs.Life = obs.MaxLife
		obs.RespawnTime = time.Time{}
		mapState.pathfinder.occupy(obs.Pos, obs.Dims, 1)
		s.clearObstacleCells(mapState, obs.EntityBase)
	}
}

// Projectiles keep their course through restored obstacles.
func (s *GameServer) clearObstacleCells(ms *MapState, obs EntityBase) {
	for _, player := range ms.players {
		if !checkAABBCollision(obs.Pos, obs.Dims, player.Pos, player.Dims) {
			if pathCrossesObstacle(player.Path, player.Dims, obs) {
				player.Path, player.Mode = nil, IDLE
				player.PendingMove = player.TargetPos
				player.PendingMoveSequence = player.LastMovementSequence
				player.HasPendingMove = true
				s.flushPendingMove(player, ms)
			}
			continue
		}
		if cell, err := obstaclePushCell(ms, obs, player.EntityBase, player.Direction); err == nil {
			player.Pos = Point{X: float64(cell.X), Y: float64(cell.Y)}
			player.Path, player.TargetPos, player.Mode = nil, cell, IDLE
		}
	}
	for _, bot := range ms.bots {
		if bot.Behavior == BehaviorSkill {
			continue
		}
		if checkAABBCollision(obs.Pos, obs.Dims, bot.Pos, bot.Dims) {
			if cell, err := obstaclePushCell(ms, obs, bot.EntityBase, bot.Direction); err == nil {
				bot.Pos = Point{X: float64(cell.X), Y: float64(cell.Y)}
				bot.TargetPos = cell
			}
		} else if !pathCrossesObstacle(bot.Path, bot.Dims, obs) {
			continue
		}
		bot.Path, bot.Mode = nil, IDLE
		bot.CurrentTargetPlayer = ""
		bot.lastPursuitTargetPos = PointI{-1, -1}
	}
}

func obstaclePushCell(ms *MapState, obs, entity EntityBase, direction Direction) (PointI, error) {
	center := Point{X: obs.Pos.X + obs.Dims.Width*0.5, Y: obs.Pos.Y + obs.Dims.Height*0.5}
	dx := entity.Pos.X + entity.Dims.Width*0.5 - center.X
	dy := entity.Pos.Y + entity.Dims.Height*0.5 - center.Y
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
		scale = (obs.Dims.Width + entity.Dims.Width) * 0.5 / math.Abs(dx)
	}
	if dy != 0 {
		scale = math.Min(scale, (obs.Dims.Height+entity.Dims.Height)*0.5/math.Abs(dy))
	}
	x := center.X + dx*scale - entity.Dims.Width*0.5
	y := center.Y + dy*scale - entity.Dims.Height*0.5
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
	return ms.pathfinder.findClosestWalkablePoint(cell, entity.Dims)
}

func pathCrossesObstacle(path []PointI, dims Dimensions, obs EntityBase) bool {
	for _, cell := range path {
		if checkAABBCollision(obs.Pos, obs.Dims, Point{X: float64(cell.X), Y: float64(cell.Y)}, dims) {
			return true
		}
	}
	return false
}
