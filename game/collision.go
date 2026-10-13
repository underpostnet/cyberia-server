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

	for projectileID := range mapState.brains {
		if mapState.ag(projectileID).Behavior != BehaviorSkill {
			continue
		}

		projectileStats := s.CalculateStats(projectileID, mapState)
		if projectileStats.Effect <= 0 {
			continue // No damage to deal.
		}

		var isColliding bool

		// Check collision with players
		for player := range s.playersOn(mapState) {
			if mapState.vt(player.ID).ghost() { // Don't hit dead/ghost players
				continue
			}
			// Frozen players (in modal interaction) cannot be damaged.
			if player.Frozen {
				continue
			}
			// Projectiles should not damage their caster.
			if player.ID == mapState.ag(projectileID).CasterID {
				continue
			}
			if checkAABBCollision(mapState.tf(projectileID).Pos, mapState.tf(projectileID).Size, mapState.tf(player.ID).Pos, mapState.tf(player.ID).Size) {
				mapState.vt(player.ID).Life -= projectileStats.Effect
				// Attribute damage for the loot race — player and bot casters both
				// count, so a bot-only kill still drops the victim's coins and the
				// contributing bots can collect. Recorded before the death handler
				// consumes the ledger.
				recordDamage(&mapState.mt(player.ID).DamageLedger, mapState, mapState.ag(projectileID).CasterID, projectileStats.Effect)
				if mapState.vt(player.ID).Life <= 0 {
					mapState.vt(player.ID).Life = 0
					s.handlePlayerDeath(player, mapState)
				}
				// FCT: the same red "-N" for every AOI viewer.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					s.broadcastFCT(mapState, FCTDamage,
						mapState.tf(projectileID).Pos.X+mapState.tf(projectileID).Size.X*0.5,
						mapState.tf(projectileID).Pos.Y+mapState.tf(projectileID).Size.Y*0.5, dmg)
				}

				// "Thorns" effect: The player's Effect stat damages the projectile.
				playerStats := s.CalculateStats(player.ID, mapState)
				if playerStats.Effect > 0 {
					mapState.vt(projectileID).Life -= playerStats.Effect
				}

				isColliding = true
			}
		}

		// Check collision with other bots
		for otherBotID := range mapState.brains {
			if projectileID == otherBotID || mapState.ag(otherBotID).Behavior == BehaviorSkill || mapState.vt(otherBotID).ghost() {
				continue // Don't collide with self, other projectiles, or dead bots.
			}
			// Inert loot tokens are non-combat: projectiles pass through them.
			if behaviorIsInert(mapState.ag(otherBotID).Behavior) {
				continue
			}
			// Provider NPCs are immortal — projectiles pass through without damage.
			if behaviorIsImmortal(mapState.ag(otherBotID).Behavior) {
				continue
			}
			// Projectiles should not damage their caster.
			if otherBotID == mapState.ag(projectileID).CasterID {
				continue
			}
			if checkAABBCollision(mapState.tf(projectileID).Pos, mapState.tf(projectileID).Size, mapState.tf(otherBotID).Pos, mapState.tf(otherBotID).Size) {
				mapState.vt(otherBotID).Life -= projectileStats.Effect
				// Attribute damage to the firing player or bot for the loot race.
				recordDamage(&mapState.mt(otherBotID).DamageLedger, mapState, mapState.ag(projectileID).CasterID, projectileStats.Effect)
				// FCT: bot amounts are public — the same red "-N" for every AOI
				// viewer, including the killing (one-shot) blow.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					s.broadcastFCT(mapState, FCTDamage,
						mapState.tf(otherBotID).Pos.X+mapState.tf(otherBotID).Size.X*0.5,
						mapState.tf(otherBotID).Pos.Y+mapState.tf(otherBotID).Size.Y*0.5, dmg)
				}
				if mapState.vt(otherBotID).Life <= 0 {
					mapState.vt(otherBotID).Life = 0
					// Capture the victim's skin before death deactivates its
					// layers — quest `kill` objectives match on it.
					killedSkin := s.botActiveSkin(otherBotID)
					s.handleBotDeath(otherBotID, mapState)
					if killer := s.playerOn(mapState, mapState.ag(projectileID).CasterID); killer != nil {
						s.advancePlayerQuestsOnKill(killer, killedSkin)
					}
				}

				// "Thorns" effect: The other bot's Effect stat damages the projectile.
				otherBotStats := s.CalculateStats(otherBotID, mapState)
				if otherBotStats.Effect > 0 {
					mapState.vt(projectileID).Life -= otherBotStats.Effect
				}

				isColliding = true
			}
		}

		for res := range mapState.ofKind(EntityResource) {
			if s.hitStatic(mapState, projectileID, projectileStats.Effect, res,
				func() { s.handleResourceDeath(res, mapState) }) {
				isColliding = true
			}
		}
		for obs := range mapState.destructibles() {
			if s.hitStatic(mapState, projectileID, projectileStats.Effect, obs,
				func() { s.handleDestructibleDeath(obs, mapState) }) {
				isColliding = true
			}
		}

		// If the projectile is colliding, it loses life.
		if isColliding {
			mapState.vt(projectileID).Life -= s.collisionLifeLoss
			if mapState.vt(projectileID).Life <= 0 {
				// Projectile is "dead" from collision damage, mark for deletion.
				projectilesToDelete = append(projectilesToDelete, projectileID)
			}
		}
	}

	for _, id := range projectilesToDelete {
		s.despawn(mapState, id)
	}
}

// hitStatic lands a projectile hit on a static mortal entity — a resource or a
// destructible obstacle — and reports whether it hit. A ghost takes no hit. The
// damage joins the loot race ledger, and `die` runs on the lethal hit.
func (s *GameServer) hitStatic(mapState *MapState, projectile EntityID, effect float64,
	id EntityID, die func()) bool {
	shot, tf, v := mapState.tf(projectile), mapState.tf(id), mapState.vt(id)
	if v.ghost() || !checkAABBCollision(shot.Pos, shot.Size, tf.Pos, tf.Size) {
		return false
	}
	v.Life -= effect
	recordDamage(&mapState.mt(id).DamageLedger, mapState, mapState.ag(projectile).CasterID, effect)
	if v.Life <= 0 {
		v.Life = 0
		die()
	}
	// FCT: the amount is public — the same red "-N" for every AOI viewer.
	if dmg := int(effect + 0.5); dmg > 0 {
		s.broadcastFCT(mapState, FCTDamage, tf.Pos.X+tf.Size.X*0.5, tf.Pos.Y+tf.Size.Y*0.5, dmg)
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
func (s *GameServer) grantItemToBot(id EntityID, ms *MapState, itemID string, qty int) {
	if itemID == "" || qty <= 0 {
		return
	}
	ol := getOrCreateItemOL(&ms.ap(id).Layers, itemID)
	ol.Quantity += qty
}

// handlePlayerDeath sets a player to a dead/ghost state using the dead items of
// the entity-type default resolved from its pre-death active items, and scatters
// the loot: the victim's coin stake (and any configured drop items) become grid
// tokens the damage contributors may collect. The contributor set includes the
// bots that dealt damage, so a bot-only kill still drops the victim's coins per
// the instance economy rules; contributing bots collect on collision. A death
// nobody contributed to drops nothing.
func (s *GameServer) handlePlayerDeath(player *Session, mapState *MapState) {
	mortal := mapState.mt(player.ID)
	s.awardDefeat(player.ID, "player", mapState)
	// Economy: apply respawn-cost sink before saving layers (disabled by default, rate = 0).
	s.SinkRespawnCost(player)

	// Snapshot the pre-death layers (restored on respawn) and resolve the build
	// from them, before the death swap deactivates the live items.
	layersToSave := make([]ObjectLayerState, len(mapState.ap(player.ID).Layers))
	copy(layersToSave, mapState.ap(player.ID).Layers)
	mortal.PreRespawnObjectLayers = layersToSave
	build, _ := s.resolveEntityDefaultBuild("player", activeObjectLayerItemIDs(layersToSave))

	// No coin FCT — balance changes surface in the inventory-bar quantity FX.
	coinDrop := 0
	if len(contributorSet(mortal.DamageLedger)) > 0 {
		coinDrop = s.pvpCoinDropAmount(player)
		if coinDrop > 0 {
			s.addCoins(player.ID, mapState, -coinDrop)
		}
	}
	playerCenter := Vec2{X: mapState.tf(player.ID).Pos.X + mapState.tf(player.ID).Size.X*0.5, Y: mapState.tf(player.ID).Pos.Y + mapState.tf(player.ID).Size.Y*0.5}
	s.spawnDrops(mapState, playerCenter, build, coinDrop, mortal.DamageLedger)
	s.broadcastAudioEvent(mapState, AudioEventDeath, playerCenter)
	mortal.DamageLedger = nil

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

	mapState.vt(player.ID).RespawnAt = time.Now().Add(mapState.mt(player.ID).respawnDuration())
	// Recalculate stats for the ghost state to ensure consistency (mainly MaxLife).
	s.ApplyResistanceStat(player.ID, s.maps[player.MapCode])
	mapState.mo(player.ID).Mode = IDLE // Stop movement
}

// killEntity runs the death sequence shared by bots, resources and destructible
// obstacles: snapshot the live loadout for respawn, scatter the drops the damage
// contributors race to collect, swap in the dead visuals, and arm the respawn
// timer.
//
// Both the dead visuals and the drops come from the entity-type default
// resolved by the pre-death active items (most-specific subset match), so a
// skin can carry different dead/drop sets depending on its full active set.
func (s *GameServer) killEntity(id EntityID, kind string, coins int, mapState *MapState) {
	mortal := mapState.mt(id)
	s.awardDefeat(id, kind, mapState)
	layersToSave := make([]ObjectLayerState, len(mapState.ap(id).Layers))
	copy(layersToSave, mapState.ap(id).Layers)
	mortal.PreRespawnObjectLayers = layersToSave
	build, _ := s.resolveEntityDefaultBuild(kind, activeObjectLayerItemIDs(layersToSave))

	center := Vec2{X: mapState.tf(id).Pos.X + mapState.tf(id).Size.X*0.5, Y: mapState.tf(id).Pos.Y + mapState.tf(id).Size.Y*0.5}
	s.spawnDrops(mapState, center, build, coins, mortal.DamageLedger)
	mortal.DamageLedger = nil
	s.broadcastAudioEvent(mapState, AudioEventDeath, center)

	mapState.ap(id).Layers = applyDeadItems(mapState.ap(id).Layers, s.resolveDeadItemIDs(build))
	mapState.vt(id).RespawnAt = time.Now().Add(mortal.respawnDuration())
}

// handleBotDeath kills a bot. Loot and coins are not injected into the killer:
// both scatter as collectible tokens.
func (s *GameServer) handleBotDeath(id EntityID, mapState *MapState) {
	s.killEntity(id, "bot", s.botCoinDropAmount(id, mapState), mapState)

	s.ApplyResistanceStat(id, mapState)
	mapState.mo(id).Mode = IDLE // Stop movement
}

// handleResourceDeath destroys a resource. Resources carry no coins — item
// drops only, with loot priority to the top extraction contributor.
func (s *GameServer) handleResourceDeath(res EntityID, mapState *MapState) {
	s.killEntity(res, "resource", 0, mapState)
}

// handleDestructibleDeath depletes a destructible obstacle. It carries no coins
// and earns no XP; it frees its cells until it respawns.
func (s *GameServer) handleDestructibleDeath(obs EntityID, mapState *MapState) {
	s.killEntity(obs, "obstacle", 0, mapState)
	tf := mapState.tf(obs)
	mapState.pathfinder.occupy(tf.Pos, tf.Size, -1)
}

// handleRespawns checks and respawns dead players and bots.
func (s *GameServer) handleRespawns(mapState *MapState) {
	// Respawn players
	for player := range s.playersOn(mapState) {
		if !mapState.vt(player.ID).RespawnAt.IsZero() && time.Now().After(mapState.vt(player.ID).RespawnAt) {
			mapState.ap(player.ID).Layers = mapState.mt(player.ID).PreRespawnObjectLayers
			mapState.mt(player.ID).PreRespawnObjectLayers = nil
			// The restored loadout was snapshotted before the death coin drop, so
			// its coin slot is stale. Re-sync the display-only slot to the flat
			// balance (the source of truth) — otherwise the client sees a bogus
			// +N as the coin quantity jumps back up on respawn.
			s.syncCoinOL(&mapState.ap(player.ID).Layers, player.Coins)
			s.ApplyResistanceStat(player.ID, mapState) // Recalculate stats with restored items.
			mapState.vt(player.ID).Life = mapState.vt(player.ID).MaxLife
			mapState.vt(player.ID).RespawnAt = time.Time{}
		}
	}

	// Respawn bots
	for id := range mapState.brains {
		if !mapState.vt(id).RespawnAt.IsZero() && time.Now().After(mapState.vt(id).RespawnAt) {
			mapState.ap(id).Layers = mapState.mt(id).PreRespawnObjectLayers
			mapState.mt(id).PreRespawnObjectLayers = nil
			s.ApplyResistanceStat(id, mapState) // Recalculate stats with restored items.
			mapState.vt(id).Life = mapState.vt(id).MaxLife
			mapState.vt(id).RespawnAt = time.Time{}

			// Reset coin supply via the Fountain — single source of truth.
			s.FountainInitBot(id, mapState)
		}
	}

	// Respawn resources — restore original OLs and reset life.
	for res := range mapState.ofKind(EntityResource) {
		if !mapState.vt(res).RespawnAt.IsZero() && time.Now().After(mapState.vt(res).RespawnAt) {
			mapState.ap(res).Layers = mapState.mt(res).PreRespawnObjectLayers
			mapState.mt(res).PreRespawnObjectLayers = nil
			s.ApplyResistanceStat(res, mapState)
			mapState.vt(res).Life = mapState.vt(res).MaxLife
			mapState.vt(res).RespawnAt = time.Time{}
		}
	}

	// Restored obstacles block their cells before nearby entities move.
	for obs := range mapState.destructibles() {
		v := mapState.vt(obs)
		if !v.ghost() || time.Now().Before(v.RespawnAt) {
			continue
		}
		mortal := mapState.mt(obs)
		mapState.ap(obs).Layers = mortal.PreRespawnObjectLayers
		mortal.PreRespawnObjectLayers = nil
		v.Life = v.MaxLife
		v.RespawnAt = time.Time{}
		tf := *mapState.tf(obs)
		mapState.pathfinder.occupy(tf.Pos, tf.Size, 1)
		s.clearObstacleCells(mapState, tf)
	}
}

// Projectiles keep their course through restored obstacles.
func (s *GameServer) clearObstacleCells(ms *MapState, obs Transform) {
	for player := range s.playersOn(ms) {
		tf := ms.tf(player.ID)
		if !checkAABBCollision(obs.Pos, obs.Size, tf.Pos, tf.Size) {
			if pathCrossesObstacle(player.Path, tf.Size, obs) {
				player.Path, ms.mo(player.ID).Mode = nil, IDLE
				player.PendingMove = player.TargetPos
				player.HasPendingMove = true
				s.flushPendingMove(player, ms)
			}
			continue
		}
		if cell, err := obstaclePushCell(ms, obs, *tf, ms.mo(player.ID).Direction); err == nil {
			tf.Pos = Vec2{X: float64(cell.X), Y: float64(cell.Y)}
			player.Path, player.TargetPos, ms.mo(player.ID).Mode = nil, cell, IDLE
		}
	}
	for id, bot := range ms.brains {
		if ms.ag(id).Behavior == BehaviorSkill {
			continue
		}
		tf := ms.tf(id)
		if checkAABBCollision(obs.Pos, obs.Size, tf.Pos, tf.Size) {
			if cell, err := obstaclePushCell(ms, obs, *tf, ms.mo(id).Direction); err == nil {
				tf.Pos = Vec2{X: float64(cell.X), Y: float64(cell.Y)}
				bot.TargetPos = cell
			}
		} else if !pathCrossesObstacle(bot.Path, tf.Size, obs) {
			continue
		}
		ms.mo(id).Mode = IDLE
		bot.Path = nil
		bot.CurrentTargetPlayer = 0
		bot.lastPursuitTargetPos = Cell{-1, -1}
	}
}

func obstaclePushCell(ms *MapState, obs, entity Transform, direction Direction) (Cell, error) {
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
	cell := Cell{X: int(math.Round(x)), Y: int(math.Round(y))}
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

func pathCrossesObstacle(path []Cell, size Vec2, obs Transform) bool {
	for _, cell := range path {
		if checkAABBCollision(obs.Pos, obs.Size, Vec2{X: float64(cell.X), Y: float64(cell.Y)}, size) {
			return true
		}
	}
	return false
}
