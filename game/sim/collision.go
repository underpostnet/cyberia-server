package sim

import (
	"math"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// handleSkillCollisions manages collisions for 'skill' behavior bots (projectiles).
// It checks for intersections with players and other bots, applies damage,
// and handles death, removing the projectile upon impact.
func (s *Sim) handleSkillCollisions(mapState *world.MapState) {
	projectilesToDelete := []world.EntityID{}

	for projectileID := range mapState.OfKind(world.EntityBot) {
		if mapState.Ag(projectileID).Behavior != content.BehaviorSkill {
			continue
		}

		projectileStats := s.CalculateStats(projectileID, mapState)
		if projectileStats.Effect <= 0 {
			continue // No damage to deal.
		}

		var isColliding bool

		// Check collision with players
		for player := range s.PlayersOn(mapState) {
			if mapState.Vt(player.ID).Ghost() { // Don't hit dead/ghost players
				continue
			}
			// Frozen players (in modal interaction) cannot be damaged.
			if player.Frozen {
				continue
			}
			// Projectiles should not damage their caster.
			if player.ID == mapState.Ag(projectileID).CasterID || s.friendlyHit(mapState, projectileID, player.ID) {
				continue
			}
			if world.CheckAABBCollision(mapState.Tf(projectileID).Pos, mapState.Tf(projectileID).Size, mapState.Tf(player.ID).Pos, mapState.Tf(player.ID).Size) {
				mapState.Vt(player.ID).Life -= projectileStats.Effect
				// Attribute damage for the loot race — player and bot casters both
				// count, so a bot-only kill still drops the victim's coins and the
				// contributing bots can collect. Recorded before the death handler
				// consumes the ledger.
				content.RecordDamage(&mapState.Mt(player.ID).DamageLedger, mapState, mapState.Ag(projectileID).CasterID, projectileStats.Effect)
				if mapState.Vt(player.ID).Life <= 0 {
					mapState.Vt(player.ID).Life = 0
					s.handlePlayerDeath(player, mapState)
				}
				// FCT: the same red "-N" for every AOI viewer.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					s.BroadcastFCT(mapState, world.FCTDamage,
						mapState.Tf(projectileID).Pos.X+mapState.Tf(projectileID).Size.X*0.5,
						mapState.Tf(projectileID).Pos.Y+mapState.Tf(projectileID).Size.Y*0.5, dmg)
				}

				// "Thorns" effect: The player's Effect stat damages the projectile.
				playerStats := s.CalculateStats(player.ID, mapState)
				if playerStats.Effect > 0 {
					mapState.Vt(projectileID).Life -= playerStats.Effect
				}

				isColliding = true
			}
		}

		// Check collision with other bots
		for otherBotID := range mapState.OfKind(world.EntityBot) {
			if projectileID == otherBotID || mapState.Ag(otherBotID).Behavior == content.BehaviorSkill || mapState.Vt(otherBotID).Ghost() {
				continue // Don't collide with self, other projectiles, or dead bots.
			}
			// Inert loot tokens are non-combat: projectiles pass through them.
			if content.BehaviorIsInert(mapState.Ag(otherBotID).Behavior) {
				continue
			}
			// Provider NPCs are immortal — projectiles pass through without damage.
			if content.BehaviorIsImmortal(mapState.Ag(otherBotID).Behavior) {
				continue
			}
			// Projectiles should not damage their caster.
			if otherBotID == mapState.Ag(projectileID).CasterID || s.friendlyHit(mapState, projectileID, otherBotID) {
				continue
			}
			if world.CheckAABBCollision(mapState.Tf(projectileID).Pos, mapState.Tf(projectileID).Size, mapState.Tf(otherBotID).Pos, mapState.Tf(otherBotID).Size) {
				mapState.Vt(otherBotID).Life -= projectileStats.Effect
				// Attribute damage to the firing player or bot for the loot race.
				content.RecordDamage(&mapState.Mt(otherBotID).DamageLedger, mapState, mapState.Ag(projectileID).CasterID, projectileStats.Effect)
				// FCT: bot amounts are public — the same red "-N" for every AOI
				// viewer, including the killing (one-shot) blow.
				if dmg := int(projectileStats.Effect + 0.5); dmg > 0 {
					s.BroadcastFCT(mapState, world.FCTDamage,
						mapState.Tf(otherBotID).Pos.X+mapState.Tf(otherBotID).Size.X*0.5,
						mapState.Tf(otherBotID).Pos.Y+mapState.Tf(otherBotID).Size.Y*0.5, dmg)
				}
				if mapState.Vt(otherBotID).Life <= 0 {
					mapState.Vt(otherBotID).Life = 0
					// Capture the victim's skin before death deactivates its
					// layers — quest `kill` objectives match on it.
					killedSkin := s.BotActiveSkin(otherBotID)
					s.handleBotDeath(otherBotID, mapState)
					if killer := s.PlayerOn(mapState, mapState.Ag(projectileID).CasterID); killer != nil {
						s.AdvancePlayerQuestsOnKill(killer, killedSkin)
					}
				}

				// "Thorns" effect: The other bot's Effect stat damages the projectile.
				otherBotStats := s.CalculateStats(otherBotID, mapState)
				if otherBotStats.Effect > 0 {
					mapState.Vt(projectileID).Life -= otherBotStats.Effect
				}

				isColliding = true
			}
		}

		for res := range mapState.OfKind(world.EntityResource) {
			if s.hitStatic(mapState, projectileID, projectileStats.Effect, res,
				func() { s.handleResourceDeath(res, mapState) }) {
				isColliding = true
			}
		}
		for obs := range mapState.Destructibles() {
			if s.hitStatic(mapState, projectileID, projectileStats.Effect, obs,
				func() { s.handleDestructibleDeath(obs, mapState) }) {
				isColliding = true
			}
		}

		// If the projectile is colliding, it loses life.
		if isColliding {
			mapState.Vt(projectileID).Life -= s.CollisionLifeLoss
			if mapState.Vt(projectileID).Life <= 0 {
				// Projectile is "dead" from collision damage, mark for deletion.
				projectilesToDelete = append(projectilesToDelete, projectileID)
			}
		}
	}

	for _, id := range projectilesToDelete {
		s.Despawn(mapState, id)
	}
}

// hitStatic lands a projectile hit on a static mortal entity — a resource or a
// destructible obstacle — and reports whether it hit. A ghost takes no hit. The
// damage joins the loot race ledger, and `die` runs on the lethal hit.
func (s *Sim) hitStatic(mapState *world.MapState, projectile world.EntityID, effect float64,
	id world.EntityID, die func()) bool {
	shot, tf, v := mapState.Tf(projectile), mapState.Tf(id), mapState.Vt(id)
	if v.Ghost() || !world.CheckAABBCollision(shot.Pos, shot.Size, tf.Pos, tf.Size) {
		return false
	}
	v.Life -= effect
	content.RecordDamage(&mapState.Mt(id).DamageLedger, mapState, mapState.Ag(projectile).CasterID, effect)
	if v.Life <= 0 {
		v.Life = 0
		die()
	}
	// FCT: the amount is public — the same red "-N" for every AOI viewer.
	if dmg := int(effect + 0.5); dmg > 0 {
		s.BroadcastFCT(mapState, world.FCTDamage, tf.Pos.X+tf.Size.X*0.5, tf.Pos.Y+tf.Size.Y*0.5, dmg)
	}
	return true
}

// applyDeadItems deactivates every layer and (re)activates the configured dead
// item IDs, appending any that are missing. Shared by every entity death path so
// the dead/ghost visual swap is identical for players, bots, and resources.
func applyDeadItems(layers []world.ObjectLayerState, deadItemIDs []string) []world.ObjectLayerState {
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
			layers = append(layers, world.ObjectLayerState{ItemID: deadID, Active: true, Quantity: 1})
		}
	}
	return layers
}

// handlePlayerDeath sets a player to a dead/ghost state using the dead items of
// the entity-type default resolved from its pre-death active items, and scatters
// the loot: the victim's coin stake (and any configured drop items) become grid
// tokens the damage contributors may collect. The contributor set includes the
// bots that dealt damage, so a bot-only kill still drops the victim's coins per
// the instance economy rules; contributing bots collect on collision. A death
// nobody contributed to drops nothing.
func (s *Sim) handlePlayerDeath(player *world.Session, mapState *world.MapState) {
	mortal := mapState.Mt(player.ID)
	s.AwardDefeat(player.ID, "player", mapState, s.Social.PartyMembers)
	// Economy: apply respawn-cost sink before saving layers (disabled by default, rate = 0).
	s.SinkRespawnCost(player)

	// Snapshot the pre-death layers (restored on respawn) and resolve the build
	// from them, before the death swap deactivates the live items.
	layersToSave := make([]world.ObjectLayerState, len(mapState.Ap(player.ID).Layers))
	copy(layersToSave, mapState.Ap(player.ID).Layers)
	mortal.PreRespawnObjectLayers = layersToSave
	build, _ := s.ResolveEntityDefaultBuild("player", content.ActiveObjectLayerItemIDs(layersToSave))

	// No coin FCT — balance changes surface in the inventory-bar quantity FX.
	coinDrop := 0
	if len(content.ContributorSet(mortal.DamageLedger)) > 0 {
		coinDrop = s.PvpCoinDropAmount(player)
		if coinDrop > 0 {
			s.AddCoins(player.ID, mapState, -coinDrop)
		}
	}
	playerCenter := world.Vec2{X: mapState.Tf(player.ID).Pos.X + mapState.Tf(player.ID).Size.X*0.5, Y: mapState.Tf(player.ID).Pos.Y + mapState.Tf(player.ID).Size.Y*0.5}
	s.SpawnDrops(mapState, playerCenter, build, coinDrop, mortal.DamageLedger, s.LossLock(player))
	s.BroadcastAudioEvent(mapState, world.AudioEventDeath, playerCenter)
	mortal.DamageLedger = nil

	// The Fragmented State reuses the loadout from the player's last death;
	// first death (or an emptied loadout) falls back to the build's dead items.
	// Normalized so a multi-skin deadItemIds config activates only its leading
	// id per type — the rest stay inventory the player can switch to.
	deadLoadout := player.DeadLoadoutItemIDs
	if len(deadLoadout) == 0 {
		deadLoadout = s.ResolveDeadItemIDs(build)
	}
	deadLoadout = s.NormalizeDeadLoadout(deadLoadout)
	carried := len(mapState.Ap(player.ID).Layers)
	mapState.Ap(player.ID).Layers = applyDeadItems(mapState.Ap(player.ID).Layers, deadLoadout)
	s.SeatStacks(mapState.Ap(player.ID).Layers, carried)
	player.DeadLoadoutItemIDs = content.ActiveObjectLayerItemIDs(mapState.Ap(player.ID).Layers)

	mapState.Vt(player.ID).RespawnAt = time.Now().Add(content.RespawnWait(mapState.Mt(player.ID)))
	// Recalculate stats for the ghost state to ensure consistency (mainly MaxLife).
	s.ApplyResistanceStat(player.ID, s.Maps[player.MapCode])
	mapState.Mo(player.ID).Mode = world.IDLE // Stop movement
}

// killEntity runs the death sequence shared by bots, resources and destructible
// obstacles: snapshot the live loadout for respawn, scatter the drops the damage
// contributors race to collect, swap in the dead visuals, and arm the respawn
// timer.
//
// Both the dead visuals and the drops come from the entity-type default
// resolved by the pre-death active items (most-specific subset match), so a
// skin can carry different dead/drop sets depending on its full active set.
func (s *Sim) killEntity(id world.EntityID, kind string, coins int, mapState *world.MapState) {
	mortal := mapState.Mt(id)
	s.AwardDefeat(id, kind, mapState, s.Social.PartyMembers)
	layersToSave := make([]world.ObjectLayerState, len(mapState.Ap(id).Layers))
	copy(layersToSave, mapState.Ap(id).Layers)
	mortal.PreRespawnObjectLayers = layersToSave
	build, _ := s.ResolveEntityDefaultBuild(kind, content.ActiveObjectLayerItemIDs(layersToSave))

	center := world.Vec2{X: mapState.Tf(id).Pos.X + mapState.Tf(id).Size.X*0.5, Y: mapState.Tf(id).Pos.Y + mapState.Tf(id).Size.Y*0.5}
	s.SpawnDrops(mapState, center, build, coins, mortal.DamageLedger, world.DropLock{})
	mortal.DamageLedger = nil
	s.BroadcastAudioEvent(mapState, world.AudioEventDeath, center)

	mapState.Ap(id).Layers = applyDeadItems(mapState.Ap(id).Layers, s.ResolveDeadItemIDs(build))
	mapState.Vt(id).RespawnAt = time.Now().Add(content.RespawnWait(mortal))
}

// handleBotDeath kills a bot. Loot and coins are not injected into the killer:
// both scatter as collectible tokens.
func (s *Sim) handleBotDeath(id world.EntityID, mapState *world.MapState) {
	s.killEntity(id, "bot", s.BotCoinDropAmount(id, mapState), mapState)

	s.ApplyResistanceStat(id, mapState)
	mapState.Mo(id).Mode = world.IDLE // Stop movement
}

// handleResourceDeath destroys a resource. Resources carry no coins — item
// drops only, with loot priority to the top extraction contributor.
func (s *Sim) handleResourceDeath(res world.EntityID, mapState *world.MapState) {
	s.killEntity(res, "resource", 0, mapState)
}

// handleDestructibleDeath depletes a destructible obstacle. It carries no coins
// and earns no XP; it frees its cells until it respawns.
func (s *Sim) handleDestructibleDeath(obs world.EntityID, mapState *world.MapState) {
	s.killEntity(obs, "obstacle", 0, mapState)
	tf := mapState.Tf(obs)
	mapState.Pathfinder().Occupy(tf.Pos, tf.Size, -1)
}

// handleRespawns checks and respawns dead players and bots.
func (s *Sim) handleRespawns(mapState *world.MapState) {
	// Respawn players
	for player := range s.PlayersOn(mapState) {
		if !mapState.Vt(player.ID).RespawnAt.IsZero() && time.Now().After(mapState.Vt(player.ID).RespawnAt) {
			mapState.Ap(player.ID).Layers = mapState.Mt(player.ID).PreRespawnObjectLayers
			mapState.Mt(player.ID).PreRespawnObjectLayers = nil
			// The restored loadout was snapshotted before the death coin drop, so
			// its coin slot is stale. Re-sync the display-only slot to the flat
			// balance (the source of truth) — otherwise the client sees a bogus
			// +N as the coin quantity jumps back up on respawn.
			s.SyncCoinOL(&mapState.Ap(player.ID).Layers, player.Coins)
			s.ApplyResistanceStat(player.ID, mapState) // Recalculate stats with restored items.
			mapState.Vt(player.ID).Life = mapState.Vt(player.ID).MaxLife
			mapState.Vt(player.ID).RespawnAt = time.Time{}
		}
	}

	// Respawn bots
	for id := range mapState.OfKind(world.EntityBot) {
		if !mapState.Vt(id).RespawnAt.IsZero() && time.Now().After(mapState.Vt(id).RespawnAt) {
			mapState.Ap(id).Layers = mapState.Mt(id).PreRespawnObjectLayers
			mapState.Mt(id).PreRespawnObjectLayers = nil
			s.ApplyResistanceStat(id, mapState) // Recalculate stats with restored items.
			mapState.Vt(id).Life = mapState.Vt(id).MaxLife
			mapState.Vt(id).RespawnAt = time.Time{}

			// Reset coin supply via the Fountain — single source of truth.
			s.FountainInitBot(id, mapState)
		}
	}

	// Respawn resources — restore original OLs and reset life.
	for res := range mapState.OfKind(world.EntityResource) {
		if !mapState.Vt(res).RespawnAt.IsZero() && time.Now().After(mapState.Vt(res).RespawnAt) {
			mapState.Ap(res).Layers = mapState.Mt(res).PreRespawnObjectLayers
			mapState.Mt(res).PreRespawnObjectLayers = nil
			s.ApplyResistanceStat(res, mapState)
			mapState.Vt(res).Life = mapState.Vt(res).MaxLife
			mapState.Vt(res).RespawnAt = time.Time{}
		}
	}

	// Restored obstacles block their cells before nearby entities move.
	for obs := range mapState.Destructibles() {
		v := mapState.Vt(obs)
		if !v.Ghost() || time.Now().Before(v.RespawnAt) {
			continue
		}
		mortal := mapState.Mt(obs)
		mapState.Ap(obs).Layers = mortal.PreRespawnObjectLayers
		mortal.PreRespawnObjectLayers = nil
		v.Life = v.MaxLife
		v.RespawnAt = time.Time{}
		tf := *mapState.Tf(obs)
		mapState.Pathfinder().Occupy(tf.Pos, tf.Size, 1)
		s.clearObstacleCells(mapState, tf)
	}
}

// Projectiles keep their course through restored obstacles.
func (s *Sim) clearObstacleCells(ms *world.MapState, obs world.Transform) {
	for player := range s.PlayersOn(ms) {
		tf := ms.Tf(player.ID)
		if !world.CheckAABBCollision(obs.Pos, obs.Size, tf.Pos, tf.Size) {
			if pathCrossesObstacle(player.Path, tf.Size, obs) {
				player.Path, ms.Mo(player.ID).Mode = nil, world.IDLE
				player.PendingMove = player.TargetPos
				player.HasPendingMove = true
				s.flushPendingMove(player, ms)
			}
			continue
		}
		if cell, err := obstaclePushCell(ms, obs, *tf, ms.Mo(player.ID).Direction); err == nil {
			tf.Pos = world.Vec2{X: float64(cell.X), Y: float64(cell.Y)}
			player.Path, player.TargetPos, ms.Mo(player.ID).Mode = nil, cell, world.IDLE
		}
	}
	for id := range ms.OfKind(world.EntityBot) {
		bot := ms.Br(id)
		if ms.Ag(id).Behavior == content.BehaviorSkill {
			continue
		}
		tf := ms.Tf(id)
		if world.CheckAABBCollision(obs.Pos, obs.Size, tf.Pos, tf.Size) {
			if cell, err := obstaclePushCell(ms, obs, *tf, ms.Mo(id).Direction); err == nil {
				tf.Pos = world.Vec2{X: float64(cell.X), Y: float64(cell.Y)}
				bot.TargetPos = cell
			}
		} else if !pathCrossesObstacle(bot.Path, tf.Size, obs) {
			continue
		}
		ms.Mo(id).Mode = world.IDLE
		bot.Path = nil
		bot.CurrentTargetPlayer = 0
		bot.LastPursuitTargetPos = world.Cell{X: -1, Y: -1}
	}
}

func obstaclePushCell(ms *world.MapState, obs, entity world.Transform, direction world.Direction) (world.Cell, error) {
	center := world.Vec2{X: obs.Pos.X + obs.Size.X*0.5, Y: obs.Pos.Y + obs.Size.Y*0.5}
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
	cell := world.Cell{X: int(math.Round(x)), Y: int(math.Round(y))}
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
	cell = world.ClampCellToGrid(cell, ms.GridW(), ms.GridH())
	return ms.Pathfinder().FindClosestWalkablePoint(cell, entity.Size)
}

func pathCrossesObstacle(path []world.Cell, size world.Vec2, obs world.Transform) bool {
	for _, cell := range path {
		if world.CheckAABBCollision(obs.Pos, obs.Size, world.Vec2{X: float64(cell.X), Y: float64(cell.Y)}, size) {
			return true
		}
	}
	return false
}
