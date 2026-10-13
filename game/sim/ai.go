package sim

import (
	"math"
	"math/rand"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// Provider NPCs barely move: when idle they take an occasional short step
// within a tight radius of their spawn (provider-static never moves at all).
const (
	providerWanderRadius        = 1.0
	providerWanderChancePerTick = 0.004
)

// randomPointWithinRadius returns a random walkable Cell within radius from center (or {-1,-1} on failure).
func (s *Sim) randomPointWithinRadius(ms *world.MapState, center world.Vec2, radius float64, dims world.Vec2) world.Cell {
	for tries := 0; tries < 30; tries++ {
		ang := rand.Float64() * 2 * math.Pi
		r := rand.Float64() * radius
		x := int(math.Round(center.X + math.Cos(ang)*r))
		y := int(math.Round(center.Y + math.Sin(ang)*r))
		if x < 0 || x >= ms.GridW() || y < 0 || y >= ms.GridH() {
			continue
		}
		if ms.Pathfinder().IsWalkable(x, y, dims) {
			return world.Cell{X: x, Y: y}
		}
		if c, err := ms.Pathfinder().FindClosestWalkablePoint(world.Cell{X: x, Y: y}, dims); err == nil {
			return c
		}
	}
	return world.Cell{X: -1, Y: -1}
}

// update bots per map
func (s *Sim) updateBots(mapState *world.MapState) {
	for id := range mapState.OfKind(world.EntityBot) {
		bot := mapState.Br(id)
		// If bot is dead, skip all AI logic.
		if mapState.Vt(id).Ghost() {
			continue
		}

		// Recalculate stats to update MaxLife based on current equipment.
		s.ApplyResistanceStat(id, mapState)

		// Check for expiration
		if !bot.ExpiresAt.IsZero() && time.Now().After(bot.ExpiresAt) {
			s.Despawn(mapState, id)
			continue
		}

		// Inert tokens (scattered loot drops) hold position: no wander, aggro,
		// or skills. Collection and lifetime are handled elsewhere (loot.go).
		if content.BehaviorIsInert(mapState.Ag(id).Behavior) {
			continue
		}

		botStats := s.CalculateStats(id, mapState)

		// --- SKILL PROJECTILE BEHAVIOR ---
		// Skill projectile bots move in a straight line and are removed if they go out of bounds.
		if mapState.Ag(id).Behavior == content.BehaviorSkill {
			// dt-based integration. entityBaseSpeed is cells/second; one tick
			// is tickDuration. (The old code used `... / s.fps` which is a
			// frame-count integration that silently misbehaves when the loop
			// runs slow or tickRate changes.)
			projectileStep := s.EntityBaseSpeed * s.projectileSpeedMultiplier * TickDuration.Seconds()
			// Calculate movement vector from bot's direction
			dirX, dirY := getDirectionVector(mapState.Mo(id).Direction)

			// Update position
			mapState.Tf(id).Pos.X += dirX * projectileStep
			mapState.Tf(id).Pos.Y += dirY * projectileStep

			// Remove projectile if it goes out of map bounds
			if mapState.Tf(id).Pos.X < 0 || mapState.Tf(id).Pos.X > float64(mapState.GridW()) ||
				mapState.Tf(id).Pos.Y < 0 || mapState.Tf(id).Pos.Y > float64(mapState.GridH()) {
				s.Despawn(mapState, id)
				continue // Skip further processing for this bot
			}
			s.handleBotSkills(id, mapState, world.Vec2{X: mapState.Tf(id).Pos.X, Y: mapState.Tf(id).Pos.Y})
			// For skill projectiles, we don't need to call updateBotPosition or other AI logic
			// as their movement is simple and direct.
			continue
		}
		// --- END SKILL PROJECTILE BEHAVIOR ---

		// --- PROVIDER BEHAVIOR (mission/action givers) ---
		// Immortal and near-stationary: never aggroes or casts. provider-static
		// stays put; provider takes an occasional short step near its spawn.
		if content.BehaviorIsProvider(mapState.Ag(id).Behavior) {
			if !content.BehaviorIsStationary(mapState.Ag(id).Behavior) {
				if (mapState.Mo(id).Mode != world.WALKING || len(bot.Path) == 0) && rand.Float64() < providerWanderChancePerTick {
					target := s.randomPointWithinRadius(mapState, bot.SpawnCenter, providerWanderRadius, mapState.Tf(id).Size)
					if target.X >= 0 {
						if pth, err := mapState.Pathfinder().Astar(world.Cell{X: int(math.Round(mapState.Tf(id).Pos.X)), Y: int(math.Round(mapState.Tf(id).Pos.Y))}, target, mapState.Tf(id).Size); err == nil && len(pth) > 0 {
							bot.Path = pth
							bot.TargetPos = target
							mapState.Mo(id).Mode = world.WALKING
						}
					}
				}
				s.updateBotPosition(id, mapState, botStats)
			}
			continue
		}
		// --- END PROVIDER BEHAVIOR ---

		// Hostile logic: check for nearest player in aggro range
		if mapState.Ag(id).Behavior == content.BehaviorHostile {
			var nearestID world.EntityID
			nearestDist := math.MaxFloat64
			var nearestPlayer *world.Session
			for p := range s.PlayersOn(mapState) {
				// Don't target dead/ghost players
				if mapState.Vt(p.ID).Life <= 0 || mapState.Vt(p.ID).Ghost() {
					continue
				}

				dx := mapState.Tf(p.ID).Pos.X - mapState.Tf(id).Pos.X
				dy := mapState.Tf(p.ID).Pos.Y - mapState.Tf(id).Pos.Y
				dist := math.Sqrt(dx*dx + dy*dy)
				if dist < nearestDist {
					nearestDist = dist
					nearestID = p.ID
					nearestPlayer = p
				}
			}
			if nearestPlayer != nil && nearestDist <= bot.AggroRange {
				// Always face the player when in aggro range.
				// This makes the bot appear to be "aiming", especially when idle.
				// If the bot is walking, this direction will be updated by updateBotPosition
				// to follow the path, which is the desired behavior.
				dirX := mapState.Tf(nearestPlayer.ID).Pos.X - mapState.Tf(id).Pos.X
				dirY := mapState.Tf(nearestPlayer.ID).Pos.Y - mapState.Tf(id).Pos.Y
				if dirX != 0 || dirY != 0 {
					mapState.Mo(id).Direction = directionFromVector(dirX, dirY)
				}

				// Hostile bots that are idle (not moving) have a chance to use a skill.
				if mapState.Mo(id).Mode == world.IDLE && bot.ExpiresAt.IsZero() {
					cooldown := s.CalculateActionCooldown(botStats)
					if time.Since(bot.LastAction) >= cooldown {
						s.handleBotSkills(id, mapState, mapState.Tf(nearestPlayer.ID).Pos)
						bot.LastAction = time.Now()
					}
				}

				// Pursue the player: recompute path if newly acquired or player moved cell
				playerCell := world.Cell{X: int(math.Round(mapState.Tf(nearestPlayer.ID).Pos.X)), Y: int(math.Round(mapState.Tf(nearestPlayer.ID).Pos.Y))}
				needRepath := bot.CurrentTargetPlayer != nearestID || bot.LastPursuitTargetPos != playerCell
				if needRepath {
					start := world.Cell{X: int(math.Round(mapState.Tf(id).Pos.X)), Y: int(math.Round(mapState.Tf(id).Pos.Y))}
					target, err := mapState.Pathfinder().FindClosestWalkablePoint(playerCell, mapState.Tf(id).Size)
					if err == nil {
						if p, err2 := mapState.Pathfinder().Astar(start, target, mapState.Tf(id).Size); err2 == nil && len(p) > 0 {
							bot.Path = p
							bot.TargetPos = target
							mapState.Mo(id).Mode = world.WALKING
							bot.CurrentTargetPlayer = nearestID
							bot.LastPursuitTargetPos = playerCell

							// Trigger probabilistic regen on acquiring a target.
							s.handleProbabilisticRegen(id, mapState)

							// Skill activation for permanent bots acquiring a player target
							if bot.ExpiresAt.IsZero() {
								cooldown := s.CalculateActionCooldown(botStats)
								if time.Since(bot.LastAction) >= cooldown {
									s.handleBotSkills(id, mapState, mapState.Tf(nearestPlayer.ID).Pos)
									bot.LastAction = time.Now()
								}

							}
						}
					}
				}
			} else {
				// No player in range: clear pursuit (if any) and ensure wandering behavior
				if bot.CurrentTargetPlayer != 0 {
					bot.CurrentTargetPlayer = 0
					bot.LastPursuitTargetPos = world.Cell{X: -1, Y: -1}
				}

				// IMPORTANT CHANGE: When not pursuing, hostile bots behave like passive bots:
				// - if they have no current walking path, generate a new random wander target within spawn radius.
				if mapState.Mo(id).Mode != world.WALKING || len(bot.Path) == 0 {
					target := s.randomPointWithinRadius(mapState, bot.SpawnCenter, bot.SpawnRadius, mapState.Tf(id).Size)
					if target.X >= 0 {
						if pth, err := mapState.Pathfinder().Astar(world.Cell{X: int(math.Round(mapState.Tf(id).Pos.X)), Y: int(math.Round(mapState.Tf(id).Pos.Y))}, target, mapState.Tf(id).Size); err == nil && len(pth) > 0 {
							bot.Path = pth
							bot.TargetPos = target
							mapState.Mo(id).Mode = world.WALKING

							// Trigger probabilistic regen on wander.
							s.handleProbabilisticRegen(id, mapState)

							// Skill activation for permanent bots taking a wander "action"
							if bot.ExpiresAt.IsZero() {
								cooldown := s.CalculateActionCooldown(botStats)
								if time.Since(bot.LastAction) >= cooldown {
									s.handleBotSkills(id, mapState, world.Vec2{X: float64(target.X), Y: float64(target.Y)})
									bot.LastAction = time.Now()
								}
							}
						} else {
							// if we can't path to a target, remain idle until next tick tries again
							if len(bot.Path) == 0 {
								mapState.Mo(id).Mode = world.IDLE
							}
						}
					}
				}
			}
		} else {
			// Passive: if no path, create wandering path
			if len(bot.Path) == 0 || mapState.Mo(id).Mode != world.WALKING {
				target := s.randomPointWithinRadius(mapState, bot.SpawnCenter, bot.SpawnRadius, mapState.Tf(id).Size)
				if target.X >= 0 {
					if pth, err := mapState.Pathfinder().Astar(world.Cell{X: int(math.Round(mapState.Tf(id).Pos.X)), Y: int(math.Round(mapState.Tf(id).Pos.Y))}, target, mapState.Tf(id).Size); err == nil && len(pth) > 0 {
						bot.Path = pth
						bot.TargetPos = target
						mapState.Mo(id).Mode = world.WALKING

						// Trigger probabilistic regen on wander.
						s.handleProbabilisticRegen(id, mapState)

						// Skill activation for permanent bots taking a wander "action"
						if bot.ExpiresAt.IsZero() {
							cooldown := s.CalculateActionCooldown(botStats)
							if time.Since(bot.LastAction) >= cooldown {
								s.handleBotSkills(id, mapState, world.Vec2{X: float64(target.X), Y: float64(target.Y)})
								bot.LastAction = time.Now()
							}
						}
					}
				}
			}
		}
		// Move bot along its path
		s.updateBotPosition(id, mapState, botStats)
	}
}

// update bot movement similar to players
func (s *Sim) updateBotPosition(id world.EntityID, mapState *world.MapState, stats content.ComputedStats) {
	bot := mapState.Br(id)
	if mapState.Mo(id).Mode != world.WALKING {
		return
	}
	speed := s.CalculateMovementSpeed(stats)

	pos, path, dir, facing := stepAlongPath(mapState.Tf(id).Pos, bot.Path, speed, TickDuration.Seconds())
	mapState.Tf(id).Pos, bot.Path = pos, path
	if facing {
		mapState.Mo(id).Direction = dir
	}
	if len(bot.Path) == 0 {
		mapState.Mo(id).Mode = world.IDLE
	}
}

// getDirectionVector converts a Direction enum to a normalized (dx, dy) vector.
// This helper is used for calculating projectile trajectories.
func getDirectionVector(dir world.Direction) (float64, float64) {
	// Normalize diagonal vectors to maintain consistent speed
	const diag = 0.70710678118 // 1 / sqrt(2)
	switch dir {
	case world.UP:
		return 0, -1
	case world.UP_RIGHT:
		return diag, -diag
	case world.RIGHT:
		return 1, 0
	case world.DOWN_RIGHT:
		return diag, diag
	case world.DOWN:
		return 0, 1
	case world.DOWN_LEFT:
		return -diag, diag
	case world.LEFT:
		return -1, 0
	case world.UP_LEFT:
		return -diag, -diag
	default: // NONE or any other unexpected direction
		return 0, 0
	}
}
