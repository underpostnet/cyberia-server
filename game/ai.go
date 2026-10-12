package game

import (
	"math"
	"math/rand"
	"time"
)

// Provider NPCs barely move: when idle they take an occasional short step
// within a tight radius of their spawn (provider-static never moves at all).
const (
	providerWanderRadius        = 1.0
	providerWanderChancePerTick = 0.004
)

// randomPointWithinRadius returns a random walkable Cell within radius from center (or {-1,-1} on failure).
func (s *GameServer) randomPointWithinRadius(ms *MapState, center Vec2, radius float64, dims Vec2) Cell {
	for tries := 0; tries < 30; tries++ {
		ang := rand.Float64() * 2 * math.Pi
		r := rand.Float64() * radius
		x := int(math.Round(center.X + math.Cos(ang)*r))
		y := int(math.Round(center.Y + math.Sin(ang)*r))
		if x < 0 || x >= ms.gridW || y < 0 || y >= ms.gridH {
			continue
		}
		if ms.pathfinder.isWalkable(x, y, dims) {
			return Cell{X: x, Y: y}
		}
		if c, err := ms.pathfinder.findClosestWalkablePoint(Cell{X: x, Y: y}, dims); err == nil {
			return c
		}
	}
	return Cell{-1, -1}
}

// update bots per map
func (s *GameServer) updateBots(mapState *MapState) {
	for id, bot := range mapState.brains {
		// If bot is dead, skip all AI logic.
		if mapState.vt(id).ghost() {
			continue
		}

		// Recalculate stats to update MaxLife based on current equipment.
		s.ApplyResistanceStat(id, mapState)

		// Check for expiration
		if !bot.ExpiresAt.IsZero() && time.Now().After(bot.ExpiresAt) {
			mapState.despawn(id)
			continue
		}

		// Inert tokens (scattered loot drops) hold position: no wander, aggro,
		// or skills. Collection and lifetime are handled elsewhere (loot.go).
		if behaviorIsInert(mapState.ag(id).Behavior) {
			continue
		}

		botStats := s.CalculateStats(id, mapState)

		// --- SKILL PROJECTILE BEHAVIOR ---
		// Skill projectile bots move in a straight line and are removed if they go out of bounds.
		if mapState.ag(id).Behavior == BehaviorSkill {
			// dt-based integration. entityBaseSpeed is cells/second; one tick
			// is s.tickDuration. (The old code used `... / s.fps` which is a
			// frame-count integration that silently misbehaves when the loop
			// runs slow or tickRate changes.)
			projectileStep := s.entityBaseSpeed * s.projectileSpeedMultiplier * s.tickDuration.Seconds()
			// Calculate movement vector from bot's direction
			dirX, dirY := getDirectionVector(mapState.mo(id).Direction)

			// Update position
			mapState.tf(id).Pos.X += dirX * projectileStep
			mapState.tf(id).Pos.Y += dirY * projectileStep

			// Remove projectile if it goes out of map bounds
			if mapState.tf(id).Pos.X < 0 || mapState.tf(id).Pos.X > float64(mapState.gridW) ||
				mapState.tf(id).Pos.Y < 0 || mapState.tf(id).Pos.Y > float64(mapState.gridH) {
				mapState.despawn(id)
				continue // Skip further processing for this bot
			}
			s.handleBotSkills(id, mapState, Vec2{X: mapState.tf(id).Pos.X, Y: mapState.tf(id).Pos.Y})
			// For skill projectiles, we don't need to call updateBotPosition or other AI logic
			// as their movement is simple and direct.
			continue
		}
		// --- END SKILL PROJECTILE BEHAVIOR ---

		// --- PROVIDER BEHAVIOR (mission/action givers) ---
		// Immortal and near-stationary: never aggroes or casts. provider-static
		// stays put; provider takes an occasional short step near its spawn.
		if behaviorIsProvider(mapState.ag(id).Behavior) {
			if !behaviorIsStationary(mapState.ag(id).Behavior) {
				if (mapState.mo(id).Mode != WALKING || len(bot.Path) == 0) && rand.Float64() < providerWanderChancePerTick {
					target := s.randomPointWithinRadius(mapState, bot.SpawnCenter, providerWanderRadius, mapState.tf(id).Size)
					if target.X >= 0 {
						if pth, err := mapState.pathfinder.Astar(Cell{X: int(math.Round(mapState.tf(id).Pos.X)), Y: int(math.Round(mapState.tf(id).Pos.Y))}, target, mapState.tf(id).Size); err == nil && len(pth) > 0 {
							bot.Path = pth
							bot.TargetPos = target
							mapState.mo(id).Mode = WALKING
						}
					}
				}
				s.updateBotPosition(id, mapState, botStats)
			}
			continue
		}
		// --- END PROVIDER BEHAVIOR ---

		// Hostile logic: check for nearest player in aggro range
		if mapState.ag(id).Behavior == BehaviorHostile {
			var nearestID EntityID
			nearestDist := math.MaxFloat64
			var nearestPlayer *PlayerState
			for _, p := range mapState.players {
				// Don't target dead/ghost players
				if mapState.vt(p.ID).Life <= 0 || mapState.vt(p.ID).ghost() {
					continue
				}

				dx := mapState.tf(p.ID).Pos.X - mapState.tf(id).Pos.X
				dy := mapState.tf(p.ID).Pos.Y - mapState.tf(id).Pos.Y
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
				dirX := mapState.tf(nearestPlayer.ID).Pos.X - mapState.tf(id).Pos.X
				dirY := mapState.tf(nearestPlayer.ID).Pos.Y - mapState.tf(id).Pos.Y
				if dirX != 0 || dirY != 0 {
					mapState.mo(id).Direction = directionFromVector(dirX, dirY)
				}

				// Hostile bots that are idle (not moving) have a chance to use a skill.
				if mapState.mo(id).Mode == IDLE && bot.ExpiresAt.IsZero() {
					cooldown := s.CalculateActionCooldown(botStats)
					if time.Since(bot.lastAction) >= cooldown {
						s.handleBotSkills(id, mapState, mapState.tf(nearestPlayer.ID).Pos)
						bot.lastAction = time.Now()
					}
				}

				// Pursue the player: recompute path if newly acquired or player moved cell
				playerCell := Cell{X: int(math.Round(mapState.tf(nearestPlayer.ID).Pos.X)), Y: int(math.Round(mapState.tf(nearestPlayer.ID).Pos.Y))}
				needRepath := bot.CurrentTargetPlayer != nearestID || bot.lastPursuitTargetPos != playerCell
				if needRepath {
					start := Cell{X: int(math.Round(mapState.tf(id).Pos.X)), Y: int(math.Round(mapState.tf(id).Pos.Y))}
					target, err := mapState.pathfinder.findClosestWalkablePoint(playerCell, mapState.tf(id).Size)
					if err == nil {
						if p, err2 := mapState.pathfinder.Astar(start, target, mapState.tf(id).Size); err2 == nil && len(p) > 0 {
							bot.Path = p
							bot.TargetPos = target
							mapState.mo(id).Mode = WALKING
							bot.CurrentTargetPlayer = nearestID
							bot.lastPursuitTargetPos = playerCell

							// Trigger probabilistic regen on acquiring a target.
							s.handleProbabilisticRegen(id, mapState)

							// Skill activation for permanent bots acquiring a player target
							if bot.ExpiresAt.IsZero() {
								cooldown := s.CalculateActionCooldown(botStats)
								if time.Since(bot.lastAction) >= cooldown {
									s.handleBotSkills(id, mapState, mapState.tf(nearestPlayer.ID).Pos)
									bot.lastAction = time.Now()
								}

							}
						}
					}
				}
			} else {
				// No player in range: clear pursuit (if any) and ensure wandering behavior
				if bot.CurrentTargetPlayer != 0 {
					bot.CurrentTargetPlayer = 0
					bot.lastPursuitTargetPos = Cell{-1, -1}
				}

				// IMPORTANT CHANGE: When not pursuing, hostile bots behave like passive bots:
				// - if they have no current walking path, generate a new random wander target within spawn radius.
				if mapState.mo(id).Mode != WALKING || len(bot.Path) == 0 {
					target := s.randomPointWithinRadius(mapState, bot.SpawnCenter, bot.SpawnRadius, mapState.tf(id).Size)
					if target.X >= 0 {
						if pth, err := mapState.pathfinder.Astar(Cell{X: int(math.Round(mapState.tf(id).Pos.X)), Y: int(math.Round(mapState.tf(id).Pos.Y))}, target, mapState.tf(id).Size); err == nil && len(pth) > 0 {
							bot.Path = pth
							bot.TargetPos = target
							mapState.mo(id).Mode = WALKING

							// Trigger probabilistic regen on wander.
							s.handleProbabilisticRegen(id, mapState)

							// Skill activation for permanent bots taking a wander "action"
							if bot.ExpiresAt.IsZero() {
								cooldown := s.CalculateActionCooldown(botStats)
								if time.Since(bot.lastAction) >= cooldown {
									s.handleBotSkills(id, mapState, Vec2{X: float64(target.X), Y: float64(target.Y)})
									bot.lastAction = time.Now()
								}
							}
						} else {
							// if we can't path to a target, remain idle until next tick tries again
							if len(bot.Path) == 0 {
								mapState.mo(id).Mode = IDLE
							}
						}
					}
				}
			}
		} else {
			// Passive: if no path, create wandering path
			if len(bot.Path) == 0 || mapState.mo(id).Mode != WALKING {
				target := s.randomPointWithinRadius(mapState, bot.SpawnCenter, bot.SpawnRadius, mapState.tf(id).Size)
				if target.X >= 0 {
					if pth, err := mapState.pathfinder.Astar(Cell{X: int(math.Round(mapState.tf(id).Pos.X)), Y: int(math.Round(mapState.tf(id).Pos.Y))}, target, mapState.tf(id).Size); err == nil && len(pth) > 0 {
						bot.Path = pth
						bot.TargetPos = target
						mapState.mo(id).Mode = WALKING

						// Trigger probabilistic regen on wander.
						s.handleProbabilisticRegen(id, mapState)

						// Skill activation for permanent bots taking a wander "action"
						if bot.ExpiresAt.IsZero() {
							cooldown := s.CalculateActionCooldown(botStats)
							if time.Since(bot.lastAction) >= cooldown {
								s.handleBotSkills(id, mapState, Vec2{X: float64(target.X), Y: float64(target.Y)})
								bot.lastAction = time.Now()
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
func (s *GameServer) updateBotPosition(id EntityID, mapState *MapState, stats ComputedStats) {
	bot := mapState.br(id)
	if mapState.mo(id).Mode != WALKING {
		return
	}
	speed := s.CalculateMovementSpeed(stats)

	pos, path, dir, facing := stepAlongPath(mapState.tf(id).Pos, bot.Path, speed, s.tickDuration.Seconds())
	mapState.tf(id).Pos, bot.Path = pos, path
	if facing {
		mapState.mo(id).Direction = dir
	}
	if len(bot.Path) == 0 {
		mapState.mo(id).Mode = IDLE
	}
}

// getDirectionVector converts a Direction enum to a normalized (dx, dy) vector.
// This helper is used for calculating projectile trajectories.
func getDirectionVector(dir Direction) (float64, float64) {
	// Normalize diagonal vectors to maintain consistent speed
	const diag = 0.70710678118 // 1 / sqrt(2)
	switch dir {
	case UP:
		return 0, -1
	case UP_RIGHT:
		return diag, -diag
	case RIGHT:
		return 1, 0
	case DOWN_RIGHT:
		return diag, diag
	case DOWN:
		return 0, 1
	case DOWN_LEFT:
		return -diag, diag
	case LEFT:
		return -1, 0
	case UP_LEFT:
		return -diag, -diag
	default: // NONE or any other unexpected direction
		return 0, 0
	}
}
