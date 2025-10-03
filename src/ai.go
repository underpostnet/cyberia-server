package game

import (
	"log"
	"math"
	"math/rand"
	"time"
)

// randomPointWithinRadius returns a random walkable PointI within radius from center (or {-1,-1} on failure).
func (s *GameServer) randomPointWithinRadius(ms *MapState, center Point, radius float64, dims Dimensions) PointI {
	for tries := 0; tries < 30; tries++ {
		ang := rand.Float64() * 2 * math.Pi
		r := rand.Float64() * radius
		x := int(math.Round(center.X + math.Cos(ang)*r))
		y := int(math.Round(center.Y + math.Sin(ang)*r))
		if x < 0 || x >= ms.gridW || y < 0 || y >= ms.gridH {
			continue
		}
		if ms.pathfinder.isWalkable(x, y, dims) {
			return PointI{X: x, Y: y}
		}
		if c, err := ms.pathfinder.findClosestWalkablePoint(PointI{X: x, Y: y}, dims); err == nil {
			return c
		}
	}
	return PointI{-1, -1}
}

// update bots per map
func (s *GameServer) updateBots(mapState *MapState) {
	for botID, bot := range mapState.bots {
		// If bot is dead, skip all AI logic.
		if bot.IsGhost() {
			continue
		}

		// Recalculate stats to update MaxLife based on current equipment.
		s.ApplyResistanceStat(bot, mapState)

		// Check for expiration
		if !bot.ExpiresAt.IsZero() && time.Now().After(bot.ExpiresAt) {
			delete(mapState.bots, botID)
			continue
		}

		// --- NEW BULLET BEHAVIOR ---
		// Bullet bots move in a straight line and are removed if they go out of bounds.
		if bot.Behavior == "bullet" {
			// Bullets move at twice player speed. playerSpeed is in units per second.
			// The game loop runs at 10 FPS (100ms per tick), so we divide by 10 to get movement per tick.
			bulletStep := (s.playerSpeed * 2.0) / 10.0
			// Calculate movement vector from bot's direction
			dirX, dirY := getDirectionVector(bot.Direction)

			// Update position
			bot.Pos.X += dirX * bulletStep
			bot.Pos.Y += dirY * bulletStep

			// Remove bullet if it goes out of map bounds
			if bot.Pos.X < 0 || bot.Pos.X > float64(mapState.gridW) ||
				bot.Pos.Y < 0 || bot.Pos.Y > float64(mapState.gridH) {
				delete(mapState.bots, botID)
				continue // Skip further processing for this bot
			}
			// For bullets, we don't need to call updateBotPosition or other AI logic
			// as their movement is simple and direct.
			continue
		}
		// --- END NEW BULLET BEHAVIOR ---

		// Hostile logic: check for nearest player in aggro range
		if bot.Behavior == "hostile" {
			nearestID := ""
			nearestDist := math.MaxFloat64
			var nearestPlayer *PlayerState
			for _, p := range mapState.players {
				// Don't target dead/ghost players
				if p.Life <= 0 || p.IsGhost() {
					continue
				}

				dx := p.Pos.X - bot.Pos.X
				dy := p.Pos.Y - bot.Pos.Y
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
				dirX := nearestPlayer.Pos.X - bot.Pos.X
				dirY := nearestPlayer.Pos.Y - bot.Pos.Y
				if dirX != 0 || dirY != 0 {
					s.updateBotDirection(bot, dirX, dirY)
				}

				// Hostile bots that are idle (not moving) have a chance to use a skill.
				if bot.Mode == IDLE && bot.ExpiresAt.IsZero() {
					botStats := s.CalculateStats(bot, mapState)
					cooldown := s.CalculateActionCooldown(botStats)
					if time.Since(bot.lastAction) >= cooldown {
						s.handleBotSkills(bot, mapState, nearestPlayer.Pos)
						bot.lastAction = time.Now()
					}
				}

				// Pursue the player: recompute path if newly acquired or player moved cell
				playerCell := PointI{X: int(math.Round(nearestPlayer.Pos.X)), Y: int(math.Round(nearestPlayer.Pos.Y))}
				needRepath := bot.CurrentTargetPlayer != nearestID || bot.lastPursuitTargetPos != playerCell
				if needRepath {
					start := PointI{X: int(math.Round(bot.Pos.X)), Y: int(math.Round(bot.Pos.Y))}
					target, err := mapState.pathfinder.findClosestWalkablePoint(playerCell, bot.Dims)
					if err == nil {
						if p, err2 := mapState.pathfinder.Astar(start, target, bot.Dims); err2 == nil && len(p) > 0 {
							bot.Path = p
							bot.TargetPos = target
							bot.Mode = WALKING
							bot.CurrentTargetPlayer = nearestID
							bot.lastPursuitTargetPos = playerCell

							// Trigger probabilistic regen on acquiring a target.
							s.handleProbabilisticRegen(bot, mapState)

							// Skill activation for permanent bots acquiring a player target
							if bot.ExpiresAt.IsZero() {
								botStats := s.CalculateStats(bot, mapState)
								cooldown := s.CalculateActionCooldown(botStats)
								if time.Since(bot.lastAction) >= cooldown {
									s.handleBotSkills(bot, mapState, nearestPlayer.Pos)
									bot.lastAction = time.Now()
								}

							}
						}
					}
				}
			} else {
				// No player in range: clear pursuit (if any) and ensure wandering behavior
				if bot.CurrentTargetPlayer != "" {
					bot.CurrentTargetPlayer = ""
					bot.lastPursuitTargetPos = PointI{-1, -1}
				}

				// IMPORTANT CHANGE: When not pursuing, hostile bots behave like passive bots:
				// - if they have no current walking path, generate a new random wander target within spawn radius.
				if bot.Mode != WALKING || len(bot.Path) == 0 {
					target := s.randomPointWithinRadius(mapState, bot.SpawnCenter, bot.SpawnRadius, bot.Dims)
					if target.X >= 0 {
						if pth, err := mapState.pathfinder.Astar(PointI{X: int(math.Round(bot.Pos.X)), Y: int(math.Round(bot.Pos.Y))}, target, bot.Dims); err == nil && len(pth) > 0 {
							bot.Path = pth
							bot.TargetPos = target
							bot.Mode = WALKING

							// Trigger probabilistic regen on wander.
							s.handleProbabilisticRegen(bot, mapState)

							// Skill activation for permanent bots taking a wander "action"
							if bot.ExpiresAt.IsZero() {
								botStats := s.CalculateStats(bot, mapState)
								cooldown := s.CalculateActionCooldown(botStats)
								if time.Since(bot.lastAction) >= cooldown {
									s.handleBotSkills(bot, mapState, Point{X: float64(target.X), Y: float64(target.Y)})
									bot.lastAction = time.Now()
								}
							}
						} else {
							// if we can't path to a target, remain idle until next tick tries again
							if len(bot.Path) == 0 {
								bot.Mode = IDLE
							}
						}
					}
				}
			}
		} else {
			// Passive: if no path, create wandering path
			if len(bot.Path) == 0 || bot.Mode != WALKING {
				target := s.randomPointWithinRadius(mapState, bot.SpawnCenter, bot.SpawnRadius, bot.Dims)
				if target.X >= 0 {
					if pth, err := mapState.pathfinder.Astar(PointI{X: int(math.Round(bot.Pos.X)), Y: int(math.Round(bot.Pos.Y))}, target, bot.Dims); err == nil && len(pth) > 0 {
						bot.Path = pth
						bot.TargetPos = target
						bot.Mode = WALKING

						// Trigger probabilistic regen on wander.
						s.handleProbabilisticRegen(bot, mapState)

						// Skill activation for permanent bots taking a wander "action"
						if bot.ExpiresAt.IsZero() {
							botStats := s.CalculateStats(bot, mapState)
							cooldown := s.CalculateActionCooldown(botStats)
							if time.Since(bot.lastAction) >= cooldown {
								s.handleBotSkills(bot, mapState, Point{X: float64(target.X), Y: float64(target.Y)})
								bot.lastAction = time.Now()
							}
						}
					}
				}
			}
		}
		// Move bot along its path
		s.updateBotPosition(bot, mapState)

		// --- LAYER RESTRICTION & VALIDATION LOGIC ---
		activeLayerCount := 0
		activeLayerTypes := make(map[string]bool)
		hasAnySkin := false
		firstSkinIndex := -1
		activeSkinCount := 0

		// First pass: enforce max layers and unique types by deactivating violators.
		for i := range bot.ObjectLayers {
			layer := &bot.ObjectLayers[i] // Work with a pointer to modify
			if !layer.Active {
				continue
			}

			var itemType string
			if itemData, ok := s.objectLayerDataCache[layer.ItemID]; ok {
				itemType = itemData.Data.Item.Type
			} else {
				itemType = "generic"
			}

			// Rule: unique types for active layers.
			if _, exists := activeLayerTypes[itemType]; exists {
				log.Printf("Bot %s has a duplicate active item type '%s' with item '%s'. Deactivating.", bot.ID, itemType, layer.ItemID)
				layer.Active = false
				continue
			}

			// Rule: max 4 active layers.
			if activeLayerCount >= 4 {
				log.Printf("Bot %s has more than 4 active layers. Deactivating item '%s'.", bot.ID, layer.ItemID)
				layer.Active = false
				continue
			}

			activeLayerCount++
			activeLayerTypes[itemType] = true
		}

		// Second pass: ensure bot has at least one active skin. This is a safeguard.
		for i, layer := range bot.ObjectLayers {
			var isSkin bool
			if itemData, ok := s.objectLayerDataCache[layer.ItemID]; ok {
				isSkin = itemData.Data.Item.Type == "skin"
			}

			if !isSkin {
				continue
			}

			hasAnySkin = true
			if firstSkinIndex == -1 {
				firstSkinIndex = i
			}

			if layer.Active {
				activeSkinCount++
			}
		}

		if hasAnySkin && activeSkinCount == 0 && firstSkinIndex != -1 {
			// No active skins, but at least one skin exists. Activate the first one found.
			bot.ObjectLayers[firstSkinIndex].Active = true
			log.Printf("Bot %s had no active skins. Force-activated skin '%s'.", bot.ID, bot.ObjectLayers[firstSkinIndex].ItemID)
		}
	}
}

// update bot movement similar to players
func (s *GameServer) updateBotPosition(bot *BotState, mapState *MapState) {
	if bot.Mode == WALKING && len(bot.Path) > 0 {
		targetNode := bot.Path[0]
		dx := float64(targetNode.X) - bot.Pos.X
		dy := float64(targetNode.Y) - bot.Pos.Y
		dist := math.Sqrt(dx*dx + dy*dy)
		step := s.playerSpeed / 10.0

		if dist < step {
			bot.Pos = Point{X: float64(targetNode.X), Y: float64(targetNode.Y)}
			bot.Path = bot.Path[1:]
			if len(bot.Path) == 0 {
				bot.Mode = IDLE
			} else {
				next := bot.Path[0]
				dirX := float64(next.X) - bot.Pos.X
				dirY := float64(next.Y) - bot.Pos.Y
				norm := math.Sqrt(dirX*dirX + dirY*dirY)
				if norm > 0 {
					dirX /= norm
					dirY /= norm
					s.updateBotDirection(bot, dirX, dirY)
				}
			}
		} else {
			dirX, dirY := dx/dist, dy/dist
			bot.Pos.X += dirX * step
			bot.Pos.Y += dirY * step
			s.updateBotDirection(bot, dirX, dirY)
		}
	}
}

func (s *GameServer) updateBotDirection(bot *BotState, dirX, dirY float64) {
	angle := math.Atan2(dirY, dirX)
	if angle < 0 {
		angle += 2 * math.Pi
	}
	directionIndex := (int(math.Round(angle/(math.Pi/4))) + 2) % 8
	bot.Direction = Direction(directionIndex)
}

// getDirectionVector converts a Direction enum to a normalized (dx, dy) vector.
// This helper is used for calculating bullet trajectories.
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
