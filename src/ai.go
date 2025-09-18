package game

import (
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
		// Check for expiration
		if !bot.ExpiresAt.IsZero() && time.Now().After(bot.ExpiresAt) {
			delete(mapState.bots, botID)
			continue
		}

		// Hostile logic: check for nearest player in aggro range
		if bot.Behavior == "hostile" {
			nearestID := ""
			nearestDist := math.MaxFloat64
			var nearestPlayer *PlayerState
			for _, p := range mapState.players {
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
					}
				}
			}
		}
		// Move bot along its path
		s.updateBotPosition(bot, mapState)
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
				bot.Direction = NONE
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
