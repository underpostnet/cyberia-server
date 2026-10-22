package sim

import (
	"math"
	"math/rand"
	"time"

	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// directionFromVector maps a movement vector onto one of the eight facings.
func directionFromVector(dirX, dirY float64) world.Direction {
	angle := math.Atan2(dirY, dirX)
	if angle < 0 {
		angle += 2 * math.Pi
	}
	return world.Direction((int(math.Round(angle/(math.Pi/4))) + 2) % 8)
}

// stepAlongPath advances pos toward the head of path by speed*dt cells.
//
// dt-based integration: speed is cells/second and dt is the authoritative
// simulation step (1 / tickRate). A frame count instead of dt silently breaks
// movement whenever the loop runs slow or the tickRate config changes.
//
// The rest of the step after a node walks the next leg, so the speed is the
// same on every leg. The client self_motion_step mirrors this function.
//
// It returns the new position, the path left to walk, and the facing for this
// step. The bool is false when the walk ends, so the caller decides what an
// arriving entity faces.
func stepAlongPath(pos world.Vec2, path []world.Cell, speed, dt float64) (world.Vec2, []world.Cell, world.Direction, bool) {
	step := speed * dt // cells per simulation tick
	for len(path) > 0 {
		node := path[0]
		dx := float64(node.X) - pos.X
		dy := float64(node.Y) - pos.Y
		dist := math.Sqrt(dx*dx + dy*dy)
		if dist > step {
			dirX, dirY := dx/dist, dy/dist
			pos.X += dirX * step
			pos.Y += dirY * step
			return pos, path, directionFromVector(dirX, dirY), true
		}
		pos = world.Vec2{X: float64(node.X), Y: float64(node.Y)}
		path = path[1:]
		step -= dist
	}
	return pos, path, world.NONE, false
}

func (s *Sim) updatePlayerPosition(player *world.Session, mapState *world.MapState) {
	// Dead players can't move.
	if mapState.Vt(player.ID).Ghost() {
		return
	}

	playerStats := s.CalculateStats(player.ID, mapState)
	speed := s.CalculatePlayerMovementSpeed(playerStats)

	if mapState.Mo(player.ID).Mode != world.WALKING {
		return
	}

	tf := mapState.Tf(player.ID)
	pos, path, dir, facing := stepAlongPath(tf.Pos, player.Path, speed, TickDuration.Seconds())
	tf.Pos, player.Path = pos, path
	if facing {
		mapState.Mo(player.ID).Direction = dir
	}
	if len(player.Path) == 0 {
		mapState.Mo(player.ID).Mode = world.IDLE
		mapState.Mo(player.ID).Direction = world.NONE
	}
}

// ---------- Portals ----------
func (s *Sim) checkPortal(player *world.Session, mapState *world.MapState) {
	// Teleport is only possible while alive. A dead player standing on a portal
	// neither accrues hold progress nor teleports; entering ghost state cancels
	// any in-progress hold.
	if mapState.Vt(player.ID).Ghost() {
		player.OnPortal = false
		return
	}

	onPortal := false
	var activePortal world.EntityID
	playerRect := mapState.Tf(player.ID).Rect()
	for portal := range mapState.OfKind(world.EntityPortal) {
		if world.RectsOverlap(playerRect, mapState.Tf(portal).Rect()) {
			onPortal = true
			activePortal = portal
			break
		}
	}
	if onPortal {
		quote := world.PortalQuote{Portal: activePortal, Fee: s.PortalFee(mapState, activePortal), Economy: s.EconomyKey()}
		switch {
		case !player.OnPortal || player.PortalQuote != quote:
			// A new hold, or a fee that changed during the hold: the hold starts again at the fee shown.
			player.OnPortal, player.TimeOnPortal, player.PortalQuote = true, time.Now(), quote
		case time.Since(player.TimeOnPortal) > s.PortalHoldTime:
			// A player that cannot pay the fee stays, and hears why once per hold.
			if reason := s.PortalFeeRefusal(player, quote.Fee); reason != "" {
				player.TimeOnPortal = time.Now()
				world.SendAck(player, "portal_ack", reason, map[string]interface{}{"fee": quote.Fee})
				return
			}
			if s.teleportPlayer(player, mapState, activePortal) {
				s.ChargePortalFee(player, quote.Fee)
			}
		}
	} else {
		player.OnPortal = false
	}
}

// teleportPlayer moves the player through a portal and reports whether it moved.
func (s *Sim) teleportPlayer(player *world.Session, mapState *world.MapState, portal world.EntityID) bool {
	// s.mu is held by the caller (phasePortals). Do not release it: map
	// mutations of the map slots must not race other readers.

	target, linked := mapState.Pt(portal)
	if !linked {
		logx.Warnf("Teleportation failed: portal %s has no target.", portal)
		return false
	}
	destMapCode := target.TargetMap
	destMapState, ok := s.Maps[destMapCode]
	if !ok {
		logx.Warnf("Teleportation failed: destination map %q not found.", destMapCode)
		return false
	}

	// A negative target cell is a random portal: spawn anywhere on the target
	// map. Otherwise spawn near the target cell, inside the spawn radius.
	var spawnX, spawnY float64
	if target.TargetCell.X < 0 || target.TargetCell.Y < 0 {
		spawnX = rand.Float64() * float64(destMapState.GridW())
		spawnY = rand.Float64() * float64(destMapState.GridH())
	} else {
		spawnX = float64(target.TargetCell.X) + rand.Float64()*s.portalSpawnRadius*2 - s.portalSpawnRadius
		spawnY = float64(target.TargetCell.Y) + rand.Float64()*s.portalSpawnRadius*2 - s.portalSpawnRadius
	}

	currentMapState := s.Maps[player.MapCode]
	size := currentMapState.Tf(player.ID).Size
	destPosI, err := destMapState.Pathfinder().FindClosestWalkablePoint(world.Cell{X: int(spawnX), Y: int(spawnY)}, size)
	if err != nil {
		logx.Warnf("Could not find a walkable spawn point: %v", err)
		return false
	}
	row := currentMapState.Take(player.ID)
	row.Transform.Pos = world.Vec2{X: float64(destPosI.X), Y: float64(destPosI.Y)}
	player.MapCode = destMapCode
	player.TargetPos = world.Cell{}
	player.Path = []world.Cell{}
	row.Motion.Mode = world.TELEPORTING
	player.OnPortal = false
	destMapState.Put(player.ID, row)
	s.Teleported = append(s.Teleported, player)
	return true
}
