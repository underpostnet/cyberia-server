package game

import (
	"math"
	"math/rand"
	"time"

	"cyberia-server/logx"
)

// directionFromVector maps a movement vector onto one of the eight facings.
func directionFromVector(dirX, dirY float64) Direction {
	angle := math.Atan2(dirY, dirX)
	if angle < 0 {
		angle += 2 * math.Pi
	}
	return Direction((int(math.Round(angle/(math.Pi/4))) + 2) % 8)
}

// stepAlongPath advances pos toward the head of path by speed*dt cells.
//
// dt-based integration: speed is cells/second and dt is the authoritative
// simulation step (1 / tickRate). A frame count instead of dt silently breaks
// movement whenever the loop runs slow or the tickRate config changes.
//
// It returns the new position, the path left to walk, and the facing for this
// step. The bool is false when the step produced no heading — an empty path
// or a zero-length hop — so the caller decides what an arriving entity faces.
func stepAlongPath(pos Vec2, path []Cell, speed, dt float64) (Vec2, []Cell, Direction, bool) {
	if len(path) == 0 {
		return pos, path, NONE, false
	}

	targetNode := path[0]
	dx := float64(targetNode.X) - pos.X
	dy := float64(targetNode.Y) - pos.Y
	dist := math.Sqrt(dx*dx + dy*dy)
	step := speed * dt // cells per simulation tick

	if dist >= step {
		dirX, dirY := dx/dist, dy/dist
		pos.X += dirX * step
		pos.Y += dirY * step
		return pos, path, directionFromVector(dirX, dirY), true
	}

	// Node reached. Face the next leg, if there is one.
	pos = Vec2{X: float64(targetNode.X), Y: float64(targetNode.Y)}
	path = path[1:]
	if len(path) == 0 {
		return pos, path, NONE, false
	}

	next := path[0]
	dirX := float64(next.X) - pos.X
	dirY := float64(next.Y) - pos.Y
	norm := math.Sqrt(dirX*dirX + dirY*dirY)
	if norm == 0 {
		return pos, path, NONE, false
	}
	return pos, path, directionFromVector(dirX/norm, dirY/norm), true
}

func (s *GameServer) updatePlayerPosition(player *Session, mapState *MapState) {
	// Dead players can't move.
	if mapState.vt(player.ID).ghost() {
		return
	}

	playerStats := s.CalculateStats(player.ID, mapState)
	speed := s.CalculatePlayerMovementSpeed(playerStats)

	if mapState.mo(player.ID).Mode != WALKING {
		return
	}

	tf := mapState.tf(player.ID)
	pos, path, dir, facing := stepAlongPath(tf.Pos, player.Path, speed, s.tickDuration.Seconds())
	tf.Pos, player.Path = pos, path
	if facing {
		mapState.mo(player.ID).Direction = dir
	}
	if len(player.Path) == 0 {
		mapState.mo(player.ID).Mode = IDLE
		mapState.mo(player.ID).Direction = NONE
	}
}

// ---------- Portals ----------
func (s *GameServer) checkPortal(player *Session, mapState *MapState) {
	// Teleport is only possible while alive. A dead player standing on a portal
	// neither accrues hold progress nor teleports; entering ghost state cancels
	// any in-progress hold.
	if mapState.vt(player.ID).ghost() {
		player.OnPortal = false
		return
	}

	onPortal := false
	var activePortal EntityID
	playerRect := mapState.tf(player.ID).rect()
	for portal := range mapState.ofKind(EntityPortal) {
		if rectsOverlap(playerRect, mapState.tf(portal).rect()) {
			onPortal = true
			activePortal = portal
			break
		}
	}
	if onPortal {
		if !player.OnPortal {
			player.OnPortal = true
			player.TimeOnPortal = time.Now()
		} else if time.Since(player.TimeOnPortal) > s.portalHoldTime {
			s.teleportPlayer(player, mapState, activePortal)
		}
	} else {
		player.OnPortal = false
	}
}

func (s *GameServer) teleportPlayer(player *Session, mapState *MapState, portal EntityID) {
	// s.mu is held by the caller (phasePortals). Do not release it: map
	// mutations of the map slots must not race other readers.

	target, linked := mapState.pt(portal)
	if !linked {
		logx.Warnf("Teleportation failed: portal %s has no target.", portal)
		return
	}
	destMapCode := target.TargetMap
	destMapState, ok := s.maps[destMapCode]
	if !ok {
		logx.Warnf("Teleportation failed: destination map %q not found.", destMapCode)
		return
	}

	// A negative target cell is a random portal: spawn anywhere on the target
	// map. Otherwise spawn near the target cell, inside the spawn radius.
	var spawnX, spawnY float64
	if target.TargetCell.X < 0 || target.TargetCell.Y < 0 {
		spawnX = rand.Float64() * float64(destMapState.gridW)
		spawnY = rand.Float64() * float64(destMapState.gridH)
	} else {
		spawnX = float64(target.TargetCell.X) + rand.Float64()*s.portalSpawnRadius*2 - s.portalSpawnRadius
		spawnY = float64(target.TargetCell.Y) + rand.Float64()*s.portalSpawnRadius*2 - s.portalSpawnRadius
	}

	currentMapState := s.maps[player.MapCode]
	size := currentMapState.tf(player.ID).Size
	destPosI, err := destMapState.pathfinder.findClosestWalkablePoint(Cell{X: int(spawnX), Y: int(spawnY)}, size)
	if err != nil {
		logx.Warnf("Could not find a walkable spawn point: %v", err)
		return
	}
	row := currentMapState.take(player.ID)
	row.transform.Pos = Vec2{X: float64(destPosI.X), Y: float64(destPosI.Y)}
	player.MapCode = destMapCode
	player.TargetPos = Cell{}
	player.Path = []Cell{}
	row.motion.Mode = TELEPORTING
	player.OnPortal = false
	destMapState.put(player.ID, row)
	s.teleported = append(s.teleported, player)
}
