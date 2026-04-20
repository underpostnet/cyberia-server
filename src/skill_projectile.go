package game

import (
	"math"
	"math/rand"
	"time"

	"github.com/google/uuid"
)

// executeProjectileSkill is the "projectile" skill handler.
// It spawns a fast-moving "skill" projectile bot towards the caster's target.
// Works for both PlayerState and BotState via SkillContext.
func (s *GameServer) executeProjectileSkill(ctx SkillContext) {
	casterStats := s.CalculateStats(ctx.Caster, ctx.MapState)

	projectileItemIDs := []string{ctx.SummonedEntityItemID}
	if ctx.SummonedEntityItemID == "" {
		if d, ok := s.entityDefaults["skill"]; ok && len(d.LiveItemIDs) > 0 {
			projectileItemIDs = d.LiveItemIDs
		}
	}

	// Intelligence stat increases the chance of the skill activating.
	if rand.Float64() >= math.Min(s.projectileSpawnChance+(casterStats.Intelligence/100.0), s.maxChance) {
		return
	}

	// Extract caster properties via type switch.
	var casterID, mapCode string
	var casterPos Point
	var casterDir Direction
	switch e := ctx.Caster.(type) {
	case *PlayerState:
		casterID = e.ID
		mapCode = e.MapCode
		casterPos = e.Pos
		casterDir = e.Direction
	case *BotState:
		casterID = e.ID
		mapCode = e.MapCode
		casterPos = e.Pos
		casterDir = e.Direction
	}

	// Calculate direction from caster to target.
	dx := ctx.Target.X - casterPos.X
	dy := ctx.Target.Y - casterPos.Y
	dist := math.Sqrt(dx*dx + dy*dy)

	var projectileDirection Direction
	if dist > 0 {
		dirX, dirY := dx/dist, dy/dist
		angle := math.Atan2(dirY, dirX)
		if angle < 0 {
			angle += 2 * math.Pi
		}
		directionIndex := (int(math.Round(angle/(math.Pi/4))) + 2) % 8
		projectileDirection = Direction(directionIndex)
	} else {
		projectileDirection = casterDir
	}

	// Acquire the lock only if the caller doesn't already hold it.
	if !ctx.CallerHoldsLock {
		s.mu.Lock()
		defer s.mu.Unlock()
	}

	// Range stat increases the projectile's lifetime in milliseconds.
	projectileLifetime := (time.Duration(s.projectileLifetimeMs) * time.Millisecond) +
		(time.Duration(casterStats.Range) * time.Millisecond)
	projectileDims := Dimensions{Width: s.projectileWidth, Height: s.projectileHeight}
	projectileBaseLife := s.entityBaseMaxLife

	// Pre-compensate for the first tick's movement so the projectile appears
	// at the caster's position on the first rendered frame.
	projectileStep := (s.entityBaseSpeed * s.projectileSpeedMultiplier) / float64(s.fps)
	dirX, dirY := getDirectionVector(projectileDirection)
	spawnX := casterPos.X - (dirX * projectileStep)
	spawnY := casterPos.Y - (dirY * projectileStep)

	var projectileOLs []ObjectLayerState
	for _, itemID := range projectileItemIDs {
		projectileOLs = append(projectileOLs, ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
	}

	projectileBot := &BotState{
		ID:           uuid.New().String(),
		MapCode:      mapCode,
		Pos:          Point{X: spawnX, Y: spawnY},
		Dims:         projectileDims,
		Behavior:     "skill",
		Direction:    projectileDirection,
		ExpiresAt:    time.Now().Add(projectileLifetime),
		ObjectLayers: projectileOLs,
		CasterID:     casterID,
		MaxLife:      projectileBaseLife,
		Life:         projectileBaseLife,
		Color:        s.colors["SKILL"],
	}

	ctx.MapState.bots[projectileBot.ID] = projectileBot
	s.ApplyResistanceStat(projectileBot, ctx.MapState)
	projectileBot.Life = projectileBot.MaxLife
}
