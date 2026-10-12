package game

import (
	"math"
	"math/rand"
	"time"
)

// executeProjectileSkill is the "projectile" skill handler.
// It spawns a fast-moving "skill" projectile bot towards the caster's target.
// Works for players and bots alike: the context names the caster by id.
func (s *GameServer) executeProjectileSkill(ctx SkillContext) {
	casterStats := s.CalculateStats(ctx.CasterID, ctx.MapState)

	projectileItemIDs := []string{ctx.SummonedEntityItemID}
	if ctx.SummonedEntityItemID == "" {
		if d, ok := s.entityDefaults["skill"]; ok && len(d.LiveItemIDs) > 0 {
			projectileItemIDs = d.LiveItemIDs
		}
	}

	if rand.Float64() >= s.summonChance(s.projectileSpawnChance, casterStats) {
		return
	}

	casterID := ctx.CasterID
	casterPos := ctx.MapState.tf(casterID).Pos
	casterDir := ctx.MapState.mo(casterID).Direction

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

	projectileLifetime := summonLifetime(s.projectileLifetimeMs, casterStats)
	projectileDims := Vec2{X: s.projectileWidth, Y: s.projectileHeight}
	projectileBaseLife := s.entityBaseMaxLife

	// Pre-compensate for the first tick's movement so the projectile appears
	// at the caster's position on the first rendered frame.
	// dt-based: one-tick pre-displacement so the first replicated frame shows
	// the projectile at the caster's position rather than one tick downstream.
	projectileStep := s.entityBaseSpeed * s.projectileSpeedMultiplier * s.tickDuration.Seconds()
	dirX, dirY := getDirectionVector(projectileDirection)
	spawnX := casterPos.X - (dirX * projectileStep)
	spawnY := casterPos.Y - (dirY * projectileStep)

	var projectileOLs []ObjectLayerState
	for _, itemID := range projectileItemIDs {
		projectileOLs = append(projectileOLs, ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
	}

	projectileBot := &BotState{
		EntityBase: EntityBase{
			ID: s.newEntityID(),
		},
		MapCode:   ctx.MapState.code,
		ExpiresAt: time.Now().Add(projectileLifetime),
		// One-shot palette lookup at spawn; sim never re-reads.
	}

	ctx.MapState.bots[projectileBot.ID] = projectileBot
	ctx.MapState.spawn(projectileBot.ID, EntityData{Kind: EntityBot}, Transform{Pos: Vec2{X: spawnX, Y: spawnY}, Size: projectileDims}, Appearance{Layers: projectileOLs})
	ctx.MapState.addVitals(projectileBot.ID, Vitals{MaxLife: projectileBaseLife, Life: projectileBaseLife})
	ctx.MapState.addMotion(projectileBot.ID, Motion{Direction: projectileDirection})
	ctx.MapState.addAgent(projectileBot.ID, Agent{Behavior: BehaviorSkill, CasterID: casterID})
	ctx.MapState.addMortal(projectileBot.ID, &Mortal{})
	s.ApplyResistanceStat(projectileBot.ID, ctx.MapState)
	ctx.MapState.vt(projectileBot.ID).Life = ctx.MapState.vt(projectileBot.ID).MaxLife
}
