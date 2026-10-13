package game

import (
	"math"
	"math/rand"
	"time"

	"cyberia-server/game/world"
)

// executeProjectileSkill is the "projectile" skill handler.
// It spawns a fast-moving "skill" projectile bot towards the caster's target.
// Works for players and bots alike: the context names the caster by id.
func (s *Sim) executeProjectileSkill(ctx SkillContext) {
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
	casterPos := ctx.MapState.Tf(casterID).Pos
	casterDir := ctx.MapState.Mo(casterID).Direction

	// Calculate direction from caster to target.
	dx := ctx.Target.X - casterPos.X
	dy := ctx.Target.Y - casterPos.Y
	dist := math.Sqrt(dx*dx + dy*dy)

	var projectileDirection world.Direction
	if dist > 0 {
		dirX, dirY := dx/dist, dy/dist
		angle := math.Atan2(dirY, dirX)
		if angle < 0 {
			angle += 2 * math.Pi
		}
		directionIndex := (int(math.Round(angle/(math.Pi/4))) + 2) % 8
		projectileDirection = world.Direction(directionIndex)
	} else {
		projectileDirection = casterDir
	}

	// Acquire the lock only if the caller doesn't already hold it.
	if !ctx.CallerHoldsLock {
		s.Mu.Lock()
		defer s.Mu.Unlock()
	}

	projectileLifetime := summonLifetime(s.projectileLifetimeMs, casterStats)
	projectileDims := world.Vec2{X: s.projectileWidth, Y: s.projectileHeight}
	projectileBaseLife := s.entityBaseMaxLife

	// Pre-compensate for the first tick's movement so the projectile appears
	// at the caster's position on the first rendered frame.
	// dt-based: one-tick pre-displacement so the first replicated frame shows
	// the projectile at the caster's position rather than one tick downstream.
	projectileStep := s.entityBaseSpeed * s.projectileSpeedMultiplier * tickDuration.Seconds()
	dirX, dirY := getDirectionVector(projectileDirection)
	spawnX := casterPos.X - (dirX * projectileStep)
	spawnY := casterPos.Y - (dirY * projectileStep)

	var projectileOLs []world.ObjectLayerState
	for _, itemID := range projectileItemIDs {
		projectileOLs = append(projectileOLs, world.ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
	}

	projectileBot := s.NewEntityID()
	brain := &world.Brain{
		ExpiresAt: time.Now().Add(projectileLifetime),
		// One-shot palette lookup at spawn; sim never re-reads.
	}

	ctx.MapState.AddBrain(projectileBot, brain)
	ctx.MapState.Spawn(projectileBot, world.EntityData{Kind: world.EntityBot}, world.Transform{Pos: world.Vec2{X: spawnX, Y: spawnY}, Size: projectileDims}, world.Appearance{Layers: projectileOLs})
	ctx.MapState.AddVitals(projectileBot, world.Vitals{MaxLife: projectileBaseLife, Life: projectileBaseLife})
	ctx.MapState.AddMotion(projectileBot, world.Motion{Direction: projectileDirection})
	ctx.MapState.AddAgent(projectileBot, world.Agent{Behavior: BehaviorSkill, CasterID: casterID})
	ctx.MapState.AddMortal(projectileBot, &world.Mortal{})
	s.ApplyResistanceStat(projectileBot, ctx.MapState)
	ctx.MapState.Vt(projectileBot).Life = ctx.MapState.Vt(projectileBot).MaxLife
}
