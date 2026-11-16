package sim

import (
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// executeProjectileSkill is the "projectile" skill handler. It spawns a
// "skill" projectile bot that flies straight along the aim of the caster, at
// any angle. Its facing is the nearest of the eight, for its sprite.
func (s *Sim) executeProjectileSkill(ctx SkillContext) {
	casterStats := s.CalculateStats(ctx.CasterID, ctx.MapState)

	projectileItemIDs := []string{ctx.SummonedEntityItemID}
	if ctx.SummonedEntityItemID == "" {
		if d, ok := s.EntityDefaults["skill"]; ok && len(d.LiveItemIDs) > 0 {
			projectileItemIDs = d.LiveItemIDs
		}
	}

	casterID := ctx.CasterID
	casterPos := ctx.MapState.Tf(casterID).Pos
	speed := s.EntityBaseSpeed * s.projectileSpeedMultiplier
	velocity := world.Vec2{X: ctx.Aim.X * speed, Y: ctx.Aim.Y * speed}

	projectileLifetime := content.SummonLifetime(s.projectileLifetimeMs, casterStats)
	projectileDims := world.Vec2{X: s.projectileWidth, Y: s.projectileHeight}
	projectileBaseLife := s.EntityBaseMaxLife

	// One tick back along the flight, so the first replicated frame shows the
	// projectile at the caster's position.
	spawnX := casterPos.X - velocity.X*TickDuration.Seconds()
	spawnY := casterPos.Y - velocity.Y*TickDuration.Seconds()

	var projectileOLs []world.ObjectLayerState
	for _, itemID := range projectileItemIDs {
		projectileOLs = append(projectileOLs, world.ObjectLayerState{ItemID: itemID, Active: true, Quantity: 1})
	}

	projectileBot := s.NewEntityID()
	brain := &world.Brain{
		ExpiresAt: time.Now().Add(projectileLifetime),
		Velocity:  velocity,
	}

	ctx.MapState.AddBrain(projectileBot, brain)
	ctx.MapState.Spawn(projectileBot, world.EntityData{Kind: world.EntityBot}, world.Transform{Pos: world.Vec2{X: spawnX, Y: spawnY}, Size: projectileDims}, world.Appearance{Layers: projectileOLs})
	ctx.MapState.AddVitals(projectileBot, world.Vitals{MaxLife: projectileBaseLife, Life: projectileBaseLife})
	ctx.MapState.AddMotion(projectileBot, world.Motion{Direction: directionFromVector(ctx.Aim.X, ctx.Aim.Y)})
	ctx.MapState.AddAgent(projectileBot, s.summonAgent(ctx.MapState, content.BehaviorSkill, casterID))
	ctx.MapState.AddMortal(projectileBot, &world.Mortal{})
	s.ApplyResistanceStat(projectileBot, ctx.MapState)
	ctx.MapState.Vt(projectileBot).Life = ctx.MapState.Vt(projectileBot).MaxLife
}
