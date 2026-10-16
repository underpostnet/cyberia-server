package sim

import (
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/social"
	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
)

// Sim is the simulation step and its config.
type Sim struct {
	*content.Content
	// Social is the chat, the parties and the friends of the players on this
	// server.
	Social *social.Social
	// Friendships runs friend actions against the engine; nil in anonymous
	// mode.
	Friendships    FriendBroker
	InstanceCode   string            // INSTANCE_CODE — selects which instance to load
	PlayerSpawn    PlayerSpawnConfig // authoritative initial spawn for new players
	PortalHoldTime time.Duration
	// Teleported holds the players a portal moved this map step. gameLoop
	// sends their snapshots after phasePortals and clears it.
	Teleported []*world.Session

	// bot related defaults
	botAggroRange float64

	// Skill config
	projectileSpawnChance           float64
	projectileLifetimeMs            int
	projectileWidth                 float64
	projectileHeight                float64
	projectileSpeedMultiplier       float64
	doppelgangerSpawnChance         float64
	doppelgangerLifetimeMs          int
	doppelgangerSpawnRadius         float64
	doppelgangerInitialLifeFraction float64

	// Portal defaults
	portalSpawnRadius float64
}

// ApplyConfig applies the simulation part of the instance config, then the
// content part. Caller holds s.mu.
func (s *Sim) ApplyConfig(cfg *pb.InstanceConfig, rules content.ProgressionRules) {
	s.Content.ApplyConfig(cfg, rules)
	s.PortalHoldTime = time.Duration(cfg.GetPortalHoldTimeMs()) * time.Millisecond
	s.portalSpawnRadius = cfg.GetPortalSpawnRadius()

	// Bot defaults
	s.botAggroRange = cfg.GetBotAggroRange()

	// Skill params (nested under SkillRules in proto)
	sr := cfg.GetSkillRules()
	if sr == nil {
		sr = &pb.SkillRules{}
	}
	s.projectileSpawnChance = sr.GetProjectileSpawnChance()
	s.projectileLifetimeMs = int(sr.GetProjectileLifetimeMs())
	s.projectileWidth = sr.GetProjectileWidth()
	s.projectileHeight = sr.GetProjectileHeight()
	s.projectileSpeedMultiplier = sr.GetProjectileSpeedMultiplier()
	s.doppelgangerSpawnChance = sr.GetDoppelgangerSpawnChance()
	s.doppelgangerLifetimeMs = int(sr.GetDoppelgangerLifetimeMs())
	s.doppelgangerSpawnRadius = sr.GetDoppelgangerSpawnRadius()
	s.doppelgangerInitialLifeFraction = sr.GetDoppelgangerInitialLifeFraction()

	// Register built-in skill handlers now that skillConfig is populated.
	s.InitSkills()
}
