package game

import (
	"testing"
	"time"
)

// Every stat point is worth what the shared contract says, and no chance the
// stats raise passes the configured cap.
func TestStatScalesApply(t *testing.T) {
	s := &GameServer{
		entityBaseSpeed:             5,
		entityBaseActionCooldown:    500 * time.Millisecond,
		entityBaseMinActionCooldown: 100 * time.Millisecond,
		lifeRegenChance:             0.15,
		maxChance:                   0.95,
		projectileSpawnChance:       0.75,
	}
	if got := s.CalculateMovementSpeed(ComputedStats{Agility: 10}); got != 5.5 {
		t.Fatalf("agility 10 speed = %v, want 5.5", got)
	}
	if got := s.CalculateActionCooldown(ComputedStats{Utility: 10}); got != 450*time.Millisecond {
		t.Fatalf("utility 10 cooldown = %v, want 450ms", got)
	}
	if got := summonLifetime(2000, ComputedStats{Range: 10}); got != 2500*time.Millisecond {
		t.Fatalf("range 10 lifetime = %v, want 2.5s", got)
	}
	if got := s.summonChance(s.projectileSpawnChance, ComputedStats{Intelligence: 3}); got != 0.9 {
		t.Fatalf("intelligence 3 summon chance = %v, want 0.9", got)
	}
	if got := s.summonChance(s.projectileSpawnChance, ComputedStats{Intelligence: 10}); got != 0.95 {
		t.Fatalf("summon chance passed the cap: %v", got)
	}
	if got := s.chance(s.lifeRegenChance, 10*statScales.Utility); got != 0.25 {
		t.Fatalf("utility 10 regen chance = %v, want 0.25", got)
	}
	s.maxChance = 10000
	if got := s.chance(0.5, 1); got != 1 {
		t.Fatalf("a cap outside (0, 1] must read as 1, got %v", got)
	}
}

// Regeneration heals the entity's own base amount plus a tenth of a point per
// resistance point, never past its maximum, and only when the roll passes.
func TestRegenAmountFollowsResistance(t *testing.T) {
	s, ms, p := movementFixture(t)
	s.lifeRegenChance, s.maxChance = 1, 1
	p.LifeRegen = 1
	p.Life, p.MaxLife = 50, 100
	p.Progression = NewEntityProgression(1, false, s.progressionConfig())
	base := s.progressionConfig().BaseAtLevel(1).Resistance
	s.handleProbabilisticRegen(p, ms)
	if want := 50 + 1 + base*statRegenPerResistance; p.Life != want {
		t.Fatalf("life after regen = %v, want %v", p.Life, want)
	}
	p.Life = 99.9
	s.handleProbabilisticRegen(p, ms)
	if p.Life != 100 {
		t.Fatalf("regen passed the maximum: %v", p.Life)
	}
	s.lifeRegenChance = 0
	p.Life = 50
	s.handleProbabilisticRegen(p, ms)
	if p.Life != 50 {
		t.Fatal("regen fired at zero chance")
	}
}
