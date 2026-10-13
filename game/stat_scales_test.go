package game

import (
	"testing"
	"time"

	"cyberia-server/game/content"
)

// Every stat point is worth what the shared contract says, and no chance the
// stats raise passes the configured cap.
func TestStatScalesApply(t *testing.T) {
	s := &Sim{Content: &content.Content{
		EntityBaseSpeed:             5,
		EntityBaseActionCooldown:    500 * time.Millisecond,
		EntityBaseMinActionCooldown: 100 * time.Millisecond,
		LifeRegenChance:             0.15,
		MaxChance:                   0.95,
	}, projectileSpawnChance: 0.75}
	if got := s.CalculateMovementSpeed(content.ComputedStats{Agility: 10}); got != 5.5 {
		t.Fatalf("agility 10 speed = %v, want 5.5", got)
	}
	if got := s.CalculateActionCooldown(content.ComputedStats{Utility: 10}); got != 450*time.Millisecond {
		t.Fatalf("utility 10 cooldown = %v, want 450ms", got)
	}
	if got := content.SummonLifetime(2000, content.ComputedStats{Range: 10}); got != 2500*time.Millisecond {
		t.Fatalf("range 10 lifetime = %v, want 2.5s", got)
	}
	if got := s.SummonChance(s.projectileSpawnChance, content.ComputedStats{Intelligence: 3}); got != 0.9 {
		t.Fatalf("intelligence 3 summon chance = %v, want 0.9", got)
	}
	if got := s.SummonChance(s.projectileSpawnChance, content.ComputedStats{Intelligence: 10}); got != 0.95 {
		t.Fatalf("summon chance passed the cap: %v", got)
	}
	if got := s.Chance(s.LifeRegenChance, 10*content.StatScales.Utility); got != 0.25 {
		t.Fatalf("utility 10 regen chance = %v, want 0.25", got)
	}
	s.MaxChance = 10000
	if got := s.Chance(0.5, 1); got != 1 {
		t.Fatalf("a cap outside (0, 1] must read as 1, got %v", got)
	}
}

// Regeneration heals the entity's own base amount plus a tenth of a point per
// resistance point, never past its maximum, and only when the roll passes.
func TestRegenAmountFollowsResistance(t *testing.T) {
	s, ms, p := movementFixture(t)
	s.LifeRegenChance, s.MaxChance = 1, 1
	ms.Mt(p.ID).LifeRegen = 1
	ms.Vt(p.ID).Life, ms.Vt(p.ID).MaxLife = 50, 100
	s.SetProgression(p.ID, content.NewEntityProgression(1, false, s.ProgressionConfig()))
	base := s.ProgressionConfig().BaseAtLevel(1).Resistance
	s.handleProbabilisticRegen(p.ID, ms)
	if want := 50 + 1 + base*content.StatRegenPerResistance; ms.Vt(p.ID).Life != want {
		t.Fatalf("life after regen = %v, want %v", ms.Vt(p.ID).Life, want)
	}
	ms.Vt(p.ID).Life = 99.9
	s.handleProbabilisticRegen(p.ID, ms)
	if ms.Vt(p.ID).Life != 100 {
		t.Fatalf("regen passed the maximum: %v", ms.Vt(p.ID).Life)
	}
	s.LifeRegenChance = 0
	ms.Vt(p.ID).Life = 50
	s.handleProbabilisticRegen(p.ID, ms)
	if ms.Vt(p.ID).Life != 50 {
		t.Fatal("regen fired at zero chance")
	}
}
