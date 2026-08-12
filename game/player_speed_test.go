package game

import "testing"

// Players walk on playerBaseSpeed; every other entity keeps entityBaseSpeed.
func TestPlayerMovementSpeedIsSeparateFromEntitySpeed(t *testing.T) {
	s := &GameServer{entityBaseSpeed: 5, playerBaseSpeed: 9}
	none := ComputedStats{}

	if got := s.CalculatePlayerMovementSpeed(none); got != 9 {
		t.Fatalf("player speed = %v, want the player base 9", got)
	}
	if got := s.CalculateMovementSpeed(none); got != 5 {
		t.Fatalf("entity speed = %v, want the entity base 5 — bots must not follow the player", got)
	}
}

// An instance that never sets playerBaseSpeed keeps its current pace.
func TestPlayerMovementSpeedFallsBackToEntitySpeed(t *testing.T) {
	s := &GameServer{entityBaseSpeed: 5}
	for _, unset := range []float64{0, -1} {
		s.playerBaseSpeed = unset
		if got := s.CalculatePlayerMovementSpeed(ComputedStats{}); got != 5 {
			t.Fatalf("playerBaseSpeed=%v must fall back to 5, got %v", unset, got)
		}
	}
}

// Agility scales the player base, not the entity base.
func TestPlayerMovementSpeedAppliesAgilityToThePlayerBase(t *testing.T) {
	s := &GameServer{entityBaseSpeed: 5, playerBaseSpeed: 10}
	got := s.CalculatePlayerMovementSpeed(ComputedStats{Agility: 50})
	if got != 15 {
		t.Fatalf("10 base at +50%% agility = 15, got %v", got)
	}
}

// The movement phase and the snapshot self block must agree: the client
// predicts with the speed the snapshot carries, so a mismatch would show up as
// a correction on every frame.
func TestPlayerSpeedIsConsistentBetweenMovementAndSnapshot(t *testing.T) {
	s := &GameServer{entityBaseSpeed: 5, playerBaseSpeed: 9}
	stats := ComputedStats{Agility: 20}
	movement := s.CalculatePlayerMovementSpeed(stats)
	// buildSnapshotSelf writes MoveSpeed from the same call.
	if snapshot := s.CalculatePlayerMovementSpeed(stats); snapshot != movement {
		t.Fatalf("snapshot %v != movement %v", snapshot, movement)
	}
}
