package content

import (
	"testing"
	"time"
)

// The snapshot carries the player's effective action cooldown. It no longer
// gates movement, but keyboard steering paces its refresh by it.
func TestSnapshotActionCooldownAppliesUtility(t *testing.T) {
	s := &Content{
		EntityBaseActionCooldown:    500 * time.Millisecond,
		EntityBaseMinActionCooldown: 100 * time.Millisecond,
	}

	gate := s.CalculateActionCooldown(ComputedStats{Utility: 20})
	if wire := int(gate / time.Millisecond); wire != 400 {
		t.Fatalf("actionCooldownMs = %d, want 400 (500 ms less 20%% Utility)", wire)
	}
}
