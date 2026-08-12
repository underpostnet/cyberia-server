package game

import (
	"testing"
	"time"
)

// movementFixture builds the smallest world phaseInput needs: one walkable grid
// and a player holding a client.
func movementFixture(t *testing.T) (*GameServer, *MapState, *PlayerState) {
	t.Helper()

	mapState := &MapState{
		gridW:      20,
		gridH:      20,
		pathfinder: NewPathfinder(20, 20),
		players:    map[string]*PlayerState{},
		bots:       map[string]*BotState{},
	}

	s := &GameServer{
		maps:                        map[string]*MapState{"test": mapState},
		statsCache:                  map[string]statsCacheEntry{},
		entityBaseSpeed:             5,
		entityBaseActionCooldown:    500 * time.Millisecond,
		entityBaseMinActionCooldown: 100 * time.Millisecond,
	}

	player := &PlayerState{}
	player.ID = "p1"
	player.Pos = Point{X: 1, Y: 1}
	player.Dims = Dimensions{Width: 1, Height: 1}
	player.MapCode = "test"
	player.Life, player.MaxLife = 100, 100
	player.Client = &Client{playerID: player.ID}
	mapState.players[player.ID] = player

	return s, mapState, player
}

// tapTick queues taps and runs one full input phase over them, which is the only
// path that re-plans movement.
func tapTick(s *GameServer, mapState *MapState, player *PlayerState, taps ...InputCommand) {
	for _, cmd := range taps {
		EnqueueInput(player, cmd)
	}
	s.phaseInput(0, mapState)
}

func tap(seq uint32, x, y float64) InputCommand {
	return InputCommand{Kind: InputKindPlayerAction, Sequence: seq, TargetX: x, TargetY: y}
}

// A tap re-plans on the tick it lands, and publishes the sequence the client
// waits on before it adopts the authoritative route.
func TestTapReplansOnTheSameTick(t *testing.T) {
	s, mapState, player := movementFixture(t)

	tapTick(s, mapState, player, tap(7, 10, 10))

	if player.LastMovementSequence != 7 {
		t.Fatalf("moveAck = %d, want 7 — a planned tap must publish its sequence",
			player.LastMovementSequence)
	}
	if player.Mode != WALKING {
		t.Fatalf("mode = %v, want WALKING", player.Mode)
	}
	if player.TargetPos.X != 10 || player.TargetPos.Y != 10 {
		t.Fatalf("target = %v, want (10,10)", player.TargetPos)
	}
}

// The behaviour this whole design exists for: a change of direction takes effect
// on the very next tick. No action cooldown stands between the tap and the walk,
// so flicking left and right turns the player left and right.
func TestConsecutiveTicksTurnImmediately(t *testing.T) {
	s, mapState, player := movementFixture(t)

	tapTick(s, mapState, player, tap(1, 18, 1))
	if player.TargetPos.X != 18 {
		t.Fatalf("first target = %v, want x=18", player.TargetPos)
	}

	// No sleep: the action cooldown must not gate this.
	tapTick(s, mapState, player, tap(2, 1, 18))

	if player.TargetPos.X != 1 || player.TargetPos.Y != 18 {
		t.Fatalf("second target = %v, want (1,18) — the turn must not wait for a cooldown",
			player.TargetPos)
	}
	if player.LastMovementSequence != 2 {
		t.Fatalf("moveAck = %d, want 2", player.LastMovementSequence)
	}
}

// Taps that land in one tick describe one instant, so only the newest is
// planned. This is what bounds the pathfinder now that the cooldown is gone,
// and it costs the player nothing: the dropped taps were already superseded.
func TestTapsInOneTickCoalesceToTheNewest(t *testing.T) {
	s, mapState, player := movementFixture(t)

	tapTick(s, mapState, player,
		tap(1, 18, 1),
		tap(2, 1, 18),
		tap(3, 18, 18))

	if player.TargetPos.X != 18 || player.TargetPos.Y != 18 {
		t.Fatalf("target = %v, want the newest tap (18,18)", player.TargetPos)
	}
	if player.LastMovementSequence != 3 {
		t.Fatalf("moveAck = %d, want 3 — only the newest tap is planned",
			player.LastMovementSequence)
	}
}

// A superseded tap is acked on arrival but never planned. The client must be
// able to tell those apart, or it would adopt a route belonging to a target it
// has already abandoned.
func TestAckCoversSupersededTapsButMoveAckDoesNot(t *testing.T) {
	s, mapState, player := movementFixture(t)

	EnqueueInput(player, tap(1, 18, 1))
	EnqueueInput(player, tap(2, 1, 18))

	if player.LastAckedInputSequence != 2 {
		t.Fatalf("ack = %d, want 2 — arrival acks every command",
			player.LastAckedInputSequence)
	}
	if player.LastMovementSequence != 0 {
		t.Fatalf("moveAck = %d, want 0 — nothing is planned before the phase runs",
			player.LastMovementSequence)
	}

	s.phaseInput(0, mapState)

	if player.LastMovementSequence != 2 {
		t.Fatalf("moveAck = %d, want 2", player.LastMovementSequence)
	}
}

// moveAck never runs backward: out-of-order sequences would otherwise unlock a
// stale route on the client.
func TestMoveAckIsMonotonic(t *testing.T) {
	s, mapState, player := movementFixture(t)

	tapTick(s, mapState, player, tap(9, 10, 10))
	tapTick(s, mapState, player, tap(4, 1, 18))

	if player.LastMovementSequence != 9 {
		t.Fatalf("moveAck = %d, want 9 — a lower sequence must not lower it",
			player.LastMovementSequence)
	}
}

// Re-tapping the destination already being walked runs no search. That is the
// shape of a held steering key and of a spam-tapping client, so skipping it
// removes the cheapest way to charge the tick budget.
func TestRepeatOfTheCurrentTargetSkipsThePathfinder(t *testing.T) {
	s, mapState, player := movementFixture(t)

	tapTick(s, mapState, player, tap(1, 10, 10))

	sentinel := []PointI{{X: 42, Y: 42}}
	player.Path = sentinel

	tapTick(s, mapState, player, tap(2, 10, 10))

	if len(player.Path) != 1 || player.Path[0] != sentinel[0] {
		t.Fatalf("path = %v, want the sentinel untouched — an unchanged target must not re-path",
			player.Path)
	}
}

// A frozen player is refused movement outright, and the refusal must not leave a
// pending intent behind to fire on a later tick.
func TestFrozenPlayerNeverPlansMovement(t *testing.T) {
	s, mapState, player := movementFixture(t)
	player.Frozen = true

	tapTick(s, mapState, player, tap(1, 10, 10))

	if player.LastMovementSequence != 0 {
		t.Fatalf("moveAck = %d, want 0 — a frozen player does not move",
			player.LastMovementSequence)
	}
	if player.HasPendingMove {
		t.Fatal("pending move survived the flush — it would fire once the freeze lifts")
	}
}

// The snapshot carries the player's effective action cooldown. It no longer
// gates movement, but keyboard steering paces its refresh by it.
func TestSnapshotActionCooldownAppliesUtility(t *testing.T) {
	s := &GameServer{
		entityBaseActionCooldown:    500 * time.Millisecond,
		entityBaseMinActionCooldown: 100 * time.Millisecond,
	}

	gate := s.CalculateActionCooldown(ComputedStats{Utility: 20})
	if wire := int(gate / time.Millisecond); wire != 400 {
		t.Fatalf("actionCooldownMs = %d, want 400 (500 ms less 20%% Utility)", wire)
	}
}
