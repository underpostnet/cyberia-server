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
		players:    map[EntityID]*PlayerState{},
		bots:       map[EntityID]*BotState{},
	}

	s := &GameServer{
		maps:                        map[string]*MapState{"test": mapState},
		entityBaseSpeed:             5,
		entityBaseActionCooldown:    500 * time.Millisecond,
		entityBaseMinActionCooldown: 100 * time.Millisecond,
	}

	player := &PlayerState{}
	player.ID = 1
	player.MapCode = "test"
	player.Life, player.MaxLife = 100, 100
	player.Client = &Client{playerID: player.ID}
	mapState.players[player.ID] = player
	mapState.spawn(player.ID, Transform{Pos: Vec2{X: 1, Y: 1}, Size: Vec2{X: 1, Y: 1}}, Appearance{})

	return s, mapState, player
}

// placePlayer puts a player that holds layers on the map "test" of s.
func placePlayer(s *GameServer, id EntityID, layers []ObjectLayerState) *PlayerState {
	if s.maps == nil {
		s.maps = map[string]*MapState{}
	}
	ms := s.maps["test"]
	if ms == nil {
		ms = &MapState{players: map[EntityID]*PlayerState{}}
		s.maps["test"] = ms
	}
	p := &PlayerState{EntityBase: EntityBase{ID: id}, MapCode: "test"}
	ms.players[id] = p
	ms.spawn(id, Transform{}, Appearance{Layers: layers})
	return p
}

// tapTick queues taps and runs one full input phase over them, which is the only
// path that re-plans movement.
func tapTick(s *GameServer, mapState *MapState, player *PlayerState, taps ...InputCommand) {
	EnqueueInputs(player, taps)
	s.phaseInput(0, mapState)
}

func tap(seq uint32, x, y float64) InputCommand {
	return InputCommand{Kind: InputKindPlayerAction, Sequence: seq, TargetX: x, TargetY: y}
}

// A tap re-plans on the tick it lands.
func TestTapReplansOnTheSameTick(t *testing.T) {
	s, mapState, player := movementFixture(t)

	tapTick(s, mapState, player, tap(7, 10, 10))

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
}

// The cursor moves when a tick applies a command, not when it is queued. A
// snapshot built between enqueue and the next tick must not claim it.
func TestEnqueueIsNotConsumption(t *testing.T) {
	s, mapState, player := movementFixture(t)

	EnqueueInputs(player, []InputCommand{tap(1, 18, 1), tap(2, 1, 18)})

	if player.InputConsumedThrough != 0 {
		t.Fatalf("cursor = %d, want 0 — enqueue is not consumption",
			player.InputConsumedThrough)
	}

	s.phaseInput(0, mapState)

	if player.InputConsumedThrough != 2 {
		t.Fatalf("cursor = %d, want 2 — the tick consumes every command",
			player.InputConsumedThrough)
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

	if player.Mode == WALKING {
		t.Fatal("mode = WALKING — a frozen player does not move")
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

// The snapshot carries the time left on the shared action cooldown. A tap
// starts it; it reads 0 once the next tap fires skills again.
func TestSnapshotActionReadyInFollowsTheSkillGate(t *testing.T) {
	s, mapState, player := movementFixture(t)

	if ready := s.buildSnapshotSelf(player, mapState).ActionReadyInMs; ready != 0 {
		t.Fatalf("actionReadyInMs = %d before any tap, want 0", ready)
	}
	s.HandlePlayerTapAction(player, mapState, Vec2{X: 2, Y: 2})
	if ready := s.buildSnapshotSelf(player, mapState).ActionReadyInMs; ready <= 0 || ready > 500 {
		t.Fatalf("actionReadyInMs = %d after a tap, want 1..500", ready)
	}
	player.NextSkillAt = time.Now().Add(-time.Millisecond)
	if ready := s.buildSnapshotSelf(player, mapState).ActionReadyInMs; ready != 0 {
		t.Fatalf("actionReadyInMs = %d after the cooldown, want 0", ready)
	}
}
