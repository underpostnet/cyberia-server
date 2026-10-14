package sim

import (
	"math"
	"testing"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// movementFixture builds the smallest world phaseInput needs: one walkable grid
// and a player holding a client.
func movementFixture(t *testing.T) (*Sim, *world.MapState, *world.Session) {
	t.Helper()

	mapState := world.NewMapState("test", 20, 20)

	s := &Sim{Content: &content.Content{
		World: &world.World{
			Maps:     map[string]*world.MapState{"test": mapState},
			Sessions: map[world.EntityID]*world.Session{},
		},
		EntityBaseSpeed:             5,
		EntityBaseActionCooldown:    500 * time.Millisecond,
		EntityBaseMinActionCooldown: 100 * time.Millisecond,
	}}

	player := &world.Session{}
	player.ID = 1
	player.MapCode = "test"
	player.Client = nopConn{}
	s.Sessions[player.ID] = player
	mapState.Spawn(player.ID, world.EntityData{Kind: world.EntityPlayer}, world.Transform{Pos: world.Vec2{X: 1, Y: 1}, Size: world.Vec2{X: 1, Y: 1}}, world.Appearance{})
	mapState.AddVitals(player.ID, world.Vitals{Life: 100, MaxLife: 100})
	mapState.AddMotion(player.ID, world.Motion{Direction: world.NONE, Mode: world.IDLE})
	mapState.AddMortal(player.ID, &world.Mortal{})
	mapState.AddPlayer(player.ID)

	return s, mapState, player
}

// nopConn is a client link that drops every message.
type nopConn struct{}

func (nopConn) Send(string, any) {}

// tapTick queues taps and runs one full input phase over them, which is the only
// path that re-plans movement.
func tapTick(s *Sim, mapState *world.MapState, player *world.Session, taps ...world.InputCommand) {
	world.EnqueueInputs(player, taps)
	s.PhaseInput(0, mapState)
}

func tap(seq uint32, x, y float64) world.InputCommand {
	return world.InputCommand{Kind: world.InputKindPlayerAction, Sequence: seq, TargetX: x, TargetY: y}
}

// A tap re-plans on the tick it lands.
func TestTapReplansOnTheSameTick(t *testing.T) {
	s, mapState, player := movementFixture(t)

	tapTick(s, mapState, player, tap(7, 10, 10))

	if mapState.Mo(player.ID).Mode != world.WALKING {
		t.Fatalf("mode = %v, want WALKING", mapState.Mo(player.ID).Mode)
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

	world.EnqueueInputs(player, []world.InputCommand{tap(1, 18, 1), tap(2, 1, 18)})

	if player.InputConsumedThrough != 0 {
		t.Fatalf("cursor = %d, want 0 — enqueue is not consumption",
			player.InputConsumedThrough)
	}

	s.PhaseInput(0, mapState)

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

	sentinel := []world.Cell{{X: 42, Y: 42}}
	player.Path = sentinel

	tapTick(s, mapState, player, tap(2, 10, 10))

	if len(player.Path) != 1 || player.Path[0] != sentinel[0] {
		t.Fatalf("path = %v, want the sentinel untouched — an unchanged target must not re-path",
			player.Path)
	}
}

// A step that reaches a node walks the rest of its length on the next leg.
func TestStepCarriesPastANode(t *testing.T) {
	path := []world.Cell{{X: 1, Y: 0}, {X: 1, Y: 1}}
	pos, path, dir, facing := stepAlongPath(world.Vec2{X: 0.9, Y: 0}, path, 3, 0.1)

	if len(path) != 1 || math.Abs(pos.X-1) > 1e-9 || math.Abs(pos.Y-0.2) > 1e-9 {
		t.Fatalf("pos = %v, path = %v, want (1, 0.2) on the last leg", pos, path)
	}
	if !facing || dir != world.DOWN {
		t.Fatalf("facing = %v %v, want DOWN — the facing of the leg the step ends on", dir, facing)
	}
}

// A re-plan drops the start cell when the player has passed it, so a turn never
// walks back.
func TestReplanDropsThePassedStartCell(t *testing.T) {
	s, mapState, player := movementFixture(t)
	mapState.Tf(player.ID).Pos = world.Vec2{X: 1.4, Y: 1}

	tapTick(s, mapState, player, tap(1, 5, 1))

	if len(player.Path) == 0 || player.Path[0] != (world.Cell{X: 2, Y: 1}) {
		t.Fatalf("path = %v, want it to start at (2,1)", player.Path)
	}
}

// A frozen player is refused movement outright, and the refusal must not leave a
// pending intent behind to fire on a later tick.
func TestFrozenPlayerNeverPlansMovement(t *testing.T) {
	s, mapState, player := movementFixture(t)
	player.Frozen = true

	tapTick(s, mapState, player, tap(1, 10, 10))

	if mapState.Mo(player.ID).Mode == world.WALKING {
		t.Fatal("mode = WALKING — a frozen player does not move")
	}
	if player.HasPendingMove {
		t.Fatal("pending move survived the flush — it would fire once the freeze lifts")
	}
}

// The snapshot carries the player's effective action cooldown. It no longer
// gates movement, but keyboard steering paces its refresh by it.
func TestSnapshotActionCooldownAppliesUtility(t *testing.T) {
	s := &content.Content{
		EntityBaseActionCooldown:    500 * time.Millisecond,
		EntityBaseMinActionCooldown: 100 * time.Millisecond,
	}

	gate := s.CalculateActionCooldown(content.ComputedStats{Utility: 20})
	if wire := int(gate / time.Millisecond); wire != 400 {
		t.Fatalf("actionCooldownMs = %d, want 400 (500 ms less 20%% Utility)", wire)
	}
}
