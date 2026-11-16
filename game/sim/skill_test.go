package sim

import (
	"math"
	"testing"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// ackConn records the messages a player receives.
type ackConn struct {
	types    []string
	payloads []any
}

func (c *ackConn) Send(msgType string, payload any) {
	c.types = append(c.types, msgType)
	c.payloads = append(c.payloads, payload)
}

// skillFixture is the movement fixture with a gun that shoots, an emitter that
// summons a double, a gel that heals by itself and a coin whose drop is an
// event. The player holds the gun and the emitter active.
func skillFixture(t *testing.T) (*Sim, *world.MapState, *world.Session) {
	t.Helper()
	s, ms, player := movementFixture(t)
	s.NewEntityID() // summons take ids after the player
	s.EntityBaseMaxLife = 100
	s.projectileSpawnChance, s.doppelgangerSpawnChance = 1, 1
	s.projectileLifetimeMs, s.projectileWidth, s.projectileHeight, s.projectileSpeedMultiplier = 1000, 1, 1, 3
	s.SkillConfig = map[string][]content.SkillDefinition{
		"gun":     {{SkillID: "shot", LogicEventID: "projectile", SummonedEntityItemID: "bullet"}},
		"emitter": {{SkillID: "echo", LogicEventID: "doppelganger", SummonedEntityItemID: "$active_skin"}},
		"gel":     {{SkillID: "dose", LogicEventID: "zero-waste-auto-consume-heal"}},
		"coin":    {{SkillID: "scatter", LogicEventID: "coin_drop_or_transaction", SummonedEntityItemID: "coin"}},
	}
	ms.Ap(player.ID).Layers = []world.ObjectLayerState{
		{ItemID: "gun", Active: true, Quantity: 1},
		{ItemID: "emitter", Active: true, Quantity: 1},
		{ItemID: "gel", Active: true, Quantity: 1},
		{ItemID: "coin", Active: true, Quantity: 1},
	}
	return s, ms, player
}

// rolls makes every chance roll draw value and counts the rolls.
func rolls(t *testing.T, value float64) *int {
	t.Helper()
	count := 0
	skillRoll = func() float64 {
		count++
		return value
	}
	t.Cleanup(func() { skillRoll = defaultSkillRoll })
	return &count
}

var defaultSkillRoll = skillRoll

// summons counts the bots each caster made, by behavior.
func summons(ms *world.MapState, caster world.EntityID) map[string]int {
	out := map[string]int{}
	for id := range ms.OfKind(world.EntityBot) {
		if ms.Ag(id).CasterID == caster {
			out[ms.Ag(id).Behavior]++
		}
	}
	return out
}

func TestEveryManualAndAutomaticLogicHasAHandler(t *testing.T) {
	for id, logic := range content.SkillLogics {
		handler, handled := skillHandlers[id]
		switch logic.Activation {
		case content.SkillManual:
			if !handled || handler.chance == nil {
				t.Errorf("manual logic %q needs a handler with a chance", id)
			}
		case content.SkillAutomatic:
			if !handled || handler.chance != nil {
				t.Errorf("automatic logic %q needs a handler without a chance", id)
			}
		case content.SkillEvent:
			if handled {
				t.Errorf("event logic %q runs in its gameplay event, not as a skill", id)
			}
		default:
			t.Errorf("logic %q has no activation", id)
		}
	}
	for id := range skillHandlers {
		if _, known := content.SkillLogics[id]; !known {
			t.Errorf("handler %q runs no logic of the contract", id)
		}
	}
	if !SupportsSkill("coin_drop_or_transaction") || SupportsSkill("teleport") {
		t.Fatal("the server supports each logic of the contract and nothing else")
	}
}

// Aims of the tests.
var (
	facing = SkillAim{Mode: world.AimFacing}
	auto   = SkillAim{Mode: world.AimAuto}
)

func aimAt(x, y float64) SkillAim {
	return SkillAim{Mode: world.AimDirection, Direction: world.Vec2{X: x, Y: y}}
}

// facingVector is the unit vector of a facing.
func facingVector(dir world.Direction) world.Vec2 {
	v := content.DirectionVectors[dir]
	return world.Vec2{X: v[0], Y: v[1]}
}

// near reports whether two vectors match to within float error.
func near(a, b world.Vec2) bool { return math.Abs(a.X-b.X) < 1e-9 && math.Abs(a.Y-b.Y) < 1e-9 }

// projectileFacings counts the projectiles of a caster by their facing.
func projectileFacings(ms *world.MapState, caster world.EntityID) map[world.Direction]int {
	out := map[world.Direction]int{}
	for id := range ms.OfKind(world.EntityBot) {
		if ms.Ag(id).CasterID == caster && ms.Ag(id).Behavior == content.BehaviorSkill {
			out[ms.Mo(id).Direction]++
		}
	}
	return out
}

func TestOneActivationRunsOnlyTheSkillItNames(t *testing.T) {
	s, ms, player := skillFixture(t)
	count := rolls(t, 0)

	if result := s.ActivateSkill(player.ID, ms, "echo", facing, time.Now()); result.Outcome != SkillCast || !result.Accepted() {
		t.Fatalf("result = %+v, want an accepted cast", result)
	}
	if got := summons(ms, player.ID); got[content.BehaviorPassive] != 1 || got[content.BehaviorSkill] != 0 {
		t.Fatalf("summons = %v, want the double alone", got)
	}
	if *count != 1 {
		t.Fatalf("rolls = %d, want one per accepted attempt", *count)
	}
}

func TestOnlyAManualSkillOfAnActiveItemActivates(t *testing.T) {
	s, ms, player := skillFixture(t)
	count := rolls(t, 0)
	ms.Ap(player.ID).Layers[0].Active = false

	for _, id := range []string{"shot", "dose", "scatter", "unknown"} {
		if result := s.ActivateSkill(player.ID, ms, id, facing, time.Now()); result.Outcome != SkillUnavailable || result.Accepted() {
			t.Fatalf("%s = %+v, want a refused unavailable", id, result)
		}
	}
	if !ms.Mt(player.ID).NextSkillAt.IsZero() || *count != 0 || len(summons(ms, player.ID)) != 0 {
		t.Fatal("a refused attempt starts no cooldown, rolls nothing and spawns nothing")
	}
}

func TestTwoItemsThatDefineOneSkillIdTwoWaysRunNeither(t *testing.T) {
	s, ms, player := skillFixture(t)
	rolls(t, 0)
	s.SkillConfig["rifle"] = []content.SkillDefinition{{SkillID: "shot", LogicEventID: "projectile", SummonedEntityItemID: "shell"}}
	ms.Ap(player.ID).Layers = append(ms.Ap(player.ID).Layers, world.ObjectLayerState{ItemID: "rifle", Active: true, Quantity: 1})

	if result := s.ActivateSkill(player.ID, ms, "shot", facing, time.Now()); result.Outcome != SkillUnavailable {
		t.Fatalf("a conflicting skill id = %+v, want unavailable", result)
	}
	s.SkillConfig["rifle"][0].SummonedEntityItemID = "bullet"
	if result := s.ActivateSkill(player.ID, ms, "shot", facing, time.Now()); result.Outcome != SkillCast {
		t.Fatalf("one skill id with one definition on two items = %+v, want a cast", result)
	}
}

func TestTheCooldownRefusesWithoutARollAndAMisfireSpendsIt(t *testing.T) {
	s, ms, player := skillFixture(t)
	count := rolls(t, 1)
	now := time.Now()

	if result := s.ActivateSkill(player.ID, ms, "shot", aimAt(1, 0), now); result.Outcome != SkillMisfire || !result.Accepted() {
		t.Fatalf("result = %+v, want an accepted misfire", result)
	}
	readyAt := ms.Mt(player.ID).NextSkillAt
	if !readyAt.After(now) || len(summons(ms, player.ID)) != 0 {
		t.Fatal("a misfire spends the action cooldown and spawns nothing")
	}
	if result := s.ActivateSkill(player.ID, ms, "echo", facing, readyAt.Add(-time.Nanosecond)); result.Outcome != SkillCooldown {
		t.Fatalf("result = %+v, want the shared action cooldown", result)
	}
	if *count != 1 || ms.Mt(player.ID).NextSkillAt != readyAt {
		t.Fatal("a refused attempt neither rolls nor moves the cooldown")
	}
	if result := s.ActivateSkill(player.ID, ms, "echo", facing, readyAt); result.Outcome != SkillMisfire || *count != 2 {
		t.Fatalf("result = %+v, want a second attempt once the cooldown ends", result)
	}
}

func TestAGhostOrAPlayerInStasisCannotCast(t *testing.T) {
	s, ms, player := skillFixture(t)
	rolls(t, 0)
	ms.Vt(player.ID).RespawnAt = time.Now().Add(time.Minute)
	if result := s.ActivateSkill(player.ID, ms, "shot", facing, time.Now()); result.Outcome != SkillBlocked {
		t.Fatalf("a ghost = %+v, want blocked", result)
	}

	s, ms, player = skillFixture(t)
	conn := &ackConn{}
	player.Client = conn
	FreezePlayer(player, ms)
	s.handleSkillActivateInput(player, ms, &world.InputCommand{Sequence: 3, SkillID: "shot"})
	ack := conn.payloads[len(conn.payloads)-1].(map[string]interface{})
	if ack["reason"] != string(SkillBlocked) || ack["accepted"] != false || len(summons(ms, player.ID)) != 0 {
		t.Fatalf("a player in stasis = %v, want a refused blocked", ack)
	}
}

func TestTheAckAnswersTheSequenceTargetAndAimOfTheActivation(t *testing.T) {
	s, ms, player := skillFixture(t)
	rolls(t, 0)
	conn := &ackConn{}
	player.Client = conn
	enemy := hostileAt(ms, 400, 6, 1)

	cmd := world.InputCommand{Kind: world.InputKindSkillActivate, Sequence: 7, SkillID: "shot", AimMode: world.AimAuto}
	tapTick(s, ms, player, cmd)
	tapTick(s, ms, player, cmd)

	var acks []map[string]interface{}
	for i, msgType := range conn.types {
		if msgType == "skill_ack" {
			acks = append(acks, conn.payloads[i].(map[string]interface{}))
		}
	}
	if len(acks) != 1 || acks[0]["ok"] != true || acks[0]["accepted"] != true || acks[0]["seq"] != uint32(7) ||
		acks[0]["skillId"] != "shot" || acks[0]["targetId"] != enemy || acks[0]["x"] != 1.0 || acks[0]["y"] != 0.0 {
		t.Fatalf("acks = %v, want one cast for sequence 7 at the enemy on the right", acks)
	}
	if ready, _ := acks[0]["readyInMs"].(int); ready <= 0 {
		t.Fatalf("readyInMs = %v, want the cooldown the cast started", acks[0]["readyInMs"])
	}
	if summons(ms, player.ID)[content.BehaviorSkill] != 1 {
		t.Fatal("a sequence applies once: the resent activation casts nothing")
	}
}

func TestAFacingAimFliesEachOfTheEightFacings(t *testing.T) {
	for dir := world.UP; dir < world.NONE; dir++ {
		s, ms, player := skillFixture(t)
		rolls(t, 0)
		ms.Mo(player.ID).Direction = dir
		if result := s.ActivateSkill(player.ID, ms, "shot", facing, time.Now()); result.Aim != facingVector(dir) {
			t.Fatalf("facing %d flew %v", dir, result.Aim)
		}
		if got := projectileFacings(ms, player.ID); got[dir] != 1 || len(got) != 1 {
			t.Fatalf("facing %d: projectiles %v", dir, got)
		}
		v := content.DirectionVectors[dir]
		if directionFromVector(v[0], v[1]) != dir || directionFromVector(3*v[0], 3*v[1]) != dir {
			t.Fatalf("the vector of facing %d maps back to another facing", dir)
		}
	}
}

func TestADirectionAimFliesItsExactAngleAndTurnsAnIdleCaster(t *testing.T) {
	s, ms, player := skillFixture(t)
	rolls(t, 0)
	ms.Mo(player.ID).Direction = world.UP

	want := world.Vec2{X: -2, Y: 0.4}.Unit()
	if result := s.ActivateSkill(player.ID, ms, "shot", aimAt(-2, 0.4), time.Now()); !near(result.Aim, want) {
		t.Fatalf("a drag a little below left flew %v, want %v", result.Aim, want)
	}
	if ms.Mo(player.ID).Direction != world.LEFT {
		t.Fatal("an idle caster turns to the facing nearest its cast")
	}
	if got := projectileFacings(ms, player.ID); got[world.LEFT] != 1 {
		t.Fatalf("projectile facings = %v, want the sprite facing nearest the aim", got)
	}
	ms.Mt(player.ID).NextSkillAt = time.Time{}
	if result := s.ActivateSkill(player.ID, ms, "shot", facing, time.Now()); result.Aim != facingVector(world.LEFT) {
		t.Fatalf("the next facing aim flew %v, want the facing of the last cast", result.Aim)
	}
}

func TestAProjectileFliesStraightAtItsLaunchVelocity(t *testing.T) {
	s, ms, player := skillFixture(t)
	rolls(t, 0)
	aim := world.Vec2{X: 3, Y: 1}.Unit() // 18.4°: between two facings
	s.ActivateSkill(player.ID, ms, "shot", aimAt(3, 1), time.Now())

	var projectile world.EntityID
	for id := range ms.OfKind(world.EntityBot) {
		projectile = id
	}
	speed := s.EntityBaseSpeed * s.projectileSpeedMultiplier
	if !near(ms.Br(projectile).Velocity, world.Vec2{X: aim.X * speed, Y: aim.Y * speed}) {
		t.Fatalf("velocity = %v, want the aim at the projectile speed", ms.Br(projectile).Velocity)
	}
	caster := ms.Tf(player.ID).Pos
	for tick := 1; tick <= 5; tick++ {
		s.updateBots(ms)
		step := float64(tick-1) * speed * TickDuration.Seconds()
		want := world.Vec2{X: caster.X + aim.X*step, Y: caster.Y + aim.Y*step}
		if !near(ms.Tf(projectile).Pos, want) {
			t.Fatalf("tick %d: projectile at %v, want %v on the aim line", tick, ms.Tf(projectile).Pos, want)
		}
	}
}

// hostileAt places a hostile bot at a cell.
func hostileAt(ms *world.MapState, id world.EntityID, x, y float64) world.EntityID {
	addBot(ms, id, world.Transform{Pos: world.Vec2{X: x, Y: y}, Size: world.Vec2{X: 1, Y: 1}}, &world.Brain{}, world.Motion{}, content.BehaviorHostile)
	return id
}

func TestAnAutoAimPicksTheBestHostileInReach(t *testing.T) {
	s, ms, player := skillFixture(t)
	rolls(t, 0)
	ms.Mo(player.ID).Direction = world.RIGHT // the player stands at (1, 1)
	reach := skillHandlers["projectile"].reach(s, content.ComputedStats{})

	behind := hostileAt(ms, 400, 0, 1) // one cell behind
	ahead := hostileAt(ms, 401, 4, 1)  // three cells ahead
	addBot(ms, 402, world.Transform{Pos: world.Vec2{X: 2, Y: 1}, Size: world.Vec2{X: 1, Y: 1}}, &world.Brain{}, world.Motion{}, content.BehaviorPassive)
	far := hostileAt(ms, 403, 1+reach+2, 1) // past the reach

	if got := s.autoTarget(ms, player.ID, 0, reach); got != ahead {
		t.Fatalf("target = %d, want the hostile ahead before a nearer one behind and the passive bot", got)
	}
	if got := s.autoTarget(ms, player.ID, behind, reach); got != behind {
		t.Fatalf("target = %d, want the preferred target while it stays a candidate", got)
	}
	if got := s.autoTarget(ms, player.ID, far, reach); got != ahead {
		t.Fatalf("target = %d, want a preferred target out of reach dropped", got)
	}
	ms.Br(behind).CurrentTargetPlayer = player.ID
	if got := s.autoTarget(ms, player.ID, 0, reach); got != behind {
		t.Fatalf("target = %d, want the hostile that pursues the player first", got)
	}
	ms.Vt(behind).RespawnAt = time.Now().Add(time.Minute)
	ms.Vt(ahead).RespawnAt = time.Now().Add(time.Minute)
	if got := s.autoTarget(ms, player.ID, behind, reach); got != 0 {
		t.Fatalf("target = %d, want none: the dead are no candidates", got)
	}
}

func TestAnAutoAimBreaksTiesByDistanceThenId(t *testing.T) {
	s, ms, player := skillFixture(t)
	ms.Mo(player.ID).Direction = world.DOWN
	reach := skillHandlers["projectile"].reach(s, content.ComputedStats{})
	hostileAt(ms, 411, 1, 5)
	hostileAt(ms, 410, 1, 5)
	hostileAt(ms, 409, 1, 7)
	for range 20 {
		if got := s.autoTarget(ms, player.ID, 0, reach); got != 410 {
			t.Fatalf("target = %d, want the nearest with the lowest id, every time", got)
		}
	}
}

func TestAnAutoAimTakesAPlayerOnlyAfterItHurtTheCaster(t *testing.T) {
	s, ms, player := skillFixture(t)
	reach := skillHandlers["projectile"].reach(s, content.ComputedStats{})
	other := &world.Session{}
	other.ID, other.MapCode, other.Client = 50, "test", nopConn{}
	s.Sessions[other.ID] = other
	ms.Spawn(other.ID, world.EntityData{Kind: world.EntityPlayer}, world.Transform{Pos: world.Vec2{X: 3, Y: 1}, Size: world.Vec2{X: 1, Y: 1}}, world.Appearance{})
	ms.AddVitals(other.ID, world.Vitals{Life: 100, MaxLife: 100})
	ms.AddMotion(other.ID, world.Motion{Direction: world.SpawnFacing})
	ms.AddMortal(other.ID, &world.Mortal{})
	ms.AddPlayer(other.ID)

	if got := s.autoTarget(ms, player.ID, other.ID, reach); got != 0 {
		t.Fatalf("target = %d, want no player that never hurt the caster", got)
	}
	ms.Mt(player.ID).DamageLedger = map[world.EntityID]float64{other.ID: 5}
	if got := s.autoTarget(ms, player.ID, 0, reach); got != other.ID {
		t.Fatalf("target = %d, want the player that hurt the caster", got)
	}
	other.Frozen = true
	if got := s.autoTarget(ms, player.ID, 0, reach); got != 0 {
		t.Fatalf("target = %d, want no player in stasis", got)
	}
}

func TestAnAutoAimWithNoTargetFliesTheFacing(t *testing.T) {
	s, ms, player := skillFixture(t)
	rolls(t, 0)
	ms.Mo(player.ID).Direction = world.UP_LEFT

	result := s.ActivateSkill(player.ID, ms, "shot", auto, time.Now())
	if result.Outcome != SkillCast || result.Target != 0 || result.Aim != facingVector(world.UP_LEFT) {
		t.Fatalf("result = %+v, want a cast along the facing with no target", result)
	}
}

func TestAnAutoAimFliesStraightAtItsTarget(t *testing.T) {
	s, ms, player := skillFixture(t)
	rolls(t, 0)
	ms.Mo(player.ID).Direction = world.RIGHT // the player stands at (1, 1)
	enemy := hostileAt(ms, 400, 4, 2)

	result := s.ActivateSkill(player.ID, ms, "shot", auto, time.Now())
	if want := (world.Vec2{X: 3, Y: 1}).Unit(); result.Target != enemy || !near(result.Aim, want) {
		t.Fatalf("result = %+v, want the cast at the center of the enemy, %v", result, want)
	}
}

func TestATapRegeneratesWithoutTheSkillCooldown(t *testing.T) {
	s, ms, player := skillFixture(t)
	rolls(t, 0)

	s.handlePlayerActionInput(player, ms, &world.InputCommand{TargetX: 3, TargetY: 3})
	if ms.Mt(player.ID).NextRegenAt.IsZero() || !ms.Mt(player.ID).NextSkillAt.IsZero() {
		t.Fatal("a tap starts the regeneration cooldown alone")
	}
	if result := s.ActivateSkill(player.ID, ms, "shot", aimAt(1, 0), time.Now()); result.Outcome != SkillCast {
		t.Fatalf("a skill right after a tap = %+v, want a cast", result)
	}
}

func TestABotCastsOneManualSkillOfItsActiveItems(t *testing.T) {
	s, ms, _ := skillFixture(t)
	rolls(t, 0)
	bot := world.EntityID(300)
	addBot(ms, bot, world.Transform{Pos: world.Vec2{X: 5, Y: 5}, Size: world.Vec2{X: 1, Y: 1}}, &world.Brain{}, world.Motion{}, content.BehaviorHostile)
	ms.AddMortal(bot, &world.Mortal{})
	ms.Ap(bot).Layers = []world.ObjectLayerState{
		{ItemID: "gun", Active: true, Quantity: 1},
		{ItemID: "coin", Active: true, Quantity: 1},
		{ItemID: "gel", Active: true, Quantity: 1},
	}

	s.castBotSkill(bot, ms, world.Vec2{X: 9, Y: 5})
	s.castBotSkill(bot, ms, world.Vec2{X: 9, Y: 5})

	if got := summons(ms, bot); got[content.BehaviorSkill] != 1 || len(got) != 1 {
		t.Fatalf("summons = %v, want one projectile inside one cooldown", got)
	}
}
