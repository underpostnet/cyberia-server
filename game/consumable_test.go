package game

import (
	"cyberia-server/game/content"
	"cyberia-server/game/world"
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

// gelHeal is the resistance of the gel consumable: the life one unit restores.
const gelHeal = 20

// healingServer is the movement fixture with the gel consumable bound to the
// zero-waste heal skill, the unbound crate resource, and the player holding the
// given stacks at the given life.
func healingServer(t *testing.T, life float64, stacks ...world.ObjectLayerState) (*Sim, *world.MapState, *world.Session) {
	t.Helper()
	s, ms, player := movementFixture(t)
	if err := s.ReplaceObjectLayerCache(map[string]*content.ObjectLayer{
		"gel":   {Data: content.ObjectLayerData{Item: content.Item{ID: "gel", Type: "consumable"}, Stats: content.Stats{Resistance: gelHeal}}},
		"crate": {Data: content.ObjectLayerData{Item: content.Item{ID: "crate", Type: "resource"}}},
	}); err != nil {
		t.Fatal(err)
	}
	s.SkillConfig = map[string][]content.SkillDefinition{
		"gel": {{LogicEventID: "zero-waste-auto-consume-heal", SummonedEntityItemID: "gel"}},
	}
	s.InitSkills()
	ms.Vt(player.ID).Life = life
	ms.Ap(player.ID).Layers = stacks
	return s, ms, player
}

func gel(quantity int) world.ObjectLayerState {
	return world.ObjectLayerState{ItemID: "gel", Quantity: quantity}
}

func TestAHeldConsumableWaitsUntilTheMissingLifeTakesItsWholeHeal(t *testing.T) {
	s, ms, player := healingServer(t, 100-gelHeal+1, gel(2))

	s.phaseSkills(0, ms)
	if ms.Vt(player.ID).Life != 100-gelHeal+1 || itemQty(s.World, player, "gel") != 2 {
		t.Fatalf("life=%v gel=%d, want no heal while it would waste life", ms.Vt(player.ID).Life, itemQty(s.World, player, "gel"))
	}

	ms.Vt(player.ID).Life = 100 - gelHeal
	s.phaseSkills(0, ms)
	if ms.Vt(player.ID).Life != 100 || itemQty(s.World, player, "gel") != 1 {
		t.Fatalf("life=%v gel=%d, want one inactive unit consumed to full life", ms.Vt(player.ID).Life, itemQty(s.World, player, "gel"))
	}
	if !ms.Mt(player.ID).NextConsumeAt.After(time.Now()) {
		t.Fatal("a consumed unit must start the consume cooldown")
	}
}

func TestTheConsumeCooldownSpacesTwoHeals(t *testing.T) {
	s, ms, player := healingServer(t, 10, gel(3))

	s.phaseSkills(0, ms)
	s.phaseSkills(0, ms)
	if ms.Vt(player.ID).Life != 10+gelHeal || itemQty(s.World, player, "gel") != 2 {
		t.Fatalf("life=%v gel=%d, want one heal inside the cooldown", ms.Vt(player.ID).Life, itemQty(s.World, player, "gel"))
	}

	ms.Mt(player.ID).NextConsumeAt = time.Now()
	s.phaseSkills(0, ms)
	if ms.Vt(player.ID).Life != 10+2*gelHeal || itemQty(s.World, player, "gel") != 1 {
		t.Fatalf("life=%v gel=%d, want a second heal once the cooldown ends", ms.Vt(player.ID).Life, itemQty(s.World, player, "gel"))
	}
}

func TestTheLastUnitLeavesTheInventory(t *testing.T) {
	s, ms, player := healingServer(t, 10, gel(1))

	s.phaseSkills(0, ms)

	if len(ms.Ap(player.ID).Layers) != 0 {
		t.Fatalf("an emptied stack stays: %+v", ms.Ap(player.ID).Layers)
	}
}

func TestNothingHealsAGhostOrFromAnItemWithoutTheSkill(t *testing.T) {
	s, ms, ghost := healingServer(t, 10, gel(1))
	ms.Vt(ghost.ID).RespawnAt = time.Now().Add(time.Minute)
	s.phaseSkills(0, ms)
	if ms.Vt(ghost.ID).Life != 10 || itemQty(s.World, ghost, "gel") != 1 {
		t.Fatalf("a ghost healed: life=%v gel=%d", ms.Vt(ghost.ID).Life, itemQty(s.World, ghost, "gel"))
	}

	s, ms, player := healingServer(t, 10, world.ObjectLayerState{ItemID: "crate", Quantity: 1})
	s.phaseSkills(0, ms)
	if ms.Vt(player.ID).Life != 10 || itemQty(s.World, player, "crate") != 1 {
		t.Fatalf("an item without the skill healed: life=%v crate=%d", ms.Vt(player.ID).Life, itemQty(s.World, player, "crate"))
	}
}

func TestOnlyALootCollectingBotHealsFromItsHeldUnits(t *testing.T) {
	s, ms, _ := healingServer(t, 100)
	ids := map[string]world.EntityID{content.BehaviorHostile: 200, content.BehaviorDrop: 201, content.BehaviorProvider: 202}
	for behavior, id := range ids {
		addBot(ms, id, world.Transform{Size: world.Vec2{X: 1, Y: 1}}, &world.Brain{}, world.Motion{}, behavior)
		ms.Ap(id).Layers = []world.ObjectLayerState{gel(1)}
		ms.Vt(id).Life = 10
	}

	s.phaseSkills(0, ms)

	if raider := ids[content.BehaviorHostile]; ms.Vt(raider).Life != 10+gelHeal || len(ms.Ap(raider).Layers) != 0 {
		t.Fatalf("the hostile bot must heal from its gel: life=%v layers=%+v", ms.Vt(raider).Life, ms.Ap(raider).Layers)
	}
	for _, behavior := range []string{content.BehaviorDrop, content.BehaviorProvider} {
		if bot := ids[behavior]; ms.Vt(bot).Life != 10 || len(ms.Ap(bot).Layers) != 1 {
			t.Fatalf("a %s bot consumed its gel", behavior)
		}
	}
}

func TestTheSkillMapMarksOnlyAHeldSkill(t *testing.T) {
	s := NewGameServer()
	if err := s.ApplyInstanceConfig(&pb.InstanceConfig{SkillConfig: []*pb.SkillConfigEntry{
		{TriggerItemId: "gel", Skills: []*pb.SkillDetail{{LogicEventId: "zero-waste-auto-consume-heal", SummonedEntityItemId: "gel"}}},
		{TriggerItemId: "pistol", Skills: []*pb.SkillDetail{{LogicEventId: "projectile", SummonedEntityItemId: "bullet"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	skills := s.buildSkillMap()
	if !skills["gel"][0].Held || skills["pistol"][0].Held {
		t.Fatalf("skill map = %+v, want only the zero-waste heal held", skills)
	}
}
