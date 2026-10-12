package game

import (
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

// gelHeal is the resistance of the gel consumable: the life one unit restores.
const gelHeal = 20

// healingServer is the movement fixture with the gel consumable bound to the
// zero-waste heal skill, the unbound crate resource, and the player holding the
// given stacks at the given life.
func healingServer(t *testing.T, life float64, stacks ...ObjectLayerState) (*GameServer, *MapState, *PlayerState) {
	t.Helper()
	s, ms, player := movementFixture(t)
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{
		"gel":   {Data: ObjectLayerData{Item: Item{ID: "gel", Type: "consumable"}, Stats: Stats{Resistance: gelHeal}}},
		"crate": {Data: ObjectLayerData{Item: Item{ID: "crate", Type: "resource"}}},
	}); err != nil {
		t.Fatal(err)
	}
	s.skillConfig = map[string][]SkillDefinition{
		"gel": {{LogicEventID: "zero-waste-auto-consume-heal", SummonedEntityItemID: "gel"}},
	}
	s.InitSkills()
	ms.vt(player.ID).Life = life
	ms.ap(player.ID).Layers = stacks
	return s, ms, player
}

func gel(quantity int) ObjectLayerState {
	return ObjectLayerState{ItemID: "gel", Quantity: quantity}
}

func TestAHeldConsumableWaitsUntilTheMissingLifeTakesItsWholeHeal(t *testing.T) {
	s, ms, player := healingServer(t, 100-gelHeal+1, gel(2))

	s.phaseSkills(0, ms)
	if ms.vt(player.ID).Life != 100-gelHeal+1 || s.playerItemQuantity(player, "gel") != 2 {
		t.Fatalf("life=%v gel=%d, want no heal while it would waste life", ms.vt(player.ID).Life, s.playerItemQuantity(player, "gel"))
	}

	ms.vt(player.ID).Life = 100 - gelHeal
	s.phaseSkills(0, ms)
	if ms.vt(player.ID).Life != 100 || s.playerItemQuantity(player, "gel") != 1 {
		t.Fatalf("life=%v gel=%d, want one inactive unit consumed to full life", ms.vt(player.ID).Life, s.playerItemQuantity(player, "gel"))
	}
	if !ms.mt(player.ID).NextConsumeAt.After(time.Now()) {
		t.Fatal("a consumed unit must start the consume cooldown")
	}
}

func TestTheConsumeCooldownSpacesTwoHeals(t *testing.T) {
	s, ms, player := healingServer(t, 10, gel(3))

	s.phaseSkills(0, ms)
	s.phaseSkills(0, ms)
	if ms.vt(player.ID).Life != 10+gelHeal || s.playerItemQuantity(player, "gel") != 2 {
		t.Fatalf("life=%v gel=%d, want one heal inside the cooldown", ms.vt(player.ID).Life, s.playerItemQuantity(player, "gel"))
	}

	ms.mt(player.ID).NextConsumeAt = time.Now()
	s.phaseSkills(0, ms)
	if ms.vt(player.ID).Life != 10+2*gelHeal || s.playerItemQuantity(player, "gel") != 1 {
		t.Fatalf("life=%v gel=%d, want a second heal once the cooldown ends", ms.vt(player.ID).Life, s.playerItemQuantity(player, "gel"))
	}
}

func TestTheLastUnitLeavesTheInventory(t *testing.T) {
	s, ms, player := healingServer(t, 10, gel(1))

	s.phaseSkills(0, ms)

	if len(ms.ap(player.ID).Layers) != 0 {
		t.Fatalf("an emptied stack stays: %+v", ms.ap(player.ID).Layers)
	}
}

func TestNothingHealsAGhostOrFromAnItemWithoutTheSkill(t *testing.T) {
	s, ms, ghost := healingServer(t, 10, gel(1))
	ms.vt(ghost.ID).RespawnAt = time.Now().Add(time.Minute)
	s.phaseSkills(0, ms)
	if ms.vt(ghost.ID).Life != 10 || s.playerItemQuantity(ghost, "gel") != 1 {
		t.Fatalf("a ghost healed: life=%v gel=%d", ms.vt(ghost.ID).Life, s.playerItemQuantity(ghost, "gel"))
	}

	s, ms, player := healingServer(t, 10, ObjectLayerState{ItemID: "crate", Quantity: 1})
	s.phaseSkills(0, ms)
	if ms.vt(player.ID).Life != 10 || s.playerItemQuantity(player, "crate") != 1 {
		t.Fatalf("an item without the skill healed: life=%v crate=%d", ms.vt(player.ID).Life, s.playerItemQuantity(player, "crate"))
	}
}

func TestOnlyALootCollectingBotHealsFromItsHeldUnits(t *testing.T) {
	s, ms, _ := healingServer(t, 100)
	ids := map[string]EntityID{BehaviorHostile: 200, BehaviorDrop: 201, BehaviorProvider: 202}
	for behavior, id := range ids {
		addBot(ms, id, Transform{Size: Vec2{X: 1, Y: 1}}, &Brain{}, Motion{}, behavior)
		ms.ap(id).Layers = []ObjectLayerState{gel(1)}
		ms.vt(id).Life = 10
	}

	s.phaseSkills(0, ms)

	if raider := ids[BehaviorHostile]; ms.vt(raider).Life != 10+gelHeal || len(ms.ap(raider).Layers) != 0 {
		t.Fatalf("the hostile bot must heal from its gel: life=%v layers=%+v", ms.vt(raider).Life, ms.ap(raider).Layers)
	}
	for _, behavior := range []string{BehaviorDrop, BehaviorProvider} {
		if bot := ids[behavior]; ms.vt(bot).Life != 10 || len(ms.ap(bot).Layers) != 1 {
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
