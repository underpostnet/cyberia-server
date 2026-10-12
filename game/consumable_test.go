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
	player.Life = life
	player.ObjectLayers = stacks
	return s, ms, player
}

func gel(quantity int) ObjectLayerState {
	return ObjectLayerState{ItemID: "gel", Quantity: quantity}
}

func TestAHeldConsumableWaitsUntilTheMissingLifeTakesItsWholeHeal(t *testing.T) {
	s, ms, player := healingServer(t, 100-gelHeal+1, gel(2))

	s.phaseSkills(0, ms)
	if player.Life != 100-gelHeal+1 || s.playerItemQuantity(player, "gel") != 2 {
		t.Fatalf("life=%v gel=%d, want no heal while it would waste life", player.Life, s.playerItemQuantity(player, "gel"))
	}

	player.Life = 100 - gelHeal
	s.phaseSkills(0, ms)
	if player.Life != 100 || s.playerItemQuantity(player, "gel") != 1 {
		t.Fatalf("life=%v gel=%d, want one inactive unit consumed to full life", player.Life, s.playerItemQuantity(player, "gel"))
	}
	if !player.NextConsumeAt.After(time.Now()) {
		t.Fatal("a consumed unit must start the consume cooldown")
	}
}

func TestTheConsumeCooldownSpacesTwoHeals(t *testing.T) {
	s, ms, player := healingServer(t, 10, gel(3))

	s.phaseSkills(0, ms)
	s.phaseSkills(0, ms)
	if player.Life != 10+gelHeal || s.playerItemQuantity(player, "gel") != 2 {
		t.Fatalf("life=%v gel=%d, want one heal inside the cooldown", player.Life, s.playerItemQuantity(player, "gel"))
	}

	player.NextConsumeAt = time.Now()
	s.phaseSkills(0, ms)
	if player.Life != 10+2*gelHeal || s.playerItemQuantity(player, "gel") != 1 {
		t.Fatalf("life=%v gel=%d, want a second heal once the cooldown ends", player.Life, s.playerItemQuantity(player, "gel"))
	}
}

func TestTheLastUnitLeavesTheInventory(t *testing.T) {
	s, ms, player := healingServer(t, 10, gel(1))

	s.phaseSkills(0, ms)

	if len(player.ObjectLayers) != 0 {
		t.Fatalf("an emptied stack stays: %+v", player.ObjectLayers)
	}
}

func TestNothingHealsAGhostOrFromAnItemWithoutTheSkill(t *testing.T) {
	s, ms, ghost := healingServer(t, 10, gel(1))
	ghost.RespawnTime = time.Now().Add(time.Minute)
	s.phaseSkills(0, ms)
	if ghost.Life != 10 || s.playerItemQuantity(ghost, "gel") != 1 {
		t.Fatalf("a ghost healed: life=%v gel=%d", ghost.Life, s.playerItemQuantity(ghost, "gel"))
	}

	s, ms, player := healingServer(t, 10, ObjectLayerState{ItemID: "crate", Quantity: 1})
	s.phaseSkills(0, ms)
	if player.Life != 10 || s.playerItemQuantity(player, "crate") != 1 {
		t.Fatalf("an item without the skill healed: life=%v crate=%d", player.Life, s.playerItemQuantity(player, "crate"))
	}
}

func TestOnlyALootCollectingBotHealsFromItsHeldUnits(t *testing.T) {
	s, ms, _ := healingServer(t, 100)
	ids := map[string]EntityID{BehaviorHostile: 200, BehaviorDrop: 201, BehaviorProvider: 202}
	for behavior, id := range ids {
		ms.bots[id] = &BotState{EntityBase: EntityBase{ID: id, ObjectLayers: []ObjectLayerState{gel(1)}},
			Mortal: Mortal{Life: 10, MaxLife: 100}, Behavior: behavior}
		ms.spawn(id, Transform{Size: Vec2{X: 1, Y: 1}})
	}

	s.phaseSkills(0, ms)

	if raider := ms.bots[ids[BehaviorHostile]]; raider.Life != 10+gelHeal || len(raider.ObjectLayers) != 0 {
		t.Fatalf("the hostile bot must heal from its gel: life=%v layers=%+v", raider.Life, raider.ObjectLayers)
	}
	for _, behavior := range []string{BehaviorDrop, BehaviorProvider} {
		if bot := ms.bots[ids[behavior]]; bot.Life != 10 || len(bot.ObjectLayers) != 1 {
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
