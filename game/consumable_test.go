package game

import (
	"testing"
	"time"
)

// usingPlayer is the movement fixture player holding the given stacks, with
// the gel consumable and the crate resource bound. No loot default names a buff
// factor, so a use restores 10% of max life.
func usingPlayer(t *testing.T, life float64, stacks ...ObjectLayerState) (*GameServer, *MapState, *PlayerState) {
	t.Helper()
	s, ms, player := movementFixture(t)
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{
		"gel":   {Data: ObjectLayerData{Item: Item{ID: "gel", Type: "consumable"}}},
		"crate": {Data: ObjectLayerData{Item: Item{ID: "crate", Type: "resource"}}},
	}); err != nil {
		t.Fatal(err)
	}
	player.Life = life
	player.ObjectLayers = stacks
	return s, ms, player
}

func use(seq uint32, itemID string) InputCommand {
	return InputCommand{Kind: InputKindItemUse, Sequence: seq, ItemID: itemID}
}

func TestUsingAConsumableHealsAndSpendsOneUnit(t *testing.T) {
	s, ms, player := usingPlayer(t, 50, ObjectLayerState{ItemID: "gel", Quantity: 2})

	tapTick(s, ms, player, use(1, "gel"))

	if player.Life != 60 || s.playerItemQuantity(player, "gel") != 1 {
		t.Fatalf("life=%v gel=%d, want 60 and 1", player.Life, s.playerItemQuantity(player, "gel"))
	}
	if !player.NextItemUseAt.After(time.Now()) {
		t.Fatal("a use must start the item-use cooldown")
	}
}

func TestAUseHealsByTheBuffFactorOfTheItemsLootDefault(t *testing.T) {
	s, ms, player := usingPlayer(t, 50, ObjectLayerState{ItemID: "gel", Quantity: 1})
	player.MaxLife = 200
	s.entityDefaultBuilds = []EntityTypeDefaultConfig{
		{EntityType: "loot", LiveItemIDs: []string{"gel"}, BuffFactors: map[string]float64{"gel": 0.3}},
	}

	tapTick(s, ms, player, use(1, "gel"))

	if player.Life != 110 {
		t.Fatalf("life = %v, want 30%% of max life 200 restored", player.Life)
	}
}

func TestTheLastUnitLeavesTheInventory(t *testing.T) {
	s, ms, player := usingPlayer(t, 50, ObjectLayerState{ItemID: "gel", Quantity: 1})

	tapTick(s, ms, player, use(1, "gel"))

	if len(player.ObjectLayers) != 0 {
		t.Fatalf("an emptied stack stays: %+v", player.ObjectLayers)
	}
}

func TestARejectedUseSpendsNothing(t *testing.T) {
	cases := map[string]struct {
		itemID string
		setup  func(*PlayerState)
	}{
		itemUseRejectDead:          {"gel", func(p *PlayerState) { p.RespawnTime = time.Now().Add(time.Minute) }},
		itemUseRejectNotOwned:      {"salve", func(*PlayerState) {}},
		itemUseRejectNotConsumable: {"crate", func(*PlayerState) {}},
		itemUseRejectCooldown:      {"gel", func(p *PlayerState) { p.NextItemUseAt = time.Now().Add(time.Second) }},
		itemUseRejectFullLife:      {"gel", func(p *PlayerState) { p.Life = p.MaxLife }},
	}
	for want, tc := range cases {
		s, ms, player := usingPlayer(t, 50,
			ObjectLayerState{ItemID: "gel", Quantity: 1}, ObjectLayerState{ItemID: "crate", Quantity: 1})
		tc.setup(player)
		life := player.Life

		if got := s.itemUseRejection(player, tc.itemID, time.Now()); got != want {
			t.Errorf("%s: rejection = %q", want, got)
		}
		tapTick(s, ms, player, use(1, tc.itemID))
		if player.Life != life || s.playerItemQuantity(player, "gel") != 1 || s.playerItemQuantity(player, "crate") != 1 {
			t.Errorf("%s: a rejected use changed life %v→%v or the stacks %+v", want, life, player.Life, player.ObjectLayers)
		}
	}
}

func TestARepeatedUseInsideTheCooldownAppliesOnce(t *testing.T) {
	s, ms, player := usingPlayer(t, 20, ObjectLayerState{ItemID: "gel", Quantity: 3})

	tapTick(s, ms, player, use(1, "gel"), use(2, "gel"))
	tapTick(s, ms, player, use(3, "gel"))

	if player.Life != 30 || s.playerItemQuantity(player, "gel") != 2 {
		t.Fatalf("life=%v gel=%d, want one use: 30 and 2", player.Life, s.playerItemQuantity(player, "gel"))
	}
}

func TestItemUseNamesOneItem(t *testing.T) {
	if _, ok := parseInput("item_use", &inputPayload{}); ok {
		t.Fatal("an item_use without an item id must fail validation")
	}
	cmd, ok := parseInput("item_use", &inputPayload{ItemID: "gel"})
	if !ok || cmd.Kind != InputKindItemUse || cmd.ItemID != "gel" {
		t.Fatalf("item_use parsed to %+v", cmd)
	}
}
