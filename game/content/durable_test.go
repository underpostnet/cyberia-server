package content

import (
	"reflect"
	"testing"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
	"cyberia-server/game/world/worldtest"
)

// durableFixture is a level-1 account player "prof" in a world that binds
// sword, gem and two skins, with one quest and one vault terminal.
func durableFixture(layers ...world.ObjectLayerState) (*Content, *world.Session) {
	s := &Content{World: &world.World{}, CoinItemID: "coin"}
	s.ObjectLayerDataCache = map[string]*ObjectLayer{
		"sword":  {Cid: "cid-sword", Data: ObjectLayerData{Item: Item{ID: "sword", Type: "weapon"}}},
		"gem":    {Cid: "cid-gem", Data: ObjectLayerData{Item: Item{ID: "gem", Type: "material"}}},
		"skin-a": {Cid: "cid-a", Data: ObjectLayerData{Item: Item{ID: "skin-a", Type: "skin"}}},
		"skin-b": {Cid: "cid-b", Data: ObjectLayerData{Item: Item{ID: "skin-b", Type: "skin"}}},
	}
	s.EquipmentRules = EquipmentRulesConfig{
		ActiveItemTypes: map[string]bool{"skin": true, "weapon": true},
		OnePerType:      true,
		RequireSkin:     true,
	}
	s.EntityDefaults = map[string]EntityTypeDefaultConfig{
		"player": {DefaultObjectLayers: []world.ObjectLayerState{{ItemID: "skin-a", Active: true, Quantity: 1}}},
	}
	s.questDefs = map[string]*CyberiaQuest{
		"q1": {Code: "q1", Steps: []QuestStep{{ID: "s1", Objectives: []QuestObjective{{Type: "kill", ItemID: "rat", Quantity: 3}}}}},
	}
	s.actionCache = map[world.EntityID]*CyberiaAction{50: {Code: "bank", StorageSlots: 4}}
	player := worldtest.PlacePlayer(s.World, 1, layers)
	player.PlayerID, player.AccountID = "prof", "acc"
	return s, player
}

func TestRestoreThenCaptureKeepsTheSavedState(t *testing.T) {
	s, player := durableFixture()
	saved := persist.State{
		Coins:   30,
		XP:      450,
		Bag:     []persist.Stack{{ItemID: "skin-b", CID: "cid-b", Quantity: 1, Slot: 0, Active: true}, {ItemID: "sword", CID: "cid-sword", Quantity: 1, Slot: 2, Active: true}},
		Storage: []persist.Stack{{ItemID: "gem", CID: "cid-gem", Quantity: 7, Slot: 5}},
		Vaults:  []persist.Vault{{ActionCode: "bank", Stacks: []persist.Stack{{ItemID: "gem", CID: "cid-gem", Quantity: 2, Slot: 1}}}},
		Quests: []persist.Quest{{Code: "q1", Status: "active", StartedAtMs: 1000,
			Steps: []persist.QuestStep{{StepID: "s1", Objectives: []persist.Objective{{Current: 2, Required: 3}}}}}},
	}
	s.RestoreState(player, saved)
	got := s.DurableState(player)
	got.Position = persist.Position{}
	if !reflect.DeepEqual(got, saved) {
		t.Fatalf("a restored state captures back unchanged:\n got %+v\nwant %+v", got, saved)
	}
	if level := s.EntityProgression(player.ID, s.Maps["test"]).Level; level != s.ProgressionConfig().LevelAtXP(450) {
		t.Fatalf("the level derives from the saved XP, got %d", level)
	}
}

func TestStacksTheWorldCannotLoadAreHeldUntouched(t *testing.T) {
	s, player := durableFixture()
	s.RestoreState(player, persist.State{
		Bag: []persist.Stack{
			{ItemID: "sword", CID: "cid-sword-v1", Quantity: 1, Slot: 0},
			{ItemID: "relic", CID: "cid-relic", Quantity: 1, Slot: 1},
			{ItemID: "gem", CID: "cid-gem", Quantity: 1, Slot: 30},
		},
		Vaults: []persist.Vault{{ActionCode: "gone", Stacks: []persist.Stack{{ItemID: "gem", CID: "cid-gem", Quantity: 4, Slot: 0}}}},
	})
	reasons := map[string]string{}
	for _, h := range s.Held[player.PlayerID] {
		reasons[h.Stack.ItemID+"/"+h.Container] = h.Reason
	}
	want := map[string]string{
		"sword/bag": persist.HeldRebound,
		"relic/bag": persist.HeldUnknown,
		"gem/vault": persist.HeldNoRoom,
	}
	if !reflect.DeepEqual(reasons, want) {
		t.Fatalf("held reasons = %v, want %v", reasons, want)
	}
	if gem := bagAt(s, player, 0); gem.ItemID != "gem" {
		t.Fatalf("a stack out of the open slots moves to the lowest free one, bag slot 0 = %+v", gem)
	}

	// The binding returns: the held stack loads at the next session.
	s.ObjectLayerDataCache["sword"].Cid = "cid-sword-v1"
	state := s.DurableState(player)
	s2, again := durableFixture()
	s2.ObjectLayerDataCache["sword"].Cid = "cid-sword-v1"
	s2.RestoreState(again, state)
	back := false
	for _, l := range *s2.InventoryLayers(again) {
		back = back || (l.ItemID == "sword" && l.CID == "cid-sword-v1")
	}
	if !back || len(s2.Held[again.PlayerID]) != 2 {
		t.Fatalf("a held stack whose definition loads again returns to its container: %+v held=%+v", *s2.InventoryLayers(again), s2.Held[again.PlayerID])
	}
}

func TestStacksMergeOnlyWithTheSameDefinition(t *testing.T) {
	s, player := durableFixture(world.ObjectLayerState{ItemID: "gem", CID: "cid-gem-v0", Quantity: 2})
	s.SeatStacks(*s.InventoryLayers(player), 0)
	if !s.ExchangeItems(player, nil, []Stack{{ItemID: "gem", Qty: 3}}) {
		t.Fatal("the grant fits")
	}
	if old, granted := bagAt(s, player, 0), bagAt(s, player, 1); old.Quantity != 2 || granted.Quantity != 3 || granted.CID != "cid-gem" {
		t.Fatalf("a grant of a rebound label takes its own stack: %+v and %+v", old, granted)
	}
	move(s, player, world.BoxBag, 1, world.BoxBag, 0, 3)
	if a, b := bagAt(s, player, 0), bagAt(s, player, 1); a.Quantity != 3 || a.CID != "cid-gem" || b.Quantity != 2 || b.CID != "cid-gem-v0" {
		t.Fatalf("stacks of two definitions swap, never stack: %+v and %+v", a, b)
	}
}

func TestNormalizeLoadoutFollowsTheEquipmentRules(t *testing.T) {
	s, player := durableFixture()
	s.RestoreState(player, persist.State{Bag: []persist.Stack{
		{ItemID: "skin-a", CID: "cid-a", Quantity: 1, Slot: 0, Active: true},
		{ItemID: "skin-b", CID: "cid-b", Quantity: 1, Slot: 1, Active: true},
		{ItemID: "gem", CID: "cid-gem", Quantity: 1, Slot: 2, Active: true},
	}})
	if a, b, gem := bagAt(s, player, 0), bagAt(s, player, 1), bagAt(s, player, 2); !a.Active || b.Active || gem.Active {
		t.Fatalf("one skin stays worn and a material never is: %+v %+v %+v", a, b, gem)
	}

	s2, bare := durableFixture()
	s2.RestoreState(bare, persist.State{Bag: []persist.Stack{{ItemID: "sword", CID: "cid-sword", Quantity: 1, Slot: 0}}})
	skin := false
	for _, l := range *s2.InventoryLayers(bare) {
		skin = skin || (l.ItemID == "skin-a" && l.Active && l.CID == "cid-a")
	}
	if !skin {
		t.Fatalf("a player with no skin gets the default skin: %+v", *s2.InventoryLayers(bare))
	}
}

func TestQuestsRestoreFromTheirDefinitionAndUnknownOnesStayDormant(t *testing.T) {
	s, player := durableFixture()
	s.RestoreState(player, persist.State{Quests: []persist.Quest{
		{Code: "q1", Status: "completed", CompletedAtMs: 5000, Steps: []persist.QuestStep{{StepID: "s1"}}},
		{Code: "retired", Status: "active", Steps: []persist.QuestStep{{StepID: "x", Objectives: []persist.Objective{{Current: 1, Required: 2}}}}},
	}})
	qp, ok := s.playerQuest(player, "q1")
	if !ok || qp.Status != "completed" || qp.Steps[0].Objectives[0].Type != "kill" || !qp.Steps[0].Consumed {
		t.Fatalf("a completed quest restores complete with its objective types: %+v", qp)
	}
	quests := s.DurableState(player).Quests
	if len(quests) != 2 || quests[1].Code != "retired" || quests[1].Steps[0].Objectives[0].Current != 1 {
		t.Fatalf("a quest the world no longer defines keeps its record: %+v", quests)
	}
	qp.Status = "failed"
	if quests := s.DurableState(player).Quests; len(quests) != 1 {
		t.Fatalf("an abandoned quest has no record: %+v", quests)
	}
}

func TestARunningCraftIsRefundedWhenTheSessionEnds(t *testing.T) {
	s, player := durableFixture()
	s.crafts = map[world.EntityID]*ActiveCraft{player.ID: {Refund: []Stack{{ItemID: "gem", Qty: 2}}}}
	s.RefundCraft(player)
	if gem := bagAt(s, player, 0); gem.ItemID != "gem" || gem.Quantity != 2 {
		t.Fatalf("the refund goes to the bag: %+v", *s.InventoryLayers(player))
	}

	full := make([]world.ObjectLayerState, 0, 24)
	for range 24 {
		full = append(full, world.ObjectLayerState{ItemID: "sword", CID: "cid-sword", Quantity: 1})
	}
	s2, packed := durableFixture(full...)
	s2.SeatStacks(*s2.InventoryLayers(packed), 0)
	s2.crafts = map[world.EntityID]*ActiveCraft{packed.ID: {Refund: []Stack{{ItemID: "gem", Qty: 2}}}}
	s2.RefundCraft(packed)
	if held := s2.Held[packed.PlayerID]; len(held) != 1 || held[0].Reason != persist.HeldNoRoom || held[0].Stack.Quantity != 2 {
		t.Fatalf("a refund the bag cannot take is held: %+v", held)
	}
}
