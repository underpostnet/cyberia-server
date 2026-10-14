package content

import (
	"fmt"
	"testing"
	"time"

	"cyberia-server/game/world"
	"cyberia-server/game/world/worldtest"
)

// The generated rules are the release defaults, level for level.
func TestCapacityRulesMatchTheReleaseFormulas(t *testing.T) {
	for level := 1; level <= 120; level++ {
		if want := min(24, 8+(level-1)/3); InventoryCapacity.Slots(level) != want {
			t.Fatalf("inventory at level %d: got %d, want %d", level, InventoryCapacity.Slots(level), want)
		}
		if want := min(40, 16+2*((level-1)/5)); StorageCapacity.Slots(level) != want {
			t.Fatalf("storage at level %d: got %d, want %d", level, StorageCapacity.Slots(level), want)
		}
	}
	if InventoryCapacity.Slots(0) != 8 {
		t.Fatal("a level below 1 counts as level 1")
	}
}

// stacksPlayer places a level-1 player that carries n one-unit stacks and coins.
func stacksPlayer(t *testing.T, n int) (*Content, *world.Session) {
	t.Helper()
	s := &Content{World: &world.World{}, CoinItemID: "coin"}
	layers := []world.ObjectLayerState{{ItemID: "coin", Quantity: 30}}
	for i := 0; i < n; i++ {
		layers = append(layers, world.ObjectLayerState{ItemID: fmt.Sprintf("item-%d", i), Quantity: 1})
	}
	s.SeatStacks(layers, 0)
	player := worldtest.PlacePlayer(s.World, 1, layers)
	player.PlayerID = "p-1"
	player.Coins = 30
	return s, player
}

// A grant of a held item joins its stack in the lowest slot. A new item takes
// the lowest free slot, so the gap a stack left fills first.
func TestGrantsJoinTheFirstStackOrTakeTheLowestFreeSlot(t *testing.T) {
	s, player := stacksPlayer(t, 3)
	layers := s.InventoryLayers(player)
	*layers = append(*layers, world.ObjectLayerState{ItemID: "item-0", Quantity: 4, Slot: 5})

	if !s.ExchangeItems(player, []Stack{{ItemID: "item-1", Qty: 1}}, []Stack{{ItemID: "item-0", Qty: 2}, {ItemID: "new", Qty: 1}}) {
		t.Fatal("the exchange fits")
	}
	at := func(slot int) world.ObjectLayerState { return (*layers)[s.bagStack(*layers, slot)] }
	if at(0).Quantity != 3 || at(5).Quantity != 4 || at(1).ItemID != "new" || at(2).ItemID != "item-2" {
		t.Fatalf("bag = %+v", *layers)
	}
}

// A take drains one stack at a time: the stacks not worn first, in slot
// order, whatever their order in the list.
func TestATakeDrainsStacksInSlotOrder(t *testing.T) {
	layers := []world.ObjectLayerState{
		{ItemID: "gel", Quantity: 2, Slot: 6},
		{ItemID: "gel", Quantity: 1, Slot: 0, Active: true},
		{ItemID: "gel", Quantity: 2, Slot: 3},
	}
	RemoveLayerItem(&layers, "gel", 1)
	if layers[0].Quantity != 2 || layers[2].Quantity != 1 {
		t.Fatalf("layers = %+v, want the unworn stack in slot 3 drained first", layers)
	}
	RemoveLayerItem(&layers, "gel", 3)
	if len(layers) != 1 || !layers[0].Active || layers[0].Slot != 0 {
		t.Fatalf("layers = %+v, want only the worn stack kept", layers)
	}
}

func TestInventoryFitsCountsStacksNotUnits(t *testing.T) {
	s, player := stacksPlayer(t, InventoryCapacity.Slots(1))
	if !s.InventoryFits(player, nil, []Stack{{ItemID: "item-0", Qty: 500}}) {
		t.Fatal("more of a held item always fits")
	}
	if !s.InventoryFits(player, nil, []Stack{{ItemID: "coin", Qty: 500}}) {
		t.Fatal("coins never take a slot")
	}
	if s.InventoryFits(player, nil, []Stack{{ItemID: "new", Qty: 1}}) {
		t.Fatal("a full inventory takes no new stack")
	}
	if !s.InventoryFits(player, []Stack{{ItemID: "item-3", Qty: 1}}, []Stack{{ItemID: "new", Qty: 1}}) {
		t.Fatal("a take that empties a stack frees its slot for the give")
	}
}

// A player over capacity, say from an oversized spawn kit, may trade stacks
// but never add one.
func TestInventoryFitsNeverGrowsPastCapacity(t *testing.T) {
	s, player := stacksPlayer(t, InventoryCapacity.Slots(1)+2)
	if s.InventoryFits(player, nil, []Stack{{ItemID: "new", Qty: 1}}) {
		t.Fatal("an over-capacity inventory takes no new stack")
	}
	if !s.InventoryFits(player, []Stack{{ItemID: "item-0", Qty: 1}}, []Stack{{ItemID: "new", Qty: 1}}) {
		t.Fatal("a swap keeps the stack count and fits")
	}
}

func TestExchangeItemsIsAllOrNothing(t *testing.T) {
	s, player := stacksPlayer(t, InventoryCapacity.Slots(1))
	before := append([]world.ObjectLayerState(nil), s.Ap(player.ID, player.MapCode).Layers...)

	if s.ExchangeItems(player, []Stack{{ItemID: "coin", Qty: 10}}, []Stack{{ItemID: "new", Qty: 1}}) {
		t.Fatal("a give with no room must refuse the whole exchange")
	}
	if s.ExchangeItems(player, []Stack{{ItemID: "item-0", Qty: 2}}, nil) {
		t.Fatal("a take the player lacks must refuse the whole exchange")
	}
	if player.Coins != 30 || fmt.Sprint(before) != fmt.Sprint(s.Ap(player.ID, player.MapCode).Layers) {
		t.Fatal("a refused exchange changes nothing")
	}

	if !s.ExchangeItems(player, []Stack{{ItemID: "coin", Qty: 10}, {ItemID: "item-0", Qty: 1}}, []Stack{{ItemID: "new", Qty: 2}}) {
		t.Fatal("paying with a whole stack makes room for the purchase")
	}
	if player.Coins != 20 || s.playerItemQuantity(player, "new") != 2 || s.playerItemQuantity(player, "item-0") != 0 {
		t.Fatalf("exchange result: coins=%d new=%d", player.Coins, s.playerItemQuantity(player, "new"))
	}
}

// While a player waits to respawn, its inventory is the loadout the respawn
// restores, so a grant in that window survives the respawn.
func TestDeadPlayerGrantsLandInTheRestoredLoadout(t *testing.T) {
	s, player := stacksPlayer(t, 1)
	ms := s.Maps[player.MapCode]
	ms.Mt(player.ID).PreRespawnObjectLayers = append([]world.ObjectLayerState(nil), ms.Ap(player.ID).Layers...)
	ms.Ap(player.ID).Layers = []world.ObjectLayerState{{ItemID: "ghost", Active: true, Quantity: 1}}
	ms.Vt(player.ID).RespawnAt = time.Now().Add(time.Minute)

	if !s.ExchangeItems(player, nil, []Stack{{ItemID: "reward", Qty: 1}}) {
		t.Fatal("a dead player with room takes the grant")
	}
	if LayerQuantity(ms.Mt(player.ID).PreRespawnObjectLayers, "reward") != 1 {
		t.Fatal("the grant must land in the loadout the respawn restores")
	}
	if LayerQuantity(ms.Ap(player.ID).Layers, "reward") != 0 {
		t.Fatal("the dead visuals carry no inventory")
	}
}

// An assembly needs room for its outputs beside its ingredients, and holds
// both until it ends, so its payout and its refund always fit.
func TestCraftHoldsTheSlotsOfItsIngredientsAndOutputs(t *testing.T) {
	s, player := stacksPlayer(t, InventoryCapacity.Slots(1)-1)
	bot := worldtest.PlaceBot(s.World, 2, world.Vitals{})
	player.AOI = world.Rectangle{MinX: -10, MinY: -10, MaxX: 10, MaxY: 10}
	s.actionCache = map[world.EntityID]*CyberiaAction{bot: {CraftRecipes: []ActionCraftRecipe{{
		Ingredients: []Stack{{ItemID: "item-0", Qty: 1}},
		OutputItems: []Stack{{ItemID: "blade", Qty: 1}, {ItemID: "hilt", Qty: 1}},
		CraftTimeMs: 1000,
	}}}}
	craft := &world.InputCommand{EntityID: bot, RecipeIndex: 0}

	s.HandleCraftItem(player, craft)
	if s.crafts[player.ID] != nil || s.playerItemQuantity(player, "item-0") != 1 {
		t.Fatal("two outputs beside the ingredient do not fit one open slot: nothing starts")
	}

	s.removePlayerItem(player, "item-1", 1)
	s.HandleCraftItem(player, craft)
	if s.crafts[player.ID] == nil {
		t.Fatal("with room for both outputs the assembly starts")
	}
	if s.InventoryFits(player, nil, []Stack{{ItemID: "loot", Qty: 1}}) {
		t.Fatal("a running assembly keeps its slots from other grants")
	}

	s.HandleCraftCancel(player)
	if s.crafts[player.ID] != nil || s.playerItemQuantity(player, "item-0") != 1 {
		t.Fatal("a cancel refunds the ingredient into its held slot")
	}
}

func TestShopRefusesAFullInventoryWithoutCharging(t *testing.T) {
	s, player := stacksPlayer(t, InventoryCapacity.Slots(1))
	bot := worldtest.PlaceBot(s.World, 2, world.Vitals{})
	player.AOI = world.Rectangle{MinX: -10, MinY: -10, MaxX: 10, MaxY: 10}
	s.actionCache = map[world.EntityID]*CyberiaAction{bot: {ShopItems: []ActionShopItem{
		{ItemID: "blade", PriceItemID: "coin", PriceQty: 5},
	}}}

	s.HandleShopBuy(player, &world.InputCommand{EntityID: bot, ItemID: "blade", Quantity: 1})
	if player.Coins != 30 || s.playerItemQuantity(player, "blade") != 0 {
		t.Fatalf("a refused purchase charges nothing: coins=%d", player.Coins)
	}
}

// A finished quest waits for room for its rewards, then completes on the
// next tick that finds it.
func TestQuestRewardsWaitForRoom(t *testing.T) {
	s, player := stacksPlayer(t, InventoryCapacity.Slots(1))
	s.questDefs = map[string]*CyberiaQuest{"q": {Code: "q",
		Steps:   []QuestStep{{ID: "s", Objectives: []QuestObjective{{Type: "kill", ItemID: "rat", Quantity: 1}}}},
		Rewards: []QuestReward{{ItemID: "medal", Quantity: 1}, {ItemID: "coin", Quantity: 5}},
	}}
	s.grantQuest(player, "q")

	s.AdvancePlayerQuestsOnKill(player, "rat")
	qp, _ := s.playerQuest(player, "q")
	if qp.Status != "active" || !s.questSnapshot(qp).RewardsHeld || player.Coins != 30 {
		t.Fatalf("rewards that do not fit wait: status=%s coins=%d", qp.Status, player.Coins)
	}

	s.CompleteHeldQuests(player)
	if qp.Status != "active" {
		t.Fatal("no room yet: still held")
	}
	s.removePlayerItem(player, "item-0", 1)
	s.CompleteHeldQuests(player)
	if qp.Status != "completed" || s.playerItemQuantity(player, "medal") != 1 || player.Coins != 35 {
		t.Fatalf("room frees the rewards: status=%s medal=%d coins=%d", qp.Status, s.playerItemQuantity(player, "medal"), player.Coins)
	}
}

// A token stays on its cell for a player with no room; a coin token never
// needs one.
func TestLootNeedsRoomExceptForCoins(t *testing.T) {
	s, player := stacksPlayer(t, InventoryCapacity.Slots(1))
	ms := s.Maps[player.MapCode]
	item := &world.Brain{DropItemID: "gem", DropQuantity: 1, LootOutcome: world.LootOutcomeItem}
	coin := &world.Brain{DropItemID: "coin", DropQuantity: 9, LootOutcome: world.LootOutcomeCoin}
	if s.LootFits(ms, item, player) {
		t.Fatal("a full inventory leaves an item token on the ground")
	}
	if !s.LootFits(ms, coin, player) {
		t.Fatal("coins take no slot")
	}
}
