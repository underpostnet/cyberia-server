package game

import (
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

// The recipe book crosses a process boundary (engine-cyberia → gRPC → here) and
// is addressed by index, so the decode and the selection/affordance checks the
// synthesis handler depends on are pinned here.

func TestProtoToActionDecodesCraftRecipes(t *testing.T) {
	msg := &pb.CyberiaActionMessage{
		Code:  "loc-fallback-map-0-15-16",
		Label: "Eiri",
		CraftRecipes: []*pb.ActionCraftRecipe{{
			OutputItems: []*pb.ActionCraftItem{{ItemId: "hatchet", Qty: 1}},
			Ingredients: []*pb.ActionCraftItem{
				{ItemId: "wood-drop-1", Qty: 2},
				{ItemId: "coin", Qty: 5},
			},
		}},
	}

	action := protoToAction(msg)
	if len(action.CraftRecipes) != 1 {
		t.Fatalf("want 1 recipe, got %d", len(action.CraftRecipes))
	}
	r := action.CraftRecipes[0]
	if len(r.OutputItems) != 1 || r.OutputItems[0].ItemID != "hatchet" || r.OutputItems[0].Qty != 1 {
		t.Fatalf("outputs did not survive the wire: %+v", r.OutputItems)
	}
	if len(r.Ingredients) != 2 || r.Ingredients[1].ItemID != "coin" || r.Ingredients[1].Qty != 5 {
		t.Fatalf("ingredients did not survive the wire: %+v", r.Ingredients)
	}
}

// The client names a recipe by index, so an out-of-range or negative index must
// resolve to nothing rather than the wrong schematic.
func TestCraftRecipeAtBoundsTheIndex(t *testing.T) {
	action := &CyberiaAction{CraftRecipes: []ActionCraftRecipe{
		{OutputItems: []ActionCraftItem{{ItemID: "hatchet", Qty: 1}}},
		{OutputItems: []ActionCraftItem{{ItemID: "tim-knife", Qty: 1}}},
	}}

	if r := craftRecipeAt(action, 1); r == nil || r.OutputItems[0].ItemID != "tim-knife" {
		t.Fatalf("want the second recipe, got %+v", r)
	}
	for _, index := range []int{-1, 2, 99} {
		if r := craftRecipeAt(action, index); r != nil {
			t.Fatalf("index %d must resolve to nothing, got %+v", index, r)
		}
	}
	if r := craftRecipeAt(nil, 0); r != nil {
		t.Fatal("an entity with no bound action has no recipes")
	}
}

func TestPlayerCanCraftChecksEveryIngredient(t *testing.T) {
	s := &GameServer{coinItemID: "coin"}
	player := &PlayerState{
		EntityBase: EntityBase{ID: "p1", ObjectLayers: []ObjectLayerState{
			{ItemID: "wood-drop-1", Quantity: 2},
		}},
		Coins: 5,
	}
	recipe := &ActionCraftRecipe{Ingredients: []ActionCraftItem{
		{ItemID: "wood-drop-1", Qty: 2},
		{ItemID: "coin", Qty: 5},
	}}

	if !s.playerCanCraft(player, recipe) {
		t.Fatal("holding exactly the required amounts must be craftable")
	}
	// One short of any single line blocks the whole synthesis.
	player.Coins = 4
	if s.playerCanCraft(player, recipe) {
		t.Fatal("a short currency line must block the synthesis")
	}
	player.Coins = 5
	player.ObjectLayers[0].Quantity = 1
	if s.playerCanCraft(player, recipe) {
		t.Fatal("a short component line must block the synthesis")
	}
	// A free line is always satisfied, so a recipe of them always runs.
	if !s.playerCanCraft(player, &ActionCraftRecipe{
		Ingredients: []ActionCraftItem{{ItemID: "wood-drop-9", Qty: 0}},
	}) {
		t.Fatal("a zero-quantity line costs nothing")
	}
}

// botHasCraft feeds botHasUsableAction, so an assembler advertises itself
// overhead exactly like a vendor does.
func TestBotHasCraftGatesTheCapabilityBit(t *testing.T) {
	book := &CyberiaAction{CraftRecipes: []ActionCraftRecipe{
		{OutputItems: []ActionCraftItem{{ItemID: "hatchet", Qty: 1}}},
	}}
	s := &GameServer{actionCache: map[string]*CyberiaAction{
		"assembler": book,
		"talker":    {DialogCode: "default-lain"},
		"dead":      book,
	}}

	assembler := &BotState{EntityBase: EntityBase{ID: "assembler"}}
	talker := &BotState{EntityBase: EntityBase{ID: "talker"}}
	dead := &BotState{
		EntityBase: EntityBase{ID: "dead"},
		Mortal:     Mortal{RespawnTime: time.Now().Add(time.Second)},
	}

	if !s.botHasCraft(assembler) || !s.botHasUsableAction(assembler) {
		t.Fatal("a live bot with a recipe book is an assembler")
	}
	if s.botHasCraft(talker) || s.botHasUsableAction(talker) {
		t.Fatal("an action with no recipes is not an assembler")
	}
	if s.botHasCraft(dead) {
		t.Fatal("a dead assembler must not advertise")
	}
	if flags := s.botInteractionFlags(false, false, true); flags&InteractionFlagAction == 0 {
		t.Fatalf("a usable action must light the action bit, got 0x%x", flags)
	}
}

// An authored duration is content, so it is validated rather than trusted: a
// negative one resolves instantly and an absurd one is capped so a player can
// never be stranded in the assembly modal.
func TestCraftTimeMsValidatesTheAuthoredDuration(t *testing.T) {
	cases := []struct{ authored, want int }{
		{0, 0}, {-1, 0}, {3000, 3000},
		{craftMaxTimeMs, craftMaxTimeMs}, {craftMaxTimeMs + 1, craftMaxTimeMs},
	}
	for _, c := range cases {
		if got := craftTimeMs(&ActionCraftRecipe{CraftTimeMs: c.authored}); got != c.want {
			t.Fatalf("craftTimeMs(%d) = %d, want %d", c.authored, got, c.want)
		}
	}
}

// The ingredients leave the inventory when the bar starts, so cancelling before
// the timer elapses has to put every one of them back and pay out nothing.
func TestCraftCancelRefundsAndCompletionPaysOut(t *testing.T) {
	s := &GameServer{coinItemID: "coin", statsCache: map[string]statsCacheEntry{}}
	newPlayer := func() *PlayerState {
		return &PlayerState{
			EntityBase: EntityBase{ID: "p1", ObjectLayers: []ObjectLayerState{
				{ItemID: "wood-drop-1", Quantity: 2},
			}},
			Coins: 5,
		}
	}
	recipe := &ActionCraftRecipe{
		OutputItems: []ActionCraftItem{{ItemID: "hatchet", Qty: 1}},
		Ingredients: []ActionCraftItem{
			{ItemID: "wood-drop-1", Qty: 2},
			{ItemID: "coin", Qty: 5},
		},
	}
	// Stand in for the consume phase handleCraftItem runs before arming a timer.
	start := func(p *PlayerState, at time.Time) {
		for _, in := range recipe.Ingredients {
			s.removePlayerItem(p, in.ItemID, in.Qty)
		}
		p.ActiveCraft = &ActiveCraft{
			Refund:     recipe.Ingredients,
			Outputs:    recipe.OutputItems,
			CompleteAt: at,
		}
	}

	cancelled := newPlayer()
	start(cancelled, time.Now().Add(time.Hour))
	if s.playerItemQuantity(cancelled, "wood-drop-1") != 0 || cancelled.Coins != 0 {
		t.Fatal("starting an assembly must consume its ingredients up front")
	}
	s.handleCraftCancel(cancelled)
	if cancelled.ActiveCraft != nil {
		t.Fatal("cancel must clear the running assembly")
	}
	if s.playerItemQuantity(cancelled, "wood-drop-1") != 2 || cancelled.Coins != 5 {
		t.Fatalf("cancel must refund every ingredient, got wood=%d coins=%d",
			s.playerItemQuantity(cancelled, "wood-drop-1"), cancelled.Coins)
	}
	if s.playerItemQuantity(cancelled, "hatchet") != 0 {
		t.Fatal("a cancelled assembly must not pay out")
	}
	// Cancelling again is a no-op rather than a second refund.
	s.handleCraftCancel(cancelled)
	if s.playerItemQuantity(cancelled, "wood-drop-1") != 2 {
		t.Fatal("cancelling twice must not refund twice")
	}

	// A timer still running is left alone; an elapsed one pays out exactly once.
	pending := newPlayer()
	start(pending, time.Now().Add(time.Hour))
	elapsed := newPlayer()
	start(elapsed, time.Now().Add(-time.Millisecond))
	mapState := &MapState{players: map[string]*PlayerState{"pending": pending, "elapsed": elapsed}}

	s.completeCrafts(mapState)
	if pending.ActiveCraft == nil || s.playerItemQuantity(pending, "hatchet") != 0 {
		t.Fatal("an assembly whose timer has not elapsed must keep running")
	}
	if elapsed.ActiveCraft != nil || s.playerItemQuantity(elapsed, "hatchet") != 1 {
		t.Fatalf("an elapsed assembly must pay out once, got %d",
			s.playerItemQuantity(elapsed, "hatchet"))
	}
	s.completeCrafts(mapState)
	if s.playerItemQuantity(elapsed, "hatchet") != 1 {
		t.Fatal("a completed assembly must not pay out again")
	}
}

// The interact modal is still open when an assembly resolves, so neither
// finishing nor cancelling one may leave the player killable.
func TestCraftKeepsTheSessionFreeze(t *testing.T) {
	s := &GameServer{
		coinItemID: "coin",
		statsCache: map[string]statsCacheEntry{},
		actionCache: map[string]*CyberiaAction{
			"assembler": {Code: "loc-0-0", CraftRecipes: []ActionCraftRecipe{{
				OutputItems: []ActionCraftItem{{ItemID: "hatchet", Qty: 1}},
			}}},
		},
	}
	armed := func() *PlayerState {
		p := &PlayerState{EntityBase: EntityBase{ID: "p1"}}
		p.ActiveCraft = &ActiveCraft{
			EntityID:   "assembler",
			Refund:     []ActionCraftItem{{ItemID: "coin", Qty: 5}},
			Outputs:    []ActionCraftItem{{ItemID: "hatchet", Qty: 1}},
			CompleteAt: time.Now().Add(-time.Millisecond),
		}
		return p
	}

	completed := armed()
	s.completeCrafts(&MapState{players: map[string]*PlayerState{"p": completed}})
	if !completed.Frozen || completed.FreezeReason != "interact" {
		t.Fatalf("completing an assembly must hold the session freeze, got frozen=%v reason=%q",
			completed.Frozen, completed.FreezeReason)
	}

	cancelled := armed()
	s.handleCraftCancel(cancelled)
	if !cancelled.Frozen || cancelled.FreezeReason != "interact" {
		t.Fatalf("cancelling an assembly must hold the session freeze, got frozen=%v reason=%q",
			cancelled.Frozen, cancelled.FreezeReason)
	}
	// Only the modal's own freeze_end releases it.
	ThawPlayer(cancelled, "interact")
	if cancelled.Frozen {
		t.Fatal("freeze_end \"interact\" must release the player")
	}
}
