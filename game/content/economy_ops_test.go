package content

import (
	"testing"
	"time"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
	"cyberia-server/game/world/worldtest"
	pb "cyberia-server/gen/proto"
)

// recorder is a client link that keeps every message the server sends.
type recorder struct{ msgs []sent }

type sent struct {
	kind    string
	payload map[string]interface{}
	value   any
}

func (r *recorder) Send(kind string, payload any) {
	fields, _ := payload.(map[string]interface{})
	r.msgs = append(r.msgs, sent{kind, fields, payload})
}

// last is the newest message of kind, or nil.
func (r *recorder) last(kind string) map[string]interface{} {
	for i := len(r.msgs) - 1; i >= 0; i-- {
		if r.msgs[i].kind == kind {
			return r.msgs[i].payload
		}
	}
	return nil
}

// economyFixture is a world with a terminal bot that buys back med-gel and
// burns scrap, a player in its view holding both, and the economy:
//
//	med-gel — buyback 3 coins per unit
//	scrap   — burn 1 coin per unit, salvage 2 bolts per unit; no trade, no world drop
//	other   — the defaults: trade and world drop
func economyFixture(t *testing.T) (*Content, *world.Session, world.EntityID, *recorder) {
	t.Helper()
	s := &Content{World: &world.World{}, CoinItemID: "coin"}
	s.ObjectLayerDataCache = map[string]*ObjectLayer{
		"med-gel": {Cid: "cid-med"}, "scrap": {Cid: "cid-scrap"}, "bolt": {Cid: "cid-bolt"}, "knife": {Cid: "cid-knife"},
	}
	s.economy = EconomyRules{
		Code: "standard", Revision: 4, MaxBalance: 1000,
		ItemDefaults: ItemPolicy{Trade: true, WorldDrop: true},
		Items: map[string]ItemPolicy{
			"med-gel": {ItemID: "med-gel", CID: "cid-med", Trade: true, WorldDrop: true, Buyback: true, BuybackPayout: 3},
			"scrap":   {ItemID: "scrap", CID: "cid-scrap", Burn: true, BurnCoinFee: 1, Salvage: []Stack{{ItemID: "bolt", Qty: 2}}},
		},
	}
	player := worldtest.PlacePlayer(s.World, 1, []world.ObjectLayerState{
		{ItemID: "med-gel", CID: "cid-med", Quantity: 5, Slot: 0},
		{ItemID: "scrap", CID: "cid-scrap", Quantity: 4, Slot: 1},
	})
	player.AOI = world.Rectangle{MinX: -20, MinY: -20, MaxX: 20, MaxY: 20}
	player.Coins = 10
	rec := &recorder{}
	player.Client = rec
	bot := worldtest.PlaceBot(s.World, 2, world.Vitals{})
	for s.NewEntityID() < bot {
	}
	s.actionCache = map[world.EntityID]*CyberiaAction{
		bot: {Code: "stall", BuybackItems: []string{"med-gel", "scrap"}, BurnItems: []string{"scrap", "med-gel"}},
	}
	return s, player, bot, rec
}

func stackAt(s *Content, player *world.Session, slot int) Stack {
	stack, _, _ := s.PlacedStack(player, Place{Box: world.BoxBag, Slot: slot})
	return stack
}

// quoteFor asks the price of the bag stack in slot at bot and returns the ack.
func quoteFor(s *Content, player *world.Session, rec *recorder, bot world.EntityID, kind string, slot int) map[string]interface{} {
	s.HandleItemQuote(player, &world.InputCommand{EntityID: bot, QuoteKind: kind, Move: world.ItemMove{From: world.BoxBag, FromSlot: slot}})
	return rec.last("item_quote_ack")
}

// confirmQuote confirms a quote for qty units.
func confirmQuote(quoteID interface{}, qty int) *world.InputCommand {
	id, _ := quoteID.(string)
	return &world.InputCommand{QuoteID: id, Move: world.ItemMove{Qty: qty}}
}

// A buyback pays the quoted payout per unit, once. A listed item the policy
// refuses, an empty slot and a full wallet change nothing.
func TestBuybackPaysTheQuotedPayout(t *testing.T) {
	s, player, bot, rec := economyFixture(t)

	q := quoteFor(s, player, rec, bot, QuoteSell, 0)
	if q["ok"] != true || q["unit"] != 3 || q["maxQuantity"] != 5 {
		t.Fatalf("med-gel is quoted 3 per unit for the 5 held: %v", q)
	}
	s.HandleItemSell(player, confirmQuote(q["quoteId"], 2))
	if ack := rec.last("item_sell_ack"); ack["ok"] != true || player.Coins != 16 || stackAt(s, player, 0).Qty != 3 {
		t.Fatalf("2 med-gel sell for 6 coins: ack=%v coins=%d left=%d", ack, player.Coins, stackAt(s, player, 0).Qty)
	}
	if q := quoteFor(s, player, rec, bot, QuoteSell, 1); q["reason"] != RefusePolicy {
		t.Fatalf("a listed item the policy refuses is not quoted: %v", q)
	}
	gel := s.economy.Items["med-gel"]
	gel.BuybackPayout = 0
	s.economy.Items["med-gel"] = gel
	if q := quoteFor(s, player, rec, bot, QuoteSell, 0); q["reason"] != RefusePolicy {
		t.Fatalf("a buyback that pays nothing is refused: %v", q)
	}
	listed := s.actionCache[bot].BuybackItems
	s.actionCache[bot].BuybackItems = nil
	if q := quoteFor(s, player, rec, bot, QuoteSell, 0); q["reason"] != RefuseNotOffered {
		t.Fatalf("an item the merchant does not list is not offered: %v", q)
	}
	gel.BuybackPayout, s.actionCache[bot].BuybackItems = 3, listed
	s.economy.Items["med-gel"] = gel
	if q := quoteFor(s, player, rec, bot, QuoteSell, 5); q["reason"] != RefuseNoStack {
		t.Fatalf("an empty slot is not quoted: %v", q)
	}

	s.economy.MaxBalance = 18
	q = quoteFor(s, player, rec, bot, QuoteSell, 0)
	if q["maxQuantity"] != 0 {
		t.Fatalf("a wallet with 2 coins of room takes no unit at 3: %v", q)
	}
	s.HandleItemSell(player, confirmQuote(q["quoteId"], 1))
	if ack := rec.last("item_sell_ack"); ack["reason"] != RefuseWalletFull || player.Coins != 16 || stackAt(s, player, 0).Qty != 3 {
		t.Fatalf("a payout past the wallet maximum is refused, never clamped: %v coins=%d", ack, player.Coins)
	}
}

// A quote runs once, while fresh, for the stack, price and economy it named.
func TestAQuoteRunsOnceForWhatItNamed(t *testing.T) {
	s, player, bot, rec := economyFixture(t)

	q := quoteFor(s, player, rec, bot, QuoteSell, 0)
	s.HandleItemSell(player, confirmQuote(q["quoteId"], 1))
	s.HandleItemSell(player, confirmQuote(q["quoteId"], 1))
	if ack := rec.last("item_sell_ack"); ack["reason"] != RefuseQuoteGone || player.Coins != 13 {
		t.Fatalf("a used quote never runs again: %v coins=%d", ack, player.Coins)
	}

	q = quoteFor(s, player, rec, bot, QuoteSell, 0)
	s.economy.Revision++
	s.economy.Items["med-gel"] = ItemPolicy{ItemID: "med-gel", CID: "cid-med", Trade: true, WorldDrop: true, Buyback: true, BuybackPayout: 4}
	s.HandleItemSell(player, confirmQuote(q["quoteId"], 1))
	if ack := rec.last("item_sell_ack"); ack["reason"] != RefuseQuote || ack["unit"] != 4 || player.Coins != 13 {
		t.Fatalf("a changed price refuses the quote and names the new one: %v", ack)
	}

	q = quoteFor(s, player, rec, bot, QuoteSell, 0)
	s.quotes[player.ID].expires = time.Now().Add(-time.Second)
	s.HandleItemSell(player, confirmQuote(q["quoteId"], 1))
	if ack := rec.last("item_sell_ack"); ack["reason"] != RefuseQuoteGone || player.Coins != 13 {
		t.Fatalf("an expired quote never runs: %v", ack)
	}

	q = quoteFor(s, player, rec, bot, QuoteSell, 0)
	(*s.InventoryLayers(player))[0].CID = "cid-med-old"
	s.HandleItemSell(player, confirmQuote(q["quoteId"], 1))
	if ack := rec.last("item_sell_ack"); ack["reason"] != RefuseNoStack || player.Coins != 13 {
		t.Fatalf("a stack that changed after its quote is not the quoted stack: %v", ack)
	}

	q = quoteFor(s, player, rec, bot, QuoteSell, 0)
	s.HandleItemBurn(player, confirmQuote(q["quoteId"], 1))
	if ack := rec.last("item_burn_ack"); ack["reason"] != RefuseQuoteGone {
		t.Fatalf("a sale quote never confirms a burn: %v", ack)
	}
}

// A burn takes the units, charges the quoted fee per unit and grants the
// salvage of each unit in one change. A refused burn changes nothing.
func TestBurnConsumesChargesAndSalvagesAtOnce(t *testing.T) {
	s, player, bot, rec := economyFixture(t)

	q := quoteFor(s, player, rec, bot, QuoteBurn, 1)
	s.HandleItemBurn(player, confirmQuote(q["quoteId"], 2))
	bolts := s.playerItemQuantity(player, "bolt")
	if ack := rec.last("item_burn_ack"); ack["ok"] != true || stackAt(s, player, 1).Qty != 2 || player.Coins != 8 || bolts != 4 {
		t.Fatalf("2 scrap burn for 2 coins into 4 bolts: ack=%v scrap=%d coins=%d bolts=%d",
			ack, stackAt(s, player, 1).Qty, player.Coins, bolts)
	}

	player.Coins = 1
	q = quoteFor(s, player, rec, bot, QuoteBurn, 1)
	if q["maxQuantity"] != 1 {
		t.Fatalf("1 coin pays one burn at 1: %v", q)
	}
	s.HandleItemBurn(player, confirmQuote(q["quoteId"], 2))
	if ack := rec.last("item_burn_ack"); ack["reason"] != RefuseInsufficient || stackAt(s, player, 1).Qty != 2 || s.playerItemQuantity(player, "bolt") != 4 {
		t.Fatalf("a burn the wallet cannot pay changes nothing: %v", ack)
	}
	if q := quoteFor(s, player, rec, bot, QuoteBurn, 0); q["reason"] != RefusePolicy {
		t.Fatalf("a listed item the policy does not let burn is not quoted: %v", q)
	}
}

// A burn whose salvage finds no room changes nothing.
func TestBurnWithoutRoomForTheSalvageChangesNothing(t *testing.T) {
	s, player, bot, rec := economyFixture(t)
	layers := s.InventoryLayers(player)
	for slot := 2; slot < s.InventorySlots(player); slot++ {
		*layers = append(*layers, world.ObjectLayerState{ItemID: "knife", CID: "cid-knife", Quantity: 1, Slot: slot})
	}
	q := quoteFor(s, player, rec, bot, QuoteBurn, 1)
	s.HandleItemBurn(player, confirmQuote(q["quoteId"], 1))
	if ack := rec.last("item_burn_ack"); ack["reason"] != RefuseFull || stackAt(s, player, 1).Qty != 4 || player.Coins != 10 {
		t.Fatalf("no room for the bolts: the scrap and the coins stay, got %v", ack)
	}
}

// A stack whose definition is not the one the world binds, or not the one
// its policy compiled against, takes part in no operation.
func TestAStackOfAnotherDefinitionIsIncompatible(t *testing.T) {
	s, player, bot, rec := economyFixture(t)
	(*s.InventoryLayers(player))[0].CID = "cid-med-old"
	if q := quoteFor(s, player, rec, bot, QuoteSell, 0); q["reason"] != RefuseIncompatible || stackAt(s, player, 0).Qty != 5 {
		t.Fatalf("a rebound stack is never quoted: %v", q)
	}
	(*s.InventoryLayers(player))[0].CID = "cid-med"
	s.ObjectLayerDataCache["med-gel"] = &ObjectLayer{Cid: "cid-med-new"}
	if _, reason := s.ItemPolicyOf("med-gel", "cid-med-new"); reason != RefuseIncompatible {
		t.Fatalf("a policy compiled against another definition does not apply, got %q", reason)
	}
}

// The craft fee follows one rule: the recipe fee, else the surcharge of each
// produced unit, else the economy craft fee, else none. Two ingredients and
// one output pay the output surcharge once, never per ingredient.
func TestCraftFeePrecedence(t *testing.T) {
	s, _, _, _ := economyFixture(t)
	twoIntoOne := &ActionCraftRecipe{
		Ingredients: []Stack{{ItemID: "scrap", Qty: 1}, {ItemID: "med-gel", Qty: 1}},
		OutputItems: []Stack{{ItemID: "knife", Qty: 1}},
	}
	if fee := s.CraftFee(twoIntoOne); fee != 0 {
		t.Fatalf("no rule names a fee: 0, got %d", fee)
	}
	s.economy.Fees.Craft = 5
	if fee := s.CraftFee(twoIntoOne); fee != 5 {
		t.Fatalf("the economy fee applies once per assembly: 5, got %d", fee)
	}
	s.economy.Items["knife"] = ItemPolicy{ItemID: "knife", CID: "cid-knife", CraftSurcharge: 7}
	if fee := s.CraftFee(twoIntoOne); fee != 7 {
		t.Fatalf("the output surcharge replaces the economy fee, once per produced unit: 7, got %d", fee)
	}
	twoIntoOne.OutputItems[0].Qty = 3
	if fee := s.CraftFee(twoIntoOne); fee != 21 {
		t.Fatalf("three produced units pay three surcharges: 21, got %d", fee)
	}
	recipeFee := 2
	twoIntoOne.Fee = &recipeFee
	if fee := s.CraftFee(twoIntoOne); fee != 2 {
		t.Fatalf("the recipe fee overrides every default: 2, got %d", fee)
	}
	zero := 0
	twoIntoOne.Fee = &zero
	if fee := s.CraftFee(twoIntoOne); fee != 0 {
		t.Fatalf("an explicit recipe fee of 0 stays 0, got %d", fee)
	}
}

// An assembly takes its fee with its ingredients, refuses a quote it does not
// charge, and a cancel returns both.
func TestCraftChargesTheFeeOnceAndRefundsItOnCancel(t *testing.T) {
	s, player, bot, rec := economyFixture(t)
	s.economy.Fees.Craft = 4
	s.actionCache[bot].CraftRecipes = []ActionCraftRecipe{{
		Ingredients: []Stack{{ItemID: "scrap", Qty: 1}, {ItemID: "med-gel", Qty: 1}},
		OutputItems: []Stack{{ItemID: "knife", Qty: 1}},
		CraftTimeMs: 60000,
	}}
	s.HandleCraftItem(player, &world.InputCommand{EntityID: bot, RecipeIndex: 0, Quote: 3})
	if ack := rec.last("craft_ack"); ack["reason"] != RefuseQuote || player.Coins != 10 || s.crafts[player.ID] != nil {
		t.Fatalf("a quote the server does not charge refuses the assembly: %v", ack)
	}
	s.HandleCraftItem(player, &world.InputCommand{EntityID: bot, RecipeIndex: 0, Quote: 4})
	if s.crafts[player.ID] == nil || player.Coins != 6 || stackAt(s, player, 0).Qty != 4 || stackAt(s, player, 1).Qty != 3 {
		t.Fatalf("the fee leaves once with the ingredients: coins=%d", player.Coins)
	}
	s.HandleCraftCancel(player)
	if player.Coins != 10 || stackAt(s, player, 0).Qty != 5 || stackAt(s, player, 1).Qty != 4 {
		t.Fatalf("a cancel returns the fee and the ingredients: coins=%d", player.Coins)
	}
	player.Coins = 3
	s.HandleCraftItem(player, &world.InputCommand{EntityID: bot, RecipeIndex: 0, Quote: 4})
	if ack := rec.last("craft_ack"); ack["reason"] != RefuseInsufficient || s.crafts[player.ID] != nil || stackAt(s, player, 0).Qty != 5 {
		t.Fatalf("a fee the wallet cannot pay refuses the assembly: %v", ack)
	}
}

// A coin credit past the wallet maximum is refused and changes nothing.
func TestExchangeRefusesACoinCreditPastTheMaximum(t *testing.T) {
	s, player, _, _ := economyFixture(t)
	s.economy.MaxBalance = 12
	if s.ExchangeItems(player, nil, []Stack{{ItemID: "coin", Qty: 3}}) || player.Coins != 10 {
		t.Fatalf("a reward past the wallet maximum is refused, coins=%d", player.Coins)
	}
	if !s.ExchangeItems(player, nil, []Stack{{ItemID: "coin", Qty: 2}}) || player.Coins != 12 {
		t.Fatalf("a reward up to the maximum is paid, coins=%d", player.Coins)
	}
}

// Rules out of the engine contract never apply.
func TestEconomyRulesRefuseInvalidPolicies(t *testing.T) {
	valid := &pb.EconomyRules{
		BotSpawnCoins: 50, PlayerSpawnCoins: 50, MaxBalance: 1000,
		Fees:         &pb.OperationFees{Portal: 1, Craft: 2},
		ItemDefaults: &pb.ItemPolicy{Trade: true, WorldDrop: true},
		ItemPolicies: []*pb.ItemPolicy{{ItemId: "scrap", OlCid: "cid", Burn: true, Salvage: []*pb.SalvageLine{{ItemId: "bolt", Qty: 2}}}},
	}
	if err := EconomyRulesFromProto(valid).Validate(); err != nil {
		t.Fatalf("a valid snapshot applies: %v", err)
	}
	for name, change := range map[string]func(*pb.EconomyRules){
		"a maximum below the spawn coins": func(r *pb.EconomyRules) { r.MaxBalance = 10 },
		"a row without its definition":    func(r *pb.EconomyRules) { r.ItemPolicies[0].OlCid = "" },
		"a salvage of itself":             func(r *pb.EconomyRules) { r.ItemPolicies[0].Salvage[0].ItemId = "scrap" },
		"a salvage line of 0 units":       func(r *pb.EconomyRules) { r.ItemPolicies[0].Salvage[0].Qty = 0 },
		"a fee past its bound":            func(r *pb.EconomyRules) { r.Fees.Craft = 2000000 },
	} {
		rules := &pb.EconomyRules{
			BotSpawnCoins: valid.BotSpawnCoins, PlayerSpawnCoins: valid.PlayerSpawnCoins, MaxBalance: valid.MaxBalance,
			Fees:         &pb.OperationFees{Portal: 1, Craft: 2},
			ItemDefaults: &pb.ItemPolicy{Trade: true, WorldDrop: true},
			ItemPolicies: []*pb.ItemPolicy{{ItemId: "scrap", OlCid: "cid", Burn: true, Salvage: []*pb.SalvageLine{{ItemId: "bolt", Qty: 2}}}},
		}
		change(rules)
		if err := EconomyRulesFromProto(rules).Validate(); err == nil {
			t.Fatalf("%s must not apply", name)
		}
	}
}

// fakeDurability applies marks and links when the test says so.
type fakeDurability struct {
	captures int
	applied  map[string]persist.Mark
	refused  map[string]bool
	links    map[string]persist.Outcome
	refusals map[string]string
	members  map[string][]persist.LinkMember
	effects  map[string]persist.Effect
	refuse   bool // Link answers false
	unlinked []string
}

func newFakeDurability() *fakeDurability {
	return &fakeDurability{
		applied: map[string]persist.Mark{}, refused: map[string]bool{},
		links: map[string]persist.Outcome{}, refusals: map[string]string{},
		members: map[string][]persist.LinkMember{}, effects: map[string]persist.Effect{},
	}
}

func (f *fakeDurability) Capture(string, persist.State, bool) persist.Mark {
	f.captures++
	return persist.Mark{Epoch: 1, Seq: uint64(f.captures)}
}

func (f *fakeDurability) Outcome(playerID string, mark persist.Mark) persist.Outcome {
	switch {
	case f.refused[playerID]:
		return persist.Refused
	case f.applied[playerID].Seq >= mark.Seq:
		return persist.Applied
	}
	return persist.Pending
}

func (f *fakeDurability) Link(op, _ string, members []persist.LinkMember, effect persist.Effect) bool {
	if f.refuse {
		return false
	}
	f.links[op], f.members[op], f.effects[op] = persist.Pending, members, effect
	return true
}

func (f *fakeDurability) LinkOutcome(op string) (persist.Outcome, string) {
	return f.links[op], f.refusals[op]
}
func (f *fakeDurability) Unlink(op string) { f.unlinked = append(f.unlinked, op) }

// only is the one pending link of the fake.
func (f *fakeDurability) only(t *testing.T) string {
	t.Helper()
	if len(f.links) != 1 {
		t.Fatalf("one link waits: %v", f.links)
	}
	for op := range f.links {
		return op
	}
	return ""
}

// A guest drops a stack of its bag: one public token keeps its item,
// definition and units, and its owner takes it back unchanged.
func TestADroppedStackKeepsItsDefinitionAndComesBack(t *testing.T) {
	s, player, _, rec := economyFixture(t)
	ms := worldtest.Map(s.World)
	s.HandleItemDrop(player, &world.InputCommand{Move: world.ItemMove{From: world.BoxBag, FromSlot: 0, Qty: 2}, TargetX: -1, TargetY: -1})
	ack := rec.last("item_drop_ack")
	if ack["ok"] != true || ack["status"] != DropPublic || stackAt(s, player, 0).Qty != 3 {
		t.Fatalf("a guest drop is public at once: %v left=%d", ack, stackAt(s, player, 0).Qty)
	}
	token := ack["dropId"].(world.EntityID)
	drop := ms.Br(token)
	if drop.DropItemID != "med-gel" || drop.DropCID != "cid-med" || drop.DropQuantity != 2 || drop.DropOwner != player.PlayerID ||
		drop.LootCollection != world.LootCollectPlayers || drop.ExpiresAt.IsZero() {
		t.Fatalf("the token keeps the stack and its owner: %+v", drop)
	}
	if !LootAdmits(drop, player.ID) || s.ResolveBotDropCollector(ms, token) != 0 {
		t.Fatal("any player takes a player drop, its owner too, and no bot")
	}
	if !s.ApplyLootToPlayer(ms, drop, player) || stackAt(s, player, 0).Qty != 5 {
		t.Fatalf("the owner takes its drop back onto its stack, got %d", stackAt(s, player, 0).Qty)
	}
}

// A drop from the personal storage leaves the storage.
func TestADropFromStorageLeavesTheStorage(t *testing.T) {
	s, player, _, rec := economyFixture(t)
	s.PersonalStorage = map[world.PlayerID][]Stack{player.PlayerID: {{ItemID: "knife", CID: "cid-knife", Qty: 1, Slot: 3}}}
	s.HandleItemDrop(player, &world.InputCommand{Move: world.ItemMove{From: world.BoxStorage, FromSlot: 3, Qty: 1}, TargetX: -1, TargetY: -1})
	if ack := rec.last("item_drop_ack"); ack["ok"] != true || len(s.PlayerStorage(player)) != 0 {
		t.Fatalf("the stored knife drops: %v storage=%v", ack, s.PlayerStorage(player))
	}
}

// A policy without world drops, a cell out of reach and a dead player keep
// the stack where it is.
func TestADropTheRulesRefuseKeepsTheStack(t *testing.T) {
	s, player, _, rec := economyFixture(t)
	s.HandleItemDrop(player, &world.InputCommand{Move: world.ItemMove{From: world.BoxBag, FromSlot: 1, Qty: 1}, TargetX: -1, TargetY: -1})
	if ack := rec.last("item_drop_ack"); ack["reason"] != RefusePolicy || stackAt(s, player, 1).Qty != 4 {
		t.Fatalf("scrap never drops: %v", ack)
	}
	s.HandleItemDrop(player, &world.InputCommand{Move: world.ItemMove{From: world.BoxBag, FromSlot: 0, Qty: 1}, TargetX: 15, TargetY: 15})
	if ack := rec.last("item_drop_ack"); ack["reason"] != RefuseOutOfRange || stackAt(s, player, 0).Qty != 5 {
		t.Fatalf("a cell out of reach refuses the drop: %v", ack)
	}
}

// A token of a stack whose label names another definition now is taken as a
// held stack: its definition never changes.
func TestATokenOfAnotherDefinitionIsHeld(t *testing.T) {
	s, player, _, _ := economyFixture(t)
	ms := worldtest.Map(s.World)
	drop := &world.Brain{DropItemID: "knife", DropCID: "cid-knife-old", DropQuantity: 1, LootCollection: world.LootCollectPlayers}
	if !s.LootFits(ms, drop, player) || !s.ApplyLootToPlayer(ms, drop, player) {
		t.Fatal("the stack is taken")
	}
	held := s.HeldView(player)
	if len(held) != 1 || held[0].CID != "cid-knife-old" || held[0].Reason != persist.HeldRebound || s.playerItemQuantity(player, "knife") != 0 {
		t.Fatalf("it is held with its own definition: %+v", held)
	}
}
