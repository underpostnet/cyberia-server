package content

import (
	"testing"
	"time"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
)

// fakeMarkets records the views asked for.
type fakeMarkets struct{ queries []MarketQuery }

func (f *fakeMarkets) ViewMarket(q MarketQuery) { f.queries = append(f.queries, q) }

// marketFixture is the account fixture with a terminal of the market profile
// "bazaar" (fixed price and auction, listing fee 2, sale fee 1, prices 1–1000,
// 1, 12 or 48 hours) that lists med-gel and scrap in both modes, and a market
// policy on med-gel.
func marketFixture(t *testing.T) (*Content, *world.Session, world.EntityID, *recorder, *fakeDurability, *fakeMarkets) {
	t.Helper()
	s, player, bot, rec := economyFixture(t)
	durability, markets := newFakeDurability(), &fakeMarkets{}
	s.Durability, s.Markets = durability, markets
	player.PlayerID, player.AccountID, player.DisplayName = "prof-1", "acc-1", "neo"
	gel := s.economy.Items["med-gel"]
	gel.Market = true
	s.economy.Items["med-gel"] = gel
	s.BindMarkets([]*pb.MarketProfile{{
		Code: "bazaar", Revision: 3, Status: "active", Scope: "universe", ScopeKey: "u1", Ref: "dd/universe:u1/bazaar",
		CurrencyCode: "coin", Modes: []string{"fixed", "auction"}, Hours: []uint32{1, 12, 48}, RaisePercent: 5, MinRaise: 1,
		ListingFee: 2, SaleFee: 1, PriceMin: 1, PriceMax: 1000, ListingsPerPlayer: 20,
	}})
	s.actionCache[bot].Market = ActionMarket{
		MarketCode: "bazaar", Selection: "items", ItemIDs: []string{"med-gel", "scrap"}, ListingModes: []string{"fixed", "auction"},
	}
	return s, player, bot, rec, durability, markets
}

// market changes the profile of the fixture.
func market(s *Content, change func(*MarketProfile)) {
	profile := s.marketProfiles["bazaar"]
	change(&profile)
	s.marketProfiles["bazaar"] = profile
}

func listCmd(bot world.EntityID, slot, qty int, mode string, price int) *world.InputCommand {
	return &world.InputCommand{EntityID: bot, Move: world.ItemMove{From: world.BoxBag, FromSlot: slot, Qty: qty},
		Market: world.MarketInput{Mode: mode, Price: price, Hours: 12}}
}

// A listing takes the stack and the fee at once and commits the listing with
// the state; a refusal returns both.
func TestAListingEscrowsTheStackAndTheFee(t *testing.T) {
	s, player, bot, rec, durability, markets := marketFixture(t)
	s.HandleMarketList(player, listCmd(bot, 0, 3, "fixed", 40))
	op := durability.only(t)
	m := durability.effects[op].Market
	if m == nil || m.MarketCode != "bazaar" || m.Mode != "fixed" || m.ItemID != "med-gel" || m.CID != "cid-med" ||
		m.Quantity != 3 || m.Price != 40 || m.Hours != 12 || m.ListingFee != 2 || m.SaleFee != 1 || m.SellerName != "neo" {
		t.Fatalf("the listing carries its stack and terms: %+v", m)
	}
	if stackAt(s, player, 0).Qty != 2 || player.Coins != 8 {
		t.Fatalf("the stack and the fee leave at once: qty=%d coins=%d", stackAt(s, player, 0).Qty, player.Coins)
	}
	durability.links[op], durability.refusals[op] = persist.Refused, "market_full"
	s.SettleLinkedOps(time.Now())
	if stackAt(s, player, 0).Qty != 5 || player.Coins != 10 {
		t.Fatalf("a refused listing returns the stack and the fee: qty=%d coins=%d", stackAt(s, player, 0).Qty, player.Coins)
	}
	if ack := rec.last("market_ack"); ack["reason"] != "market_full" || ack["action"] != opMarketList {
		t.Fatalf("the seller learns why: %v", ack)
	}
	if len(markets.queries) != 0 {
		t.Fatal("no view was seen, so none refreshes")
	}
}

// A listing needs the terminal to offer the item and the mode, the market to
// take it at that duration and price, the item policy to allow it, and an
// account player. Each refusal names its cause.
func TestAListingNeedsTheTerminalTheMarketThePolicyAndAnAccount(t *testing.T) {
	for name, c := range map[string]struct {
		slot   int
		mode   string
		price  int
		change func(*Content, *world.Session, world.EntityID)
		reason string
	}{
		"an item whose policy allows no market": {1, "fixed", 5, nil, RefusePolicy},
		"a mode the terminal does not offer": {0, "auction", 5, func(s *Content, _ *world.Session, b world.EntityID) {
			s.actionCache[b].Market.ListingModes = []string{"fixed"}
		}, RefuseNotOffered},
		"a mode the market does not support": {0, "auction", 5, func(s *Content, _ *world.Session, _ world.EntityID) {
			market(s, func(m *MarketProfile) { m.Modes = []string{"fixed"} })
		}, RefuseNotOffered},
		"an item the terminal does not list": {0, "fixed", 5, func(s *Content, _ *world.Session, b world.EntityID) {
			s.actionCache[b].Market.ItemIDs = []string{"scrap"}
		}, RefuseNotOffered},
		"a price over the range of the market": {0, "fixed", 1001, nil, RefuseTerms},
		"a duration the market does not offer": {0, "fixed", 5, func(s *Content, _ *world.Session, _ world.EntityID) {
			market(s, func(m *MarketProfile) { m.Hours = []int{1, 48} })
		}, RefuseTerms},
		"a retired market": {0, "fixed", 5, func(s *Content, _ *world.Session, _ world.EntityID) {
			market(s, func(m *MarketProfile) { m.Status = "retired" })
		}, RefuseMarketClosed},
		"a market that does not fit the world": {0, "fixed", 5, func(s *Content, _ *world.Session, _ world.EntityID) {
			market(s, func(m *MarketProfile) { m.Defect = "the world belongs to no universe" })
		}, RefuseMarketClosed},
		"a guest": {0, "fixed", 5, func(_ *Content, p *world.Session, _ world.EntityID) { p.AccountID = "" }, RefuseGuest},
	} {
		s, player, bot, rec, durability, _ := marketFixture(t)
		if c.change != nil {
			c.change(s, player, bot)
		}
		s.HandleMarketList(player, listCmd(bot, c.slot, 1, c.mode, c.price))
		if ack := rec.last("market_ack"); ack["reason"] != c.reason || len(durability.links) != 0 || stackAt(s, player, 0).Qty != 5 {
			t.Fatalf("%s: %v", name, ack)
		}
	}
}

// A terminal of the selection "policy" lists every item whose policy allows a
// listing, with no item list of its own.
func TestAPolicyTerminalListsWhatThePolicyAllows(t *testing.T) {
	s, player, bot, _, durability, _ := marketFixture(t)
	s.actionCache[bot].Market = ActionMarket{MarketCode: "bazaar", Selection: "policy", ListingModes: []string{"fixed", "auction"}}
	s.HandleMarketList(player, listCmd(bot, 0, 1, "auction", 10))
	if m := durability.effects[durability.only(t)].Market; m == nil || m.ItemID != "med-gel" || m.Mode != "auction" {
		t.Fatalf("the policy terminal lists med-gel: %+v", m)
	}
}

// showListing makes the player see one listing at the terminal.
func showListing(s *Content, player *world.Session, bot world.EntityID, listing MarketListing) {
	s.ShowMarket(MarketQuery{Player: player.ID, Profile: player.PlayerID, Terminal: bot, Code: "bazaar"},
		MarketView{Listings: []MarketListing{listing}, Claims: []MarketClaim{{ClaimID: "l1:sale", Reason: "sale", Coins: 6},
			{ClaimID: "l2:won", Reason: "won", ItemID: "bolt", CID: "cid-bolt", Quantity: 4}}}, time.Now())
}

// A buy takes the price at once; the stack arrives once the engine applied it,
// and the price returns when it refused.
func TestABuyPaysThePriceAndReceivesTheStack(t *testing.T) {
	s, player, bot, rec, durability, markets := marketFixture(t)
	showListing(s, player, bot, MarketListing{ListingID: "l9", Revision: 2, ItemID: "knife", CID: "cid-knife", Quantity: 1,
		Mode: "fixed", Price: 7, EndsAt: time.Now().Add(time.Hour)})
	if last := rec.msgs[len(rec.msgs)-1]; last.kind != "market" {
		t.Fatalf("the view reaches the player: %s", last.kind)
	}
	cmd := &world.InputCommand{EntityID: bot, Market: world.MarketInput{ListingID: "l9"}}
	s.HandleMarketBuy(player, cmd)
	op := durability.only(t)
	if m := durability.effects[op].Market; m.ListingID != "l9" || m.Revision != 2 || m.Price != 7 || m.ItemID != "knife" {
		t.Fatalf("the buy names what the player saw: %+v", m)
	}
	if player.Coins != 3 || s.playerItemQuantity(player, "knife") != 0 {
		t.Fatal("the price leaves at once, the stack waits")
	}
	durability.links[op] = persist.Applied
	s.SettleLinkedOps(time.Now())
	if player.Coins != 3 || s.playerItemQuantity(player, "knife") != 1 || rec.last("market_ack")["ok"] != true {
		t.Fatalf("the applied buy gives the stack: coins=%d", player.Coins)
	}
	if len(markets.queries) != 1 || markets.queries[0].Terminal != bot {
		t.Fatal("the player sees the market again")
	}

	s.HandleMarketBuy(player, cmd)
	op = durability.only(t)
	durability.links[op], durability.refusals[op] = persist.Refused, RefuseGone
	s.SettleLinkedOps(time.Now())
	if player.Coins != 3 {
		t.Fatalf("a refused buy returns the price: coins=%d", player.Coins)
	}
}

// A terminal that lists at fixed prices only shows the auctions of its shared
// market and takes bids on them: its modes restrict what is listed there.
func TestAFixedOnlyTerminalShowsAndBidsOnSharedAuctions(t *testing.T) {
	s, player, bot, rec, durability, _ := marketFixture(t)
	s.actionCache[bot].Market.ListingModes = []string{"fixed"}
	showListing(s, player, bot, MarketListing{ListingID: "a1", ItemID: "knife", CID: "cid-knife", Quantity: 1,
		Mode: "auction", Price: 5, MinBid: 5, EndsAt: time.Now().Add(time.Hour)})
	view, _ := rec.msgs[len(rec.msgs)-1].value.(marketPayload)
	if len(view.Listings) != 1 || view.Listings[0].Mode != "auction" || len(view.ListingModes) != 1 ||
		view.ListingModes[0] != "fixed" || view.Ref != "dd/universe:u1/bazaar" || view.ListingFee != 2 {
		t.Fatalf("the view shows the shared auction and the rules of this terminal: %+v", view)
	}
	s.HandleMarketBid(player, &world.InputCommand{EntityID: bot, Market: world.MarketInput{ListingID: "a1", Price: 5}})
	if m := durability.effects[durability.only(t)].Market; m.ListingID != "a1" || m.Price != 5 {
		t.Fatalf("the bid goes out: %+v", m)
	}
}

// A bid under the lowest one the auction takes never leaves the wallet.
func TestABidUnderTheMinimumIsRefused(t *testing.T) {
	s, player, bot, rec, durability, _ := marketFixture(t)
	showListing(s, player, bot, MarketListing{ListingID: "a1", ItemID: "knife", CID: "cid-knife", Quantity: 1,
		Mode: "auction", Price: 5, MinBid: 6, EndsAt: time.Now().Add(time.Hour)})
	s.HandleMarketBid(player, &world.InputCommand{EntityID: bot, Market: world.MarketInput{ListingID: "a1", Price: 5}})
	if ack := rec.last("market_ack"); ack["reason"] != RefuseBidLow || len(durability.links) != 0 {
		t.Fatalf("a low bid is refused here: %v", ack)
	}
	s.HandleMarketBid(player, &world.InputCommand{EntityID: bot, Market: world.MarketInput{ListingID: "a1", Price: 6}})
	if m := durability.effects[durability.only(t)].Market; m.Price != 6 || player.Coins != 4 {
		t.Fatalf("the bid leaves the wallet: %+v coins=%d", m, player.Coins)
	}
}

// A collect takes every claim of the view, and a player that leaves while it
// waits closes with the claims only when it applies.
func TestACollectTakesTheClaimsOfTheView(t *testing.T) {
	s, player, bot, _, durability, _ := marketFixture(t)
	showListing(s, player, bot, MarketListing{ListingID: "x", EndsAt: time.Now()})
	s.HandleMarketCollect(player, &world.InputCommand{EntityID: bot})
	op := durability.only(t)
	m := durability.effects[op].Market
	if len(m.ClaimIDs) != 2 || m.Coins != 6 || len(m.Stacks) != 1 || m.Stacks[0].ItemID != "bolt" {
		t.Fatalf("the collect names the claims and their goods: %+v", m)
	}
	applied, refused := s.FinalStates(player)
	if applied.Coins != 16 || refused.Coins != 10 {
		t.Fatalf("the final states follow the outcome: %d %d", applied.Coins, refused.Coins)
	}
	durability.links[op] = persist.Applied
	s.SettleLinkedOps(time.Now())
	if player.Coins != 16 || s.playerItemQuantity(player, "bolt") != 4 {
		t.Fatalf("the claims arrive: coins=%d", player.Coins)
	}
}
