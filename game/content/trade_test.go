package content

import (
	"testing"
	"time"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
	"cyberia-server/game/world/worldtest"
)

// tradeFixture is the economy fixture with a second player in view: a holds
// med-gel and scrap and 10 coins, b holds a knife and 20 coins.
func tradeFixture(t *testing.T) (*Content, *world.Session, *world.Session, *recorder, *recorder) {
	t.Helper()
	s, a, _, recA := economyFixture(t)
	b := worldtest.PlacePlayer(s.World, 3, []world.ObjectLayerState{{ItemID: "knife", CID: "cid-knife", Quantity: 1, Slot: 0}})
	for s.NewEntityID() < b.ID {
	}
	b.AOI, b.Coins = a.AOI, 20
	a.PlayerID, b.PlayerID = "prof-a", "prof-b"
	recB := &recorder{}
	b.Client = recB
	return s, a, b, recA, recB
}

// openTrade runs a request and its acceptance.
func openTrade(t *testing.T, s *Content, a, b *world.Session) *Trade {
	t.Helper()
	s.HandleTradeRequest(a, &world.InputCommand{EntityID: b.ID})
	s.HandleTradeReply(b, &world.InputCommand{EntityID: a.ID, Active: true})
	trade, _ := s.tradeOf(a)
	if trade == nil || trade.State != TradeOpen {
		t.Fatalf("the trade opens: %+v", trade)
	}
	return trade
}

func offer(s *Content, player *world.Session, coins int, lines ...world.OfferLine) {
	s.HandleTradeOffer(player, &world.InputCommand{Offer: lines, Coins: coins})
}

func confirm(s *Content, player *world.Session, revision uint32) {
	s.HandleTradeConfirm(player, &world.InputCommand{Revision: revision})
}

// Two guests exchange their offers at once when both confirm the same revision.
func TestAGuestTradeExchangesBothOffers(t *testing.T) {
	s, a, b, recA, recB := tradeFixture(t)
	trade := openTrade(t, s, a, b)
	offer(s, a, 4, world.OfferLine{Slot: 0, Qty: 2})
	offer(s, b, 0, world.OfferLine{Slot: 0, Qty: 1})
	confirm(s, a, trade.Revision)
	confirm(s, b, trade.Revision)

	if s.playerItemQuantity(a, "knife") != 1 || stackAt(s, a, 0).Qty != 3 || a.Coins != 6 {
		t.Fatalf("a gave 2 med-gel and 4 coins for the knife: knife=%d gel=%d coins=%d",
			s.playerItemQuantity(a, "knife"), stackAt(s, a, 0).Qty, a.Coins)
	}
	if s.playerItemQuantity(b, "med-gel") != 2 || s.playerItemQuantity(b, "knife") != 0 || b.Coins != 24 {
		t.Fatalf("b gave the knife for 2 med-gel and 4 coins: gel=%d coins=%d", s.playerItemQuantity(b, "med-gel"), b.Coins)
	}
	for _, layer := range *s.InventoryLayers(b) {
		if layer.ItemID == "med-gel" && layer.CID != "cid-med" {
			t.Fatalf("the received stack keeps its definition, got %q", layer.CID)
		}
	}
	if recA.last("trade_ack")["reason"] != tradeDone || recB.last("trade_ack")["reason"] != tradeDone || s.trades[a.ID] != nil {
		t.Fatal("both sides learn the trade is done, and it ends")
	}
}

// An offer change clears both confirmations; a confirmation of an older
// revision is refused and moves nothing.
func TestAnOfferChangeClearsTheConfirmations(t *testing.T) {
	s, a, b, _, recB := tradeFixture(t)
	trade := openTrade(t, s, a, b)
	offer(s, a, 0, world.OfferLine{Slot: 0, Qty: 1})
	seen := trade.Revision
	confirm(s, b, seen)
	offer(s, a, 0, world.OfferLine{Slot: 0, Qty: 5})
	if trade.Sides[1].Confirmed {
		t.Fatal("an offer change clears the confirmation of the other side")
	}
	confirm(s, b, seen)
	confirm(s, a, trade.Revision)
	if recB.last("trade_ack")["reason"] != tradeRevision || stackAt(s, a, 0).Qty != 5 {
		t.Fatal("a stale confirmation is refused and nothing moves")
	}
}

// An item the policy keeps from trading, a missing stack and more coins than
// the wallet holds never enter an offer.
func TestAnOfferTheRulesRefuseIsNotTaken(t *testing.T) {
	s, a, b, recA, _ := tradeFixture(t)
	trade := openTrade(t, s, a, b)
	for name, c := range map[string]struct {
		coins int
		line  world.OfferLine
		want  string
	}{
		"scrap never trades": {0, world.OfferLine{Slot: 1, Qty: 1}, RefusePolicy},
		"an empty slot":      {0, world.OfferLine{Slot: 6, Qty: 1}, RefuseNoStack},
		"more units":         {0, world.OfferLine{Slot: 0, Qty: 6}, RefuseNoStack},
		"more coins":         {11, world.OfferLine{Slot: 0, Qty: 1}, RefuseInsufficient},
	} {
		offer(s, a, c.coins, c.line)
		if ack := recA.last("trade_ack"); ack["reason"] != c.want || len(trade.Sides[0].Lines) != 0 {
			t.Fatalf("%s: want %s, got %v", name, c.want, ack)
		}
	}
}

// While a trade is open no explicit input moves the goods it holds, and a
// guest never trades with an account.
func TestATradeHoldsTheGoodsAndKeepsSessionsApart(t *testing.T) {
	s, a, b, recA, _ := tradeFixture(t)
	openTrade(t, s, a, b)
	s.HandleItemDrop(a, &world.InputCommand{Move: world.ItemMove{From: world.BoxBag, FromSlot: 0, Qty: 1}, TargetX: -1, TargetY: -1})
	if ack := recA.last("item_drop_ack"); ack["reason"] != RefuseTrading || stackAt(s, a, 0).Qty != 5 {
		t.Fatalf("a trading player drops nothing: %v", ack)
	}
	s.HandleTradeCancel(b)
	if s.trades[a.ID] != nil || recA.last("trade_ack")["reason"] != tradeCanceled {
		t.Fatal("a cancel ends the trade for both sides")
	}
	b.AccountID = "acc-b"
	s.HandleTradeRequest(a, &world.InputCommand{EntityID: b.ID})
	if recA.last("trade_ack")["reason"] != tradeMixed || s.trades[a.ID] != nil {
		t.Fatal("a guest never trades with an account")
	}
}

// A commit that finds a bag without room for what it receives moves nothing
// and reopens the trade.
func TestACommitWithoutRoomReopensTheTrade(t *testing.T) {
	s, a, b, recA, _ := tradeFixture(t)
	layers := s.InventoryLayers(b)
	for slot := 1; slot < s.InventorySlots(b); slot++ {
		*layers = append(*layers, world.ObjectLayerState{ItemID: "bolt", CID: "cid-bolt", Quantity: 1, Slot: slot})
	}
	trade := openTrade(t, s, a, b)
	offer(s, a, 0, world.OfferLine{Slot: 0, Qty: 1})
	confirm(s, a, trade.Revision)
	confirm(s, b, trade.Revision)
	if recA.last("trade_ack")["reason"] != RefuseFull || stackAt(s, a, 0).Qty != 5 || trade.Sides[0].Confirmed {
		t.Fatalf("no room: nothing moves and the trade waits for new confirmations: %v", recA.last("trade_ack"))
	}
}

// An account trade moves both offers to escrow and links both post-trade
// states. Applied, each side receives the other's escrow; the link ends after
// the world holds the result.
func TestAnAccountTradeCommitsBothProfilesTogether(t *testing.T) {
	s, a, b, _, _ := tradeFixture(t)
	durability := newFakeDurability()
	s.Durability = durability
	a.AccountID, b.AccountID = "acc-a", "acc-b"
	trade := openTrade(t, s, a, b)
	offer(s, a, 4, world.OfferLine{Slot: 0, Qty: 2})
	offer(s, b, 0, world.OfferLine{Slot: 0, Qty: 1})
	confirm(s, a, trade.Revision)
	confirm(s, b, trade.Revision)

	if trade.State != TradeCommitting || stackAt(s, a, 0).Qty != 3 || a.Coins != 6 || s.playerItemQuantity(b, "knife") != 0 {
		t.Fatal("the offers leave both bags into escrow while the commit runs")
	}
	if s.playerItemQuantity(a, "knife") != 0 || s.playerItemQuantity(b, "med-gel") != 0 || b.Coins != 20 {
		t.Fatal("nothing arrives before the engine applied the trade")
	}
	members := durability.members[trade.ID]
	if len(members) != 2 || members[0].State.Coins != 6 || members[1].State.Coins != 24 || len(members[1].State.Bag) != 1 ||
		members[1].State.Bag[0].ItemID != "med-gel" || members[1].State.Bag[0].CID != "cid-med" {
		t.Fatalf("the link carries both post-trade states: %+v", members)
	}
	if s.incomingCoins(b) != 4 || s.coinRoom(b) != 1000-20-4 {
		t.Fatal("the coins on their way hold room in the wallet")
	}

	s.UpdateTrades(worldtest.Map(s.World), time.Now())
	if trade.State != TradeCommitting {
		t.Fatal("a pending link changes nothing")
	}
	durability.links[trade.ID] = persist.Applied
	s.UpdateTrades(worldtest.Map(s.World), time.Now())
	if s.playerItemQuantity(a, "knife") != 1 || s.playerItemQuantity(b, "med-gel") != 2 || b.Coins != 24 || a.Coins != 6 {
		t.Fatalf("applied: each side holds the other's offer: knife=%d gel=%d", s.playerItemQuantity(a, "knife"), s.playerItemQuantity(b, "med-gel"))
	}
	if len(durability.unlinked) != 1 || s.trades[a.ID] != nil || s.trades[b.ID] != nil {
		t.Fatal("the link ends once, after the world holds the result")
	}
}

// A refused link gives each side its own offer back.
func TestARefusedAccountTradeReturnsBothOffers(t *testing.T) {
	s, a, b, recA, _ := tradeFixture(t)
	durability := newFakeDurability()
	s.Durability = durability
	a.AccountID, b.AccountID = "acc-a", "acc-b"
	trade := openTrade(t, s, a, b)
	offer(s, a, 4, world.OfferLine{Slot: 0, Qty: 2})
	offer(s, b, 0, world.OfferLine{Slot: 0, Qty: 1})
	confirm(s, a, trade.Revision)
	confirm(s, b, trade.Revision)
	durability.links[trade.ID] = persist.Refused
	s.UpdateTrades(worldtest.Map(s.World), time.Now())
	if s.playerItemQuantity(a, "med-gel") != 5 || a.Coins != 10 || s.playerItemQuantity(b, "knife") != 1 || s.playerItemQuantity(a, "knife") != 0 {
		t.Fatal("refused: every stack and coin is back with its owner")
	}
	if recA.last("trade_ack")["reason"] != tradeUnsaved || s.trades[a.ID] != nil {
		t.Fatal("both sides learn the trade did not save")
	}
}

// A side that leaves while its trade commits closes with both outcomes: with
// the escrow of the other side, and with its own back.
func TestALeavingSideGetsAFinalStatePerOutcome(t *testing.T) {
	s, a, b, _, _ := tradeFixture(t)
	durability := newFakeDurability()
	s.Durability = durability
	a.AccountID, b.AccountID = "acc-a", "acc-b"
	trade := openTrade(t, s, a, b)
	offer(s, a, 4, world.OfferLine{Slot: 0, Qty: 2})
	offer(s, b, 0, world.OfferLine{Slot: 0, Qty: 1})
	confirm(s, a, trade.Revision)
	confirm(s, b, trade.Revision)

	applied, refused, linked := s.TradeFinalStates(b)
	if !linked || applied.Coins != 24 || refused.Coins != 20 {
		t.Fatalf("both outcomes carry their coins: applied=%d refused=%d", applied.Coins, refused.Coins)
	}
	holds := func(state persist.State, itemID string, qty int) bool {
		for _, st := range state.Bag {
			if st.ItemID == itemID && st.Quantity == qty {
				return true
			}
		}
		return false
	}
	if !holds(applied, "med-gel", 2) || holds(applied, "knife", 1) || !holds(refused, "knife", 1) {
		t.Fatalf("applied holds a's offer, refused holds b's own: %+v / %+v", applied.Bag, refused.Bag)
	}
	if !trade.Sides[1].Gone || s.trades[b.ID] != nil {
		t.Fatal("the side leaves the trade")
	}
	durability.links[trade.ID] = persist.Applied
	s.UpdateTrades(worldtest.Map(s.World), time.Now())
	if s.playerItemQuantity(a, "knife") != 1 {
		t.Fatal("the side that stays receives the other offer")
	}
}
