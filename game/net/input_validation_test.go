package net

import (
	"strings"
	"testing"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

func TestParseInputCarriesTheStasisBool(t *testing.T) {
	for _, want := range []bool{true, false} {
		cmd, ok := parseInput("player_stasis", &inputPayload{Stasis: want})
		if !ok || cmd.Kind != world.InputKindPlayerStasis || cmd.Active != want {
			t.Fatalf("player_stasis %v: got ok=%v %+v", want, ok, cmd)
		}
	}
}

// A chat line names a known channel and carries clean text; only a direct
// line names a target.
func TestParseInputValidatesChat(t *testing.T) {
	cmd, ok := parseInput("chat", &inputPayload{Channel: "party", Text: " hi\tall "})
	if !ok || cmd.ChatChannel != world.ChatParty || cmd.Text != "hi all" || cmd.EntityID != 0 {
		t.Fatalf("party line: ok=%v %+v", ok, cmd)
	}
	if cmd, ok := parseInput("chat", &inputPayload{Channel: "direct", ToID: 4, Text: "psst"}); !ok || cmd.EntityID != 4 {
		t.Fatalf("direct line: ok=%v %+v", ok, cmd)
	}
	for name, p := range map[string]*inputPayload{
		"unknown channel":   {Channel: "guild", Text: "hi"},
		"no channel":        {Text: "hi"},
		"blank text":        {Channel: "world", Text: " \n "},
		"direct, no target": {Channel: "direct", Text: "hi"},
		"world, a target":   {Channel: "world", ToID: 4, Text: "hi"},
	} {
		if _, ok := parseInput("chat", p); ok {
			t.Fatalf("%s: must be rejected", name)
		}
	}
}

// The engine validates status notes without truncation.
func TestParseInputPreservesTheStatusNote(t *testing.T) {
	cmd, ok := parseInput("status_note", &inputPayload{Text: " out\n  farming "})
	if !ok || cmd.Kind != world.InputKindStatusNote || cmd.Text != " out\n  farming " {
		t.Fatalf("note: ok=%v %+v", ok, cmd)
	}
	cmd, _ = parseInput("status_note", &inputPayload{Text: strings.Repeat("x", content.StatusNoteMaxRunes+5)})
	if len(cmd.Text) != content.StatusNoteMaxRunes+5 {
		t.Fatalf("a long note reaches engine validation: bound=%d length=%d", content.StatusNoteMaxRunes, len(cmd.Text))
	}
	if cmd, ok := parseInput("status_note", &inputPayload{}); !ok || cmd.Text != "" {
		t.Fatalf("an empty note clears: ok=%v %+v", ok, cmd)
	}
	for _, text := range []string{"🌲", "e\u0301", "👩‍👩‍👧‍👦", strings.Repeat("🌲", 81)} {
		if got, ok := parseInput("status_note", &inputPayload{Text: text}); !ok || got.Text != text {
			t.Fatalf("note changed before engine validation: %q -> %q", text, got.Text)
		}
	}

}

func TestParseInputNamesThePartyTarget(t *testing.T) {
	if cmd, ok := parseInput("party_invite", &inputPayload{ToID: 9}); !ok || cmd.EntityID != 9 {
		t.Fatalf("invite: ok=%v %+v", ok, cmd)
	}
	if cmd, ok := parseInput("party_reply", &inputPayload{FromID: 9, Accept: true}); !ok || cmd.EntityID != 9 || !cmd.Active {
		t.Fatalf("reply: ok=%v %+v", ok, cmd)
	}
	for _, kind := range []string{"party_invite", "party_kick", "party_reply"} {
		if _, ok := parseInput(kind, &inputPayload{}); ok {
			t.Fatalf("%s without a player must be rejected", kind)
		}
	}
	for _, kind := range []string{"party_leave", "party_disband"} {
		if _, ok := parseInput(kind, &inputPayload{}); !ok {
			t.Fatalf("%s needs no field", kind)
		}
	}
}

// A drop and a quote name a bag or storage slot, never the wallet or a vault;
// a quote names its terminal and kind, a drop never names a terminal. A sale
// and a burn name the quote they confirm and the units.
func TestParseInputValidatesStackOperations(t *testing.T) {
	cmd, ok := parseInput("item_drop", &inputPayload{Box: "storage", Slot: 3, Quantity: 2, X: -1, Y: -1})
	if !ok || cmd.Move.From != world.BoxStorage || cmd.Move.FromSlot != 3 || cmd.Move.Qty != 2 || cmd.TargetX != -1 {
		t.Fatalf("a storage drop at the feet parses: %+v", cmd)
	}
	cmd, ok = parseInput("item_quote", &inputPayload{Kind: "burn", Box: "bag", Slot: 4, EntityID: 9})
	if !ok || cmd.QuoteKind != content.QuoteBurn || cmd.EntityID != 9 || cmd.Move.FromSlot != 4 {
		t.Fatalf("a quote names its terminal, kind and stack: %+v", cmd)
	}
	if cmd, ok := parseInput("item_sell", &inputPayload{QuoteID: "q1", Quantity: 3}); !ok || cmd.QuoteID != "q1" || cmd.Move.Qty != 3 {
		t.Fatalf("a sale names its quote and units: %+v", cmd)
	}
	for name, c := range map[string]struct {
		kind string
		p    inputPayload
	}{
		"the wallet":               {"item_drop", inputPayload{Box: "bag", Slot: world.WalletSlot, Quantity: 1}},
		"a vault":                  {"item_drop", inputPayload{Box: "vault", Slot: 0, Quantity: 1}},
		"no units":                 {"item_drop", inputPayload{Box: "bag", Slot: 0}},
		"a drop at a terminal":     {"item_drop", inputPayload{Box: "bag", Slot: 0, Quantity: 1, EntityID: 3}},
		"a far target":             {"item_drop", inputPayload{Box: "bag", Slot: 0, Quantity: 1, X: 1e9, Y: 0}},
		"a quote with no terminal": {"item_quote", inputPayload{Kind: "sell", Box: "bag", Slot: 0}},
		"a quote of no kind":       {"item_quote", inputPayload{Kind: "gift", Box: "bag", Slot: 0, EntityID: 3}},
		"a sale with no quote":     {"item_sell", inputPayload{Quantity: 1}},
		"a burn with no units":     {"item_burn", inputPayload{QuoteID: "q1"}},
	} {
		if _, ok := parseInput(c.kind, &c.p); ok {
			t.Fatalf("%s must not parse", name)
		}
	}
}

// A market input names its terminal and its listing, or the stack and terms
// of a listing.
func TestParseInputValidatesMarketInputs(t *testing.T) {
	cmd, ok := parseInput("market_list", &inputPayload{EntityID: 9, Box: "bag", Slot: 2, Quantity: 3, Mode: "auction", Price: 40, Hours: 12})
	if !ok || cmd.EntityID != 9 || cmd.Move.FromSlot != 2 || cmd.Move.Qty != 3 ||
		cmd.Market != (world.MarketInput{Mode: "auction", Price: 40, Hours: 12}) {
		t.Fatalf("a listing parses: %+v", cmd)
	}
	if cmd, ok := parseInput("market_bid", &inputPayload{EntityID: 9, ListingID: "l1", Price: 6}); !ok || cmd.Market.ListingID != "l1" || cmd.Market.Price != 6 {
		t.Fatalf("a bid parses: %+v", cmd)
	}
	if cmd, ok := parseInput("market_open", &inputPayload{EntityID: 9, ItemID: "gel", After: "1_a"}); !ok || cmd.ItemID != "gel" || cmd.Market.After != "1_a" {
		t.Fatalf("a view request parses: %+v", cmd)
	}
	for name, c := range map[string]struct {
		kind string
		p    inputPayload
	}{
		"a listing without a terminal": {"market_list", inputPayload{Box: "bag", Slot: 0, Quantity: 1, Mode: "fixed", Price: 1, Hours: 1}},
		"a listing without a price":    {"market_list", inputPayload{EntityID: 9, Box: "bag", Slot: 0, Quantity: 1, Mode: "fixed", Hours: 1}},
		"a listing from a vault":       {"market_list", inputPayload{EntityID: 9, Box: "vault", Slot: 0, Quantity: 1, Mode: "fixed", Price: 1, Hours: 1}},
		"a buy without a listing":      {"market_buy", inputPayload{EntityID: 9}},
		"a bid without a price":        {"market_bid", inputPayload{EntityID: 9, ListingID: "l1"}},
		"a collect without a terminal": {"market_collect", inputPayload{}},
	} {
		if _, ok := parseInput(c.kind, &c.p); ok {
			t.Fatalf("%s must not parse", name)
		}
	}
}

// A trade offer is bounded and carries bag slots and coins.
func TestParseInputValidatesTradeOffers(t *testing.T) {
	cmd, ok := parseInput("trade_offer", &inputPayload{Items: []offerLine{{Slot: 2, Qty: 3}}, Coins: 7})
	if !ok || len(cmd.Offer) != 1 || cmd.Offer[0] != (world.OfferLine{Slot: 2, Qty: 3}) || cmd.Coins != 7 {
		t.Fatalf("an offer parses: %+v", cmd)
	}
	tooMany := make([]offerLine, content.TradeOfferMaxStacks+1)
	for i := range tooMany {
		tooMany[i] = offerLine{Slot: i, Qty: 1}
	}
	for name, p := range map[string]inputPayload{
		"too many stacks":      {Items: tooMany},
		"negative coins":       {Coins: -1},
		"a zero line":          {Items: []offerLine{{Slot: 0, Qty: 0}}},
		"a slot past the grid": {Items: []offerLine{{Slot: content.StorageMaxSlots, Qty: 1}}},
	} {
		if _, ok := parseInput("trade_offer", &p); ok {
			t.Fatalf("%s must not parse", name)
		}
	}
	if cmd, ok := parseInput("trade_confirm", &inputPayload{Revision: 4}); !ok || cmd.Revision != 4 {
		t.Fatalf("a confirmation names its revision: %+v", cmd)
	}
}
