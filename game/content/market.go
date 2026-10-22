// Package content — market.go
//
// A market terminal is a bot whose action has a market capability. The engine
// keeps the listings and the claims of each market, of one world or of one
// universe; this server shows them and runs each operation of a player as a
// linked operation (linked_op.go) with the market change in one transaction:
//
//	list    the stack leaves the bag and the listing fee the wallet
//	cancel  an open listing without bids comes back
//	buy     the price leaves the wallet; the stack arrives
//	bid     the bid leaves the wallet; an outbid returns it as a claim
//	collect the pending claims of the player arrive
//
// The market broker reads a view off the tick. Each operation names a listing
// or the claims of the last view the player saw; the engine checks them
// against its records and refuses a change. A listing checks the terminal
// (selection, listing modes), the profile (status, durations, price range)
// and the item policy here; the engine checks the profile again. Only account
// players use a market.
//
// Cross-process contract:
//
//	market_open (client→server) — entityId, itemId, mode, after
//	market (server→client)      — entityId, the terms of the market profile, the
//	                              selection, itemIds and listingModes of the terminal,
//	                              itemId, mode, after, listings, next, mine, claims
//	market_list                 — entityId, box, slot, quantity, mode, price, hours
//	market_cancel, market_buy   — entityId, listingId
//	market_bid                  — entityId, listingId, price
//	market_collect              — entityId
//	market_ack (server→client)  — action, ok, reason
//
// Caller MUST hold s.mu.
package content

import (
	"slices"
	"time"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// Kinds of a market operation.
const (
	opMarketList    = "market_list"
	opMarketCancel  = "market_cancel"
	opMarketBuy     = "market_buy"
	opMarketBid     = "market_bid"
	opMarketCollect = "market_collect"
)

// Refusals of a market operation.
const (
	RefuseGuest   = "guest_account"   // a guest has no profile for a market
	RefuseGone    = "listing_gone"    // the listing is sold, cancelled or ended
	RefuseChanged = "listing_changed" // the listing changed since the view
	RefuseBidLow  = "bid_low"         // the bid is under the lowest one the auction takes
	// RefuseMarketClosed: the profile is unknown, does not fit this world, or
	// is retired and takes no new listing.
	RefuseMarketClosed = "market_closed"
	RefuseTerms        = "terms_refused" // a duration, price or bid outside the terms of the market
)

// MarketListing is one listing of a market view.
type MarketListing struct {
	ListingID  string    `json:"listingId"`
	Revision   uint32    `json:"revision"`
	SellerName string    `json:"sellerName"`
	ItemID     string    `json:"itemId"`
	CID        string    `json:"olCid"`
	Quantity   int       `json:"quantity"`
	Mode       string    `json:"mode"`
	Price      int       `json:"price"`
	Bid        int       `json:"bid"`
	Bids       int       `json:"bids"`
	MinBid     int       `json:"minBid"`
	EndsAt     time.Time `json:"-"`
	EndsInMs   int64     `json:"endsInMs"`
	Mine       bool      `json:"mine"`
	Leading    bool      `json:"leading"`
}

// MarketClaim is coins or a stack a market owes the viewer.
type MarketClaim struct {
	ClaimID    string `json:"claimId"`
	Reason     string `json:"reason"`
	Coins      int    `json:"coins"`
	ItemID     string `json:"itemId,omitempty"`
	CID        string `json:"olCid,omitempty"`
	Quantity   int    `json:"quantity,omitempty"`
	MarketCode string `json:"marketCode"`
}

// MarketView is a page of the open listings of a market, the open listings
// of the viewer there and its pending claims.
type MarketView struct {
	Listings []MarketListing `json:"listings"`
	Next     string          `json:"next"`
	Mine     []MarketListing `json:"mine"`
	Claims   []MarketClaim   `json:"claims"`
	// ScopeKey and Revision are the market and the profile revision the
	// engine read the view of.
	ScopeKey string `json:"-"`
	Revision uint64 `json:"-"`
}

// MarketQuery asks for the view of the market of a terminal: a page of its
// listings, of one item or one mode when set.
type MarketQuery struct {
	Player              world.EntityID
	Profile             world.PlayerID
	Terminal            world.EntityID
	Code                string
	ItemID, Mode, After string
}

// MarketBroker reads market views from the engine off the tick, and hands
// each one to ShowMarket.
type MarketBroker interface {
	ViewMarket(q MarketQuery)
}

// marketSeen is the last view a player saw.
type marketSeen struct {
	query MarketQuery
	view  MarketView
}

// marketPayload is what a terminal shows: the terms of its market, what a
// seller lists here, and a page of the market.
type marketPayload struct {
	EntityID world.EntityID `json:"entityId"`
	MarketProfile
	Selection    string   `json:"selection"`
	ItemIDs      []string `json:"itemIds"`
	ListingModes []string `json:"listingModes"`
	ItemID       string   `json:"itemId"`
	Mode         string   `json:"mode"`
	After        string   `json:"after"`
	MarketView
}

// marketTerminal is the terminal in view of an account player and the profile
// of its market, or the refusal. A retired market still opens.
func (s *Content) marketTerminal(player *world.Session, terminal world.EntityID) (ActionMarket, MarketProfile, string) {
	ms := s.findBot(terminal)
	action := s.actionCache[terminal]
	switch {
	case player.AccountID == "" || s.Durability == nil || s.Markets == nil:
		return ActionMarket{}, MarketProfile{}, RefuseGuest
	case s.Vt(player.ID, player.MapCode).Ghost():
		return ActionMarket{}, MarketProfile{}, RefuseDead
	case s.Trading(player):
		return ActionMarket{}, MarketProfile{}, RefuseTrading
	case ms == nil || ms.Vt(terminal).Ghost() || !s.inPlayerView(player, terminal, ms):
		return ActionMarket{}, MarketProfile{}, RefuseOutOfRange
	case action == nil || action.Market.MarketCode == "":
		return ActionMarket{}, MarketProfile{}, RefuseNotOffered
	}
	profile, known := s.marketProfiles[action.Market.MarketCode]
	if !known || profile.Defect != "" {
		return action.Market, profile, RefuseMarketClosed
	}
	return action.Market, profile, ""
}

func (s *Content) sendMarketAck(player *world.Session, kind, reason string) {
	world.SendAck(player, "market_ack", reason, map[string]interface{}{"action": kind})
}

// HandleMarketOpen asks the broker for a page of the market of a terminal.
func (s *Content) HandleMarketOpen(player *world.Session, cmd *world.InputCommand) {
	terminal, _, reason := s.marketTerminal(player, cmd.EntityID)
	if reason != "" {
		s.sendMarketAck(player, "market_open", reason)
		return
	}
	s.Markets.ViewMarket(MarketQuery{
		Player: player.ID, Profile: player.PlayerID, Terminal: cmd.EntityID, Code: terminal.MarketCode,
		ItemID: cmd.ItemID, Mode: cmd.Market.Mode, After: cmd.Market.After,
	})
}

// ShowMarket keeps the view a player asked for and sends it. A player that
// left gets nothing.
func (s *Content) ShowMarket(q MarketQuery, view MarketView, now time.Time) {
	player := s.Sessions[q.Player]
	if player == nil || player.PlayerID != q.Profile {
		return
	}
	if s.marketViews == nil {
		s.marketViews = make(map[world.EntityID]*marketSeen)
	}
	action := s.actionCache[q.Terminal]
	profile, known := s.marketProfiles[q.Code]
	if action == nil || !known || action.Market.MarketCode != q.Code {
		s.sendMarketAck(player, "market_open", RefuseMarketClosed)
		return
	}
	s.marketViews[q.Player] = &marketSeen{query: q, view: view}
	logx.Debugf("[MARKET] view player=%s terminal=%d market=%s engineScopeKey=%s engineRevision=%d listings=%d",
		player.PlayerID, q.Terminal, profile.Ref, view.ScopeKey, view.Revision, len(view.Listings))
	shown := view
	shown.Listings, shown.Mine = endsIn(view.Listings, now), endsIn(view.Mine, now)
	world.SendMessage(player, "market", marketPayload{
		EntityID: q.Terminal, MarketProfile: profile, Selection: action.Market.Selection,
		ItemIDs: action.Market.ItemIDs, ListingModes: profile.listingModes(action.Market),
		ItemID: q.ItemID, Mode: q.Mode, After: q.After, MarketView: shown,
	})
}

// MarketUnavailable tells a player that its market view could not be read.
func (s *Content) MarketUnavailable(q MarketQuery) {
	if player := s.Sessions[q.Player]; player != nil && player.PlayerID == q.Profile {
		s.sendMarketAck(player, "market_open", "unavailable")
	}
}

func endsIn(listings []MarketListing, now time.Time) []MarketListing {
	out := slices.Clone(listings)
	for i := range out {
		out[i].EndsInMs = max(0, out[i].EndsAt.Sub(now).Milliseconds())
	}
	return out
}

// seenListing is a listing of the last view the player saw at terminal.
func (s *Content) seenListing(player *world.Session, terminal world.EntityID, listingID string) (MarketListing, bool) {
	seen := s.marketViews[player.ID]
	if seen == nil || seen.query.Terminal != terminal {
		return MarketListing{}, false
	}
	for _, listing := range append(slices.Clone(seen.view.Listings), seen.view.Mine...) {
		if listing.ListingID == listingID {
			return listing, true
		}
	}
	return MarketListing{}, false
}

// marketOp is the linked operation of a market operation: once the engine
// answers, the player learns the result and sees the market again.
func (s *Content) marketOp(kind string, gain, refund Goods) *linkedOp {
	w := &linkedOp{kind: kind, gain: gain, refund: refund}
	w.settled = func(outcome persist.Outcome, refusal string, player *world.Session, _ time.Time) {
		s.recordEconomy(kind, refusal)
		logx.Infof("[ECONOMY] %s player=%s outcome=%d refusal=%q coins=%+d stacks=%d economy=%s",
			kind, w.profile, outcome, refusal, gain.Coins-refund.Coins, len(gain.Stacks), s.EconomyKey())
		if player == nil {
			return
		}
		s.sendMarketAck(player, kind, refusal)
		if seen := s.marketViews[player.ID]; seen != nil && s.Markets != nil {
			s.Markets.ViewMarket(seen.query)
		}
	}
	return w
}

// startMarketOp links a market operation of player, or tells it why not.
func (s *Content) startMarketOp(player *world.Session, code string, escrow Transfer, op persist.MarketOp, w *linkedOp) {
	op.MarketCode = code
	reason := s.linkOp(player, newOpID(w.kind), escrow, persist.Effect{Market: &op}, w)
	if reason != "" {
		s.recordEconomy(w.kind, reason)
		s.sendMarketAck(player, w.kind, reason)
	}
}

// listRefusal is why a stack cannot be listed at a terminal, or "", with the
// policy the stack runs under: the terminal offers the mode and the item, the
// market takes new listings at this duration and price, and the item policy
// allows a listing.
func (s *Content) listRefusal(terminal ActionMarket, profile MarketProfile, stack Stack, held bool,
	cmd *world.InputCommand) (ItemPolicy, string) {
	in := cmd.Market
	policy, policyReason := s.ItemPolicyOf(stack.ItemID, stack.CID)
	switch {
	case !held || cmd.Move.Qty < 1 || !s.takesSlot(stack.ItemID) || s.DeadItemIDs[stack.ItemID]:
		return policy, RefuseNoStack
	case profile.Status != marketStatusActive:
		return policy, RefuseMarketClosed
	case !slices.Contains(profile.listingModes(terminal), in.Mode):
		return policy, RefuseNotOffered
	case terminal.Selection != "policy" && !slices.Contains(terminal.ItemIDs, stack.ItemID):
		return policy, RefuseNotOffered
	case !slices.Contains(profile.Hours, in.Hours) || in.Price < profile.PriceMin || in.Price > profile.PriceMax:
		return policy, RefuseTerms
	case policyReason != "":
		return policy, policyReason
	case !policy.Market:
		return policy, RefusePolicy
	}
	return policy, ""
}

// HandleMarketList lists units of a bag or storage stack at a fixed price or
// as an auction. The listing fee leaves at once; a refusal returns it. Each
// attempt logs the market it resolved to and the policy of the item.
func (s *Content) HandleMarketList(player *world.Session, cmd *world.InputCommand) {
	terminal, profile, reason := s.marketTerminal(player, cmd.EntityID)
	place := Place{Box: cmd.Move.From, Slot: cmd.Move.FromSlot}
	stack, _, held := s.PlacedStack(player, place)
	var policy ItemPolicy
	if reason == "" {
		policy, reason = s.listRefusal(terminal, profile, stack, held, cmd)
	}
	logx.Infof("[MARKET] list player=%s terminal=%d market=%s item=%s cid=%s selection=%s listingModes=%v policyMarket=%t reason=%q",
		player.PlayerID, cmd.EntityID, profile.Ref, stack.ItemID, stack.CID, terminal.Selection,
		profile.listingModes(terminal), policy.Market, reason)
	if reason != "" {
		s.recordEconomy(opMarketList, reason)
		s.sendMarketAck(player, opMarketList, reason)
		return
	}
	qty, fee := min(cmd.Move.Qty, stack.Qty), profile.ListingFee
	listed := Stack{ItemID: stack.ItemID, CID: stack.CID, Qty: qty}
	s.startMarketOp(player, terminal.MarketCode, Transfer{Takes: []Debit{{Place: place, Qty: qty}}, Coins: -int64(fee)},
		persist.MarketOp{
			Mode: cmd.Market.Mode, ItemID: listed.ItemID, CID: listed.CID, Quantity: qty, Price: cmd.Market.Price,
			Hours: cmd.Market.Hours, ListingFee: fee, SaleFee: profile.SaleFee, SellerName: player.DisplayName,
		},
		s.marketOp(opMarketList, Goods{}, Goods{Coins: int64(fee), Stacks: []Stack{listed}}))
}

// HandleMarketCancel takes back an open listing of the player without bids.
func (s *Content) HandleMarketCancel(player *world.Session, cmd *world.InputCommand) {
	s.listingOp(player, cmd, opMarketCancel, func(listing MarketListing) (Transfer, Goods, Goods, string) {
		if !listing.Mine || listing.Bids > 0 {
			return Transfer{}, Goods{}, Goods{}, RefuseChanged
		}
		return Transfer{}, Goods{Stacks: []Stack{listingStack(listing)}}, Goods{}, ""
	})
}

// HandleMarketBuy buys a fixed-price listing of another seller.
func (s *Content) HandleMarketBuy(player *world.Session, cmd *world.InputCommand) {
	s.listingOp(player, cmd, opMarketBuy, func(listing MarketListing) (Transfer, Goods, Goods, string) {
		if listing.Mine || listing.Mode != "fixed" {
			return Transfer{}, Goods{}, Goods{}, RefuseNotOffered
		}
		price := int64(listing.Price)
		return Transfer{Coins: -price}, Goods{Stacks: []Stack{listingStack(listing)}}, Goods{Coins: price}, ""
	})
}

// HandleMarketBid bids on an auction of another seller. The bid leaves the
// wallet until a higher bid returns it as a claim.
func (s *Content) HandleMarketBid(player *world.Session, cmd *world.InputCommand) {
	s.listingOp(player, cmd, opMarketBid, func(listing MarketListing) (Transfer, Goods, Goods, string) {
		switch {
		case listing.Mine || listing.Mode != "auction":
			return Transfer{}, Goods{}, Goods{}, RefuseNotOffered
		case cmd.Market.Price < listing.MinBid:
			return Transfer{}, Goods{}, Goods{}, RefuseBidLow
		}
		bid := int64(cmd.Market.Price)
		return Transfer{Coins: -bid}, Goods{}, Goods{Coins: bid}, ""
	})
}

func listingStack(listing MarketListing) Stack {
	return Stack{ItemID: listing.ItemID, CID: listing.CID, Qty: listing.Quantity}
}

// listingOp runs an operation on a listing of the last view: plan says what
// leaves, arrives once it applies and returns once it is refused.
func (s *Content) listingOp(player *world.Session, cmd *world.InputCommand, kind string,
	plan func(MarketListing) (Transfer, Goods, Goods, string)) {
	terminal, _, reason := s.marketTerminal(player, cmd.EntityID)
	listing, seen := s.seenListing(player, cmd.EntityID, cmd.Market.ListingID)
	var escrow Transfer
	var gain, refund Goods
	switch {
	case reason != "":
	case !seen:
		reason = RefuseGone
	default:
		escrow, gain, refund, reason = plan(listing)
	}
	if reason != "" {
		s.recordEconomy(kind, reason)
		s.sendMarketAck(player, kind, reason)
		return
	}
	price := listing.Price
	if kind == opMarketBid {
		price = cmd.Market.Price
	}
	s.startMarketOp(player, terminal.MarketCode, escrow, persist.MarketOp{
		ListingID: listing.ListingID, Revision: listing.Revision, ItemID: listing.ItemID, CID: listing.CID,
		Quantity: listing.Quantity, Price: price,
	}, s.marketOp(kind, gain, refund))
}

// HandleMarketCollect takes the pending claims of the last view into the
// goods of the player.
func (s *Content) HandleMarketCollect(player *world.Session, cmd *world.InputCommand) {
	terminal, _, reason := s.marketTerminal(player, cmd.EntityID)
	seen := s.marketViews[player.ID]
	if reason == "" && (seen == nil || seen.query.Terminal != cmd.EntityID || len(seen.view.Claims) == 0) {
		reason = "claim_gone"
	}
	if reason != "" {
		s.recordEconomy(opMarketCollect, reason)
		s.sendMarketAck(player, opMarketCollect, reason)
		return
	}
	op := persist.MarketOp{}
	var gain Goods
	for _, claim := range seen.view.Claims {
		op.ClaimIDs = append(op.ClaimIDs, claim.ClaimID)
		op.Coins += claim.Coins
		gain.Coins += int64(claim.Coins)
		if claim.Quantity > 0 {
			op.Stacks = append(op.Stacks, persist.Stack{ItemID: claim.ItemID, CID: claim.CID, Quantity: claim.Quantity})
			gain.Stacks = append(gain.Stacks, Stack{ItemID: claim.ItemID, CID: claim.CID, Qty: claim.Quantity})
		}
	}
	s.startMarketOp(player, terminal.MarketCode, Transfer{}, op, s.marketOp(opMarketCollect, gain, Goods{}))
}
