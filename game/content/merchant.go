// Package content — merchant.go
//
// Buyback and burn at an NPC terminal. A CyberiaAction lists the items its
// merchant buys back (buybackItems) and the items its terminal burns
// (burnItems); the item policy of the economy must allow the operation too.
// The list never grants what the policy refuses.
//
// A merchant is an authorized coin source: a buyback pays BuybackPayout coins
// per unit as new coins. Content creates that source on purpose: an action
// lists the item, and the policy of the item allows a buyback that pays at
// least one coin. A burn destroys the units, charges BurnCoinFee coins per unit
// and grants the salvage of each unit, as one change.
//
// A sale or a burn runs on a quote the server makes first: the price per unit
// of one stack at one terminal under one economy revision, for quoteLifetime.
// The player confirms the quote it saw, once. A quote the stack, the price or
// the economy no longer matches is refused with the current price; a used or
// expired quote never runs.
//
// Cross-process contract:
//
//	item_quote (client→server)    — kind sell | burn, entityId, box, slot
//	item_quote_ack                — quoteId, kind, itemId, unit, maxQuantity, expiresInMs; or a refusal
//	item_sell, item_burn          — quoteId, quantity
//	item_sell_ack / item_burn_ack — ok, reason, itemId, quantity, unit, coins
//
// Caller MUST hold s.mu.
package content

import (
	"crypto/rand"
	"encoding/hex"
	"slices"
	"time"

	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// Kinds of a quote.
const (
	QuoteSell = "sell"
	QuoteBurn = "burn"
)

// quoteLifetime bounds how long a player may take to confirm a quote.
const quoteLifetime = time.Minute

// quote is one priced offer the server made a player; one per player.
type quote struct {
	id       string
	burn     bool
	terminal world.EntityID
	place    Place
	itemID   string
	cid      string
	unit     int
	economy  string
	expires  time.Time
}

// botHasMerchant reports whether a live bot buys back or burns items.
func (s *Content) botHasMerchant(id world.EntityID, ms *world.MapState) bool {
	if ms == nil || ms.Vt(id).Ghost() {
		return false
	}
	action := s.actionCache[id]
	return action != nil && (len(action.BuybackItems) > 0 || len(action.BurnItems) > 0)
}

// terminalStack resolves a stack at a terminal: the action of a live bot in
// view that lists the item, and the stack in the place. It returns a refusal
// when one is missing.
func (s *Content) terminalStack(player *world.Session, terminal world.EntityID, place Place, burn bool) (Stack, string) {
	stack, _, held := s.PlacedStack(player, place)
	ms := s.findBot(terminal)
	listed := func(a *CyberiaAction) []string {
		if burn {
			return a.BurnItems
		}
		return a.BuybackItems
	}
	switch {
	case s.Vt(player.ID, player.MapCode).Ghost():
		return stack, RefuseDead
	case s.Trading(player):
		return stack, RefuseTrading
	case ms == nil || ms.Vt(terminal).Ghost() || !s.inPlayerView(player, terminal, ms):
		return stack, RefuseOutOfRange
	case !held || !s.takesSlot(stack.ItemID):
		return stack, RefuseNoStack
	case s.actionCache[terminal] == nil || !slices.Contains(listed(s.actionCache[terminal]), stack.ItemID):
		return stack, RefuseNotOffered
	}
	return stack, ""
}

// unitPrice is the coins one unit of stack pays at a merchant, or costs to
// burn, by its item policy; or a refusal.
func (s *Content) unitPrice(stack Stack, burn bool) (ItemPolicy, int, string) {
	policy, reason := s.ItemPolicyOf(stack.ItemID, stack.CID)
	switch {
	case reason != "":
		return policy, 0, reason
	case burn && !policy.Burn, !burn && (!policy.Buyback || policy.BuybackPayout < 1):
		return policy, 0, RefusePolicy
	case burn:
		return policy, policy.BurnCoinFee, ""
	}
	return policy, policy.BuybackPayout, ""
}

func quoteKind(burn bool) string {
	if burn {
		return QuoteBurn
	}
	return QuoteSell
}

// HandleItemQuote prices one stack at a terminal and keeps the quote.
func (s *Content) HandleItemQuote(player *world.Session, cmd *world.InputCommand) {
	burn := cmd.QuoteKind == QuoteBurn
	place := Place{Box: cmd.Move.From, Slot: cmd.Move.FromSlot}
	stack, reason := s.terminalStack(player, cmd.EntityID, place, burn)
	unit := 0
	if reason == "" {
		_, unit, reason = s.unitPrice(stack, burn)
	}
	ack := map[string]interface{}{"kind": quoteKind(burn), "itemId": stack.ItemID}
	if reason != "" {
		s.recordEconomy(quoteKind(burn)+"_quote", reason)
		world.SendAck(player, "item_quote_ack", reason, ack)
		return
	}
	q := &quote{
		id: randomID(), burn: burn, terminal: cmd.EntityID, place: place, itemID: stack.ItemID, cid: stack.CID,
		unit: unit, economy: s.EconomyKey(), expires: time.Now().Add(quoteLifetime),
	}
	if s.quotes == nil {
		s.quotes = make(map[world.EntityID]*quote)
	}
	s.quotes[player.ID] = q
	ack["quoteId"], ack["unit"], ack["maxQuantity"] = q.id, unit, s.quoteMax(player, stack.Qty, unit, burn)
	ack["expiresInMs"] = quoteLifetime.Milliseconds()
	world.SendAck(player, "item_quote_ack", "", ack)
}

// quoteMax is how many units of a stack of qty one confirm may cover: the
// stack, and what the wallet takes or pays.
func (s *Content) quoteMax(player *world.Session, qty, unit int, burn bool) int {
	switch {
	case unit <= 0:
		return qty
	case burn:
		return min(qty, int(player.Coins)/unit)
	}
	return int(min(int64(qty), max(s.coinRoom(player), 0)/int64(unit)))
}

// randomID is an unguessable id: 12 random bytes in hex.
func randomID() string {
	b := make([]byte, 12)
	_, _ = rand.Read(b)
	return hex.EncodeToString(b)
}

// takeQuote removes the quote a confirm names and returns it, or why it
// cannot run: missing, used, expired or of the other kind.
func (s *Content) takeQuote(player *world.Session, cmd *world.InputCommand, burn bool) (*quote, string) {
	q := s.quotes[player.ID]
	if q == nil || q.id != cmd.QuoteID || q.burn != burn {
		return nil, RefuseQuoteGone
	}
	delete(s.quotes, player.ID)
	if time.Now().After(q.expires) {
		return nil, RefuseQuoteGone
	}
	return q, ""
}

// quotedStack checks a taken quote against the world now: the same stack in
// its place at a terminal that still lists it, at the same price under the same
// economy. It returns the stack, its policy, the price now and a refusal.
func (s *Content) quotedStack(player *world.Session, q *quote) (Stack, ItemPolicy, int, string) {
	stack, reason := s.terminalStack(player, q.terminal, q.place, q.burn)
	if reason != "" {
		return stack, ItemPolicy{}, 0, reason
	}
	if stack.ItemID != q.itemID || stack.CID != q.cid {
		return stack, ItemPolicy{}, 0, RefuseNoStack
	}
	policy, unit, reason := s.unitPrice(stack, q.burn)
	if reason == "" && (unit != q.unit || s.EconomyKey() != q.economy) {
		reason = RefuseQuote
	}
	return stack, policy, unit, reason
}

func (s *Content) sendMerchantAck(player *world.Session, msgType, reason, itemID string, qty, unit int, coins int64) {
	world.SendAck(player, msgType, reason, map[string]interface{}{
		"itemId": itemID, "quantity": qty, "unit": unit, "coins": coins,
	})
}

// HandleItemSell sells units of a quoted stack to its merchant: the units
// leave, and the quoted payout enters the wallet.
func (s *Content) HandleItemSell(player *world.Session, cmd *world.InputCommand) {
	q, reason := s.takeQuote(player, cmd, false)
	var stack Stack
	var qty, unit int
	var payout int64
	if reason == "" {
		stack, _, unit, reason = s.quotedStack(player, q)
		qty = min(cmd.Move.Qty, stack.Qty)
		payout = int64(unit) * int64(qty)
	}
	if reason == "" {
		reason = s.ApplyTransfer(player, Transfer{Takes: []Debit{{Place: q.place, Qty: qty}}, Coins: payout})
	}
	s.recordEconomy("sell", reason)
	s.sendMerchantAck(player, "item_sell_ack", reason, stack.ItemID, qty, unit, payout)
	if reason == "" {
		logx.Infof("[ECONOMY] sell player=%s item=%s cid=%s qty=%d coins=+%d terminal=%s economy=%s",
			player.PlayerID, stack.ItemID, stack.CID, qty, payout, q.terminal, s.EconomyKey())
	}
}

// HandleItemBurn burns units of a quoted stack at its terminal: the units
// leave, the quoted fee leaves the wallet, and the salvage of each unit enters
// the bag, as one change.
func (s *Content) HandleItemBurn(player *world.Session, cmd *world.InputCommand) {
	q, reason := s.takeQuote(player, cmd, true)
	var stack Stack
	var policy ItemPolicy
	var qty, unit int
	var fee int64
	if reason == "" {
		stack, policy, unit, reason = s.quotedStack(player, q)
		qty = min(cmd.Move.Qty, stack.Qty)
		fee = int64(unit) * int64(qty)
	}
	if reason == "" {
		gives := make([]Stack, 0, len(policy.Salvage))
		for _, line := range policy.Salvage {
			if s.DefinitionCID(line.ItemID) == "" {
				reason = RefuseIncompatible
			}
			gives = append(gives, Stack{ItemID: line.ItemID, Qty: line.Qty * qty})
		}
		if reason == "" {
			reason = s.ApplyTransfer(player, Transfer{Takes: []Debit{{Place: q.place, Qty: qty}}, Coins: -fee, Gives: gives})
		}
	}
	s.recordEconomy("burn", reason)
	s.sendMerchantAck(player, "item_burn_ack", reason, stack.ItemID, qty, unit, fee)
	if reason != "" {
		return
	}
	s.AdvancePlayerQuestsOnGain(player)
	logx.Infof("[ECONOMY] burn player=%s item=%s cid=%s qty=%d coins=-%d terminal=%s economy=%s",
		player.PlayerID, stack.ItemID, stack.CID, qty, fee, q.terminal, s.EconomyKey())
}
