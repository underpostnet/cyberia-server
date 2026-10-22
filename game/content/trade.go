// Package content — trade.go
//
// A trade between two players of this world process. The server holds the
// whole state machine; the client shows it from the snapshot and sends intents.
//
//	requested  — the initiator asked; the partner accepts or declines
//	open       — each side offers bag stacks and coins and confirms
//	committing — both confirmed: the offers left both bags (escrow) and the
//	             joint commit runs; account players only
//
// Every offer change raises the revision and clears both confirmations; a
// confirmation names the revision it saw. While a player trades, no explicit
// input moves its goods, and the commit checks every offered stack again: its
// slot, item, definition and units, the wallets, the room of both bags and the
// policy of each item. A guest trade applies at once in memory. An account
// trade commits both profiles in one engine transaction (persist.Link): until
// the engine answers, the offers stay in escrow and no state of either player
// commits alone. Applied, each side receives the escrow of the other; refused,
// each side gets its own back. A retry after a transport error repeats the same
// operation id, so the engine applies it once. A guest never trades with an
// account: goods never cross from a session state into a profile.
//
// Cross-process contract:
//
//	trade_request (client→server) — toId: ask a visible player
//	trade_reply                   — fromId, accept
//	trade_offer                   — items [{box, slot, qty}], coins: the whole offer of the sender
//	trade_confirm                 — revision
//	trade_cancel
//	trade_ack (server→client)     — action, ok, reason
//	snapshot self.trade           — the trade of the player, absent when none
//
// Caller MUST hold s.mu.
package content

import (
	"fmt"
	"time"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// TradeState is the stage of a trade.
type TradeState uint8

const (
	TradeRequested TradeState = iota
	TradeOpen
	TradeCommitting
)

var tradeStateWords = [...]string{TradeRequested: "requested", TradeOpen: "open", TradeCommitting: "committing"}

const (
	// tradeRequestTTL bounds the wait for an answer, and tradeIdleTTL an open
	// trade without an offer change.
	tradeRequestTTL = 30 * time.Second
	tradeIdleTTL    = 3 * time.Minute
)

// Reasons a trade closes or refuses, beside the economic refusals.
const (
	tradeBusy     = "busy"          // a side is in another trade or an assembly
	tradeMixed    = "guest_account" // a guest and an account never trade
	tradeRevision = "offer_changed" // a confirmation named an older revision
	tradeDeclined = "declined"
	tradeExpired  = "expired"
	tradeCanceled = "cancelled"
	tradeLeft     = "left"
	tradeDone     = "done"
	tradeUnsaved  = "unsaved" // the engine refused the joint commit
)

// TradeLine is one offered stack: units of a bag slot, with the item and the
// definition the slot held when it was offered.
type TradeLine struct {
	Slot   int
	Qty    int
	ItemID string
	CID    string
}

// TradeSide is one player of a trade and its offer.
type TradeSide struct {
	ID        world.EntityID
	PlayerID  world.PlayerID
	Name      string
	Lines     []TradeLine
	Coins     int
	Confirmed bool
	// Escrow is what left the side while the trade commits.
	Escrow      []Stack
	EscrowCoins int
	// Gone marks a side whose session ended.
	Gone bool
}

// Trade is one trade between two players. Sides[0] is the initiator.
type Trade struct {
	// ID is the operation id of the joint commit.
	ID        string
	State     TradeState
	Revision  uint32
	Sides     [2]TradeSide
	Account   bool
	ExpiresAt time.Time
}

// tradeOf is the trade of the player and its side, or nil.
func (s *Content) tradeOf(player *world.Session) (*Trade, int) {
	t := s.trades[player.ID]
	if t == nil {
		return nil, 0
	}
	if t.Sides[1].ID == player.ID {
		return t, 1
	}
	return t, 0
}

// Trading reports whether a trade holds the goods of the player: an open or
// committing trade.
func (s *Content) Trading(player *world.Session) bool {
	t, _ := s.tradeOf(player)
	return t != nil && t.State != TradeRequested
}

// incomingCoins is the coins a committing trade will credit to the player.
func (s *Content) incomingCoins(player *world.Session) int {
	if t, side := s.tradeOf(player); t != nil && t.State == TradeCommitting {
		return t.Sides[1-side].EscrowCoins
	}
	return 0
}

func (s *Content) sendTradeAck(player *world.Session, action, reason string) {
	world.SendAck(player, "trade_ack", reason, map[string]interface{}{"action": action})
}

// closeTrade ends a trade before it commits and tells each live side why.
func (s *Content) closeTrade(t *Trade, reason string) {
	for _, side := range t.Sides {
		delete(s.trades, side.ID)
		if player := s.Sessions[side.ID]; player != nil && !side.Gone {
			s.sendTradeAck(player, "close", reason)
		}
	}
	s.recordEconomy("trade", reason)
}

// HandleTradeRequest asks a visible player for a trade.
func (s *Content) HandleTradeRequest(player *world.Session, cmd *world.InputCommand) {
	partner := s.VisiblePlayer(player, cmd.EntityID)
	reason := ""
	switch {
	case partner == nil || partner.ID == player.ID:
		reason = RefuseOutOfRange
	case s.Vt(player.ID, player.MapCode).Ghost() || s.Vt(partner.ID, partner.MapCode).Ghost():
		reason = RefuseDead
	case s.trades[player.ID] != nil || s.trades[partner.ID] != nil || s.crafts[player.ID] != nil || s.crafts[partner.ID] != nil:
		reason = tradeBusy
	case (player.AccountID == "") != (partner.AccountID == ""):
		reason = tradeMixed
	}
	if reason != "" {
		s.sendTradeAck(player, "request", reason)
		return
	}
	s.tradeSeq++
	t := &Trade{
		ID:        fmt.Sprintf("trade-%s-%d-%d", player.PlayerID, time.Now().UnixMilli(), s.tradeSeq),
		Account:   player.AccountID != "",
		ExpiresAt: time.Now().Add(tradeRequestTTL),
		Sides: [2]TradeSide{
			{ID: player.ID, PlayerID: player.PlayerID, Name: player.DisplayName},
			{ID: partner.ID, PlayerID: partner.PlayerID, Name: partner.DisplayName},
		},
	}
	if s.trades == nil {
		s.trades = make(map[world.EntityID]*Trade)
	}
	s.trades[player.ID], s.trades[partner.ID] = t, t
	s.sendTradeAck(player, "request", "")
}

// HandleTradeReply accepts or declines the trade a player asked for.
func (s *Content) HandleTradeReply(player *world.Session, cmd *world.InputCommand) {
	t, side := s.tradeOf(player)
	if t == nil || side != 1 || t.State != TradeRequested || t.Sides[0].ID != cmd.EntityID {
		s.sendTradeAck(player, "reply", RefuseOutOfRange)
		return
	}
	if !cmd.Active {
		s.closeTrade(t, tradeDeclined)
		return
	}
	t.State = TradeOpen
	t.ExpiresAt = time.Now().Add(tradeIdleTTL)
	for _, side := range t.Sides {
		s.sendTradeAck(s.Sessions[side.ID], "open", "")
	}
}

// offerLines checks an offer of the player: bag stacks of distinct slots it
// holds, of items the policy lets it trade, worn stacks that may come off.
func (s *Content) offerLines(player *world.Session, offer []world.OfferLine) ([]TradeLine, string) {
	if len(offer) > TradeOfferMaxStacks {
		return nil, RefuseNotAllowed
	}
	lines := make([]TradeLine, 0, len(offer))
	seen := make(map[int]bool, len(offer))
	for _, o := range offer {
		stack, worn, held := s.PlacedStack(player, Place{Box: world.BoxBag, Slot: o.Slot})
		if !held || seen[o.Slot] || o.Qty < 1 || o.Qty > stack.Qty || s.DeadItemIDs[stack.ItemID] {
			return nil, RefuseNoStack
		}
		seen[o.Slot] = true
		policy, reason := s.ItemPolicyOf(stack.ItemID, stack.CID)
		switch {
		case reason != "":
			return nil, reason
		case !policy.Trade:
			return nil, RefusePolicy
		case worn && o.Qty == stack.Qty && !s.bankableWhileWorn(player, stack.ItemID):
			return nil, RefuseEquipped
		}
		lines = append(lines, TradeLine{Slot: o.Slot, Qty: o.Qty, ItemID: stack.ItemID, CID: stack.CID})
	}
	return lines, ""
}

// HandleTradeOffer replaces the offer of the player.
func (s *Content) HandleTradeOffer(player *world.Session, cmd *world.InputCommand) {
	t, side := s.tradeOf(player)
	if t == nil || t.State != TradeOpen {
		s.sendTradeAck(player, "offer", tradeCanceled)
		return
	}
	lines, reason := s.offerLines(player, cmd.Offer)
	if reason == "" && (cmd.Coins < 0 || int64(cmd.Coins) > int64(player.Coins)) {
		reason = RefuseInsufficient
	}
	if reason != "" {
		s.sendTradeAck(player, "offer", reason)
		return
	}
	t.Sides[side].Lines, t.Sides[side].Coins = lines, cmd.Coins
	t.Sides[0].Confirmed, t.Sides[1].Confirmed = false, false
	t.Revision++
	t.ExpiresAt = time.Now().Add(tradeIdleTTL)
}

// HandleTradeConfirm confirms the offers of the revision the player saw. The
// second confirmation commits the trade.
func (s *Content) HandleTradeConfirm(player *world.Session, cmd *world.InputCommand) {
	t, side := s.tradeOf(player)
	switch {
	case t == nil || t.State != TradeOpen:
		s.sendTradeAck(player, "confirm", tradeCanceled)
		return
	case cmd.Revision != t.Revision:
		s.sendTradeAck(player, "confirm", tradeRevision)
		return
	}
	t.Sides[side].Confirmed = true
	if t.Sides[0].Confirmed && t.Sides[1].Confirmed {
		s.commitTrade(t)
	}
}

// HandleTradeCancel ends a trade that has not committed.
func (s *Content) HandleTradeCancel(player *world.Session) {
	if t, _ := s.tradeOf(player); t != nil && t.State != TradeCommitting {
		s.closeTrade(t, tradeCanceled)
	}
}

// tradeTransfers are the transfers of both sides: each one gives its lines
// and coins, and receives the other's when receive is true.
func tradeTransfers(t *Trade, receive bool) [2]Transfer {
	var out [2]Transfer
	for i, side := range t.Sides {
		out[i].Coins = -int64(side.Coins)
		for _, line := range side.Lines {
			out[i].Takes = append(out[i].Takes, Debit{Place: Place{Box: world.BoxBag, Slot: line.Slot}, Qty: line.Qty})
		}
		if !receive {
			continue
		}
		other := t.Sides[1-i]
		out[i].Coins += int64(other.Coins)
		for _, line := range other.Lines {
			out[i].Gives = append(out[i].Gives, Stack{ItemID: line.ItemID, CID: line.CID, Qty: line.Qty})
		}
	}
	return out
}

// tradeRefusal checks both sides again before a commit: present, in view,
// alive, the same stacks in the offered slots, and policies that still allow
// the trade.
func (s *Content) tradeRefusal(t *Trade, players [2]*world.Session) string {
	a, b := players[0], players[1]
	switch {
	case a == nil || b == nil:
		return tradeLeft
	case s.VisiblePlayer(a, b.ID) == nil:
		return RefuseOutOfRange
	case s.Vt(a.ID, a.MapCode).Ghost() || s.Vt(b.ID, b.MapCode).Ghost():
		return RefuseDead
	}
	for i, side := range t.Sides {
		for _, line := range side.Lines {
			stack, _, held := s.PlacedStack(players[i], Place{Box: world.BoxBag, Slot: line.Slot})
			if !held || stack.ItemID != line.ItemID || stack.CID != line.CID || stack.Qty < line.Qty {
				return RefuseNoStack
			}
			policy, reason := s.ItemPolicyOf(line.ItemID, line.CID)
			if reason != "" {
				return reason
			}
			if !policy.Trade {
				return RefusePolicy
			}
		}
	}
	return ""
}

// reopenTrade clears both confirmations after a refused commit and tells both
// sides why.
func (s *Content) reopenTrade(t *Trade, players [2]*world.Session, reason string) {
	t.Sides[0].Confirmed, t.Sides[1].Confirmed = false, false
	t.Revision++
	for _, player := range players {
		if player != nil {
			s.sendTradeAck(player, "commit", reason)
		}
	}
	s.recordEconomy("trade", reason)
}

// commitTrade exchanges the offers of a trade both sides confirmed. A guest
// trade applies at once; an account trade moves the offers to escrow and
// links the post-trade states of both profiles into one commit.
func (s *Content) commitTrade(t *Trade) {
	players := [2]*world.Session{s.Sessions[t.Sides[0].ID], s.Sessions[t.Sides[1].ID]}
	if reason := s.tradeRefusal(t, players); reason != "" {
		s.reopenTrade(t, players, reason)
		return
	}
	full := tradeTransfers(t, true)
	var plans [2]transferPlan
	for i := range plans {
		var reason string
		if plans[i], reason = s.planTransfer(players[i], full[i]); reason != "" {
			s.reopenTrade(t, players, reason)
			return
		}
	}
	if !t.Account {
		for i, player := range players {
			s.commitTransfer(player, plans[i])
		}
		s.finishTrade(t, players)
		return
	}

	states := [2]persist.State{s.plannedState(players[0], plans[0]), s.plannedState(players[1], plans[1])}
	escrow := tradeTransfers(t, false)
	for i, player := range players {
		plan, _ := s.planTransfer(player, escrow[i])
		s.commitTransfer(player, plan)
		t.Sides[i].EscrowCoins = t.Sides[i].Coins
		t.Sides[i].Escrow = t.Sides[i].Escrow[:0]
		for _, line := range t.Sides[i].Lines {
			t.Sides[i].Escrow = append(t.Sides[i].Escrow, Stack{ItemID: line.ItemID, CID: line.CID, Qty: line.Qty})
		}
	}
	t.State = TradeCommitting
	members := []persist.LinkMember{
		{PlayerID: string(t.Sides[0].PlayerID), State: states[0]},
		{PlayerID: string(t.Sides[1].PlayerID), State: states[1]},
	}
	if s.Durability == nil || !s.Durability.Link(t.ID, "trade", members, persist.Effect{}) {
		s.returnEscrow(t, players)
		t.State = TradeOpen
		s.reopenTrade(t, players, tradeUnsaved)
		return
	}
	for _, player := range players {
		s.sendTradeAck(player, "commit", "")
	}
	logx.Infof("[ECONOMY] trade %s committing: %s gives %d stack(s) + %d coins, %s gives %d stack(s) + %d coins, economy=%s",
		t.ID, t.Sides[0].PlayerID, len(t.Sides[0].Lines), t.Sides[0].Coins,
		t.Sides[1].PlayerID, len(t.Sides[1].Lines), t.Sides[1].Coins, s.EconomyKey())
}

// escrowGoods are the goods in escrow of a side.
func escrowGoods(side TradeSide) Goods {
	return Goods{Coins: int64(side.EscrowCoins), Stacks: side.Escrow}
}

// returnEscrow gives each live side its own escrow back.
func (s *Content) returnEscrow(t *Trade, players [2]*world.Session) {
	t.State = TradeOpen
	for i, player := range players {
		if player != nil && !t.Sides[i].Gone {
			s.receive(player, escrowGoods(t.Sides[i]))
		}
		t.Sides[i].Escrow, t.Sides[i].EscrowCoins = nil, 0
	}
}

// finishTrade ends a trade whose result the world holds.
func (s *Content) finishTrade(t *Trade, players [2]*world.Session) {
	for i, player := range players {
		delete(s.trades, t.Sides[i].ID)
		if player == nil || t.Sides[i].Gone {
			continue
		}
		s.AdvancePlayerQuestsOnGain(player)
		s.sendTradeAck(player, "close", tradeDone)
	}
	s.recordEconomy("trade", "")
	logx.Infof("[ECONOMY] trade %s done: %s ↔ %s", t.ID, t.Sides[0].PlayerID, t.Sides[1].PlayerID)
}

// settleTrade applies the outcome of a committing trade once the engine
// answered: the escrow of each side goes to the other, or back to it.
func (s *Content) settleTrade(t *Trade) {
	outcome, _ := s.Durability.LinkOutcome(t.ID)
	if outcome == persist.Pending {
		return
	}
	players := [2]*world.Session{s.Sessions[t.Sides[0].ID], s.Sessions[t.Sides[1].ID]}
	escrows := t.Sides
	if outcome == persist.Applied {
		t.State = TradeOpen
		for i, player := range players {
			if player != nil && !t.Sides[i].Gone {
				s.receive(player, escrowGoods(escrows[1-i]))
			}
		}
	} else {
		s.returnEscrow(t, players)
	}
	for i, player := range players {
		if player != nil && !t.Sides[i].Gone {
			s.CaptureNow(player, false)
		}
	}
	s.Durability.Unlink(t.ID)
	if outcome == persist.Applied {
		s.finishTrade(t, players)
		return
	}
	for i, side := range t.Sides {
		delete(s.trades, side.ID)
		if players[i] != nil && !side.Gone {
			s.sendTradeAck(players[i], "close", tradeUnsaved)
		}
	}
	s.recordEconomy("trade", tradeUnsaved)
}

// UpdateTrades runs the timed side of the trades a player of the map leads:
// the outcome of a commit, the expiry of a request or an idle trade, and the
// end of a trade whose sides died, moved apart or left. Runs in phaseLifecycle.
func (s *Content) UpdateTrades(mapState *world.MapState, now time.Time) {
	for player := range s.PlayersOn(mapState) {
		t, side := s.tradeOf(player)
		if t == nil || (side == 1 && !t.Sides[0].Gone) {
			continue
		}
		if t.State == TradeCommitting {
			s.settleTrade(t)
			continue
		}
		partner := s.Sessions[t.Sides[1-side].ID]
		switch {
		case now.After(t.ExpiresAt):
			s.closeTrade(t, tradeExpired)
		case partner == nil || t.Sides[1-side].Gone:
			s.closeTrade(t, tradeLeft)
		case s.Vt(player.ID, player.MapCode).Ghost() || s.Vt(partner.ID, partner.MapCode).Ghost():
			s.closeTrade(t, RefuseDead)
		case s.VisiblePlayer(player, partner.ID) == nil:
			s.closeTrade(t, RefuseOutOfRange)
		}
	}
}

// TradeFinalStates is the final durable state of a player that leaves while
// its trade commits, for each outcome: with the escrow of the other side, and
// with its own escrow back. False when no trade of the player commits. The
// side leaves the trade either way.
func (s *Content) TradeFinalStates(player *world.Session) (persist.State, persist.State, bool) {
	t, side := s.tradeOf(player)
	if t == nil {
		return persist.State{}, persist.State{}, false
	}
	if t.State != TradeCommitting {
		s.closeTrade(t, tradeLeft)
		return persist.State{}, persist.State{}, false
	}
	own, other := t.Sides[side], t.Sides[1-side]
	t.Sides[side].Gone = true
	delete(s.trades, player.ID)
	return s.receivedState(player, escrowGoods(other)), s.receivedState(player, escrowGoods(own)), true
}

// TradeView is the trade of a player in the self block of the snapshot.
type TradeView struct {
	PartnerID   world.EntityID `json:"partnerId"`
	PartnerName string         `json:"partnerName"`
	// State is requested, incoming, open or committing.
	State       string         `json:"state"`
	Revision    uint32         `json:"revision"`
	Mine        TradeOfferView `json:"mine"`
	Theirs      TradeOfferView `json:"theirs"`
	ExpiresInMs int64          `json:"expiresInMs"`
}

// TradeOfferView is one offer of a trade.
type TradeOfferView struct {
	Items     []TradeItemView `json:"items"`
	Coins     int             `json:"coins"`
	Confirmed bool            `json:"confirmed"`
}

// TradeItemView is one offered stack. Slot names the bag slot of an own line.
type TradeItemView struct {
	ItemID   string `json:"itemId"`
	CID      string `json:"cid"`
	Quantity int    `json:"quantity"`
	Slot     int    `json:"slot"`
}

func offerView(side TradeSide, own bool) TradeOfferView {
	view := TradeOfferView{Coins: side.Coins, Confirmed: side.Confirmed, Items: make([]TradeItemView, 0, len(side.Lines))}
	for _, line := range side.Lines {
		slot := -1
		if own {
			slot = line.Slot
		}
		view.Items = append(view.Items, TradeItemView{ItemID: line.ItemID, CID: line.CID, Quantity: line.Qty, Slot: slot})
	}
	return view
}

// PlayerTrade is the trade of the player for its snapshot, or nil.
func (s *Content) PlayerTrade(player *world.Session) *TradeView {
	t, side := s.tradeOf(player)
	if t == nil {
		return nil
	}
	state := tradeStateWords[t.State]
	if t.State == TradeRequested && side == 1 {
		state = "incoming"
	}
	return &TradeView{
		PartnerID:   t.Sides[1-side].ID,
		PartnerName: t.Sides[1-side].Name,
		State:       state,
		Revision:    t.Revision,
		Mine:        offerView(t.Sides[side], true),
		Theirs:      offerView(t.Sides[1-side], false),
		ExpiresInMs: max(0, time.Until(t.ExpiresAt).Milliseconds()),
	}
}
