package net

import (
	"encoding/json"
	"regexp"
	"slices"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/sim"
	"cyberia-server/game/social"
	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// OLMeta is the JSON shape sent to the client for each ObjectLayer: the cid of the definition
// bound to the label, and its content. Matches populate_object_layer_from_json.
type OLMeta struct {
	Cid  string                  `json:"cid"`
	Data content.ObjectLayerData `json:"data"`
}

// buildSkillMap returns a compact { triggerItemId → [SkillMapEntry] } map
// derived from the server's skillConfig — sent to clients in init_data.
func (s *GameServer) buildSkillMap() map[string][]SkillMapEntry {
	out := make(map[string][]SkillMapEntry, len(s.SkillConfig))
	for triggerID, defs := range s.SkillConfig {
		out[triggerID] = skillEntries(defs)
	}
	return out
}

// skillEntries is the client shape of the skills of one trigger item; empty when it has none.
func skillEntries(defs []content.SkillDefinition) []SkillMapEntry {
	entries := make([]SkillMapEntry, 0, len(defs))
	for _, def := range defs {
		entries = append(entries, SkillMapEntry{
			LogicEventID:         def.LogicEventID,
			Name:                 def.Name,
			Description:          def.Description,
			SummonedEntityItemID: def.SummonedEntityItemID,
			Held:                 sim.IsHeldSkill(def.LogicEventID),
		})
	}
	return entries
}

// olMetadata is the objectLayers map of a metadata message.
func olMetadata(layers map[string]*content.ObjectLayer) map[string]*OLMeta {
	out := make(map[string]*OLMeta, len(layers))
	for itemID, ol := range layers {
		out[itemID] = &OLMeta{Cid: ol.Cid, Data: ol.Data}
	}
	return out
}

// sendObjectLayerChanges sends a player the definitions that changed since the
// revision its client holds, and their skills. Caller holds s.Mu.
func (s *GameServer) sendObjectLayerChanges(player *world.Session) {
	client := s.clients[player.ID]
	if client == nil || s.ObjectLayerRevision() == client.olSeen {
		return
	}
	changed, revision := s.ObjectLayersSince(client.olSeen)
	client.olSeen = revision
	if len(changed) > 0 {
		skills := make(map[string][]SkillMapEntry, len(changed))
		for itemID := range changed {
			skills[itemID] = skillEntries(s.SkillConfig[itemID])
		}
		s.recordDefinitionsShared(len(changed))
		world.SendMessage(player, "metadata", map[string]any{"objectLayers": olMetadata(changed), "skillMap": skills})
	}
}

// sendEconomyChange sends a player the economy the world runs once its
// revision differs from the one the client holds. Caller holds s.Mu.
func (s *GameServer) sendEconomyChange(player *world.Session) {
	client := s.clients[player.ID]
	if client == nil || s.EconomyKey() == client.economySeen {
		return
	}
	client.economySeen = s.EconomyKey()
	world.SendMessage(player, "metadata", map[string]any{"economy": s.EconomyView()})
}

// sendWorldChange tells a player the world was rebuilt since the revision its
// client holds: its map views read the content again. Caller holds s.Mu.
func (s *GameServer) sendWorldChange(player *world.Session) {
	client := s.clients[player.ID]
	if client == nil || s.WorldRevision() == client.worldSeen {
		return
	}
	client.worldSeen = s.WorldRevision()
	world.SendMessage(player, "metadata", map[string]any{"worldRevision": client.worldSeen})
}

// Send packs one message and queues it on the socket.
func (c *Client) Send(msgType string, payload any) {
	if c.sock == nil {
		return
	}
	pack, err := Pack(msgType, payload)
	if err != nil {
		logx.Errorf("[sendMessage] pack %q failed: %v", msgType, err)
		return
	}
	if !c.sock.Send(pack) {
		logx.Debugf("Client %s queue full — dropped %q.", c.playerID, msgType)
	}
}

// readPump runs the client read loop until the connection fails.
func (c *Client) readPump(server *GameServer) {
	defer func() {
		if r := recover(); r != nil {
			logx.Errorf("[readPump] PANIC player=%s: %v", c.playerID, r)
		}
		logx.Debugf("[readPump] closing player=%s", c.playerID)
		server.recordWsDisconnect()
		// Never block here. A full unregister queue must not pin this
		// goroutine, so fall back to tearing the client down directly.
		select {
		case server.unregister <- c:
		default:
			server.detachClient(c)
		}
	}()
	c.sock.Receive(func(pack []byte) { c.receiveMessage(pack, server) })
}

// inputKinds maps the inner type word of a client event to the internal input
// kind. The kind enum stays internal; only this table knows the wire words.
var inputKinds = map[string]world.InputKind{
	"player_action": world.InputKindPlayerAction,
	"item_active":   world.InputKindItemActivation,
	"player_stasis": world.InputKindPlayerStasis,
	"chat":          world.InputKindChat,
	"talk_done":     world.InputKindTalkDone,
	"quest_abandon": world.InputKindQuestAbandon,
	"quest_accept":  world.InputKindQuestAccept,
	"shop_buy":      world.InputKindShopBuy,
	"craft_item":    world.InputKindCraftItem,
	"craft_cancel":  world.InputKindCraftCancel,
	"item_move":     world.InputKindItemMove,
	"party_invite":  world.InputKindPartyInvite,
	"party_reply":   world.InputKindPartyReply,
	"party_leave":   world.InputKindPartyLeave,
	"party_kick":    world.InputKindPartyKick,
	"party_disband": world.InputKindPartyDisband,

	"friends_list":   world.InputKindFriendList,
	"friend_request": world.InputKindFriendRequest,
	"friend_reply":   world.InputKindFriendReply,
	"friend_remove":  world.InputKindFriendRemove,
	"status_note":    world.InputKindStatusNote,

	"item_drop":     world.InputKindItemDrop,
	"item_quote":    world.InputKindItemQuote,
	"item_sell":     world.InputKindItemSell,
	"item_burn":     world.InputKindItemBurn,
	"trade_request": world.InputKindTradeRequest,
	"trade_reply":   world.InputKindTradeReply,
	"trade_offer":   world.InputKindTradeOffer,
	"trade_confirm": world.InputKindTradeConfirm,
	"trade_cancel":  world.InputKindTradeCancel,

	"market_open":    world.InputKindMarketOpen,
	"market_list":    world.InputKindMarketList,
	"market_cancel":  world.InputKindMarketCancel,
	"market_buy":     world.InputKindMarketBuy,
	"market_bid":     world.InputKindMarketBid,
	"market_collect": world.InputKindMarketCollect,
}

// inputPayload holds every client event payload field. Each event type fills
// the subset it needs; the rest stay zero.
type inputPayload struct {
	Seq       uint32  `json:"seq"`
	Frame     uint32  `json:"frame"`
	Timestamp float64 `json:"timestamp"`

	X float64 `json:"x"` // player_action
	Y float64 `json:"y"`

	ItemID string `json:"itemId"` // item_active, shop_buy
	Slot   int    `json:"slot"`   // item_active
	Active bool   `json:"active"` // item_active

	Stasis bool `json:"stasis"` // player_stasis

	Channel string         `json:"channel"` // chat
	ToID    world.EntityID `json:"toId"`    // chat direct, party_invite, party_kick, friend_request
	Text    string         `json:"text"`    // chat, status_note

	FromID world.EntityID `json:"fromId"` // party_reply
	Accept bool           `json:"accept"` // party_reply, friend_reply

	FriendshipID string `json:"friendshipId"` // friend_reply, friend_remove

	EntityID   world.EntityID `json:"entityId"`   // talk_done, quest_accept, shop_buy, item_move to a vault
	DialogCode string         `json:"dialogCode"` // talk_done
	QuestCode  string         `json:"questCode"`  // quest_*

	Quantity    int `json:"quantity"`    // shop_buy, item_drop, item_sell, item_burn
	RecipeIndex int `json:"recipeIndex"` // craft_item

	From     string `json:"from"` // item_move
	To       string `json:"to"`
	FromSlot int    `json:"fromSlot"`
	ToSlot   int    `json:"toSlot"`
	Qty      int    `json:"qty"`

	Box     string `json:"box"`     // item_drop, item_quote: the stack's box and slot
	Quote   int64  `json:"quote"`   // craft_item: the coins the player saw
	Kind    string `json:"kind"`    // item_quote: sell | burn
	QuoteID string `json:"quoteId"` // item_sell, item_burn

	Items    []offerLine `json:"items"`    // trade_offer
	Coins    int         `json:"coins"`    // trade_offer
	Revision uint32      `json:"revision"` // trade_confirm

	ListingID string `json:"listingId"` // market_cancel, market_buy, market_bid
	Mode      string `json:"mode"`      // market_list; market_open: the listings of one mode
	Price     int    `json:"price"`     // market_list, market_bid
	Hours     int    `json:"hours"`     // market_list
	After     string `json:"after"`     // market_open
}

// offerLine is one bag stack of a trade_offer.
type offerLine struct {
	Slot int `json:"slot"`
	Qty  int `json:"quantity"`
}

// eventsPayload is the payload of "events", the one uplink input message.
type eventsPayload struct {
	Events []Message `json:"events"`
}

const (
	// resendInterval spaces the input_resend requests of one gap.
	resendInterval = 250 * time.Millisecond
	// gapWait bounds how long a gap holds the stream: a client that never
	// sends the events again loses them, not every event after them.
	gapWait = time.Second
)

// inputResend asks the client to send its events again from From: the server
// queued every event before it and none after it.
type inputResend struct {
	From uint32 `json:"from"`
}

// receiveMessage is the single client → server dispatch point once a player is
// placed: every message is one batch of client events. The batch becomes one
// InputCommand per event and is enqueued whole; phaseInput applies each
// command once.
//
// The events of a connection form one sequence. An event at or below the last
// queued one repeats it and is skipped. An event past the next one means a
// batch never arrived, which the rate limit causes: the server refuses the
// batch and asks for the events again from the gap (input_resend), because
// applying them past it would lose the missing ones for good. A gap no resend
// fills within gapWait is skipped.
//
// A protocol violation evicts: a message that is not a batch, a batch over
// maxInputQueue, or an event without a seq. An event that fails
// validation costs a strike and is still enqueued as InputKindUnknown, so its
// seq is consumed.
func (c *Client) receiveMessage(pack []byte, server *GameServer) {
	// Rate limit first: an over-budget frame costs no parsing work.
	if allowed, evict := c.limiter.allow(); !allowed {
		c.dropped = true
		server.recordWsRateLimited()
		if evict {
			c.evict(server, "input rate limit")
		}
		return
	}

	msg, err := Unpack(pack)
	var batch eventsPayload
	if err != nil || msg.Type != "events" || json.Unmarshal(msg.Payload, &batch) != nil {
		c.evict(server, "bad message")
		return
	}
	if len(batch.Events) > world.MaxInputQueue {
		c.evict(server, "event batch too large")
		return
	}

	cmds := make([]world.InputCommand, 0, len(batch.Events))
	next := c.queuedThrough + 1
	for _, ev := range batch.Events {
		var p inputPayload
		if err := json.Unmarshal(ev.Payload, &p); err != nil || p.Seq == 0 {
			c.evict(server, "event without seq")
			return
		}
		if p.Seq < next {
			continue
		}
		if p.Seq > next {
			if c.resendFrom != next || time.Since(c.gapSince) < gapWait {
				c.askResend(server, next)
				return
			}
			logx.Warnf("[input] player=%s: no resend filled the gap %d..%d; skipped", c.playerID, next, p.Seq-1)
			server.recordInputGapSkipped()
			next = p.Seq
		}
		next++
		// The player closes its session: no later event of the batch counts.
		if ev.Type == "session_leave" {
			var leaving struct {
				Travel string `json:"travel"`
			}
			_ = json.Unmarshal(ev.Payload, &leaving)
			c.queue(server, cmds, p.Seq)
			go server.leave(c, leaving.Travel)
			return
		}
		cmd, ok := parseInput(ev.Type, &p)
		if !ok {
			logx.Debugf("Invalid %q event from player %s", ev.Type, c.playerID)
			if c.limiter.strike() {
				c.evict(server, "repeated protocol violations")
				return
			}
			cmd = world.InputCommand{Kind: world.InputKindUnknown}
		}
		cmd.Sequence, cmd.Frame, cmd.Timestamp = p.Seq, p.Frame, p.Timestamp
		cmds = append(cmds, cmd)
	}
	c.queue(server, cmds, next-1)
}

// askResend sends input_resend from the first missing event, once per gap
// and resendInterval, so a burst of refused batches asks once. TCP keeps the
// order, so a gap no dropped message explains is a protocol violation.
func (c *Client) askResend(server *GameServer, from uint32) {
	if !c.dropped && c.limiter.strike() {
		c.evict(server, "repeated sequence gaps")
		return
	}
	now := time.Now()
	if c.resendFrom != from {
		c.gapSince = now
	} else if now.Sub(c.resendAt) < resendInterval {
		return
	}
	c.resendFrom, c.resendAt = from, now
	server.recordInputResend()
	c.Send("input_resend", inputResend{From: from})
}

// queue enqueues the commands of a batch and records the last sequence it
// consumed, through.
func (c *Client) queue(server *GameServer, cmds []world.InputCommand, through uint32) {
	if c.dispatchInputs(server, cmds) {
		c.queuedThrough, c.dropped = through, false
	}
}

// parseInput validates one event payload and builds its command. False means
// the type word is unknown or the payload fails validation.
func parseInput(msgType string, p *inputPayload) (world.InputCommand, bool) {
	kind, known := inputKinds[msgType]
	if !known {
		return world.InputCommand{}, false
	}
	cmd := world.InputCommand{Kind: kind}
	switch kind {
	case world.InputKindPlayerAction:
		// A tap target reaches the pathfinder directly. Reject anything that
		// is not a finite, plausible coordinate before it costs tick time.
		if !sim.ValidTapTarget(p.X, p.Y) {
			return world.InputCommand{}, false
		}
		cmd.TargetX = p.X
		cmd.TargetY = p.Y
	case world.InputKindItemActivation:
		if p.ItemID == "" || !sim.ValidIdentifier(p.ItemID) || p.Slot < world.WalletSlot || p.Slot >= content.StorageMaxSlots {
			return world.InputCommand{}, false
		}
		cmd.ItemID = p.ItemID
		cmd.Slot = p.Slot
		cmd.Active = p.Active
	case world.InputKindPlayerStasis:
		cmd.Active = p.Stasis
	case world.InputKindChat:
		channel, known := world.ParseChatChannel(p.Channel)
		text := social.CleanText(p.Text, content.ChatTextMaxRunes)
		if !known || text == "" || (channel == world.ChatDirect) != (p.ToID != 0) {
			return world.InputCommand{}, false
		}
		cmd.ChatChannel = channel
		cmd.EntityID = p.ToID
		cmd.Text = text
	case world.InputKindTalkDone:
		if p.EntityID == 0 || !sim.ValidIdentifier(p.DialogCode) {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.DialogCode = p.DialogCode
	case world.InputKindQuestAbandon:
		if p.QuestCode == "" || !sim.ValidIdentifier(p.QuestCode) {
			return world.InputCommand{}, false
		}
		cmd.ItemID = p.QuestCode
	case world.InputKindQuestAccept:
		if p.EntityID == 0 || p.QuestCode == "" || !sim.ValidIdentifier(p.QuestCode) {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.QuestCode
	case world.InputKindShopBuy:
		if p.EntityID == 0 || p.ItemID == "" || !sim.ValidIdentifier(p.ItemID) {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.ItemID = p.ItemID
		cmd.Quantity = sim.ClampQuantity(p.Quantity)
	case world.InputKindCraftItem:
		if p.EntityID == 0 || p.RecipeIndex < 0 || p.Quote < 0 {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.RecipeIndex = p.RecipeIndex
		cmd.Quote = p.Quote
	case world.InputKindItemDrop:
		box, known := world.ParseBox(p.Box)
		if !known || box == world.BoxVault || p.Slot < 0 || p.Slot >= content.StorageMaxSlots || p.Quantity < 1 ||
			p.EntityID != 0 || !sim.ValidTapTarget(p.X, p.Y) {
			return world.InputCommand{}, false
		}
		cmd.Move = world.ItemMove{From: box, FromSlot: p.Slot, Qty: sim.ClampQuantity(p.Quantity)}
		cmd.TargetX, cmd.TargetY = p.X, p.Y
	case world.InputKindItemQuote:
		box, known := world.ParseBox(p.Box)
		if !known || box == world.BoxVault || p.Slot < 0 || p.Slot >= content.StorageMaxSlots || p.EntityID == 0 ||
			(p.Kind != content.QuoteSell && p.Kind != content.QuoteBurn) {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.Move = world.ItemMove{From: box, FromSlot: p.Slot}
		cmd.QuoteKind = p.Kind
	case world.InputKindMarketOpen:
		if p.EntityID == 0 || !sim.ValidIdentifier(p.ItemID) || !sim.ValidIdentifier(p.After) ||
			(p.Mode != "" && !slices.Contains(content.MarketModes, p.Mode)) {
			return world.InputCommand{}, false
		}
		cmd.EntityID, cmd.ItemID = p.EntityID, p.ItemID
		cmd.Market.After, cmd.Market.Mode = p.After, p.Mode
	case world.InputKindMarketList:
		box, known := world.ParseBox(p.Box)
		if !known || box == world.BoxVault || p.Slot < 0 || p.Slot >= content.StorageMaxSlots || p.Quantity < 1 ||
			p.EntityID == 0 || p.Mode == "" || !sim.ValidIdentifier(p.Mode) || p.Price < 1 || p.Hours < 1 {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.Move = world.ItemMove{From: box, FromSlot: p.Slot, Qty: sim.ClampQuantity(p.Quantity)}
		cmd.Market = world.MarketInput{Mode: p.Mode, Price: p.Price, Hours: p.Hours}
	case world.InputKindMarketCancel, world.InputKindMarketBuy, world.InputKindMarketBid:
		if p.EntityID == 0 || p.ListingID == "" || !sim.ValidIdentifier(p.ListingID) ||
			(cmd.Kind == world.InputKindMarketBid && p.Price < 1) {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.Market = world.MarketInput{ListingID: p.ListingID, Price: p.Price}
	case world.InputKindMarketCollect:
		if p.EntityID == 0 {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
	case world.InputKindItemSell, world.InputKindItemBurn:
		if p.QuoteID == "" || !sim.ValidIdentifier(p.QuoteID) || p.Quantity < 1 {
			return world.InputCommand{}, false
		}
		cmd.QuoteID = p.QuoteID
		cmd.Move = world.ItemMove{Qty: sim.ClampQuantity(p.Quantity)}
	case world.InputKindTradeRequest:
		if p.ToID == 0 {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.ToID
	case world.InputKindTradeReply:
		if p.FromID == 0 {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.FromID
		cmd.Active = p.Accept
	case world.InputKindTradeOffer:
		if len(p.Items) > content.TradeOfferMaxStacks || p.Coins < 0 {
			return world.InputCommand{}, false
		}
		for _, line := range p.Items {
			if line.Slot < 0 || line.Slot >= content.StorageMaxSlots || line.Qty < 1 {
				return world.InputCommand{}, false
			}
			cmd.Offer = append(cmd.Offer, world.OfferLine{Slot: line.Slot, Qty: sim.ClampQuantity(line.Qty)})
		}
		cmd.Coins = p.Coins
	case world.InputKindTradeConfirm:
		cmd.Revision = p.Revision
	case world.InputKindItemMove:
		move, ok := parseItemMove(p)
		if !ok {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.Move = move
	case world.InputKindPartyInvite, world.InputKindPartyKick:
		if p.ToID == 0 {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.ToID
	case world.InputKindPartyReply:
		if p.FromID == 0 {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.FromID
		cmd.Active = p.Accept
	case world.InputKindFriendRequest:
		if p.ToID == 0 {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.ToID
	case world.InputKindFriendReply, world.InputKindFriendRemove:
		if !friendshipIDPattern.MatchString(p.FriendshipID) {
			return world.InputCommand{}, false
		}
		cmd.FriendshipID = p.FriendshipID
		cmd.Active = p.Accept
	case world.InputKindStatusNote:
		cmd.Text = p.Text
	}
	return cmd, true
}

// friendshipIDPattern is the shape of a friendship id: a MongoDB ObjectId.
var friendshipIDPattern = regexp.MustCompile(`^[0-9a-f]{24}$`)

// parseItemMove validates one move at the trust boundary: both places name a
// known box and a slot in range, the wallet slot belongs to the bag, a vault
// move names its terminal, and a move joins the bag and at most one container.
func parseItemMove(p *inputPayload) (world.ItemMove, bool) {
	from, knownFrom := world.ParseBox(p.From)
	to, knownTo := world.ParseBox(p.To)
	inRange := func(box world.Box, slot int) bool {
		return (slot >= 0 && slot < content.StorageMaxSlots) || (box == world.BoxBag && slot == world.WalletSlot)
	}
	vault := from == world.BoxVault || to == world.BoxVault
	if !knownFrom || !knownTo || !inRange(from, p.FromSlot) || !inRange(to, p.ToSlot) || p.Qty < 1 ||
		vault != (p.EntityID != 0) || (from != world.BoxBag && to != world.BoxBag && from != to) {
		return world.ItemMove{}, false
	}
	return world.ItemMove{From: from, To: to, FromSlot: p.FromSlot, ToSlot: p.ToSlot, Qty: sim.ClampQuantity(p.Qty)}, true
}

// evict closes an abusive connection. The read loop ends on the closed socket
// and readPump runs the normal teardown.
func (c *Client) evict(server *GameServer, reason string) {
	server.recordWsEvicted()
	logx.Warnf("[evict] player=%s ip=%s: %s", c.playerID, c.ip, reason)
	c.sock.Close()
}

// dispatchInputs enqueues one batch on the player's per-tick input queue.
// phaseInput drains and applies each command exactly once. A batch that does
// not fit evicts the client; the result reports whether it was queued.
func (c *Client) dispatchInputs(server *GameServer, cmds []world.InputCommand) bool {
	fits := true
	server.Mu.Lock()
	if player := server.Sessions[c.playerID]; player != nil {
		fits = world.EnqueueInputs(player, cmds)
	}
	server.Mu.Unlock()
	if !fits {
		c.evict(server, "input queue full")
	}
	return fits
}
