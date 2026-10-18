package net

import (
	"encoding/json"
	"regexp"

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

	Quantity    int `json:"quantity"`    // shop_buy
	RecipeIndex int `json:"recipeIndex"` // craft_item

	From     string `json:"from"` // item_move
	To       string `json:"to"`
	FromSlot int    `json:"fromSlot"`
	ToSlot   int    `json:"toSlot"`
	Qty      int    `json:"qty"`
}

// eventsPayload is the payload of "events", the one uplink input message.
type eventsPayload struct {
	Events []Message `json:"events"`
}

// receiveMessage is the single client → server dispatch point once a player is
// placed: every message is one batch of client events. The batch becomes one
// InputCommand per event and is enqueued whole; phaseInput applies each
// command once.
//
// A protocol violation evicts: a message that is not a batch, a batch over
// maxInputQueue, or an event without a seq. An event that fails
// validation costs a strike and is still enqueued as InputKindUnknown, so its
// seq is consumed.
func (c *Client) receiveMessage(pack []byte, server *GameServer) {
	// Rate limit first: an over-budget frame costs no parsing work.
	if allowed, evict := c.limiter.allow(); !allowed {
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
	for _, ev := range batch.Events {
		// The player closes its session: no later event of the batch counts.
		if ev.Type == "session_leave" {
			var leaving struct {
				Travel string `json:"travel"`
			}
			_ = json.Unmarshal(ev.Payload, &leaving)
			c.dispatchInputs(server, cmds)
			go server.leave(c, leaving.Travel)
			return
		}
		var p inputPayload
		if err := json.Unmarshal(ev.Payload, &p); err != nil || p.Seq == 0 {
			c.evict(server, "event without seq")
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
	c.dispatchInputs(server, cmds)
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
		if p.EntityID == 0 || p.RecipeIndex < 0 {
			return world.InputCommand{}, false
		}
		cmd.EntityID = p.EntityID
		cmd.RecipeIndex = p.RecipeIndex
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
// not fit evicts the client.
func (c *Client) dispatchInputs(server *GameServer, cmds []world.InputCommand) {
	fits := true
	server.Mu.Lock()
	if player := server.Sessions[c.playerID]; player != nil {
		fits = world.EnqueueInputs(player, cmds)
	}
	server.Mu.Unlock()
	if !fits {
		c.evict(server, "input queue full")
	}
}
