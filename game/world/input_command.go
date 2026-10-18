// Package world — input_command.go
//
// InputCommand is one client event: the unit of client→server input.
//
// One "events" message carries many commands, in sequence order. Each one is
// the envelope {"type", "payload"}; receiveMessage maps the inner type word to
// an InputKind. Every payload carries seq, frame and timestamp. Every seq is
// consumed: an event that fails validation is enqueued as InputKindUnknown, so
// phaseInput moves the cursor past it and InputConsumedThrough lands on the
// last seq of each batch. phaseInput applies a seq once: one at or below the
// cursor is dropped.
//
// Ownership:
//   - Built and enqueued by handlers.go (per-WS-goroutine).
//   - Consumed exclusively by phaseInput in simulation_phases.go.
//   - Session.InputQueue is the single rendezvous; no other code path
//     mutates entity state in response to client input.

package world

// InputKind enumerates the input categories the server accepts. It is
// internal — only receiveMessage knows which wire word maps to which kind.
type InputKind uint8

const (
	// InputKindUnknown is an event that failed validation: consumed, never applied.
	InputKindUnknown      InputKind = iota
	InputKindPlayerAction           // tap move + skill trigger
	InputKindItemActivation
	InputKindPlayerStasis // client modal count > 0 — freeze, else thaw
	InputKindChat
	InputKindTalkDone      // dialogue read to the end — advance talk objectives
	InputKindQuestAbandon  // drop an active quest — moves it to failed
	InputKindQuestAccept   // explicitly accept the NPC's offered quest
	InputKindShopBuy       // buy one catalog item from a vendor action
	InputKindCraftItem     // assemble one recipe at an assembler action
	InputKindCraftCancel   // abort the running assembly and refund it
	InputKindItemMove      // move units of one stack to a slot of the bag or of a container
	InputKindPartyInvite   // invite a visible player to the party
	InputKindPartyReply    // accept or decline a pending invite
	InputKindPartyLeave    // leave the party
	InputKindPartyKick     // the leader removes a member
	InputKindPartyDisband  // the leader ends the party
	InputKindFriendList    // ask for the friend list
	InputKindFriendRequest // ask a visible account player for friendship
	InputKindFriendReply   // accept or decline a friend request
	InputKindFriendRemove  // end a friendship
	InputKindStatusNote    // set or clear the status note of the account
)

// ChatChannel is who one chat line reaches.
type ChatChannel uint8

const (
	ChatWorld  ChatChannel = iota // every player on this server
	ChatParty                     // the members of the sender's party
	ChatDirect                    // one player on the sender's map
)

var chatChannelWords = [...]string{ChatWorld: "world", ChatParty: "party", ChatDirect: "direct"}

// String is the wire word of the channel.
func (c ChatChannel) String() string { return chatChannelWords[c] }

// ParseChatChannel maps a wire word to its channel.
func ParseChatChannel(word string) (ChatChannel, bool) {
	for c, w := range chatChannelWords {
		if w == word {
			return ChatChannel(c), true
		}
	}
	return 0, false
}

// InputCommand is the unit of client→server input.
type InputCommand struct {
	Kind      InputKind
	Sequence  uint32  // monotonic per-client sequence number
	Frame     uint32  // client fixed-step count at push. Not a server tick.
	Timestamp float64 // client GetTime() at push. No reader.
	// Payload fields — only the ones relevant to Kind are populated.
	TargetX     float64     // PlayerAction
	TargetY     float64     // PlayerAction
	ItemID      string      // ItemActivation, ShopBuy
	Slot        int         // ItemActivation — the bag slot of the stack, or -1 for the first stack of ItemID
	Active      bool        // ItemActivation; PlayerStasis — the stasis bool; PartyReply — accept
	ChatChannel ChatChannel // Chat
	Text        string      // Chat; StatusNote — clean, empty clears the note
	EntityID    EntityID    // Chat direct target; party target; TalkDone, ShopBuy, CraftItem, ItemMove to a vault — the NPC entity
	DialogCode  string      // TalkDone — the dialogue group the player just read
	Quantity    int         // ShopBuy — units (clamped server-side)
	RecipeIndex int         // CraftItem — index into the action's craftRecipes
	Move        ItemMove    // ItemMove
	// FriendshipID names the friendship of a FriendReply or FriendRemove.
	FriendshipID string
}

// MaxInputQueue bounds one player's queue between two ticks. It equals the
// client CLIENT_EVENT_CAP, so one batch from a correct client always fits.
const MaxInputQueue = 512

// EnqueueInputs appends one batch to the player's InputQueue, whole, so one
// phaseInput drains it. Called under the world mutex. It never drops: a batch
// that does not fit returns false, and the caller evicts the client.
func EnqueueInputs(p *Session, cmds []InputCommand) bool {
	if len(p.InputQueue)+len(cmds) > MaxInputQueue {
		return false
	}
	p.InputQueue = append(p.InputQueue, cmds...)
	return true
}

// Box is the container a slot of an item move belongs to.
type Box uint8

const (
	BoxBag     Box = iota // the carried inventory
	BoxStorage            // the personal storage
	BoxVault              // the NPC vault at the terminal of the move
)

var boxWords = [...]string{BoxBag: "bag", BoxStorage: "storage", BoxVault: "vault"}

// ParseBox maps a wire word to its box.
func ParseBox(word string) (Box, bool) {
	for b, w := range boxWords {
		if w == word {
			return Box(b), true
		}
	}
	return 0, false
}

// WalletSlot is the bag slot of the coin: the wallet takes no cell.
const WalletSlot = -1

// ItemMove moves Qty units of the stack in FromSlot of From to ToSlot of To.
type ItemMove struct {
	From, To         Box
	FromSlot, ToSlot int
	Qty              int
}
