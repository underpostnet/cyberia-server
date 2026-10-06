// Package game — input_command.go
//
// InputCommand is one client event: the unit of client→server input.
//
// One "events" message carries many commands, in sequence order. Each one is
// the envelope {"type", "payload"}; receiveMessage maps the inner type word to
// an InputKind. Every payload carries seq, frame and timestamp. Every seq is
// consumed: an event that fails validation is enqueued as InputKindUnknown, so
// phaseInput moves the cursor past it and InputConsumedThrough lands on the
// last seq of each batch.
//
// Ownership:
//   - Built and enqueued by handlers.go (per-WS-goroutine).
//   - Consumed exclusively by phaseInput in simulation_phases.go.
//   - PlayerState.InputQueue is the single rendezvous; no other code path
//     mutates entity state in response to client input.

package game

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
	InputKindTalkDone     // dialogue read to the end — advance talk objectives
	InputKindQuestAbandon // drop an active quest — moves it to failed
	InputKindQuestAccept  // explicitly accept the NPC's offered quest
	InputKindShopBuy      // buy one catalog item from a vendor action
	InputKindCraftItem    // assemble one recipe at an assembler action
	InputKindCraftCancel  // abort the running assembly and refund it
	InputKindStorageOpen  // bind a storage vault and read its contents
	InputKindItemOps      // replay one session's deposits and withdrawals
)

// InputCommand is the unit of client→server input.
type InputCommand struct {
	Kind      InputKind
	Sequence  uint32  // monotonic per-client sequence number
	Frame     uint32  // client fixed-step count at push. Not a server tick.
	Timestamp float64 // client GetTime() at push. No reader.
	// Payload fields — only the ones relevant to Kind are populated.
	TargetX     float64  // PlayerAction
	TargetY     float64  // PlayerAction
	ItemID      string   // ItemActivation, Chat target, ShopBuy
	Active      bool     // ItemActivation; PlayerStasis — the stasis bool
	ChatText    string   // Chat
	EntityID    string   // TalkDone, ShopBuy, CraftItem, StorageOpen, ItemOps — the NPC entity
	DialogCode  string   // TalkDone — the dialogue group the player just read
	Quantity    int      // ShopBuy — units (clamped server-side)
	RecipeIndex int      // CraftItem — index into the action's craftRecipes
	Ops         []itemOp // ItemOps — applied front to back
}

// maxInputQueue bounds one player's queue between two ticks. It equals the
// client CLIENT_EVENT_CAP, so one batch from a correct client always fits.
const maxInputQueue = 512

// EnqueueInputs appends one batch to the player's InputQueue, whole, so one
// phaseInput drains it. Called under the world mutex. It never drops: a batch
// that does not fit returns false, and the caller evicts the client.
func EnqueueInputs(p *PlayerState, cmds []InputCommand) bool {
	if len(p.InputQueue)+len(cmds) > maxInputQueue {
		return false
	}
	p.InputQueue = append(p.InputQueue, cmds...)
	return true
}
