// Package game — input_command.go
//
// InputCommand is the canonical client-→-server input frame. It carries
// everything required for an authoritative server to (a) apply the input on
// the correct tick, (b) acknowledge it back to the client for prediction
// reconciliation, and (c) gate stale or replayed inputs.
//
// The wire carries the JSON envelope {"type", "payload"}; receiveMessage in
// handlers.go maps the type word to an InputKind. The payload tick + seq
// fields are optional; zero values are accepted and handled by the simulation.
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
	InputKindUnknown InputKind = iota
	InputKindHandshake
	InputKindPlayerAction // tap move + skill trigger
	InputKindItemActivation
	InputKindFreezeStart
	InputKindFreezeEnd
	InputKindChat
	InputKindDlgStart        // dialogue opened — freeze + bind context
	InputKindDlgComplete     // all lines read — advance talk/quest, unfreeze
	InputKindDlgCancel       // dismissed early — unfreeze, no progress
	InputKindQuestAbandon    // drop an active quest — moves it to failed
	InputKindQuestAccept     // explicitly accept the NPC's offered quest
	InputKindShopBuy         // buy one catalog item from a vendor action
	InputKindCraftItem       // assemble one recipe at an assembler action
	InputKindCraftCancel     // abort the running assembly and refund it
	InputKindStorageOpen     // bind a storage vault and read its grid
	InputKindStorageMove     // relocate a vault slot onto a free cell
	InputKindStorageSwap     // exchange two occupied vault cells
	InputKindStorageTransfer // move a stack across the vault boundary
)

// InputCommand is the unit of client→server input.
type InputCommand struct {
	Kind       InputKind
	ClientTick uint32 // client-side estimated server tick when emitted
	Sequence   uint32 // monotonic per-client sequence number
	// Payload fields — only the ones relevant to Kind are populated.
	TargetX     float64 // PlayerAction
	TargetY     float64 // PlayerAction
	ItemID      string  // ItemActivation, Chat target, ShopBuy
	Active      bool    // ItemActivation
	Reason      string  // FreezeStart, FreezeEnd
	ChatText    string  // Chat
	EntityID    string  // DlgStart, DlgComplete, DlgCancel, ShopBuy, CraftItem, Storage* — the NPC entity
	DialogCode  string  // DlgComplete — the dialogue group the player just read
	Quantity    int     // ShopBuy, StorageTransfer, StorageMove — units (clamped server-side)
	FromIndex   int     // Storage* — source slot, linear index into the vault
	ToIndex     int     // Storage* — target slot, linear index into the vault
	Deposit     bool    // StorageTransfer — into the vault, else out of it
	RecipeIndex int     // CraftItem — index into the action's craftRecipes
}

// EnqueueInput pushes a command onto the player's InputQueue. Called from
// the WS read goroutine; phaseInput drains under the world mutex on the
// next tick. Bounded length keeps a buggy client from causing unbounded
// growth — overflow drops the oldest entry, preserving recency. The
// evicted slot is zeroed to release string references for GC.
func EnqueueInput(p *PlayerState, cmd InputCommand) {
	const maxQueue = 64
	if len(p.InputQueue) >= maxQueue {
		// Zero the evicted slot so string fields (ItemID, ChatText, etc.)
		// are released for GC rather than retained by the backing array.
		p.InputQueue[0] = InputCommand{}
		copy(p.InputQueue, p.InputQueue[1:])
		p.InputQueue = p.InputQueue[:len(p.InputQueue)-1]
	}
	p.InputQueue = append(p.InputQueue, cmd)
	if cmd.Sequence > p.LastAckedInputSequence {
		p.LastAckedInputSequence = cmd.Sequence
	}
}
