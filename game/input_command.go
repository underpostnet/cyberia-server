// Package game — input_command.go
//
// InputCommand is the canonical client-→-server input frame. It carries
// everything required for an authoritative server to (a) apply the input on
// the correct tick, (b) acknowledge it back to the client for prediction
// reconciliation, and (c) gate stale or replayed inputs.
//
// Wire layout (binary, uplink opcodes 0x10–0x16):
//
//	[u8 kind][payload-by-kind][u32 clientTick][u32 sequence]
//
// The clientTick + sequence suffix is read optionally; zero values are
// accepted and handled by the simulation.
//
// Ownership:
//   - Built and enqueued by handlers.go (per-WS-goroutine).
//   - Consumed exclusively by phaseInput in simulation_phases.go.
//   - PlayerState.InputQueue is the single rendezvous; no other code path
//     mutates entity state in response to client input.

package game

// InputKind enumerates the input categories the server accepts. Numeric
// values mirror the binary uplink opcodes for trace-ability.
type InputKind uint8

const (
	InputKindUnknown        InputKind = 0x00
	InputKindHandshake      InputKind = 0x10
	InputKindPlayerAction   InputKind = 0x11 // tap move + skill trigger
	InputKindItemActivation InputKind = 0x12
	InputKindFreezeStart    InputKind = 0x13
	InputKindFreezeEnd      InputKind = 0x14
	InputKindChat           InputKind = 0x15
	InputKindGetItemsIDs    InputKind = 0x16
	InputKindDlgStart       InputKind = 0x17 // dialogue opened — freeze + bind context
	InputKindDlgComplete    InputKind = 0x18 // all lines read — advance talk/quest, unfreeze
	InputKindDlgCancel      InputKind = 0x19 // dismissed early — unfreeze, no progress
	InputKindQuestAbandon   InputKind = 0x1A // drop an active quest — moves it to failed
	InputKindQuestAccept    InputKind = 0x1B // explicitly accept the NPC's offered quest
)

// InputCommand is the unit of client→server input.
type InputCommand struct {
	Kind       InputKind
	ClientTick uint32 // client-side estimated server tick when emitted
	Sequence   uint32 // monotonic per-client sequence number
	// Payload fields — only the ones relevant to Kind are populated.
	TargetX    float64 // PlayerAction
	TargetY    float64 // PlayerAction
	ItemID     string  // ItemActivation, GetItemsIDs, Chat target
	Active     bool    // ItemActivation
	Reason     string  // FreezeStart, FreezeEnd
	ChatText   string  // Chat
	EntityID   string  // DlgStart, DlgComplete, DlgCancel — the NPC entity
	DialogCode string  // DlgComplete — the dialogue group the player just read
}

// EnqueueInput pushes a command onto the player's InputQueue. Called from
// the WS read goroutine; phaseInput drains under the world mutex on the
// next tick. Bounded length keeps a buggy client from causing unbounded
// growth — overflow drops the oldest entry, preserving recency.
func EnqueueInput(p *PlayerState, cmd InputCommand) {
	const maxQueue = 64
	if len(p.InputQueue) >= maxQueue {
		copy(p.InputQueue, p.InputQueue[1:])
		p.InputQueue = p.InputQueue[:len(p.InputQueue)-1]
	}
	p.InputQueue = append(p.InputQueue, cmd)
	if cmd.Sequence > p.LastAckedInputSequence {
		p.LastAckedInputSequence = cmd.Sequence
	}
}
