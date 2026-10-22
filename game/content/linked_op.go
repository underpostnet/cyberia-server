// Package content — linked_op.go
//
// A linked operation is one change of a single account player that commits
// with an engine effect in one transaction (persist.Link): a world drop, a
// pickup or a market operation. What the player gives leaves the world at
// once, in escrow. Applied, the player receives the gain; refused, the refund.
// A live player captures its state before the link ends, so its next commit
// carries the result. A player that leaves while the operation waits closes
// with the state of each outcome.
//
// Caller MUST hold s.mu.
package content

import (
	"time"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
)

// linkedOp is one linked operation that waits for its commit.
type linkedOp struct {
	kind    string
	player  world.EntityID
	profile world.PlayerID
	gain    Goods // what the player receives once it applied
	refund  Goods // what returns to the player once it was refused
	// settled runs once the goods moved; player is nil when it left.
	settled func(outcome persist.Outcome, refusal string, player *world.Session, now time.Time)
}

// newOpID is a fresh operation id of kind.
func newOpID(kind string) string { return kind + "-" + randomID() }

// linkOp starts the linked operation w of player under op: escrow leaves the
// goods of the player, and the state with escrow gone and the gain received
// commits with effect. It returns the refusal, or "".
func (s *Content) linkOp(player *world.Session, op string, escrow Transfer, effect persist.Effect, w *linkedOp) string {
	if s.Durability == nil || player.AccountID == "" {
		return RefuseNotAllowed
	}
	bound, held := s.boundStacks(w.gain.Stacks)
	full, reason := s.planTransfer(player, Transfer{Takes: escrow.Takes, Coins: escrow.Coins + w.gain.Coins, Gives: bound})
	if reason != "" {
		return reason
	}
	plan, reason := s.planTransfer(player, escrow)
	if reason != "" {
		return reason
	}
	state := s.plannedState(player, full)
	state.Held = append(state.Held, held...)
	if !s.Durability.Link(op, w.kind, []persist.LinkMember{{PlayerID: string(player.PlayerID), State: state}}, effect) {
		return RefuseSaving
	}
	s.commitTransfer(player, plan)
	w.player, w.profile = player.ID, player.PlayerID
	if s.linkedOps == nil {
		s.linkedOps = make(map[string]*linkedOp)
	}
	s.linkedOps[op] = w
	return ""
}

// SettleLinkedOps applies each linked operation the engine answered. Runs
// once per tick, after every map stepped.
func (s *Content) SettleLinkedOps(now time.Time) {
	for op, w := range s.linkedOps {
		outcome, refusal := s.Durability.LinkOutcome(op)
		if outcome == persist.Pending {
			continue
		}
		delete(s.linkedOps, op)
		player := s.Sessions[w.player]
		if player != nil {
			if outcome == persist.Applied {
				s.receive(player, w.gain)
			} else {
				s.receive(player, w.refund)
			}
			s.CaptureNow(player, false)
		}
		s.Durability.Unlink(op)
		w.settled(outcome, refusal, player, now)
	}
}

// linkedFinalStates is the final durable state of a player that leaves while
// its linked operation waits, for each outcome. False when none waits.
func (s *Content) linkedFinalStates(player *world.Session) (persist.State, persist.State, bool) {
	for _, w := range s.linkedOps {
		if w.player == player.ID {
			return s.receivedState(player, w.gain), s.receivedState(player, w.refund), true
		}
	}
	return persist.State{}, persist.State{}, false
}

// FinalStates is the final durable state of a player that leaves, for each
// outcome of its pending link: applied and refused. Both are its state now
// when no operation of the player waits.
func (s *Content) FinalStates(player *world.Session) (persist.State, persist.State) {
	if applied, refused, ok := s.TradeFinalStates(player); ok {
		return applied, refused
	}
	if applied, refused, ok := s.linkedFinalStates(player); ok {
		return applied, refused
	}
	state := s.DurableState(player)
	return state, state
}
