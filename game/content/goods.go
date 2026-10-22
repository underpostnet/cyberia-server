// Package content — goods.go
//
// Goods a player receives from an operation that settled after it started: a
// trade, a pickup or a market operation. A stack whose label names another
// definition now is held with its own definition, and a stack the bag cannot
// take is held too: nothing received is lost.
//
// Caller MUST hold s.mu.
package content

import (
	"cyberia-server/game/persist"
	"cyberia-server/game/world"
)

// Goods are coins and stacks a player receives.
type Goods struct {
	Coins  int64
	Stacks []Stack
}

// heldStack is a stack the world keeps for its owner outside its containers.
func heldStack(stack Stack, reason string) persist.Held {
	return persist.Held{
		Stack:     durableStack(stack.ItemID, stack.CID, stack.Qty, 0, false),
		Container: persist.ContainerBag,
		Reason:    reason,
	}
}

// boundStacks splits stacks into the ones of the definition the world binds
// and the ones held because their label names another definition now.
func (s *Content) boundStacks(stacks []Stack) ([]Stack, []persist.Held) {
	var bound []Stack
	var held []persist.Held
	for _, st := range stacks {
		if st.CID != "" && st.CID != s.DefinitionCID(st.ItemID) {
			held = append(held, heldStack(st, persist.HeldRebound))
			continue
		}
		bound = append(bound, st)
	}
	return bound, held
}

// receivedState is the durable state of player once goods arrived. Coins
// stop at the wallet maximum.
func (s *Content) receivedState(player *world.Session, goods Goods) persist.State {
	bound, held := s.boundStacks(goods.Stacks)
	plan, reason := s.planTransfer(player, Transfer{Coins: goods.Coins, Gives: bound})
	if reason == "" {
		state := s.plannedState(player, plan)
		state.Held = append(state.Held, held...)
		return state
	}
	state := s.DurableState(player)
	state.Coins = uint32(min(int64(state.Coins)+goods.Coins, s.maxBalance()))
	for _, st := range bound {
		held = append(held, heldStack(st, persist.HeldNoRoom))
	}
	state.Held = append(state.Held, held...)
	return state
}

// receive gives player goods, as receivedState plans them.
func (s *Content) receive(player *world.Session, goods Goods) {
	if goods.Coins == 0 && len(goods.Stacks) == 0 {
		return
	}
	bound, held := s.boundStacks(goods.Stacks)
	if s.ApplyTransfer(player, Transfer{Coins: goods.Coins, Gives: bound}) != "" {
		s.AddCoins(player.ID, s.Maps[player.MapCode], int(goods.Coins))
		for _, st := range bound {
			if s.ApplyTransfer(player, Transfer{Gives: []Stack{st}}) != "" {
				held = append(held, heldStack(st, persist.HeldNoRoom))
			}
		}
	}
	if len(held) > 0 {
		s.setHeld(player.PlayerID, append(s.Held[player.PlayerID], held...))
	}
	if len(goods.Stacks) > 0 {
		s.AdvancePlayerQuestsOnGain(player)
	}
}
