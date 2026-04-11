// Package game — frozen_state.go
//
// FrozenInteractionState is a general-purpose mechanism that protects a
// player during modal interactions (inventory, dialogue, future special
// modes) without pausing the rest of the real-time sandbox.
//
// While frozen:
//   - The player receives NO incoming damage or effects (skill collisions skip them).
//   - The player cannot execute actions (taps, skills, movement commands are rejected).
//   - No other entity can target the player for events.
//   - Stats computation for the player is paused (cached values remain stable).
//   - The rest of the world continues running normally.
//
// The Go server (relayer) is the single source of truth for this state.
// The client sends "freeze_start" / "freeze_end" messages with a reason
// string, and the server broadcasts the frozen flag back via the binary
// AOI self-player payload so the client always reflects the authoritative
// state.
//
// Usage:
//
//	FreezePlayer(player, "dialogue")   // enter frozen state
//	ThawPlayer(player, "dialogue")     // exit frozen state
//	player.Frozen                      // fast bool check in hot paths
//
// The caller MUST hold server.mu when calling these functions.
package game

import (
	"log"
	"time"
)

// FreezePlayer puts a player into FrozenInteractionState for the given
// reason.  If the player is already frozen, this is a no-op (the existing
// reason is preserved — first-writer wins).
//
// Caller MUST hold server.mu.
func FreezePlayer(player *PlayerState, reason string) {
	if player.Frozen {
		// Bridge transition: override the reason so the new modal "owns"
		// the freeze.  The old modal's subsequent freeze_end will carry the
		// stale reason and be rejected by ThawPlayer's reason-match check,
		// keeping the player frozen throughout the transition — no gap.
		if player.FreezeReason != reason {
			log.Printf("[FREEZE] Player %s bridge: %q -> %q",
				player.ID, player.FreezeReason, reason)
			player.FreezeReason = reason
		}
		return
	}
	player.Frozen = true
	player.FreezeReason = reason
	player.FreezeStart = time.Now()

	// Clear any in-flight movement so the player doesn't drift while frozen.
	player.Path = nil
	player.Mode = IDLE

	log.Printf("[FREEZE] Player %s frozen (reason=%q)", player.ID, reason)
}

// ThawPlayer exits FrozenInteractionState.  The supplied reason must match
// the active freeze reason — this prevents a stale "dialogue end" from
// thawing a player who has since entered "inventory" freeze.
// If reason is empty, it thaws unconditionally (force-thaw for disconnect
// cleanup).
//
// Caller MUST hold server.mu.
func ThawPlayer(player *PlayerState, reason string) {
	if !player.Frozen {
		return
	}
	if reason != "" && player.FreezeReason != reason {
		log.Printf("[FREEZE] Player %s thaw reason mismatch: active=%q, requested=%q — ignoring",
			player.ID, player.FreezeReason, reason)
		return
	}

	dur := time.Since(player.FreezeStart)
	log.Printf("[FREEZE] Player %s thawed (reason=%q, duration=%v)", player.ID, player.FreezeReason, dur)

	player.Frozen = false
	player.FreezeReason = ""
	player.FreezeStart = time.Time{}
}

// IsFrozen is a convenience predicate.  In hot paths (collision loop, game
// loop), reading player.Frozen directly is fine — this exists for
// readability in handler code.
func IsFrozen(player *PlayerState) bool {
	return player.Frozen
}
