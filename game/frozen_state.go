// Package game — frozen_state.go
//
// FrozenInteractionState protects a player while a client modal is open,
// without a pause of the rest of the real-time sandbox.
//
// While frozen:
//   - The player receives NO incoming damage or effects (skill collisions skip them).
//   - The player cannot execute actions (taps, skills, movement commands are rejected).
//   - No other entity can target the player for events.
//   - Temporary stat effects keep their server expiry times.
//   - The rest of the world continues running normally.
//
// The client event "player_stasis" is the only writer after join. The server
// sends the flag back as "frozen" in the AOI self-player payload.
//
// The caller MUST hold server.mu when calling these functions.
package game

import (
	"cyberia-server/logx"
	"time"
)

// FreezePlayer puts a player into FrozenInteractionState. No-op when frozen.
//
// Caller MUST hold server.mu.
func FreezePlayer(player *PlayerState) {
	if player.Frozen {
		return
	}
	player.Frozen = true
	player.FreezeStart = time.Now()

	// Clear any in-flight movement so the player doesn't drift while frozen.
	player.Path = nil
	player.Mode = IDLE

	logx.Debugf("[FREEZE] Player %s frozen", player.ID)
}

// ThawPlayer exits FrozenInteractionState. No-op when not frozen.
//
// Caller MUST hold server.mu.
func ThawPlayer(player *PlayerState) {
	if !player.Frozen {
		return
	}
	logx.Debugf("[FREEZE] Player %s thawed (duration=%v)", player.ID, time.Since(player.FreezeStart))

	player.Frozen = false
	player.FreezeStart = time.Time{}
}
