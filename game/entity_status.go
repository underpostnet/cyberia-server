// Package game — entity_status.go
//
// Entity Status Indicator (ESI) system — the server-authoritative module that
// computes the overhead status icon for every entity in the AOI each tick.
//
// ── Industry context ─────────────────────────────────────────────────────
// Overhead status indicators are the standard MMORPG mechanism for conveying
// entity aggro state, mood, and special status without requiring the client
// to inspect multiple fields.  The server decides the icon once per AOI
// frame; the client renders it blindly.
//
// ── Protocol ─────────────────────────────────────────────────────────────
// The status icon is a single u8 appended after `effective_level` in the
// binary AOI payload for players, bots, resources, and self-player. Values
// 0–7 are defined below; 8–255 are reserved for future states (quest,
// charmed, feared, trading, AFK, …) — no protocol version bump required.
//
// ── Status Icon ID Constants ─────────────────────────────────────────────
// MUST stay in sync with the C client constants in entity_status.h.
package game

const (
	StatusNone              uint8 = 0 // No icon (skill/coin bots, world objects)
	StatusPassive           uint8 = 1 // Passive bot (no weapon)
	StatusHostile           uint8 = 2 // Hostile bot (has weapon, aggro indicator)
	StatusFrozen            uint8 = 3 // Player in FrozenInteractionState (modal)
	StatusPlayer            uint8 = 4 // Normal player (alive, not frozen)
	StatusDead              uint8 = 5 // Entity is dead / respawning
	StatusResource          uint8 = 6 // Resource entity — static, exploitable
	StatusResourceExtracted uint8 = 7 // Resource entity depleted / extracted
)

// PlayerStatusIcon computes the overhead status icon for a player.
//
// Priority chain (highest → lowest):
//  1. Dead/respawning → StatusDead
//  2. Frozen (modal)  → StatusFrozen
//  3. Alive           → StatusPlayer
func PlayerStatusIcon(p *PlayerState) uint8 {
	if p.IsGhost() {
		return StatusDead
	}
	if p.Frozen {
		return StatusFrozen
	}
	return StatusPlayer
}

// BotStatusIcon computes the overhead status icon for a bot.
//
// Priority chain:
//  1. Skill/coin bot  → StatusNone (no overhead icon for projectiles)
//  2. Dead/respawning → StatusDead
//  3. Hostile         → StatusHostile
//  4. Default         → StatusPassive
func BotStatusIcon(b *BotState) uint8 {
	if b.Behavior == "skill" || b.Behavior == "coin" {
		return StatusNone
	}
	if b.IsGhost() {
		return StatusDead
	}
	if b.Behavior == "hostile" {
		return StatusHostile
	}
	return StatusPassive
}

// ResourceStatusIcon computes the overhead status icon for a resource entity.
//
// Priority chain:
//  1. Dead/destroyed  → StatusResourceExtracted
//  2. Alive           → StatusResource
func ResourceStatusIcon(r *ResourceState) uint8 {
	if r.IsGhost() {
		return StatusResourceExtracted
	}
	return StatusResource
}
