// Package sim — tick.go
//
// The tick is the universal coordinate of the authoritative simulation.
//
// Conventions:
//   - The tick advances exactly once per simulation step.
//   - DefaultTickRate is the simulation Hz; tickDuration = 1 / DefaultTickRate.
//   - Each tick sends one snapshot per player.
//   - The string "fps" must not appear on the server. Use TickRate.
//
// On the wire, every client→server event carries the client frame plus a
// monotonic sequence number. Each snapshot carries the client frame it
// stands for and the last consumed sequence, enabling client replay.

package sim

import "time"

// DefaultTickRate is the only simulation rate. It must equal the client
// TICK_RATE_HZ.
const DefaultTickRate = 30

// TickDuration is one simulation step.
const TickDuration = time.Second / DefaultTickRate
