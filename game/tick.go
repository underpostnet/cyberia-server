// Package game — tick.go
//
// Tick is the universal coordinate of the authoritative simulation. Every
// piece of timing in the server (simulation, replication, prediction
// acknowledgement, AOI delta baselines, FCT events, AI scheduling) carries
// or derives from a Tick value.
//
// Conventions:
//   - Tick advances exactly once per simulation step.
//   - Tick is monotonic for the lifetime of a world; it resets only on
//     world rebuild (which currently never happens after bootstrap).
//   - tickRate is the simulation Hz; tickDuration = 1 / tickRate.
//   - snapshotRate is the replication Hz; typically lower than tickRate
//     (e.g. tick = 30 Hz, snapshot = 20 Hz). Snapshots are independent of
//     the simulation cadence.
//   - The string "fps" must not appear on the server. Use TickRate.
//
// On the wire, every server→client AOI frame carries the tick it was
// produced at. Every client→server InputCommand carries the client's
// estimated tick plus a monotonic sequence number. The server echoes back
// the last processed sequence per player in each snapshot, enabling client
// reconciliation.

package game

import "time"

// Tick is the simulation step counter. uint32 gives ~828 days at 60 Hz —
// enough for any conceivable session and small enough to be cheap on wire.
type Tick uint32

// InputSequence is a client-local monotonic counter attached to every
// InputCommand. It is echoed back on snapshots as `LastAckedSequence` so
// clients can drop acknowledged commands from their prediction buffer.
type InputSequence uint32

// Default simulation/replication rates. Override via InstanceConfig.tick_rate
// (and InstanceConfig.snapshot_rate where applicable).
const (
	DefaultTickRate     = 30
	DefaultSnapshotRate = 20
)

// computeTickDuration — derives tickDuration from a tickRate Hz value.
// Returns the default if rate is non-positive.
func computeTickDuration(rate int) time.Duration {
	if rate <= 0 {
		rate = DefaultTickRate
	}
	return time.Second / time.Duration(rate)
}
