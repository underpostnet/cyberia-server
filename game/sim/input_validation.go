// Package sim — input_validation.go
//
// Validation of client-controlled input, applied at the protocol boundary in
// receiveMessage before a command reaches the per-player queue.
//
// The simulation trusts InputCommand fields. A tap target goes straight into
// the pathfinder, so a non-finite or far out-of-grid coordinate makes the
// search expand the full reachable area and spend the whole tick budget. Every
// numeric field a client controls gets a range check here.

package sim

import "math"

const (
	// maxItemIDLen bounds any client-supplied identifier. Authored item and
	// quest codes are short slugs.
	maxItemIDLen = 128

	// MaxTapCoordinate bounds a tap before the map is known. Grid clamping
	// happens later in the handler, which is the only place the map size is
	// available. This rejects the absurd values only.
	MaxTapCoordinate = 100000
)

// ValidTapTarget reports whether a tap coordinate pair can reach the pathfinder.
// NaN and Inf are rejected: int(NaN) is undefined and produces a garbage cell.
func ValidTapTarget(x, y float64) bool {
	if math.IsNaN(x) || math.IsNaN(y) || math.IsInf(x, 0) || math.IsInf(y, 0) {
		return false
	}
	return math.Abs(x) <= MaxTapCoordinate && math.Abs(y) <= MaxTapCoordinate
}

// ValidIdentifier reports whether a client-supplied id is a plausible slug.
func ValidIdentifier(id string) bool { return len(id) <= maxItemIDLen }

// ClampQuantity bounds a client-supplied count. Handlers clamp again against
// what the player actually holds; this stops an absurd value earlier.
func ClampQuantity(q int) int {
	if q < 0 {
		return 0
	}
	if q > maxStackQuantity {
		return maxStackQuantity
	}
	return q
}

// maxStackQuantity bounds any single client-supplied item count.
const maxStackQuantity = 1_000_000
