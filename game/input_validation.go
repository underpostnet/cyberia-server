// Package game — input_validation.go
//
// Validation of client-controlled input, applied at the protocol boundary in
// receiveMessage before a command reaches the per-player queue.
//
// The simulation trusts InputCommand fields. A tap target goes straight into
// the pathfinder, so a non-finite or far out-of-grid coordinate makes the
// search expand the full reachable area and spend the whole tick budget. Every
// numeric field a client controls gets a range check here.

package game

import "math"

const (
	// maxChatRunes bounds one chat line. The socket read limit already bounds
	// the frame; this bounds what the simulation stores and re-broadcasts.
	maxChatRunes = 512

	// maxItemIDLen bounds any client-supplied identifier. Authored item and
	// quest codes are short slugs.
	maxItemIDLen = 128

	// maxTapCoordinate bounds a tap before the map is known. Grid clamping
	// happens later in the handler, which is the only place the map size is
	// available. This rejects the absurd values only.
	maxTapCoordinate = 100000
)

// validTapTarget reports whether a tap coordinate pair can reach the pathfinder.
// NaN and Inf are rejected: int(NaN) is undefined and produces a garbage cell.
func validTapTarget(x, y float64) bool {
	if math.IsNaN(x) || math.IsNaN(y) || math.IsInf(x, 0) || math.IsInf(y, 0) {
		return false
	}
	return math.Abs(x) <= maxTapCoordinate && math.Abs(y) <= maxTapCoordinate
}

// clampCellToGrid puts a cell inside the map. The handler applies it so a tap
// near an edge still moves the player instead of failing the path search.
func clampCellToGrid(p PointI, gridW, gridH int) PointI {
	if p.X < 0 {
		p.X = 0
	} else if p.X > gridW-1 {
		p.X = gridW - 1
	}
	if p.Y < 0 {
		p.Y = 0
	} else if p.Y > gridH-1 {
		p.Y = gridH - 1
	}
	return p
}

// validIdentifier reports whether a client-supplied id is a plausible slug.
func validIdentifier(id string) bool { return len(id) <= maxItemIDLen }

// truncateRunes cuts a string to at most n runes without splitting one.
func truncateRunes(s string, n int) string {
	count := 0
	for i := range s {
		if count == n {
			return s[:i]
		}
		count++
	}
	return s
}

// validSlotIndex bounds a storage slot index before it indexes a vault.
func validSlotIndex(i int) bool { return i >= 0 && i <= storageMaxSlots }

// clampQuantity bounds a client-supplied count. Handlers clamp again against
// what the player actually holds; this stops an absurd value earlier.
func clampQuantity(q int) int {
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
