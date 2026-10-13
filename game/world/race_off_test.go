//go:build !race

package world

// raceEnabled reports whether the race detector instruments this build. Timing
// assertions are meaningless under it.
const raceEnabled = false
