//go:build !race

package game

// raceEnabled reports whether the race detector instruments this build. Timing
// assertions are meaningless under it.
const raceEnabled = false
