package world

import (
	"testing"
)

func TestClampCellToGridKeepsTargetsOnTheMap(t *testing.T) {
	const w, h = 100, 80
	cases := []struct{ in, want Cell }{
		{Cell{50, 40}, Cell{50, 40}},
		{Cell{-1, -1}, Cell{0, 0}},
		{Cell{-9999, 5}, Cell{0, 5}},
		{Cell{100, 80}, Cell{99, 79}},
		{Cell{99999, 99999}, Cell{99, 79}},
		{Cell{0, 0}, Cell{0, 0}},
		{Cell{99, 79}, Cell{99, 79}},
	}
	for _, c := range cases {
		if got := ClampCellToGrid(c.in, w, h); got != c.want {
			t.Fatalf("clampCellToGrid(%v) = %v, want %v", c.in, got, c.want)
		}
	}
}
