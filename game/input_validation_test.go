package game

import (
	"math"
	"strings"
	"testing"
)

func TestValidTapTargetRejectsUnusableCoordinates(t *testing.T) {
	ok := [][2]float64{{0, 0}, {12.5, 8.25}, {-1, -1}, {99, 99}, {maxTapCoordinate, -maxTapCoordinate}}
	for _, c := range ok {
		if !validTapTarget(c[0], c[1]) {
			t.Fatalf("(%v,%v) is an ordinary tap and must be accepted", c[0], c[1])
		}
	}

	nan := math.NaN()
	inf := math.Inf(1)
	bad := [][2]float64{
		{nan, 0}, {0, nan}, {nan, nan},
		{inf, 0}, {0, -inf},
		{maxTapCoordinate + 1, 0}, {0, -(maxTapCoordinate + 1)},
		{1e18, 1e18},
	}
	for _, c := range bad {
		if validTapTarget(c[0], c[1]) {
			t.Fatalf("(%v,%v) must be rejected before it reaches the pathfinder", c[0], c[1])
		}
	}
}

func TestClampCellToGridKeepsTargetsOnTheMap(t *testing.T) {
	const w, h = 100, 80
	cases := []struct{ in, want PointI }{
		{PointI{50, 40}, PointI{50, 40}},
		{PointI{-1, -1}, PointI{0, 0}},
		{PointI{-9999, 5}, PointI{0, 5}},
		{PointI{100, 80}, PointI{99, 79}},
		{PointI{99999, 99999}, PointI{99, 79}},
		{PointI{0, 0}, PointI{0, 0}},
		{PointI{99, 79}, PointI{99, 79}},
	}
	for _, c := range cases {
		if got := clampCellToGrid(c.in, w, h); got != c.want {
			t.Fatalf("clampCellToGrid(%v) = %v, want %v", c.in, got, c.want)
		}
	}
}

func TestTruncateRunesKeepsMultiByteCharactersWhole(t *testing.T) {
	if got := truncateRunes("hello", 10); got != "hello" {
		t.Fatalf("a short string is unchanged, got %q", got)
	}
	if got := truncateRunes("hello", 2); got != "he" {
		t.Fatalf("truncateRunes(hello, 2) = %q", got)
	}
	// Four-byte runes must not be cut in the middle.
	emoji := strings.Repeat("🙂", 8)
	got := truncateRunes(emoji, 3)
	if len([]rune(got)) != 3 {
		t.Fatalf("want 3 runes, got %d (%q)", len([]rune(got)), got)
	}
	if !utf8Valid(got) {
		t.Fatalf("truncation split a rune: %q", got)
	}

	long := strings.Repeat("a", maxChatRunes*2)
	if len(truncateRunes(long, maxChatRunes)) != maxChatRunes {
		t.Fatal("a long chat line must be cut to the limit")
	}
}

func utf8Valid(s string) bool {
	for _, r := range s {
		if r == '�' {
			return false
		}
	}
	return true
}

func TestValidIdentifierBoundsClientSuppliedIDs(t *testing.T) {
	if !validIdentifier("") || !validIdentifier("iron-hatchet") {
		t.Fatal("ordinary slugs must pass")
	}
	if !validIdentifier(strings.Repeat("x", maxItemIDLen)) {
		t.Fatal("an id at the limit must pass")
	}
	if validIdentifier(strings.Repeat("x", maxItemIDLen+1)) {
		t.Fatal("an oversize id must be rejected")
	}
}

func TestValidSlotIndexBoundsVaultAccess(t *testing.T) {
	for _, i := range []int{0, 1, storageMaxSlots} {
		if !validSlotIndex(i) {
			t.Fatalf("slot %d is in range", i)
		}
	}
	for _, i := range []int{-1, storageMaxSlots + 1, 1 << 30} {
		if validSlotIndex(i) {
			t.Fatalf("slot %d must be rejected", i)
		}
	}
}

func TestClampQuantityBoundsClientCounts(t *testing.T) {
	cases := []struct{ in, want int }{
		{5, 5}, {0, 0}, {-1, 0}, {-1 << 30, 0},
		{maxStackQuantity, maxStackQuantity},
		{maxStackQuantity + 1, maxStackQuantity},
		{1 << 40, maxStackQuantity},
	}
	for _, c := range cases {
		if got := clampQuantity(c.in); got != c.want {
			t.Fatalf("clampQuantity(%d) = %d, want %d", c.in, got, c.want)
		}
	}
}
