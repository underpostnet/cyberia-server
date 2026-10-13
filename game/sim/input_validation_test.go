package sim

import (
	"math"
	"strings"
	"testing"
)

func TestValidTapTargetRejectsUnusableCoordinates(t *testing.T) {
	ok := [][2]float64{{0, 0}, {12.5, 8.25}, {-1, -1}, {99, 99}, {MaxTapCoordinate, -MaxTapCoordinate}}
	for _, c := range ok {
		if !ValidTapTarget(c[0], c[1]) {
			t.Fatalf("(%v,%v) is an ordinary tap and must be accepted", c[0], c[1])
		}
	}

	nan := math.NaN()
	inf := math.Inf(1)
	bad := [][2]float64{
		{nan, 0}, {0, nan}, {nan, nan},
		{inf, 0}, {0, -inf},
		{MaxTapCoordinate + 1, 0}, {0, -(MaxTapCoordinate + 1)},
		{1e18, 1e18},
	}
	for _, c := range bad {
		if ValidTapTarget(c[0], c[1]) {
			t.Fatalf("(%v,%v) must be rejected before it reaches the pathfinder", c[0], c[1])
		}
	}
}

func TestTruncateRunesKeepsMultiByteCharactersWhole(t *testing.T) {
	if got := TruncateRunes("hello", 10); got != "hello" {
		t.Fatalf("a short string is unchanged, got %q", got)
	}
	if got := TruncateRunes("hello", 2); got != "he" {
		t.Fatalf("truncateRunes(hello, 2) = %q", got)
	}
	// Four-byte runes must not be cut in the middle.
	emoji := strings.Repeat("🙂", 8)
	got := TruncateRunes(emoji, 3)
	if len([]rune(got)) != 3 {
		t.Fatalf("want 3 runes, got %d (%q)", len([]rune(got)), got)
	}
	if !utf8Valid(got) {
		t.Fatalf("truncation split a rune: %q", got)
	}

	long := strings.Repeat("a", MaxChatRunes*2)
	if len(TruncateRunes(long, MaxChatRunes)) != MaxChatRunes {
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
	if !ValidIdentifier("") || !ValidIdentifier("iron-hatchet") {
		t.Fatal("ordinary slugs must pass")
	}
	if !ValidIdentifier(strings.Repeat("x", maxItemIDLen)) {
		t.Fatal("an id at the limit must pass")
	}
	if ValidIdentifier(strings.Repeat("x", maxItemIDLen+1)) {
		t.Fatal("an oversize id must be rejected")
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
		if got := ClampQuantity(c.in); got != c.want {
			t.Fatalf("clampQuantity(%d) = %d, want %d", c.in, got, c.want)
		}
	}
}
