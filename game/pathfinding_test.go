package game

import (
	"testing"
	"time"
)

var unitDims = Dimensions{Width: 1, Height: 1}

// openGrid returns an empty walkable grid.
func openGrid(size int) *Pathfinder { return NewPathfinder(size, size) }

// splitGrid seals the map with a vertical wall, so a target on the far side is
// unreachable. This is the shape that makes A* expand the whole reachable area.
func splitGrid(size int) *Pathfinder {
	pf := NewPathfinder(size, size)
	for y := 0; y < size; y++ {
		pf.grid[y][size/2] = 1
	}
	return pf
}

func TestAstarFindsShortestPath(t *testing.T) {
	pf := openGrid(20)

	straight, err := pf.Astar(PointI{0, 0}, PointI{9, 0}, unitDims)
	if err != nil {
		t.Fatalf("open grid must have a path: %v", err)
	}
	if len(straight) != 10 {
		t.Fatalf("straight run of 10 cells expected, got %d: %v", len(straight), straight)
	}

	// 8-connected movement: the diagonal costs the same as a straight step.
	diagonal, err := pf.Astar(PointI{0, 0}, PointI{5, 5}, unitDims)
	if err != nil {
		t.Fatalf("diagonal must have a path: %v", err)
	}
	if len(diagonal) != 6 {
		t.Fatalf("diagonal of 6 cells expected, got %d: %v", len(diagonal), diagonal)
	}
}

func TestAstarPathIsContiguousAndWalkable(t *testing.T) {
	pf := splitGrid(30)
	path, err := pf.Astar(PointI{2, 2}, PointI{12, 25}, unitDims)
	if err != nil {
		t.Fatalf("target on the near side must be reachable: %v", err)
	}
	if path[0] != (PointI{2, 2}) {
		t.Fatalf("path must start at the origin, got %v", path[0])
	}
	if path[len(path)-1] != (PointI{12, 25}) {
		t.Fatalf("path must end at the target, got %v", path[len(path)-1])
	}
	for i, cell := range path {
		if !pf.isWalkable(cell.X, cell.Y, unitDims) {
			t.Fatalf("path step %d enters a blocked cell %v", i, cell)
		}
		if i == 0 {
			continue
		}
		dx, dy := cell.X-path[i-1].X, cell.Y-path[i-1].Y
		if dx < -1 || dx > 1 || dy < -1 || dy > 1 || (dx == 0 && dy == 0) {
			t.Fatalf("path step %d jumps from %v to %v", i, path[i-1], cell)
		}
	}
}

func TestAstarReportsUnreachableTarget(t *testing.T) {
	pf := splitGrid(30)
	if _, err := pf.Astar(PointI{2, 2}, PointI{25, 15}, unitDims); err == nil {
		t.Fatal("a walled-off target must return an error, not a path")
	}
}

// An endpoint outside the grid is rejected before the search starts. Without
// this the search expands every reachable cell and then fails anyway.
func TestAstarRejectsOffGridEndpoints(t *testing.T) {
	pf := openGrid(20)
	for _, c := range []struct {
		name       string
		start, end PointI
	}{
		{"end far positive", PointI{1, 1}, PointI{1 << 20, 1 << 20}},
		{"end negative", PointI{1, 1}, PointI{-5, -5}},
		{"start off grid", PointI{-1, 0}, PointI{5, 5}},
		{"end just past the edge", PointI{1, 1}, PointI{20, 20}},
	} {
		if _, err := pf.Astar(c.start, c.end, unitDims); err == nil {
			t.Fatalf("%s: must be rejected", c.name)
		}
	}
}

// The search scratch is reused across calls, so a stale mark from the previous
// search must never block the next one.
func TestAstarRepeatedSearchesStayCorrect(t *testing.T) {
	pf := splitGrid(24)
	for i := 0; i < 50; i++ {
		if _, err := pf.Astar(PointI{1, 1}, PointI{20, 12}, unitDims); err == nil {
			t.Fatalf("run %d: unreachable target must stay unreachable", i)
		}
		path, err := pf.Astar(PointI{1, 1}, PointI{10, 20}, unitDims)
		if err != nil {
			t.Fatalf("run %d: reachable target must stay reachable: %v", i, err)
		}
		if path[len(path)-1] != (PointI{10, 20}) {
			t.Fatalf("run %d: wrong endpoint %v", i, path[len(path)-1])
		}
	}
}

// The worst ordinary case is an unreachable target on a full-size map. One tap
// must stay well inside the 60 Hz tick budget, because several taps can land
// in the same tick.
func TestAstarWorstCaseFitsTheTickBudget(t *testing.T) {
	if raceEnabled {
		t.Skip("race instrumentation makes wall-clock timing meaningless")
	}
	pf := splitGrid(100)
	dims := Dimensions{Width: 2, Height: 2}

	// Warm the scratch buffers so the first allocation is not measured.
	pf.Astar(PointI{2, 2}, PointI{95, 50}, dims)

	start := time.Now()
	const runs = 20
	for i := 0; i < runs; i++ {
		pf.Astar(PointI{2, 2}, PointI{95, 50}, dims)
	}
	perCall := time.Since(start) / runs

	tickBudget := time.Second / 60
	if perCall > tickBudget/4 {
		t.Fatalf("unreachable-target search costs %v, over a quarter of the %v tick budget",
			perCall.Round(time.Microsecond), tickBudget)
	}
	t.Logf("unreachable-target search = %v (tick budget %v)",
		perCall.Round(time.Microsecond), tickBudget)
}

func TestFindClosestWalkablePointHandlesOffGridInput(t *testing.T) {
	pf := splitGrid(20)

	// A blocked cell resolves to a walkable neighbour.
	got, err := pf.findClosestWalkablePoint(PointI{10, 5}, unitDims)
	if err != nil {
		t.Fatalf("a blocked cell must resolve to a neighbour: %v", err)
	}
	if !pf.isWalkable(got.X, got.Y, unitDims) {
		t.Fatalf("resolved cell %v is not walkable", got)
	}

	// A point far outside has no in-grid neighbour and must fail fast.
	if _, err := pf.findClosestWalkablePoint(PointI{1 << 20, 1 << 20}, unitDims); err == nil {
		t.Fatal("a far off-grid point must not resolve")
	}
}
