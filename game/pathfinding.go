package game

import (
	"container/heap"
	"fmt"
	"math"
	"math/rand"
)

// Pathfinding structures and algorithms

type Node struct {
	X, Y    int
	g, h, f float64
	parent  *Node
	index   int
}

type PriorityQueue []*Node

func (pq PriorityQueue) Len() int           { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool { return pq[i].f < pq[j].f }
func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}
func (pq *PriorityQueue) Push(x interface{}) {
	n := len(*pq)
	item := x.(*Node)
	item.index = n
	*pq = append(*pq, item)
}
func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	item.index = -1
	*pq = old[0 : n-1]
	return item
}

// Pathfinder grid and utilities

type Pathfinder struct {
	gridW, gridH int
	obstacles    map[string]ObjectState
	grid         [][]int

	// Search scratch, one cell per grid cell, reused by every Astar call.
	// Every caller holds the world lock, so only one search runs at a time.
	// The generation stamp makes a reset O(1) instead of a full clear.
	gen       uint32
	closedGen []uint32
	openGen   []uint32
	openNode  []*Node
}

// NewPathfinder creates a new pathfinder with a grid of specified dimensions.
func NewPathfinder(w, h int) *Pathfinder {
	p := &Pathfinder{gridW: w, gridH: h}
	p.grid = make([][]int, h)
	for y := 0; y < h; y++ {
		p.grid[y] = make([]int, w)
	}
	return p
}

func (pf *Pathfinder) cellIndex(x, y int) int { return y*pf.gridW + x }

func (pf *Pathfinder) inGrid(x, y int) bool {
	return x >= 0 && x < pf.gridW && y >= 0 && y < pf.gridH
}

// beginSearch advances the generation stamp so the previous search's marks
// read as stale. It grows the scratch buffers when the grid changes size.
func (pf *Pathfinder) beginSearch() {
	cells := pf.gridW * pf.gridH
	if len(pf.closedGen) != cells {
		pf.closedGen = make([]uint32, cells)
		pf.openGen = make([]uint32, cells)
		pf.openNode = make([]*Node, cells)
		pf.gen = 0
	}
	pf.gen++
	if pf.gen == 0 {
		for i := range pf.closedGen {
			pf.closedGen[i], pf.openGen[i] = 0, 0
		}
		pf.gen = 1
	}
}

// isCellWalkable checks if a single grid cell is walkable.
func (pf *Pathfinder) isCellWalkable(x, y int) bool {
	return x >= 0 && x < pf.gridW && y >= 0 && y < pf.gridH && pf.grid[y][x] == 0
}

// isWalkable checks if a rectangle of a given dimension is walkable.
func (pf *Pathfinder) isWalkable(x, y int, playerDims Dimensions) bool {
	w, h := int(math.Ceil(playerDims.Width)), int(math.Ceil(playerDims.Height))
	for dy := 0; dy < h; dy++ {
		for dx := 0; dx < w; dx++ {
			if !pf.isCellWalkable(x+dx, y+dy) {
				return false
			}
		}
	}
	return true
}

// findRandomWalkablePoint finds a random walkable point for a given dimension.
func (pf *Pathfinder) findRandomWalkablePoint(dims Dimensions) (PointI, error) {
	for attempts := 0; attempts < 200; attempts++ {
		x, y := rand.Intn(pf.gridW-int(dims.Width)), rand.Intn(pf.gridH-int(dims.Height))
		if pf.isWalkable(x, y, dims) {
			return PointI{X: x, Y: y}, nil
		}
	}
	return PointI{}, fmt.Errorf("could not find a walkable point")
}

// findClosestWalkablePoint finds the closest walkable point to a given point.
func (pf *Pathfinder) findClosestWalkablePoint(p PointI, playerDims Dimensions) (PointI, error) {
	if pf.isWalkable(p.X, p.Y, playerDims) {
		return p, nil
	}
	// Only in-grid cells enter the queue, so the sweep visits each cell once.
	pf.beginSearch()
	queue := []PointI{p}
	if pf.inGrid(p.X, p.Y) {
		pf.closedGen[pf.cellIndex(p.X, p.Y)] = pf.gen
	}
	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]
		if pf.isWalkable(current.X, current.Y, playerDims) {
			return current, nil
		}
		for _, move := range [4]PointI{{1, 0}, {-1, 0}, {0, 1}, {0, -1}} {
			nx, ny := current.X+move.X, current.Y+move.Y
			if !pf.inGrid(nx, ny) {
				continue
			}
			idx := pf.cellIndex(nx, ny)
			if pf.closedGen[idx] == pf.gen {
				continue
			}
			pf.closedGen[idx] = pf.gen
			queue = append(queue, PointI{X: nx, Y: ny})
		}
	}
	return PointI{}, fmt.Errorf("no walkable point found near (%d, %d)", p.X, p.Y)
}

var astarMoves = [8]PointI{{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}

// Astar performs the A* pathfinding algorithm.
//
// Runs in phaseInput under the world lock, so its cost is charged to the tick
// budget. The closed set keeps each cell to one expansion, and the open index
// finds a queued node in O(1). Both are necessary: an unreachable target makes
// the search expand the full reachable area.
func (pf *Pathfinder) Astar(start, end PointI, playerDims Dimensions) ([]PointI, error) {
	if !pf.inGrid(start.X, start.Y) || !pf.inGrid(end.X, end.Y) {
		return nil, fmt.Errorf("path endpoint outside the grid")
	}
	pf.beginSearch()

	openSet := make(PriorityQueue, 0, 64)
	heap.Init(&openSet)
	startNode := &Node{X: start.X, Y: start.Y, h: heuristic(start.X, start.Y, end.X, end.Y)}
	startNode.f = startNode.h
	heap.Push(&openSet, startNode)
	startIdx := pf.cellIndex(start.X, start.Y)
	pf.openGen[startIdx], pf.openNode[startIdx] = pf.gen, startNode

	for openSet.Len() > 0 {
		current := heap.Pop(&openSet).(*Node)
		if current.X == end.X && current.Y == end.Y {
			return reconstructPath(current), nil
		}
		currentIdx := pf.cellIndex(current.X, current.Y)
		pf.openGen[currentIdx] = 0
		pf.closedGen[currentIdx] = pf.gen

		for _, move := range astarMoves {
			nx, ny := current.X+move.X, current.Y+move.Y
			if !pf.isWalkable(nx, ny, playerDims) {
				continue
			}
			idx := pf.cellIndex(nx, ny)
			if pf.closedGen[idx] == pf.gen {
				continue
			}
			tentativeGScore := current.g + 1
			if pf.openGen[idx] == pf.gen {
				if queued := pf.openNode[idx]; tentativeGScore < queued.g {
					queued.g = tentativeGScore
					queued.f = queued.g + queued.h
					queued.parent = current
					heap.Fix(&openSet, queued.index)
				}
				continue
			}
			newNode := &Node{
				X:      nx,
				Y:      ny,
				g:      tentativeGScore,
				h:      heuristic(nx, ny, end.X, end.Y),
				parent: current,
			}
			newNode.f = newNode.g + newNode.h
			pf.openGen[idx], pf.openNode[idx] = pf.gen, newNode
			heap.Push(&openSet, newNode)
		}
	}
	return nil, fmt.Errorf("could not find a path")
}

func heuristic(x1, y1, x2, y2 int) float64 {
	return math.Abs(float64(x1-x2)) + math.Abs(float64(y1-y2))
}

func reconstructPath(n *Node) []PointI {
	path := make([]PointI, 0)
	for n != nil {
		path = append(path, PointI{X: n.X, Y: n.Y})
		n = n.parent
	}
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return path
}

// rectsOverlap checks if two rectangles overlap.
func rectsOverlap(r1, r2 Rectangle) bool {
	return r1.MinX < r2.MaxX && r1.MaxX > r2.MinX && r1.MinY < r2.MaxY && r1.MaxY > r2.MinY
}
