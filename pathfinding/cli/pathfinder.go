package main

import (
	"container/heap"
	"fmt"
	"math"
	"math/rand"
)

//----------------------------------------------------------------------------------------------------------------------
// 1. Data Structures & Interfaces
// These types encapsulate the data needed for the grid, pathfinding nodes, and the priority queue.
//----------------------------------------------------------------------------------------------------------------------

// Point represents a 2D coordinate.
type Point struct {
	X float64
	Y float64
}

// PointI is an integer grid coordinate.
type PointI struct {
	X, Y int
}

// Rectangle represents a rectangle for collision detection.
type Rectangle struct {
	MinX, MinY, MaxX, MaxY float64
}

// Node represents a single cell in the grid for the A* algorithm.
type Node struct {
	X, Y    int
	g, h, f float64
	parent  *Node
	index   int // heap index
}

// A* priority queue (min-heap) implementation.
type PriorityQueue []*Node

// Len, Less, and Swap methods are required for the heap.Interface.
func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
	return pq[i].f < pq[j].f
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}

// Push adds an item to the priority queue.
func (pq *PriorityQueue) Push(x interface{}) {
	n := len(*pq)
	item := x.(*Node)
	item.index = n
	*pq = append(*pq, item)
}

// Pop removes and returns the smallest item from the priority queue.
func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil // avoid memory leak
	item.index = -1
	*pq = old[0 : n-1]
	return item
}

// update changes the priority of a node in the queue.
func (pq *PriorityQueue) update(node *Node, g, h float64) {
	node.g = g
	node.h = h
	node.f = g + h
	heap.Fix(pq, node.index)
}

// Pathfinder holds the state and logic for the A* search.
type Pathfinder struct {
	gridW, gridH     int
	obstacles        []Rectangle
	objectW, objectH float64
	grid             [][]int // 0 = free, 1 = obstacle
	start, end       PointI
}

//----------------------------------------------------------------------------------------------------------------------
// 2. Core Logic & Methods
// These methods implement the A* algorithm and its supporting functions.
//----------------------------------------------------------------------------------------------------------------------

// NewPathfinder creates and initializes a new Pathfinder instance.
func NewPathfinder(w, h int) *Pathfinder {
	p := &Pathfinder{gridW: w, gridH: h}
	p.grid = make([][]int, h)
	for y := 0; y < h; y++ {
		p.grid[y] = make([]int, w)
	}
	return p
}

// GenerateMap creates a procedural grid with randomly placed obstacles, ensuring
// they do not overlap with the start and end points.
func (pf *Pathfinder) GenerateMap(numObstacles int, start, end PointI, objW, objH float64) {
	pf.obstacles = []Rectangle{} // Clear previous obstacles
	for y := 0; y < pf.gridH; y++ {
		for x := 0; x < pf.gridW; x++ {
			pf.grid[y][x] = 0
		}
	}

	pf.objectW = objW
	pf.objectH = objH
	pf.start = start
	pf.end = end

	maxObsW := int(math.Max(1, float64(pf.gridW)/10.0))
	maxObsH := int(math.Max(1, float64(pf.gridH)/10.0))

	// Create forbidden rects around start/end so obstacles don't spawn on them
	startRect := Rectangle{MinX: float64(start.X) - objW, MinY: float64(start.Y) - objH, MaxX: float64(start.X) + objW, MaxY: float64(start.Y) + objH}
	endRect := Rectangle{MinX: float64(end.X) - objW, MinY: float64(end.Y) - objH, MaxX: float64(end.X) + objW, MaxY: float64(end.Y) + objH}

	attempts := 0
	randMaxX := pf.gridW - 1
	randMaxY := pf.gridH - 1

	for placed := 0; placed < numObstacles && attempts < numObstacles*10; attempts++ {
		w := rand.Intn(maxObsW) + 1
		h := rand.Intn(maxObsH) + 1
		minX := rand.Intn(randMaxX + 1)
		minY := rand.Intn(randMaxY + 1)
		maxX := minX + w
		maxY := minY + h
		if maxX >= pf.gridW {
			maxX = pf.gridW - 1
		}
		if maxY >= pf.gridH {
			maxY = pf.gridH - 1
		}

		obs := Rectangle{MinX: float64(minX), MinY: float64(minY), MaxX: float64(maxX), MaxY: float64(maxY)}

		// avoid overlapping start/end buffers
		if rectsOverlap(obs, startRect) || rectsOverlap(obs, endRect) {
			continue
		}

		// Mark grid cells
		for y := minY; y <= maxY; y++ {
			for x := minX; x <= maxX; x++ {
				if x >= 0 && x < pf.gridW && y >= 0 && y < pf.gridH {
					pf.grid[y][x] = 1
				}
			}
		}

		pf.obstacles = append(pf.obstacles, obs)
		placed++
	}
}

// findPath runs A* on the integer grid and returns a slice of PointI.
func (pf *Pathfinder) findPath(start, end PointI) ([]PointI, error) {
	if !pf.isWalkable(start.X, start.Y) {
		return nil, fmt.Errorf("start not walkable")
	}
	if !pf.isWalkable(end.X, end.Y) {
		return nil, fmt.Errorf("end not walkable")
	}

	open := make(PriorityQueue, 0)
	heap.Init(&open)

	// Map to quickly check if a node is already in the open set
	openSet := make(map[int]*Node)

	startNode := &Node{X: start.X, Y: start.Y, g: 0, h: heuristic(start.X, start.Y, end.X, end.Y)}
	startNode.f = startNode.g + startNode.h
	heap.Push(&open, startNode)
	openSet[keyFrom(start.X, start.Y, pf.gridW)] = startNode

	gScore := make(map[int]float64)
	gScore[keyFrom(start.X, start.Y, pf.gridW)] = 0

	dirs := []struct {
		dx, dy int
		cost   float64
	}{
		{1, 0, 1}, {-1, 0, 1}, {0, 1, 1}, {0, -1, 1},
		{1, 1, math.Sqrt2}, {1, -1, math.Sqrt2}, {-1, 1, math.Sqrt2}, {-1, -1, math.Sqrt2},
	}

	for open.Len() > 0 {
		current := heap.Pop(&open).(*Node)
		ck := keyFrom(current.X, current.Y, pf.gridW)
		delete(openSet, ck) // Remove from openSet

		if ck == keyFrom(end.X, end.Y, pf.gridW) {
			// rebuild path
			path := []PointI{}
			for n := current; n != nil; n = n.parent {
				path = append(path, PointI{n.X, n.Y})
			}
			// reverse
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			return path, nil
		}

		for _, d := range dirs {
			nx, ny := current.X+d.dx, current.Y+d.dy
			if !pf.isWalkable(nx, ny) {
				continue
			}

			tentG := current.g + d.cost
			nk := keyFrom(nx, ny, pf.gridW)

			// If we've found a better path to an existing node, update it.
			if existingNode, ok := openSet[nk]; ok {
				if tentG < existingNode.g {
					existingNode.g = tentG
					existingNode.parent = current
					gScore[nk] = tentG
					open.update(existingNode, tentG, existingNode.h)
				}
				continue
			}

			if existing, ok := gScore[nk]; ok && tentG >= existing {
				continue
			}

			gScore[nk] = tentG
			hn := heuristic(nx, ny, end.X, end.Y)
			node := &Node{X: nx, Y: ny, g: tentG, h: hn, f: tentG + hn, parent: current}
			heap.Push(&open, node)
			openSet[nk] = node // Add to openSet
		}
	}

	return nil, fmt.Errorf("no path found")
}

// isWalkable checks whether the object centered at (x,y) fits without colliding
// with any obstacle by sampling the grid cells that the object's bounding box covers.
func (pf *Pathfinder) isWalkable(x, y int) bool {
	// bounds check for center
	if x < 0 || x >= pf.gridW || y < 0 || y >= pf.gridH {
		return false
	}

	halfW := pf.objectW / 2.0
	halfH := pf.objectH / 2.0
	minX := int(math.Floor(float64(x) - halfW))
	minY := int(math.Floor(float64(y) - halfH))
	maxX := int(math.Ceil(float64(x) + halfW))
	maxY := int(math.Ceil(float64(y) + halfH))

	if minX < 0 {
		minX = 0
	}
	if minY < 0 {
		minY = 0
	}
	if maxX >= pf.gridW {
		maxX = pf.gridW - 1
	}
	if maxY >= pf.gridH {
		maxY = pf.gridH - 1
	}

	for yy := minY; yy <= maxY; yy++ {
		for xx := minX; xx <= maxX; xx++ {
			if pf.grid[yy][xx] == 1 {
				return false
			}
		}
	}
	return true
}

//----------------------------------------------------------------------------------------------------------------------
// 3. Helper Functions
// These functions were moved from heuristic.go to consolidate the pathfinding logic.
//----------------------------------------------------------------------------------------------------------------------

// heuristic: Euclidean distance (admissible for diagonal movement)
func heuristic(ax, ay, bx, by int) float64 {
	dx := float64(ax - bx)
	dy := float64(ay - by)
	return math.Hypot(dx, dy)
}

// rectsOverlap is a simple AABB collision detection helper.
func rectsOverlap(a, b Rectangle) bool {
	return a.MaxX > b.MinX && a.MinX < b.MaxX && a.MaxY > b.MinY && a.MinY < b.MaxY
}

// keyFrom generates a unique integer key from a 2D coordinate.
func keyFrom(x, y, width int) int { return y*width + x }
