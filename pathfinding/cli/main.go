package main

import (
	"container/heap"
	"fmt"
	"math"
	"math/rand"
	"os"
	"strconv"
	"time"
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

func rectsOverlap(a, b Rectangle) bool {
	return a.MaxX > b.MinX && a.MinX < b.MaxX && a.MaxY > b.MinY && a.MinY < b.MaxY
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

// heuristic: Euclidean distance (admissible for diagonal movement)
func (pf *Pathfinder) heuristic(ax, ay, bx, by int) float64 {
	dx := float64(ax - bx)
	dy := float64(ay - by)
	return math.Hypot(dx, dy)
}

// findPath runs A* on the integer grid and returns a slice of PointI.
func (pf *Pathfinder) findPath(start, end PointI, objW, objH float64) ([]PointI, error) {
	pf.objectW = objW
	pf.objectH = objH

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

	startNode := &Node{X: start.X, Y: start.Y, g: 0, h: pf.heuristic(start.X, start.Y, end.X, end.Y)}
	startNode.f = startNode.g + startNode.h
	heap.Push(&open, startNode)
	openSet[keyFrom(start.X, start.Y, pf.gridW)] = startNode

	came := make(map[int]*Node)
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
			hn := pf.heuristic(nx, ny, end.X, end.Y)
			node := &Node{X: nx, Y: ny, g: tentG, h: hn, f: tentG + hn, parent: current}
			heap.Push(&open, node)
			openSet[nk] = node // Add to openSet
			came[nk] = current
		}
	}

	return nil, fmt.Errorf("no path found")
}

func keyFrom(x, y, width int) int { return y*width + x }

// printGrid animates the path by printing the grid at each step.
func (pf *Pathfinder) printGrid(current PointI, path []PointI, pathIndex int) {
	// ANSI escape codes to clear the screen and move the cursor to the home position
	fmt.Print("\033[2J\033[H")

	fmt.Println("--------------------------------------------------")
	fmt.Printf("Animating pathfinding... Step %d\n", pathIndex+1)
	fmt.Printf("Current Position: (%d, %d)\n", current.X, current.Y)
	fmt.Println("--------------------------------------------------")

	// Calculate the rendering size of the object
	renderW := int(math.Floor(pf.objectW))
	renderH := int(math.Floor(pf.objectH))
	if renderW < 1 {
		renderW = 1
	}
	if renderH < 1 {
		renderH = 1
	}

	// Calculate the top-left corner of the object for rendering
	offsetX := int(math.Floor(float64(current.X) - float64(renderW)/2.0 + 0.5))
	offsetY := int(math.Floor(float64(current.Y) - float64(renderH)/2.0 + 0.5))

	for y := 0; y < pf.gridH; y++ {
		for x := 0; x < pf.gridW; x++ {
			isPath := false
			for i := 0; i <= pathIndex; i++ {
				if path[i].X == x && path[i].Y == y {
					isPath = true
					break
				}
			}

			// Check if the current grid cell is part of the object's rendering box
			isObject := x >= offsetX && x < offsetX+renderW && y >= offsetY && y < offsetY+renderH

			if isObject {
				fmt.Print("X ") // Moving object (simplified visual)
			} else if x == pf.start.X && y == pf.start.Y {
				fmt.Print("S ")
			} else if x == pf.end.X && y == pf.end.Y {
				fmt.Print("E ")
			} else if pf.grid[y][x] == 1 {
				fmt.Print("● ") // Obstacle
			} else if isPath {
				fmt.Print("+ ") // Path trail
			} else {
				fmt.Print(". ") // Empty space
			}
		}
		fmt.Println()
	}
	time.Sleep(50 * time.Millisecond) // Adjust sleep time to change animation speed
}

func main() {
	if len(os.Args) != 9 {
		fmt.Println("Usage: go run main.go <start_x> <start_y> <end_x> <end_y> <obj_w> <obj_h> <grid_width> <grid_height>")
		os.Exit(1)
	}

	parse := func(s string) float64 {
		v, err := strconv.ParseFloat(s, 64)
		if err != nil {
			fmt.Printf("Invalid arg: %s\n", s)
			os.Exit(1)
		}
		return v
	}

	startXf := parse(os.Args[1])
	startYf := parse(os.Args[2])
	endXf := parse(os.Args[3])
	endYf := parse(os.Args[4])
	objW := parse(os.Args[5])
	objH := parse(os.Args[6])
	gridWf := parse(os.Args[7])
	gridHf := parse(os.Args[8])

	gridW := int(gridWf)
	gridH := int(gridHf)
	start := PointI{int(math.Round(startXf)), int(math.Round(startYf))}
	end := PointI{int(math.Round(endXf)), int(math.Round(endYf))}

	if gridW <= 1 || gridH <= 1 {
		fmt.Println("grid must be >1")
		os.Exit(1)
	}
	if start.X < 0 || start.X >= gridW || start.Y < 0 || start.Y >= gridH {
		fmt.Println("start out of bounds")
		os.Exit(1)
	}
	if end.X < 0 || end.X >= gridW || end.Y < 0 || end.Y >= gridH {
		fmt.Println("end out of bounds")
		os.Exit(1)
	}

	rand.Seed(time.Now().UnixNano())

	const maxAttempts = 10
	for attempt := 1; attempt <= maxAttempts; attempt++ {
		pf := NewPathfinder(gridW, gridH)
		// Decreased obstacle density for a higher chance of finding a path
		numObs := int((gridW * gridH) / 80)
		pf.GenerateMap(numObs, start, end, objW, objH)

		fmt.Println("--------------------------------------------------")
		fmt.Printf("Attempt %d of %d:\n", attempt, maxAttempts)
		fmt.Printf("  Grid size: %dx%d\n", gridW, gridH)
		fmt.Printf("  Object size: %.2fx%.2f\n", objW, objH)
		fmt.Printf("  Start: (%d, %d)  End: (%d, %d)\n", start.X, start.Y, end.X, end.Y)
		fmt.Println("--------------------------------------------------")

		path, err := pf.findPath(start, end, objW, objH)
		if err == nil {
			fmt.Println("Path found! Beginning animation.")
			// Animate the path
			for i, p := range path {
				pf.printGrid(p, path, i)
			}
			fmt.Println("--------------------------------------------------")
			fmt.Println("Animation complete. Total path length:", len(path), "steps.")
			os.Exit(0)
		}

		fmt.Printf("Error in attempt %d: %v. Retrying...\n", attempt, err)
	}

	fmt.Println("--------------------------------------------------")
	fmt.Printf("Failed to find a path after %d attempts. Exiting.\n", maxAttempts)
	os.Exit(1)
}
