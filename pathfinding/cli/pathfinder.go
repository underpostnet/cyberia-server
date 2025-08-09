package main

import (
	"container/heap"
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"os"
	"strings"
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
func NewPathfinder(w, h int, objW, objH float64) *Pathfinder {
	p := &Pathfinder{
		gridW:   w,
		gridH:   h,
		objectW: objW,
		objectH: objH,
	}
	p.grid = make([][]int, h)
	for y := 0; y < h; y++ {
		p.grid[y] = make([]int, w)
	}
	return p
}

// SetStartAndEnd sets the start and end points for visualization.
func (pf *Pathfinder) SetStartAndEnd(start, end PointI) {
	pf.start = start
	pf.end = end
}

// GenerateMap creates a procedural grid with randomly placed obstacles, ensuring
// they do not overlap with the start and end points.
func (pf *Pathfinder) GenerateMap(numObstacles int, start, end PointI) {
	pf.obstacles = []Rectangle{} // Clear previous obstacles
	for y := 0; y < pf.gridH; y++ {
		for x := 0; x < pf.gridW; x++ {
			pf.grid[y][x] = 0
		}
	}

	maxObsW := int(math.Max(1, float64(pf.gridW)/10.0))
	maxObsH := int(math.Max(1, float64(pf.gridH)/10.0))

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

		// Skip placing the obstacle if it overlaps with the start or end points
		startRect := Rectangle{MinX: float64(start.X), MinY: float64(start.Y), MaxX: float64(start.X), MaxY: float64(start.Y)}
		endRect := Rectangle{MinX: float64(end.X), MinY: float64(end.Y), MaxX: float64(end.X), MaxY: float64(end.Y)}
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

// LoadObstacles populates the Pathfinder's obstacles from a JSON file.
func (pf *Pathfinder) LoadObstacles(filePath string) error {
	data, err := os.ReadFile(filePath)
	if err != nil {
		return fmt.Errorf("failed to read file: %w", err)
	}

	var loadedObstacles []Rectangle
	if err := json.Unmarshal(data, &loadedObstacles); err != nil {
		return fmt.Errorf("failed to unmarshal JSON: %w", err)
	}
	pf.obstacles = loadedObstacles

	// Re-build the grid based on the loaded obstacles
	for y := 0; y < pf.gridH; y++ {
		for x := 0; x < pf.gridW; x++ {
			pf.grid[y][x] = 0
			if pf.isObstacle(x, y) {
				pf.grid[y][x] = 1
			}
		}
	}

	return nil
}

// AnimateGrid generates a string representation of the grid for a console animation.
// It now correctly renders the object size by rounding down to the nearest integer.
func (pf *Pathfinder) AnimateGrid(current PointI, path []PointI) string {
	var sb strings.Builder
	pathMap := make(map[PointI]bool)
	for _, p := range path {
		pathMap[p] = true
	}

	// Calculate the integer dimensions of the object by rounding down
	objW := int(math.Floor(pf.objectW))
	objH := int(math.Floor(pf.objectH))
	if objW < 1 {
		objW = 1
	} // ensure minimum size is 1
	if objH < 1 {
		objH = 1
	} // ensure minimum size is 1

	// Calculate the top-left corner of the object's representation
	startX := current.X - objW/2
	startY := current.Y - objH/2

	for y := 0; y < pf.gridH; y++ {
		for x := 0; x < pf.gridW; x++ {
			p := PointI{X: x, Y: y}

			// Check if the current cell is part of the moving object's approximated bounding box.
			isObjectCell := false
			if x >= startX && x < startX+objW && y >= startY && y < startY+objH {
				isObjectCell = true
			}

			if isObjectCell {
				sb.WriteString("• ")
			} else {
				switch {
				case p == pf.start:
					sb.WriteString("S ")
				case p == pf.end:
					sb.WriteString("E ")
				case pathMap[p]:
					sb.WriteString("P ")
				case pf.grid[y][x] == 1:
					sb.WriteString("O ")
				default:
					sb.WriteString(". ")
				}
			}
		}
		sb.WriteString("\n")
	}
	return sb.String()
}

// isObstacle checks if a grid cell is covered by any obstacle.
func (pf *Pathfinder) isObstacle(x, y int) bool {
	cellRect := Rectangle{MinX: float64(x), MinY: float64(y), MaxX: float64(x + 1), MaxY: float64(y + 1)}
	for _, obs := range pf.obstacles {
		if rectsOverlap(cellRect, obs) {
			return true
		}
	}
	return false
}

// SaveObstacles saves the current obstacles to a JSON file.
func (pf *Pathfinder) SaveObstacles(filePath string) error {
	data, err := json.MarshalIndent(pf.obstacles, "", "  ")
	if err != nil {
		return fmt.Errorf("failed to marshal obstacles to JSON: %w", err)
	}

	if err := os.WriteFile(filePath, data, 0644); err != nil {
		return fmt.Errorf("failed to write file: %w", err)
	}

	return nil
}

// findPath runs A* on the integer grid and returns a slice of PointI.
// If the end point is not walkable, it finds the closest walkable point.
// If a path to the final destination cannot be found, it finds a path to the closest reachable point.
func (pf *Pathfinder) findPath(start, end PointI) ([]PointI, PointI, error) {

	// If the end point is not walkable, find the closest walkable point.
	if !pf.isWalkable(end.X, end.Y) {
		newEnd, err := pf.findClosestWalkablePoint(end)
		if err != nil {
			return nil, PointI{}, fmt.Errorf("end point not walkable and no close alternatives found")
		}
		end = newEnd
	}

	open := make(PriorityQueue, 0)
	heap.Init(&open)

	openSet := make(map[int]*Node)
	gScore := make(map[int]float64)

	startNode := &Node{X: start.X, Y: start.Y, g: 0, h: heuristic(start.X, start.Y, end.X, end.Y)}
	startNode.f = startNode.g + startNode.h
	heap.Push(&open, startNode)
	openSet[keyFrom(start.X, start.Y, pf.gridW)] = startNode
	gScore[keyFrom(start.X, start.Y, pf.gridW)] = 0

	// Track the node in the open set that is closest to the target
	var closestNode *Node = startNode

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
		delete(openSet, ck)

		// Check if we've found a better candidate for the closest node
		if current.h < closestNode.h {
			closestNode = current
		}

		if ck == keyFrom(end.X, end.Y, pf.gridW) {
			path := []PointI{}
			for n := current; n != nil; n = n.parent {
				path = append(path, PointI{n.X, n.Y})
			}
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			return path, end, nil
		}

		for _, d := range dirs {
			nx, ny := current.X+d.dx, current.Y+d.dy
			if !pf.isWalkable(nx, ny) {
				continue
			}

			tentG := current.g + d.cost
			nk := keyFrom(nx, ny, pf.gridW)

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
			openSet[nk] = node
		}
	}

	// If the loop finishes without finding a path, rebuild the path to the closest node
	if closestNode != nil {
		fmt.Println("Warning: Could not find a path to the exact destination. Found a path to the closest reachable point.")
		path := []PointI{}
		for n := closestNode; n != nil; n = n.parent {
			path = append(path, PointI{n.X, n.Y})
		}
		for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
			path[i], path[j] = path[j], path[i]
		}
		return path, PointI{X: closestNode.X, Y: closestNode.Y}, nil
	}

	return nil, end, fmt.Errorf("no path found")
}

// findClosestWalkablePoint searches for the nearest walkable point to a given point.
func (pf *Pathfinder) findClosestWalkablePoint(point PointI) (PointI, error) {
	// First, determine the valid range of coordinates for the object's center.
	// This ensures the object's bounding box is always within the grid.
	halfW := pf.objectW / 2.0
	halfH := pf.objectH / 2.0

	// Clamp the point to the range where the object's center can validly exist.
	clampedPoint := PointI{
		X: int(math.Max(math.Floor(halfW), math.Min(float64(point.X), float64(pf.gridW)-math.Ceil(halfW)))),
		Y: int(math.Max(math.Floor(halfH), math.Min(float64(point.Y), float64(pf.gridH)-math.Ceil(halfH)))),
	}

	// If the clamped point is already walkable, return it.
	if pf.isWalkable(clampedPoint.X, clampedPoint.Y) {
		return clampedPoint, nil
	}

	queue := []PointI{clampedPoint}
	visited := make(map[int]bool)
	visited[keyFrom(clampedPoint.X, clampedPoint.Y, pf.gridW)] = true

	// Explore outwards in a spiral pattern
	dirs := []struct {
		dx, dy int
	}{
		{1, 0}, {-1, 0}, {0, 1}, {0, -1},
		{1, 1}, {1, -1}, {-1, 1}, {-1, -1},
	}

	for len(queue) > 0 {
		curr := queue[0]
		queue = queue[1:]

		if pf.isWalkable(curr.X, curr.Y) {
			return curr, nil
		}

		for _, d := range dirs {
			nx, ny := curr.X+d.dx, curr.Y+d.dy
			// Ensure the next point is within the valid range for the object's center
			if float64(nx) >= math.Floor(halfW) && float64(nx) <= float64(pf.gridW)-math.Ceil(halfW) &&
				float64(ny) >= math.Floor(halfH) && float64(ny) <= float64(pf.gridH)-math.Ceil(halfH) {
				k := keyFrom(nx, ny, pf.gridW)
				if !visited[k] {
					visited[k] = true
					queue = append(queue, PointI{nx, ny})
				}
			}
		}
	}

	return PointI{}, fmt.Errorf("no walkable point found")
}

// isWalkable checks whether the object centered at (x,y) fits without colliding
// with any obstacle by sampling the grid cells that the object's bounding box covers.
func (pf *Pathfinder) isWalkable(x, y int) bool {
	_, reason := pf.isWalkableVerbose(x, y)
	return reason == ""
}

// isWalkableVerbose provides a detailed reason if a point is not walkable.
func (pf *Pathfinder) isWalkableVerbose(x, y int) (bool, string) {
	// Create a bounding box for the object at the given grid cell.
	halfW := pf.objectW / 2.0
	halfH := pf.objectH / 2.0
	objRect := Rectangle{
		MinX: float64(x) - halfW,
		MinY: float64(y) - halfH,
		MaxX: float64(x) + halfW,
		MaxY: float64(y) + halfH,
	}

	// Check for out-of-bounds collision or collision with obstacles.
	if objRect.MinX < 0 {
		return false, fmt.Sprintf("out of bounds (minX=%.2f < 0)", objRect.MinX)
	}
	if objRect.MaxX > float64(pf.gridW) {
		return false, fmt.Sprintf("out of bounds (maxX=%.2f > gridW=%d)", objRect.MaxX, pf.gridW)
	}
	if objRect.MinY < 0 {
		return false, fmt.Sprintf("out of bounds (minY=%.2f < 0)", objRect.MinY)
	}
	if objRect.MaxY > float64(pf.gridH) {
		return false, fmt.Sprintf("out of bounds (maxY=%.2f > gridH=%d)", objRect.MaxY, pf.gridH)
	}
	if pf.CheckCollision(objRect) {
		return false, "collides with an obstacle"
	}
	return true, ""
}

// CheckCollision checks if the given rectangle overlaps with any of the
// obstacles in the Pathfinder's map.
func (pf *Pathfinder) CheckCollision(rect Rectangle) bool {
	for _, obs := range pf.obstacles {
		if rectsOverlap(rect, obs) {
			return true
		}
	}
	return false
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

// validateStartPoint checks if the initial start point is walkable and finds a new one if not.
func validateStartPoint(pf *Pathfinder, start *PointI) {
	if walkable, reason := pf.isWalkableVerbose(start.X, start.Y); !walkable {
		fmt.Printf("Warning: The initial start point (%d, %d) is not walkable. Reason: %s. Searching for a new random start point.\n", start.X, start.Y, reason)

		foundNewStart := false
		const maxAttempts = 100
		for i := 0; i < maxAttempts; i++ {
			randX := rand.Intn(gridW)
			randY := rand.Intn(gridH)
			if pf.isWalkable(randX, randY) {
				start.X = randX
				start.Y = randY
				foundNewStart = true
				fmt.Printf("New walkable start point found at (%d, %d).\n", start.X, start.Y)
				break
			}
		}

		if !foundNewStart {
			fmt.Println("Error: Could not find a new walkable start point after multiple attempts.")
			os.Exit(1)
		}
	}
}

// validateEndPoint checks if the end point is walkable and finds a new one if not.
func validateEndPoint(pf *Pathfinder, end *PointI) {
	// The findClosestWalkablePoint function already handles out-of-bounds points by clamping them.
	// So we can directly call it without a separate bounds check here.
	if !pf.isWalkable(end.X, end.Y) {
		newEnd, err := pf.findClosestWalkablePoint(*end)
		if err != nil {
			fmt.Println("Error: End point is not walkable and no close alternatives found.")
			os.Exit(1)
		}
		*end = newEnd
		fmt.Printf("Warning: The initial end point was not walkable or was out of bounds. Using closest walkable point at (%d, %d).\n", newEnd.X, newEnd.Y)
	}
}

// runPathfindingAndAnimation executes the A* algorithm and animates the result if requested.
func runPathfindingAndAnimation(pf *Pathfinder) {
	start := pf.start
	end := pf.end

	fmt.Println("--------------------------------------------------")
	fmt.Printf("  Grid size: %dx%d\n", gridW, gridH)
	fmt.Printf("  Object size: %.2fx%.2f\n", objW, objH)
	fmt.Printf("  Start: (%d, %d)  End: (%d, %d)\n", start.X, start.Y, end.X, end.Y)
	fmt.Println("--------------------------------------------------")

	path, closestPoint, err := pf.findPath(start, end)
	if err != nil {
		fmt.Println("Error finding path:", err)
		os.Exit(1)
	}

	fmt.Println("Path found!")
	if show {
		fmt.Printf("Animating path to closest point (%d, %d)\n", closestPoint.X, closestPoint.Y)
		for i, p := range path {
			fmt.Printf("\033[H\033[2J") // Clear screen for animation
			fmt.Println(pf.AnimateGrid(p, path))
			fmt.Printf("Step %d of %d: (%d, %d)\n", i+1, len(path), p.X, p.Y)
			time.Sleep(100 * time.Millisecond) // Simulate animation delay
		}
	}
}
