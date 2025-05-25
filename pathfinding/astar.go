package pathfinding

import (
	"container/heap" // Standard library for priority queue (heap)
	"fmt"            // For formatted I/O
	"log"            // For logging events
	"math"           // For mathematical operations like sqrt and abs

	"cyberia-server/instance" // Updated import path to access InstanceState
)

// AStarNode represents a node in the A* algorithm's search space.
type AStarNode struct {
	X, Y   int        // Coordinates in the maze grid
	G      float64    // Cost from start node to this node
	H      float64    // Heuristic cost from this node to the end node
	F      float64    // Total estimated cost (G + H)
	Parent *AStarNode // Reference to the parent node for path reconstruction
	Index  int        // Index in the priority queue (required by container/heap)
}

// PriorityQueue implements heap.Interface for AStarNode to manage the open set.
type PriorityQueue []*AStarNode

func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
	// Lower F-score means higher priority (min-heap).
	return pq[i].F < pq[j].F
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].Index = i
	pq[j].Index = j
}

func (pq *PriorityQueue) Push(x interface{}) {
	n := len(*pq)
	node := x.(*AStarNode)
	node.Index = n
	*pq = append(*pq, node)
}

func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	node := old[n-1]
	old[n-1] = nil  // Avoid memory leaks
	node.Index = -1 // Mark as removed
	*pq = old[0 : n-1]
	return node
}

// heuristic calculates the squared Euclidean distance between two AStarNodes.
// This is an admissible and consistent heuristic for grid-based movement.
func heuristic(a, b *AStarNode) float64 {
	dx := float64(a.X - b.X)
	dy := float64(a.Y - b.Y)
	return (dx * dx) + (dy * dy)
}

// FindPath calculates an A* path between two world coordinates.
// It converts world coordinates to maze coordinates, performs A* search,
// and converts the resulting path back to world coordinates.
// It now takes an *instance.InstanceState to access the maze data.
func FindPath(is *instance.InstanceState, startWorldX, startWorldY, endWorldX, endWorldY float64) ([]struct{ X, Y float64 }, error) {
	is.Mu.RLock()         // Access exported Mu
	defer is.Mu.RUnlock() // Access exported Mu

	startMazeX, startMazeY := is.WorldToMazeCoords(startWorldX, startWorldY)
	endMazeX, endMazeY := is.WorldToMazeCoords(endWorldX, endWorldY)

	// Validate start and end positions: must be within maze bounds and walkable.
	if !(startMazeX >= 0 && startMazeX < is.MazeCellsX &&
		startMazeY >= 0 && startMazeY < is.MazeCellsY &&
		is.SimplifiedMaze[startMazeY][startMazeX] == 0) {
		return nil, fmt.Errorf("start position (maze: %d,%d) is invalid or an obstacle", startMazeX, startMazeY)
	}
	if !(endMazeX >= 0 && endMazeX < is.MazeCellsX &&
		endMazeY >= 0 && endMazeY < is.MazeCellsY &&
		is.SimplifiedMaze[endMazeY][endMazeX] == 0) {
		return nil, fmt.Errorf("end position (maze: %d,%d) is invalid or an obstacle", endMazeX, endMazeY)
	}

	log.Printf("Pathfinding: from maze (%d,%d) to (%d,%d)...\n", startMazeX, startMazeY, endMazeX, endMazeY)

	startNode := &AStarNode{X: startMazeX, Y: startMazeY, G: 0}
	endNode := &AStarNode{X: endMazeX, Y: endMazeY}

	startNode.H = heuristic(startNode, endNode)
	startNode.F = startNode.G + startNode.H

	openSet := make(PriorityQueue, 0)
	heap.Push(&openSet, startNode)

	// gScore stores the cost from start to a node.
	gScore := make(map[string]float64)
	gScore[fmt.Sprintf("%d,%d", startNode.X, startNode.Y)] = 0

	// cameFrom stores the path reconstruction chain.
	cameFrom := make(map[string]*AStarNode)

	// Directions for 8-way movement (cardinal and diagonal).
	// Costs: 1.0 for cardinal, sqrt(2) for diagonal.
	neighborsDirs := []struct {
		dx, dy int
		cost   float64
	}{
		{0, 1, 1.0},          // Down
		{0, -1, 1.0},         // Up
		{1, 0, 1.0},          // Right
		{-1, 0, 1.0},         // Left
		{1, 1, math.Sqrt2},   // Down-Right
		{1, -1, math.Sqrt2},  // Up-Right
		{-1, 1, math.Sqrt2},  // Down-Left
		{-1, -1, math.Sqrt2}, // Up-Left
	}

	// Safety break to prevent infinite loops in unpathable or complex mazes.
	maxIterations := (is.MazeCellsX * is.MazeCellsY * 2)
	iterations := 0

	for openSet.Len() > 0 {
		iterations++
		if iterations > maxIterations {
			log.Printf("WARNING: Pathfinding aborted due to too many iterations. No path found within limits.")
			return nil, fmt.Errorf("pathfinding aborted: too many iterations")
		}

		current := heap.Pop(&openSet).(*AStarNode)
		currentKey := fmt.Sprintf("%d,%d", current.X, current.Y)

		// If the goal is reached, reconstruct and return the path.
		if current.X == endMazeX && current.Y == endMazeY {
			pathMaze := []struct{ X, Y int }{}
			for current != nil {
				pathMaze = append([]struct{ X, Y int }{{X: current.X, Y: current.Y}}, pathMaze...)
				current = cameFrom[fmt.Sprintf("%d,%d", current.X, current.Y)]
			}

			// Convert maze coordinates path to world coordinates.
			pathWorld := make([]struct{ X, Y float64 }, len(pathMaze))
			for i, p := range pathMaze {
				pathWorld[i].X, pathWorld[i].Y = is.MazeToWorldCoords(p.X, p.Y)
			}
			log.Printf("Pathfinding: Path found with %d steps.\n", len(pathWorld))
			return pathWorld, nil
		}

		// Explore neighbors of the current node.
		for _, dir := range neighborsDirs {
			neighborX, neighborY := current.X+dir.dx, current.Y+dir.dy
			neighborKey := fmt.Sprintf("%d,%d", neighborX, neighborY)

			// Check if neighbor is within bounds and not an obstacle.
			if neighborX < 0 || neighborX >= is.MazeCellsX ||
				neighborY < 0 || neighborY >= is.MazeCellsY ||
				is.SimplifiedMaze[neighborY][neighborX] == 1 {
				continue // Skip invalid or obstructed neighbors
			}

			tentativeGScore := gScore[currentKey] + dir.cost

			// If this path to neighbor is better than any previous one, update it.
			if val, ok := gScore[neighborKey]; !ok || tentativeGScore < val {
				newNode := &AStarNode{X: neighborX, Y: neighborY}
				cameFrom[neighborKey] = current
				gScore[neighborKey] = tentativeGScore
				newNode.G = tentativeGScore
				newNode.H = heuristic(newNode, endNode)
				newNode.F = newNode.G + newNode.H

				// Add or update neighbor in the open set.
				foundInOpenSet := false
				for i, node := range openSet {
					if node.X == newNode.X && node.Y == newNode.Y {
						if newNode.F < node.F { // If new path is better, update priority
							openSet[i] = newNode
							heap.Fix(&openSet, i) // Re-heapify after update
						}
						foundInOpenSet = true
						break
					}
				}
				if !foundInOpenSet {
					heap.Push(&openSet, newNode) // Add new node to open set
				}
			}
		}
	}

	log.Printf("Pathfinding: No path found to destination.")
	return nil, fmt.Errorf("no path found")
}
