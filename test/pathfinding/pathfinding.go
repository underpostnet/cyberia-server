package pathfinding

import (
	"container/heap"
	"fmt"
	"log"
	"math"

	"cyberia-server/test/network_state" // Updated import path
)

// AStarNode represents a node in the A* algorithm's search space.
type AStarNode struct {
	X, Y   int
	G      float64
	H      float64
	F      float64
	Parent *AStarNode
	Index  int
}

// PriorityQueue implements heap.Interface for AStarNode to manage the open set.
type PriorityQueue []*AStarNode

func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
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
	old[n-1] = nil
	node.Index = -1
	*pq = old[0 : n-1]
	return node
}

// heuristic calculates the squared Euclidean distance between two AStarNodes.
func heuristic(a, b *AStarNode) float64 {
	dx := float64(a.X - b.X)
	dy := float64(a.Y - b.Y)
	return (dx * dx) + (dy * dy)
}

// FindPath calculates an A* path between two world coordinates.
// It converts world coordinates to maze coordinates, performs A* search,
// and converts the resulting path back to world coordinates.
// It now takes an *network_state.NetworkState to access the maze data.
func FindPath(ns *network_state.NetworkState, startWorldX, startWorldY, endWorldX, endWorldY float64) ([]struct{ X, Y float64 }, error) {
	ns.Mu.RLock()
	defer ns.Mu.RUnlock()

	startMazeX, startMazeY := ns.WorldToMazeCoords(startWorldX, startWorldY)
	endMazeX, endMazeY := ns.WorldToMazeCoords(endWorldX, endWorldY)

	// Validate start and end positions: must be within maze bounds and walkable.
	if !(startMazeX >= 0 && startMazeX < ns.MazeCellsX &&
		startMazeY >= 0 && startMazeY < ns.MazeCellsY &&
		ns.SimplifiedMaze[startMazeY][startMazeX] == 0) {
		return nil, fmt.Errorf("start position (maze: %d,%d) is invalid or an obstacle", startMazeX, startMazeY)
	}
	if !(endMazeX >= 0 && endMazeX < ns.MazeCellsX &&
		endMazeY >= 0 && endMazeY < ns.MazeCellsY &&
		ns.SimplifiedMaze[endMazeY][endMazeX] == 0) {
		return nil, fmt.Errorf("end position (maze: %d,%d) is invalid or an obstacle", endMazeX, endMazeY)
	}

	log.Printf("Pathfinding: from maze (%d,%d) to (%d,%d)...\n", startMazeX, startMazeY, endMazeX, endMazeY)

	startNode := &AStarNode{X: startMazeX, Y: startMazeY, G: 0}
	endNode := &AStarNode{X: endMazeX, Y: endMazeY}

	startNode.H = heuristic(startNode, endNode)
	startNode.F = startNode.G + startNode.H

	openSet := make(PriorityQueue, 0)
	heap.Push(&openSet, startNode)

	gScore := make(map[string]float64)
	gScore[fmt.Sprintf("%d,%d", startNode.X, startNode.Y)] = 0

	cameFrom := make(map[string]*AStarNode)

	neighborsDirs := []struct {
		dx, dy int
		cost   float64
	}{
		{0, 1, 1.0},
		{0, -1, 1.0},
		{1, 0, 1.0},
		{-1, 0, 1.0},
		{1, 1, math.Sqrt2},
		{1, -1, math.Sqrt2},
		{-1, 1, math.Sqrt2},
		{-1, -1, math.Sqrt2},
	}

	maxIterations := (ns.MazeCellsX * ns.MazeCellsY * 2)
	iterations := 0

	for openSet.Len() > 0 {
		iterations++
		if iterations > maxIterations {
			log.Printf("WARNING: Pathfinding aborted due to too many iterations. No path found within limits.")
			return nil, fmt.Errorf("pathfinding aborted: too many iterations")
		}

		current := heap.Pop(&openSet).(*AStarNode)
		currentKey := fmt.Sprintf("%d,%d", current.X, current.Y)

		if current.X == endMazeX && current.Y == endMazeY {
			pathMaze := []struct{ X, Y int }{}
			for current != nil {
				pathMaze = append([]struct{ X, Y int }{{X: current.X, Y: current.Y}}, pathMaze...)
				current = cameFrom[fmt.Sprintf("%d,%d", current.X, current.Y)]
			}

			pathWorld := make([]struct{ X, Y float64 }, len(pathMaze))
			for i, p := range pathMaze {
				pathWorld[i].X, pathWorld[i].Y = ns.MazeToWorldCoords(p.X, p.Y)
			}
			log.Printf("Pathfinding: Path found with %d steps.\n", len(pathWorld))
			return pathWorld, nil
		}

		for _, dir := range neighborsDirs {
			neighborX, neighborY := current.X+dir.dx, current.Y+dir.dy
			neighborKey := fmt.Sprintf("%d,%d", neighborX, neighborY)

			if neighborX < 0 || neighborX >= ns.MazeCellsX ||
				neighborY < 0 || neighborY >= ns.MazeCellsY ||
				ns.SimplifiedMaze[neighborY][neighborX] == 1 {
				continue
			}

			tentativeGScore := gScore[currentKey] + dir.cost

			if val, ok := gScore[neighborKey]; !ok || tentativeGScore < val {
				newNode := &AStarNode{X: neighborX, Y: neighborY}
				cameFrom[neighborKey] = current
				gScore[neighborKey] = tentativeGScore
				newNode.G = tentativeGScore
				newNode.H = heuristic(newNode, endNode)
				newNode.F = newNode.G + newNode.H

				foundInOpenSet := false
				for i, node := range openSet {
					if node.X == newNode.X && node.Y == newNode.Y {
						if newNode.F < node.F {
							openSet[i] = newNode
							heap.Fix(&openSet, i)
						}
						foundInOpenSet = true
						break
					}
				}
				if !foundInOpenSet {
					heap.Push(&openSet, newNode)
				}
			}
		}
	}

	log.Printf("Pathfinding: No path found to destination.")
	return nil, fmt.Errorf("no path found")
}
