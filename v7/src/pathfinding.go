package game

import (
	"container/heap"
	"fmt"
	"log"
	"math"
	"math/rand"
	"time"

	"github.com/google/uuid"
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

// GenerateObstacles places random obstacles on the map, avoiding specified portal rectangles.
func (pf *Pathfinder) GenerateObstacles(numObstacles int, portalRects []Rectangle) {
	pf.obstacles = make(map[string]ObjectState)
	for y := range pf.grid {
		for x := range pf.grid[y] {
			pf.grid[y][x] = 0
		}
	}

	maxObsW, maxObsH, minObsDim := int(math.Max(2, float64(pf.gridW)/15.0)), int(math.Max(2, float64(pf.gridH)/15.0)), 2
	rand.Seed(time.Now().UnixNano())
	for placed, attempts := 0, 0; placed < numObstacles && attempts < numObstacles*10; attempts++ {
		w, h := rand.Intn(maxObsW-minObsDim+1) + minObsDim, rand.Intn(maxObsH-minObsDim+1) + minObsDim
		minX, minY := rand.Intn(pf.gridW - w), rand.Intn(pf.gridH - h)
		obsRect := Rectangle{MinX: float64(minX), MinY: float64(minY), MaxX: float64(minX + w), MaxY: float64(minY + h)}

		overlap := false
		for _, pRect := range portalRects {
			if rectsOverlap(obsRect, pRect) {
				overlap = true
				break
			}
		}
		if overlap {
			continue
		}

		for _, existingObs := range pf.obstacles {
			existingRect := Rectangle{
				MinX: existingObs.Pos.X, MinY: existingObs.Pos.Y,
				MaxX: existingObs.Pos.X + existingObs.Dims.Width, MaxY: existingObs.Pos.Y + existingObs.Dims.Height,
			}
			if rectsOverlap(obsRect, existingRect) {
				overlap = true
				break
			}
		}
		if overlap {
			continue
		}

		obs := ObjectState{
			ID:   uuid.New().String(),
			Pos:  Point{X: float64(minX), Y: float64(minY)},
			Dims: Dimensions{Width: float64(w), Height: float64(h)},
			Type: "obstacle",
		}
		for y := minY; y < minY+h; y++ {
			for x := minX; x < minX+w; x++ {
				if x >= 0 && x < pf.gridW && y >= 0 && y < pf.gridH {
					pf.grid[y][x] = 1
				}
			}
		}
		pf.obstacles[obs.ID] = obs
		placed++
	}
	log.Printf("Map generation complete. Placed %d obstacles.", len(pf.obstacles))
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
	queue := []PointI{p}
	visited := make(map[PointI]bool)
	visited[p] = true
	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]
		if pf.isWalkable(current.X, current.Y, playerDims) {
			return current, nil
		}
		for _, move := range []PointI{{1, 0}, {-1, 0}, {0, 1}, {0, -1}} {
			neighbor := PointI{X: current.X + move.X, Y: current.Y + move.Y}
			if neighbor.X >= 0 && neighbor.X < pf.gridW && neighbor.Y >= 0 && neighbor.Y < pf.gridH && !visited[neighbor] {
				visited[neighbor] = true
				queue = append(queue, neighbor)
			}
		}
	}
	return PointI{}, fmt.Errorf("no walkable point found near (%d, %d)", p.X, p.Y)
}

// Astar performs the A* pathfinding algorithm.
func (pf *Pathfinder) Astar(start, end PointI, playerDims Dimensions) ([]PointI, error) {
	openSet := make(PriorityQueue, 0)
	heap.Init(&openSet)
	startNode := &Node{X: start.X, Y: start.Y, h: heuristic(start.X, start.Y, end.X, end.Y)}
	heap.Push(&openSet, startNode)
	gScore := make(map[PointI]float64)
	gScore[start] = 0.0
	for openSet.Len() > 0 {
		current := heap.Pop(&openSet).(*Node)
		if current.X == end.X && current.Y == end.Y {
			return reconstructPath(current), nil
		}
		for _, move := range []PointI{{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}} {
			neighborPos := PointI{X: current.X + move.X, Y: current.Y + move.Y}
			if !pf.isWalkable(neighborPos.X, neighborPos.Y, playerDims) {
				continue
			}
			tentativeGScore := gScore[PointI{X: current.X, Y: current.Y}] + 1
			if val, ok := gScore[neighborPos]; !ok || tentativeGScore < val {
				if neighborNode := findNodeInPQ(&openSet, neighborPos); neighborNode != nil {
					if tentativeGScore < neighborNode.g {
						neighborNode.g = tentativeGScore
						neighborNode.f = neighborNode.g + neighborNode.h
						neighborNode.parent = current
						heap.Fix(&openSet, neighborNode.index)
					}
				} else {
					newNode := &Node{
						X:      neighborPos.X,
						Y:      neighborPos.Y,
						g:      tentativeGScore,
						h:      heuristic(neighborPos.X, neighborPos.Y, end.X, end.Y),
						parent: current,
					}
					newNode.f = newNode.g + newNode.h
					gScore[neighborPos] = newNode.g
					heap.Push(&openSet, newNode)
				}
			}
		}
	}
	return nil, fmt.Errorf("could not find a path")
}

func heuristic(x1, y1, x2, y2 int) float64 {
	return math.Abs(float64(x1-x2)) + math.Abs(float64(y1-y2))
}

func findNodeInPQ(pq *PriorityQueue, p PointI) *Node {
	for _, node := range *pq {
		if node.X == p.X && node.Y == p.Y {
			return node
		}
	}
	return nil
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
