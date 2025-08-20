package main

import (
	"container/heap"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"math/rand"
	"net/http"
	"sync"
	"time"

	"github.com/google/uuid"
	"github.com/gorilla/websocket"
)

//----------------------------------------------------------------------------------------------------------------------
// 1. Data Structures & Interfaces
//----------------------------------------------------------------------------------------------------------------------

type Point struct {
	X float64 `json:"X"`
	Y float64 `json:"Y"`
}

type PointI struct {
	X, Y int
}

type Rectangle struct {
	MinX, MinY, MaxX, MaxY float64
}

type Dimensions struct {
	Width  float64 `json:"Width"`
	Height float64 `json:"Height"`
}

type Direction int

const (
	UP Direction = iota
	UP_RIGHT
	RIGHT
	DOWN_RIGHT
	DOWN
	DOWN_LEFT
	LEFT
	UP_LEFT
	NONE
)

type ObjectLayerMode int

const (
	IDLE ObjectLayerMode = iota
	WALKING
	TELEPORTING
)

type ObjectState struct {
	ID          string     `json:"id"`
	Pos         Point      `json:"Pos"`
	Dims        Dimensions `json:"Dims"`
	Type        string     `json:"Type"`
	PortalLabel string     `json:"PortalLabel,omitempty"`
}

type PlayerState struct {
	ID             string          `json:"id"`
	MapID          int             `json:"MapID"`
	Pos            Point           `json:"Pos"`
	Dims           Dimensions      `json:"Dims"`
	Path           []PointI        `json:"path"`
	TargetPos      PointI          `json:"targetPos"`
	AOI            Rectangle       `json:"AOI"`
	Client         *Client         `json:"-"`
	Direction      Direction       `json:"direction"`
	Mode           ObjectLayerMode `json:"mode"`
	OnPortal       bool            `json:"onPortal"`
	TimeOnPortal   time.Time       `json:"-"`
	ActivePortalID string          `json:"activePortalID"`
	SumStatsLimit  int             `json:"sumStatsLimit"`
}

type BotState struct {
	ID                   string          `json:"id"`
	MapID                int             `json:"MapID"`
	Pos                  Point           `json:"Pos"`
	Dims                 Dimensions      `json:"Dims"`
	Path                 []PointI        `json:"path"`
	TargetPos            PointI          `json:"targetPos"`
	Direction            Direction       `json:"direction"`
	Mode                 ObjectLayerMode `json:"mode"`
	Behavior             string          `json:"behavior"` // "hostile" or "passive"
	SpawnCenter          Point           `json:"spawnCenter"`
	SpawnRadius          float64         `json:"spawnRadius"`
	AggroRange           float64         `json:"aggroRange"`
	CurrentTargetPlayer  string          `json:"-"` // player ID currently being pursued (if any)
	lastPursuitTargetPos PointI          `json:"-"` // cached player's last cell to know when to re-path
}

type PortalConfig struct {
	DestMapID       int
	DestPortalIndex int
	SpawnRadius     float64
}

type PortalState struct {
	ID           string
	Pos          Point
	Dims         Dimensions
	PortalConfig *PortalConfig
	Label        string `json:"Label"`
}

type MapState struct {
	pathfinder   *Pathfinder
	obstacles    map[string]ObjectState
	foregrounds  map[string]ObjectState
	portals      map[string]*PortalState
	players      map[string]*PlayerState
	bots         map[string]*BotState
	gridW, gridH int
}

type Client struct {
	conn        *websocket.Conn
	playerID    string
	send        chan []byte
	lastAction  time.Time
	playerState *PlayerState
}

type GameServer struct {
	mu             sync.Mutex
	maps           map[int]*MapState
	clients        map[string]*Client
	register       chan *Client
	unregister     chan *Client
	aoiRadius      float64
	portalHoldTime time.Duration
	playerSpeed    float64

	cellSize         float64
	fps              int
	interpolationMs  int
	defaultObjWidth  float64
	defaultObjHeight float64
	colors           map[string]ColorRGBA

	cameraSmoothing float64
	cameraZoom      float64

	defaultWidthScreenFactor  float64
	defaultHeightScreenFactor float64

	devUi bool

	// bot related defaults
	botsPerMap    int
	botAggroRange float64
}

type ColorRGBA struct {
	R int `json:"r"`
	G int `json:"g"`
	B int `json:"b"`
	A int `json:"a"`
}

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

type Pathfinder struct {
	gridW, gridH int
	obstacles    map[string]ObjectState
	grid         [][]int
}

//----------------------------------------------------------------------------------------------------------------------
// 2. Core Logic & Methods
//----------------------------------------------------------------------------------------------------------------------

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
	for placed, attempts := 0, 0; placed < numObstacles && attempts < numObstacles*10; attempts++ {
		w, h := rand.Intn(maxObsW-minObsDim+1)+minObsDim, rand.Intn(maxObsH-minObsDim+1)+minObsDim
		minX, minY := rand.Intn(pf.gridW-w), rand.Intn(pf.gridH-h)
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
			ID: uuid.New().String(), Pos: Point{X: float64(minX), Y: float64(minY)},
			Dims: Dimensions{Width: float64(w), Height: float64(h)}, Type: "obstacle",
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

// generatePortals creates and places portals on a map.
func (ms *MapState) generatePortals(numPortals int) []*PortalState {
	portals := make([]*PortalState, 0, numPortals)
	for i := 0; i < numPortals; i++ {
		dims := Dimensions{Width: float64(rand.Intn(3) + 2), Height: float64(rand.Intn(3) + 2)}
		posI, err := ms.pathfinder.findRandomWalkablePoint(dims)
		if err != nil {
			log.Printf("Error placing portal: %v", err)
			continue
		}
		portal := &PortalState{
			ID: uuid.New().String(), Pos: Point{X: float64(posI.X), Y: float64(posI.Y)}, Dims: dims,
		}
		portals = append(portals, portal)
		for y := posI.Y; y < posI.Y+int(dims.Height); y++ {
			for x := posI.X; x < posI.X+int(dims.Width); x++ {
				if x >= 0 && x < ms.gridW && y >= 0 && y < ms.gridH {
					ms.pathfinder.grid[y][x] = 1
				}
			}
		}
	}
	return portals
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

// NewGameServer creates a new game server.
func NewGameServer() *GameServer {
	gs := &GameServer{
		maps:           make(map[int]*MapState),
		clients:        make(map[string]*Client),
		register:       make(chan *Client),
		unregister:     make(chan *Client),
		aoiRadius:      15.0,
		portalHoldTime: 2 * time.Second,
		playerSpeed:    24.0,

		cellSize:                  12.0,
		fps:                       60,
		interpolationMs:           200,
		defaultObjWidth:           1.0,
		defaultObjHeight:          1.0,
		cameraSmoothing:           0.15,
		cameraZoom:                2,
		defaultWidthScreenFactor:  1,
		defaultHeightScreenFactor: 0.95,
		devUi:                     true,
		botsPerMap:                10,
		botAggroRange:             10.0, // default aggression range (x)
		colors: map[string]ColorRGBA{
			"BACKGROUND":   {R: 30, G: 30, B: 30, A: 255},
			"OBSTACLE":     {R: 100, G: 100, B: 100, A: 255},
			"FOREGROUND":   {R: 60, G: 140, B: 60, A: 220},
			"PLAYER":       {R: 0, G: 200, B: 255, A: 255},
			"OTHER_PLAYER": {R: 255, G: 100, B: 0, A: 255},
			"PATH":         {R: 255, G: 255, B: 0, A: 128},
			"TARGET":       {R: 255, G: 255, B: 0, A: 255},
			"AOI":          {R: 255, G: 0, B: 255, A: 51},
			"DEBUG_TEXT":   {R: 220, G: 220, B: 220, A: 255},
			"ERROR_TEXT":   {R: 255, G: 50, B: 50, A: 255},
			"PORTAL":       {R: 180, G: 50, B: 255, A: 180},
			"PORTAL_LABEL": {R: 240, G: 240, B: 240, A: 255},
			"UI_TEXT":      {R: 255, G: 255, B: 255, A: 255},
			"MAP_BOUNDARY": {R: 255, G: 255, B: 255, A: 255},
		},
	}
	gs.createMaps()
	return gs
}

func (s *GameServer) createMaps() {
	log.Println("Creating game maps...")
	const (
		numMaps    = 3
		gridSizeW  = 100
		gridSizeH  = 100
		numObs     = 100
		numPortals = 2
	)

	portalConfigs := [numMaps][]PortalConfig{
		{
			{DestMapID: 1, DestPortalIndex: 0, SpawnRadius: 2.0},
			{DestMapID: 0, DestPortalIndex: 1, SpawnRadius: 2.0},
		},
		{
			{DestMapID: 0, DestPortalIndex: 0, SpawnRadius: 2.0},
			{DestMapID: 2, DestPortalIndex: 1, SpawnRadius: 2.0},
		},
		{
			{DestMapID: 1, DestPortalIndex: 1, SpawnRadius: 2.0},
			{DestMapID: 2, DestPortalIndex: 0, SpawnRadius: 2.0},
		},
	}

	allMapsPortals = make([][]*PortalState, numMaps)

	for i := 0; i < numMaps; i++ {
		ms := &MapState{
			gridW:       gridSizeW,
			gridH:       gridSizeH,
			players:     make(map[string]*PlayerState),
			portals:     make(map[string]*PortalState),
			obstacles:   make(map[string]ObjectState),
			foregrounds: make(map[string]ObjectState),
			pathfinder:  NewPathfinder(gridSizeW, gridSizeH),
			bots:        make(map[string]*BotState),
		}

		portals := ms.generatePortals(numPortals)
		allMapsPortals[i] = portals

		for _, p := range portals {
			ms.portals[p.ID] = p
		}

		portalRects := make([]Rectangle, len(portals))
		for idx, p := range portals {
			portalRects[idx] = Rectangle{MinX: p.Pos.X, MinY: p.Pos.Y, MaxX: p.Pos.X + p.Dims.Width, MaxY: p.Pos.Y + p.Dims.Height}
		}

		ms.pathfinder.GenerateObstacles(numObs, portalRects)
		ms.obstacles = ms.pathfinder.obstacles

		for _, obs := range ms.obstacles {
			fg := ObjectState{
				ID:   uuid.New().String(),
				Pos:  Point{X: obs.Pos.X, Y: obs.Pos.Y - obs.Dims.Height},
				Dims: obs.Dims,
				Type: "foreground",
			}
			ms.foregrounds[fg.ID] = fg
		}

		// instantiate bots for this map
		s.instantiateBots(ms, i)

		s.maps[i] = ms
	}

	for mapID, portals := range allMapsPortals {
		for portalIndex, portal := range portals {
			portal.PortalConfig = &portalConfigs[mapID][portalIndex]
			destPortal := allMapsPortals[portal.PortalConfig.DestMapID][portal.PortalConfig.DestPortalIndex]
			portal.Label = fmt.Sprintf("Map %d, Pos: (%d, %d)",
				portal.PortalConfig.DestMapID,
				int(destPortal.Pos.X),
				int(destPortal.Pos.Y),
			)
		}
	}

	log.Println("Maps created and portals linked.")
}

// instantiateBots creates bots for a map with random passive/hostile distribution.
func (s *GameServer) instantiateBots(ms *MapState, mapID int) {
	for i := 0; i < s.botsPerMap; i++ {
		// random dims and spawn point
		dims := Dimensions{Width: float64(rand.Intn(4) + 1), Height: float64(rand.Intn(4) + 1)}
		startPosI, err := ms.pathfinder.findRandomWalkablePoint(dims)
		if err != nil {
			continue
		}
		startPos := Point{X: float64(startPosI.X), Y: float64(startPosI.Y)}
		spawnRadius := rand.Float64()*6.0 + 3.0 // 3..9
		behavior := "passive"
		if rand.Intn(2) == 0 {
			behavior = "hostile"
		}

		bot := &BotState{
			ID:          uuid.New().String(),
			MapID:       mapID,
			Pos:         startPos,
			Dims:        dims,
			Path:        []PointI{},
			TargetPos:   PointI{-1, -1},
			Direction:   NONE,
			Mode:        IDLE,
			Behavior:    behavior,
			SpawnCenter: startPos,
			SpawnRadius: spawnRadius,
			AggroRange:  s.botAggroRange,
		}

		// initial wandering path: random point within spawn radius (both passive and hostile should wander initially)
		target := s.randomPointWithinRadius(ms, bot.SpawnCenter, bot.SpawnRadius, bot.Dims)
		if target.X >= 0 {
			if pth, err := ms.pathfinder.Astar(PointI{X: int(math.Round(bot.Pos.X)), Y: int(math.Round(bot.Pos.Y))}, target, bot.Dims); err == nil && len(pth) > 0 {
				bot.Path = pth
				bot.TargetPos = target
				bot.Mode = WALKING
			}
		}
		ms.bots[bot.ID] = bot
	}
}

// randomPointWithinRadius returns a random walkable PointI within radius from center (or {-1,-1} on failure).
func (s *GameServer) randomPointWithinRadius(ms *MapState, center Point, radius float64, dims Dimensions) PointI {
	for tries := 0; tries < 30; tries++ {
		// random angle + distance
		ang := rand.Float64() * 2 * math.Pi
		r := rand.Float64() * radius
		x := int(math.Round(center.X + math.Cos(ang)*r))
		y := int(math.Round(center.Y + math.Sin(ang)*r))
		if x < 0 || x >= ms.gridW || y < 0 || y >= ms.gridH {
			continue
		}
		if ms.pathfinder.isWalkable(x, y, dims) {
			return PointI{X: x, Y: y}
		}
		// try closest walkable
		if c, err := ms.pathfinder.findClosestWalkablePoint(PointI{X: x, Y: y}, dims); err == nil {
			return c
		}
	}
	return PointI{-1, -1}
}

func (s *GameServer) Run() {
	go s.listenForClients()
	go s.gameLoop()
}

func (s *GameServer) listenForClients() {
	log.Println("Starting client listener...")
	for {
		select {
		case client := <-s.register:
			s.mu.Lock()
			s.clients[client.playerID] = client
			s.mu.Unlock()
		case client := <-s.unregister:
			s.mu.Lock()
			if _, ok := s.clients[client.playerID]; ok {
				delete(s.clients, client.playerID)
				playerState := client.playerState
				if playerState != nil {
					mapState, ok := s.maps[playerState.MapID]
					if ok {
						delete(mapState.players, client.playerID)
					}
				}
				close(client.send)
			}
			s.mu.Unlock()
		}
	}
}

func (s *GameServer) gameLoop() {
	ticker := time.NewTicker(100 * time.Millisecond) // 10 FPS
	defer ticker.Stop()

	for range ticker.C {
		s.mu.Lock()
		for _, mapState := range s.maps {
			// update players
			for _, player := range mapState.players {
				s.updatePlayerPosition(player, mapState)
				s.checkPortal(player, mapState)
			}
			// update bots (bot AI)
			s.updateBots(mapState)
			// update AOIs
			s.updateAOIs(mapState)
		}
		s.mu.Unlock()
	}
}

// update bots per map
func (s *GameServer) updateBots(mapState *MapState) {
	for _, bot := range mapState.bots {
		// Hostile logic: check for nearest player in aggro range
		if bot.Behavior == "hostile" {
			nearestID := ""
			nearestDist := math.MaxFloat64
			var nearestPlayer *PlayerState
			for _, p := range mapState.players {
				dx := p.Pos.X - bot.Pos.X
				dy := p.Pos.Y - bot.Pos.Y
				dist := math.Sqrt(dx*dx + dy*dy)
				if dist < nearestDist {
					nearestDist = dist
					nearestID = p.ID
					nearestPlayer = p
				}
			}
			if nearestPlayer != nil && nearestDist <= bot.AggroRange {
				// Pursue the player: recompute path if newly acquired or player moved cell
				playerCell := PointI{X: int(math.Round(nearestPlayer.Pos.X)), Y: int(math.Round(nearestPlayer.Pos.Y))}
				needRepath := bot.CurrentTargetPlayer != nearestID || bot.lastPursuitTargetPos != playerCell
				if needRepath {
					start := PointI{X: int(math.Round(bot.Pos.X)), Y: int(math.Round(bot.Pos.Y))}
					target, err := mapState.pathfinder.findClosestWalkablePoint(playerCell, bot.Dims)
					if err == nil {
						if p, err2 := mapState.pathfinder.Astar(start, target, bot.Dims); err2 == nil && len(p) > 0 {
							bot.Path = p
							bot.TargetPos = target
							bot.Mode = WALKING
							bot.CurrentTargetPlayer = nearestID
							bot.lastPursuitTargetPos = playerCell
						}
					}
				}
			} else {
				// No player in range: clear pursuit (if any) and ensure wandering behavior
				if bot.CurrentTargetPlayer != "" {
					bot.CurrentTargetPlayer = ""
					bot.lastPursuitTargetPos = PointI{-1, -1}
				}

				// IMPORTANT CHANGE: When not pursuing, hostile bots behave like passive bots:
				// - if they have no current walking path, generate a new random wander target within spawn radius.
				if bot.Mode != WALKING || len(bot.Path) == 0 {
					target := s.randomPointWithinRadius(mapState, bot.SpawnCenter, bot.SpawnRadius, bot.Dims)
					if target.X >= 0 {
						if pth, err := mapState.pathfinder.Astar(PointI{X: int(math.Round(bot.Pos.X)), Y: int(math.Round(bot.Pos.Y))}, target, bot.Dims); err == nil && len(pth) > 0 {
							bot.Path = pth
							bot.TargetPos = target
							bot.Mode = WALKING
						} else {
							// if we can't path to a target, remain idle until next tick tries again
							if len(bot.Path) == 0 {
								bot.Mode = IDLE
							}
						}
					}
				}
			}
		} else {
			// Passive: if no path, create wandering path
			if len(bot.Path) == 0 || bot.Mode != WALKING {
				target := s.randomPointWithinRadius(mapState, bot.SpawnCenter, bot.SpawnRadius, bot.Dims)
				if target.X >= 0 {
					if pth, err := mapState.pathfinder.Astar(PointI{X: int(math.Round(bot.Pos.X)), Y: int(math.Round(bot.Pos.Y))}, target, bot.Dims); err == nil && len(pth) > 0 {
						bot.Path = pth
						bot.TargetPos = target
						bot.Mode = WALKING
					}
				}
			}
		}
		// Move bot along its path
		s.updateBotPosition(bot, mapState)
	}
}

// update bot movement similar to players
func (s *GameServer) updateBotPosition(bot *BotState, mapState *MapState) {
	if bot.Mode == WALKING && len(bot.Path) > 0 {
		targetNode := bot.Path[0]
		dx := float64(targetNode.X) - bot.Pos.X
		dy := float64(targetNode.Y) - bot.Pos.Y
		dist := math.Sqrt(dx*dx + dy*dy)
		step := s.playerSpeed / 10.0 // same tick scaling as players

		if dist < step {
			// snap
			bot.Pos = Point{X: float64(targetNode.X), Y: float64(targetNode.Y)}
			bot.Path = bot.Path[1:]
			if len(bot.Path) == 0 {
				bot.Mode = IDLE
				bot.Direction = NONE
				// Next tick, wandering behavior (passive or hostile-not-pursuing) will assign a new target
			} else {
				// compute direction toward next node
				next := bot.Path[0]
				dirX := float64(next.X) - bot.Pos.X
				dirY := float64(next.Y) - bot.Pos.Y
				norm := math.Sqrt(dirX*dirX + dirY*dirY)
				if norm > 0 {
					dirX /= norm
					dirY /= norm
					s.updateBotDirection(bot, dirX, dirY)
				}
			}
		} else {
			// normal interpolation
			dirX, dirY := dx/dist, dy/dist
			bot.Pos.X += dirX * step
			bot.Pos.Y += dirY * step
			s.updateBotDirection(bot, dirX, dirY)
		}
	}
}

func (s *GameServer) updateBotDirection(bot *BotState, dirX, dirY float64) {
	angle := math.Atan2(dirY, dirX)
	if angle < 0 {
		angle += 2 * math.Pi
	}
	directionIndex := (int(math.Round(angle/(math.Pi/4))) + 2) % 8
	bot.Direction = Direction(directionIndex)
}

// ---------- CHANGE: updatePlayerPosition now recalculates direction when arriving to a node ----------
func (s *GameServer) updatePlayerPosition(player *PlayerState, mapState *MapState) {
	if player.Mode == WALKING && len(player.Path) > 0 {
		targetNode := player.Path[0]
		dx := float64(targetNode.X) - player.Pos.X
		dy := float64(targetNode.Y) - player.Pos.Y

		dist := math.Sqrt(dx*dx + dy*dy)
		step := s.playerSpeed / 10.0 // Calculate movement step based on FPS

		if dist < step {
			// snap to node
			player.Pos = Point{X: float64(targetNode.X), Y: float64(targetNode.Y)}
			// consume node
			player.Path = player.Path[1:]
			if len(player.Path) == 0 {
				// no more nodes -> idle
				player.Mode = IDLE
				player.Direction = NONE
			} else {
				// There is a next node: recompute direction toward next node (important!)
				next := player.Path[0]
				dirX := float64(next.X) - player.Pos.X
				dirY := float64(next.Y) - player.Pos.Y
				norm := math.Sqrt(dirX*dirX + dirY*dirY)
				if norm > 0 {
					dirX /= norm
					dirY /= norm
					s.updatePlayerDirection(player, dirX, dirY)
				}
			}
		} else {
			// normal interpolation toward current node
			dirX, dirY := dx/dist, dy/dist
			player.Pos.X += dirX * step
			player.Pos.Y += dirY * step
			s.updatePlayerDirection(player, dirX, dirY)
		}
	}
}

// -------------------------------------------------------------------------

func (s *GameServer) updatePlayerDirection(player *PlayerState, dirX, dirY float64) {
	angle := math.Atan2(dirY, dirX)
	if angle < 0 {
		angle += 2 * math.Pi
	}
	directionIndex := (int(math.Round(angle/(math.Pi/4))) + 2) % 8
	player.Direction = Direction(directionIndex)
}

func (s *GameServer) checkPortal(player *PlayerState, mapState *MapState) {
	onPortal := false
	var activePortal *PortalState
	for _, portal := range mapState.portals {
		portalRect := Rectangle{
			MinX: portal.Pos.X, MinY: portal.Pos.Y,
			MaxX: portal.Pos.X + portal.Dims.Width, MaxY: portal.Pos.Y + portal.Dims.Height,
		}
		playerRect := Rectangle{
			MinX: player.Pos.X, MinY: player.Pos.Y,
			MaxX: player.Pos.X + player.Dims.Width, MaxY: player.Pos.Y + player.Dims.Height,
		}

		if rectsOverlap(playerRect, portalRect) {
			onPortal = true
			activePortal = portal
			break
		}
	}

	if onPortal {
		if !player.OnPortal {
			player.OnPortal = true
			player.TimeOnPortal = time.Now()
			player.ActivePortalID = activePortal.ID
		} else if time.Since(player.TimeOnPortal) > s.portalHoldTime {
			s.teleportPlayer(player, activePortal)
		}
	} else {
		player.OnPortal = false
		player.ActivePortalID = ""
	}
}

func (s *GameServer) teleportPlayer(player *PlayerState, portal *PortalState) {
	s.mu.Unlock()     // Unlock to prevent deadlock during map access
	defer s.mu.Lock() // Ensure lock is re-acquired before function exit

	if portal.PortalConfig == nil {
		log.Println("Teleportation failed: Portal config is nil.")
		return
	}

	destMapID := portal.PortalConfig.DestMapID
	destPortalIndex := portal.PortalConfig.DestPortalIndex

	if destMapID >= len(allMapsPortals) || destPortalIndex >= len(allMapsPortals[destMapID]) {
		log.Printf("Teleportation failed: Destination portal index out of bounds. MapID: %d, PortalIndex: %d", destMapID, destPortalIndex)
		return
	}
	destPortal := allMapsPortals[destMapID][destPortalIndex]

	destMapState, ok := s.maps[destMapID]
	if !ok {
		log.Printf("Teleportation failed: Destination map %d not found.", destMapID)
		return
	}

	spawnX := destPortal.Pos.X + (destPortal.Dims.Width / 2) + rand.Float64()*portal.PortalConfig.SpawnRadius*2 - portal.PortalConfig.SpawnRadius
	spawnY := destPortal.Pos.Y + (destPortal.Dims.Height / 2) + rand.Float64()*portal.PortalConfig.SpawnRadius*2 - portal.PortalConfig.SpawnRadius

	destPosI, err := destMapState.pathfinder.findClosestWalkablePoint(PointI{X: int(spawnX), Y: int(spawnY)}, player.Dims)
	if err != nil {
		log.Printf("Could not find a walkable spawn point: %v", err)
		return
	}

	currentMapState, ok := s.maps[player.MapID]
	if ok {
		delete(currentMapState.players, player.ID)
	}

	player.MapID = destMapID
	player.Pos = Point{X: float64(destPosI.X), Y: float64(destPosI.Y)}
	player.TargetPos = PointI{}
	player.Path = []PointI{}
	player.Mode = TELEPORTING
	player.OnPortal = false
	destMapState.players[player.ID] = player

	s.sendAOI(player)
}

func (s *GameServer) updateAOIs(mapState *MapState) {
	for _, player := range mapState.players {
		player.AOI = Rectangle{
			MinX: player.Pos.X - s.aoiRadius,
			MinY: player.Pos.Y - s.aoiRadius,
			MaxX: player.Pos.X + player.Dims.Width + s.aoiRadius,
			MaxY: player.Pos.Y + player.Dims.Height + s.aoiRadius,
		}
		s.sendAOI(player)
	}
}

func (s *GameServer) sendAOI(player *PlayerState) {
	mapState, ok := s.maps[player.MapID]
	if !ok {
		log.Printf("Map %d not found for player %s.", player.MapID, player.ID)
		return
	}

	// visible players
	visiblePlayersMap := make(map[string]map[string]interface{})
	for _, otherPlayer := range mapState.players {
		if otherPlayer.ID == player.ID {
			continue
		}
		otherRect := Rectangle{
			MinX: otherPlayer.Pos.X, MinY: otherPlayer.Pos.Y,
			MaxX: otherPlayer.Pos.X + otherPlayer.Dims.Width, MaxY: otherPlayer.Pos.Y + otherPlayer.Dims.Height,
		}
		if rectsOverlap(player.AOI, otherRect) {
			visiblePlayersMap[otherPlayer.ID] = map[string]interface{}{
				"id":        otherPlayer.ID,
				"Pos":       otherPlayer.Pos,
				"Dims":      otherPlayer.Dims,
				"Type":      "player",
				"direction": int(otherPlayer.Direction),
				"mode":      int(otherPlayer.Mode),
			}
		}
	}

	visibleGridObjectsMap := make(map[string]map[string]interface{})
	// Add obstacles
	for _, obstacle := range mapState.obstacles {
		obstacleRect := Rectangle{
			MinX: obstacle.Pos.X, MinY: obstacle.Pos.Y,
			MaxX: obstacle.Pos.X + obstacle.Dims.Width, MaxY: obstacle.Pos.Y + obstacle.Dims.Height,
		}
		if rectsOverlap(player.AOI, obstacleRect) {
			visibleGridObjectsMap[obstacle.ID] = map[string]interface{}{
				"id":   obstacle.ID,
				"Pos":  obstacle.Pos,
				"Dims": obstacle.Dims,
				"Type": obstacle.Type,
			}
		}
	}

	// Add portals
	for _, portal := range mapState.portals {
		portalRect := Rectangle{
			MinX: portal.Pos.X, MinY: portal.Pos.Y,
			MaxX: portal.Pos.X + portal.Dims.Width, MaxY: portal.Pos.Y + portal.Dims.Height,
		}
		if rectsOverlap(player.AOI, portalRect) {
			visibleGridObjectsMap[portal.ID] = map[string]interface{}{
				"id":          portal.ID,
				"Pos":         portal.Pos,
				"Dims":        portal.Dims,
				"Type":        "portal",
				"PortalLabel": portal.Label,
			}
		}
	}

	for _, fg := range mapState.foregrounds {
		fgRect := Rectangle{
			MinX: fg.Pos.X, MinY: fg.Pos.Y,
			MaxX: fg.Pos.X + fg.Dims.Width, MaxY: fg.Pos.Y + fg.Dims.Height,
		}
		if rectsOverlap(player.AOI, fgRect) {
			visibleGridObjectsMap[fg.ID] = map[string]interface{}{
				"id":   fg.ID,
				"Pos":  fg.Pos,
				"Dims": fg.Dims,
				"Type": "foreground",
			}
		}
	}

	// Add bots to visibleGridObjectsMap (client should render Type="bot")
	for _, bot := range mapState.bots {
		botRect := Rectangle{
			MinX: bot.Pos.X, MinY: bot.Pos.Y,
			MaxX: bot.Pos.X + bot.Dims.Width, MaxY: bot.Pos.Y + bot.Dims.Height,
		}
		if rectsOverlap(player.AOI, botRect) {
			visibleGridObjectsMap[bot.ID] = map[string]interface{}{
				"id":        bot.ID,
				"Pos":       bot.Pos,
				"Dims":      bot.Dims,
				"Type":      "bot",
				"behavior":  bot.Behavior,
				"direction": int(bot.Direction),
				"mode":      int(bot.Mode),
			}
		}
	}

	playerObj := map[string]interface{}{
		"id":             player.ID,
		"MapID":          player.MapID,
		"Pos":            player.Pos,
		"Dims":           player.Dims,
		"path":           player.Path,
		"targetPos":      player.TargetPos,
		"AOI":            player.AOI,
		"direction":      int(player.Direction),
		"mode":           int(player.Mode),
		"onPortal":       player.OnPortal,
		"activePortalID": player.ActivePortalID,
		"sumStatsLimit":  player.SumStatsLimit,
	}

	payloadMap := map[string]interface{}{
		"playerID":           player.ID,
		"player":             playerObj,
		"visiblePlayers":     visiblePlayersMap,
		"visibleGridObjects": visibleGridObjectsMap,
	}

	message, err := json.Marshal(map[string]interface{}{"type": "aoi_update", "payload": payloadMap})
	if err != nil {
		log.Printf("Error marshaling AOI update: %v", err)
		return
	}

	select {
	case player.Client.send <- message:
	default:
		log.Printf("Client %s message channel is full.", player.ID)
	}
}

// Handler for WebSocket connections.
func (s *GameServer) handleConnections(w http.ResponseWriter, r *http.Request) {
	upgrader := websocket.Upgrader{
		ReadBufferSize:  1024,
		WriteBufferSize: 1024,
		CheckOrigin: func(r *http.Request) bool {
			return true
		},
	}
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println("Upgrade error:", err)
		return
	}

	s.mu.Lock()

	playerID := uuid.New().String()
	playerDims := Dimensions{
		Width:  float64(rand.Intn(4) + 1),
		Height: float64(rand.Intn(4) + 1),
	}

	startMapID := rand.Intn(len(s.maps))
	startMapState := s.maps[startMapID]
	startPosI, err := startMapState.pathfinder.findRandomWalkablePoint(playerDims)
	if err != nil {
		log.Printf("Could not place new player: %v", err)
		conn.Close()
		s.mu.Unlock()
		return
	}

	sumLimit := rand.Intn(26) + 1000

	playerState := &PlayerState{
		ID:            playerID,
		MapID:         startMapID,
		Pos:           Point{X: float64(startPosI.X), Y: float64(startPosI.Y)},
		Dims:          playerDims,
		Path:          []PointI{},
		TargetPos:     PointI{-1, -1},
		Direction:     NONE,
		Mode:          IDLE,
		SumStatsLimit: sumLimit,
	}

	client := &Client{
		conn:        conn,
		playerID:    playerID,
		send:        make(chan []byte, 256),
		lastAction:  time.Now(),
		playerState: playerState,
	}
	playerState.Client = client

	startMapState.players[playerID] = playerState

	initPayload := map[string]interface{}{
		"gridW":                     startMapState.gridW,
		"gridH":                     startMapState.gridH,
		"defaultObjectWidth":        s.defaultObjWidth,
		"defaultObjectHeight":       s.defaultObjHeight,
		"cellSize":                  s.cellSize,
		"fps":                       s.fps,
		"interpolationMs":           s.interpolationMs,
		"aoiRadius":                 s.aoiRadius,
		"colors":                    s.colors,
		"cameraSmoothing":           s.cameraSmoothing,
		"cameraZoom":                s.cameraZoom,
		"defaultWidthScreenFactor":  s.defaultWidthScreenFactor,
		"defaultHeightScreenFactor": s.defaultHeightScreenFactor,
		"devUi":                     s.devUi,
		"sumStatsLimit":             playerState.SumStatsLimit,
	}

	initMsg, _ := json.Marshal(map[string]interface{}{"type": "init_data", "payload": initPayload})

	select {
	case client.send <- initMsg:
	default:
		log.Printf("Client %s init channel full.", client.playerID)
	}

	s.mu.Unlock()

	s.register <- client

	go client.writePump()
	go client.readPump(s)
}

// readPump handles incoming messages from the client.
func (c *Client) readPump(server *GameServer) {
	defer func() {
		server.unregister <- c
		c.conn.Close()
	}()
	c.conn.SetReadLimit(512)
	c.conn.SetReadDeadline(time.Now().Add(60 * time.Second))
	c.conn.SetPongHandler(func(string) error {
		c.conn.SetReadDeadline(time.Now().Add(60 * time.Second))
		return nil
	})
	for {
		_, message, err := c.conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("error: %v", err)
			}
			break
		}
		var msg map[string]interface{}
		if err := json.Unmarshal(message, &msg); err != nil {
			log.Printf("Error unmarshaling message: %v", err)
			continue
		}
		if msg["type"] == "player_action" {
			payload := msg["payload"].(map[string]interface{})
			targetX := payload["targetX"].(float64)
			targetY := payload["targetY"].(float64)

			server.mu.Lock()
			player, ok := server.maps[c.playerState.MapID].players[c.playerID]
			if !ok {
				log.Println("Player not found in map")
				server.mu.Unlock()
				continue
			}
			mapState, ok := server.maps[player.MapID]
			if !ok {
				log.Println("Player map not found")
				server.mu.Unlock()
				continue
			}
			server.mu.Unlock()

			startPosI := PointI{X: int(math.Round(player.Pos.X)), Y: int(math.Round(player.Pos.Y))}
			targetPosI := PointI{X: int(math.Round(targetX)), Y: int(math.Round(targetY))}

			newPath, err := mapState.pathfinder.Astar(startPosI, targetPosI, player.Dims)
			var usedTarget PointI = targetPosI
			if err != nil {
				closest, cerr := mapState.pathfinder.findClosestWalkablePoint(targetPosI, player.Dims)
				if cerr != nil {
					log.Printf("Pathfinding failed for player %s, no closest walkable: %v", c.playerID, err)
					server.mu.Lock()
					dx := float64(targetPosI.X) - player.Pos.X
					dy := float64(targetPosI.Y) - player.Pos.Y
					dist := math.Sqrt(dx*dx + dy*dy)
					if dist > 0 {
						dirX, dirY := dx/dist, dy/dist
						server.updatePlayerDirection(player, dirX, dirY)
						player.Mode = WALKING
						player.TargetPos = targetPosI
					} else {
						player.Mode = IDLE
					}
					server.mu.Unlock()
					continue
				}
				newPath, err = mapState.pathfinder.Astar(startPosI, closest, player.Dims)
				if err != nil {
					log.Printf("Pathfinding failed for player %s even to closest walkable: %v", c.playerID, err)
					server.mu.Lock()
					dx := float64(closest.X) - player.Pos.X
					dy := float64(closest.Y) - player.Pos.Y
					dist := math.Sqrt(dx*dx + dy*dy)
					if dist > 0 {
						dirX, dirY := dx/dist, dy/dist
						server.updatePlayerDirection(player, dirX, dirY)
						player.Mode = WALKING
						player.TargetPos = closest
					} else {
						player.Mode = IDLE
					}
					server.mu.Unlock()
					continue
				}
				usedTarget = closest
			}

			if newPath != nil && len(newPath) > 0 {
				first := newPath[0]
				server.mu.Lock()
				player.Path = newPath
				player.TargetPos = usedTarget
				player.Mode = WALKING
				dx := float64(first.X) - player.Pos.X
				dy := float64(first.Y) - player.Pos.Y
				dist := math.Sqrt(dx*dx + dy*dy)
				if dist > 0 {
					dirX, dirY := dx/dist, dy/dist
					server.updatePlayerDirection(player, dirX, dirY)
				}
				server.mu.Unlock()
			} else {
				server.mu.Lock()
				if startPosI.X == targetPosI.X && startPosI.Y == targetPosI.Y {
					player.Mode = IDLE
				}
				server.mu.Unlock()
			}
		}
	}
}

// writePump writes messages to the WebSocket connection.
func (c *Client) writePump() {
	ticker := time.NewTicker(30 * time.Second)
	defer func() {
		ticker.Stop()
		c.conn.Close()
	}()
	for {
		select {
		case message, ok := <-c.send:
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if !ok {
				c.conn.WriteMessage(websocket.CloseMessage, []byte{})
				return
			}
			w, err := c.conn.NextWriter(websocket.TextMessage)
			if err != nil {
				return
			}
			w.Write(message)
			if err := w.Close(); err != nil {
				return
			}
		case <-ticker.C:
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if err := c.conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				return
			}
		}
	}
}

func main() {
	rand.Seed(time.Now().UnixNano())
	server := NewGameServer()
	go server.Run()

	http.HandleFunc("/ws", server.handleConnections)
	log.Println("Server started on :8080")
	err := http.ListenAndServe(":8080", nil)
	if err != nil {
		log.Fatal("ListenAndServe:", err)
	}
}

// Global variable to hold portal states for cross-map linking
var allMapsPortals [][]*PortalState
