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
)

// ObjectState represents a game object. Note the `omitempty` tag for the portal label.
type ObjectState struct {
	ID          string     `json:"id"`
	Pos         Point      `json:"Pos"`
	Dims        Dimensions `json:"Dims"`
	Type        string     `json:"Type"`                  // "player", "obstacle", "portal"
	PortalLabel string     `json:"PortalLabel,omitempty"` // NEW: Label for portals
}

type PlayerState struct {
	ID             string
	MapID          int
	Pos            Point
	Dims           Dimensions
	Path           []PointI
	TargetPos      PointI
	AOI            Rectangle
	Client         *Client
	Direction      Direction
	Mode           ObjectLayerMode
	OnPortal       bool
	TimeOnPortal   time.Time
	ActivePortalID string
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
}

type MapState struct {
	pathfinder   *Pathfinder
	obstacles    map[string]ObjectState
	portals      map[string]*PortalState
	players      map[string]*PlayerState
	gridW, gridH int
}

// AOIUpdatePayload now includes more detailed player info for the UI.
type AOIUpdatePayload struct {
	PlayerID           string                 `json:"playerID"`
	Player             PlayerState            `json:"player"`
	VisiblePlayers     map[string]ObjectState `json:"visiblePlayers"`
	VisibleGridObjects map[string]ObjectState `json:"visibleGridObjects"`
}

type Client struct {
	conn       *websocket.Conn
	playerID   string
	send       chan []byte
	isPlayer   bool
	lastAction time.Time
}

type GameServer struct {
	mu         sync.Mutex
	maps       map[int]*MapState
	clients    map[string]*Client
	register   chan *Client
	unregister chan *Client
	aoiRadius  float64
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
func (pq *PriorityQueue) update(node *Node, g, h float64) {
	node.g = g
	node.h = h
	node.f = g + h
	heap.Fix(pq, node.index)
}

type Pathfinder struct {
	gridW, gridH int
	obstacles    map[string]ObjectState
	grid         [][]int
}

//----------------------------------------------------------------------------------------------------------------------
// 2. Core Logic & Methods
//----------------------------------------------------------------------------------------------------------------------

func NewPathfinder(w, h int) *Pathfinder {
	p := &Pathfinder{
		gridW:     w,
		gridH:     h,
		obstacles: make(map[string]ObjectState),
	}
	p.grid = make([][]int, h)
	for y := 0; y < h; y++ {
		p.grid[y] = make([]int, w)
	}
	return p
}

func (pf *Pathfinder) GenerateObstacles(numObstacles int, portalRects []Rectangle) {
	log.Println("Generating new map obstacles...")
	pf.obstacles = make(map[string]ObjectState)
	for y := 0; y < pf.gridH; y++ {
		for x := 0; x < pf.gridW; x++ {
			pf.grid[y][x] = 0
		}
	}

	maxObsW, maxObsH, minObsDim := int(math.Max(2, float64(pf.gridW)/15.0)), int(math.Max(2, float64(pf.gridH)/15.0)), 2
	attempts, placed := 0, 0

	for placed < numObstacles && attempts < numObstacles*10 {
		w, h := rand.Intn(maxObsW-minObsDim+1)+minObsDim, rand.Intn(maxObsH-minObsDim+1)+minObsDim
		minX, minY := rand.Intn(pf.gridW-w), rand.Intn(pf.gridH-h)
		maxX, maxY := minX+w, minY+h
		obsRect := Rectangle{MinX: float64(minX), MinY: float64(minY), MaxX: float64(maxX), MaxY: float64(maxY)}

		overlapsPortal := false
		for _, pRect := range portalRects {
			if rectsOverlap(obsRect, pRect) {
				overlapsPortal = true
				break
			}
		}
		if overlapsPortal {
			attempts++
			continue
		}

		overlapsExisting := false
		for _, existingObs := range pf.obstacles {
			existingRect := Rectangle{
				MinX: existingObs.Pos.X, MinY: existingObs.Pos.Y,
				MaxX: existingObs.Pos.X + existingObs.Dims.Width, MaxY: existingObs.Pos.Y + existingObs.Dims.Height,
			}
			if rectsOverlap(obsRect, existingRect) {
				overlapsExisting = true
				break
			}
		}
		if overlapsExisting {
			attempts++
			continue
		}

		obs := ObjectState{
			ID:   uuid.New().String(),
			Pos:  Point{X: float64(minX), Y: float64(minY)},
			Dims: Dimensions{Width: float64(w), Height: float64(h)},
			Type: "obstacle",
		}

		for y := minY; y < maxY; y++ {
			for x := minX; x < maxX; x++ {
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

func (ms *MapState) generatePortals(numPortals int) []*PortalState {
	portals := make([]*PortalState, 0, numPortals)
	for i := 0; i < numPortals; i++ {
		dims := Dimensions{Width: float64(rand.Intn(3) + 2), Height: float64(rand.Intn(3) + 2)}
		posI, err := ms.pathfinder.findRandomWalkablePoint(dims)
		if err != nil {
			log.Printf("Error placing portal: %v", err)
			continue
		}
		pos := Point{X: float64(posI.X), Y: float64(posI.Y)}
		portal := &PortalState{
			ID:   uuid.New().String(),
			Pos:  pos,
			Dims: dims,
		}
		portals = append(portals, portal)
		for y := int(pos.Y); y < int(pos.Y+dims.Height); y++ {
			for x := int(pos.X); x < int(pos.X+dims.Width); x++ {
				if x >= 0 && x < ms.gridW && y >= 0 && y < ms.gridH {
					ms.pathfinder.grid[y][x] = 1 // Mark as occupied
				}
			}
		}
	}
	return portals
}

func (pf *Pathfinder) isCellWalkable(x, y int) bool {
	if x < 0 || x >= pf.gridW || y < 0 || y >= pf.gridH {
		return false
	}
	return pf.grid[y][x] == 0
}

func (pf *Pathfinder) isWalkable(x, y int, playerDims Dimensions) bool {
	playerW, playerH := int(math.Ceil(playerDims.Width)), int(math.Ceil(playerDims.Height))
	for dy := 0; dy < playerH; dy++ {
		for dx := 0; dx < playerW; dx++ {
			if !pf.isCellWalkable(x+dx, y+dy) {
				return false
			}
		}
	}
	return true
}

func (pf *Pathfinder) findRandomWalkablePoint(playerDims Dimensions) (PointI, error) {
	for attempts := 0; attempts < 200; attempts++ {
		x := rand.Intn(pf.gridW - int(playerDims.Width))
		y := rand.Intn(pf.gridH - int(playerDims.Height))
		if pf.isWalkable(x, y, playerDims) {
			return PointI{X: x, Y: y}, nil
		}
	}
	return PointI{}, fmt.Errorf("could not find a walkable point")
}

func (pf *Pathfinder) Astar(start, end PointI, playerDims Dimensions) ([]PointI, error) {
	if !pf.isWalkable(start.X, start.Y, playerDims) || !pf.isWalkable(end.X, end.Y, playerDims) {
		return nil, fmt.Errorf("start or end point is not walkable")
	}
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
				gScore[neighborPos] = tentativeGScore
				h := heuristic(neighborPos.X, neighborPos.Y, end.X, end.Y)
				heap.Push(&openSet, &Node{X: neighborPos.X, Y: neighborPos.Y, g: tentativeGScore, h: h, f: tentativeGScore + h, parent: current})
			}
		}
	}
	return nil, fmt.Errorf("could not find path")
}

func heuristic(x1, y1, x2, y2 int) float64 {
	return math.Abs(float64(x1-x2)) + math.Abs(float64(y1-y2))
}

func reconstructPath(node *Node) []PointI {
	path := make([]PointI, 0)
	for node != nil {
		path = append([]PointI{{X: node.X, Y: node.Y}}, path...)
		node = node.parent
	}
	return path
}

func rectsOverlap(r1, r2 Rectangle) bool {
	return r1.MinX < r2.MaxX && r1.MaxX > r2.MinX && r1.MinY < r2.MaxY && r1.MaxY > r2.MinY
}

func NewGameServer() *GameServer {
	rand.Seed(time.Now().UnixNano())
	server := &GameServer{
		clients:    make(map[string]*Client),
		maps:       make(map[int]*MapState),
		register:   make(chan *Client),
		unregister: make(chan *Client),
		aoiRadius:  25.0,
	}

	for i := 1; i <= 3; i++ {
		mapState := &MapState{
			gridW:      100,
			gridH:      100,
			pathfinder: NewPathfinder(100, 100),
			obstacles:  make(map[string]ObjectState),
			portals:    make(map[string]*PortalState),
			players:    make(map[string]*PlayerState),
		}
		portalCount := rand.Intn(2) + 2 // 2 to 3 portals
		portals := mapState.generatePortals(portalCount)
		for _, p := range portals {
			mapState.portals[p.ID] = p
		}

		var portalRects []Rectangle
		for _, p := range portals {
			portalRects = append(portalRects, Rectangle{
				MinX: p.Pos.X, MinY: p.Pos.Y,
				MaxX: p.Pos.X + p.Dims.Width, MaxY: p.Pos.Y + p.Dims.Height,
			})
		}
		mapState.pathfinder.GenerateObstacles(100, portalRects)
		mapState.obstacles = mapState.pathfinder.obstacles
		server.maps[i] = mapState
	}

	// REVISED: More logical portal configuration to prevent instant loops.
	portalConfigs := map[int][]PortalConfig{
		1: {
			{DestMapID: 2, DestPortalIndex: 0, SpawnRadius: 15.0},
			{DestMapID: 3, DestPortalIndex: 0, SpawnRadius: 10.0},
			{DestMapID: 2, DestPortalIndex: 1, SpawnRadius: 20.0},
		},
		2: {
			{DestMapID: 3, DestPortalIndex: 1, SpawnRadius: 15.0},
			{DestMapID: 1, DestPortalIndex: 0, SpawnRadius: 10.0},
			{DestMapID: 3, DestPortalIndex: 0, SpawnRadius: 20.0},
		},
		3: {
			{DestMapID: 1, DestPortalIndex: 1, SpawnRadius: 15.0},
			{DestMapID: 2, DestPortalIndex: 0, SpawnRadius: 10.0},
			{DestMapID: 1, DestPortalIndex: 0, SpawnRadius: 20.0},
		},
	}

	for mapID, ms := range server.maps {
		var portalList []*PortalState
		for _, p := range ms.portals {
			portalList = append(portalList, p)
		}
		for i, p := range portalList {
			if configs, ok := portalConfigs[mapID]; ok && i < len(configs) {
				p.PortalConfig = &configs[i]
			}
		}
	}

	return server
}

func (s *GameServer) Run() {
	go s.handleClientConnections()
	ticker := time.NewTicker(50 * time.Millisecond) // 20 FPS
	defer ticker.Stop()

	for range ticker.C {
		s.mu.Lock()
		for mapID, ms := range s.maps {
			for _, player := range ms.players {
				s.updatePlayerPosition(player)
				if !player.OnPortal { // Only check for new portal collision if not already on one
					s.checkAndHandlePortalCollision(player, ms)
				}
				s.teleportPlayerIfReady(player, mapID)
			}
			for _, player := range ms.players {
				s.sendAOI(player, ms)
			}
		}
		s.mu.Unlock()
	}
}

// REVISED: Renamed and refined teleportation logic.
func (s *GameServer) teleportPlayerIfReady(player *PlayerState, currentMapID int) {
	if !player.OnPortal || time.Since(player.TimeOnPortal) < 1*time.Second {
		return
	}

	currentMap := s.maps[currentMapID]
	portal, ok := currentMap.portals[player.ActivePortalID]
	if !ok || portal.PortalConfig == nil {
		player.OnPortal = false // Reset if portal is invalid
		return
	}

	config := portal.PortalConfig
	destMap, ok := s.maps[config.DestMapID]
	if !ok {
		log.Printf("Dest map %d not found for player %s", config.DestMapID, player.ID)
		player.OnPortal = false
		return
	}

	var destPortals []*PortalState
	for _, p := range destMap.portals {
		destPortals = append(destPortals, p)
	}

	if len(destPortals) <= config.DestPortalIndex {
		log.Printf("Dest portal index %d out of bounds for map %d", config.DestPortalIndex, config.DestMapID)
		player.OnPortal = false
		return
	}

	destPortal := destPortals[config.DestPortalIndex]
	spawnCenter := PointI{X: int(destPortal.Pos.X + destPortal.Dims.Width/2), Y: int(destPortal.Pos.Y + destPortal.Dims.Height/2)}

	newPos, err := destMap.pathfinder.findRandomPointInRadius(spawnCenter, config.SpawnRadius, player.Dims)
	if err != nil {
		log.Printf("Could not find spawn point for player %s, placing at portal center", player.ID)
		newPos = spawnCenter
	}

	// Move player
	delete(currentMap.players, player.ID)
	player.MapID = config.DestMapID
	player.Pos = Point{X: float64(newPos.X), Y: float64(newPos.Y)}
	player.Path = nil       // Clear path
	player.OnPortal = false // Reset portal state
	player.ActivePortalID = ""
	destMap.players[player.ID] = player

	log.Printf("Player %s teleported from map %d to map %d", player.ID, currentMapID, config.DestMapID)
}

func (pf *Pathfinder) findRandomPointInRadius(center PointI, radius float64, playerDims Dimensions) (PointI, error) {
	for i := 0; i < 100; i++ {
		angle, dist := rand.Float64()*2*math.Pi, rand.Float64()*radius
		x := int(float64(center.X) + dist*math.Cos(angle))
		y := int(float64(center.Y) + dist*math.Sin(angle))
		if pf.isWalkable(x, y, playerDims) {
			return PointI{X: x, Y: y}, nil
		}
	}
	return PointI{}, fmt.Errorf("could not find walkable point in radius")
}

func (s *GameServer) checkAndHandlePortalCollision(player *PlayerState, ms *MapState) {
	playerRect := Rectangle{
		MinX: player.Pos.X, MinY: player.Pos.Y,
		MaxX: player.Pos.X + player.Dims.Width, MaxY: player.Pos.Y + player.Dims.Height,
	}
	for _, portal := range ms.portals {
		portalRect := Rectangle{
			MinX: portal.Pos.X, MinY: portal.Pos.Y,
			MaxX: portal.Pos.X + portal.Dims.Width, MaxY: portal.Pos.Y + portal.Dims.Height,
		}
		if rectsOverlap(playerRect, portalRect) {
			player.OnPortal = true
			player.TimeOnPortal = time.Now()
			player.ActivePortalID = portal.ID
			player.Path = nil // Stop movement when hitting a portal
			player.Mode = IDLE
			return // Exit after finding one portal
		}
	}
}

func (s *GameServer) updatePlayerPosition(player *PlayerState) {
	if len(player.Path) > 0 {
		nextPoint := player.Path[0]
		targetX, targetY := float64(nextPoint.X), float64(nextPoint.Y)
		dist := math.Hypot(targetX-player.Pos.X, targetY-player.Pos.Y)

		// INCREASED SPEED: from 5.0 to 10.0
		speed := 10.0
		moveDist := speed * (50.0 / 1000.0)

		if dist > moveDist {
			player.Mode = WALKING
			dx, dy := (targetX-player.Pos.X)/dist, (targetY-player.Pos.Y)/dist
			player.Pos.X += dx * moveDist
			player.Pos.Y += dy * moveDist
			player.Direction = getDirection(dx, dy)
		} else {
			player.Pos.X, player.Pos.Y = targetX, targetY
			player.Path = player.Path[1:]
			if len(player.Path) == 0 {
				player.Mode = IDLE
				player.Direction = NONE
			}
		}
	}
}

func getDirection(dx, dy float64) Direction {
	angle := math.Atan2(dy, dx) * (180 / math.Pi)
	if angle < 0 {
		angle += 360
	}
	if angle >= 337.5 || angle < 22.5 {
		return RIGHT
	}
	if angle >= 22.5 && angle < 67.5 {
		return DOWN_RIGHT
	}
	if angle >= 67.5 && angle < 112.5 {
		return DOWN
	}
	if angle >= 112.5 && angle < 157.5 {
		return DOWN_LEFT
	}
	if angle >= 157.5 && angle < 202.5 {
		return LEFT
	}
	if angle >= 202.5 && angle < 247.5 {
		return UP_LEFT
	}
	if angle >= 247.5 && angle < 292.5 {
		return UP
	}
	if angle >= 292.5 && angle < 337.5 {
		return UP_RIGHT
	}
	return NONE
}

func (s *GameServer) sendAOI(player *PlayerState, ms *MapState) {
	if player.Client == nil {
		return
	}
	player.AOI = Rectangle{
		MinX: player.Pos.X - s.aoiRadius, MinY: player.Pos.Y - s.aoiRadius,
		MaxX: player.Pos.X + player.Dims.Width + s.aoiRadius, MaxY: player.Pos.Y + player.Dims.Height + s.aoiRadius,
	}

	visiblePlayers := make(map[string]ObjectState)
	for id, otherPlayer := range ms.players {
		if id != player.ID {
			otherPlayerRect := Rectangle{
				MinX: otherPlayer.Pos.X, MinY: otherPlayer.Pos.Y,
				MaxX: otherPlayer.Pos.X + otherPlayer.Dims.Width, MaxY: otherPlayer.Pos.Y + otherPlayer.Dims.Height,
			}
			if rectsOverlap(player.AOI, otherPlayerRect) {
				visiblePlayers[id] = ObjectState{ID: otherPlayer.ID, Pos: otherPlayer.Pos, Dims: otherPlayer.Dims, Type: "player"}
			}
		}
	}

	visibleGridObjects := make(map[string]ObjectState)
	for id, obs := range ms.obstacles {
		obsRect := Rectangle{MinX: obs.Pos.X, MinY: obs.Pos.Y, MaxX: obs.Pos.X + obs.Dims.Width, MaxY: obs.Pos.Y + obs.Dims.Height}
		if rectsOverlap(player.AOI, obsRect) {
			visibleGridObjects[id] = obs
		}
	}

	// NEW: Add portals with labels to visible objects
	for id, p := range ms.portals {
		portalRect := Rectangle{MinX: p.Pos.X, MinY: p.Pos.Y, MaxX: p.Pos.X + p.Dims.Width, MaxY: p.Pos.Y + p.Dims.Height}
		if rectsOverlap(player.AOI, portalRect) {
			label := "Unknown Portal"
			if p.PortalConfig != nil {
				destMapID := p.PortalConfig.DestMapID
				destPortalIndex := p.PortalConfig.DestPortalIndex

				// Find destination portal to add coordinates to label
				var destPortalPos Point
				if destMap, ok := s.maps[destMapID]; ok {
					var destPortals []*PortalState
					for _, dp := range destMap.portals {
						destPortals = append(destPortals, dp)
					}
					if destPortalIndex < len(destPortals) {
						destPortalPos = destPortals[destPortalIndex].Pos
					}
				}
				label = fmt.Sprintf("To Map %d (~%.0f, %.0f)", destMapID, destPortalPos.X, destPortalPos.Y)
			}
			visibleGridObjects[id] = ObjectState{ID: p.ID, Pos: p.Pos, Dims: p.Dims, Type: "portal", PortalLabel: label}
		}
	}

	payload := AOIUpdatePayload{
		PlayerID:           player.ID,
		Player:             *player,
		VisiblePlayers:     visiblePlayers,
		VisibleGridObjects: visibleGridObjects,
	}
	jsonMessage, err := json.Marshal(map[string]interface{}{"type": "aoi_update", "payload": payload})
	if err != nil {
		log.Printf("json marshal error: %v", err)
		return
	}
	select {
	case player.Client.send <- jsonMessage:
	default:
		log.Printf("Player %s send channel is full.", player.ID)
	}
}

func (s *GameServer) handleClientConnections() {
	for {
		select {
		case client := <-s.register:
			s.mu.Lock()
			s.clients[client.playerID] = client
			spawnMapID := rand.Intn(len(s.maps)) + 1
			spawnMapState := s.maps[spawnMapID]
			playerDims := Dimensions{Width: 1.0, Height: 1.0}
			startPosI, err := spawnMapState.pathfinder.findRandomWalkablePoint(playerDims)
			if err != nil {
				log.Printf("Error finding spawn point: %v", err)
				startPosI = PointI{X: 10, Y: 10}
			}
			player := &PlayerState{
				ID:     client.playerID,
				MapID:  spawnMapID,
				Pos:    Point{X: float64(startPosI.X), Y: float64(startPosI.Y)},
				Dims:   playerDims,
				Path:   make([]PointI, 0),
				Client: client,
				Mode:   IDLE,
			}
			spawnMapState.players[player.ID] = player
			s.mu.Unlock()

			initData, _ := json.Marshal(map[string]interface{}{
				"type": "init_data",
				"payload": map[string]interface{}{
					"playerID": client.playerID, "gridW": spawnMapState.gridW, "gridH": spawnMapState.gridH, "aoiRadius": s.aoiRadius,
				},
			})
			client.send <- initData
			log.Printf("Client registered: %s, spawned on map %d", client.playerID, spawnMapID)

		case client := <-s.unregister:
			s.mu.Lock()
			if _, ok := s.clients[client.playerID]; ok {
				delete(s.clients, client.playerID)
				for _, ms := range s.maps {
					if _, ok := ms.players[client.playerID]; ok {
						delete(ms.players, client.playerID)
						break
					}
				}
				close(client.send)
				log.Printf("Client unregistered: %s", client.playerID)
			}
			s.mu.Unlock()
		}
	}
}

func (s *GameServer) handleWebSocket(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println("Upgrade error:", err)
		return
	}
	client := &Client{conn: conn, playerID: uuid.New().String(), send: make(chan []byte, 256)}
	s.register <- client
	go writePump(conn, client)
	go readPump(conn, s, client)
}

//----------------------------------------------------------------------------------------------------------------------
// 3. Helper Functions & Main
//----------------------------------------------------------------------------------------------------------------------

var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin:     func(r *http.Request) bool { return true },
}

func readPump(conn *websocket.Conn, s *GameServer, client *Client) {
	defer func() {
		s.unregister <- client
		conn.Close()
	}()
	conn.SetReadLimit(512)
	conn.SetReadDeadline(time.Now().Add(60 * time.Second))
	conn.SetPongHandler(func(string) error { conn.SetReadDeadline(time.Now().Add(60 * time.Second)); return nil })

	for {
		_, message, err := conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("read error: %v", err)
			}
			break
		}

		var req map[string]interface{}
		if err := json.Unmarshal(message, &req); err != nil {
			continue
		}
		if reqType, ok := req["type"].(string); ok && reqType == "path_request" {
			payload, _ := req["payload"].(map[string]interface{})
			s.mu.Lock()
			var player *PlayerState
			var mapState *MapState
			for _, ms := range s.maps {
				if p, ok := ms.players[client.playerID]; ok {
					player, mapState = p, ms
					break
				}
			}
			s.mu.Unlock()

			if player == nil || mapState == nil {
				continue
			}

			if targetPosData, ok := payload["targetPos"].(map[string]interface{}); ok {
				targetX := int(targetPosData["X"].(float64))
				targetY := int(targetPosData["Y"].(float64))
				path, err := mapState.pathfinder.Astar(
					PointI{X: int(player.Pos.X), Y: int(player.Pos.Y)},
					PointI{X: targetX, Y: targetY},
					player.Dims,
				)
				s.mu.Lock()
				if err != nil {
					player.Path = nil
					player.Mode = IDLE
				} else {
					player.Path = path
					player.TargetPos = PointI{X: targetX, Y: targetY}
				}
				s.mu.Unlock()
			}
		}
	}
}

func writePump(conn *websocket.Conn, client *Client) {
	ticker := time.NewTicker(30 * time.Second)
	defer func() {
		ticker.Stop()
		conn.Close()
	}()
	for {
		select {
		case message, ok := <-client.send:
			conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if !ok {
				conn.WriteMessage(websocket.CloseMessage, []byte{})
				return
			}
			w, err := conn.NextWriter(websocket.TextMessage)
			if err != nil {
				return
			}
			w.Write(message)
			n := len(client.send)
			for i := 0; i < n; i++ {
				w.Write(<-client.send)
			}
			if err := w.Close(); err != nil {
				return
			}
		case <-ticker.C:
			conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if err := conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				return
			}
		}
	}
}

func main() {
	gameServer := NewGameServer()
	go gameServer.Run()
	http.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		gameServer.handleWebSocket(w, r)
	})
	log.Println("Starting server on :8080")
	if err := http.ListenAndServe(":8080", nil); err != nil {
		log.Fatal("ListenAndServe: ", err)
	}
}
