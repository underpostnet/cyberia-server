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
	Client         *Client         `json:"-"` // Ignore for JSON marshaling
	Direction      Direction       `json:"direction"`
	Mode           ObjectLayerMode `json:"mode"`
	OnPortal       bool            `json:"onPortal"`
	TimeOnPortal   time.Time       `json:"-"` // Ignore
	ActivePortalID string          `json:"activePortalID"`
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

type Pathfinder struct {
	gridW, gridH int
	obstacles    map[string]ObjectState
	grid         [][]int
}

//----------------------------------------------------------------------------------------------------------------------
// 2. Core Logic & Methods
//----------------------------------------------------------------------------------------------------------------------

func NewPathfinder(w, h int) *Pathfinder {
	p := &Pathfinder{gridW: w, gridH: h}
	p.grid = make([][]int, h)
	for y := 0; y < h; y++ {
		p.grid[y] = make([]int, w)
	}
	return p
}

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

func (pf *Pathfinder) isCellWalkable(x, y int) bool {
	return x >= 0 && x < pf.gridW && y >= 0 && y < pf.gridH && pf.grid[y][x] == 0
}

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

func (pf *Pathfinder) findRandomWalkablePoint(dims Dimensions) (PointI, error) {
	for attempts := 0; attempts < 200; attempts++ {
		x, y := rand.Intn(pf.gridW-int(dims.Width)), rand.Intn(pf.gridH-int(dims.Height))
		if pf.isWalkable(x, y, dims) {
			return PointI{X: x, Y: y}, nil
		}
	}
	return PointI{}, fmt.Errorf("could not find a walkable point")
}

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
		clients: make(map[string]*Client), maps: make(map[int]*MapState),
		register: make(chan *Client), unregister: make(chan *Client), aoiRadius: 25.0,
	}

	for i := 1; i <= 3; i++ {
		mapState := &MapState{
			gridW: 100, gridH: 100, pathfinder: NewPathfinder(100, 100),
			obstacles: make(map[string]ObjectState), portals: make(map[string]*PortalState),
			players: make(map[string]*PlayerState),
		}
		portals := mapState.generatePortals(rand.Intn(2) + 2)
		var portalRects []Rectangle
		for _, p := range portals {
			mapState.portals[p.ID] = p
			portalRects = append(portalRects, Rectangle{
				MinX: p.Pos.X, MinY: p.Pos.Y, MaxX: p.Pos.X + p.Dims.Width, MaxY: p.Pos.Y + p.Dims.Height,
			})
		}
		mapState.pathfinder.GenerateObstacles(100, portalRects)
		mapState.obstacles = mapState.pathfinder.obstacles
		server.maps[i] = mapState
	}

	portalConfigs := map[int][]PortalConfig{
		1: {{2, 0, 15}, {3, 0, 10}, {2, 1, 20}},
		2: {{3, 1, 15}, {1, 0, 10}, {3, 0, 20}},
		3: {{1, 1, 15}, {2, 0, 10}, {1, 0, 20}},
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
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()

	for range ticker.C {
		s.mu.Lock()
		for mapID, ms := range s.maps {
			for _, player := range ms.players {
				s.updatePlayerPosition(player)
				if !player.OnPortal {
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

func (s *GameServer) teleportPlayerIfReady(player *PlayerState, currentMapID int) {
	if !player.OnPortal || time.Since(player.TimeOnPortal) < 1*time.Second {
		return
	}
	currentMap := s.maps[currentMapID]
	portal, ok := currentMap.portals[player.ActivePortalID]
	if !ok || portal.PortalConfig == nil {
		player.OnPortal = false
		return
	}
	config := portal.PortalConfig
	destMap, ok := s.maps[config.DestMapID]
	if !ok {
		player.OnPortal = false
		return
	}
	var destPortals []*PortalState
	for _, p := range destMap.portals {
		destPortals = append(destPortals, p)
	}
	if len(destPortals) <= config.DestPortalIndex {
		player.OnPortal = false
		return
	}
	destPortal := destPortals[config.DestPortalIndex]
	spawnCenter := PointI{X: int(destPortal.Pos.X + destPortal.Dims.Width/2), Y: int(destPortal.Pos.Y + destPortal.Dims.Height/2)}
	newPos, err := destMap.pathfinder.findRandomPointInRadius(spawnCenter, config.SpawnRadius, player.Dims)
	if err != nil {
		newPos = spawnCenter
	}
	delete(currentMap.players, player.ID)
	player.MapID, player.Pos, player.Path, player.OnPortal, player.ActivePortalID = config.DestMapID, Point{X: float64(newPos.X), Y: float64(newPos.Y)}, nil, false, ""
	destMap.players[player.ID] = player
	log.Printf("Player %s teleported from map %d to map %d", player.ID, currentMapID, config.DestMapID)
}

func (pf *Pathfinder) findRandomPointInRadius(center PointI, radius float64, dims Dimensions) (PointI, error) {
	for i := 0; i < 100; i++ {
		angle, dist := rand.Float64()*2*math.Pi, rand.Float64()*radius
		x, y := int(float64(center.X)+dist*math.Cos(angle)), int(float64(center.Y)+dist*math.Sin(angle))
		if pf.isWalkable(x, y, dims) {
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
			player.OnPortal, player.TimeOnPortal, player.ActivePortalID = true, time.Now(), portal.ID
			player.Path, player.Mode = nil, IDLE
			return
		}
	}
}

func (s *GameServer) updatePlayerPosition(player *PlayerState) {
	if len(player.Path) > 0 {
		nextPoint := player.Path[0]
		targetX, targetY := float64(nextPoint.X), float64(nextPoint.Y)
		dist := math.Hypot(targetX-player.Pos.X, targetY-player.Pos.Y)

		// FIX: Correctly declare and use speed variable
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
				player.Mode, player.Direction = IDLE, NONE
			}
		}
	}
}

func getDirection(dx, dy float64) Direction {
	angle := math.Atan2(dy, dx)*(180/math.Pi) + 360
	angle = math.Mod(angle, 360)
	switch {
	case angle >= 337.5 || angle < 22.5:
		return RIGHT
	case angle >= 22.5 && angle < 67.5:
		return DOWN_RIGHT
	case angle >= 67.5 && angle < 112.5:
		return DOWN
	case angle >= 112.5 && angle < 157.5:
		return DOWN_LEFT
	case angle >= 157.5 && angle < 202.5:
		return LEFT
	case angle >= 202.5 && angle < 247.5:
		return UP_LEFT
	case angle >= 247.5 && angle < 292.5:
		return UP
	case angle >= 292.5 && angle < 337.5:
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
	visiblePlayers, visibleGridObjects := make(map[string]ObjectState), make(map[string]ObjectState)
	for id, other := range ms.players {
		if id != player.ID && rectsOverlap(player.AOI, Rectangle{other.Pos.X, other.Pos.Y, other.Pos.X + other.Dims.Width, other.Pos.Y + other.Dims.Height}) {
			visiblePlayers[id] = ObjectState{ID: other.ID, Pos: other.Pos, Dims: other.Dims, Type: "player"}
		}
	}
	for id, obs := range ms.obstacles {
		if rectsOverlap(player.AOI, Rectangle{obs.Pos.X, obs.Pos.Y, obs.Pos.X + obs.Dims.Width, obs.Pos.Y + obs.Dims.Height}) {
			visibleGridObjects[id] = obs
		}
	}
	for id, p := range ms.portals {
		if rectsOverlap(player.AOI, Rectangle{p.Pos.X, p.Pos.Y, p.Pos.X + p.Dims.Width, p.Pos.Y + p.Dims.Height}) {
			label := "Unknown"
			if p.PortalConfig != nil {
				label = fmt.Sprintf("To Map %d", p.PortalConfig.DestMapID)
			}
			visibleGridObjects[id] = ObjectState{ID: p.ID, Pos: p.Pos, Dims: p.Dims, Type: "portal", PortalLabel: label}
		}
	}

	// FIX: Correctly initialize the payload without trying to dereference a string
	payload := AOIUpdatePayload{player.ID, *player, visiblePlayers, visibleGridObjects}
	jsonMessage, err := json.Marshal(map[string]interface{}{"type": "aoi_update", "payload": payload})
	if err == nil {
		select {
		case player.Client.send <- jsonMessage:
		default:
		}
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
			playerDims := Dimensions{Width: 1.0 + rand.Float64()*0.5, Height: 1.0 + rand.Float64()*0.5}
			startPosI, err := spawnMapState.pathfinder.findRandomWalkablePoint(playerDims)
			if err != nil {
				startPosI = PointI{X: 10, Y: 10}
			}
			player := &PlayerState{
				ID: client.playerID, MapID: spawnMapID, Pos: Point{X: float64(startPosI.X), Y: float64(startPosI.Y)},
				Dims: playerDims, Path: make([]PointI, 0), Client: client, Mode: IDLE, Direction: NONE,
			}
			spawnMapState.players[player.ID] = player
			s.mu.Unlock()
			initData, _ := json.Marshal(map[string]interface{}{
				"type": "init_data", "payload": map[string]interface{}{"playerID": client.playerID},
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
			}
			s.mu.Unlock()
		}
	}
}

func (s *GameServer) handleWebSocket(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
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

var upgrader = websocket.Upgrader{CheckOrigin: func(r *http.Request) bool { return true }}

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
			break
		}
		var req map[string]interface{}
		if err := json.Unmarshal(message, &req); err != nil {
			continue
		}
		if reqType, _ := req["type"].(string); reqType == "path_request" {
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
			if player == nil {
				continue
			}

			if targetPosData, ok := payload["targetPos"].(map[string]interface{}); ok {
				targetX, Y := int(targetPosData["X"].(float64)), int(targetPosData["Y"].(float64))

				targetPoint, err := mapState.pathfinder.findClosestWalkablePoint(PointI{X: targetX, Y: Y}, player.Dims)
				if err != nil {
					continue
				}

				startPoint := PointI{X: int(player.Pos.X), Y: int(player.Pos.Y)}
				path, err := mapState.pathfinder.Astar(startPoint, targetPoint, player.Dims)

				s.mu.Lock()
				if err != nil {
					player.Path = nil
				} else {
					player.Path = path
					player.TargetPos = targetPoint
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
			for i := 0; i < len(client.send); i++ {
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
