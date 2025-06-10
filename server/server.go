package server

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"sync"
	"time"

	"cyberia-server/network_state" // Updated import path

	"github.com/gorilla/websocket"
)

// ChatMessage represents a single chat message.
type ChatMessage struct {
	Sender    string `json:"sender"`
	Text      string `json:"text"`
	Timestamp string `json:"time"`
}

// NetworkStateServer manages WebSocket connections and the network state.
type NetworkStateServer struct {
	upgrader         websocket.Upgrader
	networkState     *network_state.NetworkState
	clients          map[*WebSocketClient]bool
	playerIDToClient map[string]**WebSocketClient
	register         chan *WebSocketClient
	unregister       chan *WebSocketClient
	playerCounter    int
	lastUpdateTime   time.Time
	clientsMutex     sync.RWMutex
	chatRooms        map[string][]ChatMessage // New: Stores chat messages per room
	chatRoomsMutex   sync.RWMutex             // New: Mutex for concurrent access to chatRooms
}

// NewNetworkStateServer initializes a new NetworkStateServer instance.
func NewNetworkStateServer() *NetworkStateServer {
	ns := &NetworkStateServer{
		upgrader: websocket.Upgrader{
			ReadBufferSize:  1024,
			WriteBufferSize: 1024,
			CheckOrigin: func(r *http.Request) bool {
				return true
			},
		},
		networkState:     network_state.NewNetworkState(),
		clients:          make(map[*WebSocketClient]bool),
		playerIDToClient: make(map[string]**WebSocketClient),
		register:         make(chan *WebSocketClient),
		unregister:       make(chan *WebSocketClient),
		playerCounter:    0,
		clientsMutex:     sync.RWMutex{},
		chatRooms:        make(map[string][]ChatMessage), // New: Initialize chatRooms map
		chatRoomsMutex:   sync.RWMutex{},                 // New: Initialize chatRooms mutex
	}

	ns.addInitialNetworkObjects()
	return ns
}

// addInitialNetworkObjects places predefined static obstacles in the network state world.
func (ns *NetworkStateServer) addInitialNetworkObjects() {
	// Use constants from network_state package
	worldWidth := network_state.WORLD_WIDTH
	worldHeight := network_state.WORLD_HEIGHT
	networkObjectSize := network_state.NETWORK_OBJECT_SIZE

	wallCoords := []struct{ X, Y float64 }{}
	seenCoords := make(map[struct{ X, Y float64 }]bool)

	gridCellsX := worldWidth / networkObjectSize
	gridCellsY := worldHeight / networkObjectSize
	gridCenterX := int(gridCellsX / 2)
	gridCenterY := int(gridCellsY / 2)

	// Define the rectangular obstacle mounds in grid coordinates (start_x, start_y, width, height)
	obstacleMoundsGrid := [][]int{
		{gridCenterX - 15, gridCenterY - 10, 6, 8},
		{gridCenterX + 9, gridCenterY - 10, 6, 8},
		{gridCenterX - 3, gridCenterY - 18, 6, 8},
		{gridCenterX - 3, gridCenterY + 10, 6, 8},
		{gridCenterX - 12, gridCenterY - 2, 4, 4},
		{gridCenterX + 8, gridCenterY - 2, 4, 4},
		{gridCenterX - 2, gridCenterY - 12, 4, 4},
		{gridCenterX - 2, gridCenterY + 8, 4, 4},
		{gridCenterX - 20, gridCenterY - 20, 3, 3},
		{gridCenterX + 17, gridCenterY - 20, 3, 3},
		{gridCenterX - 20, gridCenterY + 17, 3, 3},
		{gridCenterX + 17, gridCenterY + 17, 3, 3},
		{gridCenterX - 8, gridCenterY - 8, 2, 2},
		{gridCenterX + 6, gridCenterY + 6, 2, 2},
	}

	for _, mound := range obstacleMoundsGrid {
		startXGrid, startYGrid, widthGrid, heightGrid := mound[0], mound[1], mound[2], mound[3]
		for yOffset := 0; yOffset < heightGrid; yOffset++ {
			for xOffset := 0; xOffset < widthGrid; xOffset++ {
				gridX := startXGrid + xOffset
				gridY := startYGrid + yOffset

				if gridX >= 0 && gridX < int(gridCellsX) && gridY >= 0 && gridY < int(gridCellsY) {
					worldX := float64(gridX * networkObjectSize)
					worldY := float64(gridY * networkObjectSize)
					coord := struct{ X, Y float64 }{X: worldX, Y: worldY}

					if _, seen := seenCoords[coord]; !seen {
						wallCoords = append(wallCoords, coord)
						seenCoords[coord] = true
					}
				}
			}
		}
	}

	for i, pos := range wallCoords {
		obstacleID := fmt.Sprintf("obstacle_%d", i)
		obstacle := network_state.NewNetworkObject(
			obstacleID,
			pos.X, pos.Y,
			network_state.RED,
			true,
			0,
			"wall",
		)
		ns.networkState.AddNetworkObject(obstacle)
		log.Printf("Added obstacle: %s at (%.0f, %.0f)", obstacleID, pos.X, pos.Y)
	}
	log.Printf("Added %d initial obstacles.", len(wallCoords))
	ns.networkState.BuildSimplifiedMaze()
}

// handleWebSocketConnection upgrades HTTP requests to WebSocket connections.
func (ns *NetworkStateServer) handleWebSocketConnection(w http.ResponseWriter, r *http.Request) {
	conn, err := ns.upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("WebSocket upgrade failed: %v", err)
		return
	}

	ns.clientsMutex.Lock()
	ns.playerCounter++
	playerID := fmt.Sprintf("player_%d", ns.playerCounter)
	client := NewWebSocketClient(conn, playerID)
	ns.clients[client] = true
	ns.playerIDToClient[playerID] = &client
	ns.clientsMutex.Unlock()

	ns.register <- client

	posX, posY, err := ns.networkState.GetRandomAvailablePosition()
	if err != nil {
		log.Printf("ERROR: No available position for new player %s. Closing connection. Error: %v", playerID, err)
		conn.Close()
		return
	}

	playerObj := network_state.NewNetworkObject(
		playerID,
		posX, posY,
		network_state.BLUE,
		false,
		200,
		"player",
	)
	ns.networkState.AddNetworkObject(playerObj)

	ns.networkState.Mu.RLock()
	_, objAdded := ns.networkState.NetworkObjects[playerID]
	ns.networkState.Mu.RUnlock()
	if !objAdded {
		log.Printf("CRITICAL ERROR: Player network object %s was not added to network state. Disconnecting client %s.", playerID, conn.RemoteAddr().String())
		conn.Close()
		return
	}

	playerAssignedMsg, _ := json.Marshal(map[string]interface{}{
		"type":      "player_assigned",
		"player_id": playerID,
		"x":         posX,
		"y":         posY,
	})
	err = conn.WriteMessage(websocket.TextMessage, playerAssignedMsg)
	if err != nil {
		log.Printf("ERROR: Failed to send player_assigned message to client %s: %v. Disconnecting.", playerID, err)
		conn.Close()
		return
	}
	log.Printf("Player %s assigned to client %s.", playerID, conn.RemoteAddr().String())

	go client.WritePump()
	go client.ReadPump(ns)
}

// registerClient adds a new WebSocketClient to the server's active client lists.
func (ns *NetworkStateServer) registerClient(client *WebSocketClient) {
	log.Printf("Client %s (ID: %s) fully registered.", client.conn.RemoteAddr().String(), client.playerID)
	ns.sendFullNetworkState()
}

// unregisterClient removes a WebSocketClient from the server's active client lists.
func (ns *NetworkStateServer) unregisterClient(client *WebSocketClient) {
	ns.clientsMutex.Lock()
	if _, ok := ns.clients[client]; ok {
		delete(ns.clients, client)
		delete(ns.playerIDToClient, client.playerID)
		close(client.send)
	}
	ns.clientsMutex.Unlock()

	ns.networkState.RemoveNetworkObject(client.playerID)
	log.Printf("Client %s (ID: %s) unregistered and player network object removed.", client.conn.RemoteAddr().String(), client.playerID)
	ns.sendFullNetworkState()
}

// broadcastNetworkStateLoop periodically updates network object positions and broadcasts the full network state.
func (ns *NetworkStateServer) broadcastNetworkStateLoop() {
	ns.lastUpdateTime = time.Now()
	ticker := time.NewTicker(network_state.NETWORK_STATE_TICK_INTERVAL)
	defer ticker.Stop()

	for range ticker.C {
		deltaTime := time.Since(ns.lastUpdateTime).Seconds()
		ns.lastUpdateTime = time.Now()

		ns.networkState.Mu.Lock()
		for _, obj := range ns.networkState.NetworkObjects {
			if !obj.IsObstacle {
				obj.UpdatePosition(deltaTime)
				if len(obj.Path) > 0 && obj.PathIndex >= len(obj.Path) {
					obj.Path = []struct{ X, Y float64 }{}
					obj.PathIndex = 0
					log.Printf("Player %s reached destination.", obj.ID)
				}
			}
		}
		ns.networkState.Mu.Unlock()

		ns.sendFullNetworkState()
	}
}

// sendFullNetworkState marshals the current network state and sends it to all connected clients.
func (ns *NetworkStateServer) sendFullNetworkState() {
	ns.networkState.Mu.RLock()
	defer ns.networkState.Mu.RUnlock()

	serializableObjects := make(map[string]network_state.NetworkObject)
	for id, obj := range ns.networkState.NetworkObjects {
		serializableObjects[id] = *obj
	}

	broadcastMsg, err := json.Marshal(map[string]interface{}{
		"type":            "network_state_update",
		"network_objects": serializableObjects,
	})
	if err != nil {
		log.Printf("ERROR: Failed to marshal network state for broadcast: %v", err)
		return
	}

	ns.clientsMutex.RLock()
	clientsToBroadcast := make([]*WebSocketClient, 0, len(ns.clients))
	for client := range ns.clients {
		clientsToBroadcast = append(clientsToBroadcast, client)
	}
	ns.clientsMutex.RUnlock()

	for _, client := range clientsToBroadcast {
		select {
		case client.send <- broadcastMsg:
		default:
			log.Printf("WARNING: Client %s send buffer full, attempting to unregister.", client.playerID)
			ns.unregister <- client
		}
	}
}

// sendChatMessageToClients marshals and sends a chat message to all connected clients in a specific room.
func (ns *NetworkStateServer) sendChatMessageToClients(roomID string, message ChatMessage) {
	ns.clientsMutex.RLock()
	clientsToBroadcast := make([]*WebSocketClient, 0, len(ns.clients))
	for client := range ns.clients {
		clientsToBroadcast = append(clientsToBroadcast, client)
	}
	ns.clientsMutex.RUnlock()

	chatMsg := map[string]interface{}{
		"type": "server_chat_message",
		"data": map[string]interface{}{
			"room_id": roomID,
			"sender":  message.Sender,
			"text":    message.Text,
			"time":    message.Timestamp,
		},
	}

	broadcastMsg, err := json.Marshal(chatMsg)
	if err != nil {
		log.Printf("ERROR: Failed to marshal chat message for broadcast: %v", err)
		return
	}

	for _, client := range clientsToBroadcast {
		select {
		case client.send <- broadcastMsg:
		default:
			log.Printf("WARNING: Client %s send buffer full, attempting to unregister.", client.playerID)
			ns.unregister <- client
		}
	}
}

// Run starts the NetworkStateServer's main loops.
func (ns *NetworkStateServer) Run() {
	go func() {
		for {
			select {
			case client := <-ns.register:
				ns.registerClient(client)
			case client := <-ns.unregister:
				ns.unregisterClient(client)
			}
		}
	}()

	go ns.broadcastNetworkStateLoop()

	http.HandleFunc("/ws", ns.handleWebSocketConnection)
}
