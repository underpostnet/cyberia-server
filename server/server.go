package server

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"sync"
	"time"

	"cyberia-server/instance" // Updated import path
	// Updated import path
	"github.com/gorilla/websocket" // Gorilla WebSocket library for plain WebSockets
)

// InstanceServer manages WebSocket connections and the instance state.
type InstanceServer struct {
	upgrader         websocket.Upgrader           // WebSocket upgrader for HTTP requests
	instanceState    *instance.InstanceState      // The central instance state
	clients          map[*WebSocketClient]bool    // Set of active WebSocketClient connections
	playerIDToClient map[string]**WebSocketClient // Map from player ID to WebSocketClient for quick lookup
	register         chan *WebSocketClient        // Channel for new client registrations
	unregister       chan *WebSocketClient        // Channel for client unregistrations
	playerCounter    int                          // Counter for assigning unique player IDs
	lastUpdateTime   time.Time                    // Timestamp of the last instance state update
	clientsMutex     sync.RWMutex                 // Mutex protecting 'clients' and 'playerIDToClient' maps
}

// NewInstanceServer initializes a new InstanceServer instance.
func NewInstanceServer() *InstanceServer {
	is := &InstanceServer{
		upgrader: websocket.Upgrader{
			ReadBufferSize:  1024,
			WriteBufferSize: 1024,
			CheckOrigin: func(r *http.Request) bool {
				// Allow all origins for development. RESTRICT THIS IN PRODUCTION!
				return true
			},
		},
		instanceState:    instance.NewInstanceState(), // Initialize instance state
		clients:          make(map[*WebSocketClient]bool),
		playerIDToClient: make(map[string]**WebSocketClient),
		register:         make(chan *WebSocketClient),
		unregister:       make(chan *WebSocketClient),
		playerCounter:    0,
		clientsMutex:     sync.RWMutex{},
	}

	is.addInitialObstacles() // Populate the world with static obstacles
	return is
}

// addInitialObstacles places predefined static obstacles in the instance world.
func (is *InstanceServer) addInitialObstacles() {
	obstaclePositions := []struct{ X, Y float64 }{
		{200, 200}, {250, 200}, {300, 200},
		{200, 250}, {300, 250},
		{200, 300}, {250, 300}, {300, 300},
		{700, 700}, {750, 700}, {800, 700},
		{700, 750}, {800, 750},
		{700, 800}, {750, 800}, {800, 800},
		{100, 500}, {150, 500}, {200, 500},
		{500, 100}, {500, 150}, {500, 200},
		{1000, 1000}, {1050, 1000}, {1100, 1000},
		{1000, 1050}, {1100, 1050},
		{1000, 1100}, {1050, 1100}, {1100, 1100},
	}
	for i, pos := range obstaclePositions {
		obstacleID := fmt.Sprintf("obstacle_%d", i)
		obstacle := &instance.InstanceObject{ // Use InstanceObject from instance package
			ID:         obstacleID,
			X:          pos.X,
			Y:          pos.Y,
			Color:      instance.RED, // Use Color from instance package
			IsObstacle: true,
			Speed:      0, // Obstacles do not move
			ObjectType: "wall",
		}
		is.instanceState.AddObject(obstacle)
		log.Printf("Added obstacle: %s at (%.0f, %.0f)", obstacleID, pos.X, pos.Y)
	}
	log.Printf("Added %d initial obstacles.", len(obstaclePositions))
	is.instanceState.BuildSimplifiedMaze() // Build the A* maze once after all static obstacles are added.
}

// handleWebSocketConnection upgrades HTTP requests to WebSocket connections.
func (is *InstanceServer) handleWebSocketConnection(w http.ResponseWriter, r *http.Request) {
	conn, err := is.upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("WebSocket upgrade failed: %v", err)
		return
	}

	// Assign a unique player ID and create a new WebSocketClient.
	is.clientsMutex.Lock()
	is.playerCounter++
	playerID := fmt.Sprintf("player_%d", is.playerCounter)
	client := NewWebSocketClient(conn, playerID) // Use constructor
	is.clients[client] = true
	is.playerIDToClient[playerID] = &client // Store pointer to client
	is.clientsMutex.Unlock()

	is.register <- client // Signal the InstanceServer's main loop to register this client

	// Find a random available position for the new player.
	posX, posY, err := is.instanceState.GetRandomAvailablePosition()
	if err != nil {
		log.Printf("ERROR: No available position for new player %s. Closing connection. Error: %v", playerID, err)
		conn.Close() // Close connection if no position is found
		return
	}

	// Create and add the player's InstanceObject to the instance state.
	playerObj := &instance.InstanceObject{ // Use InstanceObject
		ID:         playerID,
		X:          posX,
		Y:          posY,
		Color:      instance.BLUE, // Use Color
		IsObstacle: false,
		Speed:      200, // Player movement speed
		ObjectType: "player",
	}
	is.instanceState.AddObject(playerObj)

	// Verify the player object was successfully added to the instance state.
	is.instanceState.Mu.RLock() // Access exported Mu
	_, objAdded := is.instanceState.Objects[playerID]
	is.instanceState.Mu.RUnlock() // Access exported Mu
	if !objAdded {
		log.Printf("CRITICAL ERROR: Player object %s was not added to instance state. Disconnecting client %s.", playerID, conn.RemoteAddr().String())
		conn.Close()
		return
	}

	// Send the 'player_assigned' message directly and synchronously to the client.
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

	// Start read and write goroutines for the client.
	go client.WritePump()
	go client.ReadPump(is) // Use client method, pass server for message handling
}

// registerClient adds a new WebSocketClient to the server's active client lists.
func (is *InstanceServer) registerClient(client *WebSocketClient) {
	log.Printf("Client %s (ID: %s) fully registered.", client.conn.RemoteAddr().String(), client.playerID)
	is.sendFullInstanceState() // Broadcast full state to all clients, including the new one.
}

// unregisterClient removes a WebSocketClient from the server's active client lists.
// It also cleans up the associated InstanceObject.
func (is *InstanceServer) unregisterClient(client *WebSocketClient) {
	is.clientsMutex.Lock()
	if _, ok := is.clients[client]; ok {
		delete(is.clients, client)
		delete(is.playerIDToClient, client.playerID)
		close(client.send) // Close the client's send channel to signal writePump termination
	}
	is.clientsMutex.Unlock()

	// Remove the player's InstanceObject from the instance state.
	is.instanceState.RemoveObject(client.playerID)
	log.Printf("Client %s (ID: %s) unregistered and player object removed.", client.conn.RemoteAddr().String(), client.playerID)
	is.sendFullInstanceState() // Broadcast updated state to remaining clients.
}

// broadcastInstanceStateLoop periodically updates instance object positions and broadcasts the full instance state.
func (is *InstanceServer) broadcastInstanceStateLoop() {
	is.lastUpdateTime = time.Now()
	ticker := time.NewTicker(instance.INSTANCE_TICK_INTERVAL) // Use constant from instance package
	defer ticker.Stop()

	for range ticker.C {
		deltaTime := time.Since(is.lastUpdateTime).Seconds()
		is.lastUpdateTime = time.Now()

		is.instanceState.Mu.Lock() // Access exported Mu
		for _, obj := range is.instanceState.Objects {
			if !obj.IsObstacle { // Only update positions of non-obstacle objects (players)
				obj.UpdatePosition(deltaTime)
				// If player reached end of path, clear it.
				if len(obj.Path) > 0 && obj.PathIndex >= len(obj.Path) {
					obj.Path = []struct{ X, Y float64 }{}
					obj.PathIndex = 0
					log.Printf("Player %s reached destination.", obj.ID)
				}
			}
		}
		is.instanceState.Mu.Unlock() // Access exported Mu

		is.sendFullInstanceState() // Broadcast the updated state to all clients
	}
}

// sendFullInstanceState marshals the current instance state and sends it to all connected clients.
func (is *InstanceServer) sendFullInstanceState() {
	is.instanceState.Mu.RLock()         // Access exported Mu
	defer is.instanceState.Mu.RUnlock() // Access exported Mu

	// Create a serializable copy of instance objects.
	serializableObjects := make(map[string]instance.InstanceObject) // Use InstanceObject
	for id, obj := range is.instanceState.Objects {
		serializableObjects[id] = *obj // Dereference to copy the struct value
	}

	// Prepare the 'instance_state_update' message.
	broadcastMsg, err := json.Marshal(map[string]interface{}{
		"type":    "instance_state_update",
		"objects": serializableObjects,
	})
	if err != nil {
		log.Printf("ERROR: Failed to marshal instance state for broadcast: %v", err)
		return
	}

	is.clientsMutex.RLock()
	// Create a temporary slice of clients to iterate over, preventing map modification during iteration issues.
	clientsToBroadcast := make([]*WebSocketClient, 0, len(is.clients))
	for client := range is.clients {
		clientsToBroadcast = append(clientsToBroadcast, client)
	}
	is.clientsMutex.RUnlock()

	// Send the broadcast message to each client.
	for _, client := range clientsToBroadcast {
		select {
		case client.send <- broadcastMsg:
			// Message sent successfully to client's send channel.
		default:
			// Client's send buffer is full, indicating a potential issue or slow client.
			log.Printf("WARNING: Client %s send buffer full, attempting to unregister.", client.playerID)
			is.unregister <- client // Signal unregistration to main server loop
		}
	}
}

// Run starts the InstanceServer's main loops.
func (is *InstanceServer) Run() {
	// Start a goroutine to handle client registration and unregistration requests.
	go func() {
		for {
			select {
			case client := <-is.register:
				is.registerClient(client)
			case client := <-is.unregister:
				is.unregisterClient(client)
			}
		}
	}()

	// Start the instance state broadcast loop in a separate goroutine.
	go is.broadcastInstanceStateLoop()

	// Register the WebSocket endpoint.
	http.HandleFunc("/ws", is.handleWebSocketConnection)
}
