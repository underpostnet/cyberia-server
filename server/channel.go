package server

import (
	"encoding/json"
	"log"
	"sync"
	"time"

	"cyberia-server/config"
	"cyberia-server/network_state"
)

const (
	// maxChatHistory defines the maximum number of chat messages stored per channel.
	maxChatHistory = 100
)

// Channel represents a distinct game world instance with its own state and clients.
type Channel struct {
	ID               string
	networkState     *network_state.NetworkState
	clients          map[*WebSocketClient]bool
	clientsMutex     sync.RWMutex
	factory          *network_state.ServerNetworkObjectFactory // Used to generate initial state
	chatHistory      []ChatMessage
	chatHistoryMutex sync.RWMutex
	ticker           *time.Ticker
	done             chan struct{} // Signals the channel to shut down its Run loop
}

// NewChannel creates and initializes a new game Channel.
// It populates the channel with initial persistent objects (walls, bots)
// using the provided factory and starts its update loop.
func NewChannel(id string, factory *network_state.ServerNetworkObjectFactory) *Channel {
	log.Printf("Channel %s: Initializing...", id)
	ns := network_state.NewNetworkState()

	// Generate initial persistent objects (walls, bots) for the channel.
	// GenerateInitialState requires a playerID; we use a temporary one and then
	// remove the dummy player object it creates. This is a workaround for the
	// current factory design, which bundles player creation with world creation.
	tempPlayerIDForSetup := "channel_init_temp_player_" + id
	initialObjects, err := factory.GenerateInitialState(id, tempPlayerIDForSetup)
	if err != nil {
		log.Printf("Channel %s: ERROR Failed to generate initial state from factory: %v. Proceeding with empty state.", id, err)
		initialObjects = make(map[string]*network_state.NetworkObject) // Fallback
	}

	// Remove the temporary player object if it was created by the factory
	if _, ok := initialObjects[tempPlayerIDForSetup]; ok {
		delete(initialObjects, tempPlayerIDForSetup)
	}

	ns.InitializeWithObjects(initialObjects)
	ns.BuildSimplifiedMaze() // Crucial after adding obstacles from initialObjects

	ch := &Channel{
		ID:           id,
		networkState: ns,
		clients:      make(map[*WebSocketClient]bool),
		factory:      factory, // Retain factory if needed for other operations later
		chatHistory:  make([]ChatMessage, 0, maxChatHistory),
		ticker:       time.NewTicker(config.NETWORK_STATE_TICK_INTERVAL),
		done:         make(chan struct{}),
	}

	go ch.Run() // Start the channel's update and broadcast loop
	log.Printf("Channel %s: Initialized and running.", id)
	return ch
}

// Run is the main loop for the channel, ticking at NETWORK_STATE_TICK_INTERVAL.
// It updates object positions and broadcasts the network state to clients.
func (c *Channel) Run() {
	log.Printf("Channel %s: Starting update loop.", c.ID)
	defer func() {
		c.ticker.Stop()
		log.Printf("Channel %s: Update loop stopped.", c.ID)
	}()

	for {
		select {
		case <-c.ticker.C:
			c.updateAndBroadcast()
		case <-c.done:
			return
		}
	}
}

// updateAndBroadcast handles a single tick of game logic: updating positions and broadcasting.
func (c *Channel) updateAndBroadcast() {
	deltaTime := float64(config.NETWORK_STATE_TICK_INTERVAL) / float64(time.Second)

	c.networkState.Mu.Lock()
	for _, obj := range c.networkState.NetworkObjects {
		if len(obj.Path) > 0 { // Only update objects that have a path (e.g., moving players, bots)
			obj.UpdatePosition(deltaTime)
		}
	}
	c.networkState.Mu.Unlock()

	c.broadcastNetworkState()
}

// broadcastNetworkState sends the current state of all network objects in the channel to its clients.
func (c *Channel) broadcastNetworkState() {
	c.networkState.Mu.RLock()
	// Create a snapshot to minimize lock time, though marshaling might still be significant.
	// For very large states, consider more granular updates or delta encoding.
	objectsSnapshot := make(map[string]*network_state.NetworkObject, len(c.networkState.NetworkObjects))
	for id, obj := range c.networkState.NetworkObjects {
		objectsSnapshot[id] = obj // Shallow copy of pointers is fine here
	}
	c.networkState.Mu.RUnlock()

	c.clientsMutex.RLock()
	if len(c.clients) == 0 { // No clients, no broadcast
		c.clientsMutex.RUnlock()
		return
	}

	var cleanBackgroundColor []int
	if c.ID == config.ChannelAlphaID {
		cleanBackgroundColor = config.ChannelAlphaCleanBackgroundColor
	} else if c.ID == config.ChannelBetaID {
		cleanBackgroundColor = config.ChannelBetaCleanBackgroundColor
	}

	// Only marshal if there are clients to send to
	stateUpdateMsg, err := json.Marshal(map[string]interface{}{
		"type":                   "network_state_update",
		"network_objects":        objectsSnapshot,
		"clean_background_color": cleanBackgroundColor,
	})
	if err != nil {
		c.clientsMutex.RUnlock()
		log.Printf("Channel %s: ERROR marshaling network state for broadcast: %v", c.ID, err)
		return
	}

	for client := range c.clients {
		select {
		case client.send <- stateUpdateMsg:
		default:
			log.Printf("Channel %s: WARNING Client %s send buffer full during network state broadcast.", c.ID, client.playerID)
			// The client's WritePump will likely handle unregistration if persistently blocked.
		}
	}
	c.clientsMutex.RUnlock()
}

// AddClient adds a WebSocketClient to the channel, creates their player network object,
// and sends them the recent chat history.
func (c *Channel) AddClient(client *WebSocketClient) {
	c.clientsMutex.Lock()
	c.clients[client] = true
	c.clientsMutex.Unlock()

	client.ChannelID = c.ID // Associate client with this channel

	// Determine spawn point for the player
	spawnPoints, ok := config.ChannelPlayerSpawns[c.ID]
	if !ok || len(spawnPoints) == 0 {
		spawnPoints = [][2]float64{config.DefaultPlayerSpawn} // Fallback to default
	}
	chosenSpawn := spawnPoints[0] // Use the first configured spawn point for simplicity

	playerObj := network_state.NewNetworkObject(
		client.playerID,
		chosenSpawn[0], chosenSpawn[1],
		config.PlayerDefaultColor,
		false, // IsObstacle
		config.DefaultPlayerSpeed,
		"PLAYER",
		config.DefaultObjectLayerIDs["PLAYER"],
		true, // IsPersistent (player objects exist as long as client is connected)
	)
	c.networkState.AddNetworkObject(playerObj) // This handles internal locking for NetworkState

	log.Printf("Channel %s: Client %s (PlayerID: %s) added. Player object created at (%.0f, %.0f).", c.ID, client.conn.RemoteAddr().String(), client.playerID, playerObj.X, playerObj.Y)

	c.sendChatHistoryToClient(client)
	// The "player_assigned" message, including initial position, is sent by server.go
	// after this function returns, using the playerObj created here.
}

// RemoveClient removes a WebSocketClient from the channel and their associated player network object.
func (c *Channel) RemoveClient(client *WebSocketClient) {
	removed := false
	c.clientsMutex.Lock()
	if _, exists := c.clients[client]; exists {
		delete(c.clients, client)
		removed = true
	}
	c.clientsMutex.Unlock()

	if removed {
		log.Printf("Channel %s: Client %s (PlayerID: %s) removed from active list.", c.ID, client.conn.RemoteAddr().String(), client.playerID)
		c.networkState.RemoveNetworkObject(client.playerID) // This handles internal locking
		log.Printf("Channel %s: Player object %s removed from network state.", c.ID, client.playerID)
	}
}

// AddChatMessage adds a message to the channel's chat history.
// Broadcasting is handled by NetworkStateServer.sendChatMessageToChannelClients.
func (c *Channel) AddChatMessage(message ChatMessage) {
	c.chatHistoryMutex.Lock()
	defer c.chatHistoryMutex.Unlock()

	c.chatHistory = append(c.chatHistory, message)
	if len(c.chatHistory) > maxChatHistory {
		c.chatHistory = c.chatHistory[len(c.chatHistory)-maxChatHistory:]
	}
}

// sendChatHistoryToClient sends the recent chat messages to a newly connected client.
func (c *Channel) sendChatHistoryToClient(client *WebSocketClient) {
	c.chatHistoryMutex.RLock()
	defer c.chatHistoryMutex.RUnlock()

	if len(c.chatHistory) == 0 {
		return
	}

	historyData := make([]interface{}, len(c.chatHistory))
	for i, msg := range c.chatHistory {
		historyData[i] = msg // ChatMessage struct is already JSON-serializable
	}

	payload, err := json.Marshal(map[string]interface{}{
		"type": "chat_history",
		"data": historyData,
	})
	if err != nil {
		log.Printf("Channel %s: ERROR marshaling chat history for client %s: %v", c.ID, client.playerID, err)
		return
	}

	select {
	case client.send <- payload:
	default:
		log.Printf("Channel %s: WARNING Client %s send buffer full when sending chat history.", c.ID, client.playerID)
	}
}

// Close signals the channel to shut down its Run loop and cleans up resources.
func (c *Channel) Close() {
	log.Printf("Channel %s: Closing...", c.ID)
	close(c.done) // Signal Run loop to stop

	c.clientsMutex.Lock()
	c.clients = make(map[*WebSocketClient]bool) // Clear client list
	c.clientsMutex.Unlock()
	// Individual client connections are closed by the server's unregister mechanism.
	log.Printf("Channel %s: Closed.", c.ID)
}
