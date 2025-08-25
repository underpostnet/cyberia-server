package server

import (
	"cyberia-server/test/config"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"sync"

	// Already imported
	// Updated import path

	"github.com/gorilla/websocket"
)

// ChatMessage represents a single chat message.
type ChatMessage struct {
	RoomID    string `json:"room_id"` // Added RoomID to identify chat room within a channel
	Sender    string `json:"sender"`
	Text      string `json:"text"`
	Timestamp string `json:"time"`
}

// NetworkStateServer manages WebSocket connections and the network state.
type NetworkStateServer struct {
	upgrader         websocket.Upgrader
	channelManager   *ChannelManager // Replaces single networkState
	clients          map[*WebSocketClient]bool
	playerIDToClient map[string]**WebSocketClient
	register         chan *WebSocketClient
	unregister       chan *WebSocketClient
	playerCounter    int
	clientsMutex     sync.RWMutex
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
		channelManager:   NewChannelManager(), // Initialize ChannelManager
		clients:          make(map[*WebSocketClient]bool),
		playerIDToClient: make(map[string]**WebSocketClient),
		register:         make(chan *WebSocketClient),
		unregister:       make(chan *WebSocketClient),
		playerCounter:    0,
		clientsMutex:     sync.RWMutex{},
	}
	return ns
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

	// Assign client to default channel and create player object within that channel
	ns.channelManager.AddClientToChannel(client, config.DefaultChannelID)

	// Retrieve the player object to get its initial position for the player_assigned message
	// This assumes AddClientToChannel correctly creates and places the player.
	defaultChan, _ := ns.channelManager.GetChannel(config.DefaultChannelID)
	defaultChan.networkState.Mu.RLock()
	playerObj, playerExists := defaultChan.networkState.NetworkObjects[playerID]
	defaultChan.networkState.Mu.RUnlock()

	if !playerExists {
		log.Printf("CRITICAL ERROR: Player object %s not found after adding to channel. Disconnecting.", playerID)
		conn.Close()
		return
	}

	playerAssignedMsg, _ := json.Marshal(map[string]interface{}{
		"type":       "player_assigned",
		"player_id":  playerID,
		"x":          playerObj.X, // Send actual spawn position
		"y":          playerObj.Y,
		"channel_id": client.ChannelID, // Inform client of their initial channel
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
	// Initial state will be sent by the channel's broadcast or a specific "welcome" message.
	// For now, let the channel's regular broadcast handle it.
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

	ns.channelManager.RemoveClientFromChannel(client) // Remove from its channel
	log.Printf("Client %s (ID: %s) unregistered and player network object removed.", client.conn.RemoteAddr().String(), client.playerID)
}

// sendChatMessageToChannelClients sends a chat message to all clients in a specific channel.
func (ns *NetworkStateServer) sendChatMessageToChannelClients(channel *Channel, message ChatMessage) {
	channel.clientsMutex.RLock()
	defer channel.clientsMutex.RUnlock()

	chatMsgData := map[string]interface{}{
		"room_id": message.RoomID, // Use RoomID from message struct
		"sender":  message.Sender,
		"text":    message.Text,
		"time":    message.Timestamp,
	}
	// Add player_id to the message data if needed by client for self-identification
	// chatMsgData["player_id"] = message.Sender // Assuming sender is player_id

	broadcastMsg, err := json.Marshal(map[string]interface{}{
		"type": "server_chat_message",
		"data": chatMsgData,
	})
	if err != nil {
		log.Printf("Channel %s: ERROR: Failed to marshal chat message for broadcast: %v", channel.ID, err)
		return
	}

	for client := range channel.clients {
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

	// Broadcast loops are now managed per channel by the ChannelManager/Channel itself.

	http.HandleFunc("/ws", ns.handleWebSocketConnection)
}
