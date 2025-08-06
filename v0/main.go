package main

import (
	"encoding/json"
	"log"
	"net/http"
	"sync"
	"time"

	"github.com/google/uuid"
	"github.com/gorilla/websocket"
)

// Player represents a connected client in the game.
type Player struct {
	ID    string          `json:"id"`
	X     float64         `json:"x"`
	Y     float64         `json:"y"`
	MapID int             `json:"mapId"`
	Conn  *websocket.Conn `json:"-"` // Ignore this field during JSON marshaling
	Send  chan []byte     `json:"-"` // Ignore this field during JSON marshaling
}

// Message represents the WebSocket message structure for communication.
type Message struct {
	Type     string   `json:"type"`
	PlayerID string   `json:"playerId,omitempty"`
	X        float64  `json:"x,omitempty"`
	Y        float64  `json:"y,omitempty"`
	MapID    int      `json:"mapId,omitempty"`
	Players  []Player `json:"players,omitempty"`
}

// Hub maintains the set of active clients and broadcasts messages.
type Hub struct {
	// Registered clients.
	clients map[*websocket.Conn]*Player

	// Inbound messages from the clients.
	broadcast chan []byte

	// Register requests from the clients.
	register chan *Player

	// Unregister requests from clients.
	unregister chan *Player

	mu sync.RWMutex // Mutex to protect clients map
}

// NewHub creates and returns a new Hub instance.
func NewHub() *Hub {
	return &Hub{
		broadcast:  make(chan []byte),
		register:   make(chan *Player),
		unregister: make(chan *Player),
		clients:    make(map[*websocket.Conn]*Player),
	}
}

// Run starts the hub's main loop, handling client registration, unregistration, and message broadcasting.
func (h *Hub) Run() {
	for {
		select {
		case player := <-h.register:
			h.mu.Lock()
			h.clients[player.Conn] = player
			h.mu.Unlock()
			log.Printf("Player %s connected. Total players: %d", player.ID, len(h.clients))

			// Send initial player ID to the newly connected client
			initMsg := Message{
				Type:     "init",
				PlayerID: player.ID,
			}
			if err := player.Conn.WriteJSON(initMsg); err != nil {
				log.Printf("Error sending init message to player %s: %v", player.ID, err)
				player.Conn.Close()
				h.unregister <- player
			}

		case player := <-h.unregister:
			h.mu.Lock()
			if _, ok := h.clients[player.Conn]; ok {
				delete(h.clients, player.Conn)
				close(player.Send)
			}
			h.mu.Unlock()
			log.Printf("Player %s disconnected. Total players: %d", player.ID, len(h.clients))

			// Notify other clients about disconnection
			disconnectMsg := Message{
				Type:     "player_disconnect",
				PlayerID: player.ID,
			}
			h.broadcastMessage(disconnectMsg)

		case message := <-h.broadcast:
			h.mu.RLock()
			for conn, player := range h.clients {
				select {
				case player.Send <- message:
				default:
					close(player.Send)
					delete(h.clients, conn)
				}
			}
			h.mu.RUnlock()
		}
	}
}

// broadcastMessage sends a structured message to all connected clients.
func (h *Hub) broadcastMessage(msg Message) {
	jsonMsg, err := json.Marshal(msg)
	if err != nil {
		log.Printf("Error marshalling message: %v", err)
		return
	}
	h.broadcast <- jsonMsg
}

// GetPlayersOnMap returns a list of all players currently on a specific map.
func (h *Hub) GetPlayersOnMap(mapID int) []Player {
	h.mu.RLock()
	defer h.mu.RUnlock()

	var playersOnMap []Player
	for _, p := range h.clients {
		if p.MapID == mapID {
			playersOnMap = append(playersOnMap, Player{
				ID:    p.ID,
				X:     p.X,
				Y:     p.Y,
				MapID: p.MapID,
			})
		}
	}
	return playersOnMap
}

// ReadPump reads messages from the WebSocket connection and sends them to the hub.
func (p *Player) ReadPump(hub *Hub) {
	defer func() {
		hub.unregister <- p
		p.Conn.Close()
	}()
	p.Conn.SetReadLimit(512)
	p.Conn.SetReadDeadline(time.Now().Add(60 * time.Second)) // Pong wait time
	p.Conn.SetPongHandler(func(string) error { p.Conn.SetReadDeadline(time.Now().Add(60 * time.Second)); return nil })

	for {
		_, message, err := p.Conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("Error reading message from player %s: %v", p.ID, err)
			}
			break
		}

		var msg Message
		if err := json.Unmarshal(message, &msg); err != nil {
			log.Printf("Error unmarshalling message from player %s: %v", p.ID, err)
			continue
		}

		switch msg.Type {
		case "player_update":
			// Update player's state in the hub
			hub.mu.Lock()
			if existingPlayer, ok := hub.clients[p.Conn]; ok {
				existingPlayer.X = msg.X
				existingPlayer.Y = msg.Y
				existingPlayer.MapID = msg.MapID
				p.X = msg.X // Update the player instance in the goroutine
				p.Y = msg.Y
				p.MapID = msg.MapID
			}
			hub.mu.Unlock()

			// Broadcast updated player list for the current map
			playersOnMap := hub.GetPlayersOnMap(p.MapID)
			updateMsg := Message{
				Type:    "player_update",
				Players: playersOnMap,
			}
			hub.broadcastMessage(updateMsg)

		default:
			log.Printf("Unknown message type from player %s: %s", p.ID, msg.Type)
		}
	}
}

// WritePump writes messages from the hub to the WebSocket connection.
func (p *Player) WritePump() {
	ticker := time.NewTicker(50 * time.Second) // Ping interval
	defer func() {
		ticker.Stop()
		p.Conn.Close()
	}()
	for {
		select {
		case message, ok := <-p.Send:
			p.Conn.SetWriteDeadline(time.Now().Add(10 * time.Second)) // Write wait time
			if !ok {
				// The hub closed the channel.
				p.Conn.WriteMessage(websocket.CloseMessage, []byte{})
				return
			}

			w, err := p.Conn.NextWriter(websocket.TextMessage)
			if err != nil {
				return
			}
			w.Write(message)

			// Add queued chat messages to the current WebSocket message.
			n := len(p.Send)
			for i := 0; i < n; i++ {
				w.Write(<-p.Send)
			}

			if err := w.Close(); err != nil {
				return
			}
		case <-ticker.C:
			p.Conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if err := p.Conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				return
			}
		}
	}
}

var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin: func(r *http.Request) bool {
		// Allow all origins for development. In production, restrict this.
		return true
	},
}

// ServeWs handles WebSocket requests from clients.
func ServeWs(hub *Hub, w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println(err)
		return
	}

	playerID := uuid.New().String()
	player := &Player{
		ID:    playerID,
		Conn:  conn,
		Send:  make(chan []byte, 256), // Buffered channel for sending messages
		X:     0.0,                    // Initial default position
		Y:     0.0,
		MapID: 1, // Initial map ID
	}

	hub.register <- player

	// Allow collection of memory for the player when the websocket connection is closed
	go player.WritePump()
	go player.ReadPump(hub)
}

func main() {
	hub := NewHub()
	go hub.Run() // Start the hub in a goroutine

	http.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		ServeWs(hub, w, r)
	})

	port := ":8080"
	log.Printf("WebSocket server starting on port %s", port)
	err := http.ListenAndServe(port, nil)
	if err != nil {
		log.Fatalf("Error starting server: %v", err)
	}
}
