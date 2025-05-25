package server

import (
	"log"
	"time"

	"github.com/gorilla/websocket" // Gorilla WebSocket library for plain WebSockets
)

const (
	// WebSocket heartbeat settings to detect disconnected clients
	PING_INTERVAL = 10 * time.Second // Frequency of sending ping messages
	PONG_WAIT     = 60 * time.Second // Time to wait for a pong response before considering client disconnected
)

// WebSocketClient represents a single connected client.
type WebSocketClient struct {
	conn     *websocket.Conn // The raw WebSocket connection
	send     chan []byte     // Channel for outgoing messages to this client
	playerID string          // The unique player ID associated with this client
	done     chan struct{}   // Signal channel for goroutine termination
}

// NewWebSocketClient creates and returns a new WebSocketClient instance.
func NewWebSocketClient(conn *websocket.Conn, playerID string) *WebSocketClient {
	return &WebSocketClient{
		conn:     conn,
		send:     make(chan []byte, 256), // Buffered channel for sending messages
		playerID: playerID,
		done:     make(chan struct{}),
	}
}

// ReadPump continuously reads messages from the WebSocket connection.
// It handles disconnection detection and signals the WritePump to terminate.
func (c *WebSocketClient) ReadPump(server *InstanceServer) { // Parameter changed to *InstanceServer
	// Ensure connection is closed and client unregistered when this goroutine exits.
	defer func() {
		server.unregisterClient(c) // Unregister the client from the server's active list
		close(c.done)              // Signal the WritePump to terminate
		c.conn.Close()             // Close the underlying WebSocket connection
	}()

	// Set a read deadline and a pong handler for heartbeat.
	c.conn.SetReadDeadline(time.Now().Add(PONG_WAIT))
	c.conn.SetPongHandler(func(string) error {
		c.conn.SetReadDeadline(time.Now().Add(PONG_WAIT)) // Extend deadline on pong
		return nil
	})

	for {
		_, message, err := c.conn.ReadMessage() // Read messages
		if err != nil {
			// Log unexpected close errors, indicating a client disconnection.
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseNormalClosure) {
				log.Printf("Client %s: Unexpected WebSocket close error: %v", c.playerID, err)
			} else {
				log.Printf("Client %s: WebSocket read error (non-unexpected close): %v", c.playerID, err)
			}
			break // Exit loop on any read error, triggering defer
		}
		// Pass the received message to the server's message handler for processing.
		server.handleClientMessage(c, message) // Call server's handler
	}
}

// WritePump continuously sends messages from the 'send' channel to the WebSocket connection.
// It also sends periodic pings for heartbeat and terminates gracefully on signal.
func (c *WebSocketClient) WritePump() {
	ticker := time.NewTicker(PING_INTERVAL) // Ticker for sending periodic pings
	defer func() {
		ticker.Stop()  // Stop the ticker on exit
		c.conn.Close() // Ensure connection is closed on exit
	}()

	for {
		select {
		case message, ok := <-c.send:
			// Attempt to send a message from the 'send' channel.
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second)) // Set write deadline
			if !ok {
				// The 'send' channel was closed, indicating client unregistration.
				// Send a normal close message to the client before exiting.
				c.conn.WriteMessage(websocket.CloseMessage, []byte{})
				return // Terminate goroutine
			}

			// Send each message as a separate WebSocket frame.
			if err := c.conn.WriteMessage(websocket.TextMessage, message); err != nil {
				log.Printf("Client %s: Error sending message: %v", c.playerID, err)
				return // Terminate goroutine on write error
			}

		case <-ticker.C:
			// Send a ping message on ticker tick.
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second)) // Set write deadline for ping
			if err := c.conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				log.Printf("Client %s: Error sending ping: %v", c.playerID, err)
				return // Terminate goroutine on ping error
			}
		case <-c.done:
			// Received termination signal from ReadPump.
			log.Printf("Client %s: WritePump received done signal, terminating.", c.playerID)
			// Attempt to send a normal close message before exiting.
			err := c.conn.WriteMessage(websocket.CloseMessage, websocket.FormatCloseMessage(websocket.CloseNormalClosure, ""))
			if err != nil {
				log.Printf("Client %s: Error sending final close message: %v", c.playerID, err)
			}
			return // Terminate goroutine
		}
	}
}
