package server

import (
	"log"
	"time"

	"github.com/gorilla/websocket"
)

const (
	PING_INTERVAL = 10 * time.Second
	PONG_WAIT     = 60 * time.Second
)

// WebSocketClient represents a single connected client.
type WebSocketClient struct {
	conn      *websocket.Conn
	send      chan []byte
	playerID  string
	ChannelID string
	done      chan struct{}
}

// NewWebSocketClient creates a new WebSocketClient instance.
func NewWebSocketClient(conn *websocket.Conn, playerID string) *WebSocketClient {
	return &WebSocketClient{
		conn:     conn,
		send:     make(chan []byte, 256),
		playerID: playerID,
		done:     make(chan struct{}),
	}
}

// ReadPump continuously reads messages from the WebSocket connection.
func (c *WebSocketClient) ReadPump(server *NetworkStateServer) {
	defer func() {
		server.unregisterClient(c)
		close(c.done)
		c.conn.Close()
	}()

	c.conn.SetReadDeadline(time.Now().Add(PONG_WAIT))
	c.conn.SetPongHandler(func(string) error {
		c.conn.SetReadDeadline(time.Now().Add(PONG_WAIT))
		return nil
	})

	for {
		_, message, err := c.conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseNormalClosure) {
				log.Printf("Client %s: Unexpected WebSocket close error: %v", c.playerID, err)
			} else {
				log.Printf("Client %s: WebSocket read error: %v", c.playerID, err)
			}
			break
		}
		server.handleClientMessage(c, message)
	}
}

// WritePump continuously sends messages from the 'send' channel to the WebSocket connection.
func (c *WebSocketClient) WritePump() {
	ticker := time.NewTicker(PING_INTERVAL)
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

			if err := c.conn.WriteMessage(websocket.TextMessage, message); err != nil {
				log.Printf("Client %s: Error sending message: %v", c.playerID, err)
				return
			}

		case <-ticker.C:
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if err := c.conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				log.Printf("Client %s: Error sending ping: %v", c.playerID, err)
				return
			}
		case <-c.done:
			log.Printf("Client %s: WritePump received done signal, terminating.", c.playerID)
			err := c.conn.WriteMessage(websocket.CloseMessage, websocket.FormatCloseMessage(websocket.CloseNormalClosure, ""))
			if err != nil {
				log.Printf("Client %s: Error sending final close message: %v", c.playerID, err)
			}
			return
		}
	}
}
