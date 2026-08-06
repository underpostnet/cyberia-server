// Package socket moves bytes on one WebSocket connection.
// It never parses or builds messages — that is the job of package serial.
package socket

import (
	"net/http"
	"time"

	"github.com/gorilla/websocket"
)

const (
	sendQueue = 256
	// readLimit bounds one inbound client frame. The largest client → server
	// message is a chat line inside the JSON envelope.
	readLimit    = 8 * 1024
	readTimeout  = 60 * time.Second
	writeTimeout = 10 * time.Second
	pingInterval = 30 * time.Second
)

// Upgrader accepts a WebSocket connection. Compression pays back the size of
// the JSON frames — snapshot keys repeat and deflate well.
var Upgrader = websocket.Upgrader{
	ReadBufferSize:    1024,
	WriteBufferSize:   1024,
	EnableCompression: true,
	CheckOrigin:       func(r *http.Request) bool { return true },
}

// Metrics counts bytes and errors. Every field is optional.
type Metrics struct {
	Read       func(n int)
	Write      func(n int)
	ReadError  func()
	WriteError func()
}

// Socket is the single writer for one connection.
type Socket struct {
	conn    *websocket.Conn
	send    chan []byte
	metrics Metrics
}

// New starts the write pump for an accepted connection.
func New(conn *websocket.Conn, metrics Metrics) *Socket {
	conn.EnableWriteCompression(true)
	s := &Socket{conn: conn, send: make(chan []byte, sendQueue), metrics: metrics}
	go s.writePump()
	return s
}

// Send queues a pack for the write pump. It returns false when the queue is
// full — the caller decides if the message is worth a retry.
func (s *Socket) Send(pack []byte) bool {
	select {
	case s.send <- pack:
		return true
	default:
		return false
	}
}

// Receive runs the read loop until the connection fails. It calls onPack for
// each frame.
func (s *Socket) Receive(onPack func(pack []byte)) {
	s.conn.SetReadLimit(readLimit)
	s.conn.SetReadDeadline(time.Now().Add(readTimeout))
	s.conn.SetPongHandler(func(string) error {
		return s.conn.SetReadDeadline(time.Now().Add(readTimeout))
	})
	for {
		_, pack, err := s.conn.ReadMessage()
		if err != nil {
			if s.metrics.ReadError != nil {
				s.metrics.ReadError()
			}
			return
		}
		if len(pack) == 0 {
			continue
		}
		if s.metrics.Read != nil {
			s.metrics.Read(len(pack))
		}
		onPack(pack)
	}
}

// Close closes the connection. Safe to call more than once. The write pump
// exits on its next write or ping.
// ponytail: the send queue stays open so a concurrent Send can never panic.
func (s *Socket) Close() { s.conn.Close() }

// writePump is the only goroutine that writes to the connection. It also
// sends the keep-alive pings.
func (s *Socket) writePump() {
	ticker := time.NewTicker(pingInterval)
	defer ticker.Stop()
	for {
		select {
		case pack := <-s.send:
			s.conn.SetWriteDeadline(time.Now().Add(writeTimeout))
			if err := s.conn.WriteMessage(websocket.BinaryMessage, pack); err != nil {
				if s.metrics.WriteError != nil {
					s.metrics.WriteError()
				}
				return
			}
			if s.metrics.Write != nil {
				s.metrics.Write(len(pack))
			}
		case <-ticker.C:
			s.conn.SetWriteDeadline(time.Now().Add(writeTimeout))
			if err := s.conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				if s.metrics.WriteError != nil {
					s.metrics.WriteError()
				}
				return
			}
		}
	}
}
