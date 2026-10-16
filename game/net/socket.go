// Package net — socket.go
//
// The socket moves bytes on one WebSocket connection. It never parses or
// builds messages — that is the job of serial.go.
package net

import (
	"sync"
	"sync/atomic"
	"time"

	"github.com/gorilla/websocket"
)

const (
	sendQueue = 256
	// readLimit bounds one inbound client frame: one batch of client events.
	// It holds a full batch of 512 taps, so a batch over the cap reaches the
	// cap check in receiveMessage instead of the socket.
	readLimit    = 64 * 1024
	readTimeout  = 60 * time.Second
	writeTimeout = 10 * time.Second
	pingInterval = 30 * time.Second
	// authReadLimit bounds the first frame: the auth message.
	authReadLimit = 2 * 1024
)

// Metrics counts bytes and errors. Every field is optional.
type Metrics struct {
	Read       func(n int)
	Write      func(n int)
	ReadError  func()
	WriteError func()
}

// maxConsecutiveDrops is how many queue-full drops in a row mark a peer as
// too slow to keep. At the snapshot rate this is a few seconds of a client
// that reads nothing, which no live connection does.
const maxConsecutiveDrops = 64

// Socket is the single writer for one connection.
type Socket struct {
	conn    *websocket.Conn
	send    chan []byte
	end     chan []byte
	metrics Metrics

	closeOnce sync.Once
	// ending marks a connection whose last message is queued; the write pump
	// closes it.
	ending atomic.Bool
	// drops counts consecutive failed sends. A successful send resets it.
	drops atomic.Uint32
}

// NewSocket starts the write pump for an accepted connection.
func NewSocket(conn *websocket.Conn, metrics Metrics) *Socket {
	conn.EnableWriteCompression(true)
	s := &Socket{conn: conn, send: make(chan []byte, sendQueue), end: make(chan []byte, 1), metrics: metrics}
	go s.writePump()
	return s
}

// Send queues a pack for the write pump. It returns false when the queue is
// full — the caller decides if the message is worth a retry.
//
// The send never blocks, so one slow peer cannot hold up the caller. A peer
// that stays full for maxConsecutiveDrops sends is closed: its queue will not
// drain and every later snapshot would be dropped anyway.
func (s *Socket) Send(pack []byte) bool {
	select {
	case s.send <- pack:
		s.drops.Store(0)
		return true
	default:
		if s.drops.Add(1) >= maxConsecutiveDrops {
			s.Close()
		}
		return false
	}
}

// ReadFirst reads the first frame of the connection: at most authReadLimit
// bytes within timeout.
func (s *Socket) ReadFirst(timeout time.Duration) ([]byte, error) {
	s.conn.SetReadLimit(authReadLimit)
	s.conn.SetReadDeadline(time.Now().Add(timeout))
	_, pack, err := s.conn.ReadMessage()
	if err == nil && s.metrics.Read != nil {
		s.metrics.Read(len(pack))
	}
	return pack, err
}

// End writes pack as the last message, then closes the connection with a
// normal close frame. A Close after End waits for it; writeTimeout bounds the
// wait.
func (s *Socket) End(pack []byte) {
	if !s.ending.CompareAndSwap(false, true) {
		return
	}
	s.end <- pack
	time.AfterFunc(writeTimeout, s.shut)
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

// Close closes the connection, unless End already queued its last message.
// Safe to call more than once, from any goroutine. The write pump exits on its
// next write or ping. The send queue stays open so a concurrent Send can never
// panic.
func (s *Socket) Close() {
	if !s.ending.Load() {
		s.shut()
	}
}

func (s *Socket) shut() { s.closeOnce.Do(func() { s.conn.Close() }) }

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
		case pack := <-s.end:
			s.conn.SetWriteDeadline(time.Now().Add(writeTimeout))
			if s.conn.WriteMessage(websocket.BinaryMessage, pack) == nil {
				s.conn.WriteMessage(websocket.CloseMessage, websocket.FormatCloseMessage(websocket.CloseNormalClosure, ""))
			}
			s.shut()
			return
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
