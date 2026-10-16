// Package net — origin.go
//
// The browser origins that may open the game socket. A pattern holds at most
// one "*", which matches any run of characters; any other pattern matches one
// origin exactly. A request without an Origin header is not from a browser
// page and passes: the launch ticket, not the origin, authenticates it.
package net

import (
	"net/http"
	"strings"

	"github.com/gorilla/websocket"
)

// originAllowed reports whether origin matches one of patterns.
func originAllowed(patterns []string, origin string) bool {
	for _, pattern := range patterns {
		prefix, suffix, wild := strings.Cut(pattern, "*")
		switch {
		case !wild && pattern == origin:
			return true
		case wild && len(origin) >= len(prefix)+len(suffix) && strings.HasPrefix(origin, prefix) && strings.HasSuffix(origin, suffix):
			return true
		}
	}
	return false
}

// newUpgrader accepts a WebSocket connection from an allowed origin.
// Compression pays back the size of the JSON frames — snapshot keys repeat and
// deflate well.
func newUpgrader(origins []string) websocket.Upgrader {
	return websocket.Upgrader{
		ReadBufferSize:    1024,
		WriteBufferSize:   1024,
		EnableCompression: true,
		CheckOrigin: func(r *http.Request) bool {
			origin := r.Header.Get("Origin")
			return origin == "" || originAllowed(origins, origin)
		},
	}
}
