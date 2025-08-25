package main

import (
	"fmt"
	"log"
	"net/http"

	"cyberia-server/server"
)

const (
	SERVER_HOST = "0.0.0.0"
	SERVER_PORT = 5000
)

func main() {
	// Create a new instance of the NetworkStateServer.
	// This server manages WebSocket connections and the network state.
	networkStateServer := server.NewNetworkStateServer()

	// Start the NetworkStateServer's main loop and WebSocket handling in a goroutine.
	// The Run method will set up HTTP handlers and start listening.
	go networkStateServer.Run()

	// Register a simple HTTP handler for the root path.
	// This is useful for checking if the server is alive.
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Write([]byte("Go Plain WebSocket Network State Server Running. Connect to /ws for network state."))
	})

	log.Printf("Starting HTTP server on http://%s:%d\n", SERVER_HOST, SERVER_PORT)
	// Start the HTTP server. This call blocks until a fatal error occurs.
	log.Fatal(http.ListenAndServe(fmt.Sprintf("%s:%d", SERVER_HOST, SERVER_PORT), nil))
}
