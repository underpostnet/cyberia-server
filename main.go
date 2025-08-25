package main

import (
	"log"
	"math/rand"
	"net/http"
	"time"

	game "cyberia-server/src"
)

func main() {
	rand.Seed(time.Now().UnixNano())
	s := game.NewGameServer()
	go s.Run()

	http.HandleFunc("/ws", s.HandleConnections)
	log.Println("Server started on :8080")
	if err := http.ListenAndServe(":8080", nil); err != nil {
		log.Fatal("ListenAndServe:", err)
	}
}
