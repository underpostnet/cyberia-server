package main

import (
	"fmt"
	"math/rand"
	"os"
	"time"

	game "cyberia-server/v7/src"
)

func main() {
	rand.Seed(time.Now().UnixNano())

	out := game.DefaultOutPath()
	if len(os.Args) > 1 {
		out = os.Args[1]
	}
	out = game.NormalizePath(out)

	ol := game.BuildRandomObjectLayer()
	if err := game.WriteJSON(out, ol); err != nil {
		fmt.Fprintf(os.Stderr, "error: %v\n", err)
		os.Exit(1)
	}
	fmt.Printf("Wrote ObjectLayer JSON to %s\n", out)
}
