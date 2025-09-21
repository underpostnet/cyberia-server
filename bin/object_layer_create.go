package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"math/rand"
	"net/http"
	"os"
	"time"

	"cyberia-server/api"
	game "cyberia-server/src"
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

	// Also send to API to create the object layer using default admin env
	cfg := api.LoadConfig()
	token, err := api.GenerateToken(cfg.JWTSecret, cfg.JWTIssuer, cfg.AdminUsername, api.RoleAdmin, time.Hour)
	if err != nil {
		fmt.Fprintf(os.Stderr, "warn: could not generate admin token: %v\n", err)
		return
	}
	body, err := json.Marshal(ol)
	if err != nil {
		fmt.Fprintf(os.Stderr, "warn: could not marshal object layer: %v\n", err)
		return
	}
	req, err := http.NewRequest(http.MethodPost, "http://localhost:8080/api/v1/object-layers", bytes.NewReader(body))
	if err != nil {
		fmt.Fprintf(os.Stderr, "warn: building request failed: %v\n", err)
		return
	}
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Authorization", "Bearer "+token)
	client := &http.Client{Timeout: 10 * time.Second}
	resp, err := client.Do(req)
	if err != nil {
		fmt.Fprintf(os.Stderr, "warn: API request failed: %v\n", err)
		return
	}
	defer resp.Body.Close()
	if resp.StatusCode < 200 || resp.StatusCode >= 300 {
		fmt.Fprintf(os.Stderr, "warn: API create returned status %s\n", resp.Status)
		return
	}
	fmt.Println("Created ObjectLayer via API successfully.")
}
