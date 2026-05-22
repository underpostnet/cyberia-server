package main

import (
	"context"
	"log"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"time"

	api "cyberia-server/api"
	game "cyberia-server/game"
	"cyberia-server/grpcclient"

	"github.com/go-chi/chi/v5"
	"github.com/joho/godotenv"
)

// findEnvFile searches for a .env file starting from the current working
// directory and walking up until it finds one or reaches the project root
// (identified by go.mod). This allows `go run` from cmd/cyberia-server/
// to pick up the project-root .env.
func findEnvFile() string {
	dir, err := os.Getwd()
	if err != nil {
		return ""
	}
	for {
		candidate := filepath.Join(dir, ".env")
		if _, err := os.Stat(candidate); err == nil {
			return candidate
		}
		// Stop at project root (where go.mod lives) to avoid escaping the repo.
		if _, err := os.Stat(filepath.Join(dir, "go.mod")); err == nil {
			break
		}
		parent := filepath.Dir(dir)
		if parent == dir {
			break
		}
		dir = parent
	}
	return ""
}

func main() {
	// Load .env file if present (does not override already-set env vars).
	// Walks up from CWD so that running from cmd/cyberia-server/ still finds
	// the project-root .env file.
	envPath := findEnvFile()
	if envPath != "" {
		if err := godotenv.Load(envPath); err != nil {
			log.Printf("Failed to load %s: %v", envPath, err)
		} else {
			log.Printf("Loaded env from %s", envPath)
		}
	} else {
		log.Println("No .env file found; relying on environment variables.")
	}

	// Core game server
	s := game.NewGameServer()

	// Set the engine API base URL (forwarded to clients for binary blob fetches)
	if apiUrl := os.Getenv("ENGINE_API_BASE_URL"); apiUrl != "" {
		s.SetEngineApiBaseUrl(apiUrl)
	}

	// ── Data loading: gRPC ──────────
	// ENGINE_GRPC_ADDRESS selects the Engine gRPC endpoint.
	// Defaults to localhost:50051 for local/dev runs (same default as grpcclient).
	// The server exits if the Engine is unreachable — no fallback world is generated.
	instanceCode := os.Getenv("INSTANCE_CODE")
	grpcAddr := os.Getenv("ENGINE_GRPC_ADDRESS")
	if grpcAddr == "" {
		grpcAddr = "localhost:50051"
		log.Printf("ENGINE_GRPC_ADDRESS not set — defaulting to %s", grpcAddr)
	}
	{
		gc, err := grpcclient.New(grpcclient.Config{
			Address: grpcAddr,
		})
		if err != nil {
			log.Fatalf("Engine gRPC required: dial %s failed: %v", grpcAddr, err)
		}
		wb := grpcclient.NewWorldBuilder(gc, s)
		wb.InstanceCode = instanceCode
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Minute)
		if err := wb.LoadAll(ctx); err != nil {
			cancel()
			log.Fatalf("Engine gRPC world load failed: %v — Engine must be running before starting cyberia-server.", err)
		}
		cancel()

		// Start hot-reload loop if configured
		reloadSec := os.Getenv("ENGINE_GRPC_RELOAD_INTERVAL_SEC")
		if reloadSec != "" {
			if sec, err := strconv.Atoi(reloadSec); err == nil && sec > 0 {
				wb.ReloadInterval = time.Duration(sec) * time.Second
				wb.StartReloadLoop()
				defer wb.Stop()
			}
		}
		defer gc.Close()
	}

	go s.Run()

	r := chi.NewRouter()

	// Use environment variable `STATIC_DIR` or default to `./public` for static files.
	// This serves the static frontend application.
	staticDir := os.Getenv("STATIC_DIR")
	if staticDir == "" {
		staticDir = "./public"
	}
	r.Handle("/*", game.StaticFileServer(staticDir, "/index.html"))

	// Mount REST API under /api
	r.Mount("/api", api.NewAPIRouter(s))
	// Keep websocket endpoint
	r.HandleFunc("/ws", s.HandleConnections)

	port := os.Getenv("SERVER_PORT")
	if port == "" {
		port = "8081"
	}

	srv := &http.Server{
		Addr:    ":" + port,
		Handler: r,
	}

	log.Printf("Server started on :%s", port)

	// Fire READY_CMD in the background once the server is bound and listening.
	// Mirrors the ready_cmd behaviour in server.py (5th argument pattern).
	if readyCmd := os.Getenv("READY_CMD"); readyCmd != "" {
		go func() {
			// Small grace period so the HTTP server is fully accepting connections.
			time.Sleep(500 * time.Millisecond)
			log.Printf("Executing READY_CMD: %s", readyCmd)
			cmd := exec.Command("sh", "-c", readyCmd)
			cmd.Stdout = os.Stdout
			cmd.Stderr = os.Stderr
			if err := cmd.Run(); err != nil {
				log.Printf("READY_CMD failed: %v", err)
			}
		}()
	}

	if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		log.Fatal("ListenAndServe:", err)
	}
}
