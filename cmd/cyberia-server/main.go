package main

import (
	"context"
	"log"
	"net"
	"net/http"
	"os"
	"os/exec"

	"path/filepath"
	"strconv"
	"time"

	api "cyberia-server/api"
	"cyberia-server/api/problem"
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

// findPublicDir locates the static asset directory. Resolution order:
//  1. The literal value of STATIC_DIR if set and the directory exists
//     and contains an index.html.
//  2. ./public relative to CWD if it contains an index.html.
//  3. Walk up to the directory containing go.mod and use ./public there.
//
// This makes `go run ./cmd/cyberia-server` work from any subdirectory
// without forcing the operator to set STATIC_DIR. Returning an empty
// string is the caller's signal that no usable directory was found.
func findPublicDir() string {
	hasIndex := func(d string) bool {
		_, err := os.Stat(filepath.Join(d, "index.html"))
		return err == nil
	}
	if envDir := os.Getenv("STATIC_DIR"); envDir != "" {
		if abs, err := filepath.Abs(envDir); err == nil && hasIndex(abs) {
			return abs
		}
	}
	if cwd, err := os.Getwd(); err == nil {
		if local := filepath.Join(cwd, "public"); hasIndex(local) {
			return local
		}
		dir := cwd
		for {
			if _, err := os.Stat(filepath.Join(dir, "go.mod")); err == nil {
				root := filepath.Join(dir, "public")
				if hasIndex(root) {
					return root
				}
				break
			}
			parent := filepath.Dir(dir)
			if parent == dir {
				break
			}
			dir = parent
		}
	}
	return ""
}

func runUnderpostStatus(status string) {
	containerID := os.Getenv("CONTAINER_DEPLOY_ID")
	if containerID == "" {
		return
	}
	var value string
	if status == "error" {
		value = "error"
	} else {
		value = containerID + "-" + status
	}
	cmd := exec.Command("underpost", "config", "set", "container-status", value)
	if out, err := cmd.CombinedOutput(); err != nil {
		log.Printf("[status] underpost config set container-status %s: %v\n%s", value, err, out)
	}
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
			runUnderpostStatus("error")
			log.Fatalf("Engine gRPC required: dial %s failed: %v", grpcAddr, err)
		}
		wb := grpcclient.NewWorldBuilder(gc, s)
		wb.InstanceCode = instanceCode
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Minute)
		if err := wb.LoadAll(ctx); err != nil {
			cancel()
			runUnderpostStatus("error")
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

	// Resolve the static asset directory. findPublicDir walks up from
	// CWD so `go run ./cmd/cyberia-server` works regardless of where
	// it's invoked from; the operator can still pin a directory via
	// STATIC_DIR. An empty result is fatal — without a public/index.html
	// the dashboard renders as a blank page, which is the white-screen
	// failure mode this branch exists to prevent.
	staticDir := findPublicDir()
	if staticDir == "" {
		log.Fatalf("Static asset directory not found. Expected ./public with an index.html either in CWD, " +
			"next to go.mod, or pointed at by STATIC_DIR. Run `npm run cyberia:dashboard` from the engine " +
			"repo root to regenerate it.")
	}
	log.Printf("Serving static assets from %s", staticDir)
	r.Handle("/*", game.StaticFileServer(staticDir, "/index.html"))

	// Resolve the dereferenceable base URI for RFC 9457 problem.type
	// values. Default keeps the dev URI; ops override per-deploy via
	// CYBERIA_PROBLEM_BASE_URI (e.g. https://api.cyberiaonline.com/errors).
	if base := os.Getenv("CYBERIA_PROBLEM_BASE_URI"); base != "" {
		problem.SetBaseTypeURI(base)
	}

	// Mount REST API under /api. Options drive CORS allow-list, instance
	// labelling in metric responses, and request timeout.
	r.Mount("/api", api.NewAPIRouter(api.RouterOptions{
		GameServer:   s,
		InstanceCode: instanceCode,
	}))
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

	// Bind the TCP port first so we can signal "running" before entering
	// the accept loop. A failed bind immediately signals "error" and exits.
	ln, listenErr := net.Listen("tcp", ":"+port)
	if listenErr != nil {
		runUnderpostStatus("error")
		log.Fatal("Listen:", listenErr)
	}
	runUnderpostStatus("running-deployment")

	if err := srv.Serve(ln); err != nil && err != http.ErrServerClosed {
		runUnderpostStatus("error")
		log.Fatal("Serve:", err)
	}
}
