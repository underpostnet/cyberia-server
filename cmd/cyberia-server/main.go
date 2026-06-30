package main

import (
	"context"
	"log"
	"net"
	"net/http"
	"os/exec"
	"path/filepath"
	"runtime/debug"
	"time"

	api "cyberia-server/api"
	"cyberia-server/config"
	game "cyberia-server/game"
	"cyberia-server/grpcclient"
	"cyberia-server/httpserver"
	"cyberia-server/httpserver/problem"
	"cyberia-server/logx"

	"github.com/go-chi/chi/v5"
	"github.com/joho/godotenv"
)

func underpostConfigSet(key, value string) {
	cmd := exec.Command("underpost", "config", "set", key, value)
	if out, err := cmd.CombinedOutput(); err != nil {
		log.Printf("[status] underpost config set %s %s: %v\n%s", key, value, err, out)
	}
}

// runUnderpostStatus publishes a lifecycle transition.
//   - container-status is the dynamic runtime-status, set on every transition
//     (success or error) so the deploy monitor can detect failure.
//   - start-container-status is the insulated readiness marker, set only once the
//     server is running, so a later fault can never clear pod readiness.
func runUnderpostStatus(containerID, status string) {
	if containerID == "" {
		return
	}
	value := "error"
	if status != "error" {
		value = containerID + "-" + status
	}
	underpostConfigSet("container-status", value)
	if status == "running-deployment" {
		underpostConfigSet("start-container-status", value)
	}
}

func main() {
	// Load .env from CWD (project root) if present. Does not override
	// already-set env vars. Absence is fine — env vars may be set directly.
	if err := godotenv.Load("../../.env"); err != nil {
		log.Println("No .env file; relying on environment variables.")
	}

	// Centralized leveled JSON logging — resolves LOG_LEVEL / APP_ENV. Must run
	// before any goroutine logs so the level is in effect everywhere.
	logx.Init()

	// The Go runtime parses GOMEMLIMIT from the environment at startup; set it
	// in the K8s manifest to ~90% of the pod memory limit so the GC runs
	// aggressively before the cgroup OOM-kills the pod. SetMemoryLimit(-1) reads
	// the active soft limit without changing it, for ops visibility.
	logx.Infof("GC soft memory limit (GOMEMLIMIT) = %d bytes (math.MaxInt64 ⇒ unset)", debug.SetMemoryLimit(-1))

	// All environment configuration is resolved here, once.
	cfg, err := config.Load()
	if err != nil {
		runUnderpostStatus(cfg.ContainerDeployID, "error")
		log.Fatalf("config: %v", err)
	}

	// Core game server
	s := game.NewGameServer()
	if cfg.EngineAPIBaseURL != "" {
		// Forwarded to clients for binary blob fetches.
		s.SetEngineApiBaseUrl(cfg.EngineAPIBaseURL)
	}

	// ── Data loading: gRPC ──────────
	// The server exits if the Engine is unreachable — no fallback world is generated.
	gc, err := grpcclient.New(cfg.EngineGRPCAddress)
	if err != nil {
		runUnderpostStatus(cfg.ContainerDeployID, "error")
		log.Fatalf("Engine gRPC required: dial %s failed: %v", cfg.EngineGRPCAddress, err)
	}
	defer gc.Close()

	wb := grpcclient.NewWorldBuilder(gc, s)
	wb.InstanceCode = cfg.InstanceCode
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Minute)
	if err := wb.LoadAll(ctx); err != nil {
		cancel()
		runUnderpostStatus(cfg.ContainerDeployID, "error")
		log.Fatalf("Engine gRPC world load failed: %v — Engine must be running before starting cyberia-server.", err)
	}
	cancel()

	// Start hot-reload loop if configured (0 = disabled).
	if cfg.GRPCReloadInterval > 0 {
		wb.ReloadInterval = cfg.GRPCReloadInterval
		wb.StartReloadLoop()
		defer wb.Stop()
	}

	go s.Run()

	r := chi.NewRouter()

	// Resolve static dir relative to the binary at cmd/cyberia-server/.
	// StaticFileServer fatals if the dir or its index.html is missing.
	staticDir := cfg.StaticDir
	if !filepath.IsAbs(staticDir) {
		staticDir = filepath.Clean(filepath.Join("..", "..", staticDir))
	}
	log.Printf("Serving static assets from %s", staticDir)
	r.Handle("/*", httpserver.StaticFileServer(staticDir, "/index.html"))

	// Override the RFC 9457 problem.type base URI when set; otherwise the
	// package keeps its dev default.
	if cfg.ProblemBaseURI != "" {
		problem.SetBaseTypeURI(cfg.ProblemBaseURI)
	}

	// Mount REST API under /api. Options drive CORS allow-list, instance
	// labelling in metric responses, and request timeout.
	api.Mount(r, api.RouterOptions{
		GameServer:     s,
		InstanceCode:   cfg.InstanceCode,
		AllowedOrigins: cfg.CORSAllowedOrigins,
	})
	// Keep websocket endpoint
	r.HandleFunc("/ws", s.HandleConnections)

	srv := &http.Server{
		Addr:    ":" + cfg.ServerPort,
		Handler: r,
	}

	log.Printf("Server started on :%s", cfg.ServerPort)

	// Bind the TCP port first so we can signal "running" before entering
	// the accept loop. A failed bind immediately signals "error" and exits.
	ln, listenErr := net.Listen("tcp", ":"+cfg.ServerPort)
	if listenErr != nil {
		runUnderpostStatus(cfg.ContainerDeployID, "error")
		log.Fatal("Listen:", listenErr)
	}
	runUnderpostStatus(cfg.ContainerDeployID, "running-deployment")

	if err := srv.Serve(ln); err != nil && err != http.ErrServerClosed {
		runUnderpostStatus(cfg.ContainerDeployID, "error")
		log.Fatal("Serve:", err)
	}
}
