package main

import (
	"context"
	"log"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"time"

	api "cyberia-server/api"
	"cyberia-server/api/problem"
	"cyberia-server/config"
	game "cyberia-server/game"
	"cyberia-server/grpcclient"

	"github.com/go-chi/chi/v5"
	"github.com/joho/godotenv"
)

func runUnderpostStatus(containerID, status string) {
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
	// Load .env from CWD (project root) if present. Does not override
	// already-set env vars. Absence is fine — env vars may be set directly.
	if err := godotenv.Load("../../.env"); err != nil {
		log.Println("No .env file; relying on environment variables.")
	}

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

	// Panic if the static dir lacks index.html — serving a dir without one
	// renders the dashboard as a blank page (the white-screen failure mode).
	// Resolve static dir relative to the binary at cmd/cyberia-server/.
	staticDir := cfg.StaticDir
	if !filepath.IsAbs(staticDir) {
		staticDir = filepath.Clean(filepath.Join("..", "..", staticDir))
	}
	if _, err := os.Stat(filepath.Join(staticDir, "index.html")); err != nil {
		panic("no " + filepath.Join(staticDir, "index.html") + " — " + err.Error())
	}
	log.Printf("Serving static assets from %s", staticDir)
	r.Handle("/*", game.StaticFileServer(staticDir, "/index.html"))

	// Override the RFC 9457 problem.type base URI when set; otherwise the
	// package keeps its dev default.
	if cfg.ProblemBaseURI != "" {
		problem.SetBaseTypeURI(cfg.ProblemBaseURI)
	}

	// Mount REST API under /api. Options drive CORS allow-list, instance
	// labelling in metric responses, and request timeout.
	r.Mount("/api", api.NewAPIRouter(api.RouterOptions{
		GameServer:     s,
		InstanceCode:   cfg.InstanceCode,
		AllowedOrigins: cfg.CORSAllowedOrigins,
	}))
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
