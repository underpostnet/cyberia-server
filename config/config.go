// Package config centralizes all environment-variable configuration for
// cyberia-server. Load reads the environment once at startup, applies
// defaults, and fails fast on missing required or malformed values so the
// rest of the program never touches os.Getenv directly.
package config

import (
	"fmt"
	"os"
	"strconv"
	"strings"
	"time"
)

// Config holds all runtime configuration sourced from environment variables.
// One field per variable; defaults are applied in Load.
type Config struct {
	// ServerPort is the TCP port the HTTP/WebSocket server binds.
	// SERVER_PORT, default "8081".
	ServerPort string

	// StaticDir is the static asset directory served at "/".
	// STATIC_DIR, default "public".
	StaticDir string

	// InstanceCode selects the Engine instance to load. INSTANCE_CODE, required.
	InstanceCode string

	// EngineGRPCAddress is the Engine gRPC endpoint (host:port).
	// ENGINE_GRPC_ADDRESS, default "localhost:50051".
	EngineGRPCAddress string

	// EngineAPIBaseURL is forwarded to clients for binary blob fetches.
	// ENGINE_API_BASE_URL, optional (empty = unset).
	EngineAPIBaseURL string

	// GRPCReloadInterval enables Engine hot-reload polling.
	// ENGINE_GRPC_RELOAD_INTERVAL_SEC, 0 = disabled.
	GRPCReloadInterval time.Duration

	// CORSAllowedOrigins is the CORS allow-list.
	// CYBERIA_CORS_ALLOWED_ORIGINS (comma-separated), dev default when unset.
	CORSAllowedOrigins []string

	// ProblemBaseURI overrides the RFC 9457 problem.type base URI.
	// CYBERIA_PROBLEM_BASE_URI, optional (empty = keep package default).
	ProblemBaseURI string

	// ContainerDeployID labels underpost container-status reports.
	// CONTAINER_DEPLOY_ID, optional (empty = status reporting disabled).
	ContainerDeployID string
}

// defaultCORSOrigins is the dev-friendly allow-list used when
// CYBERIA_CORS_ALLOWED_ORIGINS is unset.
var defaultCORSOrigins = []string{"http://localhost:*", "https://*.cyberiaonline.com"}

// DefaultCORSOrigins returns a copy of the dev-friendly CORS allow-list used
// when CYBERIA_CORS_ALLOWED_ORIGINS is unset. Exposed so the api package can
// share the same default without re-declaring it.
func DefaultCORSOrigins() []string {
	out := make([]string, len(defaultCORSOrigins))
	copy(out, defaultCORSOrigins)
	return out
}

// Load reads configuration from the environment, applies defaults, and
// returns an error when a required variable is missing or malformed. The
// returned Config is populated even on error (notably ContainerDeployID) so
// callers can still report deploy status before exiting.
func Load() (Config, error) {
	c := Config{
		ServerPort:         getEnv("SERVER_PORT", "8081"),
		StaticDir:          getEnv("STATIC_DIR", "public"),
		InstanceCode:       os.Getenv("INSTANCE_CODE"),
		EngineGRPCAddress:  getEnv("ENGINE_GRPC_ADDRESS", "localhost:50051"),
		EngineAPIBaseURL:   os.Getenv("ENGINE_API_BASE_URL"),
		ProblemBaseURI:     os.Getenv("CYBERIA_PROBLEM_BASE_URI"),
		ContainerDeployID:  os.Getenv("CONTAINER_DEPLOY_ID"),
		CORSAllowedOrigins: parseCSV(os.Getenv("CYBERIA_CORS_ALLOWED_ORIGINS"), defaultCORSOrigins),
	}

	if c.InstanceCode == "" {
		return c, fmt.Errorf("INSTANCE_CODE required")
	}

	reload, err := parseReloadInterval(os.Getenv("ENGINE_GRPC_RELOAD_INTERVAL_SEC"))
	if err != nil {
		return c, err
	}
	c.GRPCReloadInterval = reload

	return c, nil
}

// getEnv returns the value of key, or def when key is unset or empty.
func getEnv(key, def string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return def
}

// parseCSV splits a comma-separated value into trimmed, non-empty parts,
// returning def when the result is empty.
func parseCSV(raw string, def []string) []string {
	if raw == "" {
		return def
	}
	parts := strings.Split(raw, ",")
	out := make([]string, 0, len(parts))
	for _, p := range parts {
		if p = strings.TrimSpace(p); p != "" {
			out = append(out, p)
		}
	}
	if len(out) == 0 {
		return def
	}
	return out
}

// parseReloadInterval converts ENGINE_GRPC_RELOAD_INTERVAL_SEC to a Duration.
// Empty means disabled (0). A non-numeric or negative value is a fatal
// misconfiguration rather than a silently-ignored default.
func parseReloadInterval(raw string) (time.Duration, error) {
	if raw == "" {
		return 0, nil
	}
	sec, err := strconv.Atoi(raw)
	if err != nil {
		return 0, fmt.Errorf("ENGINE_GRPC_RELOAD_INTERVAL_SEC %q: %w", raw, err)
	}
	if sec < 0 {
		return 0, fmt.Errorf("ENGINE_GRPC_RELOAD_INTERVAL_SEC must be >= 0, got %d", sec)
	}
	return time.Duration(sec) * time.Second, nil
}
