// Package config centralizes all environment-variable configuration for
// cyberia-server. Load reads the environment once at startup, applies
// defaults, and fails fast on missing required or malformed values so the
// rest of the program never touches os.Getenv directly.
package config

import (
	"fmt"
	"os"
	"regexp"
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

	// BasePath is the public path this instance consumes. The complete HTTP,
	// REST, static, and WebSocket router is mounted here. CYBERIA_BASE_PATH,
	// default "/".
	BasePath string

	// InstanceCode selects the Engine instance to load. INSTANCE_CODE, required.
	InstanceCode string

	// EngineGRPCAddress is the Engine gRPC endpoint (host:port).
	// ENGINE_GRPC_ADDRESS, default "localhost:50051".
	EngineGRPCAddress string

	// EngineAPIBaseURL is the internal engine-cyberia origin for
	// server-to-server calls (cyberia-server -> engine-cyberia). Infrastructure
	// only; never forwarded to clients. ENGINE_API_BASE_URL, optional.
	EngineAPIBaseURL string

	// EnginePublicURL is the client-visible engine-cyberia (Content Authority)
	// origin forwarded to clients for every content/asset/metadata request.
	// Distinct from EngineAPIBaseURL. ENGINE_PUBLIC_URL, optional (empty = unset).
	EnginePublicURL string

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

	// WSLimits overrides the WebSocket admission and rate thresholds. Every
	// field is nil when its variable is unset, which keeps the built-in
	// default. See WSLimits for the variable names.
	WSLimits WSLimits

	// ServerAPIKey is the INTERNAL shared secret engine-cyberia presents to
	// trigger a hot reload (gRPC control service or REST fallback). Never
	// exposed to game clients. CYBERIA_SERVER_API_KEY; unset disables the
	// trigger entirely (fail closed).
	ServerAPIKey string

	// HotReloadGRPCAddress is the listen address of the hot-reload control
	// gRPC service. CYBERIA_HOT_RELOAD_GRPC_ADDRESS, default ":50052".
	HotReloadGRPCAddress string
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
	basePath, err := normalizeBasePath(os.Getenv("CYBERIA_BASE_PATH"))
	c := Config{
		ServerPort:           getEnv("SERVER_PORT", "8081"),
		StaticDir:            getEnv("STATIC_DIR", "../../public"),
		BasePath:             basePath,
		InstanceCode:         os.Getenv("INSTANCE_CODE"),
		EngineGRPCAddress:    getEnv("ENGINE_GRPC_ADDRESS", "localhost:50051"),
		EngineAPIBaseURL:     os.Getenv("ENGINE_API_BASE_URL"),
		EnginePublicURL:      os.Getenv("ENGINE_PUBLIC_URL"),
		ProblemBaseURI:       os.Getenv("CYBERIA_PROBLEM_BASE_URI"),
		ContainerDeployID:    os.Getenv("CONTAINER_DEPLOY_ID"),
		ServerAPIKey:         os.Getenv("CYBERIA_SERVER_API_KEY"),
		HotReloadGRPCAddress: getEnv("CYBERIA_HOT_RELOAD_GRPC_ADDRESS", ":50052"),
		CORSAllowedOrigins:   parseCSV(os.Getenv("CYBERIA_CORS_ALLOWED_ORIGINS"), defaultCORSOrigins),
	}
	if err != nil {
		return c, err
	}

	if c.InstanceCode == "" {
		return c, fmt.Errorf("INSTANCE_CODE required")
	}

	reload, err := parseReloadInterval(os.Getenv("ENGINE_GRPC_RELOAD_INTERVAL_SEC"))
	if err != nil {
		return c, err
	}
	c.GRPCReloadInterval = reload

	limits, err := loadWSLimits()
	if err != nil {
		return c, err
	}
	c.WSLimits = limits

	return c, nil
}

// WSLimits carries the WebSocket admission and rate overrides. A nil field
// means the variable is unset and the built-in default applies. An explicit 0
// disables that one limit.
//
// DisableAll is the single switch for a load test: it turns every limit off,
// which is required when many simulated clients share one address.
type WSLimits struct {
	// CYBERIA_DISABLE_CONNECTION_LIMITS — "1"/"true" turns every limit off.
	DisableAll bool

	MaxConnections      *int // CYBERIA_MAX_CONNECTIONS
	MaxConnectionsPerIP *int // CYBERIA_MAX_CONNECTIONS_PER_IP
	ConnectRatePerIP    *int // CYBERIA_CONNECT_RATE_PER_IP
	ConnectBurstPerIP   *int // CYBERIA_CONNECT_BURST_PER_IP
	MessageRate         *int // CYBERIA_MESSAGE_RATE
	MessageBurst        *int // CYBERIA_MESSAGE_BURST
	MaxStrikes          *int // CYBERIA_MAX_STRIKES
}

func loadWSLimits() (WSLimits, error) {
	out := WSLimits{DisableAll: parseBool(os.Getenv("CYBERIA_DISABLE_CONNECTION_LIMITS"))}
	for _, f := range []struct {
		key  string
		dest **int
	}{
		{"CYBERIA_MAX_CONNECTIONS", &out.MaxConnections},
		{"CYBERIA_MAX_CONNECTIONS_PER_IP", &out.MaxConnectionsPerIP},
		{"CYBERIA_CONNECT_RATE_PER_IP", &out.ConnectRatePerIP},
		{"CYBERIA_CONNECT_BURST_PER_IP", &out.ConnectBurstPerIP},
		{"CYBERIA_MESSAGE_RATE", &out.MessageRate},
		{"CYBERIA_MESSAGE_BURST", &out.MessageBurst},
		{"CYBERIA_MAX_STRIKES", &out.MaxStrikes},
	} {
		v, err := getEnvIntPtr(f.key)
		if err != nil {
			return out, err
		}
		*f.dest = v
	}
	return out, nil
}

// getEnvIntPtr reads an optional non-negative integer. Unset returns nil. A
// malformed value is an error rather than a silent fallback, so a typo in a
// limit never ships as a surprise default.
func getEnvIntPtr(key string) (*int, error) {
	raw := strings.TrimSpace(os.Getenv(key))
	if raw == "" {
		return nil, nil
	}
	n, err := strconv.Atoi(raw)
	if err != nil || n < 0 {
		return nil, fmt.Errorf("invalid %s %q: expected a non-negative integer", key, raw)
	}
	return &n, nil
}

func parseBool(raw string) bool {
	switch strings.ToLower(strings.TrimSpace(raw)) {
	case "1", "true", "yes", "on":
		return true
	}
	return false
}

var basePathPattern = regexp.MustCompile(`^/[A-Za-z0-9][A-Za-z0-9._-]*$`)

// normalizeBasePath accepts the same single-segment paths as multiInstance.
// Empty and root select the ordinary root router; a trailing slash is ignored.
func normalizeBasePath(raw string) (string, error) {
	path := strings.TrimSpace(raw)
	if path == "" || path == "/" {
		return "/", nil
	}
	if !strings.HasPrefix(path, "/") {
		path = "/" + path
	}
	path = strings.TrimRight(path, "/")
	if !basePathPattern.MatchString(path) {
		return path, fmt.Errorf("invalid CYBERIA_BASE_PATH %q: expected / or one URL path segment", raw)
	}
	return path, nil
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
