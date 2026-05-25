// Package api owns the cyberia-server REST surface under /api/v1.
//
// Contract:
//   - All success responses are application/json; charset=utf-8.
//   - All error responses are application/problem+json (RFC 9457) and
//     constructed via the cyberia-server/api/problem package.
//   - Resource representations are stable: field names use snake_case
//     and are versioned implicitly via the /v1 prefix. Breaking changes
//     require a /v2 carve-out.
//   - Health and load enums are closed sets (see HealthStatus and
//     WorkloadLevel constants below); clients may treat any unknown
//     value as the next-worse known state.
//   - Caching: every endpoint sets Cache-Control: no-store. Metrics are
//     observability data with sub-second relevance.
//
// Endpoints (also documented in openapi.yaml):
//
//	GET /api/v1/health                — liveness, RFC-style
//	GET /api/v1/health/ready          — readiness (gRPC world loaded?)
//	GET /api/v1/metrics               — full metrics snapshot
//	GET /api/v1/metrics/health        — health subset
//	GET /api/v1/metrics/entities      — entity census
//	GET /api/v1/metrics/websocket     — websocket throughput + state
//	GET /api/v1/metrics/workload      — workload / capacity
//	GET /api/v1/metrics/runtime       — simulation cadence + Go runtime
//	GET /api/v1/metrics/maps          — per-map breakdown
//	GET /api/v1/metrics/maps/{code}   — single-map breakdown
package api

import (
	"encoding/json"
	"net/http"
	"runtime"
	"sync"
	"time"

	"cyberia-server/api/problem"
	game "cyberia-server/game"

	"github.com/go-chi/chi/v5"
)

// HealthStatus is the closed set of overall system health values.
// Ordered from best to worst — a client that sees an unknown value
// MAY treat it as the next-worse known state.
type HealthStatus string

const (
	HealthHealthy     HealthStatus = "healthy"
	HealthDegraded    HealthStatus = "degraded"
	HealthCritical    HealthStatus = "critical"
	HealthMaintenance HealthStatus = "maintenance"
)

// WebSocketStatus is the closed set of websocket server states.
type WebSocketStatus string

const (
	WebSocketRunning  WebSocketStatus = "running"
	WebSocketStopping WebSocketStatus = "stopping"
	WebSocketError    WebSocketStatus = "error"
	WebSocketCrashed  WebSocketStatus = "crashed"
)

// WorkloadLevel is the closed set of load buckets. The dashboard maps
// each bucket to a color band; "moderate" is the historical name
// (kept stable to avoid a breaking client change).
type WorkloadLevel string

const (
	WorkloadLow      WorkloadLevel = "low"
	WorkloadModerate WorkloadLevel = "moderate"
	WorkloadHigh     WorkloadLevel = "high"
	WorkloadCritical WorkloadLevel = "critical"
)

// EntityTypeMetrics summarises one entity-type slice of the world census.
type EntityTypeMetrics struct {
	Type                 string `json:"type"`
	Count                int    `json:"count"`
	TotalObjectLayers    int    `json:"total_object_layers"`
	ActiveObjectLayers   int    `json:"active_object_layers"`
	InactiveObjectLayers int    `json:"inactive_object_layers"`
}

// EntityMetrics aggregates the world census plus per-type splits.
type EntityMetrics struct {
	TotalEntities            int                 `json:"total_entities"`
	EntitiesByType           []EntityTypeMetrics `json:"entities_by_type"`
	TotalObjectLayers        int                 `json:"total_object_layers"`
	ActiveObjectLayers       int                 `json:"active_object_layers"`
	InactiveObjectLayers     int                 `json:"inactive_object_layers"`
	AvgObjectLayersPerEntity float64             `json:"avg_object_layers_per_entity"`
}

// WorkloadMetrics is the capacity envelope and current bucket.
type WorkloadMetrics struct {
	LoadPercentage    float64       `json:"load_percentage"`
	MaxEntityCapacity int           `json:"max_entity_capacity"`
	MaxObjectLayers   int           `json:"max_object_layers"`
	CurrentLoad       WorkloadLevel `json:"current_load"`
}

// WebSocketServerMetrics combines connection state with cumulative
// throughput counters (since process start). Rates are not computed
// server-side — the dashboard derives them from successive snapshots.
type WebSocketServerMetrics struct {
	Status             WebSocketStatus `json:"status"`
	ActiveConnections  int             `json:"active_connections"`
	UptimeSec          int64           `json:"uptime_sec"`
	MessagesInTotal    uint64          `json:"messages_in_total"`
	MessagesOutTotal   uint64          `json:"messages_out_total"`
	BytesInTotal       uint64          `json:"bytes_in_total"`
	BytesOutTotal      uint64          `json:"bytes_out_total"`
	ReadErrorsTotal    uint64          `json:"read_errors_total"`
	WriteErrorsTotal   uint64          `json:"write_errors_total"`
	ConnectsTotal      uint64          `json:"connects_total"`
	DisconnectsTotal   uint64          `json:"disconnects_total"`
	LastErrorMessage   string          `json:"last_error_message,omitempty"`
	LastErrorTime      *time.Time      `json:"last_error_time,omitempty"`
}

// RuntimeMetrics combines simulation cadence with Go process telemetry.
type RuntimeMetrics struct {
	game.RuntimeMetricsSnapshot
	GoVersion    string `json:"go_version"`
	NumGoroutine int    `json:"num_goroutine"`
	NumCPU       int    `json:"num_cpu"`
	GoMaxProcs   int    `json:"gomaxprocs"`
	HeapAllocMB  uint64 `json:"heap_alloc_mb"`
	HeapSysMB    uint64 `json:"heap_sys_mb"`
	NumGC        uint32 `json:"num_gc"`
}

// MetricsResponse is the full /metrics envelope.
type MetricsResponse struct {
	Timestamp         time.Time              `json:"timestamp"`
	InstanceCode      string                 `json:"instance_code,omitempty"`
	Health            HealthStatus           `json:"health"`
	HealthDescription string                 `json:"health_description"`
	ServerUptimeSec   int64                  `json:"server_uptime_sec"`
	Entities          EntityMetrics          `json:"entities"`
	WebSocket         WebSocketServerMetrics `json:"websocket"`
	Workload          WorkloadMetrics        `json:"workload"`
	Runtime           RuntimeMetrics         `json:"runtime"`
	Maps              []game.MapBreakdown    `json:"maps"`
}

// MetricsHandler owns the metrics endpoints. It is safe for concurrent
// use; per-request reads are coordinated by h.mu, while heavy world
// reads are delegated to GameServer methods (which take their own lock).
type MetricsHandler struct {
	gameServer *game.GameServer

	mu              sync.RWMutex
	serverStartTime time.Time
	wsStatus        WebSocketStatus
	lastError       string
	lastErrorTime   *time.Time
	instanceCode    string

	// Thresholds. Exposed for tests; not user-tunable yet.
	warningEntityThreshold       int
	criticalEntityThreshold      int
	warningObjectLayerThreshold  int
	criticalObjectLayerThreshold int
	maxEntityCapacity            int
	maxObjectLayers              int
}

// NewMetricsHandler wires a MetricsHandler to a GameServer.
func NewMetricsHandler(gs *game.GameServer, instanceCode string) *MetricsHandler {
	return &MetricsHandler{
		gameServer:                   gs,
		serverStartTime:              time.Now(),
		wsStatus:                     WebSocketRunning,
		instanceCode:                 instanceCode,
		warningEntityThreshold:       800,
		criticalEntityThreshold:      950,
		warningObjectLayerThreshold:  8000,
		criticalObjectLayerThreshold: 9500,
		maxEntityCapacity:            3000,
		maxObjectLayers:              10000,
	}
}

// SetWebSocketStatus lets the process lifecycle (e.g. graceful shutdown)
// flip the reported status. Not currently wired but kept for the
// preStop hook in conf.instances.json to call once we expose it.
func (h *MetricsHandler) SetWebSocketStatus(s WebSocketStatus) {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.wsStatus = s
}

// Routes mounts the metrics endpoints onto an existing chi.Router.
// The /v1 prefix is added by the parent router.
func (h *MetricsHandler) Routes(r chi.Router) {
	r.Get("/metrics", h.GetMetrics)
	r.Get("/metrics/health", h.GetHealth)
	r.Get("/metrics/entities", h.GetEntities)
	r.Get("/metrics/websocket", h.GetWebSocket)
	r.Get("/metrics/workload", h.GetWorkload)
	r.Get("/metrics/runtime", h.GetRuntime)
	r.Get("/metrics/maps", h.GetMaps)
	r.Get("/metrics/maps/{code}", h.GetMapByCode)
}

// ── Handlers ─────────────────────────────────────────────────────────────

// GetMetrics returns the full metrics snapshot.
func (h *MetricsHandler) GetMetrics(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, h.collectMetrics())
}

// GetHealth returns a minimal liveness-style envelope. Suitable for
// dashboards; for Kubernetes probes use /api/v1/health (router-level).
func (h *MetricsHandler) GetHealth(w http.ResponseWriter, r *http.Request) {
	m := h.collectMetrics()
	writeJSON(w, http.StatusOK, map[string]any{
		"timestamp":   m.Timestamp,
		"health":      m.Health,
		"description": m.HealthDescription,
		"uptime_sec":  m.ServerUptimeSec,
	})
}

func (h *MetricsHandler) GetEntities(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, h.collectMetrics().Entities)
}

func (h *MetricsHandler) GetWebSocket(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, map[string]any{
		"timestamp": time.Now().UTC(),
		"websocket": h.collectWebSocketMetrics(),
	})
}

func (h *MetricsHandler) GetWorkload(w http.ResponseWriter, r *http.Request) {
	m := h.collectMetrics()
	writeJSON(w, http.StatusOK, m.Workload)
}

func (h *MetricsHandler) GetRuntime(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, h.collectRuntimeMetrics())
}

func (h *MetricsHandler) GetMaps(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, map[string]any{
		"timestamp": time.Now().UTC(),
		"maps":      h.gameServer.MapBreakdowns(),
	})
}

// GetMapByCode returns the breakdown for a single map. 404 with a
// problem+json envelope when the code is unknown.
func (h *MetricsHandler) GetMapByCode(w http.ResponseWriter, r *http.Request) {
	code := chi.URLParam(r, "code")
	if code == "" {
		problem.Write(w, r, problem.BadRequest("map code is required"))
		return
	}
	b, ok := h.gameServer.MapBreakdownByCode(code)
	if !ok {
		problem.Write(w, r, problem.NotFound("no map loaded with code "+code).
			WithExtension("map_code", code))
		return
	}
	writeJSON(w, http.StatusOK, b)
}

// ── Collectors ───────────────────────────────────────────────────────────

// collectMetrics walks the world once and assembles a full MetricsResponse.
// Lock ordering: h.mu (RLock) → GameServer.* (each method takes its own lock).
func (h *MetricsHandler) collectMetrics() *MetricsResponse {
	h.mu.RLock()
	defer h.mu.RUnlock()

	entityMetrics := h.collectEntityMetrics()
	workloadMetrics := h.calculateWorkloadMetrics(entityMetrics)
	wsMetrics := h.collectWebSocketMetricsLocked()
	runtimeMetrics := h.collectRuntimeMetricsLocked()
	maps := h.gameServer.MapBreakdowns()

	health, desc := h.determineHealth(entityMetrics, workloadMetrics, wsMetrics)

	return &MetricsResponse{
		Timestamp:         time.Now().UTC(),
		InstanceCode:      h.instanceCode,
		Health:            health,
		HealthDescription: desc,
		ServerUptimeSec:   int64(time.Since(h.serverStartTime).Seconds()),
		Entities:          entityMetrics,
		WebSocket:         wsMetrics,
		Workload:          workloadMetrics,
		Runtime:           runtimeMetrics,
		Maps:              maps,
	}
}

// collectWebSocketMetrics is the lock-public wrapper for callers that
// have not already taken h.mu.
func (h *MetricsHandler) collectWebSocketMetrics() WebSocketServerMetrics {
	h.mu.RLock()
	defer h.mu.RUnlock()
	return h.collectWebSocketMetricsLocked()
}

// collectWebSocketMetricsLocked assumes h.mu is already held.
func (h *MetricsHandler) collectWebSocketMetricsLocked() WebSocketServerMetrics {
	rc := h.gameServer.RuntimeMetrics()
	return WebSocketServerMetrics{
		Status:            h.wsStatus,
		ActiveConnections: h.gameServer.GetConnectedClientsCount(),
		UptimeSec:         int64(time.Since(h.serverStartTime).Seconds()),
		MessagesInTotal:   rc.WsMessagesIn,
		MessagesOutTotal:  rc.WsMessagesOut,
		BytesInTotal:      rc.WsBytesIn,
		BytesOutTotal:     rc.WsBytesOut,
		ReadErrorsTotal:   rc.WsReadErrors,
		WriteErrorsTotal:  rc.WsWriteErrors,
		ConnectsTotal:     rc.WsConnectsTotal,
		DisconnectsTotal:  rc.WsDisconnectsTotal,
		LastErrorMessage:  h.lastError,
		LastErrorTime:     h.lastErrorTime,
	}
}

func (h *MetricsHandler) collectRuntimeMetrics() RuntimeMetrics {
	h.mu.RLock()
	defer h.mu.RUnlock()
	return h.collectRuntimeMetricsLocked()
}

func (h *MetricsHandler) collectRuntimeMetricsLocked() RuntimeMetrics {
	var ms runtime.MemStats
	runtime.ReadMemStats(&ms)
	return RuntimeMetrics{
		RuntimeMetricsSnapshot: h.gameServer.RuntimeMetrics(),
		GoVersion:              runtime.Version(),
		NumGoroutine:           runtime.NumGoroutine(),
		NumCPU:                 runtime.NumCPU(),
		GoMaxProcs:             runtime.GOMAXPROCS(0),
		HeapAllocMB:            ms.HeapAlloc / (1024 * 1024),
		HeapSysMB:              ms.HeapSys / (1024 * 1024),
		NumGC:                  ms.NumGC,
	}
}

// entityTypeOrder is the stable render order for entities_by_type.
// Operators expect this list to be schema-consistent across responses
// (every type appears even when count == 0) so the dashboard table
// columns and per-type alerts stay in fixed positions.
var entityTypeOrder = []string{"player", "bot", "floor", "obstacle", "foreground", "portal", "resource"}

func (h *MetricsHandler) collectEntityMetrics() EntityMetrics {
	metrics := EntityMetrics{EntitiesByType: []EntityTypeMetrics{}}
	if h.gameServer == nil {
		return metrics
	}

	// Single locked traversal: avoids the 4 separate world-lock
	// acquisitions the prior implementation used and guarantees that
	// per-type counts here agree with the per-map breakdown returned
	// by /v1/metrics/maps.
	census := h.gameServer.EntityCensus()

	for _, et := range entityTypeOrder {
		c := census[et]
		tm := EntityTypeMetrics{
			Type:                 et,
			Count:                c.Count,
			TotalObjectLayers:    c.TotalObjectLayers,
			ActiveObjectLayers:   c.ActiveObjectLayers,
			InactiveObjectLayers: c.InactiveObjectLayers,
		}
		metrics.EntitiesByType = append(metrics.EntitiesByType, tm)
		metrics.TotalEntities += tm.Count
		metrics.TotalObjectLayers += tm.TotalObjectLayers
		metrics.ActiveObjectLayers += tm.ActiveObjectLayers
		metrics.InactiveObjectLayers += tm.InactiveObjectLayers
	}

	if metrics.TotalEntities > 0 {
		metrics.AvgObjectLayersPerEntity = float64(metrics.TotalObjectLayers) / float64(metrics.TotalEntities)
	}
	return metrics
}

func (h *MetricsHandler) calculateWorkloadMetrics(em EntityMetrics) WorkloadMetrics {
	w := WorkloadMetrics{
		MaxEntityCapacity: h.maxEntityCapacity,
		MaxObjectLayers:   h.maxObjectLayers,
	}
	entityPct := pct(em.TotalEntities, h.maxEntityCapacity)
	layerPct := pct(em.TotalObjectLayers, h.maxObjectLayers)
	w.LoadPercentage = entityPct
	if layerPct > entityPct {
		w.LoadPercentage = layerPct
	}
	switch {
	case w.LoadPercentage >= 90:
		w.CurrentLoad = WorkloadCritical
	case w.LoadPercentage >= 70:
		w.CurrentLoad = WorkloadHigh
	case w.LoadPercentage >= 40:
		w.CurrentLoad = WorkloadModerate
	default:
		w.CurrentLoad = WorkloadLow
	}
	return w
}

func pct(num, denom int) float64 {
	if denom <= 0 {
		return 0
	}
	return float64(num) / float64(denom) * 100
}

func (h *MetricsHandler) determineHealth(em EntityMetrics, wm WorkloadMetrics, ws WebSocketServerMetrics) (HealthStatus, string) {
	switch ws.Status {
	case WebSocketError, WebSocketCrashed:
		return HealthCritical, "WebSocket server is in an error state — unable to accept new connections."
	case WebSocketStopping:
		return HealthMaintenance, "Server is performing a graceful shutdown — new connections are rejected."
	}

	if wm.CurrentLoad == WorkloadCritical {
		return HealthCritical, "Workload is critical (≥ 90 %) — saturation risk is imminent."
	}
	if wm.CurrentLoad == WorkloadHigh {
		if em.TotalEntities >= h.warningEntityThreshold || em.TotalObjectLayers >= h.warningObjectLayerThreshold {
			return HealthDegraded, "Workload is high and entity/layer counts are near threshold — degraded responsiveness expected."
		}
		return HealthDegraded, "Workload is high (70–90 %) — monitor closely."
	}
	if em.TotalEntities >= h.warningEntityThreshold || em.TotalObjectLayers >= h.warningObjectLayerThreshold {
		return HealthDegraded, "Entity or object-layer count approaching configured threshold."
	}
	return HealthHealthy, "All systems operational."
}

// ── Response helpers ─────────────────────────────────────────────────────

// writeJSON serializes v as JSON with charset=utf-8 and a no-store cache
// directive. Errors from the encoder are intentionally not surfaced —
// the response is already committed by the time WriteHeader fires.
func writeJSON(w http.ResponseWriter, status int, v any) {
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	w.Header().Set("Cache-Control", "no-store")
	w.Header().Set("X-Content-Type-Options", "nosniff")
	w.WriteHeader(status)
	_ = json.NewEncoder(w).Encode(v)
}
