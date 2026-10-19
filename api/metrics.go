// Package api serves operational metrics under /api/v1.
package api

import (
	"encoding/json"
	"net/http"
	"runtime"
	"time"

	"cyberia-server/game/net"
	"cyberia-server/httpserver/problem"

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

// WorkloadLevel reports the share of the tick budget used by simulation and replication.
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
	LoadPercentage float64 `json:"load_percentage"`
	// Zero means no configured entity or object-layer limit.
	MaxEntityCapacity int           `json:"max_entity_capacity"`
	MaxObjectLayers   int           `json:"max_object_layers"`
	CurrentLoad       WorkloadLevel `json:"current_load"`
}

// WebSocketServerMetrics combines connection state with cumulative
// throughput counters (since process start). Rates are not computed
// server-side — the dashboard derives them from successive snapshots.
type WebSocketServerMetrics struct {
	Status            WebSocketStatus `json:"status"`
	ActiveConnections int             `json:"active_connections"`
	MaxConnections    int             `json:"max_connections"`
	UptimeSec         int64           `json:"uptime_sec"`
	MessagesInTotal   uint64          `json:"messages_in_total"`
	MessagesOutTotal  uint64          `json:"messages_out_total"`
	BytesInTotal      uint64          `json:"bytes_in_total"`
	BytesOutTotal     uint64          `json:"bytes_out_total"`
	ReadErrorsTotal   uint64          `json:"read_errors_total"`
	WriteErrorsTotal  uint64          `json:"write_errors_total"`
	ConnectsTotal     uint64          `json:"connects_total"`
	DisconnectsTotal  uint64          `json:"disconnects_total"`
	LastErrorMessage  string          `json:"last_error_message,omitempty"`
	LastErrorTime     *time.Time      `json:"last_error_time,omitempty"`
}

// RuntimeMetrics combines simulation cadence with Go process telemetry.
type RuntimeMetrics struct {
	net.RuntimeMetricsSnapshot
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
	Maps              []net.MapBreakdown     `json:"maps"`
	// SessionMode is "account" when players sign in and their state persists.
	SessionMode net.SessionMode `json:"session_mode"`
	// Persistence is the persist pipeline of an account server.
	Persistence *PersistenceMetrics `json:"persistence,omitempty"`
}

// PersistenceMetrics is the persist pipeline: commits, retries, the queue and
// the checkpoints.
type PersistenceMetrics struct {
	Sessions             int     `json:"sessions"`
	PendingStates        int     `json:"pending_states"`
	OldestPendingSec     float64 `json:"oldest_pending_sec"`
	UnsavedPlayers       int     `json:"unsaved_players"`
	CommitsTotal         uint64  `json:"commits_total"`
	DuplicatesTotal      uint64  `json:"duplicates_total"`
	RetriesTotal         uint64  `json:"retries_total"`
	StaleTotal           uint64  `json:"stale_total"`
	InvalidTotal         uint64  `json:"invalid_total"`
	LostFinalTotal       uint64  `json:"lost_final_total"`
	AvgCommitMs          float64 `json:"avg_commit_ms"`
	CheckpointsTotal     uint64  `json:"checkpoints_total"`
	CheckpointFailures   uint64  `json:"checkpoint_failures_total"`
	PositionsTotal       uint64  `json:"positions_total"`
	LastCheckpointAgeSec float64 `json:"last_checkpoint_age_sec"`
}

func (h *MetricsHandler) collectPersistence() *PersistenceMetrics {
	snap, ok := h.gameServer.Persistence()
	if !ok {
		return nil
	}
	out := &PersistenceMetrics{
		Sessions:           snap.Sessions,
		PendingStates:      snap.Pending,
		OldestPendingSec:   snap.OldestPending.Seconds(),
		UnsavedPlayers:     snap.Unsaved,
		CommitsTotal:       snap.Commits,
		DuplicatesTotal:    snap.Duplicates,
		RetriesTotal:       snap.Retries,
		StaleTotal:         snap.Stale,
		InvalidTotal:       snap.Invalid,
		LostFinalTotal:     snap.LostFinal,
		AvgCommitMs:        float64(snap.AvgCommitDuration.Microseconds()) / 1000,
		CheckpointsTotal:   snap.Checkpoints,
		CheckpointFailures: snap.CheckpointFailures,
		PositionsTotal:     snap.Positions,
	}
	if !snap.LastCheckpoint.IsZero() {
		out.LastCheckpointAgeSec = time.Since(snap.LastCheckpoint).Seconds()
	}
	return out
}

// MetricsHandler reads metrics from the game server.
type MetricsHandler struct {
	gameServer *net.GameServer

	serverStartTime time.Time
	instanceCode    string
}

// NewMetricsHandler wires a MetricsHandler to a GameServer.
func NewMetricsHandler(gs *net.GameServer, instanceCode string) *MetricsHandler {
	return &MetricsHandler{
		gameServer:      gs,
		serverStartTime: time.Now(),
		instanceCode:    instanceCode,
	}
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
		"websocket": h.collectWebSocketMetrics(h.gameServer.RuntimeMetrics()),
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

func (h *MetricsHandler) collectMetrics() *MetricsResponse {
	runtimeMetrics := h.collectRuntimeMetrics()
	entityMetrics := h.collectEntityMetrics()
	wsMetrics := h.collectWebSocketMetrics(runtimeMetrics.RuntimeMetricsSnapshot)
	workloadMetrics := calculateWorkloadMetrics(runtimeMetrics)
	maps := h.gameServer.MapBreakdowns()

	health, desc := h.determineHealth(workloadMetrics, wsMetrics)

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
		SessionMode:       h.gameServer.Mode(),
		Persistence:       h.collectPersistence(),
	}
}

func (h *MetricsHandler) collectWebSocketMetrics(rc net.RuntimeMetricsSnapshot) WebSocketServerMetrics {
	status := WebSocketRunning
	if h.gameServer.IsDraining() {
		status = WebSocketStopping
	}
	return WebSocketServerMetrics{
		Status:            status,
		ActiveConnections: rc.WsActiveConns,
		MaxConnections:    rc.MaxConnections,
		UptimeSec:         int64(time.Since(h.serverStartTime).Seconds()),
		MessagesInTotal:   rc.WsMessagesIn,
		MessagesOutTotal:  rc.WsMessagesOut,
		BytesInTotal:      rc.WsBytesIn,
		BytesOutTotal:     rc.WsBytesOut,
		ReadErrorsTotal:   rc.WsReadErrors,
		WriteErrorsTotal:  rc.WsWriteErrors,
		ConnectsTotal:     rc.WsConnectsTotal,
		DisconnectsTotal:  rc.WsDisconnectsTotal,
	}
}

func (h *MetricsHandler) collectRuntimeMetrics() RuntimeMetrics {
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

// Every entity type appears in this order, including empty types.
var entityTypeOrder = []string{"player", "bot", "floor", "obstacle", "foreground", "portal", "resource"}

func (h *MetricsHandler) collectEntityMetrics() EntityMetrics {
	metrics := EntityMetrics{EntitiesByType: []EntityTypeMetrics{}}
	if h.gameServer == nil {
		return metrics
	}

	// One traversal gives consistent counts for all entity types.
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

func calculateWorkloadMetrics(rm RuntimeMetrics) WorkloadMetrics {
	w := WorkloadMetrics{}
	if rm.TickDurationMs > 0 {
		w.LoadPercentage = rm.TickWorkMs / rm.TickDurationMs * 100
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

func (h *MetricsHandler) determineHealth(wm WorkloadMetrics, ws WebSocketServerMetrics) (HealthStatus, string) {
	if !h.gameServer.HasMaps() || h.gameServer.IsDraining() || ws.Status == WebSocketStopping {
		return HealthMaintenance, "The world is loading or the server is draining."
	}
	if ws.Status == WebSocketError || ws.Status == WebSocketCrashed {
		return HealthCritical, "The WebSocket server cannot accept connections."
	}
	switch wm.CurrentLoad {
	case WorkloadCritical:
		return HealthCritical, "Tick work uses at least 90% of the tick budget."
	case WorkloadHigh:
		return HealthDegraded, "Tick work uses at least 70% of the tick budget."
	default:
		return HealthHealthy, "All systems operational."
	}
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
