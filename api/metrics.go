package api

import (
	"encoding/json"
	"fmt"
	"net/http"
	"sync"
	"time"

	game "cyberia-server/src"

	"github.com/go-chi/chi/v5"
)

// HealthStatus represents the overall health of the system
type HealthStatus string

const (
	HealthHealthy     HealthStatus = "healthy"
	HealthOk          HealthStatus = "ok"
	HealthWarning     HealthStatus = "warning"
	HealthDegraded    HealthStatus = "degraded"
	HealthCritical    HealthStatus = "critical"
	HealthDown        HealthStatus = "down"
	HealthMaintenance HealthStatus = "maintenance"
)

// WebSocketStatus represents the state of the WebSocket server
type WebSocketStatus string

const (
	WebSocketRunning  WebSocketStatus = "running"
	WebSocketStopping WebSocketStatus = "stopping"
	WebSocketError    WebSocketStatus = "error"
	WebSocketCrashed  WebSocketStatus = "crashed"
)

// EntityTypeMetrics holds metrics for a specific entity type
type EntityTypeMetrics struct {
	Type                 string `json:"type"`
	Count                int    `json:"count"`
	TotalObjectLayers    int    `json:"total_object_layers"`
	ActiveObjectLayers   int    `json:"active_object_layers"`
	InactiveObjectLayers int    `json:"inactive_object_layers"`
}

// EntityMetrics holds all entity-related metrics
type EntityMetrics struct {
	TotalEntities            int                 `json:"total_entities"`
	EntitiesByType           []EntityTypeMetrics `json:"entities_by_type"`
	TotalObjectLayers        int                 `json:"total_object_layers"`
	ActiveObjectLayers       int                 `json:"active_object_layers"`
	InactiveObjectLayers     int                 `json:"inactive_object_layers"`
	AvgObjectLayersPerEntity float64             `json:"avg_object_layers_per_entity"`
}

// WorkloadMetrics tracks the current system workload
type WorkloadMetrics struct {
	LoadPercentage    float64 `json:"load_percentage"`
	MaxEntityCapacity int     `json:"max_entity_capacity"`
	MaxObjectLayers   int     `json:"max_object_layers"`
	CurrentLoad       string  `json:"current_load"` // "low", "medium", "high", "critical"
}

// WebSocketServerMetrics holds WebSocket server status
type WebSocketServerMetrics struct {
	Status            WebSocketStatus `json:"status"`
	ActiveConnections int             `json:"active_connections"`
	UptimeSec         int64           `json:"uptime_sec"`
	LastErrorMessage  string          `json:"last_error_message,omitempty"`
	LastErrorTime     *time.Time      `json:"last_error_time,omitempty"`
}

// MetricsResponse is the complete metrics response structure
type MetricsResponse struct {
	Timestamp         time.Time              `json:"timestamp"`
	Health            HealthStatus           `json:"health"`
	HealthDescription string                 `json:"health_description"`
	Entities          EntityMetrics          `json:"entities"`
	WebSocket         WebSocketServerMetrics `json:"websocket"`
	Workload          WorkloadMetrics        `json:"workload"`
	ServerUptime      int64                  `json:"server_uptime_sec"`
}

// MetricsHandler manages metrics collection and reporting
type MetricsHandler struct {
	cfg              Config
	db               *DB
	gameServer       *game.GameServer
	mu               sync.RWMutex
	serverStartTime  time.Time
	webSocketMetrics WebSocketServerMetrics
	lastMetricsTime  time.Time

	// Thresholds for health status
	warningEntityThreshold       int
	criticalEntityThreshold      int
	warningObjectLayerThreshold  int
	criticalObjectLayerThreshold int
}

// NewMetricsHandler creates a new metrics handler
func NewMetricsHandler(cfg Config, db *DB, gameServer *game.GameServer) *MetricsHandler {
	now := time.Now()
	return &MetricsHandler{
		cfg:                          cfg,
		db:                           db,
		gameServer:                   gameServer,
		serverStartTime:              now,
		warningEntityThreshold:       800,  // Warning at 80% capacity
		criticalEntityThreshold:      950,  // Critical at 95% capacity
		warningObjectLayerThreshold:  8000, // Warning at 80% capacity
		criticalObjectLayerThreshold: 9500, // Critical at 95% capacity
		lastMetricsTime:              now,
		webSocketMetrics: WebSocketServerMetrics{
			Status:            WebSocketRunning,
			ActiveConnections: 0,
			UptimeSec:         0,
		},
	}
}

// Routes registers metrics routes
func (h *MetricsHandler) Routes(r chi.Router) {
	r.Get("/metrics", h.GetMetrics)
	r.Get("/metrics/health", h.GetHealth)
	r.Get("/metrics/entities", h.GetEntities)
	r.Get("/metrics/websocket", h.GetWebSocket)
	r.Get("/metrics/workload", h.GetWorkload)
}

// GetMetrics returns complete metrics
func (h *MetricsHandler) GetMetrics(w http.ResponseWriter, r *http.Request) {
	metrics := h.collectMetrics()

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(metrics)
}

// GetHealth returns only health status
func (h *MetricsHandler) GetHealth(w http.ResponseWriter, r *http.Request) {
	metrics := h.collectMetrics()
	response := map[string]interface{}{
		"timestamp":   metrics.Timestamp,
		"health":      metrics.Health,
		"description": metrics.HealthDescription,
		"uptime_sec":  metrics.ServerUptime,
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(response)
}

// GetEntities returns only entity metrics
func (h *MetricsHandler) GetEntities(w http.ResponseWriter, r *http.Request) {
	metrics := h.collectMetrics()

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(metrics.Entities)
}

// GetWebSocket returns only WebSocket metrics
func (h *MetricsHandler) GetWebSocket(w http.ResponseWriter, r *http.Request) {
	h.mu.RLock()
	defer h.mu.RUnlock()

	// Sync WebSocket metrics from game server
	wsMetrics := h.syncWebSocketMetricsFromGameServer()

	response := map[string]interface{}{
		"timestamp": time.Now(),
		"websocket": wsMetrics,
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(response)
}

// GetWorkload returns only workload metrics
func (h *MetricsHandler) GetWorkload(w http.ResponseWriter, r *http.Request) {
	metrics := h.collectMetrics()

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(metrics.Workload)
}

// collectMetrics gathers all metrics from the system
func (h *MetricsHandler) collectMetrics() *MetricsResponse {
	h.mu.RLock()
	defer h.mu.RUnlock()

	entityMetrics := h.collectEntityMetrics()
	workloadMetrics := h.calculateWorkloadMetrics(entityMetrics)

	// Sync all WebSocket metrics from game server
	wsMetrics := h.syncWebSocketMetricsFromGameServer()

	health, healthDesc := h.determineHealth(entityMetrics, workloadMetrics, wsMetrics)

	return &MetricsResponse{
		Timestamp:         time.Now(),
		Health:            health,
		HealthDescription: healthDesc,
		Entities:          entityMetrics,
		WebSocket:         wsMetrics,
		Workload:          workloadMetrics,
		ServerUptime:      int64(time.Since(h.serverStartTime).Seconds()),
	}
}

// syncWebSocketMetricsFromGameServer syncs all WebSocket metrics from the game server
func (h *MetricsHandler) syncWebSocketMetricsFromGameServer() WebSocketServerMetrics {
	wsMetrics := h.webSocketMetrics
	wsMetrics.UptimeSec = int64(time.Since(h.serverStartTime).Seconds())

	// Sync active connections from game server
	if h.gameServer != nil {
		wsMetrics.ActiveConnections = h.gameServer.GetConnectedClientsCount()
	}

	return wsMetrics
}

// collectEntityMetrics gathers entity and object layer metrics from actual entity states
func (h *MetricsHandler) collectEntityMetrics() EntityMetrics {
	metrics := EntityMetrics{
		EntitiesByType: []EntityTypeMetrics{},
	}

	// Get counts directly from game server state
	if h.gameServer == nil {
		return metrics
	}

	// Entity types to track
	entityTypes := []string{"player", "bot", "floor", "obstacle", "foreground", "portal"}

	// Get entity counts from game server
	entityCounts := h.gameServer.GetEntityCounts()

	for _, eType := range entityTypes {
		typeMetrics := EntityTypeMetrics{
			Type:                 eType,
			Count:                0,
			TotalObjectLayers:    0,
			ActiveObjectLayers:   0,
			InactiveObjectLayers: 0,
		}

		// Count entities based on type
		switch eType {
		case "player":
			typeMetrics.Count = entityCounts["players"]
		case "bot":
			typeMetrics.Count = entityCounts["bots"]
		case "floor":
			typeMetrics.Count = entityCounts["floors"]
		case "obstacle":
			typeMetrics.Count = entityCounts["obstacles"]
		case "foreground":
			typeMetrics.Count = entityCounts["foregrounds"]
		case "portal":
			typeMetrics.Count = entityCounts["portals"]
		}

		// Count object layers for this entity type from game server state
		objLayerCount := h.countObjectLayersFromGameState(eType)
		typeMetrics.TotalObjectLayers = objLayerCount.Total
		typeMetrics.ActiveObjectLayers = objLayerCount.Active
		typeMetrics.InactiveObjectLayers = objLayerCount.Inactive

		metrics.EntitiesByType = append(metrics.EntitiesByType, typeMetrics)
		metrics.TotalEntities += typeMetrics.Count
		metrics.TotalObjectLayers += typeMetrics.TotalObjectLayers
		metrics.ActiveObjectLayers += typeMetrics.ActiveObjectLayers
		metrics.InactiveObjectLayers += typeMetrics.InactiveObjectLayers
	}

	// Calculate average
	if metrics.TotalEntities > 0 {
		metrics.AvgObjectLayersPerEntity = float64(metrics.TotalObjectLayers) / float64(metrics.TotalEntities)
	}

	return metrics
}

// ObjectLayerCount holds counts of object layers
type ObjectLayerCount struct {
	Total    int
	Active   int
	Inactive int
}

// countObjectLayersFromGameState counts object layers from actual entity states
// Active = ObjectLayerState.Active == true
// Inactive = ObjectLayerState.Active == false
func (h *MetricsHandler) countObjectLayersFromGameState(entityType string) ObjectLayerCount {
	count := ObjectLayerCount{Total: 0, Active: 0, Inactive: 0}

	if h.gameServer == nil {
		return count
	}

	// Get counts from game server which tracks actual entity states
	switch entityType {
	case "player":
		playerLayers := h.gameServer.GetPlayerObjectLayers()
		count.Total = playerLayers.Total
		count.Active = playerLayers.Active
		count.Inactive = playerLayers.Inactive

	case "bot":
		botLayers := h.gameServer.GetBotObjectLayers()
		count.Total = botLayers.Total
		count.Active = botLayers.Active
		count.Inactive = botLayers.Inactive

	case "floor":
		floorLayers := h.gameServer.GetFloorObjectLayers()
		count.Total = floorLayers.Total
		count.Active = floorLayers.Active
		count.Inactive = floorLayers.Inactive

	case "obstacle", "foreground", "portal":
		// These static entity types don't have object layers that are toggled
		// Their object layers are always considered "active" (always present)
		count.Total = 0
		count.Active = 0
		count.Inactive = 0
	}

	return count
}

// calculateWorkloadMetrics calculates current workload based on entity metrics
func (h *MetricsHandler) calculateWorkloadMetrics(entityMetrics EntityMetrics) WorkloadMetrics {
	const maxEntityCapacity = 3000
	const maxObjectLayers = 10000

	workload := WorkloadMetrics{
		MaxEntityCapacity: maxEntityCapacity,
		MaxObjectLayers:   maxObjectLayers,
	}

	// Calculate entity load percentage
	entityLoadPct := float64(entityMetrics.TotalEntities) / float64(maxEntityCapacity) * 100

	// Calculate object layer load percentage
	objLayerLoadPct := float64(entityMetrics.TotalObjectLayers) / float64(maxObjectLayers) * 100

	// Overall load is the higher of the two
	workload.LoadPercentage = entityLoadPct
	if objLayerLoadPct > entityLoadPct {
		workload.LoadPercentage = objLayerLoadPct
	}

	// Determine current load level
	if workload.LoadPercentage < 40 {
		workload.CurrentLoad = "low"
	} else if workload.LoadPercentage < 70 {
		workload.CurrentLoad = "medium"
	} else if workload.LoadPercentage < 90 {
		workload.CurrentLoad = "high"
	} else {
		workload.CurrentLoad = "critical"
	}

	return workload
}

// determineHealth determines overall system health based on metrics
func (h *MetricsHandler) determineHealth(entityMetrics EntityMetrics, workloadMetrics WorkloadMetrics, wsMetrics WebSocketServerMetrics) (HealthStatus, string) {
	// Check WebSocket status first
	if wsMetrics.Status == WebSocketError || wsMetrics.Status == WebSocketCrashed {
		return HealthCritical, "WebSocket server error or crashed - unable to accept connections"
	}

	if wsMetrics.Status == WebSocketStopping {
		return HealthMaintenance, "Server is performing graceful shutdown - no new connections accepted"
	}

	// Check workload first as it's most critical
	if workloadMetrics.CurrentLoad == "critical" {
		return HealthDown, "System workload at critical levels (>90%) - service may become unavailable"
	}

	if workloadMetrics.CurrentLoad == "high" {
		// Additional check on entity counts
		if entityMetrics.TotalEntities >= h.warningEntityThreshold ||
			entityMetrics.TotalObjectLayers >= h.warningObjectLayerThreshold {
			return HealthDegraded, "System workload is high (70-90%) with entity/layer counts approaching thresholds - reduced functionality expected"
		}
		return HealthWarning, "System workload is high (70-90%) - monitor performance closely"
	}

	// Warning thresholds - only check when workload is medium or low
	if workloadMetrics.CurrentLoad == "medium" {
		if entityMetrics.TotalEntities >= h.warningEntityThreshold ||
			entityMetrics.TotalObjectLayers >= h.warningObjectLayerThreshold {
			return HealthWarning, "Entity or object layer count approaching threshold limits - monitor for escalation"
		}
	}

	// All systems normal
	if workloadMetrics.CurrentLoad == "low" || workloadMetrics.CurrentLoad == "medium" {
		if wsMetrics.ActiveConnections > 0 {
			connStr := "connection"
			if wsMetrics.ActiveConnections > 1 {
				connStr = "connections"
			}
			return HealthHealthy, fmt.Sprintf("All systems operational - %d active %s", wsMetrics.ActiveConnections, connStr)
		}
		return HealthHealthy, "Server ready and operational - awaiting connections"
	}

	return HealthOk, "System operational within normal parameters"
}

// UpdateWebSocketMetrics updates WebSocket connection metrics and connection counts
func (h *MetricsHandler) UpdateWebSocketMetrics(status WebSocketStatus, activeConnections int) {
	h.mu.Lock()
	defer h.mu.Unlock()

	h.webSocketMetrics.Status = status
	h.webSocketMetrics.ActiveConnections = activeConnections

	// Update uptime regardless of status
	h.webSocketMetrics.UptimeSec = int64(time.Since(h.serverStartTime).Seconds())
}

// RecordWebSocketError records a WebSocket error
func (h *MetricsHandler) RecordWebSocketError(errorMsg string) {
	h.mu.Lock()
	defer h.mu.Unlock()

	now := time.Now()
	h.webSocketMetrics.Status = WebSocketError
	h.webSocketMetrics.LastErrorMessage = errorMsg
	h.webSocketMetrics.LastErrorTime = &now
}

// SetWebSocketStatus sets the WebSocket status
func (h *MetricsHandler) SetWebSocketStatus(status WebSocketStatus) {
	h.mu.Lock()
	defer h.mu.Unlock()

	h.webSocketMetrics.Status = status
}
