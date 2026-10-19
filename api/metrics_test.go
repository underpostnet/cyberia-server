package api

import (
	"encoding/json"
	"fmt"
	"math"
	"net/http"
	"net/http/httptest"
	"testing"

	"cyberia-server/game/net"
	"cyberia-server/game/world"
)

func TestWorkloadUsesMeasuredTickWork(t *testing.T) {
	for _, tc := range []struct {
		work float64
		want WorkloadLevel
	}{
		{0, WorkloadLow}, {3.99, WorkloadLow}, {4, WorkloadModerate},
		{6.99, WorkloadModerate}, {7, WorkloadHigh}, {8.99, WorkloadHigh},
		{9, WorkloadCritical}, {12, WorkloadCritical},
	} {
		t.Run(fmt.Sprint(tc.work), func(t *testing.T) {
			got := calculateWorkloadMetrics(RuntimeMetrics{RuntimeMetricsSnapshot: net.RuntimeMetricsSnapshot{
				TickDurationMs: 10, TickWorkMs: tc.work,
			}})
			if got.CurrentLoad != tc.want || math.Abs(got.LoadPercentage-tc.work*10) > 1e-9 {
				t.Fatalf("got %+v, want %s at %v%%", got, tc.want, tc.work*10)
			}
		})
	}
	if got := calculateWorkloadMetrics(RuntimeMetrics{}); got.LoadPercentage != 0 || got.CurrentLoad != WorkloadLow {
		t.Fatalf("no timing sample: %+v", got)
	}
}

func TestSceneryDoesNotDegradeHealthOrBecomePlayers(t *testing.T) {
	gs := net.NewGameServer()
	limits := net.DefaultConnectionLimits()
	limits.MaxConnections = 7
	gs.SetConnectionLimits(limits)
	ms := world.NewMapState("test", 100, 100)
	gs.Maps["test"] = ms
	for id := 1; id <= 10000; id++ {
		ms.Spawn(world.EntityID(id), world.EntityData{Kind: "floor"}, world.Transform{}, world.Appearance{})
	}
	handler := NewMetricsHandler(gs, "test")
	rec := httptest.NewRecorder()
	handler.GetMetrics(rec, httptest.NewRequest(http.MethodGet, "/api/v1/metrics", nil))
	var response MetricsResponse
	if err := json.Unmarshal(rec.Body.Bytes(), &response); err != nil {
		t.Fatal(err)
	}
	if response.Entities.TotalEntities != 10000 || response.Health != HealthHealthy {
		t.Fatalf("entities=%d, health=%s", response.Entities.TotalEntities, response.Health)
	}
	if response.Workload.LoadPercentage != 0 || response.WebSocket.ActiveConnections != 0 || response.WebSocket.MaxConnections != 7 {
		t.Fatalf("workload=%+v, websocket=%+v", response.Workload, response.WebSocket)
	}
	gs.BeginDrain()
	response = *handler.collectMetrics()
	if response.Health != HealthMaintenance || response.WebSocket.Status != WebSocketStopping {
		t.Fatalf("draining: health=%s, websocket=%s", response.Health, response.WebSocket.Status)
	}
}

func TestHealthUsesTickPressureAndReadiness(t *testing.T) {
	gs := net.NewGameServer()
	handler := NewMetricsHandler(gs, "test")
	if got, _ := handler.determineHealth(WorkloadMetrics{CurrentLoad: WorkloadLow}, WebSocketServerMetrics{}); got != HealthMaintenance {
		t.Fatalf("world is loading: got %s", got)
	}
	gs.Maps["test"] = world.NewMapState("test", 1, 1)
	for _, tc := range []struct {
		load WorkloadLevel
		want HealthStatus
	}{{WorkloadLow, HealthHealthy}, {WorkloadModerate, HealthHealthy}, {WorkloadHigh, HealthDegraded}, {WorkloadCritical, HealthCritical}} {
		if got, _ := handler.determineHealth(WorkloadMetrics{CurrentLoad: tc.load}, WebSocketServerMetrics{Status: WebSocketRunning}); got != tc.want {
			t.Fatalf("%s: got %s, want %s", tc.load, got, tc.want)
		}
	}
}
