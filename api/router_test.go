package api

import (
	"net/http"
	"net/http/httptest"
	"testing"

	"cyberia-server/game/net"
)

func TestReadinessFailsWhileTheServerDrains(t *testing.T) {
	gs := net.NewGameServer()
	handler := readinessHandlerFor(gs)

	rec := httptest.NewRecorder()
	handler(rec, httptest.NewRequest(http.MethodGet, "/api/v1/health/ready", nil))
	if rec.Code != http.StatusServiceUnavailable {
		t.Fatalf("a server with no world answered %d, want 503", rec.Code)
	}

	gs.BeginDrain()
	rec = httptest.NewRecorder()
	handler(rec, httptest.NewRequest(http.MethodGet, "/api/v1/health/ready", nil))
	if rec.Code != http.StatusServiceUnavailable {
		t.Fatalf("a draining server answered %d, want 503", rec.Code)
	}
}
