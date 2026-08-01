package main

import (
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/go-chi/chi/v5"
)

func TestMountAtVariantBasePath(t *testing.T) {
	app := chi.NewRouter()
	app.Get("/", func(w http.ResponseWriter, _ *http.Request) { w.WriteHeader(http.StatusOK) })
	app.Get("/ws", func(w http.ResponseWriter, _ *http.Request) { w.WriteHeader(http.StatusNoContent) })
	app.Get("/api/v1/metrics", func(w http.ResponseWriter, _ *http.Request) { w.WriteHeader(http.StatusAccepted) })
	root := chi.NewRouter()
	mountAtBasePath(root, "/FOREST", app)

	dashboard := httptest.NewRecorder()
	root.ServeHTTP(dashboard, httptest.NewRequest(http.MethodGet, "/FOREST/", nil))
	if dashboard.Code != http.StatusOK {
		t.Fatalf("GET /FOREST/ status = %d, want %d", dashboard.Code, http.StatusOK)
	}

	variant := httptest.NewRecorder()
	root.ServeHTTP(variant, httptest.NewRequest(http.MethodGet, "/FOREST/ws", nil))
	if variant.Code != http.StatusNoContent {
		t.Fatalf("GET /FOREST/ws status = %d, want %d", variant.Code, http.StatusNoContent)
	}

	metrics := httptest.NewRecorder()
	root.ServeHTTP(metrics, httptest.NewRequest(http.MethodGet, "/FOREST/api/v1/metrics", nil))
	if metrics.Code != http.StatusAccepted {
		t.Fatalf("GET /FOREST/api/v1/metrics status = %d, want %d", metrics.Code, http.StatusAccepted)
	}

	legacy := httptest.NewRecorder()
	root.ServeHTTP(legacy, httptest.NewRequest(http.MethodGet, "/ws", nil))
	if legacy.Code != http.StatusNoContent {
		t.Fatalf("legacy GET /ws status = %d, want %d", legacy.Code, http.StatusNoContent)
	}
}

func TestMountAtRootBasePath(t *testing.T) {
	app := chi.NewRouter()
	app.Get("/ws", func(w http.ResponseWriter, _ *http.Request) { w.WriteHeader(http.StatusNoContent) })
	root := chi.NewRouter()
	mountAtBasePath(root, "/", app)

	response := httptest.NewRecorder()
	root.ServeHTTP(response, httptest.NewRequest(http.MethodGet, "/ws", nil))
	if response.Code != http.StatusNoContent {
		t.Fatalf("GET /ws status = %d, want %d", response.Code, http.StatusNoContent)
	}
}
