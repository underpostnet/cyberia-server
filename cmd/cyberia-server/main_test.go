package main

import (
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/go-chi/chi/v5"
)

func TestMountAtVariantBasePath(t *testing.T) {
	app := chi.NewRouter()
	app.Get("/ws", func(w http.ResponseWriter, _ *http.Request) { w.WriteHeader(http.StatusNoContent) })
	root := chi.NewRouter()
	mountAtBasePath(root, "/FOREST", app)

	variant := httptest.NewRecorder()
	root.ServeHTTP(variant, httptest.NewRequest(http.MethodGet, "/FOREST/ws", nil))
	if variant.Code != http.StatusNoContent {
		t.Fatalf("GET /FOREST/ws status = %d, want %d", variant.Code, http.StatusNoContent)
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
