package httpserver

import (
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestStaticFileServerConsumesVariantBasePath(t *testing.T) {
	dir := t.TempDir()
	if err := os.WriteFile(filepath.Join(dir, "index.html"), []byte("<html><head></head><body>dashboard</body></html>"), 0o600); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(dir, "dashboard.js"), []byte("dashboard-asset"), 0o600); err != nil {
		t.Fatal(err)
	}

	handler := StaticFileServer(dir, "/index.html", "/TEST")

	dashboard := httptest.NewRecorder()
	handler.ServeHTTP(dashboard, httptest.NewRequest(http.MethodGet, "/TEST/", nil))
	if dashboard.Code != http.StatusOK {
		t.Fatalf("GET /TEST/ status = %d, want %d", dashboard.Code, http.StatusOK)
	}
	if body := dashboard.Body.String(); !strings.Contains(body, "window.CYBERIA_BASE_PATH=\"/TEST\"") {
		t.Fatalf("GET /TEST/ did not inject base path: %s", body)
	}

	asset := httptest.NewRecorder()
	handler.ServeHTTP(asset, httptest.NewRequest(http.MethodGet, "/TEST/dashboard.js", nil))
	if asset.Code != http.StatusOK {
		t.Fatalf("GET /TEST/dashboard.js status = %d, want %d", asset.Code, http.StatusOK)
	}
	if body := asset.Body.String(); body != "dashboard-asset" {
		t.Fatalf("GET /TEST/dashboard.js body = %q, want dashboard-asset", body)
	}

	legacyRoot := httptest.NewRecorder()
	handler.ServeHTTP(legacyRoot, httptest.NewRequest(http.MethodGet, "/", nil))
	if legacyRoot.Code != http.StatusOK {
		t.Fatalf("legacy GET / status = %d, want %d", legacyRoot.Code, http.StatusOK)
	}
}

func TestStaticPathWithinBaseIsSegmentBounded(t *testing.T) {
	tests := map[string]string{
		"/TEST":              "/",
		"/TEST/":             "/",
		"/TEST/dashboard.js": "/dashboard.js",
		"/TESTING/":          "/TESTING/",
		"/":                  "/",
	}
	for input, want := range tests {
		if got := staticPathWithinBase(input, "/TEST"); got != want {
			t.Errorf("staticPathWithinBase(%q, /TEST) = %q, want %q", input, got, want)
		}
	}
}
