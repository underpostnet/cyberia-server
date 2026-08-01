// Package httpserver holds HTTP-transport plumbing that is not part of the
// game simulation: static asset serving, SPA fallback, and related hardening.
// It depends on nothing in the game package — keep it that way.
package httpserver

import (
	"bytes"
	"encoding/json"
	"net/http"
	"os"
	"path/filepath"
	"strings"

	"cyberia-server/logx"
)

// unavailable answers every request with 503 — used when dir itself can't be served.
func unavailable(reason string) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		http.Error(w, "static assets unavailable: "+reason, http.StatusServiceUnavailable)
	})
}

// StaticFileServer serves static assets from dir.
//
// Invariants:
//
//   - Resolves every request path under dir; rejects `../` traversal.
//   - dir or its fallback file missing → logs a warning and serves 503 instead
//     of the dashboard; other httpserver routes keep working.
//   - Unmatched GET/HEAD paths fall back to fallbackPath (SPA). Envoy owns
//     externally visible custom status responses and serves them before a
//     request can reach this application.
//   - HTML responses carry window.CYBERIA_BASE_PATH so the dashboard knows
//     its public sub-path.
//   - Asset files get nosniff + a short cache TTL; HTML is served no-store.
func StaticFileServer(dir string, fallbackPath string, basePath string) http.Handler {
	absDir, err := filepath.Abs(dir)
	if err != nil {
		logx.Errorf("StaticFileServer: cannot resolve %q: %v", dir, err)
		return unavailable("bad static dir")
	}
	info, err := os.Stat(absDir)
	if err != nil || !info.IsDir() {
		logx.Errorf("StaticFileServer: %s is not a directory: %v", absDir, err)
		return unavailable("static dir missing")
	}
	indexFS := filepath.Join(absDir, filepath.FromSlash(fallbackPath))
	hasIndex := true
	if _, err := os.Stat(indexFS); err != nil {
		logx.Errorf("StaticFileServer: fallback %s missing: %v — "+
			"run `npm run cyberia:dashboard` to regenerate the dashboard.", indexFS, err)
		hasIndex = false
	}

	// This instance's URL sub-path ("/FOREST", "/TEST", "" default), pre-encoded
	// once as a JS string literal for injection into HTML responses.
	basePathJSON, _ := json.Marshal(basePath)

	fs := http.FileServer(http.Dir(absDir))

	// serveHTML writes an HTML file, splicing window.CYBERIA_BASE_PATH
	// in before </head> (prepended if there is no head).
	serveHTML := func(w http.ResponseWriter, path string) {
		body, err := os.ReadFile(path)
		if err != nil {
			http.Error(w, "static asset unavailable", http.StatusServiceUnavailable)
			return
		}
		script := []byte("<script>window.CYBERIA_BASE_PATH=" + string(basePathJSON) + ";</script>")
		if i := bytes.Index(body, []byte("</head>")); i >= 0 {
			body = append(body[:i:i], append(script, body[i:]...)...)
		} else {
			body = append(script, body...)
		}
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		w.Header().Set("X-Content-Type-Options", "nosniff")
		w.Header().Set("Cache-Control", "no-store")
		w.WriteHeader(http.StatusOK)
		_, _ = w.Write(body)
	}

	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		// CORS for static assets (the API has its own CORS middleware).
		w.Header().Set("Access-Control-Allow-Origin", "*")
		w.Header().Set("Access-Control-Allow-Methods", "GET, HEAD, OPTIONS")
		w.Header().Set("Access-Control-Allow-Headers", "Accept, Content-Type")

		switch r.Method {
		case http.MethodOptions:
			w.WriteHeader(http.StatusOK)
			return
		case http.MethodGet, http.MethodHead:
			// fall through
		default:
			w.Header().Set("Allow", "GET, HEAD, OPTIONS")
			http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
			return
		}

		// chi's Mount strips basePath for route matching but intentionally keeps
		// r.URL.Path unchanged. Static file resolution must remove the public
		// prefix explicitly; otherwise /TEST/ looks for <dir>/TEST and 404s.
		urlPath := staticPathWithinBase(r.URL.Path, basePath)
		if urlPath == "" || urlPath == "/" {
			if !hasIndex {
				http.Error(w, "dashboard not built", http.StatusServiceUnavailable)
				return
			}
			serveHTML(w, indexFS)
			return
		}

		// Resolve under absDir and reject escapes: filepath.Join cleans `..` but
		// can still yield a sibling-absolute path, so re-check the prefix.
		resolved := filepath.Join(absDir, filepath.FromSlash(urlPath))
		if !strings.HasPrefix(resolved+string(os.PathSeparator), absDir+string(os.PathSeparator)) &&
			resolved != absDir {
			http.Error(w, "forbidden", http.StatusForbidden)
			return
		}

		// Real file? HTML gets base-path injection; other assets served verbatim.
		if info, err := os.Stat(resolved); err == nil && !info.IsDir() {
			if strings.HasSuffix(resolved, ".html") {
				serveHTML(w, resolved)
				return
			}
			w.Header().Set("X-Content-Type-Options", "nosniff")
			w.Header().Set("Cache-Control", "public, max-age=300")
			staticRequest := r.Clone(r.Context())
			staticURL := *r.URL
			staticURL.Path = urlPath
			staticURL.RawPath = ""
			staticRequest.URL = &staticURL
			fs.ServeHTTP(w, staticRequest)
			return
		}

		// Unknown path → 404. Custom status pages are an Envoy concern.
		http.NotFound(w, r)
	})
}

// staticPathWithinBase maps a public variant path onto the shared static
// directory while retaining the root compatibility mount. Segment-bounded
// matching ensures /TEST does not strip an unrelated /TESTING path.
func staticPathWithinBase(urlPath string, basePath string) string {
	basePath = strings.TrimSuffix(basePath, "/")
	if basePath == "" || basePath == "/" {
		return urlPath
	}
	if urlPath == basePath {
		return "/"
	}
	if strings.HasPrefix(urlPath, basePath+"/") {
		return strings.TrimPrefix(urlPath, basePath)
	}
	return urlPath
}
