// Package httpserver holds HTTP-transport plumbing that is not part of the
// game simulation: static asset serving, SPA fallback, and related hardening.
// It depends on nothing in the game package — keep it that way.
package httpserver

import (
	"bytes"
	"encoding/json"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strings"
)

// StaticFileServer serves static assets from dir.
//
// Invariants:
//
//   - Resolves every request path under dir; rejects `../` traversal.
//   - Refuses to start if dir is missing or holds no fallback file.
//   - An unmatched path serves 404.html with a 404 status when present (so
//     /FOREST/<bad> lands on this instance's 404, addressable at /404);
//     otherwise it falls back to fallbackPath (SPA) for GET/HEAD. 405 otherwise.
//   - HTML responses (index.html, 404.html) carry window.CYBERIA_BASE_PATH,
//     injected from the CYBERIA_BASE_PATH env, so the instance-aware dashboard
//     and 404 know their sub-path — the reverse proxy strips that prefix before
//     the request reaches this server.
//   - Asset files get nosniff + a short cache TTL; HTML is served no-store.
func StaticFileServer(dir string, fallbackPath string) http.Handler {
	absDir, err := filepath.Abs(dir)
	if err != nil {
		log.Fatalf("StaticFileServer: cannot resolve %q: %v", dir, err)
	}
	info, err := os.Stat(absDir)
	if err != nil || !info.IsDir() {
		log.Fatalf("StaticFileServer: %s is not a directory: %v", absDir, err)
	}
	indexFS := filepath.Join(absDir, filepath.FromSlash(fallbackPath))
	if _, err := os.Stat(indexFS); err != nil {
		log.Fatalf("StaticFileServer: fallback %s missing: %v — "+
			"run `npm run cyberia:dashboard` to regenerate the dashboard.", indexFS, err)
	}

	// Optional per-instance 404 page (built by `cyberia run-workflow
	// build-cyberia-404`). Absent → unknown paths fall back to the SPA shell.
	notFoundFS := filepath.Join(absDir, "404.html")
	_, notFoundErr := os.Stat(notFoundFS)
	hasNotFound := notFoundErr == nil

	// This instance's URL sub-path ("/FOREST", "/TEST", "" default), pre-encoded
	// once as a JS string literal for injection into HTML responses.
	basePathJSON, _ := json.Marshal(os.Getenv("CYBERIA_BASE_PATH"))

	fs := http.FileServer(http.Dir(absDir))

	// serveHTML writes an HTML file at status, splicing window.CYBERIA_BASE_PATH
	// in before </head> (prepended if there is no head).
	serveHTML := func(w http.ResponseWriter, path string, status int) {
		body, err := os.ReadFile(path)
		if err != nil {
			http.Error(w, "not found", http.StatusNotFound)
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
		w.WriteHeader(status)
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

		urlPath := r.URL.Path
		if urlPath == "" || urlPath == "/" {
			serveHTML(w, indexFS, http.StatusOK)
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

		// The instance's 404 page, addressable directly at /404.
		if (urlPath == "/404" || urlPath == "/404/") && hasNotFound {
			serveHTML(w, notFoundFS, http.StatusNotFound)
			return
		}

		// Real file? HTML gets base-path injection; other assets served verbatim.
		if info, err := os.Stat(resolved); err == nil && !info.IsDir() {
			if strings.HasSuffix(resolved, ".html") {
				serveHTML(w, resolved, http.StatusOK)
				return
			}
			w.Header().Set("X-Content-Type-Options", "nosniff")
			w.Header().Set("Cache-Control", "public, max-age=300")
			fs.ServeHTTP(w, r)
			return
		}

		// Unknown path → this instance's 404 page (404), or the SPA shell when no
		// 404.html shipped.
		if hasNotFound {
			serveHTML(w, notFoundFS, http.StatusNotFound)
			return
		}
		w.Header().Set("X-Content-Type-Options", "nosniff")
		w.Header().Set("Cache-Control", "no-store")
		http.ServeFile(w, r, indexFS)
	})
}
