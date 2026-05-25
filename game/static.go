package game

import (
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strings"
)

// StaticFileServer serves static assets from dir and falls back to
// fallbackPath (typically /index.html) for SPA-style routing.
//
// Hardening guarantees:
//
//   - The handler resolves every request path under dir to prevent
//     `../` traversal (an unresolved `http.ServeFile` call would happily
//     read outside dir if the URL contained `..`).
//   - If dir does not exist or does not contain a fallback file, the
//     handler refuses to start. The previous implementation would
//     silently `os.MkdirAll` an empty dir, then serve an empty 404 —
//     the visible symptom was a blank white page in the browser.
//   - Falls back to the SPA index ONLY for GET/HEAD requests and only
//     when no real file matches; other methods get 405.
//   - Adds Content-Type-Options + a short cache TTL for the asset
//     directory; the SPA shell itself is served `no-store` so dashboard
//     edits propagate immediately during ops work.
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

	fs := http.FileServer(http.Dir(absDir))

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

		// Resolve the requested path under absDir without allowing
		// `..` to escape. filepath.Join already cleans the result; we
		// re-check the prefix because path-traversal payloads can
		// produce a sibling-absolute path after cleaning.
		urlPath := r.URL.Path
		if urlPath == "" || urlPath == "/" {
			urlPath = fallbackPath
		}
		resolved := filepath.Join(absDir, filepath.FromSlash(urlPath))
		if !strings.HasPrefix(resolved+string(os.PathSeparator), absDir+string(os.PathSeparator)) &&
			resolved != absDir {
			http.Error(w, "forbidden", http.StatusForbidden)
			return
		}

		// Real file? Serve it with a modest cache header.
		if info, err := os.Stat(resolved); err == nil && !info.IsDir() {
			w.Header().Set("X-Content-Type-Options", "nosniff")
			w.Header().Set("Cache-Control", "public, max-age=300")
			fs.ServeHTTP(w, r)
			return
		}

		// SPA fallback: serve the dashboard shell. no-store keeps live
		// ops edits visible on the next refresh without a cache bust.
		w.Header().Set("X-Content-Type-Options", "nosniff")
		w.Header().Set("Cache-Control", "no-store")
		http.ServeFile(w, r, indexFS)
	})
}
