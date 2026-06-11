// Package httpserver holds HTTP-transport plumbing that is not part of the
// game simulation: static asset serving, SPA fallback, and related hardening.
// It depends on nothing in the game package — keep it that way.
package httpserver

import (
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strings"
)

// StaticFileServer serves static assets from dir, falling back to
// fallbackPath (typically /index.html) for SPA-style routing.
//
// Invariants:
//
//   - Resolves every request path under dir; rejects `../` traversal.
//   - Refuses to start if dir is missing or holds no fallback file.
//   - Serves the SPA fallback only for GET/HEAD when no real file matches;
//     other methods get 405.
//   - Asset files get nosniff + a short cache TTL; the SPA shell is served
//     no-store so dashboard edits show on the next refresh.
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

		// Resolve under absDir and reject escapes: filepath.Join cleans
		// `..` but can still yield a sibling-absolute path, so re-check the prefix.
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
