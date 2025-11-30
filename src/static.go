package game

import (
	"log"
	"net/http"
	"os"
)

// StaticFileServer serves static files from a given directory.
func StaticFileServer(dir string, fallbackPath string) http.Handler {
	// Check if the directory exists
	if _, err := os.Stat(dir); os.IsNotExist(err) {
		log.Fatalf("Static directory does not exist: %s", dir)
	}

	// Create a file server handler
	fs := http.FileServer(http.Dir(dir))

	// Return a handler that serves the files or falls back to index.html for SPAs
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		// Add CORS headers for cross-origin requests
		w.Header().Set("Access-Control-Allow-Origin", "*")
		w.Header().Set("Access-Control-Allow-Methods", "GET, OPTIONS")
		w.Header().Set("Access-Control-Allow-Headers", "Accept, Content-Type")

		// Handle preflight requests
		if r.Method == "OPTIONS" {
			w.WriteHeader(http.StatusOK)
			return
		}

		// Check if the requested path is a file that exists
		if _, err := os.Stat(dir + r.URL.Path); err == nil {
			fs.ServeHTTP(w, r)
			return
		}

		// Fallback to the specified path (e.g., index.html) for all other requests
		// This is useful for single-page applications (SPAs) where the client-side
		// router handles the routes.
		http.ServeFile(w, r, dir+fallbackPath)
	})
}
