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
