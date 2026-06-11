package api

import (
	_ "embed"
	"encoding/json"
	"net/http"

	"cyberia-server/httpserver/problem"

	"github.com/go-chi/chi/v5"
	"gopkg.in/yaml.v3"
)

// openapiYAML is the source of truth for the API contract. The JSON
// variant is derived once at boot via yaml.Unmarshal → json.Marshal so
// that a single edit to openapi.yaml updates every distribution
// (yaml endpoint, json endpoint, Swagger UI, and the Postman link
// rendered in the UI footer).
//
//go:embed openapi.yaml
var openapiYAML []byte

// postmanJSON is the v2.1 Postman collection. Kept hand-maintained
// alongside openapi.yaml; CI lints the two for drift.
//
//go:embed postman_collection.json
var postmanJSON []byte

// DocsHandler serves the OpenAPI document, Postman collection, and
// Swagger UI shell. It is stateless and safe to reuse.
type DocsHandler struct {
	openapiJSON []byte
}

// NewDocsHandler converts the embedded YAML into JSON once at construction
// time. If the YAML fails to parse, the JSON endpoint returns 500 with
// a problem+json envelope — but the YAML and HTML endpoints still work,
// so the dashboard never goes fully dark on a spec typo.
func NewDocsHandler() *DocsHandler {
	d := &DocsHandler{}
	var anyDoc any
	if err := yaml.Unmarshal(openapiYAML, &anyDoc); err == nil {
		if jsonBytes, jerr := json.MarshalIndent(toJSONCompatible(anyDoc), "", "  "); jerr == nil {
			d.openapiJSON = jsonBytes
		}
	}
	return d
}

// Routes mounts the docs subtree onto an existing chi.Router under /v1.
func (d *DocsHandler) Routes(r chi.Router) {
	r.Get("/openapi.yaml", d.serveOpenAPIYAML)
	r.Get("/openapi.json", d.serveOpenAPIJSON)
	r.Get("/postman.json", d.servePostman)
	r.Get("/docs", d.serveSwaggerUI)
}

func (d *DocsHandler) serveOpenAPIYAML(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/yaml; charset=utf-8")
	w.Header().Set("Cache-Control", "public, max-age=300")
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write(openapiYAML)
}

func (d *DocsHandler) serveOpenAPIJSON(w http.ResponseWriter, r *http.Request) {
	if d.openapiJSON == nil {
		problem.Write(w, r, problem.InternalServerError("openapi spec failed to parse at boot"))
		return
	}
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	w.Header().Set("Cache-Control", "public, max-age=300")
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write(d.openapiJSON)
}

func (d *DocsHandler) servePostman(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	w.Header().Set("Cache-Control", "public, max-age=300")
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write(postmanJSON)
}

// serveSwaggerUI returns a minimal Swagger UI shell wired to
// /api/v1/openapi.json. The UI assets are CDN-hosted to avoid bloating
// the Go binary with megabytes of static JS; deployments that need
// offline docs should proxy the CDN at the edge.
func (d *DocsHandler) serveSwaggerUI(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	w.Header().Set("Cache-Control", "public, max-age=300")
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write([]byte(swaggerHTML))
}

// Swagger UI shell. We deliberately ship the upstream stylesheet
// unmodified — a partial dark theme broke contrast in the Servers
// panel (white bg + gray-on-black text) because Swagger UI styles
// dozens of internal selectors that a thin override cannot reach
// consistently. Only the footer below the UI is custom, and it
// matches the light Swagger surface (white bg, dark text) so the
// page reads as one continuous panel.
const swaggerHTML = `<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Cyberia Server — API Reference</title>
  <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui.css">
  <link rel="icon" type="image/x-icon" href="/favicon.ico">
  <style>
    body { margin: 0; background: #fafafa; }
    #footer {
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      font-size: 13px;
      color: #3b4151;
      padding: 16px 24px;
      background: #ffffff;
      border-top: 1px solid #e1e4e8;
    }
    #footer .label { color: #6b7280; margin-right: 8px; }
    #footer a {
      color: #4990e2;
      text-decoration: none;
      margin-right: 18px;
      font-weight: 600;
    }
    #footer a:hover { text-decoration: underline; }
  </style>
</head>
<body>
  <div id="swagger-ui"></div>
  <div id="footer">
    <span class="label">Downloads:</span>
    <a href="/api/v1/openapi.yaml">openapi.yaml</a>
    <a href="/api/v1/openapi.json">openapi.json</a>
    <a href="/api/v1/postman.json" download="cyberia-server.postman_collection.json">postman_collection.json</a>
  </div>
  <script src="https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui-bundle.js"></script>
  <script>
    window.ui = SwaggerUIBundle({
      url: '/api/v1/openapi.json',
      dom_id: '#swagger-ui',
      deepLinking: true,
      defaultModelsExpandDepth: 0,
      docExpansion: 'list'
    });
  </script>
</body>
</html>`

// toJSONCompatible converts the map[interface{}]interface{} that
// gopkg.in/yaml.v3 produces into a json-marshalable shape (string keys).
func toJSONCompatible(in any) any {
	switch v := in.(type) {
	case map[string]any:
		out := make(map[string]any, len(v))
		for k, vv := range v {
			out[k] = toJSONCompatible(vv)
		}
		return out
	case map[any]any:
		out := make(map[string]any, len(v))
		for k, vv := range v {
			ks, ok := k.(string)
			if !ok {
				continue
			}
			out[ks] = toJSONCompatible(vv)
		}
		return out
	case []any:
		out := make([]any, len(v))
		for i, vv := range v {
			out[i] = toJSONCompatible(vv)
		}
		return out
	default:
		return v
	}
}
