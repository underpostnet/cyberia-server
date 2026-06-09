// Package problem implements RFC 9457 (Problem Details for HTTP APIs).
//
// All errors returned by the cyberia-server REST API MUST flow through
// this package. The wire format is documented in
// https://www.rfc-editor.org/rfc/rfc9457 and consumed by clients that
// branch on `type` (a stable, dereferenceable URI) — never on the
// human-readable `title` or `detail`.
//
// The Content-Type is fixed to application/problem+json. The HTTP status
// code is mirrored into the `status` member so that JSON-only consumers
// (browsers, log aggregators) do not need to read response headers.
package problem

import (
	"encoding/json"
	"net/http"
)

// BaseTypeURI is the documentation root for problem types. Each named
// problem in this package appends a stable kebab-case slug — once
// published, a slug must never change semantics. The host is overridable
// via SetBaseTypeURI so deployments behind a custom domain still publish
// dereferenceable problem URIs.
var baseTypeURI = "https://api.cyberiaonline.com/errors"

// SetBaseTypeURI overrides the host portion of every Problem.Type URI
// produced by this package. Intended to be called once at server boot,
// from cmd/cyberia-server/main.go, based on an env var.
func SetBaseTypeURI(uri string) {
	if uri == "" {
		return
	}
	baseTypeURI = uri
}

// Problem is the on-the-wire representation of an RFC 9457 document.
// Extensions (any additional, ad-hoc fields) are flattened into the top
// level object via MarshalJSON — the RFC explicitly allows this and
// clients are required to ignore unknown members.
type Problem struct {
	// Type is a URI reference that identifies the problem class. Clients
	// branch on this value. Defaults to about:blank if unset.
	Type string `json:"type"`
	// Title is a short, human-readable summary, kept stable per Type.
	Title string `json:"title"`
	// Status mirrors the HTTP status code.
	Status int `json:"status"`
	// Detail is a human-readable explanation specific to this occurrence.
	Detail string `json:"detail,omitempty"`
	// Instance is a URI reference that identifies the specific occurrence,
	// typically the request URI. Populated automatically by Write().
	Instance string `json:"instance,omitempty"`

	// Extensions carries additional members. They are merged into the
	// outer JSON object — clients see them as siblings of Type/Title/etc.
	Extensions map[string]any `json:"-"`
}

// MarshalJSON flattens Extensions into the top-level JSON object.
func (p Problem) MarshalJSON() ([]byte, error) {
	out := map[string]any{
		"type":   p.Type,
		"title":  p.Title,
		"status": p.Status,
	}
	if p.Detail != "" {
		out["detail"] = p.Detail
	}
	if p.Instance != "" {
		out["instance"] = p.Instance
	}
	for k, v := range p.Extensions {
		// Don't let extensions overwrite reserved fields silently.
		switch k {
		case "type", "title", "status", "detail", "instance":
			continue
		}
		out[k] = v
	}
	return json.Marshal(out)
}

// WithExtension returns a copy of p with the given extension set.
// Chainable so handlers can decorate problems inline.
func (p Problem) WithExtension(key string, value any) Problem {
	ext := make(map[string]any, len(p.Extensions)+1)
	for k, v := range p.Extensions {
		ext[k] = v
	}
	ext[key] = value
	p.Extensions = ext
	return p
}

// WithDetail returns a copy of p with a new Detail line.
func (p Problem) WithDetail(detail string) Problem {
	p.Detail = detail
	return p
}

// Write serializes the problem to w with the correct Content-Type and
// status code, deriving Instance from the request URI if unset. This is
// the only sanctioned path for emitting an error in the API package.
func Write(w http.ResponseWriter, r *http.Request, p Problem) {
	if p.Status == 0 {
		p.Status = http.StatusInternalServerError
	}
	if p.Type == "" {
		p.Type = "about:blank"
	}
	if p.Instance == "" && r != nil {
		p.Instance = r.URL.RequestURI()
	}
	body, err := json.Marshal(p)
	if err != nil {
		// Marshalling a Problem cannot fail in practice (all string/int
		// members) — fall back to a minimal valid problem document so
		// the client still gets the right Content-Type and status.
		body = []byte(`{"type":"about:blank","title":"Internal Server Error","status":500}`)
		p.Status = http.StatusInternalServerError
	}
	w.Header().Set("Content-Type", "application/problem+json; charset=utf-8")
	w.Header().Set("Cache-Control", "no-store")
	w.WriteHeader(p.Status)
	_, _ = w.Write(body)
}

// typeURI builds an absolute problem type URI for the given slug.
func typeURI(slug string) string {
	return baseTypeURI + "/" + slug
}

// Named problem constructors. These are the only sanctioned set — adding
// a new constant requires also adding the URI to the public problem
// catalogue at <baseTypeURI>/<slug>.

// NotFound — RFC 9110 §15.5.5. Use when a route is reachable but the
// addressed resource does not exist (e.g. /metrics/maps/UNKNOWN).
func NotFound(detail string) Problem {
	return Problem{
		Type:   typeURI("not-found"),
		Title:  "Resource not found",
		Status: http.StatusNotFound,
		Detail: detail,
	}
}

// MethodNotAllowed — RFC 9110 §15.5.6. Allow header must be populated
// by the caller before Write — see chi's MethodNotAllowedHandler.
func MethodNotAllowed(detail string) Problem {
	return Problem{
		Type:   typeURI("method-not-allowed"),
		Title:  "Method not allowed",
		Status: http.StatusMethodNotAllowed,
		Detail: detail,
	}
}

// BadRequest — RFC 9110 §15.5.1. Use for malformed query strings,
// out-of-range parameters, or any client-side validation failure.
func BadRequest(detail string) Problem {
	return Problem{
		Type:   typeURI("bad-request"),
		Title:  "Bad request",
		Status: http.StatusBadRequest,
		Detail: detail,
	}
}

// InternalServerError — RFC 9110 §15.6.1. Reserved for unrecoverable
// server faults; the panic recoverer in the router emits this.
func InternalServerError(detail string) Problem {
	return Problem{
		Type:   typeURI("internal-server-error"),
		Title:  "Internal server error",
		Status: http.StatusInternalServerError,
		Detail: detail,
	}
}

// ServiceUnavailable — RFC 9110 §15.6.4. Use when the simulation is
// not ready (e.g. gRPC world load still in progress).
func ServiceUnavailable(detail string) Problem {
	return Problem{
		Type:   typeURI("service-unavailable"),
		Title:  "Service unavailable",
		Status: http.StatusServiceUnavailable,
		Detail: detail,
	}
}
