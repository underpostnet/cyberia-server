package hotreload

import (
	"encoding/json"
	"net/http"
	"strings"

	"cyberia-server/httpserver/problem"

	"github.com/go-chi/chi/v5"
)

// APIKeyHeader carries the shared secret on the REST transport. The key may
// also arrive as `Authorization: Bearer <key>`.
const APIKeyHeader = "X-Cyberia-Server-Api-Key"

// restRequest is the POST body; the key may instead ride a header.
type restRequest struct {
	APIKey       string `json:"apiKey"`
	Mode         string `json:"mode"`
	InstanceCode string `json:"instanceCode"`
}

// presentedKey resolves the caller's key: header first, then bearer token,
// then the JSON body — so operators can pick whichever their proxy allows.
func presentedKey(r *http.Request, body restRequest) string {
	if key := r.Header.Get(APIKeyHeader); key != "" {
		return key
	}
	if auth := r.Header.Get("Authorization"); strings.HasPrefix(auth, "Bearer ") {
		return strings.TrimPrefix(auth, "Bearer ")
	}
	return body.APIKey
}

// Routes mounts the REST fallback under the caller's subtree:
//
//	POST /hot-reload — trigger a reload (requires CYBERIA_SERVER_API_KEY)
func (s *Service) Routes(r chi.Router) {
	r.Post("/hot-reload", s.handleTrigger)
}

func (s *Service) handleTrigger(w http.ResponseWriter, r *http.Request) {
	var body restRequest
	// An empty/!JSON body is fine when the key rides a header.
	_ = json.NewDecoder(http.MaxBytesReader(w, r.Body, 4<<10)).Decode(&body)

	res, err := s.Trigger(r.Context(), presentedKey(r, body), Mode(body.Mode), body.InstanceCode)
	if err != nil {
		switch err {
		case ErrUnauthorized:
			// Same response whether the key is wrong or the trigger is
			// disabled — never confirm a key's existence to a prober.
			problem.Write(w, r, problem.Problem{
				Title:  "Forbidden",
				Status: http.StatusForbidden,
				Detail: "hot-reload trigger requires a valid internal API key",
			})
		case ErrBusy:
			problem.Write(w, r, problem.Problem{
				Title:  "Conflict",
				Status: http.StatusConflict,
				Detail: err.Error(),
			})
		default:
			problem.Write(w, r, problem.InternalServerError(err.Error()))
		}
		return
	}

	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	w.Header().Set("Cache-Control", "no-store")
	w.WriteHeader(http.StatusOK)
	_ = json.NewEncoder(w).Encode(res)
}
