// Package engine_client — registry.go
//
// Server registration. Each game server process reports itself to the Data
// Server registry under its own id. Replicas of one deployment report the same
// public URL, and the registry groups them into one logical endpoint.
//
// A report is idempotent and carries the next sequence number of the process,
// so the engine drops a report that arrives after a newer one. A document
// expires after the engine's TTL, so the report repeats on a ticker.
package engine_client

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"strings"
	"sync"
	"time"

	"cyberia-server/engineapi"
	"cyberia-server/logx"
)

const (
	registryRoute = "/cyberia-server-registry"
	// The engine drops a process three intervals after its last report.
	registryInterval = 20 * time.Second
	registryTimeout  = 10 * time.Second
)

// Registry states: a ready process takes new sessions, a draining one keeps its players and takes none.
const (
	StateReady    = "ready"
	StateDraining = "draining"
)

// Report is one registration body.
type Report struct {
	ServerID     string `json:"serverId"`
	URL          string `json:"url"`
	InstanceCode string `json:"instanceCode"`
	Name         string `json:"name"`
	State        string `json:"state"`
	Seq          uint64 `json:"seq"`
	// Build names the binary of the process: replicas of one build speak one protocol.
	Build string `json:"build,omitempty"`
	// ReleaseID is the content release the world of the process was built from.
	ReleaseID string `json:"releaseId,omitempty"`
}

// Registry reports one process to the Data Server registry.
type Registry struct {
	baseURL, apiKey string
	client          *http.Client

	mu      sync.Mutex
	report  Report
	release func() string
	stop    context.CancelFunc
}

// NewRegistry prepares the reports of one process. An empty URL or key
// disables them: the process stays off the list.
func NewRegistry(baseURL, apiKey string, report Report) *Registry {
	return &Registry{baseURL: strings.TrimRight(baseURL, "/"), apiKey: apiKey, client: &http.Client{}, report: report}
}

func (r *Registry) enabled() bool { return r.report.URL != "" && r.apiKey != "" }

// FollowRelease makes every report name the release release answers now.
// Call it before Start.
func (r *Registry) FollowRelease(release func() string) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.release = release
}

// next is the report of a state with the next sequence number.
func (r *Registry) next(state string) Report {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.report.Seq++
	r.report.State = state
	if r.release != nil {
		r.report.ReleaseID = r.release()
	}
	return r.report
}

// send posts one report.
func (r *Registry) send(ctx context.Context, report Report) error {
	body, err := json.Marshal(report)
	if err != nil {
		return fmt.Errorf("engine_client: register: %w", err)
	}
	ctx, cancel := context.WithTimeout(ctx, registryTimeout)
	defer cancel()
	req, err := http.NewRequestWithContext(ctx, http.MethodPost, r.baseURL+engineapi.Path(registryRoute), bytes.NewReader(body))
	if err != nil {
		return fmt.Errorf("engine_client: register: %w", err)
	}
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("X-Cyberia-Server-Api-Key", r.apiKey)
	resp, err := r.client.Do(req)
	if err != nil {
		return fmt.Errorf("engine_client: register: %w", err)
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return fmt.Errorf("engine_client: register: HTTP %d", resp.StatusCode)
	}
	return nil
}

// Start reports the process ready now and then every registryInterval.
func (r *Registry) Start() {
	if !r.enabled() {
		logx.Warnf("[Registry] disabled: CYBERIA_PUBLIC_URL or CYBERIA_SERVER_API_KEY unset")
		return
	}
	ctx, stop := context.WithCancel(context.Background())
	r.mu.Lock()
	r.stop = stop
	r.mu.Unlock()
	go func() {
		ticker := time.NewTicker(registryInterval)
		defer ticker.Stop()
		for {
			if err := r.send(ctx, r.next(StateReady)); err != nil && ctx.Err() == nil {
				logx.Warnf("[Registry] report failed: %v", err)
			}
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
			}
		}
	}()
}

// Drain stops the ready reports and reports the process draining. A ready
// report still in flight carries a lower sequence number, so the engine
// keeps the draining state.
func (r *Registry) Drain(ctx context.Context) error {
	if !r.enabled() {
		return nil
	}
	r.mu.Lock()
	if r.stop != nil {
		r.stop()
	}
	r.mu.Unlock()
	return r.send(ctx, r.next(StateDraining))
}
