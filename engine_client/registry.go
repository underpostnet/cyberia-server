// Package engine_client — registry.go
//
// Server registration. The game server reports itself to the Data Server
// registry, which is where the game client reads its websocket URL.
//
// The report is idempotent: the engine upserts on serverURL. A document
// expires after the engine's TTL, so the report repeats on a ticker.
package engine_client

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"strings"
	"time"

	"cyberia-server/logx"
)

const (
	registryPath = "/api/cyberia-server-registry"
	// The engine drops a server 180 s after its last report.
	registryInterval = 60 * time.Second
	registryTimeout  = 10 * time.Second
)

// Report is one server registration body.
type Report struct {
	ServerURL    string `json:"serverUrl"`
	InstanceCode string `json:"instanceCode"`
	Name         string `json:"name"`
}

// Register posts one report to the Data Server registry.
func Register(ctx context.Context, baseURL, apiKey string, report Report) error {
	body, err := json.Marshal(report)
	if err != nil {
		return fmt.Errorf("engine_client: register: %w", err)
	}
	ctx, cancel := context.WithTimeout(ctx, registryTimeout)
	defer cancel()

	req, err := http.NewRequestWithContext(ctx, http.MethodPost, strings.TrimRight(baseURL, "/")+registryPath, bytes.NewReader(body))
	if err != nil {
		return fmt.Errorf("engine_client: register: %w", err)
	}
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("X-Cyberia-Server-Api-Key", apiKey)

	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		return fmt.Errorf("engine_client: register: %w", err)
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return fmt.Errorf("engine_client: register: HTTP %d", resp.StatusCode)
	}
	return nil
}

// StartRegistry reports at startup and then every registryInterval. An empty
// serverURL or apiKey disables it: the server stays off the list.
func StartRegistry(ctx context.Context, baseURL, apiKey string, report Report) {
	if report.ServerURL == "" || apiKey == "" {
		logx.Warnf("[Registry] disabled: --game-server-public-url or CYBERIA_SERVER_API_KEY unset")
		return
	}
	go func() {
		ticker := time.NewTicker(registryInterval)
		defer ticker.Stop()
		for {
			if err := Register(ctx, baseURL, apiKey, report); err != nil {
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
