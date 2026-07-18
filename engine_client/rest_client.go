// Package engine_client — rest_client.go
//
// RestClient is the REST boot-fallback DataSource. It consumes the engine's
// /api/cyberia-instance/boot/* endpoints, which serve the same payloads as
// the gRPC CyberiaDataService, and decodes them into the generated proto
// types via protojson (the REST JSON uses the proto lowerCamelCase names).
package engine_client

import (
	"context"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"strings"
	"time"

	game "cyberia-server/game"
	pb "cyberia-server/gen/proto"

	"google.golang.org/protobuf/encoding/protojson"
	"google.golang.org/protobuf/proto"
)

const restBootPath = "/api/cyberia-instance/boot"

// RestClient implements DataSource over the engine REST boot endpoints.
type RestClient struct {
	base string
	http *http.Client
}

// NewRestClient targets the engine REST origin (ENGINE_API_BASE_URL).
// Per-call deadlines come from request contexts, not a client-wide timeout.
func NewRestClient(baseURL string) *RestClient {
	return &RestClient{
		base: strings.TrimRight(baseURL, "/"),
		http: &http.Client{},
	}
}

// Close satisfies DataSource; the HTTP client holds no persistent connection state to tear down.
func (c *RestClient) Close() error { return nil }

// restEnvelope is the engine REST response wrapper: {status, data|message}.
type restEnvelope struct {
	Status  string          `json:"status"`
	Message string          `json:"message"`
	Data    json.RawMessage `json:"data"`
}

var restUnmarshal = protojson.UnmarshalOptions{DiscardUnknown: true}

func (c *RestClient) getData(ctx context.Context, path string, timeout time.Duration) (json.RawMessage, error) {
	ctx, cancel := context.WithTimeout(ctx, timeout)
	defer cancel()

	req, err := http.NewRequestWithContext(ctx, http.MethodGet, c.base+restBootPath+path, nil)
	if err != nil {
		return nil, fmt.Errorf("engine_client: GET %s: %w", path, err)
	}
	resp, err := c.http.Do(req)
	if err != nil {
		return nil, fmt.Errorf("engine_client: GET %s: %w", path, err)
	}
	defer resp.Body.Close()

	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, fmt.Errorf("engine_client: GET %s: read body: %w", path, err)
	}
	var env restEnvelope
	if err := json.Unmarshal(body, &env); err != nil {
		return nil, fmt.Errorf("engine_client: GET %s: HTTP %d: %w", path, resp.StatusCode, err)
	}
	if resp.StatusCode != http.StatusOK || env.Status != "success" {
		return nil, fmt.Errorf("engine_client: GET %s: HTTP %d: %s", path, resp.StatusCode, env.Message)
	}
	return env.Data, nil
}

func (c *RestClient) getProto(ctx context.Context, path string, timeout time.Duration, msg proto.Message) error {
	data, err := c.getData(ctx, path, timeout)
	if err != nil {
		return err
	}
	if err := restUnmarshal.Unmarshal(data, msg); err != nil {
		return fmt.Errorf("engine_client: GET %s: decode: %w", path, err)
	}
	return nil
}

// Ping checks Engine liveness (boot/ping).
func (c *RestClient) Ping(ctx context.Context) (int64, error) {
	var msg pb.PingResponse
	if err := c.getProto(ctx, "/ping", callTimeout, &msg); err != nil {
		return 0, err
	}
	return msg.GetServerTimeMs(), nil
}

// FetchObjectLayer fetches a single ObjectLayer by item ID (boot/object-layer/:itemId).
func (c *RestClient) FetchObjectLayer(ctx context.Context, itemID string) (*game.ObjectLayer, error) {
	var msg pb.ObjectLayerMessage
	if err := c.getProto(ctx, "/object-layer/"+url.PathEscape(itemID), callTimeout, &msg); err != nil {
		return nil, err
	}
	return protoToObjectLayer(&msg), nil
}

// FetchFullInstance returns the instance, maps, and referenced ObjectLayers
// (boot/full-instance[/:instanceCode]).
func (c *RestClient) FetchFullInstance(ctx context.Context, instanceCode string) (*pb.GetFullInstanceResponse, error) {
	path := "/full-instance"
	if instanceCode != "" {
		path += "/" + url.PathEscape(instanceCode)
	}
	var msg pb.GetFullInstanceResponse
	if err := c.getProto(ctx, path, 2*time.Minute, &msg); err != nil {
		return nil, err
	}
	return &msg, nil
}

// FetchObjectLayerManifest returns the manifest of all item IDs + hashes
// (boot/object-layer-manifest).
func (c *RestClient) FetchObjectLayerManifest(ctx context.Context) ([]ManifestEntry, error) {
	var msg pb.GetObjectLayerManifestResponse
	if err := c.getProto(ctx, "/object-layer-manifest", callTimeout, &msg); err != nil {
		return nil, err
	}
	entries := make([]ManifestEntry, 0, len(msg.GetEntries()))
	for _, e := range msg.GetEntries() {
		entries = append(entries, ManifestEntry{ItemID: e.GetItemId(), Sha256: e.GetSha256()})
	}
	return entries, nil
}
