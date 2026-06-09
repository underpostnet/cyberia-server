// Package grpcclient wraps the generated CyberiaDataServiceClient with
// connection lifecycle management, automatic reconnection, and convenience
// methods that convert protobuf messages into the game server's domain types.
package grpcclient

import (
	"context"
	"fmt"
	"log"
	"sync"
	"time"

	pb "cyberia-server/gen/proto"
	game "cyberia-server/game"

	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"
	"google.golang.org/grpc/keepalive"
)

// Connection and per-RPC deadlines for the Engine gRPC server.
// gRPC runs over the Kubernetes internal network (ClusterIP) — always insecure.
const (
	dialTimeout = 10 * time.Second
	callTimeout = 30 * time.Second
)

// Client is a high-level wrapper around the generated gRPC client.
type Client struct {
	mu     sync.RWMutex
	conn   *grpc.ClientConn
	svc    pb.CyberiaDataServiceClient
	closed bool
}

// New dials the Engine gRPC server at address (host:port) and returns a
// connected Client. The address is required (resolved by package config).
func New(address string) (*Client, error) {
	opts := []grpc.DialOption{
		grpc.WithDefaultCallOptions(
			grpc.MaxCallRecvMsgSize(64 * 1024 * 1024), // 64 MB
		),
		grpc.WithKeepaliveParams(keepalive.ClientParameters{
			Time:                30 * time.Second,
			Timeout:             10 * time.Second,
			PermitWithoutStream: true,
		}),
		grpc.WithTransportCredentials(insecure.NewCredentials()),
	}

	ctx, cancel := context.WithTimeout(context.Background(), dialTimeout)
	defer cancel()

	conn, err := grpc.DialContext(ctx, address, opts...)
	if err != nil {
		return nil, fmt.Errorf("grpcclient: dial %s: %w", address, err)
	}

	log.Printf("gRPC client connected to Engine at %s", address)
	return &Client{
		conn: conn,
		svc:  pb.NewCyberiaDataServiceClient(conn),
	}, nil
}

// Close tears down the underlying gRPC connection.
func (c *Client) Close() error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.closed {
		return nil
	}
	c.closed = true
	return c.conn.Close()
}

// ═══════════════════════════════════════════════════════════════════
// RPC wrappers
// ═══════════════════════════════════════════════════════════════════

// Ping checks Engine liveness.
func (c *Client) Ping(ctx context.Context) (int64, error) {
	ctx, cancel := context.WithTimeout(ctx, callTimeout)
	defer cancel()
	resp, err := c.svc.Ping(ctx, &pb.PingRequest{})
	if err != nil {
		return 0, err
	}
	return resp.GetServerTimeMs(), nil
}

// FetchObjectLayer fetches a single ObjectLayer by item ID.
func (c *Client) FetchObjectLayer(ctx context.Context, itemID string) (*game.ObjectLayer, error) {
	ctx, cancel := context.WithTimeout(ctx, callTimeout)
	defer cancel()

	msg, err := c.svc.GetObjectLayer(ctx, &pb.GetObjectLayerRequest{ItemId: itemID})
	if err != nil {
		return nil, fmt.Errorf("GetObjectLayer(%s): %w", itemID, err)
	}
	return protoToObjectLayer(msg), nil
}

// FetchFullInstance returns the instance, maps, and referenced ObjectLayers.
func (c *Client) FetchFullInstance(ctx context.Context, instanceCode string) (*pb.GetFullInstanceResponse, error) {
	ctx, cancel := context.WithTimeout(ctx, 2*time.Minute)
	defer cancel()

	resp, err := c.svc.GetFullInstance(ctx, &pb.GetFullInstanceRequest{InstanceCode: instanceCode})
	if err != nil {
		return nil, fmt.Errorf("GetFullInstance(%s): %w", instanceCode, err)
	}
	return resp, nil
}

// FetchMapData returns map data for a single map code.
// instanceCode identifies the calling Go server instance so the Engine can
// update the global map-code registry for live tracking.
func (c *Client) FetchMapData(ctx context.Context, mapCode, instanceCode string) (*pb.MapDataMessage, error) {
	ctx, cancel := context.WithTimeout(ctx, callTimeout)
	defer cancel()

	resp, err := c.svc.GetMapData(ctx, &pb.GetMapDataRequest{MapCode: mapCode, InstanceCode: instanceCode})
	if err != nil {
		return nil, fmt.Errorf("GetMapData(%s): %w", mapCode, err)
	}
	return resp.GetMap(), nil
}

// ManifestEntry is a lightweight item-ID + hash pair for diffing.
type ManifestEntry struct {
	ItemID string
	Sha256 string
}

// FetchObjectLayerManifest returns the manifest of all item IDs + hashes.
func (c *Client) FetchObjectLayerManifest(ctx context.Context) ([]ManifestEntry, error) {
	ctx, cancel := context.WithTimeout(ctx, callTimeout)
	defer cancel()

	resp, err := c.svc.GetObjectLayerManifest(ctx, &pb.GetObjectLayerManifestRequest{})
	if err != nil {
		return nil, fmt.Errorf("GetObjectLayerManifest: %w", err)
	}

	entries := make([]ManifestEntry, 0, len(resp.GetEntries()))
	for _, e := range resp.GetEntries() {
		entries = append(entries, ManifestEntry{
			ItemID: e.GetItemId(),
			Sha256: e.GetSha256(),
		})
	}
	return entries, nil
}

// ═══════════════════════════════════════════════════════════════════
// Proto → domain converters
// ═══════════════════════════════════════════════════════════════════

func protoToObjectLayer(msg *pb.ObjectLayerMessage) *game.ObjectLayer {
	ol := &game.ObjectLayer{
		ID:     msg.GetMongoId(),
		Sha256: msg.GetSha256(),
		Cid:    msg.GetCid(),
		Data: game.ObjectLayerData{
			Stats: game.Stats{
				Effect:       int(msg.GetStats().GetEffect()),
				Resistance:   int(msg.GetStats().GetResistance()),
				Agility:      int(msg.GetStats().GetAgility()),
				Range:        int(msg.GetStats().GetRange()),
				Intelligence: int(msg.GetStats().GetIntelligence()),
				Utility:      int(msg.GetStats().GetUtility()),
			},
			Item: game.Item{
				ID:          msg.GetItem().GetId(),
				Type:        msg.GetItem().GetType(),
				Description: msg.GetItem().GetDescription(),
				Activable:   msg.GetItem().GetActivable(),
			},
		},
	}

	if l := msg.GetLedger(); l != nil {
		ol.Data.Ledger = &game.Ledger{
			Type:    l.GetType(),
			Address: l.GetAddress(),
		}
	}
	if r := msg.GetRender(); r != nil {
		ol.Data.Render = &game.Render{
			Cid:         r.GetCid(),
			MetadataCid: r.GetMetadataCid(),
		}
	}

	return ol
}


