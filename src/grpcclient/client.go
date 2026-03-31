// Package grpcclient wraps the generated CyberiaDataServiceClient with
// connection lifecycle management, automatic reconnection, and convenience
// methods that convert protobuf messages into the game server's domain types.
package grpcclient

import (
	"context"
	"fmt"
	"io"
	"log"
	"os"
	"sync"
	"time"

	pb "cyberia-server/proto"
	game "cyberia-server/src"

	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"
	"google.golang.org/grpc/keepalive"
)

// Config holds connection parameters for the Engine gRPC server.
// gRPC runs over the Kubernetes internal network (ClusterIP) — always insecure.
type Config struct {
	// Address is host:port of the Engine gRPC server (default "localhost:50051").
	Address string

	// ConnectTimeout is the dial deadline (default 10s).
	ConnectTimeout time.Duration

	// CallTimeout is the per-RPC deadline (default 30s).
	CallTimeout time.Duration
}

func (c *Config) defaults() {
	if c.Address == "" {
		if addr := os.Getenv("ENGINE_GRPC_ADDRESS"); addr != "" {
			c.Address = addr
		} else {
			c.Address = "localhost:50051"
		}
	}
	if c.ConnectTimeout == 0 {
		c.ConnectTimeout = 10 * time.Second
	}
	if c.CallTimeout == 0 {
		c.CallTimeout = 30 * time.Second
	}
}

// Client is a high-level wrapper around the generated gRPC client.
type Client struct {
	mu     sync.RWMutex
	cfg    Config
	conn   *grpc.ClientConn
	svc    pb.CyberiaDataServiceClient
	closed bool
}

// New creates a new Client and dials the Engine gRPC server.
func New(cfg Config) (*Client, error) {
	cfg.defaults()

	opts := []grpc.DialOption{
		grpc.WithDefaultCallOptions(
			grpc.MaxCallRecvMsgSize(64 * 1024 * 1024), // 64 MB
		),
		grpc.WithKeepaliveParams(keepalive.ClientParameters{
			Time:                30 * time.Second,
			Timeout:             10 * time.Second,
			PermitWithoutStream: true,
		}),
	}

	opts = append(opts, grpc.WithTransportCredentials(insecure.NewCredentials()))

	ctx, cancel := context.WithTimeout(context.Background(), cfg.ConnectTimeout)
	defer cancel()

	conn, err := grpc.DialContext(ctx, cfg.Address, opts...)
	if err != nil {
		return nil, fmt.Errorf("grpcclient: dial %s: %w", cfg.Address, err)
	}

	c := &Client{
		cfg:  cfg,
		conn: conn,
		svc:  pb.NewCyberiaDataServiceClient(conn),
	}
	log.Printf("gRPC client connected to Engine at %s", cfg.Address)
	return c, nil
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
	ctx, cancel := context.WithTimeout(ctx, c.cfg.CallTimeout)
	defer cancel()
	resp, err := c.svc.Ping(ctx, &pb.PingRequest{})
	if err != nil {
		return 0, err
	}
	return resp.GetServerTimeMs(), nil
}

// FetchObjectLayerBatch streams all ObjectLayers from Engine and converts
// them into the game server's *ObjectLayer type, keyed by item ID.
func (c *Client) FetchObjectLayerBatch(ctx context.Context, itemTypeFilter string) (map[string]*game.ObjectLayer, error) {
	ctx, cancel := context.WithTimeout(ctx, 5*time.Minute) // large dataset
	defer cancel()

	stream, err := c.svc.GetObjectLayerBatch(ctx, &pb.GetObjectLayerBatchRequest{
		ItemTypeFilter: itemTypeFilter,
	})
	if err != nil {
		return nil, fmt.Errorf("GetObjectLayerBatch: %w", err)
	}

	cache := make(map[string]*game.ObjectLayer)
	count := 0
	for {
		msg, err := stream.Recv()
		if err == io.EOF {
			break
		}
		if err != nil {
			return nil, fmt.Errorf("GetObjectLayerBatch stream recv: %w", err)
		}
		ol := protoToObjectLayer(msg)
		if ol.Data.Item.ID != "" {
			cache[ol.Data.Item.ID] = ol
		}
		count++
		if count%50 == 0 {
			log.Printf("gRPC: streamed %d ObjectLayers so far...", count)
		}
	}
	log.Printf("gRPC: received %d ObjectLayers total, %d cached.", count, len(cache))
	return cache, nil
}

// FetchObjectLayer fetches a single ObjectLayer by item ID.
func (c *Client) FetchObjectLayer(ctx context.Context, itemID string) (*game.ObjectLayer, error) {
	ctx, cancel := context.WithTimeout(ctx, c.cfg.CallTimeout)
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
	ctx, cancel := context.WithTimeout(ctx, c.cfg.CallTimeout)
	defer cancel()

	resp, err := c.svc.GetMapData(ctx, &pb.GetMapDataRequest{MapCode: mapCode, InstanceCode: instanceCode})
	if err != nil {
		return nil, fmt.Errorf("GetMapData(%s): %w", mapCode, err)
	}
	return resp.GetMap(), nil
}

// FetchAtlasSpriteSheet returns atlas metadata for an item key.
func (c *Client) FetchAtlasSpriteSheet(ctx context.Context, itemKey string) (*pb.AtlasSpriteSheetMessage, error) {
	ctx, cancel := context.WithTimeout(ctx, c.cfg.CallTimeout)
	defer cancel()

	resp, err := c.svc.GetAtlasSpriteSheet(ctx, &pb.GetAtlasSpriteSheetRequest{ItemKey: itemKey})
	if err != nil {
		return nil, fmt.Errorf("GetAtlasSpriteSheet(%s): %w", itemKey, err)
	}
	return resp, nil
}

// FetchAtlasSpriteSheetBatch streams all AtlasSpriteSheets and converts
// them into the game server's *AtlasData type, keyed by item key.
func (c *Client) FetchAtlasSpriteSheetBatch(ctx context.Context) (map[string]*game.AtlasData, error) {
	ctx, cancel := context.WithTimeout(ctx, 5*time.Minute)
	defer cancel()

	stream, err := c.svc.GetAtlasSpriteSheetBatch(ctx, &pb.GetAtlasSpriteSheetBatchRequest{})
	if err != nil {
		return nil, fmt.Errorf("GetAtlasSpriteSheetBatch: %w", err)
	}

	cache := make(map[string]*game.AtlasData)
	count := 0
	for {
		msg, err := stream.Recv()
		if err == io.EOF {
			break
		}
		if err != nil {
			return nil, fmt.Errorf("GetAtlasSpriteSheetBatch stream recv: %w", err)
		}
		ad := protoToAtlasData(msg)
		if ad.ItemKey != "" {
			cache[ad.ItemKey] = ad
		}
		count++
	}
	log.Printf("gRPC: received %d AtlasSpriteSheets, %d cached.", count, len(cache))
	return cache, nil
}

// ManifestEntry is a lightweight item-ID + hash pair for diffing.
type ManifestEntry struct {
	ItemID string
	Sha256 string
}

// FetchObjectLayerManifest returns the manifest of all item IDs + hashes.
func (c *Client) FetchObjectLayerManifest(ctx context.Context) ([]ManifestEntry, error) {
	ctx, cancel := context.WithTimeout(ctx, c.cfg.CallTimeout)
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
		ID:            msg.GetMongoId(),
		Sha256:        msg.GetSha256(),
		Cid:           msg.GetCid(),
		FrameDuration: int(msg.GetFrameDuration()),
		IsStateless:   msg.GetIsStateless(),
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

// protoToAtlasData converts an AtlasSpriteSheetMessage to a game.AtlasData.
func protoToAtlasData(msg *pb.AtlasSpriteSheetMessage) *game.AtlasData {
	ad := &game.AtlasData{
		FileID:       msg.GetFileId(),
		ItemKey:      msg.GetItemKey(),
		AtlasWidth:   int(msg.GetAtlasWidth()),
		AtlasHeight:  int(msg.GetAtlasHeight()),
		CellPixelDim: int(msg.GetCellPixelDim()),
	}
	if f := msg.GetFrames(); f != nil {
		ad.Frames = game.DirFrames{
			UpIdle:           protoFrameList(f.GetUpIdle()),
			DownIdle:         protoFrameList(f.GetDownIdle()),
			RightIdle:        protoFrameList(f.GetRightIdle()),
			LeftIdle:         protoFrameList(f.GetLeftIdle()),
			UpRightIdle:      protoFrameList(f.GetUpRightIdle()),
			DownRightIdle:    protoFrameList(f.GetDownRightIdle()),
			UpLeftIdle:       protoFrameList(f.GetUpLeftIdle()),
			DownLeftIdle:     protoFrameList(f.GetDownLeftIdle()),
			DefaultIdle:      protoFrameList(f.GetDefaultIdle()),
			UpWalking:        protoFrameList(f.GetUpWalking()),
			DownWalking:      protoFrameList(f.GetDownWalking()),
			RightWalking:     protoFrameList(f.GetRightWalking()),
			LeftWalking:      protoFrameList(f.GetLeftWalking()),
			UpRightWalking:   protoFrameList(f.GetUpRightWalking()),
			DownRightWalking: protoFrameList(f.GetDownRightWalking()),
			UpLeftWalking:    protoFrameList(f.GetUpLeftWalking()),
			DownLeftWalking:  protoFrameList(f.GetDownLeftWalking()),
			NoneIdle:         protoFrameList(f.GetNoneIdle()),
		}
	}
	return ad
}

func protoFrameList(frames []*pb.FrameMetadata) []game.FrameMeta {
	if len(frames) == 0 {
		return nil
	}
	out := make([]game.FrameMeta, len(frames))
	for i, f := range frames {
		out[i] = game.FrameMeta{
			X: int(f.GetX()), Y: int(f.GetY()),
			Width: int(f.GetWidth()), Height: int(f.GetHeight()),
			FrameIndex: int(f.GetFrameIndex()),
		}
	}
	return out
}
