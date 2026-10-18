package engine_client

import (
	"context"
	"testing"

	"cyberia-server/game/content"
	"cyberia-server/game/net"
	pb "cyberia-server/gen/proto"
)

// catalogDataSource answers the engine catalog of a hot reload: its manifest
// and the definitions it binds now. Each fetch is counted.
type catalogDataSource struct {
	DataSource
	manifest []ManifestEntry
	bound    map[string]*content.ObjectLayer
	fetched  []string
	release  string
}

func (c *catalogDataSource) Ping(context.Context) (int64, error) { return 1, nil }

func (c *catalogDataSource) FetchObjectLayerManifest(context.Context) ([]ManifestEntry, error) {
	return c.manifest, nil
}

func (c *catalogDataSource) FetchObjectLayer(_ context.Context, itemID string) (*content.ObjectLayer, error) {
	c.fetched = append(c.fetched, itemID)
	if ol, ok := c.bound[itemID]; ok {
		return ol, nil
	}
	return nil, content.ErrUnbound
}

func (c *catalogDataSource) FetchFullInstance(context.Context, string) (*pb.GetFullInstanceResponse, error) {
	return &pb.GetFullInstanceResponse{
		Version: "same", Config: &pb.InstanceConfig{}, Instance: &pb.InstanceMessage{Code: "test"}, ReleaseId: c.release,
	}, nil
}

func TestAFullReloadFollowsThePromotedReleaseEvenWhenTheWorldIsTheSame(t *testing.T) {
	source := &catalogDataSource{release: "r2"}
	builder := NewWorldBuilder(source, net.NewGameServer())
	builder.lastInstanceVersion, builder.releaseID = "same", "r1"
	if err := builder.ReloadWorld(context.Background()); err != nil {
		t.Fatal(err)
	}
	if builder.ReleaseID() != "r2" {
		t.Fatalf("the server reports the release it serves now: %q", builder.ReleaseID())
	}
}

func TestHotReloadRefreshesOnlyTheLabelsTheServerHolds(t *testing.T) {
	server := net.NewGameServer()
	if err := server.ReplaceObjectLayerCache(map[string]*content.ObjectLayer{
		"sword":   {Cid: "cid-sword-v1", Data: content.ObjectLayerData{Item: content.Item{ID: "sword"}}},
		"lantern": {Cid: "cid-lantern", Data: content.ObjectLayerData{Item: content.Item{ID: "lantern"}}},
		"shield":  {Cid: "cid-shield", Data: content.ObjectLayerData{Item: content.Item{ID: "shield"}}},
	}); err != nil {
		t.Fatal(err)
	}
	source := &catalogDataSource{
		manifest: []ManifestEntry{
			{ItemID: "sword", Cid: "cid-sword-v2"},
			{ItemID: "shield", Cid: "cid-shield"},
			{ItemID: "elsewhere", Cid: "cid-elsewhere"},
		},
		bound: map[string]*content.ObjectLayer{
			"sword": {Cid: "cid-sword-v2", Data: content.ObjectLayerData{Item: content.Item{ID: "sword"}}},
		},
	}
	builder := NewWorldBuilder(source, server)
	builder.lastInstanceVersion = "same"
	if err := builder.HotReload(context.Background()); err != nil {
		t.Fatal(err)
	}
	if len(source.fetched) != 1 || source.fetched[0] != "sword" {
		t.Fatalf("only the rebound label is fetched, never a label the world does not hold: %v", source.fetched)
	}
	manifest := server.ObjectLayerManifest()
	if manifest["sword"] != "cid-sword-v2" || manifest["shield"] != "cid-shield" {
		t.Fatalf("the rebound label takes its new definition: %v", manifest)
	}
	if _, ok := manifest["lantern"]; ok {
		t.Fatal("an unbound label leaves the cache")
	}
	if _, ok := manifest["elsewhere"]; ok {
		t.Fatal("a hot reload never preloads the catalog")
	}
}
