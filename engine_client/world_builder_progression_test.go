package engine_client

import (
	"context"
	"testing"

	"cyberia-server/game"
	pb "cyberia-server/gen/proto"
	"google.golang.org/protobuf/proto"
)

type progressionDataSource struct {
	DataSource
	response *pb.GetFullInstanceResponse
}

func (source *progressionDataSource) FetchFullInstance(context.Context, string) (*pb.GetFullInstanceResponse, error) {
	return source.response, nil
}

func TestReloadRejectsInvalidStatsBeforeMutationAndAllowsRetry(t *testing.T) {
	server := game.NewGameServer()
	if err := server.ReplaceObjectLayerCache(map[string]*game.ObjectLayer{
		"sword": {Data: game.ObjectLayerData{Stats: game.Stats{Effect: -100}}},
	}); err != nil {
		t.Fatal(err)
	}
	source := &progressionDataSource{response: &pb.GetFullInstanceResponse{
		Version: "changed", Config: &pb.InstanceConfig{}, Instance: &pb.InstanceMessage{Code: "test"},
		ObjectLayers: []*pb.ObjectLayerMessage{{Item: &pb.ItemInfo{Id: "sword"}, Stats: &pb.Stats{Effect: 101}}},
	}}
	builder := NewWorldBuilder(source, server)
	builder.lastInstanceVersion = "current"
	if builder.ReloadWorld(context.Background()) == nil {
		t.Fatal("invalid reload accepted")
	}
	layer, _ := server.GetObjectLayerData("sword")
	if builder.lastInstanceVersion != "current" || layer.Data.Stats.Effect != -100 {
		t.Fatal("invalid reload changed state")
	}
	source.response.ObjectLayers[0].Stats.Effect = 100
	if err := builder.ReloadWorld(context.Background()); err != nil {
		t.Fatal(err)
	}
	layer, _ = server.GetObjectLayerData("sword")
	if builder.lastInstanceVersion != "changed" || layer.Data.Stats.Effect != 100 {
		t.Fatal("corrected reload was skipped")
	}
}

func TestProtobufPreservesSignedStatsAndLevels(t *testing.T) {
	want := &pb.GetFullInstanceResponse{
		Config:       &pb.InstanceConfig{ProgressionRules: &pb.ProgressionRules{MaxLevel: 100, XpPerLevel: 100}},
		ObjectLayers: []*pb.ObjectLayerMessage{{Stats: &pb.Stats{Effect: -100, Resistance: 100}}},
		Maps:         []*pb.MapDataMessage{{Entities: []*pb.EntityMessage{{Level: 7}}}},
	}
	data, err := proto.Marshal(want)
	if err != nil {
		t.Fatal(err)
	}
	got := &pb.GetFullInstanceResponse{}
	if err := proto.Unmarshal(data, got); err != nil || !proto.Equal(want, got) {
		t.Fatalf("protobuf round trip: %v, %v", err, got)
	}
}
