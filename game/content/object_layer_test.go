package content

import (
	"slices"
	"testing"

	"cyberia-server/game/world"
)

func definition(itemID, cid string) *ObjectLayer {
	return &ObjectLayer{Cid: cid, Data: ObjectLayerData{Item: Item{ID: itemID}}}
}

func TestResolvedDefinitionsLeaveOnlyWhenNothingHoldsThem(t *testing.T) {
	w := &world.World{Maps: map[string]*world.MapState{}, Sessions: map[world.EntityID]*world.Session{}}
	s := NewContent(w)
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{"sword": definition("sword", "cid-sword")}); err != nil {
		t.Fatal(err)
	}
	s.AddResolvedObjectLayers(map[string]*ObjectLayer{
		"lantern": definition("lantern", "cid-lantern"),
		"mask":    definition("mask", "cid-mask"),
		"gem":     definition("gem", "cid-gem"),
		"sword":   definition("sword", "cid-other"),
		"broken":  {Cid: "cid-broken", Data: ObjectLayerData{Item: Item{ID: "broken"}, Stats: Stats{Effect: 101}}},
	})
	if ol, _ := s.GetObjectLayerData("sword"); ol.Cid != "cid-sword" {
		t.Fatal("a resolved definition never replaces a cached one")
	}
	if _, ok := s.GetObjectLayerData("broken"); ok {
		t.Fatal("a definition this server cannot run is refused")
	}
	if missing := s.MissingObjectLayers([]string{"sword", "lantern", "relic", "relic", ""}); !slices.Equal(missing, []string{"relic"}) {
		t.Fatalf("missing = %v, want [relic]", missing)
	}

	ms := world.NewMapState("test", 8, 8)
	w.Maps["test"] = ms
	ms.Spawn(1, world.EntityData{Kind: world.EntityPlayer}, world.Transform{}, world.Appearance{
		Layers: []world.ObjectLayerState{{ItemID: "lantern", Quantity: 1}},
	})
	s.PersonalStorage = map[world.PlayerID][]Stack{"p1": {{ItemID: "gem", Qty: 1}}}

	evicted := s.EvictResolvedObjectLayers()
	if !slices.Equal(evicted, []string{"mask"}) {
		t.Fatalf("evicted = %v, want [mask]", evicted)
	}
	for _, itemID := range []string{"sword", "lantern", "gem"} {
		if _, ok := s.GetObjectLayerData(itemID); !ok {
			t.Fatalf("%s stays: the world names it, or something holds it", itemID)
		}
	}
	if manifest := s.ObjectLayerManifest(); len(manifest) != 3 || manifest["lantern"] != "cid-lantern" {
		t.Fatalf("manifest = %v", manifest)
	}
}

func TestARebuiltWorldMakesItsLabelsWarm(t *testing.T) {
	s := NewContent(&world.World{Maps: map[string]*world.MapState{}, Sessions: map[world.EntityID]*world.Session{}})
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{"sword": definition("sword", "cid-sword")}); err != nil {
		t.Fatal(err)
	}
	s.AddResolvedObjectLayers(map[string]*ObjectLayer{"lantern": definition("lantern", "cid-lantern")})
	if err := s.ReplaceWarmObjectLayers(map[string]*ObjectLayer{"lantern": definition("lantern", "cid-lantern")}); err != nil {
		t.Fatal(err)
	}
	if evicted := s.EvictResolvedObjectLayers(); !slices.Equal(evicted, []string{"sword"}) {
		t.Fatalf("evicted = %v: the world no longer names sword, and lantern is warm now", evicted)
	}
}

func TestClientsReceiveTheDefinitionsThatChanged(t *testing.T) {
	s := NewContent(&world.World{Maps: map[string]*world.MapState{}, Sessions: map[world.EntityID]*world.Session{}})
	if err := s.ReplaceObjectLayerCache(map[string]*ObjectLayer{"sword": definition("sword", "cid-sword")}); err != nil {
		t.Fatal(err)
	}
	all, joined := s.ObjectLayers()
	if len(all) != 1 || joined == 0 {
		t.Fatalf("a join receives the whole cache: %v at %d", all, joined)
	}
	if err := s.PatchObjectLayerCache(map[string]*ObjectLayer{"sword": definition("sword", "cid-sword")}, nil); err != nil {
		t.Fatal(err)
	}
	if s.ObjectLayerRevision() != joined {
		t.Fatal("a refresh with the same definition is no change")
	}
	if err := s.PatchObjectLayerCache(map[string]*ObjectLayer{"sword": definition("sword", "cid-sword-2")}, nil); err != nil {
		t.Fatal(err)
	}
	changed, rebound := s.ObjectLayersSince(joined)
	if len(changed) != 1 || changed["sword"].Cid != "cid-sword-2" {
		t.Fatalf("a new definition of a label is a change: %v", changed)
	}
	s.AddResolvedObjectLayers(map[string]*ObjectLayer{"lantern": definition("lantern", "cid-lantern")})
	if changed, _ := s.ObjectLayersSince(rebound); len(changed) != 1 || changed["lantern"] == nil {
		t.Fatalf("a resolved definition is a change: %v", changed)
	}
	if s.ResolvedObjectLayerCount() != 1 {
		t.Fatal("the lantern is the one definition outside the warm set")
	}
	if evicted := s.EvictResolvedObjectLayers(); !slices.Equal(evicted, []string{"lantern"}) {
		t.Fatalf("evicted = %v, want [lantern]", evicted)
	}
	if changed, _ := s.ObjectLayersSince(rebound); len(changed) != 0 || s.ResolvedObjectLayerCount() != 0 {
		t.Fatalf("an evicted label is no change to send: %v", changed)
	}
}
