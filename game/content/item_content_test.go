package content

import (
	"slices"
	"testing"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
)

// gunContent is the content of a gun whose projectile summons a bullet.
func gunContent(release string) *ItemContent {
	return &ItemContent{
		ReleaseID: release,
		Layers:    map[string]*ObjectLayer{"gun": definition("gun", "cid-gun"), "bullet": definition("bullet", "cid-bullet")},
		Skills:    map[string][]SkillDefinition{"gun": {{LogicEventID: "projectile", SummonedEntityItemID: "bullet"}}},
	}
}

func supportsProjectile(logicEventID string) bool { return logicEventID == "projectile" }

func TestTheContentOfAnItemRunsWholeOrNotAtAll(t *testing.T) {
	cases := []struct {
		name    string
		change  func(c *ItemContent)
		reason  string
		outcome string
	}{
		{"runnable", func(c *ItemContent) {}, "", OutcomeResolved},
		{"unbound", func(c *ItemContent) { c.Unavailable = map[string]string{"gun": "unbound"} }, persist.HeldUnknown, OutcomeUnbound},
		{"another release", func(c *ItemContent) { c.ReleaseID = "r2" }, persist.HeldIncompatible, OutcomeIncompatible},
		{"a summon it cannot carry", func(c *ItemContent) { c.Unavailable = map[string]string{"bullet": "too_deep"} }, persist.HeldUnknown, OutcomeUnavailable},
		{"another label", func(c *ItemContent) { c.Layers["gun"] = definition("rifle", "cid-gun") }, persist.HeldUnknown, OutcomeMismatch},
		{"stats it cannot run", func(c *ItemContent) { c.Layers["bullet"].Data.Stats.Effect = 101 }, persist.HeldUnknown, OutcomeMismatch},
		{"an unknown handler", func(c *ItemContent) { c.Skills["gun"][0].LogicEventID = "teleport" }, persist.HeldUnsupported, OutcomeUnsupported},
	}
	for _, tc := range cases {
		c := gunContent("r1")
		tc.change(c)
		if reason, outcome := ItemContentRefusal("gun", c, "r1", supportsProjectile); reason != tc.reason || outcome != tc.outcome {
			t.Fatalf("%s: got (%q, %q), want (%q, %q)", tc.name, reason, outcome, tc.reason, tc.outcome)
		}
	}
}

func TestTheWarmSkillOfALabelWinsOverAResolvedOne(t *testing.T) {
	s := NewContent(&world.World{Maps: map[string]*world.MapState{}, Sessions: map[world.EntityID]*world.Session{}})
	s.warmSkills = map[string][]SkillDefinition{"bullet": {{LogicEventID: "doppelganger"}}}
	c := gunContent("")
	c.Skills["bullet"] = []SkillDefinition{{LogicEventID: "projectile"}}
	s.AddResolvedContent(map[string]*ItemContent{"gun": c})
	if s.SkillConfig["gun"][0].SummonedEntityItemID != "bullet" || s.SkillConfig["bullet"][0].LogicEventID != "doppelganger" {
		t.Fatalf("skills = %v", s.SkillConfig)
	}
	if !slices.Equal(s.ResolvedRoots(), []string{"gun"}) || !slices.Equal(s.requires["gun"], []string{"bullet"}) {
		t.Fatalf("roots = %v, requires = %v", s.ResolvedRoots(), s.requires)
	}
}

func TestClientsReceiveASkillThatChanged(t *testing.T) {
	s := NewContent(&world.World{Maps: map[string]*world.MapState{}, Sessions: map[world.EntityID]*world.Session{}})
	s.AddResolvedContent(map[string]*ItemContent{"gun": gunContent("")})
	_, seen := s.ObjectLayers()
	changed := gunContent("")
	changed.Skills["gun"][0].Name = "Burst"
	s.AddResolvedContent(map[string]*ItemContent{"gun": changed})
	if since, _ := s.ObjectLayersSince(seen); len(since) != 1 || since["gun"] == nil {
		t.Fatalf("a changed skill sends its label again: %v", since)
	}
	_, seen = s.ObjectLayers()
	s.AddResolvedContent(map[string]*ItemContent{"gun": changed})
	if since, _ := s.ObjectLayersSince(seen); len(since) != 0 {
		t.Fatalf("the same content sends nothing: %v", since)
	}
}

func TestResolvedContentStaysWhileAHeldItemNeedsIt(t *testing.T) {
	w := &world.World{Maps: map[string]*world.MapState{}, Sessions: map[world.EntityID]*world.Session{}}
	s := NewContent(w)
	s.AddResolvedContent(map[string]*ItemContent{"gun": gunContent("")})
	ms := world.NewMapState("test", 8, 8)
	w.Maps["test"] = ms
	ms.Spawn(1, world.EntityData{Kind: world.EntityPlayer}, world.Transform{}, world.Appearance{
		Layers: []world.ObjectLayerState{{ItemID: "gun", Quantity: 1}},
	})
	if evicted := s.EvictResolvedObjectLayers(); len(evicted) != 0 {
		t.Fatalf("the gun and the bullet its skill summons stay, evicted %v", evicted)
	}
	ms.Despawn(1)
	evicted := s.EvictResolvedObjectLayers()
	slices.Sort(evicted)
	if !slices.Equal(evicted, []string{"bullet", "gun"}) || s.SkillConfig["gun"] != nil || len(s.ResolvedRoots()) != 0 {
		t.Fatalf("once nothing holds the gun its content leaves: evicted %v, skills %v", evicted, s.SkillConfig)
	}
}

func TestAStackWhoseContentTheWorldRefusedIsHeldWithTheReason(t *testing.T) {
	s := NewContent(&world.World{Maps: map[string]*world.MapState{}, Sessions: map[world.EntityID]*world.Session{}})
	s.MarkUnavailable(map[string]string{"gun": persist.HeldUnsupported})
	if reason := s.holdReason(persist.Stack{ItemID: "gun", CID: "cid-gun"}); reason != persist.HeldUnsupported {
		t.Fatalf("reason = %q", reason)
	}
	if reason := s.holdReason(persist.Stack{ItemID: "ghost", CID: "cid-ghost"}); reason != persist.HeldUnknown {
		t.Fatalf("reason = %q", reason)
	}
	s.AddResolvedContent(map[string]*ItemContent{"gun": gunContent("")})
	if reason := s.holdReason(persist.Stack{ItemID: "gun", CID: "cid-gun"}); reason != "" {
		t.Fatalf("content resolved later loads the stack, got %q", reason)
	}
}
