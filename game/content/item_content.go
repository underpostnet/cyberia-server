// Package content — item_content.go
//
// A player can bring an item the world does not warm. The world then takes
// the content of that item from the engine: its definition, its skill and the
// definitions the skill summons. It takes that content whole or not at all,
// and only from the release the world serves. The warm set wins: a label the
// world warms keeps the definition and the skill of the world.
package content

import (
	"maps"
	"slices"

	"cyberia-server/game/persist"
)

// ItemContent is the content the engine serves for one item label.
type ItemContent struct {
	// ReleaseID is the content release it comes from, "" for the workspace.
	ReleaseID string
	// Layers holds the definition of the label and of each label its skill summons.
	Layers map[string]*ObjectLayer
	// Skills holds the skills of the labels in Layers that have one.
	Skills map[string][]SkillDefinition
	// Unavailable is each label the engine cannot carry, with its reason.
	Unavailable map[string]string
}

// Outcomes of an item content, as the content metrics count them.
const (
	OutcomeResolved     = "resolved"
	OutcomeUnbound      = "unbound"
	OutcomeMismatch     = "mismatch"
	OutcomeUnavailable  = "unavailable"
	OutcomeIncompatible = "incompatible"
	OutcomeUnsupported  = "unsupported"
)

// ItemContentRefusal is why the world cannot run the content of label, as the
// hold reason of its stacks and the metric outcome, or "" and
// OutcomeResolved. release is the release of the world; supported tells
// whether the server runs a skill handler.
func ItemContentRefusal(label string, c *ItemContent, release string, supported func(string) bool) (string, string) {
	if c.Unavailable[label] == "unbound" {
		return persist.HeldUnknown, OutcomeUnbound
	}
	if c.ReleaseID != release {
		return persist.HeldIncompatible, OutcomeIncompatible
	}
	if len(c.Unavailable) > 0 {
		return persist.HeldUnknown, OutcomeUnavailable
	}
	root := c.Layers[label]
	if root == nil || root.Data.Item.ID != label || root.Cid == "" || validateObjectLayers(c.Layers) != nil {
		return persist.HeldUnknown, OutcomeMismatch
	}
	for _, defs := range c.Skills {
		for _, def := range defs {
			if !supported(def.LogicEventID) {
				return persist.HeldUnsupported, OutcomeUnsupported
			}
		}
	}
	return "", OutcomeResolved
}

// AddResolvedContent takes the content resolved for the labels a player
// brings. A label the cache holds keeps its definition, and a label the world
// warms keeps its skill. Caller holds s.mu, so no sweep runs before the player
// holds them.
func (s *Content) AddResolvedContent(contents map[string]*ItemContent) {
	if s.resolvedSkills == nil {
		s.resolvedSkills = make(map[string][]SkillDefinition)
		s.requires = make(map[string][]string)
	}
	for root, c := range contents {
		s.AddResolvedObjectLayers(c.Layers)
		maps.Copy(s.resolvedSkills, c.Skills)
		required := slices.Sorted(maps.Keys(c.Layers))
		s.requires[root] = slices.DeleteFunc(required, func(label string) bool { return label == root })
		delete(s.unavailable, root)
	}
	s.composeSkills()
}

// DropResolvedSkill removes the resolved skill of a label: its content no
// longer runs here. Its definition stays while a player holds it. Caller holds s.mu.
func (s *Content) DropResolvedSkill(label string) {
	delete(s.resolvedSkills, label)
	delete(s.requires, label)
	s.composeSkills()
}

// ResolvedRoots is each label resolved for a player, in label order. Caller holds s.mu.
func (s *Content) ResolvedRoots() []string {
	return slices.Sorted(maps.Keys(s.requires))
}

// MarkUnavailable records why the world holds the stacks of each label.
// Caller holds s.mu.
func (s *Content) MarkUnavailable(reasons map[string]string) {
	if s.unavailable == nil {
		s.unavailable = make(map[string]string)
	}
	maps.Copy(s.unavailable, reasons)
}

// composeSkills makes SkillConfig the resolved skills, overridden by the warm
// skills. A cached label whose skills changed counts a definition revision, so
// every client receives them. Caller holds s.mu.
func (s *Content) composeSkills() {
	skills := make(map[string][]SkillDefinition, len(s.warmSkills)+len(s.resolvedSkills))
	maps.Copy(skills, s.resolvedSkills)
	maps.Copy(skills, s.warmSkills)
	previous := s.SkillConfig
	s.SkillConfig = skills
	s.OlMu.Lock()
	defer s.OlMu.Unlock()
	for _, label := range slices.Sorted(maps.Keys(labelUnion(previous, skills))) {
		_, cached := s.ObjectLayerDataCache[label]
		if cached && !slices.Equal(previous[label], skills[label]) {
			s.olRevision++
			s.olChanged[label] = s.olRevision
		}
	}
}

// labelUnion is the set of the labels of two skill maps.
func labelUnion(a, b map[string][]SkillDefinition) map[string]bool {
	labels := make(map[string]bool, len(a)+len(b))
	for label := range a {
		labels[label] = true
	}
	for label := range b {
		labels[label] = true
	}
	return labels
}

// keepResolved adds to keep every label a kept label requires, transitively.
func (s *Content) keepResolved(keep map[string]bool) {
	queue := slices.Collect(maps.Keys(keep))
	for len(queue) > 0 {
		label := queue[0]
		queue = queue[1:]
		for _, required := range s.requires[label] {
			if !keep[required] {
				keep[required] = true
				queue = append(queue, required)
			}
		}
	}
}
