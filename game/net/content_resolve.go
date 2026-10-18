// Package net — content_resolve.go
//
// A player can arrive with stacks from another world of its universe. Before
// placement the server resolves each label its world does not hold through the
// engine, outside the world lock: the definition, its skill and the
// definitions the skill summons, from the release the world serves. The world
// runs that content whole, or holds the stacks of the label with the reason.
// Every client receives a definition and its skills once the cache holds them.
// Resolved content leaves the cache once nothing in the world holds or needs
// it, and a reload resolves it again, so no skill of another release stays.
package net

import (
	"context"
	"sync"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/persist"
	"cyberia-server/game/sim"
	"cyberia-server/logx"
)

const (
	// contentResolveBudget bounds the content one placement waits for.
	contentResolveBudget = 5 * time.Second
	// contentAttempts is how many times one fetch asks the engine, and
	// contentRetryWait the pause after a transport failure.
	contentAttempts  = 2
	contentRetryWait = 250 * time.Millisecond
)

// ContentSource resolves an item label to the content the engine serves for it.
type ContentSource interface {
	FetchItemContent(ctx context.Context, itemID string) (*content.ItemContent, error)
}

// resolveCall is one fetch in flight; every placement that needs the label waits on it.
type resolveCall struct {
	done    chan struct{}
	content *content.ItemContent
	err     error
}

// contentResolver fetches each label once at a time, whatever the number of placements that need it.
type contentResolver struct {
	source   ContentSource
	mu       sync.Mutex
	inflight map[string]*resolveCall
}

// SetContentSource enables the resolution of the labels a player brings. Call it before Run.
func (s *GameServer) SetContentSource(source ContentSource) {
	s.resolver = &contentResolver{source: source, inflight: make(map[string]*resolveCall)}
}

// FollowRelease makes release the content release of the world: resolved
// content of any other release does not run. Call it before Run.
func (s *GameServer) FollowRelease(release func() string) {
	s.release = release
}

// contentRelease is the content release the world serves, "" for the workspace.
func (s *GameServer) contentRelease() string {
	if s.release == nil {
		return ""
	}
	return s.release()
}

func (r *contentResolver) fetch(ctx context.Context, itemID string) (*content.ItemContent, error) {
	r.mu.Lock()
	call, joined := r.inflight[itemID]
	if !joined {
		call = &resolveCall{done: make(chan struct{})}
		r.inflight[itemID] = call
		go func() {
			fetchCtx, cancel := context.WithTimeout(context.Background(), contentResolveBudget)
			defer cancel()
			call.content, call.err = fetchWithRetry(fetchCtx, r.source, itemID)
			r.mu.Lock()
			delete(r.inflight, itemID)
			r.mu.Unlock()
			close(call.done)
		}()
	}
	r.mu.Unlock()
	select {
	case <-call.done:
		return call.content, call.err
	case <-ctx.Done():
		return nil, ctx.Err()
	}
}

// fetchWithRetry asks the engine again after a transport failure, while ctx allows.
func fetchWithRetry(ctx context.Context, source ContentSource, itemID string) (*content.ItemContent, error) {
	var err error
	for attempt := 0; attempt < contentAttempts; attempt++ {
		var c *content.ItemContent
		if c, err = source.FetchItemContent(ctx, itemID); err == nil {
			return c, nil
		}
		select {
		case <-ctx.Done():
			return nil, err
		case <-time.After(contentRetryWait):
		}
	}
	return nil, err
}

// savedLabels is every label a saved state names.
func savedLabels(state persist.State) []string {
	var labels []string
	add := func(stacks []persist.Stack) {
		for _, st := range stacks {
			labels = append(labels, st.ItemID)
		}
	}
	add(state.Bag)
	add(state.Storage)
	for _, v := range state.Vaults {
		add(v.Stacks)
	}
	for _, h := range state.Held {
		labels = append(labels, h.Stack.ItemID)
	}
	return labels
}

// resolveSaved fetches the content of the saved labels this world does not
// hold. It returns the content this world runs, and the hold reason of each
// label whose content it refuses. A transport failure is an error, and the
// placement must not go on.
func (s *GameServer) resolveSaved(ctx context.Context, state persist.State) (map[string]*content.ItemContent, map[string]string, error) {
	if s.resolver == nil {
		return nil, nil, nil
	}
	labels := savedLabels(state)
	missing := s.MissingObjectLayers(labels)
	s.recordContent("resident", len(distinct(labels))-len(missing))
	if len(missing) == 0 {
		return nil, nil, nil
	}
	ctx, cancel := context.WithTimeout(ctx, contentResolveBudget)
	defer cancel()
	resolved := make(map[string]*content.ItemContent, len(missing))
	unavailable := make(map[string]string)
	for _, itemID := range missing {
		c, err := s.resolver.fetch(ctx, itemID)
		if err != nil {
			s.recordContent("error", 1)
			return nil, nil, err
		}
		reason, outcome := content.ItemContentRefusal(itemID, c, s.contentRelease(), sim.SupportsSkill)
		s.recordContent(outcome, 1)
		if reason != "" {
			unavailable[itemID] = reason
			logx.Infof("[Content] %s does not run here (%s): its stacks stay held", itemID, outcome)
			continue
		}
		resolved[itemID] = c
	}
	return resolved, unavailable, nil
}

// RefreshResolvedContent resolves the content of every resolved label again,
// from the release the world serves now. A label whose content no longer runs
// here loses its skill; its definition stays while a player holds it.
func (s *GameServer) RefreshResolvedContent(ctx context.Context) {
	if s.resolver == nil {
		return
	}
	s.Mu.Lock()
	roots := s.ResolvedRoots()
	s.Mu.Unlock()
	refreshed := make(map[string]*content.ItemContent, len(roots))
	var dropped []string
	for _, root := range roots {
		c, err := s.resolver.fetch(ctx, root)
		if err == nil {
			if reason, _ := content.ItemContentRefusal(root, c, s.contentRelease(), sim.SupportsSkill); reason == "" {
				refreshed[root] = c
				continue
			}
		}
		dropped = append(dropped, root)
	}
	s.Mu.Lock()
	for _, root := range dropped {
		s.DropResolvedSkill(root)
	}
	s.AddResolvedContent(refreshed)
	s.Mu.Unlock()
	s.recordContent("refreshed", len(refreshed))
	s.recordContent("dropped", len(dropped))
	if len(dropped) > 0 {
		logx.Infof("[Content] the skills of %v no longer run here", dropped)
	}
}

// distinct is the set of the non-empty labels.
func distinct(labels []string) map[string]bool {
	set := make(map[string]bool, len(labels))
	for _, label := range labels {
		if label != "" {
			set[label] = true
		}
	}
	return set
}

// contentSweep is how often the resolved definitions nothing holds leave the cache.
const contentSweep = 30 * time.Second

// EvictResolvedContent drops the resolved definitions nothing in the world holds.
func (s *GameServer) EvictResolvedContent() {
	s.Mu.Lock()
	evicted := s.EvictResolvedObjectLayers()
	s.Mu.Unlock()
	s.recordDefinitionsEvicted(len(evicted))
	if len(evicted) > 0 {
		logx.Infof("[Content] evicted %d resolved definition(s) nothing holds: %v", len(evicted), evicted)
	}
}

// RunContentSweep evicts the resolved definitions nothing holds every
// contentSweep, until ctx ends.
func (s *GameServer) RunContentSweep(ctx context.Context) {
	ticker := time.NewTicker(contentSweep)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			s.EvictResolvedContent()
		}
	}
}
