package persist

import (
	"context"
	"sync"
	"time"

	"cyberia-server/logx"
)

// Lease names one gameplay session: the profile, its account and the epoch the
// engine gave the session. Every write carries it.
type Lease struct {
	PlayerID  string
	AccountID string
	Epoch     uint64
}

// Mark names one captured state: the session epoch and the sequence.
type Mark struct {
	Epoch, Seq uint64
}

// CommitRequest is one whole-state commit.
type CommitRequest struct {
	Lease
	Seq             uint64
	State           State
	QuestsChanged   bool // State.Quests is the whole quest set
	PositionChanged bool // State.Position is newer than the saved one
	PositionSeq     uint64
}

// Refusals of a commit. Any other refusal is a defect of the state.
const (
	RefusalStaleSession = "stale_session"
	RefusalUnknown      = "unknown"
)

// CommitResult is the answer of the engine to a commit. A refusal is final.
type CommitResult struct {
	Refusal   string
	Duplicate bool
}

// Checkpoint renews one lease and carries the position when it changed.
type Checkpoint struct {
	Lease
	MapCode     string
	Position    *Position
	PositionSeq uint64
}

// Ended is a session the engine ended, with its reason.
type Ended struct {
	PlayerID string
	Reason   string
}

// Reasons a session ends from the engine side.
const (
	EndedElsewhere = "signed_in_elsewhere"
	EndedRevoked   = "revoked"
	EndedInvalid   = "invalid_state"
)

// Store is the engine side of the pipeline. An error is a transport failure,
// worth a retry; a refusal is an answer.
type Store interface {
	Commit(ctx context.Context, req CommitRequest) (CommitResult, error)
	Checkpoint(ctx context.Context, checkpoints []Checkpoint) ([]Ended, error)
	Release(ctx context.Context, lease Lease) error
}

const (
	callTimeout = 5 * time.Second
	retryMin    = 200 * time.Millisecond
	retryMax    = 5 * time.Second
	// DegradedAfter is how long the commits of a player may fail before the
	// player is unsaved.
	DegradedAfter = 15 * time.Second
	// maxInFlight bounds the commits sent at once.
	maxInFlight = 16
	// finalWindow bounds how long a closed session retries its final state.
	finalWindow = 10 * time.Minute
	// settledMemory is how long a released session still answers Committed.
	settledMemory = 2 * time.Minute
)

// record is the pipeline state of one session.
type record struct {
	lease Lease
	state State  // newest capture, canonical
	seq   uint64 // sequence of state
	// durable is the newest sequence the engine applied.
	durable  uint64
	inflight bool
	// Quest and position changes are versioned, so a commit carries them until
	// one that carried them is applied.
	questsV, questsSentV, questsDurableV uint64
	posV, posSentV, posDurableV          uint64
	posSeq                               uint64
	checkpointed                         Position
	pendingSince                         time.Time
	failingSince                         time.Time
	nextTry                              time.Time
	backoff                              time.Duration
	closing                              bool
	closedAt                             time.Time
	// evicted marks a live session the server must end.
	evicted bool
	// fenced is the reason the engine refused a commit, or "". A fenced
	// session writes nothing more.
	fenced string
	done   chan struct{}
}

type settledKey struct {
	playerID string
	epoch    uint64
}

// settled is the last applied sequence of a session that left normally.
type settled struct {
	seq uint64
	at  time.Time
}

// Pipeline holds the durable side of every account session of one server.
type Pipeline struct {
	store Store
	now   func() time.Time

	mu       sync.Mutex
	records  map[string]*record
	settled  map[settledKey]settled
	inFlight int
	endedQ   []Ended
	wake     chan struct{}
	lease    time.Duration
	stats    Stats
}

// New returns an empty pipeline over store.
func New(store Store) *Pipeline {
	return &Pipeline{
		store:   store,
		now:     time.Now,
		records: make(map[string]*record),
		settled: make(map[settledKey]settled),
		wake:    make(chan struct{}, 1),
		lease:   60 * time.Second,
	}
}

func (p *Pipeline) signal() {
	select {
	case p.wake <- struct{}{}:
	default:
	}
}

// SetLease records the lease length the engine grants. Checkpoints run every
// third of it.
func (p *Pipeline) SetLease(lease time.Duration) {
	if lease <= 0 {
		return
	}
	p.mu.Lock()
	p.lease = lease
	p.mu.Unlock()
}

// Open starts a session with the state the engine loaded, which is durable at
// sequence 0.
func (p *Pipeline) Open(lease Lease, loaded State) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if old := p.records[lease.PlayerID]; old != nil {
		logx.Errorf("[Persist] player=%s opened while epoch %d was still pending", lease.PlayerID, old.lease.Epoch)
		p.drop(lease.PlayerID, old)
	}
	p.records[lease.PlayerID] = &record{lease: lease, state: loaded.canonical(), done: make(chan struct{})}
}

// Capture records the newest state of a live session and returns its mark.
// A state with new goods takes a new sequence and is queued; a new position
// alone waits for a checkpoint, unless commitPosition asks for it.
func (p *Pipeline) Capture(playerID string, state State, commitPosition bool) Mark {
	p.mu.Lock()
	defer p.mu.Unlock()
	r := p.records[playerID]
	if r == nil || r.closing {
		return Mark{}
	}
	p.take(r, state.canonical(), commitPosition)
	return Mark{Epoch: r.lease.Epoch, Seq: r.seq}
}

func (p *Pipeline) take(r *record, state State, commitPosition bool) {
	goods := !sameGoods(r.state, state)
	if goods {
		if !questsEqual(r.state, state) {
			r.questsV++
		}
	}
	if commitPosition && state.Position.MapCode != "" {
		r.posV++
	}
	r.state = state
	if goods || commitPosition {
		r.seq++
		if r.pendingSince.IsZero() {
			r.pendingSince = p.now()
		}
		p.signal()
	}
}

func questsEqual(a, b State) bool {
	return sameGoods(State{Quests: a.Quests}, State{Quests: b.Quests})
}

// Close ends a session with its final state, position included. The returned
// channel closes once the final state is applied and the lease released, or
// the engine fenced the session, or finalWindow passed; Committed with the
// returned mark tells which.
func (p *Pipeline) Close(playerID string, final State) (<-chan struct{}, Mark) {
	p.mu.Lock()
	defer p.mu.Unlock()
	r := p.records[playerID]
	if r == nil {
		done := make(chan struct{})
		close(done)
		return done, Mark{}
	}
	if !r.closing {
		if r.fenced == "" {
			p.take(r, final.canonical(), true)
		}
		r.closing = true
		r.closedAt = p.now()
		p.signal()
	}
	return r.done, Mark{Epoch: r.lease.Epoch, Seq: r.seq}
}

// Lease is the lease of a live session: open, neither closing nor fenced.
func (p *Pipeline) Lease(playerID string) (Lease, bool) {
	p.mu.Lock()
	defer p.mu.Unlock()
	r := p.records[playerID]
	if r == nil || r.closing || r.fenced != "" {
		return Lease{}, false
	}
	return r.lease, true
}

// PendingAccount is the done channel of a session of an account that is still
// in the pipeline, or nil.
func (p *Pipeline) PendingAccount(accountID string) <-chan struct{} {
	p.mu.Lock()
	defer p.mu.Unlock()
	for _, r := range p.records {
		if r.lease.AccountID == accountID {
			return r.done
		}
	}
	return nil
}

// Committed reports whether the engine applied the marked state of a session,
// or a later one.
func (p *Pipeline) Committed(playerID string, mark Mark) bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	if r := p.records[playerID]; r != nil && r.lease.Epoch == mark.Epoch {
		return r.durable >= mark.Seq
	}
	s, ok := p.settled[settledKey{playerID, mark.Epoch}]
	return ok && s.seq >= mark.Seq
}

// Unsaved reports whether the commits of a live session fail for longer than
// DegradedAfter.
func (p *Pipeline) Unsaved(playerID string) bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	r := p.records[playerID]
	return r != nil && !r.failingSince.IsZero() && p.now().Sub(r.failingSince) >= DegradedAfter
}

// TakeEnded returns and clears the live sessions the server must end.
func (p *Pipeline) TakeEnded() []Ended {
	p.mu.Lock()
	defer p.mu.Unlock()
	out := p.endedQ
	p.endedQ = nil
	return out
}

// Flush waits until every closed session left the pipeline, or ctx ends. It
// returns how many are left.
func (p *Pipeline) Flush(ctx context.Context) int {
	for {
		p.mu.Lock()
		var waits []chan struct{}
		for _, r := range p.records {
			if r.closing {
				waits = append(waits, r.done)
			}
		}
		p.mu.Unlock()
		if len(waits) == 0 {
			return 0
		}
		select {
		case <-waits[0]:
		case <-ctx.Done():
			return len(waits)
		}
	}
}

// Run sends commits and releases until ctx ends.
func (p *Pipeline) Run(ctx context.Context) {
	timer := time.NewTimer(retryMin)
	defer timer.Stop()
	for {
		p.dispatch(ctx)
		timer.Reset(retryMin)
		select {
		case <-ctx.Done():
			return
		case <-p.wake:
		case <-timer.C:
		}
	}
}

// dispatch starts the work every record is due for.
func (p *Pipeline) dispatch(ctx context.Context) {
	p.mu.Lock()
	defer p.mu.Unlock()
	now := p.now()
	for key, s := range p.settled {
		if now.Sub(s.at) > settledMemory {
			delete(p.settled, key)
		}
	}
	for id, r := range p.records {
		switch {
		case r.inflight || now.Before(r.nextTry):
		case r.fenced != "" && r.closing:
			p.drop(id, r)
		case r.fenced != "":
		case r.closing && now.Sub(r.closedAt) > finalWindow:
			logx.Errorf("[Persist] player=%s final state not saved after %v; seq %d lost", id, finalWindow, r.seq)
			p.stats.LostFinal++
			p.drop(id, r)
		case r.seq > r.durable && p.inFlight < maxInFlight:
			p.send(ctx, r)
		case r.seq == r.durable && r.closing:
			r.inflight = true
			go p.release(ctx, r)
		}
	}
}

func (p *Pipeline) drop(id string, r *record) {
	delete(p.records, id)
	close(r.done)
}

// send starts one commit of the newest state. Caller holds p.mu.
func (p *Pipeline) send(ctx context.Context, r *record) {
	req := CommitRequest{
		Lease:           r.lease,
		Seq:             r.seq,
		State:           r.state,
		QuestsChanged:   r.questsV > r.questsDurableV,
		PositionChanged: r.posV > r.posDurableV,
	}
	if req.PositionChanged {
		r.posSeq++
		req.PositionSeq = r.posSeq
	}
	r.questsSentV, r.posSentV = r.questsV, r.posV
	r.inflight = true
	p.inFlight++
	sent := p.now()
	go func() {
		callCtx, cancel := context.WithTimeout(ctx, callTimeout)
		result, err := p.store.Commit(callCtx, req)
		cancel()
		p.committed(r, req, result, err, p.now().Sub(sent))
	}()
}

// committed applies the answer to one commit.
func (p *Pipeline) committed(r *record, req CommitRequest, result CommitResult, err error, took time.Duration) {
	p.mu.Lock()
	defer p.mu.Unlock()
	r.inflight = false
	p.inFlight--
	p.stats.observe(took)
	if err != nil {
		p.stats.Retries++
		if r.failingSince.IsZero() {
			r.failingSince = p.now()
		}
		r.backoff = min(max(r.backoff*2, retryMin), retryMax)
		r.nextTry = p.now().Add(r.backoff)
		logx.Warnf("[Persist] commit player=%s seq=%d failed (retry in %v): %v", r.lease.PlayerID, req.Seq, r.backoff, err)
		return
	}
	r.failingSince, r.backoff, r.nextTry = time.Time{}, 0, time.Time{}
	switch {
	case result.Refusal == RefusalStaleSession || result.Refusal == RefusalUnknown:
		p.stats.Stale++
		logx.Warnf("[Persist] commit player=%s seq=%d refused: %s", r.lease.PlayerID, req.Seq, result.Refusal)
		p.fence(r, EndedElsewhere)
		return
	case result.Refusal != "":
		// A defect of the state never heals by a retry; the session ends so no
		// further change is lost silently.
		p.stats.Invalid++
		logx.Errorf("[Persist] commit player=%s seq=%d refused as %s; ending the session", r.lease.PlayerID, req.Seq, result.Refusal)
		p.fence(r, EndedInvalid)
		return
	case result.Duplicate:
		p.stats.Duplicates++
	default:
		p.stats.Commits++
	}
	r.durable = max(r.durable, req.Seq)
	if r.durable >= r.seq {
		r.pendingSince = time.Time{}
	}
	r.questsDurableV = max(r.questsDurableV, r.questsSentV)
	r.posDurableV = max(r.posDurableV, r.posSentV)
	if req.PositionChanged {
		r.checkpointed = req.State.Position
	}
	p.signal()
}

// evict asks the server to end a live session. Its final state still
// commits. Caller holds p.mu.
func (p *Pipeline) evict(r *record, reason string) {
	if r.closing || r.evicted {
		return
	}
	r.evicted = true
	p.endedQ = append(p.endedQ, Ended{PlayerID: r.lease.PlayerID, Reason: reason})
}

// fence stops every write of a session the engine refused, and ends it.
// Caller holds p.mu.
func (p *Pipeline) fence(r *record, reason string) {
	if r.fenced != "" {
		return
	}
	r.fenced = reason
	p.evict(r, reason)
	p.signal()
}

// release ends the lease of a closed session whose final state is applied.
func (p *Pipeline) release(ctx context.Context, r *record) {
	callCtx, cancel := context.WithTimeout(ctx, callTimeout)
	err := p.store.Release(callCtx, r.lease)
	cancel()
	p.mu.Lock()
	defer p.mu.Unlock()
	r.inflight = false
	if err != nil {
		r.backoff = min(max(r.backoff*2, retryMin), retryMax)
		r.nextTry = p.now().Add(r.backoff)
		logx.Warnf("[Persist] release player=%s failed (retry in %v): %v", r.lease.PlayerID, r.backoff, err)
		return
	}
	if p.records[r.lease.PlayerID] == r {
		p.settled[settledKey{r.lease.PlayerID, r.lease.Epoch}] = settled{seq: r.durable, at: p.now()}
		p.drop(r.lease.PlayerID, r)
	}
}

// RunCheckpoints renews the leases and writes the moved positions every third
// of the lease, until ctx ends.
func (p *Pipeline) RunCheckpoints(ctx context.Context) {
	for {
		p.mu.Lock()
		every := p.lease / 3
		p.mu.Unlock()
		select {
		case <-ctx.Done():
			return
		case <-time.After(every):
		}
		p.Checkpoint(ctx)
	}
}

// Checkpoint sends one checkpoint of every session the engine has not fenced.
func (p *Pipeline) Checkpoint(ctx context.Context) {
	p.mu.Lock()
	var batch []Checkpoint
	for _, r := range p.records {
		if r.fenced != "" {
			continue
		}
		c := Checkpoint{Lease: r.lease, MapCode: r.state.Position.MapCode}
		if pos := r.state.Position; pos.MapCode != "" && pos != r.checkpointed {
			r.posSeq++
			c.Position, c.PositionSeq = &pos, r.posSeq
		}
		batch = append(batch, c)
	}
	p.mu.Unlock()
	if len(batch) == 0 {
		return
	}

	callCtx, cancel := context.WithTimeout(ctx, callTimeout)
	ended, err := p.store.Checkpoint(callCtx, batch)
	cancel()

	p.mu.Lock()
	defer p.mu.Unlock()
	if err != nil {
		p.stats.CheckpointFailures++
		logx.Warnf("[Persist] checkpoint of %d session(s) failed: %v", len(batch), err)
		return
	}
	p.stats.Checkpoints++
	p.stats.LastCheckpoint = p.now()
	gone := make(map[string]string, len(ended))
	for _, e := range ended {
		gone[e.PlayerID] = e.Reason
	}
	for _, c := range batch {
		r := p.records[c.PlayerID]
		if r == nil || r.lease != c.Lease {
			continue
		}
		if reason, ok := gone[c.PlayerID]; ok {
			p.evict(r, reason)
			continue
		}
		if c.Position != nil {
			p.stats.Positions++
			r.checkpointed = *c.Position
		}
	}
}
