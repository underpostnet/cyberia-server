package persist

import (
	"context"
	"errors"
	"math/rand/v2"
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

// Refusals of a commit. Any other refusal of a single commit is a defect of
// the state.
const (
	RefusalStaleSession = "stale_session"
	RefusalUnknown      = "unknown"
	RefusalInvalid      = "invalid"
	// RefusalIntegrity: the database refuses the state as it stands. A retry
	// fails the same way until an operator repairs the records.
	RefusalIntegrity = "integrity"
)

// ErrRejected marks a call error a retry repeats: the engine rejects the call
// itself (the key, the method or an argument).
var ErrRejected = errors.New("rejected by the engine")

// Durability is how far the engine holds the state of a session.
type Durability string

const (
	Saved    Durability = "saved"    // the engine applied every captured change
	Saving   Durability = "saving"   // a change waits for its commit
	Degraded Durability = "degraded" // commits fail past DegradedAfter and retry
	Recovery Durability = "recovery" // the engine refuses the state; a slow retry waits for a repair
)

// Unsaved reports whether new goods could not be committed now.
func (d Durability) Unsaved() bool { return d == Degraded || d == Recovery }

// Failures of the commits of a session.
const (
	FailureTransport = "transport" // the call failed; a fast retry follows
	FailureIntegrity = "integrity" // the engine refuses the state as it stands
	FailureRejected  = "rejected"  // the engine rejects the call itself
)

// CommitResult is the answer of the engine to a commit. A refusal is final.
type CommitResult struct {
	Refusal   string
	Duplicate bool
}

// Checkpoint renews one lease, carries the position when it changed and
// reports the health of the session.
type Checkpoint struct {
	Lease
	MapCode     string
	Position    *Position
	PositionSeq uint64
	Health      Health
}

// Health is how far the engine holds the state of one session.
type Health struct {
	Durability Durability
	Closing    bool          // the session ended; its final state is on its way
	Seq        uint64        // the newest captured state
	DurableSeq uint64        // the newest state the engine applied
	UnsavedFor time.Duration // age of the oldest change not applied; 0 when saved
	Failure    string
}

// Unsaved reports whether new goods could not be committed now.
func (h Health) Unsaved() bool { return h.Durability.Unsaved() }

// Ended is a session the engine ended, with its reason.
type Ended struct {
	PlayerID string
	Reason   string
}

// Reasons a session ends from the engine side. A checkpoint may also name
// "ended": an operator ended the session.
const (
	EndedElsewhere = "signed_in_elsewhere"
	EndedRevoked   = "revoked"
	EndedInvalid   = "invalid_state"
)

// Store is the engine side of the pipeline. An error is a transport failure,
// worth a retry; a refusal is an answer.
type Store interface {
	Commit(ctx context.Context, req CommitRequest) (CommitResult, error)
	CommitJoint(ctx context.Context, req JointRequest) (JointResult, error)
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
	// recoveryRetry spaces the tries of a state the engine refuses as it stands.
	recoveryRetry = 30 * time.Second
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
	// failure is the class of the failure since failingSince, or "".
	failure  string
	nextTry  time.Time
	backoff  time.Duration
	closing  bool
	closedAt time.Time
	// evicted marks a live session the server must end.
	evicted bool
	// fenced is the reason the engine refused a commit, or "". A fenced
	// session writes nothing more.
	fenced string
	// link is the pending joint commit of the session, or nil (link.go).
	link *link
	done chan struct{}
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
	links    map[string]*link
	resolved map[string]resolved
	settled  map[settledKey]settled
	inFlight int
	endedQ   []Ended
	wake     chan struct{}
	// closed asks for a checkpoint now: a session closed with a change the
	// engine has not applied, and the engine must know it is closing.
	closed chan struct{}
	lease  time.Duration
	stats  Stats
}

// New returns an empty pipeline over store.
func New(store Store) *Pipeline {
	return &Pipeline{
		store:    store,
		now:      time.Now,
		records:  make(map[string]*record),
		links:    make(map[string]*link),
		resolved: make(map[string]resolved),
		settled:  make(map[settledKey]settled),
		wake:     make(chan struct{}, 1),
		closed:   make(chan struct{}, 1),
		lease:    60 * time.Second,
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
		if old.seq > old.durable && old.fenced == "" {
			p.stats.Superseded++
			logx.Errorf("[Persist] player=%s epoch %d replaced epoch %d before seq %d was saved", lease.PlayerID, lease.Epoch, old.lease.Epoch, old.seq)
		}
		p.drop(lease.PlayerID, old)
	}
	p.records[lease.PlayerID] = &record{lease: lease, state: loaded.canonical(), done: make(chan struct{})}
}

// Adopt starts a session with the unsettled state of the closed session it
// replaces (Unsettled): the whole state, quests and position included, is
// the first commit of the new epoch. The new epoch fences the writes of the
// replaced session at the engine.
func (p *Pipeline) Adopt(lease Lease, state State) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if old := p.records[lease.PlayerID]; old != nil {
		p.drop(lease.PlayerID, old)
	}
	p.stats.Adopted++
	p.records[lease.PlayerID] = &record{
		lease: lease, state: state.canonical(), seq: 1, questsV: 1, posV: 1,
		pendingSince: p.now(), done: make(chan struct{}),
	}
	p.signal()
}

// Unsettled is the newest state of a closed session of the profile that the
// engine has not applied, while no joint commit of it is pending and the
// engine has not fenced it. A new session of the profile on this server
// adopts it, so a reconnect loses nothing the old session could not save.
func (p *Pipeline) Unsettled(playerID string) (State, bool) {
	p.mu.Lock()
	defer p.mu.Unlock()
	r := p.records[playerID]
	if r == nil || !r.closing || r.fenced != "" || r.link != nil || r.seq <= r.durable {
		return State{}, false
	}
	return r.state, true
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
		p.closing(r)
		p.signal()
	}
	return r.done, Mark{Epoch: r.lease.Epoch, Seq: r.seq}
}

// closing marks a session closed and, while the engine has not applied its
// state, asks for a checkpoint that tells the engine. Caller holds p.mu.
func (p *Pipeline) closing(r *record) {
	r.closing = true
	r.closedAt = p.now()
	if r.seq > r.durable || r.link != nil {
		select {
		case p.closed <- struct{}{}:
		default:
		}
	}
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

// PendingLink is the channel that closes when the pending joint commit of a
// session of the account resolves, or nil when none is pending.
func (p *Pipeline) PendingLink(accountID string) <-chan struct{} {
	p.mu.Lock()
	defer p.mu.Unlock()
	for _, r := range p.records {
		if r.lease.AccountID == accountID && r.link != nil && r.link.outcome == Pending {
			return r.link.resolved
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

// Health is how far the engine holds the state of a session; Saved for a
// session the pipeline does not hold.
func (p *Pipeline) Health(playerID string) Health {
	p.mu.Lock()
	defer p.mu.Unlock()
	if r := p.records[playerID]; r != nil {
		return p.healthOf(r)
	}
	return Health{Durability: Saved}
}

// durabilityOf derives the durability of a record. Caller holds p.mu.
func (p *Pipeline) durabilityOf(r *record) Durability {
	switch {
	case r.failure == FailureIntegrity || r.failure == FailureRejected:
		return Recovery
	case r.failure != "" && p.now().Sub(r.failingSince) >= DegradedAfter:
		return Degraded
	case r.seq > r.durable || r.link != nil:
		return Saving
	}
	return Saved
}

// healthOf is the health a checkpoint reports. Caller holds p.mu.
func (p *Pipeline) healthOf(r *record) Health {
	h := Health{Durability: p.durabilityOf(r), Closing: r.closing, Seq: r.seq, DurableSeq: r.durable, Failure: r.failure}
	if h.Durability != Saved && !r.pendingSince.IsZero() {
		h.UnsavedFor = p.now().Sub(r.pendingSince)
	}
	return h
}

// classify names the class of a failed call or refusal and the wait before
// the next try. A transport failure backs off from retryMin to retryMax; a
// refusal of the state or a rejected call waits recoveryRetry. Each wait is
// jittered, so retries of many sessions spread out.
func classify(err error, refusal string, backoff time.Duration) (string, time.Duration) {
	failure, wait := FailureTransport, min(max(backoff*2, retryMin), retryMax)
	switch {
	case refusal == RefusalIntegrity:
		failure, wait = FailureIntegrity, recoveryRetry
	case errors.Is(err, ErrRejected):
		failure, wait = FailureRejected, recoveryRetry
	}
	return failure, wait
}

// jittered is a wait between half of d and d.
func jittered(d time.Duration) time.Duration {
	return d/2 + rand.N(d/2+1)
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
	for op, r := range p.resolved {
		if now.Sub(r.at) > settledMemory {
			delete(p.resolved, op)
		}
	}
	for id, r := range p.records {
		switch {
		case r.inflight || now.Before(r.nextTry):
		case r.link != nil:
			// The link commits it (sendLinks).
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
	p.sendLinks(ctx, now)
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
	if err != nil || result.Refusal == RefusalIntegrity {
		p.failed(r, req.Seq, err, result.Refusal)
		return
	}
	if r.failure != "" {
		logx.Infof("[Persist] player=%s saved again at seq=%d after %v (%s)", r.lease.PlayerID, req.Seq, p.now().Sub(r.failingSince), r.failure)
	}
	r.failingSince, r.failure, r.backoff, r.nextTry = time.Time{}, "", 0, time.Time{}
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

// failed records one failed commit and schedules the next try. The first
// failure of an incident and each change of its class log; the retries of
// one incident only count. Caller holds p.mu.
func (p *Pipeline) failed(r *record, seq uint64, err error, refusal string) {
	failure, wait := classify(err, refusal, r.backoff)
	switch failure {
	case FailureIntegrity:
		p.stats.Integrity++
	case FailureRejected:
		p.stats.Rejected++
	default:
		p.stats.Retries++
	}
	if r.failingSince.IsZero() {
		r.failingSince = p.now()
	}
	if failure != r.failure {
		if failure == FailureTransport {
			logx.Warnf("[Persist] commit player=%s seq=%d failed; retrying: %v", r.lease.PlayerID, seq, err)
		} else {
			logx.Errorf("[Persist] commit player=%s seq=%d %s; the state stays and retries every %v until the engine takes it: %v %s",
				r.lease.PlayerID, seq, failure, recoveryRetry, err, refusal)
		}
	}
	r.failure, r.backoff = failure, wait
	r.nextTry = p.now().Add(jittered(wait))
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
		_, r.backoff = classify(err, "", r.backoff)
		r.nextTry = p.now().Add(jittered(r.backoff))
		logx.Warnf("[Persist] release player=%s failed (retry in %v): %v", r.lease.PlayerID, r.backoff, err)
		return
	}
	if p.records[r.lease.PlayerID] == r {
		p.settled[settledKey{r.lease.PlayerID, r.lease.Epoch}] = settled{seq: r.durable, at: p.now()}
		p.drop(r.lease.PlayerID, r)
	}
}

// RunCheckpoints renews the leases, writes the moved positions and reports the
// health of every session every third of the lease, and at once after a
// session closed unsaved, until ctx ends.
func (p *Pipeline) RunCheckpoints(ctx context.Context) {
	for {
		p.mu.Lock()
		every := p.lease / 3
		p.mu.Unlock()
		select {
		case <-ctx.Done():
			return
		case <-time.After(every):
		case <-p.closed:
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
		c := Checkpoint{Lease: r.lease, MapCode: r.state.Position.MapCode, Health: p.healthOf(r)}
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
