package persist

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"testing"
	"time"
)

// fakeStore is an engine that applies a commit only above the last applied
// sequence of the epoch it holds, like the real one.
type fakeStore struct {
	mu        sync.Mutex
	epoch     uint64
	applied   uint64
	state     State
	commits   []CommitRequest
	fail      int   // transport failures before the next answer
	failWith  error // the error of those failures; a transport failure when nil
	refuse    int   // integrity refusals before the next answer
	released  []Lease
	ended     []Ended
	positions []Checkpoint
}

func (f *fakeStore) Commit(_ context.Context, req CommitRequest) (CommitResult, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.commits = append(f.commits, req)
	if f.fail > 0 {
		f.fail--
		if f.failWith != nil {
			return CommitResult{}, f.failWith
		}
		return CommitResult{}, errors.New("engine down")
	}
	if f.refuse > 0 {
		f.refuse--
		return CommitResult{Refusal: RefusalIntegrity}, nil
	}
	switch {
	case req.Epoch != f.epoch:
		return CommitResult{Refusal: RefusalStaleSession}, nil
	case req.Seq <= f.applied:
		return CommitResult{Duplicate: true}, nil
	}
	f.applied, f.state = req.Seq, req.State
	return CommitResult{}, nil
}

func (f *fakeStore) CommitJoint(context.Context, JointRequest) (JointResult, error) {
	return JointResult{}, errors.New("one session only")
}

func (f *fakeStore) Checkpoint(_ context.Context, cps []Checkpoint) ([]Ended, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.positions = append(f.positions, cps...)
	return f.ended, nil
}

func (f *fakeStore) Release(_ context.Context, lease Lease) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.released = append(f.released, lease)
	return nil
}

func (f *fakeStore) snapshot() (uint64, State, int, int) {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.applied, f.state, len(f.commits), len(f.released)
}

var lease = Lease{PlayerID: "p1", AccountID: "a1", Epoch: 7}

func running(t *testing.T, store *fakeStore) *Pipeline {
	t.Helper()
	p := New(store)
	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(cancel)
	go p.Run(ctx)
	return p
}

func eventually(t *testing.T, what string, cond func() bool) {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		if cond() {
			return
		}
		time.Sleep(2 * time.Millisecond)
	}
	t.Fatalf("timed out waiting for %s", what)
}

func TestCaptureCommitsOnlyNewGoods(t *testing.T) {
	store := &fakeStore{epoch: 7}
	p := running(t, store)
	p.Open(lease, State{Coins: 5})

	if mark := p.Capture("p1", State{Coins: 5, Position: Position{MapCode: "m", X: 3}}, false); mark.Seq != 0 {
		t.Fatalf("a moved position alone takes no sequence, got %+v", mark)
	}
	mark := p.Capture("p1", State{Coins: 9}, false)
	if mark != (Mark{Epoch: 7, Seq: 1}) {
		t.Fatalf("new goods take sequence 1 of the epoch, got %+v", mark)
	}
	eventually(t, "the commit", func() bool { return p.Committed("p1", mark) })
	if applied, state, _, _ := store.snapshot(); applied != 1 || state.Coins != 9 {
		t.Fatalf("the engine holds sequence 1 with 9 coins, got %d %+v", applied, state)
	}
}

func TestCommitsCoalesceToTheNewestState(t *testing.T) {
	store := &fakeStore{epoch: 7, fail: 1}
	p := New(store) // not running: the captures pile up first
	p.Open(lease, State{})
	for coins := uint32(1); coins <= 5; coins++ {
		p.Capture("p1", State{Coins: coins}, false)
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	go p.Run(ctx)

	eventually(t, "the newest state", func() bool { return p.Committed("p1", Mark{Epoch: 7, Seq: 5}) })
	_, state, sent, _ := store.snapshot()
	if state.Coins != 5 || sent != 2 {
		t.Fatalf("one failed send and one retry carry the newest state: coins=%d sent=%d", state.Coins, sent)
	}
}

func TestARetriedSequenceIsADuplicate(t *testing.T) {
	store := &fakeStore{epoch: 7, applied: 3}
	p := running(t, store)
	p.Open(lease, State{})
	for coins := uint32(1); coins <= 3; coins++ {
		p.Capture("p1", State{Coins: coins}, false)
	}
	eventually(t, "the answer", func() bool { return p.Committed("p1", Mark{Epoch: 7, Seq: 3}) })
	if p.Snapshot().Duplicates == 0 {
		t.Fatal("a sequence the engine already applied counts as a duplicate")
	}
}

func TestAStaleEpochFencesAndEndsTheSession(t *testing.T) {
	store := &fakeStore{epoch: 8}
	p := running(t, store)
	p.Open(lease, State{})
	p.Capture("p1", State{Coins: 1}, false)
	eventually(t, "the eviction", func() bool {
		ended := p.TakeEnded()
		return len(ended) == 1 && ended[0].Reason == EndedElsewhere
	})
	_, _, sent, _ := store.snapshot()
	p.Capture("p1", State{Coins: 2}, false)
	done, _ := p.Close("p1", State{Coins: 3})
	eventually(t, "the fenced session to leave", func() bool {
		select {
		case <-done:
			return true
		default:
			return false
		}
	})
	if _, _, after, released := store.snapshot(); after != sent || released != 0 {
		t.Fatalf("a fenced session writes and releases nothing more: sent %d→%d released %d", sent, after, released)
	}
}

func TestCloseCommitsTheFinalStateThenReleases(t *testing.T) {
	store := &fakeStore{epoch: 7}
	p := running(t, store)
	p.Open(lease, State{})
	final := State{Coins: 4, Position: Position{MapCode: "m", X: 1, Y: 2}}
	done, _ := p.Close("p1", final)
	<-done
	_, state, _, released := store.snapshot()
	if state.Coins != 4 || state.Position != final.Position || released != 1 {
		t.Fatalf("the final state with its position is applied before the release: %+v released=%d", state, released)
	}
	if !p.Committed("p1", Mark{Epoch: 7, Seq: 1}) {
		t.Fatal("a released session still answers for its applied states")
	}
	if _, ok := p.Unsettled("p1"); ok || p.Health("p1").Durability != Saved {
		t.Fatal("a released session leaves the pipeline")
	}
}

func TestFailingCommitsDegradeThePlayer(t *testing.T) {
	store := &fakeStore{epoch: 7, fail: 1 << 30}
	p := running(t, store)
	start := time.Now()
	p.now = func() time.Time { return start }
	p.Open(lease, State{})
	p.Capture("p1", State{Coins: 1}, false)
	eventually(t, "a failure", func() bool { return p.Snapshot().Retries > 0 })
	if d := p.Health("p1").Durability; d != Saving {
		t.Fatalf("one failure is not an outage: %s", d)
	}
	p.mu.Lock()
	p.now = func() time.Time { return start.Add(DegradedAfter + time.Second) }
	p.mu.Unlock()
	if p.Health("p1").Durability != Degraded || p.Snapshot().Unsaved != 1 {
		t.Fatal("failures past DegradedAfter degrade the player")
	}
}

// A state the database refuses as it stands is no transport failure: the
// session keeps it, stays live, retries slowly and saves once it is repaired.
func TestAnIntegrityRefusalWaitsForARepair(t *testing.T) {
	store := &fakeStore{epoch: 7, refuse: 1}
	p := running(t, store)
	p.Open(lease, State{})
	mark := p.Capture("p1", State{Coins: 3}, false)
	eventually(t, "the refusal", func() bool { return p.Health("p1").Durability == Recovery })
	p.mu.Lock()
	wait := p.records["p1"].nextTry.Sub(p.now())
	p.mu.Unlock()
	if wait < recoveryRetry/2-time.Second {
		t.Fatalf("a refused state waits for a repair, not a fast retry: next try in %v", wait)
	}
	if ended := p.TakeEnded(); len(ended) != 0 {
		t.Fatalf("an integrity refusal ends no session: %+v", ended)
	}
	if _, _, sent, _ := store.snapshot(); sent != 1 {
		t.Fatalf("one try until the next one is due, got %d", sent)
	}
	if s := p.Snapshot(); s.Integrity != 1 || s.Recovery != 1 || s.Retries != 0 {
		t.Fatalf("the snapshot counts the refusal and the session in recovery: %+v", s)
	}

	p.mu.Lock()
	p.records["p1"].nextTry = time.Time{}
	p.mu.Unlock()
	p.signal()
	eventually(t, "the repaired commit", func() bool { return p.Committed("p1", mark) })
	if p.Health("p1").Durability != Saved {
		t.Fatal("the session is saved again once the engine takes the state")
	}
}

func TestARejectedCallWaitsForARepair(t *testing.T) {
	store := &fakeStore{epoch: 7, fail: 1, failWith: fmt.Errorf("%w: wrong server key", ErrRejected)}
	p := running(t, store)
	p.Open(lease, State{})
	p.Capture("p1", State{Coins: 1}, false)
	eventually(t, "the rejection", func() bool { return p.Health("p1").Durability == Recovery })
	p.mu.Lock()
	defer p.mu.Unlock()
	if r := p.records["p1"]; r.failure != FailureRejected || r.nextTry.Sub(p.now()) < recoveryRetry/2-time.Second {
		t.Fatalf("a rejected call retries slowly: %s in %v", r.failure, r.nextTry.Sub(p.now()))
	}
}

// A reconnect on this server adopts what the closed session could not save:
// the whole state, quests and position, becomes the first commit of the new
// epoch, and the older epoch writes nothing more.
func TestANewEpochAdoptsTheUnsettledState(t *testing.T) {
	store := &fakeStore{epoch: 7, refuse: 1 << 30}
	p := running(t, store)
	p.Open(lease, State{})
	quest := Quest{Code: "q1", Status: "active"}
	p.Close("p1", State{Coins: 9, Quests: []Quest{quest}, Position: Position{MapCode: "m", X: 4}})
	eventually(t, "the refusal", func() bool { return p.Health("p1").Durability == Recovery })

	unsettled, ok := p.Unsettled("p1")
	if !ok || unsettled.Coins != 9 {
		t.Fatalf("the closed session holds its unsaved final state: %v %+v", ok, unsettled)
	}
	store.mu.Lock()
	store.epoch, store.refuse = 8, 0
	store.mu.Unlock()
	next := Lease{PlayerID: "p1", AccountID: "a1", Epoch: 8}
	p.Adopt(next, unsettled)
	eventually(t, "the adopted commit", func() bool { return p.Committed("p1", Mark{Epoch: 8, Seq: 1}) })

	store.mu.Lock()
	defer store.mu.Unlock()
	last := store.commits[len(store.commits)-1]
	if last.Epoch != 8 || last.State.Coins != 9 || !last.QuestsChanged || !last.PositionChanged {
		t.Fatalf("the first commit of the new epoch carries the whole adopted state: %+v", last)
	}
	for _, c := range store.commits {
		if c.Epoch == 7 && c.Seq > 1 {
			t.Fatalf("the older epoch writes nothing after the adoption: %+v", c)
		}
	}
	if s := p.Snapshot(); s.Adopted != 1 {
		t.Fatalf("the adoption is counted: %+v", s)
	}
}

func TestASessionClosedUnsavedReportsItAtOnce(t *testing.T) {
	store := &fakeStore{epoch: 7, fail: 1 << 30}
	p := running(t, store)
	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(cancel)
	go p.RunCheckpoints(ctx)
	p.Open(lease, State{})
	p.Close("p1", State{Coins: 2})
	eventually(t, "the closing checkpoint", func() bool {
		store.mu.Lock()
		defer store.mu.Unlock()
		return len(store.positions) > 0
	})
	store.mu.Lock()
	defer store.mu.Unlock()
	h := store.positions[0].Health
	if !h.Closing || h.Seq != 1 || h.DurableSeq != 0 || h.Durability == Saved {
		t.Fatalf("the checkpoint tells the engine the session closed with an unsaved state: %+v", h)
	}
}

func TestCheckpointsSendMovedPositionsAndEvict(t *testing.T) {
	store := &fakeStore{epoch: 7}
	p := New(store)
	p.Open(lease, State{Position: Position{MapCode: "m", X: 1}})
	p.Checkpoint(context.Background())
	p.Capture("p1", State{Position: Position{MapCode: "m", X: 2}}, false)
	p.Checkpoint(context.Background())
	if len(store.positions) != 2 || store.positions[1].Position == nil || store.positions[0].Position == nil {
		t.Fatalf("each checkpoint renews the lease and carries a moved position: %+v", store.positions)
	}
	if store.positions[1].PositionSeq <= store.positions[0].PositionSeq {
		t.Fatal("position sequences rise")
	}
	p.Checkpoint(context.Background())
	if store.positions[2].Position != nil {
		t.Fatal("an unmoved position is not sent again")
	}
	store.ended = []Ended{{PlayerID: "p1", Reason: EndedRevoked}}
	p.Checkpoint(context.Background())
	if ended := p.TakeEnded(); len(ended) != 1 || ended[0].Reason != EndedRevoked {
		t.Fatalf("a session the engine ended is evicted: %+v", ended)
	}
}

func TestCommittedHoldsAcrossEpochs(t *testing.T) {
	store := &fakeStore{epoch: 7}
	p := running(t, store)
	p.Open(lease, State{})
	mark := p.Capture("p1", State{Coins: 1}, false)
	done, _ := p.Close("p1", State{Coins: 1})
	<-done
	store.epoch = 8
	p.Open(Lease{PlayerID: "p1", AccountID: "a1", Epoch: 8}, State{Coins: 1})
	if !p.Committed("p1", mark) {
		t.Fatal("a mark of the released epoch stays committed")
	}
	if p.Committed("p1", Mark{Epoch: 8, Seq: 1}) {
		t.Fatal("the new epoch has applied nothing yet")
	}
}
