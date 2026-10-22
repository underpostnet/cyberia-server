package persist

import (
	"context"
	"errors"
	"sync"
	"testing"
)

// jointStore is an engine of several profiles. It applies a commit only above
// the last applied sequence of the epoch it holds, and a joint commit whole or
// not at all, like the real one.
type jointStore struct {
	mu      sync.Mutex
	epochs  map[string]uint64
	applied map[string]uint64
	states  map[string]State
	singles []CommitRequest
	joints  []JointRequest
	gone    map[string]bool // world drops a pickup finds claimed
	fail    int             // transport failures before the next joint answer
	block   bool            // joint commits wait until it clears
	release chan struct{}
}

func newJointStore() *jointStore {
	return &jointStore{
		epochs:  map[string]uint64{"a": 7, "b": 9},
		applied: map[string]uint64{},
		gone:    map[string]bool{},
		states:  map[string]State{},
		release: make(chan struct{}),
	}
}

func (f *jointStore) Commit(_ context.Context, req CommitRequest) (CommitResult, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.singles = append(f.singles, req)
	switch {
	case req.Epoch != f.epochs[req.PlayerID]:
		return CommitResult{Refusal: RefusalStaleSession}, nil
	case req.Seq <= f.applied[req.PlayerID]:
		return CommitResult{Duplicate: true}, nil
	}
	f.applied[req.PlayerID], f.states[req.PlayerID] = req.Seq, req.State
	return CommitResult{}, nil
}

func (f *jointStore) CommitJoint(ctx context.Context, req JointRequest) (JointResult, error) {
	f.mu.Lock()
	block := f.block
	f.mu.Unlock()
	if block {
		select {
		case <-f.release:
		case <-ctx.Done():
			return JointResult{}, ctx.Err()
		}
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	f.joints = append(f.joints, req)
	if f.fail > 0 {
		f.fail--
		return JointResult{}, errors.New("engine down")
	}
	if f.gone[req.Effect.Claim] {
		return JointResult{Refusal: RefusalDropGone, PlayerID: req.Commits[0].PlayerID}, nil
	}
	duplicates := 0
	for _, c := range req.Commits {
		if c.Epoch != f.epochs[c.PlayerID] {
			return JointResult{Refusal: RefusalStaleSession, PlayerID: c.PlayerID}, nil
		}
		if c.Seq <= f.applied[c.PlayerID] {
			duplicates++
		}
	}
	if duplicates == len(req.Commits) {
		return JointResult{Duplicate: true}, nil
	}
	for _, c := range req.Commits {
		f.applied[c.PlayerID], f.states[c.PlayerID] = c.Seq, c.State
	}
	return JointResult{}, nil
}

func (f *jointStore) Checkpoint(context.Context, []Checkpoint) ([]Ended, error) { return nil, nil }
func (f *jointStore) Release(context.Context, Lease) error                      { return nil }

func (f *jointStore) coins(playerID string) uint32 {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.states[playerID].Coins
}

func (f *jointStore) counts() (int, int) {
	f.mu.Lock()
	defer f.mu.Unlock()
	return len(f.singles), len(f.joints)
}

func openPair(t *testing.T, store *jointStore) *Pipeline {
	t.Helper()
	p := New(store)
	p.Open(Lease{PlayerID: "a", AccountID: "acc-a", Epoch: 7}, State{Coins: 10})
	p.Open(Lease{PlayerID: "b", AccountID: "acc-b", Epoch: 9}, State{Coins: 20})
	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(cancel)
	go p.Run(ctx)
	return p
}

func outcome(p *Pipeline, op string) Outcome {
	o, _ := p.LinkOutcome(op)
	return o
}

func pair(a, b uint32) []LinkMember {
	return []LinkMember{{PlayerID: "a", State: State{Coins: a}}, {PlayerID: "b", State: State{Coins: b}}}
}

// A link commits both states in one call and no state alone.
func TestALinkCommitsBothStatesTogether(t *testing.T) {
	store := newJointStore()
	p := openPair(t, store)
	if !p.Link("op-1", "trade", pair(6, 24), Effect{}) {
		t.Fatal("two live sessions link")
	}
	eventually(t, "the joint commit", func() bool { return outcome(p, "op-1") == Applied })
	if store.coins("a") != 6 || store.coins("b") != 24 {
		t.Fatalf("both post-trade states apply: a=%d b=%d", store.coins("a"), store.coins("b"))
	}
	if singles, joints := store.counts(); singles != 0 || joints != 1 {
		t.Fatalf("one joint commit and no single one: singles=%d joints=%d", singles, joints)
	}
	p.Unlink("op-1")
	p.Capture("a", State{Coins: 8}, false)
	eventually(t, "the single commit after the link", func() bool { return store.coins("a") == 8 })
}

// While a link waits, a newer state of a member never commits alone: it would
// undo the trade the link carries.
func TestAMemberCommitsNothingAloneWhileLinked(t *testing.T) {
	store := newJointStore()
	store.block = true
	p := openPair(t, store)
	p.Link("op-1", "trade", pair(6, 24), Effect{})
	p.Capture("a", State{Coins: 99}, false)
	if singles, _ := store.counts(); singles != 0 || outcome(p, "op-1") != Pending {
		t.Fatal("a linked member waits")
	}
	close(store.release)
	eventually(t, "the joint commit", func() bool { return outcome(p, "op-1") == Applied })
	if singles, _ := store.counts(); singles != 0 {
		t.Fatal("the newer state still waits for the unlink")
	}
	p.Unlink("op-1")
	eventually(t, "the newer state", func() bool { return store.coins("a") == 99 })
}

// A transport failure retries the same operation at the same sequences; the
// engine applies it once.
func TestAJointCommitRetriesTheSameOperation(t *testing.T) {
	store := newJointStore()
	store.fail = 1
	p := openPair(t, store)
	p.Link("op-1", "trade", pair(6, 24), Effect{})
	eventually(t, "the retried joint commit", func() bool { return outcome(p, "op-1") == Applied })
	store.mu.Lock()
	defer store.mu.Unlock()
	if len(store.joints) != 2 || store.joints[0].Op != store.joints[1].Op ||
		store.joints[0].Commits[0].Seq != store.joints[1].Commits[0].Seq {
		t.Fatalf("the retry repeats the operation and its sequences: %+v", store.joints)
	}
}

// A refusal for one member refuses the whole operation: no state applies, and
// the stale member is fenced.
func TestARefusedLinkAppliesNothing(t *testing.T) {
	store := newJointStore()
	store.epochs["b"] = 10
	p := openPair(t, store)
	p.Link("op-1", "trade", pair(6, 24), Effect{})
	eventually(t, "the refusal", func() bool { return outcome(p, "op-1") == Refused })
	if store.coins("a") != 0 || store.coins("b") != 0 {
		t.Fatal("no state of a refused operation applies")
	}
	if _, live := p.Lease("b"); live {
		t.Fatal("the stale member writes nothing more")
	}
	if _, live := p.Lease("a"); !live {
		t.Fatal("the other member stays live")
	}
}

// A member that closes while its link waits commits the final state of the
// outcome: with the trade when it applied.
func TestAMemberClosedDuringALinkTakesTheFinalOfItsOutcome(t *testing.T) {
	store := newJointStore()
	store.block = true
	p := openPair(t, store)
	p.Link("op-1", "trade", pair(6, 24), Effect{})
	done, mark := p.CloseLinked("b", State{Coins: 24, XP: 1}, State{Coins: 20, XP: 1})
	close(store.release)
	<-done
	if !p.Committed("b", mark) || store.coins("b") != 24 {
		t.Fatalf("the applied trade closes with its final state: coins=%d", store.coins("b"))
	}
	p.Unlink("op-1")
}

// The outcome of a marked state: pending while it may apply, applied once it
// did, refused when its session can no longer commit it.
func TestTheOutcomeOfAMarkedState(t *testing.T) {
	store := newJointStore()
	p := New(store)
	p.Open(Lease{PlayerID: "a", AccountID: "acc-a", Epoch: 7}, State{})
	mark := p.Capture("a", State{Coins: 3}, false)
	if p.Outcome("a", mark) != Pending {
		t.Fatal("a queued state is pending")
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	go p.Run(ctx)
	eventually(t, "the commit", func() bool { return p.Outcome("a", mark) == Applied })
	if p.Outcome("a", Mark{Epoch: 6, Seq: 1}) != Refused {
		t.Fatal("a state of another epoch never applies")
	}
}

// A link of one session carries its effect: the world drop it creates.
func TestALinkOfOneSessionCarriesItsEffect(t *testing.T) {
	store := newJointStore()
	p := openPair(t, store)
	drop := &WorldDrop{MapCode: "m", ItemID: "gel", CID: "cid-gel", Quantity: 2, Owner: "a"}
	if !p.Link("drop-1", "drop", []LinkMember{{PlayerID: "a", State: State{Coins: 4}}}, Effect{Drop: drop}) {
		t.Fatal("one live session links")
	}
	eventually(t, "the drop commit", func() bool { return outcome(p, "drop-1") == Applied })
	store.mu.Lock()
	defer store.mu.Unlock()
	if len(store.joints) != 1 || store.joints[0].Effect.Drop != drop || store.joints[0].Kind != "drop" {
		t.Fatalf("the joint commit carries the drop: %+v", store.joints)
	}
}

// A claim of a world drop that is gone refuses the operation and ends no
// session: the next state of the member commits alone.
func TestAGoneDropRefusesThePickupAndKeepsTheSession(t *testing.T) {
	store := newJointStore()
	store.gone["drop-9"] = true
	p := openPair(t, store)
	p.Link("pick-1", "pickup", []LinkMember{{PlayerID: "a", State: State{Coins: 4}}}, Effect{Claim: "drop-9"})
	eventually(t, "the refusal", func() bool { return outcome(p, "pick-1") == Refused })
	if _, refusal := p.LinkOutcome("pick-1"); refusal != RefusalDropGone {
		t.Fatalf("the outcome names the refusal: %q", refusal)
	}
	if _, live := p.Lease("a"); !live {
		t.Fatal("a gone drop ends no session")
	}
	p.Capture("a", State{Coins: 3}, false)
	p.Unlink("pick-1")
	eventually(t, "the next state", func() bool { return store.coins("a") == 3 })
}

// A link whose members all left still answers its outcome for a while.
func TestALinkWithoutLiveMembersStillAnswersItsOutcome(t *testing.T) {
	store := newJointStore()
	store.block = true
	p := openPair(t, store)
	p.Link("drop-1", "drop", []LinkMember{{PlayerID: "a", State: State{Coins: 4}}}, Effect{Drop: &WorldDrop{ItemID: "gel"}})
	done, _ := p.CloseLinked("a", State{Coins: 4}, State{Coins: 4})
	close(store.release)
	<-done
	if got, _ := p.LinkOutcome("drop-1"); got != Applied {
		t.Fatalf("the outcome outlives its members: %v", got)
	}
}

// A takeover waits on PendingLink: it closes once the joint commit resolves,
// and no account has one before or after.
func TestPendingLinkClosesWhenTheLinkResolves(t *testing.T) {
	store := newJointStore()
	store.block = true
	p := openPair(t, store)
	if p.PendingLink("acc-a") != nil {
		t.Fatal("no link is pending before one is made")
	}
	p.Link("op-1", "trade", pair(6, 24), Effect{})
	pending := p.PendingLink("acc-a")
	if pending == nil || p.Health("a").Durability != Saving {
		t.Fatal("a pending link is visible to a takeover and keeps the member saving")
	}
	select {
	case <-pending:
		t.Fatal("the link has not resolved yet")
	default:
	}
	close(store.release)
	eventually(t, "the resolution", func() bool {
		select {
		case <-pending:
			return true
		default:
			return false
		}
	})
	if outcome(p, "op-1") != Applied || p.PendingLink("acc-a") != nil {
		t.Fatal("the resolved link applied and is no longer pending")
	}
}
