package persist

import (
	"context"
	"time"

	"cyberia-server/logx"
)

// Outcome is how a marked state or a link ends.
type Outcome uint8

const (
	// Pending: the commit may still apply.
	Pending Outcome = iota
	// Applied: the engine applied it.
	Applied
	// Refused: the engine refused it, or it can no longer apply.
	Refused
)

// LinkMember is one session of a link and the state it commits.
type LinkMember struct {
	PlayerID string
	State    State
}

// WorldDrop is a stack an account player dropped on the ground of a world.
// A drop link carries its Lifetime; a restored drop carries its id, owner and
// expiry.
type WorldDrop struct {
	ID        string
	MapCode   string
	X, Y      float64
	ItemID    string
	CID       string
	Quantity  int
	Owner     string
	Lifetime  time.Duration
	ExpiresAt time.Time
}

// MarketOp is the market operation a link commits with the state of its
// player. The engine checks every field against its records.
type MarketOp struct {
	MarketCode  string
	ListingID   string // cancel, buy, bid
	Revision    uint32 // cancel, buy, bid: the revision the player saw
	Mode        string // list
	ItemID, CID string // list, cancel, buy: the stack
	Quantity    int
	Price       int // list: the price or the opening bid; buy: the price; bid: the bid
	Hours       int // list
	ListingFee  int // list: the fees the seller saw
	SaleFee     int
	SellerName  string
	ClaimIDs    []string // collect
	Coins       int      // collect: the coins of the claims
	Stacks      []Stack  // collect: the stacks of the claims
}

// Effect is what a link writes beside the states of its members: the world
// drop a drop creates, the id of the world drop a pickup claims, or a market
// operation.
type Effect struct {
	Drop   *WorldDrop
	Claim  string
	Market *MarketOp
}

// JointRequest commits the states of its sessions and its effect in one
// transaction.
type JointRequest struct {
	Op, Kind string
	Commits  []CommitRequest
	Effect   Effect
}

// JointResult is the answer of the engine to a joint commit. A refusal names
// its player and refuses every state.
type JointResult struct {
	Refusal, PlayerID string
	Duplicate         bool
}

// RefusalDropGone refuses a pickup whose world drop is claimed, expired or
// unknown. Like every refusal of a joint commit but RefusalStaleSession,
// RefusalUnknown and RefusalInvalid, it refuses the operation and ends no
// session.
const RefusalDropGone = "drop_gone"

// link is one operation that commits the states of its sessions and its
// effect at once. While it is pending, its members send no commit alone: a
// newer state of a member waits until the link resolves.
type link struct {
	op, kind string
	members  []*record
	seqs     []uint64
	states   []State
	questsV  []uint64
	effect   Effect
	outcome  Outcome
	refusal  string
	// resolved closes when the outcome is no longer Pending.
	resolved     chan struct{}
	inflight     bool
	nextTry      time.Time
	backoff      time.Duration
	failingSince time.Time
	// finals holds the final state of each member closed while the link was
	// pending, by outcome: index 0 when applied, 1 when refused.
	finals map[*record][2]State
}

// resolved is the outcome of a link that left the pipeline.
type resolved struct {
	outcome Outcome
	refusal string
	at      time.Time
}

// Link joins the given states of live sessions and effect into one atomic
// commit under operation id op. Each state takes a new sequence of its
// session. False when a session is not live, already linked, or op is in use.
func (p *Pipeline) Link(op, kind string, members []LinkMember, effect Effect) bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	if _, used := p.links[op]; used || len(members) == 0 {
		return false
	}
	records := make([]*record, len(members))
	for i, m := range members {
		r := p.records[m.PlayerID]
		if r == nil || r.closing || r.fenced != "" || r.link != nil {
			return false
		}
		records[i] = r
	}
	l := &link{op: op, kind: kind, members: records, effect: effect, resolved: make(chan struct{}), finals: make(map[*record][2]State)}
	for i, r := range records {
		before := r.seq
		p.take(r, members[i].State.canonical(), false)
		// The joint state takes a sequence no commit of the session used.
		if r.seq == before {
			r.seq++
			if r.pendingSince.IsZero() {
				r.pendingSince = p.now()
			}
		}
		r.link = l
		l.seqs = append(l.seqs, r.seq)
		l.states = append(l.states, r.state)
		l.questsV = append(l.questsV, r.questsV)
	}
	p.links[op] = l
	p.signal()
	return true
}

// LinkOutcome is the outcome of the link op and the refusal of the engine.
// A link that left the pipeline answers for settledMemory; an unknown op is
// Refused.
func (p *Pipeline) LinkOutcome(op string) (Outcome, string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if l := p.links[op]; l != nil {
		return l.outcome, l.refusal
	}
	if r, ok := p.resolved[op]; ok {
		return r.outcome, r.refusal
	}
	return Refused, ""
}

// forget removes a resolved link and remembers its outcome. Caller holds p.mu.
func (p *Pipeline) forget(l *link) {
	delete(p.links, l.op)
	p.resolved[l.op] = resolved{outcome: l.outcome, refusal: l.refusal, at: p.now()}
}

// Unlink ends a resolved link: each live member commits its newest state
// alone again. Call it once the world holds the result of the outcome.
func (p *Pipeline) Unlink(op string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	l := p.links[op]
	if l == nil || l.outcome == Pending {
		return
	}
	for _, r := range l.members {
		if r.link == l {
			r.link = nil
		}
	}
	p.forget(l)
	p.signal()
}

// CloseLinked ends a session with a final state for each outcome of its link:
// applied is the state with the result of the operation, refused the state
// without it. A session without a pending link closes with applied.
func (p *Pipeline) CloseLinked(playerID string, applied, refused State) (<-chan struct{}, Mark) {
	p.mu.Lock()
	if r := p.records[playerID]; r != nil && !r.closing && r.link != nil {
		switch l := r.link; l.outcome {
		case Pending:
			defer p.mu.Unlock()
			p.closing(r)
			l.finals[r] = [2]State{applied.canonical(), refused.canonical()}
			return r.done, Mark{Epoch: r.lease.Epoch, Seq: r.seq + 1}
		case Refused:
			applied = refused
		}
		r.link = nil
	}
	p.mu.Unlock()
	return p.Close(playerID, applied)
}

// sendLinks starts the joint commit of every pending link that is due.
// Caller holds p.mu.
func (p *Pipeline) sendLinks(ctx context.Context, now time.Time) {
	for _, l := range p.links {
		if l.outcome != Pending || l.inflight || now.Before(l.nextTry) || p.inFlight >= maxInFlight {
			continue
		}
		busy := false
		for _, r := range l.members {
			busy = busy || r.inflight
		}
		if busy {
			continue
		}
		req := JointRequest{Op: l.op, Kind: l.kind, Effect: l.effect}
		for i, r := range l.members {
			req.Commits = append(req.Commits, CommitRequest{
				Lease: r.lease, Seq: l.seqs[i], State: l.states[i], QuestsChanged: true,
			})
		}
		l.inflight = true
		p.inFlight++
		sent := p.now()
		go func() {
			callCtx, cancel := context.WithTimeout(ctx, callTimeout)
			result, err := p.store.CommitJoint(callCtx, req)
			cancel()
			p.jointCommitted(l, result, err, p.now().Sub(sent))
		}()
	}
}

// jointCommitted applies the answer to one joint commit. A transport error is
// retried with the same request: the engine answers a repeat as a duplicate.
func (p *Pipeline) jointCommitted(l *link, result JointResult, err error, took time.Duration) {
	p.mu.Lock()
	defer p.mu.Unlock()
	l.inflight = false
	p.inFlight--
	p.stats.observe(took)
	if err != nil {
		p.stats.Retries++
		if l.failingSince.IsZero() {
			l.failingSince = p.now()
			logx.Warnf("[Persist] joint commit %s failed; retrying the same operation: %v", l.op, err)
		}
		var failure string
		failure, l.backoff = classify(err, "", l.backoff)
		l.nextTry = p.now().Add(jittered(l.backoff))
		for _, r := range l.members {
			if r.failingSince.IsZero() {
				r.failingSince = l.failingSince
			}
			r.failure = failure
		}
		return
	}
	for _, r := range l.members {
		r.failingSince, r.failure = time.Time{}, ""
	}
	if result.Refusal != "" {
		p.stats.JointRefused++
		logx.Warnf("[Persist] joint commit %s refused for player=%s: %s", l.op, result.PlayerID, result.Refusal)
		l.outcome, l.refusal = Refused, result.Refusal
		close(l.resolved)
		for _, r := range l.members {
			if r.lease.PlayerID != result.PlayerID {
				continue
			}
			switch result.Refusal {
			case RefusalStaleSession, RefusalUnknown:
				p.fence(r, EndedElsewhere)
			case RefusalInvalid:
				p.fence(r, EndedInvalid)
			}
		}
	} else {
		p.stats.Joints++
		l.outcome = Applied
		close(l.resolved)
		for i, r := range l.members {
			r.durable = max(r.durable, l.seqs[i])
			r.questsDurableV = max(r.questsDurableV, l.questsV[i])
			if r.durable >= r.seq {
				r.pendingSince = time.Time{}
			}
		}
	}
	p.settleFinals(l)
	p.signal()
}

// settleFinals takes the final state of each member closed while its link was
// pending, by the outcome, and frees it from the link. Caller holds p.mu.
func (p *Pipeline) settleFinals(l *link) {
	for r, finals := range l.finals {
		final := finals[0]
		if l.outcome == Refused {
			final = finals[1]
		}
		if r.fenced == "" {
			p.take(r, final, true)
		}
		r.link = nil
		delete(l.finals, r)
	}
	for _, r := range l.members {
		if r.link == l {
			return
		}
	}
	// No live member is left to unlink it.
	p.forget(l)
}

// Outcome is how a marked state of a session ends: applied once the engine
// applied it or a later state, refused when its session can no longer commit
// it, pending while it may still apply.
func (p *Pipeline) Outcome(playerID string, mark Mark) Outcome {
	p.mu.Lock()
	defer p.mu.Unlock()
	if r := p.records[playerID]; r != nil && r.lease.Epoch == mark.Epoch {
		switch {
		case r.durable >= mark.Seq:
			return Applied
		case r.fenced != "":
			return Refused
		}
		return Pending
	}
	if s, ok := p.settled[settledKey{playerID, mark.Epoch}]; ok && s.seq >= mark.Seq {
		return Applied
	}
	return Refused
}
