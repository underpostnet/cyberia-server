package persist

import "time"

// Stats counts the work of the pipeline since the server started.
type Stats struct {
	Commits            uint64 // applied
	Duplicates         uint64 // already applied; the engine changed nothing
	Retries            uint64 // transport failures, each retried
	Integrity          uint64 // refused by the database as they stand, each retried slowly
	Rejected           uint64 // calls the engine rejects itself, each retried slowly
	Stale              uint64 // refused for an older epoch
	Invalid            uint64 // refused as malformed
	LostFinal          uint64 // final states dropped after finalWindow
	Adopted            uint64 // unsettled states a new session of the profile took over
	Superseded         uint64 // unsaved states a new epoch replaced
	Joints             uint64 // joint commits applied
	JointRefused       uint64 // joint commits refused: no state of the operation applied
	Checkpoints        uint64
	CheckpointFailures uint64
	Positions          uint64 // positions written by checkpoints
	LastCheckpoint     time.Time
	commitSeconds      float64
	commitCount        uint64
}

// Snapshot is the pipeline state for the metrics API.
type Snapshot struct {
	Stats
	Sessions          int           // live and closing
	Pending           int           // sessions with an unapplied state
	OldestPending     time.Duration // age of the oldest unapplied state
	Unsaved           int           // sessions degraded or in recovery
	Recovery          int           // sessions the engine refuses to save as they stand
	AvgCommitDuration time.Duration
}

func (s *Stats) observe(took time.Duration) {
	s.commitSeconds += took.Seconds()
	s.commitCount++
}

// Snapshot reads the counters and the queue.
func (p *Pipeline) Snapshot() Snapshot {
	p.mu.Lock()
	defer p.mu.Unlock()
	out := Snapshot{Stats: p.stats, Sessions: len(p.records)}
	if p.stats.commitCount > 0 {
		out.AvgCommitDuration = time.Duration(p.stats.commitSeconds / float64(p.stats.commitCount) * float64(time.Second))
	}
	now := p.now()
	for _, r := range p.records {
		if r.seq <= r.durable {
			continue
		}
		out.Pending++
		out.OldestPending = max(out.OldestPending, now.Sub(r.pendingSince))
		switch d := p.durabilityOf(r); {
		case d == Recovery:
			out.Recovery++
			out.Unsaved++
		case d.Unsaved():
			out.Unsaved++
		}
	}
	return out
}
