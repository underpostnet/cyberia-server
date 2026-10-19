package net

import (
	"sync"
	"testing"
	"time"

	"cyberia-server/game/sim"
)

func TestTickWorkUsesARecentWindow(t *testing.T) {
	var counters runtimeCounters
	if got := counters.tickWorkMs(); got != 0 {
		t.Fatalf("no ticks: got %v ms", got)
	}
	counters.recordTick(10 * time.Millisecond)
	counters.recordTick(30 * time.Millisecond)
	if got := counters.tickWorkMs(); got != 20 {
		t.Fatalf("two ticks: got %v ms, want 20", got)
	}
	for range sim.DefaultTickRate {
		counters.recordTick(time.Millisecond)
	}
	if got := counters.tickWorkMs(); got != 1 {
		t.Fatalf("recent ticks: got %v ms, want 1", got)
	}
}

func TestTickWorkSupportsConcurrentReads(t *testing.T) {
	var counters runtimeCounters
	var workers sync.WaitGroup
	for range 4 {
		workers.Go(func() {
			for range 100 {
				counters.recordTick(time.Millisecond)
				if got := counters.tickWorkMs(); got != 1 {
					t.Errorf("got %v ms, want 1", got)
				}
			}
		})
	}
	workers.Wait()
}

func TestRuntimeMetricsUsesTheAdmissionLimitAndCount(t *testing.T) {
	server := NewGameServer()
	limits := DefaultConnectionLimits()
	limits.MaxConnections = 17
	server.SetConnectionLimits(limits)
	if got := server.guard.admit("127.0.0.1"); got != admitted {
		t.Fatalf("admission: %s", got)
	}
	snapshot := server.RuntimeMetrics()
	if snapshot.MaxConnections != 17 || snapshot.WsActiveConns != 1 {
		t.Fatalf("capacity=%d, connections=%d", snapshot.MaxConnections, snapshot.WsActiveConns)
	}
	server.SetConnectionLimits(UnlimitedConnectionLimits())
	if got := server.RuntimeMetrics().MaxConnections; got != 0 {
		t.Fatalf("unlimited capacity: got %d", got)
	}
}
