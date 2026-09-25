package game

import (
	"context"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"
)

func TestDrainRefusesNewSessions(t *testing.T) {
	s := NewGameServer()
	if s.IsDraining() {
		t.Fatal("a new server must admit sessions")
	}
	s.BeginDrain()
	s.BeginDrain()
	if !s.IsDraining() {
		t.Fatal("BeginDrain must stop admission")
	}

	rec := httptest.NewRecorder()
	s.HandleConnections(rec, httptest.NewRequest(http.MethodGet, "/ws", nil))
	if rec.Code != http.StatusServiceUnavailable {
		t.Fatalf("a draining server answered %d, want 503", rec.Code)
	}
}

func TestWaitDrainedEndsWithoutClientsOrAtTheDeadline(t *testing.T) {
	s := NewGameServer()
	if remaining := s.WaitDrained(context.Background(), time.Millisecond); remaining != 0 {
		t.Fatalf("an empty server reported %d players", remaining)
	}

	s.clients["held"] = &Client{}
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Millisecond)
	defer cancel()
	started := time.Now()
	if remaining := s.WaitDrained(ctx, 5*time.Millisecond); remaining != 1 {
		t.Fatalf("the deadline reported %d players, want 1", remaining)
	}
	if time.Since(started) > time.Second {
		t.Fatal("WaitDrained outlived its deadline")
	}
}
