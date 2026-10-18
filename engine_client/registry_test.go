package engine_client

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"
	"time"
)

// registryRecorder is the engine registry route: it keeps every report it takes.
type registryRecorder struct {
	mu      sync.Mutex
	reports []Report
	keys    []string
}

func (r *registryRecorder) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	var report Report
	json.NewDecoder(req.Body).Decode(&report)
	r.mu.Lock()
	r.reports = append(r.reports, report)
	r.keys = append(r.keys, req.Header.Get("X-Cyberia-Server-Api-Key"))
	r.mu.Unlock()
	w.WriteHeader(http.StatusOK)
}

func (r *registryRecorder) snapshot() []Report {
	r.mu.Lock()
	defer r.mu.Unlock()
	return append([]Report(nil), r.reports...)
}

func TestARegistryReportsReadyThenDrainingWithRisingSequences(t *testing.T) {
	recorder := &registryRecorder{}
	engine := httptest.NewServer(recorder)
	defer engine.Close()

	registry := NewRegistry(engine.URL, "key-1", Report{
		ServerID: "pod-a-1234", URL: "https://server.example.com/test", InstanceCode: "test", Name: "Test World",
		Build: "0123456789abcdef",
	})
	release := "r1"
	var releaseMu sync.Mutex
	registry.FollowRelease(func() string {
		releaseMu.Lock()
		defer releaseMu.Unlock()
		return release
	})
	registry.Start()
	deadline := time.Now().Add(2 * time.Second)
	for len(recorder.snapshot()) == 0 && time.Now().Before(deadline) {
		time.Sleep(10 * time.Millisecond)
	}
	releaseMu.Lock()
	release = "r2"
	releaseMu.Unlock()
	if err := registry.Drain(context.Background()); err != nil {
		t.Fatal(err)
	}
	reports := recorder.snapshot()
	if len(reports) < 2 {
		t.Fatalf("reports = %+v, want a ready report then a draining one", reports)
	}
	first, last := reports[0], reports[len(reports)-1]
	if first.State != StateReady || first.Seq != 1 || first.ServerID != "pod-a-1234" || first.URL != "https://server.example.com/test" {
		t.Fatalf("first report = %+v", first)
	}
	if last.State != StateDraining || last.Seq <= first.Seq {
		t.Fatalf("the draining report carries a higher sequence: %+v", last)
	}
	if first.Build != "0123456789abcdef" || first.ReleaseID != "r1" || last.ReleaseID != "r2" {
		t.Fatalf("every report names the build and the release served at that moment: %+v %+v", first, last)
	}
	for _, key := range recorder.keys {
		if key != "key-1" {
			t.Fatalf("every report carries the server key, got %q", key)
		}
	}
	time.Sleep(50 * time.Millisecond)
	if after := recorder.snapshot(); len(after) != len(reports) {
		t.Fatal("no ready report follows the draining one")
	}
}

func TestARegistryWithoutPublicURLOrKeyStaysOffTheList(t *testing.T) {
	recorder := &registryRecorder{}
	engine := httptest.NewServer(recorder)
	defer engine.Close()
	for _, registry := range []*Registry{
		NewRegistry(engine.URL, "key-1", Report{ServerID: "pod", InstanceCode: "test"}),
		NewRegistry(engine.URL, "", Report{ServerID: "pod", URL: "https://server.example.com", InstanceCode: "test"}),
	} {
		registry.Start()
		if err := registry.Drain(context.Background()); err != nil {
			t.Fatal(err)
		}
	}
	time.Sleep(50 * time.Millisecond)
	if n := len(recorder.snapshot()); n != 0 {
		t.Fatalf("reports = %d, want 0", n)
	}
}
