package game

import (
	"fmt"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"
)

func testLimits() ConnectionLimits {
	l := DefaultConnectionLimits()
	l.MaxConnections = 4
	l.MaxConnectionsPerIP = 2
	l.ConnectRatePerIP = 1000 // out of the way unless a case targets it
	l.ConnectBurstPerIP = 1000
	return l
}

func TestGuardBoundsConnectionsPerAddress(t *testing.T) {
	g := newConnectionGuard(testLimits())

	if r := g.admit("10.0.0.1"); r != admitted {
		t.Fatalf("first connection must be admitted, got %q", r)
	}
	if r := g.admit("10.0.0.1"); r != admitted {
		t.Fatalf("second connection must be admitted, got %q", r)
	}
	if r := g.admit("10.0.0.1"); r != refusedIPSaturated {
		t.Fatalf("third from the same address must be refused, got %q", r)
	}
	// A different address is unaffected: the per-IP ceiling is not global.
	if r := g.admit("10.0.0.2"); r != admitted {
		t.Fatalf("a second address must still connect, got %q", r)
	}

	// Releasing frees the slot for a reconnect.
	g.release("10.0.0.1")
	if r := g.admit("10.0.0.1"); r != admitted {
		t.Fatalf("a freed slot must be reusable, got %q", r)
	}
}

func TestGuardBoundsTotalConnections(t *testing.T) {
	g := newConnectionGuard(testLimits()) // MaxConnections 4, 2 per IP
	for i, ip := range []string{"1.1.1.1", "1.1.1.1", "2.2.2.2", "2.2.2.2"} {
		if r := g.admit(ip); r != admitted {
			t.Fatalf("connection %d must be admitted, got %q", i, r)
		}
	}
	if r := g.admit("3.3.3.3"); r != refusedServerFull {
		t.Fatalf("past the total ceiling must refuse, got %q", r)
	}
	total, addresses := g.counts()
	if total != 4 || addresses != 2 {
		t.Fatalf("counts = (%d conns, %d addresses), want (4, 2)", total, addresses)
	}
}

func TestGuardBoundsConnectRatePerAddress(t *testing.T) {
	l := testLimits()
	l.MaxConnections = 1000
	l.MaxConnectionsPerIP = 1000
	l.ConnectRatePerIP = 1  // one per second sustained
	l.ConnectBurstPerIP = 3 // three back to back
	g := newConnectionGuard(l)

	for i := 0; i < 3; i++ {
		if r := g.admit("9.9.9.9"); r != admitted {
			t.Fatalf("burst connection %d must be admitted, got %q", i, r)
		}
	}
	if r := g.admit("9.9.9.9"); r != refusedIPRateLimits {
		t.Fatalf("past the burst must be rate limited, got %q", r)
	}
	// Another address has its own bucket.
	if r := g.admit("8.8.8.8"); r != admitted {
		t.Fatalf("a different address keeps its own budget, got %q", r)
	}
}

// release must never drive a count below zero, whatever order paths run in.
func TestGuardReleaseIsSafeWhenUnbalanced(t *testing.T) {
	g := newConnectionGuard(testLimits())
	g.release("7.7.7.7") // never admitted
	g.admit("7.7.7.7")
	g.release("7.7.7.7")
	g.release("7.7.7.7") // duplicate
	total, addresses := g.counts()
	if total != 0 || addresses != 0 {
		t.Fatalf("counts = (%d, %d), want (0, 0)", total, addresses)
	}
}

func TestTokenBucketRefillsOverTime(t *testing.T) {
	b := newTokenBucket(10, 3) // 10/s, burst 3
	now := time.Now()
	for i := 0; i < 3; i++ {
		if !b.allow(now) {
			t.Fatalf("burst token %d must be available", i)
		}
	}
	if b.allow(now) {
		t.Fatal("an empty bucket must refuse")
	}
	// 100 ms at 10/s puts exactly one token back.
	if !b.allow(now.Add(100 * time.Millisecond)) {
		t.Fatal("one token must be back after 100ms")
	}
	if b.allow(now.Add(100 * time.Millisecond)) {
		t.Fatal("only one token accrues in 100ms")
	}
	// Refill never exceeds the capacity.
	if !b.allow(now.Add(time.Hour)) {
		t.Fatal("a long idle period must refill")
	}
	for i := 0; i < 2; i++ {
		b.allow(now.Add(time.Hour))
	}
	if b.allow(now.Add(time.Hour)) {
		t.Fatal("refill must cap at the burst size, not accumulate")
	}
}

// A client at the real client's peak tap rate must never be limited; a spamming
// client must be limited and then evicted.
func TestInputLimiterPassesRealClientRateAndEvictsSpam(t *testing.T) {
	limits := DefaultConnectionLimits()

	l := newInputLimiter(limits)
	// 10 taps/s for 5 s — the fast-human ceiling for the real client.
	for second := 0; second < 5; second++ {
		for tap := 0; tap < 10; tap++ {
			if ok, _ := l.allow(); !ok {
				t.Fatalf("real client rate must never be limited (second %d, tap %d)", second, tap)
			}
		}
		time.Sleep(20 * time.Millisecond) // compressed wall clock
	}
	if l.strikes != 0 {
		t.Fatalf("a well-behaved client must carry no strikes, got %d", l.strikes)
	}

	spam := newInputLimiter(limits)
	evicted := false
	for i := 0; i < 5000 && !evicted; i++ {
		_, evict := spam.allow()
		evicted = evict
	}
	if !evicted {
		t.Fatal("a client spamming without pause must be evicted")
	}
}

// Protocol violations alone must evict, even inside the message budget.
func TestInputLimiterEvictsOnRepeatedProtocolViolations(t *testing.T) {
	l := newInputLimiter(DefaultConnectionLimits())
	for i := 1; i < DefaultMaxStrikes; i++ {
		if l.strike() {
			t.Fatalf("evicted early at strike %d", i)
		}
	}
	if !l.strike() {
		t.Fatalf("strike %d must evict", DefaultMaxStrikes)
	}
}

func TestClientIPPrefersForwardedHeader(t *testing.T) {
	cases := []struct {
		name       string
		remoteAddr string
		headers    map[string]string
		want       string
	}{
		{"remote addr", "203.0.113.7:54321", nil, "203.0.113.7"},
		{"forwarded chain", "10.0.0.1:1", map[string]string{"X-Forwarded-For": "198.51.100.4, 10.0.0.1"}, "198.51.100.4"},
		{"forwarded single", "10.0.0.1:1", map[string]string{"X-Forwarded-For": "198.51.100.9"}, "198.51.100.9"},
		{"real ip", "10.0.0.1:1", map[string]string{"X-Real-Ip": "198.51.100.5"}, "198.51.100.5"},
		{"ipv6 remote", "[2001:db8::1]:443", nil, "2001:db8::1"},
	}
	for _, c := range cases {
		r := httptest.NewRequest(http.MethodGet, "/ws", nil)
		r.RemoteAddr = c.remoteAddr
		for k, v := range c.headers {
			r.Header.Set(k, v)
		}
		if got := clientIP(r); got != c.want {
			t.Fatalf("%s: clientIP = %q, want %q", c.name, got, c.want)
		}
	}
}

func intPtr(v int) *int { return &v }

func TestResolveConnectionLimitsAppliesOverridesOverDefaults(t *testing.T) {
	defaults := DefaultConnectionLimits()

	// No overrides keeps every default.
	if got := ResolveConnectionLimits(ConnectionLimitOverrides{}); got != defaults {
		t.Fatalf("no overrides must keep the defaults, got %+v", got)
	}

	// One override changes only that field.
	got := ResolveConnectionLimits(ConnectionLimitOverrides{MaxConnectionsPerIP: intPtr(512)})
	if got.MaxConnectionsPerIP != 512 {
		t.Fatalf("per-IP override ignored: %d", got.MaxConnectionsPerIP)
	}
	if got.MaxConnections != defaults.MaxConnections || got.MessageRate != defaults.MessageRate {
		t.Fatalf("an override must not disturb other fields: %+v", got)
	}
}

// An explicit 0 disables one limit; it must not read as "keep the default".
func TestResolveConnectionLimitsTreatsZeroAsDisabled(t *testing.T) {
	got := ResolveConnectionLimits(ConnectionLimitOverrides{
		ConnectRatePerIP: intPtr(0),
		MessageRate:      intPtr(0),
	})
	if got.ConnectRatePerIP != 0 || got.MessageRate != 0 {
		t.Fatalf("explicit zero must disable, got %+v", got)
	}
	if got.MaxConnectionsPerIP != DefaultMaxConnectionsPerIP {
		t.Fatal("disabling one limit must leave the others at their defaults")
	}
}

func TestResolveConnectionLimitsDisableAllWins(t *testing.T) {
	got := ResolveConnectionLimits(ConnectionLimitOverrides{
		DisableAll:          true,
		MaxConnectionsPerIP: intPtr(3),
		MessageRate:         intPtr(5),
	})
	if !got.Unlimited() {
		t.Fatalf("DisableAll must override every field, got %+v", got)
	}
}

// A burst below the sustained rate would throttle below what was asked for.
func TestResolveConnectionLimitsRaisesBurstToTheRate(t *testing.T) {
	got := ResolveConnectionLimits(ConnectionLimitOverrides{
		ConnectRatePerIP: intPtr(50),
		MessageRate:      intPtr(200),
	})
	if got.ConnectBurstPerIP < 50 {
		t.Fatalf("connect burst %v must not sit below the rate 50", got.ConnectBurstPerIP)
	}
	if got.MessageBurst < 200 {
		t.Fatalf("message burst %v must not sit below the rate 200", got.MessageBurst)
	}
}

// The unlimited profile must admit far past every default ceiling, from one
// address, with no pause — the load-test shape.
func TestUnlimitedProfileAdmitsEverything(t *testing.T) {
	g := newConnectionGuard(UnlimitedConnectionLimits())
	const wanted = 5000
	for i := 0; i < wanted; i++ {
		if r := g.admit("127.0.0.1"); r != admitted {
			t.Fatalf("connection %d refused (%q) with limits disabled", i, r)
		}
	}
	if total, _ := g.counts(); total != wanted {
		t.Fatalf("want %d tracked connections, got %d", wanted, total)
	}
	for i := 0; i < wanted; i++ {
		g.release("127.0.0.1")
	}
	if total, addresses := g.counts(); total != 0 || addresses != 0 {
		t.Fatalf("counts must return to zero, got (%d, %d)", total, addresses)
	}
}

// With the message budget disabled, no volume of input is ever limited.
func TestUnlimitedProfileNeverRateLimitsInput(t *testing.T) {
	l := newInputLimiter(UnlimitedConnectionLimits())
	for i := 0; i < 100000; i++ {
		ok, evict := l.allow()
		if !ok || evict {
			t.Fatalf("message %d limited (ok=%v evict=%v) with rate limiting disabled", i, ok, evict)
		}
	}
}

func TestDescribeReportsTheEffectiveProfile(t *testing.T) {
	if got := UnlimitedConnectionLimits().Describe(); !strings.Contains(got, "ALL LIMITS DISABLED") {
		t.Fatalf("the unlimited profile must be obvious in the log, got %q", got)
	}
	// Derived from the constants, not copied: the shipped values are policy and
	// may change without making this test wrong.
	got := DefaultConnectionLimits().Describe()
	for _, want := range []string{
		fmt.Sprintf("perIP=%d", DefaultMaxConnectionsPerIP),
		fmt.Sprintf("connectRate=%d/s", DefaultConnectRatePerIP),
		fmt.Sprintf("msgRate=%d/s", DefaultMessageRate),
	} {
		if !strings.Contains(got, want) {
			t.Fatalf("Describe() = %q, missing %q", got, want)
		}
	}
	off := ResolveConnectionLimits(ConnectionLimitOverrides{MessageRate: intPtr(0)}).Describe()
	if !strings.Contains(off, "msgRate=off") {
		t.Fatalf("a disabled limit must read as off, got %q", off)
	}
}
