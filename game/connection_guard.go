// Package game — connection_guard.go
//
// Admission control and abuse limits for the WebSocket endpoint.
//
// Three layers, in the order a connection meets them:
//
//  1. admit    — total connections, connections per IP, new connections per IP
//     per second. Runs before the HTTP upgrade.
//  2. rateLimiter — a token bucket per connection, charged one token per
//     inbound message. Runs in receiveMessage.
//  3. strike counting — protocol and rate violations. A client that keeps
//     violating gets disconnected.
//
// IP is one layer, not the identity. A shared NAT holds many real players, so
// the per-IP ceiling is generous and the per-connection message budget carries
// the real protection.
//
// Thresholds come from the real client traffic profile. The client emits one
// player_action per discrete mouse press (IsMouseButtonPressed is edge
// triggered), so a fast human peaks near 10 taps per second. UI actions
// (freeze, equip, dialogue, storage) are human paced and much rarer.

package game

import (
	"net"
	"net/http"
	"strconv"
	"strings"
	"sync"
	"time"
)

const (
	// DefaultMaxConnections bounds total concurrent WebSocket clients.
	DefaultMaxConnections = 2000

	// DefaultMaxConnectionsPerIP bounds concurrent connections from one
	// address. High enough for a household or a NAT, low enough that one
	// address cannot take the whole server.
	DefaultMaxConnectionsPerIP = 5

	// DefaultConnectRatePerIP bounds new connections per second from one
	// address, with DefaultConnectBurstPerIP allowed in a burst. A reconnect
	// storm from one client stays inside the burst.
	DefaultConnectRatePerIP  = 8
	DefaultConnectBurstPerIP = 32

	// DefaultMessageRate is the sustained inbound message budget per
	// connection, in messages per second. A fast human peaks near 10 taps per
	// second; 30 leaves room for taps plus UI actions in the same second.
	DefaultMessageRate = 30

	// DefaultMessageBurst absorbs a short burst without a strike.
	DefaultMessageBurst = 60

	// DefaultMaxStrikes is how many violations a connection may accumulate
	// before the server closes it. Tokens refill, so an occasional overrun
	// costs one strike and recovers.
	DefaultMaxStrikes = 20

	// strikeDecayInterval is how often one strike is forgiven. A client that
	// misbehaves rarely never reaches the limit.
	strikeDecayInterval = 10 * time.Second
)

// ConnectionLimits holds the tunable admission and rate thresholds.
//
// Zero disables one limit. All zero is the unlimited profile, which exists for
// load tests that drive many connections from one address. Never run a public
// deploy with a limit disabled.
type ConnectionLimits struct {
	MaxConnections      int
	MaxConnectionsPerIP int
	ConnectRatePerIP    float64
	ConnectBurstPerIP   float64
	MessageRate         float64
	MessageBurst        float64
	MaxStrikes          int
}

// DefaultConnectionLimits returns the shipped thresholds.
func DefaultConnectionLimits() ConnectionLimits {
	return ConnectionLimits{
		MaxConnections:      DefaultMaxConnections,
		MaxConnectionsPerIP: DefaultMaxConnectionsPerIP,
		ConnectRatePerIP:    DefaultConnectRatePerIP,
		ConnectBurstPerIP:   DefaultConnectBurstPerIP,
		MessageRate:         DefaultMessageRate,
		MessageBurst:        DefaultMessageBurst,
		MaxStrikes:          DefaultMaxStrikes,
	}
}

// UnlimitedConnectionLimits disables every admission and rate limit. It is for
// load testing against a private instance, where every simulated client shares
// one address and would otherwise be refused.
func UnlimitedConnectionLimits() ConnectionLimits { return ConnectionLimits{} }

// Unlimited reports whether every limit is off.
func (l ConnectionLimits) Unlimited() bool { return l == ConnectionLimits{} }

// ConnectionLimitOverrides carries optional per-field overrides. A nil field
// keeps the default; a pointer to 0 disables that limit.
type ConnectionLimitOverrides struct {
	DisableAll          bool
	MaxConnections      *int
	MaxConnectionsPerIP *int
	ConnectRatePerIP    *int
	ConnectBurstPerIP   *int
	MessageRate         *int
	MessageBurst        *int
	MaxStrikes          *int
}

// ResolveConnectionLimits is the one place that turns configuration into the
// effective thresholds. DisableAll wins over every other field.
func ResolveConnectionLimits(o ConnectionLimitOverrides) ConnectionLimits {
	if o.DisableAll {
		return UnlimitedConnectionLimits()
	}
	limits := DefaultConnectionLimits()
	applyInt := func(dst *int, src *int) {
		if src != nil {
			*dst = *src
		}
	}
	applyFloat := func(dst *float64, src *int) {
		if src != nil {
			*dst = float64(*src)
		}
	}
	applyInt(&limits.MaxConnections, o.MaxConnections)
	applyInt(&limits.MaxConnectionsPerIP, o.MaxConnectionsPerIP)
	applyFloat(&limits.ConnectRatePerIP, o.ConnectRatePerIP)
	applyFloat(&limits.ConnectBurstPerIP, o.ConnectBurstPerIP)
	applyFloat(&limits.MessageRate, o.MessageRate)
	applyFloat(&limits.MessageBurst, o.MessageBurst)
	applyInt(&limits.MaxStrikes, o.MaxStrikes)

	// A burst below the sustained rate would throttle below what was asked
	// for. Raise it to the rate rather than surprise the operator.
	if limits.ConnectRatePerIP > 0 && limits.ConnectBurstPerIP < limits.ConnectRatePerIP {
		limits.ConnectBurstPerIP = limits.ConnectRatePerIP
	}
	if limits.MessageRate > 0 && limits.MessageBurst < limits.MessageRate {
		limits.MessageBurst = limits.MessageRate
	}
	return limits
}

// Describe renders the effective limits for the startup log.
func (l ConnectionLimits) Describe() string {
	if l.Unlimited() {
		return "ALL LIMITS DISABLED (load-test profile — do not use in production)"
	}
	field := func(name string, v float64, unit string) string {
		if v <= 0 {
			return name + "=off"
		}
		return name + "=" + strconv.FormatFloat(v, 'g', -1, 64) + unit
	}
	return strings.Join([]string{
		field("maxConns", float64(l.MaxConnections), ""),
		field("perIP", float64(l.MaxConnectionsPerIP), ""),
		field("connectRate", l.ConnectRatePerIP, "/s"),
		field("connectBurst", l.ConnectBurstPerIP, ""),
		field("msgRate", l.MessageRate, "/s"),
		field("msgBurst", l.MessageBurst, ""),
		field("maxStrikes", float64(l.MaxStrikes), ""),
	}, " ")
}

// tokenBucket is a monotonic-clock token bucket. Not safe for concurrent use;
// each one belongs to a single owner.
type tokenBucket struct {
	tokens   float64
	capacity float64
	rate     float64
	last     time.Time
}

func newTokenBucket(rate, capacity float64) *tokenBucket {
	return &tokenBucket{tokens: capacity, capacity: capacity, rate: rate, last: time.Now()}
}

// allow charges one token. It reports false when the bucket is empty.
func (b *tokenBucket) allow(now time.Time) bool {
	if elapsed := now.Sub(b.last).Seconds(); elapsed > 0 {
		b.last = now
		if b.tokens += elapsed * b.rate; b.tokens > b.capacity {
			b.tokens = b.capacity
		}
	}
	if b.tokens < 1 {
		return false
	}
	b.tokens--
	return true
}

// connectionGuard tracks per-IP connection counts and connect rates.
type connectionGuard struct {
	mu     sync.Mutex
	limits ConnectionLimits
	total  int
	perIP  map[string]int
	rate   map[string]*tokenBucket
}

func newConnectionGuard(limits ConnectionLimits) *connectionGuard {
	return &connectionGuard{
		limits: limits,
		perIP:  make(map[string]int),
		rate:   make(map[string]*tokenBucket),
	}
}

// admissionResult says why a connection was refused. Empty means admitted.
type admissionResult string

const (
	admitted            admissionResult = ""
	refusedServerFull   admissionResult = "server at capacity"
	refusedIPSaturated  admissionResult = "too many connections from this address"
	refusedIPRateLimits admissionResult = "connecting too fast from this address"
)

// admit reserves a slot for one connection. The caller must call release for
// every admitted connection, on every exit path.
func (g *connectionGuard) admit(ip string) admissionResult {
	g.mu.Lock()
	defer g.mu.Unlock()

	if g.limits.MaxConnections > 0 && g.total >= g.limits.MaxConnections {
		return refusedServerFull
	}
	if g.limits.MaxConnectionsPerIP > 0 && g.perIP[ip] >= g.limits.MaxConnectionsPerIP {
		return refusedIPSaturated
	}
	if g.limits.ConnectRatePerIP > 0 {
		bucket, ok := g.rate[ip]
		if !ok {
			bucket = newTokenBucket(g.limits.ConnectRatePerIP, g.limits.ConnectBurstPerIP)
			g.rate[ip] = bucket
		}
		if !bucket.allow(time.Now()) {
			return refusedIPRateLimits
		}
	}

	g.total++
	g.perIP[ip]++
	return admitted
}

// release frees the slot an admitted connection held. Safe to call once per
// admitted connection; extra calls are ignored.
func (g *connectionGuard) release(ip string) {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.perIP[ip] <= 0 {
		return
	}
	g.total--
	if g.perIP[ip]--; g.perIP[ip] == 0 {
		delete(g.perIP, ip)
		// The rate bucket is kept only while the address is connecting or
		// connected, so an idle address does not hold memory.
		if b, ok := g.rate[ip]; ok && b.tokens >= b.capacity {
			delete(g.rate, ip)
		}
	}
}

func (g *connectionGuard) counts() (total int, addresses int) {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.total, len(g.perIP)
}

// clientIP extracts the peer address. It trusts X-Forwarded-For only when the
// gateway sets it, which is the deploy shape for every Cyberia hostname.
func clientIP(r *http.Request) string {
	if fwd := r.Header.Get("X-Forwarded-For"); fwd != "" {
		if first, _, found := strings.Cut(fwd, ","); found {
			return strings.TrimSpace(first)
		}
		return strings.TrimSpace(fwd)
	}
	if real := strings.TrimSpace(r.Header.Get("X-Real-Ip")); real != "" {
		return real
	}
	host, _, err := net.SplitHostPort(r.RemoteAddr)
	if err != nil {
		return r.RemoteAddr
	}
	return host
}

// inputLimiter is the per-connection message budget and strike counter. It is
// touched only by that connection's read goroutine.
type inputLimiter struct {
	bucket     *tokenBucket
	strikes    int
	maxStrikes int
	lastDecay  time.Time
}

func newInputLimiter(limits ConnectionLimits) *inputLimiter {
	l := &inputLimiter{maxStrikes: limits.MaxStrikes, lastDecay: time.Now()}
	// A zero rate leaves bucket nil, which allow() reads as "no limit".
	if limits.MessageRate > 0 {
		l.bucket = newTokenBucket(limits.MessageRate, limits.MessageBurst)
	}
	return l
}

// allow charges one message. It reports whether the message may be processed
// and whether the connection has earned a disconnect.
func (l *inputLimiter) allow() (ok bool, evict bool) {
	if l.bucket == nil {
		return true, false // rate limiting disabled
	}
	now := time.Now()
	if decays := int(now.Sub(l.lastDecay) / strikeDecayInterval); decays > 0 {
		l.lastDecay = now
		if l.strikes -= decays; l.strikes < 0 {
			l.strikes = 0
		}
	}
	if l.bucket.allow(now) {
		return true, false
	}
	return false, l.strike()
}

// strike records one protocol or rate violation. It reports whether the
// connection has passed the limit and must be closed.
func (l *inputLimiter) strike() (evict bool) {
	l.strikes++
	return l.maxStrikes > 0 && l.strikes >= l.maxStrikes
}
