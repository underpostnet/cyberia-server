package game

import (
	"sync/atomic"
	"time"
)

// runtimeCounters holds lock-free, monotonically increasing counters
// used by the metrics API. They are mutated on the websocket read/write
// hot paths and the gameLoop, so every field is accessed through the
// sync/atomic package — taking GameServer.mu on each frame would push
// contention into the simulation loop.
//
// Counters reset to zero on process restart; rate calculations are the
// responsibility of the caller (delta over wall-clock window).
type runtimeCounters struct {
	wsMessagesIn       atomic.Uint64
	wsMessagesOut      atomic.Uint64
	wsBytesIn          atomic.Uint64
	wsBytesOut         atomic.Uint64
	wsReadErrors       atomic.Uint64
	wsWriteErrors      atomic.Uint64
	wsConnectsTotal    atomic.Uint64
	wsDisconnectsTotal atomic.Uint64

	// Last observed tick + wall time. Snapshotted on every read so a
	// caller can derive a tick/s rate without holding the world lock.
	lastObservedTick atomic.Uint32
	lastObservedAt   atomic.Int64 // unix nanoseconds
}

// RuntimeMetricsSnapshot is a read-only view of the runtime counters
// at a single instant. Fields are unmarshalled directly into the API
// response so the JSON tags match the wire contract.
type RuntimeMetricsSnapshot struct {
	TickRate           int     `json:"tick_rate_hz"`
	SnapshotRate       int     `json:"snapshot_rate_hz"`
	CurrentTick        uint32  `json:"current_tick"`
	TickDurationMs     float64 `json:"tick_duration_ms"`
	AoiRadius          float64 `json:"aoi_radius"`
	WsMessagesIn       uint64  `json:"ws_messages_in_total"`
	WsMessagesOut      uint64  `json:"ws_messages_out_total"`
	WsBytesIn          uint64  `json:"ws_bytes_in_total"`
	WsBytesOut         uint64  `json:"ws_bytes_out_total"`
	WsReadErrors       uint64  `json:"ws_read_errors_total"`
	WsWriteErrors      uint64  `json:"ws_write_errors_total"`
	WsConnectsTotal    uint64  `json:"ws_connects_total"`
	WsDisconnectsTotal uint64  `json:"ws_disconnects_total"`
}

// recordWsRead is called from readPump whenever a frame is decoded
// successfully. Bytes excludes the websocket framing overhead.
func (s *GameServer) recordWsRead(n int) {
	s.counters.wsMessagesIn.Add(1)
	s.counters.wsBytesIn.Add(uint64(n))
}

// recordWsWrite is called from writePump after a successful Close()
// (which is the flush boundary for gorilla/websocket).
func (s *GameServer) recordWsWrite(n int) {
	s.counters.wsMessagesOut.Add(1)
	s.counters.wsBytesOut.Add(uint64(n))
}

func (s *GameServer) recordWsReadError()  { s.counters.wsReadErrors.Add(1) }
func (s *GameServer) recordWsWriteError() { s.counters.wsWriteErrors.Add(1) }
func (s *GameServer) recordWsConnect()    { s.counters.wsConnectsTotal.Add(1) }
func (s *GameServer) recordWsDisconnect() { s.counters.wsDisconnectsTotal.Add(1) }

// RuntimeMetrics returns a lock-free snapshot of the simulation cadence
// counters and per-process throughput. Safe to call from the HTTP API
// without contending with the simulation tick.
func (s *GameServer) RuntimeMetrics() RuntimeMetricsSnapshot {
	// tickRate/snapshotRate/aoiRadius are only mutated inside
	// ApplyInstanceConfig, which holds s.mu. Read under the same lock
	// to avoid a torn read on the float64 / int fields.
	s.mu.Lock()
	tickRate := s.tickRate
	snapRate := s.snapshotRate
	tickDur := s.tickDuration
	aoi := s.aoiRadius
	tick := s.currentTick
	s.mu.Unlock()

	s.counters.lastObservedTick.Store(tick)
	s.counters.lastObservedAt.Store(time.Now().UnixNano())

	return RuntimeMetricsSnapshot{
		TickRate:           tickRate,
		SnapshotRate:       snapRate,
		CurrentTick:        tick,
		TickDurationMs:     float64(tickDur.Microseconds()) / 1000.0,
		AoiRadius:          aoi,
		WsMessagesIn:       s.counters.wsMessagesIn.Load(),
		WsMessagesOut:      s.counters.wsMessagesOut.Load(),
		WsBytesIn:          s.counters.wsBytesIn.Load(),
		WsBytesOut:         s.counters.wsBytesOut.Load(),
		WsReadErrors:       s.counters.wsReadErrors.Load(),
		WsWriteErrors:      s.counters.wsWriteErrors.Load(),
		WsConnectsTotal:    s.counters.wsConnectsTotal.Load(),
		WsDisconnectsTotal: s.counters.wsDisconnectsTotal.Load(),
	}
}

// MapBreakdown is a per-map summary used by the dashboard map table.
type MapBreakdown struct {
	MapCode     string `json:"map_code"`
	GridW       int    `json:"grid_w"`
	GridH       int    `json:"grid_h"`
	Players     int    `json:"players"`
	Bots        int    `json:"bots"`
	Floors      int    `json:"floors"`
	Obstacles   int    `json:"obstacles"`
	Foregrounds int    `json:"foregrounds"`
	Portals     int    `json:"portals"`
	Resources   int    `json:"resources"`
	Total       int    `json:"total_entities"`
}

// HasMaps reports whether the gRPC WorldBuilder has finished its
// initial LoadAll. Used by the readiness probe to surface 503 until
// the world is ready.
func (s *GameServer) HasMaps() bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return len(s.maps) > 0
}

// EntityTypeCensus pairs an entity-type count with the ObjectLayer
// totals carried by entities of that type. Every entity in the world
// (player, bot, floor, obstacle, foreground, portal, resource) holds
// an `ObjectLayers` slice; the API used to assume otherwise and
// silently zeroed out the static types, producing misleading totals.
type EntityTypeCensus struct {
	Count                int
	TotalObjectLayers    int
	ActiveObjectLayers   int
	InactiveObjectLayers int
}

// EntityCensus returns the count + object-layer breakdown for every
// entity type, computed in a single locked traversal. Callers should
// prefer this over GetEntityCounts + per-type GetXxxObjectLayers,
// which acquired the world lock four times and disagreed with the
// per-map breakdown on totals for static entities.
//
// The returned map always contains the same seven keys so that
// callers can render a stable, ordered table without nil checks.
func (s *GameServer) EntityCensus() map[string]EntityTypeCensus {
	s.mu.Lock()
	defer s.mu.Unlock()

	out := map[string]EntityTypeCensus{
		"player":     {},
		"bot":        {},
		"floor":      {},
		"obstacle":   {},
		"foreground": {},
		"portal":     {},
		"resource":   {},
	}

	addObj := func(key string, layers []ObjectLayerState) {
		c := out[key]
		c.Count++
		for _, l := range layers {
			c.TotalObjectLayers++
			if l.Active {
				c.ActiveObjectLayers++
			} else {
				c.InactiveObjectLayers++
			}
		}
		out[key] = c
	}

	for _, m := range s.maps {
		for _, p := range m.players {
			addObj("player", p.ObjectLayers)
		}
		for _, b := range m.bots {
			addObj("bot", b.ObjectLayers)
		}
		for _, f := range m.floors {
			addObj("floor", f.ObjectLayers)
		}
		for _, o := range m.obstacles {
			addObj("obstacle", o.ObjectLayers)
		}
		for _, fg := range m.foregrounds {
			addObj("foreground", fg.ObjectLayers)
		}
		for _, p := range m.portals {
			addObj("portal", p.ObjectLayers)
		}
		for _, r := range m.resources {
			addObj("resource", r.ObjectLayers)
		}
	}
	return out
}

// MapBreakdowns returns a per-map entity census in stable map-code order.
// Stable order lets the dashboard render deterministically.
func (s *GameServer) MapBreakdowns() []MapBreakdown {
	s.mu.Lock()
	defer s.mu.Unlock()

	out := make([]MapBreakdown, 0, len(s.maps))
	for code, m := range s.maps {
		b := MapBreakdown{
			MapCode:     code,
			GridW:       m.gridW,
			GridH:       m.gridH,
			Players:     len(m.players),
			Bots:        len(m.bots),
			Floors:      len(m.floors),
			Obstacles:   len(m.obstacles),
			Foregrounds: len(m.foregrounds),
			Portals:     len(m.portals),
			Resources:   len(m.resources),
		}
		b.Total = b.Players + b.Bots + b.Floors + b.Obstacles + b.Foregrounds + b.Portals + b.Resources
		out = append(out, b)
	}
	sortBreakdowns(out)
	return out
}

// MapBreakdownByCode returns the breakdown for a specific map, or
// (zero, false) if no map with that code is loaded.
func (s *GameServer) MapBreakdownByCode(code string) (MapBreakdown, bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	m, ok := s.maps[code]
	if !ok {
		return MapBreakdown{}, false
	}
	b := MapBreakdown{
		MapCode:     code,
		GridW:       m.gridW,
		GridH:       m.gridH,
		Players:     len(m.players),
		Bots:        len(m.bots),
		Floors:      len(m.floors),
		Obstacles:   len(m.obstacles),
		Foregrounds: len(m.foregrounds),
		Portals:     len(m.portals),
		Resources:   len(m.resources),
	}
	b.Total = b.Players + b.Bots + b.Floors + b.Obstacles + b.Foregrounds + b.Portals + b.Resources
	return b, true
}

// sortBreakdowns sorts in-place by MapCode ascending. Implemented inline
// to avoid pulling sort just for one call site.
func sortBreakdowns(b []MapBreakdown) {
	for i := 1; i < len(b); i++ {
		for j := i; j > 0 && b[j].MapCode < b[j-1].MapCode; j-- {
			b[j], b[j-1] = b[j-1], b[j]
		}
	}
}
