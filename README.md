<p align="center">
  <img src="https://www.cyberiaonline.com/assets/splash/apple-touch-icon-precomposed.png" alt="CYBERIA online"/>
</p>

<div align="center">

<h1>cyberia server</h1>

</div>

**`cyberia-server`** is the authoritative simulation runtime for the Cyberia MMO extension on [Underpost Platform](../src/client/public/cyberia-docs/UNDERPOST-PLATFORM.md). It owns world state, advances a fixed-rate simulation tick, drains typed input commands from connected clients, and dispatches AOI-filtered snapshots on a separately-paced replication tick.

It is **not** the content authority — that role belongs to `engine-cyberia`, which serves world configuration over gRPC at boot. It is **not** the render-policy authority — that role belongs to `cyberia-client`, which owns its presentation defaults locally.

## Architecture

Three processes, sequential startup, non-overlapping roles:

```
┌────────────────────────────────────────────────────────────────┐
│  engine-cyberia  (Node.js)         — content authority         │
│  ├─ MongoDB (maps, entities, object layers, atlas metadata)    │
│  ├─ REST API (:4005)                                           │
│  └─ gRPC server (:50051)                                       │
└────────────────────┬───────────────────────────────────────────┘
                     │ gRPC GetFullInstance (boot only)
                     │ → world configuration: AOI radius, economy
                     │   rules, skill rules, equipment rules,
                     │   entity gameplay defaults
                     ▼
┌────────────────────────────────────────────────────────────────┐
│  cyberia-server  (Go, this repo)   — authoritative simulation  │
│  ├─ simulation tick @ tickRate Hz                              │
│  ├─ replication tick @ snapshotRate Hz (decoupled)             │
│  ├─ WebSocket (:8081/ws)  → binary AOI snapshots               │
│  │                        ← typed input commands               │
│  ├─ REST API (:8081/api/v1/*)  → health, metrics               │
│  └─ Static file server  (WASM client via STATIC_DIR)           │
└────────────────────┬───────────────────────────────────────────┘
                     │ WebSocket binary
                     ▼
┌────────────────────────────────────────────────────────────────┐
│  cyberia-client  (C / WASM)        — presentation runtime      │
│  ├─ Raylib + Emscripten WebAssembly                            │
│  ├─ prediction · reconciliation · interpolation                │
│  ├─ AOI snapshot decoder                                       │
│  └─ client-owned presentation defaults + optional client hints │
└────────────────────────────────────────────────────────────────┘
```

Architecture is documented end-to-end in [`ARCHITECTURE.md`](../src/client/public/cyberia-docs/ARCHITECTURE.md). The umbrella product is [Underpost Platform](../src/client/public/cyberia-docs/UNDERPOST-PLATFORM.md).

### Startup order — sequential

1. **Persistent backend / sidecar data layer** — databases, `engine-cyberia` (gRPC + REST), static asset backend.
2. **cyberia-server** — dials engine-cyberia gRPC, loads world configuration, opens WebSocket.
3. **cyberia-client** — connects to cyberia-server WebSocket.

Do not orchestrate these in parallel. The server exits on gRPC dial failure rather than fabricate a world.

## Core capabilities

- **Tick-based simulation** — fixed-rate `tickRate` (default 30 Hz), `dt`-based integration with `tickDuration`. Frame-count integration is not used anywhere on the server.
- **Decoupled replication** — AOI snapshots emitted at `snapshotRate` (default 20 Hz), independent of the simulation tick.
- **Typed input command pipeline** — binary WebSocket frames decoded once into `InputCommand{kind, clientTick, sequence, payload}`, enqueued per-player, drained by `phaseInput` under the world mutex. No JSON intermediate on the binary path; no synchronous game-state mutation on the WebSocket read goroutine.
- **Simulation phases** — `phaseInput → phaseLifecycle → phaseSkills → phaseAI → phaseMovement → phasePortals`. Phases are the only functions allowed to mutate authoritative world state.
- **Binary AOI snapshot protocol** — compact wire format with `tick` and `lastAcked` in the header so clients can reconcile their predicted self against authoritative state.
- **gRPC world load + hot reload** — world configuration loaded once at boot via `GetFullInstance`; ObjectLayer cache hot-reloaded by sha256 manifest diff.
- **A\* pathfinding** — grid-based, used by both player movement (when input commands request a destination) and bot AI.
- **Dynamic bot AI** — passive/hostile behaviors driven by the same skill system as players.
- **Item-based skill system** — `skillConfig[]` maps a `triggerItemId` to an ordered `logicEventIds[]`; configured per-instance and shipped via gRPC.
- **Off-chain economy** — Fountain & Sink coin model, tracked in-memory; blockchain integration is not active in the current configuration.

### Strict ownership boundary

`cyberia-server` holds **no** presentation state. There is no field on the server for palette, status-icon iconId or border color, camera smoothing or zoom, dev-overlay flag, screen-factor overrides, or interpolation window. Those live entirely in `cyberia-client`'s compile-time defaults, with optional per-instance overrides served by engine-cyberia at `GET /api/cyberia-client-hints/:instanceCode`. The Go process never calls that endpoint.

A small internal RGBA table inside `sim_palette.go` exists solely to fill the optional per-entity color bytes the AOI wire carries for portals, skill projectiles, and freshly spawned players. The client treats those bytes as a hint and resolves the actual fallback color from its own palette.

## Passive Stats System

Each `ObjectLayer` item carries a `Stats` struct. The server computes an entity's effective stats by summing all **active** layers' stat values, then recursively adding the caster's stats if the entity was summoned (e.g. a bullet or doppelganger inherits its caster's stats).

| Stat             | Effect                                                      | Formula                                                                                                  |
| ---------------- | ----------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| **Effect**       | Life removed on collision/impact                            | Damage dealt = `bulletStats.Effect`; "thorns" reverse damage applies to the attacker                     |
| **Resistance**   | Expands `MaxLife` cap and increases regen heal amount       | `MaxLife = entityBaseMaxLife + Resistance`                                                               |
| **Agility**      | Movement speed multiplier                                   | `speed = entityBaseSpeed × (1 + Agility / 100)`                                                          |
| **Range**        | Lifetime of summoned entities (bullets, doppelgangers)      | `ExpiresAt += Range ms`                                                                                  |
| **Intelligence** | Spawn probability for summoned entities                     | `P(spawn) = min(baseChance + Intelligence / 100, maxChance)`                                             |
| **Utility**      | Action cooldown reduction + probabilistic life regen chance | `cooldown = baseCooldown × (1 − Utility / 100)`; `P(regen) = min(baseChance + Utility / 100, maxChance)` |

All percentage formulas are consistent: **1 stat point = 1%** scaling across Effect, Agility, Intelligence, and Utility.

## Skill System

Skills map a **trigger item** (the active `ObjectLayer` item equipped by a player or bot) to an ordered list of **logic event IDs**. This mapping is configured per-instance in `CyberiaInstanceConf.skillConfig[]` and transmitted to the Go server via gRPC at startup.

```
ObjectLayer item (equipped + active)
    └─ skillConfig: triggerItemId → skills[]
           ├─ "projectile"              → executeProjectileSkill
           ├─ "doppelganger"            → executeDoppelgangerSkill
           └─ "coin_drop_or_transaction" → no-op on action (fires automatically on kill via HandleOnKillSkills)
```

**Skill trigger flow:**

1. Player sends `player_action` WebSocket message (rate-limited by `CalculateActionCooldown`)
2. `HandlePlayerActionSkills` iterates active `ObjectLayers`, checks each `ItemID` against `skillConfig`
3. For each matching `SkillDefinition`, iterates `LogicEventIDs` in order and dispatches to the corresponding handler
4. Bots follow the same path via `handleBotSkills`, called when they acquire a new target or wander

**Economy skill (`coin_drop_or_transaction`):**  
Registered in `skillConfig` so the C client's `skillMap` correctly associates the coin item. The actual loot transfer (`executeCoinDropOnKill`) fires automatically when a bullet kills an entity — no player action required.

**`SkillRules` tuning parameters** (all configurable per-instance):

| Parameter                         | Description                                             |
| --------------------------------- | ------------------------------------------------------- |
| `bulletSpawnChance`               | Base probability of spawning a bullet on each action    |
| `bulletLifetimeMs`                | Base bullet travel duration before expiry               |
| `bulletWidth/Height`              | Bullet hitbox dimensions (grid units)                   |
| `bulletSpeedMultiplier`           | Bullet speed as a multiple of `entityBaseSpeed`         |
| `doppelgangerSpawnChance`         | Base probability of spawning a doppelganger clone       |
| `doppelgangerLifetimeMs`          | Doppelganger wander duration before expiry              |
| `doppelgangerSpawnRadius`         | Wander radius around the caster's position (grid units) |
| `doppelgangerInitialLifeFraction` | Fraction of `entityBaseMaxLife` the clone spawns with   |

## Installation

```sh
go mod tidy
```

## Environment Variables

| Variable                          | Default           | Description                                                             |
| --------------------------------- | ----------------- | ----------------------------------------------------------------------- |
| `ENGINE_GRPC_ADDRESS`             | `localhost:50051` | Engine gRPC server address — **required** (server exits if unreachable) |
| `INSTANCE_CODE`                   | `default`         | Instance code to load (`default` → Engine returns fallback)             |
| `ENGINE_API_BASE_URL`             | _(empty)_         | Engine HTTP URL (forwarded to clients)                                  |
| `ENGINE_GRPC_RELOAD_INTERVAL_SEC` | _(disabled)_      | ObjectLayer hot-reload interval in seconds                              |
| `SERVER_PORT`                     | `8081`            | HTTP/WS listen port                                                     |
| `STATIC_DIR`                      | `./public`        | Static file directory (WASM client)                                     |
| `ENGINE_GRPC_CA_CERT`             | _(empty)_         | CA certificate for mTLS                                                 |
| `ENGINE_GRPC_CLIENT_CERT`         | _(empty)_         | Client certificate for mTLS                                             |
| `ENGINE_GRPC_CLIENT_KEY`          | _(empty)_         | Client private key for mTLS                                             |

## Development

Create a `.env` file:

```sh
ENGINE_GRPC_ADDRESS=localhost:50051
INSTANCE_CODE=cyberia-main
ENGINE_API_BASE_URL=http://localhost:4005
SERVER_PORT=8081
```

Run (requires Node.js Engine running with gRPC on :50051):

```sh
go run main.go
```

> **Note**: The Engine must be running before starting `cyberia-server`. If `INSTANCE_CODE` is not set or doesn't match a database record, the Engine returns a minimal fallback instance automatically — the Go server will still start successfully.

### Full local development stack

```sh
# Terminal 1: Engine (gRPC :50051 + REST :4005)
cd /home/dd/engine && npm run dev

# Terminal 2: Go server (WS :8081)
cd /home/dd/engine/cyberia-server && go run main.go

# Terminal 3: C/WASM client (HTTP :8082)
cd /home/dd/engine/cyberia-client
source ~/.emsdk/emsdk_env.sh
make -f Web.mk serve-development
```

### Serving the WASM client from the Go server

```sh
# Build WASM client
cd /home/dd/engine/cyberia-client
make -f Web.mk clean && make -f Web.mk web
cp -r bin/web/debug/* /home/dd/engine/cyberia-server/public/

# Go server serves both WS and static files on :8081
cd /home/dd/engine/cyberia-server
STATIC_DIR=./public go run main.go
```

## Production

```sh
ENGINE_GRPC_ADDRESS=<engine-clusterIP>:50051 \
INSTANCE_CODE=cyberia-main \
ENGINE_API_BASE_URL=https://www.cyberiaonline.com \
SERVER_PORT=8081 \
STATIC_DIR=./public \
./cyberia-server
```

Build:

```sh
go build -o cyberia-server .
```

## REST API

```
GET /api/v1/health              → {"status":"ok"}
GET /api/v1/metrics             → full metrics response
GET /api/v1/metrics/health      → {"health":"healthy","uptime_sec":...}
GET /api/v1/metrics/entities    → entity counts by type
GET /api/v1/metrics/websocket   → {"status":"running","active_connections":N}
GET /api/v1/metrics/workload    → {"load_percentage":X,"current_load":"low|medium|high|critical"}
```

## Proto

Service definition: [`proto/cyberia.proto`](proto/cyberia.proto)

Regenerate Go code after editing:

```sh
export PATH=$PATH:$(go env GOPATH)/bin
protoc --go_out=. --go_opt=paths=source_relative \
       --go-grpc_out=. --go-grpc_opt=paths=source_relative \
       proto/cyberia.proto
```
