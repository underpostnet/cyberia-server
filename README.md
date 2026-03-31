<p align="center">
  <img src="https://www.cyberiaonline.com/assets/splash/apple-touch-icon-precomposed.png" alt="CYBERIA online"/>
</p>

<div align="center">

<h1>cyberia server</h1>

</div>

Go game server for **Cyberia Online MMO**. Manages real-time multiplayer state via WebSockets using a binary AOI protocol, with all game data sourced from the Node.js Engine via gRPC.

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│  Node.js Engine                                                │
│  ├─ MongoDB (instances, maps, entities, ObjectLayers, atlases) │
│  ├─ Express REST API (:4005)                                   │
│  └─ gRPC server (:50051)                                       │
└────────────────────┬───────────────────────────────────────────┘
                     │ gRPC (cluster-internal)
                     ▼
┌────────────────────────────────────────────────────────────────┐
│  Go Game Server (this repo)                                    │
│  ├─ gRPC client → WorldBuilder → ApplyInstanceConfig           │
│  ├─ In-memory game state (maps, entities, pathfinding)         │
│  ├─ WebSocket (:8081/ws) → binary AOI protocol                 │
│  ├─ REST API (:8081/api/v1/*) → health, metrics                │
│  └─ Static file server (WASM client via STATIC_DIR)            │
└────────────────────┬───────────────────────────────────────────┘
                     │ WebSocket (binary)
                     ▼
┌────────────────────────────────────────────────────────────────┐
│  C/WASM Client                                                 │
│  ├─ Raylib + Emscripten WebAssembly                            │
│  ├─ Binary AOI decoder (positions, directions, colors, items)  │
│  └─ Atlas sprite sheet renderer                                │
└────────────────────────────────────────────────────────────────┘
```

## Core Features

- **Binary AOI Protocol**: Compact binary WebSocket messages with only render-essential data (positions, directions, modes, colors, item stacks)
- **gRPC Data Pipeline**: All game configuration, maps, entities, ObjectLayers, and AtlasSpriteSheets loaded from the Engine at startup via gRPC
- **Hot-Reload**: ObjectLayer cache diffing via sha256 manifests — surgical in-memory replacement without restart
- **Fallback Instance**: When the requested instance is not found in MongoDB, the Engine (`grpc-server.js`) returns a minimal playable instance with a 64×64 floor grid and default config. The Go server always requires gRPC — it exits on connection failure
- **Item-based Skill System**: Skills are configured per-instance via `skillConfig[]` (`triggerItemId` → `logicEventIds[]` ordered array mapping)
- **A\* Pathfinding**: Grid-based pathfinding for player movement and bot AI
- **Dynamic Bot AI**: Bots with passive/hostile behaviors using pathfinding and the skill system
- **Per-Entity Color**: Each entity color is resolved from a named palette (`PLAYER`, `OTHER_PLAYER`, `BOT`, etc.) configured per-instance via `colors[]`. Used for solid-color rendering when no sprite sheets are available
- **`SkillRules` config**: Bullet and doppelganger tuning parameters (spawn chances, lifetimes, sizes, speeds) are grouped under `skillRules` in `CyberiaInstanceConf` and transmitted as a nested proto message
- **Off-chain Economy**: Coin quantities are tracked in-memory as `ObjectLayerState.Quantity` on entity object layers. Blockchain integration is **not active** — the economy operates fully off-chain

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
    └─ skillConfig: triggerItemId → logicEventIds[]
           ├─ "atlas_pistol_mk2_logic"   → executePlayerBulletSkill / executeBotBulletSkill
           ├─ "doppelganger"             → executePlayerDoppelgangerSkill / executeBotDoppelgangerSkill
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
