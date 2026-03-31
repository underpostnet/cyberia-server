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
- **Item-based Skill System**: Skills are configured per-instance via `skillConfig[]` (triggerItemId → logicEventId mapping)
- **A\* Pathfinding**: Grid-based pathfinding for player movement and bot AI
- **Dynamic Bot AI**: Bots with passive/hostile behaviors using pathfinding and the skill system
- **Per-Entity Color**: Each entity color is resolved from a named palette (`PLAYER`, `OTHER_PLAYER`, `BOT`, etc.) configured per-instance via `gameConfig.colors[]`. Used for solid-color rendering when no sprite sheets are available
- **`SkillRules` config**: Bullet and doppelganger tuning parameters (spawn chances, lifetimes, sizes, speeds) are grouped under `gameConfig.skillRules` in MongoDB and transmitted as a nested proto message

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
