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
│  ├─ WebSocket (:8080/ws) → binary AOI protocol                 │
│  ├─ REST API (:8080/api/v1/*) → health, metrics                │
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
- **Fallback Instance**: If no instance exists in the database or gRPC is unavailable, the server creates a minimal empty map with auto-generated floors and obstacles for immediate multiplayer testing
- **Item-based Skill System**: Skills are configured per-instance via `skillConfig[]` (triggerItemId → logicEventId mapping)
- **A\* Pathfinding**: Grid-based pathfinding for player movement and bot AI
- **Dynamic Bot AI**: Bots with passive/hostile behaviors using pathfinding and the skill system
- **Per-Entity Color**: Players and bots have an RGBA color for solid-color rendering when no sprite sheets are available

## Installation

```sh
go mod tidy
```

## Environment Variables

| Variable                          | Default             | Description                                         |
| --------------------------------- | ------------------- | --------------------------------------------------- |
| `ENGINE_GRPC_ADDRESS`             | _(empty → no gRPC)_ | Engine gRPC server address (e.g. `localhost:50051`) |
| `INSTANCE_CODE`                   | _(empty)_           | Instance code to load (e.g. `cyberia-main`)         |
| `ENGINE_API_BASE_URL`             | _(empty)_           | Engine HTTP URL (forwarded to clients)              |
| `ENGINE_GRPC_RELOAD_INTERVAL_SEC` | _(disabled)_        | ObjectLayer hot-reload interval in seconds          |
| `SERVER_PORT`                     | `8080`              | HTTP/WS listen port                                 |
| `STATIC_DIR`                      | `./public`          | Static file directory (WASM client)                 |
| `ENGINE_GRPC_CA_CERT`             | _(empty)_           | CA certificate for mTLS                             |
| `ENGINE_GRPC_CLIENT_CERT`         | _(empty)_           | Client certificate for mTLS                         |
| `ENGINE_GRPC_CLIENT_KEY`          | _(empty)_           | Client private key for mTLS                         |

## Development

Create a `.env` file:

```sh
ENGINE_GRPC_ADDRESS=localhost:50051
INSTANCE_CODE=cyberia-main
ENGINE_API_BASE_URL=http://localhost:4005
SERVER_PORT=8080
```

Run (requires Node.js Engine running with gRPC on :50051):

```sh
go run main.go
```

### Without Engine (standalone)

The server runs without gRPC. It creates a fallback empty map for multiplayer testing:

```sh
# No .env needed — server starts with empty world + fallback map
go run main.go
```

Players connect via WebSocket and appear as solid colored rectangles (no sprites, no items).

### Full local development stack

```sh
# Terminal 1: Engine (gRPC :50051 + REST :4005)
cd /home/dd/engine && npm run dev

# Terminal 2: Go server (WS :8080)
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

# Go server serves both WS and static files on :8080
cd /home/dd/engine/cyberia-server
STATIC_DIR=./public go run main.go
```

## Production

```sh
ENGINE_GRPC_ADDRESS=<engine-clusterIP>:50051 \
INSTANCE_CODE=cyberia-main \
ENGINE_API_BASE_URL=https://www.cyberiaonline.com \
SERVER_PORT=8080 \
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
