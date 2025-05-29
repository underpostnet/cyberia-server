# Cyberia Server

`cyberia-server` is a Go-based server for a real-time multiplayer online (MMO) instance. It facilitates low-latency client communication via WebSockets, manages a dynamic game world state, and employs an A\* pathfinding algorithm for intelligent object movement.

## Core Features

- **Real-time Communication:** Utilizes WebSockets for efficient, persistent client connections.
- **Dynamic Network State:** Manages all in-game entities (players, obstacles, temporary objects) within a shared, synchronized world state.
- **Pathfinding:** Implements the A\* algorithm for robust and efficient object navigation.
- **Concurrent Operations:** Designed to handle multiple clients and concurrent world updates leveraging Go's goroutines and concurrency primitives.

## Architecture

The server's architecture is modular, primarily organized into the following Go packages:

- **`server/`**: Manages WebSocket connections, client lifecycle, and message handling.
- **`network_state/`**: Defines and manages the core game world state, including network objects, the grid, and the simplified maze for pathfinding.
- **`pathfinding/`**: Provides the A\* algorithm implementation for calculating optimal paths within the game world.

## Getting Started

### Prerequisites

- Go (version 1.18+ recommended)

### Building and Running

1.  **Build the executable:**

    ```bash
    go build -o cyberia-server .
    ```

2.  **Run the server:**

    ```bash
    ./cyberia-server
    ```

    The server will typically listen for WebSocket connections on `ws://0.0.0.0:5000/ws`.
