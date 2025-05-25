# cyberia-server

`cyberia-server` is a Go-based backend server for a massively multiplayer online (MMO) instance. It uses WebSockets for real-time communication, manages the instance world's state, and incorporates A\* pathfinding for object movement.

## Features

- **Real-time Communication:** Uses WebSockets for client interaction.

- **Instance World:** Manages all objects (players, obstacles) within a single instance.

- **Pathfinding:** Objects navigate the world using the A\* algorithm.

- **Concurrent Operations:** Handles many clients and instance updates efficiently with Go's goroutines.

## Project Structure

The project is organized into the following key packages:

- `main.go`: The server's entry point.

- `server/`: Contains WebSocket server logic and client communication.

  - `server.go`: Defines the core `InstanceServer`.

  - `client.go`: Manages individual `WebSocketClient` connections.

  - `handlers.go`: Processes incoming client messages.

- `instance/`: Manages the instance world's state.

  - `state.go`: Defines the `InstanceState` and its world manipulation methods.

  - `object.go`: Defines the `InstanceObject` (entities in the world).

  - `constants.go`: Stores instance-specific constants.

- `pathfinding/`: Implements the A\* pathfinding algorithm.

  - `astar.go`: Contains the A\* algorithm logic.

## Getting Started

### Prerequisites

- Go (version 1.18 or higher recommended)

- `go.mod` for dependency management (handled by `go run` or `go build`)

### Running the Server

1.  **Clone the repository:**

    ```bash
    git clone <your-repo-url>/cyberia-server.git
    cd cyberia-server
    ```

2.  **Run the server:**

    ```bash
    go run main.go
    ```

    The server will start on `http://0.0.0.0:5000` and listen for WebSocket connections on `ws://0.0.0.0:5000/ws`.

## Usage

Clients connect to the server via WebSockets at `ws://<server-ip>:5000/ws`.

### Client-Server Communication

- **Client Sends `client_move_request`:**

  ```json
  {
    "type": "client_move_request",
    "data": {
      "target_x": 123.45,
      "target_y": 678.9
    }
  }
  ```

- **Server Broadcasts `instance_state_update`:**

  ```json
  {
    "type": "instance_state_update",
    "objects": {
      "player_1": {
        "obj_id": "player_1",
        "x": 100.0,
        "y": 150.0,
        "color": { "R": 0, "G": 0, "B": 255, "A": 255 },
        "is_obstacle": false,
        "speed": 200,
        "path": [
          { "X": 100, "Y": 200 },
          { "X": 150, "Y": 250 }
        ],
        "path_index": 0
      },
      "obstacle_0": {
        "obj_id": "obstacle_0",
        "x": 200.0,
        "y": 200.0,
        "color": { "R": 80, "G": 80, "B": 80, "A": 255 },
        "is_obstacle": true,
        "speed": 0,
        "path": null,
        "path_index": 0
      }
      // ... other objects
    }
  }
  ```

- **Server Sends `player_assigned` (on connect):**

  ```json
  {
    "type": "player_assigned",
    "player_id": "player_1",
    "x": 500.0,
    "y": 500.0
  }
  ```

- **Server Sends `player_path_update` (to specific client):**

  ```json
  {
    "type": "player_path_update",
    "player_id": "player_1",
    "path": [
      { "X": 100, "Y": 200 },
      { "X": 150, "Y": 250 },
      { "X": 200, "Y": 300 }
    ]
  }
  ```
