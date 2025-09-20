<p align="center">
  <img src="https://www.cyberiaonline.com/assets/splash/apple-touch-icon-precomposed.png" alt="CYBERIA online"/>
</p>

<div align="center">

<h1>cyberia server</h1>

</div>

Core Go-based server for **Cyberia online MMO instance**. It facilitates low-latency client communication via WebSockets, manages a dynamic game world state, and employs an A\* pathfinding algorithm for intelligent object movement.

## Core Features

*   **Real-time Multiplayer**: Uses WebSockets for low-latency communication between clients and the server.
*   **Area of Interest (AOI)**: Efficiently broadcasts game state updates only for entities within a player's visible range, reducing network traffic and client-side processing.
*   **Dynamic Bot AI**: Bots with distinct behaviors (passive, hostile) that navigate the world using A* pathfinding and can engage with players using the same skill system.
*   **Item-based Skill System**: A flexible system where items, represented as `ObjectLayer`s, can grant players and bots active or passive skills.
*   **RESTful Management API**: A secure, role-based API (using JWTs) for managing game assets like items and user accounts.
*   **Procedural Map Generation**: Game maps, including obstacles and inter-map portals, are generated on server startup.

## Installation & Setup

1.  **Clone the repository:**
    ```sh
    git clone https://github.com/underpostnet/cyberia-server.git
    cd cyberia-server
    ```

2.  **Configure Environment:**
    The server is configured via environment variables. You can create a `.env` file in the root directory for local development.

    Here is a minimal example `.env` file:
    ```env
    # MongoDB connection string
    MONGO_URI="mongodb://localhost:27017"

    # Secret for signing JWTs
    JWT_SECRET="your-super-secret-key-that-is-long"

    # Initial admin user to be seeded on first run
    SEED_ADMIN="true"
    ADMIN_USERNAME="admin"
    ADMIN_EMAIL="admin@example.com"
    ADMIN_PASSWORD="your-secure-password"

    # Path to the static frontend files
    STATIC_DIR="../engine/src/client/public/cyberia"
    ```

3.  **Install Dependencies:**
    Navigate to the project root and run:
    ```sh
    go mod tidy
    ```

4.  **Run the Server:**
    ```sh
    go run main.go
    ```
    The server will start on `:8080` by default.

