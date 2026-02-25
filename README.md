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
*   **Procedural Map Generation**: Game maps, including obstacles and inter-map portals, are generated on server startup.

## Installation & Setup

1.  **Clone the repository:**
    ```sh
    git clone https://github.com/underpostnet/cyberia-server.git
    cd cyberia-server
    ```

2.  **Install Dependencies:**
    Navigate to the project root and run:
    ```sh
    go mod tidy
    ```

3.  **Run the Server:**
    ```sh
    go run main.go
    ```
    The server will start on `:8080` by default.
