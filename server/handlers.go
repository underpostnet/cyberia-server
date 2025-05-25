package server

import (
	"encoding/json"
	"log"

	"cyberia-server/pathfinding" // Updated import path
)

// clientMessage represents the generic structure of messages from the client.
type clientMessage struct {
	Type string          `json:"type"` // Type of message (e.g., "client_move_request")
	Data json.RawMessage `json:"data"` // Raw JSON payload, to be unmarshaled based on Type
}

// clientMoveRequestData represents the specific data for a "client_move_request" message.
type clientMoveRequestData struct {
	TargetX float64 `json:"target_x"` // Target X-coordinate for movement
	TargetY float64 `json:"target_y"` // Target Y-coordinate for movement
}

// handleClientMessage processes incoming JSON messages from a specific client.
func (is *InstanceServer) handleClientMessage(client *WebSocketClient, message []byte) {
	var msg clientMessage
	if err := json.Unmarshal(message, &msg); err != nil {
		log.Printf("Client %s: ERROR unmarshaling incoming message: %v", client.playerID, err)
		return
	}

	switch msg.Type {
	case "client_move_request":
		var moveData clientMoveRequestData
		if err := json.Unmarshal(msg.Data, &moveData); err != nil {
			log.Printf("Client %s: ERROR unmarshaling move request data: %v", client.playerID, err)
			return
		}
		is.processClientMoveRequest(client, moveData)
	default:
		log.Printf("Client %s: WARNING unknown message type '%s'.", msg.Type, client.playerID)
	}
}

// processClientMoveRequest handles the logic for a client's movement request.
func (is *InstanceServer) processClientMoveRequest(client *WebSocketClient, data clientMoveRequestData) {
	is.instanceState.Mu.RLock() // Access exported Mu
	playerObj, objExists := is.instanceState.Objects[client.playerID]
	is.instanceState.Mu.RUnlock() // Access exported Mu

	if !objExists {
		log.Printf("ERROR: Player object %s not found for move request.", client.playerID)
		errMsg, _ := json.Marshal(map[string]string{"type": "message", "text": "Your player object was not found on the server."})
		select {
		case client.send <- errMsg: // Attempt to send error message back to client
		default:
			log.Printf("WARNING: Failed to send error message to client %s (send channel full).", client.playerID)
		}
		return
	}

	// Calculate path using A* pathfinding.
	// Call FindPath from the pathfinding package
	path, err := pathfinding.FindPath(
		is.instanceState, // Pass the InstanceState to the pathfinding function
		playerObj.X, playerObj.Y,
		data.TargetX, data.TargetY,
	)
	if err != nil {
		log.Printf("WARNING: No path found for player %s from (%.0f,%.0f) to (%.0f,%.0f). Error: %v", client.playerID, playerObj.X, playerObj.Y, data.TargetX, data.TargetY, err)
		errMsg, _ := json.Marshal(map[string]string{"type": "message", "text": "No path found to that location!"})
		select {
		case client.send <- errMsg: // Attempt to send error message back to client
		default:
			log.Printf("WARNING: Failed to send error message to client %s (send channel full).", client.playerID)
		}
		return
	}

	// Update player object's path in the instance state.
	is.instanceState.Mu.Lock() // Access exported Mu
	playerObj.Path = path
	playerObj.PathIndex = 0
	is.instanceState.Mu.Unlock() // Access exported Mu
	log.Printf("Calculated path for %s: %d steps.", client.playerID, len(path))

	// Send 'player_path_update' event to the specific client.
	pathUpdateMsg, _ := json.Marshal(map[string]interface{}{
		"type":      "player_path_update",
		"player_id": client.playerID,
		"path":      path,
	})
	select {
	case client.send <- pathUpdateMsg:
	default:
		log.Printf("WARNING: Failed to send player_path_update to client %s (send channel full).", client.playerID)
	}
}
