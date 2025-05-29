package server

import (
	"encoding/json"
	"log"

	"cyberia-server/pathfinding"
	// Updated import path
)

// clientMessage represents the generic structure of messages from the client.
type clientMessage struct {
	Type string          `json:"type"`
	Data json.RawMessage `json:"data"`
}

// clientMoveRequestData represents the specific data for a "client_move_request" message.
type clientMoveRequestData struct {
	TargetX float64 `json:"target_x"`
	TargetY float64 `json:"target_y"`
}

// handleClientMessage processes incoming JSON messages from a specific client.
func (ns *NetworkStateServer) handleClientMessage(client *WebSocketClient, message []byte) {
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
		ns.processClientMoveRequest(client, moveData)
	default:
		log.Printf("Client %s: WARNING unknown message type '%s'.", msg.Type, client.playerID)
	}
}

// processClientMoveRequest handles the logic for a client's movement request.
func (ns *NetworkStateServer) processClientMoveRequest(client *WebSocketClient, data clientMoveRequestData) {
	ns.networkState.Mu.RLock()
	playerObj, objExists := ns.networkState.NetworkObjects[client.playerID]
	ns.networkState.Mu.RUnlock()

	if !objExists {
		log.Printf("ERROR: Player network object %s not found for move request.", client.playerID)
		errMsg, _ := json.Marshal(map[string]string{"type": "message", "text": "Your player network object was not found on the server."})
		select {
		case client.send <- errMsg:
		default:
			log.Printf("WARNING: Failed to send error message to client %s (send channel full).", client.playerID)
		}
		return
	}

	path, err := pathfinding.FindPath(
		ns.networkState,
		playerObj.X, playerObj.Y,
		data.TargetX, data.TargetY,
	)
	if err != nil {
		log.Printf("WARNING: No path found for player %s from (%.0f,%.0f) to (%.0f,%.0f). Error: %v", client.playerID, playerObj.X, playerObj.Y, data.TargetX, data.TargetY, err)
		errMsg, _ := json.Marshal(map[string]string{"type": "message", "text": "No path found to that location!"})
		select {
		case client.send <- errMsg:
		default:
			log.Printf("WARNING: Failed to send error message to client %s (send channel full).", client.playerID)
		}
		return
	}

	ns.networkState.Mu.Lock()
	playerObj.Path = path
	playerObj.PathIndex = 0
	ns.networkState.Mu.Unlock()
	log.Printf("Calculated path for %s: %d steps.", client.playerID, len(path))

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
