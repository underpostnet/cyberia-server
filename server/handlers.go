package server

import (
	"encoding/json"
	// "cyberia-server/config" // Already implicitly used via server/channel.go -> network_state -> config
	"log"
	"time" // New: Import time for timestamps

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

// clientChatMessageData represents the specific data for a "client_chat_message" message.
type clientChatMessageData struct {
	RoomID string `json:"room_id"`
	Text   string `json:"text"`
}

// clientChangeChannelRequestData represents data for changing channels.
type clientChangeChannelRequestData struct {
	ChannelID string `json:"channel_id"`
}

// handleClientMessage processes incoming JSON messages from a specific client.
func (ns *NetworkStateServer) handleClientMessage(client *WebSocketClient, message []byte) {
	var msg clientMessage
	if err := json.Unmarshal(message, &msg); err != nil {
		log.Printf("Client %s: ERROR unmarshaling incoming message: %v", client.playerID, err)
		return
	}

	currentChannel, exists := ns.channelManager.GetChannel(client.ChannelID)
	if !exists {
		log.Printf("Client %s: ERROR current channel %s not found.", client.playerID, client.ChannelID)
		return
	}
	switch msg.Type {
	case "client_move_request":
		var moveData clientMoveRequestData
		if err := json.Unmarshal(msg.Data, &moveData); err != nil {
			log.Printf("Client %s: ERROR unmarshaling move request data: %v", client.playerID, err)
			return
		}
		ns.processClientMoveRequest(client, currentChannel, moveData)
	case "client_chat_message": // New: Handle chat messages
		var chatData clientChatMessageData
		if err := json.Unmarshal(msg.Data, &chatData); err != nil {
			log.Printf("Client %s: ERROR unmarshaling chat message data: %v", client.playerID, err)
			return
		}
		ns.processClientChatMessage(client, currentChannel, chatData) // Pass currentChannel
	case "client_change_channel_request":
		var changeChannelData clientChangeChannelRequestData
		if err := json.Unmarshal(msg.Data, &changeChannelData); err != nil {
			log.Printf("Client %s: ERROR unmarshaling change channel data: %v", client.playerID, err)
			return
		}
		ns.processClientChangeChannelRequest(client, changeChannelData.ChannelID)
	default:
		log.Printf("Client %s in channel %s: WARNING unknown message type '%s'.", client.playerID, client.ChannelID, msg.Type)
	}
}

// processClientMoveRequest handles the logic for a client's movement request.
func (ns *NetworkStateServer) processClientMoveRequest(client *WebSocketClient, channel *Channel, data clientMoveRequestData) {
	channel.networkState.Mu.RLock()
	playerObj, objExists := channel.networkState.NetworkObjects[client.playerID]
	channel.networkState.Mu.RUnlock()

	if !objExists {
		log.Printf("ERROR: Player network object %s not found in channel %s for move request.", client.playerID, channel.ID)
		errMsg, _ := json.Marshal(map[string]string{
			"type": "error_message", // Use a distinct type for errors
			"text": "Your player object was not found in the current channel.",
		})
		select {
		case client.send <- errMsg:
		default:
			log.Printf("WARNING: Failed to send error message to client %s (send channel full).", client.playerID)
		}
		return
	}

	path, err := pathfinding.FindPath(
		channel.networkState, // Use channel's network state
		playerObj.X, playerObj.Y,
		data.TargetX, data.TargetY,
	)
	if err != nil {
		log.Printf("Channel %s: WARNING: No path found for player %s from (%.0f,%.0f) to (%.0f,%.0f). Error: %v", channel.ID, client.playerID, playerObj.X, playerObj.Y, data.TargetX, data.TargetY, err)
		errMsg, _ := json.Marshal(map[string]string{
			"type": "error_message",
			"text": "No path found to that location in this channel!",
		})
		select {
		case client.send <- errMsg:
		default:
			log.Printf("WARNING: Failed to send error message to client %s (send channel full).", client.playerID)
		}
		return
	}

	channel.networkState.Mu.Lock()
	playerObj.Path = path
	playerObj.PathIndex = 0
	channel.networkState.Mu.Unlock()
	log.Printf("Channel %s: Calculated path for %s: %d steps.", channel.ID, client.playerID, len(path))

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

// processClientChatMessage handles a client's chat message request.
func (ns *NetworkStateServer) processClientChatMessage(client *WebSocketClient, channel *Channel, data clientChatMessageData) {
	// Validate room ID and message content
	if data.RoomID == "" || data.Text == "" {
		log.Printf("Channel %s: Client %s: Invalid chat message (empty room ID or text).", channel.ID, client.playerID)
		return
	}

	// Create a new ChatMessage
	message := ChatMessage{
		RoomID:    data.RoomID, // Store RoomID with the message
		Sender:    client.playerID,
		Text:      data.Text,
		Timestamp: time.Now().Format("15:04"), // Format: HH:MM
	}

	channel.AddChatMessage(message) // Add to channel-specific chat history

	log.Printf("Channel %s: Client %s sent chat message to room %s: %s", channel.ID, client.playerID, data.RoomID, data.Text)

	// Broadcast the message to all clients
	ns.sendChatMessageToChannelClients(channel, message)
}

// processClientChangeChannelRequest handles a client's request to switch channels.
func (ns *NetworkStateServer) processClientChangeChannelRequest(client *WebSocketClient, newChannelID string) {
	if client.ChannelID == newChannelID {
		log.Printf("Client %s already in channel %s. No change.", client.playerID, newChannelID)
		return
	}

	_, newChannelExists := ns.channelManager.GetChannel(newChannelID)
	if !newChannelExists {
		log.Printf("Client %s requested to switch to non-existent channel %s.", client.playerID, newChannelID)
		// Optionally send an error message back to the client
		return
	}

	ns.channelManager.SwitchClientChannel(client, newChannelID)

	// Send confirmation and new channel's initial state (or player_assigned with channel_id)
	// The AddClientToChannel in ChannelManager should handle creating the player object.
	// The new channel's broadcast loop will eventually send the full state.
	// We can send a specific "channel_assigned" message.
	channelAssignedMsg, _ := json.Marshal(map[string]interface{}{
		"type":       "channel_assigned", // Client should handle this
		"channel_id": newChannelID,
		// player_id is already known by client, but can be re-sent if player object is recreated
	})
	client.send <- channelAssignedMsg
}
