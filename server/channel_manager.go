package server

import (
	"cyberia-server/config"
	"cyberia-server/network_state"
	"log"
	"sync"
	"time"
)

// ChannelManager manages all active game channels.
type ChannelManager struct {
	channels      map[string]*Channel
	channelsMutex sync.RWMutex
	factory       *network_state.ServerNetworkObjectFactory
}

// NewChannelManager creates a new ChannelManager.
func NewChannelManager() *ChannelManager {
	cm := &ChannelManager{
		channels: make(map[string]*Channel),
		factory:  network_state.NewServerNetworkObjectFactory(time.Now().UnixNano()),
	}
	// Initialize default channels
	for _, channelID := range config.AvailableChannels {
		cm.getOrCreateChannel(channelID)
	}
	return cm
}

func (cm *ChannelManager) getOrCreateChannel(channelID string) *Channel {
	cm.channelsMutex.RLock()
	ch, exists := cm.channels[channelID]
	cm.channelsMutex.RUnlock()

	if !exists {
		cm.channelsMutex.Lock()
		// Double check after acquiring write lock
		ch, exists = cm.channels[channelID]
		if !exists {
			log.Printf("Creating new channel: %s", channelID)
			ch = NewChannel(channelID, cm.factory)
			cm.channels[channelID] = ch
		}
		cm.channelsMutex.Unlock()
	}
	return ch
}

// AddClientToChannel adds a client to a specific channel.
func (cm *ChannelManager) AddClientToChannel(client *WebSocketClient, channelID string) {
	ch := cm.getOrCreateChannel(channelID)
	if ch != nil {
		ch.AddClient(client)
	} else {
		log.Printf("Error: Could not get or create channel %s for client %s", channelID, client.playerID)
	}
}

// RemoveClientFromChannel removes a client from their current channel.
func (cm *ChannelManager) RemoveClientFromChannel(client *WebSocketClient) {
	if client.ChannelID == "" {
		return // Client not in any channel
	}
	cm.channelsMutex.RLock()
	ch, exists := cm.channels[client.ChannelID]
	cm.channelsMutex.RUnlock()

	if exists {
		ch.RemoveClient(client)
	}
}

// SwitchClientChannel moves a client from their current channel to a new one.
func (cm *ChannelManager) SwitchClientChannel(client *WebSocketClient, newChannelID string) {
	cm.RemoveClientFromChannel(client)          // Remove from old channel
	cm.AddClientToChannel(client, newChannelID) // Add to new channel
	log.Printf("Client %s switched to channel %s", client.playerID, newChannelID)
}

func (cm *ChannelManager) GetChannel(channelID string) (*Channel, bool) {
	cm.channelsMutex.RLock()
	defer cm.channelsMutex.RUnlock()
	ch, exists := cm.channels[channelID]
	return ch, exists
}

// CloseAllChannels gracefully shuts down all channels.
func (cm *ChannelManager) CloseAllChannels() {
	cm.channelsMutex.Lock()
	defer cm.channelsMutex.Unlock()
	for _, ch := range cm.channels {
		ch.Close()
	}
	log.Println("All channels closed.")
}
