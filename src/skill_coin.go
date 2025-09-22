package game

import "log"

// executeCoinDropOnKill handles the logic for transferring coins from a killed entity to the killer.
// This is a core mechanic, not a conditional skill.
func (s *GameServer) executeCoinDropOnKill(caster interface{}, victim interface{}, server *GameServer) {
	// 1. Find the victim's coin layer and check if they have coins.
	var victimCoinLayer *ObjectLayerState
	var victimID string
	var victimObjectLayers []ObjectLayerState

	switch v := victim.(type) {
	case *PlayerState:
		victimID = v.ID
		victimObjectLayers = v.ObjectLayers
	case *BotState:
		victimID = v.ID
		victimObjectLayers = v.ObjectLayers
	default:
		log.Printf("executeCoinDropOnKill called with unknown victim type")
		return
	}

	for i := range victimObjectLayers {
		if victimObjectLayers[i].ItemID == "coin" {
			victimCoinLayer = &victimObjectLayers[i]
			break
		}
	}

	if victimCoinLayer == nil || victimCoinLayer.Quantity <= 0 {
		return // Victim has no coins to drop.
	}

	// 2. Find or create the caster's coin layer.
	var casterCoinLayer *ObjectLayerState
	var casterID string

	casterCoinLayer, casterID = s.findOrCreateCoinLayer(caster)
	if casterCoinLayer == nil {
		log.Printf("executeCoinDropOnKill failed to find or create coin layer for caster")
		return
	}

	// 3. Perform the transaction.
	amountToTransfer := victimCoinLayer.Quantity
	casterCoinLayer.Quantity += amountToTransfer
	victimCoinLayer.Quantity = 0

	log.Printf("Caster %s triggered 'coin_drop_or_transaction', looting %d coins from %s.", casterID, amountToTransfer, victimID)

}

// findOrCreateCoinLayer finds the coin layer for a given entity (player or bot).
// If the entity doesn't have a coin layer, it creates one.
// It returns the coin layer and the entity's ID.
func (s *GameServer) findOrCreateCoinLayer(entity interface{}) (*ObjectLayerState, string) {
	switch e := entity.(type) {
	case *PlayerState:
		for i := range e.ObjectLayers {
			if e.ObjectLayers[i].ItemID == "coin" {
				return &e.ObjectLayers[i], e.ID
			}
		}
		// If player doesn't have a coin layer, add one.
		e.ObjectLayers = append(e.ObjectLayers, ObjectLayerState{ItemID: "coin", Active: false, Quantity: 0})
		return &e.ObjectLayers[len(e.ObjectLayers)-1], e.ID
	case *BotState:
		for i := range e.ObjectLayers {
			if e.ObjectLayers[i].ItemID == "coin" {
				return &e.ObjectLayers[i], e.ID
			}
		}
		// If bot doesn't have a coin layer, add one.
		e.ObjectLayers = append(e.ObjectLayers, ObjectLayerState{ItemID: "coin", Active: false, Quantity: 0})
		return &e.ObjectLayers[len(e.ObjectLayers)-1], e.ID
	default:
		return nil, ""
	}
}
