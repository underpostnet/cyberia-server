// Package game — phase_input_handlers.go
//
// Typed handlers consumed by phaseInput. Each function applies one
// InputCommand to authoritative world state, assuming the caller already
// holds s.mu (phaseInput runs inside the simulation tick).
//
// What lives here
//   - Movement / skill dispatch for PLAYER_ACTION taps.
//   - Inventory mutation for ITEM_ACTIVATION (with equipment-rule validation).
//   - Chat relay.
//   - Skill→item lookup response (get_items_ids).
//
// What does NOT live here
//   - JSON parsing of any kind. Inputs arrive pre-decoded as typed
//     InputCommand values from handlers.go.
//   - Lock management. phaseInput holds the world lock for the entire
//     drain pass.
//   - Per-WS-goroutine state. Cooldown tracking still uses
//     Client.lastAction (mutated under s.mu inside this file) for now.

package game

import (
	"encoding/json"
	"math"
	"time"

	"cyberia-server/logx"
)

// handlePlayerActionInput applies a TAP from the client: movement intent
// + skill trigger. Skills fire on every valid tap (probability-gated);
// movement re-planning is additionally gated by the per-player action
// cooldown so spam-tapping doesn't churn the pathfinder.
func (s *GameServer) handlePlayerActionInput(player *PlayerState, mapState *MapState, cmd *InputCommand) {
	if player.IsGhost() {
		return
	}
	if player.Frozen {
		return
	}

	stats := s.CalculateStats(player, mapState)
	cooldown := s.CalculateActionCooldown(stats)
	movementReady := player.Client == nil || time.Since(player.Client.lastAction) >= cooldown
	if movementReady && player.Client != nil {
		player.Client.lastAction = time.Now()
	}

	// Skills + regen run on every accepted tap.
	target := Point{X: cmd.TargetX, Y: cmd.TargetY}
	s.HandlePlayerTapAction(player, mapState, target)

	if !movementReady {
		return
	}

	startPosI := PointI{X: int(math.Round(player.Pos.X)), Y: int(math.Round(player.Pos.Y))}
	targetPosI := PointI{X: int(math.Round(cmd.TargetX)), Y: int(math.Round(cmd.TargetY))}

	newPath, err := mapState.pathfinder.Astar(startPosI, targetPosI, player.Dims)
	usedTarget := targetPosI
	if err != nil {
		closest, cerr := mapState.pathfinder.findClosestWalkablePoint(targetPosI, player.Dims)
		if cerr != nil {
			s.faceTargetWithoutPath(player, float64(targetPosI.X), float64(targetPosI.Y))
			player.TargetPos = targetPosI
			return
		}
		newPath, err = mapState.pathfinder.Astar(startPosI, closest, player.Dims)
		if err != nil {
			s.faceTargetWithoutPath(player, float64(closest.X), float64(closest.Y))
			player.TargetPos = closest
			return
		}
		usedTarget = closest
	}

	if len(newPath) > 0 {
		first := newPath[0]
		player.Path = newPath
		player.TargetPos = usedTarget
		player.Mode = WALKING
		dx := float64(first.X) - player.Pos.X
		dy := float64(first.Y) - player.Pos.Y
		if dist := math.Sqrt(dx*dx + dy*dy); dist > 0 {
			s.updatePlayerDirection(player, dx/dist, dy/dist)
		}
	} else if startPosI.X == targetPosI.X && startPosI.Y == targetPosI.Y {
		player.Mode = IDLE
	}
}

// faceTargetWithoutPath updates direction/mode when pathfinding fails but
// we still want the entity to face the tap. Extracted from the JSON
// handler so both PlayerAction fallback branches share one helper.
func (s *GameServer) faceTargetWithoutPath(player *PlayerState, tx, ty float64) {
	dx := tx - player.Pos.X
	dy := ty - player.Pos.Y
	dist := math.Sqrt(dx*dx + dy*dy)
	if dist > 0 {
		s.updatePlayerDirection(player, dx/dist, dy/dist)
		player.Mode = WALKING
	} else {
		player.Mode = IDLE
	}
}

// handleItemActivationInput toggles an inventory slot and re-validates
// the equipment rules.
func (s *GameServer) handleItemActivationInput(player *PlayerState, cmd *InputCommand) {
	itemID := cmd.ItemID
	active := cmd.Active

	isDead := player.IsGhost() || player.Life <= 0
	isDeadItem := s.deadItemIDs[itemID]

	// Dead-state items are the Fragmented State's aesthetic loadout — they
	// equip only while dead, under the same equipment rules as live items.
	if !isDead && isDeadItem {
		logx.Debugf("Player %s tried to equip dead-state item '%s' while alive; rejecting.", player.ID, itemID)
		return
	}

	// Live-item activations while dead queue onto PreRespawnObjectLayers.
	if isDead && !isDeadItem {
		if active {
			s.queuePreRespawnActivation(player, itemID)
		}
		return
	}

	targetItemIndex := -1
	for i := range player.ObjectLayers {
		if player.ObjectLayers[i].ItemID == itemID {
			targetItemIndex = i
			break
		}
	}
	if targetItemIndex == -1 {
		logx.Debugf("Player %s tried to activate non-existent item '%s'.", player.ID, itemID)
		return
	}

	originalState := player.ObjectLayers[targetItemIndex].Active
	player.ObjectLayers[targetItemIndex].Active = active

	// Type guard — only activeItemTypes may be turned active.
	if active && len(s.equipmentRules.ActiveItemTypes) > 0 {
		reqType := s.itemType(itemID)
		if reqType != "" && !s.equipmentRules.ActiveItemTypes[reqType] {
			logx.Debugf("Player %s tried to activate '%s' (type=%q) — not in activeItemTypes; rejecting.",
				player.ID, itemID, reqType)
			player.ObjectLayers[targetItemIndex].Active = originalState
			return
		}
	}

	// Rules operate within the current state's item set: dead items during
	// the Fragmented State, live items otherwise.
	inState := func(id string) bool { return s.deadItemIDs[id] == isDead }

	// OnePerType — deactivate any other active in-state item with the same type.
	if active && s.equipmentRules.OnePerType {
		requested := s.itemType(itemID)
		if requested != "" {
			for i := range player.ObjectLayers {
				if i == targetItemIndex {
					continue
				}
				if !player.ObjectLayers[i].Active || !inState(player.ObjectLayers[i].ItemID) {
					continue
				}
				if s.itemType(player.ObjectLayers[i].ItemID) == requested {
					player.ObjectLayers[i].Active = false
				}
			}
		}
	}

	// RequireSkin / maxActiveLayers corrections, scoped to in-state items.
	activeLayerCount, activeSkinCount, hasAnySkin, firstSkinIndex := 0, 0, false, -1
	for i, layer := range player.ObjectLayers {
		isSkin := inState(layer.ItemID) && s.itemType(layer.ItemID) == "skin"
		if isSkin {
			hasAnySkin = true
			if firstSkinIndex == -1 {
				firstSkinIndex = i
			}
			if layer.Active {
				activeSkinCount++
			}
		}
		if layer.Active {
			activeLayerCount++
		}
	}
	if s.equipmentRules.RequireSkin && activeSkinCount == 0 {
		if isDead && s.ghostItemID != "" {
			// Unequipping the last dead skin is allowed: the ghost item from
			// gRPC config takes over as the Fragmented State visual.
			activateOrAppendLayer(&player.ObjectLayers, s.ghostItemID)
		} else if hasAnySkin && firstSkinIndex != -1 {
			player.ObjectLayers[firstSkinIndex].Active = true
		}
	}
	if activeLayerCount > s.maxActiveLayers {
		player.ObjectLayers[targetItemIndex].Active = originalState
	}

	// The Fragmented State loadout persists across deaths.
	if isDead {
		player.DeadLoadoutItemIDs = activeObjectLayerItemIDs(player.ObjectLayers)
	}

	s.InvalidateStats(player)
	s.ApplyResistanceStat(player, s.maps[player.MapCode])
}

// queuePreRespawnActivation marks a pending activation that takes effect
// when the dead player respawns. Extracted to keep handleItemActivationInput
// readable.
func (s *GameServer) queuePreRespawnActivation(player *PlayerState, itemID string) {
	if player.PreRespawnObjectLayers == nil {
		return
	}
	for i := range player.PreRespawnObjectLayers {
		if player.PreRespawnObjectLayers[i].ItemID != itemID {
			continue
		}
		player.PreRespawnObjectLayers[i].Active = true
		logx.Debugf("Player %s is dead — queued activation of '%s' on revive.", player.ID, itemID)
		if s.equipmentRules.OnePerType {
			reqType := s.itemType(itemID)
			if reqType != "" {
				for j := range player.PreRespawnObjectLayers {
					if j == i {
						continue
					}
					if player.PreRespawnObjectLayers[j].Active &&
						s.itemType(player.PreRespawnObjectLayers[j].ItemID) == reqType {
						player.PreRespawnObjectLayers[j].Active = false
					}
				}
			}
		}
		return
	}
}

// itemType returns the ObjectLayer item type for an itemID, or "" if the
// ObjectLayer cache has no entry. Pure helper — no state mutation.
func (s *GameServer) itemType(itemID string) string {
	if data, ok := s.GetObjectLayerData(itemID); ok {
		return data.Data.Item.Type
	}
	return ""
}

// handleChatInput is a pure relay — no game-state mutation. Forwards a
// chat message to the target player on the same map.
func (s *GameServer) handleChatInput(sender *PlayerState, mapState *MapState, cmd *InputCommand) {
	toID := cmd.ItemID // chat target id reuses the ItemID slot
	text := cmd.ChatText
	if toID == "" || text == "" {
		return
	}
	if len(text) > 256 {
		text = text[:256]
	}
	relay, err := json.Marshal(map[string]interface{}{
		"type": "chat",
		"payload": map[string]interface{}{
			"from": sender.ID,
			"text": text,
		},
	})
	if err != nil {
		return
	}
	target, ok := mapState.players[toID]
	if !ok || target.Client == nil {
		return
	}
	select {
	case target.Client.send <- relay:
	default:
		// Receiver channel full — drop. Chat is not durable.
	}
}

// handleGetItemsIDsInput answers a synchronous skill-item-id query. The
// response goes back to the requesting player's WS write channel.
func (s *GameServer) handleGetItemsIDsInput(player *PlayerState, cmd *InputCommand) {
	if cmd.ItemID == "" || player.Client == nil {
		return
	}
	associated := s.GetAssociatedSkillItemIDs(cmd.ItemID)
	response, err := json.Marshal(map[string]interface{}{
		"type": "skill_item_ids",
		"payload": map[string]interface{}{
			"requestedItemId":   cmd.ItemID,
			"associatedItemIds": associated,
		},
	})
	if err != nil {
		return
	}
	select {
	case player.Client.send <- response:
	default:
	}
}
