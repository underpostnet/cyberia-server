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
//
// What does NOT live here
//   - JSON parsing of any kind. Inputs arrive pre-decoded as typed
//     InputCommand values from handlers.go.
//   - Lock management. phaseInput holds the world lock for the entire
//     drain pass.
//   - Pathfinder rate limiting. Movement re-plans at most once per player per
//     tick because phaseInput calls flushPendingMove once; the handlers only
//     record intent.

package game

import (
	"math"

	"cyberia-server/logx"
)

// handlePlayerActionInput applies a TAP from the client: skill trigger now,
// movement intent recorded for the end of the phase.
//
// Skills fire on every valid tap, probability-gated by Intelligence. Movement
// does not re-plan here: taps that arrive inside one tick all describe the same
// instant, and only the newest of them names where the player wants to go, so
// the intent is recorded and phaseInput re-plans once per tick from the last
// one. That coalescing is the whole bound on pathfinder cost — one A* per
// player per tick whatever the tap rate — and it costs the player nothing,
// because the tap it drops is one a later tap in the same tick superseded.
// Nothing else may throttle the re-plan: a walk that cannot turn until some
// cooldown elapses reads as input lag and sets off in the abandoned direction.
func (s *GameServer) handlePlayerActionInput(player *PlayerState, mapState *MapState, cmd *InputCommand) {
	if player.IsGhost() {
		return
	}
	if player.Frozen {
		return
	}

	// Skills + regen run on every accepted tap.
	target := Point{X: cmd.TargetX, Y: cmd.TargetY}
	s.HandlePlayerTapAction(player, mapState, target)

	player.PendingMove = PointI{X: int(math.Round(cmd.TargetX)), Y: int(math.Round(cmd.TargetY))}
	player.PendingMoveSequence = cmd.Sequence
	player.HasPendingMove = true
}

// flushPendingMove re-plans the walk from the newest tap of this tick. Called
// once per player per tick by phaseInput, after the input queue is drained.
//
// Every re-plan stamps LastMovementSequence, which the snapshot echoes as
// moveAck. Arrival is not acceptance — a superseded tap is acknowledged and
// never planned — so the client needs moveAck to know which command the route
// it is being handed was planned for.
func (s *GameServer) flushPendingMove(player *PlayerState, mapState *MapState) {
	if !player.HasPendingMove {
		return
	}
	target := player.PendingMove
	sequence := player.PendingMoveSequence
	player.HasPendingMove = false

	// Re-check the refusals the recording handler already applied: a FreezeStart
	// or a death later in the same queue lands between the record and this
	// flush, and the tap must not outlive it.
	if player.IsGhost() || player.Frozen {
		return
	}

	if sequence > player.LastMovementSequence {
		player.LastMovementSequence = sequence
	}

	// Clamp into the map. An off-grid target would otherwise make Astar expand
	// the full reachable area and then fail, spending the tick budget.
	startPosI := clampCellToGrid(
		PointI{X: int(math.Round(player.Pos.X)), Y: int(math.Round(player.Pos.Y))},
		mapState.gridW, mapState.gridH)
	targetPosI := clampCellToGrid(target, mapState.gridW, mapState.gridH)

	// A repeat of the destination already being walked needs no search. This is
	// the shape a spam-tapping client takes, and it is also the shape of a held
	// steering key, so skipping it removes the cheapest attack and the most
	// common redundant search at once.
	if player.Mode == WALKING && player.TargetPos == targetPosI && len(player.Path) > 0 {
		return
	}

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
	target, ok := mapState.players[toID]
	if !ok {
		return
	}
	sendMessage(target, "chat", map[string]interface{}{
		"from": sender.ID,
		"text": text,
	})
}
