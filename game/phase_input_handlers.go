// Package game — phase_input_handlers.go
//
// Typed handlers consumed by phaseInput. Each function applies one
// InputCommand to authoritative world state, assuming the caller already
// holds s.mu (phaseInput runs inside the simulation tick).
//
// What lives here
//   - Movement / skill dispatch for PLAYER_ACTION taps.
//   - Inventory mutation for ITEM_ACTIVATION (with equipment-rule validation).
//     ITEM_USE lives in consumable.go.
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

// Taps record movement and request cooldown-limited skills.
func (s *GameServer) handlePlayerActionInput(player *Session, mapState *MapState, cmd *InputCommand) {
	if mapState.vt(player.ID).ghost() {
		return
	}
	if player.Frozen {
		return
	}

	target := Vec2{X: cmd.TargetX, Y: cmd.TargetY}
	s.HandlePlayerTapAction(player, mapState, target)

	player.PendingMove = Cell{X: int(math.Round(cmd.TargetX)), Y: int(math.Round(cmd.TargetY))}
	player.HasPendingMove = true
}

// flushPendingMove re-plans the walk from the newest tap of this tick. Called
// once per player per tick by phaseInput, after the input queue is drained.
func (s *GameServer) flushPendingMove(player *Session, mapState *MapState) {
	if !player.HasPendingMove {
		return
	}
	target := player.PendingMove
	player.HasPendingMove = false

	// Re-check the refusals the recording handler already applied: a player_stasis
	// or a death later in the same queue lands between the record and this
	// flush, and the tap must not outlive it.
	if mapState.vt(player.ID).ghost() || player.Frozen {
		return
	}

	// Clamp into the map. An off-grid target would otherwise make Astar expand
	// the full reachable area and then fail, spending the tick budget.
	startPosI := clampCellToGrid(
		Cell{X: int(math.Round(mapState.tf(player.ID).Pos.X)), Y: int(math.Round(mapState.tf(player.ID).Pos.Y))},
		mapState.gridW, mapState.gridH)
	targetPosI := clampCellToGrid(target, mapState.gridW, mapState.gridH)

	// A repeat of the destination already being walked needs no search. This is
	// the shape a spam-tapping client takes, and it is also the shape of a held
	// steering key, so skipping it removes the cheapest attack and the most
	// common redundant search at once.
	if mapState.mo(player.ID).Mode == WALKING && player.TargetPos == targetPosI && len(player.Path) > 0 {
		return
	}

	newPath, err := mapState.pathfinder.Astar(startPosI, targetPosI, mapState.tf(player.ID).Size)
	usedTarget := targetPosI
	if err != nil {
		closest, cerr := mapState.pathfinder.findClosestWalkablePoint(targetPosI, mapState.tf(player.ID).Size)
		if cerr != nil {
			s.faceTargetWithoutPath(player, mapState, float64(targetPosI.X), float64(targetPosI.Y))
			player.TargetPos = targetPosI
			return
		}
		newPath, err = mapState.pathfinder.Astar(startPosI, closest, mapState.tf(player.ID).Size)
		if err != nil {
			s.faceTargetWithoutPath(player, mapState, float64(closest.X), float64(closest.Y))
			player.TargetPos = closest
			return
		}
		usedTarget = closest
	}

	if len(newPath) > 0 {
		first := newPath[0]
		player.Path = newPath
		player.TargetPos = usedTarget
		mapState.mo(player.ID).Mode = WALKING
		dx := float64(first.X) - mapState.tf(player.ID).Pos.X
		dy := float64(first.Y) - mapState.tf(player.ID).Pos.Y
		if dist := math.Sqrt(dx*dx + dy*dy); dist > 0 {
			mapState.mo(player.ID).Direction = directionFromVector(dx/dist, dy/dist)
		}
	} else if startPosI.X == targetPosI.X && startPosI.Y == targetPosI.Y {
		mapState.mo(player.ID).Mode = IDLE
	}
}

// faceTargetWithoutPath updates direction/mode when pathfinding fails but
// we still want the entity to face the tap. Extracted from the JSON
// handler so both PlayerAction fallback branches share one helper.
func (s *GameServer) faceTargetWithoutPath(player *Session, mapState *MapState, tx, ty float64) {
	dx := tx - mapState.tf(player.ID).Pos.X
	dy := ty - mapState.tf(player.ID).Pos.Y
	dist := math.Sqrt(dx*dx + dy*dy)
	if dist > 0 {
		mapState.mo(player.ID).Direction = directionFromVector(dx/dist, dy/dist)
		mapState.mo(player.ID).Mode = WALKING
	} else {
		mapState.mo(player.ID).Mode = IDLE
	}
}

// handleItemActivationInput toggles an inventory slot and re-validates
// the equipment rules.
func (s *GameServer) handleItemActivationInput(player *Session, cmd *InputCommand) {
	itemID := cmd.ItemID
	active := cmd.Active

	v := s.vt(player.ID, player.MapCode)
	isDead := v.ghost() || v.Life <= 0
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
	for i := range s.ap(player.ID, player.MapCode).Layers {
		if s.ap(player.ID, player.MapCode).Layers[i].ItemID == itemID {
			targetItemIndex = i
			break
		}
	}
	if targetItemIndex == -1 {
		logx.Debugf("Player %s tried to activate non-existent item '%s'.", player.ID, itemID)
		return
	}

	originalState := s.ap(player.ID, player.MapCode).Layers[targetItemIndex].Active
	s.ap(player.ID, player.MapCode).Layers[targetItemIndex].Active = active

	// Type guard — only activeItemTypes may be turned active.
	if active && len(s.equipmentRules.ActiveItemTypes) > 0 {
		reqType := s.itemType(itemID)
		if reqType != "" && !s.equipmentRules.ActiveItemTypes[reqType] {
			logx.Debugf("Player %s tried to activate '%s' (type=%q) — not in activeItemTypes; rejecting.",
				player.ID, itemID, reqType)
			s.ap(player.ID, player.MapCode).Layers[targetItemIndex].Active = originalState
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
			for i := range s.ap(player.ID, player.MapCode).Layers {
				if i == targetItemIndex {
					continue
				}
				if !s.ap(player.ID, player.MapCode).Layers[i].Active || !inState(s.ap(player.ID, player.MapCode).Layers[i].ItemID) {
					continue
				}
				if s.itemType(s.ap(player.ID, player.MapCode).Layers[i].ItemID) == requested {
					s.ap(player.ID, player.MapCode).Layers[i].Active = false
				}
			}
		}
	}

	// RequireSkin / maxActiveLayers corrections, scoped to in-state items.
	activeLayerCount, activeSkinCount, hasAnySkin, firstSkinIndex := 0, 0, false, -1
	for i, layer := range s.ap(player.ID, player.MapCode).Layers {
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
			activateOrAppendLayer(&s.ap(player.ID, player.MapCode).Layers, s.ghostItemID)
		} else if hasAnySkin && firstSkinIndex != -1 {
			s.ap(player.ID, player.MapCode).Layers[firstSkinIndex].Active = true
		}
	}
	if activeLayerCount > s.maxActiveLayers {
		s.ap(player.ID, player.MapCode).Layers[targetItemIndex].Active = originalState
	}

	// The Fragmented State loadout persists across deaths.
	if isDead {
		player.DeadLoadoutItemIDs = activeObjectLayerItemIDs(s.ap(player.ID, player.MapCode).Layers)
	}

	s.ApplyResistanceStat(player.ID, s.maps[player.MapCode])
}

// queuePreRespawnActivation marks a pending activation that takes effect
// when the dead player respawns. Extracted to keep handleItemActivationInput
// readable.
func (s *GameServer) queuePreRespawnActivation(player *Session, itemID string) {
	layers := s.maps[player.MapCode].mt(player.ID).PreRespawnObjectLayers
	if layers == nil {
		return
	}
	for i := range layers {
		if layers[i].ItemID != itemID {
			continue
		}
		layers[i].Active = true
		logx.Debugf("Player %s is dead — queued activation of '%s' on revive.", player.ID, itemID)
		if s.equipmentRules.OnePerType {
			reqType := s.itemType(itemID)
			if reqType != "" {
				for j := range layers {
					if j == i {
						continue
					}
					if layers[j].Active &&
						s.itemType(layers[j].ItemID) == reqType {
						layers[j].Active = false
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
func (s *GameServer) handleChatInput(sender *Session, mapState *MapState, cmd *InputCommand) {
	toID := cmd.EntityID
	text := cmd.ChatText
	if toID == 0 || text == "" {
		return
	}
	if len(text) > 256 {
		text = text[:256]
	}
	target := s.playerOn(mapState, toID)
	if target == nil {
		return
	}
	sendMessage(target, "chat", map[string]interface{}{
		"from": sender.ID,
		"text": text,
	})
}
