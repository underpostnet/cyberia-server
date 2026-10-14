// Package sim — phase_input_handlers.go
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

package sim

import (
	"math"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// Taps record movement and request cooldown-limited skills.
func (s *Sim) handlePlayerActionInput(player *world.Session, mapState *world.MapState, cmd *world.InputCommand) {
	if mapState.Vt(player.ID).Ghost() {
		return
	}
	if player.Frozen {
		return
	}

	target := world.Vec2{X: cmd.TargetX, Y: cmd.TargetY}
	s.HandlePlayerTapAction(player, mapState, target)

	player.PendingMove = world.Cell{X: int(math.Round(cmd.TargetX)), Y: int(math.Round(cmd.TargetY))}
	player.HasPendingMove = true
}

// flushPendingMove re-plans the walk from the newest tap of this tick. Called
// once per player per tick by phaseInput, after the input queue is drained.
func (s *Sim) flushPendingMove(player *world.Session, mapState *world.MapState) {
	if !player.HasPendingMove {
		return
	}
	target := player.PendingMove
	player.HasPendingMove = false

	// Re-check the refusals the recording handler already applied: a player_stasis
	// or a death later in the same queue lands between the record and this
	// flush, and the tap must not outlive it.
	if mapState.Vt(player.ID).Ghost() || player.Frozen {
		return
	}

	// Clamp into the map. An off-grid target would otherwise make Astar expand
	// the full reachable area and then fail, spending the tick budget.
	startPosI := world.ClampCellToGrid(
		world.Cell{X: int(math.Round(mapState.Tf(player.ID).Pos.X)), Y: int(math.Round(mapState.Tf(player.ID).Pos.Y))},
		mapState.GridW(), mapState.GridH())
	targetPosI := world.ClampCellToGrid(target, mapState.GridW(), mapState.GridH())

	// A repeat of the destination already being walked needs no search. This is
	// the shape a spam-tapping client takes, and it is also the shape of a held
	// steering key, so skipping it removes the cheapest attack and the most
	// common redundant search at once.
	if mapState.Mo(player.ID).Mode == world.WALKING && player.TargetPos == targetPosI && len(player.Path) > 0 {
		return
	}

	newPath, err := mapState.Pathfinder().Astar(startPosI, targetPosI, mapState.Tf(player.ID).Size)
	usedTarget := targetPosI
	if err != nil {
		closest, cerr := mapState.Pathfinder().FindClosestWalkablePoint(targetPosI, mapState.Tf(player.ID).Size)
		if cerr != nil {
			s.faceTargetWithoutPath(player, mapState, float64(targetPosI.X), float64(targetPosI.Y))
			player.TargetPos = targetPosI
			return
		}
		newPath, err = mapState.Pathfinder().Astar(startPosI, closest, mapState.Tf(player.ID).Size)
		if err != nil {
			s.faceTargetWithoutPath(player, mapState, float64(closest.X), float64(closest.Y))
			player.TargetPos = closest
			return
		}
		usedTarget = closest
	}

	if len(newPath) > 0 {
		// The search starts at the rounded cell. Drop each node the player has passed.
		pos := mapState.Tf(player.ID).Pos
		for len(newPath) > 1 {
			head, next := newPath[0], newPath[1]
			if (float64(head.X)-pos.X)*float64(next.X-head.X)+(float64(head.Y)-pos.Y)*float64(next.Y-head.Y) >= 0 {
				break
			}
			newPath = newPath[1:]
		}
		first := newPath[0]
		player.Path = newPath
		player.TargetPos = usedTarget
		mapState.Mo(player.ID).Mode = world.WALKING
		dx := float64(first.X) - mapState.Tf(player.ID).Pos.X
		dy := float64(first.Y) - mapState.Tf(player.ID).Pos.Y
		if dist := math.Sqrt(dx*dx + dy*dy); dist > 0 {
			mapState.Mo(player.ID).Direction = directionFromVector(dx/dist, dy/dist)
		}
	} else if startPosI.X == targetPosI.X && startPosI.Y == targetPosI.Y {
		mapState.Mo(player.ID).Mode = world.IDLE
	}
}

// faceTargetWithoutPath updates direction/mode when pathfinding fails but
// we still want the entity to face the tap. Extracted from the JSON
// handler so both PlayerAction fallback branches share one helper.
func (s *Sim) faceTargetWithoutPath(player *world.Session, mapState *world.MapState, tx, ty float64) {
	dx := tx - mapState.Tf(player.ID).Pos.X
	dy := ty - mapState.Tf(player.ID).Pos.Y
	dist := math.Sqrt(dx*dx + dy*dy)
	if dist > 0 {
		mapState.Mo(player.ID).Direction = directionFromVector(dx/dist, dy/dist)
		mapState.Mo(player.ID).Mode = world.WALKING
	} else {
		mapState.Mo(player.ID).Mode = world.IDLE
	}
}

// handleItemActivationInput toggles an inventory slot and re-validates
// the equipment rules.
func (s *Sim) handleItemActivationInput(player *world.Session, cmd *world.InputCommand) {
	itemID := cmd.ItemID
	active := cmd.Active

	v := s.Vt(player.ID, player.MapCode)
	isDead := v.Ghost() || v.Life <= 0
	isDeadItem := s.DeadItemIDs[itemID]

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
	for i := range s.Ap(player.ID, player.MapCode).Layers {
		if s.Ap(player.ID, player.MapCode).Layers[i].ItemID == itemID {
			targetItemIndex = i
			break
		}
	}
	if targetItemIndex == -1 {
		logx.Debugf("Player %s tried to activate non-existent item '%s'.", player.ID, itemID)
		return
	}

	originalState := s.Ap(player.ID, player.MapCode).Layers[targetItemIndex].Active
	s.Ap(player.ID, player.MapCode).Layers[targetItemIndex].Active = active

	// Type guard — only activeItemTypes may be turned active.
	if active && len(s.EquipmentRules.ActiveItemTypes) > 0 {
		reqType := s.ItemType(itemID)
		if reqType != "" && !s.EquipmentRules.ActiveItemTypes[reqType] {
			logx.Debugf("Player %s tried to activate '%s' (type=%q) — not in activeItemTypes; rejecting.",
				player.ID, itemID, reqType)
			s.Ap(player.ID, player.MapCode).Layers[targetItemIndex].Active = originalState
			return
		}
	}

	// Rules operate within the current state's item set: dead items during
	// the Fragmented State, live items otherwise.
	inState := func(id string) bool { return s.DeadItemIDs[id] == isDead }

	// OnePerType — deactivate any other active in-state item with the same type.
	if active && s.EquipmentRules.OnePerType {
		requested := s.ItemType(itemID)
		if requested != "" {
			for i := range s.Ap(player.ID, player.MapCode).Layers {
				if i == targetItemIndex {
					continue
				}
				if !s.Ap(player.ID, player.MapCode).Layers[i].Active || !inState(s.Ap(player.ID, player.MapCode).Layers[i].ItemID) {
					continue
				}
				if s.ItemType(s.Ap(player.ID, player.MapCode).Layers[i].ItemID) == requested {
					s.Ap(player.ID, player.MapCode).Layers[i].Active = false
				}
			}
		}
	}

	// RequireSkin / maxActiveLayers corrections, scoped to in-state items.
	activeLayerCount, activeSkinCount, hasAnySkin, firstSkinIndex := 0, 0, false, -1
	for i, layer := range s.Ap(player.ID, player.MapCode).Layers {
		isSkin := inState(layer.ItemID) && s.ItemType(layer.ItemID) == "skin"
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
	if s.EquipmentRules.RequireSkin && activeSkinCount == 0 {
		if isDead && s.GhostItemID != "" {
			// Unequipping the last dead skin is allowed: the ghost item from
			// gRPC config takes over as the Fragmented State visual.
			content.ActivateOrAppendLayer(&s.Ap(player.ID, player.MapCode).Layers, s.GhostItemID)
		} else if hasAnySkin && firstSkinIndex != -1 {
			s.Ap(player.ID, player.MapCode).Layers[firstSkinIndex].Active = true
		}
	}
	if activeLayerCount > s.MaxActiveLayers {
		s.Ap(player.ID, player.MapCode).Layers[targetItemIndex].Active = originalState
	}

	// The Fragmented State loadout persists across deaths.
	if isDead {
		player.DeadLoadoutItemIDs = content.ActiveObjectLayerItemIDs(s.Ap(player.ID, player.MapCode).Layers)
	}

	s.ApplyResistanceStat(player.ID, s.Maps[player.MapCode])
}

// queuePreRespawnActivation marks a pending activation that takes effect
// when the dead player respawns. Extracted to keep handleItemActivationInput
// readable.
func (s *Sim) queuePreRespawnActivation(player *world.Session, itemID string) {
	layers := s.Maps[player.MapCode].Mt(player.ID).PreRespawnObjectLayers
	if layers == nil {
		return
	}
	for i := range layers {
		if layers[i].ItemID != itemID {
			continue
		}
		layers[i].Active = true
		logx.Debugf("Player %s is dead — queued activation of '%s' on revive.", player.ID, itemID)
		if s.EquipmentRules.OnePerType {
			reqType := s.ItemType(itemID)
			if reqType != "" {
				for j := range layers {
					if j == i {
						continue
					}
					if layers[j].Active &&
						s.ItemType(layers[j].ItemID) == reqType {
						layers[j].Active = false
					}
				}
			}
		}
		return
	}
}

// handleChatInput is a pure relay — no game-state mutation. Forwards a
// chat message to the target player on the same map.
func (s *Sim) handleChatInput(sender *world.Session, mapState *world.MapState, cmd *world.InputCommand) {
	toID := cmd.EntityID
	text := cmd.ChatText
	if toID == 0 || text == "" {
		return
	}
	if len(text) > 256 {
		text = text[:256]
	}
	target := s.PlayerOn(mapState, toID)
	if target == nil {
		return
	}
	world.SendMessage(target, "chat", map[string]interface{}{
		"from": sender.ID,
		"text": text,
	})
}
