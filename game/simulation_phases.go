// Package game — simulation_phases.go
//
// Named simulation phases. The game loop in server.go calls these in a fixed
// order once per Tick. Each phase has a single responsibility and operates on
// one MapState. Phase functions are the ONLY allowed mutators of authoritative
// world state.
//
// Fixed phase order (per Tick):
//
//	1. phaseInput       drain queued InputCommand per player
//	2. phaseLifecycle   respawn timers, despawn expirations
//	3. phaseSkills      skill projectile collisions
//	4. phaseAI          bot behaviour decisions (path planning)
//	5. phaseMovement    integrate positions using tickDuration
//	6. phaseLoot        player↔drop-token collection + priority validation
//	7. phasePortals     portal entry / teleport
//	8. phaseReplication one AOI snapshot per player
//
// A phase MUST NOT:
//   - touch any presentation concern. PresentationHints has been deleted
//     from the server; palette / camera / devUi / status-icon visuals live
//     entirely on the client and at the engine's /api/v1/cyberia-client-hints
//     REST endpoint. The Go server does not even hold these values.
//   - block on I/O.
//   - launch goroutines that mutate world state.
//
// A phase MAY:
//   - read and write fields of MapState and Session.
//   - read GameServer config (rules) fields that influence gameplay.

package game

import (
	"time"
)

// phaseInput drains queued InputCommand entries for each player in this
// map and dispatches each command to its typed handler. This is the
// authoritative ingestion point — no other code path is allowed to apply
// client input to world state.
//
// Contract:
//   - Caller holds s.mu.
//   - Handlers run synchronously and may freely read/write world state.
//   - Each command is consumed exactly once. The queue is emptied at the
//     end of the phase.
//   - Per-player input budget is bounded by maxInputQueue (512) in
//     EnqueueInputs, so a malicious client cannot starve other tick work.
//   - Movement re-plans at most once per player per tick. Handlers record the
//     intent; flushPendingMove runs the single A*. This is the only bound on
//     pathfinder cost, so no other path may call it.
func (s *Sim) phaseInput(tick uint32, mapState *MapState) {
	for player := range s.playersOn(mapState) {
		if len(player.InputQueue) == 0 {
			continue
		}
		queue := player.InputQueue
		player.InputQueue = nil // drained; further EnqueueInputs appends a fresh slice
		for i := range queue {
			s.applyInputCommand(player, mapState, &queue[i])
			// The cursor means consumed. A snapshot built between this
			// tick and the next must not claim a command no tick has run.
			if queue[i].Sequence > player.InputConsumedThrough {
				player.InputConsumedThrough = queue[i].Sequence
			}
			// An event pushed in client frame F takes effect in step F + 1,
			// so this tick stands for that step.
			player.FrameOffset = int64(queue[i].Frame) + 1 - int64(tick)
		}
		s.flushPendingMove(player, mapState)
	}
}

// applyInputCommand is the typed dispatcher. Kept on *GameServer so it
// can call into existing simulation helpers without re-acquiring the
// world lock. Each branch maps to one InputKind constant.
func (s *Sim) applyInputCommand(player *Session, mapState *MapState, cmd *InputCommand) {
	switch cmd.Kind {
	case InputKindPlayerAction:
		s.handlePlayerActionInput(player, mapState, cmd)
	case InputKindItemActivation:
		s.handleItemActivationInput(player, cmd)
	case InputKindPlayerStasis:
		if cmd.Active {
			FreezePlayer(player, mapState)
		} else {
			ThawPlayer(player)
		}
	case InputKindChat:
		s.handleChatInput(player, mapState, cmd)
	case InputKindTalkDone:
		s.handleTalkDone(player, cmd)
	case InputKindQuestAbandon:
		s.handleQuestAbandon(player, cmd)
	case InputKindQuestAccept:
		s.handleQuestAccept(player, cmd)
	case InputKindShopBuy:
		s.handleShopBuy(player, cmd)
	case InputKindCraftItem:
		s.handleCraftItem(player, cmd)
	case InputKindCraftCancel:
		s.handleCraftCancel(player)
	case InputKindItemOps:
		s.handleItemOps(player, cmd)
	case InputKindUnknown:
		// failed validation: consumed, never applied
	}
}

// phaseLifecycle handles respawn timers and similar lifecycle transitions.
// Currently delegates to the existing helper which also subsumes coin-drop
// FCT events (those will move to phaseEconomy in a follow-up).
func (s *Sim) phaseLifecycle(tick uint32, mapState *MapState) {
	_ = tick
	for id := range mapState.Mortals() {
		s.ApplyResistanceStat(id, mapState)
	}
	s.handleRespawns(mapState)
	s.completeCrafts(mapState)
}

// phaseSkills resolves skill projectile collisions and runs the skills of held items for this tick.
func (s *Sim) phaseSkills(tick uint32, mapState *MapState) {
	_ = tick
	s.handleSkillCollisions(mapState)
	for player := range s.playersOn(mapState) {
		s.dispatchHeldSkills(player.ID, mapState)
	}
	for id := range mapState.brains {
		if behaviorCollectsLoot(mapState.ag(id).Behavior) {
			s.dispatchHeldSkills(id, mapState)
		}
	}
}

// phaseAI runs bot behaviour. The current implementation also performs the
// bot's positional integration inside updateBots — that movement remains
// here for now but should be split out to phaseMovement in a follow-up so
// that ai/movement separation is enforced on both player and bot paths.
func (s *Sim) phaseAI(tick uint32, mapState *MapState) {
	_ = tick
	s.updateBots(mapState)
}

// phaseMovement integrates player positions using dt = tickDuration.
// Bots are handled inside phaseAI for now (see note above).
func (s *Sim) phaseMovement(tick uint32, mapState *MapState) {
	_ = tick
	for player := range s.playersOn(mapState) {
		s.updatePlayerPosition(player, mapState)
		// TELEPORTING is a one-snapshot-interval signal set by teleportPlayer.
		// phaseReplication (or the immediate send after phasePortals) delivers
		// it once; we reset it here so the player becomes IDLE rather than staying
		// TELEPORTING indefinitely.  The client uses this flag to suppress
		// pos_prev interpolation on the teleport snapshot, preventing a visible
		// cross-map lerp.
		if mapState.mo(player.ID).Mode == TELEPORTING {
			mapState.mo(player.ID).Mode = IDLE
		}
	}
}

// phasePortals checks each player against portals and triggers teleports
// when the portal hold time has elapsed.
func (s *Sim) phasePortals(tick uint32, mapState *MapState) {
	_ = tick
	for player := range s.playersOn(mapState) {
		s.checkPortal(player, mapState)
	}
}

// phaseLoot resolves token collisions for one tick. The first collector the
// token's collection policy admits takes it, players before bots: its outcome
// applies, the pickup animation event goes to every viewer and the token is
// removed in the same step, so one token applies once. Caller holds s.mu.
func (s *Sim) phaseLoot(tick uint32, mapState *MapState) {
	_ = tick
	now := time.Now()
	s.respawnWorldLoot(mapState, now)

	for dropID := range mapState.ofKind(EntityBot) {
		drop := mapState.br(dropID)
		if !behaviorIsInert(mapState.ag(dropID).Behavior) {
			continue
		}
		// Mid-launch tokens register no collision until they settle on the cell.
		if now.Before(drop.CollectableAt) {
			continue
		}

		cx := mapState.tf(dropID).Pos.X + mapState.tf(dropID).Size.X*0.5
		cy := mapState.tf(dropID).Pos.Y + mapState.tf(dropID).Size.Y*0.5

		var collectorID EntityID
		var granted bool
		if player := s.resolveDropCollector(mapState, dropID); player != nil {
			granted, collectorID = s.applyLootToPlayer(mapState, drop, player), player.ID
		} else if bot := s.resolveBotDropCollector(mapState, dropID); bot != 0 {
			granted, collectorID = s.applyLootToBot(mapState, drop, bot), bot
		} else {
			continue
		}
		s.broadcastDropCollect(mapState, dropID, collectorID, cx, cy, granted)
		if drop.LootOrigin == LootOriginWorld && drop.LootOutcome == LootOutcomeConsume {
			s.scheduleLootRespawn(mapState, dropID, now)
		}
		s.despawn(mapState, dropID)
	}
}
