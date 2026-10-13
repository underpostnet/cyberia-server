// Package sim — simulation_phases.go
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

package sim

import (
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// PhaseInput drains queued InputCommand entries for each player in this
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
func (s *Sim) PhaseInput(tick uint32, mapState *world.MapState) {
	for player := range s.PlayersOn(mapState) {
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
func (s *Sim) applyInputCommand(player *world.Session, mapState *world.MapState, cmd *world.InputCommand) {
	switch cmd.Kind {
	case world.InputKindPlayerAction:
		s.handlePlayerActionInput(player, mapState, cmd)
	case world.InputKindItemActivation:
		s.handleItemActivationInput(player, cmd)
	case world.InputKindPlayerStasis:
		if cmd.Active {
			FreezePlayer(player, mapState)
		} else {
			ThawPlayer(player)
		}
	case world.InputKindChat:
		s.handleChatInput(player, mapState, cmd)
	case world.InputKindTalkDone:
		s.HandleTalkDone(player, cmd)
	case world.InputKindQuestAbandon:
		s.HandleQuestAbandon(player, cmd)
	case world.InputKindQuestAccept:
		s.HandleQuestAccept(player, cmd)
	case world.InputKindShopBuy:
		s.HandleShopBuy(player, cmd)
	case world.InputKindCraftItem:
		s.HandleCraftItem(player, cmd)
	case world.InputKindCraftCancel:
		s.HandleCraftCancel(player)
	case world.InputKindItemOps:
		s.HandleItemOps(player, cmd)
	case world.InputKindUnknown:
		// failed validation: consumed, never applied
	}
}

// PhaseLifecycle handles respawn timers and similar lifecycle transitions.
// Currently delegates to the existing helper which also subsumes coin-drop
// FCT events (those will move to phaseEconomy in a follow-up).
func (s *Sim) PhaseLifecycle(tick uint32, mapState *world.MapState) {
	_ = tick
	for id := range mapState.Mortals() {
		s.ApplyResistanceStat(id, mapState)
	}
	s.handleRespawns(mapState)
	s.CompleteCrafts(mapState)
}

// PhaseSkills resolves skill projectile collisions and runs the skills of held items for this tick.
func (s *Sim) PhaseSkills(tick uint32, mapState *world.MapState) {
	_ = tick
	s.handleSkillCollisions(mapState)
	for player := range s.PlayersOn(mapState) {
		s.dispatchHeldSkills(player.ID, mapState)
	}
	for id := range mapState.OfKind(world.EntityBot) {
		if content.BehaviorCollectsLoot(mapState.Ag(id).Behavior) {
			s.dispatchHeldSkills(id, mapState)
		}
	}
}

// PhaseAI runs bot behaviour. The current implementation also performs the
// bot's positional integration inside updateBots — that movement remains
// here for now but should be split out to phaseMovement in a follow-up so
// that ai/movement separation is enforced on both player and bot paths.
func (s *Sim) PhaseAI(tick uint32, mapState *world.MapState) {
	_ = tick
	s.updateBots(mapState)
}

// PhaseMovement integrates player positions using dt = tickDuration.
// Bots are handled inside phaseAI for now (see note above).
func (s *Sim) PhaseMovement(tick uint32, mapState *world.MapState) {
	_ = tick
	for player := range s.PlayersOn(mapState) {
		s.updatePlayerPosition(player, mapState)
		// TELEPORTING is a one-snapshot-interval signal set by teleportPlayer.
		// phaseReplication (or the immediate send after phasePortals) delivers
		// it once; we reset it here so the player becomes IDLE rather than staying
		// TELEPORTING indefinitely.  The client uses this flag to suppress
		// pos_prev interpolation on the teleport snapshot, preventing a visible
		// cross-map lerp.
		if mapState.Mo(player.ID).Mode == world.TELEPORTING {
			mapState.Mo(player.ID).Mode = world.IDLE
		}
	}
}

// PhasePortals checks each player against portals and triggers teleports
// when the portal hold time has elapsed.
func (s *Sim) PhasePortals(tick uint32, mapState *world.MapState) {
	_ = tick
	for player := range s.PlayersOn(mapState) {
		s.checkPortal(player, mapState)
	}
}

// PhaseLoot resolves token collisions for one tick. The first collector the
// token's collection policy admits takes it, players before bots: its outcome
// applies, the pickup animation event goes to every viewer and the token is
// removed in the same step, so one token applies once. Caller holds s.mu.
func (s *Sim) PhaseLoot(tick uint32, mapState *world.MapState) {
	_ = tick
	now := time.Now()
	s.RespawnWorldLoot(mapState, now)

	for dropID := range mapState.OfKind(world.EntityBot) {
		drop := mapState.Br(dropID)
		if !content.BehaviorIsInert(mapState.Ag(dropID).Behavior) {
			continue
		}
		// Mid-launch tokens register no collision until they settle on the cell.
		if now.Before(drop.CollectableAt) {
			continue
		}

		cx := mapState.Tf(dropID).Pos.X + mapState.Tf(dropID).Size.X*0.5
		cy := mapState.Tf(dropID).Pos.Y + mapState.Tf(dropID).Size.Y*0.5

		var collectorID world.EntityID
		var granted bool
		if player := s.ResolveDropCollector(mapState, dropID); player != nil {
			granted, collectorID = s.ApplyLootToPlayer(mapState, drop, player), player.ID
		} else if bot := s.ResolveBotDropCollector(mapState, dropID); bot != 0 {
			granted, collectorID = s.ApplyLootToBot(mapState, drop, bot), bot
		} else {
			continue
		}
		s.BroadcastDropCollect(mapState, dropID, collectorID, cx, cy, granted)
		if drop.LootOrigin == world.LootOriginWorld && drop.LootOutcome == world.LootOutcomeConsume {
			s.ScheduleLootRespawn(mapState, dropID, now)
		}
		s.Despawn(mapState, dropID)
	}
}
