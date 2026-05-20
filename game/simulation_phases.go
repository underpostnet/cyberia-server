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
//	5. phaseMovement    integrate positions using s.tickDuration
//	6. phasePortals     portal entry / teleport
//
// Replication (phaseReplication) runs on its own ticker at snapshotRate and
// is invoked from server.go directly.
//
// A phase MUST NOT:
//   - touch any presentation concern. PresentationHints has been deleted
//     from the server; palette / camera / devUi / status-icon visuals live
//     entirely on the client and at the engine's /api/cyberia-client-hints
//     REST endpoint. The Go server does not even hold these values.
//   - block on I/O.
//   - launch goroutines that mutate world state.
//
// A phase MAY:
//   - read and write fields of MapState, PlayerState, BotState.
//   - read GameServer config (rules) fields that influence gameplay.

package game

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
//   - Per-player input budget is bounded by the queue capacity in
//     EnqueueInput (currently 64) so a malicious client cannot starve
//     other tick work.
func (s *GameServer) phaseInput(tick uint32, mapState *MapState) {
	for _, player := range mapState.players {
		if len(player.InputQueue) == 0 {
			continue
		}
		queue := player.InputQueue
		player.InputQueue = nil // drained; further EnqueueInput appends a fresh slice
		for i := range queue {
			s.applyInputCommand(player, mapState, &queue[i])
		}
	}
	_ = tick
}

// applyInputCommand is the typed dispatcher. Kept on *GameServer so it
// can call into existing simulation helpers without re-acquiring the
// world lock. Each branch maps to one InputKind constant.
func (s *GameServer) applyInputCommand(player *PlayerState, mapState *MapState, cmd *InputCommand) {
	switch cmd.Kind {
	case InputKindPlayerAction:
		s.handlePlayerActionInput(player, mapState, cmd)
	case InputKindItemActivation:
		s.handleItemActivationInput(player, cmd)
	case InputKindFreezeStart:
		reason := cmd.Reason
		if reason == "" {
			reason = "freeze"
		}
		FreezePlayer(player, reason)
	case InputKindFreezeEnd:
		reason := cmd.Reason
		if reason == "" {
			reason = "freeze"
		}
		ThawPlayer(player, reason)
	case InputKindChat:
		s.handleChatInput(player, mapState, cmd)
	case InputKindGetItemsIDs:
		s.handleGetItemsIDsInput(player, cmd)
	case InputKindHandshake, InputKindUnknown:
		// no-op
	}
}

// phaseLifecycle handles respawn timers and similar lifecycle transitions.
// Currently delegates to the existing helper which also subsumes coin-drop
// FCT events (those will move to phaseEconomy in a follow-up).
func (s *GameServer) phaseLifecycle(tick uint32, mapState *MapState) {
	_ = tick
	s.handleRespawns(mapState)
}

// phaseSkills resolves skill projectile collisions for this tick.
func (s *GameServer) phaseSkills(tick uint32, mapState *MapState) {
	_ = tick
	s.handleSkillCollisions(mapState)
}

// phaseAI runs bot behaviour. The current implementation also performs the
// bot's positional integration inside updateBots — that movement remains
// here for now but should be split out to phaseMovement in a follow-up so
// that ai/movement separation is enforced on both player and bot paths.
func (s *GameServer) phaseAI(tick uint32, mapState *MapState) {
	_ = tick
	s.updateBots(mapState)
}

// phaseMovement integrates player positions using dt = s.tickDuration.
// Bots are handled inside phaseAI for now (see note above).
func (s *GameServer) phaseMovement(tick uint32, mapState *MapState) {
	_ = tick
	for _, player := range mapState.players {
		s.updatePlayerPosition(player, mapState)
		// TELEPORTING is a one-snapshot-interval signal set by teleportPlayer.
		// phaseReplication (or the immediate sendAOI in teleportPlayer) delivers
		// it once; we reset it here so the player becomes IDLE rather than staying
		// TELEPORTING indefinitely.  The client uses this flag to suppress
		// pos_prev interpolation on the teleport snapshot, preventing a visible
		// cross-map lerp.
		if player.Mode == TELEPORTING {
			player.Mode = IDLE
		}
	}
}

// phasePortals checks each player against portals and triggers teleports
// when the portal hold time has elapsed.
func (s *GameServer) phasePortals(tick uint32, mapState *MapState) {
	_ = tick
	for _, player := range mapState.players {
		s.checkPortal(player, mapState)
	}
}

// phaseReplication runs on the snapshot ticker. Iterates connected players
// for this map, computes their AOI, encodes a snapshot, and sends it. This
// path is the only one that produces server→client AOI frames at steady
// state (the bootstrap full snapshot on connect is sent from handlers.go).
//
// `tick` is the simulation tick at the moment the snapshot was produced.
// The wire encoder embeds it in the snapshot header so the client can use
// it for prediction reconciliation and interpolation.
func (s *GameServer) phaseReplication(tick uint32, mapState *MapState) {
	for _, player := range mapState.players {
		player.AOI = Rectangle{
			MinX: player.Pos.X - s.aoiRadius,
			MinY: player.Pos.Y - s.aoiRadius,
			MaxX: player.Pos.X + player.Dims.Width + s.aoiRadius,
			MaxY: player.Pos.Y + player.Dims.Height + s.aoiRadius,
		}
		player.LastSnapshotTick = tick
		s.sendAOI(player)
	}
}
