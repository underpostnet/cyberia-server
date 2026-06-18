// Package game — action.go
//
// Server-side cyberia-action runtime: the interactive-NPC / dialogue (talk)
// layer. engine-cyberia owns the persisted CyberiaAction content and exposes it
// over REST; the Go simulation binds each action to the entity on its source
// cell and drives the dialogue handshake (dlg_start / dlg_complete / dlg_cancel).
// Reading a dialogue advances quest `talk` objectives (see quest.go) — that is
// the only coupling between the two otherwise-independent flows.
//
// Cross-process contract:
//
//	dlg_start    (client→server) — freeze the player, snapshot the interaction
//	                               context (action + NPC skin) for validation
//	dlg_complete (client→server) — advance talk objectives for the talked-to NPC,
//	                               send dlg_ack, thaw
//	dlg_cancel   (client→server) — clear context, thaw, no progress
//
// Caller MUST hold s.mu for every handler / mutation helper here (they run inside
// phaseInput). Binding runs at world (re)build time, also under s.mu.
package game

import (
	"log"

	pb "cyberia-server/gen/proto"
)

// CyberiaAction mirrors src/api/cyberia-action/cyberia-action.model.js.
// Only the fields the simulation needs are decoded.
// ActionQuestDialogue maps a quest this NPC handles to the dialogue shown for
// it (offer + talk-objective validation).
type ActionQuestDialogue struct {
	QuestCode  string `json:"questCode"`
	DialogCode string `json:"dialogCode"`
}

type CyberiaAction struct {
	Code               string                `json:"code"`
	Label              string                `json:"label"`
	SourceMapCode      string                `json:"sourceMapCode"`
	SourceCellX        int                   `json:"sourceCellX"`
	SourceCellY        int                   `json:"sourceCellY"`
	DialogCode         string                `json:"dialogCode"`
	QuestDialogueCodes []ActionQuestDialogue `json:"questDialogueCodes"`
}

// actionCell returns the cell key for an action's source cell.
func actionCell(a *CyberiaAction) cellKey {
	return cellKey{a.SourceMapCode, a.SourceCellX, a.SourceCellY}
}

func protoToAction(a *pb.CyberiaActionMessage) *CyberiaAction {
	ca := &CyberiaAction{
		Code:          a.GetCode(),
		Label:         a.GetLabel(),
		SourceMapCode: a.GetSourceMapCode(),
		SourceCellX:   int(a.GetSourceCellX()),
		SourceCellY:   int(a.GetSourceCellY()),
		DialogCode:    a.GetDialogCode(),
	}
	for _, qd := range a.GetQuestDialogueCodes() {
		ca.QuestDialogueCodes = append(ca.QuestDialogueCodes, ActionQuestDialogue{
			QuestCode: qd.GetQuestCode(), DialogCode: qd.GetDialogCode(),
		})
	}
	return ca
}

// bindActions caches the CyberiaAction content the engine delivered with the
// world over gRPC and binds each action to the runtime entity on its source cell
// — keyed by the bot's stable spawn-centre, not its live position, since action
// NPCs wander within their radius.
//
// Caller MUST hold s.mu.
func (s *GameServer) bindActions(mapCodes []string, actions []*pb.CyberiaActionMessage) {
	s.actionCache = make(map[string]*CyberiaAction)

	mapSet := make(map[string]bool, len(mapCodes))
	for _, c := range mapCodes {
		mapSet[c] = true
	}

	byCell := make(map[cellKey]*CyberiaAction)
	for _, a := range actions {
		ca := protoToAction(a)
		if !mapSet[ca.SourceMapCode] {
			continue
		}
		byCell[actionCell(ca)] = ca
	}

	bound := 0
	for _, ms := range s.maps {
		for _, bot := range ms.bots {
			action, ok := byCell[botSpawnCell(bot)]
			if !ok {
				continue
			}
			bot.ActionCode = action.Code
			s.actionCache[bot.ID] = action
			bound++
		}
	}
	log.Printf("[ActionContent] gRPC delivered %d actions; bound %d entities", len(actions), bound)
}

// logActionProviders prints every entity bound to a cyberia-action — map, action
// code, label, the quests bound to its cell, entity id, and initial cell. Called
// after the instance graph so operators can confirm the action NPCs were
// instantiated and bound.
func (s *GameServer) logActionProviders() {
	if len(s.actionCache) == 0 {
		log.Printf("[ActionProviders] none bound (no actions delivered via gRPC, or none match an entity cell)")
		return
	}
	log.Printf("[ActionProviders] %d action-provider entities:", len(s.actionCache))
	for _, ms := range s.maps {
		for _, bot := range ms.bots {
			if bot.ActionCode == "" {
				continue
			}
			a := s.actionCache[bot.ID]
			label := ""
			var quests []string
			if a != nil {
				label = a.Label
				quests = s.questsByCell[actionCell(a)]
			}
			log.Printf("[ActionProviders]   map=%s action=%s label=%q quests=%v entity=%s initPos=(%d,%d)",
				bot.MapCode, bot.ActionCode, label, quests, bot.ID,
				int(bot.Pos.X), int(bot.Pos.Y))
		}
	}
}

// findBot returns the bot with this entity ID across all maps, or nil.
//
// Caller MUST hold s.mu.
func (s *GameServer) findBot(entityID string) *BotState {
	for _, ms := range s.maps {
		if b := ms.bots[entityID]; b != nil {
			return b
		}
	}
	return nil
}

// botActiveSkin returns the active skin item ID of the bot with this entity ID
// (searched across maps), or "" if not found. The bot's own active skin — not
// any action field — is what 'talk' objectives match against.
func (s *GameServer) botActiveSkin(entityID string) string {
	bot := s.findBot(entityID)
	if bot == nil {
		return ""
	}
	return s.botActiveSkinOf(bot)
}

// botActiveSkinOf reads a bot's active skin directly (no lookup), so the AOI
// path that already holds the bot avoids an O(N) per-entity map search.
func (s *GameServer) botActiveSkinOf(bot *BotState) string {
	// When the NPC is dead its active layers hold the ghost skin; the live skin
	// is preserved in PreRespawnObjectLayers. Read from there so a talk objective
	// still validates against a quest-giver that just died — the interaction
	// context is independent of the entity's alive/AOI state.
	layers := bot.ObjectLayers
	if bot.IsGhost() && len(bot.PreRespawnObjectLayers) > 0 {
		layers = bot.PreRespawnObjectLayers
	}
	for _, ol := range layers {
		if ol.Active && s.itemType(ol.ItemID) == "skin" {
			return ol.ItemID
		}
	}
	return ""
}

// botInteractionFlags builds the per-player interaction capability bitmask for
// NPC `bot`: the action bit when it exposes a bound cyberia-action (always
// interactable), and the quest bit when it surfaces any quest to the player
// (`hasQuests`, from botQuestCodes — including completed feedback). The two
// capabilities are independent — a bot may carry either, both, or neither.
func (s *GameServer) botInteractionFlags(bot *BotState, hasQuests bool) uint8 {
	var flags uint8
	if bot.ActionCode != "" {
		flags |= InteractionFlagAction
	}
	if hasQuests {
		flags |= InteractionFlagQuest
	}
	return flags
}

// ── Dialogue handlers (phaseInput) ──────────────────────────────────────────

// handleDlgStart binds the dialogue context and freezes the player. The freeze
// grants modal protection (immunity); it is released on dlg_complete / dlg_cancel.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleDlgStart(player *PlayerState, cmd *InputCommand) {
	if player.IsGhost() {
		return
	}
	// Snapshot the provider's interaction context now, while the bot is in range.
	// dlg_complete validates against this frozen snapshot, not a live re-lookup,
	// so the talk objective still resolves if the bot later dies or leaves AOI.
	player.ActiveDialogueEntityID = cmd.EntityID
	player.ActiveDialogueAction = s.actionCache[cmd.EntityID]
	player.ActiveDialogueSkin = s.botActiveSkin(cmd.EntityID)
	FreezePlayer(player, "dialogue")
}

// handleDlgCancel releases the dialogue freeze without recording progress.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleDlgCancel(player *PlayerState, cmd *InputCommand) {
	if player.ActiveDialogueEntityID == "" || player.ActiveDialogueEntityID != cmd.EntityID {
		return
	}
	player.ActiveDialogueEntityID = ""
	player.ActiveDialogueAction = nil
	player.ActiveDialogueSkin = ""
	ThawPlayer(player, "dialogue")
}

// handleDlgComplete is the authoritative dialogue-completion path. It validates
// the frozen dialogue context, advances the `talk` objectives that target the
// NPC the player spoke with (quest.go), and notifies the client via dlg_ack.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleDlgComplete(player *PlayerState, cmd *InputCommand) {
	// Validate: drop unless this matches the dialogue the player opened.
	if player.ActiveDialogueEntityID == "" || player.ActiveDialogueEntityID != cmd.EntityID {
		return
	}
	// Resolve against the snapshot frozen at dlg_start, not a live re-lookup, so
	// completion is independent of the bot's current alive/AOI state.
	action := player.ActiveDialogueAction
	talkedSkin := player.ActiveDialogueSkin
	player.ActiveDialogueEntityID = ""
	player.ActiveDialogueAction = nil
	player.ActiveDialogueSkin = ""
	ThawPlayer(player, "dialogue")

	if action == nil {
		return
	}

	// dlg_complete NEVER grants a quest — acceptance is explicit (quest_accept,
	// the Take Quest button). Reading the dialogue only advances `talk` objectives
	// that target the NPC the player spoke with.
	var affected []QuestSnapshotEntry
	objectivesDone := s.advanceTalkObjectives(player, talkedSkin, &affected)
	if s.advanceCollectObjectives(player, &affected) {
		objectivesDone = true
	}

	s.sendDlgAck(player, "", objectivesDone, affected)
}
