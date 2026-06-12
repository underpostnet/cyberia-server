// Package game — action_quest.go
//
// Server-side Action & Quest runtime for the Talk / Quest-Talk system.
//
// engine-cyberia owns the persisted CyberiaAction / CyberiaQuest content and
// exposes it over REST. The Go simulation is the authority that *resolves*
// which action an entity provides and *validates* dialogue completion before
// advancing quest progress. The client never declares the action type, quest
// code, or dialogue codes — it only reports which entity it talked to and
// which dialogue group it finished reading (dlg_complete). Everything else is
// resolved here from actionCache / questDefs, both bound at instance init.
//
// Cross-process contract:
//
//	dlg_start    (client→server) — bind PlayerState.ActiveDialogueEntityID, freeze
//	dlg_complete (client→server) — validate context, branch on action.Type,
//	                               advance talk objectives, send dlg_ack, thaw
//	dlg_cancel   (client→server) — clear context, thaw, no progress
//	dlg_ack      (server→client) — notify-only: questGranted + objectivesDone
//	                               (+ affected quest snapshot entries for the
//	                                Quest Journal store)
//
// Caller MUST hold s.mu for every handler / mutation helper in this file
// (they run inside phaseInput). Only loadActionContent does network I/O and
// it runs at world (re)build time, also under s.mu.
package game

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"sort"
	"strings"
	"time"

	pb "cyberia-server/gen/proto"
)

// ── Content shapes (mirror engine-cyberia Mongo schemas) ────────────────────

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

// dialogCodeForQuest returns the dialogue code this action shows for questCode,
// or "" when the NPC doesn't handle that quest.
func (a *CyberiaAction) dialogCodeForQuest(questCode string) string {
	for _, qd := range a.QuestDialogueCodes {
		if qd.QuestCode == questCode {
			return qd.DialogCode
		}
	}
	return ""
}

// CyberiaQuest mirrors src/api/cyberia-quest/cyberia-quest.model.js.
type CyberiaQuest struct {
	Code              string        `json:"code"`
	Title             string        `json:"title"`
	Description       string        `json:"description"`
	UnlocksQuestCodes []string      `json:"unlocksQuestCodes"`
	PrerequisiteCodes []string      `json:"prerequisiteCodes"`
	SourceMapCode     string        `json:"sourceMapCode"`
	SourceCellX       int           `json:"sourceCellX"`
	SourceCellY       int           `json:"sourceCellY"`
	Steps             []QuestStep   `json:"steps"`
	Rewards           []QuestReward `json:"rewards"`
}

type QuestStep struct {
	ID          string           `json:"id"`
	Description string           `json:"description"`
	Objectives  []QuestObjective `json:"objectives"`
}

type QuestObjective struct {
	Type     string `json:"type"` // collect | talk | kill
	ItemID   string `json:"itemId"`
	Quantity int    `json:"quantity"`
}

type QuestReward struct {
	ItemID   string `json:"itemId"`
	Quantity int    `json:"quantity"`
}

// ── Per-player progress (session authority) ─────────────────────────────────

// QuestProgress is the runtime, computed-completeness progress record.
// Step/quest completion is always derived from Current >= Required, never
// stored as a flag (mirrors CyberiaQuestProgress semantics).
type QuestProgress struct {
	QuestCode string              `json:"questCode"`
	Status    string              `json:"status"` // active | completed
	Steps     []QuestStepProgress `json:"steps"`
	StartedAt time.Time           `json:"-"`
}

type QuestStepProgress struct {
	StepID     string                   `json:"stepId"`
	Objectives []QuestObjectiveProgress `json:"objectives"`
	// Consumed marks that this step's `collect` items were removed from the
	// player's inventory on step completion (idempotent guard).
	Consumed bool `json:"-"`
}

type QuestObjectiveProgress struct {
	Type     string `json:"type"`
	ItemID   string `json:"itemId"`
	Current  int    `json:"current"`
	Required int    `json:"required"`
}

// QuestSnapshotEntry is the client-facing projection used by both init_data
// and dlg_ack.  It carries ONLY authoritative runtime data that the
// simulation owns: code, status, and progress counters.  All metadata
// (title, description, steps, rewards) is fetched by the C client from
// the engine REST endpoint /api/cyberia-quest/:code — the simulation
// never transmits non-authoritative presentation data.
type QuestSnapshotEntry struct {
	Code           string `json:"code"`
	Status         string `json:"status"`         // "active" | "completed"
	ActiveStep     string `json:"activeStep"`     // first incomplete step ID
	ObjectivesText string `json:"objectivesText"` // "2/3 kill scp-2040, 1/1 talk wason"
}

// ── Instance init: bind gRPC-delivered content to entities ──────────────────

// bindActionContent caches the CyberiaAction / CyberiaQuest content that the
// engine delivered with the world over gRPC and binds each action to the
// runtime entity at its (sourceMapCode, sourceCellX, sourceCellY) cell. The
// Go server never fetches this over a separate channel — for the procedural
// fallback the engine's fallback builder fills it from the canonical defaults.
//
// Caller MUST hold s.mu.
func (s *GameServer) bindActionContent(mapCodes []string,
	actions []*pb.CyberiaActionMessage, quests []*pb.CyberiaQuestMessage) {
	s.actionCache = make(map[string]*CyberiaAction)
	s.questDefs = make(map[string]*CyberiaQuest)

	mapSet := make(map[string]bool, len(mapCodes))
	for _, c := range mapCodes {
		mapSet[c] = true
	}

	// Index quests by their source cell — an action awards the quest(s) whose
	// source cell matches its own. Codes are sorted so per-player resolution is
	// deterministic.
	s.questsByCell = make(map[cellKey][]string)
	for _, q := range quests {
		cq := protoToQuest(q)
		s.questDefs[cq.Code] = cq
		if cq.SourceMapCode != "" {
			k := cellKey{cq.SourceMapCode, cq.SourceCellX, cq.SourceCellY}
			s.questsByCell[k] = append(s.questsByCell[k], cq.Code)
		}
	}
	for k := range s.questsByCell {
		sort.Strings(s.questsByCell[k])
	}

	// Index actions by (mapCode,cellX,cellY) for entity binding.
	byCell := make(map[cellKey]*CyberiaAction)
	for _, a := range actions {
		ca := protoToAction(a)
		if !mapSet[ca.SourceMapCode] {
			continue
		}
		byCell[cellKey{ca.SourceMapCode, ca.SourceCellX, ca.SourceCellY}] = ca
	}

	bound := 0
	for mapCode, ms := range s.maps {
		for _, bot := range ms.bots {
			// Bind by the bot's stable spawn-centre (its initial cell), not its
			// live position — action NPCs wander within their radius, so live
			// Pos drifts off the action's source cell after the first tick.
			key := cellKey{mapCode, int(bot.SpawnCenter.X), int(bot.SpawnCenter.Y)}
			action, ok := byCell[key]
			if !ok {
				continue
			}
			bot.ActionCode = action.Code
			s.actionCache[bot.ID] = action
			bound++
		}
	}
	log.Printf("[ActionContent] gRPC delivered %d actions, %d quests; bound %d entities",
		len(actions), len(quests), bound)
}

// cellKey identifies a map cell shared by an action and the quest(s) it awards.
type cellKey struct {
	mapCode string
	cellX   int
	cellY   int
}

// actionCell returns the cell key for an action's source cell.
func actionCell(a *CyberiaAction) cellKey {
	return cellKey{a.SourceMapCode, a.SourceCellX, a.SourceCellY}
}

// questAcceptableFor reports whether the player may start quest `code` now: it
// is not already active or completed (a failed/abandoned one is re-acceptable)
// and all its prerequisites are completed.
//
// Caller MUST hold s.mu.
func (s *GameServer) questAcceptableFor(player *PlayerState, code string) bool {
	if _, ok := s.questDefs[code]; !ok {
		return false
	}
	if qp, ok := s.playerQuest(player, code); ok && (qp.Status == "active" || qp.Status == "completed") {
		return false
	}
	return s.prerequisitesMet(player, code)
}

// resolveAcceptableQuest returns the first acceptable quest bound to the action's
// cell, or "" when none can be started right now.
//
// Caller MUST hold s.mu.
func (s *GameServer) resolveAcceptableQuest(player *PlayerState, action *CyberiaAction) string {
	for _, code := range s.questsByCell[actionCell(action)] {
		if s.questAcceptableFor(player, code) {
			return code
		}
	}
	return ""
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

func protoToQuest(q *pb.CyberiaQuestMessage) *CyberiaQuest {
	cq := &CyberiaQuest{
		Code:              q.GetCode(),
		Title:             q.GetTitle(),
		Description:       q.GetDescription(),
		UnlocksQuestCodes: q.GetUnlocksQuestCodes(),
		PrerequisiteCodes: q.GetPrerequisiteCodes(),
		SourceMapCode:     q.GetSourceMapCode(),
		SourceCellX:       int(q.GetSourceCellX()),
		SourceCellY:       int(q.GetSourceCellY()),
	}
	for _, st := range q.GetSteps() {
		step := QuestStep{ID: st.GetId(), Description: st.GetDescription()}
		for _, o := range st.GetObjectives() {
			step.Objectives = append(step.Objectives, QuestObjective{
				Type: o.GetType(), ItemID: o.GetItemId(), Quantity: int(o.GetQuantity()),
			})
		}
		cq.Steps = append(cq.Steps, step)
	}
	for _, r := range q.GetRewards() {
		cq.Rewards = append(cq.Rewards, QuestReward{
			ItemID: r.GetItemId(), Quantity: int(r.GetQuantity()),
		})
	}
	return cq
}

// logActionProviders prints every entity bound to an action — map, action
// code, entity id, initial cell (the centre of its movement radius), and
// action type. Called after the instance graph so operators can confirm the
// mission NPCs were instantiated and bound.
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

// ── Dialogue handlers (phaseInput) ──────────────────────────────────────────

// handleDlgStart binds the dialogue context and freezes the player. The
// freeze grants modal protection (immunity); it is released on
// dlg_complete / dlg_cancel.
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

// handleQuestAbandon drops an active quest, moving it to the failed section.
// Only active quests can be abandoned; completed quests are immutable. The
// failed record is kept (not deleted) so the journal can show it and so the
// grantor NPC can re-offer the quest (prerequisites permitting).
//
// Caller MUST hold s.mu.
func (s *GameServer) handleQuestAbandon(player *PlayerState, cmd *InputCommand) {
	qp, ok := s.playerQuest(player, cmd.ItemID)
	if !ok || qp.Status != "active" {
		return
	}
	qp.Status = "failed"
	s.persistQuestProgress(player, qp)
	s.sendQuestUpdate(player, []QuestSnapshotEntry{s.questSnapshot(qp)})
	log.Printf("[Quest] player %s abandoned quest %q", player.ID, qp.QuestCode)
}

// handleDlgComplete is the authoritative completion path. It validates the
// dialogue context, branches on the resolved action type, advances talk
// objectives for quest-talk, and notifies the client via dlg_ack.
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

// handleQuestAccept grants the quest the interacted NPC offers — the only path
// to start a mission. Validates that the entity provides an action with a grant
// code, the prerequisites are met, and the quest is not already active or
// completed (a failed/abandoned quest may be re-accepted). Quests are never
// granted implicitly by reading dialogue.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleQuestAccept(player *PlayerState, cmd *InputCommand) {
	action := s.actionCache[cmd.EntityID]
	if action == nil {
		return
	}
	// The client names which mission to accept (an NPC can offer several). It
	// must be a quest bound to this NPC's cell and acceptable right now; when the
	// client sends no code, fall back to the first acceptable one.
	code := cmd.ItemID
	if code != "" {
		if !containsStr(s.questsByCell[actionCell(action)], code) || !s.questAcceptableFor(player, code) {
			return
		}
	} else {
		code = s.resolveAcceptableQuest(player, action)
	}
	if code == "" {
		return
	}
	qp := s.grantQuest(player, code)
	if qp == nil {
		return
	}
	// Count items the player already holds toward the first step's collect
	// objectives at accept time — and settle/complete the step immediately when
	// they already satisfy it — so progress never waits on a fresh pickup.
	affected := []QuestSnapshotEntry{}
	s.advanceCollectObjectives(player, &affected)
	if !questSnapshotContains(affected, code) {
		affected = append(affected, s.questSnapshot(qp))
	}
	s.sendDlgAck(player, code, false, affected)
}

// questSnapshotContains reports whether a snapshot for the quest code is already
// in the list, so the granted-quest snapshot isn't appended twice.
func questSnapshotContains(entries []QuestSnapshotEntry, code string) bool {
	for i := range entries {
		if entries[i].Code == code {
			return true
		}
	}
	return false
}

// botActiveSkin returns the active skin item ID of the bot with this entity ID
// (searched across maps), or "" if not found. The bot's own active skin — not
// any action field — is what 'talk' objectives match against.
func (s *GameServer) botActiveSkin(entityID string) string {
	for _, ms := range s.maps {
		bot := ms.bots[entityID]
		if bot == nil {
			continue
		}
		// When the NPC is dead its active layers hold the ghost skin; the live
		// skin is preserved in PreRespawnObjectLayers. Read from there so a talk
		// objective still validates against a quest-giver that just died — the
		// interaction context is independent of the entity's alive/AOI state.
		layers := bot.ObjectLayers
		if bot.IsGhost() && len(bot.PreRespawnObjectLayers) > 0 {
			layers = bot.PreRespawnObjectLayers
		}
		for _, ol := range layers {
			if ol.Active && s.itemType(ol.ItemID) == "skin" {
				return ol.ItemID
			}
		}
	}
	return ""
}

// ── Quest helpers ───────────────────────────────────────────────────────────

func (s *GameServer) ensurePlayerQuests(player *PlayerState) {
	if player.Quests == nil {
		player.Quests = make(map[string]*QuestProgress)
	}
}

// playerQuest returns the player's progress for a quest code, if any.
func (s *GameServer) playerQuest(player *PlayerState, code string) (*QuestProgress, bool) {
	if player.Quests == nil {
		return nil, false
	}
	qp, ok := player.Quests[code]
	return qp, ok
}

// questCompleted reports whether the player has finished a quest.
func (s *GameServer) questCompleted(player *PlayerState, code string) bool {
	qp, ok := s.playerQuest(player, code)
	return ok && qp.Status == "completed"
}

// playerHasActionInteraction reports whether `player` currently has a real
// interaction with the action NPC `bot`: an acceptable offer (a grantable quest
// not yet active/completed with prerequisites met), or an active quest whose
// current step needs a talk with this NPC's skin. Drives the per-player Action
// Provider status so the quest-talk icon is shown only when actionable.
//
// Caller MUST hold s.mu.
func (s *GameServer) playerHasActionInteraction(player *PlayerState, action *CyberiaAction, bot *BotState) bool {
	if s.resolveAcceptableQuest(player, action) != "" {
		return true
	}
	skin := s.botActiveSkin(bot.ID)
	if skin == "" {
		return false
	}
	for _, qp := range player.Quests {
		if qp.Status != "active" {
			continue
		}
		stepIdx := firstIncompleteStepIndex(qp)
		if stepIdx < 0 {
			continue
		}
		for i := range qp.Steps[stepIdx].Objectives {
			op := &qp.Steps[stepIdx].Objectives[i]
			if op.Type == "talk" && op.ItemID == skin && op.Current < op.Required {
				return true
			}
		}
	}
	return false
}

// prerequisitesMet reports whether every prerequisite quest of `code` is
// completed for the player (AND logic). Unknown quests fail closed.
func (s *GameServer) prerequisitesMet(player *PlayerState, code string) bool {
	def, ok := s.questDefs[code]
	if !ok {
		return false
	}
	for _, pre := range def.PrerequisiteCodes {
		if pre == "" {
			continue
		}
		if !s.questCompleted(player, pre) {
			return false
		}
	}
	return true
}

// grantQuest creates an active progress record from the quest definition,
// denormalizing required counts. Best-effort persists to engine REST. Returns
// nil when the quest is unknown or its prerequisites are unmet.
func (s *GameServer) grantQuest(player *PlayerState, code string) *QuestProgress {
	def, ok := s.questDefs[code]
	if !ok {
		log.Printf("[Quest] grant skipped — unknown quest %q", code)
		return nil
	}
	if !s.prerequisitesMet(player, code) {
		log.Printf("[Quest] grant denied — prerequisites unmet for %q", code)
		return nil
	}
	s.ensurePlayerQuests(player)
	qp := &QuestProgress{
		QuestCode: code,
		Status:    "active",
		StartedAt: time.Now(),
		Steps:     make([]QuestStepProgress, 0, len(def.Steps)),
	}
	for _, step := range def.Steps {
		sp := QuestStepProgress{StepID: step.ID}
		for _, obj := range step.Objectives {
			req := obj.Quantity
			if req <= 0 {
				req = 1
			}
			sp.Objectives = append(sp.Objectives, QuestObjectiveProgress{
				Type:     obj.Type,
				ItemID:   obj.ItemID,
				Current:  0,
				Required: req,
			})
		}
		qp.Steps = append(qp.Steps, sp)
	}
	player.Quests[code] = qp
	s.persistQuestProgress(player, qp)
	log.Printf("[Quest] player %s granted quest %q", player.ID, code)
	return qp
}

// firstIncompleteStepIndex returns the index of the active step (the first
// step whose objectives are not all satisfied), or -1 when the quest is done.
func firstIncompleteStepIndex(qp *QuestProgress) int {
	for i := range qp.Steps {
		if !stepComplete(&qp.Steps[i]) {
			return i
		}
	}
	return -1
}

func stepComplete(sp *QuestStepProgress) bool {
	for i := range sp.Objectives {
		if sp.Objectives[i].Current < sp.Objectives[i].Required {
			return false
		}
	}
	return true
}

func questComplete(qp *QuestProgress) bool {
	return firstIncompleteStepIndex(qp) == -1
}

// advanceTalkObjectives advances, across all the player's active quests, every
// current-step `talk` objective that targets the NPC the player just spoke with
// (matched by the skin frozen at dlg_start). Matching on the skin — the NPC's
// identity — rather than the client-supplied dialogCode makes validation
// deterministic: it can't be stranded by the client sending a stale or
// not-yet-resolved per-quest dialogCode, and it stays correct when one NPC line
// serves several quests (Wason's intro and bounty report-back). It is also
// independent of the provider's current alive/AOI state, since the skin was
// captured while the modal was opening.
//
// On step/quest completion it delivers rewards. Returns true if it advanced.
func (s *GameServer) advanceTalkObjectives(
	player *PlayerState,
	talkedSkin string,
	affected *[]QuestSnapshotEntry,
) bool {
	if talkedSkin == "" {
		return false
	}
	advanced := false
	for _, qp := range player.Quests {
		if qp.Status != "active" {
			continue
		}
		stepIdx := firstIncompleteStepIndex(qp)
		if stepIdx < 0 {
			continue
		}
		sp := &qp.Steps[stepIdx]
		questAdvanced := false
		for oi := range sp.Objectives {
			op := &sp.Objectives[oi]
			if op.Type != "talk" || op.Current >= op.Required || op.ItemID != talkedSkin {
				continue
			}
			op.Current++
			questAdvanced = true
		}
		if questAdvanced {
			s.finalizeQuestProgress(player, qp, affected)
			advanced = true
		}
	}
	return advanced
}

// finalizeQuestProgress records an advanced quest: consuming the collect items
// of any newly-completed step, then completing the quest (rewards + unlocks)
// when all steps are done, otherwise persisting and snapshotting the new
// progress. Always appends exactly one snapshot entry per advance.
func (s *GameServer) finalizeQuestProgress(player *PlayerState, qp *QuestProgress, affected *[]QuestSnapshotEntry) {
	s.settleCompletedSteps(player, qp)
	if questComplete(qp) {
		s.completeQuest(player, qp, affected)
	} else {
		s.persistQuestProgress(player, qp)
		*affected = append(*affected, s.questSnapshot(qp))
	}
}

// settleCompletedSteps removes the `collect` items of every step that is now
// complete and not yet consumed. A step's collect items are only taken once all
// of its objectives are satisfied, so partial progress never drains inventory.
func (s *GameServer) settleCompletedSteps(player *PlayerState, qp *QuestProgress) {
	for i := range qp.Steps {
		sp := &qp.Steps[i]
		if sp.Consumed || !stepComplete(sp) {
			continue
		}
		for j := range sp.Objectives {
			op := &sp.Objectives[j]
			if op.Type == "collect" {
				s.removePlayerItem(player, op.ItemID, op.Required)
			}
		}
		sp.Consumed = true
	}
}

// removePlayerItem deducts qty of itemID from the player's inventory. Coins
// route through the flat balance; other items drain matching ObjectLayer rows.
func (s *GameServer) removePlayerItem(player *PlayerState, itemID string, qty int) {
	if qty <= 0 {
		return
	}
	if itemID == s.coinItemID {
		if int(player.Coins) <= qty {
			player.Coins = 0
		} else {
			player.Coins -= uint32(qty)
		}
		return
	}
	remaining := qty
	kept := player.ObjectLayers[:0]
	for i := range player.ObjectLayers {
		ol := player.ObjectLayers[i]
		if ol.ItemID == itemID && remaining > 0 {
			take := ol.Quantity
			if take > remaining {
				take = remaining
			}
			ol.Quantity -= take
			remaining -= take
			if ol.Quantity <= 0 {
				continue // drop the emptied stack so it leaves the inventory bar
			}
		}
		kept = append(kept, ol)
	}
	player.ObjectLayers = kept
	s.InvalidateStats(player)
}

// playerItemQuantity returns how many of itemID the player holds. Coins route
// through the flat balance; everything else sums matching ObjectLayer rows.
func (s *GameServer) playerItemQuantity(player *PlayerState, itemID string) int {
	if itemID == s.coinItemID {
		return int(player.Coins)
	}
	total := 0
	for i := range player.ObjectLayers {
		if player.ObjectLayers[i].ItemID == itemID {
			total += player.ObjectLayers[i].Quantity
		}
	}
	return total
}

// advanceKillObjectives advances any active-step `kill` objective whose target
// skin matches the just-killed entity's skin. Returns true if any advanced.
//
// Caller MUST hold s.mu.
func (s *GameServer) advanceKillObjectives(player *PlayerState, killedSkin string, affected *[]QuestSnapshotEntry) bool {
	if player == nil || killedSkin == "" {
		return false
	}
	advanced := false
	for _, qp := range player.Quests {
		if qp.Status != "active" {
			continue
		}
		stepIdx := firstIncompleteStepIndex(qp)
		if stepIdx < 0 {
			continue
		}
		sp := &qp.Steps[stepIdx]
		stepAdvanced := false
		for oi := range sp.Objectives {
			op := &sp.Objectives[oi]
			if op.Type != "kill" || op.ItemID != killedSkin || op.Current >= op.Required {
				continue
			}
			op.Current++
			stepAdvanced = true
			advanced = true
		}
		if stepAdvanced {
			s.finalizeQuestProgress(player, qp, affected)
		}
	}
	return advanced
}

// advanceCollectObjectives reconciles every active-step `collect` objective with
// the player's current inventory (idempotent: progress = min(required, held)).
// Run after any inventory gain. Returns true if any objective changed.
//
// Caller MUST hold s.mu.
func (s *GameServer) advanceCollectObjectives(player *PlayerState, affected *[]QuestSnapshotEntry) bool {
	if player == nil {
		return false
	}
	advanced := false
	for _, qp := range player.Quests {
		// Reconcile the active step; if completing it reveals a new step whose
		// collect objectives the player already satisfies, keep going so the
		// next step counts its held items too (each completed step's items are
		// deducted by settleCompletedSteps before the next reconcile reads them).
		for qp.Status == "active" {
			stepIdx := firstIncompleteStepIndex(qp)
			if stepIdx < 0 {
				break
			}
			sp := &qp.Steps[stepIdx]
			stepAdvanced := false
			for oi := range sp.Objectives {
				op := &sp.Objectives[oi]
				if op.Type != "collect" {
					continue
				}
				held := s.playerItemQuantity(player, op.ItemID)
				if held > op.Required {
					held = op.Required
				}
				if held != op.Current {
					op.Current = held
					stepAdvanced = true
				}
			}
			if !stepAdvanced {
				break
			}
			advanced = true
			s.finalizeQuestProgress(player, qp, affected)
			if firstIncompleteStepIndex(qp) == stepIdx {
				break // collect counted but step not fully done (e.g. needs a talk)
			}
		}
	}
	return advanced
}

// advancePlayerQuestsOnGain runs the inventory-driven objective checks (collect
// today) and pushes a live quest update to the player when anything changed.
//
// Caller MUST hold s.mu.
func (s *GameServer) advancePlayerQuestsOnGain(player *PlayerState) {
	var affected []QuestSnapshotEntry
	s.advanceCollectObjectives(player, &affected)
	if len(affected) > 0 {
		s.sendQuestUpdate(player, affected)
	}
}

// advancePlayerQuestsOnKill runs kill-objective advancement for the killer plus
// the inventory reconcile (loot/coins from the kill), pushing one live update.
//
// Caller MUST hold s.mu.
func (s *GameServer) advancePlayerQuestsOnKill(player *PlayerState, killedSkin string) {
	var affected []QuestSnapshotEntry
	s.advanceKillObjectives(player, killedSkin, &affected)
	s.advanceCollectObjectives(player, &affected)
	if len(affected) > 0 {
		s.sendQuestUpdate(player, affected)
	}
}

// completeQuest marks a quest completed, delivers its rewards, unlocks
// successors, and records the affected snapshot entries.
func (s *GameServer) completeQuest(player *PlayerState, qp *QuestProgress, affected *[]QuestSnapshotEntry) {
	qp.Status = "completed"
	s.deliverQuestRewards(player, qp.QuestCode)
	s.persistQuestProgress(player, qp)
	*affected = append(*affected, s.questSnapshot(qp))
	log.Printf("[Quest] player %s completed quest %q", player.ID, qp.QuestCode)
	// Successors are NOT auto-granted. Completing this quest only satisfies the
	// successors' prerequisites; the player must still explicitly accept each
	// one from its grantor NPC. Quests are never assigned automatically.
}

// deliverQuestRewards grants each reward item to the player's inventory and
// emits an item-gain FCT. Reuses the inventory-grant convention from the
// resource-drop path. Coins route through the flat coin balance.
func (s *GameServer) deliverQuestRewards(player *PlayerState, code string) {
	def, ok := s.questDefs[code]
	if !ok {
		return
	}
	cx := player.Pos.X + player.Dims.Width*0.5
	cy := player.Pos.Y + player.Dims.Height*0.5
	for _, r := range def.Rewards {
		if r.ItemID == "" || r.Quantity <= 0 {
			continue
		}
		if r.ItemID == s.coinItemID {
			s.addCoins(player, r.Quantity)
			sendFCT(player, FCTTypeCoinGain, cx, cy, r.Quantity)
			continue
		}
		found := false
		for i := range player.ObjectLayers {
			if player.ObjectLayers[i].ItemID == r.ItemID {
				player.ObjectLayers[i].Quantity += r.Quantity
				found = true
				break
			}
		}
		if !found {
			player.ObjectLayers = append(player.ObjectLayers, ObjectLayerState{
				ItemID:   r.ItemID,
				Active:   false,
				Quantity: r.Quantity,
			})
		}
		sendItemFCT(player, FCTTypeItemGain, cx, cy, r.Quantity, r.ItemID)
	}
	s.InvalidateStats(player)
}

// questSnapshot projects a progress record into the client-facing entry.
// Only authoritative runtime data is included — code, status, and progress
// counters.  All metadata (title, description, steps, rewards) is fetched
// by the C client from the engine REST endpoint /api/cyberia-quest/:code.
func (s *GameServer) questSnapshot(qp *QuestProgress) QuestSnapshotEntry {
	entry := QuestSnapshotEntry{Code: qp.QuestCode, Status: qp.Status}
	def, ok := s.questDefs[qp.QuestCode]
	if stepIdx := firstIncompleteStepIndex(qp); stepIdx >= 0 {
		sp := &qp.Steps[stepIdx]
		if ok && stepIdx < len(def.Steps) {
			entry.ActiveStep = def.Steps[stepIdx].Description
		}
		parts := make([]string, 0, len(sp.Objectives))
		for i := range sp.Objectives {
			op := &sp.Objectives[i]
			parts = append(parts, fmt.Sprintf("%d/%d %s %s", op.Current, op.Required, op.Type, op.ItemID))
		}
		entry.ObjectivesText = strings.Join(parts, ", ")
	}
	return entry
}

// buildQuestSnapshot projects all of a player's quests for the init_data
// payload. Returns an empty (non-nil) slice for a fresh guest.
func (s *GameServer) buildQuestSnapshot(player *PlayerState) []QuestSnapshotEntry {
	out := make([]QuestSnapshotEntry, 0, len(player.Quests))
	for _, qp := range player.Quests {
		out = append(out, s.questSnapshot(qp))
	}
	return out
}

// sendDlgAck pushes the notify-only dialogue acknowledgement to the player.
// affected carries any quest entries the client should upsert into its store.
func (s *GameServer) sendDlgAck(player *PlayerState, questGranted string, objectivesDone bool, affected []QuestSnapshotEntry) {
	if player.Client == nil {
		return
	}
	msg, err := json.Marshal(map[string]interface{}{
		"type": "dlg_ack",
		"payload": map[string]interface{}{
			"questGranted":   questGranted,
			"objectivesDone": objectivesDone,
			"quests":         affected,
		},
	})
	if err != nil {
		return
	}
	select {
	case player.Client.send <- msg:
	default:
	}
}

// sendQuestUpdate pushes a live quest-progress update outside the dialogue flow
// (kill / collect advancement). It reuses the dlg_ack envelope the client
// already consumes: questGranted is empty and objectivesDone marks that real
// progress landed, so the journal and notification paths fire identically.
//
// Caller MUST hold s.mu.
func (s *GameServer) sendQuestUpdate(player *PlayerState, affected []QuestSnapshotEntry) {
	if player.Client == nil || len(affected) == 0 {
		return
	}
	s.sendDlgAck(player, "", true, affected)
}

// persistQuestProgress best-effort mirrors a progress record to engine REST.
// Fire-and-forget in a goroutine so the simulation tick never blocks on the
// network; failure is logged and ignored (in-memory state is the session
// authority). The snapshot is copied so the goroutine never races the world.
func (s *GameServer) persistQuestProgress(player *PlayerState, qp *QuestProgress) {
	if s.engineApiBaseUrl == "" {
		return
	}
	body := map[string]interface{}{
		"playerId":  player.ID,
		"questCode": qp.QuestCode,
		"status":    qp.Status,
	}
	go s.enginePostJSON("/api/cyberia-quest-progress", body)
}

// enginePostJSON performs a best-effort POST; errors are logged only.
func (s *GameServer) enginePostJSON(path string, body interface{}) {
	url := strings.TrimRight(s.engineApiBaseUrl, "/") + path
	buf, err := json.Marshal(body)
	if err != nil {
		return
	}
	client := &http.Client{Timeout: 8 * time.Second}
	resp, err := client.Post(url, "application/json", strings.NewReader(string(buf)))
	if err != nil {
		log.Printf("[Quest] persist POST %s failed: %v", path, err)
		return
	}
	resp.Body.Close()
}

func containsStr(list []string, v string) bool {
	for _, s := range list {
		if s == v {
			return true
		}
	}
	return false
}
