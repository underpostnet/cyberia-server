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
	"strings"
	"time"
)

// ── Content shapes (mirror engine-cyberia Mongo schemas) ────────────────────

// CyberiaAction mirrors src/api/cyberia-action/cyberia-action.model.js.
// Only the fields the simulation needs are decoded.
type CyberiaAction struct {
	Code               string   `json:"code"`
	Type               string   `json:"type"`
	Label              string   `json:"label"`
	SourceMapCode      string   `json:"sourceMapCode"`
	SourceCellX        int      `json:"sourceCellX"`
	SourceCellY        int      `json:"sourceCellY"`
	ProvideItemID      string   `json:"provideItemId"`
	GrantQuestCode     string   `json:"grantQuestCode"`
	DialogCode         string   `json:"dialogCode"`
	QuestDialogueCodes []string `json:"questDialogueCodes"`
}

// CyberiaQuest mirrors src/api/cyberia-quest/cyberia-quest.model.js.
type CyberiaQuest struct {
	Code              string        `json:"code"`
	Title             string        `json:"title"`
	Description       string        `json:"description"`
	UnlocksQuestCodes []string      `json:"unlocksQuestCodes"`
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
}

type QuestObjectiveProgress struct {
	Type     string `json:"type"`
	ItemID   string `json:"itemId"`
	Current  int    `json:"current"`
	Required int    `json:"required"`
}

// QuestSnapshotEntry is the client-facing projection used by both init_data
// and dlg_ack so the C client's quest_store can render the Quest Journal
// without extra REST calls.
type QuestSnapshotEntry struct {
	Code           string `json:"code"`
	Title          string `json:"title"`
	Description    string `json:"description"`
	Status         string `json:"status"`
	ActiveStep     string `json:"activeStep"`
	ObjectivesText string `json:"objectivesText"`
}

// ── Instance init: REST fetch + entity binding ──────────────────────────────

// engineRESTList holds the paginated list envelope returned by the engine
// content controllers ({ data, total, page, totalPages }).
type engineRESTList[T any] struct {
	Data []T `json:"data"`
}

// loadActionContent fetches the CyberiaAction / CyberiaQuest documents for the
// instance's maps from engine-cyberia REST and binds each action to the
// matching runtime entity. Resilient: any fetch failure leaves the caches
// empty and logs — the world still runs, entities simply provide no actions.
//
// Caller MUST hold s.mu.
func (s *GameServer) loadActionContent(mapCodes []string) {
	s.actionCache = make(map[string]*CyberiaAction)
	s.questDefs = make(map[string]*CyberiaQuest)

	if s.engineApiBaseUrl == "" {
		log.Printf("[ActionContent] ENGINE_API_BASE_URL unset — skipping action/quest load")
		return
	}

	mapSet := make(map[string]bool, len(mapCodes))
	for _, c := range mapCodes {
		mapSet[c] = true
	}

	var actions engineRESTList[CyberiaAction]
	if err := s.engineGetJSON("/api/cyberia-action?limit=10000", &actions); err != nil {
		log.Printf("[ActionContent] action fetch failed: %v", err)
	}
	var quests engineRESTList[CyberiaQuest]
	if err := s.engineGetJSON("/api/cyberia-quest?limit=10000", &quests); err != nil {
		log.Printf("[ActionContent] quest fetch failed: %v", err)
	}

	for i := range quests.Data {
		q := quests.Data[i]
		s.questDefs[q.Code] = &q
	}

	// Index actions by (mapCode,cellX,cellY) for entity binding.
	type cellKey struct {
		mapCode    string
		cellX      int
		cellY      int
	}
	byCell := make(map[cellKey]*CyberiaAction)
	for i := range actions.Data {
		a := actions.Data[i]
		if !mapSet[a.SourceMapCode] {
			continue
		}
		byCell[cellKey{a.SourceMapCode, a.SourceCellX, a.SourceCellY}] = &a
	}

	bound := 0
	for mapCode, ms := range s.maps {
		for _, bot := range ms.bots {
			key := cellKey{mapCode, int(bot.Pos.X), int(bot.Pos.Y)}
			action, ok := byCell[key]
			if !ok {
				continue
			}
			bot.ActionCode = action.Code
			s.actionCache[bot.ID] = action
			bound++
		}
	}
	log.Printf("[ActionContent] loaded %d actions, %d quests; bound %d entities",
		len(actions.Data), len(s.questDefs), bound)
}

// engineGetJSON performs a GET against the engine REST API and decodes the
// JSON body into out. Short timeout; never blocks the simulation for long.
func (s *GameServer) engineGetJSON(path string, out interface{}) error {
	url := strings.TrimRight(s.engineApiBaseUrl, "/") + path
	client := &http.Client{Timeout: 8 * time.Second}
	resp, err := client.Get(url)
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return fmt.Errorf("GET %s → %d", path, resp.StatusCode)
	}
	return json.NewDecoder(resp.Body).Decode(out)
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
	player.ActiveDialogueEntityID = cmd.EntityID
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
	ThawPlayer(player, "dialogue")
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
	player.ActiveDialogueEntityID = ""
	ThawPlayer(player, "dialogue")

	action := s.actionCache[cmd.EntityID]
	if action == nil {
		return
	}

	// `talk` — acknowledge only, no quest side effects.
	if action.Type == "talk" {
		s.sendDlgAck(player, "", false, nil)
		return
	}

	// `quest-talk` — grant on first contact, then advance talk objectives.
	var affected []QuestSnapshotEntry
	questGranted := ""
	if action.GrantQuestCode != "" {
		if _, exists := s.playerQuest(player, action.GrantQuestCode); !exists {
			if qp := s.grantQuest(player, action.GrantQuestCode); qp != nil {
				questGranted = action.GrantQuestCode
				affected = append(affected, s.questSnapshot(qp))
			}
		}
	}

	objectivesDone := s.advanceTalkObjectives(player, action, cmd.DialogCode, &affected)

	s.sendDlgAck(player, questGranted, objectivesDone, affected)
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

// grantQuest creates an active progress record from the quest definition,
// denormalizing required counts. Best-effort persists to engine REST.
func (s *GameServer) grantQuest(player *PlayerState, code string) *QuestProgress {
	def, ok := s.questDefs[code]
	if !ok {
		log.Printf("[Quest] grant skipped — unknown quest %q", code)
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

// advanceTalkObjectives increments any active-step `talk` objective in the
// player's active quests that matches the action's provideItemId, but only
// when the just-read dialogCode is one of the action's questDialogueCodes.
// On quest completion it delivers rewards and unlocks successors. Returns
// true if any objective advanced.
func (s *GameServer) advanceTalkObjectives(
	player *PlayerState,
	action *CyberiaAction,
	dialogCode string,
	affected *[]QuestSnapshotEntry,
) bool {
	if !containsStr(action.QuestDialogueCodes, dialogCode) {
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
		for oi := range sp.Objectives {
			op := &sp.Objectives[oi]
			if op.Type != "talk" || op.ItemID != action.ProvideItemID {
				continue
			}
			if op.Current < op.Required {
				op.Current++
				advanced = true
			}
		}
		if advanced && questComplete(qp) {
			s.completeQuest(player, qp, affected)
		} else if advanced {
			s.persistQuestProgress(player, qp)
			*affected = append(*affected, s.questSnapshot(qp))
		}
	}
	return advanced
}

// completeQuest marks a quest completed, delivers its rewards, unlocks
// successors, and records the affected snapshot entries.
func (s *GameServer) completeQuest(player *PlayerState, qp *QuestProgress, affected *[]QuestSnapshotEntry) {
	qp.Status = "completed"
	s.deliverQuestRewards(player, qp.QuestCode)
	s.persistQuestProgress(player, qp)
	*affected = append(*affected, s.questSnapshot(qp))
	log.Printf("[Quest] player %s completed quest %q", player.ID, qp.QuestCode)

	def, ok := s.questDefs[qp.QuestCode]
	if !ok {
		return
	}
	for _, next := range def.UnlocksQuestCodes {
		if next == "" {
			continue
		}
		if _, exists := s.playerQuest(player, next); exists {
			continue
		}
		if nqp := s.grantQuest(player, next); nqp != nil {
			*affected = append(*affected, s.questSnapshot(nqp))
		}
	}
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

// questSnapshot projects a progress record into the client-facing entry,
// resolving title/description and the active-step summary from the def.
func (s *GameServer) questSnapshot(qp *QuestProgress) QuestSnapshotEntry {
	entry := QuestSnapshotEntry{Code: qp.QuestCode, Status: qp.Status}
	def, ok := s.questDefs[qp.QuestCode]
	if ok {
		entry.Title = def.Title
		entry.Description = def.Description
	}
	if entry.Title == "" {
		entry.Title = qp.QuestCode
	}
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
