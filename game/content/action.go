// Package content — action.go
//
// Server-side cyberia-action runtime: the interactive-NPC / dialogue (talk)
// layer. engine-cyberia owns the persisted CyberiaAction content and exposes it
// over REST; the Go simulation binds each action to the entity on its source
// cell. Reading a dialogue advances quest `talk` objectives (see quest.go) —
// that is the only coupling between the two otherwise-independent flows.
//
// Cross-process contract:
//
//	talk_done (client→server) — the client read dialogue D with entity E to the
//	                            end: advance talk objectives for that NPC
//
// Caller MUST hold s.mu for every handler / mutation helper here (they run inside
// phaseInput). Binding runs at world (re)build time, also under s.mu.
package content

import (
	"cyberia-server/game/world"
	"cyberia-server/logx"

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

// ActionShopItem is one vendor catalog row: an item on sale and what it costs.
// Items are named by item id; the engine's catalog resolves their definitions.
type ActionShopItem struct {
	ItemID      string `json:"itemId"`
	PriceItemID string `json:"priceItemId"`
	PriceQty    int    `json:"priceQty"`
}

// ActionCraftRecipe is one assembler recipe: ingredients consumed, outputs
// produced. Recipes are addressed by their index in the action's list.
type ActionCraftRecipe struct {
	OutputItems []Stack `json:"outputItems"`
	Ingredients []Stack `json:"ingredients"`
	CraftTimeMs int     `json:"craftTimeMs"`
}

type CyberiaAction struct {
	Code               string                `json:"code"`
	Label              string                `json:"label"`
	SourceMapCode      string                `json:"sourceMapCode"`
	SourceCellX        int                   `json:"sourceCellX"`
	SourceCellY        int                   `json:"sourceCellY"`
	DialogCode         string                `json:"dialogCode"`
	QuestDialogueCodes []ActionQuestDialogue `json:"questDialogueCodes"`
	ShopItems          []ActionShopItem      `json:"shopItems"`
	CraftRecipes       []ActionCraftRecipe   `json:"craftRecipes"`
	StorageSlots       int                   `json:"storageSlots"`
}

// actionCell returns the cell key for an action's source cell.
func actionCell(a *CyberiaAction) world.CellKey {
	return world.CellKey{MapCode: a.SourceMapCode, CellX: a.SourceCellX, CellY: a.SourceCellY}
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
	for _, si := range a.GetShopItems() {
		ca.ShopItems = append(ca.ShopItems, ActionShopItem{
			ItemID:      si.GetItemId(),
			PriceItemID: si.GetPriceItemId(),
			PriceQty:    int(si.GetPriceQty()),
		})
	}
	for _, r := range a.GetCraftRecipes() {
		ca.CraftRecipes = append(ca.CraftRecipes, ActionCraftRecipe{
			OutputItems: protoToCraftItems(r.GetOutputItems()),
			Ingredients: protoToCraftItems(r.GetIngredients()),
			CraftTimeMs: int(r.GetCraftTimeMs()),
		})
	}
	ca.StorageSlots = int(a.GetStorageSlots())
	return ca
}

func protoToCraftItems(items []*pb.ActionCraftItem) []Stack {
	out := make([]Stack, 0, len(items))
	for _, i := range items {
		out = append(out, Stack{ItemID: i.GetItemId(), Qty: int(i.GetQty())})
	}
	return out
}

// BindActions caches the CyberiaAction content the engine delivered with the
// world over gRPC and binds each action to the runtime entity on its source cell
// — keyed by the bot's stable spawn-centre, not its live position, since action
// NPCs wander within their radius.
//
// Caller MUST hold s.mu.
func (s *Content) BindActions(mapCodes []string, actions []*pb.CyberiaActionMessage) {
	s.actionCache = make(map[world.EntityID]*CyberiaAction)

	mapSet := make(map[string]bool, len(mapCodes))
	for _, c := range mapCodes {
		mapSet[c] = true
	}

	byCell := make(map[world.CellKey]*CyberiaAction)
	for _, a := range actions {
		ca := protoToAction(a)
		if !mapSet[ca.SourceMapCode] {
			continue
		}
		byCell[actionCell(ca)] = ca
	}

	bound := 0
	for _, ms := range s.Maps {
		for id := range ms.OfKind(world.EntityBot) {
			action, ok := byCell[botSpawnCell(ms, id)]
			if !ok {
				continue
			}
			ms.Ag(id).ActionCode = action.Code
			s.actionCache[id] = action
			bound++
		}
	}
	logx.Infof("[ActionContent] gRPC delivered %d actions; bound %d entities", len(actions), bound)
}

// LogActionProviders prints every entity bound to a cyberia-action — map, action
// code, label, the quests bound to its cell, entity id, and initial cell. Called
// after the instance graph so operators can confirm the action NPCs were
// instantiated and bound.
func (s *Content) LogActionProviders() {
	if len(s.actionCache) == 0 {
		logx.Infof("[ActionProviders] none bound (no actions delivered via gRPC, or none match an entity cell)")
		return
	}
	logx.Debugf("[ActionProviders] %d action-provider entities:", len(s.actionCache))
	for _, ms := range s.Maps {
		for id := range ms.OfKind(world.EntityBot) {
			if ms.Ag(id).ActionCode == "" {
				continue
			}
			a := s.actionCache[id]
			label := ""
			var quests []string
			if a != nil {
				label = a.Label
				quests = s.questsByCell[actionCell(a)]
			}
			logx.Debugf("[ActionProviders]   map=%s action=%s label=%q quests=%v entity=%d initPos=(%d,%d)",
				ms.Code(), ms.Ag(id).ActionCode, label, quests, id,
				int(ms.Tf(id).Pos.X), int(ms.Tf(id).Pos.Y))
		}
	}
}

// findBot returns the map that holds the bot with this entity ID, or nil.
//
// Caller MUST hold s.mu.
func (s *Content) findBot(id world.EntityID) *world.MapState {
	for _, ms := range s.Maps {
		if ms.IsKind(id, world.EntityBot) {
			return ms
		}
	}
	return nil
}

// BotActiveSkin returns the active skin item ID of the bot with this entity ID
// (searched across maps), or "" if not found. The bot's own active skin — not
// any action field — is what 'talk' objectives match against.
func (s *Content) BotActiveSkin(id world.EntityID) string {
	ms := s.findBot(id)
	if ms == nil {
		return ""
	}
	return s.activeSkinOf(id, ms)
}

// PlayerSkin is the active skin of a player: the item that draws its identity
// in social views. "" when it wears none.
func (s *Content) PlayerSkin(p *world.Session) string {
	ms := s.Maps[p.MapCode]
	if ms == nil {
		return ""
	}
	return s.activeSkinOf(p.ID, ms)
}

// activeSkinOf reads the active skin of an entity on a map directly (no
// lookup), so the AOI path that already holds it avoids an O(N) map search.
func (s *Content) activeSkinOf(id world.EntityID, ms *world.MapState) string {
	// A dead entity's active layers hold the ghost skin; the live skin is
	// preserved in PreRespawnObjectLayers. Read from there so a talk objective
	// still validates against a quest-giver that just died, and a dead player
	// keeps its identity.
	layers := ms.Ap(id).Layers
	if ms.Vt(id).Ghost() && len(ms.Mt(id).PreRespawnObjectLayers) > 0 {
		layers = ms.Mt(id).PreRespawnObjectLayers
	}
	for _, ol := range layers {
		if ol.Active && s.ItemType(ol.ItemID) == "skin" {
			return ol.ItemID
		}
	}
	return ""
}

// BotInteractionFlags builds the per-player interaction capability bitmask.
// Both bits mean "something actionable for this player right now" — they drive
// the overhead attention icons, so neither may light for mere presence of
// interactions: the action bit when a pending action-talk-quest can be advanced
// here (`hasPendingActionTalk`, from pendingActionTalkDialog — NOT merely a
// bound action) or the entity is a live vendor (`hasShop`, a capability any
// player can use on sight), the quest bit only for an actionable quest
// (`hasActionableQuest`, from botHasActionableQuest — acceptable or advanceable,
// never completed feedback). The full quest list still travels separately in
// the bot block (botQuestCodes), so the interact modal keeps its Quest tab and
// completed feedback without any icon.
func (s *Content) BotInteractionFlags(hasActionableQuest, hasPendingActionTalk, hasUsableAction bool) uint8 {
	var flags uint8
	if hasPendingActionTalk || hasUsableAction {
		flags |= InteractionFlagAction
	}
	if hasActionableQuest {
		flags |= InteractionFlagQuest
	}
	return flags
}

// PendingActionTalkDialogs returns, PARALLEL to questCodes, the dialogue code
// this NPC's action maps for each quest the player can advance here right now:
// entry i is non-empty when questCodes[i] is active, its current step has an
// incomplete `talk` objective matching this bot's skin, and the action maps
// that quest to a dialogue (questDialogueCodes). Every mapped quest gets its
// own entry, so the client can surface one quest-talk button per objective.
//
// The parallel shape lets the client label each button from the quest metadata
// it already fetches by code. craft/storage are reference-only; the shop
// capability lives in shop.go.
//
// Caller MUST hold s.mu.
func (s *Content) PendingActionTalkDialogs(player *world.Session, id world.EntityID, ms *world.MapState, questCodes []string) []string {
	dialogs := make([]string, len(questCodes))
	action := s.actionCache[id]
	if action == nil || len(action.QuestDialogueCodes) == 0 {
		return dialogs
	}
	skin := s.activeSkinOf(id, ms)
	if skin == "" {
		return dialogs
	}
	for i, code := range questCodes {
		qp, ok := s.playerQuest(player, code)
		if !ok || qp.Status != "active" {
			continue
		}
		stepIdx := firstIncompleteStepIndex(qp)
		if stepIdx < 0 || !stepHasIncompleteTalk(&qp.Steps[stepIdx], skin) {
			continue
		}
		dialogs[i] = questTalkDialogCode(action, code)
	}
	return dialogs
}

// BotHasUsableAction reports whether a bot exposes an action capability any
// player can use on sight — a live vendor or assembler today. The single place
// that decides what lights the action-provider bit, so adding a capability does
// not grow botInteractionFlags.
//
// Caller MUST hold s.mu.
func (s *Content) BotHasUsableAction(id world.EntityID, ms *world.MapState) bool {
	return s.botHasShop(id, ms) || s.botHasCraft(id, ms) || s.botHasStorage(id, ms)
}

// inPlayerView reports whether an entity can be interacted with: same map,
// and inside the player's own AOI — the visibility window that put the entity on
// screen in the first place.
//
// Caller MUST hold s.mu.
func (s *Content) inPlayerView(player *world.Session, id world.EntityID, ms *world.MapState) bool {
	if ms.Code() != player.MapCode {
		return false
	}
	tf := ms.Tf(id)
	rect := world.Rectangle{
		MinX: tf.Pos.X, MinY: tf.Pos.Y,
		MaxX: tf.Pos.X + tf.Size.X, MaxY: tf.Pos.Y + tf.Size.Y,
	}
	return world.RectsOverlap(player.AOI, rect)
}

// VisiblePlayer is the player id inside the viewer's view, or nil.
//
// Caller MUST hold s.mu.
func (s *Content) VisiblePlayer(viewer *world.Session, id world.EntityID) *world.Session {
	ms := s.Maps[viewer.MapCode]
	target := s.PlayerOn(ms, id)
	if target == nil || !s.inPlayerView(viewer, id, ms) {
		return nil
	}
	return target
}

// ── Dialogue (phaseInput) ───────────────────────────────────────────────────

// HandleTalkDone advances the `talk` objectives for a dialogue the client read
// to the end. The skin comes from botActiveSkin at apply time, which reads a
// dead bot's pre-respawn skin and has no AOI test. When the NPC's action maps a
// quest→dialogue, the dialogue code must match that mapping
// (advanceTalkObjectives). It never grants a quest — that is quest_accept.
//
// Caller MUST hold s.mu.
func (s *Content) HandleTalkDone(player *world.Session, cmd *world.InputCommand) {
	if s.Vt(player.ID, player.MapCode).Ghost() {
		return
	}
	var affected []QuestSnapshotEntry
	s.advanceTalkObjectives(player, s.BotActiveSkin(cmd.EntityID), s.actionCache[cmd.EntityID],
		cmd.DialogCode, &affected)
	s.advanceCollectObjectives(player, &affected)
	s.sendQuestUpdate(player, affected)
}
