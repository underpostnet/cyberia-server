// Package game — craft.go
//
// Authoritative assembler transactions. engine-cyberia owns the recipe book: a
// CyberiaAction carrying `craftRecipes` makes the entity on its source cell a
// fabrication terminal, and the recipes arrive with the world over gRPC
// (action.go). The client renders the same book by action code over REST and
// addresses a recipe by its index; it never decides whether a synthesis is
// affordable.
//
// Cross-process contract:
//
//	craft_item (client→server) — entity + recipe index the player assembled
//	craft_ack  (server→client) — accepted/rejected, so the presentation layer can
//	                             play the synthesis FX or surface the reason
//
// Caller MUST hold s.mu for every handler here (they run inside phaseInput).
package game

import (
	"time"

	"cyberia-server/logx"
)

// Rejection reasons echoed back on craft_ack. The client maps them to a message.
const (
	craftRejectNoAssembler = "no_assembler"
	craftRejectNoRecipe    = "no_recipe"
	craftRejectOutOfRange  = "out_of_range"
	craftRejectMissing     = "missing_ingredients"
	craftRejectBusy        = "already_assembling"
)

// craftMaxTimeMs bounds an authored assembly duration. A recipe longer than
// this would strand the player in a modal they cannot leave except by
// cancelling, so the content value is clamped rather than trusted.
const craftMaxTimeMs = 60000

// ActiveCraft is one assembly in flight. Ingredients are already consumed, so
// Refund is what a cancel returns and Outputs is what completion grants.
type ActiveCraft struct {
	EntityID    string
	RecipeIndex int
	Refund      []ActionCraftItem
	Outputs     []ActionCraftItem
	CompleteAt  time.Time
}

// craftTimeMs validates an authored duration into the accepted range. Zero is
// legal and means the assembly resolves on the same tick it starts.
func craftTimeMs(recipe *ActionCraftRecipe) int {
	if recipe.CraftTimeMs < 0 {
		return 0
	}
	if recipe.CraftTimeMs > craftMaxTimeMs {
		return craftMaxTimeMs
	}
	return recipe.CraftTimeMs
}

// craftRecipeAt returns the recipe an action holds at `index`, or nil.
func craftRecipeAt(action *CyberiaAction, index int) *ActionCraftRecipe {
	if action == nil || index < 0 || index >= len(action.CraftRecipes) {
		return nil
	}
	return &action.CraftRecipes[index]
}

// botHasCraft reports whether a bot is a live assembler — its bound action
// carries at least one recipe and it is not in its dead/respawn state. Feeds
// botHasUsableAction alongside the vendor capability.
//
// Caller MUST hold s.mu.
func (s *GameServer) botHasCraft(bot *BotState) bool {
	if bot == nil || bot.IsGhost() {
		return false
	}
	action := s.actionCache[bot.ID]
	return action != nil && len(action.CraftRecipes) > 0
}

// playerCanCraft reports whether the player holds every ingredient a recipe
// needs. A line with a non-positive quantity is free.
//
// Caller MUST hold s.mu.
func (s *GameServer) playerCanCraft(player *PlayerState, recipe *ActionCraftRecipe) bool {
	for _, in := range recipe.Ingredients {
		if in.Qty <= 0 {
			continue
		}
		if s.playerItemQuantity(player, in.ItemID) < in.Qty {
			return false
		}
	}
	return true
}

// handleCraftItem starts an assembly: validate the assembler, the recipe and
// every ingredient, then consume the inputs and arm the timer. Consuming up
// front is what lets the client play the ingredient-loss animation the moment
// the progress bar starts; a cancel before completion refunds them.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleCraftItem(player *PlayerState, cmd *InputCommand) {
	if player.IsGhost() {
		return
	}
	if player.ActiveCraft != nil {
		s.sendCraftAck(player, cmd.EntityID, cmd.RecipeIndex, 0, craftRejectBusy)
		return
	}
	bot := s.findBot(cmd.EntityID)
	if bot == nil {
		s.sendCraftAck(player, cmd.EntityID, cmd.RecipeIndex, 0, craftRejectNoAssembler)
		return
	}
	if !botInPlayerRange(player, bot) {
		s.sendCraftAck(player, cmd.EntityID, cmd.RecipeIndex, 0, craftRejectOutOfRange)
		return
	}
	recipe := craftRecipeAt(s.actionCache[bot.ID], cmd.RecipeIndex)
	if recipe == nil || len(recipe.OutputItems) == 0 {
		s.sendCraftAck(player, cmd.EntityID, cmd.RecipeIndex, 0, craftRejectNoRecipe)
		return
	}
	// Assert the modal protection here rather than trusting the client to still
	// hold it, so an assembly can never run while the player is exposed.
	s.holdProviderFreeze(player, bot.ID)

	// All-or-nothing: a partial consume would burn ingredients for nothing.
	if !s.playerCanCraft(player, recipe) {
		s.sendCraftAck(player, cmd.EntityID, cmd.RecipeIndex, 0, craftRejectMissing)
		return
	}

	refund := make([]ActionCraftItem, 0, len(recipe.Ingredients))
	for _, in := range recipe.Ingredients {
		if in.Qty <= 0 {
			continue
		}
		s.removePlayerItem(player, in.ItemID, in.Qty)
		refund = append(refund, in)
	}

	ms := craftTimeMs(recipe)
	player.ActiveCraft = &ActiveCraft{
		EntityID:    bot.ID,
		RecipeIndex: cmd.RecipeIndex,
		Refund:      refund,
		Outputs:     append([]ActionCraftItem(nil), recipe.OutputItems...),
		CompleteAt:  time.Now().Add(time.Duration(ms) * time.Millisecond),
	}
	s.sendCraftAck(player, cmd.EntityID, cmd.RecipeIndex, ms, "")

	logx.Debugf("[CRAFT] player %s started recipe %d at %s (%dms)",
		player.ID, cmd.RecipeIndex, bot.ID, ms)
}

// handleCraftCancel aborts the running assembly and refunds its ingredients.
// Ignored once the timer has already paid out — there is nothing to abort.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleCraftCancel(player *PlayerState) {
	active := player.ActiveCraft
	if active == nil {
		return
	}
	player.ActiveCraft = nil
	for _, in := range active.Refund {
		s.addPlayerItem(player, in.ItemID, in.Qty)
	}
	// The interact modal is still open behind the assembly card, so the session
	// keeps its protection until the player closes it.
	s.holdProviderFreeze(player, active.EntityID)
	logx.Debugf("[CRAFT] player %s cancelled recipe %d", player.ID, active.RecipeIndex)
}

// completeCrafts pays out every assembly whose timer elapsed this tick. Runs in
// phaseLifecycle alongside the other timed transitions.
//
// Caller MUST hold s.mu.
func (s *GameServer) completeCrafts(mapState *MapState) {
	now := time.Now()
	for _, player := range mapState.players {
		active := player.ActiveCraft
		if active == nil || now.Before(active.CompleteAt) {
			continue
		}
		player.ActiveCraft = nil
		for _, out := range active.Outputs {
			s.addPlayerItem(player, out.ItemID, out.Qty)
		}
		// An assembly is an inventory gain like any other — reconcile collect
		// objectives so a quest step satisfied by assembling advances at once.
		s.advancePlayerQuestsOnGain(player)
		// The player is still standing in the interact modal watching the
		// result arrive: re-assert the session freeze so finishing an assembly
		// never leaves them killable. Their freeze_end on closing the modal
		// releases it.
		s.holdProviderFreeze(player, active.EntityID)
		logx.Debugf("[CRAFT] player %s completed recipe %d", player.ID, active.RecipeIndex)
	}
}

// sendCraftAck answers a start request. An empty reason means the assembly is
// running and `craftTimeMs` is the authoritative duration the client's progress
// bar adopts; anything else is a rejection code it renders.
func (s *GameServer) sendCraftAck(player *PlayerState, entityID string, recipeIndex,
	craftTimeMs int, reason string) {
	sendMessage(player, "craft_ack", map[string]interface{}{
		"entityId":    entityID,
		"recipeIndex": recipeIndex,
		"craftTimeMs": craftTimeMs,
		"ok":          reason == "",
		"reason":      reason,
	})
}
