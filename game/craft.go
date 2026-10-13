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
//	self.craft (snapshot)      — the running assembly, absent when none runs
//
// The client makes the same checks before it sends, so a refusal is a desync
// and the server drops it without an answer.
//
// Caller MUST hold s.mu for every handler here (they run inside phaseInput).
package game

import (
	"time"

	"cyberia-server/logx"
)

// craftMaxTimeMs bounds an authored assembly duration. A recipe longer than
// this would strand the player in a modal they cannot leave except by
// cancelling, so the content value is clamped rather than trusted.
const craftMaxTimeMs = 60000

// ActiveCraft is one assembly in flight. Ingredients are already consumed, so
// Refund is what a cancel returns and Outputs is what completion grants.
type ActiveCraft struct {
	EntityID    EntityID
	RecipeIndex int
	Refund      []ActionCraftItem
	Outputs     []ActionCraftItem
	CompleteAt  time.Time
}

// craftTimeMs validates an authored duration into the accepted range. Zero is
// legal and means the assembly resolves on the same tick it starts.
func craftTimeMs(recipe *ActionCraftRecipe) int {
	return min(max(recipe.CraftTimeMs, 0), craftMaxTimeMs)
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
func (s *Content) botHasCraft(id EntityID, ms *MapState) bool {
	if ms == nil || ms.vt(id).ghost() {
		return false
	}
	action := s.actionCache[id]
	return action != nil && len(action.CraftRecipes) > 0
}

// playerCanCraft reports whether the player holds every ingredient a recipe
// needs. A line with a non-positive quantity is free.
//
// Caller MUST hold s.mu.
func (s *Content) playerCanCraft(player *Session, recipe *ActionCraftRecipe) bool {
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
func (s *Content) handleCraftItem(player *Session, cmd *InputCommand) {
	if s.vt(player.ID, player.MapCode).ghost() {
		return
	}
	if s.crafts[player.ID] != nil {
		return
	}
	id := cmd.EntityID
	ms := s.findBot(id)
	if ms == nil || !s.botInPlayerRange(player, id, ms) {
		return
	}
	recipe := craftRecipeAt(s.actionCache[id], cmd.RecipeIndex)
	if recipe == nil || len(recipe.OutputItems) == 0 {
		return
	}
	// All-or-nothing: a partial consume would burn ingredients for nothing.
	if !s.playerCanCraft(player, recipe) {
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

	durationMs := craftTimeMs(recipe)
	if s.crafts == nil {
		s.crafts = make(map[EntityID]*ActiveCraft)
	}
	s.crafts[player.ID] = &ActiveCraft{
		EntityID:    id,
		RecipeIndex: cmd.RecipeIndex,
		Refund:      refund,
		Outputs:     append([]ActionCraftItem(nil), recipe.OutputItems...),
		CompleteAt:  time.Now().Add(time.Duration(durationMs) * time.Millisecond),
	}

	logx.Debugf("[CRAFT] player %s started recipe %d at %s (%dms)",
		player.ID, cmd.RecipeIndex, id, durationMs)
}

// handleCraftCancel aborts the running assembly and refunds its ingredients.
// Ignored once the timer has already paid out — there is nothing to abort.
//
// Caller MUST hold s.mu.
func (s *Content) handleCraftCancel(player *Session) {
	active := s.crafts[player.ID]
	if active == nil {
		return
	}
	delete(s.crafts, player.ID)
	for _, in := range active.Refund {
		s.addPlayerItem(player, in.ItemID, in.Qty)
	}
	logx.Debugf("[CRAFT] player %s cancelled recipe %d", player.ID, active.RecipeIndex)
}

// completeCrafts pays out every assembly whose timer elapsed this tick. Runs in
// phaseLifecycle alongside the other timed transitions.
//
// Caller MUST hold s.mu.
func (s *Content) completeCrafts(mapState *MapState) {
	now := time.Now()
	for player := range s.playersOn(mapState) {
		active := s.crafts[player.ID]
		if active == nil || now.Before(active.CompleteAt) {
			continue
		}
		delete(s.crafts, player.ID)
		for _, out := range active.Outputs {
			s.addPlayerItem(player, out.ItemID, out.Qty)
		}
		// An assembly is an inventory gain like any other — reconcile collect
		// objectives so a quest step satisfied by assembling advances at once.
		s.advancePlayerQuestsOnGain(player)
		logx.Debugf("[CRAFT] player %s completed recipe %d", player.ID, active.RecipeIndex)
	}
}

// SnapshotCraft is the running assembly in the self block.
type SnapshotCraft struct {
	EntityID    EntityID `json:"entityId"`
	RecipeIndex int      `json:"recipeIndex"`
	RemainingMs int      `json:"remainingMs"`
}

// snapshotCraft is the player's running assembly, or nil when none runs.
func (s *Content) snapshotCraft(player *Session) *SnapshotCraft {
	active := s.crafts[player.ID]
	if active == nil {
		return nil
	}
	return &SnapshotCraft{
		EntityID:    active.EntityID,
		RecipeIndex: active.RecipeIndex,
		RemainingMs: max(int(time.Until(active.CompleteAt).Milliseconds()), 0),
	}
}
