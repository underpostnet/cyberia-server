// Package content — craft.go
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
package content

import (
	"time"

	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// craftMaxTimeMs bounds an authored assembly duration. A recipe longer than
// this would strand the player in a modal they cannot leave except by
// cancelling, so the content value is clamped rather than trusted.
const craftMaxTimeMs = 60000

// ActiveCraft is one assembly in flight. Ingredients are already consumed, so
// Refund is what a cancel returns and Outputs is what completion grants.
type ActiveCraft struct {
	EntityID    world.EntityID
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
func (s *Content) botHasCraft(id world.EntityID, ms *world.MapState) bool {
	if ms == nil || ms.Vt(id).Ghost() {
		return false
	}
	action := s.actionCache[id]
	return action != nil && len(action.CraftRecipes) > 0
}

// playerCanCraft reports whether the player holds every ingredient a recipe
// needs. A line with a non-positive quantity is free.
//
// Caller MUST hold s.mu.
func (s *Content) playerCanCraft(player *world.Session, recipe *ActionCraftRecipe) bool {
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

// HandleCraftItem starts an assembly: validate the assembler, the recipe and
// every ingredient, then consume the inputs and arm the timer. Consuming up
// front is what lets the client play the ingredient-loss animation the moment
// the progress bar starts; a cancel before completion refunds them.
//
// Caller MUST hold s.mu.
func (s *Content) HandleCraftItem(player *world.Session, cmd *world.InputCommand) {
	if s.Vt(player.ID, player.MapCode).Ghost() {
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
		s.crafts = make(map[world.EntityID]*ActiveCraft)
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

// HandleCraftCancel aborts the running assembly and refunds its ingredients.
// Ignored once the timer has already paid out — there is nothing to abort.
//
// Caller MUST hold s.mu.
func (s *Content) HandleCraftCancel(player *world.Session) {
	active := s.crafts[player.ID]
	if active == nil {
		return
	}
	delete(s.crafts, player.ID)
	for _, in := range active.Refund {
		s.AddPlayerItem(player, in.ItemID, in.Qty)
	}
	logx.Debugf("[CRAFT] player %s cancelled recipe %d", player.ID, active.RecipeIndex)
}

// CompleteCrafts pays out every assembly whose timer elapsed this tick. Runs in
// phaseLifecycle alongside the other timed transitions.
//
// Caller MUST hold s.mu.
func (s *Content) CompleteCrafts(mapState *world.MapState) {
	now := time.Now()
	for player := range s.PlayersOn(mapState) {
		active := s.crafts[player.ID]
		if active == nil || now.Before(active.CompleteAt) {
			continue
		}
		delete(s.crafts, player.ID)
		for _, out := range active.Outputs {
			s.AddPlayerItem(player, out.ItemID, out.Qty)
		}
		// An assembly is an inventory gain like any other — reconcile collect
		// objectives so a quest step satisfied by assembling advances at once.
		s.AdvancePlayerQuestsOnGain(player)
		logx.Debugf("[CRAFT] player %s completed recipe %d", player.ID, active.RecipeIndex)
	}
}

// SnapshotCraft is the running assembly in the self block.
type SnapshotCraft struct {
	EntityID    world.EntityID `json:"entityId"`
	RecipeIndex int            `json:"recipeIndex"`
	RemainingMs int            `json:"remainingMs"`
}

// SnapshotCraft is the player's running assembly, or nil when none runs.
func (s *Content) SnapshotCraft(player *world.Session) *SnapshotCraft {
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
