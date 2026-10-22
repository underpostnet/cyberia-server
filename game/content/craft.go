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
//	craft_item (client→server) — entity + recipe index the player assembled, quote: the fee it saw
//	craft_ack (server→client)  — a refused assembly and its reason
//	self.craft (snapshot)      — the running assembly, absent when none runs
//
// The client makes the same checks before it sends. A refusal answers
// craft_ack with its reason. The assembly holds the slots of its ingredients
// and outputs (capacity.go), so its payout and refund fit.
//
// The fee of an assembly (CraftFee) leaves the wallet with the ingredients
// and returns with them on a cancel. craft_item names the fee the player saw;
// another amount refuses it.
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
	Refund      []Stack
	Outputs     []Stack
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

// craftTakes is what an assembly takes from the player: each ingredient with
// a positive quantity, one line per item, and the fee in coins.
func (s *Content) craftTakes(recipe *ActionCraftRecipe, fee int) []Stack {
	var takes []Stack
	add := func(itemID string, qty int) {
		if qty <= 0 {
			return
		}
		for i := range takes {
			if takes[i].ItemID == itemID {
				takes[i].Qty += qty
				return
			}
		}
		takes = append(takes, Stack{ItemID: itemID, Qty: qty})
	}
	for _, in := range recipe.Ingredients {
		add(in.ItemID, in.Qty)
	}
	add(s.CoinItemID, fee)
	return takes
}

// HandleCraftItem starts an assembly: validate the assembler, the recipe, the
// fee, every ingredient and the room for the outputs, then consume the inputs
// and the fee and arm the timer. Consuming up front is what lets the client
// play the ingredient-loss animation the moment the progress bar starts; a
// cancel before completion refunds them.
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
	if ms == nil || !s.inPlayerView(player, id, ms) {
		return
	}
	recipe := craftRecipeAt(s.actionCache[id], cmd.RecipeIndex)
	if recipe == nil || len(recipe.OutputItems) == 0 {
		return
	}
	fee := s.CraftFee(recipe)
	takes := s.craftTakes(recipe, fee)
	// All-or-nothing: a partial consume would burn ingredients for nothing.
	// The ingredients stay in their slots as the refund, so the outputs need
	// room beside them.
	reason := ""
	switch {
	case int64(fee) != cmd.Quote:
		reason = RefuseQuote
	case !s.InventoryFits(player, nil, recipe.OutputItems):
		reason = RefuseFull
	}
	for _, take := range takes {
		if reason == "" && s.playerItemQuantity(player, take.ItemID) < take.Qty {
			reason = RefuseNoStack
			if take.ItemID == s.CoinItemID {
				reason = RefuseInsufficient
			}
		}
	}
	s.recordEconomy("craft", reason)
	if reason != "" {
		world.SendAck(player, "craft_ack", reason, map[string]interface{}{
			"entityId": id, "recipeIndex": cmd.RecipeIndex, "fee": fee,
		})
		return
	}
	for _, take := range takes {
		s.removePlayerItem(player, take.ItemID, take.Qty)
	}

	durationMs := craftTimeMs(recipe)
	if s.crafts == nil {
		s.crafts = make(map[world.EntityID]*ActiveCraft)
	}
	s.crafts[player.ID] = &ActiveCraft{
		EntityID:    id,
		RecipeIndex: cmd.RecipeIndex,
		Refund:      takes,
		Outputs:     append([]Stack(nil), recipe.OutputItems...),
		CompleteAt:  time.Now().Add(time.Duration(durationMs) * time.Millisecond),
	}

	logx.Debugf("[CRAFT] player %s started recipe %d at %s (%dms, fee %d)",
		player.ID, cmd.RecipeIndex, id, durationMs, fee)
}

// HandleCraftCancel aborts the running assembly and refunds its ingredients.
// Ignored once the timer has already paid out — there is nothing to abort.
//
// Caller MUST hold s.mu.
func (s *Content) HandleCraftCancel(player *world.Session) {
	active := s.crafts[player.ID]
	if active == nil || !s.ExchangeItems(player, nil, active.Refund) {
		return
	}
	delete(s.crafts, player.ID)
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
		if active == nil || now.Before(active.CompleteAt) || !s.ExchangeItems(player, nil, active.Outputs) {
			continue
		}
		delete(s.crafts, player.ID)
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
