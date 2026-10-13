package game

// economy.go — Fountain & Sink Economy Module
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │  COIN BALANCE DESIGN (flat + display split)                             │
// │                                                                         │
// │  entity.Coins uint32   ← SINGLE SOURCE OF TRUTH for all economy logic  │
// │                           O(1) read/write, no OL traversal.             │
// │                                                                         │
// │  coin ObjectLayer slot ← DISPLAY ONLY (inventory bar / modal)          │
// │    .Active   = false     coins can never be "worn" or activated.        │
// │    .Quantity = Coins     synced by syncCoinOL() on every mutation.      │
// │                                                                         │
// │  WHY SPLIT:                                                             │
// │  • Economy math uses Coins directly (O(1)); avoids iterating OLs.      │
// │  • Coin slot is always Active:false — inventory UI reflects this        │
// │    with a lock indicator (non-activable) automatically.                 │
// │  • No risk of coin OL being activated by the item_activation handler.  │
// │  • ObjectLayer.activable:false is enforced server-side and visible      │
// │    to the C client via the ObjectLayer metadata fetch.                  │
// │                                                                         │
// │  FOUNTAINS  (new coin supply enters the economy)                        │
// │  • botSpawnCoins    — each bot spawns carrying N coins; reset on        │
// │                       every respawn → infinite mint bounded by kill     │
// │                       activity rate.                                    │
// │  • playerSpawnCoins — starting wallet for every guest session.          │
// │                       Currently in-memory only (no DB persistence).     │
// │                                                                         │
// │  KILL LOOT  (zero-sum redistribution via grid drops)                    │
// │  Every kill scatters the victim's coin stake as a grid token that only  │
// │  damage contributors — players and bots — may race to collect (loot.go).│
// │  One model for every victim; a kill with no contributor drops nothing.  │
// │  Victim │ rate field               │ amount source                      │
// │  Bot    │ coinKillPercentVsBot     │ botCoinDropAmount                  │
// │  Player │ coinKillPercentVsPlayer  │ pvpCoinDropAmount                  │
// │  Floor: amount < coinKillMinAmount → use minimum instead.              │
// │                                                                         │
// │  SINKS  (coin removal; all 0 = disabled by default)                    │
// │  • respawnCostPercent  — fraction burned on player death                │
// │  • portalFee           — flat fee per portal use                        │
// │  • craftingFeePercent  — fraction of item value burned on crafting      │
// │                                                                         │
// │  FCT AND AUDIO EVENTS                                                   │
// │  Damage and regen broadcast to the AOI; XP goes to its earner only.     │
// │  Coin changes emit no FCT — the inventory-bar quantity FX shows them.   │
// │  Item gains emit no FCT — that feedback lives in the loot grid.         │
// │  A defeat and a level gained broadcast their audio id to the AOI, so    │
// │  every viewer hears the same event.                                     │
// └─────────────────────────────────────────────────────────────────────────┘

import (
	"math"

	"cyberia-server/logx"
)

// ── Wire message helpers ──────────────────────────────────────────────────

// broadcastAt delivers one message to every player whose AOI covers the
// event point — identical feedback for every viewer, so per-recipient
// amounts can never leak.
func (s *Content) broadcastAt(mapState *MapState, msgType string, payload any, worldX, worldY float64) {
	pt := Rectangle{MinX: worldX, MinY: worldY, MaxX: worldX, MaxY: worldY}
	for player := range s.playersOn(mapState) {
		if rectsOverlap(player.AOI, pt) {
			sendMessage(player, msgType, payload)
		}
	}
}

// broadcastFCT delivers the same combat-text event to every viewer in reach.
func (s *Content) broadcastFCT(mapState *MapState, kind string, worldX, worldY float64, value int) {
	s.broadcastAt(mapState, "combat_text", CombatText{Kind: kind, WorldX: worldX, WorldY: worldY, Value: value}, worldX, worldY)
}

// broadcastAudioEvent raises one audio logic id for every viewer in reach, so
// a defeat or a level gained sounds for everyone who can see it.
func (s *Content) broadcastAudioEvent(mapState *MapState, logicEventID string, at Vec2) {
	s.broadcastAt(mapState, "audio_event", AudioEvent{LogicEventID: logicEventID, WorldX: at.X, WorldY: at.Y}, at.X, at.Y)
}

// ── Coin helpers (flat-field design) ─────────────────────────────────────────
//
// Rule: always use these helpers for coins — never manipulate the coin
// ObjectLayer slot's Quantity directly.
//
// Invariant after every helper call:
//   entity.Coins == coin ObjectLayer slot .Quantity
//   coin ObjectLayer slot .Active == false (always)

// syncCoinOL ensures the coin ObjectLayer slot reflects the current flat balance.
// The slot is created if absent.  Active is always forced to false: coins are
// a non-activable item (display-only in the inventory bar and modal).
func (s *Content) syncCoinOL(layers *[]ObjectLayerState, amount uint32) {
	ol := getOrCreateItemOL(layers, s.coinItemID)
	ol.Quantity = int(amount)
	ol.Active = false // coins are never activable — enforced here
}

// wallet returns the flat coin balance of player or bot id on ms.
func (s *Content) wallet(id EntityID, ms *MapState) *uint32 {
	if ms.IsKind(id, EntityBot) {
		return &ms.br(id).Coins
	}
	return &s.sessions[id].Coins
}

// setCoins sets the flat balance of id and synchronises the display-only coin
// ObjectLayer slot (Active: false, Quantity = amount).
func (s *Content) setCoins(id EntityID, ms *MapState, amount uint32) {
	*s.wallet(id, ms) = amount
	s.syncCoinOL(&ms.ap(id).Layers, amount)
}

// addCoins adds delta to the balance of id (clamped to [0, MaxUint32]) and
// syncs the display coin OL slot. Returns the new balance.
func (s *Content) addCoins(id EntityID, ms *MapState, delta int) uint32 {
	next := uint32(max(int(*s.wallet(id, ms))+delta, 0))
	s.setCoins(id, ms, next)
	return next
}

// ── Generic item-quantity helpers (non-coin) ──────────────────────────────
//
// These operate exclusively on ObjectLayerState.Quantity for non-coin items.
// Do NOT pass s.coinItemID to these functions; use the coin helpers above.

// getOrCreateItemOL returns a pointer to the ObjectLayerState for itemID,
// appending an inactive slot (Quantity=0) if none exists.
func getOrCreateItemOL(layers *[]ObjectLayerState, itemID string) *ObjectLayerState {
	for i := range *layers {
		if (*layers)[i].ItemID == itemID {
			return &(*layers)[i]
		}
	}
	*layers = append(*layers, ObjectLayerState{ItemID: itemID, Active: false, Quantity: 0})
	return &(*layers)[len(*layers)-1]
}

// layerQuantity is how many units of itemID the stacks of layers hold.
func layerQuantity(layers []ObjectLayerState, itemID string) int {
	total := 0
	for _, ol := range layers {
		if ol.ItemID == itemID {
			total += ol.Quantity
		}
	}
	return total
}

// removeLayerItem drains qty of itemID from the stacks of layers. An emptied stack leaves them, so
// it leaves the inventory bar.
func removeLayerItem(layers *[]ObjectLayerState, itemID string, qty int) {
	kept := (*layers)[:0]
	for _, ol := range *layers {
		if ol.ItemID == itemID && qty > 0 {
			take := min(ol.Quantity, qty)
			ol.Quantity -= take
			qty -= take
			if ol.Quantity <= 0 {
				continue
			}
		}
		kept = append(kept, ol)
	}
	*layers = kept
}

// addPlayerItem credits qty of itemID to the player's inventory — the single
// grant path for quest rewards, shop purchases, and any future item fountain.
// Coins route through the flat balance so the display slot stays in sync;
// everything else lands on its ObjectLayer stack, inactive. The counterpart of
// removePlayerItem (quest.go).
func (s *Content) addPlayerItem(player *Session, itemID string, qty int) {
	if itemID == "" || qty <= 0 {
		return
	}
	if itemID == s.coinItemID {
		s.addCoins(player.ID, s.maps[player.MapCode], qty)
	} else {
		getOrCreateItemOL(&s.ap(player.ID, player.MapCode).Layers, itemID).Quantity += qty
	}
	// Either branch can append a slot, so the cached stat block is stale.
}

// ── Fountain helpers ──────────────────────────────────────────────────────

// FountainInitPlayer credits the player's starting wallet on first connect.
// It always runs, so the seeded coin slot mirrors the balance even at zero.
func (s *Content) FountainInitPlayer(player *Session) {
	s.setCoins(player.ID, s.maps[player.MapCode], uint32(max(s.playerSpawnCoins, 0)))
}

// FountainInitBot credits a bot's coin pool at spawn / respawn time.
// Implements the botSpawnCoins fountain (infinite mint bounded by kill rate).
func (s *Content) FountainInitBot(id EntityID, ms *MapState) {
	s.setCoins(id, ms, uint32(max(s.botSpawnCoins, 0)))
}

// ── Kill loot amounts ─────────────────────────────────────────────────────

// pvpCoinDropAmount computes how many coins a dead player scatters as a grid
// drop, mirroring botCoinDropAmount at the vs-player rate: the victim's
// balance, floored, with the configured minimum, capped at the balance.
// Players carry only what they hold (no mint). Caller holds s.mu.
func (s *Content) pvpCoinDropAmount(player *Session) int {
	effectiveBalance := int(player.Coins)
	if effectiveBalance <= 0 {
		return 0
	}
	amount := int(math.Floor(float64(effectiveBalance) * s.coinKillPercentVsPlayer))
	if s.coinKillMinAmount > 0 && amount < s.coinKillMinAmount {
		amount = s.coinKillMinAmount
	}
	if amount > effectiveBalance {
		amount = effectiveBalance
	}
	return amount
}

// botCoinDropAmount computes how many coins a dead bot scatters, mirroring the
// PvE kill-transfer formula: the bot's balance (minted up to botSpawnCoins) at
// the vs-bot rate, floored, with the configured minimum. Caller holds s.mu.
func (s *Content) botCoinDropAmount(id EntityID, ms *MapState) int {
	effectiveBalance := int(ms.br(id).Coins)
	if effectiveBalance < s.botSpawnCoins {
		effectiveBalance = s.botSpawnCoins
	}
	if effectiveBalance <= 0 {
		return 0
	}
	amount := int(math.Floor(float64(effectiveBalance) * s.coinKillPercentVsBot))
	if s.coinKillMinAmount > 0 && amount < s.coinKillMinAmount {
		amount = s.coinKillMinAmount
	}
	if amount > effectiveBalance {
		amount = effectiveBalance
	}
	return amount
}

// ── Sink helpers ──────────────────────────────────────────────────────────

// SinkRespawnCost burns a percentage of the dead player's coins on respawn.
// Alpha default: respawnCostPercent = 0.0 (disabled).
func (s *Content) SinkRespawnCost(player *Session) {
	if s.respawnCostPercent <= 0 {
		return
	}
	balance := int(player.Coins)
	burn := int(math.Floor(float64(balance) * s.respawnCostPercent))
	if burn <= 0 {
		return
	}
	// No coin FCT — balance changes surface in the inventory-bar quantity FX.
	s.addCoins(player.ID, s.maps[player.MapCode], -burn)
	logx.Debugf("[ECONOMY] Respawn sink: %s burned %d coins (%.0f%%)",
		player.ID, burn, s.respawnCostPercent*100)
}
