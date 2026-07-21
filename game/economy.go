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
// │  FCT EVENTS                                                             │
// │  Combat only (damage/regen, broadcast to AOI). Coin changes emit no     │
// │  FCT — the inventory-bar quantity FX shows them. Gains no               │
// │  longer emit FCT — that feedback lives in the loot grid.                 │
// └─────────────────────────────────────────────────────────────────────────┘

import (
	"encoding/binary"
	"math"

	"cyberia-server/logx"
)

// ── Wire message helpers ──────────────────────────────────────────────────

// buildFCTMsg encodes a 14-byte Floating Combat Text message.
//
// Wire format (little-endian):
//
// [0]      u8   MsgTypeFCT (0x04)
// [1]      u8   fctType
// [2..5]   f32  worldX
// [6..9]   f32  worldY
// [10..13] u32  value (always positive; sign implied by fctType)
func buildFCTMsg(fctType byte, worldX, worldY float64, value int) []byte {
	buf := make([]byte, 14)
	buf[0] = MsgTypeFCT
	buf[1] = fctType
	binary.LittleEndian.PutUint32(buf[2:], math.Float32bits(float32(worldX)))
	binary.LittleEndian.PutUint32(buf[6:], math.Float32bits(float32(worldY)))
	binary.LittleEndian.PutUint32(buf[10:], uint32(value))
	return buf
}

// broadcastFCT delivers the same FCT event to every player whose AOI covers
// the event point — identical feedback for every viewer, so per-recipient
// amounts can never leak. Non-blocking (cosmetic).
func broadcastFCT(mapState *MapState, fctType byte, worldX, worldY float64, value int) {
	msg := buildFCTMsg(fctType, worldX, worldY, value)
	pt := Rectangle{MinX: worldX, MinY: worldY, MaxX: worldX, MaxY: worldY}
	for _, player := range mapState.players {
		if player.Client == nil || !rectsOverlap(player.AOI, pt) {
			continue
		}
		select {
		case player.Client.send <- msg:
		default:
		}
	}
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
func (s *GameServer) syncCoinOL(layers *[]ObjectLayerState, amount uint32) {
	ol := getOrCreateItemOL(layers, s.coinItemID)
	ol.Quantity = int(amount)
	ol.Active = false // coins are never activable — enforced here
}

// coinQuantity returns the entity's flat coin balance.  O(1).
// entity must be *PlayerState or *BotState.
func coinQuantity(entity interface{}) uint32 {
	switch e := entity.(type) {
	case *PlayerState:
		return e.Coins
	case *BotState:
		return e.Coins
	}
	return 0
}

// setCoinQuantity sets the flat Coins field and synchronises the display-only
// coin ObjectLayer slot (Active: false, Quantity = amount).
// entity must be *PlayerState or *BotState.
func (s *GameServer) setCoinQuantity(entity interface{}, amount uint32) {
	switch e := entity.(type) {
	case *PlayerState:
		e.Coins = amount
		s.syncCoinOL(&e.ObjectLayers, amount)
	case *BotState:
		e.Coins = amount
		s.syncCoinOL(&e.ObjectLayers, amount)
	}
}

// addCoins adds delta to entity.Coins (clamped to [0, MaxUint32]) and syncs
// the display coin OL slot.  Returns the new balance.
func (s *GameServer) addCoins(entity interface{}, delta int) uint32 {
	cur := int(coinQuantity(entity))
	cur += delta
	if cur < 0 {
		cur = 0
	}
	next := uint32(cur)
	s.setCoinQuantity(entity, next)
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

// ── Fountain helpers ──────────────────────────────────────────────────────

// FountainInitPlayer credits the player's starting wallet on first connect.
// Sets entity.Coins and syncs the display coin OL slot.
func (s *GameServer) FountainInitPlayer(player *PlayerState) {
	if s.playerSpawnCoins <= 0 {
		return
	}
	s.setCoinQuantity(player, uint32(s.playerSpawnCoins))
	s.InvalidateStats(player)
}

// FountainInitBot credits a bot's coin pool at spawn / respawn time.
// Implements the botSpawnCoins fountain (infinite mint bounded by kill rate).
func (s *GameServer) FountainInitBot(bot *BotState) {
	if s.botSpawnCoins <= 0 {
		return
	}
	s.setCoinQuantity(bot, uint32(s.botSpawnCoins))
}

// ── Kill loot amounts ─────────────────────────────────────────────────────

// pvpCoinDropAmount computes how many coins a dead player scatters as a grid
// drop, mirroring botCoinDropAmount at the vs-player rate: the victim's
// balance, floored, with the configured minimum, capped at the balance.
// Players carry only what they hold (no mint). Caller holds s.mu.
func (s *GameServer) pvpCoinDropAmount(player *PlayerState) int {
	effectiveBalance := int(coinQuantity(player))
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
func (s *GameServer) botCoinDropAmount(bot *BotState) int {
	effectiveBalance := int(coinQuantity(bot))
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
func (s *GameServer) SinkRespawnCost(player *PlayerState) {
	if s.respawnCostPercent <= 0 {
		return
	}
	balance := int(coinQuantity(player))
	burn := int(math.Floor(float64(balance) * s.respawnCostPercent))
	if burn <= 0 {
		return
	}
	// No coin FCT — balance changes surface in the inventory-bar quantity FX.
	s.addCoins(player, -burn)
	s.InvalidateStats(player)
	logx.Debugf("[ECONOMY] Respawn sink: %s burned %d coins (%.0f%%)",
		player.ID, burn, s.respawnCostPercent*100)
}
