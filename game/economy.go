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
// │  KILL TRANSFER  (zero-sum redistribution between wallets)               │
// │  Kill scenario    │ rate field               │ purpose                  │
// │  Player → Bot     │ coinKillPercentVsBot     │ reward farming           │
// │  Player → Player  │ coinKillPercentVsPlayer  │ reward PvP (gentler)    │
// │  Bot    → Bot     │ coinKillPercentVsBot     │ bots loot each other    │
// │  Bot    → Player  │ coinKillPercentVsPlayer  │ risk / punishment       │
// │  Floor: transfer < coinKillMinAmount → use minimum instead.            │
// │                                                                         │
// │  SINKS  (coin removal; all 0 = disabled by default)                    │
// │  • respawnCostPercent  — fraction burned on player death                │
// │  • portalFee           — flat fee per portal use                        │
// │  • craftingFeePercent  — fraction of item value burned on crafting      │
// │                                                                         │
// │  FCT EVENTS                                                             │
// │  Coin ops → MsgTypeFCT (14-byte, fixed) via sendFCT().                 │
// │  Item qty  → MsgTypeItemFCT (variable) via sendItemFCT().              │
// └─────────────────────────────────────────────────────────────────────────┘

import (
"encoding/binary"
"log"
"math"
)

// ── Wire message helpers ──────────────────────────────────────────────────

// buildFCTMsg encodes a 14-byte Floating Combat Text message.
//
// Wire format (little-endian):
//
//[0]      u8   MsgTypeFCT (0x04)
//[1]      u8   fctType
//[2..5]   f32  worldX
//[6..9]   f32  worldY
//[10..13] u32  value (always positive; sign implied by fctType)
func buildFCTMsg(fctType byte, worldX, worldY float64, value int) []byte {
buf := make([]byte, 14)
buf[0] = MsgTypeFCT
buf[1] = fctType
binary.LittleEndian.PutUint32(buf[2:], math.Float32bits(float32(worldX)))
binary.LittleEndian.PutUint32(buf[6:], math.Float32bits(float32(worldY)))
binary.LittleEndian.PutUint32(buf[10:], uint32(value))
return buf
}

// sendFCT delivers a Floating Combat Text message to a player's WebSocket.
// Non-blocking: drops silently when the channel is full (cosmetic only).
func sendFCT(player *PlayerState, fctType byte, worldX, worldY float64, value int) {
if player == nil || player.Client == nil {
return
}
msg := buildFCTMsg(fctType, worldX, worldY, value)
select {
case player.Client.send <- msg:
default:
}
}

// buildItemFCTMsg encodes a variable-length Item FCT message.
//
// Wire format (little-endian):
//
//[0]      u8   MsgTypeItemFCT (0x05)
//[1]      u8   fctType — FCTTypeItemGain or FCTTypeItemLoss
//[2..5]   f32  worldX
//[6..9]   f32  worldY
//[10..13] u32  quantity (always positive)
//[14]     u8   itemId length (0–63)
//[15..]   str  itemId bytes
func buildItemFCTMsg(fctType byte, worldX, worldY float64, quantity int, itemID string) []byte {
idLen := len(itemID)
if idLen > 63 {
idLen = 63
}
buf := make([]byte, 15+idLen)
buf[0] = MsgTypeItemFCT
buf[1] = fctType
binary.LittleEndian.PutUint32(buf[2:], math.Float32bits(float32(worldX)))
binary.LittleEndian.PutUint32(buf[6:], math.Float32bits(float32(worldY)))
binary.LittleEndian.PutUint32(buf[10:], uint32(quantity))
buf[14] = byte(idLen)
copy(buf[15:], itemID[:idLen])
return buf
}

// sendItemFCT delivers an Item FCT message to a player's WebSocket channel.
// Use for non-coin item quantity changes (wood, stone, potions, etc.).
func sendItemFCT(player *PlayerState, fctType byte, worldX, worldY float64, quantity int, itemID string) {
if player == nil || player.Client == nil || quantity <= 0 {
return
}
msg := buildItemFCTMsg(fctType, worldX, worldY, quantity, itemID)
select {
case player.Client.send <- msg:
default:
}
}

// ── Coin helpers (flat-field design) ─────────────────────────────────────────
//
// Rule: always use these helpers for coins.  Never call addItemQty/setItemQty
// with s.coinItemID — those functions are for non-coin items only.
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

// getItemQty returns the Quantity for itemID in layers, or 0 if absent.
func getItemQty(layers []ObjectLayerState, itemID string) int {
for i := range layers {
if layers[i].ItemID == itemID {
return layers[i].Quantity
}
}
return 0
}

// addItemQty adds delta to the ObjectLayer Quantity for itemID (non-coin items).
// Creates the OL slot if absent.  Clamps to ≥ 0.  Returns new quantity.
func (s *GameServer) addItemQty(entity interface{}, itemID string, delta int) int {
switch e := entity.(type) {
case *PlayerState:
ol := getOrCreateItemOL(&e.ObjectLayers, itemID)
ol.Quantity += delta
if ol.Quantity < 0 {
ol.Quantity = 0
}
return ol.Quantity
case *BotState:
ol := getOrCreateItemOL(&e.ObjectLayers, itemID)
ol.Quantity += delta
if ol.Quantity < 0 {
ol.Quantity = 0
}
return ol.Quantity
}
return 0
}

// setItemQty sets ObjectLayer Quantity for itemID exactly (non-coin items).
// Creates the OL slot if absent.  Clamps to ≥ 0.
func (s *GameServer) setItemQty(entity interface{}, itemID string, amount int) {
if amount < 0 {
amount = 0
}
switch e := entity.(type) {
case *PlayerState:
ol := getOrCreateItemOL(&e.ObjectLayers, itemID)
ol.Quantity = amount
case *BotState:
ol := getOrCreateItemOL(&e.ObjectLayers, itemID)
ol.Quantity = amount
}
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

// ── Kill Transfer ─────────────────────────────────────────────────────────

// ExecuteKillTransfer applies kill-economy rules when any entity dies.
//
// All four scenarios are handled:
//
//Player → Bot     coinKillPercentVsBot    (PvE reward)
//Player → Player  coinKillPercentVsPlayer (PvP reward)
//Bot    → Bot     coinKillPercentVsBot    (bot loot chain)
//Bot    → Player  coinKillPercentVsPlayer (risk / punishment)
//
// Balances are read via entity.Coins (flat field, O(1)).
// Bots receive an infinite-mint guarantee: effectiveBalance ≥ botSpawnCoins.
func (s *GameServer) ExecuteKillTransfer(caster interface{}, victim interface{}) {
// ── 1. Resolve victim ──────────────────────────────────────────────
var victimID     string
var victimPos    Point
var victimIsBot  bool
var victimPlayer *PlayerState

switch v := victim.(type) {
case *PlayerState:
victimID     = v.ID
victimPos    = v.Pos
victimPlayer = v
case *BotState:
victimID    = v.ID
victimPos   = v.Pos
victimIsBot = true
default:
log.Printf("[ECONOMY] ExecuteKillTransfer: unknown victim type %T", victim)
return
}

// ── 2. Effective balance ───────────────────────────────────────────
// Players: only what they actually carry (no mint).
// Bots:    guaranteed ≥ botSpawnCoins regardless of current balance.
effectiveBalance := int(coinQuantity(victim))
if victimIsBot && effectiveBalance < s.botSpawnCoins {
effectiveBalance = s.botSpawnCoins
}
if effectiveBalance <= 0 {
return // victim has no coins — nothing to transfer
}

// ── 3. Select rate ─────────────────────────────────────────────────
rate := s.coinKillPercentVsBot
if !victimIsBot {
rate = s.coinKillPercentVsPlayer
}

// ── 4. Compute transfer ────────────────────────────────────────────
transfer := int(math.Floor(float64(effectiveBalance) * rate))
if s.coinKillMinAmount > 0 && transfer < s.coinKillMinAmount {
transfer = s.coinKillMinAmount
}
// Cap transfer at what the victim actually holds (bots may have 0 on early death).
actualVictimBalance := effectiveBalance
if !victimIsBot {
actualVictimBalance = int(coinQuantity(victim))
}
if transfer > actualVictimBalance {
transfer = actualVictimBalance
}
if transfer <= 0 {
return
}

// ── 5. Deduct from victim ──────────────────────────────────────────
s.addCoins(victim, -transfer)
s.InvalidateStats(victim)

// ── 6. Credit caster ──────────────────────────────────────────────
s.addCoins(caster, transfer)
s.InvalidateStats(caster)

var casterPlayer *PlayerState
if cp, ok := caster.(*PlayerState); ok {
casterPlayer = cp
}

log.Printf("[ECONOMY] Kill transfer: %s looted %d coins from %s "+
"(effectiveBal=%d, rate=%.0f%%, min=%d, victimIsBot=%v)",
econEntityID(caster), transfer, victimID,
effectiveBalance, rate*100, s.coinKillMinAmount, victimIsBot)

// ── 7. FCT events (cosmetic, non-blocking) ──────────────────────────
if casterPlayer != nil {
sendFCT(casterPlayer, FCTTypeCoinGain, victimPos.X, victimPos.Y, transfer)
}
if victimPlayer != nil {
sendFCT(victimPlayer, FCTTypeCoinLoss, victimPos.X, victimPos.Y, transfer)
}
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
s.addCoins(player, -burn)
s.InvalidateStats(player)
sendFCT(player, FCTTypeCoinLoss, player.Pos.X, player.Pos.Y, burn)
log.Printf("[ECONOMY] Respawn sink: %s burned %d coins (%.0f%%)",
player.ID, burn, s.respawnCostPercent*100)
}

// SinkPortalFee burns a flat fee when a player uses a portal.
// Alpha default: portalFee = 0 (disabled).
func (s *GameServer) SinkPortalFee(player *PlayerState) {
if s.portalFee <= 0 {
return
}
balance := int(coinQuantity(player))
burn := s.portalFee
if burn > balance {
burn = balance
}
if burn <= 0 {
return
}
s.addCoins(player, -burn)
s.InvalidateStats(player)
sendFCT(player, FCTTypeCoinLoss, player.Pos.X, player.Pos.Y, burn)
log.Printf("[ECONOMY] Portal sink: %s burned %d coins (flat fee)", player.ID, burn)
}

// ── Internal helpers ──────────────────────────────────────────────────────

// econEntityID extracts a loggable ID from any entity interface.
func econEntityID(entity interface{}) string {
switch e := entity.(type) {
case *PlayerState:
return "player:" + e.ID
case *BotState:
return "bot:" + e.ID
}
return "unknown"
}
