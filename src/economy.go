package game

// economy.go — Fountain & Sink Economy Module
//
// Implements the off-chain coin economy for Cyberia following the
// industry-standard Fountain & Sink model used in Ultima Online,
// EverQuest, World of Warcraft, EVE Online, etc.
//
// ┌─────────────────────────────────────────────────────────────────────┐
// │                   Economy Architecture                               │
// │                                                                      │
// │  FOUNTAINS  (coin injection — new supply enters the economy)         │
// │  ──────────────────────────────────────────────────────────          │
// │  • botSpawnCoins    — each bot spawns with N coins; reset to N       │
// │                       on every respawn → infinite mint bounded by    │
// │                       how fast players can kill bots.                │
// │  • playerSpawnCoins — starting wallet for every guest session.       │
// │                       Lost on page refresh (no persistence alpha).   │
// │                                                                      │
// │  KILL TRANSFER  (redistribution — zero-sum between wallets)          │
// │  ──────────────────────────────────────────────────────────          │
// │  Kill scenario    │ rate field              │ purpose                 │
// │  ─────────────────┼─────────────────────────┼──────────────────────  │
// │  Player → Bot     │ coinKillPercentVsBot    │ reward farming         │
// │  Player → Player  │ coinKillPercentVsPlayer │ reward PvP (gentler)   │
// │  Bot    → Bot     │ coinKillPercentVsBot    │ bots loot each other   │
// │  Bot    → Player  │ coinKillPercentVsPlayer │ risk / punishment      │
// │                                                                      │
// │  Floor: if transfer < coinKillMinAmount → use minimum instead.       │
// │  Only triggers when victim has at least 1 coin; bots are guaranteed  │
// │  to yield ≥ botSpawnCoins worth of coins via the infinite mint.      │
// │                                                                      │
// │  SINKS  (coin removal — alpha stubs, all default to 0 = disabled)   │
// │  ─────────────────────────────────────────────────────────────       │
// │  • respawnCostPercent — fraction burned on player death              │
// │  • portalFee          — flat fee per portal use                      │
// │  • craftingFeePercent — fraction of item value burned on crafting    │
// │                                                                      │
// │  FLOATING COMBAT TEXT  (client-side visual feedback)                 │
// │  ──────────────────────────────────────────────────                  │
// │  Every economy event fires a binary MsgTypeFCT message (14 bytes)   │
// │  to the affected player's WebSocket channel.  The C client renders   │
// │  animated "+N" / "−N" numbers via floating_combat_text.c.           │
// │  FCTTypeDamage / FCTTypeRegen are reserved for future combat events. │
// │                                                                      │
// │  ON-CHAIN BRIDGE  (future)                                           │
// │  ─────────────────────────                                           │
// │  Off-chain coins map 1-to-1 to CKY (ERC-1155 token ID 0) on the     │
// │  Hyperledger Besu deployment.  The in-memory wallet is a hot cache   │
// │  that will be persisted and synced to the chain once players         │
// │  authenticate with secp256k1 keys (EIP-712 signed claims).          │
// │  See hardhat/WHITE-PAPER.md §7 "Tokenomics".                        │
// └─────────────────────────────────────────────────────────────────────┘

import (
	"encoding/binary"
	"log"
	"math"
)

// buildFCTMsg encodes a 14-byte Floating Combat Text message.
//
// Wire format (little-endian):
//
//	[0]      u8   MsgTypeFCT (0x04)
//	[1]      u8   fctType — one of FCTType* constants in aoi_binary.go
//	[2..5]   f32  worldX  — position of the affected entity
//	[6..9]   f32  worldY
//	[10..13] u32  value   — always positive; sign implied by fctType
func buildFCTMsg(fctType byte, worldX, worldY float64, value int) []byte {
	buf := make([]byte, 14)
	buf[0] = MsgTypeFCT
	buf[1] = fctType
	binary.LittleEndian.PutUint32(buf[2:], math.Float32bits(float32(worldX)))
	binary.LittleEndian.PutUint32(buf[6:], math.Float32bits(float32(worldY)))
	binary.LittleEndian.PutUint32(buf[10:], uint32(value))
	return buf
}

// sendFCT delivers a Floating Combat Text message to a player's WebSocket channel.
// Non-blocking: if the channel is full the event is silently dropped.
// FCT is purely cosmetic — dropping a frame causes no game-state inconsistency.
func sendFCT(player *PlayerState, fctType byte, worldX, worldY float64, value int) {
	if player == nil || player.Client == nil {
		return
	}
	msg := buildFCTMsg(fctType, worldX, worldY, value)
	select {
	case player.Client.send <- msg:
	default:
		// Channel full — cosmetic event dropped, never block game loop.
	}
}

// ── Fountain helpers ──────────────────────────────────────────────────────

// FountainInitPlayer credits the player's starting wallet on first connect.
// Implements the playerSpawnCoins fountain — the entry point into the
// economy for every guest session.  Coins are lost on disconnect (no
// persistence in alpha).
func (s *GameServer) FountainInitPlayer(player *PlayerState) {
	if s.coinItemID == "" || s.playerSpawnCoins <= 0 {
		return
	}
	layer := s.findOrCreateCoinLayer(player)
	if layer == nil {
		return
	}
	layer.Quantity = s.playerSpawnCoins
	s.InvalidateStats(player)
}

// FountainInitBot credits a bot's coin pool at spawn / respawn time.
// Implements the botSpawnCoins fountain — bots carry coins as loot that
// players claim on kill.  The pool resets on every respawn so bots act as
// a perpetual, renewable coin source (infinite mint bounded by player
// kill activity).
func (s *GameServer) FountainInitBot(bot *BotState) {
	if s.coinItemID == "" || s.botSpawnCoins <= 0 {
		return
	}
	var coinLayer *ObjectLayerState
	for i := range bot.ObjectLayers {
		if bot.ObjectLayers[i].ItemID == s.coinItemID {
			coinLayer = &bot.ObjectLayers[i]
			break
		}
	}
	if coinLayer == nil {
		bot.ObjectLayers = append(bot.ObjectLayers,
			ObjectLayerState{ItemID: s.coinItemID, Active: false, Quantity: 0})
		coinLayer = &bot.ObjectLayers[len(bot.ObjectLayers)-1]
	}
	coinLayer.Quantity = s.botSpawnCoins
}

// ── Kill Transfer ─────────────────────────────────────────────────────────

// ExecuteKillTransfer applies the kill-economy rules when any entity is killed.
//
// Handles all four kill scenarios:
//   - Player kills Bot:    PvE rate (coinKillPercentVsBot)
//   - Player kills Player: PvP rate (coinKillPercentVsPlayer, gentler)
//   - Bot kills Bot:       PvE rate (bot loots bot; fountain continues)
//   - Bot kills Player:    PvP rate (player loses coins to the bot)
//
// Transfer is only executed when the effective victim balance > 0.
// Bots receive an infinite-mint guarantee: even if their layer shows < botSpawnCoins,
// effectiveBalance is floored to botSpawnCoins so the kill is always rewarding.
//
// The transfer amount is: max( floor(effectiveBalance * rate), coinKillMinAmount )
// clamped to the victim's actual layer balance so no wallet goes negative for players.
//
// Both caster and victim receive a FCTTypeCoinGain / FCTTypeCoinLoss event
// if they are human players.
func (s *GameServer) ExecuteKillTransfer(caster interface{}, victim interface{}) {
	if s.coinItemID == "" {
		return
	}

	// ── 1. Resolve victim ─────────────────────────────────────────────
	var victimCoinPtr *ObjectLayerState
	var victimID      string
	var victimPos     Point
	var victimIsBot   bool
	var victimPlayer  *PlayerState

	switch v := victim.(type) {
	case *PlayerState:
		victimID     = v.ID
		victimPos    = v.Pos
		victimPlayer = v
		for i := range v.ObjectLayers {
			if v.ObjectLayers[i].ItemID == s.coinItemID {
				victimCoinPtr = &v.ObjectLayers[i]
				break
			}
		}
	case *BotState:
		victimID    = v.ID
		victimPos   = v.Pos
		victimIsBot = true
		for i := range v.ObjectLayers {
			if v.ObjectLayers[i].ItemID == s.coinItemID {
				victimCoinPtr = &v.ObjectLayers[i]
				break
			}
		}
	default:
		log.Printf("[ECONOMY] ExecuteKillTransfer: unknown victim type %T", victim)
		return
	}

	// ── 2. Effective balance ───────────────────────────────────────────
	// Players: only what they actually carry (no mint).
	// Bots:    guaranteed ≥ botSpawnCoins regardless of layer value
	//          (implements the infinite-mint fountain).
	effectiveBalance := 0
	if victimCoinPtr != nil {
		effectiveBalance = victimCoinPtr.Quantity
	}
	if victimIsBot && effectiveBalance < s.botSpawnCoins {
		effectiveBalance = s.botSpawnCoins
	}
	if effectiveBalance <= 0 {
		return // Victim has no coins — nothing to transfer.
	}

	// ── 3. Select rate ────────────────────────────────────────────────
	// victimIsBot=true  → PvE rate (also covers Bot→Bot: bot looting another bot)
	// victimIsBot=false → PvP rate (Player kills Player OR Bot kills Player)
	rate := s.coinKillPercentVsBot
	if !victimIsBot {
		rate = s.coinKillPercentVsPlayer
	}

	// ── 4. Compute transfer ───────────────────────────────────────────
	transfer := int(math.Floor(float64(effectiveBalance) * rate))
	if s.coinKillMinAmount > 0 && transfer < s.coinKillMinAmount {
		transfer = s.coinKillMinAmount
	}
	// Never take more than the victim's actual layer balance.
	actualVictimBalance := effectiveBalance
	if victimCoinPtr != nil && !victimIsBot {
		actualVictimBalance = victimCoinPtr.Quantity
	}
	if transfer > actualVictimBalance {
		transfer = actualVictimBalance
	}
	if transfer <= 0 {
		return
	}

	// ── 5. Deduct from victim ─────────────────────────────────────────
	if victimCoinPtr != nil {
		victimCoinPtr.Quantity -= transfer
		if victimCoinPtr.Quantity < 0 {
			victimCoinPtr.Quantity = 0
		}
		s.InvalidateStats(victim)
	}
	// (For bots the deficit doesn't matter — FountainInitBot resets on respawn.)

	// ── 6. Credit caster ──────────────────────────────────────────────
	casterCoinPtr := s.findOrCreateCoinLayer(caster)
	if casterCoinPtr == nil {
		log.Printf("[ECONOMY] ExecuteKillTransfer: could not resolve caster coin layer")
		return
	}
	casterCoinPtr.Quantity += transfer
	s.InvalidateStats(caster)

	var casterPlayer *PlayerState
	if cp, ok := caster.(*PlayerState); ok {
		casterPlayer = cp
	}

	log.Printf("[ECONOMY] Kill transfer: %s looted %d coins from %s "+
		"(effectiveBal=%d, rate=%.0f%%, min=%d, victimIsBot=%v)",
		econEntityID(caster), transfer, victimID,
		effectiveBalance, rate*100, s.coinKillMinAmount, victimIsBot)

	// ── 7. FCT events (cosmetic, non-blocking) ─────────────────────────
	// Gain: tell the caster how many coins they just received.
	if casterPlayer != nil {
		sendFCT(casterPlayer, FCTTypeCoinGain, victimPos.X, victimPos.Y, transfer)
	}
	// Loss: tell a human victim how many coins were taken.
	if victimPlayer != nil {
		sendFCT(victimPlayer, FCTTypeCoinLoss, victimPos.X, victimPos.Y, transfer)
	}
}

// ── Sink helpers ──────────────────────────────────────────────────────────

// SinkRespawnCost burns a percentage of the dead player's coins on respawn.
// Alpha default: respawnCostPercent = 0.0 (disabled).
// Design intent: risk penalty that discourages consequence-free farming.
func (s *GameServer) SinkRespawnCost(player *PlayerState) {
	if s.respawnCostPercent <= 0 || s.coinItemID == "" {
		return
	}
	for i := range player.ObjectLayers {
		if player.ObjectLayers[i].ItemID != s.coinItemID {
			continue
		}
		burn := int(math.Floor(float64(player.ObjectLayers[i].Quantity) * s.respawnCostPercent))
		if burn <= 0 {
			return
		}
		player.ObjectLayers[i].Quantity -= burn
		if player.ObjectLayers[i].Quantity < 0 {
			player.ObjectLayers[i].Quantity = 0
		}
		s.InvalidateStats(player)
		sendFCT(player, FCTTypeCoinLoss, player.Pos.X, player.Pos.Y, burn)
		log.Printf("[ECONOMY] Respawn sink: %s burned %d coins (%.0f%%)",
			player.ID, burn, s.respawnCostPercent*100)
		return
	}
}

// SinkPortalFee burns a flat fee when a player uses a portal.
// Alpha default: portalFee = 0 (disabled).
// Design intent: travel tax that drains accumulated hoards over time.
func (s *GameServer) SinkPortalFee(player *PlayerState) {
	if s.portalFee <= 0 || s.coinItemID == "" {
		return
	}
	for i := range player.ObjectLayers {
		if player.ObjectLayers[i].ItemID != s.coinItemID {
			continue
		}
		burn := s.portalFee
		if burn > player.ObjectLayers[i].Quantity {
			burn = player.ObjectLayers[i].Quantity
		}
		if burn <= 0 {
			return
		}
		player.ObjectLayers[i].Quantity -= burn
		s.InvalidateStats(player)
		sendFCT(player, FCTTypeCoinLoss, player.Pos.X, player.Pos.Y, burn)
		log.Printf("[ECONOMY] Portal sink: %s burned %d coins (flat fee)", player.ID, burn)
		return
	}
}

// ── Internal helpers ──────────────────────────────────────────────────────

// findOrCreateCoinLayer returns a pointer to the coin ObjectLayerState for
// the given entity, appending a zero-quantity entry if none exists.
func (s *GameServer) findOrCreateCoinLayer(entity interface{}) *ObjectLayerState {
	switch e := entity.(type) {
	case *PlayerState:
		for i := range e.ObjectLayers {
			if e.ObjectLayers[i].ItemID == s.coinItemID {
				return &e.ObjectLayers[i]
			}
		}
		e.ObjectLayers = append(e.ObjectLayers,
			ObjectLayerState{ItemID: s.coinItemID, Active: false, Quantity: 0})
		return &e.ObjectLayers[len(e.ObjectLayers)-1]
	case *BotState:
		for i := range e.ObjectLayers {
			if e.ObjectLayers[i].ItemID == s.coinItemID {
				return &e.ObjectLayers[i]
			}
		}
		e.ObjectLayers = append(e.ObjectLayers,
			ObjectLayerState{ItemID: s.coinItemID, Active: false, Quantity: 0})
		return &e.ObjectLayers[len(e.ObjectLayers)-1]
	}
	return nil
}

// econEntityID extracts a loggable ID string from any entity interface.
func econEntityID(entity interface{}) string {
	switch e := entity.(type) {
	case *PlayerState:
		return "player:" + e.ID
	case *BotState:
		return "bot:" + e.ID
	}
	return "unknown"
}
