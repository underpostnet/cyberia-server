package content

// economy.go — the coin model of an instance.
//
// Session.Coins of a player and Brain.Coins of a bot hold every balance. The
// coin ObjectLayer stack only shows it: SyncCoinOL keeps it inactive and at the
// balance.
//
// Where coins come from and go:
//   - A new session starts with PlayerSpawnCoins. The wallet ends with the session.
//   - A bot holds BotSpawnCoins at each spawn. Its death drop is new coins: the
//     wallet stays with the bot until its respawn resets it.
//   - A dead player loses its drop to a loot token. A death without damage
//     contributors drops nothing.
//   - A contributor, player or bot, that collects a coin token gains its coins.
//     An uncollected token expires with them.
//   - A price or a cost in coins leaves the economy. A reward in coins enters it.
//     A vault moves coins between the wallet and the vault.
//   - RespawnCostPercent burns a share of the coins of a dead player. 0 disables it.
//
// Damage and regen text reaches every viewer; XP text reaches its earner only.
// A coin or item change shows no text: the inventory bar and the loot grid show it.

import (
	"fmt"
	"math"
	"slices"

	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
	"cyberia-server/logx"
)

// EconomyRules is the coin model of an instance. A rate is a fraction of 1 of
// the coins of the victim, by the type of the victim.
type EconomyRules struct {
	BotSpawnCoins, PlayerSpawnCoins, CoinKillMinAmount                int
	CoinKillPercentVsBot, CoinKillPercentVsPlayer, RespawnCostPercent float64
}

func EconomyRulesFromProto(p *pb.EconomyRules) EconomyRules {
	return EconomyRules{
		BotSpawnCoins: int(p.GetBotSpawnCoins()), PlayerSpawnCoins: int(p.GetPlayerSpawnCoins()),
		CoinKillMinAmount: int(p.GetCoinKillMinAmount()), CoinKillPercentVsBot: p.GetCoinKillPercentVsBot(),
		CoinKillPercentVsPlayer: p.GetCoinKillPercentVsPlayer(), RespawnCostPercent: p.GetRespawnCostPercent(),
	}
}

// Validate refuses a rule outside the bounds of the engine contract.
func (r EconomyRules) Validate() error {
	if !r.validBounds() {
		return fmt.Errorf("invalid economy rules")
	}
	return nil
}

// killDrop is what a death drops of balance at rate: at least the minimum, and
// never more than the balance.
func (r EconomyRules) killDrop(balance int, rate float64) int {
	if balance <= 0 {
		return 0
	}
	return min(balance, max(int(math.Floor(float64(balance)*rate)), r.CoinKillMinAmount))
}

// ── Wire message helpers ──────────────────────────────────────────────────

// broadcastAt delivers one message to every player whose AOI covers the
// event point — identical feedback for every viewer, so per-recipient
// amounts can never leak.
func (s *Content) broadcastAt(mapState *world.MapState, msgType string, payload any, worldX, worldY float64) {
	pt := world.Rectangle{MinX: worldX, MinY: worldY, MaxX: worldX, MaxY: worldY}
	for player := range s.PlayersOn(mapState) {
		if world.RectsOverlap(player.AOI, pt) {
			world.SendMessage(player, msgType, payload)
		}
	}
}

// BroadcastFCT delivers the same combat-text event to every viewer in reach.
func (s *Content) BroadcastFCT(mapState *world.MapState, kind string, worldX, worldY float64, value int) {
	s.broadcastAt(mapState, "combat_text", world.CombatText{Kind: kind, WorldX: worldX, WorldY: worldY, Value: value}, worldX, worldY)
}

// BroadcastAudioEvent raises one audio logic id for every viewer in reach, so
// a defeat or a level gained sounds for everyone who can see it.
func (s *Content) BroadcastAudioEvent(mapState *world.MapState, logicEventID string, at world.Vec2) {
	s.broadcastAt(mapState, "audio_event", world.AudioEvent{LogicEventID: logicEventID, WorldX: at.X, WorldY: at.Y}, at.X, at.Y)
}

// ── Coin helpers (flat-field design) ─────────────────────────────────────────
//
// Rule: always use these helpers for coins — never manipulate the coin
// ObjectLayer slot's Quantity directly.
//
// Invariant after every helper call:
//   entity.Coins == coin ObjectLayer slot .Quantity
//   coin ObjectLayer slot .Active == false (always)

// SyncCoinOL ensures the coin ObjectLayer slot reflects the current flat balance.
// The slot is created if absent.  Active is always forced to false: coins are
// a non-activable item (display-only in the inventory bar and modal).
func (s *Content) SyncCoinOL(layers *[]world.ObjectLayerState, amount uint32) {
	ol := getOrCreateItemOL(layers, s.CoinItemID)
	ol.Quantity = int(amount)
	ol.Active = false // coins are never activable — enforced here
}

// wallet returns the flat coin balance of player or bot id on ms.
func (s *Content) wallet(id world.EntityID, ms *world.MapState) *uint32 {
	if ms.IsKind(id, world.EntityBot) {
		return &ms.Br(id).Coins
	}
	return &s.Sessions[id].Coins
}

// setCoins sets the flat balance of id and synchronises the display-only coin
// ObjectLayer slot (Active: false, Quantity = amount).
func (s *Content) setCoins(id world.EntityID, ms *world.MapState, amount uint32) {
	*s.wallet(id, ms) = amount
	s.SyncCoinOL(&ms.Ap(id).Layers, amount)
}

// AddCoins adds delta to the balance of id (clamped to [0, MaxUint32]) and
// syncs the display coin OL slot. Returns the new balance.
func (s *Content) AddCoins(id world.EntityID, ms *world.MapState, delta int) uint32 {
	next := uint32(min(max(int64(*s.wallet(id, ms))+int64(delta), 0), math.MaxUint32))
	s.setCoins(id, ms, next)
	return next
}

// ── Generic item-quantity helpers (non-coin) ──────────────────────────────
//
// These operate exclusively on ObjectLayerState.Quantity for non-coin items.
// Do NOT pass s.coinItemID to these functions; use the coin helpers above.

// getOrCreateItemOL returns a pointer to the ObjectLayerState for itemID,
// appending an inactive slot (Quantity=0) if none exists.
func getOrCreateItemOL(layers *[]world.ObjectLayerState, itemID string) *world.ObjectLayerState {
	for i := range *layers {
		if (*layers)[i].ItemID == itemID {
			return &(*layers)[i]
		}
	}
	*layers = append(*layers, world.ObjectLayerState{ItemID: itemID, Active: false, Quantity: 0})
	return &(*layers)[len(*layers)-1]
}

// LayerQuantity is how many units of itemID the stacks of layers hold.
func LayerQuantity(layers []world.ObjectLayerState, itemID string) int {
	total := 0
	for _, ol := range layers {
		if ol.ItemID == itemID {
			total += ol.Quantity
		}
	}
	return total
}

// RemoveLayerItem drains qty of itemID from the stacks of layers one stack at a
// time: the stacks not worn first, each in slot order. An emptied stack leaves
// them, and its slot frees.
func RemoveLayerItem(layers *[]world.ObjectLayerState, itemID string, qty int) {
	var order []int
	for i := range *layers {
		if (*layers)[i].ItemID == itemID {
			order = append(order, i)
		}
	}
	slices.SortStableFunc(order, func(a, b int) int {
		la, lb := (*layers)[a], (*layers)[b]
		switch {
		case la.Active == lb.Active:
			return la.Slot - lb.Slot
		case la.Active:
			return 1
		}
		return -1
	})
	for _, i := range order {
		take := min((*layers)[i].Quantity, qty)
		(*layers)[i].Quantity -= take
		qty -= take
	}
	*layers = slices.DeleteFunc(*layers, func(layer world.ObjectLayerState) bool {
		return layer.ItemID == itemID && layer.Quantity <= 0
	})
}

// ── Fountain helpers ──────────────────────────────────────────────────────

// FountainInitPlayer credits the player's starting wallet on first connect.
// It always runs, so the seeded coin slot mirrors the balance even at zero.
func (s *Content) FountainInitPlayer(player *world.Session) {
	s.setCoins(player.ID, s.Maps[player.MapCode], uint32(s.economy.PlayerSpawnCoins))
}

// FountainInitBot sets the wallet of a bot at spawn and respawn.
func (s *Content) FountainInitBot(id world.EntityID, ms *world.MapState) {
	s.setCoins(id, ms, uint32(s.economy.BotSpawnCoins))
}

// ── Kill loot amounts ─────────────────────────────────────────────────────

// PvpCoinDropAmount is what a dead player drops of its wallet. Caller holds s.mu.
func (s *Content) PvpCoinDropAmount(player *world.Session) int {
	return s.economy.killDrop(int(player.Coins), s.economy.CoinKillPercentVsPlayer)
}

// BotCoinDropAmount is what a dead bot drops, of at least BotSpawnCoins.
// Caller holds s.mu.
func (s *Content) BotCoinDropAmount(id world.EntityID, ms *world.MapState) int {
	return s.economy.killDrop(max(int(ms.Br(id).Coins), s.economy.BotSpawnCoins), s.economy.CoinKillPercentVsBot)
}

// ── Sink helpers ──────────────────────────────────────────────────────────

// SinkRespawnCost burns RespawnCostPercent of the coins of a dead player.
func (s *Content) SinkRespawnCost(player *world.Session) {
	if s.economy.RespawnCostPercent <= 0 {
		return
	}
	balance := int(player.Coins)
	burn := int(math.Floor(float64(balance) * s.economy.RespawnCostPercent))
	if burn <= 0 {
		return
	}
	// No coin FCT — balance changes surface in the inventory-bar quantity FX.
	s.AddCoins(player.ID, s.Maps[player.MapCode], -burn)
	logx.Debugf("[ECONOMY] Respawn sink: %s burned %d coins (%.0f%%)",
		player.ID, burn, s.economy.RespawnCostPercent*100)
}
