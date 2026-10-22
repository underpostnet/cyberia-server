package content

// economy.go — the economy of an instance: its coin model, the wallet bound,
// the operation fees and the item policies of its profile.
//
// Session.Coins of a player and Brain.Coins of a bot hold every balance. The
// coin ObjectLayer stack only shows it: SyncCoinOL keeps it inactive and at the
// balance. It is no stack: no trade, drop, sale or burn moves it.
//
// Where coins come from and go:
//   - A new guest session or account profile starts with PlayerSpawnCoins.
//   - A bot holds BotSpawnCoins at each spawn. Its death drop is new coins: the
//     wallet stays with the bot until its respawn resets it.
//   - A dead player loses its drop to a loot token. A death without damage
//     contributors drops nothing.
//   - A contributor, player or bot, that collects a coin token gains its coins.
//     An uncollected token expires with them.
//   - A price, a fee or a burn cost leaves the economy. A reward or a buyback
//     payout enters it. A vault moves coins between the wallet and the vault,
//     and a trade between two wallets.
//   - RespawnCostPercent burns a share of the coins of a dead player. 0 disables it.
//
// A player credit never passes MaxBalance: the operation that would pass it is
// refused, and its token stays on the ground.
//
// Damage and regen text reaches every viewer; XP text reaches its earner only.
// A coin or item change shows no text: the inventory bar and the loot grid show it.

import (
	"fmt"
	"maps"
	"math"
	"slices"

	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
	"cyberia-server/logx"
)

// EconomyRules is the economy of an instance: the coin model of its profile,
// the wallet maximum, the operation fees and the item policies. A rate is a
// fraction of 1 of the coins of the victim, by the type of the victim.
type EconomyRules struct {
	BotSpawnCoins, PlayerSpawnCoins, CoinKillMinAmount                int
	CoinKillPercentVsBot, CoinKillPercentVsPlayer, RespawnCostPercent float64
	// AuthorityID, Code and Revision name the profile snapshot: a code is local
	// to its authority. An empty code is a world without content.
	AuthorityID string
	Code        string
	Revision    uint64
	// MaxBalance bounds every wallet; 0 is the contract maximum. A credit past
	// it is refused.
	MaxBalance   uint32
	Fees         OperationFees
	ItemDefaults ItemPolicy
	// Items holds the policy of each item with a row, by item id.
	Items map[string]ItemPolicy
}

// OperationFees is the coins one operation that succeeds charges once.
type OperationFees struct {
	Portal int `json:"portal"`
	Craft  int `json:"craft"`
}

// ItemPolicy is what one item may do in the economy. CID is the definition the
// row compiled against; the defaults carry none. Coin amounts are per unit.
type ItemPolicy struct {
	ItemID         string `json:"-"`
	CID            string `json:"-"`
	Trade          bool   `json:"trade"`
	WorldDrop      bool   `json:"worldDrop"`
	Buyback        bool   `json:"buyback"`
	BuybackPayout  int    `json:"buybackPayout"`
	Burn           bool   `json:"burn"`
	BurnCoinFee    int    `json:"burnCoinFee"`
	CraftSurcharge int    `json:"craftSurcharge"`
	Market         bool   `json:"market"`
	// PortalFee is what a portal charges for each active portal item of the
	// label; nil adds nothing, 0 makes the portal free.
	PortalFee *int `json:"-"`
	// Salvage is what a burn grants per burned unit.
	Salvage []Stack `json:"salvage"`
}

// EconomyView is the economy a client shows: the fees, the wallet maximum and
// each item policy, so every amount it shows is the one the server charges.
type EconomyView struct {
	Code     string `json:"code"`
	Revision uint64 `json:"revision"`
	// CoinItemID names the item that shows the wallet.
	CoinItemID   string                `json:"coinItemId"`
	MaxBalance   uint32                `json:"maxBalance"`
	Fees         OperationFees         `json:"fees"`
	ItemDefaults ItemPolicy            `json:"itemDefaults"`
	Items        map[string]ItemPolicy `json:"items"`
}

// EconomyView is the economy of the world for its clients. Caller holds s.mu.
func (s *Content) EconomyView() EconomyView {
	return EconomyView{
		Code: s.economy.Code, Revision: s.economy.Revision, CoinItemID: s.CoinItemID, MaxBalance: uint32(s.maxBalance()),
		Fees: s.economy.Fees, ItemDefaults: s.economy.ItemDefaults, Items: s.economy.Items,
	}
}

func itemPolicyFromProto(p *pb.ItemPolicy) ItemPolicy {
	policy := ItemPolicy{
		ItemID: p.GetItemId(), CID: p.GetOlCid(),
		Trade: p.GetTrade(), WorldDrop: p.GetWorldDrop(), Buyback: p.GetBuyback(), Burn: p.GetBurn(),
		BuybackPayout: int(p.GetBuybackPayout()), BurnCoinFee: int(p.GetBurnCoinFee()),
		CraftSurcharge: int(p.GetCraftSurcharge()), Market: p.GetMarket(),
	}
	if p != nil && p.PortalFee != nil {
		fee := int(p.GetPortalFee())
		policy.PortalFee = &fee
	}
	for _, line := range p.GetSalvage() {
		policy.Salvage = append(policy.Salvage, Stack{ItemID: line.GetItemId(), Qty: int(line.GetQty())})
	}
	return policy
}

func EconomyRulesFromProto(p *pb.EconomyRules) EconomyRules {
	rules := EconomyRules{
		BotSpawnCoins: int(p.GetBotSpawnCoins()), PlayerSpawnCoins: int(p.GetPlayerSpawnCoins()),
		CoinKillMinAmount: int(p.GetCoinKillMinAmount()), CoinKillPercentVsBot: p.GetCoinKillPercentVsBot(),
		CoinKillPercentVsPlayer: p.GetCoinKillPercentVsPlayer(), RespawnCostPercent: p.GetRespawnCostPercent(),
		AuthorityID: p.GetAuthorityId(), Code: p.GetCode(), Revision: p.GetRevision(), MaxBalance: p.GetMaxBalance(),
		Fees:         OperationFees{Portal: int(p.GetFees().GetPortal()), Craft: int(p.GetFees().GetCraft())},
		ItemDefaults: itemPolicyFromProto(p.GetItemDefaults()),
		Items:        make(map[string]ItemPolicy, len(p.GetItemPolicies())),
	}
	for _, row := range p.GetItemPolicies() {
		rules.Items[row.GetItemId()] = itemPolicyFromProto(row)
	}
	return rules
}

// Validate refuses rules outside the bounds of the engine contract: a coin
// rule, a fee or an item policy out of bounds, a wallet maximum below a spawn
// wallet, an item row without its item and definition, or a salvage line out
// of bounds.
func (r EconomyRules) Validate() error {
	if !r.validBounds() || !r.Fees.validBounds() || !r.ItemDefaults.validBounds() {
		return fmt.Errorf("invalid economy rules")
	}
	if r.MaxBalance > 0 && int64(r.MaxBalance) < int64(max(r.PlayerSpawnCoins, r.BotSpawnCoins)) {
		return fmt.Errorf("invalid economy rules: wallet maximum %d", r.MaxBalance)
	}
	for itemID, policy := range r.Items {
		if itemID == "" || policy.CID == "" || !policy.validBounds() {
			return fmt.Errorf("invalid item policy %q", itemID)
		}
		for _, line := range policy.Salvage {
			if line.ItemID == "" || line.ItemID == itemID || line.Qty < 1 || line.Qty > SalvageQuantityMax {
				return fmt.Errorf("invalid salvage of item policy %q", itemID)
			}
		}
	}
	return nil
}

// Economy is the economy the world runs. Caller holds s.mu.
func (s *Content) Economy() EconomyRules { return s.economy }

// EconomyKey names the economy snapshot the world runs: authority/code@revision.
// Caller holds s.mu.
func (s *Content) EconomyKey() string {
	if s.economy.AuthorityID == "" {
		return fmt.Sprintf("%s@%d", s.economy.Code, s.economy.Revision)
	}
	return fmt.Sprintf("%s/%s@%d", s.economy.AuthorityID, s.economy.Code, s.economy.Revision)
}

// Refusals of an economic operation. A client maps each one to a message.
const (
	RefuseNotAllowed   = "not_allowed"        // a guest, or an offer past its limits
	RefuseNotOffered   = "not_offered"        // the terminal does not offer the item or the mode
	RefusePolicy       = "policy_refused"     // the item policy of the economy refuses the operation
	RefuseIncompatible = "incompatible"       // the stack runs another definition than the world or the policy
	RefuseNoStack      = "no_stack"           // no such stack, or fewer units than asked
	RefuseEquipped     = "equipped"           // the equipment rules keep the worn stack on
	RefuseInsufficient = "insufficient_funds" // the wallet lacks the coins
	RefuseWalletFull   = "wallet_full"        // the credit passes the wallet maximum
	RefuseFull         = "inventory_full"     // the bag has no room for what the operation grants
	RefuseQuote        = "quote_changed"      // the price changed since the quote the player confirmed
	RefuseQuoteGone    = "quote_gone"         // the quote is used, expired or unknown
	RefuseOutOfRange   = "out_of_range"       // the partner or the terminal is out of view
	RefuseTrading      = "trading"            // a trade holds the goods of the player
	RefuseDead         = "dead"               // the player waits to respawn
	RefuseSaving       = "saving"             // an earlier operation of the player still waits for its commit
)

// ItemPolicyOf is the policy the stack of itemID of definition cid runs
// under, or a refusal: a stack whose definition is not the one the world binds,
// or not the one its row compiled against, takes part in no economic operation.
// Caller holds s.mu.
func (s *Content) ItemPolicyOf(itemID, cid string) (ItemPolicy, string) {
	bound := s.DefinitionCID(itemID)
	if bound == "" || cid != bound {
		return ItemPolicy{}, RefuseIncompatible
	}
	policy, ok := s.economy.Items[itemID]
	if !ok {
		policy = s.economy.ItemDefaults
		policy.ItemID, policy.CID = itemID, bound
		return policy, ""
	}
	if policy.CID != bound {
		return ItemPolicy{}, RefuseIncompatible
	}
	return policy, ""
}

// maxBalance is the wallet maximum.
func (s *Content) maxBalance() int64 {
	if s.economy.MaxBalance == 0 {
		return CoinBalanceMax
	}
	return int64(s.economy.MaxBalance)
}

// coinRoom is how many coins the wallet of a player still takes: the wallet
// maximum less the balance and the coins a committing trade will credit.
// Caller holds s.mu.
func (s *Content) coinRoom(player *world.Session) int64 {
	return s.maxBalance() - int64(player.Coins) - int64(s.incomingCoins(player))
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

// AddCoins adds delta to the balance of id, clamped to [0, the wallet
// maximum], and syncs the display coin OL slot. A player credit checks its room
// first (coinRoom), so it never clamps. Returns the new balance.
func (s *Content) AddCoins(id world.EntityID, ms *world.MapState, delta int) uint32 {
	next := uint32(min(max(int64(*s.wallet(id, ms))+int64(delta), 0), s.maxBalance()))
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

// recordEconomy counts one economic operation by its outcome: "ok" or the
// refusal. Caller holds s.mu.
func (s *Content) recordEconomy(op, refusal string) {
	if refusal == "" {
		refusal = "ok"
	}
	if s.economyOutcomes == nil {
		s.economyOutcomes = make(map[string]uint64)
	}
	s.economyOutcomes[op+":"+refusal]++
}

// EconomyOutcomes is the count of each operation outcome since the start, as
// "op:outcome". Caller holds s.mu.
func (s *Content) EconomyOutcomes() map[string]uint64 { return maps.Clone(s.economyOutcomes) }

// PortalFee is the coins one transit through a portal charges: the sum of the
// portalFee of each active portal item that names one, up to PortalFeeMax;
// else the portal fee of the economy. A portal item runs its bound definition,
// so a row compiled against another adds nothing. Caller holds s.mu.
func (s *Content) PortalFee(ms *world.MapState, portal world.EntityID) int {
	var sum int64
	named := false
	for _, layer := range ms.Ap(portal).Layers {
		if !layer.Active {
			continue
		}
		policy, reason := s.ItemPolicyOf(layer.ItemID, s.DefinitionCID(layer.ItemID))
		if reason != "" || policy.PortalFee == nil {
			continue
		}
		named = true
		sum += int64(*policy.PortalFee)
	}
	if !named {
		return s.economy.Fees.Portal
	}
	return int(min(sum, PortalFeeMax))
}

// PortalFeeRefusal is why the player cannot pay fee, or "". Caller holds s.mu.
func (s *Content) PortalFeeRefusal(player *world.Session, fee int) string {
	if fee > 0 && int64(player.Coins) < int64(fee) {
		s.recordEconomy("portal", RefuseInsufficient)
		return RefuseInsufficient
	}
	return ""
}

// ChargePortalFee takes fee for a transit that happened, once. Caller holds s.mu.
func (s *Content) ChargePortalFee(player *world.Session, fee int) {
	if fee <= 0 {
		return
	}
	s.AddCoins(player.ID, s.Maps[player.MapCode], -fee)
	s.recordEconomy("portal", "")
}

// CraftFee is the coins one assembly of a recipe charges, by one rule: the fee
// the recipe names, else the craft surcharge of each produced unit, else the
// craft fee of the economy, else none. The rules never add up, and no
// ingredient charges a fee. Caller holds s.mu.
func (s *Content) CraftFee(recipe *ActionCraftRecipe) int {
	if recipe.Fee != nil {
		return *recipe.Fee
	}
	surcharge := 0
	for _, out := range recipe.OutputItems {
		policy, ok := s.economy.Items[out.ItemID]
		if !ok {
			policy = s.economy.ItemDefaults
		}
		surcharge += policy.CraftSurcharge * max(out.Qty, 0)
	}
	if surcharge > 0 {
		return surcharge
	}
	return s.economy.Fees.Craft
}
