// Package content — storage.go
//
// The two containers a player banks items in, and the one move that carries
// stacks between them and the bag. Each container holds stacks in slots, like
// the bag.
//
//   - NPC vault: a CyberiaAction carrying `storageSlots` makes the entity on
//     its source cell a vault terminal. The world brings its fixed capacity
//     over gRPC (action.go). The player opens it at the terminal.
//   - Personal storage: the player opens it from its own profile, anywhere.
//     Its capacity grows with the player's level (StorageCapacity). The
//     wallet never goes in.
//
// Both are session state owned by the player's PlayerID; ForgetPlayer drops
// them with the session. A stored item does nothing until it is withdrawn.
//
// Cross-process contract:
//
//	item_move (client→server) — units of one stack to a slot of the bag, the storage or a vault
//	snapshot self.vaults      — every NPC vault of the player
//	snapshot self.storage     — the personal storage
//
// No ack: the next snapshot carries the inventory and both containers.
//
// Caller MUST hold s.mu for every handler here (they run inside phaseInput).
package content

import (
	"slices"
	"sort"

	"cyberia-server/game/world"
)

// StorageMaxSlots bounds an authored vault capacity, matching
// ITEM_SLOT_GRID_MAX_SLOTS on the client so a vault never exceeds its grid.
const StorageMaxSlots = 64

// storageCapacity clamps an authored capacity into the renderable range.
func storageCapacity(slots int) int {
	return min(max(slots, 0), StorageMaxSlots)
}

// vaultCapacity is the capacity of the vault a bot terminal offers, 0 when
// the bot is no live vault terminal.
//
// Caller MUST hold s.mu.
func (s *Content) vaultCapacity(id world.EntityID, ms *world.MapState) int {
	if ms == nil || ms.Vt(id).Ghost() {
		return 0
	}
	if action := s.actionCache[id]; action != nil {
		return storageCapacity(action.StorageSlots)
	}
	return 0
}

// botHasStorage reports whether a bot is a live vault terminal. Feeds
// botHasUsableAction alongside the vendor and assembler capabilities.
//
// Caller MUST hold s.mu.
func (s *Content) botHasStorage(id world.EntityID, ms *world.MapState) bool {
	return s.vaultCapacity(id, ms) >= 1
}

// slotStack is the index of the stack in slot, or -1.
func slotStack(stacks []Stack, slot int) int {
	for i := range stacks {
		if stacks[i].Slot == slot {
			return i
		}
	}
	return -1
}

// moveSet is the copy of the bag, the wallet and the open container that one
// move works on. Nothing changes until the move commits.
type moveSet struct {
	s        *Content
	player   *world.Session
	bag      []world.ObjectLayerState
	coins    int
	stacks   []Stack // the open container
	capacity int     // the open slots of the container
}

// item is the item and quantity in a place; "" for an empty one.
func (w *moveSet) item(box world.Box, slot int) (string, int) {
	switch {
	case box == world.BoxBag && slot == world.WalletSlot:
		return w.s.CoinItemID, w.coins
	case box == world.BoxBag:
		if at := w.s.bagStack(w.bag, slot); at >= 0 {
			return w.bag[at].ItemID, w.bag[at].Quantity
		}
	default:
		if at := slotStack(w.stacks, slot); at >= 0 {
			return w.stacks[at].ItemID, w.stacks[at].Qty
		}
	}
	return "", 0
}

// worn reports whether the bag stack in slot is active.
func (w *moveSet) worn(box world.Box, slot int) bool {
	at := w.s.bagStack(w.bag, slot)
	return box == world.BoxBag && at >= 0 && w.bag[at].Active
}

// takes reports whether a place may receive itemID: the wallet takes coins
// alone, the storage takes no coins, and a slot must be open.
func (w *moveSet) takes(box world.Box, slot int, itemID string) bool {
	coin := itemID == w.s.CoinItemID
	switch {
	case box == world.BoxBag && slot == world.WalletSlot:
		return coin
	case box == world.BoxBag:
		return !coin && slot >= 0 && slot < w.s.InventorySlots(w.player)
	case box == world.BoxStorage && coin:
		return false
	}
	return slot >= 0 && slot < w.capacity
}

func (w *moveSet) take(box world.Box, slot, qty int) {
	switch {
	case box == world.BoxBag && slot == world.WalletSlot:
		w.coins -= qty
	case box == world.BoxBag:
		at := w.s.bagStack(w.bag, slot)
		if w.bag[at].Quantity -= qty; w.bag[at].Quantity <= 0 {
			w.bag = slices.Delete(w.bag, at, at+1)
		}
	default:
		at := slotStack(w.stacks, slot)
		if w.stacks[at].Qty -= qty; w.stacks[at].Qty <= 0 {
			w.stacks = slices.Delete(w.stacks, at, at+1)
		}
	}
}

// put adds qty of itemID to a place: a new stack, or more of its stack. A worn
// stack keeps its flag on the stack it joins.
func (w *moveSet) put(box world.Box, slot int, itemID string, qty int, worn bool) {
	switch {
	case box == world.BoxBag && slot == world.WalletSlot:
		w.coins += qty
	case box == world.BoxBag:
		if at := w.s.bagStack(w.bag, slot); at >= 0 {
			w.bag[at].Quantity += qty
			w.bag[at].Active = w.bag[at].Active || worn
		} else {
			w.bag = append(w.bag, world.ObjectLayerState{ItemID: itemID, Quantity: qty, Slot: slot, Active: worn})
		}
	default:
		if at := slotStack(w.stacks, slot); at >= 0 {
			w.stacks[at].Qty += qty
		} else {
			w.stacks = append(w.stacks, Stack{ItemID: itemID, Qty: qty, Slot: slot})
		}
	}
}

// swap trades the slots of two whole stacks of one box.
func (w *moveSet) swap(box world.Box, a, b int) {
	if box == world.BoxBag {
		i, j := w.s.bagStack(w.bag, a), w.s.bagStack(w.bag, b)
		w.bag[i].Slot, w.bag[j].Slot = b, a
		return
	}
	i, j := slotStack(w.stacks, a), slotStack(w.stacks, b)
	w.stacks[i].Slot, w.stacks[j].Slot = b, a
}

// apply runs one move on the copy. Onto an empty slot it places the units, onto
// a stack of the same item it stacks them, and onto another stack of the same
// box it swaps the two whole stacks. False refuses the move.
func (w *moveSet) apply(m world.ItemMove) bool {
	if m.From == m.To && m.FromSlot == m.ToSlot {
		return false
	}
	itemID, held := w.item(m.From, m.FromSlot)
	qty := min(m.Qty, held)
	if itemID == "" || qty < 1 || !w.takes(m.To, m.ToSlot, itemID) {
		return false
	}
	whole := qty == held
	worn := w.worn(m.From, m.FromSlot)
	// Putting a whole worn stack away takes it off, so the equipment rules decide.
	if worn && whole && m.To != world.BoxBag && !w.s.bankableWhileWorn(w.player, itemID) {
		return false
	}
	switch target, _ := w.item(m.To, m.ToSlot); {
	case target == "" || target == itemID:
		w.take(m.From, m.FromSlot, qty)
		w.put(m.To, m.ToSlot, itemID, qty, worn && whole && m.To == world.BoxBag)
	case m.From == m.To && whole && w.takes(m.From, m.FromSlot, target):
		w.swap(m.From, m.FromSlot, m.ToSlot)
	default:
		return false
	}
	return true
}

// HandleItemMove applies one move of the player between its bag and the open
// container, or inside one of them. A move that breaks a rule changes nothing.
func (s *Content) HandleItemMove(player *world.Session, cmd *world.InputCommand) {
	ms := s.Maps[player.MapCode]
	if ms.Vt(player.ID).Ghost() {
		return
	}
	m := cmd.Move
	w := moveSet{s: s, player: player, bag: slices.Clone(*s.InventoryLayers(player)), coins: int(player.Coins)}
	var vaultCode string
	switch {
	case m.From == world.BoxVault || m.To == world.BoxVault:
		terminal := s.findBot(cmd.EntityID)
		if terminal == nil || !s.inPlayerView(player, cmd.EntityID, terminal) {
			return
		}
		if w.capacity = s.vaultCapacity(cmd.EntityID, terminal); w.capacity < 1 {
			return
		}
		vaultCode = s.actionCache[cmd.EntityID].Code
		w.stacks = slices.Clone(s.Vaults[player.PlayerID][vaultCode])
	case m.From == world.BoxStorage || m.To == world.BoxStorage:
		w.capacity = s.StorageSlots(player)
		w.stacks = slices.Clone(s.PersonalStorage[player.PlayerID])
	}
	if !w.apply(m) || !s.fitsBag(player, w.bag) {
		return
	}

	*s.InventoryLayers(player) = w.bag
	if delta := w.coins - int(player.Coins); delta != 0 {
		s.AddCoins(player.ID, ms, delta)
	}
	switch {
	case vaultCode != "":
		if s.Vaults == nil {
			s.Vaults = make(map[world.PlayerID]map[string][]Stack)
		}
		if s.Vaults[player.PlayerID] == nil {
			s.Vaults[player.PlayerID] = make(map[string][]Stack)
		}
		s.Vaults[player.PlayerID][vaultCode] = w.stacks
	case m.From == world.BoxStorage || m.To == world.BoxStorage:
		if s.PersonalStorage == nil {
			s.PersonalStorage = make(map[world.PlayerID][]Stack)
		}
		s.PersonalStorage[player.PlayerID] = w.stacks
	}
	s.ApplyResistanceStat(player.ID, ms)
	// A withdrawal is an inventory gain like any other: a collect objective it
	// satisfies advances at once.
	if m.To == world.BoxBag && m.From != world.BoxBag {
		s.AdvancePlayerQuestsOnGain(player)
	}
}

// bankableWhileWorn reports whether an equipped item may be taken off to be banked.
//
// The loadout has to survive without it, and the equipment rules say what that means: requireSkin
// keeps a skin on the entity, so the last active one stays put while a second skin — or a weapon,
// or a breastplate — is free to go. Anything the rules do not govern was never held back.
func (s *Content) bankableWhileWorn(player *world.Session, itemID string) bool {
	if !s.EquipmentRules.RequireSkin || s.ItemType(itemID) != "skin" {
		return true
	}
	for _, layer := range s.Ap(player.ID, player.MapCode).Layers {
		if layer.ItemID == itemID || !layer.Active {
			continue
		}
		if s.ItemType(layer.ItemID) == "skin" {
			return true
		}
	}
	return false
}

// SnapshotVault is one vault of the player in the self block. Capacity stays
// in the action metadata the client already holds.
type SnapshotVault struct {
	ActionCode string  `json:"actionCode"`
	Items      []Stack `json:"items"`
}

// PlayerVaults lists every NPC vault of the player, in action code order.
//
// Caller MUST hold s.mu.
func (s *Content) PlayerVaults(player *world.Session) []SnapshotVault {
	vaults := make([]SnapshotVault, 0, len(s.Vaults[player.PlayerID]))
	for code, stacks := range s.Vaults[player.PlayerID] {
		vaults = append(vaults, SnapshotVault{ActionCode: code, Items: stacks})
	}
	sort.Slice(vaults, func(i, j int) bool { return vaults[i].ActionCode < vaults[j].ActionCode })
	return vaults
}

// PlayerStorage is the personal storage of the player, never nil.
//
// Caller MUST hold s.mu.
func (s *Content) PlayerStorage(player *world.Session) []Stack {
	if stacks := s.PersonalStorage[player.PlayerID]; stacks != nil {
		return stacks
	}
	return []Stack{}
}

// ForgetPlayer drops the containers of a player whose session ended.
//
// Caller MUST hold s.mu.
func (s *Content) ForgetPlayer(id world.PlayerID) {
	delete(s.Vaults, id)
	delete(s.PersonalStorage, id)
}
