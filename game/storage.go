// Package game — storage.go
//
// Authoritative personal storage. engine-cyberia owns the vault's capacity: a
// CyberiaAction carrying `storageSlots` makes the entity on its source cell a
// storage terminal, and the value arrives with the world over gRPC (action.go).
// Contents are runtime state held here for the session, shaped so each slot
// maps 1:1 onto a future Mongoose document.
//
// A slot's address is its linear index in 0..capacity-1. The client wraps those
// slots into however many columns its panel width admits, so a row/column pair
// is presentation and can never be authoritative.
//
// Cross-process contract:
//
//	storage_open     (client→server) — bind the vault, reply with its contents
//	storage_move     (client→server) — relocate a slot onto an empty slot, or
//	                                   merge it into one holding the same item
//	storage_swap     (client→server) — exchange two occupied slots
//	storage_transfer (client→server) — move a stack between vault and inventory
//	storage_state    (server→client) — the authoritative vault after any of them
//
// Every mutation answers with the full vault: the client applies its drop
// optimistically and reconciles from this, so a rejected op self-heals without
// a bespoke error path.
//
// Caller MUST hold s.mu for every handler here (they run inside phaseInput).
package game

import "cyberia-server/logx"

// StorageSlot is one occupied slot. The JSON tags are the persistence shape:
// itemId + qty + index is everything a document needs.
type StorageSlot struct {
	ItemID string `json:"itemId"`
	Qty    int    `json:"qty"`
	Index  int    `json:"index"`
}

// storageKey scopes a vault to one player at one action cell — storage is
// personal, so two players at the same terminal never see each other's stock.
type storageKey struct {
	PlayerID   string
	ActionCode string
}

// storageMaxSlots bounds an authored capacity, matching ITEM_SLOT_GRID_MAX_SLOTS
// on the client so a vault can never exceed the grid that renders it.
const storageMaxSlots = 64

// storageCapacity clamps an authored capacity into the renderable range.
func storageCapacity(slots int) int {
	if slots < 0 {
		return 0
	}
	if slots > storageMaxSlots {
		return storageMaxSlots
	}
	return slots
}

// storageVault resolves the vault a player has at a bot's bound action, or nil
// when that entity is not a storage terminal.
//
// Caller MUST hold s.mu.
func (s *GameServer) storageVault(player *PlayerState, bot *BotState) (storageKey, int, bool) {
	if bot == nil || bot.IsGhost() {
		return storageKey{}, 0, false
	}
	action := s.actionCache[bot.ID]
	if action == nil {
		return storageKey{}, 0, false
	}
	capacity := storageCapacity(action.StorageSlots)
	if capacity < 1 {
		return storageKey{}, 0, false
	}
	return storageKey{PlayerID: player.ID, ActionCode: action.Code}, capacity, true
}

// botHasStorage reports whether a bot is a live storage terminal. Feeds
// botHasUsableAction alongside the vendor and assembler capabilities.
//
// Caller MUST hold s.mu.
func (s *GameServer) botHasStorage(bot *BotState) bool {
	if bot == nil || bot.IsGhost() {
		return false
	}
	action := s.actionCache[bot.ID]
	return action != nil && storageCapacity(action.StorageSlots) >= 1
}

// storageSlotAt returns the position in the vault of the slot occupying an
// index, or -1 when that index is free.
func storageSlotAt(slots []StorageSlot, index int) int {
	for i := range slots {
		if slots[i].Index == index {
			return i
		}
	}
	return -1
}

// resolveStorage validates a request against the entity it names and returns
// the vault's key, capacity and liveness. Every handler starts here.
//
// Caller MUST hold s.mu.
func (s *GameServer) resolveStorage(player *PlayerState, entityID string) (storageKey, int, bool) {
	if player.IsGhost() {
		return storageKey{}, 0, false
	}
	bot := s.findBot(entityID)
	if bot == nil || !botInPlayerRange(player, bot) {
		return storageKey{}, 0, false
	}
	key, capacity, ok := s.storageVault(player, bot)
	if !ok {
		return storageKey{}, 0, false
	}
	// Trading with a vault is an interaction: assert the modal protection so a
	// transfer can never resolve while the player is exposed.
	s.holdProviderFreeze(player, bot.ID)
	return key, capacity, true
}

// handleStorageOpen binds the vault and answers with its current contents.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleStorageOpen(player *PlayerState, cmd *InputCommand) {
	key, capacity, ok := s.resolveStorage(player, cmd.EntityID)
	if !ok {
		return
	}
	s.sendStorageState(player, cmd.EntityID, capacity, s.storage[key])
}

// handleStorageMove moves a slot onto another index of the same vault: free,
// or holding the same item, in which case the two stacks merge. A Quantity
// below what the source holds splits it, leaving the remainder where it was.
// A target holding a different item is refused — that drop is a swap.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleStorageMove(player *PlayerState, cmd *InputCommand) {
	key, capacity, ok := s.resolveStorage(player, cmd.EntityID)
	if !ok {
		return
	}
	slots := s.storage[key]
	from := storageSlotAt(slots, cmd.FromIndex)
	to := storageSlotAt(slots, cmd.ToIndex)
	if from < 0 || cmd.FromIndex == cmd.ToIndex ||
		cmd.ToIndex < 0 || cmd.ToIndex >= capacity ||
		(to >= 0 && slots[to].ItemID != slots[from].ItemID) {
		s.sendStorageState(player, cmd.EntityID, capacity, slots)
		return
	}
	slots = storageRelocate(slots, from, to, cmd.ToIndex, cmd.Quantity)
	s.storage[key] = slots
	s.sendStorageState(player, cmd.EntityID, capacity, slots)
}

// storageRelocate moves qty out of the slot at position `from` onto toIndex.
// `to` is the position of the slot already at toIndex holding the same item, or
// -1 when that index is free. A qty below what the source holds splits it,
// leaving the remainder in place; anything else moves the whole stack.
func storageRelocate(slots []StorageSlot, from, to, toIndex, qty int) []StorageSlot {
	whole := qty <= 0 || qty >= slots[from].Qty
	if whole {
		qty = slots[from].Qty
	}

	if to < 0 {
		if whole {
			slots[from].Index = toIndex
			return slots
		}
		slots[from].Qty -= qty
		return append(slots, StorageSlot{ItemID: slots[from].ItemID, Qty: qty, Index: toIndex})
	}

	slots[to].Qty += qty
	if whole {
		return append(slots[:from], slots[from+1:]...)
	}
	slots[from].Qty -= qty
	return slots
}

// handleStorageSwap exchanges the indices of two occupied slots.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleStorageSwap(player *PlayerState, cmd *InputCommand) {
	key, capacity, ok := s.resolveStorage(player, cmd.EntityID)
	if !ok {
		return
	}
	slots := s.storage[key]
	from := storageSlotAt(slots, cmd.FromIndex)
	to := storageSlotAt(slots, cmd.ToIndex)
	if from < 0 || to < 0 || from == to {
		s.sendStorageState(player, cmd.EntityID, capacity, slots)
		return
	}
	slots[from].Index, slots[to].Index = slots[to].Index, slots[from].Index
	s.sendStorageState(player, cmd.EntityID, capacity, slots)
}

// handleStorageTransfer moves a stack across the vault boundary: Deposit takes
// it out of the player's inventory into the named slot, Withdraw does the
// reverse. Quantities are clamped to what the source actually holds.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleStorageTransfer(player *PlayerState, cmd *InputCommand) {
	key, capacity, ok := s.resolveStorage(player, cmd.EntityID)
	if !ok {
		return
	}
	slots := s.storage[key]
	if cmd.Deposit {
		slots = s.storageDeposit(player, slots, capacity, cmd)
	} else {
		slots = s.storageWithdraw(player, slots, cmd)
	}
	s.storage[key] = slots
	s.sendStorageState(player, cmd.EntityID, capacity, slots)
}

// playerItemActive reports whether the player currently has this item equipped.
// An active layer is worn, not stock, so it can never be banked.
func playerItemActive(player *PlayerState, itemID string) bool {
	for i := range player.ObjectLayers {
		if player.ObjectLayers[i].ItemID == itemID && player.ObjectLayers[i].Active {
			return true
		}
	}
	return false
}

// storageDeposit consumes the stack from the player and lands it on the target
// slot, merging when the slot already holds the same item. An equipped item is
// refused: the client hides the drag for one, and the rule is enforced here too.
//
// Caller MUST hold s.mu.
func (s *GameServer) storageDeposit(player *PlayerState, slots []StorageSlot, capacity int,
	cmd *InputCommand) []StorageSlot {
	if cmd.ToIndex < 0 || cmd.ToIndex >= capacity {
		return slots
	}
	if playerItemActive(player, cmd.ItemID) {
		return slots
	}
	qty := cmd.Quantity
	if held := s.playerItemQuantity(player, cmd.ItemID); qty > held {
		qty = held
	}
	if qty <= 0 {
		return slots
	}
	at := storageSlotAt(slots, cmd.ToIndex)
	if at >= 0 && slots[at].ItemID != cmd.ItemID {
		return slots
	}

	s.removePlayerItem(player, cmd.ItemID, qty)
	if at >= 0 {
		slots[at].Qty += qty
		return slots
	}
	return append(slots, StorageSlot{ItemID: cmd.ItemID, Qty: qty, Index: cmd.ToIndex})
}

// storageWithdraw drains the named slot back into the player's inventory,
// dropping it once it is empty.
//
// Caller MUST hold s.mu.
func (s *GameServer) storageWithdraw(player *PlayerState, slots []StorageSlot,
	cmd *InputCommand) []StorageSlot {
	at := storageSlotAt(slots, cmd.FromIndex)
	if at < 0 {
		return slots
	}
	qty := cmd.Quantity
	if qty > slots[at].Qty {
		qty = slots[at].Qty
	}
	if qty <= 0 {
		return slots
	}

	s.addPlayerItem(player, slots[at].ItemID, qty)
	// A withdrawal is an inventory gain like any other — reconcile collect
	// objectives so a quest step satisfied by it advances immediately.
	s.advancePlayerQuestsOnGain(player)

	slots[at].Qty -= qty
	if slots[at].Qty > 0 {
		return slots
	}
	logx.Debugf("[STORAGE] player %s emptied slot %d", player.ID, cmd.FromIndex)
	return append(slots[:at], slots[at+1:]...)
}

// sendStorageState pushes the authoritative vault, which the client adopts
// wholesale over its optimistic view.
func (s *GameServer) sendStorageState(player *PlayerState, entityID string, capacity int,
	slots []StorageSlot) {
	if slots == nil {
		slots = []StorageSlot{}
	}
	sendMessage(player, "storage_state", map[string]interface{}{
		"entityId": entityID,
		"capacity": capacity,
		"slots":    slots,
	})
}
