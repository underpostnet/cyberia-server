// Package game — storage.go
//
// Authoritative personal storage. engine-cyberia owns the vault's capacity: a
// CyberiaAction carrying `storageSlots` makes the entity on its source cell a
// storage terminal, and the value arrives with the world over gRPC (action.go).
// Contents are runtime state held here for the session, shaped so each stack
// maps 1:1 onto a future Mongoose document.
//
// The vault is a bag of stacks. Cell order is client presentation.
//
// Cross-process contract:
//
//	storage_open  (client→server) — bind the vault, reply with its contents
//	item_ops      (client→server) — the deposits and withdrawals of one session,
//	                                replayed in order against the server's items
//	storage_state (server→client) — the vault, once per open
//
// A replayed op clamps to what the source holds. No ack: the next snapshot
// carries the inventory.
//
// Caller MUST hold s.mu for every handler here (they run inside phaseInput).
package game

import "cyberia-server/logx"

// StorageSlot is one stack. The JSON tags are the persistence shape.
type StorageSlot struct {
	ItemID string `json:"itemId"`
	Qty    int    `json:"qty"`
}

// itemOp is one replayed vault operation: Qty of ItemID into the vault when
// ToVault, else out of it.
type itemOp struct {
	ItemID  string `json:"itemId"`
	Qty     int    `json:"qty"`
	ToVault bool   `json:"toVault"`
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
	return min(max(slots, 0), storageMaxSlots)
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

// storageStackOf returns the position of the stack holding itemID, or -1.
func storageStackOf(slots []StorageSlot, itemID string) int {
	for i := range slots {
		if slots[i].ItemID == itemID {
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

// handleItemOps replays one session's vault ops, front to back, against the
// server's own items. It sends no reply.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleItemOps(player *PlayerState, cmd *InputCommand) {
	key, capacity, ok := s.resolveStorage(player, cmd.EntityID)
	if !ok {
		return
	}
	slots := s.storage[key]
	for _, op := range cmd.Ops {
		if op.ToVault {
			slots = s.storageDeposit(player, slots, capacity, op.ItemID, op.Qty)
		} else {
			slots = s.storageWithdraw(player, slots, op.ItemID, op.Qty)
		}
	}
	s.storage[key] = slots
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

// bankableWhileWorn reports whether an equipped item may be taken off to be banked.
//
// The loadout has to survive without it, and the equipment rules say what that means: requireSkin
// keeps a skin on the entity, so the last active one stays put while a second skin — or a weapon,
// or a breastplate — is free to go. Anything the rules do not govern was never held back.
func (s *GameServer) bankableWhileWorn(player *PlayerState, itemID string) bool {
	if !s.equipmentRules.RequireSkin || s.itemType(itemID) != "skin" {
		return true
	}
	for i := range player.ObjectLayers {
		layer := player.ObjectLayers[i]
		if layer.ItemID == itemID || !layer.Active {
			continue
		}
		if s.itemType(layer.ItemID) == "skin" {
			return true
		}
	}
	return false
}

// setLayerActive flips one carried layer, leaving the rest of the inventory alone.
func setLayerActive(layers []ObjectLayerState, itemID string, active bool) {
	for i := range layers {
		if layers[i].ItemID == itemID {
			layers[i].Active = active
			return
		}
	}
}

// storageDeposit moves qty of itemID from the player into the vault, merging
// into its stack. A new stack is refused once the vault holds capacity stacks.
// A worn item is taken off first, and only refused when the equipment rules
// need it worn.
//
// Caller MUST hold s.mu.
func (s *GameServer) storageDeposit(player *PlayerState, slots []StorageSlot, capacity int,
	itemID string, qty int) []StorageSlot {
	at := storageStackOf(slots, itemID)
	if at < 0 && len(slots) >= capacity {
		return slots
	}
	if playerItemActive(player, itemID) {
		if !s.bankableWhileWorn(player, itemID) {
			return slots
		}
		// Putting something away is taking it off: what is banked is no longer worn.
		setLayerActive(player.ObjectLayers, itemID, false)
	}
	if held := s.playerItemQuantity(player, itemID); qty > held {
		qty = held
	}
	if qty <= 0 {
		return slots
	}

	s.removePlayerItem(player, itemID, qty)
	if at >= 0 {
		slots[at].Qty += qty
		return slots
	}
	return append(slots, StorageSlot{ItemID: itemID, Qty: qty})
}

// storageWithdraw moves qty of itemID from the vault back into the player's
// inventory, dropping the stack once it is empty.
//
// Caller MUST hold s.mu.
func (s *GameServer) storageWithdraw(player *PlayerState, slots []StorageSlot,
	itemID string, qty int) []StorageSlot {
	at := storageStackOf(slots, itemID)
	if at < 0 {
		return slots
	}
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
	logx.Debugf("[STORAGE] player %s emptied stack %s", player.ID, itemID)
	return append(slots[:at], slots[at+1:]...)
}

// sendStorageState pushes the vault. The client seeds its grid from it.
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
