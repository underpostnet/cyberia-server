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
//	item_ops (client→server) — the deposits and withdrawals of one session,
//	                           replayed in order against the server's items
//	snapshot self.vaults     — every vault of the player, every tick
//
// A replayed op clamps to what the source holds. No ack: the next snapshot
// carries the inventory and the vaults.
//
// Caller MUST hold s.mu for every handler here (they run inside phaseInput).
package game

import (
	"sort"

	"cyberia-server/logx"
)

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
	PlayerID   EntityID
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
func (s *GameServer) storageVault(player *Session, id EntityID, ms *MapState) (storageKey, int, bool) {
	if ms == nil || ms.vt(id).ghost() {
		return storageKey{}, 0, false
	}
	action := s.actionCache[id]
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
func (s *GameServer) botHasStorage(id EntityID, ms *MapState) bool {
	if ms == nil || ms.vt(id).ghost() {
		return false
	}
	action := s.actionCache[id]
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
func (s *GameServer) resolveStorage(player *Session, entityID EntityID) (storageKey, int, bool) {
	if s.vt(player.ID, player.MapCode).ghost() {
		return storageKey{}, 0, false
	}
	id := entityID
	ms := s.findBot(id)
	if ms == nil || !s.botInPlayerRange(player, id, ms) {
		return storageKey{}, 0, false
	}
	key, capacity, ok := s.storageVault(player, id, ms)
	if !ok {
		return storageKey{}, 0, false
	}
	return key, capacity, true
}

// handleItemOps replays one session's vault ops, front to back, against the
// server's own items. It sends no reply.
//
// Caller MUST hold s.mu.
func (s *GameServer) handleItemOps(player *Session, cmd *InputCommand) {
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
func (s *GameServer) playerItemActive(player *Session, itemID string) bool {
	for i := range s.ap(player.ID, player.MapCode).Layers {
		if s.ap(player.ID, player.MapCode).Layers[i].ItemID == itemID && s.ap(player.ID, player.MapCode).Layers[i].Active {
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
func (s *GameServer) bankableWhileWorn(player *Session, itemID string) bool {
	if !s.equipmentRules.RequireSkin || s.itemType(itemID) != "skin" {
		return true
	}
	for i := range s.ap(player.ID, player.MapCode).Layers {
		layer := s.ap(player.ID, player.MapCode).Layers[i]
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
func (s *GameServer) storageDeposit(player *Session, slots []StorageSlot, capacity int,
	itemID string, qty int) []StorageSlot {
	at := storageStackOf(slots, itemID)
	if at < 0 && len(slots) >= capacity {
		return slots
	}
	if s.playerItemActive(player, itemID) {
		if !s.bankableWhileWorn(player, itemID) {
			return slots
		}
		// Putting something away is taking it off: what is banked is no longer worn.
		setLayerActive(s.ap(player.ID, player.MapCode).Layers, itemID, false)
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
func (s *GameServer) storageWithdraw(player *Session, slots []StorageSlot,
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

// SnapshotVault is one vault of the player in the self block. Capacity stays
// in the action metadata the client already holds.
type SnapshotVault struct {
	ActionCode string        `json:"actionCode"`
	Items      []StorageSlot `json:"items"`
}

// playerVaults lists every vault of the player, in action code order.
//
// Caller MUST hold s.mu.
func (s *GameServer) playerVaults(player *Session) []SnapshotVault {
	// ponytail: a walk over every vault of every player, per player per tick;
	// key the store by player if the vault count grows.
	vaults := []SnapshotVault{}
	for key, slots := range s.storage {
		if key.PlayerID == player.ID {
			vaults = append(vaults, SnapshotVault{ActionCode: key.ActionCode, Items: slots})
		}
	}
	sort.Slice(vaults, func(i, j int) bool { return vaults[i].ActionCode < vaults[j].ActionCode })
	return vaults
}
