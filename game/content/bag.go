// Package content — bag.go
//
// The bag of a player holds stacks in slots. A slot holds one stack: one item
// at any quantity. One item can fill several stacks. A stack keeps its slot
// until a move changes it, so a stack that leaves frees its cell and the other
// stacks stay. The coin and the ghost visual take no slot.
//
// Caller MUST hold s.mu.
package content

import (
	"slices"

	"cyberia-server/game/world"
)

// InventoryStack is one carried stack on the wire. Slot is world.WalletSlot for
// a stack that takes no slot.
type InventoryStack struct {
	ItemID   string `json:"itemId"`
	Active   bool   `json:"active"`
	Quantity int    `json:"quantity"`
	Slot     int    `json:"slot"`
}

// freeSlot is the lowest slot no stack of layers takes.
func (s *Content) freeSlot(layers []world.ObjectLayerState) int {
	taken := make(map[int]bool, len(layers))
	for _, layer := range layers {
		if s.takesSlot(layer.ItemID) {
			taken[layer.Slot] = true
		}
	}
	slot := 0
	for taken[slot] {
		slot++
	}
	return slot
}

// SeatStacks gives the lowest free slot to each stack of layers, from index
// from on, that takes one.
func (s *Content) SeatStacks(layers []world.ObjectLayerState, from int) {
	for i := from; i < len(layers); i++ {
		if s.takesSlot(layers[i].ItemID) {
			layers[i].Slot = s.freeSlot(layers[:i])
		}
	}
}

// bagStack is the index of the stack in slot, or -1.
func (s *Content) bagStack(layers []world.ObjectLayerState, slot int) int {
	for i := range layers {
		if s.takesSlot(layers[i].ItemID) && layers[i].Slot == slot {
			return i
		}
	}
	return -1
}

// firstStack is the index of the matching stack in the lowest slot, or -1.
func firstStack(layers []world.ObjectLayerState, match func(world.ObjectLayerState) bool) int {
	at := -1
	for i := range layers {
		if match(layers[i]) && (at < 0 || layers[i].Slot < layers[at].Slot) {
			at = i
		}
	}
	return at
}

// StackIndex is the index of the stack of itemID in slot, or of its first stack
// for a slot below 0. -1 when there is none.
func (s *Content) StackIndex(layers []world.ObjectLayerState, itemID string, slot int) int {
	if slot < 0 {
		return firstStack(layers, func(l world.ObjectLayerState) bool { return l.ItemID == itemID })
	}
	if at := s.bagStack(layers, slot); at >= 0 && layers[at].ItemID == itemID {
		return at
	}
	return -1
}

// stackInto adds qty of itemID, of the definition the world binds now, to the
// bag: onto its first stack of that definition, else as a new stack in the
// lowest free slot.
func (s *Content) stackInto(bag []world.ObjectLayerState, itemID string, qty int) []world.ObjectLayerState {
	cid := s.DefinitionCID(itemID)
	if at := firstStack(bag, func(l world.ObjectLayerState) bool { return l.ItemID == itemID && l.CID == cid }); at >= 0 {
		bag[at].Quantity += qty
		return bag
	}
	return append(bag, world.ObjectLayerState{ItemID: itemID, CID: cid, Quantity: qty, Slot: s.freeSlot(bag)})
}

// StampDefinitions gives every owned stack of layers without a definition the
// one the world binds now.
func (s *Content) StampDefinitions(layers []world.ObjectLayerState) {
	for i := range layers {
		if layers[i].CID == "" && s.takesSlot(layers[i].ItemID) {
			layers[i].CID = s.DefinitionCID(layers[i].ItemID)
		}
	}
}

// occupiedSlots counts the stacks of layers that take a slot, and one slot for
// each item a running assembly pays out that layers holds no stack of.
func (s *Content) occupiedSlots(player *world.Session, layers []world.ObjectLayerState) int {
	n := 0
	held := make(map[string]bool)
	for _, layer := range layers {
		if s.takesSlot(layer.ItemID) && layer.Quantity > 0 {
			n++
			held[layer.ItemID] = true
		}
	}
	if craft := s.crafts[player.ID]; craft != nil {
		for _, stack := range slices.Concat(craft.Refund, craft.Outputs) {
			if s.takesSlot(stack.ItemID) && !held[stack.ItemID] {
				held[stack.ItemID] = true
				n++
			}
		}
	}
	return n
}

// fitsBag reports whether bag may replace the bag of the player: it takes no
// more slots than are open, or than the bag took before.
func (s *Content) fitsBag(player *world.Session, bag []world.ObjectLayerState) bool {
	before := s.occupiedSlots(player, *s.InventoryLayers(player))
	return s.occupiedSlots(player, bag) <= max(s.InventorySlots(player), before)
}

// InventoryView is the carried inventory on the wire, without the ghost visual.
func (s *Content) InventoryView(layers []world.ObjectLayerState) []InventoryStack {
	out := make([]InventoryStack, 0, len(layers))
	for _, layer := range layers {
		if layer.ItemID == s.GhostItemID {
			continue
		}
		slot := world.WalletSlot
		if s.takesSlot(layer.ItemID) {
			slot = layer.Slot
		}
		out = append(out, InventoryStack{ItemID: layer.ItemID, Active: layer.Active, Quantity: layer.Quantity, Slot: slot})
	}
	return out
}
