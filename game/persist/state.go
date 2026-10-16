// Package persist commits the durable state of account players to the engine.
//
// The simulation captures the whole state of a player under the world lock.
// This package sends it outside the lock: one commit per player in flight, in
// sequence order, retried until the engine answers. A commit is the whole
// state, so a newer capture supersedes an older one that failed, and the queue
// never holds more than one state per player.
//
// The package knows no world types; ids are the engine's strings.
package persist

import (
	"reflect"
)

// Stack is one owned stack.
type Stack struct {
	ItemID   string
	CID      string
	Quantity int
	Slot     int
	Active   bool
}

// Vault is the private vault of a player at one NPC action.
type Vault struct {
	ActionCode string
	Stacks     []Stack
}

// Held is an owned stack the world cannot load now.
type Held struct {
	Stack      Stack
	Container  string // ContainerBag | ContainerStorage | ContainerVault
	ActionCode string // vault only
	Reason     string // HeldRebound | HeldUnknown | HeldNoRoom
}

// Containers and reasons of a held stack.
const (
	ContainerBag     = "bag"
	ContainerStorage = "storage"
	ContainerVault   = "vault"

	HeldRebound = "rebound" // the label names another definition now
	HeldUnknown = "unknown" // the world has no such label
	HeldNoRoom  = "no_room" // the container had no free slot at load
)

// Position is a cell on a map. An empty MapCode is no position.
type Position struct {
	MapCode string
	X, Y    float64
}

// Objective is one objective counter of a quest step.
type Objective struct {
	Current, Required int
}

// QuestStep is the objective counters of one step.
type QuestStep struct {
	StepID     string
	Objectives []Objective
}

// Quest is the progress of one quest.
type Quest struct {
	Code          string
	Status        string // active | completed
	Steps         []QuestStep
	StartedAtMs   int64
	CompletedAtMs int64
}

// State is the whole durable state of one player.
type State struct {
	Coins    uint32
	XP       uint64
	Bag      []Stack
	Storage  []Stack
	Vaults   []Vault
	Held     []Held
	Quests   []Quest
	Position Position
}

// canonical returns the state with every empty list nil, so two equal states
// compare equal whoever built them.
func (s State) canonical() State {
	s.Bag = canonicalStacks(s.Bag)
	s.Storage = canonicalStacks(s.Storage)
	if len(s.Vaults) == 0 {
		s.Vaults = nil
	}
	for i := range s.Vaults {
		s.Vaults[i].Stacks = canonicalStacks(s.Vaults[i].Stacks)
	}
	if len(s.Held) == 0 {
		s.Held = nil
	}
	if len(s.Quests) == 0 {
		s.Quests = nil
	}
	for i := range s.Quests {
		if len(s.Quests[i].Steps) == 0 {
			s.Quests[i].Steps = nil
		}
		for j := range s.Quests[i].Steps {
			if len(s.Quests[i].Steps[j].Objectives) == 0 {
				s.Quests[i].Steps[j].Objectives = nil
			}
		}
	}
	return s
}

func canonicalStacks(stacks []Stack) []Stack {
	if len(stacks) == 0 {
		return nil
	}
	return stacks
}

// sameGoods reports whether two canonical states hold the same possessions
// and progress. The position is compared apart: it alone never needs a commit.
func sameGoods(a, b State) bool {
	return a.Coins == b.Coins && a.XP == b.XP &&
		reflect.DeepEqual(a.Bag, b.Bag) && reflect.DeepEqual(a.Storage, b.Storage) &&
		reflect.DeepEqual(a.Vaults, b.Vaults) && reflect.DeepEqual(a.Held, b.Held) &&
		reflect.DeepEqual(a.Quests, b.Quests)
}
