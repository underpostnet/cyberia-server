// Package content — durable.go
//
// The durable state of an account player: what a commit carries and what a
// session loads. Coins, total XP, the bag, the personal storage, the NPC
// vaults, the held stacks, the quests and the position.
//
// Every owned stack keeps the definition (CID) it was granted with. At load a
// stack whose label now names another definition, or no definition, is held:
// it stays in the profile untouched and never enters the world.
//
// Caller MUST hold s.mu for every method here.
package content

import (
	"cmp"
	"slices"
	"time"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
)

// Durability is the durable side of the account players of this server; nil
// in anonymous mode.
type Durability interface {
	// Capture records the newest state of a player and returns its mark.
	Capture(playerID string, state persist.State, commitPosition bool) persist.Mark
	// Committed reports whether the engine applied a marked state of a player.
	Committed(playerID string, mark persist.Mark) bool
}

// CaptureNow records the state of an account player at once and returns its
// mark; zero for a guest.
func (s *Content) CaptureNow(player *world.Session, commitPosition bool) persist.Mark {
	if s.Durability == nil || player.AccountID == "" {
		return persist.Mark{}
	}
	return s.Durability.Capture(string(player.PlayerID), s.DurableState(player), commitPosition)
}

// DropReleased reports whether a token may be collected: a token made of what
// a player lost waits until the engine applied that loss.
func (s *Content) DropReleased(drop *world.Brain) bool {
	lock := drop.Lock
	return lock.PlayerID == "" || s.Durability == nil || s.Durability.Committed(lock.PlayerID, lock.Mark)
}

// LossLock is the lock of the coins a player loses now: its state is captured
// at once. Zero for a guest.
func (s *Content) LossLock(player *world.Session) world.DropLock {
	mark := s.CaptureNow(player, false)
	if mark.Seq == 0 {
		return world.DropLock{}
	}
	return world.DropLock{PlayerID: string(player.PlayerID), Mark: mark}
}

// DefinitionCID is the definition the world binds to a label, or "" when the
// world has no such label.
func (s *Content) DefinitionCID(itemID string) string {
	if ol, ok := s.GetObjectLayerData(itemID); ok {
		return ol.Cid
	}
	return ""
}

func durableStack(itemID, cid string, qty, slot int, active bool) persist.Stack {
	return persist.Stack{ItemID: itemID, CID: cid, Quantity: qty, Slot: slot, Active: active}
}

func bySlot(a, b persist.Stack) int { return cmp.Compare(a.Slot, b.Slot) }

func durableStacks(stacks []Stack) []persist.Stack {
	out := make([]persist.Stack, 0, len(stacks))
	for _, st := range stacks {
		out = append(out, durableStack(st.ItemID, st.CID, st.Qty, st.Slot, false))
	}
	slices.SortFunc(out, bySlot)
	return out
}

// PositionOf is the durable position of a player: its map and its place on it.
func (s *Content) PositionOf(player *world.Session) persist.Position {
	pos := s.Maps[player.MapCode].Tf(player.ID).Pos
	return persist.Position{MapCode: player.MapCode, X: pos.X, Y: pos.Y}
}

// DurableState is the whole durable state of a player.
func (s *Content) DurableState(player *world.Session) persist.State {
	ms := s.Maps[player.MapCode]
	state := persist.State{
		Coins:    player.Coins,
		XP:       s.EntityProgression(player.ID, ms).XP,
		Storage:  durableStacks(s.PersonalStorage[player.PlayerID]),
		Held:     slices.Clone(s.Held[player.PlayerID]),
		Position: s.PositionOf(player),
	}
	for _, layer := range *s.InventoryLayers(player) {
		if s.takesSlot(layer.ItemID) && !s.DeadItemIDs[layer.ItemID] && layer.Quantity > 0 {
			state.Bag = append(state.Bag, durableStack(layer.ItemID, layer.CID, layer.Quantity, layer.Slot, layer.Active))
		}
	}
	slices.SortFunc(state.Bag, bySlot)
	for code, stacks := range s.Vaults[player.PlayerID] {
		if len(stacks) > 0 {
			state.Vaults = append(state.Vaults, persist.Vault{ActionCode: code, Stacks: durableStacks(stacks)})
		}
	}
	slices.SortFunc(state.Vaults, func(a, b persist.Vault) int { return cmp.Compare(a.ActionCode, b.ActionCode) })
	for _, qp := range s.quests[player.ID] {
		if qp.Status == "active" || qp.Status == "completed" {
			state.Quests = append(state.Quests, durableQuest(qp))
		}
	}
	state.Quests = append(state.Quests, s.dormantQuests[player.PlayerID]...)
	slices.SortFunc(state.Quests, func(a, b persist.Quest) int { return cmp.Compare(a.Code, b.Code) })
	return state
}

func durableQuest(qp *QuestProgress) persist.Quest {
	q := persist.Quest{Code: qp.QuestCode, Status: qp.Status, StartedAtMs: qp.StartedAt.UnixMilli()}
	if !qp.CompletedAt.IsZero() {
		q.CompletedAtMs = qp.CompletedAt.UnixMilli()
	}
	for _, sp := range qp.Steps {
		step := persist.QuestStep{StepID: sp.StepID}
		for _, op := range sp.Objectives {
			step.Objectives = append(step.Objectives, persist.Objective{Current: op.Current, Required: op.Required})
		}
		q.Steps = append(q.Steps, step)
	}
	return q
}

// holdReason is why a saved stack cannot load into this world, or "".
func (s *Content) holdReason(stack persist.Stack) string {
	switch cid := s.DefinitionCID(stack.ItemID); {
	case cid == "" && s.unavailable[stack.ItemID] != "":
		return s.unavailable[stack.ItemID]
	case cid == "":
		return persist.HeldUnknown
	case cid != stack.CID:
		return persist.HeldRebound
	}
	return ""
}

// seat places loaded stacks into a container of slots cells. A stack keeps
// its slot when the slot is open and free, else takes the lowest free one. A
// stack the world cannot load, or that finds no slot, is returned as held.
func (s *Content) seat(stacks []persist.Stack, slots int, container, actionCode string) (seated []persist.Stack, held []persist.Held) {
	taken := make(map[int]bool)
	var homeless []persist.Stack
	for _, st := range stacks {
		if reason := s.holdReason(st); reason != "" {
			held = append(held, persist.Held{Stack: st, Container: container, ActionCode: actionCode, Reason: reason})
			continue
		}
		if st.Slot >= 0 && st.Slot < slots && !taken[st.Slot] {
			taken[st.Slot] = true
			seated = append(seated, st)
			continue
		}
		homeless = append(homeless, st)
	}
	free := 0
	for _, st := range homeless {
		for taken[free] {
			free++
		}
		if free >= slots {
			held = append(held, persist.Held{Stack: st, Container: container, ActionCode: actionCode, Reason: persist.HeldNoRoom})
			continue
		}
		st.Slot, taken[free] = free, true
		seated = append(seated, st)
	}
	return seated, held
}

func containerStacks(stacks []persist.Stack) []Stack {
	out := make([]Stack, 0, len(stacks))
	for _, st := range stacks {
		out = append(out, Stack{ItemID: st.ItemID, CID: st.CID, Qty: st.Quantity, Slot: st.Slot})
	}
	return out
}

// vaultSlots is the capacity of the vault of an action code in this world, 0
// when no terminal offers it.
func (s *Content) vaultSlots(actionCode string) int {
	for _, action := range s.actionCache {
		if action.Code == actionCode {
			return storageCapacity(action.StorageSlots)
		}
	}
	return 0
}

// RestoreState loads a saved state onto a player that was just spawned with
// its default stack. Held stacks whose definition loads again return to their
// container when it has room.
func (s *Content) RestoreState(player *world.Session, state persist.State) {
	ms := s.Maps[player.MapCode]
	rules := s.ProgressionConfig()
	s.SetProgression(player.ID, EntityProgression{Level: rules.LevelAtXP(state.XP), XP: state.XP})
	s.setCoins(player.ID, ms, state.Coins)

	byContainer := map[string][]persist.Stack{persist.ContainerBag: state.Bag, persist.ContainerStorage: state.Storage}
	vaults := make(map[string][]persist.Stack, len(state.Vaults))
	for _, v := range state.Vaults {
		vaults[v.ActionCode] = v.Stacks
	}
	var held []persist.Held
	for _, h := range state.Held {
		switch {
		case s.holdReason(h.Stack) != "":
			held = append(held, h)
		case h.Container == persist.ContainerVault:
			vaults[h.ActionCode] = append(vaults[h.ActionCode], h.Stack)
		default:
			byContainer[h.Container] = append(byContainer[h.Container], h.Stack)
		}
	}

	bag, out := s.seat(byContainer[persist.ContainerBag], s.InventorySlots(player), persist.ContainerBag, "")
	held = append(held, out...)
	layers := &ms.Ap(player.ID).Layers
	*layers = slices.DeleteFunc(*layers, func(l world.ObjectLayerState) bool { return s.takesSlot(l.ItemID) })
	for _, st := range bag {
		*layers = append(*layers, world.ObjectLayerState{ItemID: st.ItemID, CID: st.CID, Quantity: st.Quantity, Slot: st.Slot, Active: st.Active})
	}
	s.NormalizeLoadout(player)

	storage, out := s.seat(byContainer[persist.ContainerStorage], s.StorageSlots(player), persist.ContainerStorage, "")
	held = append(held, out...)
	if len(storage) > 0 {
		if s.PersonalStorage == nil {
			s.PersonalStorage = make(map[world.PlayerID][]Stack)
		}
		s.PersonalStorage[player.PlayerID] = containerStacks(storage)
	}
	for code, stacks := range vaults {
		seated, out := s.seat(stacks, s.vaultSlots(code), persist.ContainerVault, code)
		held = append(held, out...)
		if len(seated) == 0 {
			continue
		}
		if s.Vaults == nil {
			s.Vaults = make(map[world.PlayerID]map[string][]Stack)
		}
		if s.Vaults[player.PlayerID] == nil {
			s.Vaults[player.PlayerID] = make(map[string][]Stack)
		}
		s.Vaults[player.PlayerID][code] = containerStacks(seated)
	}
	s.setHeld(player.PlayerID, held)
	s.restoreQuests(player, state.Quests)
	s.ApplyResistanceStat(player.ID, ms)
}

func (s *Content) setHeld(id world.PlayerID, held []persist.Held) {
	if len(held) == 0 {
		delete(s.Held, id)
		return
	}
	if s.Held == nil {
		s.Held = make(map[world.PlayerID][]persist.Held)
	}
	s.Held[id] = held
}

// HeldStack is an owned stack the world cannot load now: its owner sees it by
// its definition, and cannot use it here.
type HeldStack struct {
	ItemID   string `json:"itemId"`
	CID      string `json:"cid"`
	Quantity int    `json:"quantity"`
	Reason   string `json:"reason"`
}

// HeldView is the held stacks of a player.
func (s *Content) HeldView(player *world.Session) []HeldStack {
	held := s.Held[player.PlayerID]
	out := make([]HeldStack, 0, len(held))
	for _, h := range held {
		out = append(out, HeldStack{ItemID: h.Stack.ItemID, CID: h.Stack.CID, Quantity: h.Stack.Quantity, Reason: h.Reason})
	}
	return out
}

// restoreQuests loads saved quest progress. The objectives take their type and
// target from the current definition; a quest the world no longer defines
// stays dormant and keeps its record.
func (s *Content) restoreQuests(player *world.Session, quests []persist.Quest) {
	s.ensurePlayerQuests(player)
	var dormant []persist.Quest
	for _, q := range quests {
		def, ok := s.questDefs[q.Code]
		if !ok {
			dormant = append(dormant, q)
			continue
		}
		qp := &QuestProgress{QuestCode: q.Code, Status: q.Status, StartedAt: time.UnixMilli(q.StartedAtMs)}
		if q.CompletedAtMs > 0 {
			qp.CompletedAt = time.UnixMilli(q.CompletedAtMs)
		}
		saved := make(map[string]persist.QuestStep, len(q.Steps))
		for _, step := range q.Steps {
			saved[step.StepID] = step
		}
		for _, step := range def.Steps {
			sp := QuestStepProgress{StepID: step.ID}
			for i, obj := range step.Objectives {
				op := QuestObjectiveProgress{Type: obj.Type, ItemID: obj.ItemID, Required: max(obj.Quantity, 1)}
				if objs := saved[step.ID].Objectives; i < len(objs) {
					op.Current = min(objs[i].Current, op.Required)
				}
				if q.Status == "completed" {
					op.Current = op.Required
				}
				sp.Objectives = append(sp.Objectives, op)
			}
			// A step settles its collect items in the tick it completes.
			sp.Consumed = stepComplete(&sp)
			qp.Steps = append(qp.Steps, sp)
		}
		s.quests[player.ID][q.Code] = qp
	}
	if len(dormant) > 0 {
		if s.dormantQuests == nil {
			s.dormantQuests = make(map[world.PlayerID][]persist.Quest)
		}
		s.dormantQuests[player.PlayerID] = dormant
	}
}

// RefundCraft pays back the ingredients of a running assembly when the
// session ends. A refund the bag cannot take is held.
func (s *Content) RefundCraft(player *world.Session) {
	active := s.crafts[player.ID]
	if active == nil {
		return
	}
	delete(s.crafts, player.ID)
	if s.ExchangeItems(player, nil, active.Refund) {
		return
	}
	held := s.Held[player.PlayerID]
	for _, st := range active.Refund {
		if st.ItemID == s.CoinItemID {
			s.AddCoins(player.ID, s.Maps[player.MapCode], st.Qty)
			continue
		}
		held = append(held, persist.Held{
			Stack:     durableStack(st.ItemID, s.DefinitionCID(st.ItemID), st.Qty, 0, false),
			Container: persist.ContainerBag,
			Reason:    persist.HeldNoRoom,
		})
	}
	s.setHeld(player.PlayerID, held)
}

// NormalizeLoadout makes the worn stacks of a just-loaded player follow the
// equipment rules: active types only, one worn stack per item and per type,
// at most MaxActiveLayers, and a skin when the rules require one. Without a
// skin in the bag, the player gets the skin of its default stack.
func (s *Content) NormalizeLoadout(player *world.Session) {
	ap := s.Maps[player.MapCode].Ap(player.ID)
	order := make([]int, 0, len(ap.Layers))
	for i := range ap.Layers {
		if s.takesSlot(ap.Layers[i].ItemID) {
			order = append(order, i)
		}
	}
	slices.SortFunc(order, func(a, b int) int { return cmp.Compare(ap.Layers[a].Slot, ap.Layers[b].Slot) })

	rules := s.EquipmentRules
	worn, items, types, skin := 0, map[string]bool{}, map[string]bool{}, false
	for _, i := range order {
		layer := &ap.Layers[i]
		if !layer.Active {
			continue
		}
		typ := s.ItemType(layer.ItemID)
		allowed := len(rules.ActiveItemTypes) == 0 || rules.ActiveItemTypes[typ]
		if !allowed || items[layer.ItemID] || (rules.OnePerType && typ != "" && types[typ]) ||
			(s.MaxActiveLayers > 0 && worn >= s.MaxActiveLayers) {
			layer.Active = false
			continue
		}
		items[layer.ItemID], types[typ], skin = true, true, skin || typ == "skin"
		worn++
	}
	if !rules.RequireSkin || skin {
		return
	}
	for _, i := range order {
		if s.ItemType(ap.Layers[i].ItemID) == "skin" {
			ap.Layers[i].Active = true
			return
		}
	}
	for _, layer := range s.EntityDefaults["player"].DefaultObjectLayers {
		if s.ItemType(layer.ItemID) == "skin" {
			ap.Layers = append(ap.Layers, world.ObjectLayerState{
				ItemID: layer.ItemID, CID: s.DefinitionCID(layer.ItemID), Quantity: 1, Slot: s.freeSlot(ap.Layers), Active: true,
			})
			return
		}
	}
}
