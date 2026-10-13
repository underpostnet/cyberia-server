package world

import (
	"iter"
	"maps"
	"strconv"
)

// String lets a log print an id with %s.
func (id EntityID) String() string { return strconv.FormatUint(uint64(id), 10) }

// NewEntityID returns the next entity id. Call it under s.mu.
func (s *World) NewEntityID() EntityID {
	s.lastEntityID++
	return s.lastEntityID
}

// NewMapState returns an empty map of w×h cells.
func NewMapState(code string, w, h int) *MapState {
	return &MapState{code: code, gridW: w, gridH: h, pathfinder: NewPathfinder(w, h)}
}

// Code returns the map code.
func (ms *MapState) Code() string { return ms.code }

// GridW returns the grid width in cells.
func (ms *MapState) GridW() int { return ms.gridW }

// GridH returns the grid height in cells.
func (ms *MapState) GridH() int { return ms.gridH }

// Pathfinder returns the pathfinder over the grid.
func (ms *MapState) Pathfinder() *Pathfinder { return ms.pathfinder }

// Spawn gives id a slot with its universal components and returns the slot.
// Only the owner of the entity lifetime calls it: the loader, the spawner and
// the portal.
func (ms *MapState) Spawn(id EntityID, data EntityData, tf Transform, ap Appearance) int {
	if ms.index == nil {
		ms.index = make(map[EntityID]int)
	}
	// ponytail: linear scan from a cursor, a free-list head if a profile says so.
	slot := -1
	for n := 0; n < len(ms.ids); n++ {
		i := (ms.cursor + n) % len(ms.ids)
		if ms.ids[i] == 0 {
			slot = i
			break
		}
	}
	if slot < 0 {
		slot = len(ms.ids)
		ms.ids = append(ms.ids, 0)
		ms.mask = append(ms.mask, 0)
		ms.data = append(ms.data, EntityData{})
		ms.transform = append(ms.transform, Transform{})
		ms.appearance = append(ms.appearance, Appearance{})
		ms.vitals = append(ms.vitals, Vitals{})
		ms.motion = append(ms.motion, Motion{})
		ms.agent = append(ms.agent, Agent{})
		ms.portal = append(ms.portal, Portal{})
	}
	ms.ids[slot] = id
	ms.mask[slot] = MaskData | MaskTransform | MaskAppearance
	ms.data[slot] = data
	ms.transform[slot] = tf
	ms.appearance[slot] = ap
	ms.index[id] = slot
	ms.cursor = slot + 1
	return slot
}

// slot returns the slot of id. The id must have a slot in this map.
func (ms *MapState) slot(id EntityID) int {
	slot, ok := ms.index[id]
	if !ok {
		panic("entity " + id.String() + " has no slot")
	}
	return slot
}

// Tf returns the transform of id.
func (ms *MapState) Tf(id EntityID) *Transform { return &ms.transform[ms.slot(id)] }

// Ap returns the appearance of id.
func (ms *MapState) Ap(id EntityID) *Appearance { return &ms.appearance[ms.slot(id)] }

// Ap returns the appearance of an entity on the map mapCode.
func (s *World) Ap(id EntityID, mapCode string) *Appearance { return s.Maps[mapCode].Ap(id) }

// AddVitals gives id its vitals.
func (ms *MapState) AddVitals(id EntityID, v Vitals) {
	slot := ms.slot(id)
	ms.vitals[slot] = v
	ms.mask[slot] |= MaskVitals
}

// Vt returns the vitals of id. The id must have them.
func (ms *MapState) Vt(id EntityID) *Vitals {
	slot := ms.slot(id)
	if ms.mask[slot]&MaskVitals == 0 {
		panic("entity " + id.String() + " has no vitals")
	}
	return &ms.vitals[slot]
}

// AddMotion gives id its motion.
func (ms *MapState) AddMotion(id EntityID, m Motion) {
	slot := ms.slot(id)
	ms.motion[slot] = m
	ms.mask[slot] |= MaskMotion
}

// Mo returns the motion of id. The id must have it.
func (ms *MapState) Mo(id EntityID) *Motion {
	slot := ms.slot(id)
	if ms.mask[slot]&MaskMotion == 0 {
		panic("entity " + id.String() + " has no motion")
	}
	return &ms.motion[slot]
}

// AddAgent gives id its agent.
func (ms *MapState) AddAgent(id EntityID, a Agent) {
	slot := ms.slot(id)
	ms.agent[slot] = a
	ms.mask[slot] |= MaskAgent
}

// Ag returns the agent of id. The id must have it.
func (ms *MapState) Ag(id EntityID) *Agent {
	slot := ms.slot(id)
	if ms.mask[slot]&MaskAgent == 0 {
		panic("entity " + id.String() + " has no agent")
	}
	return &ms.agent[slot]
}

// ag returns the agent of a bot on the map mapCode.
func (s *World) ag(id EntityID, mapCode string) *Agent { return s.Maps[mapCode].Ag(id) }

// AddPortal links portal id to its target.
func (ms *MapState) AddPortal(id EntityID, p Portal) {
	slot := ms.slot(id)
	ms.portal[slot] = p
	ms.mask[slot] |= MaskPortal
}

// Pt returns the target of portal id, and false while it is unlinked.
func (ms *MapState) Pt(id EntityID) (*Portal, bool) {
	slot := ms.slot(id)
	return &ms.portal[slot], ms.mask[slot]&MaskPortal != 0
}

// AddPlayer marks id as an entity a client controls.
func (ms *MapState) AddPlayer(id EntityID) { ms.mask[ms.slot(id)] |= MaskPlayer }

// IsKind reports whether id is on the map with kind k.
func (ms *MapState) IsKind(id EntityID, k string) bool {
	slot, ok := ms.index[id]
	return ok && ms.data[slot].Kind == k
}

// Has reports whether id has every component in m.
func (ms *MapState) Has(id EntityID, m ComponentMask) bool {
	return ms.mask[ms.slot(id)]&m == m
}

// AddMortal gives id its mortal row.
func (ms *MapState) AddMortal(id EntityID, m *Mortal) {
	if ms.mortals == nil {
		ms.mortals = make(map[EntityID]*Mortal)
	}
	ms.mortals[id] = m
}

// Mt returns the mortal row of id. The id must have one.
func (ms *MapState) Mt(id EntityID) *Mortal {
	m, ok := ms.mortals[id]
	if !ok {
		panic("entity " + id.String() + " has no mortal row")
	}
	return m
}

// HasMortal reports whether id has a mortal row.
func (ms *MapState) HasMortal(id EntityID) bool { return ms.mortals[id] != nil }

// Mortals yields the id of every entity with a mortal row.
func (ms *MapState) Mortals() iter.Seq[EntityID] { return maps.Keys(ms.mortals) }

// AddBrain gives bot id its brain.
func (ms *MapState) AddBrain(id EntityID, b *Brain) {
	if ms.brains == nil {
		ms.brains = make(map[EntityID]*Brain)
	}
	ms.brains[id] = b
}

// Br returns the brain of bot id. The id must have one.
func (ms *MapState) Br(id EntityID) *Brain {
	b, ok := ms.brains[id]
	if !ok {
		panic("entity " + id.String() + " has no brain")
	}
	return b
}

// IDs yields the id of every entity, in slot order.
func (ms *MapState) IDs() iter.Seq[EntityID] {
	return func(yield func(EntityID) bool) {
		for _, id := range ms.ids {
			if id != 0 && !yield(id) {
				return
			}
		}
	}
}

// OfKind yields the id of every entity of kind k, in slot order.
func (ms *MapState) OfKind(k string) iter.Seq[EntityID] {
	return func(yield func(EntityID) bool) {
		for i, id := range ms.ids {
			if id != 0 && ms.data[i].Kind == k && !yield(id) {
				return
			}
		}
	}
}

// Destructibles yields every destructible obstacle: the scenery that has vitals.
func (ms *MapState) Destructibles() iter.Seq[EntityID] {
	return func(yield func(EntityID) bool) {
		for i, id := range ms.ids {
			if id != 0 && ms.data[i].Kind == EntityScenery && ms.mask[i]&MaskVitals != 0 && !yield(id) {
				return
			}
		}
	}
}

// PlayersOn yields the session of every player on ms, in slot order.
func (s *World) PlayersOn(ms *MapState) iter.Seq[*Session] {
	return func(yield func(*Session) bool) {
		for id := range ms.OfKind(EntityPlayer) {
			if !yield(s.Sessions[id]) {
				return
			}
		}
	}
}

// PlayerOn returns the session of id when id is a player on ms, else nil.
func (s *World) PlayerOn(ms *MapState, id EntityID) *Session {
	if slot, ok := ms.index[id]; ok && ms.data[slot].Kind == EntityPlayer {
		return s.Sessions[id]
	}
	return nil
}

// Count returns the number of entities of kind k.
func (ms *MapState) Count(k string) int {
	n := 0
	for range ms.OfKind(k) {
		n++
	}
	return n
}

// Vt returns the vitals of an entity on the map mapCode.
func (s *World) Vt(id EntityID, mapCode string) *Vitals { return s.Maps[mapCode].Vt(id) }

// SceneryClass names a scenery entity by its flags, for logs and the dashboard.
func SceneryClass(d EntityData) string {
	switch {
	case d.DrawOrder == DrawFloor:
		return "floor"
	case d.DrawOrder == DrawForeground:
		return "foreground"
	case d.Flags&FlagBlocks != 0:
		return "obstacle"
	}
	return "static"
}

// SceneryCount counts the scenery entities of one class.
func (ms *MapState) SceneryCount(class string) int {
	n := 0
	for i, id := range ms.ids {
		if id != 0 && ms.data[i].Kind == EntityScenery && SceneryClass(ms.data[i]) == class {
			n++
		}
	}
	return n
}

// Rect is the box the transform covers.
func (t *Transform) Rect() Rectangle {
	return Rectangle{MinX: t.Pos.X, MinY: t.Pos.Y, MaxX: t.Pos.X + t.Size.X, MaxY: t.Pos.Y + t.Size.Y}
}

// EntityRow is every component of one slot. Row leaves the side-table rows
// nil; take fills them for a move between maps.
type EntityRow struct {
	Mask       ComponentMask
	Data       EntityData
	Transform  Transform
	Appearance Appearance
	Vitals     Vitals
	Motion     Motion
	Agent      Agent
	Portal     Portal
	Mortal     *Mortal
	Brain      *Brain
}

// Row copies the components of id.
func (ms *MapState) Row(id EntityID) EntityRow {
	i := ms.slot(id)
	return EntityRow{Mask: ms.mask[i], Data: ms.data[i], Transform: ms.transform[i], Appearance: ms.appearance[i],
		Vitals: ms.vitals[i], Motion: ms.motion[i], Agent: ms.agent[i], Portal: ms.portal[i]}
}

// Take copies the row of id and frees its slot.
func (ms *MapState) Take(id EntityID) EntityRow {
	row := ms.Row(id)
	row.Mortal, row.Brain = ms.mortals[id], ms.brains[id]
	ms.Despawn(id)
	return row
}

// Put gives id a slot that holds row.
func (ms *MapState) Put(id EntityID, row EntityRow) {
	i := ms.Spawn(id, row.Data, row.Transform, row.Appearance)
	ms.vitals[i] = row.Vitals
	ms.motion[i] = row.Motion
	ms.agent[i] = row.Agent
	ms.portal[i] = row.Portal
	ms.mask[i] = row.Mask
	if row.Mortal != nil {
		ms.AddMortal(id, row.Mortal)
	}
	if row.Brain != nil {
		ms.AddBrain(id, row.Brain)
	}
}

// Despawn frees the slot of id. The component arrays keep stale values; the
// clear mask makes them unreadable.
func (ms *MapState) Despawn(id EntityID) {
	slot, ok := ms.index[id]
	if !ok {
		return
	}
	ms.ids[slot] = 0
	ms.mask[slot] = 0
	delete(ms.index, id)
	delete(ms.mortals, id)
	delete(ms.brains, id)
}
