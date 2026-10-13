package game

import (
	"iter"
	"maps"
	"strconv"
)

// String lets a log print an id with %s.
func (id EntityID) String() string { return strconv.FormatUint(uint64(id), 10) }

// newEntityID returns the next entity id. Call it under s.mu.
func (s *World) newEntityID() EntityID {
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

// spawn gives id a slot with its universal components and returns the slot.
// Only the owner of the entity lifetime calls it: the loader, the spawner and
// the portal.
func (ms *MapState) spawn(id EntityID, data EntityData, tf Transform, ap Appearance) int {
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
	ms.mask[slot] = maskData | maskTransform | maskAppearance
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

// tf returns the transform of id.
func (ms *MapState) tf(id EntityID) *Transform { return &ms.transform[ms.slot(id)] }

// ap returns the appearance of id.
func (ms *MapState) ap(id EntityID) *Appearance { return &ms.appearance[ms.slot(id)] }

// ap returns the appearance of an entity on the map mapCode.
func (s *World) ap(id EntityID, mapCode string) *Appearance { return s.maps[mapCode].ap(id) }

// addVitals gives id its vitals.
func (ms *MapState) addVitals(id EntityID, v Vitals) {
	slot := ms.slot(id)
	ms.vitals[slot] = v
	ms.mask[slot] |= maskVitals
}

// vt returns the vitals of id. The id must have them.
func (ms *MapState) vt(id EntityID) *Vitals {
	slot := ms.slot(id)
	if ms.mask[slot]&maskVitals == 0 {
		panic("entity " + id.String() + " has no vitals")
	}
	return &ms.vitals[slot]
}

// addMotion gives id its motion.
func (ms *MapState) addMotion(id EntityID, m Motion) {
	slot := ms.slot(id)
	ms.motion[slot] = m
	ms.mask[slot] |= maskMotion
}

// mo returns the motion of id. The id must have it.
func (ms *MapState) mo(id EntityID) *Motion {
	slot := ms.slot(id)
	if ms.mask[slot]&maskMotion == 0 {
		panic("entity " + id.String() + " has no motion")
	}
	return &ms.motion[slot]
}

// addAgent gives id its agent.
func (ms *MapState) addAgent(id EntityID, a Agent) {
	slot := ms.slot(id)
	ms.agent[slot] = a
	ms.mask[slot] |= maskAgent
}

// ag returns the agent of id. The id must have it.
func (ms *MapState) ag(id EntityID) *Agent {
	slot := ms.slot(id)
	if ms.mask[slot]&maskAgent == 0 {
		panic("entity " + id.String() + " has no agent")
	}
	return &ms.agent[slot]
}

// ag returns the agent of a bot on the map mapCode.
func (s *World) ag(id EntityID, mapCode string) *Agent { return s.maps[mapCode].ag(id) }

// addPortal links portal id to its target.
func (ms *MapState) addPortal(id EntityID, p Portal) {
	slot := ms.slot(id)
	ms.portal[slot] = p
	ms.mask[slot] |= maskPortal
}

// pt returns the target of portal id, and false while it is unlinked.
func (ms *MapState) pt(id EntityID) (*Portal, bool) {
	slot := ms.slot(id)
	return &ms.portal[slot], ms.mask[slot]&maskPortal != 0
}

// AddPlayer marks id as an entity a client controls.
func (ms *MapState) AddPlayer(id EntityID) { ms.mask[ms.slot(id)] |= maskPlayer }

// IsKind reports whether id is on the map with kind k.
func (ms *MapState) IsKind(id EntityID, k string) bool {
	slot, ok := ms.index[id]
	return ok && ms.data[slot].Kind == k
}

// has reports whether id has every component in m.
func (ms *MapState) has(id EntityID, m ComponentMask) bool {
	return ms.mask[ms.slot(id)]&m == m
}

// addMortal gives id its mortal row.
func (ms *MapState) addMortal(id EntityID, m *Mortal) {
	if ms.mortals == nil {
		ms.mortals = make(map[EntityID]*Mortal)
	}
	ms.mortals[id] = m
}

// mt returns the mortal row of id. The id must have one.
func (ms *MapState) mt(id EntityID) *Mortal {
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

// addBrain gives bot id its brain.
func (ms *MapState) addBrain(id EntityID, b *Brain) {
	if ms.brains == nil {
		ms.brains = make(map[EntityID]*Brain)
	}
	ms.brains[id] = b
}

// br returns the brain of bot id. The id must have one.
func (ms *MapState) br(id EntityID) *Brain {
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

// ofKind yields the id of every entity of kind k, in slot order.
func (ms *MapState) ofKind(k string) iter.Seq[EntityID] {
	return func(yield func(EntityID) bool) {
		for i, id := range ms.ids {
			if id != 0 && ms.data[i].Kind == k && !yield(id) {
				return
			}
		}
	}
}

// destructibles yields every destructible obstacle: the scenery that has vitals.
func (ms *MapState) destructibles() iter.Seq[EntityID] {
	return func(yield func(EntityID) bool) {
		for i, id := range ms.ids {
			if id != 0 && ms.data[i].Kind == EntityScenery && ms.mask[i]&maskVitals != 0 && !yield(id) {
				return
			}
		}
	}
}

// playersOn yields the session of every player on ms, in slot order.
func (s *World) playersOn(ms *MapState) iter.Seq[*Session] {
	return func(yield func(*Session) bool) {
		for id := range ms.ofKind(EntityPlayer) {
			if !yield(s.sessions[id]) {
				return
			}
		}
	}
}

// playerOn returns the session of id when id is a player on ms, else nil.
func (s *World) playerOn(ms *MapState, id EntityID) *Session {
	if slot, ok := ms.index[id]; ok && ms.data[slot].Kind == EntityPlayer {
		return s.sessions[id]
	}
	return nil
}

// count returns the number of entities of kind k.
func (ms *MapState) count(k string) int {
	n := 0
	for range ms.ofKind(k) {
		n++
	}
	return n
}

// vt returns the vitals of an entity on the map mapCode.
func (s *World) vt(id EntityID, mapCode string) *Vitals { return s.maps[mapCode].vt(id) }

// sceneryClass names a scenery entity by its flags, for logs and the dashboard.
func sceneryClass(d EntityData) string {
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

// sceneryCount counts the scenery entities of one class.
func (ms *MapState) sceneryCount(class string) int {
	n := 0
	for i, id := range ms.ids {
		if id != 0 && ms.data[i].Kind == EntityScenery && sceneryClass(ms.data[i]) == class {
			n++
		}
	}
	return n
}

// rect is the box the transform covers.
func (t *Transform) rect() Rectangle {
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

// take copies the row of id and frees its slot.
func (ms *MapState) take(id EntityID) EntityRow {
	row := ms.Row(id)
	row.Mortal, row.Brain = ms.mortals[id], ms.brains[id]
	ms.despawn(id)
	return row
}

// put gives id a slot that holds row.
func (ms *MapState) put(id EntityID, row EntityRow) {
	i := ms.spawn(id, row.Data, row.Transform, row.Appearance)
	ms.vitals[i] = row.Vitals
	ms.motion[i] = row.Motion
	ms.agent[i] = row.Agent
	ms.portal[i] = row.Portal
	ms.mask[i] = row.Mask
	if row.Mortal != nil {
		ms.addMortal(id, row.Mortal)
	}
	if row.Brain != nil {
		ms.addBrain(id, row.Brain)
	}
}

// despawn frees the slot of id. The component arrays keep stale values; the
// clear mask makes them unreadable.
func (ms *MapState) despawn(id EntityID) {
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
