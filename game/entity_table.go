package game

import "strconv"

// String lets a log print an id with %s.
func (id EntityID) String() string { return strconv.FormatUint(uint64(id), 10) }

// newEntityID returns the next entity id. Call it under s.mu.
func (s *GameServer) newEntityID() EntityID {
	s.lastEntityID++
	return s.lastEntityID
}

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
func (s *GameServer) ap(id EntityID, mapCode string) *Appearance { return s.maps[mapCode].ap(id) }

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
func (s *GameServer) ag(id EntityID, mapCode string) *Agent { return s.maps[mapCode].ag(id) }

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

// vt returns the vitals of an entity on the map mapCode.
func (s *GameServer) vt(id EntityID, mapCode string) *Vitals { return s.maps[mapCode].vt(id) }

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

// entityRow is every component of one slot, for a move between maps.
type entityRow struct {
	mask       ComponentMask
	data       EntityData
	transform  Transform
	appearance Appearance
	vitals     Vitals
	motion     Motion
	agent      Agent
	portal     Portal
}

// take copies the row of id and frees its slot.
func (ms *MapState) take(id EntityID) entityRow {
	i := ms.slot(id)
	row := entityRow{ms.mask[i], ms.data[i], ms.transform[i], ms.appearance[i], ms.vitals[i], ms.motion[i], ms.agent[i], ms.portal[i]}
	ms.despawn(id)
	return row
}

// put gives id a slot that holds row.
func (ms *MapState) put(id EntityID, row entityRow) {
	i := ms.spawn(id, row.data, row.transform, row.appearance)
	ms.vitals[i] = row.vitals
	ms.motion[i] = row.motion
	ms.agent[i] = row.agent
	ms.portal[i] = row.portal
	ms.mask[i] = row.mask
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
}
