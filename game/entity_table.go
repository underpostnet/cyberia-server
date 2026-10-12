package game

import "strconv"

// String lets a log print an id with %s.
func (id EntityID) String() string { return strconv.FormatUint(uint64(id), 10) }

// newEntityID returns the next entity id. Call it under s.mu.
func (s *GameServer) newEntityID() EntityID {
	s.lastEntityID++
	return s.lastEntityID
}

// spawn gives id a slot with its transform and returns the slot. Only the
// owner of the entity lifetime calls it: the loader, the spawner and the portal.
func (ms *MapState) spawn(id EntityID, tf Transform) int {
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
		ms.transform = append(ms.transform, Transform{})
	}
	ms.ids[slot] = id
	ms.mask[slot] = maskTransform
	ms.transform[slot] = tf
	ms.index[id] = slot
	ms.cursor = slot + 1
	return slot
}

// rect is the box the transform covers.
func (t *Transform) rect() Rectangle {
	return Rectangle{MinX: t.Pos.X, MinY: t.Pos.Y, MaxX: t.Pos.X + t.Size.X, MaxY: t.Pos.Y + t.Size.Y}
}

// tf returns the transform of id. The id must have a slot in this map.
func (ms *MapState) tf(id EntityID) *Transform {
	slot, ok := ms.index[id]
	if !ok {
		panic("entity " + id.String() + " has no slot")
	}
	return &ms.transform[slot]
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
