package worldtest

import (
	"cyberia-server/game/world"
)

// Map returns the map "test" of s, made on first use.
func Map(s *world.World) *world.MapState {
	if s.Maps == nil {
		s.Maps = map[string]*world.MapState{}
	}
	if s.Maps["test"] == nil {
		s.Maps["test"] = world.NewMapState("test", 0, 0)
	}
	return s.Maps["test"]
}

// PlacePlayer puts a player that holds layers on the map "test" of s.
func PlacePlayer(s *world.World, id world.EntityID, layers []world.ObjectLayerState) *world.Session {
	ms := Map(s)
	p := &world.Session{ID: id, MapCode: "test"}
	if s.Sessions == nil {
		s.Sessions = map[world.EntityID]*world.Session{}
	}
	s.Sessions[id] = p
	ms.Spawn(id, world.EntityData{Kind: world.EntityPlayer}, world.Transform{}, world.Appearance{Layers: layers})
	ms.AddVitals(id, world.Vitals{})
	ms.AddMortal(id, &world.Mortal{})
	ms.AddPlayer(id)
	return p
}

// PlaceBot puts a bot with vitals v on the map "test" of s.
func PlaceBot(s *world.World, id world.EntityID, v world.Vitals) world.EntityID {
	ms := Map(s)
	ms.Spawn(id, world.EntityData{Kind: world.EntityBot}, world.Transform{}, world.Appearance{})
	ms.AddVitals(id, v)
	ms.AddMortal(id, &world.Mortal{})
	ms.AddBrain(id, &world.Brain{})
	return id
}
