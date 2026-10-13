package net

import (
	"testing"
	"time"

	"cyberia-server/game/world"
	"cyberia-server/game/world/worldtest"
)

// holdingFixture is a server with a 2 s portal hold, a viewer, and a second
// player in view that has stood on a portal for 1 s.
func holdingFixture(t *testing.T) (*GameServer, *world.MapState, *world.Session, *world.Session) {
	t.Helper()
	s := NewGameServer()
	s.PortalHoldTime = 2 * time.Second
	viewer := worldtest.PlacePlayer(s.World, 1, nil)
	viewer.AOI = world.Rectangle{MaxX: 20, MaxY: 20}
	traveler := worldtest.PlacePlayer(s.World, 300, nil)
	ms := worldtest.Map(s.World)
	*ms.Tf(traveler.ID) = world.Transform{Pos: world.Vec2{X: 5, Y: 5}, Size: world.Vec2{X: 1, Y: 1}}
	*ms.Vt(traveler.ID) = world.Vitals{Life: 100, MaxLife: 100}
	traveler.OnPortal = true
	traveler.TimeOnPortal = time.Now().Add(-time.Second)
	return s, ms, viewer, traveler
}

func TestEveryViewerSeesThePortalHoldOfAPlayer(t *testing.T) {
	s, ms, viewer, traveler := holdingFixture(t)

	for _, e := range s.buildSnapshot(viewer, ms).Entities {
		if e.ID != traveler.ID {
			continue
		}
		if !e.OnPortal || e.PortalHoldProgress < 0.45 || e.PortalHoldProgress > 0.55 {
			t.Fatalf("traveler hold = %v %v, want on a portal at half the charge", e.OnPortal, e.PortalHoldProgress)
		}
		return
	}
	t.Fatal("the traveler is missing from the snapshot")
}

func TestThePortalHoldOfThePlayerItselfIsFullAtMostAndAbsentOffAPortal(t *testing.T) {
	s, ms, _, traveler := holdingFixture(t)
	traveler.TimeOnPortal = time.Now().Add(-time.Minute)
	if self := s.buildSnapshotSelf(traveler, ms); !self.OnPortal || self.PortalHoldProgress != 1 {
		t.Fatalf("self hold = %v %v, want a full charge", self.OnPortal, self.PortalHoldProgress)
	}

	traveler.OnPortal = false
	if self := s.buildSnapshotSelf(traveler, ms); self.OnPortal || self.PortalHoldProgress != 0 {
		t.Fatalf("self hold = %v %v, want none off a portal", self.OnPortal, self.PortalHoldProgress)
	}
}
