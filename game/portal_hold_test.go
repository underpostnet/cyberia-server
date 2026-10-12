package game

import (
	"testing"
	"time"
)

// holdingFixture is the movement fixture with a 2 s portal hold and a second
// player in view that has stood on a portal for 1 s.
func holdingFixture(t *testing.T) (*GameServer, *MapState, *PlayerState, *PlayerState) {
	t.Helper()
	s, ms, viewer := movementFixture(t)
	s.portalHoldTime = 2 * time.Second
	viewer.AOI = Rectangle{MaxX: 20, MaxY: 20}
	traveler := &PlayerState{}
	traveler.ID = 300
	traveler.Pos = Point{X: 5, Y: 5}
	traveler.Dims = Dimensions{Width: 1, Height: 1}
	traveler.Life, traveler.MaxLife = 100, 100
	traveler.OnPortal = true
	traveler.TimeOnPortal = time.Now().Add(-time.Second)
	ms.players[traveler.ID] = traveler
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
