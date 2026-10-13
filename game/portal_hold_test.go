package game

import (
	"cyberia-server/game/world"
	"testing"
	"time"
)

// holdingFixture is the movement fixture with a 2 s portal hold and a second
// player in view that has stood on a portal for 1 s.
func holdingFixture(t *testing.T) (*GameServer, *world.MapState, *world.Session, *world.Session) {
	t.Helper()
	s, ms, viewer := movementFixture(t)
	s.portalHoldTime = 2 * time.Second
	viewer.AOI = world.Rectangle{MaxX: 20, MaxY: 20}
	traveler := &world.Session{ID: 300, MapCode: viewer.MapCode}
	traveler.OnPortal = true
	traveler.TimeOnPortal = time.Now().Add(-time.Second)
	s.Sessions[traveler.ID] = traveler
	ms.Spawn(traveler.ID, world.EntityData{Kind: world.EntityPlayer}, world.Transform{Pos: world.Vec2{X: 5, Y: 5}, Size: world.Vec2{X: 1, Y: 1}}, world.Appearance{})
	ms.AddVitals(traveler.ID, world.Vitals{Life: 100, MaxLife: 100})
	ms.AddMortal(traveler.ID, &world.Mortal{})
	return &GameServer{Sim: s}, ms, viewer, traveler
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
