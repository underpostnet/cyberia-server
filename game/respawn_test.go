package game

import (
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

// waitsAbout fails unless the entity respawns want after now.
func waitsAbout(t *testing.T, kind string, v *Vitals, now time.Time, want time.Duration) {
	t.Helper()
	if wait := v.RespawnAt.Sub(now); wait < want || wait > want+time.Second {
		t.Errorf("%s waits %v, want %v", kind, wait, want)
	}
}

func TestEachPlacedEntityWaitsItsOwnRespawnTime(t *testing.T) {
	s := lootServer(t, &pb.EntityTypeDefault{EntityType: "obstacle", LiveItemIds: []string{"crate"}, Behavior: BehaviorDestructible})
	buildWorld(s, []string{"m"},
		&pb.EntityMessage{EntityType: "bot", InitCellX: 1, InitCellY: 1, RespawnDurationMs: 7000},
		&pb.EntityMessage{EntityType: "resource", InitCellX: 5, InitCellY: 5, RespawnDurationMs: 8000},
		&pb.EntityMessage{EntityType: "obstacle", InitCellX: 10, InitCellY: 10, DimX: 1, DimY: 1,
			ObjectLayerItemIds: []string{"crate"}, RespawnDurationMs: 9000},
	)
	ms := s.maps["m"]
	bots, resources, destructibles := ms.count(EntityBot), ms.count(EntityResource), countOf(ms.destructibles())
	if bots != 1 || resources != 1 || destructibles != 1 {
		t.Fatalf("built %d bots, %d resources, %d destructibles; want one of each", bots, resources, destructibles)
	}
	player := woundedPlayer(s.World, ms, 103, Vec2{}, 0)
	player.MapCode = "m"

	now := time.Now()
	for bot := range ms.ofKind(EntityBot) {
		s.handleBotDeath(bot, ms)
		waitsAbout(t, "bot", ms.vt(bot), now, 7*time.Second)
	}
	for res := range ms.ofKind(EntityResource) {
		s.handleResourceDeath(res, ms)
		waitsAbout(t, "resource", ms.vt(res), now, 8*time.Second)
	}
	for obs := range ms.destructibles() {
		s.handleDestructibleDeath(obs, ms)
		waitsAbout(t, "destructible", ms.vt(obs), now, 9*time.Second)
	}
	s.handlePlayerDeath(player, ms)
	waitsAbout(t, "player", ms.vt(player.ID), now, respawnDurationDefaultMs*time.Millisecond)
}

func TestAPlacedEntityWithoutARespawnTimeTakesTheDefault(t *testing.T) {
	s := lootServer(t)
	buildWorld(s, []string{"m"}, &pb.EntityMessage{EntityType: "resource", InitCellX: 5, InitCellY: 5})
	now := time.Now()
	for res := range s.maps["m"].ofKind(EntityResource) {
		s.handleResourceDeath(res, s.maps["m"])
		waitsAbout(t, "resource", s.maps["m"].vt(res), now, respawnDurationDefaultMs*time.Millisecond)
	}
}
