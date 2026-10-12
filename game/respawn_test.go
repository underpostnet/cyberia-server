package game

import (
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

// waitsAbout fails unless the mortal respawns want after now.
func waitsAbout(t *testing.T, kind string, mortal *Mortal, now time.Time, want time.Duration) {
	t.Helper()
	if wait := mortal.RespawnTime.Sub(now); wait < want || wait > want+time.Second {
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
	if len(ms.bots) != 1 || len(ms.resources) != 1 || len(ms.destructibles) != 1 {
		t.Fatalf("built %d bots, %d resources, %d destructibles; want one of each",
			len(ms.bots), len(ms.resources), len(ms.destructibles))
	}
	player := woundedPlayer(ms, 103, Vec2{}, 0)
	player.MapCode = "m"

	now := time.Now()
	for _, bot := range ms.bots {
		s.handleBotDeath(bot, ms)
		waitsAbout(t, "bot", &bot.Mortal, now, 7*time.Second)
	}
	for _, res := range ms.resources {
		s.handleResourceDeath(res, ms)
		waitsAbout(t, "resource", &res.Mortal, now, 8*time.Second)
	}
	for _, obs := range ms.destructibles {
		s.handleDestructibleDeath(obs, ms)
		waitsAbout(t, "destructible", &obs.Mortal, now, 9*time.Second)
	}
	s.handlePlayerDeath(player, ms)
	waitsAbout(t, "player", &player.Mortal, now, respawnDurationDefaultMs*time.Millisecond)
}

func TestAPlacedEntityWithoutARespawnTimeTakesTheDefault(t *testing.T) {
	s := lootServer(t)
	buildWorld(s, []string{"m"}, &pb.EntityMessage{EntityType: "resource", InitCellX: 5, InitCellY: 5})
	now := time.Now()
	for _, res := range s.maps["m"].resources {
		s.handleResourceDeath(res, s.maps["m"])
		waitsAbout(t, "resource", &res.Mortal, now, respawnDurationDefaultMs*time.Millisecond)
	}
}
