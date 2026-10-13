package sim

import (
	"cyberia-server/game/content"
	"cyberia-server/game/world"
	"testing"
	"time"

	pb "cyberia-server/gen/proto"
)

// waitsAbout fails unless the entity respawns want after now.
func waitsAbout(t *testing.T, kind string, v *world.Vitals, now time.Time, want time.Duration) {
	t.Helper()
	if wait := v.RespawnAt.Sub(now); wait < want || wait > want+time.Second {
		t.Errorf("%s waits %v, want %v", kind, wait, want)
	}
}

func TestEachPlacedEntityWaitsItsOwnRespawnTime(t *testing.T) {
	s := lootServer(t, &pb.EntityTypeDefault{EntityType: "obstacle", LiveItemIds: []string{"crate"}, Behavior: content.BehaviorDestructible})
	buildWorld(s, []string{"m"},
		&pb.EntityMessage{EntityType: "bot", InitCellX: 1, InitCellY: 1, RespawnDurationMs: 7000},
		&pb.EntityMessage{EntityType: "resource", InitCellX: 5, InitCellY: 5, RespawnDurationMs: 8000},
		&pb.EntityMessage{EntityType: "obstacle", InitCellX: 10, InitCellY: 10, DimX: 1, DimY: 1,
			ObjectLayerItemIds: []string{"crate"}, RespawnDurationMs: 9000},
	)
	ms := s.Maps["m"]
	bots, resources, destructibles := ms.Count(world.EntityBot), ms.Count(world.EntityResource), countOf(ms.Destructibles())
	if bots != 1 || resources != 1 || destructibles != 1 {
		t.Fatalf("built %d bots, %d resources, %d destructibles; want one of each", bots, resources, destructibles)
	}
	player := woundedPlayer(s.World, ms, 103, world.Vec2{}, 0)
	player.MapCode = "m"

	now := time.Now()
	for bot := range ms.OfKind(world.EntityBot) {
		s.handleBotDeath(bot, ms)
		waitsAbout(t, "bot", ms.Vt(bot), now, 7*time.Second)
	}
	for res := range ms.OfKind(world.EntityResource) {
		s.handleResourceDeath(res, ms)
		waitsAbout(t, "resource", ms.Vt(res), now, 8*time.Second)
	}
	for obs := range ms.Destructibles() {
		s.handleDestructibleDeath(obs, ms)
		waitsAbout(t, "destructible", ms.Vt(obs), now, 9*time.Second)
	}
	s.handlePlayerDeath(player, ms)
	waitsAbout(t, "player", ms.Vt(player.ID), now, content.RespawnWait(&world.Mortal{}))
}

func TestAPlacedEntityWithoutARespawnTimeTakesTheDefault(t *testing.T) {
	s := lootServer(t)
	buildWorld(s, []string{"m"}, &pb.EntityMessage{EntityType: "resource", InitCellX: 5, InitCellY: 5})
	now := time.Now()
	for res := range s.Maps["m"].OfKind(world.EntityResource) {
		s.handleResourceDeath(res, s.Maps["m"])
		waitsAbout(t, "resource", s.Maps["m"].Vt(res), now, content.RespawnWait(&world.Mortal{}))
	}
}
