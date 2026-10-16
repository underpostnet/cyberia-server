package sim

import (
	"testing"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/persist"
	"cyberia-server/game/world"
)

// fakeDurability applies a mark only once the test says so.
type fakeDurability struct {
	captured int
	applied  persist.Mark
}

func (f *fakeDurability) Capture(string, persist.State, bool) persist.Mark {
	f.captured++
	return persist.Mark{Epoch: 1, Seq: uint64(f.captured)}
}

func (f *fakeDurability) Committed(_ string, mark persist.Mark) bool {
	return mark.Epoch == f.applied.Epoch && mark.Seq <= f.applied.Seq
}

func TestCoinsAPlayerLostWaitForTheEngine(t *testing.T) {
	s, ms, collector := movementFixture(t)
	for range 1000 {
		s.NewEntityID()
	}
	s.CoinItemID = "coin"
	durability := &fakeDurability{}
	s.Durability = durability
	victim := woundedPlayer(s.World, ms, 108, world.Vec2{X: 15, Y: 15}, 50)
	victim.PlayerID, victim.AccountID, victim.MapCode = "prof-v", "acc-v", ms.Code()

	lock := s.LossLock(victim)
	if lock.PlayerID != "prof-v" || lock.Mark.Seq != 1 {
		t.Fatalf("a lost coin locks on the captured state of its owner: %+v", lock)
	}
	s.SpawnDrops(ms, world.Vec2{X: 10, Y: 10}, content.EntityTypeDefaultConfig{}, 7,
		map[world.EntityID]float64{collector.ID: 5}, lock)
	tokens := drops(ms)
	if len(tokens) != 1 {
		t.Fatalf("one coin token, got %d", len(tokens))
	}
	token := tokens[0]
	ms.Br(token).CollectableAt = time.Time{}
	ms.Tf(collector.ID).Pos = ms.Tf(token).Pos

	s.PhaseLoot(0, ms)
	if !ms.IsKind(token, world.EntityBot) || collector.Coins != 0 {
		t.Fatal("nobody takes the coins before the engine applied their loss")
	}
	durability.applied = lock.Mark
	s.PhaseLoot(0, ms)
	if ms.IsKind(token, world.EntityBot) || collector.Coins != 7 {
		t.Fatalf("the coins are collectable once the loss is applied: coins=%d", collector.Coins)
	}
}

func TestAnUnsavedPlayerMakesNoTrade(t *testing.T) {
	s, ms, player := movementFixture(t)
	player.Unsaved = true
	player.InputQueue = []world.InputCommand{{Kind: world.InputKindItemMove, Sequence: 1,
		Move: world.ItemMove{From: world.BoxBag, To: world.BoxStorage, Qty: 1}}}
	s.PhaseInput(0, ms)
	if player.InputConsumedThrough != 1 {
		t.Fatal("a refused trade is still consumed")
	}
	if len(s.PlayerStorage(player)) != 0 {
		t.Fatal("an unsaved player banks nothing")
	}
}
