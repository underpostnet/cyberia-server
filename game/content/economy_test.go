package content

import (
	"math"
	"testing"

	"cyberia-server/game/world"
	"cyberia-server/game/world/worldtest"
	pb "cyberia-server/gen/proto"
)

// A wallet stays in [0, MaxUint32]: a gain past the top clamps and a loss past
// zero stops at zero. The coin stack shows the balance.
func TestAddCoinsClampsTheWallet(t *testing.T) {
	s := &Content{World: &world.World{}, CoinItemID: "coin"}
	player := worldtest.PlacePlayer(s.World, 1, nil)
	ms := worldtest.Map(s.World)
	player.Coins = math.MaxUint32 - 5
	if got := s.AddCoins(player.ID, ms, 10); got != math.MaxUint32 {
		t.Fatalf("balance = %d, want MaxUint32", got)
	}
	if got := s.AddCoins(player.ID, ms, -math.MaxUint32-10); got != 0 || LayerQuantity(ms.Ap(player.ID).Layers, "coin") != 0 {
		t.Fatalf("balance = %d, want 0", got)
	}
}

// A death drops its rate of the balance, at least the minimum, and never more
// than the balance. A bot drops of at least its spawn wallet. An explicit 0
// rate stays 0.
func TestKillDropFollowsTheVictimRules(t *testing.T) {
	s := &Content{World: &world.World{}, CoinItemID: "coin"}
	s.ApplyConfig(&pb.InstanceConfig{EconomyRules: &pb.EconomyRules{
		BotSpawnCoins: 50, CoinKillPercentVsBot: 0.4, CoinKillPercentVsPlayer: 0, CoinKillMinAmount: 10,
	}}, defaultProgressionRules())
	player := worldtest.PlacePlayer(s.World, 1, nil)
	ms := worldtest.Map(s.World)

	for _, c := range []struct {
		coins uint32
		want  int
	}{{0, 0}, {4, 4}, {100, 10}} {
		player.Coins = c.coins
		if got := s.PvpCoinDropAmount(player); got != c.want {
			t.Fatalf("player with %d coins drops %d, want %d", c.coins, got, c.want)
		}
	}
	bot := worldtest.PlaceBot(s.World, 2, world.Vitals{})
	ms.Br(bot).Coins = 0
	if got := s.BotCoinDropAmount(bot, ms); got != 20 {
		t.Fatalf("bot drop = %d, want 40%% of the 50 spawn coins", got)
	}
}

// A rule out of bounds fails validation, so a reload changes nothing.
func TestEconomyRulesRefuseValuesOutOfBounds(t *testing.T) {
	for _, rules := range []*pb.EconomyRules{
		{CoinKillPercentVsBot: 1.5},
		{CoinKillPercentVsPlayer: -0.1},
		{RespawnCostPercent: math.NaN()},
		{BotSpawnCoins: -1},
		{CoinKillMinAmount: 2000000},
	} {
		if EconomyRulesFromProto(rules).Validate() == nil {
			t.Fatalf("accepted %+v", rules)
		}
	}
	if err := EconomyRulesFromProto(nil).Validate(); err != nil {
		t.Fatalf("zero rules: %v", err)
	}
}
