package game

import (
	"testing"

	"cyberia-server/game/world"
)

func TestParseInputCarriesTheStasisBool(t *testing.T) {
	for _, want := range []bool{true, false} {
		cmd, ok := parseInput("player_stasis", &inputPayload{Stasis: want})
		if !ok || cmd.Kind != world.InputKindPlayerStasis || cmd.Active != want {
			t.Fatalf("player_stasis %v: got ok=%v %+v", want, ok, cmd)
		}
	}
}
