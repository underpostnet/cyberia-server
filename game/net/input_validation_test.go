package net

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

// A chat line names a known channel and carries clean text; only a direct
// line names a target.
func TestParseInputValidatesChat(t *testing.T) {
	cmd, ok := parseInput("chat", &inputPayload{Channel: "party", Text: " hi\tall "})
	if !ok || cmd.ChatChannel != world.ChatParty || cmd.ChatText != "hi all" || cmd.EntityID != 0 {
		t.Fatalf("party line: ok=%v %+v", ok, cmd)
	}
	if cmd, ok := parseInput("chat", &inputPayload{Channel: "direct", ToID: 4, Text: "psst"}); !ok || cmd.EntityID != 4 {
		t.Fatalf("direct line: ok=%v %+v", ok, cmd)
	}
	for name, p := range map[string]*inputPayload{
		"unknown channel":   {Channel: "guild", Text: "hi"},
		"no channel":        {Text: "hi"},
		"blank text":        {Channel: "world", Text: " \n "},
		"direct, no target": {Channel: "direct", Text: "hi"},
		"world, a target":   {Channel: "world", ToID: 4, Text: "hi"},
	} {
		if _, ok := parseInput("chat", p); ok {
			t.Fatalf("%s: must be rejected", name)
		}
	}
}

func TestParseInputNamesThePartyTarget(t *testing.T) {
	if cmd, ok := parseInput("party_invite", &inputPayload{ToID: 9}); !ok || cmd.EntityID != 9 {
		t.Fatalf("invite: ok=%v %+v", ok, cmd)
	}
	if cmd, ok := parseInput("party_reply", &inputPayload{FromID: 9, Accept: true}); !ok || cmd.EntityID != 9 || !cmd.Active {
		t.Fatalf("reply: ok=%v %+v", ok, cmd)
	}
	for _, kind := range []string{"party_invite", "party_kick", "party_reply"} {
		if _, ok := parseInput(kind, &inputPayload{}); ok {
			t.Fatalf("%s without a player must be rejected", kind)
		}
	}
	for _, kind := range []string{"party_leave", "party_disband"} {
		if _, ok := parseInput(kind, &inputPayload{}); !ok {
			t.Fatalf("%s needs no field", kind)
		}
	}
}
