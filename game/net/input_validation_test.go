package net

import (
	"strings"
	"testing"

	"cyberia-server/game/content"
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
	if !ok || cmd.ChatChannel != world.ChatParty || cmd.Text != "hi all" || cmd.EntityID != 0 {
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

// The engine validates status notes without truncation.
func TestParseInputPreservesTheStatusNote(t *testing.T) {
	cmd, ok := parseInput("status_note", &inputPayload{Text: " out\n  farming "})
	if !ok || cmd.Kind != world.InputKindStatusNote || cmd.Text != " out\n  farming " {
		t.Fatalf("note: ok=%v %+v", ok, cmd)
	}
	cmd, _ = parseInput("status_note", &inputPayload{Text: strings.Repeat("x", content.StatusNoteMaxRunes+5)})
	if len(cmd.Text) != content.StatusNoteMaxRunes+5 {
		t.Fatalf("a long note reaches engine validation: bound=%d length=%d", content.StatusNoteMaxRunes, len(cmd.Text))
	}
	if cmd, ok := parseInput("status_note", &inputPayload{}); !ok || cmd.Text != "" {
		t.Fatalf("an empty note clears: ok=%v %+v", ok, cmd)
	}
	for _, text := range []string{"🌲", "e\u0301", "👩‍👩‍👧‍👦", strings.Repeat("🌲", 81)} {
		if got, ok := parseInput("status_note", &inputPayload{Text: text}); !ok || got.Text != text {
			t.Fatalf("note changed before engine validation: %q -> %q", text, got.Text)
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
