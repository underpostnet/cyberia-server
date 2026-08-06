package serial

import (
	"encoding/json"
	"testing"
)

func TestEnvelopeRoundTrip(t *testing.T) {
	type action struct {
		X    float64 `json:"x"`
		Y    float64 `json:"y"`
		Tick uint32  `json:"tick"`
		Seq  uint32  `json:"seq"`
	}
	sent := action{X: 12.5, Y: -3, Tick: 42, Seq: 7}

	pack, err := Pack("player_action", sent)
	if err != nil {
		t.Fatalf("Pack: %v", err)
	}

	msg, err := Unpack(pack)
	if err != nil {
		t.Fatalf("Unpack: %v", err)
	}
	if msg.Type != "player_action" {
		t.Fatalf("type = %q, want player_action", msg.Type)
	}

	var got action
	if err := json.Unmarshal(msg.Payload, &got); err != nil {
		t.Fatalf("payload: %v", err)
	}
	if got != sent {
		t.Fatalf("payload = %+v, want %+v", got, sent)
	}
}
