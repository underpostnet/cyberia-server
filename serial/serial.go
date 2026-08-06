// Package serial packs objects to bytes and unpacks bytes to objects.
// Every WebSocket message uses the same JSON envelope, in both directions.
package serial

import "encoding/json"

// Message is the wire envelope. Type is the only dispatch key.
type Message struct {
	Type    string          `json:"type"`
	Payload json.RawMessage `json:"payload"`
}

// Pack wraps a payload in the envelope and returns the JSON bytes.
func Pack(msgType string, payload any) ([]byte, error) {
	raw, err := json.Marshal(payload)
	if err != nil {
		return nil, err
	}
	return json.Marshal(Message{Type: msgType, Payload: raw})
}

// Unpack reads an envelope from JSON bytes.
func Unpack(data []byte) (Message, error) {
	var msg Message
	err := json.Unmarshal(data, &msg)
	return msg, err
}
