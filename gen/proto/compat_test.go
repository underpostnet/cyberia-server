package proto

import (
	"testing"

	"google.golang.org/protobuf/reflect/protoreflect"
)

// Protobuf evolution: a field number that carried another meaning is never reused, and a
// renamed field keeps its old name reserved. A decoder of an older build must not read a new
// field as the one it replaced.
func TestReservedLegacyFieldsAreNotReused(t *testing.T) {
	cases := []struct {
		message  protoreflect.MessageDescriptor
		numbers  []protoreflect.FieldNumber
		names    []protoreflect.Name
		expected map[protoreflect.Name]protoreflect.FieldNumber
	}{
		{
			message:  (&ObjectLayerMessage{}).ProtoReflect().Descriptor(),
			numbers:  []protoreflect.FieldNumber{1, 6, 8, 9, 10, 11},
			names:    []protoreflect.Name{"mongo_id", "sha256", "content_hash", "profile"},
			expected: map[protoreflect.Name]protoreflect.FieldNumber{"render": 5, "cid": 7},
		},
		{
			message:  (&Ledger{}).ProtoReflect().Descriptor(),
			numbers:  []protoreflect.FieldNumber{1},
			names:    []protoreflect.Name{"type", "address"},
			expected: map[protoreflect.Name]protoreflect.FieldNumber{"contract_address": 2, "token_id": 3, "standard": 4, "chain_id": 5},
		},
		{
			message:  (&ObjectLayerManifestEntry{}).ProtoReflect().Descriptor(),
			numbers:  []protoreflect.FieldNumber{2, 4},
			names:    []protoreflect.Name{"sha256", "content_hash"},
			expected: map[protoreflect.Name]protoreflect.FieldNumber{"item_id": 1, "cid": 3},
		},
	}

	for _, tc := range cases {
		fields := tc.message.Fields()
		for _, number := range tc.numbers {
			if field := fields.ByNumber(number); field != nil {
				t.Errorf("%s: field %d is reserved but carries %q", tc.message.FullName(), number, field.Name())
			}
			if !isReservedNumber(tc.message, number) {
				t.Errorf("%s: field %d is not declared reserved", tc.message.FullName(), number)
			}
		}
		for _, name := range tc.names {
			if !isReservedName(tc.message, name) {
				t.Errorf("%s: name %q is not declared reserved", tc.message.FullName(), name)
			}
		}
		for name, number := range tc.expected {
			field := fields.ByName(name)
			if field == nil {
				t.Fatalf("%s: field %q is missing", tc.message.FullName(), name)
			}
			if field.Number() != number {
				t.Errorf("%s: field %q = %d, want %d", tc.message.FullName(), name, field.Number(), number)
			}
		}
	}
}

func isReservedNumber(message protoreflect.MessageDescriptor, number protoreflect.FieldNumber) bool {
	ranges := message.ReservedRanges()
	for i := 0; i < ranges.Len(); i++ {
		if r := ranges.Get(i); number >= r[0] && number < r[1] {
			return true
		}
	}
	return false
}

func isReservedName(message protoreflect.MessageDescriptor, name protoreflect.Name) bool {
	names := message.ReservedNames()
	for i := 0; i < names.Len(); i++ {
		if names.Get(i) == name {
			return true
		}
	}
	return false
}
