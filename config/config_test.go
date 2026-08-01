package config

import "testing"

func TestNormalizeBasePath(t *testing.T) {
	tests := []struct {
		name    string
		input   string
		want    string
		wantErr bool
	}{
		{name: "empty is root", input: "", want: "/"},
		{name: "root", input: "/", want: "/"},
		{name: "variant", input: "/FOREST", want: "/FOREST"},
		{name: "adds slash", input: "TEST", want: "/TEST"},
		{name: "trailing slash", input: "/FOREST/", want: "/FOREST"},
		{name: "rejects nested paths", input: "/FOREST/admin", want: "/FOREST/admin", wantErr: true},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got, err := normalizeBasePath(tt.input)
			if (err != nil) != tt.wantErr {
				t.Fatalf("normalizeBasePath(%q) error = %v, wantErr %v", tt.input, err, tt.wantErr)
			}
			if got != tt.want {
				t.Fatalf("normalizeBasePath(%q) = %q, want %q", tt.input, got, tt.want)
			}
		})
	}
}
