package config

import (
	"os"
	"testing"
)

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

// The WebSocket limits are read from one place. Unset keeps the default, an
// explicit 0 disables, and a malformed value is an error rather than a silent
// fallback to the default.
func TestLoadWSLimits(t *testing.T) {
	keys := []string{
		"CYBERIA_DISABLE_CONNECTION_LIMITS",
		"CYBERIA_MAX_CONNECTIONS", "CYBERIA_MAX_CONNECTIONS_PER_IP",
		"CYBERIA_CONNECT_RATE_PER_IP", "CYBERIA_CONNECT_BURST_PER_IP",
		"CYBERIA_MESSAGE_RATE", "CYBERIA_MESSAGE_BURST", "CYBERIA_MAX_STRIKES",
	}
	clear := func() {
		for _, k := range keys {
			os.Unsetenv(k)
		}
	}
	t.Cleanup(clear)

	clear()
	got, err := loadWSLimits()
	if err != nil {
		t.Fatalf("an empty environment must load: %v", err)
	}
	if got.DisableAll || got.MaxConnections != nil || got.ConnectRatePerIP != nil {
		t.Fatalf("unset variables must stay nil, got %+v", got)
	}

	clear()
	os.Setenv("CYBERIA_MAX_CONNECTIONS_PER_IP", "512")
	os.Setenv("CYBERIA_CONNECT_RATE_PER_IP", "0")
	got, err = loadWSLimits()
	if err != nil {
		t.Fatalf("valid values must load: %v", err)
	}
	if got.MaxConnectionsPerIP == nil || *got.MaxConnectionsPerIP != 512 {
		t.Fatalf("per-IP not read: %+v", got.MaxConnectionsPerIP)
	}
	if got.ConnectRatePerIP == nil || *got.ConnectRatePerIP != 0 {
		t.Fatal("an explicit 0 must be preserved, not treated as unset")
	}
	if got.MessageRate != nil {
		t.Fatal("an untouched variable must stay nil")
	}

	clear()
	for _, truthy := range []string{"1", "true", "TRUE", "yes", "on"} {
		os.Setenv("CYBERIA_DISABLE_CONNECTION_LIMITS", truthy)
		got, err = loadWSLimits()
		if err != nil || !got.DisableAll {
			t.Fatalf("%q must enable the disable switch", truthy)
		}
	}
	for _, falsy := range []string{"0", "false", "no", ""} {
		os.Setenv("CYBERIA_DISABLE_CONNECTION_LIMITS", falsy)
		got, _ = loadWSLimits()
		if got.DisableAll {
			t.Fatalf("%q must not enable the disable switch", falsy)
		}
	}

	clear()
	for _, bad := range []string{"abc", "-5", "12x"} {
		os.Setenv("CYBERIA_MESSAGE_RATE", bad)
		if _, err := loadWSLimits(); err == nil {
			t.Fatalf("%q must be an error, not a silent default", bad)
		}
	}
}
