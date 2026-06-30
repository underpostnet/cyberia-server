package logx

import (
	"log/slog"
	"testing"
)

func TestResolveLevel(t *testing.T) {
	cases := []struct {
		name     string
		logLevel string
		appEnv   string
		want     slog.Level
	}{
		{"explicit debug wins over prod", "debug", "production", slog.LevelDebug},
		{"explicit warn", "WARN", "", slog.LevelWarn},
		{"explicit warning alias", "warning", "", slog.LevelWarn},
		{"explicit error trimmed", "  error ", "", slog.LevelError},
		{"production fallback", "", "production", slog.LevelInfo},
		{"prod alias", "", "prod", slog.LevelInfo},
		{"staging fallback", "", "staging", slog.LevelInfo},
		{"dev default", "", "development", slog.LevelDebug},
		{"both empty defaults debug", "", "", slog.LevelDebug},
		{"unknown log level falls through to env", "verbose", "production", slog.LevelInfo},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			if got := resolveLevel(c.logLevel, c.appEnv); got != c.want {
				t.Errorf("resolveLevel(%q, %q) = %v, want %v", c.logLevel, c.appEnv, got, c.want)
			}
		})
	}
}
