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
		{"development opts into debug", "", "development", slog.LevelDebug},
		{"dev alias", "", "dev", slog.LevelDebug},
		{"both empty fail-safe to info", "", "", slog.LevelInfo},
		{"unknown env fail-safe to info", "", "whatever", slog.LevelInfo},
		{"unknown log level falls through to env", "verbose", "production", slog.LevelInfo},
		{"explicit debug still wins in prod", "debug", "", slog.LevelDebug},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			if got := resolveLevel(c.logLevel, c.appEnv); got != c.want {
				t.Errorf("resolveLevel(%q, %q) = %v, want %v", c.logLevel, c.appEnv, got, c.want)
			}
		})
	}
}
