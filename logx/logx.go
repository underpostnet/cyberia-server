// Package logx is the single source of truth for cyberia-server logging.
//
// It wraps log/slog with a JSON handler (clean structured output for K8s log
// collectors) and a runtime-adjustable level. The level is resolved once at
// startup from the environment:
//
//	LOG_LEVEL  — explicit: debug | info | warn | error (wins when set)
//	APP_ENV    — fallback: development/dev/local/test ⇒ debug, everything
//	             else (including unset) ⇒ info
//
// The fallback is deliberately fail-safe toward info: a forgotten env var
// yields production-quiet logging, never debug spam. Development opts INTO
// debug via APP_ENV=development or LOG_LEVEL=debug.
//
// Hot-path, per-event chatter (per-kill economy transfers, per-input
// rejections, per-connection registration) MUST log at Debug so production
// (info) drops it without a recompile. Reserve Info for lifecycle milestones,
// Warn for recoverable anomalies, Error for faults that need attention.
//
// Printf-style helpers (Debugf/Infof/Warnf/Errorf) exist so existing
// log.Printf call sites convert with a one-token edit; the formatted string
// lands in the JSON "msg" field.
package logx

import (
	"context"
	"fmt"
	"log/slog"
	"os"
	"strings"
)

// levelVar backs the active logger so the threshold can change at runtime
// (e.g. a future admin endpoint) without rebuilding the handler.
var levelVar = new(slog.LevelVar)

var logger = slog.New(slog.NewJSONHandler(os.Stdout, &slog.HandlerOptions{Level: levelVar}))

// Init resolves the log level from the environment and installs the JSON
// logger as the slog default. Call once at startup, before other goroutines
// start. Safe to call without arguments; absence of both env vars yields the
// development default (debug).
func Init() {
	levelVar.Set(resolveLevel(os.Getenv("LOG_LEVEL"), os.Getenv("APP_ENV")))
	slog.SetDefault(logger)
}

// SetLevel overrides the active threshold at runtime.
func SetLevel(l slog.Level) { levelVar.Set(l) }

// Level returns the active threshold.
func Level() slog.Level { return levelVar.Level() }

// resolveLevel implements the LOG_LEVEL-wins, APP_ENV-fallback policy.
func resolveLevel(logLevel, appEnv string) slog.Level {
	switch strings.ToLower(strings.TrimSpace(logLevel)) {
	case "debug":
		return slog.LevelDebug
	case "info":
		return slog.LevelInfo
	case "warn", "warning":
		return slog.LevelWarn
	case "error":
		return slog.LevelError
	}
	switch strings.ToLower(strings.TrimSpace(appEnv)) {
	case "development", "dev", "local", "test":
		return slog.LevelDebug
	default:
		// Unset or any production-like value ⇒ info. Fail-safe: never spam
		// debug when the environment is not explicitly a dev environment.
		return slog.LevelInfo
	}
}

// Enabled reports whether a level would be emitted — guard expensive log-arg
// construction with this in the hottest paths.
func Enabled(l slog.Level) bool { return logger.Enabled(context.Background(), l) }

func Debugf(format string, args ...any) { logAt(slog.LevelDebug, format, args...) }
func Infof(format string, args ...any)  { logAt(slog.LevelInfo, format, args...) }
func Warnf(format string, args ...any)  { logAt(slog.LevelWarn, format, args...) }
func Errorf(format string, args ...any) { logAt(slog.LevelError, format, args...) }

func logAt(l slog.Level, format string, args ...any) {
	if !logger.Enabled(context.Background(), l) {
		return
	}
	logger.Log(context.Background(), l, fmt.Sprintf(format, args...))
}
