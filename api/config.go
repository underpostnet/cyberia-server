package api

import (
	"log"
	"os"
	"time"
)

// Config holds API configuration loaded from environment variables.
type Config struct {
	MongoURI      string
	MongoDatabase string
	JWTSecret     string
	JWTIssuer     string
	ReadTimeout   time.Duration
	WriteTimeout  time.Duration
	SeedAdmin     bool
	AdminEmail    string
	AdminUsername string
	AdminPassword string
}

func LoadConfig() Config {
	cfg := Config{
		MongoURI:      getEnv("MONGO_URI", "mongodb://mongodb-0.mongodb-service:27017"),
		MongoDatabase: getEnv("MONGO_DB", "example2-cyberia"),
		JWTSecret:     getEnv("JWT_SECRET", "dev-secret-change-me"),
		JWTIssuer:     getEnv("JWT_ISSUER", "cyberia-server"),
		ReadTimeout:   parseDuration(getEnv("API_READ_TIMEOUT", "15s"), 15*time.Second),
		WriteTimeout:  parseDuration(getEnv("API_WRITE_TIMEOUT", "15s"), 15*time.Second),
		SeedAdmin:     getEnv("ADMIN_SEED", "true") == "true",
		AdminEmail:    getEnv("ADMIN_EMAIL", "admin@example.com"),
		AdminUsername: getEnv("ADMIN_USERNAME", "admin"),
		AdminPassword: getEnv("ADMIN_PASSWORD", "ChangeMe1!"),
	}
	if cfg.JWTSecret == "dev-secret-change-me" {
		log.Println("[WARN] Using default JWT secret; set JWT_SECRET in production")
	}
	return cfg
}

func getEnv(key, def string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return def
}

func parseDuration(s string, def time.Duration) time.Duration {
	d, err := time.ParseDuration(s)
	if err != nil {
		return def
	}
	return d
}
