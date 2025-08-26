package api

import (
	"context"
	"errors"
	"log"
	"strings"
	"time"

	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/mongo"
	"golang.org/x/crypto/bcrypt"
)

// SeedDefaultAdmin ensures at least one admin exists. Uses cfg Admin* when creating.
func SeedDefaultAdmin(ctx context.Context, cfg Config, db *DB) error {
	if !cfg.SeedAdmin {
		return nil
	}
	col := db.Collection("users")
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	// Is there any admin?
	count, err := col.CountDocuments(ctx, bson.M{"role": RoleAdmin})
	if err != nil {
		return err
	}
	if count > 0 {
		return nil
	}

	email := strings.TrimSpace(strings.ToLower(cfg.AdminEmail))
	username := strings.TrimSpace(cfg.AdminUsername)
	if email == "" || username == "" || len(cfg.AdminPassword) < 6 {
		log.Println("[WARN] Admin seed skipped: invalid ADMIN_* config values")
		return nil
	}
	pwHash, err := bcrypt.GenerateFromPassword([]byte(cfg.AdminPassword), bcrypt.DefaultCost)
	if err != nil {
		return err
	}
	u := User{
		Email:        email,
		Username:     username,
		PasswordHash: string(pwHash),
		Role:         RoleAdmin,
		CreatedAt:    time.Now(),
		UpdatedAt:    time.Now(),
	}
	if _, err := col.InsertOne(ctx, u); err != nil {
		// tolerate duplicate key errors in case of races
		var we mongo.WriteException
		if errors.As(err, &we) {
			for _, e := range we.WriteErrors {
				if e.Code == 11000 {
					log.Println("[INFO] Admin already exists (dup key)")
					return nil
				}
			}
		}
		return err
	}
	log.Println("[INFO] Seeded default admin user")
	return nil
}
