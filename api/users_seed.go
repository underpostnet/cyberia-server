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

	email := strings.TrimSpace(strings.ToLower(cfg.AdminEmail))
	username := strings.TrimSpace(cfg.AdminUsername)

	if email == "" || username == "" {
		log.Println("[WARN] Admin seed skipped: invalid ADMIN_EMAIL or ADMIN_USERNAME")
		return nil
	}

	if err := ValidatePassword(cfg.AdminPassword); err != nil {
		log.Printf("[WARN] Admin seed skipped: invalid ADMIN_PASSWORD: %v", err)
		return nil
	}

	log.Printf("[INFO] Seeding admin with: Email='%s', Username='%s', Password='%s'", email, username, cfg.AdminPassword)

	// 1. Remove all existing admins to ensure a clean slate for admin privileges
	if _, err := col.DeleteMany(ctx, bson.M{"role": RoleAdmin}); err != nil {
		return err
	}

	// 2. Remove any user (even non-admins) that conflicts with the target admin credentials
	// This prevents duplicate key errors if a regular user has the admin's email or username.
	conflictFilter := bson.M{
		"$or": []bson.M{
			{"email": email},
			{"username": username},
		},
	}
	if _, err := col.DeleteMany(ctx, conflictFilter); err != nil {
		return err
	}

	// 3. Create the default admin user
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
			isDup := false
			for _, e := range we.WriteErrors {
				if e.Code == 11000 {
					isDup = true
					break
				}
			}
			if isDup {
				log.Println("[INFO] Admin already exists (dup key)")
			} else {
				return err
			}
		} else {
			return err
		}
	} else {
		log.Println("[INFO] Seeded default admin user")
	}

	// 4. List all admins to verify
	cursor, err := col.Find(ctx, bson.M{"role": RoleAdmin})
	if err != nil {
		log.Printf("[ERROR] Failed to list admins: %v", err)
		return nil
	}
	defer cursor.Close(ctx)

	var admins []User
	if err := cursor.All(ctx, &admins); err != nil {
		log.Printf("[ERROR] Failed to decode admins: %v", err)
		return nil
	}

	log.Println("----- Current Admins in DB -----")
	for _, admin := range admins {
		log.Printf("ID: %s | Username: %s | Email: %s | Role: %s", admin.ID.Hex(), admin.Username, admin.Email, admin.Role)
	}
	log.Println("--------------------------------")

	return nil
}
