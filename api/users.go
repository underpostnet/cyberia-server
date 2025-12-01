package api

import (
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"strings"
	"time"

	"github.com/go-chi/chi/v5"
	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/bson/primitive"
	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
	"golang.org/x/crypto/bcrypt"
)

// User represents an account in the system.
type User struct {
	ID           primitive.ObjectID `bson:"_id,omitempty" json:"id"`
	Email        string             `bson:"email" json:"email"`
	Username     string             `bson:"username" json:"username"`
	PasswordHash string             `bson:"password_hash" json:"-"`
	Role         string             `bson:"role" json:"role"`
	CreatedAt    time.Time          `bson:"created_at" json:"created_at"`
	UpdatedAt    time.Time          `bson:"updated_at" json:"updated_at"`
}

// UserHandler holds deps for user routes.
type UserHandler struct {
	cfg Config
	db  *DB
	col *mongo.Collection
}

func NewUserHandler(cfg Config, db *DB) *UserHandler {
	return &UserHandler{cfg: cfg, db: db, col: db.Collection("users")}
}

// Routes registers user-related routes.
func (h *UserHandler) Routes(r chi.Router) {
	// Auth endpoints
	r.Post("/auth/register", h.Register)
	r.Post("/auth/login", h.Login)

	// Current user
	r.With(AuthMiddleware(h.cfg)).Get("/users/me", h.Me)

	// Admin/moderator management
	r.With(AuthMiddleware(h.cfg), RequireRole(RoleModerator)).Get("/users", h.List)
	r.With(AuthMiddleware(h.cfg), RequireRole(RoleModerator)).Get("/users/{id}", h.Get)

	// Update: self can update limited fields; admin can update any incl. role
	r.With(AuthMiddleware(h.cfg)).Put("/users/{id}", h.Update)
	// Delete: admin only
	r.With(AuthMiddleware(h.cfg), RequireRole(RoleAdmin)).Delete("/users/{id}", h.Delete)
}

// Register creates a basic user account with role "user".
func (h *UserHandler) Register(w http.ResponseWriter, r *http.Request) {
	var in struct {
		Email    string `json:"email"`
		Username string `json:"username"`
		Password string `json:"password"`
	}
	if err := json.NewDecoder(r.Body).Decode(&in); err != nil {
		errorJSON(w, http.StatusBadRequest, "invalid json")
		return
	}
	in.Email = strings.TrimSpace(strings.ToLower(in.Email))
	in.Username = strings.TrimSpace(in.Username)
	if in.Email == "" || in.Username == "" {
		errorJSON(w, http.StatusBadRequest, "email and username required")
		return
	}
	if err := ValidatePassword(in.Password); err != nil {
		errorJSON(w, http.StatusBadRequest, err.Error())
		return
	}
	pwHash, err := bcrypt.GenerateFromPassword([]byte(in.Password), bcrypt.DefaultCost)
	if err != nil {
		errorJSON(w, http.StatusInternalServerError, "password hashing failed")
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()
	u := User{
		Email:        in.Email,
		Username:     in.Username,
		PasswordHash: string(pwHash),
		Role:         RoleUser,
		CreatedAt:    time.Now(),
		UpdatedAt:    time.Now(),
	}
	// Unique by email
	_, err = h.col.Indexes().CreateOne(ctx, mongo.IndexModel{Keys: bson.D{{Key: "email", Value: 1}}, Options: options.Index().SetUnique(true)})
	if err != nil {
		// ignore index creation errors after first time
	}
	res, err := h.col.InsertOne(ctx, u)
	if err != nil {
		if we, ok := err.(mongo.WriteException); ok {
			for _, e := range we.WriteErrors {
				if e.Code == 11000 {
					errorJSON(w, http.StatusConflict, "email already exists")
					return
				}
			}
		}
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, http.StatusCreated, bson.M{"id": res.InsertedID})
}

// Login verifies credentials and returns a JWT.
func (h *UserHandler) Login(w http.ResponseWriter, r *http.Request) {
	var in struct {
		Email    string `json:"email"`
		Password string `json:"password"`
	}
	if err := json.NewDecoder(r.Body).Decode(&in); err != nil {
		errorJSON(w, http.StatusBadRequest, "invalid json")
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()
	in.Email = strings.TrimSpace(strings.ToLower(in.Email))
	var u User
	if err := h.col.FindOne(ctx, bson.M{"email": in.Email}).Decode(&u); err != nil {
		errorJSON(w, http.StatusUnauthorized, "invalid credentials")
		return
	}
	if err := bcrypt.CompareHashAndPassword([]byte(u.PasswordHash), []byte(in.Password)); err != nil {
		errorJSON(w, http.StatusUnauthorized, "invalid credentials")
		return
	}
	token, err := GenerateToken(h.cfg.JWTSecret, h.cfg.JWTIssuer, u.ID.Hex(), u.Role, 24*time.Hour)
	if err != nil {
		errorJSON(w, http.StatusInternalServerError, "could not generate token")
		return
	}
	writeJSON(w, http.StatusOK, bson.M{"token": token})
}

// Me returns the current authenticated user doc.
func (h *UserHandler) Me(w http.ResponseWriter, r *http.Request) {
	claims, err := getClaims(r)
	if err != nil {
		errorJSON(w, http.StatusUnauthorized, "unauthenticated")
		return
	}
	uid, _ := primitive.ObjectIDFromHex(claims.Sub)
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()
	var u User
	if err := h.col.FindOne(ctx, bson.M{"_id": uid}).Decode(&u); err != nil {
		errorJSON(w, http.StatusNotFound, "user not found")
		return
	}
	u.PasswordHash = ""
	writeJSON(w, http.StatusOK, u)
}

// List users with pagination (moderator+)
func (h *UserHandler) List(w http.ResponseWriter, r *http.Request) {
	page := clamp(parseInt(r.URL.Query().Get("page"), 1), 1, 1000000)
	pageSize := clamp(parseInt(r.URL.Query().Get("page_size"), 20), 1, 100)
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()

	filter := bson.M{}
	opts := options.Find().SetSkip(int64((page - 1) * pageSize)).SetLimit(int64(pageSize)).SetSort(bson.D{{Key: "_id", Value: -1}})
	cur, err := h.col.Find(ctx, filter, opts)
	if err != nil {
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}
	defer cur.Close(ctx)
	var items []User
	if err := cur.All(ctx, &items); err != nil {
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}
	for i := range items {
		items[i].PasswordHash = ""
	}
	total, _ := h.col.CountDocuments(ctx, filter)
	writeJSON(w, http.StatusOK, apiListResponse[User]{Items: items, Page: page, PageSize: pageSize, TotalItems: total})
}

// Get a user by id (moderator+)
func (h *UserHandler) Get(w http.ResponseWriter, r *http.Request) {
	idStr := chi.URLParam(r, "id")
	id, err := primitive.ObjectIDFromHex(idStr)
	if err != nil {
		errorJSON(w, http.StatusBadRequest, "invalid id")
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()
	var u User
	if err := h.col.FindOne(ctx, bson.M{"_id": id}).Decode(&u); err != nil {
		if errors.Is(err, mongo.ErrNoDocuments) {
			errorJSON(w, http.StatusNotFound, "not found")
			return
		}
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}
	u.PasswordHash = ""
	writeJSON(w, http.StatusOK, u)
}

// Update user. Self can change username; admin can also change role.
func (h *UserHandler) Update(w http.ResponseWriter, r *http.Request) {
	idStr := chi.URLParam(r, "id")
	id, err := primitive.ObjectIDFromHex(idStr)
	if err != nil {
		errorJSON(w, http.StatusBadRequest, "invalid id")
		return
	}
	claims, err := getClaims(r)
	if err != nil {
		errorJSON(w, http.StatusUnauthorized, "unauthenticated")
		return
	}
	callerID, _ := primitive.ObjectIDFromHex(claims.Sub)
	isAdmin := roleRank[claims.Role] >= roleRank[RoleAdmin]
	isSelf := callerID == id

	var in struct {
		Username *string `json:"username"`
		Role     *string `json:"role"`
	}
	if err := json.NewDecoder(r.Body).Decode(&in); err != nil {
		errorJSON(w, http.StatusBadRequest, "invalid json")
		return
	}
	update := bson.M{"updated_at": time.Now()}
	if in.Username != nil && strings.TrimSpace(*in.Username) != "" {
		if !isSelf && !isAdmin {
			errorJSON(w, http.StatusForbidden, "insufficient role")
			return
		}
		update["username"] = strings.TrimSpace(*in.Username)
	}
	if in.Role != nil {
		if !isAdmin {
			errorJSON(w, http.StatusForbidden, "only admin can change role")
			return
		}
		if _, ok := roleRank[*in.Role]; !ok {
			errorJSON(w, http.StatusBadRequest, "invalid role")
			return
		}
		update["role"] = *in.Role
	}
	if len(update) == 1 { // only updated_at present
		writeJSON(w, http.StatusOK, bson.M{"updated": 0})
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()
	res, err := h.col.UpdateByID(ctx, id, bson.M{"$set": update})
	if err != nil {
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}
	if res.MatchedCount == 0 {
		errorJSON(w, http.StatusNotFound, "not found")
		return
	}
	writeJSON(w, http.StatusOK, bson.M{"updated": res.ModifiedCount})
}

// Delete user (admin only)
func (h *UserHandler) Delete(w http.ResponseWriter, r *http.Request) {
	idStr := chi.URLParam(r, "id")
	id, err := primitive.ObjectIDFromHex(idStr)
	if err != nil {
		errorJSON(w, http.StatusBadRequest, "invalid id")
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()
	res, err := h.col.DeleteOne(ctx, bson.M{"_id": id})
	if err != nil {
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}
	if res.DeletedCount == 0 {
		errorJSON(w, http.StatusNotFound, "not found")
		return
	}
	w.WriteHeader(http.StatusNoContent)
}
