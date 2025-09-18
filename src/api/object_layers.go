package api

import (
	"context"
	"encoding/json"
	"errors"
	"log"
	"net/http"
	"strconv"
	"time"

	game "cyberia-server/src"

	"github.com/go-chi/chi/v5"
	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/bson/primitive"
	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
)

// ObjectLayerHandler groups dependencies.
type ObjectLayerHandler struct {
	cfg Config
	db  *DB
	col *mongo.Collection
}

func NewObjectLayerHandler(cfg Config, db *DB) *ObjectLayerHandler {
	return &ObjectLayerHandler{
		cfg: cfg,
		db:  db,
		col: db.Collection("objectlayers"),
	}
}

// Routes registers routes for object layers.
func (h *ObjectLayerHandler) Routes(r chi.Router) {
	// Public read endpoints (guest allowed)
	r.With().Get("/object-layers", h.List)
	r.With().Get("/object-layers/{id}", h.Get)
	// Write endpoints: require auth and roles
	r.With(AuthMiddleware(h.cfg), RequireRole(RoleModerator)).Post("/object-layers", h.Create)
	r.With(AuthMiddleware(h.cfg), RequireRole(RoleModerator)).Put("/object-layers/{id}", h.Update)
	r.With(AuthMiddleware(h.cfg), RequireRole(RoleAdmin)).Delete("/object-layers/{id}", h.Delete)
}

// List GET /object-layers?page=1&page_size=20
func (h *ObjectLayerHandler) List(w http.ResponseWriter, r *http.Request) {
	page := clamp(parseInt(r.URL.Query().Get("page"), 1), 1, 1000000)
	pageSize := clamp(parseInt(r.URL.Query().Get("page_size"), 20), 1, 100)
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()

	// Optional filter by embedded item id: data.item.id
	filter := bson.M{}
	if itemID := r.URL.Query().Get("item_id"); itemID != "" {
		filter["data.item.id"] = itemID
	}
	opts := options.Find().SetSkip(int64((page - 1) * pageSize)).SetLimit(int64(pageSize)).SetSort(bson.D{{Key: "_id", Value: -1}})
	cur, err := h.col.Find(ctx, filter, opts)
	if err != nil {
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}
	defer cur.Close(ctx)
	var items []game.ObjectLayer
	if err := cur.All(ctx, &items); err != nil {
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}
	total, err := h.col.CountDocuments(ctx, filter)
	if err != nil {
		total = int64(len(items))
	}
	resp := apiListResponse[game.ObjectLayer]{
		Items:      items,
		Page:       page,
		PageSize:   pageSize,
		TotalItems: total,
	}
	writeJSON(w, http.StatusOK, resp)
}

// Get GET /object-layers/{id}
func (h *ObjectLayerHandler) Get(w http.ResponseWriter, r *http.Request) {
	idStr := chi.URLParam(r, "id")
	id, err := primitive.ObjectIDFromHex(idStr)
	if err != nil {
		errorJSON(w, http.StatusBadRequest, "invalid id")
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()
	var data game.ObjectLayer
	if err := h.col.FindOne(ctx, bson.M{"_id": id}).Decode(&data); err != nil {
		if errors.Is(err, mongo.ErrNoDocuments) {
			errorJSON(w, http.StatusNotFound, "not found")
			return
		}
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, http.StatusOK, data)
}

// Create POST /object-layers
func (h *ObjectLayerHandler) Create(w http.ResponseWriter, r *http.Request) {
	var payload game.ObjectLayer
	if err := decodeJSONStrict(r, &payload); err != nil {
		errorJSON(w, http.StatusBadRequest, err.Error())
		return
	}
	// compute or verify hash
	if err := payload.UpdateHash(); err != nil {
		errorJSON(w, http.StatusBadRequest, "invalid object layer content")
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()
	res, err := h.col.InsertOne(ctx, payload)
	if err != nil {
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, http.StatusCreated, bson.M{"id": res.InsertedID})
}

// Update PUT /object-layers/{id}
func (h *ObjectLayerHandler) Update(w http.ResponseWriter, r *http.Request) {
	idStr := chi.URLParam(r, "id")
	id, err := primitive.ObjectIDFromHex(idStr)
	if err != nil {
		errorJSON(w, http.StatusBadRequest, "invalid id")
		return
	}
	var payload game.ObjectLayer
	if err := decodeJSONStrict(r, &payload); err != nil {
		errorJSON(w, http.StatusBadRequest, err.Error())
		return
	}
	if err := payload.UpdateHash(); err != nil {
		errorJSON(w, http.StatusBadRequest, "invalid object layer content")
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()
	res, err := h.col.UpdateByID(ctx, id, bson.M{"$set": bson.M{"data": payload.Data}})

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

// Delete DELETE /object-layers/{id}
func (h *ObjectLayerHandler) Delete(w http.ResponseWriter, r *http.Request) {
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

// CheckObjectLayersByType verifies the integrity of object layers for a specific item type
func CheckObjectLayersByType(ctx context.Context, db *DB, itemType string) error {
	col := db.Collection("objectlayers")
	ctx, cancel := context.WithTimeout(ctx, 15*time.Second)
	defer cancel()

	// Fetch existing object layers of given type
	cur, err := col.Find(ctx, bson.M{"data.item.type": itemType})
	if err != nil {
		return err
	}
	defer cur.Close(ctx)

	var existing []string
	for cur.Next(ctx) {
		var layer game.ObjectLayer
		if err := cur.Decode(&layer); err != nil {
			return err
		}
		existing = append(existing, layer.Data.Item.ID)
	}
	log.Printf("[INFO] Total existing %s object layers: %d\n", itemType, len(existing))

	return nil
}

// Helpers

func parseInt(s string, def int) int {
	if s == "" {
		return def
	}
	v, err := strconv.Atoi(s)
	if err != nil {
		return def
	}
	return v
}

func clamp(v, minV, maxV int) int {
	if v < minV {
		return minV
	}
	if v > maxV {
		return maxV
	}
	return v
}

func decodeJSONStrict(r *http.Request, out any) error {
	dec := json.NewDecoder(r.Body)
	dec.DisallowUnknownFields()
	return dec.Decode(out)
}
