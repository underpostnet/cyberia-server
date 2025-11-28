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
	cfg        Config
	db         *DB
	col        *mongo.Collection
	gameServer *game.GameServer
}

func NewObjectLayerHandler(cfg Config, db *DB, gameServer *game.GameServer) *ObjectLayerHandler {
	return &ObjectLayerHandler{
		cfg:        cfg,
		db:         db,
		col:        db.Collection("objectlayers"),
		gameServer: gameServer,
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
	r.With(AuthMiddleware(h.cfg), RequireRole(RoleModerator)).Post("/object-layers/recache", h.Recache)
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

// Recache POST /object-layers/recache
// Accepts an array of item IDs and updates the websocket server cache with fresh database data
func (h *ObjectLayerHandler) Recache(w http.ResponseWriter, r *http.Request) {
	var payload struct {
		ItemIDs []string `json:"item_ids"`
	}
	if err := decodeJSONStrict(r, &payload); err != nil {
		errorJSON(w, http.StatusBadRequest, err.Error())
		return
	}

	if len(payload.ItemIDs) == 0 {
		errorJSON(w, http.StatusBadRequest, "item_ids array cannot be empty")
		return
	}

	ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
	defer cancel()

	// Fetch all object layers matching the provided item IDs
	filter := bson.M{"data.item.id": bson.M{"$in": payload.ItemIDs}}
	cur, err := h.col.Find(ctx, filter)
	if err != nil {
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}
	defer cur.Close(ctx)

	var layers []game.ObjectLayer
	if err := cur.All(ctx, &layers); err != nil {
		errorJSON(w, http.StatusInternalServerError, err.Error())
		return
	}

	// Build a map of fetched item IDs for tracking
	fetchedIDs := make(map[string]bool)
	cacheUpdates := make(map[string]*game.ObjectLayer)

	for i := range layers {
		itemID := layers[i].Data.Item.ID
		fetchedIDs[itemID] = true
		cacheUpdates[itemID] = &layers[i]
	}

	// Update the game server cache
	if h.gameServer != nil {
		h.gameServer.UpdateObjectLayerCache(cacheUpdates)
	}

	// Determine which IDs were not found
	var notFound []string
	for _, requestedID := range payload.ItemIDs {
		if !fetchedIDs[requestedID] {
			notFound = append(notFound, requestedID)
		}
	}

	response := bson.M{
		"updated":   len(cacheUpdates),
		"requested": len(payload.ItemIDs),
		"not_found": notFound,
	}

	writeJSON(w, http.StatusOK, response)
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
