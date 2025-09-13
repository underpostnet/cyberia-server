package api

import (
	"context"
	"fmt"
	"log"
	"time"

	game "cyberia-server/src"

	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/mongo"
)

// SeedObjectLayers ensures there are 40 skin object layers (ids skin-01..skin-40).
func SeedObjectLayers(ctx context.Context, db *DB) error {
	col := db.Collection("object_layers")
	ctx, cancel := context.WithTimeout(ctx, 15*time.Second)
	defer cancel()

	// Fetch existing skin item ids
	cur, err := col.Find(ctx, bson.M{"doc.item.type": "skin"})
	if err != nil {
		return err
	}
	defer cur.Close(ctx)
	existing := map[string]struct{}{}
	type row struct {
		Doc game.ObjectLayer `bson:"doc"`
	}
	for cur.Next(ctx) {
		var r row
		if err := cur.Decode(&r); err != nil {
			return err
		}
		if r.Doc.Data.Item.ID != "" {
			existing[r.Doc.Data.Item.ID] = struct{}{}
		}
	}
	if err := cur.Err(); err != nil {
		return err
	}

	// Prepare missing docs among skin-01..skin-40
	docs := make([]interface{}, 0, 40)
	for i := 0; i < 40; i++ {
		id := fmt.Sprintf("skin-%02d", i+1)
		if _, ok := existing[id]; ok {
			continue
		}
		ol := game.NewDefaultSkinObjectLayer(id)
		docs = append(docs, bson.M{"doc": ol})
	}
	if len(docs) == 0 {
		log.Println("[INFO] Skin object layers already at desired count (40)")
		return nil
	}
	if _, err := col.InsertMany(ctx, docs); err != nil {
		// tolerate duplicate insertion errors in case multiple instances seed concurrently
		if mongo.IsDuplicateKeyError(err) {
			log.Println("[INFO] Object layers already seeded (dup key)")
			return nil
		}
		return err
	}
	log.Printf("[INFO] Seeded %d missing skin object layers", len(docs))
	return nil
}

// ListAllObjectLayerItemIDs returns all doc.item.id values in ascending _id order.
func ListAllObjectLayerItemIDs(ctx context.Context, db *DB) ([]string, error) {
	col := db.Collection("object_layers")
	ctx, cancel := context.WithTimeout(ctx, 15*time.Second)
	defer cancel()

	cur, err := col.Find(ctx, bson.M{})
	if err != nil {
		return nil, err
	}
	defer cur.Close(ctx)
	type row struct {
		Doc game.ObjectLayer `bson:"doc"`
	}
	ids := []string{}
	for cur.Next(ctx) {
		var r row
		if err := cur.Decode(&r); err != nil {
			return nil, err
		}
		if r.Doc.Data.Item.ID != "" {
			ids = append(ids, r.Doc.Data.Item.ID)
		}
	}
	return ids, cur.Err()
}
