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

// SeedObjectLayers creates 40 random skin object layers if collection is empty.
func SeedObjectLayers(ctx context.Context, db *DB) error {
	col := db.Collection("object_layers")
	ctx, cancel := context.WithTimeout(ctx, 15*time.Second)
	defer cancel()

	count, err := col.CountDocuments(ctx, bson.M{})
	if err != nil {
		return err
	}
	if count > 0 {
		return nil
	}

	docs := make([]interface{}, 0, 40)
	for i := 0; i < 40; i++ {
		id := fmt.Sprintf("skin-%02d", i+1)
		ol := game.NewDefaultSkinObjectLayer(id)
		docs = append(docs, bson.M{"doc": ol})
	}
	if len(docs) == 0 {
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
	log.Println("[INFO] Seeded 40 skin object layers")
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
	type row struct{ Doc game.ObjectLayer `bson:"doc"` }
	ids := []string{}
	for cur.Next(ctx) {
		var r row
		if err := cur.Decode(&r); err != nil {
			return nil, err
		}
		if r.Doc.Item.ID != "" {
			ids = append(ids, r.Doc.Item.ID)
		}
	}
	return ids, cur.Err()
}
