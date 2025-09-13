package api

import (
	"context"
	"log"
	"time"

	game "cyberia-server/src"

	"go.mongodb.org/mongo-driver/bson"
)

// SeedObjectLayers ensures there are 40 skin object layers (ids skin-01..skin-40).
func SeedObjectLayers(ctx context.Context, db *DB) error {
	col := db.Collection("objectlayers")
	ctx, cancel := context.WithTimeout(ctx, 15*time.Second)
	defer cancel()

	// Fetch existing skin item ids
	cur, err := col.Find(ctx, bson.M{"doc.data.item.type": "skin"})
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

	log.Println("[INFO] Total existing skin object layers:", len(existing))

	return nil
}

// ListAllObjectLayerItemIDs returns all doc.item.id values in ascending _id order.
func ListAllObjectLayerItemIDs(ctx context.Context, db *DB) ([]string, error) {
	col := db.Collection("objectlayers")
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
