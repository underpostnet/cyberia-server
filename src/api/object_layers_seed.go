package api

import (
	"context"
	"log"
	"time"

	game "cyberia-server/src"

	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/mongo/options"
)

// SeedObjectLayers ensures there are 40 skin object layers (ids skin-01..skin-40).
func SeedObjectLayers(ctx context.Context, db *DB) error {
	col := db.Collection("objectlayers")
	ctx, cancel := context.WithTimeout(ctx, 15*time.Second)
	defer cancel()

	// Fetch existing skin item ids
	cur, err := col.Find(ctx, bson.M{"data.item.type": "skin"})
	if err != nil {
		return err
	}
	defer cur.Close(ctx)
	existing := map[string]struct{}{}
	var layer game.ObjectLayer
	for cur.Next(ctx) {
		if err := cur.Decode(&layer); err != nil {
			return err
		}
		if layer.Data.Item.ID != "" {
			existing[layer.Data.Item.ID] = struct{}{}
		}
	}
	if err := cur.Err(); err != nil {
		return err
	}

	log.Println("[INFO] Total existing skin object layers:", len(existing))

	return nil
}

// ListAllObjectLayerItemIDs returns all data.item.id values in ascending _id order.
func ListAllObjectLayerItemIDs(ctx context.Context, db *DB) ([]string, error) {
	col := db.Collection("objectlayers")
	ctx, cancel := context.WithTimeout(ctx, 15*time.Second)
	defer cancel()

	var ids []string
	cur, err := col.Find(ctx, bson.M{}, options.Find().SetProjection(bson.M{"data.item.id": 1}))
	if err != nil {
		return nil, err
	}
	defer cur.Close(ctx)
	type row struct {
		Data struct {
			Item struct {
				ID string `bson:"id"`
			} `bson:"item"`
		} `bson:"data"`
	}
	for cur.Next(ctx) {
		var r row
		if err := cur.Decode(&r); err != nil {
			return nil, err
		}
		ids = append(ids, r.Data.Item.ID)
	}
	if err := cur.Err(); err != nil {
		return nil, err
	}
	return ids, nil
}
