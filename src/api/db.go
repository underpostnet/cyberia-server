package api

import (
	"context"
	"time"

	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
)

// DB wraps the Mongo client and database handle.
type DB struct {
	Client   *mongo.Client
	Database *mongo.Database
}

func ConnectMongo(ctx context.Context, cfg Config) (*DB, error) {
	client, err := mongo.NewClient(options.Client().ApplyURI(cfg.MongoURI))
	if err != nil {
		return nil, err
	}
	ctx, cancel := context.WithTimeout(ctx, 20*time.Second)
	defer cancel()
	if err := client.Connect(ctx); err != nil {
		return nil, err
	}
	return &DB{Client: client, Database: client.Database(cfg.MongoDatabase)}, nil
}

func (db *DB) Collection(name string) *mongo.Collection {
	return db.Database.Collection(name)
}
