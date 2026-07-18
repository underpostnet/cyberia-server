// Package engine_client — dispatcher.go
//
// DataSource is the engine world-load transport contract, implemented by
// GrpcClient (primary) and RestClient (boot fallback). Dispatcher is the
// transport controller: it routes every call to gRPC first and retries over
// REST when the gRPC call fails.
package engine_client

import (
	"context"
	"fmt"
	"log"

	game "cyberia-server/game"
	pb "cyberia-server/gen/proto"
)

// DataSource provides the engine world-load surface used by WorldBuilder.
type DataSource interface {
	Ping(ctx context.Context) (int64, error)
	FetchObjectLayer(ctx context.Context, itemID string) (*game.ObjectLayer, error)
	FetchFullInstance(ctx context.Context, instanceCode string) (*pb.GetFullInstanceResponse, error)
	FetchObjectLayerManifest(ctx context.Context) ([]ManifestEntry, error)
	Close() error
}

// Dispatcher implements DataSource by trying the gRPC transport first and
// falling back to the REST boot transport on any per-call failure.
type Dispatcher struct {
	grpc DataSource
	rest DataSource
}

// NewDispatcher wires the configured transports (ENGINE_GRPC_ADDRESS /
// ENGINE_API_BASE_URL). At least one transport is required.
func NewDispatcher(grpcAddress, restBaseURL string) (*Dispatcher, error) {
	d := &Dispatcher{}
	if grpcAddress != "" {
		gc, err := NewGrpcClient(grpcAddress)
		if err != nil {
			log.Printf("[EngineClient] gRPC client init failed: %v", err)
		} else {
			d.grpc = gc
		}
	}
	if restBaseURL != "" {
		d.rest = NewRestClient(restBaseURL)
	}
	if d.grpc == nil && d.rest == nil {
		return nil, fmt.Errorf("engine_client: no transport available (set ENGINE_GRPC_ADDRESS and/or ENGINE_API_BASE_URL)")
	}
	return d, nil
}

// Close tears down both transports.
func (d *Dispatcher) Close() error {
	var err error
	if d.grpc != nil {
		err = d.grpc.Close()
	}
	if d.rest != nil {
		if restErr := d.rest.Close(); err == nil {
			err = restErr
		}
	}
	return err
}

// dispatch runs op on gRPC first, then on REST when gRPC is unavailable or fails.
func dispatch[T any](d *Dispatcher, op string, call func(DataSource) (T, error)) (T, error) {
	var zero T
	var grpcErr error
	if d.grpc != nil {
		result, err := call(d.grpc)
		if err == nil {
			return result, nil
		}
		grpcErr = err
		if d.rest == nil {
			return zero, grpcErr
		}
		log.Printf("[EngineClient] %s via gRPC failed: %v — retrying via REST boot fallback", op, grpcErr)
	}
	result, restErr := call(d.rest)
	if restErr == nil {
		return result, nil
	}
	if grpcErr != nil {
		return zero, fmt.Errorf("%s failed on both transports: gRPC (%v), REST (%w)", op, grpcErr, restErr)
	}
	return zero, restErr
}

func (d *Dispatcher) Ping(ctx context.Context) (int64, error) {
	return dispatch(d, "Ping", func(ds DataSource) (int64, error) { return ds.Ping(ctx) })
}

func (d *Dispatcher) FetchObjectLayer(ctx context.Context, itemID string) (*game.ObjectLayer, error) {
	return dispatch(d, "FetchObjectLayer", func(ds DataSource) (*game.ObjectLayer, error) {
		return ds.FetchObjectLayer(ctx, itemID)
	})
}

func (d *Dispatcher) FetchFullInstance(ctx context.Context, instanceCode string) (*pb.GetFullInstanceResponse, error) {
	return dispatch(d, "FetchFullInstance", func(ds DataSource) (*pb.GetFullInstanceResponse, error) {
		return ds.FetchFullInstance(ctx, instanceCode)
	})
}

func (d *Dispatcher) FetchObjectLayerManifest(ctx context.Context) ([]ManifestEntry, error) {
	return dispatch(d, "FetchObjectLayerManifest", func(ds DataSource) ([]ManifestEntry, error) {
		return ds.FetchObjectLayerManifest(ctx)
	})
}
