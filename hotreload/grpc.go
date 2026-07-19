package hotreload

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net"

	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
)

// The control service is served on its own gRPC server with a JSON codec
// (ForceServerCodec overrides content-subtype lookup), so the wire contract is
// the Request/Result structs below rather than a generated protobuf message.
// engine-cyberia calls it with @grpc/grpc-js makeUnaryRequest and JSON
// serializers — no .proto codegen on either side.
//
// Keep ServiceName/MethodName in sync with the engine client
// (src/projects/cyberia/hot-reload-trigger.js).
const (
	ServiceName = "cyberia.CyberiaControlService"
	MethodName  = "TriggerHotReload"
	FullMethod  = "/" + ServiceName + "/" + MethodName
)

// Request is the JSON body of a TriggerHotReload call.
type Request struct {
	APIKey       string `json:"apiKey"`
	Mode         string `json:"mode"`
	InstanceCode string `json:"instanceCode"`
}

// jsonCodec marshals every message on the control server as JSON.
type jsonCodec struct{}

func (jsonCodec) Marshal(v any) ([]byte, error)      { return json.Marshal(v) }
func (jsonCodec) Unmarshal(data []byte, v any) error { return json.Unmarshal(data, v) }
func (jsonCodec) Name() string                       { return "json" }

func triggerHandler(srv any, ctx context.Context, dec func(any) error, interceptor grpc.UnaryServerInterceptor) (any, error) {
	req := new(Request)
	if err := dec(req); err != nil {
		return nil, status.Error(codes.InvalidArgument, err.Error())
	}
	handle := func(ctx context.Context, raw any) (any, error) {
		in := raw.(*Request)
		res, err := srv.(*Service).Trigger(ctx, in.APIKey, Mode(in.Mode), in.InstanceCode)
		if err != nil {
			switch err {
			case ErrUnauthorized:
				return nil, status.Error(codes.PermissionDenied, err.Error())
			case ErrBusy:
				return nil, status.Error(codes.FailedPrecondition, err.Error())
			default:
				return nil, status.Error(codes.Internal, err.Error())
			}
		}
		return &res, nil
	}
	if interceptor == nil {
		return handle(ctx, req)
	}
	return interceptor(ctx, req, &grpc.UnaryServerInfo{Server: srv, FullMethod: FullMethod}, handle)
}

var serviceDesc = grpc.ServiceDesc{
	ServiceName: ServiceName,
	HandlerType: (*any)(nil),
	Methods: []grpc.MethodDesc{
		{MethodName: MethodName, Handler: triggerHandler},
	},
	Streams:  []grpc.StreamDesc{},
	Metadata: "hotreload/grpc.go",
}

// GRPCServer is the control-plane gRPC listener.
type GRPCServer struct {
	server *grpc.Server
	lis    net.Listener
}

// ListenGRPC starts the control service on `addr`. Returns a nil server (and
// no error) when the trigger is disabled, so callers can ignore the result.
func ListenGRPC(svc *Service, addr string) (*GRPCServer, error) {
	if !svc.Enabled() {
		return nil, nil
	}
	lis, err := net.Listen("tcp", addr)
	if err != nil {
		return nil, fmt.Errorf("hot-reload gRPC listen %s: %w", addr, err)
	}
	s := grpc.NewServer(grpc.ForceServerCodec(jsonCodec{}))
	s.RegisterService(&serviceDesc, svc)

	go func() {
		log.Printf("[HotReload] gRPC control service listening on %s", addr)
		if err := s.Serve(lis); err != nil {
			log.Printf("[HotReload] gRPC control service stopped: %v", err)
		}
	}()
	return &GRPCServer{server: s, lis: lis}, nil
}

// Stop gracefully shuts the control listener down.
func (g *GRPCServer) Stop() {
	if g == nil || g.server == nil {
		return
	}
	g.server.GracefulStop()
}
