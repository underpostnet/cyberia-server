// Package engine_client — player_client.go
//
// PlayerClient is the game server side of CyberiaPlayerService: launch
// tickets, gameplay sessions, the commits of the persist pipeline and
// friendships. Every call carries the server key. A refusal comes back as a
// value; only a transport failure is an error.
package engine_client

import (
	"context"
	"errors"
	"fmt"
	"time"

	"cyberia-server/game/net"
	"cyberia-server/game/persist"
	pb "cyberia-server/gen/proto"

	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/metadata"
	"google.golang.org/grpc/status"
)

// serverKeyHeader carries CYBERIA_SERVER_API_KEY on every player call.
const serverKeyHeader = "x-cyberia-server-api-key"

// sessionAttempts bounds the tries of one launch call. The engine answers a
// retry of the same connection with the first result.
const sessionAttempts = 3

// PlayerClient calls CyberiaPlayerService.
type PlayerClient struct {
	conn   *grpc.ClientConn
	svc    pb.CyberiaPlayerServiceClient
	apiKey string
}

// NewPlayerClient dials the Engine gRPC server at address with the server key.
func NewPlayerClient(address, apiKey string) (*PlayerClient, error) {
	if apiKey == "" {
		return nil, errors.New("engine_client: the player service needs CYBERIA_SERVER_API_KEY")
	}
	conn, err := dialEngine(address)
	if err != nil {
		return nil, err
	}
	return &PlayerClient{conn: conn, svc: pb.NewCyberiaPlayerServiceClient(conn), apiKey: apiKey}, nil
}

// Close tears down the connection.
func (c *PlayerClient) Close() error { return c.conn.Close() }

func (c *PlayerClient) keyed(ctx context.Context) context.Context {
	return metadata.AppendToOutgoingContext(ctx, serverKeyHeader, c.apiKey)
}

// retryable reports whether a call error is worth another try.
func retryable(err error) bool {
	switch status.Code(err) {
	case codes.Unavailable, codes.DeadlineExceeded, codes.Internal, codes.Aborted, codes.ResourceExhausted:
		return true
	}
	return false
}

// launchCall runs an idempotent launch call, retrying a transport failure
// while ctx allows.
func launchCall[T any](ctx context.Context, call func(context.Context) (T, error)) (T, error) {
	var zero T
	var err error
	for attempt := 0; attempt < sessionAttempts; attempt++ {
		var out T
		if out, err = call(ctx); err == nil {
			return out, nil
		}
		if !retryable(err) || ctx.Err() != nil {
			break
		}
		select {
		case <-ctx.Done():
			return zero, err
		case <-time.After(time.Duration(attempt+1) * 200 * time.Millisecond):
		}
	}
	return zero, err
}

// ConsumeTicket implements net.Accounts.
func (c *PlayerClient) ConsumeTicket(ctx context.Context, ticket, instanceCode, connectionID, serverID string) (net.Account, error) {
	resp, err := launchCall(ctx, func(ctx context.Context) (*pb.ConsumeGameLaunchTicketResponse, error) {
		return c.svc.ConsumeGameLaunchTicket(c.keyed(ctx), &pb.ConsumeGameLaunchTicketRequest{
			Ticket: ticket, InstanceCode: instanceCode, ConnectionId: connectionID, ServerId: serverID,
		})
	})
	if err != nil {
		return net.Account{}, fmt.Errorf("ConsumeGameLaunchTicket: %w", err)
	}
	return net.Account{
		Refusal:     resp.GetRefusal(),
		Principal:   resp.GetPrincipal(),
		AccountID:   resp.GetAccountId(),
		DisplayName: resp.GetDisplayName(),
	}, nil
}

// IssueResume implements net.Accounts. It runs off the tick and is not retried:
// the next renewal asks again.
func (c *PlayerClient) IssueResume(ctx context.Context, lease persist.Lease, instanceCode string) (net.IssuedTicket, error) {
	resp, err := c.svc.IssueResumeTicket(c.keyed(ctx), &pb.IssueResumeTicketRequest{
		PlayerId: lease.PlayerID, AccountId: lease.AccountID, Epoch: lease.Epoch, InstanceCode: instanceCode,
	})
	if err != nil {
		return net.IssuedTicket{}, fmt.Errorf("IssueResumeTicket: %w", err)
	}
	return net.IssuedTicket{
		Refusal:   resp.GetRefusal(),
		Ticket:    resp.GetTicket(),
		ExpiresAt: time.UnixMilli(resp.GetExpiresAtMs()),
	}, nil
}

// IssueTravel implements net.Accounts. The session asks once before it leaves;
// a failure leaves the player the launch bridge.
func (c *PlayerClient) IssueTravel(ctx context.Context, lease persist.Lease, instanceCode, targetCode string) (net.IssuedTicket, error) {
	resp, err := c.svc.IssueTravelTicket(c.keyed(ctx), &pb.IssueTravelTicketRequest{
		PlayerId: lease.PlayerID, AccountId: lease.AccountID, Epoch: lease.Epoch,
		InstanceCode: instanceCode, TargetCode: targetCode,
	})
	if err != nil {
		return net.IssuedTicket{}, fmt.Errorf("IssueTravelTicket: %w", err)
	}
	return net.IssuedTicket{
		Refusal:   resp.GetRefusal(),
		Ticket:    resp.GetTicket(),
		ExpiresAt: time.UnixMilli(resp.GetExpiresAtMs()),
	}, nil
}

// AcquireSession implements net.Accounts.
func (c *PlayerClient) AcquireSession(ctx context.Context, connectionID, instanceCode, serverID string) (net.AcquiredSession, error) {
	resp, err := launchCall(ctx, func(ctx context.Context) (*pb.AcquirePlayerSessionResponse, error) {
		return c.svc.AcquirePlayerSession(c.keyed(ctx), &pb.AcquirePlayerSessionRequest{
			ConnectionId: connectionID, InstanceCode: instanceCode, ServerId: serverID,
		})
	})
	if err != nil {
		return net.AcquiredSession{}, fmt.Errorf("AcquirePlayerSession: %w", err)
	}
	return net.AcquiredSession{
		Refusal:     resp.GetRefusal(),
		Lease:       persist.Lease{PlayerID: resp.GetPlayerId(), Epoch: resp.GetEpoch()},
		State:       stateFromProto(resp.GetState()),
		Created:     resp.GetCreated(),
		LeaseLength: time.Duration(resp.GetLeaseSeconds()) * time.Second,
	}, nil
}

// Commit implements persist.Store.
func (c *PlayerClient) Commit(ctx context.Context, req persist.CommitRequest) (persist.CommitResult, error) {
	state := stateToProto(req.State)
	state.Position.Seq = req.PositionSeq
	resp, err := c.svc.CommitPlayerState(c.keyed(ctx), &pb.CommitPlayerStateRequest{
		PlayerId:        req.PlayerID,
		Epoch:           req.Epoch,
		Seq:             req.Seq,
		State:           state,
		QuestsChanged:   req.QuestsChanged,
		PositionChanged: req.PositionChanged,
	})
	if err != nil {
		return persist.CommitResult{}, err
	}
	return persist.CommitResult{Refusal: resp.GetRefusal(), Duplicate: resp.GetDuplicate()}, nil
}

// Checkpoint implements persist.Store.
func (c *PlayerClient) Checkpoint(ctx context.Context, checkpoints []persist.Checkpoint) ([]persist.Ended, error) {
	req := &pb.CheckpointPlayerSessionsRequest{Sessions: make([]*pb.SessionCheckpoint, 0, len(checkpoints))}
	for _, cp := range checkpoints {
		entry := &pb.SessionCheckpoint{PlayerId: cp.PlayerID, AccountId: cp.AccountID, Epoch: cp.Epoch, MapCode: cp.MapCode}
		if cp.Position != nil {
			entry.Position = &pb.PlayerPosition{MapCode: cp.Position.MapCode, X: cp.Position.X, Y: cp.Position.Y, Seq: cp.PositionSeq}
		}
		req.Sessions = append(req.Sessions, entry)
	}
	resp, err := c.svc.CheckpointPlayerSessions(c.keyed(ctx), req)
	if err != nil {
		return nil, err
	}
	ended := make([]persist.Ended, 0, len(resp.GetEnded()))
	for _, e := range resp.GetEnded() {
		ended = append(ended, persist.Ended{PlayerID: e.GetPlayerId(), Reason: e.GetReason()})
	}
	return ended, nil
}

// Release implements persist.Store.
func (c *PlayerClient) Release(ctx context.Context, lease persist.Lease) error {
	_, err := c.svc.ReleasePlayerSession(c.keyed(ctx), &pb.ReleasePlayerSessionRequest{
		PlayerId: lease.PlayerID, AccountId: lease.AccountID, Epoch: lease.Epoch,
	})
	return err
}

// ListFriends implements net.Accounts.
func (c *PlayerClient) ListFriends(ctx context.Context, accountID string) (net.FriendList, error) {
	resp, err := c.svc.ListFriends(c.keyed(ctx), &pb.ListFriendsRequest{AccountId: accountID})
	if err != nil {
		return net.FriendList{}, err
	}
	list := net.FriendList{Friends: make([]net.FriendRecord, 0, len(resp.GetFriends())), Note: resp.GetNote(), NoteNextUpdateAt: time.UnixMilli(resp.GetNoteNextUpdateAtMs())}
	for _, f := range resp.GetFriends() {
		list.Friends = append(list.Friends, friendFromProto(f))
	}
	return list, nil
}

// UpdateStatusNote implements net.Accounts.
func (c *PlayerClient) UpdateStatusNote(ctx context.Context, accountID, note string) (string, error) {
	resp, err := c.svc.UpdateStatusNote(c.keyed(ctx), &pb.UpdateStatusNoteRequest{AccountId: accountID, Note: note})
	if err != nil {
		return "", err
	}
	return resp.GetRefusal(), nil
}

// UpdateFriendship implements net.Accounts.
func (c *PlayerClient) UpdateFriendship(ctx context.Context, accountID, action, targetAccountID, friendshipID string) (string, *net.FriendRecord, error) {
	resp, err := c.svc.UpdateFriendship(c.keyed(ctx), &pb.UpdateFriendshipRequest{
		AccountId: accountID, Action: action, TargetAccountId: targetAccountID, FriendshipId: friendshipID,
	})
	if err != nil {
		return "", nil, err
	}
	if resp.GetFriend() == nil {
		return resp.GetRefusal(), nil, nil
	}
	friend := friendFromProto(resp.GetFriend())
	return resp.GetRefusal(), &friend, nil
}

func friendFromProto(f *pb.Friend) net.FriendRecord {
	record := net.FriendRecord{
		FriendshipID: f.GetFriendshipId(),
		AccountID:    f.GetAccountId(),
		Name:         f.GetName(),
		Status:       f.GetStatus(),
		Online:       f.GetOnline(),
		InstanceCode: f.GetInstanceCode(),
		MapCode:      f.GetMapCode(),
		Note:         f.GetNote(),
		Avatar:       f.GetAvatar(),
	}
	if ms := f.GetExpiresAtMs(); ms > 0 {
		record.ExpiresAt = time.UnixMilli(ms)
	}
	return record
}

func stackToProto(st persist.Stack) *pb.PlayerStack {
	return &pb.PlayerStack{ItemId: st.ItemID, OlCid: st.CID, Quantity: uint32(st.Quantity), Slot: int32(st.Slot), Active: st.Active}
}

func stackFromProto(st *pb.PlayerStack) persist.Stack {
	return persist.Stack{ItemID: st.GetItemId(), CID: st.GetOlCid(), Quantity: int(st.GetQuantity()), Slot: int(st.GetSlot()), Active: st.GetActive()}
}

func stacksToProto(stacks []persist.Stack) []*pb.PlayerStack {
	out := make([]*pb.PlayerStack, 0, len(stacks))
	for _, st := range stacks {
		out = append(out, stackToProto(st))
	}
	return out
}

func stacksFromProto(stacks []*pb.PlayerStack) []persist.Stack {
	out := make([]persist.Stack, 0, len(stacks))
	for _, st := range stacks {
		out = append(out, stackFromProto(st))
	}
	return out
}

func stateToProto(s persist.State) *pb.PlayerState {
	out := &pb.PlayerState{
		Coins:    s.Coins,
		Xp:       s.XP,
		Bag:      stacksToProto(s.Bag),
		Storage:  stacksToProto(s.Storage),
		Position: &pb.PlayerPosition{MapCode: s.Position.MapCode, X: s.Position.X, Y: s.Position.Y},
	}
	for _, v := range s.Vaults {
		out.Vaults = append(out.Vaults, &pb.PlayerVault{ActionCode: v.ActionCode, Stacks: stacksToProto(v.Stacks)})
	}
	for _, h := range s.Held {
		out.Held = append(out.Held, &pb.HeldStack{Stack: stackToProto(h.Stack), Container: h.Container, ActionCode: h.ActionCode, Reason: h.Reason})
	}
	for _, q := range s.Quests {
		quest := &pb.PlayerQuest{QuestCode: q.Code, Status: q.Status, StartedAtMs: q.StartedAtMs, CompletedAtMs: q.CompletedAtMs}
		for _, step := range q.Steps {
			st := &pb.QuestStepState{StepId: step.StepID}
			for _, o := range step.Objectives {
				st.Objectives = append(st.Objectives, &pb.QuestObjectiveState{Current: uint32(o.Current), Required: uint32(o.Required)})
			}
			quest.Steps = append(quest.Steps, st)
		}
		out.Quests = append(out.Quests, quest)
	}
	return out
}

func stateFromProto(s *pb.PlayerState) persist.State {
	out := persist.State{
		Coins:   s.GetCoins(),
		XP:      s.GetXp(),
		Bag:     stacksFromProto(s.GetBag()),
		Storage: stacksFromProto(s.GetStorage()),
	}
	if p := s.GetPosition(); p != nil {
		out.Position = persist.Position{MapCode: p.GetMapCode(), X: p.GetX(), Y: p.GetY()}
	}
	for _, v := range s.GetVaults() {
		out.Vaults = append(out.Vaults, persist.Vault{ActionCode: v.GetActionCode(), Stacks: stacksFromProto(v.GetStacks())})
	}
	for _, h := range s.GetHeld() {
		out.Held = append(out.Held, persist.Held{Stack: stackFromProto(h.GetStack()), Container: h.GetContainer(), ActionCode: h.GetActionCode(), Reason: h.GetReason()})
	}
	for _, q := range s.GetQuests() {
		quest := persist.Quest{Code: q.GetQuestCode(), Status: q.GetStatus(), StartedAtMs: q.GetStartedAtMs(), CompletedAtMs: q.GetCompletedAtMs()}
		for _, step := range q.GetSteps() {
			st := persist.QuestStep{StepID: step.GetStepId()}
			for _, o := range step.GetObjectives() {
				st.Objectives = append(st.Objectives, persist.Objective{Current: int(o.GetCurrent()), Required: int(o.GetRequired())})
			}
			quest.Steps = append(quest.Steps, st)
		}
		out.Quests = append(out.Quests, quest)
	}
	return out
}
