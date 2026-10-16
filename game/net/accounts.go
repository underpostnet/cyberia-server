// Package net — accounts.go
//
// The session modes and the engine side of launches. In account mode a
// connection presents a single-use launch ticket before it becomes a player:
// an account ticket plays the persistent profile, which the persist pipeline
// commits; a guest ticket plays an ephemeral session. In anonymous mode every
// connection plays as a guest and nothing persists.
package net

import (
	"context"
	"regexp"
	"time"

	"cyberia-server/game/persist"
)

// SessionMode is how a connection becomes a player.
type SessionMode string

const (
	// SessionAnonymous admits every connection as a guest of that connection.
	SessionAnonymous SessionMode = "anonymous"
	// SessionAccount admits a connection with a launch ticket of an account or
	// of a guest.
	SessionAccount SessionMode = "account"
)

// Principals a launch ticket names.
const (
	PrincipalAccount = "account"
	PrincipalGuest   = "guest"
)

// ParseSessionMode maps a configured word to its mode.
func ParseSessionMode(word string) (SessionMode, bool) {
	switch SessionMode(word) {
	case SessionAnonymous, SessionAccount:
		return SessionMode(word), true
	}
	return "", false
}

// Refusals a connection gets before it plays, and the reasons a session ends.
// session_end carries them.
const (
	EndAuthRequired = "auth_required"
	EndInvalid      = "invalid"
	EndUnavailable  = "unavailable"
	EndElsewhere    = persist.EndedElsewhere
	EndRevoked      = persist.EndedRevoked
	EndRestart      = "server_restart"
	EndLeft         = "left"
	EndLeftSaving   = "left_saving"
)

// ticketPattern is the shape of a launch ticket: 32 random bytes in base64url.
var ticketPattern = regexp.MustCompile(`^[A-Za-z0-9_-]{43}$`)

// Account is a consumed launch ticket. Refusal is empty when accepted; the
// account fields are set for PrincipalAccount only.
type Account struct {
	Refusal     string
	Principal   string
	AccountID   string
	DisplayName string
}

// ResumeTicket is the next launch ticket of a live account session, or the
// refusal of a session that ended.
type ResumeTicket struct {
	Refusal   string
	Ticket    string
	ExpiresAt time.Time
}

// AcquiredSession is the lease and the saved state of a profile. Refusal is
// empty when acquired; Created marks a profile with no committed state;
// LeaseLength is how long the lease lives without a checkpoint.
type AcquiredSession struct {
	Refusal     string
	Lease       persist.Lease
	State       persist.State
	Created     bool
	LeaseLength time.Duration
}

// FriendRecord is one friendship or request of an account, as the engine
// returns it.
type FriendRecord struct {
	FriendshipID string
	AccountID    string
	Name         string
	Status       string // friend | incoming | outgoing
	Online       bool
	InstanceCode string
	MapCode      string
	ExpiresAt    time.Time
}

// Accounts is the engine side of account sessions and friendships. A refusal
// is a value; an error is a transport failure.
type Accounts interface {
	ConsumeTicket(ctx context.Context, ticket, instanceCode, connectionID, serverID string) (Account, error)
	IssueResume(ctx context.Context, lease persist.Lease, instanceCode string) (ResumeTicket, error)
	AcquireSession(ctx context.Context, connectionID, instanceCode, serverID string) (AcquiredSession, error)
	ListFriends(ctx context.Context, accountID string) ([]FriendRecord, error)
	UpdateFriendship(ctx context.Context, accountID, action, targetAccountID, friendshipID string) (string, *FriendRecord, error)
}
