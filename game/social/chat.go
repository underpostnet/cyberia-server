// Package social — chat.go
//
// Chat channels. The server names the sender from its own session, cleans the
// line once, and echoes it to the sender too, so every client draws only what
// the server accepted. The recipients of each channel:
//
//	world  — every player on this server
//	party  — the members of the sender's party
//	direct — one player on the sender's map, or an accepted friend anywhere on
//	         this server, and the sender
//
// Cross-process contract:
//
//	chat (client→server)     {channel, toId, text}
//	chat (server→client)     {channel, fromId, fromName, fromAvatar, toId, text, atMs}
//	chat_ack (server→client) {ok, reason, channel} — sent on a refusal only
//
// deliver is the one place a line reaches a recipient; a future mute or
// ignore list filters there.
package social

import (
	"strings"
	"time"
	"unicode"
	"unicode/utf8"

	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// A sender spends one line of its budget per message. The budget holds
// chatBurst lines and gets one back every chatRefill.
const (
	chatBurst  = 5
	chatRefill = 1500 * time.Millisecond
)

type chatBudget struct {
	lines   float64
	at      time.Time
	limited bool // the last line was refused; logs one refusal per run
}

// spend takes one line from the budget and reports whether there was one.
func (b *chatBudget) spend(now time.Time) bool {
	b.lines = min(chatBurst, b.lines+now.Sub(b.at).Seconds()/chatRefill.Seconds())
	b.at = now
	if b.lines < 1 {
		return false
	}
	b.lines--
	return true
}

// ChatLine is the `chat` message payload.
type ChatLine struct {
	Channel  string         `json:"channel"`
	FromID   world.EntityID `json:"fromId"`
	FromName string         `json:"fromName"`
	// FromAvatar is the identity preview reference of what the sender wears.
	FromAvatar string         `json:"fromAvatar,omitempty"`
	ToID       world.EntityID `json:"toId,omitempty"`
	Text       string         `json:"text"`
	AtMs       int64          `json:"atMs"` // the server time of the line, Unix ms
}

// CleanText is a line as it may travel: no control or format characters,
// single spaces, at most maxRunes runes.
func CleanText(text string, maxRunes int) string {
	var b strings.Builder
	for _, r := range text {
		switch {
		case r == utf8.RuneError || unicode.Is(unicode.Cf, r):
		case unicode.IsControl(r):
			b.WriteRune(' ')
		default:
			b.WriteRune(r)
		}
	}
	line := strings.Join(strings.Fields(b.String()), " ")
	runes := 0
	for i := range line {
		if runes == maxRunes {
			return strings.TrimSpace(line[:i])
		}
		runes++
	}
	return line
}

// Chat sends one clean line from sender, drawn by avatar, on channel. A
// direct line goes to target: a player on the sender's map, or a friend on
// any map. It returns "" or the reason it refused.
func (s *Social) Chat(sender *world.Session, avatar string, channel world.ChatChannel, target *world.Session, text string, now time.Time) string {
	var recipients []*world.Session
	line := ChatLine{Channel: channel.String(), FromID: sender.ID, FromName: sender.DisplayName, FromAvatar: avatar, Text: text, AtMs: now.UnixMilli()}
	switch channel {
	case world.ChatWorld:
		for _, p := range s.online {
			recipients = append(recipients, p)
		}
	case world.ChatParty:
		party := s.parties[sender.PlayerID]
		if party == nil {
			return ReasonNoParty
		}
		for _, id := range party.Members {
			recipients = append(recipients, s.online[id])
		}
	case world.ChatDirect:
		if target == nil || target.PlayerID == sender.PlayerID ||
			(target.MapCode != sender.MapCode && !s.Friends(sender, target)) {
			return ReasonUnavailable
		}
		line.ToID = target.ID
		recipients = []*world.Session{sender, target}
	}
	if !s.budget(sender.PlayerID, now).spend(now) {
		if b := s.budgets[sender.PlayerID]; !b.limited {
			b.limited = true
			logx.Infof("[CHAT] rate limited player=%s channel=%s", sender.ID, channel)
		}
		return ReasonRateLimited
	}
	s.budgets[sender.PlayerID].limited = false
	for _, p := range recipients {
		deliver(p, line)
	}
	return ""
}

func (s *Social) budget(id world.PlayerID, now time.Time) *chatBudget {
	b := s.budgets[id]
	if b == nil {
		b = &chatBudget{lines: chatBurst, at: now}
		s.budgets[id] = b
	}
	return b
}

func deliver(recipient *world.Session, line ChatLine) {
	world.SendMessage(recipient, "chat", line)
}
