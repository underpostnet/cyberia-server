package social

import (
	"fmt"
	"strings"
	"testing"
	"time"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// inbox records what the server sends one session.
type inbox struct{ lines []ChatLine }

func (b *inbox) Send(msgType string, payload any) {
	if line, ok := payload.(ChatLine); ok && msgType == "chat" {
		b.lines = append(b.lines, line)
	}
}

// players joins n sessions to a fresh registry.
func players(n int) (*Social, []*world.Session, []*inbox) {
	s := New()
	sessions := make([]*world.Session, n)
	inboxes := make([]*inbox, n)
	for i := range sessions {
		inboxes[i] = &inbox{}
		sessions[i] = &world.Session{ID: world.EntityID(i + 1), PlayerID: world.PlayerID(fmt.Sprintf("%08x-aaaa", i)), Client: inboxes[i]}
		s.Join(sessions[i])
	}
	return s, sessions, inboxes
}

func members(s *Social, p *world.Session) []world.PlayerID {
	if party := s.PartyOf(p.PlayerID); party != nil {
		return party.Members
	}
	return nil
}

func TestGuestNamesAreUniqueAmongOnlinePlayers(t *testing.T) {
	s := New()
	a := &world.Session{PlayerID: "abcd1234-0000"}
	b := &world.Session{PlayerID: "abcd5678-0000"}
	s.Join(a)
	s.Join(b)
	if a.DisplayName != "Guest-ABCD" || b.DisplayName != "Guest-ABCD56" {
		t.Fatalf("names: %q %q", a.DisplayName, b.DisplayName)
	}
}

func TestAnAcceptedInviteFoundsAPartyLedByTheInviter(t *testing.T) {
	s, p, _ := players(3)
	now := time.Now()
	if reason := s.Invite(p[0], p[1], now); reason != "" {
		t.Fatalf("invite: %s", reason)
	}
	if !s.Invited(p[0].PlayerID, p[1].PlayerID, now) || len(s.InvitesTo(p[1].PlayerID, now)) != 1 {
		t.Fatal("the invite waits for its answer")
	}
	if s.PartyOf(p[0].PlayerID) != nil {
		t.Fatal("an invite alone founds nothing")
	}
	if reason := s.Reply(p[1], p[0], true, now); reason != "" {
		t.Fatalf("accept: %s", reason)
	}
	party := s.PartyOf(p[0].PlayerID)
	if party == nil || party.Leader() != p[0].PlayerID || s.PartyOf(p[1].PlayerID) != party {
		t.Fatalf("party: %+v", party)
	}
	if reason := s.Invite(p[1], p[2], now); reason != ReasonNotLeader {
		t.Fatalf("only the leader invites, got %q", reason)
	}
	if reason := s.Reply(p[1], p[0], true, now); reason != ReasonExpired {
		t.Fatalf("an answered invite cannot be answered again, got %q", reason)
	}
}

func TestInvitesExpireAndCanBeDeclined(t *testing.T) {
	s, p, _ := players(3)
	now := time.Now()
	s.Invite(p[0], p[1], now)
	if reason := s.Reply(p[1], p[0], true, now.Add(InviteTTL)); reason != ReasonExpired {
		t.Fatalf("an invite past its TTL is gone, got %q", reason)
	}
	s.Invite(p[0], p[2], now)
	if reason := s.Reply(p[2], p[0], false, now); reason != "" || s.PartyOf(p[2].PlayerID) != nil {
		t.Fatalf("a decline seats nobody: %q", reason)
	}
}

func TestAPartyHoldsAtMostItsCapAndDropsTheLeadersInvitesWhenFull(t *testing.T) {
	s, p, _ := players(content.PartyMaxMembers + 1)
	now := time.Now()
	for i := 1; i < content.PartyMaxMembers-1; i++ {
		s.Invite(p[0], p[i], now)
		s.Reply(p[i], p[0], true, now)
	}
	last, extra := p[content.PartyMaxMembers-1], p[content.PartyMaxMembers]
	s.Invite(p[0], last, now)
	s.Invite(p[0], extra, now)
	if reason := s.Reply(last, p[0], true, now); reason != "" {
		t.Fatalf("the last seat: %s", reason)
	}
	if got := len(members(s, p[0])); got != content.PartyMaxMembers {
		t.Fatalf("party size %d, want %d", got, content.PartyMaxMembers)
	}
	if s.Invited(p[0].PlayerID, extra.PlayerID, now) {
		t.Fatal("a full party drops its leader's pending invites")
	}
	if reason := s.Invite(p[0], extra, now); reason != ReasonPartyFull {
		t.Fatalf("a full party invites nobody, got %q", reason)
	}
}

func TestLeaderSuccessionKickDisbandAndDisconnect(t *testing.T) {
	s, p, _ := players(4)
	now := time.Now()
	for i := 1; i < 4; i++ {
		s.Invite(p[0], p[i], now)
		s.Reply(p[i], p[0], true, now)
	}
	if reason := s.Kick(p[1], p[2]); reason != ReasonNotLeader {
		t.Fatalf("a member cannot kick, got %q", reason)
	}
	if reason := s.Kick(p[0], p[3]); reason != "" || s.PartyOf(p[3].PlayerID) != nil {
		t.Fatalf("the leader kicks: %q", reason)
	}
	s.Leave(p[0])
	if party := s.PartyOf(p[1].PlayerID); party == nil || party.Leader() != p[1].PlayerID {
		t.Fatal("the next member in join order leads after the leader disconnects")
	}
	if reason := s.Disband(p[2]); reason != ReasonNotLeader {
		t.Fatalf("a member cannot disband, got %q", reason)
	}
	if reason := s.Disband(p[1]); reason != "" || s.PartyOf(p[1].PlayerID) != nil || s.PartyOf(p[2].PlayerID) != nil {
		t.Fatalf("disband ends the party: %q", reason)
	}
	if reason := s.LeaveParty(p[1]); reason != ReasonNoParty {
		t.Fatalf("leaving no party, got %q", reason)
	}
}

func TestAPartyOfOneEnds(t *testing.T) {
	s, p, _ := players(2)
	now := time.Now()
	s.Invite(p[0], p[1], now)
	s.Reply(p[1], p[0], true, now)
	s.LeaveParty(p[1])
	if s.PartyOf(p[0].PlayerID) != nil {
		t.Fatal("the last member is in no party")
	}
}

func TestDisconnectDropsEveryInviteToAndFromThePlayer(t *testing.T) {
	s, p, _ := players(3)
	now := time.Now()
	s.Invite(p[0], p[1], now)
	s.Invite(p[2], p[0], now)
	s.Leave(p[0])
	if len(s.InvitesTo(p[1].PlayerID, now)) != 0 || s.Invited(p[2].PlayerID, p[0].PlayerID, now) {
		t.Fatal("a gone player's invites lapse")
	}
	if s.Player(p[0].PlayerID) != nil {
		t.Fatal("a gone player is not online")
	}
}

func TestATargetHoldsABoundedNumberOfInvites(t *testing.T) {
	s, p, _ := players(content.PartyInvitesMax + 2)
	now := time.Now()
	target := p[len(p)-1]
	for i := 0; i < content.PartyInvitesMax; i++ {
		if reason := s.Invite(p[i], target, now); reason != "" {
			t.Fatalf("invite %d: %s", i, reason)
		}
	}
	if reason := s.Invite(p[content.PartyInvitesMax], target, now); reason != ReasonBusy {
		t.Fatalf("one invite too many, got %q", reason)
	}
	if reason := s.Invite(p[0], target, now); reason != "" {
		t.Fatalf("a renewal is no new invite, got %q", reason)
	}
}

func TestAnInviterHoldsABoundedNumberOfInvites(t *testing.T) {
	s, p, _ := players(content.PartyInvitesMax + 2)
	now := time.Now()
	inviter := p[0]
	for i := 1; i <= content.PartyInvitesMax; i++ {
		if reason := s.Invite(inviter, p[i], now.Add(time.Duration(i)*time.Second)); reason != "" {
			t.Fatalf("invite %d: %s", i, reason)
		}
	}
	sent := s.InvitesFrom(inviter.PlayerID, now.Add(time.Minute))
	if len(sent) != content.PartyInvitesMax || sent[0].To != p[1].PlayerID {
		t.Fatalf("the sent invites, the one that expires first first: %+v", sent)
	}
	if reason := s.Invite(inviter, p[len(p)-1], now); reason != ReasonInvitesFull {
		t.Fatalf("one invite too many, got %q", reason)
	}
	if reason := s.Invite(inviter, p[1], now); reason != "" {
		t.Fatalf("a renewal is no new invite, got %q", reason)
	}
}

func TestChatChannelsReachTheirMembersOnly(t *testing.T) {
	s, p, in := players(3)
	now := time.Now()
	s.Invite(p[0], p[1], now)
	s.Reply(p[1], p[0], true, now)

	if reason := s.Chat(p[0], "", world.ChatWorld, nil, "hello", now); reason != "" {
		t.Fatalf("world: %s", reason)
	}
	if reason := s.Chat(p[0], "", world.ChatParty, nil, "team", now); reason != "" {
		t.Fatalf("party: %s", reason)
	}
	if reason := s.Chat(p[2], "", world.ChatParty, nil, "spy", now); reason != ReasonNoParty {
		t.Fatalf("a player in no party has no party channel, got %q", reason)
	}
	if reason := s.Chat(p[2], "", world.ChatDirect, p[1], "psst", now); reason != "" {
		t.Fatalf("direct: %s", reason)
	}

	texts := func(b *inbox) string {
		var out []string
		for _, l := range b.lines {
			out = append(out, l.Channel+":"+l.Text)
		}
		return strings.Join(out, ",")
	}
	if got := texts(in[0]); got != "world:hello,party:team" {
		t.Fatalf("sender: %s", got)
	}
	if got := texts(in[1]); got != "world:hello,party:team,direct:psst" {
		t.Fatalf("member: %s", got)
	}
	if got := texts(in[2]); got != "world:hello,direct:psst" {
		t.Fatalf("outsider: %s", got)
	}
	if line := in[1].lines[0]; line.FromID != p[0].ID || line.FromName != p[0].DisplayName {
		t.Fatalf("the server names the sender: %+v", line)
	}
	if line := in[1].lines[2]; line.ToID != p[1].ID {
		t.Fatalf("a direct line names its target: %+v", line)
	}
}

func TestChatBudgetRefusesFloodsAndRefills(t *testing.T) {
	s, p, in := players(1)
	now := time.Now()
	for i := 0; i < chatBurst; i++ {
		if reason := s.Chat(p[0], "", world.ChatWorld, nil, "spam", now); reason != "" {
			t.Fatalf("line %d: %s", i, reason)
		}
	}
	if reason := s.Chat(p[0], "", world.ChatWorld, nil, "spam", now); reason != ReasonRateLimited {
		t.Fatalf("a spent budget refuses, got %q", reason)
	}
	if reason := s.Chat(p[0], "", world.ChatWorld, nil, "later", now.Add(chatRefill)); reason != "" {
		t.Fatalf("the budget refills: %s", reason)
	}
	if len(in[0].lines) != chatBurst+1 {
		t.Fatalf("a refused line reaches nobody: %d lines", len(in[0].lines))
	}
}

func TestCleanTextKeepsPrintableTextWithinTheLimit(t *testing.T) {
	cases := map[string]string{
		"  hello   world  ": "hello world",
		"line\nbreak\ttab":  "line break tab",
		"rtl‮override":      "rtloverride",
		"zero​width":        "zerowidth",
		"\x00\x01\x02":      "",
		"bad\xffbyte":       "badbyte",
		"🙂 ok":              "🙂 ok",
		"   ":               "",
	}
	for in, want := range cases {
		if got := CleanText(in); got != want {
			t.Fatalf("CleanText(%q) = %q, want %q", in, got, want)
		}
	}
	long := strings.Repeat("🙂", content.ChatTextMaxRunes*2)
	if got := []rune(CleanText(long)); len(got) != content.ChatTextMaxRunes {
		t.Fatalf("a long line keeps %d whole runes, got %d", content.ChatTextMaxRunes, len(got))
	}
}

// accounts joins two account players, one on map "a" and one on map "b".
func accounts() (*Social, *world.Session, *world.Session, *inbox, *inbox) {
	s := New()
	ia, ib := &inbox{}, &inbox{}
	a := &world.Session{ID: 1, PlayerID: "prof-a", AccountID: "acc-a", DisplayName: "ana", MapCode: "a", Client: ia}
	b := &world.Session{ID: 2, PlayerID: "prof-b", AccountID: "acc-b", DisplayName: "bo", MapCode: "b", Client: ib}
	s.Join(a)
	s.Join(b)
	return s, a, b, ia, ib
}

func TestAccountPlayersKeepTheirNamesAndAreFoundByAccount(t *testing.T) {
	s, a, b, _, _ := accounts()
	if a.DisplayName != "ana" || s.Account("acc-b") != b {
		t.Fatalf("an account player keeps its name and is found by account: %q %v", a.DisplayName, s.Account("acc-b"))
	}
	s.Leave(b)
	if s.Account("acc-b") != nil {
		t.Fatal("a player that left is no longer online")
	}
}

func TestADirectLineCrossesMapsOnlyBetweenFriends(t *testing.T) {
	s, a, b, _, ib := accounts()
	now := time.Now()
	if reason := s.Chat(a, "", world.ChatDirect, b, "hi", now); reason != ReasonUnavailable {
		t.Fatalf("a stranger on another map is out of reach, got %q", reason)
	}
	s.SetFriends(a.PlayerID, []string{"acc-b"})
	s.SetFriendship("acc-a", "acc-b", true)
	if reason := s.Chat(a, "skin-a", world.ChatDirect, b, "hi", now); reason != "" || len(ib.lines) != 1 {
		t.Fatalf("a friend on another map gets the line: reason=%q lines=%d", reason, len(ib.lines))
	}
	if ib.lines[0].FromSkin != "skin-a" {
		t.Fatalf("the line names the skin that draws its sender: %+v", ib.lines[0])
	}
	s.SetFriendship("acc-a", "acc-b", false)
	if s.Friends(a, b) || s.Friends(b, a) {
		t.Fatal("a removed friendship ends on both sides")
	}
}

func TestFriendActionsNeedAnAccountAndABudget(t *testing.T) {
	s, a, _, _, _ := accounts()
	guest := &world.Session{ID: 3, PlayerID: "guest-1"}
	s.Join(guest)
	now := time.Now()
	if reason := s.SpendFriendAction(guest, now); reason != ReasonGuest {
		t.Fatalf("a guest has no friends, got %q", reason)
	}
	for i := range friendBurst {
		if reason := s.SpendFriendAction(a, now); reason != "" {
			t.Fatalf("action %d within the burst refused: %q", i, reason)
		}
	}
	if reason := s.SpendFriendAction(a, now); reason != ReasonRateLimited {
		t.Fatalf("an action past the burst waits, got %q", reason)
	}
	if reason := s.SpendFriendAction(a, now.Add(friendRefill)); reason != "" {
		t.Fatalf("the budget refills, got %q", reason)
	}
}
