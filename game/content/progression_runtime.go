package content

import (
	"fmt"
	"maps"
	"math"
	"slices"
	"time"

	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
)

func ProgressionRulesFromProto(p *pb.ProgressionRules) ProgressionRules {
	if p == nil {
		return defaultProgressionRules()
	}
	block := func(v *pb.Stats) ComputedStats {
		return ComputedStats{float64(v.GetEffect()), float64(v.GetResistance()), float64(v.GetAgility()), float64(v.GetRange()), float64(v.GetIntelligence()), float64(v.GetUtility())}
	}
	return ProgressionRules{
		MaxLevel: int(p.GetMaxLevel()), XpPerLevel: int(p.GetXpPerLevel()),
		BaseStats: block(p.GetBaseStats()), PerLevelStats: block(p.GetPerLevelStats()),
		KillXp: int(p.GetKillXp()), QuestXp: int(p.GetQuestXp()), ObjectiveXp: int(p.GetObjectiveXp()),
		MinAwardIntervalMs: int(p.GetMinAwardIntervalMs()), RepeatWindowMs: int(p.GetRepeatWindowMs()),
		MaxRepeatAwards: int(p.GetMaxRepeatAwards()), MaxAwardsPerWindow: int(p.GetMaxAwardsPerWindow()), DefaultBotLevel: int(p.GetDefaultBotLevel()),
	}
}

func (s *Content) ProgressionConfig() ProgressionRules {
	if s.progressionRules.MaxLevel == 0 {
		return defaultProgressionRules()
	}
	return s.progressionRules
}

func (s *Content) EntityProgression(id world.EntityID, ms *world.MapState) *EntityProgression {
	p := s.Progression[id]
	if p == nil {
		p = s.SetProgression(id, EntityProgression{})
	}
	if p.Level == 0 {
		player := ms.Has(id, world.MaskPlayer)
		level := s.ProgressionConfig().DefaultBotLevel
		if player {
			level = 1
		}
		*p = NewEntityProgression(level, !player, s.ProgressionConfig())
	}
	rules := s.ProgressionConfig()
	if p.Frozen {
		p.Level = max(1, min(p.Level, rules.MaxLevel))
		p.XP = rules.Threshold(p.Level)
	} else {
		p.XP = min(p.XP, rules.Threshold(rules.MaxLevel))
		p.Level = rules.LevelAtXP(p.XP)
	}
	return p
}

// SetProgression stores the progression of id and returns it.
func (s *Content) SetProgression(id world.EntityID, p EntityProgression) *EntityProgression {
	if s.Progression == nil {
		s.Progression = make(map[world.EntityID]*EntityProgression)
	}
	s.Progression[id] = &p
	return &p
}

// Despawn frees the slot of id and its content rows. A move between maps
// uses MapState.take, which keeps them.
func (s *Content) Despawn(ms *world.MapState, id world.EntityID) {
	ms.Despawn(id)
	delete(s.Progression, id)
	delete(s.quests, id)
	delete(s.crafts, id)
}

// ForgetNonPlayers drops the content rows of every entity without a session.
// A world rebuild drops every entity but the players.
func (s *Content) ForgetNonPlayers() {
	for id := range s.Progression {
		if s.Sessions[id] == nil {
			delete(s.Progression, id)
		}
	}
}

// AwardProgression runs under the simulation lock after a gameplay result.
// XP is private, so its floating text goes to the earner alone; a level gained
// is public, so its sound reaches every viewer in the earner's area.
func (s *Content) AwardProgression(player *world.Session, event progressionEvent, reward int, key string, targetLevel int, now time.Time) uint64 {
	ms := s.Maps[player.MapCode]
	p := s.EntityProgression(player.ID, ms)
	before := p.Level
	amount := p.award(s.ProgressionConfig(), event, reward, key, targetLevel, now)
	if amount == 0 {
		return 0
	}
	s.ApplyResistanceStat(player.ID, ms)
	tf := ms.Tf(player.ID)
	world.SendMessage(player, "combat_text", world.CombatText{Kind: world.FCTXp, WorldX: tf.Pos.X, WorldY: tf.Pos.Y, Value: int(amount)})
	if p.Level > before {
		s.BroadcastAudioEvent(ms, world.AudioEventLevelUp, world.Vec2{X: tf.Pos.X + tf.Size.X*0.5, Y: tf.Pos.Y + tf.Size.Y*0.5})
	}
	return amount
}

// AwardDefeat pays the XP of a defeat once to each contributor of at least 10%
// of the damage. A party earns a bot kill once: a qualifying member near the
// bot splits it among the members near the bot. party names the online members
// of a player's party, nil for none. Summons, providers, drops, self kills and
// obstacles pay nothing.
func (s *Content) AwardDefeat(victim world.EntityID, kind string, ms *world.MapState, party func(world.PlayerID) []*world.Session) {
	if ms == nil || ms.Vt(victim).Ghost() || kind == "obstacle" {
		return
	}
	if ms.Has(victim, world.MaskAgent) {
		if a := ms.Ag(victim); a.CasterID != 0 || BehaviorIsProvider(a.Behavior) || a.Behavior == BehaviorDrop {
			return
		}
	}
	ledger := ms.Mt(victim).DamageLedger
	level := 1
	if p := s.Progression[victim]; p != nil {
		level = max(1, p.Level)
	}
	var total float64
	for _, damage := range ledger {
		if damage > 0 {
			total += damage
		}
	}
	event := XpKill
	if kind == "resource" {
		event = xpObjective
	}
	pool := s.ProgressionConfig().reward(event)
	key := fmt.Sprintf("%s:%d", kind, victim)
	now := time.Now()
	paid := make(map[world.PlayerID]bool)
	for _, id := range slices.Sorted(maps.Keys(ledger)) {
		if damage := ledger[id]; id == victim || damage <= 0 || damage < total*0.1 {
			continue
		}
		earner := s.PlayerOn(ms, id)
		if !s.earnsXP(ms, earner) || paid[earner.PlayerID] {
			continue
		}
		members := party(earner.PlayerID)
		if kind != "bot" || members == nil {
			s.AwardProgression(earner, event, pool, key, level, now)
			continue
		}
		near := s.nearMembers(ms, victim, members)
		if !slices.Contains(near, earner) {
			continue
		}
		for _, member := range members {
			paid[member.PlayerID] = true
		}
		// Integer shares in party order; the first members take the remainder.
		for i, member := range near {
			share := pool / len(near)
			if i < pool%len(near) {
				share++
			}
			s.AwardProgression(member, event, share, key, level, now)
		}
	}
}

// earnsXP reports whether a player on ms can earn XP now: alive and not in stasis.
func (s *Content) earnsXP(ms *world.MapState, p *world.Session) bool {
	return p != nil && !ms.Vt(p.ID).Ghost() && !p.Frozen
}

// nearMembers is the members on ms, able to earn XP, whose centre is within
// PartyXpShareRadius cells of the centre of victim.
func (s *Content) nearMembers(ms *world.MapState, victim world.EntityID, members []*world.Session) []*world.Session {
	var near []*world.Session
	at := ms.Tf(victim).Center()
	for _, member := range members {
		if s.PlayerOn(ms, member.ID) == nil || !s.earnsXP(ms, member) {
			continue
		}
		if c := ms.Tf(member.ID).Center(); math.Hypot(c.X-at.X, c.Y-at.Y) <= PartyXpShareRadius {
			near = append(near, member)
		}
	}
	return near
}

func validateObjectLayers(cache map[string]*ObjectLayer) error {
	for id, ol := range cache {
		if ol == nil {
			return fmt.Errorf("nil object layer %s", id)
		}
		if err := ol.Data.Stats.Validate(); err != nil {
			return fmt.Errorf("object layer %s: %w", id, err)
		}
	}
	return nil
}

func ValidateWorldProgression(config *pb.InstanceConfig, maps []*pb.MapDataMessage) error {
	if config == nil {
		return fmt.Errorf("instance config is required")
	}
	rules := ProgressionRulesFromProto(config.GetProgressionRules())
	if err := rules.Validate(); err != nil {
		return err
	}
	for _, worldMap := range maps {
		for _, entity := range worldMap.GetEntities() {
			if entity.GetLevel() < 0 || int(entity.GetLevel()) > rules.MaxLevel {
				return fmt.Errorf("invalid entity level in map %s", worldMap.GetCode())
			}
		}
	}
	return nil
}
