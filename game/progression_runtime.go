package game

import (
	"fmt"
	"time"

	pb "cyberia-server/gen/proto"
)

func progressionRulesFromProto(p *pb.ProgressionRules) ProgressionRules {
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

func (s *Content) progressionConfig() ProgressionRules {
	if s.progressionRules.MaxLevel == 0 {
		return defaultProgressionRules()
	}
	return s.progressionRules
}

func (s *Content) entityProgression(id EntityID, ms *MapState) *EntityProgression {
	p := s.progression[id]
	if p == nil {
		p = s.setProgression(id, EntityProgression{})
	}
	if p.Level == 0 {
		player := ms.has(id, maskPlayer)
		level := s.progressionConfig().DefaultBotLevel
		if player {
			level = 1
		}
		*p = NewEntityProgression(level, !player, s.progressionConfig())
	}
	rules := s.progressionConfig()
	if p.Frozen {
		p.Level = max(1, min(p.Level, rules.MaxLevel))
		p.XP = rules.Threshold(p.Level)
	} else {
		p.XP = min(p.XP, rules.Threshold(rules.MaxLevel))
		p.Level = rules.LevelAtXP(p.XP)
	}
	return p
}

// setProgression stores the progression of id and returns it.
func (s *Content) setProgression(id EntityID, p EntityProgression) *EntityProgression {
	if s.progression == nil {
		s.progression = make(map[EntityID]*EntityProgression)
	}
	s.progression[id] = &p
	return &p
}

// despawn frees the slot of id and its content rows. A move between maps
// uses MapState.take, which keeps them.
func (s *Content) despawn(ms *MapState, id EntityID) {
	ms.despawn(id)
	delete(s.progression, id)
	delete(s.quests, id)
	delete(s.crafts, id)
}

// forgetNonPlayers drops the content rows of every entity without a session.
// A world rebuild drops every entity but the players.
func (s *Content) forgetNonPlayers() {
	for id := range s.progression {
		if s.sessions[id] == nil {
			delete(s.progression, id)
		}
	}
}

// awardProgression runs under the simulation lock after a gameplay result.
// XP is private, so its floating text goes to the earner alone; a level gained
// is public, so its sound reaches every viewer in the earner's area.
func (s *Content) awardProgression(player *Session, event progressionEvent, key string, targetLevel int, now time.Time) uint64 {
	ms := s.maps[player.MapCode]
	p := s.entityProgression(player.ID, ms)
	before := p.Level
	amount := p.award(s.progressionConfig(), event, key, targetLevel, now)
	if amount == 0 {
		return 0
	}
	s.ApplyResistanceStat(player.ID, ms)
	tf := ms.tf(player.ID)
	sendMessage(player, "combat_text", CombatText{Kind: FCTXp, WorldX: tf.Pos.X, WorldY: tf.Pos.Y, Value: int(amount)})
	if p.Level > before {
		s.broadcastAudioEvent(ms, AudioEventLevelUp, Vec2{X: tf.Pos.X + tf.Size.X*0.5, Y: tf.Pos.Y + tf.Size.Y*0.5})
	}
	return amount
}

// awardDefeat uses damage shares and excludes summoned entities, self kills
// and obstacles.
func (s *Content) awardDefeat(victim EntityID, kind string, ms *MapState) {
	if ms == nil || ms.vt(victim).ghost() || kind == "obstacle" {
		return
	}
	if ms.has(victim, maskAgent) {
		if a := ms.ag(victim); a.CasterID != 0 || behaviorIsProvider(a.Behavior) || a.Behavior == BehaviorDrop {
			return
		}
	}
	ledger := ms.mt(victim).DamageLedger
	level := 1
	if p := s.progression[victim]; p != nil {
		level = max(1, p.Level)
	}
	var total float64
	for _, damage := range ledger {
		if damage > 0 {
			total += damage
		}
	}
	for id, damage := range ledger {
		if id == victim || damage <= 0 || damage < total*0.1 {
			continue
		}
		entity := s.playerOn(ms, id)
		if entity == nil || ms.vt(entity.ID).ghost() || entity.Frozen {
			continue
		}
		key := fmt.Sprintf("%s:%d", kind, victim)
		event := xpKill
		if kind == "resource" {
			event = xpObjective
		}
		s.awardProgression(entity, event, key, level, time.Now())
	}
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
	rules := progressionRulesFromProto(config.GetProgressionRules())
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
