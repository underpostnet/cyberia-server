package game

import (
	"fmt"
	"math"
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

func (s *GameServer) progressionConfig() ProgressionRules {
	if s.progressionRules.MaxLevel == 0 {
		return defaultProgressionRules()
	}
	return s.progressionRules
}

type statSource interface {
	Base() EntityBase
	StatState() *Mortal
}

func (m *Mortal) StatState() *Mortal { return m }

func (s *GameServer) entityProgression(source statSource) *EntityProgression {
	p := &source.StatState().Progression
	if p.Level == 0 {
		_, player := source.(*PlayerState)
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

// awardProgression runs under the simulation lock after a gameplay result.
// XP is private, so its floating text goes to the earner alone; a level gained
// is public, so its sound reaches every viewer in the earner's area.
func (s *GameServer) awardProgression(source statSource, event progressionEvent, key string, targetLevel int, now time.Time) uint64 {
	p := s.entityProgression(source)
	before := p.Level
	amount := p.award(s.progressionConfig(), event, key, targetLevel, now)
	if amount == 0 {
		return 0
	}
	s.ApplyResistanceStat(source, nil)
	if player, ok := source.(*PlayerState); ok {
		sendMessage(player, "combat_text", CombatText{Kind: FCTXp, WorldX: player.Pos.X, WorldY: player.Pos.Y, Value: int(amount)})
		if p.Level > before {
			if ms := s.maps[player.MapCode]; ms != nil {
				broadcastAudioEvent(ms, AudioEventLevelUp, Point{X: player.Pos.X + player.Dims.Width*0.5, Y: player.Pos.Y + player.Dims.Height*0.5})
			}
		}
	}
	return amount
}

// awardDefeat uses damage shares and excludes summoned entities and self kills.
func (s *GameServer) awardDefeat(base *EntityBase, mortal *Mortal, kind string, ledger map[string]float64, ms *MapState) {
	if ms == nil || mortal.IsGhost() {
		return
	}
	if b := ms.bots[base.ID]; b != nil && (b.CasterID != "" || behaviorIsProvider(b.Behavior) || b.Behavior == BehaviorDrop) {
		return
	}
	var total float64
	for _, damage := range ledger {
		if damage > 0 {
			total += damage
		}
	}
	for id, damage := range ledger {
		if id == base.ID || damage <= 0 || damage < total*0.1 {
			continue
		}
		var entity statSource
		if p := ms.players[id]; p != nil && !p.IsGhost() && !p.Frozen {
			entity = p
		}
		if entity == nil {
			continue
		}
		key := kind + ":" + base.ID
		event := xpKill
		if kind == "resource" {
			event = xpObjective
		}
		s.awardProgression(entity, event, key, max(1, mortal.Progression.Level), time.Now())
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

// SetTemporaryModifier replaces one effect under the simulation lock.
func (s *GameServer) SetTemporaryModifier(entity statSource, modifier TemporaryModifier, now time.Time) error {
	if modifier.ID == "" || !modifier.ExpiresAt.After(now) {
		return fmt.Errorf("invalid temporary modifier")
	}
	for _, value := range modifier.Stats.Values() {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return fmt.Errorf("non-finite temporary stat")
		}
	}
	state := entity.StatState()
	for i, current := range state.TemporaryModifiers {
		if current.ID == modifier.ID {
			state.TemporaryModifiers[i] = modifier
			return nil
		}
	}
	state.TemporaryModifiers = append(state.TemporaryModifiers, modifier)
	return nil
}

func (s *GameServer) refreshEntityStats(entity statSource, ms *MapState, now time.Time) {
	state := entity.StatState()
	kept := state.TemporaryModifiers[:0]
	for _, modifier := range state.TemporaryModifiers {
		if modifier.ExpiresAt.After(now) {
			kept = append(kept, modifier)
		}
	}
	clear(state.TemporaryModifiers[len(kept):])
	state.TemporaryModifiers = kept
	s.ApplyResistanceStat(entity, ms)
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
