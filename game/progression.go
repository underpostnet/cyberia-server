package game

import (
	"fmt"
	"math"
	"sort"
	"time"
)

type ProgressionRules struct {
	MaxLevel           int
	XpPerLevel         int
	BaseStats          ComputedStats
	PerLevelStats      ComputedStats
	KillXp             int
	QuestXp            int
	ObjectiveXp        int
	MinAwardIntervalMs int
	RepeatWindowMs     int
	MaxRepeatAwards    int
	MaxAwardsPerWindow int
	DefaultBotLevel    int
}

func (r ProgressionRules) Validate() error {
	if !r.validScalarBounds() || r.DefaultBotLevel > r.MaxLevel {
		return fmt.Errorf("invalid progression rules")
	}
	for _, block := range []ComputedStats{r.BaseStats, r.PerLevelStats} {
		for _, value := range block.Values() {
			if math.IsNaN(value) || math.IsInf(value, 0) || value < 0 || value > StatModifierMax || math.Trunc(value) != value {
				return fmt.Errorf("invalid base stat curve")
			}
		}
	}
	return nil
}

// Threshold returns the total XP required for a level.
func (r ProgressionRules) Threshold(level int) uint64 {
	n := uint64(max(1, min(level, r.MaxLevel)) - 1)
	return uint64(r.XpPerLevel) * n * n
}

func (r ProgressionRules) LevelAtXP(xp uint64) int {
	return max(1, sort.Search(r.MaxLevel, func(i int) bool { return r.Threshold(i+1) > xp }))
}

func (r ProgressionRules) BaseAtLevel(level int) ComputedStats {
	base, growth := r.BaseStats.Values(), r.PerLevelStats.Values()
	for i := range base {
		base[i] += float64(max(1, min(level, r.MaxLevel))-1) * growth[i]
	}
	return statsFromValues(base)
}

type xpRepeat struct {
	count int
}

type EntityProgression struct {
	Level        int    `json:"level"`
	XP           uint64 `json:"xp"`
	Frozen       bool   `json:"frozen"`
	lastAward    time.Time
	windowStart  time.Time
	windowAwards int
	repeats      map[string]xpRepeat
	completed    map[string]bool
}

func NewEntityProgression(level int, frozen bool, rules ProgressionRules) EntityProgression {
	if level == 0 {
		level = rules.DefaultBotLevel
	}
	level = max(1, min(level, rules.MaxLevel))
	return EntityProgression{Level: level, XP: rules.Threshold(level), Frozen: frozen}
}

type progressionEvent uint8

const (
	xpKill progressionEvent = iota + 1
	xpQuest
	xpObjective
)

// award accepts resolved gameplay events from the simulation.
func (p *EntityProgression) award(r ProgressionRules, event progressionEvent, key string, targetLevel int, now time.Time) uint64 {
	if p.Frozen || p.Level < 1 || p.Level >= r.MaxLevel || key == "" {
		return 0
	}
	var reward int
	switch event {
	case xpKill:
		reward = r.KillXp
	case xpQuest:
		reward = r.QuestXp
	case xpObjective:
		reward = r.ObjectiveXp
	default:
		return 0
	}
	if reward == 0 || now.Before(p.lastAward) || (event != xpQuest && now.Sub(p.lastAward) < time.Duration(r.MinAwardIntervalMs)*time.Millisecond) {
		return 0
	}
	if now.Before(p.windowStart) {
		return 0
	}
	if p.windowStart.IsZero() || now.Sub(p.windowStart) >= time.Duration(r.RepeatWindowMs)*time.Millisecond {
		p.windowStart = now
		p.windowAwards = 0
		p.repeats = make(map[string]xpRepeat)
	}
	if event != xpQuest && p.windowAwards >= r.MaxAwardsPerWindow {
		return 0
	}
	if p.completed == nil {
		p.completed = make(map[string]bool)
	}
	eventKey := fmt.Sprintf("%d:%s", event, key)
	if event == xpQuest && p.completed[eventKey] {
		return 0
	}
	repeat := p.repeats[eventKey]
	if repeat.count >= r.MaxRepeatAwards {
		return 0
	}
	reward /= repeat.count + 1
	if event != xpQuest && targetLevel < p.Level {
		reward /= p.Level - max(1, targetLevel) + 1
	}
	if reward <= 0 {
		return 0
	}
	amount := min(uint64(reward), r.Threshold(r.MaxLevel)-p.XP)
	p.XP += amount
	for p.Level < r.MaxLevel && p.XP >= r.Threshold(p.Level+1) {
		p.Level++
	}
	p.lastAward = now
	p.windowAwards++
	repeat.count++
	p.repeats[eventKey] = repeat
	if event == xpQuest {
		p.completed[eventKey] = true
	}
	return amount
}

type TemporaryModifier struct {
	ID        string
	Stats     ComputedStats
	ExpiresAt time.Time
}

func (s Stats) Validate() error {
	for i, value := range s.computed().Values() {
		if value < StatModifierMin || value > StatModifierMax {
			return fmt.Errorf("%s outside [%d, %d]", statNames[i], StatModifierMin, StatModifierMax)
		}
	}
	return nil
}

func aggregateStats(base, layers, temporary ComputedStats) ComputedStats {
	total, ol, temp := base.Values(), layers.Values(), temporary.Values()
	for i := range total {
		total[i] = math.Max(statEffectiveFloors[i], total[i]+ol[i]+temp[i])
	}
	return statsFromValues(total)
}
