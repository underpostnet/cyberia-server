package game

import (
	"math"
	"time"
)

type StatBreakdown struct {
	Base      ComputedStats
	Layers    ComputedStats
	Temporary ComputedStats
	Effective ComputedStats
}

// statBreakdown applies floors once, after all signed contributions.
func (s *GameServer) statBreakdown(source statSource, mapState *MapState, now time.Time, visited map[string]bool) StatBreakdown {
	var out StatBreakdown
	entity := source.Base()
	if visited[entity.ID] {
		return out
	}
	visited[entity.ID] = true
	defer delete(visited, entity.ID)
	state := source.StatState()
	progression := s.entityProgression(source)
	out.Base = s.progressionConfig().BaseAtLevel(progression.Level)
	var layers, temporary [StatCount]float64
	for _, layer := range entity.ObjectLayers {
		if !layer.Active || layer.Quantity <= 0 {
			continue
		}
		if data, ok := s.GetObjectLayerData(layer.ItemID); ok {
			for i, value := range data.Data.Stats.computed().Values() {
				layers[i] += value
			}
		}
	}
	out.Layers = statsFromValues(layers)
	for _, modifier := range state.TemporaryModifiers {
		if !modifier.ExpiresAt.After(now) {
			continue
		}
		for i, value := range modifier.Stats.Values() {
			temporary[i] += value
		}
	}
	if bot, ok := source.(*BotState); ok && bot.CasterID != "" && mapState != nil {
		var caster statSource
		if p := mapState.players[bot.CasterID]; p != nil {
			caster = p
		} else if b := mapState.bots[bot.CasterID]; b != nil {
			caster = b
		}
		if caster != nil {
			inherited := s.statBreakdown(caster, mapState, now, visited)
			out.Base = inherited.Base
			inheritedLayers, inheritedTemporary := inherited.Layers.Values(), inherited.Temporary.Values()
			for i := range temporary {
				temporary[i] += inheritedLayers[i] + inheritedTemporary[i]
			}
		}
	}
	out.Temporary = statsFromValues(temporary)
	out.Effective = aggregateStats(out.Base, out.Layers, out.Temporary)
	return out
}

func (s *GameServer) CalculateStats(source interface{}, mapState *MapState) ComputedStats {
	entity, ok := source.(statSource)
	if !ok {
		return ComputedStats{}
	}
	return s.statBreakdown(entity, mapState, time.Now(), make(map[string]bool)).Effective
}

// ApplyResistanceStat keeps life within the effective maximum.
func (s *GameServer) ApplyResistanceStat(entity interface{}, mapState *MapState) {
	source, ok := entity.(statSource)
	if !ok {
		return
	}
	state := source.StatState()
	base := state.BaseMaxLife
	if base <= 0 {
		base = s.entityBaseMaxLife
	}
	state.MaxLife = max(1, base+s.CalculateStats(entity, mapState).Resistance)
	state.Life = min(state.Life, state.MaxLife)
}

// chance is a probability the stats raised from `base`, capped so nothing the
// stats touch becomes a certainty. A configured cap outside (0, 1] means 1.
func (s *GameServer) chance(base, bonus float64) float64 {
	cap := s.maxChance
	if cap <= 0 || cap > 1 {
		cap = 1
	}
	return math.Max(0, math.Min(base+bonus, cap))
}

// summonChance is the chance a skill summons, raised by intelligence.
func (s *GameServer) summonChance(base float64, stats ComputedStats) float64 {
	return s.chance(base, stats.Intelligence*statScales.Intelligence)
}

// summonLifetime is how long a summon lives, extended by range.
func summonLifetime(baseMs int, stats ComputedStats) time.Duration {
	return time.Duration(float64(baseMs)+stats.Range*statScales.Range) * time.Millisecond
}

// CalculateActionCooldown is the effective action cooldown, shortened by utility
// down to the configured minimum.
func (s *GameServer) CalculateActionCooldown(stats ComputedStats) time.Duration {
	factor := math.Max(0, 1.0-stats.Utility*statScales.Utility)
	cooldown := time.Duration(float64(s.entityBaseActionCooldown) * factor)
	if cooldown < s.entityBaseMinActionCooldown {
		return s.entityBaseMinActionCooldown
	}
	return cooldown
}

// CalculateMovementSpeed is the effective movement speed in grid units per
// second, scaled by agility.
func (s *GameServer) CalculateMovementSpeed(stats ComputedStats) float64 {
	return s.entityBaseSpeed * (1.0 + stats.Agility*statScales.Agility)
}

// CalculatePlayerMovementSpeed is the movement speed for a player. Players walk
// on playerBaseSpeed so their pace can be tuned without changing bots or
// projectiles, which stay on entityBaseSpeed. An instance that leaves
// playerBaseSpeed at 0 keeps the entity speed, so the split is opt-in.
//
// Both the movement phase and the self block of the snapshot must call this:
// the client predicts with the speed the snapshot carries, so any disagreement
// shows up as a correction on every frame.
func (s *GameServer) CalculatePlayerMovementSpeed(stats ComputedStats) float64 {
	base := s.playerBaseSpeed
	if base <= 0 {
		base = s.entityBaseSpeed
	}
	return base * (1.0 + stats.Agility*statScales.Agility)
}
