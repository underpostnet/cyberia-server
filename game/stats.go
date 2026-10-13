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
func (s *Content) statBreakdown(id EntityID, mapState *MapState, visited map[EntityID]bool) StatBreakdown {
	var out StatBreakdown
	if visited[id] {
		return out
	}
	visited[id] = true
	defer delete(visited, id)
	progression := s.entityProgression(id, mapState)
	out.Base = s.progressionConfig().BaseAtLevel(progression.Level)
	var layers, temporary [StatCount]float64
	for _, layer := range mapState.ap(id).Layers {
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
	if mapState.has(id, maskAgent) {
		// A summon inherits the stats of a caster still on its map.
		if caster := mapState.ag(id).CasterID; mapState.HasMortal(caster) {
			inherited := s.statBreakdown(caster, mapState, visited)
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

func (s *Content) CalculateStats(id EntityID, mapState *MapState) ComputedStats {
	return s.statBreakdown(id, mapState, make(map[EntityID]bool)).Effective
}

// ApplyResistanceStat keeps life within the effective maximum.
func (s *Content) ApplyResistanceStat(id EntityID, mapState *MapState) {
	base := mapState.mt(id).BaseMaxLife
	if base <= 0 {
		base = s.entityBaseMaxLife
	}
	v := mapState.vt(id)
	v.MaxLife = max(1, base+s.CalculateStats(id, mapState).Resistance)
	v.Life = min(v.Life, v.MaxLife)
}

// chance is a probability the stats raised from `base`, capped so nothing the
// stats touch becomes a certainty. A configured cap outside (0, 1] means 1.
func (s *Content) chance(base, bonus float64) float64 {
	cap := s.maxChance
	if cap <= 0 || cap > 1 {
		cap = 1
	}
	return math.Max(0, math.Min(base+bonus, cap))
}

// summonChance is the chance a skill summons, raised by intelligence.
func (s *Content) summonChance(base float64, stats ComputedStats) float64 {
	return s.chance(base, stats.Intelligence*statScales.Intelligence)
}

// summonLifetime is how long a summon lives, extended by range.
func summonLifetime(baseMs int, stats ComputedStats) time.Duration {
	return time.Duration(float64(baseMs)+stats.Range*statScales.Range) * time.Millisecond
}

// CalculateActionCooldown is the effective action cooldown, shortened by utility
// down to the configured minimum.
func (s *Content) CalculateActionCooldown(stats ComputedStats) time.Duration {
	factor := math.Max(0, 1.0-stats.Utility*statScales.Utility)
	cooldown := time.Duration(float64(s.entityBaseActionCooldown) * factor)
	if cooldown < s.entityBaseMinActionCooldown {
		return s.entityBaseMinActionCooldown
	}
	return cooldown
}

// CalculateMovementSpeed is the effective movement speed in grid units per
// second, scaled by agility.
func (s *Content) CalculateMovementSpeed(stats ComputedStats) float64 {
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
func (s *Content) CalculatePlayerMovementSpeed(stats ComputedStats) float64 {
	base := s.playerBaseSpeed
	if base <= 0 {
		base = s.entityBaseSpeed
	}
	return base * (1.0 + stats.Agility*statScales.Agility)
}
