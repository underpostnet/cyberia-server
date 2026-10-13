package content

import (
	"math"
	"time"

	"cyberia-server/game/world"
)

type StatBreakdown struct {
	Base      ComputedStats
	Layers    ComputedStats
	Temporary ComputedStats
	Effective ComputedStats
}

// StatBreakdown applies floors once, after all signed contributions.
func (s *Content) StatBreakdown(id world.EntityID, mapState *world.MapState, visited map[world.EntityID]bool) StatBreakdown {
	var out StatBreakdown
	if visited[id] {
		return out
	}
	visited[id] = true
	defer delete(visited, id)
	progression := s.EntityProgression(id, mapState)
	out.Base = s.ProgressionConfig().BaseAtLevel(progression.Level)
	var layers, temporary [StatCount]float64
	for _, layer := range mapState.Ap(id).Layers {
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
	if mapState.Has(id, world.MaskAgent) {
		// A summon inherits the stats of a caster still on its map.
		if caster := mapState.Ag(id).CasterID; mapState.HasMortal(caster) {
			inherited := s.StatBreakdown(caster, mapState, visited)
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

func (s *Content) CalculateStats(id world.EntityID, mapState *world.MapState) ComputedStats {
	return s.StatBreakdown(id, mapState, make(map[world.EntityID]bool)).Effective
}

// ApplyResistanceStat keeps life within the effective maximum.
func (s *Content) ApplyResistanceStat(id world.EntityID, mapState *world.MapState) {
	base := mapState.Mt(id).BaseMaxLife
	if base <= 0 {
		base = s.EntityBaseMaxLife
	}
	v := mapState.Vt(id)
	v.MaxLife = max(1, base+s.CalculateStats(id, mapState).Resistance)
	v.Life = min(v.Life, v.MaxLife)
}

// Chance is a probability the stats raised from `base`, capped so nothing the
// stats touch becomes a certainty. A configured cap outside (0, 1] means 1.
func (s *Content) Chance(base, bonus float64) float64 {
	cap := s.MaxChance
	if cap <= 0 || cap > 1 {
		cap = 1
	}
	return math.Max(0, math.Min(base+bonus, cap))
}

// SummonChance is the chance a skill summons, raised by intelligence.
func (s *Content) SummonChance(base float64, stats ComputedStats) float64 {
	return s.Chance(base, stats.Intelligence*StatScales.Intelligence)
}

// SummonLifetime is how long a summon lives, extended by range.
func SummonLifetime(baseMs int, stats ComputedStats) time.Duration {
	return time.Duration(float64(baseMs)+stats.Range*StatScales.Range) * time.Millisecond
}

// CalculateActionCooldown is the effective action cooldown, shortened by utility
// down to the configured minimum.
func (s *Content) CalculateActionCooldown(stats ComputedStats) time.Duration {
	factor := math.Max(0, 1.0-stats.Utility*StatScales.Utility)
	cooldown := time.Duration(float64(s.EntityBaseActionCooldown) * factor)
	if cooldown < s.EntityBaseMinActionCooldown {
		return s.EntityBaseMinActionCooldown
	}
	return cooldown
}

// CalculateMovementSpeed is the effective movement speed in grid units per
// second, scaled by agility.
func (s *Content) CalculateMovementSpeed(stats ComputedStats) float64 {
	return s.EntityBaseSpeed * (1.0 + stats.Agility*StatScales.Agility)
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
		base = s.EntityBaseSpeed
	}
	return base * (1.0 + stats.Agility*StatScales.Agility)
}
