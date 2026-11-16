package sim

import (
	"math"

	"cyberia-server/game/content"
	"cyberia-server/game/world"
)

// facingCone is the cosine of the widest angle between the facing of a caster
// and a target that still counts as ahead of it (67.5°: three facings).
const facingCone = 0.38268343236

// Tiers of an auto-target candidate; a lower tier wins.
const (
	tierEngaged = iota // a bot that pursues the caster, or an entity that hurt it
	tierAhead          // within the facing cone of the caster
	tierAround         // anywhere else in reach
)

// autoTarget is the target an auto-aimed skill of caster takes within reach:
// the preferred target while it stays a candidate, else the candidate of the
// lowest tier, then the nearest, then the lowest id. 0 when no candidate is in
// reach. The choice reads the world only, so it is the same for the same world.
//
// A candidate is a living hostile bot, or a player that hurt the caster. It is
// never the caster, a summon of its owner, a party friend, or a player in stasis.
func (s *Sim) autoTarget(ms *world.MapState, caster, preferred world.EntityID, reach float64) world.EntityID {
	from := entityCenter(ms, caster)
	facingX, facingY := getDirectionVector(ms.Mo(caster).Direction)
	ledger := ms.Mt(caster).DamageLedger

	best, bestTier, bestDist, keep := world.EntityID(0), 0, 0.0, false
	consider := func(id world.EntityID, engaged bool) {
		if id == caster || ms.Vt(id).Ghost() || s.ownerOf(ms, id) == caster || s.friendlyHit(ms, caster, id) {
			return
		}
		to := entityCenter(ms, id)
		dx, dy := to.X-from.X, to.Y-from.Y
		dist := math.Hypot(dx, dy)
		if dist == 0 || dist > reach {
			return
		}
		keep = keep || id == preferred
		tier := tierAround
		switch {
		case engaged || ledger[id] > 0:
			tier = tierEngaged
		case (dx*facingX+dy*facingY)/dist >= facingCone:
			tier = tierAhead
		}
		if best == 0 || tier < bestTier || (tier == bestTier && (dist < bestDist || (dist == bestDist && id < best))) {
			best, bestTier, bestDist = id, tier, dist
		}
	}
	for id := range ms.OfKind(world.EntityBot) {
		if ms.Ag(id).Behavior == content.BehaviorHostile {
			consider(id, ms.Br(id).CurrentTargetPlayer == caster)
		}
	}
	for player := range s.PlayersOn(ms) {
		if ledger[player.ID] > 0 && !player.Frozen {
			consider(player.ID, true)
		}
	}
	if keep {
		return preferred
	}
	return best
}

// entityCenter is the centre of an entity in cells.
func entityCenter(ms *world.MapState, id world.EntityID) world.Vec2 {
	tf := ms.Tf(id)
	return world.Vec2{X: tf.Pos.X + tf.Size.X*0.5, Y: tf.Pos.Y + tf.Size.Y*0.5}
}
