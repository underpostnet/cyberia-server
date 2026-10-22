// Package content — world_drop.go
//
// A stack an account player drops is a world drop: an engine record holds it
// from the commit that took it from the profile until a player claims it or it
// expires. The drop is a linked operation (kind drop): the state of its owner
// without the stack commits with the new record, and its token stays closed
// until that commit applies. A pickup is one too (kind pickup): the state of
// its collector with the stack commits with the claim of the record, and the
// token stays closed while the claim waits. A record that is gone removes the
// token; any other refusal opens it again.
//
// A guest drop stays in memory. A guest never takes an account drop, and an
// account player never takes a guest drop: goods never cross between a guest
// session and a profile. At boot the server puts back every public record of
// its world that has not expired.
//
// Caller MUST hold s.mu.
package content

import (
	"time"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// Kinds of a world drop operation.
const (
	opDrop   = "drop"
	opPickup = "pickup"
)

// sharesPrincipal reports whether player may take a token a player dropped:
// an account player a world drop, a guest a guest drop. Any other token admits
// both.
func sharesPrincipal(drop *world.Brain, player *world.Session) bool {
	return drop.LootOrigin != world.LootOriginPlayer || (drop.WorldDrop != "") == (player.AccountID != "")
}

// worldDropToken is the token of the world drop operation op on map code, or
// nil when it left the map.
func (s *Content) worldDropToken(code string, token world.EntityID, op string) (*world.MapState, *world.Brain) {
	ms := s.Maps[code]
	if ms == nil || !ms.IsKind(token, world.EntityBot) || ms.Br(token).Lock.Op != op {
		return ms, nil
	}
	return ms, ms.Br(token)
}

// linkDrop starts the drop of the stack in place at pos as a world drop. It
// returns the token, or the refusal.
func (s *Content) linkDrop(player *world.Session, place Place, stack Stack, origin, pos world.Vec2) (world.EntityID, string) {
	op := newOpID(opDrop)
	w := &linkedOp{kind: opDrop}
	effect := persist.Effect{Drop: &persist.WorldDrop{
		MapCode: player.MapCode, X: pos.X, Y: pos.Y, ItemID: stack.ItemID, CID: stack.CID, Quantity: stack.Qty,
		Lifetime: lootLifetime,
	}}
	if reason := s.linkOp(player, op, Transfer{Takes: []Debit{{Place: place, Qty: stack.Qty}}}, effect, w); reason != "" {
		return 0, reason
	}
	code := player.MapCode
	token := s.placeDrop(s.Maps[code], origin, pos, stack, player.PlayerID, world.DropLock{PlayerID: string(player.PlayerID), Op: op})
	w.settled = func(outcome persist.Outcome, _ string, owner *world.Session, now time.Time) {
		ms, drop := s.worldDropToken(code, token, op)
		switch {
		case drop == nil:
		case outcome == persist.Refused:
			logx.Warnf("[LOOT] world drop token %s of %s x%d removed: its drop never applied", token, stack.ItemID, stack.Qty)
			s.recordEconomy("drop_lock", "refused")
			s.Despawn(ms, token)
		default:
			drop.Lock, drop.ExpiresAt = world.DropLock{}, now.Add(lootLifetime)
			if owner != nil {
				world.SendAck(owner, "item_drop_ack", "", map[string]interface{}{
					"dropId": token, "itemId": stack.ItemID, "quantity": stack.Qty, "status": DropPublic,
				})
			}
		}
	}
	return token, ""
}

// ClaimWorldDrop starts the pickup of a world drop by an account player. The
// token stays closed until the engine answers; a player that cannot take it
// now leaves the token open.
func (s *Content) ClaimWorldDrop(ms *world.MapState, token world.EntityID, player *world.Session) {
	drop := ms.Br(token)
	stack := Stack{ItemID: drop.DropItemID, CID: drop.DropCID, Qty: drop.DropQuantity}
	op := newOpID(opPickup)
	w := &linkedOp{kind: opPickup, gain: Goods{Stacks: []Stack{stack}}}
	if s.linkOp(player, op, Transfer{}, persist.Effect{Claim: drop.WorldDrop}, w) != "" {
		return
	}
	code, expires, collector, profile := ms.Code(), drop.ExpiresAt, player.ID, player.PlayerID
	drop.Lock, drop.ExpiresAt = world.DropLock{PlayerID: string(profile), Op: op}, time.Time{}
	w.settled = func(outcome persist.Outcome, refusal string, _ *world.Session, _ time.Time) {
		s.recordEconomy("pickup", refusal)
		if outcome == persist.Applied {
			logx.Infof("[ECONOMY] pickup player=%s item=%s cid=%s qty=%d token=%s economy=%s",
				profile, stack.ItemID, stack.CID, stack.Qty, token, s.EconomyKey())
		}
		ms, drop := s.worldDropToken(code, token, op)
		switch {
		case drop == nil:
		case outcome == persist.Applied:
			tf := ms.Tf(token)
			s.BroadcastDropCollect(ms, token, collector, tf.Pos.X+tf.Size.X*0.5, tf.Pos.Y+tf.Size.Y*0.5, true)
			s.Despawn(ms, token)
		case refusal == persist.RefusalDropGone:
			logx.Warnf("[LOOT] world drop token %s removed: its record is gone", token)
			s.Despawn(ms, token)
		default:
			drop.Lock, drop.ExpiresAt = world.DropLock{}, expires
		}
	}
}

// RestoreWorldDrops puts back the public world drops of this world, each open
// until its expiry; its owner takes it back only after it stepped off it once.
// A drop of a map this world lacks stays in the engine until it expires. It
// returns how many it put back.
func (s *Content) RestoreWorldDrops(drops []persist.WorldDrop, now time.Time) int {
	restored := 0
	for _, d := range drops {
		ms := s.Maps[d.MapCode]
		if ms == nil || !d.ExpiresAt.After(now) {
			continue
		}
		center := world.Vec2{X: d.X + worldTokenDims.X*0.5, Y: d.Y + worldTokenDims.Y*0.5}
		s.spawnLootToken(ms, s.dropPosition(ms, center), worldTokenDims, &world.Brain{
			DropItemID:     d.ItemID,
			DropCID:        d.CID,
			DropQuantity:   d.Quantity,
			LootOrigin:     world.LootOriginPlayer,
			LootCollection: world.LootCollectPlayers,
			LootOutcome:    world.LootOutcomeItem,
			CollectableAt:  now,
			ExpiresAt:      d.ExpiresAt,
			WorldDrop:      d.ID,
			DropOwner:      world.PlayerID(d.Owner),
			OwnerHold:      true,
		}, 0)
		restored++
	}
	return restored
}
