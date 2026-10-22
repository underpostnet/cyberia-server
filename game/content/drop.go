// Package content — drop.go
//
// A player drops units of one stack on the ground for anyone to take. The
// stack leaves its bag or personal storage slot, and one public token keeps its
// item, its definition and its units. Only an item whose policy allows a world
// drop leaves, and only a stack of the definition the world binds.
//
// The token of an account player is a world drop (world_drop.go): it stays
// locked until the commit that removed the stack and created its record
// applied, so no state holds the stack twice. A guest's token is public at
// once. A public token expires lootLifetime after it turns public. The first
// player that touches a public token takes it; a bot never does. Its owner
// takes it back only after it stepped off the token once.
//
// Cross-process contract:
//
//	item_drop (client→server)     — box, slot, quantity and the target cell x, y (negative: at the feet)
//	item_drop_ack (server→client) — status pending | public, or a refusal reason
//
// Caller MUST hold s.mu.
package content

import (
	"math"
	"time"

	"cyberia-server/game/world"
	"cyberia-server/logx"
)

// Statuses of a dropped stack on item_drop_ack.
const (
	DropPending = "pending" // the token waits for the commit that removed the stack
	DropPublic  = "public"  // anyone can take the token
)

// HandleItemDrop drops units of the stack in one bag or storage slot on a
// cell near the player.
func (s *Content) HandleItemDrop(player *world.Session, cmd *world.InputCommand) {
	place := Place{Box: cmd.Move.From, Slot: cmd.Move.FromSlot}
	stack, _, held := s.PlacedStack(player, place)
	qty := min(cmd.Move.Qty, stack.Qty)
	reason := s.dropRefusal(player, stack, held, qty)
	ms := s.Maps[player.MapCode]
	tf := ms.Tf(player.ID)
	center := world.Vec2{X: tf.Pos.X + tf.Size.X*0.5, Y: tf.Pos.Y + tf.Size.Y*0.5}
	target := center
	if reason == "" && cmd.TargetX >= 0 && cmd.TargetY >= 0 {
		target = world.Vec2{X: cmd.TargetX, Y: cmd.TargetY}
		if math.Hypot(target.X-center.X, target.Y-center.Y) > ItemDropReach {
			reason = RefuseOutOfRange
		}
	}
	dropped := Stack{ItemID: stack.ItemID, CID: stack.CID, Qty: qty}
	pos := s.dropPosition(ms, target)
	var dropID world.EntityID
	status := DropPublic
	switch {
	case reason != "":
	case player.AccountID != "":
		dropID, reason = s.linkDrop(player, place, dropped, center, pos)
		status = DropPending
	default:
		if reason = s.ApplyTransfer(player, Transfer{Takes: []Debit{{Place: place, Qty: qty}}}); reason == "" {
			dropID = s.placeDrop(ms, center, pos, dropped, player.PlayerID, world.DropLock{})
		}
	}
	ack := map[string]interface{}{"itemId": stack.ItemID, "quantity": qty, "box": cmd.Move.From.String(), "slot": place.Slot}
	if reason != "" {
		s.recordEconomy("drop", reason)
		world.SendAck(player, "item_drop_ack", reason, ack)
		return
	}
	ack["dropId"], ack["status"] = dropID, status
	s.recordEconomy("drop", "")
	world.SendAck(player, "item_drop_ack", "", ack)
	logx.Infof("[ECONOMY] drop player=%s item=%s cid=%s qty=%d token=%s status=%s economy=%s",
		player.PlayerID, stack.ItemID, stack.CID, qty, dropID, ack["status"], s.EconomyKey())
}

// dropRefusal is why the stack cannot drop, or "".
func (s *Content) dropRefusal(player *world.Session, stack Stack, held bool, qty int) string {
	switch {
	case s.Maps[player.MapCode].Vt(player.ID).Ghost():
		return RefuseDead
	case s.Trading(player):
		return RefuseTrading
	case !held || qty < 1 || !s.takesSlot(stack.ItemID) || s.DeadItemIDs[stack.ItemID]:
		return RefuseNoStack
	}
	policy, reason := s.ItemPolicyOf(stack.ItemID, stack.CID)
	if reason == "" && !policy.WorldDrop {
		reason = RefusePolicy
	}
	return reason
}

// dropPosition is where the token of a stack dropped at target lies: on the
// walkable cell nearest target.
func (s *Content) dropPosition(ms *world.MapState, target world.Vec2) world.Vec2 {
	cell := s.clampDropCell(ms, world.Cell{X: int(math.Round(target.X)), Y: int(math.Round(target.Y))}, worldTokenDims)
	return world.Vec2{X: float64(cell.X) - worldTokenDims.X*0.5, Y: float64(cell.Y) - worldTokenDims.Y*0.5}
}

// placeDrop puts the token of a dropped stack at pos, flung from origin, and
// returns its id. The token of a world drop stays locked until its op settles.
func (s *Content) placeDrop(ms *world.MapState, origin, pos world.Vec2, stack Stack, owner world.PlayerID, lock world.DropLock) world.EntityID {
	now := time.Now()
	brain := &world.Brain{
		DropItemID:     stack.ItemID,
		DropCID:        stack.CID,
		DropQuantity:   stack.Qty,
		DropOwner:      owner,
		OwnerHold:      true,
		LootOrigin:     world.LootOriginPlayer,
		LootCollection: world.LootCollectPlayers,
		LootOutcome:    world.LootOutcomeItem,
		CollectableAt:  now.Add(lootLandingWindow),
		Lock:           lock,
		WorldDrop:      lock.Op,
	}
	if lock.Op == "" {
		brain.ExpiresAt = now.Add(lootLifetime)
	}
	dropID := s.spawnLootToken(ms, pos, worldTokenDims, brain, 0)
	s.broadcastDropSpawn(ms, dropID, origin, world.Vec2{X: pos.X + worldTokenDims.X*0.5, Y: pos.Y + worldTokenDims.Y*0.5})
	return dropID
}
