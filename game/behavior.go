package game

// Canonical bot behaviors. Mirrors the bot behaviors of SharedDefaultsCyberia.ENTITY_BEHAVIORS.
// A bot's behavior is content-authored on its matched entity-type default
// (resolved by active itemId), or derived when unset. `skill` / `coin` are
// assigned by the runtime (skill engine / economy), never authored.
// A foreground behavior is presentation only: the server sends it and the client runs it.
const (
	BehaviorPassive        = "passive"
	BehaviorHostile        = "hostile"
	BehaviorProvider       = "provider"
	BehaviorProviderStatic = "provider-static"
	BehaviorSkill          = "skill"
	BehaviorCoin           = "coin"
	// BehaviorDrop marks a transient collectible token scattered on the grid by
	// a bot death (see loot.go). Drops never move, never aggro, deal no damage,
	// and are removed on collection or lifetime expiry. Runtime-assigned only.
	BehaviorDrop = "drop"
)

// behaviorIsInert reports whether entities with behavior b run no AI: they hold
// position until removed. Covers scattered loot drops.
func behaviorIsInert(b string) bool {
	return b == BehaviorDrop
}

// behaviorIsProvider reports whether b is a mission/action giver — immortal and
// near-stationary, whether it takes occasional short steps or is fully static.
func behaviorIsProvider(b string) bool {
	return b == BehaviorProvider || b == BehaviorProviderStatic
}

// behaviorIsImmortal reports whether entities with behavior b ignore combat
// damage entirely.
func behaviorIsImmortal(b string) bool {
	return behaviorIsProvider(b)
}

// behaviorIsStationary reports whether entities with behavior b never move.
func behaviorIsStationary(b string) bool {
	return b == BehaviorProviderStatic
}

// deriveBehaviorFromLayers infers a behavior from object layers when no
// canonical behavior is bound: any active weapon-type layer → hostile, else
// passive. Runtime-spawned entities (projectiles, doppelgangers) set behavior
// directly and never go through here.
func (s *GameServer) deriveBehaviorFromLayers(layers []ObjectLayerState) string {
	for _, ol := range layers {
		if data, ok := s.GetObjectLayerData(ol.ItemID); ok {
			if data.Data.Item.Type == "weapon" {
				return BehaviorHostile
			}
		}
	}
	return BehaviorPassive
}

// boundBehavior is the behavior bound to the entity-type default the layers
// match (by active itemId — the same liveItemIds match used for the
// live/dead/drop sets), or "" when that default binds none.
func (s *GameServer) boundBehavior(entityType string, layers []ObjectLayerState) string {
	build, _ := s.resolveEntityDefaultBuild(entityType, activeObjectLayerItemIDs(layers))
	return build.Behavior
}

// resolveEntityBehavior picks a bot's behavior: the bound behavior when set,
// otherwise derived from the layers. This is the single entry point for
// assigning a spawned bot's behavior.
func (s *GameServer) resolveEntityBehavior(entityType string, layers []ObjectLayerState) string {
	if behavior := s.boundBehavior(entityType, layers); behavior != "" {
		return behavior
	}
	return s.deriveBehaviorFromLayers(layers)
}
