package content

import (
	"sync"
	"time"

	"cyberia-server/game/persist"
	"cyberia-server/game/world"
	pb "cyberia-server/gen/proto"
)

// Content is the rules and the content of the game: the instance config,
// the engine content caches and the content rows of each entity.
type Content struct {
	*world.World
	progressionRules ProgressionRules
	questsByCell     map[world.CellKey][]string // quest codes bound to each action cell

	// Caches (protected by olMu)
	OlMu                 sync.RWMutex
	ObjectLayerDataCache map[string]*ObjectLayer

	// Durability commits the durable state of account players; nil in
	// anonymous mode.
	Durability Durability

	EntityBaseSpeed float64
	// playerBaseSpeed applies to the player entity only. 0 means unset, and
	// CalculatePlayerMovementSpeed then falls back to entityBaseSpeed.
	playerBaseSpeed             float64
	EntityBaseMaxLife           float64
	EntityBaseActionCooldown    time.Duration
	EntityBaseMinActionCooldown time.Duration

	// Player defaults
	DefaultPlayerWidth     float64
	DefaultPlayerHeight    float64
	PlayerBaseLifeRegenMin float64
	PlayerBaseLifeRegenMax float64
	MaxActiveLayers        int
	InitialLifeFraction    float64

	// Combat / death
	GhostItemID       string
	CollisionLifeLoss float64
	// Every configured dead-item id — excluded from the player inventory wire
	// and from item activation. Populated exclusively from gRPC
	// entity_defaults.dead_item_ids (no hardcoded fallback).
	DeadItemIDs map[string]bool

	CoinItemID string
	economy    EconomyRules

	// Regen
	LifeRegenChance float64
	MaxChance       float64

	// Floor defaults
	DefaultFloorItemID string

	// Per-entity-type visual defaults.
	// entityDefaults keeps the last build per type for unique lookups.
	// entityDefaultBuilds preserves the full ordered config, including duplicates.
	EntityDefaults      map[string]EntityTypeDefaultConfig
	EntityDefaultBuilds []EntityTypeDefaultConfig

	// Skill map (runtime): trigger item ID → []SkillDefinition
	SkillConfig map[string][]SkillDefinition

	// Action / quest content fetched from engine-cyberia REST at instance
	// init (actions and quests are not part of the gRPC world payload).
	//   actionCache  — entityId → bound CyberiaAction (resolved by the server
	//                  on every talk_done; the client never declares it).
	//   questDefs    — questCode → CyberiaQuest definition (steps/rewards).
	// Both are rebuilt on every world (re)build; access is under s.mu.
	actionCache map[world.EntityID]*CyberiaAction
	// Vaults holds the NPC vaults of each player by action code, and
	// PersonalStorage the personal storage of each player (storage.go).
	Vaults          map[world.PlayerID]map[string][]Stack
	PersonalStorage map[world.PlayerID][]Stack
	// Held is the owned stacks of each player the world cannot load, and
	// dormantQuests the saved progress of quests it no longer defines
	// (durable.go). Both return unchanged in every commit.
	Held          map[world.PlayerID][]persist.Held
	dormantQuests map[world.PlayerID][]persist.Quest
	questDefs     map[string]*CyberiaQuest
	// Progression is the level and XP of each mortal entity. despawn deletes
	// the row.
	Progression map[world.EntityID]*EntityProgression
	// quests is the quest progress of each player, keyed by quest code. An
	// account player commits it with its durable state (durable.go).
	quests map[world.EntityID]map[string]*QuestProgress
	// crafts is the assembly running for each player. A craft consumes its
	// ingredients up front and pays out only when the timer elapses, so the
	// row is also the refund receipt a cancel needs. One at a time:
	// assembling is a modal operation.
	crafts map[world.EntityID]*ActiveCraft
	// lootRespawns are the consumed world tokens waiting to come back, by map
	// code (loot.go). A world build starts it empty.
	lootRespawns map[string][]LootRespawn

	// Equipment rules — governs activation constraints.
	EquipmentRules EquipmentRulesConfig
}

// NewContent returns the content over w with empty caches.
func NewContent(w *world.World) *Content {
	return &Content{
		World:                w,
		ObjectLayerDataCache: make(map[string]*ObjectLayer),
		SkillConfig:          make(map[string][]SkillDefinition),
	}
}

// ApplyConfig applies the rules and the content of the instance config.
// Caller holds s.mu.
func (s *Content) ApplyConfig(cfg *pb.InstanceConfig, rules ProgressionRules) {
	s.progressionRules = rules

	// Entity base stats
	s.EntityBaseSpeed = cfg.GetEntityBaseSpeed()
	s.playerBaseSpeed = cfg.GetPlayerBaseSpeed()
	s.EntityBaseMaxLife = cfg.GetEntityBaseMaxLife()
	s.EntityBaseActionCooldown = time.Duration(cfg.GetEntityBaseActionCooldownMs()) * time.Millisecond
	s.EntityBaseMinActionCooldown = time.Duration(cfg.GetEntityBaseMinActionCooldownMs()) * time.Millisecond

	// Player defaults
	s.DefaultPlayerWidth = cfg.GetDefaultPlayerWidth()
	s.DefaultPlayerHeight = cfg.GetDefaultPlayerHeight()
	s.PlayerBaseLifeRegenMin = cfg.GetPlayerBaseLifeRegenMin()
	s.PlayerBaseLifeRegenMax = cfg.GetPlayerBaseLifeRegenMax()
	s.MaxActiveLayers = int(cfg.GetMaxActiveLayers())
	s.InitialLifeFraction = cfg.GetInitialLifeFraction()

	// Combat / death
	s.CollisionLifeLoss = cfg.GetCollisionLifeLoss()

	s.economy = EconomyRulesFromProto(cfg.GetEconomyRules())
	s.LifeRegenChance = cfg.GetLifeRegenChance()
	s.MaxChance = cfg.GetMaxChance()

	// Equipment rules — governs which item types can be active.
	eqr := cfg.GetEquipmentRules()
	if eqr != nil {
		s.EquipmentRules.ActiveItemTypes = make(map[string]bool, len(eqr.GetActiveItemTypes()))
		for _, t := range eqr.GetActiveItemTypes() {
			s.EquipmentRules.ActiveItemTypes[t] = true
		}
		s.EquipmentRules.OnePerType = eqr.GetOnePerType()
		s.EquipmentRules.RequireSkin = eqr.GetRequireSkin()
	} else {
		// Sensible defaults when no equipment rules are provided.
		s.EquipmentRules.ActiveItemTypes = map[string]bool{
			"skin": true, "breastplate": true, "weapon": true, "skill": true,
		}
		s.EquipmentRules.OnePerType = true
		s.EquipmentRules.RequireSkin = true
	}

	// Per-entity-type visual defaults — keep both the ordered build list and
	// the last-seen per-type lookup for unique entity types.
	s.EntityDefaults = make(map[string]EntityTypeDefaultConfig, len(cfg.GetEntityDefaults()))
	s.EntityDefaultBuilds = make([]EntityTypeDefaultConfig, 0, len(cfg.GetEntityDefaults()))
	for _, etd := range cfg.GetEntityDefaults() {
		var dols []world.ObjectLayerState
		dropChances := make(map[string]float64)
		for _, dol := range etd.GetDefaultObjectLayers() {
			dols = append(dols, world.ObjectLayerState{
				ItemID:   dol.GetItemId(),
				Active:   dol.GetActive(),
				Quantity: int(dol.GetQuantity()),
			})
			// Only a row that states a chance is recorded. The field is optional precisely so a
			// payload from before it existed reads as absent rather than as a deliberate 0, and
			// dropChance answers 1 for anything unrecorded.
			if dol.DropChance != nil {
				dropChances[dol.GetItemId()] = dol.GetDropChance()
			}
		}
		defaultBuild := EntityTypeDefaultConfig{
			EntityType:          etd.GetEntityType(),
			LiveItemIDs:         etd.GetLiveItemIds(),
			DeadItemIDs:         etd.GetDeadItemIds(),
			DropItemIDs:         etd.GetDropItemIds(),
			DefaultObjectLayers: dols,
			DropChances:         dropChances,
			Behavior:            etd.GetBehavior(),
		}
		s.EntityDefaultBuilds = append(s.EntityDefaultBuilds, defaultBuild)
		s.EntityDefaults[defaultBuild.EntityType] = defaultBuild
	}
	if d, ok := s.EntityDefaults["player"]; ok && len(d.DeadItemIDs) > 0 {
		s.GhostItemID = d.DeadItemIDs[0]
	} else {
		s.GhostItemID = ""
	}
	s.DeadItemIDs = make(map[string]bool)
	for _, build := range s.EntityDefaultBuilds {
		for _, id := range build.DeadItemIDs {
			if id != "" {
				s.DeadItemIDs[id] = true
			}
		}
	}
	if d, ok := s.EntityDefaults["coin"]; ok && len(d.LiveItemIDs) > 0 {
		s.CoinItemID = d.LiveItemIDs[0]
	}
	if d, ok := s.EntityDefaults["floor"]; ok && len(d.LiveItemIDs) > 0 {
		s.DefaultFloorItemID = d.LiveItemIDs[0]
	}

	// Status-icon visuals (iconId + borderColor) are not on the server.
	// The simulation writes the numeric status_icon u8 into the AOI encoder;
	// visual resolution happens on the client via domain/presentation_defaults
	// and the optional /api/v1/cyberia-client-hints override.

	// Skill map
	s.SkillConfig = make(map[string][]SkillDefinition, len(cfg.GetSkillConfig()))
	for _, sc := range cfg.GetSkillConfig() {
		triggerID := sc.GetTriggerItemId()
		for _, sk := range sc.GetSkills() {
			s.SkillConfig[triggerID] = append(s.SkillConfig[triggerID], SkillDefinition{
				LogicEventID:         sk.GetLogicEventId(),
				Name:                 sk.GetName(),
				Description:          sk.GetDescription(),
				SummonedEntityItemID: sk.GetSummonedEntityItemId(),
			})
		}
	}
}
