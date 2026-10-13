package game

import (
	"sync"
	"time"

	pb "cyberia-server/gen/proto"
)

// Content is the rules and the content of the game: the instance config,
// the engine content caches and the content rows of each entity.
type Content struct {
	*World
	progressionRules ProgressionRules
	questsByCell     map[cellKey][]string // quest codes bound to each action cell

	// Caches (protected by olMu)
	olMu                 sync.RWMutex
	objectLayerDataCache map[string]*ObjectLayer

	// dataServerURL is the Data Server origin for server-to-server
	// content-authority calls (quest persistence). Never forwarded to clients:
	// each client learns its own Data Server URL from its command line.
	dataServerURL string
	// dataServerAPIKey is CYBERIA_SERVER_API_KEY, sent with every server-to-server
	// write. An empty key disables them.
	dataServerAPIKey string
	// persistQueue holds engine POSTs for persistLoop, so no HTTP call runs
	// under s.mu.
	persistQueue chan persistJob

	entityBaseSpeed float64
	// playerBaseSpeed applies to the player entity only. 0 means unset, and
	// CalculatePlayerMovementSpeed then falls back to entityBaseSpeed.
	playerBaseSpeed             float64
	entityBaseMaxLife           float64
	entityBaseActionCooldown    time.Duration
	entityBaseMinActionCooldown time.Duration

	// Player defaults
	defaultPlayerWidth     float64
	defaultPlayerHeight    float64
	playerBaseLifeRegenMin float64
	playerBaseLifeRegenMax float64
	maxActiveLayers        int
	initialLifeFraction    float64

	// Combat / death
	ghostItemID       string
	collisionLifeLoss float64
	// Every configured dead-item id — excluded from the player inventory wire
	// and from item activation. Populated exclusively from gRPC
	// entity_defaults.dead_item_ids (no hardcoded fallback).
	deadItemIDs map[string]bool

	// Economy — Fountain & Sink model
	// Fountains (coin injection)
	coinItemID       string
	botSpawnCoins    int // coins every bot carries on spawn (infinite mint)
	playerSpawnCoins int // coins given to player on first connect (guest wallet)
	// Kill Transfer (redistribution)
	coinKillPercentVsBot    float64 // fraction of victim coins transferred: PvE
	coinKillPercentVsPlayer float64 // fraction of victim coins transferred: PvP (gentler)
	coinKillMinAmount       int     // floor — every kill always pays this
	// Sinks (coin removal; all default to 0 = disabled in alpha)
	respawnCostPercent float64 // fraction burned on player death
	portalFee          int     // flat fee per portal use
	craftingFeePercent float64 // fraction of item value burned on crafting

	// Regen
	lifeRegenChance float64
	maxChance       float64

	// Floor defaults
	defaultFloorItemID string

	// Per-entity-type visual defaults.
	// entityDefaults keeps the last build per type for unique lookups.
	// entityDefaultBuilds preserves the full ordered config, including duplicates.
	entityDefaults      map[string]EntityTypeDefaultConfig
	entityDefaultBuilds []EntityTypeDefaultConfig

	// Skill map (runtime): trigger item ID → []SkillDefinition
	skillConfig map[string][]SkillDefinition

	// Action / quest content fetched from engine-cyberia REST at instance
	// init (actions and quests are not part of the gRPC world payload).
	//   actionCache  — entityId → bound CyberiaAction (resolved by the server
	//                  on every talk_done; the client never declares it).
	//   questDefs    — questCode → CyberiaQuest definition (steps/rewards).
	// Both are rebuilt on every world (re)build; access is under s.mu.
	actionCache map[EntityID]*CyberiaAction
	// storage — per (player, action code) vault contents. Session state
	// only; the slot shape is the future persistence document.
	storage   map[storageKey][]StorageSlot
	questDefs map[string]*CyberiaQuest
	// progression is the level and XP of each mortal entity. despawn deletes
	// the row.
	progression map[EntityID]*EntityProgression
	// quests is the quest progress of each player, keyed by quest code.
	// Authoritative for the session; best-effort mirrored to engine-cyberia
	// quest-progress REST for persistence.
	quests map[EntityID]map[string]*QuestProgress
	// crafts is the assembly running for each player. A craft consumes its
	// ingredients up front and pays out only when the timer elapses, so the
	// row is also the refund receipt a cancel needs. One at a time:
	// assembling is a modal operation.
	crafts map[EntityID]*ActiveCraft

	// Equipment rules — governs activation constraints.
	equipmentRules EquipmentRulesConfig
}

// applyConfig applies the rules and the content of the instance config.
// Caller holds s.mu.
func (s *Content) applyConfig(cfg *pb.InstanceConfig, rules ProgressionRules) {
	s.progressionRules = rules

	// Entity base stats
	s.entityBaseSpeed = cfg.GetEntityBaseSpeed()
	s.playerBaseSpeed = cfg.GetPlayerBaseSpeed()
	s.entityBaseMaxLife = cfg.GetEntityBaseMaxLife()
	s.entityBaseActionCooldown = time.Duration(cfg.GetEntityBaseActionCooldownMs()) * time.Millisecond
	s.entityBaseMinActionCooldown = time.Duration(cfg.GetEntityBaseMinActionCooldownMs()) * time.Millisecond

	// Player defaults
	s.defaultPlayerWidth = cfg.GetDefaultPlayerWidth()
	s.defaultPlayerHeight = cfg.GetDefaultPlayerHeight()
	s.playerBaseLifeRegenMin = cfg.GetPlayerBaseLifeRegenMin()
	s.playerBaseLifeRegenMax = cfg.GetPlayerBaseLifeRegenMax()
	s.maxActiveLayers = int(cfg.GetMaxActiveLayers())
	s.initialLifeFraction = cfg.GetInitialLifeFraction()

	// Combat / death
	s.collisionLifeLoss = cfg.GetCollisionLifeLoss()

	// Economy — Fountain & Sink model (nested EconomyRules proto message).
	er := cfg.GetEconomyRules()
	if er == nil {
		er = &pb.EconomyRules{}
	}
	s.botSpawnCoins = int(er.GetBotSpawnCoins())
	s.playerSpawnCoins = int(er.GetPlayerSpawnCoins())
	s.coinKillPercentVsBot = er.GetCoinKillPercentVsBot()
	s.coinKillPercentVsPlayer = er.GetCoinKillPercentVsPlayer()
	if s.coinKillPercentVsPlayer == 0 {
		// Default PvP to half the PvE rate if not explicitly set.
		s.coinKillPercentVsPlayer = s.coinKillPercentVsBot * 0.5
	}
	s.coinKillMinAmount = int(er.GetCoinKillMinAmount())
	s.respawnCostPercent = er.GetRespawnCostPercent()
	s.portalFee = int(er.GetPortalFee())
	s.craftingFeePercent = er.GetCraftingFeePercent()
	s.lifeRegenChance = cfg.GetLifeRegenChance()
	s.maxChance = cfg.GetMaxChance()

	// Equipment rules — governs which item types can be active.
	eqr := cfg.GetEquipmentRules()
	if eqr != nil {
		s.equipmentRules.ActiveItemTypes = make(map[string]bool, len(eqr.GetActiveItemTypes()))
		for _, t := range eqr.GetActiveItemTypes() {
			s.equipmentRules.ActiveItemTypes[t] = true
		}
		s.equipmentRules.OnePerType = eqr.GetOnePerType()
		s.equipmentRules.RequireSkin = eqr.GetRequireSkin()
	} else {
		// Sensible defaults when no equipment rules are provided.
		s.equipmentRules.ActiveItemTypes = map[string]bool{
			"skin": true, "breastplate": true, "weapon": true, "skill": true,
		}
		s.equipmentRules.OnePerType = true
		s.equipmentRules.RequireSkin = true
	}

	// Per-entity-type visual defaults — keep both the ordered build list and
	// the last-seen per-type lookup for unique entity types.
	s.entityDefaults = make(map[string]EntityTypeDefaultConfig, len(cfg.GetEntityDefaults()))
	s.entityDefaultBuilds = make([]EntityTypeDefaultConfig, 0, len(cfg.GetEntityDefaults()))
	for _, etd := range cfg.GetEntityDefaults() {
		var dols []ObjectLayerState
		dropChances := make(map[string]float64)
		for _, dol := range etd.GetDefaultObjectLayers() {
			dols = append(dols, ObjectLayerState{
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
		s.entityDefaultBuilds = append(s.entityDefaultBuilds, defaultBuild)
		s.entityDefaults[defaultBuild.EntityType] = defaultBuild
	}
	if d, ok := s.entityDefaults["player"]; ok && len(d.DeadItemIDs) > 0 {
		s.ghostItemID = d.DeadItemIDs[0]
	} else {
		s.ghostItemID = ""
	}
	s.deadItemIDs = make(map[string]bool)
	for _, build := range s.entityDefaultBuilds {
		for _, id := range build.DeadItemIDs {
			if id != "" {
				s.deadItemIDs[id] = true
			}
		}
	}
	if d, ok := s.entityDefaults["coin"]; ok && len(d.LiveItemIDs) > 0 {
		s.coinItemID = d.LiveItemIDs[0]
	}
	if d, ok := s.entityDefaults["floor"]; ok && len(d.LiveItemIDs) > 0 {
		s.defaultFloorItemID = d.LiveItemIDs[0]
	}

	// Status-icon visuals (iconId + borderColor) are not on the server.
	// The simulation writes the numeric status_icon u8 into the AOI encoder;
	// visual resolution happens on the client via domain/presentation_defaults
	// and the optional /api/v1/cyberia-client-hints override.

	// Skill map
	s.skillConfig = make(map[string][]SkillDefinition, len(cfg.GetSkillConfig()))
	for _, sc := range cfg.GetSkillConfig() {
		triggerID := sc.GetTriggerItemId()
		for _, sk := range sc.GetSkills() {
			s.skillConfig[triggerID] = append(s.skillConfig[triggerID], SkillDefinition{
				LogicEventID:         sk.GetLogicEventId(),
				Name:                 sk.GetName(),
				Description:          sk.GetDescription(),
				SummonedEntityItemID: sk.GetSummonedEntityItemId(),
			})
		}
	}
}
