# Graph Report - cyberia-server  (2026-10-15)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 965 nodes · 3346 edges · 44 communities (35 shown, 9 thin omitted)
- Extraction: 98% EXTRACTED · 2% INFERRED · 0% AMBIGUOUS · INFERRED: 53 edges (avg confidence: 0.85)
- Token cost: 29,463 input · 623 output

## Graph Freshness
- Built from commit: `296317b7`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Quest and Loot Dispatch
- Entity Progression System
- ECS World Components
- Connection Guard and Limits
- Config Loading
- Object Layer Inventory
- Go Package Runtime Bootstrap
- Bot Stats and Interaction
- Instance Simulation Binding
- Player Storage and Item Moves
- Pathfinding
- Quest Progression Tracking
- Loot Drop System
- REST Instance Client
- Bot Loot and Respawn Behavior
- Entity Snapshot Building
- Entity Metrics Collection
- API Docs Handlers
- Chat and Input Session
- Runtime Metrics Census
- API Router Setup
- Inventory Exchange and Vaults
- WebSocket Client Pump
- Dispatcher Registry Startup
- Object Layer Data Source
- Quest Accept and Providers
- Movement and Direction
- Coin Economy and Fountain
- Hot Reload Service
- Simulation Phase Loop
- Shop and Dialogue Actions
- HTTP Router Middleware
- Craft Recipe Handling
- HTTP Problem Responses
- gRPC Data Client
- Logging Setup
- Portal Teleport and Drops
- Service Tick Loop
- Consumable Item Use
- Skill Dispatch
- Skill Registration
- Service Routes
- Economy Rules Validation
- Stats Validation

## God Nodes (most connected - your core abstractions)
1. `Session` - 116 edges
2. `MapState` - 115 edges
3. `EntityID` - 86 edges
4. `MapState` - 36 edges
5. `Dispatcher` - 35 edges
6. `EntityID` - 34 edges
7. `ObjectLayerState` - 33 edges
8. `Content` - 31 edges
9. `Debugf()` - 30 edges
10. `Vec2` - 28 edges

## Surprising Connections (you probably didn't know these)
- `validateWorldProgression()` --calls--> `ValidateWorldProgression()`  [EXTRACTED]
  engine_client/world_builder.go → game/content/progression_runtime.go
- `enginePost()` --calls--> `Errorf()`  [EXTRACTED]
  game/content/quest.go → logx/logx.go
- `dispatch()` --calls--> `Warnf()`  [EXTRACTED]
  engine_client/dispatcher.go → logx/logx.go
- `MetricsHandler` --references--> `GameServer`  [EXTRACTED]
  api/metrics.go → game/net/server.go
- `MetricsResponse` --references--> `MapBreakdown`  [EXTRACTED]
  api/metrics.go → game/net/metrics_counters.go

## Import Cycles
- None detected.

## Communities (44 total, 9 thin omitted)

### Community 0 - "Quest and Loot Dispatch"
Cohesion: 0.18
Nodes (13): CapacityRule, CyberiaQuest, Item, Ledger, QuestObjective, QuestReward, QuestStep, Render (+5 more)

### Community 1 - "Entity Progression System"
Cohesion: 0.05
Nodes (27): ActiveCraft, EntityProgression, EquipmentRulesConfig, persistJob, progressionEvent, SkillDefinition, xpRepeat, Content (+19 more)

### Community 2 - "ECS World Components"
Cohesion: 0.08
Nodes (16): canConsume(), BotStatusIcon(), EntityID, MapState, Transform, World, Agent, Appearance (+8 more)

### Community 3 - "Connection Guard and Limits"
Cohesion: 0.06
Nodes (26): NewMetricsHandler(), NewContent(), clientIP(), DefaultConnectionLimits(), newConnectionGuard(), newInputLimiter(), newTokenBucket(), ResolveConnectionLimits() (+18 more)

### Community 4 - "Config Loading"
Cohesion: 0.09
Nodes (20): Config, getEnv(), getEnvIntPtr(), Load(), loadWSLimits(), normalizeBasePath(), parseBool(), parseCSV() (+12 more)

### Community 5 - "Object Layer Inventory"
Cohesion: 0.11
Nodes (15): firstStack(), Content, InventoryStack, Content, Content, ActiveObjectLayerItemIDs(), buildLayer(), dropChance() (+7 more)

### Community 6 - "Go Package Runtime Bootstrap"
Cohesion: 0.09
Nodes (7): Unpack(), jsonCodec, Request, StaticFileServer(), staticPathWithinBase(), eventsPayload, Message

### Community 7 - "Bot Stats and Interaction"
Cohesion: 0.11
Nodes (10): Content, storageCapacity(), findPortalByCell(), Sim, Sim, EntityID, portalEntry, DropCollect (+2 more)

### Community 8 - "Instance Simulation Binding"
Cohesion: 0.18
Nodes (8): protoToAction(), PlacedRespawnDuration(), protoToQuest(), entSize(), entTransform(), Sim, playerSpawnFromInstance(), printInstanceGraph()

### Community 9 - "Player Storage and Item Moves"
Cohesion: 0.13
Nodes (15): moveSet, Content, slotStack(), parseInput(), parseItemMove(), ClampQuantity(), ValidIdentifier(), ValidTapTarget() (+7 more)

### Community 10 - "Pathfinding"
Cohesion: 0.12
Nodes (9): NewMapState(), heuristic(), NewPathfinder(), reconstructPath(), Cell, Map(), Node, Pathfinder (+1 more)

### Community 11 - "Quest Progression Tracking"
Cohesion: 0.24
Nodes (6): QuestProgress, firstIncompleteStepIndex(), Content, QuestSnapshotEntry, questComplete(), stepComplete()

### Community 12 - "Loot Drop System"
Cohesion: 0.16
Nodes (9): LootRespawn, Content, LootAdmits(), LootConsumes(), Brain, Vec2, LootCollection, LootOrigin (+1 more)

### Community 13 - "REST Instance Client"
Cohesion: 0.15
Nodes (4): dispatch(), ManifestEntry, RestClient, restEnvelope

### Community 14 - "Bot Loot and Respawn Behavior"
Cohesion: 0.15
Nodes (12): BehaviorCollectsLoot(), BehaviorIsImmortal(), BehaviorIsInert(), BehaviorIsProvider(), ContributorSet(), RecordDamage(), RespawnWait(), applyDeadItems() (+4 more)

### Community 15 - "Entity Snapshot Building"
Cohesion: 0.17
Nodes (16): PlayerStatusIcon(), ResourceStatusIcon(), baseEntity(), GameServer, msUntil(), wholeStats(), DrawOrder, EntityFlags (+8 more)

### Community 16 - "Entity Metrics Collection"
Cohesion: 0.21
Nodes (11): EntityMetrics, EntityTypeMetrics, HealthStatus, pct(), MetricsHandler, MetricsResponse, RuntimeMetrics, WebSocketServerMetrics (+3 more)

### Community 17 - "API Docs Handlers"
Cohesion: 0.23
Nodes (5): DocsHandler, writeJSON(), livenessHandler(), Write(), methodNotAllowedHandler()

### Community 18 - "Chat and Input Session"
Cohesion: 0.18
Nodes (12): ActivateOrAppendLayer(), Sim, wearOneStack(), deliver(), EnqueueInputs(), InputCommand, Session, SendAck() (+4 more)

### Community 19 - "Runtime Metrics Census"
Cohesion: 0.10
Nodes (6): GameServer, MapBreakdown, RuntimeMetricsSnapshot, sortBreakdowns(), SceneryClass(), EntityTypeCensus

### Community 20 - "API Router Setup"
Cohesion: 0.16
Nodes (9): NewDocsHandler(), toJSONCompatible(), Mount(), NewAPIRouter(), readinessHandlerFor(), RouterOptions, DefaultCORSOrigins(), presentedKey() (+1 more)

### Community 21 - "Inventory Exchange and Vaults"
Cohesion: 0.17
Nodes (7): protoToCraftItems(), Content, Stack, LayerQuantity(), RemoveLayerItem(), SnapshotVault, Sim

### Community 22 - "WebSocket Client Pump"
Cohesion: 0.22
Nodes (8): Client, GameServer, Pack(), FreezePlayer(), IsHeldSkill(), Debugf(), Errorf(), SkillMapEntry

### Community 23 - "Dispatcher Registry Startup"
Cohesion: 0.15
Nodes (15): main(), runUnderpostStatus(), underpostStateSet(), NewDispatcher(), NewGrpcClient(), Register(), StartRegistry(), Report (+7 more)

### Community 24 - "Object Layer Data Source"
Cohesion: 0.19
Nodes (9): DataSource, protoToObjectLayer(), NewWorldBuilder(), validateWorldProgression(), WorldBuilder, Content, ObjectLayer, validateObjectLayers() (+1 more)

### Community 25 - "Quest Accept and Providers"
Cohesion: 0.15
Nodes (8): QuestObjectiveProgress, QuestStepProgress, actionCell(), botSpawnCell(), containsStr(), questSnapshotContains(), stepHasIncompleteTalk(), CellKey

### Community 26 - "Movement and Direction"
Cohesion: 0.18
Nodes (10): BehaviorIsStationary(), getDirectionVector(), Sim, obstaclePushCell(), directionFromVector(), stepAlongPath(), Sim, ClampCellToGrid() (+2 more)

### Community 28 - "Hot Reload Service"
Cohesion: 0.21
Nodes (5): triggerHandler(), Service, Mode, Reloader, Result

### Community 29 - "Simulation Phase Loop"
Cohesion: 0.24
Nodes (4): ThawPlayer(), Sim, Sim, MapState

### Community 30 - "Shop and Dialogue Actions"
Cohesion: 0.27
Nodes (7): ActionQuestDialogue, ActionShopItem, CyberiaAction, questTalkDialogCode(), Content, shopBuyQuantity(), shopItemFor()

### Community 31 - "HTTP Router Middleware"
Cohesion: 0.22
Nodes (4): mountAtBasePath(), NewRouter(), problemRecoverer(), RouterOptions

### Community 32 - "Craft Recipe Handling"
Cohesion: 0.27
Nodes (5): ActionCraftRecipe, craftRecipeAt(), craftTimeMs(), Content, SnapshotCraft

### Community 33 - "HTTP Problem Responses"
Cohesion: 0.42
Nodes (7): BadRequest(), InternalServerError(), MethodNotAllowed(), NotFound(), ServiceUnavailable(), typeURI(), Problem

### Community 35 - "Logging Setup"
Cohesion: 0.40
Nodes (5): Enabled(), Init(), Level(), logAt(), resolveLevel()

### Community 40 - "Skill Registration"
Cohesion: 0.67
Nodes (3): registerHeldSkill(), registerSkill(), SkillHandlerFunc

## Knowledge Gaps
- **6 isolated node(s):** `Sim`, `Sim`, `Sim`, `EconomyRules`, `Stats` (+1 more)
  These have ≤1 connection - possible missing edges. (Counts symbols only; 119 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **9 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Session` connect `Chat and Input Session` to `Quest and Loot Dispatch`, `Entity Progression System`, `ECS World Components`, `Connection Guard and Limits`, `Object Layer Inventory`, `Bot Stats and Interaction`, `Player Storage and Item Moves`, `Pathfinding`, `Quest Progression Tracking`, `Loot Drop System`, `Bot Loot and Respawn Behavior`, `Entity Snapshot Building`, `Inventory Exchange and Vaults`, `WebSocket Client Pump`, `Quest Accept and Providers`, `Movement and Direction`, `Coin Economy and Fountain`, `Simulation Phase Loop`, `Shop and Dialogue Actions`, `Craft Recipe Handling`, `Portal Teleport and Drops`?**
  _High betweenness centrality (0.164) - this node is a cross-community bridge._
- **What connects `Sim`, `Sim`, `Sim` to the rest of the system?**
  _6 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Entity Progression System` be split into smaller, more focused modules?**
  _Cohesion score 0.05157894736842105 - nodes in this community are weakly interconnected._
- **Why does `MapState` connect `Simulation Phase Loop` to `Quest and Loot Dispatch`, `Entity Progression System`, `ECS World Components`, `Connection Guard and Limits`, `Config Loading`, `Bot Stats and Interaction`, `Instance Simulation Binding`, `Pathfinding`, `Loot Drop System`, `Bot Loot and Respawn Behavior`, `Entity Snapshot Building`, `Chat and Input Session`, `WebSocket Client Pump`, `Quest Accept and Providers`, `Movement and Direction`, `Coin Economy and Fountain`, `Craft Recipe Handling`, `Portal Teleport and Drops`, `Consumable Item Use`, `Skill Dispatch`?**
  _High betweenness centrality (0.114) - this node is a cross-community bridge._
- **Should `ECS World Components` be split into smaller, more focused modules?**
  _Cohesion score 0.07948568088836938 - nodes in this community are weakly interconnected._
- **Why does `EntityID` connect `Bot Stats and Interaction` to `Quest and Loot Dispatch`, `Entity Progression System`, `ECS World Components`, `Connection Guard and Limits`, `Config Loading`, `Instance Simulation Binding`, `Player Storage and Item Moves`, `Loot Drop System`, `Bot Loot and Respawn Behavior`, `Entity Snapshot Building`, `Chat and Input Session`, `Quest Accept and Providers`, `Movement and Direction`, `Coin Economy and Fountain`, `Simulation Phase Loop`, `Shop and Dialogue Actions`, `Craft Recipe Handling`, `Portal Teleport and Drops`, `Consumable Item Use`, `Skill Dispatch`?**
  _High betweenness centrality (0.090) - this node is a cross-community bridge._
- **Should `Connection Guard and Limits` be split into smaller, more focused modules?**
  _Cohesion score 0.06219426974143955 - nodes in this community are weakly interconnected._