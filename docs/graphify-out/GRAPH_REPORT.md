# Graph Report - cyberia-server  (2026-11-16)

## Corpus Check
- 100 files · ~96,271 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1403 nodes · 5105 edges · 98 communities (41 shown, 57 thin omitted)
- Extraction: 97% EXTRACTED · 3% INFERRED · 0% AMBIGUOUS · INFERRED: 132 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `d751e30d`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- types.go
- time.Time
- EntityRow
- Client
- ComputedStats
- ObjectLayerState
- main.go
- EntityID
- connection_guard.go
- moveSet
- SkillContext
- QuestProgress
- Brain
- Goods
- MapState
- Vitals
- metrics.go
- MetricsHandler
- SendMessage
- PriorityQueue
- api/router.go
- .exchanged
- GameServer
- .vaultCapacity
- WorldBuilder
- InputCommand
- directionFromVector
- ItemPolicy
- hotreload.go
- context.Context
- CyberiaAction
- state.go
- Trade
- problem.go
- GrpcClient
- Store
- Content
- ObjectLayer
- State
- Effect
- Config
- Transfer
- Registry
- World
- Vec2
- Pipeline
- CapacityRule

## God Nodes (most connected - your core abstractions)
1. `Session` - 192 edges
2. `MapState` - 130 edges
3. `EntityID` - 116 edges
4. `Dispatcher` - 48 edges
5. `InputCommand` - 44 edges
6. `Pipeline` - 42 edges
7. `Infof()` - 39 edges
8. `ObjectLayerState` - 38 edges
9. `Warnf()` - 38 edges
10. `GameServer` - 37 edges

## Surprising Connections (you probably didn't know these)
- `NewAPIRouter()` --calls--> `DefaultCORSOrigins()`  [EXTRACTED]
  api/router.go → config/config.go
- `MetricsResponse` --references--> `SessionMode`  [EXTRACTED]
  api/metrics.go → game/net/accounts.go
- `MetricsHandler` --references--> `GameServer`  [EXTRACTED]
  api/metrics.go → game/net/server.go
- `NewMetricsHandler()` --references--> `GameServer`  [EXTRACTED]
  api/metrics.go → game/net/server.go
- `RouterOptions` --references--> `GameServer`  [EXTRACTED]
  api/router.go → game/net/server.go

## Import Cycles
- None detected.

## Communities (98 total, 57 thin omitted)

### Community 0 - "types.go"
Cohesion: 0.22
Nodes (10): QuestObjective, QuestStep, SkillLogic, Dispatcher, inputResend, resumePayload, sessionEnd, travelTicket (+2 more)

### Community 1 - "time.Time"
Cohesion: 0.08
Nodes (9): LootRespawn, FriendOp, Social, PlayerID, settled, chatBudget, Invite, invite (+1 more)

### Community 2 - "EntityRow"
Cohesion: 0.08
Nodes (11): BotStatusIcon(), ResourceStatusIcon(), Agent, Appearance, Mortal, Vitals, ComponentMask, EntityRow (+3 more)

### Community 3 - "Client"
Cohesion: 0.06
Nodes (18): restEnvelope, AcquiredSession, Pack(), Unpack(), NewSocket(), FreezePlayer(), ThawPlayer(), Debugf() (+10 more)

### Community 4 - "ComputedStats"
Cohesion: 0.05
Nodes (27): CyberiaQuest, EntityProgression, progressionEvent, QuestReward, StatBreakdown, Stats, xpRepeat, PlacedRespawnDuration() (+19 more)

### Community 5 - "ObjectLayerState"
Cohesion: 0.09
Nodes (13): firstStack(), InventoryStack, ActivateOrAppendLayer(), getOrCreateItemOL(), ActiveObjectLayerItemIDs(), buildLayer(), EntityTypeDefaultConfig, MatchesLiveItems() (+5 more)

### Community 6 - "main.go"
Cohesion: 0.09
Nodes (12): buildDigest(), main(), mountAtBasePath(), runUnderpostStatus(), serverID(), underpostStateSet(), SetBaseTypeURI(), problemRecoverer() (+4 more)

### Community 7 - "EntityID"
Cohesion: 0.07
Nodes (14): wornLayers(), ContributorSet(), RecordDamage(), RespawnWait(), applyDeadItems(), findPortalByCell(), CheckAABBCollision(), EntityID (+6 more)

### Community 8 - "connection_guard.go"
Cohesion: 0.17
Nodes (13): clientIP(), DefaultConnectionLimits(), newConnectionGuard(), newInputLimiter(), newTokenBucket(), ResolveConnectionLimits(), UnlimitedConnectionLimits(), admissionResult (+5 more)

### Community 9 - "moveSet"
Cohesion: 0.11
Nodes (18): moveSet, SnapshotVault, slotStack(), parseInput(), parseItemMove(), ClampQuantity(), ValidIdentifier(), ValidTapTarget() (+10 more)

### Community 10 - "SkillContext"
Cohesion: 0.33
Nodes (3): LayerQuantity(), SummonLifetime(), SkillContext

### Community 11 - "QuestProgress"
Cohesion: 0.09
Nodes (14): QuestObjectiveProgress, QuestProgress, QuestStepProgress, botSpawnCell(), containsStr(), firstIncompleteStepIndex(), QuestSnapshotEntry, questComplete() (+6 more)

### Community 12 - "Brain"
Cohesion: 0.17
Nodes (8): canConsume(), LootAdmits(), LootConsumes(), sharesPrincipal(), Brain, LootCollection, LootOrigin, LootOutcome

### Community 14 - "MapState"
Cohesion: 0.08
Nodes (7): BehaviorCollectsLoot(), BehaviorIsImmortal(), BehaviorIsInert(), BehaviorIsProvider(), BehaviorIsStationary(), RectsOverlap(), MapState

### Community 15 - "Vitals"
Cohesion: 0.10
Nodes (19): SnapshotCraft, HeldStack, PlayerStatusIcon(), baseEntity(), wholeStats(), SceneryClass(), DrawOrder, EntityFlags (+11 more)

### Community 16 - "metrics.go"
Cohesion: 0.17
Nodes (15): EntityMetrics, EntityTypeMetrics, HealthStatus, calculateWorkloadMetrics(), MetricsResponse, PersistenceMetrics, RuntimeMetrics, WebSocketServerMetrics (+7 more)

### Community 17 - "MetricsHandler"
Cohesion: 0.18
Nodes (6): DocsHandler, writeJSON(), MetricsHandler, livenessHandler(), Write(), methodNotAllowedHandler()

### Community 18 - "SendMessage"
Cohesion: 0.22
Nodes (4): deliver(), SendMessage(), ChatLine, Conn

### Community 20 - "api/router.go"
Cohesion: 0.14
Nodes (11): NewDocsHandler(), toJSONCompatible(), NewMetricsHandler(), Mount(), NewAPIRouter(), readinessHandlerFor(), RouterOptions, presentedKey() (+3 more)

### Community 22 - "GameServer"
Cohesion: 0.11
Nodes (9): NewContent(), SessionMode, ParseSessionMode(), newUpgrader(), originAllowed(), GameServer, NewGameServer(), New() (+1 more)

### Community 24 - "WorldBuilder"
Cohesion: 0.13
Nodes (6): DataSource, dispatch(), protoToObjectLayer(), NewWorldBuilder(), validateWorldProgression(), WorldBuilder

### Community 25 - "InputCommand"
Cohesion: 0.07
Nodes (14): Debit, Place, quote, newOpID(), quoteKind(), randomID(), EnqueueInputs(), InputCommand (+6 more)

### Community 26 - "directionFromVector"
Cohesion: 0.17
Nodes (8): getDirectionVector(), entityCenter(), directionFromVector(), stepAlongPath(), Direction, SkillAim, SkillOutcome, SkillResult

### Community 27 - "ItemPolicy"
Cohesion: 0.15
Nodes (6): EconomyRules, EconomyView, ItemPolicy, OperationFees, EconomyRulesFromProto(), itemPolicyFromProto()

### Community 28 - "hotreload.go"
Cohesion: 0.09
Nodes (12): ListenGRPC(), triggerHandler(), GRPCServer, Service, NewService(), jsonCodec, Loop, StartLoop() (+4 more)

### Community 29 - "context.Context"
Cohesion: 0.11
Nodes (13): friendFromProto(), launchCall(), pipelineError(), retryable(), PlayerClient, Account, FriendList, FriendRecord (+5 more)

### Community 30 - "CyberiaAction"
Cohesion: 0.11
Nodes (13): ActionCraftRecipe, ActionQuestDialogue, ActionShopItem, ActiveCraft, CyberiaAction, actionCell(), protoToAction(), protoToCraftItems() (+5 more)

### Community 31 - "state.go"
Cohesion: 0.11
Nodes (20): stackFromProto(), stacksFromProto(), stacksToProto(), stackToProto(), stateFromProto(), stateToProto(), bySlot(), containerStacks() (+12 more)

### Community 32 - "Trade"
Cohesion: 0.13
Nodes (10): Trade, TradeItemView, TradeLine, TradeOfferView, TradeSide, TradeState, escrowGoods(), TradeView (+2 more)

### Community 33 - "problem.go"
Cohesion: 0.42
Nodes (7): BadRequest(), InternalServerError(), MethodNotAllowed(), NotFound(), ServiceUnavailable(), typeURI(), Problem

### Community 34 - "GrpcClient"
Cohesion: 0.11
Nodes (8): NewDispatcher(), dialEngine(), NewGrpcClient(), GrpcClient, ManifestEntry, NewPlayerClient(), NewRestClient(), RestClient

### Community 36 - "Store"
Cohesion: 0.22
Nodes (4): JointResult, Ended, Store, New()

### Community 37 - "Content"
Cohesion: 0.09
Nodes (16): ActionMarket, EquipmentRulesConfig, linkedOp, MarketBroker, MarketClaim, MarketListing, marketPayload, MarketProfile (+8 more)

### Community 38 - "ObjectLayer"
Cohesion: 0.06
Nodes (22): Item, Ledger, Render, protoToItemContent(), SkillsFromConfig(), SkillDefinition, ItemContent, ItemContentRefusal() (+14 more)

### Community 39 - "State"
Cohesion: 0.14
Nodes (8): Durability, Outcome, Mark, questsEqual(), State, sameGoods(), DropLock, resolved

### Community 42 - "Effect"
Cohesion: 0.15
Nodes (8): jointToProto(), marketOpToProto(), worldDropsFromProto(), Effect, JointRequest, LinkMember, MarketOp, WorldDrop

### Community 44 - "Config"
Cohesion: 0.29
Nodes (11): Config, DefaultCORSOrigins(), getEnv(), getEnvIntPtr(), Load(), loadWSLimits(), normalizeBasePath(), parseBool() (+3 more)

### Community 46 - "Registry"
Cohesion: 0.29
Nodes (4): Registry, NewRegistry(), Report, Path()

### Community 52 - "World"
Cohesion: 0.38
Nodes (6): NewMapState(), NewPathfinder(), World, Map(), PlaceBot(), PlacePlayer()

### Community 54 - "Vec2"
Cohesion: 0.12
Nodes (12): dropChance(), stackQuantity(), obstaclePushCell(), pathCrossesObstacle(), ClampCellToGrid(), heuristic(), reconstructPath(), Cell (+4 more)

### Community 105 - "Pipeline"
Cohesion: 0.11
Nodes (11): classify(), Durability, Health, Pipeline, jittered(), Snapshot, Errorf(), link (+3 more)

## Knowledge Gaps
- **57 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Session` connect `QuestProgress` to `types.go`, `time.Time`, `Client`, `ComputedStats`, `ObjectLayerState`, `EntityID`, `moveSet`, `Brain`, `Goods`, `MapState`, `Vitals`, `SendMessage`, `.exchanged`, `GameServer`, `.vaultCapacity`, `InputCommand`, `directionFromVector`, `ItemPolicy`, `CyberiaAction`, `state.go`, `Trade`, `.BotCoinDropAmount`, `Content`, `ObjectLayer`, `State`, `Transfer`, `World`, `iter.Seq`, `Vec2`, `Pipeline`?**
  _High betweenness centrality (0.144) - this node is a cross-community bridge._
- **Should `time.Time` be split into smaller, more focused modules?**
  _Cohesion score 0.07801418439716312 - nodes in this community are weakly interconnected._
- **Why does `EntityID` connect `EntityID` to `types.go`, `time.Time`, `EntityRow`, `Client`, `ComputedStats`, `moveSet`, `SkillContext`, `QuestProgress`, `Brain`, `MapState`, `Vitals`, `SendMessage`, `GameServer`, `.vaultCapacity`, `InputCommand`, `directionFromVector`, `CyberiaAction`, `Trade`, `.BotCoinDropAmount`, `Content`, `ObjectLayer`, `World`, `Vec2`?**
  _High betweenness centrality (0.062) - this node is a cross-community bridge._
- **Should `EntityRow` be split into smaller, more focused modules?**
  _Cohesion score 0.07936507936507936 - nodes in this community are weakly interconnected._
- **Why does `Content` connect `Content` to `types.go`, `time.Time`, `Trade`, `GrpcClient`, `ComputedStats`, `ObjectLayerState`, `ObjectLayer`, `EntityID`, `Pipeline`, `QuestProgress`, `World`, `GameServer`, `InputCommand`, `ItemPolicy`, `CyberiaAction`, `state.go`?**
  _High betweenness centrality (0.049) - this node is a cross-community bridge._
- **Should `Client` be split into smaller, more focused modules?**
  _Cohesion score 0.06292966684294024 - nodes in this community are weakly interconnected._
- **Should `ComputedStats` be split into smaller, more focused modules?**
  _Cohesion score 0.05191146881287726 - nodes in this community are weakly interconnected._