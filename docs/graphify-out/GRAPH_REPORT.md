# Graph Report - cyberia-server  (2026-10-29)

## Corpus Check
- 99 files · ~95,016 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1394 nodes · 4970 edges · 103 communities (51 shown, 52 thin omitted)
- Extraction: 97% EXTRACTED · 3% INFERRED · 0% AMBIGUOUS · INFERRED: 127 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `1416ceee`
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
- MapState
- .buildMapsFromInstance
- moveSet
- Vec2
- QuestProgress
- Brain
- RestClient
- .handleSkillCollisions
- Vitals
- metrics.go
- MetricsHandler
- Session
- SceneryClass
- api/router.go
- .exchanged
- GameServer
- grpc.go
- Infof
- Debugf
- obstaclePushCell
- ItemPolicy
- hotreload.go
- context.Context
- CyberiaAction
- Stack
- Trade
- problem.go
- GrpcClient
- logAt
- Pipeline
- Content
- ItemContent
- State
- ObjectLayer
- Effect
- .place
- Config
- Transfer
- Registry
- Store
- ObjectLayerData
- MarketQuery
- Outcome
- docs.go
- World
- PriorityQueue
- Quest

## God Nodes (most connected - your core abstractions)
1. `Session` - 192 edges
2. `MapState` - 126 edges
3. `EntityID` - 110 edges
4. `Dispatcher` - 48 edges
5. `Pipeline` - 42 edges
6. `InputCommand` - 41 edges
7. `Infof()` - 39 edges
8. `ObjectLayerState` - 38 edges
9. `GameServer` - 37 edges
10. `Warnf()` - 37 edges

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

## Communities (103 total, 52 thin omitted)

### Community 0 - "types.go"
Cohesion: 0.18
Nodes (14): CapacityRule, QuestObjective, QuestStep, Dispatcher, init(), registerHeldSkill(), registerSkill(), inputResend (+6 more)

### Community 1 - "time.Time"
Cohesion: 0.05
Nodes (24): LootRespawn, NewContent(), DefaultConnectionLimits(), newConnectionGuard(), newInputLimiter(), newTokenBucket(), ResolveConnectionLimits(), UnlimitedConnectionLimits() (+16 more)

### Community 2 - "EntityRow"
Cohesion: 0.08
Nodes (12): BotStatusIcon(), Agent, Appearance, DrawOrder, EntityFlags, Mortal, ComponentMask, EntityData (+4 more)

### Community 3 - "Client"
Cohesion: 0.13
Nodes (6): Pack(), NewSocket(), EnqueueInputs(), Client, Metrics, Socket

### Community 4 - "ComputedStats"
Cohesion: 0.07
Nodes (18): EntityProgression, progressionEvent, StatBreakdown, Stats, xpRepeat, EconomyRulesFromProto(), itemPolicyFromProto(), aggregateStats() (+10 more)

### Community 5 - "ObjectLayerState"
Cohesion: 0.09
Nodes (14): firstStack(), InventoryStack, ActivateOrAppendLayer(), getOrCreateItemOL(), ActiveObjectLayerItemIDs(), buildLayer(), dropChance(), EntityTypeDefaultConfig (+6 more)

### Community 6 - "main.go"
Cohesion: 0.10
Nodes (10): buildDigest(), main(), mountAtBasePath(), runUnderpostStatus(), serverID(), underpostStateSet(), ParseSessionMode(), SetBaseTypeURI() (+2 more)

### Community 7 - "MapState"
Cohesion: 0.06
Nodes (11): wornLayers(), newOpID(), findPortalByCell(), RectsOverlap(), EntityID, MapState, portalEntry, Friend (+3 more)

### Community 8 - ".buildMapsFromInstance"
Cohesion: 0.34
Nodes (3): PlacedRespawnDuration(), entSize(), entTransform()

### Community 9 - "moveSet"
Cohesion: 0.12
Nodes (16): moveSet, SnapshotVault, slotStack(), parseInput(), parseItemMove(), ClampQuantity(), ValidIdentifier(), ValidTapTarget() (+8 more)

### Community 10 - "Vec2"
Cohesion: 0.12
Nodes (7): heuristic(), reconstructPath(), Cell, DropLock, Vec2, Node, Pathfinder

### Community 11 - "QuestProgress"
Cohesion: 0.11
Nodes (12): QuestObjectiveProgress, QuestProgress, QuestStepProgress, botSpawnCell(), containsStr(), firstIncompleteStepIndex(), QuestSnapshotEntry, questComplete() (+4 more)

### Community 12 - "Brain"
Cohesion: 0.12
Nodes (11): canConsume(), LootAdmits(), LootConsumes(), sharesPrincipal(), pathCrossesObstacle(), CheckAABBCollision(), Brain, Transform (+3 more)

### Community 14 - ".handleSkillCollisions"
Cohesion: 0.15
Nodes (8): BehaviorCollectsLoot(), BehaviorIsImmortal(), BehaviorIsInert(), BehaviorIsProvider(), ContributorSet(), RecordDamage(), RespawnWait(), applyDeadItems()

### Community 15 - "Vitals"
Cohesion: 0.12
Nodes (16): SnapshotCraft, HeldStack, PlayerStatusIcon(), ResourceStatusIcon(), baseEntity(), msUntil(), wholeStats(), Vitals (+8 more)

### Community 16 - "metrics.go"
Cohesion: 0.15
Nodes (15): EntityMetrics, EntityTypeMetrics, HealthStatus, calculateWorkloadMetrics(), MetricsResponse, PersistenceMetrics, RuntimeMetrics, WebSocketServerMetrics (+7 more)

### Community 17 - "MetricsHandler"
Cohesion: 0.20
Nodes (5): writeJSON(), MetricsHandler, livenessHandler(), clientIP(), methodNotAllowedHandler()

### Community 18 - "Session"
Cohesion: 0.07
Nodes (16): FreezePlayer(), ThawPlayer(), Sim, deliver(), InputCommand, Session, SendAck(), SendMessage() (+8 more)

### Community 20 - "api/router.go"
Cohesion: 0.13
Nodes (11): NewMetricsHandler(), Mount(), NewAPIRouter(), readinessHandlerFor(), RouterOptions, newUpgrader(), originAllowed(), presentedKey() (+3 more)

### Community 21 - ".exchanged"
Cohesion: 0.31
Nodes (3): Stack, LayerQuantity(), RemoveLayerItem()

### Community 22 - "GameServer"
Cohesion: 0.13
Nodes (6): SessionMode, GameServer, Snapshot, Errorf(), runtimeCounters, Stats

### Community 23 - "grpc.go"
Cohesion: 0.14
Nodes (6): Unpack(), GRPCServer, jsonCodec, Request, eventsPayload, Message

### Community 24 - "Infof"
Cohesion: 0.13
Nodes (7): DataSource, dispatch(), protoToObjectLayer(), NewWorldBuilder(), validateWorldProgression(), WorldBuilder, Infof()

### Community 25 - "Debugf"
Cohesion: 0.12
Nodes (9): CyberiaQuest, QuestReward, actionCell(), protoToAction(), marketProfileFromProto(), protoToQuest(), playerSpawnFromInstance(), printInstanceGraph() (+1 more)

### Community 26 - "obstaclePushCell"
Cohesion: 0.15
Nodes (9): BehaviorIsStationary(), SummonLifetime(), getDirectionVector(), obstaclePushCell(), directionFromVector(), stepAlongPath(), ClampCellToGrid(), Direction (+1 more)

### Community 27 - "ItemPolicy"
Cohesion: 0.11
Nodes (6): EconomyRules, EconomyView, ItemPolicy, OperationFees, quoteKind(), randomID()

### Community 28 - "hotreload.go"
Cohesion: 0.18
Nodes (9): ListenGRPC(), triggerHandler(), Service, NewService(), Loop, StartLoop(), Mode, Reloader (+1 more)

### Community 29 - "context.Context"
Cohesion: 0.13
Nodes (10): friendFromProto(), launchCall(), pipelineError(), retryable(), PlayerClient, FriendList, FriendRecord, IssuedTicket (+2 more)

### Community 30 - "CyberiaAction"
Cohesion: 0.13
Nodes (11): ActionCraftRecipe, ActionQuestDialogue, ActionShopItem, ActiveCraft, CyberiaAction, protoToCraftItems(), craftRecipeAt(), craftTimeMs() (+3 more)

### Community 31 - "Stack"
Cohesion: 0.11
Nodes (17): Goods, stackFromProto(), stacksFromProto(), stacksToProto(), stackToProto(), stateFromProto(), stateToProto(), bySlot() (+9 more)

### Community 32 - "Trade"
Cohesion: 0.12
Nodes (11): Trade, TradeItemView, TradeLine, TradeOfferView, TradeSide, TradeState, escrowGoods(), TradeView (+3 more)

### Community 33 - "problem.go"
Cohesion: 0.30
Nodes (9): BadRequest(), InternalServerError(), MethodNotAllowed(), NotFound(), ServiceUnavailable(), typeURI(), Write(), problemRecoverer() (+1 more)

### Community 34 - "GrpcClient"
Cohesion: 0.13
Nodes (7): NewDispatcher(), dialEngine(), NewGrpcClient(), GrpcClient, ManifestEntry, NewPlayerClient(), NewRestClient()

### Community 35 - "logAt"
Cohesion: 0.40
Nodes (5): Enabled(), Init(), Level(), logAt(), resolveLevel()

### Community 36 - "Pipeline"
Cohesion: 0.13
Nodes (10): classify(), CommitRequest, CommitResult, Durability, Health, Pipeline, jittered(), Warnf() (+2 more)

### Community 37 - "Content"
Cohesion: 0.12
Nodes (9): ActionMarket, EquipmentRulesConfig, linkedOp, MarketBroker, MarketListing, MarketProfile, Content, endsIn() (+1 more)

### Community 38 - "ItemContent"
Cohesion: 0.12
Nodes (12): protoToItemContent(), SkillsFromConfig(), SkillDefinition, ItemContent, ItemContentRefusal(), labelUnion(), distinct(), fetchWithRetry() (+4 more)

### Community 39 - "State"
Cohesion: 0.17
Nodes (5): Mark, questsEqual(), State, sameGoods(), link

### Community 40 - "ObjectLayer"
Cohesion: 0.22
Nodes (4): ObjectLayer, holdStacks(), labelSet(), validateObjectLayers()

### Community 42 - "Effect"
Cohesion: 0.15
Nodes (8): jointToProto(), marketOpToProto(), worldDropsFromProto(), Effect, JointRequest, LinkMember, MarketOp, WorldDrop

### Community 43 - ".place"
Cohesion: 0.20
Nodes (3): Account, AcquiredSession, identity

### Community 44 - "Config"
Cohesion: 0.29
Nodes (11): Config, DefaultCORSOrigins(), getEnv(), getEnvIntPtr(), Load(), loadWSLimits(), normalizeBasePath(), parseBool() (+3 more)

### Community 45 - "Transfer"
Cohesion: 0.23
Nodes (5): Debit, Place, quote, Transfer, transferPlan

### Community 46 - "Registry"
Cohesion: 0.29
Nodes (4): Registry, NewRegistry(), Report, Path()

### Community 47 - "Store"
Cohesion: 0.18
Nodes (6): JointResult, Checkpoint, Ended, Store, New(), Position

### Community 48 - "ObjectLayerData"
Cohesion: 0.20
Nodes (8): Item, Ledger, Render, ObjectLayerData, olMetadata(), skillEntries(), OLMeta, SkillMapEntry

### Community 49 - "MarketQuery"
Cohesion: 0.22
Nodes (6): MarketClaim, marketPayload, marketSeen, marketViewFromProto(), MarketQuery, MarketView

### Community 50 - "Outcome"
Cohesion: 0.28
Nodes (3): Durability, Outcome, resolved

### Community 51 - "docs.go"
Cohesion: 0.32
Nodes (3): NewDocsHandler(), toJSONCompatible(), DocsHandler

### Community 52 - "World"
Cohesion: 0.38
Nodes (6): NewMapState(), NewPathfinder(), World, Map(), PlaceBot(), PlacePlayer()

### Community 55 - "Quest"
Cohesion: 0.40
Nodes (4): durableQuest(), Quest, Objective, QuestStep

## Knowledge Gaps
- **52 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Session` connect `Session` to `types.go`, `time.Time`, `Client`, `ComputedStats`, `ObjectLayerState`, `MapState`, `moveSet`, `Vec2`, `QuestProgress`, `Brain`, `.handleSkillCollisions`, `Vitals`, `.exchanged`, `GameServer`, `Debugf`, `obstaclePushCell`, `ItemPolicy`, `CyberiaAction`, `Stack`, `Trade`, `Pipeline`, `Content`, `State`, `.place`, `Transfer`, `ObjectLayerData`, `World`, `iter.Seq`, `Quest`?**
  _High betweenness centrality (0.132) - this node is a cross-community bridge._
- **Should `time.Time` be split into smaller, more focused modules?**
  _Cohesion score 0.0539906103286385 - nodes in this community are weakly interconnected._
- **Why does `EntityID` connect `MapState` to `types.go`, `EntityRow`, `Client`, `.buildMapsFromInstance`, `moveSet`, `Vec2`, `QuestProgress`, `Brain`, `.handleSkillCollisions`, `Vitals`, `Session`, `GameServer`, `obstaclePushCell`, `ItemPolicy`, `CyberiaAction`, `Trade`, `Content`, `Transfer`, `MarketQuery`, `World`?**
  _High betweenness centrality (0.060) - this node is a cross-community bridge._
- **Should `EntityRow` be split into smaller, more focused modules?**
  _Cohesion score 0.08199643493761141 - nodes in this community are weakly interconnected._
- **Why does `Content` connect `Content` to `types.go`, `time.Time`, `ComputedStats`, `ObjectLayerState`, `MapState`, `QuestProgress`, `Session`, `GameServer`, `Debugf`, `ItemPolicy`, `CyberiaAction`, `Stack`, `Trade`, `GrpcClient`, `ItemContent`, `ObjectLayer`, `Transfer`, `MarketQuery`, `World`, `Quest`?**
  _High betweenness centrality (0.050) - this node is a cross-community bridge._
- **Should `Client` be split into smaller, more focused modules?**
  _Cohesion score 0.12807881773399016 - nodes in this community are weakly interconnected._
- **Should `ComputedStats` be split into smaller, more focused modules?**
  _Cohesion score 0.07051282051282051 - nodes in this community are weakly interconnected._