# Engineering conventions

Rules for working in this repo. Apply them to your own edits and to code you touch.

## Use ASD-STE100 Simplified Technical English
Write comments in simplified technical English.

Key rules:
- Use the approved words of ASD-STE100 when possible.
- Use one word for one idea. Do not use two words for the same thing.
- Write short sentences. Use 20 words or less for instructions.
- Use active voice. Write "Turn the switch", not "The switch must be turned"
- Write short paragraphs. Keep one topic in each paragraph.
- No filler.

The goal is easy reading.

Report back to the user following the same rules.

## Brief comments — no archaeology

Comments state the live invariant in ≤1 line. The reader needs current behavior, nothing more.

- Banned content: bug postmortems, "we tried X but Y broke", "previously", "used to", "added because of", historical justification, restoration sagas.
- Test: "If I delete this comment, will a reader misunderstand the code?" No → delete. Yes → minimum text, present tense.
- Decision rationale and bug-fix narration go in the commit message that introduced the change — there it's dated and attributable; on the line it's stale weight forever.

## One theme per commit
One commit = one logical theme. No bundling unrelated changes.

- Before `git commit`: scan the staged diff. ≥2 themes → unstage, commit each theme separately.
- Each commit stages the minimum file set needed for that theme. No drive-by edits, no "while I'm here" cleanups.
- Themes that touch a shared file (`main.go`, message parser, central state): land the feature commits first, then one final "wire X through main loop" glue commit. Don't merge themes just to avoid the glue commit.
- Commit subject names ONE concern. If you need "and" or "+" to describe it, it's two commits.

## manifests/ — engine-cyberia owns it

`manifests/` is not ours. `engine-cyberia` writes it and an external tool syncs
it here. This repo is a passive consumer.

- A change that seems to need a `manifests/` edit: **stop**. Name the file and
  the line, and let the user take it to the owner. The sync will overwrite it.
- A change that touches `manifests/` and nothing else: allowed. Keep it in its
  own commit, and warn the user the next sync can overwrite it.
- Never bundle a `manifests/` edit with source changes in one commit.
- Do not read it as the source of truth for deploy config.

## Code questions — query the graphify graph first

`graphify-out/` holds a knowledge graph of the code. Use it first for a
question about structure, relations or "where is X". Then read the source.

- `graphify-out/GRAPH_REPORT.md` — summary: main nodes, communities, cross links.
- `graphify query "<question>"` — search the graph for a question.
- `graphify path "A" "B"` — shortest path between two nodes.
- `graphify explain "X"` — a node and its neighbors.

The graph can be older than the code. Confirm each result in the source
before you edit. If `graphify-out/` is absent, read the source directly.


# System Map

Three processes. Each one has one job.

**engine-cyberia — content.** It holds what exists and how it works: object
layers, atlases, maps, instances, quests, actions, dialogue and stat rules. It
holds no simulation state. When content changes, it tells the server to
hot-reload (gRPC control, REST fallback).

**cyberia-server — authority.** It owns the state of the world and of every
entity, and it has the final word. It loads content from the engine at boot
and on hot-reload. One fixed tick runs the phases in order: input, lifecycle,
skills, AI, movement, loot, portals, then replication.

**cyberia-client — view and prediction.** It holds a full copy of the state
and writes only the entity it owns: the own player. It draws, plays audio and
runs the UI. The UI is local: it reads local state and never waits for the
socket.

Client and server mirror their folders on subject: `net/`, `sim/`, `world/`
and `content/` exist on both sides. A folder with no counterpart is not
mirrored.

## Communication

**Content — engine to both.** The client reads content over HTTPS REST
(`cyberia-client/src/data/engine_client.c`). The server reads it over gRPC,
with REST as fallback (`cyberia-server/engine_client/`). The game server never
serves content, and the engine never sees simulation state.

**Game link — one WebSocket, `/ws`, JSON envelope.**
`cyberia-client/src/net/socket.c` ↔ `cyberia-server/game/net/`.

- **Client pushes events.** The client sends what the player did, never what
  it has. Every module pushes through `client_event_push`, which stamps frame,
  sequence and time. Raw input stays in the client.
  - Deferred events send when a UI action completes: `item_ops` (the
    add/remove list of a vault session) and `talk_done`. Slot order is not
    state and never syncs.
  - Immediate events send when a fact changes. Today only `player_stasis`
    (modal open/close), because it drives immunity this tick.
- **Server pushes state.** `PhaseInput` applies each player's events in
  sequence order. It replays item ops against its own items and clamps them.
  Each snapshot is a whole state with its `frame` and
  `inputConsumedThrough`: the highest sequence the server consumed. Consumed
  is not honored. The snapshot is the answer.

**Replay and reconciliation.** The client applies its own events at once.
At the start of each tick, `prediction_reconcile` replaces the own state with
the newest snapshot, then `client_event_trim` drops events at or below
`inputConsumedThrough`. The client re-applies the events that remain, in order,
with the same motion function the server uses. There is no other correction.
Other entities take the server state; the view interpolates them.

A reconnect is a new session. Sequence, frame and cursor restart at 0. The
client drops its pending events, takes `init_data` as truth and resends one
`player_stasis`.

## Transport Layer
Goal: Transport only: the socket moves bytes. It never reads or builds a message. Client and Server must mirror 1:1.
