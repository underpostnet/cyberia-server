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

# System Map

Three processes:

| process | role | talks to |
|---|---|---|
| **cyberia-client** | game client, game canvas | game server (WebSocket), engine (REST) |
| **cyberia-server** | authoritative simulation | client (WebSocket), engine (gRPC + REST) |
| **engine-cyberia** | external content authority (assets, config data, asset data) | serves both |

Two links carry client traffic:

1. **Game link** — one WebSocket, `/ws`, JSON envelope.
   `cyberia-client/src/network/socket.c` ↔ `cyberia-server/cmd/cyberia-server/main.go`.
2. **Content link** — HTTPS REST to the engine origin.
   `cyberia-client/src/network/engine_client.c` (`emscripten_fetch`)

The game server never serves content, and the engine never sees simulation

## Transport Layer
Goal: Transport only: the socket moves bytes. It never reads or builds a message. Client and Server must mirror 1:1.
