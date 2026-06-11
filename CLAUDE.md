# Engineering conventions

Rules for working in this repo. Apply them to your own edits and to code you touch.

## Fail-fast over defensive

Panic / `log.Fatalf` when a required config or resource is missing — do not write defensive fallback or multi-location search logic.

- A loud crash surfaces misconfiguration immediately; walk-up/fallback logic adds complexity for cases that shouldn't happen in a correct deployment.
- Assume `go run` and binaries run from the project root. Don't search parent dirs for `.env`, `public/`, etc.
- Keep genuinely-optional config optional (`.env` absence is fine — env vars may be set directly), but drop multi-location resolution fallbacks.

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
