# Session state

_Last updated: 2026-08-17 (end of session, automated)_

## Branch state
- Working branch: `web-stills`, clean, fully pushed to `origin/web-stills` (HEAD `f32ec5a`).
- `master` is 2 commits behind `web-stills` (`d5f4384` completeness ruling + `f32ec5a` speed/cache
  fix, neither merged yet).
- Do NOT auto-merge `web-stills` -> `master`: the completeness-ruling hold from last session
  still applies, and today's speed change (2900 -> 2930) has not been through a full in-game
  verification pass beyond the one anchor that motivated it. Public site is unaffected either
  way (still serving the last merged `master`, through `1af9a5e`).

## What happened this session (chronological)
1. **Bug report investigated**: Henry said his sim molly on Foxtrot A ("click #12") threw
   noticeably shorter than in game - his real throw smacked high into the ship-hull "dome"
   that the sim's arc passed under. Click #12's exact coordinates were unrecoverable from the
   live log (see item 3), so the aim was reconstructed forensically: brute-force `/pov`
   render matching against Henry's saved aim-card BMP in `cards/live/`, narrowing stand +
   yaw/pitch to sub-0.1deg (stand (851,2886), yaw 46.00, pitch 61.90).
2. **Speed recalibrated to 2930** (`src/sim.rs`, commit `f32ec5a`): replaying the recovered
   throw proved the file value 2900 misses the ship-hull dome across the full +-0.2deg aim
   window Henry could plausibly have used, while >=2915 hits it exactly like his report. 2930
   is the two 2026-08-15 Sunset anchors (originally fit at gravity 1145) rescaled to the
   file-truth gravity 1125 at constant `s^2/g`; it satisfies all three now-independent
   in-game anchors. This is a real physics change - lineup outputs shift slightly; any client
   holding an open browse list needs to re-click to pick up new rows.
3. **Root-caused why the click was unrecoverable**: the previous session's "clean up
   `serve_dbg.log`" deleted the file serve was actively writing `[click]`/`[shoot]` lines to
   (stderr redirect), silently killing all future click logging with no error. Documented as
   a standing pitfall in memory: only delete that log when serve is not running.
4. **Serve solve cache added** (`src/serve.rs`, commit `f32ec5a`): opening any row in the
   picker (`n=K`) was re-running the entire browse solve from scratch just to render that
   one row's images - every row click cost the full ~16s solve. Added a single-entry cache
   keyed on `(map, tx, ty, stand, tol, list_mode)`; verified deterministic (two identical
   solves diffed byte-for-byte, 82/82 rows matched) before trusting the cache. Measured: row
   click 15.7s -> 1.35s. This was in direct response to Henry flagging picker slowness
   mid-session ("takes way too long... don't make it worse, just make it faster").
5. Rebuilt release binary, ran `cargo test --release` (7/7 pass), restarted serve, verified
   both the dome-hit reproduction and the row-click speedup live before committing.

## Open items / not yet resolved
- **Henry has not yet re-verified 2930 in-game.** It's derived from one new anchor plus two
  rescaled old ones, all self-consistent, but the fix was made and shipped to the branch
  without a fresh in-game throw confirming the new number feels right. Ask for a follow-up
  throw report before considering this closed.
- **13m beam lineup** (Triad/Haven-style long lob threading a construction I-beam): still
  unverified against file-truth physics, now further unverified against the 2930 speed bump.
  No result from Henry yet.
- **~45-degree bounce impacts have no in-game anchor** (carried over, unrelated to this
  session's changes).
- **`master` merge** for both the completeness-ruling commit and today's speed/cache commit
  is intentionally pending Henry's approval. Do not merge/deploy without his OK.
- Solve time on wide-open browse clicks is still ~16s for the FIRST click on a spot (the
  cache only helps repeat/row clicks on the same click). Not flagged as a problem, but the
  next lever if Henry complains about first-click latency specifically.

## Debt / cleanup
- `serve_dbg.log` exists (serve is running, actively writing to it as of session end) - do
  NOT delete it while `brim-lineups.exe serve` is running (see item 3 above); safe to delete
  once serve is stopped and Henry doesn't need the click history.
