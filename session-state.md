# Session state

_Last updated: 2026-08-16 (end of session, automated)_

## Branch state
- Working branch: `web-stills`, clean, fully pushed to `origin/web-stills`.
- `master` is 1 commit behind `web-stills` (only `d5f4384` "browse completeness" not yet merged).
- Do NOT auto-merge `web-stills` -> `master`: Henry asked to hold that last commit until he's
  verified the "show every reaching stand" behavior feels right in the picker. Every other
  commit from this session is already merged to `master` and deployed to GitHub Pages.
- Public site (henry-bonikowsky.github.io/brim-lineups) is live with everything through
  `1af9a5e` (pack cache bumped to v9, all 13 packs repacked and verified).

## What happened this session (chronological)
1. **Bombsite marker filter bug**: `keep_instance`'s bare `"BombSite"`/`"Bombsite_"` substring
   match was deleting real architecture (Haven's C-site floor, Ascent's B-site props, Plummet
   buildings) -> "no ground at target" holes. Fixed to a basename-prefix match.
2. **Paired-stand wedge bug**: right-clicking a stand replaced the click with its nearest
   corner pin (up to 260u away), losing real lineups. Fixed twice: first to sweep both, then
   per Henry's correction ("the corner IS where I want it") to sweep ALL corners in reach
   (not just the geometric best) with the exact click as fallback only when no corner works.
3. **Razor gate (geometry-threading filter)**: originally added `THREADS GEOMETRY` hard-drop
   two sessions ago; today it was found to be dropping 20 of 24 browse rows and mislabeling
   legitimate landing-area wall taps as `UNRELIABLE`. Now razor/graze are labels + ranking
   sinks everywhere, never drops. `graze` narrowed to require >600u carry after contact
   (`scrape` field keeps the old broad meaning as an internal polish/refine brake).
4. **Bounce friction physics**: two rounds of correction from Henry's in-game reports (a slope
   graze that glides too far, then a roof lob that carries too far) converged on FLAT full
   friction at every impact angle (not angle-scaled) - `v_t *= 1 - friction`, no curve.
5. **File-truth physics**: pulled exact values from `ValoBoard/tools/valo_dump/out/sarge_q/`
   (already-exported ability JSON) - speed 2900, gravity 1125 exactly, discovered a second
   `UpwardShift 8.0` tuning param modeled as release-height lift. Verified against Henry's
   controlled 5m wall-shot test (sim 37->44cm above crosshair vs his in-game ~49cm).
6. **Walk-mode diagnostics added**: `/shoot` now renders a full-res first-impact still with a
   precise magenta dot + prints the impact-vs-crosshair vertical offset in cm (no more
   eyeballing a soft flight video). Flight videos in walk mode now use a fixed first-person
   POV camera instead of the chase/landing cams (which pointed at the floor on short shots).
7. **Reference-finder feature**: aim-region "yellow wash" - every aim angle within +-3deg that
   still lands the throw gets painted translucent yellow on the aim card, so players can hunt
   a visual reference inside the wash. Native-server only (not in the wasm/web build - too
   many extra flights for single-threaded wasm).
8. **Pack format bug found + fixed**: repacking for deploy revealed `pack.rs` was never
   updated when `colinfo.json` (game-truth collision) shipped two days ago - EVERY map failed
   its bit-exact round-trip check. This meant the public site had been serving pre-colinfo
   collision the whole time. Fixed `pack.rs` to mirror `load_ex`'s colinfo handling; all 13
   packs rebuilt and verified, cache version bumped so browsers refetch.
9. **Completeness ruling (latest, unmerged)**: Henry: "ALL POSITIONS THAT CAN REACH THE SPIKE
   SHOULD BE SHOWN WITH THE MOST OPTIMAL ANGLE. PERIOD." Removed the 24-stand refine
   truncation, the 60m/6000u range cap, and the 20-row browse display cap. Exposed stands are
   now dropped outright in strict/browse (was: labeled and ranked last). Verified: one browse
   click went from 24 shown rows to 70, max range 7715u, ~16s solve time.

## Open items / not yet resolved
- **13m beam lineup** (Triad/Haven-style long lob threading a construction I-beam, ~9cm sim
  clearance): still unverified against the new file-truth physics. Henry was asked to re-throw
  it in game; no result yet.
- **~45-degree bounce impacts have no in-game anchor.** Both ends of the friction curve (flat
  friction) are anchored by real throws; the middle angle range is untested. Flag any report
  of a mid-angle bounce carrying visibly wrong.
- **`master` merge for the completeness-ruling commit** (`d5f4384`) is intentionally pending
  Henry's approval after trying the new picker behavior. Do not merge/deploy without his OK.
- Solve time on wide-open browse clicks is now ~16s (up from a faster truncated search) since
  every reaching stand refines. Not flagged as a problem yet, but worth watching if a future
  session gets a "too slow" complaint.

## Debt / cleanup
- None outstanding. `serve_dbg.log` (stray debug output file) was deleted at session end.
