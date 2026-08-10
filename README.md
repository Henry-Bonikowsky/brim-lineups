# brim-lineups

Computes every physically possible Brimstone incendiary lineup to a chosen target spot on
any VALORANT map, ranked by time-to-land, using physics constants and 3D geometry extracted
from the game files (patch 13.02). See `C:\dev\research\brim-molly-physics.md` for the
extraction and the math.

## Data prerequisite

Per-map dumps produced by `ValoBoard/tools/valo_dump` (`map` mode), at
`ValoBoard/third_party/valorant_dump/<Map>/` (instances.json + meshes/*.obj + navmesh.json).

## Usage

```
cargo run --release -- <mapDumpDir> --target X,Y,Z [options]

--target X,Y,Z   landing spot in UE world units (site centers are in <Map>_Mode_BombMode.json
                 / <Map>_Gameplay.json, Bomb_Site_Outline actors)
--tol 150        landing tolerance in units
--min-dist 1000  ignore trivial close tosses; lineups are throws from range
--top 15         rows printed (full set goes to lineups.json)
--eye 175        launch height: pawn camera from the files (98+77) [calibration knob]
--arc 8          UpwardArc: added to crosshair pitch, TAPERING linearly to 0 at
                 straight-up (fitted in-game 2026-08-10: a constant +8 made every
                 high lob launch too steep and land short)          [calibration knob]
--speed 2900     ProjectileSpeed from the files

--probe X,Y,Z    debug: raycasts down/up/north at a point, names the hit mesh
--throw X,Y,Z,yaw,pitch   debug: trace one throw bounce by bounce
```

Output columns: stand position, range, crosshair yaw/pitch (deg), flight time, bounces,
landing error, forgiveness (fraction of +-0.75 deg aim jitters still inside tolerance).

## Position rule

Every reported stand is a wedge coordinate: against TWO differently-facing wall faces (a
wall corner, angled walls, or an object against a wall). The solver snaps each candidate
to the capsule-pinned position (CapsuleRadius 42 from the files) - press W into the
corner and the game stops you on the exact computed spot every time. Candidates with no
wedge within reach (~110u) are discarded; a right-clicked stand in paired mode snaps the
same way or yields no lineups.

## Physics model (from the game files)

Launch 2900 u/s, gravity 1125 u/s^2 (world -2500 x scale 0.45), restitution 0.35,
friction 0.65, stop under 200 u/s, per-bounce restitution deadening
`b *= lerp(0.5..1.0, angle-from-vertical / 90)` (decoded from Projectile_BaseGrenade
bytecode). Collision surface: /Game/Environment/ meshes with live collision; NoCollision /
trigger components filtered; BVPawn volumes and the crude Box_For_Volumes cubes in
BVProjectile sublevels are excluded (2026-08-10: a real Triad throw flew straight through
one), while the accurate per-prop *Collision shells in those sublevels (Breeze pyramid,
Triad cargo tarp) do block and stay loaded.

## Native unknowns (why --eye and --arc exist)

Whether the throw originates exactly at the camera, and the UpwardArc/UpwardShift
combination, live in native code. Defaults: origin = stand + 175u (the pawn camera height
from BasePawn: CapsuleHalfHeight 98 + BaseEyeHeight 77), launch pitch = crosshair + 8 deg.
Calibrate
in-game: stand on a computed lineup, aim at the computed yaw/pitch, compare the landing;
adjust --eye (shifts short/long uniformly at all ranges) and --arc (shifts reported pitch).

## Verified

- Bounce/flight self-test: `cargo test --release` (45-deg parabola lands at vacuum range on
  a flat scene and the bounce chain terminates).
- Haven A site: 186 distinct lineups beyond 2000u, top entries are flat ~1s throws from
  A Long/Lobby ground; B site center correctly yields none within 150u (it sits under the
  temple roof).
- Lotus: 147 lineups to a site from 2000u+, several 100% forgiveness / 5u error.
- IN-GAME VALIDATED 2026-08-02: Henry threw a computed #1 lineup in-game and it landed
  exactly on target. The file-derived defaults (--eye 175, --arc 8) are confirmed correct
  with no calibration offset. The physics, geometry, solver, and aim renders are all
  verified against the live game.

## Lineup cards

`cards/index.html` renders every site's top lineups on the real minimap textures (dots = stands, ring = target, hover for aim numbers, tables with crosshair reference points). Regenerate with `.\make_cards.ps1` after new solver runs. Minimaps + camera transforms extracted from the paks (valo_dump `tex` mode + per-map AresMinimapCamera).

## Click-to-solve picker

Run `.\start_picker.ps1` (starts `brim-lineups serve` and opens http://localhost:8777/picker.html). Left-click = landing spot, right-click = standing spot (optional). Results with aim/stand/context renders appear inline; first click per map loads its scenes (~5-20s), later clicks are fast.
