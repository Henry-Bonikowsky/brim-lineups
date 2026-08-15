//! Molly flight: constants extracted from the VALORANT 13.02 files
//! (see C:\dev\research\brim-molly-physics.md). Native-only unknowns are knobs.

use crate::scene::{Scene, V3};
use nalgebra::Point3;
use parry3d::query::{Ray, RayCast};

#[derive(Clone, Copy)]
pub struct Cfg {
    pub speed: f32,       // ProjectileSpeed, file value 2900
    pub gravity: f32,     // GravityScale 0.45 x world gravity. World gravity is NOT
                          // in the map files; 2500 was an assumption. Fitted to the
                          // frame-timed 2026-08-03 clip: 1100 (world ~2440) puts the
                          // second impact at t=6.05 vs 6.04 measured.
    pub bounciness: f32,  // 0.35
    pub friction: f32,    // 0.65
    pub stop_speed: f32,  // 200
    pub eye_z: f32,       // camera height: CapsuleHalfHeight 98 + BaseEyeHeight 77 (BasePawn CDO)
    pub arc_deg: f32,     // knob: UpwardArc 8 (aim pitch offset, native combine)
    pub hand_left: f32,   // launch origin offset LEFT of the camera (the molly
                          // launcher is left-held; ThrowOffset is native).
                          // Proven by Henry aiming left-of-lineup in the sim
                          // and nailing the real lineup. Fit knob.
    pub max_time: f32,
    pub radius: f32, // projectile collision radius: flights are swept SPHERES,
                     // not rays - a box lip or roof edge the eye-line clears by
                     // a hair still blocks the real molly. GUESS (no file value
                     // extracted); calibration knob via --radius.
}

/// The game CLAMPS launch pitch: aiming higher than ~67 deg does not throw
/// steeper. Found 2026-08-15 from Henry's working Sunset lineup: his
/// recovered aim was 81.3 deg (sky-silhouette fit, IoU 0.94), the sim
/// launched it at 82.8 and died 2.6km short on a roof; the real throw's
/// range requires launch 70.0-70.5. Without a clamp no constant fits both
/// this and the aim-24/58.6 anchors. Calibration knob, +-0.25 uncertainty.
pub const LAUNCH_MAX: f32 = 70.3;

/// Crosshair (aim) pitch -> launch pitch. UpwardArc 8 is NOT a constant
/// offset: it tapers QUADRATICALLY to zero at straight-up - near-full at
/// flat aims, falling away fast on high lobs. Fitted 2026-08-10 from two
/// in-game anchors: Henry's roof-ledge throw at aim 24 flew ~1 deg higher
/// than a linear taper predicted (sim side-hit trace matched his real
/// throw at +1 deg), while his calibrated high lob at aim 58.6 needs the
/// linear-taper-minus-6%-power carry, which this curve reproduces within
/// ~1.5% at the file-validated speed/gravity. Clamped at LAUNCH_MAX.
pub fn launch_pitch(aim: f32, cfg: &Cfg) -> f32 {
    let a = aim.clamp(0.0, 90.0) / 90.0;
    (aim + cfg.arc_deg * (1.0 - a * a)).min(LAUNCH_MAX)
}

/// Inverse of launch_pitch (what to aim so the launch comes out at `launch`).
/// Launches above LAUNCH_MAX are impossible in game; callers must not
/// request them (sweeps skip). At exactly the clamp this returns the
/// LOWEST aim that reaches it.
pub fn aim_pitch(launch: f32, cfg: &Cfg) -> f32 {
    let launch = launch.min(LAUNCH_MAX);
    if launch <= cfg.arc_deg {
        launch - cfg.arc_deg // downward/flat branch: full arc applies
    } else {
        // solve k*a^2 - a + (launch - arc) = 0, k = arc/8100 (monotonic branch)
        let k = cfg.arc_deg / 8100.0;
        (1.0 - (1.0 - 4.0 * k * (launch - cfg.arc_deg)).max(0.0).sqrt()) / (2.0 * k)
    }
}

/// The projectile spawns at the hand, offset left of the camera eye; the
/// crosshair (aim reference) stays at the eye. Left of view yaw in UE coords.
pub fn hand_origin(eye: V3, yaw_deg: f32, cfg: &Cfg) -> V3 {
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    eye + V3::new(sy, -cy, 0.0) * cfg.hand_left
}

impl Default for Cfg {
    fn default() -> Self {
        Cfg {
            // s and g are jointly fitted: flight TIME pins s/g (frame-timed
            // clip), initial-arc RANGE pins s^2/g (Henry: 1-2m short at
            // s=2900/g=1100). Solving both: s=3000, g=1140. The file says
            // ProjectileSpeed 2900, but ProjectileThrowTuning has a native
            // SpeedScale default that plausibly supplies the extra ~3.5%.
            // Henry's walk-mode calibration 2026-08-03: at g=1140 the arc ran
            // 0.25-0.5m long; 1145 pulls ~0.3m back with negligible timing shift.
            // 2026-08-10: the x1.15 speed hack from the in-game ladder is
            // REVERTED - the shortfall was pitch-dependent (Henry: "the higher
            // up you look, the more inaccurate"), which no speed scale fits.
            // The real culprit was the constant +8 arc; see launch_pitch. At
            // the ladder's aim (58.6) the tapered arc at these original
            // clip-fitted constants gives the identical vacuum range the
            // x1.15 hack produced (6602 vs 6600u) - one model, all pitches
            // back to the file+clip-validated constants: the -6% trim was
            // compensating the LINEAR arc taper's error at high pitch. The
            // quadratic taper (see launch_pitch, fitted to Henry's roof-ledge
            // trace 2026-08-10) reproduces the tuned high-lob carry within
            // ~1.5% at these original values
            speed: 3000.0,
            gravity: 1145.0,
            // file DefaultBounciness 0.35. The 2026-08-03 clip fit said
            // 0.38-0.40, but that fit predates swept-sphere flights. After
            // the friction-floor fix removed the carry error, Henry's
            // 2026-08-15 roof-bounce comparison still reads the vertical
            // rebound as too strong: 0.30 is his in-game calibration.
            // Bisect further with --bounce.
            bounciness: 0.30,
            friction: 0.65,
            stop_speed: 200.0,
            eye_z: 175.0,
            arc_deg: 8.0,
            // ZERO: Henry confirmed in-game the molly launches from screen
            // center, straight, above the crosshair. No lateral hand offset.
            hand_left: 0.0,
            max_time: 8.0,
            radius: 15.0,
        }
    }
}

pub struct Outcome {
    pub rest: V3,
    pub time: f32,
    pub bounces: u32,
    /// The flight took a HARD wall impact and then carried far anyway.
    /// In-game verdict (Henry, Summit B gate bounce): such redirects are
    /// not real - the model keeps too much speed off steep wall hits.
    /// A wall TAP near the landing (kills momentum) does not set this.
    pub wall_carry: bool,
}

/// Integrate one throw. `dir` must be normalized. Semi-implicit Euler at 120 Hz
/// with a segment raycast per step (point projectile; the 1u post-bounce sphere
/// is negligible at map scale).
pub fn fly(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg) -> Option<Outcome> {
    fly_impl(scene, origin, dir, cfg, false, None, None)
}

/// Like fly, but also records the position at every integration step and the
/// step index of the FIRST bounce (for video pacing).
pub fn fly_path(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg) -> Option<(Outcome, Vec<V3>, usize)> {
    fly_path_impl(scene, origin, dir, cfg, false).map(|(o, p, f, _)| (o, p, f))
}

/// fly_path plus the path index of every PLAYER-VISIBLE bounce (keyframe stills).
pub fn fly_path_marks(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg) -> Option<(Outcome, Vec<V3>, usize, Vec<usize>)> {
    fly_path_impl(scene, origin, dir, cfg, false)
}

/// fly_path with per-impact eprintln tracing (--throw debug).
pub fn fly_path_traced(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg) -> Option<(Outcome, Vec<V3>, usize)> {
    fly_path_impl(scene, origin, dir, cfg, true).map(|(o, p, f, _)| (o, p, f))
}

fn fly_path_impl(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg, trace: bool) -> Option<(Outcome, Vec<V3>, usize, Vec<usize>)> {
    let mut path = Vec::new();
    let mut marks = Vec::new();
    let out = fly_impl(scene, origin, dir, cfg, trace, Some(&mut path), Some(&mut marks))?;
    // first bounce ~ first step where height starts rising or motion kinks;
    // recover it by re-flying is overkill: approximate as the first local
    // minimum of z after the apex
    let apex = path
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.z.total_cmp(&b.1.z))
        .map(|(i, _)| i)
        .unwrap_or(0);
    let first_bounce = (apex..path.len().saturating_sub(1))
        .find(|&i| path[i + 1].z > path[i].z + 0.01)
        .unwrap_or(path.len().saturating_sub(1));
    Some((out, path, first_bounce, marks))
}

fn fly_impl(
    scene: &Scene,
    origin: V3,
    dir: V3,
    cfg: &Cfg,
    trace: bool,
    mut record: Option<&mut Vec<V3>>,
    mut marks: Option<&mut Vec<usize>>,
) -> Option<Outcome> {
    const DT: f32 = 1.0 / 120.0;
    let mut p = origin;
    let mut v = dir * cfg.speed;
    let mut t = 0.0f32;
    let mut steps_since_bounce = 1000u32;
    let mut crevice = 0u32;
    // bounces = PLAYER-VISIBLE hops (reported, must match what you count in
    // game); contacts = every touch incl. slides and micro-hops (runaway guard)
    let mut bounces = 0u32;
    let mut contacts = 0u32;
    let mut last_wall: Option<V3> = None;
    if let Some(r) = record.as_deref_mut() {
        r.push(p);
    }
    while t < cfg.max_time {
        v.z -= cfg.gravity * DT;
        let step = v * DT;
        let len = step.norm();
        // swept SPHERE, not a ray: the real molly has a body - grazing a box
        // lip or roof edge by less than cfg.radius blocks it in game, and a
        // zero-width ray sailed through exactly those (Henry: lineups "work"
        // in sim but clip cover boxes / roofs in game). toi is a distance
        // because the cast velocity is unit-length. stop_at_penetration=false:
        // a spawn or post-bounce overlap with separating velocity is not a hit.
        // radius 0 = plain ray cast: ~6x cheaper, used by the solver's bulk
        // discovery sweep, whose tol-passers are re-flown at full radius
        let id = nalgebra::Isometry3::identity();
        let hit: Option<(f32, V3)> = if cfg.radius > 0.0 {
            let ball = parry3d::shape::Ball::new(cfg.radius);
            let opts = parry3d::query::ShapeCastOptions {
                max_time_of_impact: len,
                stop_at_penetration: false,
                ..Default::default()
            };
            parry3d::query::cast_shapes(
                &id,
                &V3::zeros(),
                &scene.mesh,
                &nalgebra::Isometry3::translation(p.x, p.y, p.z),
                &(step / len),
                &ball,
                opts,
            )
            .ok()
            .flatten()
            .map(|h| (h.time_of_impact, *h.normal1))
        } else {
            let ray = Ray::new(Point3::from(p), step / len);
            scene
                .mesh
                .cast_ray_and_get_normal(&id, &ray, len, true)
                .map(|h| (h.time_of_impact, h.normal))
        };
        if let Some((toi, hn)) = hit {
            // global grind cap: a wedged sphere can slide in near-zero time
            // fractions per contact, spinning thousands of casts for one
            // flight (2s+ per fly call). A 400-contact flight is degenerate,
            // never a usable lineup - bail
            if contacts > 400 {
                return None;
            }
            let mut n = hn;
            if n.dot(&v) > 0.0 {
                n = -n; // face the incoming velocity
            }
            t += DT * toi / len;
            p += step * (toi / len) + n * 0.5;
            // crevice guard: instant re-contacts (slope/overhang wedges) must
            // not eat the rebound; slide along the surface instead of another
            // energy-shredding reflection
            if steps_since_bounce <= 3 {
                crevice += 1;
            } else {
                crevice = 0;
            }
            steps_since_bounce = 0;
            if crevice >= 2 {
                let vn0 = v.dot(&n);
                v -= n * vn0; // kill only the into-surface component
                // sliding friction on sustained contact: without it the molly
                // skates UP steep faces (tarp slopes) barely decelerating and
                // "glitches" uphill instead of settling at the base
                let vt = v.norm();
                if vt > 1.0 {
                    v *= 1.0 - (cfg.friction * vn0.abs() / vt).min(0.35);
                }
                contacts += 1;
                if trace {
                    eprintln!("  wedge-slide t={t:.2} p=({:.0},{:.0},{:.0}) |v|={:.0}", p.x, p.y, p.z, v.norm());
                }
                continue;
            }
            // Bounciness is FLAT 0.35, no angle factor. Proven by the
            // 2026-08-03 clip: throw released t=0, second impact (tower
            // flash) at t=6.04; the max possible single arc is 5.16s, so
            // bounce 1 happened at ~4.35s and the ~1.6s second arc requires
            // restitution ~0.35-0.40 of a 2500u/s impact: plain DefaultBounciness,
            // nothing deadened. The bytecode's InterpolateRange 0..90 ->
            // 0.5..1.0 curve is the FRICTION angle scale (UE
            // bBounceAngleAffectsFriction), which lives below.
            let bounciness = cfg.bounciness;
            let vz_in = v.z;
            let vn = v.dot(&n);
            // hard wall impact (near-vertical face, substantial normal
            // speed): remember where - long carry after one is distrusted.
            // Thresholds are calibration knobs (no file values)
            if n.z.abs() < 0.5 && vn.abs() > 600.0 {
                last_wall = Some(p);
            }
            let vt = v - n * vn;
            // tangential friction scales with impact steepness: the
            // bBounceAngleAffectsFriction curve is InterpolateRange 0..90 ->
            // 0.5..1.0 - HALF friction at grazing, never zero. Scaling from
            // 0 let shallow first bounces keep nearly all forward speed and
            // skip miles past where the real molly dies (Henry, in-game
            // side-by-side 2026-08-15: sim carried to the box, his throw
            // "doesn't even get close to the target")
            let steep = (vn.abs() / v.norm().max(1.0)).clamp(0.0, 1.0);
            v = vt * (1.0 - cfg.friction * (0.5 + 0.5 * steep)) - n * vn * bounciness;
            contacts += 1;
            // a reported bounce = a NEW ARC a viewer can count: falling in,
            // rising out by >=15u (vz 180). Wall clips mid-ascent and settle
            // touches are contacts, not bounces - the number must match what
            // the player sees in game / in the video to be checkable
            if vz_in < 0.0 && v.z >= 180.0 {
                bounces += 1;
                // the contact point is pushed to `record` just below, at this index
                if let (Some(m), Some(r)) = (marks.as_deref_mut(), record.as_deref()) {
                    m.push(r.len());
                }
            }
            if trace {
                // debug-only owner lookup: ShapeCastHit carries no feature id,
                // so probe back into the just-contacted surface with a ray
                let probe = Ray::new(Point3::from(p), -n);
                let owner = match scene
                    .mesh
                    .cast_ray_and_get_normal(&id, &probe, cfg.radius * 2.0 + 2.0, true)
                    .map(|h| h.feature)
                {
                    Some(parry3d::shape::FeatureId::Face(i)) => {
                        scene.owner_of(i % scene.mesh.indices().len().max(1) as u32)
                    }
                    _ => "?",
                };
                eprintln!(
                    "  contact t={t:.2} p=({:.0},{:.0},{:.0}) n=({:.2},{:.2},{:.2}) vz_in={:.0} vz_out={:.0} |v|={:.0} bounce={} on {owner}",
                    p.x, p.y, p.z, n.x, n.y, n.z, vz_in, v.z, v.norm(), vz_in < 0.0 && v.z >= 180.0
                );
            }
            if let Some(r) = record.as_deref_mut() {
                r.push(p);
            }
            // stop rule on the BOUNCE (normal) component: hopping ends when the
            // rebound is weak AND the slide is spent; a dead hop with live
            // lateral speed keeps skipping in minimum-height hops instead
            // (the native MinBounceWhenCannotStop behavior)
            let n_speed = v.dot(&n).abs();
            let lat = (v - n * v.dot(&n)).norm();
            if n_speed < cfg.stop_speed {
                // resting requires a walkable-ish surface (native
                // BounceStopSurfaceAngle): a molly never sticks mid-slope on
                // a steep face - kill the rebound and let gravity take it down
                if (lat < cfg.stop_speed * 2.0 && n.z > 0.7) || contacts > 120 {
                    let wall_carry = last_wall.map(|w| (p - w).norm() > 1200.0).unwrap_or(false);
                    return Some(Outcome { rest: p, time: t, bounces, wall_carry });
                }
                if lat < cfg.stop_speed * 2.0 {
                    v -= n * v.dot(&n);
                    continue;
                }
                // dying-hop phase: natural decaying rebound (no artificial fixed
                // hop; keeps a small floor so the sim doesn't grind the surface)
                // with rolling loss on the slide
                let vn2 = v.dot(&n);
                let hop = vn2.abs().max(35.0).min(cfg.stop_speed);
                v = (v - n * vn2) * 0.62 + n * hop;
            }
        } else {
            p += step;
            t += DT;
            steps_since_bounce = steps_since_bounce.saturating_add(1);
            if let Some(r) = record.as_deref_mut() {
                r.push(p);
            }
            if p.z < scene.min_z - 2000.0 {
                return None; // fell out of the world
            }
        }
    }
    None
}

/// Does the fire patch actually cover `target` from a molly resting at `rest`?
/// The patch is not a free disc: it spreads along the ground in 200u cells
/// (CellSize), climbing at most 110u (StepUp) and dropping at most 210u
/// (StepDown) per cell, clamped to 450u of spread TRAVEL (ClampedRadius on
/// the wavefront path, not the crow-flies disc). A crate taller than 110u
/// therefore blocks its far side outright: wrapping around costs more path
/// than the 450u budget allows. BFS on the 5x5 cell grid.
pub fn fire_covers(scene: &Scene, rest: V3, target: V3) -> bool {
    use parry3d::query::RayCast;
    const CELL: f32 = 200.0;
    const RADIUS: f32 = 450.0;
    const STEP_UP: f32 = 110.0;
    // file value is 210, but in-game reality (Henry, Summit B): a molly
    // resting on a ~2m crate does NOT pour fire onto the ground beside it -
    // the tool claimed "covers" for exactly that and it was blatantly wrong.
    // Symmetric 110 keeps stairs/slopes spreading and blocks sheer drops
    const STEP_DOWN: f32 = 110.0;
    let id = nalgebra::Isometry3::identity();
    // ground under (x, y) near reference height z_ref, respecting step limits
    let ground_near = |x: f32, y: f32, z_ref: f32| -> Option<f32> {
        let top = z_ref + STEP_UP + 15.0;
        let ray = Ray::new(Point3::new(x, y, top), V3::new(0.0, 0.0, -1.0));
        scene
            .mesh
            .cast_ray(&id, &ray, STEP_UP + STEP_DOWN + 30.0, true)
            .map(|t| top - t)
            .filter(|g| g - z_ref <= STEP_UP && z_ref - g <= STEP_DOWN)
    };
    // flame-height line between cell centers; a wall between them blocks
    // spread. Cast 2u PAST the endpoint: a surface exactly at the endpoint
    // (wall on the cell center) must register, not boundary-miss
    let open_between = |a: V3, b: V3| -> bool {
        let d = b - a;
        let n = d.norm();
        let ray = Ray::new(Point3::from(a), d / n);
        scene.mesh.cast_ray(&id, &ray, n + 2.0, true).is_none()
    };
    let mut lit = [[false; 5]; 5];
    let mut z = [[f32::NAN; 5]; 5];
    let mut path = [[f32::INFINITY; 5]; 5]; // spread travel to reach the cell
    lit[2][2] = true;
    z[2][2] = rest.z;
    path[2][2] = 0.0;
    // fixed-point sweep; the grid is tiny, loop until no new cell lights
    let mut changed = true;
    while changed {
        changed = false;
        for i in 0..5i32 {
            for j in 0..5i32 {
                let (dx, dy) = ((i - 2) as f32 * CELL, (j - 2) as f32 * CELL);
                for (ni, nj) in [(i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)] {
                    if !(0..5).contains(&ni) || !(0..5).contains(&nj) || !lit[ni as usize][nj as usize] {
                        continue;
                    }
                    let pd = path[ni as usize][nj as usize] + CELL;
                    if pd > RADIUS || pd >= path[i as usize][j as usize] {
                        continue;
                    }
                    let zn = z[ni as usize][nj as usize];
                    let (x, y) = (rest.x + dx, rest.y + dy);
                    let Some(g) = ground_near(x, y, zn) else { continue };
                    let a = V3::new(rest.x + (ni - 2) as f32 * CELL, rest.y + (nj - 2) as f32 * CELL, zn + 60.0);
                    if !open_between(a, V3::new(x, y, g + 60.0)) {
                        continue;
                    }
                    lit[i as usize][j as usize] = true;
                    z[i as usize][j as usize] = g;
                    path[i as usize][j as usize] = pd;
                    changed = true;
                }
            }
        }
    }
    let ti = ((target.x - rest.x) / CELL).round() as i32 + 2;
    let tj = ((target.y - rest.y) / CELL).round() as i32 + 2;
    if !(0..5).contains(&ti) || !(0..5).contains(&tj) {
        return false;
    }
    lit[ti as usize][tj as usize] && (target.z - z[ti as usize][tj as usize]).abs() <= 110.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;
    use parry3d::shape::TriMesh;

    /// One big ground quad at z=0; molly thrown at 45 deg must land near the
    /// vacuum range s^2/g and come to rest (bounce chain terminates).
    #[test]
    fn parabola_lands_and_stops() {
        let quad = TriMesh::new(
            vec![
                Point3::new(-2.0e4, -2.0e4, 0.0),
                Point3::new(2.0e4, -2.0e4, 0.0),
                Point3::new(2.0e4, 2.0e4, 0.0),
                Point3::new(-2.0e4, 2.0e4, 0.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        );
        let scene = crate::scene::Scene {
            mesh: quad,
            stands: vec![],
            min_z: 0.0,
            sun: V3::new(0.55, 0.45, 0.70),
            sun_color: [1.0; 3],
            uvs: vec![],
            tri_owner: vec![(0, "ground".into())],
            tri_color: vec![(0, [0.6, 0.6, 0.6])], tri_tex: vec![],
            tri_foliage: vec![],
        };
        let cfg = Cfg::default();
        let dir = V3::new(std::f32::consts::FRAC_1_SQRT_2, 0.0, std::f32::consts::FRAC_1_SQRT_2);
        let o = fly(&scene, V3::new(0.0, 0.0, 150.0), dir, &cfg).expect("must stop");
        let vacuum = cfg.speed * cfg.speed / cfg.gravity; // 45 deg range from ground
        // visible-rebound counting: the 0.4-restitution chain off a hard 45deg
        // impact yields 2 visible hops before the rebounds go sub-perceptual
        assert!(o.bounces >= 2, "skip phase must produce visible bounces, got {}", o.bounces);
        assert!(
            o.rest.x >= vacuum * 0.95 && o.rest.x <= vacuum * 1.5,
            "rest x {} should be at or beyond the vacuum impact {vacuum} (skips carry forward)",
            o.rest.x
        );
        assert!(o.rest.z.abs() < 50.0, "rest on the ground, got z={}", o.rest.z);
    }

    /// Arc taper: full +8 flat, zero straight-up, and aim_pitch inverts
    /// launch_pitch everywhere.
    #[test]
    fn arc_taper_roundtrip() {
        let cfg = Cfg::default();
        assert!((launch_pitch(0.0, &cfg) - 8.0).abs() < 1e-4);
        assert!((launch_pitch(24.1, &cfg) - 31.53).abs() < 0.01, "near-full arc at low pitch");
        assert!((launch_pitch(58.6, &cfg) - 63.21).abs() < 0.01, "tapered arc on high lobs");
        // the game clamps launch pitch: aiming higher throws no steeper
        // (Henry's Sunset lineup, aim 81.3 -> real launch ~70)
        assert!((launch_pitch(81.3, &cfg) - LAUNCH_MAX).abs() < 1e-4, "clamped high lob");
        assert!((launch_pitch(90.0, &cfg) - LAUNCH_MAX).abs() < 1e-4, "clamped straight up");
        for aim in [-30.0f32, -5.0, 0.0, 10.0, 45.0, 58.6, 65.0] {
            let back = aim_pitch(launch_pitch(aim, &cfg), &cfg);
            assert!((back - aim).abs() < 1e-3, "roundtrip {aim} -> {back}");
        }
    }

    /// Fire on one side of a tall box must NOT cover a spot on the other
    /// side, even a SHORT box: wrapping around costs more spread travel than
    /// the 450u budget, so behind-box is never covered.
    #[test]
    fn fire_blocked_by_box() {
        let mut verts = vec![
            Point3::new(-2.0e4, -2.0e4, 0.0),
            Point3::new(2.0e4, -2.0e4, 0.0),
            Point3::new(2.0e4, 2.0e4, 0.0),
            Point3::new(-2.0e4, 2.0e4, 0.0),
        ];
        let mut tris = vec![[0u32, 1, 2], [0, 2, 3]];
        // 160u-tall wall at x=200, spanning only y=-300..300: wrapping around
        // it is geometrically possible but exceeds the spread-travel budget
        let base = verts.len() as u32;
        for (x, y, z) in [(200.0f32, -300.0f32, 0.0f32), (200.0, 300.0, 0.0), (200.0, 300.0, 160.0), (200.0, -300.0, 160.0)] {
            verts.push(Point3::new(x, y, z));
        }
        tris.push([base, base + 1, base + 2]);
        tris.push([base, base + 2, base + 3]);
        let scene = crate::scene::Scene {
            mesh: TriMesh::new(verts, tris),
            stands: vec![],
            min_z: 0.0,
            sun: V3::new(0.55, 0.45, 0.70),
            sun_color: [1.0; 3],
            uvs: vec![],
            tri_owner: vec![(0, "ground".into())],
            tri_color: vec![(0, [0.6, 0.6, 0.6])], tri_tex: vec![],
            tri_foliage: vec![],
        };
        let rest = V3::new(0.0, 0.0, 1.0);
        let behind = V3::new(400.0, 0.0, 1.0);
        let open = V3::new(-400.0, 0.0, 1.0);
        assert!(!fire_covers(&scene, rest, behind), "wall must block the spread");
        assert!(fire_covers(&scene, rest, open), "open side must be covered");
    }

    /// A molly resting ON TOP of a ~2m crate must NOT claim to cover the
    /// ground beside it (Henry, in game: the fire never reaches down).
    #[test]
    fn crate_top_does_not_cover_ground() {
        let mut verts = vec![
            Point3::new(-2.0e4, -2.0e4, 0.0),
            Point3::new(2.0e4, -2.0e4, 0.0),
            Point3::new(2.0e4, 2.0e4, 0.0),
            Point3::new(-2.0e4, 2.0e4, 0.0),
        ];
        let mut tris = vec![[0u32, 1, 2], [0, 2, 3]];
        // crate top: 300x300 slab at z=200 centered at origin
        let b = verts.len() as u32;
        for (x, y) in [(-150.0f32, -150.0f32), (150.0, -150.0), (150.0, 150.0), (-150.0, 150.0)] {
            verts.push(Point3::new(x, y, 200.0));
        }
        tris.push([b, b + 1, b + 2]);
        tris.push([b, b + 2, b + 3]);
        let scene = crate::scene::Scene {
            mesh: TriMesh::new(verts, tris),
            stands: vec![],
            min_z: 0.0,
            sun: V3::new(0.55, 0.45, 0.70),
            sun_color: [1.0; 3],
            uvs: vec![],
            tri_owner: vec![(0, "ground".into())],
            tri_color: vec![(0, [0.6, 0.6, 0.6])],
            tri_tex: vec![],
            tri_foliage: vec![],
        };
        let on_crate = V3::new(0.0, 0.0, 201.0);
        let ground_beside = V3::new(400.0, 0.0, 1.0);
        assert!(!fire_covers(&scene, on_crate, ground_beside), "2m crate top must not cover the ground");
    }

    /// A roof lip the flight line clears by ~5u must still block the molly:
    /// the projectile is a swept SPHERE (cfg.radius), not a ray. In-game
    /// failures were exactly this graze class (cover boxes, roof edges).
    #[test]
    fn graze_blocks_sphere_not_ray() {
        let mut verts = vec![
            Point3::new(-2.0e4, -2.0e4, 0.0),
            Point3::new(2.0e4, -2.0e4, 0.0),
            Point3::new(2.0e4, 2.0e4, 0.0),
            Point3::new(-2.0e4, 2.0e4, 0.0),
        ];
        let mut tris = vec![[0u32, 1, 2], [0, 2, 3]];
        // roof slab, bottom face at z=100, starting at x=500
        let b = verts.len() as u32;
        for (x, y) in [(500.0f32, -300.0f32), (2000.0, -300.0), (2000.0, 300.0), (500.0, 300.0)] {
            verts.push(Point3::new(x, y, 100.0));
        }
        tris.push([b, b + 1, b + 2]);
        tris.push([b, b + 2, b + 3]);
        let scene = crate::scene::Scene {
            mesh: TriMesh::new(verts, tris),
            stands: vec![],
            min_z: 0.0,
            sun: V3::new(0.55, 0.45, 0.70),
            sun_color: [1.0; 3],
            uvs: vec![],
            tri_owner: vec![(0, "ground".into())],
            tri_color: vec![(0, [0.6, 0.6, 0.6])],
            tri_tex: vec![],
            tri_foliage: vec![],
        };
        // flat throw at z=95: the flight line passes ~5u under the slab lip
        let cfg = Cfg::default();
        let o = fly(&scene, V3::new(400.0, 0.0, 95.0), V3::new(1.0, 0.0, 0.0), &cfg).expect("stops");
        assert!(o.rest.x < 1200.0, "sphere must clip the roof lip, rest.x={}", o.rest.x);
        let mut thin = cfg;
        thin.radius = 0.01;
        let o = fly(&scene, V3::new(400.0, 0.0, 95.0), V3::new(1.0, 0.0, 0.0), &thin).expect("stops");
        assert!(o.rest.x > 1500.0, "a zero-width flight sails under, rest.x={}", o.rest.x);
    }
}

