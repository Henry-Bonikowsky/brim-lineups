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
    // Henry's live HUD calibration (picker sliders): the UI layer is scaled
    // about bottom-center and shifted; anchors must transform the same way
    pub hud_scale: f32,
    pub hud_dy: f32, // percent of screen height, positive = down
}

/// Crosshair (aim) pitch -> launch pitch. UpwardArc 8 is NOT a constant
/// offset: it tapers linearly to zero at straight-up (native UpwardArc/
/// UpwardShift combine, fitted 2026-08-10 - a constant +8 made every high
/// lob launch too steep, and past 45 deg steeper = shorter, which matched
/// Henry's "the higher up you look, the more inaccurate". The taper at aim
/// 58.6 exactly reproduces the x1.15 range hack it replaced.)
pub fn launch_pitch(aim: f32, cfg: &Cfg) -> f32 {
    aim + cfg.arc_deg * (1.0 - aim.clamp(0.0, 90.0) / 90.0)
}

/// Inverse of launch_pitch (what to aim so the launch comes out at `launch`).
pub fn aim_pitch(launch: f32, cfg: &Cfg) -> f32 {
    if launch <= cfg.arc_deg {
        launch - cfg.arc_deg // downward/flat branch: full arc applies
    } else {
        (launch - cfg.arc_deg) / (1.0 - cfg.arc_deg / 90.0)
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
            // taper alone ran a couple meters long in-game -> Henry: -4%
            // (s,g scaled together, flight times hold)
            speed: 2880.0,
            gravity: 1099.0,
            // file DefaultBounciness is 0.35 but live walk-mode-vs-game
            // comparison (2026-08-03) shows real rebounds run hotter: the
            // measured restitution from the frame-timed clip was 0.38-0.40,
            // and 0.35 left a rebound 80u short of a box the real molly
            // clears. 0.40 matches both.
            bounciness: 0.40,
            friction: 0.65,
            stop_speed: 200.0,
            eye_z: 175.0,
            arc_deg: 8.0,
            // ZERO: Henry confirmed in-game the molly launches from screen
            // center, straight, above the crosshair. No lateral hand offset.
            hand_left: 0.0,
            max_time: 8.0,
            hud_scale: 1.0,
            hud_dy: 0.0,
        }
    }
}

pub struct Outcome {
    pub rest: V3,
    pub time: f32,
    pub bounces: u32,
}

/// Integrate one throw. `dir` must be normalized. Semi-implicit Euler at 120 Hz
/// with a segment raycast per step (point projectile; the 1u post-bounce sphere
/// is negligible at map scale).
pub fn fly(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg) -> Option<Outcome> {
    fly_impl(scene, origin, dir, cfg, false, None)
}

pub fn fly_traced(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg) -> Option<Outcome> {
    fly_impl(scene, origin, dir, cfg, true, None)
}

/// Like fly, but also records the position at every integration step and the
/// step index of the FIRST bounce (for video pacing).
pub fn fly_path(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg) -> Option<(Outcome, Vec<V3>, usize)> {
    let mut path = Vec::new();
    let out = fly_impl(scene, origin, dir, cfg, false, Some(&mut path))?;
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
    Some((out, path, first_bounce))
}

fn fly_impl(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg, trace: bool, mut record: Option<&mut Vec<V3>>) -> Option<Outcome> {
    const DT: f32 = 1.0 / 120.0;
    let mut p = origin;
    let mut v = dir * cfg.speed;
    let mut t = 0.0f32;
    let mut steps_since_bounce = 1000u32;
    let mut crevice = 0u32;
    let mut bounces = 0u32;
    if let Some(r) = record.as_deref_mut() {
        r.push(p);
    }
    while t < cfg.max_time {
        v.z -= cfg.gravity * DT;
        let step = v * DT;
        let len = step.norm();
        let ray = Ray::new(Point3::from(p), step / len);
        if let Some(hit) =
            scene.mesh.cast_ray_and_get_normal(&nalgebra::Isometry3::identity(), &ray, len, true)
        {
            let mut n = hit.normal;
            if n.dot(&v) > 0.0 {
                n = -n; // face the incoming velocity
            }
            t += DT * hit.time_of_impact / len;
            p += step * (hit.time_of_impact / len) + n * 0.5;
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
                bounces += 1;
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
            let vn = v.dot(&n);
            let vt = v - n * vn;
            // tangential friction scales with impact steepness (grazing skips
            // barely rub the surface): the bBounceAngleAffectsFriction curve
            let steep = (vn.abs() / v.norm().max(1.0)).clamp(0.0, 1.0);
            v = vt * (1.0 - cfg.friction * steep) - n * vn * bounciness;
            bounces += 1;
            if trace {
                let owner = match hit.feature {
                    parry3d::shape::FeatureId::Face(i) => {
                        scene.owner_of(i % scene.mesh.indices().len().max(1) as u32)
                    }
                    _ => "?",
                };
                eprintln!(
                    "  bounce {bounces} t={t:.2} p=({:.0},{:.0},{:.0}) n=({:.2},{:.2},{:.2}) |v|={:.0} b={bounciness:.3} on {owner}",
                    p.x, p.y, p.z, n.x, n.y, n.z, v.norm()
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
                if lat < cfg.stop_speed * 2.0 || bounces > 40 {
                    return Some(Outcome { rest: p, time: t, bounces });
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
    const STEP_DOWN: f32 = 210.0;
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
    lit[ti as usize][tj as usize] && (target.z - z[ti as usize][tj as usize]).abs() <= 220.0
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
            tri_owner: vec![(0, "ground".into())],
            tri_color: vec![(0, [0.6, 0.6, 0.6])], tri_tex: vec![],
        };
        let cfg = Cfg::default();
        let dir = V3::new(std::f32::consts::FRAC_1_SQRT_2, 0.0, std::f32::consts::FRAC_1_SQRT_2);
        let o = fly(&scene, V3::new(0.0, 0.0, 150.0), dir, &cfg).expect("must stop");
        let vacuum = cfg.speed * cfg.speed / cfg.gravity; // 45 deg range from ground
        assert!(o.bounces >= 3, "skip phase must produce several bounces, got {}", o.bounces);
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
        assert!((launch_pitch(90.0, &cfg) - 90.0).abs() < 1e-4);
        assert!((launch_pitch(58.6, &cfg) - 61.39).abs() < 0.01);
        for aim in [-30.0f32, -5.0, 0.0, 10.0, 45.0, 58.6, 75.0, 89.0] {
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
            tri_owner: vec![(0, "ground".into())],
            tri_color: vec![(0, [0.6, 0.6, 0.6])], tri_tex: vec![],
        };
        let rest = V3::new(0.0, 0.0, 1.0);
        let behind = V3::new(400.0, 0.0, 1.0);
        let open = V3::new(-400.0, 0.0, 1.0);
        assert!(!fire_covers(&scene, rest, behind), "wall must block the spread");
        assert!(fire_covers(&scene, rest, open), "open side must be covered");
    }
}

