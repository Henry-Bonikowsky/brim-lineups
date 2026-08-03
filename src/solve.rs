//! Inverse solver: every navmesh stand point x analytic ballistic aim, refined
//! by simulation, ranked by time-to-land with an aim-forgiveness measure.

use crate::scene::{Scene, V3};
use crate::sim::{fly, Cfg};
use rayon::prelude::*;

pub struct Lineup {
    pub dist: f32, // stand-to-target range (lineups are long throws, not tosses)
    pub stand: V3,
    pub yaw: f32,        // player aim yaw (deg, UE convention: atan2(y, x))
    pub pitch: f32,      // player aim pitch (deg; launch pitch minus arc knob)
    pub time: f32,
    pub bounces: u32,
    pub err: f32,        // rest-to-target distance
    pub covered: bool,   // fire actually SPREADS to the target (box-aware)
    pub forgive: f32,    // fraction of +-0.75 deg jitters still within tol
    pub spread: f32,     // worst landing deviation across those jitters (fragility)
    pub pos_forgive: f32, // fraction of ~75u stand shifts (same aim) still covering
    pub backstop: bool,  // stand is flush against geometry (exactly reproducible)
    pub aim_ref: Option<(V3, f32)>, // crosshair reference: first geometry the aim ray hits
}

impl Lineup {
    /// 2 = easy position (stand roughly there, it works), 1 = anchored precise
    /// position (back into the wall), 0 = neither (unreliable to stand for)
    pub fn pos_grade(&self) -> u8 {
        if self.pos_forgive >= 0.75 {
            2
        } else if self.backstop {
            1
        } else {
            0
        }
    }
}

fn dir_from(yaw_deg: f32, pitch_deg: f32) -> V3 {
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    V3::new(cp * cy, cp * sy, sp)
}

/// Vacuum-ballistics launch angles for speed s, gravity g, horizontal dist d,
/// height diff h: tan(theta) = (s^2 +- sqrt(s^4 - g(g d^2 + 2 h s^2))) / (g d).
fn launch_angles(s: f32, g: f32, d: f32, h: f32) -> Vec<f32> {
    let disc = s.powi(4) - g * (g * d * d + 2.0 * h * s * s);
    if disc < 0.0 || d < 1.0 {
        return vec![];
    }
    let r = disc.sqrt();
    [(s * s - r) / (g * d), (s * s + r) / (g * d)]
        .iter()
        .map(|t| t.atan().to_degrees())
        .collect()
}

/// strict=true: full-map hunt (covered + hidden-from-site gates, one best per
/// stand, deduped, ranked by mid-range preference + time). strict=false
/// (paired mode, one locked stand): every distinct working ANGLE FAMILY from
/// that stand (pitch buckets), ranked by time.
pub fn solve(scene: &Scene, stands: &[V3], target: V3, tol: f32, min_dist: f32, strict: bool, cfg: &Cfg) -> Vec<Lineup> {
    let paired = !strict && stands.len() == 1;
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    let (n_none, n_far, n_near) = (AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0));
    let max_range = cfg.speed * cfg.speed / cfg.gravity * 1.05;
    let mut all: Vec<Lineup> = stands
        .par_iter()
        .flat_map_iter(|stand| {
            let origin = stand + V3::new(0.0, 0.0, cfg.eye_z);
            let delta = target - origin;
            let d = (delta.x * delta.x + delta.y * delta.y).sqrt();
            if d > max_range || d < min_dist {
                return vec![].into_iter();
            }
            let yaw0 = delta.y.atan2(delta.x).to_degrees();
            // paired mode: ONE stand is cheap, so scan the full angle space
            // exhaustively (finds arch-threads and skims the ballistic anchor
            // misses) and keep the best lineup per ~8 deg pitch family
            let mut best: Option<Lineup> = None;
            let mut best_miss: Option<Lineup> = None;
            let mut families: std::collections::HashMap<i32, Lineup> = Default::default();
            let sweeps: Vec<(f32, f32)> = if paired {
                let mut v = Vec::new();
                let mut pitch = -35.0f32;
                while pitch <= 85.0 {
                    let mut dy = -6.0f32;
                    while dy <= 6.0 {
                        v.push((yaw0 + dy, pitch));
                        dy += 1.0;
                    }
                    pitch += 1.25;
                }
                v
            } else {
                let mut v = Vec::new();
                for (ai, base) in launch_angles(cfg.speed, cfg.gravity, d, delta.z).into_iter().enumerate() {
                    // refine around the vacuum solution: geometry + bounces move
                    // the landing; the LOW arc gets a wider downward sweep for
                    // fast skim-bounce throws
                    let lo = if ai == 0 { -10 } else { -4 };
                    for dp in lo..=4 {
                        for dy in -2..=2 {
                            v.push((yaw0 + dy as f32, base + dp as f32));
                        }
                    }
                }
                v
            };
            {
                for (yaw, pitch) in sweeps {
                    {
                        if pitch <= -89.0 || pitch >= 89.0 {
                            continue;
                        }
                        let Some(o) = fly(scene, crate::sim::hand_origin(origin, yaw, cfg), dir_from(yaw, pitch), cfg) else {
                            n_none.fetch_add(1, Relaxed);
                            continue;
                        };
                        // success = the FIRE covers the clicked spot: rest within
                        // the 450u patch radius horizontally and within the
                        // fire's vertical reach (ZLayerTolerance 200, StepUp 110
                        // / StepDown 210 from the patch files)
                        let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
                        let dz = (o.rest.z - target.z).abs();
                        let err = if dz <= 220.0 { dxy } else { dxy + (dz - 220.0) * 3.0 };
                        // distance alone is not success: the fire must SPREAD
                        // to the click (a >110u box between rest and click
                        // blocks it even at 2m)
                        let covered = err < tol && crate::sim::fire_covers(scene, o.rest, target);
                        if covered { &n_near } else { &n_far }.fetch_add(1, Relaxed);
                        let cand = Lineup {
                            dist: d,
                            stand: *stand,
                            yaw,
                            pitch: pitch - cfg.arc_deg,
                            time: o.time,
                            bounces: o.bounces,
                            err,
                            covered,
                            forgive: 0.0,
                            spread: 0.0,
                            pos_forgive: 0.0,
                            backstop: false,
                            aim_ref: None,
                        };
                        if !covered {
                            if paired && best_miss.as_ref().is_none_or(|b| cand.err < b.err) {
                                best_miss = Some(cand);
                            }
                            continue;
                        }
                        if paired {
                            let key = (pitch / 8.0).round() as i32;
                            match families.get(&key) {
                                Some(cur) if cur.time <= cand.time => {}
                                _ => {
                                    families.insert(key, cand);
                                }
                            }
                        } else if best.as_ref().is_none_or(|b| cand.time < b.time) {
                            best = Some(cand);
                        }
                    }
                }
            }
            if paired {
                let mut out: Vec<Lineup> = families.into_values().collect();
                if out.is_empty() {
                    // no throw lands within tolerance: report the closest miss so
                    // the user sees WHY (err > tol labels it)
                    out.extend(best_miss);
                }
                for b in &mut out {
                    finish(scene, target, tol, cfg, origin, b);
                }
                out.sort_by(|a, b| a.time.total_cmp(&b.time));
                return out.into_iter();
            }
            if let Some(b) = &mut best {
                // a LINEUP is thrown from cover: nobody standing anywhere around
                // the site may have line of sight to the thrower. Sample defender
                // eyes on rings around the target; ANY clear sightline kills it.
                use parry3d::query::{Ray, RayCast};
                let mut site_eyes = vec![target + V3::new(0.0, 0.0, 160.0)];
                for (ring, n) in [(350.0f32, 6usize), (650.0, 8)] {
                    for k in 0..n {
                        let a = k as f32 / n as f32 * std::f32::consts::TAU;
                        site_eyes.push(target + V3::new(a.cos() * ring, a.sin() * ring, 160.0));
                    }
                }
                let exposed = strict && site_eyes.iter().any(|se| {
                    let v = se - origin;
                    let d = v.norm();
                    let sight = Ray::new(nalgebra::Point3::from(origin), v / d);
                    scene
                        .mesh
                        .cast_ray(&nalgebra::Isometry3::identity(), &sight, d - 60.0, true)
                        .is_none()
                });
                if exposed {
                    return vec![].into_iter();
                }
                finish(scene, target, tol, cfg, origin, b);
            }
            best.into_iter().collect::<Vec<_>>().into_iter()
        })
        .collect();

    eprintln!(
        "flights: {} never stopped, {} landed far, {} within tol",
        n_none.load(Relaxed),
        n_far.load(Relaxed),
        n_near.load(Relaxed)
    );
    // a lineup the solver itself rates near-zero forgiveness is untrustworthy
    // (tiny aim error cascades, e.g. clipping a sloped roof); prefer sturdy ones
    let sturdy = |v: &mut Vec<Lineup>| {
        if v.iter().any(|l| l.forgive >= 0.25) {
            v.retain(|l| l.forgive >= 0.25);
        }
        // position rule: keep only easy-position or backstopped stands when any exist
        if v.iter().any(|l| l.pos_grade() > 0) {
            v.retain(|l| l.pos_grade() > 0);
        }
    };
    if paired {
        // angle families from the locked stand, fastest first; easy positions first
        sturdy(&mut all);
        all.sort_by(|a, b| (b.pos_grade(), a.time).partial_cmp(&(a.pos_grade(), b.time)).unwrap());
        return all;
    }
    // dedup: one lineup per 200u XY cell, keep the fastest
    let mut by_cell: std::collections::HashMap<(i64, i64), Lineup> = Default::default();
    for l in all.drain(..) {
        let key = ((l.stand.x / 200.0) as i64, (l.stand.y / 200.0) as i64);
        match by_cell.get(&key) {
            Some(cur) if cur.time <= l.time => {}
            _ => {
                by_cell.insert(key, l);
            }
        }
    }
    let mut out: Vec<Lineup> = by_cell.into_values().collect();
    sturdy(&mut out);
    // rank: easy positions first, then mid-range preference (~3000u) + speed
    let key = |l: &Lineup| {
        (l.dist - 3000.0).abs() / 1500.0 + l.time * 0.35 - l.pos_grade() as f32 * 10.0
    };
    out.sort_by(|a, b| key(a).total_cmp(&key(b)));
    out
}

/// Post-processing shared by both modes: crosshair reference point + aim
/// forgiveness for a confirmed lineup.
fn finish(scene: &Scene, target: V3, tol: f32, cfg: &Cfg, origin: V3, b: &mut Lineup) {
    use parry3d::query::{Ray, RayCast};
    let aim_dir = dir_from(b.yaw, b.pitch);
    let ray = Ray::new(nalgebra::Point3::from(origin), aim_dir);
    b.aim_ref = scene
        .mesh
        .cast_ray(&nalgebra::Isometry3::identity(), &ray, 5.0e4, true)
        .map(|t| (origin + aim_dir * t, t));
    let mut ok = 0;
    let mut worst = 0.0f32;
    for (jy, jp) in
        [(0.75, 0.0), (-0.75, 0.0), (0.0, 0.75), (0.0, -0.75), (0.75, 0.75), (-0.75, 0.75), (0.75, -0.75), (-0.75, -0.75)]
    {
        let launch_pitch = b.pitch + cfg.arc_deg + jp;
        if let Some(o) = fly(scene, crate::sim::hand_origin(origin, b.yaw + jy, cfg), dir_from(b.yaw + jy, launch_pitch), cfg) {
            let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
            let dz = (o.rest.z - target.z).abs();
            let dev = if dz <= 220.0 { dxy } else { dxy + (dz - 220.0) * 3.0 };
            worst = worst.max(dev);
            if dev < tol && crate::sim::fire_covers(scene, o.rest, target) {
                ok += 1;
            }
        } else {
            worst = worst.max(9999.0);
        }
    }
    b.forgive = ok as f32 / 8.0;
    b.spread = worst;

    // POSITION forgiveness: shift the stand ~75u with the SAME aim; if the fire
    // still covers, exact positioning does not matter ("easy position")
    let covers = |o: &crate::sim::Outcome| {
        let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
        let dz = (o.rest.z - target.z).abs();
        (if dz <= 220.0 { dxy } else { dxy + (dz - 220.0) * 3.0 }) < tol
            && crate::sim::fire_covers(scene, o.rest, target)
    };
    let launch = dir_from(b.yaw, b.pitch + cfg.arc_deg);
    let mut pok = 0;
    for (ox, oy) in [(75.0f32, 0.0), (-75.0, 0.0), (0.0, 75.0), (0.0, -75.0), (55.0, 55.0), (-55.0, 55.0), (55.0, -55.0), (-55.0, -55.0)] {
        let o2 = crate::sim::hand_origin(origin, b.yaw, cfg) + V3::new(ox, oy, 0.0);
        if fly(scene, o2, launch, cfg).as_ref().map(&covers).unwrap_or(false) {
            pok += 1;
        }
    }
    b.pos_forgive = pok as f32 / 8.0;

    // backstop: waist-height geometry within 70u in any horizontal direction
    // means the spot is exactly reproducible by pressing against it
    let waist = b.stand + V3::new(0.0, 0.0, 90.0);
    b.backstop = (0..8).any(|k| {
        let a = k as f32 / 8.0 * std::f32::consts::TAU;
        let ray = Ray::new(nalgebra::Point3::from(waist), V3::new(a.cos(), a.sin(), 0.0));
        scene.mesh.cast_ray(&nalgebra::Isometry3::identity(), &ray, 70.0, true).is_some()
    });
}
