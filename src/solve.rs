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
    pub forgive: f32,    // fraction of +-0.75 deg jitters still within tol
    pub aim_ref: Option<(V3, f32)>, // crosshair reference: first geometry the aim ray hits
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

pub fn solve(scene: &Scene, target: V3, tol: f32, min_dist: f32, strict: bool, cfg: &Cfg) -> Vec<Lineup> {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    let (n_none, n_far, n_near) = (AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0));
    let max_range = cfg.speed * cfg.speed / cfg.gravity * 1.05;
    let mut all: Vec<Lineup> = scene
        .stands
        .par_iter()
        .flat_map_iter(|stand| {
            let origin = stand + V3::new(0.0, 0.0, cfg.eye_z);
            let delta = target - origin;
            let d = (delta.x * delta.x + delta.y * delta.y).sqrt();
            if d > max_range || d < min_dist {
                return vec![].into_iter();
            }
            let yaw0 = delta.y.atan2(delta.x).to_degrees();
            let mut best: Option<Lineup> = None;
            for (ai, base) in launch_angles(cfg.speed, cfg.gravity, d, delta.z).into_iter().enumerate() {
                // refine around the vacuum solution: geometry + bounces move the
                // landing spot, the sim is the truth. The LOW arc gets a wider
                // downward sweep to catch fast skim-bounce throws that shed
                // speed on the ground short of the target.
                let lo = if ai == 0 { -10 } else { -4 };
                for dp in lo..=4 {
                    for dy in -2..=2 {
                        let (yaw, pitch) = (yaw0 + dy as f32, base + dp as f32);
                        if pitch <= -89.0 || pitch >= 89.0 {
                            continue;
                        }
                        let Some(o) = fly(scene, origin, dir_from(yaw, pitch), cfg) else {
                            n_none.fetch_add(1, Relaxed);
                            continue;
                        };
                        let err = (o.rest - target).norm();
                        if err < tol { &n_near } else { &n_far }.fetch_add(1, Relaxed);
                        if err < tol && best.as_ref().is_none_or(|b| o.time < b.time) {
                            best = Some(Lineup {
                                dist: d,
                                stand: *stand,
                                yaw,
                                pitch: pitch - cfg.arc_deg,
                                time: o.time,
                                bounces: o.bounces,
                                err,
                                forgive: 0.0,
                                aim_ref: None,
                            });
                        }
                    }
                }
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
                let aim_dir = dir_from(b.yaw, b.pitch);
                let ray = Ray::new(nalgebra::Point3::from(origin), aim_dir);
                b.aim_ref = scene
                    .mesh
                    .cast_ray(&nalgebra::Isometry3::identity(), &ray, 5.0e4, true)
                    .map(|t| (origin + aim_dir * t, t));
                // forgiveness: 8 jitters of +-0.75 deg around the found aim
                let mut ok = 0;
                for (jy, jp) in
                    [(0.75, 0.0), (-0.75, 0.0), (0.0, 0.75), (0.0, -0.75), (0.75, 0.75), (-0.75, 0.75), (0.75, -0.75), (-0.75, -0.75)]
                {
                    let launch_pitch = b.pitch + cfg.arc_deg + jp;
                    if let Some(o) = fly(scene, origin, dir_from(b.yaw + jy, launch_pitch), cfg) {
                        if (o.rest - target).norm() < tol {
                            ok += 1;
                        }
                    }
                }
                b.forgive = ok as f32 / 8.0;
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
    out.sort_by(|a, b| a.time.total_cmp(&b.time));
    out
}
