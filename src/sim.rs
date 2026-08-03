//! Molly flight: constants extracted from the VALORANT 13.02 files
//! (see C:\dev\research\brim-molly-physics.md). Native-only unknowns are knobs.

use crate::scene::{Scene, V3};
use nalgebra::Point3;
use parry3d::query::{Ray, RayCast};

#[derive(Clone, Copy)]
pub struct Cfg {
    pub speed: f32,       // ProjectileSpeed, file value 2900
    pub gravity: f32,     // |gravity| = 2500 * 0.45 = 1125
    pub bounciness: f32,  // 0.35
    pub friction: f32,    // 0.65
    pub stop_speed: f32,  // 200
    pub eye_z: f32,       // camera height: CapsuleHalfHeight 98 + BaseEyeHeight 77 (BasePawn CDO)
    pub arc_deg: f32,     // knob: UpwardArc 8 (aim pitch offset, native combine)
    pub max_time: f32,
}

impl Default for Cfg {
    fn default() -> Self {
        Cfg {
            speed: 2900.0,
            gravity: 1125.0,
            bounciness: 0.35,
            friction: 0.65,
            stop_speed: 200.0,
            eye_z: 175.0,
            arc_deg: 8.0,
            max_time: 8.0,
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

/// Like fly, but also records the position at every integration step.
pub fn fly_path(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg) -> Option<(Outcome, Vec<V3>)> {
    let mut path = Vec::new();
    fly_impl(scene, origin, dir, cfg, false, Some(&mut path)).map(|o| (o, path))
}

fn fly_impl(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg, trace: bool, mut record: Option<&mut Vec<V3>>) -> Option<Outcome> {
    const DT: f32 = 1.0 / 120.0;
    let mut p = origin;
    let mut v = dir * cfg.speed;
    let mut t = 0.0f32;
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
            // per-bounce deadening decoded from Projectile_BaseGrenade bytecode:
            // bounciness = default * lerp(0.5..1.0, degrees-from-down / 90).
            // NOT compounding: the native DefaultBounciness field exists to
            // reset the value before each bounce's angle adjustment.
            let deg = (-v.z / v.norm()).clamp(-1.0, 1.0).acos().to_degrees();
            let bounciness = cfg.bounciness * (0.5 + 0.5 * (deg / 90.0).clamp(0.0, 1.0));
            let vn = v.dot(&n);
            let vt = v - n * vn;
            // tangential friction scales with impact steepness (grazing skips
            // barely rub the surface; the native bBounceAngleAffectsFriction
            // behavior the real molly visibly has: it hops and skips a lot)
            let steep = (vn.abs() / v.norm().max(1.0)).clamp(0.0, 1.0);
            v = vt * (1.0 - cfg.friction * steep) - n * vn * bounciness;
            bounces += 1;
            if trace {
                eprintln!(
                    "  bounce {bounces} t={t:.2} p=({:.0},{:.0},{:.0}) n=({:.2},{:.2},{:.2}) |v|={:.0} b={bounciness:.3}",
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
                // skitter: minimum hop + rolling loss on the slide (tuned so the
                // in-game-validated wall throw still rests on its spot)
                let vn2 = v.dot(&n);
                v = (v - n * vn2) * 0.65 + n * (cfg.stop_speed * 0.7);
            }
        } else {
            p += step;
            t += DT;
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
}
