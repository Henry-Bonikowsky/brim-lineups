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
    pub eye_z: f32,       // knob: launch origin height over stand point (native)
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
            eye_z: 150.0,
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
    fly_impl(scene, origin, dir, cfg, false)
}

pub fn fly_traced(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg) -> Option<Outcome> {
    fly_impl(scene, origin, dir, cfg, true)
}

fn fly_impl(scene: &Scene, origin: V3, dir: V3, cfg: &Cfg, trace: bool) -> Option<Outcome> {
    const DT: f32 = 1.0 / 120.0;
    let mut p = origin;
    let mut v = dir * cfg.speed;
    let mut bounciness = cfg.bounciness;
    let mut t = 0.0f32;
    let mut bounces = 0u32;
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
            // bounciness *= lerp(0.5..1.0, degrees-from-straight-down / 90)
            let deg = (-v.z / v.norm()).clamp(-1.0, 1.0).acos().to_degrees();
            bounciness *= 0.5 + 0.5 * (deg / 90.0).clamp(0.0, 1.0);
            let vn = v.dot(&n);
            let vt = v - n * vn;
            v = vt * (1.0 - cfg.friction) - n * vn * bounciness;
            bounces += 1;
            if trace {
                eprintln!(
                    "  bounce {bounces} t={t:.2} p=({:.0},{:.0},{:.0}) n=({:.2},{:.2},{:.2}) |v|={:.0} b={bounciness:.3}",
                    p.x, p.y, p.z, n.x, n.y, n.z, v.norm()
                );
            }
            if v.norm() < cfg.stop_speed {
                return Some(Outcome { rest: p, time: t, bounces });
            }
        } else {
            p += step;
            t += DT;
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
        assert!(o.bounces >= 1, "must bounce at least once");
        assert!(
            (o.rest.x - vacuum).abs() < vacuum * 0.15,
            "rest x {} vs vacuum range {vacuum}",
            o.rest.x
        );
        assert!(o.rest.z.abs() < 50.0, "rest on the ground, got z={}", o.rest.z);
    }
}
