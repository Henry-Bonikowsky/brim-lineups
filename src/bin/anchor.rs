//! A/B harness: run a recorded in-game anchor under BOTH launch models and
//! print where each one puts the molly, so the anchor picks the model instead
//! of the model picking the constants.
//!
//! Usage:
//!   anchor <mapDumpDir> --stand X,Y --aim yaw,pitch [--speeds 2900,2930]
//!                       [--expect-z lo,hi]
//!
//! Prints, per (model, speed): launch pitch, launch speed, first-impact point
//! and the mesh it hit, plus flight time and rest. `--expect-z` marks a run
//! HIT when the first impact lands inside that z band (the anchor's recorded
//! in-game strike height).

use brim_lineups::scene::{self, V3};
use brim_lineups::sim::{self, Cfg};

fn get(args: &[String], k: &str) -> Option<String> {
    args.iter().position(|a| a == k).and_then(|i| args.get(i + 1)).cloned()
}
fn nums(s: &str) -> Vec<f32> {
    s.split(',').map(|x| x.trim().parse().expect("number")).collect()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let dump = args.first().expect("usage: anchor <mapDumpDir> --stand X,Y --aim yaw,pitch");
    let stand_xy = nums(&get(&args, "--stand").expect("--stand X,Y"));
    let aim = nums(&get(&args, "--aim").expect("--aim yaw,pitch"));
    let speeds = get(&args, "--speeds").map(|s| nums(&s)).unwrap_or_else(|| vec![2900.0, 2930.0]);
    let band = get(&args, "--expect-z").map(|s| nums(&s));

    let scene = scene::load(std::path::Path::new(dump));
    let ntris = scene.mesh.indices().len() as u32;

    // nearest recorded stand to the requested XY: gives the true ground z
    let stand = scene
        .stands
        .iter()
        .copied()
        .min_by(|a, b| {
            let d = |p: &V3| (p.x - stand_xy[0]).powi(2) + (p.y - stand_xy[1]).powi(2);
            d(a).partial_cmp(&d(b)).unwrap()
        })
        .expect("scene has no stands");
    let off = ((stand.x - stand_xy[0]).powi(2) + (stand.y - stand_xy[1]).powi(2)).sqrt();
    println!(
        "stand ({:.0},{:.0},{:.0})  [{:.0}u from requested]   aim yaw={} pitch={}",
        stand.x, stand.y, stand.z, off, aim[0], aim[1]
    );
    if let Some(b) = &band {
        println!("expecting first impact in z {:.0}..{:.0}\n", b[0], b[1]);
    }

    println!(
        "{:<8} {:>6} {:>11} {:>11} | {:>26} {:>7} | {:>5} {:>6}  {}",
        "model", "speed", "launch pitch", "launch spd", "first impact", "impact z", "t", "bounces", "hit mesh"
    );
    println!("{}", "-".repeat(120));

    for vector_launch in [false, true] {
        for &speed in &speeds {
            let cfg = Cfg { speed, vector_launch, ..Cfg::default() };
            let lp = sim::launch_pitch(aim[1], &cfg);
            let (sy, cy) = aim[0].to_radians().sin_cos();
            let (sp, cp) = lp.to_radians().sin_cos();
            let dir = V3::new(cp * cy, cp * sy, sp);
            let ls = sim::launch_speed(dir, &cfg);
            let eye = V3::new(stand.x, stand.y, stand.z + cfg.eye_z);
            let o = sim::hand_origin(eye, aim[0], &cfg);

            match sim::fly_path_marks(&scene, o, dir, &cfg) {
                Some((r, traj, first, _)) => {
                    let p = traj.get(first).copied().unwrap_or(r.rest);
                    // name the mesh at the impact by probing straight along the
                    // incoming step direction
                    let owner = traj
                        .get(first.saturating_sub(1))
                        .map(|prev| {
                            let d = (p - *prev).normalize();
                            let ray = parry3d::query::Ray::new(nalgebra::Point3::from(*prev), d);
                            use parry3d::query::RayCast;
                            scene
                                .mesh
                                .cast_ray_and_get_normal(&nalgebra::Isometry3::identity(), &ray, 1.0e5, true)
                                .map(|h| match h.feature {
                                    parry3d::shape::FeatureId::Face(f) => {
                                        scene.owner_of(f % ntris.max(1)).to_string()
                                    }
                                    _ => "?".into(),
                                })
                                .unwrap_or_else(|| "?".into())
                        })
                        .unwrap_or_else(|| "?".into());
                    let mark = match &band {
                        Some(b) if p.z >= b[0] && p.z <= b[1] => "  <== HIT",
                        Some(_) => "",
                        None => "",
                    };
                    println!(
                        "{:<8} {:>6.0} {:>11.2} {:>11.0} | {:>26} {:>8.0} | {:>5.2} {:>6}  {}{}",
                        if vector_launch { "vector" } else { "legacy" },
                        speed,
                        lp,
                        ls,
                        format!("({:.0},{:.0},{:.0})", p.x, p.y, p.z),
                        p.z,
                        r.time,
                        r.bounces,
                        owner,
                        mark
                    );
                }
                None => println!(
                    "{:<8} {:>6.0} {:>11.2} {:>11.0} | never stopped",
                    if vector_launch { "vector" } else { "legacy" },
                    speed,
                    lp,
                    ls
                ),
            }
        }
    }
}
