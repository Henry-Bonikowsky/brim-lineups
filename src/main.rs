//! brim-lineups: compute every physically possible Brimstone molly lineup to a
//! target spot on a dumped VALORANT map, ranked by time-to-land.
//!
//! Usage:
//!   brim-lineups <mapDumpDir> --target X,Y,Z [--tol 150] [--top 15]
//!                [--eye 150] [--arc 8] [--speed 2900]
//!
//! Map dumps come from ValoBoard/tools/valo_dump (`map` mode); physics constants
//! from the game files (C:\dev\research\brim-molly-physics.md). --eye and --arc
//! are the two native-unknown calibration knobs.

mod scene;
mod sim;
mod solve;

use scene::V3;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: brim-lineups <mapDumpDir> --target X,Y,Z [--tol u] [--top n] [--eye u] [--arc deg] [--speed u/s]");
        std::process::exit(1);
    }
    let dir = std::path::PathBuf::from(&args[1]);
    let get = |name: &str| -> Option<String> {
        args.iter().position(|a| a == name).and_then(|i| args.get(i + 1).cloned())
    };
    let target: V3 = {
        let t = get("--target").expect("--target X,Y,Z required");
        let p: Vec<f32> = t.split(',').map(|x| x.trim().parse().expect("target coord")).collect();
        V3::new(p[0], p[1], p[2])
    };
    let tol: f32 = get("--tol").map(|s| s.parse().unwrap()).unwrap_or(150.0);
    let top: usize = get("--top").map(|s| s.parse().unwrap()).unwrap_or(15);
    let mut cfg = sim::Cfg::default();
    if let Some(v) = get("--eye") {
        cfg.eye_z = v.parse().unwrap();
    }
    if let Some(v) = get("--arc") {
        cfg.arc_deg = v.parse().unwrap();
    }
    if let Some(v) = get("--speed") {
        cfg.speed = v.parse().unwrap();
    }

    let scene = scene::load(&dir);
    if let Some(p) = get("--probe") {
        let c: Vec<f32> = p.split(',').map(|x| x.trim().parse().unwrap()).collect();
        use parry3d::query::{Ray, RayCast};
        for (name, o, d) in [
            ("down from +2000", V3::new(c[0], c[1], c[2] + 2000.0), V3::new(0.0, 0.0, -1.0)),
            ("up from point", V3::new(c[0], c[1], c[2]), V3::new(0.0, 0.0, 1.0)),
            ("north", V3::new(c[0], c[1], c[2] + 150.0), V3::new(1.0, 0.0, 0.0)),
        ] {
            let ray = Ray::new(nalgebra::Point3::from(o), d);
            match scene.mesh.cast_ray_and_get_normal(&nalgebra::Isometry3::identity(), &ray, 1.0e5, true) {
                Some(h) => {
                    let owner = match h.feature {
                        parry3d::shape::FeatureId::Face(f) => scene.owner_of(f),
                        _ => "?",
                    };
                    println!("{name}: hit at dist {:.1} -> z={:.1} [{owner}]", h.time_of_impact, o.z + d.z * h.time_of_impact);
                }
                None => println!("{name}: no hit"),
            }
        }
        return;
    }
    if let Some(t) = get("--throw") {
        let c: Vec<f32> = t.split(',').map(|x| x.trim().parse().unwrap()).collect();
        let (sy, cy) = c[3].to_radians().sin_cos();
        let (sp, cp) = c[4].to_radians().sin_cos();
        let dir = V3::new(cp * cy, cp * sy, sp);
        let o = V3::new(c[0], c[1], c[2]);
        eprintln!("throw from ({:.0},{:.0},{:.0}) yaw={} pitch={}", o.x, o.y, o.z, c[3], c[4]);
        match sim::fly_traced(&scene, o, dir, &cfg) {
            Some(r) => eprintln!("rest ({:.0},{:.0},{:.0}) t={:.2} bounces={}", r.rest.x, r.rest.y, r.rest.z, r.time, r.bounces),
            None => eprintln!("never stopped"),
        }
        return;
    }
    let t0 = std::time::Instant::now();
    let min_dist: f32 = get("--min-dist").map(|s| s.parse().unwrap()).unwrap_or(1000.0);
    let lineups = solve::solve(&scene, target, tol, min_dist, &cfg);
    eprintln!("solved in {:.1?}: {} distinct lineups within {tol}u", t0.elapsed(), lineups.len());

    println!(
        "{:>28} {:>6} {:>7} {:>7} {:>6} {:>3} {:>6} {:>7}  {}",
        "stand (x, y, z)", "range", "yaw", "pitch", "time", "bnc", "err", "forgive", "crosshair on (x, y, z @ dist)"
    );
    for l in lineups.iter().take(top) {
        let aim = match &l.aim_ref {
            Some((p, d)) => format!("({:.0}, {:.0}, {:.0} @ {:.0}u)", p.x, p.y, p.z, d),
            None => "(open sky)".into(),
        };
        println!(
            "{:>28} {:>5.0}u {:>7.1} {:>7.1} {:>5.2}s {:>3} {:>5.0}u {:>6.0}%  {aim}",
            format!("({:.0}, {:.0}, {:.0})", l.stand.x, l.stand.y, l.stand.z),
            l.dist,
            l.yaw,
            l.pitch,
            l.time,
            l.bounces,
            l.err,
            l.forgive * 100.0
        );
    }

    // machine-readable output next to nothing in particular: cwd
    let json: Vec<String> = lineups
        .iter()
        .map(|l| {
            format!(
                "{{\"stand\":[{:.1},{:.1},{:.1}],\"range\":{:.0},\"yaw\":{:.2},\"pitch\":{:.2},\"time\":{:.3},\"bounces\":{},\"err\":{:.1},\"forgive\":{:.2},\"aim_ref\":{}}}",
                l.stand.x, l.stand.y, l.stand.z, l.dist, l.yaw, l.pitch, l.time, l.bounces, l.err, l.forgive,
                match &l.aim_ref {
                    Some((p, d)) => format!("[{:.1},{:.1},{:.1},{:.0}]", p.x, p.y, p.z, d),
                    None => "null".into(),
                }
            )
        })
        .collect();
    std::fs::write("lineups.json", format!("[{}]", json.join(","))).expect("write lineups.json");
    eprintln!("wrote lineups.json ({} entries)", lineups.len());
}
