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

mod render;
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
    let target_raw = get("--target").expect("--target X,Y,Z (or X,Y,auto) required");
    let tparts: Vec<&str> = target_raw.split(',').map(|x| x.trim()).collect();
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
    let target: V3 = {
        let x: f32 = tparts[0].parse().expect("target x");
        let y: f32 = tparts[1].parse().expect("target y");
        if tparts[2] == "auto" {
            // ground floor at (x, y): walk down through stacked hits, keep the lowest
            use parry3d::query::{Ray, RayCast};
            let mut z_top = 6000.0f32;
            let mut floor = None;
            for _ in 0..8 {
                let ray = Ray::new(nalgebra::Point3::new(x, y, z_top), V3::new(0.0, 0.0, -1.0));
                match scene.mesh.cast_ray(&nalgebra::Isometry3::identity(), &ray, z_top - scene.min_z + 100.0, true) {
                    Some(t) if t > 1.0 => {
                        z_top -= t + 5.0;
                        floor = Some(z_top + 5.0);
                    }
                    _ => break,
                }
            }
            let z = floor.expect("no ground found at target x,y");
            eprintln!("target z auto-resolved to {z:.0}");
            V3::new(x, y, z)
        } else {
            V3::new(x, y, tparts[2].parse().expect("target z"))
        }
    };
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
    if let Some(f) = get("--fan") {
        // --fan X,Y,Z,yaw0,yaw1,pitch0,pitch1,step : sweep throws from one spot,
        // print landings (for reconstructing a known lineup's aim)
        let c: Vec<f32> = f.split(',').map(|x| x.trim().parse().unwrap()).collect();
        let o = V3::new(c[0], c[1], c[2] + cfg.eye_z);
        let mut yaw = c[3];
        while yaw <= c[4] {
            let mut pitch = c[5];
            while pitch <= c[6] {
                let (sy, cy) = yaw.to_radians().sin_cos();
                let (sp, cp) = pitch.to_radians().sin_cos();
                if let Some(r) = sim::fly(&scene, o, V3::new(cp * cy, cp * sy, sp), &cfg) {
                    println!(
                        "yaw={yaw:>7.1} pitch={pitch:>6.1} -> ({:>6.0},{:>6.0},{:>5.0}) t={:.2}s b={}",
                        r.rest.x, r.rest.y, r.rest.z, r.time, r.bounces
                    );
                }
                pitch += c[7];
            }
            yaw += c[7];
        }
        return;
    }
    let t0 = std::time::Instant::now();
    let min_dist: f32 = get("--min-dist").map(|s| s.parse().unwrap()).unwrap_or(1800.0);
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

    // synthetic first-person screenshots for the top lineups: match your screen
    // to the image and the aim is reproduced (no angle HUD needed in game)
    if let Some(prefix) = get("--render") {
        // renders show what the player SEES: include decorative/no-collision meshes
        let scene = scene::load_visual(&dir);
        for (i, l) in lineups.iter().take(get("--render-top").map(|s| s.parse().unwrap()).unwrap_or(3)).enumerate() {
            let eye = l.stand + V3::new(0.0, 0.0, cfg.eye_z);
            let path = format!("{prefix}_r{}.bmp", i + 1);
            render::render(&scene, eye, l.yaw, l.pitch, &path);
            // stand-locating view: straight down from just overhead; the green
            // cross is where your feet go, image-up is your throw direction
            let spath = format!("{prefix}_s{}.bmp", i + 1);
            render::render_grid(&scene, l.stand + V3::new(0.0, 0.0, 350.0), l.yaw, -89.0, &spath);
            // wide context: drone shot from behind-above the stand along the throw
            let (syw, cyw) = l.yaw.to_radians().sin_cos();
            let wide_eye = l.stand + V3::new(-cyw * 750.0, -syw * 750.0, 1000.0);
            let wpath = format!("{prefix}_w{}.bmp", i + 1);
            render::render_marked(&scene, wide_eye, l.yaw, -50.0, &wpath, l.stand + V3::new(0.0, 0.0, 40.0));
            eprintln!("rendered {path} + stand + wide");
        }
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
