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

use brim_lineups::{pack, render, scene, serve, sim, solve};
use scene::V3;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(|a| a == "serve").unwrap_or(false) {
        let root = args.get(2).cloned().unwrap_or_else(|| r"C:\dev\active\ValoBoard\third_party\valorant_dump".into());
        let port: u16 = args.get(3).and_then(|p| p.parse().ok()).unwrap_or(8777);
        serve::serve(&root, "cards", port);
        return;
    }
    if args.get(1).map(|a| a == "pack").unwrap_or(false) {
        // pack <dumpDir> <out.blp.gz>: filtered binary map bundle for the web build
        pack::pack(std::path::Path::new(&args[2]), std::path::Path::new(&args[3]));
        return;
    }
    if args.get(1).map(|a| a == "browserefs").unwrap_or(false) {
        // DEBUG (review loop): browserefs <pack.gz> <tx> <ty> [prefix] [row]
        // site browse flow; prints top-10 refs, renders row's aim+wide
        use std::io::Read as _;
        let mut bytes = Vec::new();
        flate2::read::GzDecoder::new(std::fs::File::open(&args[2]).expect("pack"))
            .read_to_end(&mut bytes)
            .expect("gunzip");
        let (cs, vs) = scene::load_pack(&bytes);
        let cfg = sim::Cfg::default();
        let (tx, ty): (f32, f32) = (args[3].parse().unwrap(), args[4].parse().unwrap());
        let target = V3::new(tx, ty, cs.ground_z(tx, ty).expect("ground"));
        let lineups = solve::solve(&cs, Some(&vs), &cs.stands.clone(), target, 1000.0, 1800.0, true, true, &cfg);
        for (k, l) in lineups.iter().take(10).enumerate() {
            println!(
                "row {:2} t {:4.1}s err {:5.0} stand ({:.0},{:.0},{:.1}) yaw {:.3} pitch {:.3} ref: {:?}",
                k + 1, l.time, l.err, l.stand.x, l.stand.y, l.stand.z, l.yaw, l.pitch,
                l.ui_ref.map(|(n, d, g, _, _)| (n, g, d.round()))
            );
        }
        if let Some(prefix) = args.get(5) {
            let row: usize = args.get(6).and_then(|r| r.parse().ok()).unwrap_or(1);
            let l = &lineups[row - 1];
            let eye = l.stand + V3::new(0.0, 0.0, cfg.eye_z);
            // native HUD resolution: pixel-exact reference review
            let mut aim = render::render_pov_bytes(&vs, eye, l.yaw, l.pitch, 2000, 1250);
            if let Some((_, _, _, fx, fy)) = l.ui_ref {
                render::stamp_ring(&mut aim, fx, fy);
            }
            std::fs::write(format!("{prefix}_aim.bmp"), &aim).unwrap();
            let (we, wy, wp) = render::wide_cam(&vs, l.stand, l.yaw);
            let wide = render::render_marked_bytes(&vs, we, wy, wp, l.stand + V3::new(0.0, 0.0, 40.0));
            std::fs::write(format!("{prefix}_wide.bmp"), &wide).unwrap();
        }
        return;
    }
    if args.get(1).map(|a| a == "packrender").unwrap_or(false) {
        // packrender <pack.gz> <tx> <ty> <outPrefix>: browse-solve + render row 1
        // from the PACKED scenes (native run of the exact wasm code path)
        use std::io::Read as _;
        let mut bytes = Vec::new();
        flate2::read::GzDecoder::new(std::fs::File::open(&args[2]).expect("pack"))
            .read_to_end(&mut bytes)
            .expect("gunzip");
        let (cs, vs) = scene::load_pack(&bytes);
        let (tx, ty): (f32, f32) = (args[3].parse().unwrap(), args[4].parse().unwrap());
        let cfg = sim::Cfg::default();
        let tz = cs.ground_z(tx, ty).expect("ground");
        let target = V3::new(tx, ty, tz);
        let lineups = solve::solve(&cs, Some(&vs), &cs.stands.clone(), target, 1000.0, 1800.0, true, true, &cfg);
        eprintln!("{} lineups", lineups.len());
        let l = &lineups[0];
        eprintln!("row1: stand ({:.0},{:.0},{:.0}) yaw {:.1} pitch {:.1} rest ({:.0},{:.0},{:.0})",
            l.stand.x, l.stand.y, l.stand.z, l.yaw, l.pitch, l.rest.x, l.rest.y, l.rest.z);
        let eye = l.stand + V3::new(0.0, 0.0, cfg.eye_z);
        render::render(&vs, eye, l.yaw, l.pitch, &format!("{}_r.bmp", args[5]));
        render::render_grid(&vs, l.stand + V3::new(0.0, 0.0, 350.0), l.yaw, -89.0, &format!("{}_s.bmp", args[5]));
        let (we, wy, wp) = render::wide_cam(&vs, l.stand, l.yaw);
        render::render_marked(&vs, we, wy, wp, &format!("{}_w.bmp", args[5]), l.stand + V3::new(0.0, 0.0, 40.0));
        return;
    }
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
    // default tolerance = the molotov fire patch radius from the game files
    let tol: f32 = get("--tol").map(|s| s.parse().unwrap()).unwrap_or(450.0);
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
    if let Some(v) = get("--gravity") {
        cfg.gravity = v.parse().unwrap();
    }
    if let Some(v) = get("--hand") {
        cfg.hand_left = v.parse().unwrap();
    }

    let mut scene = scene::load(&dir);
    let target: V3 = {
        let x: f32 = tparts[0].parse().expect("target x");
        let y: f32 = tparts[1].parse().expect("target y");
        if tparts[2] == "auto" {
            // same navmesh-snapped ground resolution the server uses
            let z = scene.ground_z(x, y).expect("no ground found at target x,y");
            eprintln!("target z auto-resolved to {z:.0}");
            V3::new(x, y, z)
        } else {
            V3::new(x, y, tparts[2].parse().expect("target z"))
        }
    };
    // --stand X,Y[,auto]: evaluate ONLY this standing spot (paired lineup mode).
    // Z resolves to the ground under X,Y; min-dist is waived.
    let mut min_dist_override: Option<f32> = None;
    if let Some(sarg) = get("--stand") {
        let sp: Vec<&str> = sarg.split(',').map(|x| x.trim()).collect();
        let sx: f32 = sp[0].parse().expect("stand x");
        let sy: f32 = sp[1].parse().expect("stand y");
        use parry3d::query::{Ray, RayCast};
        let sz = if sp.len() > 2 && sp[2] != "auto" {
            sp[2].parse().expect("stand z")
        } else {
            let mut z_top = 6000.0f32;
            let mut floor = None;
            for _ in 0..8 {
                let ray = Ray::new(nalgebra::Point3::new(sx, sy, z_top), V3::new(0.0, 0.0, -1.0));
                match scene.mesh.cast_ray(&nalgebra::Isometry3::identity(), &ray, z_top - scene.min_z + 100.0, true) {
                    Some(t) if t > 1.0 => {
                        z_top -= t + 5.0;
                        floor = Some(z_top + 5.0);
                    }
                    _ => break,
                }
            }
            floor.expect("no ground under stand x,y")
        };
        eprintln!("paired mode: stand locked to ({sx:.0}, {sy:.0}, {sz:.0})");
        scene.stands = vec![V3::new(sx, sy, sz)];
        min_dist_override = Some(0.0);
    }
    if let Some(p) = get("--probe") {
        let c: Vec<f32> = p.split(',').map(|x| x.trim().parse().unwrap()).collect();
        use parry3d::query::{Ray, RayCast};
        // compare collision, visual, and UNFILTERED scenes: a hit only in
        // "everything" names the mesh a filter wrongly removed
        let vscene = scene::load_visual(&dir);
        let escene = scene::load_everything(&dir);
        for (label, sc) in [("collision", &scene), ("visual", &vscene), ("everything", &escene)] {
            for (name, o, d) in [
                ("down from +2000", V3::new(c[0], c[1], c[2] + 2000.0), V3::new(0.0, 0.0, -1.0)),
                ("up from point", V3::new(c[0], c[1], c[2]), V3::new(0.0, 0.0, 1.0)),
                ("north (+x)", V3::new(c[0], c[1], c[2]), V3::new(1.0, 0.0, 0.0)),
                ("south (-x)", V3::new(c[0], c[1], c[2]), V3::new(-1.0, 0.0, 0.0)),
                ("east (+y)", V3::new(c[0], c[1], c[2]), V3::new(0.0, 1.0, 0.0)),
                ("west (-y)", V3::new(c[0], c[1], c[2]), V3::new(0.0, -1.0, 0.0)),
            ] {
                let ray = Ray::new(nalgebra::Point3::from(o), d);
                match sc.mesh.cast_ray_and_get_normal(&nalgebra::Isometry3::identity(), &ray, 1.0e5, true) {
                    Some(h) => {
                        // backface hits report face id + ntris (parry): fold back
                        let ntris = sc.mesh.indices().len() as u32;
                        let owner = match h.feature {
                            parry3d::shape::FeatureId::Face(f) => sc.owner_of(f % ntris.max(1)),
                            _ => "?",
                        };
                        println!("[{label:>10}] {name}: dist {:.1} -> z={:.1} [{owner}]", h.time_of_impact, o.z + d.z * h.time_of_impact);
                    }
                    None => println!("[{label:>10}] {name}: no hit"),
                }
            }
        }
        return;
    }
    if let Some(t) = get("--xray") {
        // like --throw but against the UNFILTERED everything-scene: the first
        // contact names geometry the collision filters dropped that the real
        // map might block with
        let c: Vec<f32> = t.split(',').map(|x| x.trim().parse().unwrap()).collect();
        let (sy, cy) = c[3].to_radians().sin_cos();
        let (sp, cp) = c[4].to_radians().sin_cos();
        let dir_v = V3::new(cp * cy, cp * sy, sp);
        let escene = scene::load_everything(&dir);
        let o = sim::hand_origin(V3::new(c[0], c[1], c[2]), c[3], &cfg);
        match sim::fly_path_traced(&escene, o, dir_v, &cfg) {
            Some((r, _, _)) => eprintln!("xray rest ({:.0},{:.0},{:.0}) t={:.2}", r.rest.x, r.rest.y, r.rest.z, r.time),
            None => eprintln!("xray never stopped"),
        }
        return;
    }
    if let Some(t) = get("--throw") {
        let c: Vec<f32> = t.split(',').map(|x| x.trim().parse().unwrap()).collect();
        let (sy, cy) = c[3].to_radians().sin_cos();
        let (sp, cp) = c[4].to_radians().sin_cos();
        let dir = V3::new(cp * cy, cp * sy, sp);
        let o = sim::hand_origin(V3::new(c[0], c[1], c[2]), c[3], &cfg);
        eprintln!("throw from ({:.0},{:.0},{:.0}) yaw={} pitch={} (hand offset {})", o.x, o.y, o.z, c[3], c[4], cfg.hand_left);
        match sim::fly_path_traced(&scene, o, dir, &cfg) {
            Some((r, traj, first_bounce)) => {
                eprintln!("rest ({:.0},{:.0},{:.0}) t={:.2} bounces={}", r.rest.x, r.rest.y, r.rest.z, r.time, r.bounces);
                // path tail after the first bounce: step deltas expose
                // zigzag/oscillation the bounce-event trace hides
                for w in traj[first_bounce.saturating_sub(2)..].windows(2) {
                    let d = w[1] - w[0];
                    eprintln!("  ({:7.1},{:8.1},{:6.1}) d=({:6.1},{:6.1},{:6.1}) |d|={:5.1}", w[1].x, w[1].y, w[1].z, d.x, d.y, d.z, d.norm());
                }
            }
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
                if let Some(r) = sim::fly(&scene, sim::hand_origin(o, yaw, &cfg), V3::new(cp * cy, cp * sy, sp), &cfg) {
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
    let min_dist: f32 =
        min_dist_override.unwrap_or_else(|| get("--min-dist").map(|s| s.parse().unwrap()).unwrap_or(1800.0));
    let stands = scene.stands.clone();
    let lineups = solve::solve(&scene, None, &stands, target, tol, min_dist, min_dist_override.is_none(), false, &cfg);
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

    // flight video frames: --video N writes f0000.bmp.. for lineup #N into
    // --video-out <dir>; encode with ffmpeg afterwards. Static side-ish camera
    // fitted to the whole trajectory; trail + molly dot + target ring.
    if let Some(nstr) = get("--video") {
        let n: usize = nstr.parse().expect("--video N");
        let l = lineups.get(n - 1).expect("lineup index out of range");
        let outdir = get("--video-out").unwrap_or_else(|| "video_frames".into());
        std::fs::create_dir_all(&outdir).unwrap();
        let vscene = scene::load_visual(&dir);
        let origin = l.stand + V3::new(0.0, 0.0, cfg.eye_z);
        let launch_pitch = sim::launch_pitch(l.pitch, &cfg);
        let (sy, cy2) = l.yaw.to_radians().sin_cos();
        let (sp, cp) = launch_pitch.to_radians().sin_cos();
        let (_, traj, first_bounce) = sim::fly_path(&scene, origin, V3::new(cp * cy2, cp * sy, sp), &cfg)
            .expect("flight");
        // camera: beside and above the path midpoint, fitted by pulling back
        let mid = traj.iter().fold(V3::zeros(), |a, p| a + p) / traj.len() as f32;
        let perp = V3::new(-sy, cy2, 0.0);
        let range = (target - origin).norm().max(1500.0);
        let cam = mid + perp * (range * 0.55) + V3::new(-cy2, -sy, 0.0) * (range * 0.15)
            + V3::new(0.0, 0.0, range * 0.25);
        let look = mid - cam;
        let cam_yaw = look.y.atan2(look.x).to_degrees();
        let cam_pitch = (look.z / look.norm()).asin().to_degrees();
        const FRAMES: usize = 96;
        for f in 0..FRAMES {
            let upto = render::flight_frame_index2(f, FRAMES, 16, traj.len(), first_bounce);
            let fp = format!("{outdir}/f{f:04}.bmp");
            render::render_flight(&vscene, cam, cam_yaw, cam_pitch, &fp, target, &traj, upto);
        }
        eprintln!("wrote {FRAMES} frames to {outdir} (flight of lineup #{n})");
        return;
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
            // wide context: drone shot with a visibility-checked camera
            let (wide_eye, wyaw, wpitch) = render::wide_cam(&scene, l.stand, l.yaw);
            let wpath = format!("{prefix}_w{}.bmp", i + 1);
            render::render_marked(&scene, wide_eye, wyaw, wpitch, &wpath, l.stand + V3::new(0.0, 0.0, 40.0));
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
