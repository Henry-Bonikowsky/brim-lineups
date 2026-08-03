//! Local server so the picker solves on click: serves cards/ statically and
//! exposes GET /solve?map=&tx=&ty=[&sx=&sy=][&tol=] returning lineups as JSON
//! with fresh aim/stand/wide renders under cards/live/.
//!
//! Scenes are cached per map (collision + visual for the last map used), so the
//! first click on a map pays the load and later clicks are instant.

use crate::scene::{self, Scene, V3};
use crate::sim::Cfg;
use crate::{render, solve};
use std::io::Read as _;

pub fn serve(dumps_root: &str, cards_dir: &str, port: u16) {
    let server = tiny_http::Server::http(("127.0.0.1", port)).expect("bind");
    eprintln!("serving http://localhost:{port}/picker.html (dumps at {dumps_root})");
    // LRU of the 3 most recent maps' scenes (collision + visual)
    let mut cache_lru: Vec<(String, Scene, Scene)> = Vec::new();
    let live = format!("{cards_dir}/live");
    std::fs::create_dir_all(&live).ok();

    for req in server.incoming_requests() {
        let url = req.url().to_string();
        let (path, query) = match url.split_once('?') {
            Some((p, q)) => (p.to_string(), q.to_string()),
            None => (url.clone(), String::new()),
        };
        if path == "/solve" {
            let get = |k: &str| -> Option<String> {
                query.split('&').find_map(|kv| {
                    let (a, b) = kv.split_once('=')?;
                    (a == k).then(|| b.replace("%2C", ",").replace('+', " "))
                })
            };
            let map = match get("map") {
                Some(m) if m.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') => m,
                _ => {
                    let _ = req.respond(tiny_http::Response::from_string("bad map").with_status_code(400));
                    continue;
                }
            };
            let (tx, ty) = match (get("tx").and_then(|v| v.parse::<f32>().ok()), get("ty").and_then(|v| v.parse::<f32>().ok())) {
                (Some(a), Some(b)) => (a, b),
                _ => {
                    let _ = req.respond(tiny_http::Response::from_string("bad target").with_status_code(400));
                    continue;
                }
            };
            let stand = get("sx").and_then(|sx| Some((sx.parse::<f32>().ok()?, get("sy")?.parse::<f32>().ok()?)));
            let tol: f32 = get("tol").and_then(|v| v.parse().ok()).unwrap_or(450.0);

            if let Some(pos) = cache_lru.iter().position(|(m, _, _)| m == &map) {
                let e = cache_lru.remove(pos);
                cache_lru.push(e);
            } else {
                let dir = std::path::PathBuf::from(dumps_root).join(&map);
                eprintln!("loading scenes for {map}...");
                cache_lru.push((map.clone(), scene::load(&dir), scene::load_visual(&dir)));
                if cache_lru.len() > 3 {
                    cache_lru.remove(0);
                }
            }
            let (_, cscene, vscene) = cache_lru.last_mut().unwrap();
            let cfg = Cfg::default();
            let Some(tz) = cscene.ground_z(tx, ty) else {
                let _ = req.respond(tiny_http::Response::from_string("{\"error\":\"no ground at target\"}"));
                continue;
            };
            let target = V3::new(tx, ty, tz);
            let saved_stands = if let Some((sx, sy)) = stand {
                let Some(sz) = cscene.ground_z(sx, sy) else {
                    let _ = req.respond(tiny_http::Response::from_string("{\"error\":\"no ground at stand\"}"));
                    continue;
                };
                let orig = std::mem::replace(&mut cscene.stands, vec![V3::new(sx, sy, sz)]);
                Some(orig)
            } else {
                None
            };
            let (min_dist, strict) = if stand.is_some() { (0.0, false) } else { (1800.0, true) };
            let lineups = solve::solve(cscene, target, tol, min_dist, strict, &cfg);
            if let Some(orig) = saved_stands {
                cscene.stands = orig;
            }

            // fresh renders for the top few; unique run id to defeat caching
            let run = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_millis();
            {
                use rayon::prelude::*;
                lineups.par_iter().take(5).enumerate().for_each(|(i, l)| {
                    let eye = l.stand + V3::new(0.0, 0.0, cfg.eye_z);
                    let base = format!("live/{run}_{}", i + 1);
                    render::render(vscene, eye, l.yaw, l.pitch, &format!("{cards_dir}/{base}_r.bmp"));
                    render::render_grid(vscene, l.stand + V3::new(0.0, 0.0, 350.0), l.yaw, -89.0, &format!("{cards_dir}/{base}_s.bmp"));
                    let (wide_eye, wyaw, wpitch) = render::wide_cam(vscene, l.stand, l.yaw);
                    render::render_marked(vscene, wide_eye, wyaw, wpitch, &format!("{cards_dir}/{base}_w.bmp"), l.stand + V3::new(0.0, 0.0, 40.0));
                });
            }
            let mut rows = Vec::new();
            for (i, l) in lineups.iter().take(5).enumerate() {
                let base = format!("live/{run}_{}", i + 1);
                let aim = l
                    .aim_ref
                    .map(|(p, d)| format!("[{:.0},{:.0},{:.0},{:.0}]", p.x, p.y, p.z, d))
                    .unwrap_or("null".into());
                rows.push(format!(
                    "{{\"stand\":[{:.0},{:.0},{:.0}],\"range\":{:.0},\"yaw\":{:.1},\"pitch\":{:.1},\"time\":{:.2},\"bounces\":{},\"err\":{:.0},\"covered\":{},\"forgive\":{:.2},\"spread\":{:.0},\"pos\":{},\"aim_ref\":{},\"imgs\":[\"{base}_r.bmp\",\"{base}_s.bmp\",\"{base}_w.bmp\"]}}",
                    l.stand.x, l.stand.y, l.stand.z, l.dist, l.yaw, l.pitch, l.time, l.bounces, l.err, l.covered, l.forgive, l.spread, l.pos_grade(), aim
                ));
            }
            let body = format!(
                "{{\"target\":[{tx:.0},{ty:.0},{tz:.0}],\"count\":{},\"lineups\":[{}]}}",
                lineups.len(),
                rows.join(",")
            );
            let _ = req.respond(
                tiny_http::Response::from_string(body)
                    .with_header("Content-Type: application/json".parse::<tiny_http::Header>().unwrap()),
            );
            continue;
        }

        if path == "/video" {
            let get = |k: &str| -> Option<String> {
                query.split('&').find_map(|kv| {
                    let (a, b) = kv.split_once('=')?;
                    (a == k).then(|| b.to_string())
                })
            };
            let (Some(map), Some(tx), Some(ty)) = (
                get("map"),
                get("tx").and_then(|v| v.parse::<f32>().ok()),
                get("ty").and_then(|v| v.parse::<f32>().ok()),
            ) else {
                let _ = req.respond(tiny_http::Response::from_string("bad params").with_status_code(400));
                continue;
            };
            let stand = get("sx").and_then(|sx| Some((sx.parse::<f32>().ok()?, get("sy")?.parse::<f32>().ok()?)));
            let tol: f32 = get("tol").and_then(|v| v.parse().ok()).unwrap_or(450.0);
            let n: usize = get("n").and_then(|v| v.parse().ok()).unwrap_or(1);
            if let Some(pos) = cache_lru.iter().position(|(m, _, _)| m == &map) {
                let e = cache_lru.remove(pos);
                cache_lru.push(e);
            } else {
                let dir = std::path::PathBuf::from(dumps_root).join(&map);
                cache_lru.push((map.clone(), scene::load(&dir), scene::load_visual(&dir)));
                if cache_lru.len() > 3 {
                    cache_lru.remove(0);
                }
            }
            let (_, cscene, vscene) = cache_lru.last_mut().unwrap();
            let cfg = Cfg::default();
            let Some(tz) = cscene.ground_z(tx, ty) else {
                let _ = req.respond(tiny_http::Response::from_string("{\"error\":\"no ground\"}"));
                continue;
            };
            let target = V3::new(tx, ty, tz);
            let saved = if let Some((sx, sy)) = stand {
                let sz = cscene.ground_z(sx, sy).unwrap_or(tz);
                Some(std::mem::replace(&mut cscene.stands, vec![V3::new(sx, sy, sz)]))
            } else {
                None
            };
            let (min_dist, strict) = if stand.is_some() { (0.0, false) } else { (1800.0, true) };
            let lineups = solve::solve(cscene, target, tol, min_dist, strict, &cfg);
            if let Some(orig) = saved {
                cscene.stands = orig;
            }
            let Some(l) = lineups.get(n - 1) else {
                let _ = req.respond(tiny_http::Response::from_string("{\"error\":\"no such lineup\"}"));
                continue;
            };
            let origin = crate::sim::hand_origin(l.stand + V3::new(0.0, 0.0, cfg.eye_z), l.yaw, &cfg);
            let lp = l.pitch + cfg.arc_deg;
            let (sy2, cy2) = l.yaw.to_radians().sin_cos();
            let (sp2, cp2) = lp.to_radians().sin_cos();
            let Some((_, traj, first_bounce)) = crate::sim::fly_path(cscene, origin, V3::new(cp2 * cy2, cp2 * sy2, sp2), &cfg) else {
                let _ = req.respond(tiny_http::Response::from_string("{\"error\":\"flight failed\"}"));
                continue;
            };
            // chase cam: behind the molly along its velocity, pulled in if a wall
            // sits between the camera and the molly
            let run = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis();
            let fdir = std::env::temp_dir().join(format!("bl_vid_{run}"));
            std::fs::create_dir_all(&fdir).ok();
            const FRAMES: usize = 90;
            use parry3d::query::{Ray, RayCast};
            use rayon::prelude::*;
            (0..FRAMES).into_par_iter().for_each(|f| {
                let upto = render::flight_frame_index2(f, FRAMES, 16, traj.len(), first_bounce);
                let i = upto.min(traj.len() - 1);
                let m = traj[i];
                let vdir = (traj[(i + 3).min(traj.len() - 1)] - traj[i.saturating_sub(3)]).normalize();
                let vh = V3::new(vdir.x, vdir.y, 0.0).norm().max(0.2);
                let back = V3::new(-vdir.x / vh, -vdir.y / vh, 0.0);
                let mut cam = m + back * 380.0 + V3::new(0.0, 0.0, 170.0);
                let toc = cam - m;
                let d = toc.norm();
                let ray = Ray::new(nalgebra::Point3::from(m), toc / d);
                if let Some(t) = vscene.mesh.cast_ray(&nalgebra::Isometry3::identity(), &ray, d, true) {
                    cam = m + toc / d * (t * 0.85).max(60.0);
                }
                let look = m - cam;
                let cam_yaw = look.y.atan2(look.x).to_degrees();
                let cam_pitch = (look.z / look.norm()).asin().to_degrees();
                render::render_flight_sized(
                    vscene,
                    cam,
                    cam_yaw,
                    cam_pitch,
                    fdir.join(format!("f{f:04}.bmp")).to_str().unwrap(),
                    target,
                    &traj,
                    i,
                    640,
                    360,
                );
            });
            let out = format!("{live}/{run}_flight.mp4");
            let st = std::process::Command::new("ffmpeg")
                .args([
                    "-y", "-v", "error", "-framerate", "16",
                    "-i", fdir.join("f%04d.bmp").to_str().unwrap(),
                    "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p", &out,
                ])
                .status();
            std::fs::remove_dir_all(&fdir).ok();
            let body = match st {
                Ok(s) if s.success() => format!("{{\"video\":\"live/{run}_flight.mp4\"}}"),
                _ => "{\"error\":\"ffmpeg failed (is it on PATH?)\"}".to_string(),
            };
            let _ = req.respond(
                tiny_http::Response::from_string(body)
                    .with_header("Content-Type: application/json".parse::<tiny_http::Header>().unwrap()),
            );
            continue;
        }

        // walk mode: first-person frame at an exact stand + aim
        if path == "/pov" || path == "/shoot" {
            let get = |k: &str| -> Option<String> {
                query.split('&').find_map(|kv| {
                    let (a, b) = kv.split_once('=')?;
                    (a == k).then(|| b.to_string())
                })
            };
            let (Some(map), Some(x), Some(y), Some(yaw), Some(pitch)) = (
                get("map"),
                get("x").and_then(|v| v.parse::<f32>().ok()),
                get("y").and_then(|v| v.parse::<f32>().ok()),
                get("yaw").and_then(|v| v.parse::<f32>().ok()),
                get("pitch").and_then(|v| v.parse::<f32>().ok()),
            ) else {
                let _ = req.respond(tiny_http::Response::from_string("bad params").with_status_code(400));
                continue;
            };
            if let Some(pos) = cache_lru.iter().position(|(m, _, _)| m == &map) {
                let e = cache_lru.remove(pos);
                cache_lru.push(e);
            } else {
                let dir = std::path::PathBuf::from(dumps_root).join(&map);
                eprintln!("loading scenes for {map}...");
                cache_lru.push((map.clone(), scene::load(&dir), scene::load_visual(&dir)));
                if cache_lru.len() > 3 {
                    cache_lru.remove(0);
                }
            }
            let (_, cscene, vscene) = cache_lru.last().unwrap();
            let cfg = Cfg::default();
            let gz = cscene.ground_z(x, y);
            let Some(gz) = gz else {
                let _ = req.respond(tiny_http::Response::from_string("{\"error\":\"no ground here\"}").with_status_code(200));
                continue;
            };
            let eye = V3::new(x, y, gz + cfg.eye_z);
            if path == "/pov" {
                // width param: walking uses 640 (fast), idle refines at 1152
                let w = get("w").and_then(|v| v.parse::<usize>().ok()).unwrap_or(640).clamp(320, 1152) / 8 * 8;
                let bytes = render::render_pov_bytes(vscene, eye, yaw, pitch, w, w * 9 / 16);
                let _ = req.respond(
                    tiny_http::Response::from_data(bytes)
                        .with_header("Content-Type: image/bmp".parse::<tiny_http::Header>().unwrap()),
                );
                continue;
            }
            // /shoot: simulate the molly from EXACTLY here with EXACTLY this
            // crosshair aim (launch = crosshair + arc), like standing in game
            let lp = pitch + cfg.arc_deg;
            let (sy2, cy2) = yaw.to_radians().sin_cos();
            let (sp2, cp2) = lp.to_radians().sin_cos();
            let hand = crate::sim::hand_origin(eye, yaw, &cfg);
            let Some((out, traj, first_bounce)) = crate::sim::fly_path(cscene, hand, V3::new(cp2 * cy2, cp2 * sy2, sp2), &cfg) else {
                let _ = req.respond(tiny_http::Response::from_string("{\"error\":\"flight never settled\"}"));
                continue;
            };
            let run = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis();
            let fdir = std::env::temp_dir().join(format!("bl_vid_{run}"));
            std::fs::create_dir_all(&fdir).ok();
            const FRAMES: usize = 90;
            use parry3d::query::{Ray, RayCast};
            use rayon::prelude::*;
            (0..FRAMES).into_par_iter().for_each(|f| {
                let upto = render::flight_frame_index2(f, FRAMES, 16, traj.len(), first_bounce);
                let i = upto.min(traj.len() - 1);
                let m = traj[i];
                let vdir = (traj[(i + 3).min(traj.len() - 1)] - traj[i.saturating_sub(3)]).normalize();
                let vh = V3::new(vdir.x, vdir.y, 0.0).norm().max(0.2);
                let back = V3::new(-vdir.x / vh, -vdir.y / vh, 0.0);
                let mut cam = m + back * 380.0 + V3::new(0.0, 0.0, 170.0);
                let toc = cam - m;
                let d = toc.norm();
                let ray = Ray::new(nalgebra::Point3::from(m), toc / d);
                if let Some(t) = vscene.mesh.cast_ray(&nalgebra::Isometry3::identity(), &ray, d, true) {
                    cam = m + toc / d * (t * 0.85).max(60.0);
                }
                let look = m - cam;
                let cam_yaw = look.y.atan2(look.x).to_degrees();
                let cam_pitch = (look.z / look.norm()).asin().to_degrees();
                render::render_flight_sized(
                    vscene, cam, cam_yaw, cam_pitch,
                    fdir.join(format!("f{f:04}.bmp")).to_str().unwrap(),
                    out.rest, &traj, i, 640, 360,
                );
            });
            let out_path = format!("{live}/{run}_flight.mp4");
            let st = std::process::Command::new("ffmpeg")
                .args([
                    "-y", "-v", "error", "-framerate", "16",
                    "-i", fdir.join("f%04d.bmp").to_str().unwrap(),
                    "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p", &out_path,
                ])
                .status();
            std::fs::remove_dir_all(&fdir).ok();
            let vid = match st {
                Ok(s) if s.success() => format!("\"live/{run}_flight.mp4\""),
                _ => "null".to_string(),
            };
            let body = format!(
                "{{\"rest\":[{:.0},{:.0},{:.0}],\"time\":{:.2},\"bounces\":{},\"stand\":[{x:.0},{y:.0},{gz:.0}],\"video\":{vid}}}",
                out.rest.x, out.rest.y, out.rest.z, out.time, out.bounces
            );
            let _ = req.respond(
                tiny_http::Response::from_string(body)
                    .with_header("Content-Type: application/json".parse::<tiny_http::Header>().unwrap()),
            );
            continue;
        }

        // static files from cards/
        let rel = if path == "/" { "picker.html".to_string() } else { path.trim_start_matches('/').to_string() };
        if rel.contains("..") {
            let _ = req.respond(tiny_http::Response::from_string("no").with_status_code(403));
            continue;
        }
        match std::fs::File::open(format!("{cards_dir}/{rel}")) {
            Ok(mut f) => {
                let mut buf = Vec::new();
                f.read_to_end(&mut buf).ok();
                let ct = match rel.rsplit('.').next().unwrap_or("") {
                    "html" => "text/html; charset=utf-8",
                    "png" => "image/png",
                    "bmp" => "image/bmp",
                    "mp4" => "video/mp4",
                    "json" => "application/json",
                    _ => "application/octet-stream",
                };
                let _ = req.respond(
                    tiny_http::Response::from_data(buf)
                        .with_header(format!("Content-Type: {ct}").parse::<tiny_http::Header>().unwrap()),
                );
            }
            Err(_) => {
                let _ = req.respond(tiny_http::Response::from_string("not found").with_status_code(404));
            }
        }
    }
}
