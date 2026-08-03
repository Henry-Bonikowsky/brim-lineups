//! Synthetic lineup screenshots: first-person render of the map geometry from
//! the stand point looking along the crosshair direction. Matching the in-game
//! screen to this image reproduces the aim (Valorant fixed 103 deg horizontal FOV).

use crate::scene::{Scene, V3};

const W: usize = 960;
const H: usize = 540;
const HFOV_DEG: f32 = 103.0;

pub fn render(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, path: &str) {
    render_ex(scene, eye, yaw_deg, pitch_deg, path, false)
}

pub fn render_grid(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, path: &str) {
    render_ex(scene, eye, yaw_deg, pitch_deg, path, true)
}

fn render_ex(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, path: &str, grid: bool) {
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    let fwd = V3::new(cp * cy, cp * sy, sp);
    let right = V3::new(-sy, cy, 0.0);
    let up = fwd.cross(&right).normalize(); // screen-up: +Z when the camera is level
    let tan_h = (HFOV_DEG.to_radians() / 2.0).tan();
    let tan_v = tan_h * H as f32 / W as f32;

    let mut depth = vec![f32::INFINITY; W * H];
    let mut shade = vec![0.55f32; W * H]; // sky-ish base
    let mut is_sky = vec![true; W * H];

    let vtx = scene.mesh.vertices();
    for tri in scene.mesh.indices() {
        let p: Vec<V3> = tri.iter().map(|&i| V3::new(vtx[i as usize].x, vtx[i as usize].y, vtx[i as usize].z)).collect();
        // camera space: x right, y up, z forward
        let cam: Vec<V3> = p
            .iter()
            .map(|w| {
                let d = w - eye;
                V3::new(d.dot(&right), d.dot(&up), d.dot(&fwd))
            })
            .collect();
        if cam.iter().all(|c| c.z < 1.0) {
            continue; // fully behind
        }
        // project (skip tris crossing the near plane; acceptable for stills)
        if cam.iter().any(|c| c.z < 1.0) {
            continue;
        }
        let scr: Vec<(f32, f32, f32)> = cam
            .iter()
            .map(|c| {
                (
                    (c.x / (c.z * tan_h) * 0.5 + 0.5) * W as f32,
                    (0.5 - c.y / (c.z * tan_v) * 0.5) * H as f32,
                    c.z,
                )
            })
            .collect();
        let (min_x, max_x) = (
            scr.iter().map(|s| s.0).fold(f32::MAX, f32::min).max(0.0) as usize,
            (scr.iter().map(|s| s.0).fold(f32::MIN, f32::max).min(W as f32 - 1.0)) as usize,
        );
        let (min_y, max_y) = (
            scr.iter().map(|s| s.1).fold(f32::MAX, f32::min).max(0.0) as usize,
            (scr.iter().map(|s| s.1).fold(f32::MIN, f32::max).min(H as f32 - 1.0)) as usize,
        );
        if min_x > max_x || min_y > max_y {
            continue;
        }
        // face normal shading (world space)
        let n = (p[1] - p[0]).cross(&(p[2] - p[0]));
        let nl = n.norm();
        if nl < 1e-3 {
            continue;
        }
        let n = n / nl;
        // oblique hillshade light: floors mid-gray, walls contrast both ways
        let lambert = 0.22 + 0.72 * n.dot(&V3::new(0.55, 0.45, 0.70)).abs();
        let (ax, ay) = (scr[0].0, scr[0].1);
        let (bx, by) = (scr[1].0, scr[1].1);
        let (cx, cxy) = (scr[2].0, scr[2].1);
        let den = (by - cxy) * (ax - cx) + (cx - bx) * (ay - cxy);
        if den.abs() < 1e-6 {
            continue;
        }
        for py in min_y..=max_y {
            for px in min_x..=max_x {
                let (fx, fy) = (px as f32 + 0.5, py as f32 + 0.5);
                let l0 = ((by - cxy) * (fx - cx) + (cx - bx) * (fy - cxy)) / den;
                let l1 = ((cxy - ay) * (fx - cx) + (ax - cx) * (fy - cxy)) / den;
                let l2 = 1.0 - l0 - l1;
                if l0 < 0.0 || l1 < 0.0 || l2 < 0.0 {
                    continue;
                }
                let z = 1.0 / (l0 / scr[0].2 + l1 / scr[1].2 + l2 / scr[2].2);
                let idx = py * W + px;
                if z < depth[idx] {
                    depth[idx] = z;
                    let fog = (z / 9000.0).min(0.75);
                    shade[idx] = lambert * (1.0 - fog) + 0.55 * fog;
                    is_sky[idx] = false;
                }
            }
        }
    }

    // world-space grid on geometry (stand views): 100u lines, heavier at 500u
    if grid {
        for y in 0..H {
            for x in 0..W {
                let i = y * W + x;
                if is_sky[i] {
                    continue;
                }
                let sx = (x as f32 + 0.5) / W as f32 * 2.0 - 1.0;
                let sy = 1.0 - (y as f32 + 0.5) / H as f32 * 2.0;
                let wp = eye + (fwd + right * (sx * tan_h) + up * (sy * tan_v)) * depth[i];
                let f100 = |v: f32| (v.rem_euclid(100.0) - 50.0).abs() > 50.0 - 3.0;
                let f500 = |v: f32| (v.rem_euclid(500.0) - 250.0).abs() > 250.0 - 4.0;
                if f500(wp.x) || f500(wp.y) {
                    shade[i] *= 0.55;
                } else if f100(wp.x) || f100(wp.y) {
                    shade[i] *= 0.8;
                }
            }
        }
    }

    // BMP, bottom-up BGR
    let mut px = vec![0u8; W * H * 3];
    for y in 0..H {
        for x in 0..W {
            let i = y * W + x;
            let o = ((H - 1 - y) * W + x) * 3;
            let (b, g, r) = if is_sky[i] {
                (235u8, 195u8, 140u8) // sky blue-ish
            } else {
                let v = (shade[i].clamp(0.0, 1.0) * 255.0) as u8;
                (v, v, (v as f32 * 0.94) as u8)
            };
            px[o] = b;
            px[o + 1] = g;
            px[o + 2] = r;
        }
    }
    // crosshair: green cross at center
    for d in 0..14i32 {
        for (cx, cy) in [
            (W as i32 / 2 + d - 7, H as i32 / 2),
            (W as i32 / 2, H as i32 / 2 + d - 7),
        ] {
            if cx >= 0 && cx < W as i32 && cy >= 0 && cy < H as i32 {
                let o = ((H - 1 - cy as usize) * W + cx as usize) * 3;
                px[o] = 60;
                px[o + 1] = 255;
                px[o + 2] = 60;
            }
        }
    }
    let mut bmp = Vec::with_capacity(54 + px.len());
    let row = W * 3; // W*3 is a multiple of 4 for W=960
    let size = 54 + row * H;
    bmp.extend_from_slice(b"BM");
    bmp.extend_from_slice(&(size as u32).to_le_bytes());
    bmp.extend_from_slice(&[0; 4]);
    bmp.extend_from_slice(&54u32.to_le_bytes());
    bmp.extend_from_slice(&40u32.to_le_bytes());
    bmp.extend_from_slice(&(W as i32).to_le_bytes());
    bmp.extend_from_slice(&(H as i32).to_le_bytes());
    bmp.extend_from_slice(&1u16.to_le_bytes());
    bmp.extend_from_slice(&24u16.to_le_bytes());
    bmp.extend_from_slice(&[0; 24]);
    bmp.extend_from_slice(&px);
    std::fs::write(path, bmp).expect("write bmp");
}
