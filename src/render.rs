//! Synthetic lineup screenshots: first-person render of the map geometry from
//! the stand point looking along the crosshair direction. Matching the in-game
//! screen to this image reproduces the aim (Valorant fixed 103 deg horizontal FOV).

use crate::scene::{Scene, V3};
use std::sync::OnceLock;

/// (w, h, color top-down BGR, mask top-down) from cards/hud{,mask}.bmp, if present.
fn hud_overlay() -> Option<&'static (usize, usize, Vec<u8>, Vec<u8>)> {
    static HUD: OnceLock<Option<(usize, usize, Vec<u8>, Vec<u8>)>> = OnceLock::new();
    HUD.get_or_init(|| {
        let read = |p: &str| -> Option<(usize, usize, Vec<u8>)> {
            let d = std::fs::read(p).ok()?;
            if &d[0..2] != b"BM" {
                return None;
            }
            let off = u32::from_le_bytes(d[10..14].try_into().unwrap()) as usize;
            let w = i32::from_le_bytes(d[18..22].try_into().unwrap()) as usize;
            let h = i32::from_le_bytes(d[22..26].try_into().unwrap()) as usize;
            if u16::from_le_bytes(d[28..30].try_into().unwrap()) != 24 {
                return None;
            }
            let row = (w * 3 + 3) & !3;
            let mut px = vec![0u8; w * h * 3];
            for y in 0..h {
                let src = off + (h - 1 - y) * row;
                px[y * w * 3..y * w * 3 + w * 3].copy_from_slice(&d[src..src + w * 3]);
            }
            Some((w, h, px))
        };
        let (w, h, hud) = read("cards/hud.bmp")?;
        let (mw, mh, mask) = read("cards/hudmask.bmp")?;
        (mw == w && mh == h).then_some((w, h, hud, mask))
    })
    .as_ref()
}

const DEF_W: usize = 960;
const DEF_H: usize = 540;
const HFOV_DEG: f32 = 103.0;

pub fn render(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, path: &str) {
    render_ex(scene, eye, yaw_deg, pitch_deg, path, false, None, None, DEF_W, DEF_H)
}

pub fn render_grid(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, path: &str) {
    render_ex(scene, eye, yaw_deg, pitch_deg, path, true, None, None, DEF_W, DEF_H)
}

/// Wide context shot with a ring marking a world point (the stand spot).
pub fn render_marked(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, path: &str, mark: V3) {
    render_ex(scene, eye, yaw_deg, pitch_deg, path, true, Some(mark), None, DEF_W, DEF_H)
}

/// One flight-video frame: trail polyline up to `upto`, molly dot at `upto`,
/// ring at the target.
pub fn render_flight(
    scene: &Scene,
    eye: V3,
    yaw_deg: f32,
    pitch_deg: f32,
    path: &str,
    target: V3,
    traj: &[V3],
    upto: usize,
) {
    render_ex(scene, eye, yaw_deg, pitch_deg, path, false, Some(target), Some((traj, upto)), DEF_W, DEF_H)
}

/// Pick a context-camera position that can actually SEE the stand: behind-above
/// along the throw, else offset to the side, else overhead; each candidate is
/// pulled in front of any blocking geometry.
pub fn wide_cam(scene: &Scene, stand: V3, yaw_deg: f32) -> (V3, f32, f32) {
    use parry3d::query::{Ray, RayCast};
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let anchor = stand + V3::new(0.0, 0.0, 40.0);
    let cands = [
        stand + V3::new(-cy * 750.0, -sy * 750.0, 1000.0),
        stand + V3::new(-cy * 400.0 + sy * 500.0, -sy * 400.0 - cy * 500.0, 800.0),
        stand + V3::new(-cy * 400.0 - sy * 500.0, -sy * 400.0 + cy * 500.0, 800.0),
        stand + V3::new(0.0, 0.0, 1100.0),
    ];
    let mut best = cands[3];
    for cand in cands {
        let toc = cand - anchor;
        let d = toc.norm();
        let ray = Ray::new(nalgebra::Point3::from(anchor), toc / d);
        match scene.mesh.cast_ray(&nalgebra::Isometry3::identity(), &ray, d, true) {
            None => {
                best = cand;
                break;
            }
            Some(t) if t * 0.85 > 350.0 => {
                best = anchor + toc / d * (t * 0.85);
                break;
            }
            _ => {}
        }
    }
    let look = anchor - best;
    (best, look.y.atan2(look.x).to_degrees(), (look.z / look.norm()).asin().to_degrees())
}

/// Two-phase trajectory index for flight videos: 55% of the frames cover the
/// flight (launch to first bounce), 45% play the bounce phase in slow motion,
/// then a hold on the rest position.
pub fn flight_frame_index2(f: usize, frames: usize, hold: usize, len: usize, first_bounce: usize) -> usize {
    let live = frames - hold;
    if f >= live {
        return len - 1;
    }
    let split = (live as f32 * 0.55) as usize;
    let fb = first_bounce.min(len - 1);
    if f < split {
        (f as f32 / split as f32 * fb as f32) as usize
    } else {
        let frac = (f - split) as f32 / (live - split) as f32;
        (fb + (frac * (len - 1 - fb) as f32) as usize).min(len - 1)
    }
}

/// Flight frame at reduced size (chase-cam videos rendered on demand).
#[allow(clippy::too_many_arguments)]
pub fn render_flight_sized(
    scene: &Scene,
    eye: V3,
    yaw_deg: f32,
    pitch_deg: f32,
    path: &str,
    target: V3,
    traj: &[V3],
    upto: usize,
    w: usize,
    h: usize,
) {
    render_ex(scene, eye, yaw_deg, pitch_deg, path, false, Some(target), Some((traj, upto)), w, h)
}

#[allow(clippy::too_many_arguments)]
/// Blend the real HUD (ability bar etc.) onto a BMP-ordered pixel buffer:
/// bilinear-sampled with the feathered alpha mask, so it stays clean at any
/// render size instead of nearest-neighbor pixelation.
fn overlay_hud(px: &mut [u8], w: usize, h: usize) {
    let Some((hw, hh, hud, mask)) = hud_overlay() else { return };
    let (hw, hh) = (*hw, *hh);
    let (hw_f, hh_f) = (hw as f32, hh as f32);
    for y in 0..h {
        let fy = ((y as f32 + 0.5) / h as f32 * hh_f - 0.5).max(0.0);
        let y0 = (fy as usize).min(hh - 1);
        let y1 = (y0 + 1).min(hh - 1);
        let ty = fy - y0 as f32;
        for x in 0..w {
            let fx = ((x as f32 + 0.5) / w as f32 * hw_f - 0.5).max(0.0);
            let x0 = (fx as usize).min(hw - 1);
            let x1 = (x0 + 1).min(hw - 1);
            let tx = fx - x0 as f32;
            let bil = |buf: &[u8], c: usize| -> f32 {
                let v = |xx: usize, yy: usize| buf[(yy * hw + xx) * 3 + c] as f32;
                (v(x0, y0) * (1.0 - tx) + v(x1, y0) * tx) * (1.0 - ty)
                    + (v(x0, y1) * (1.0 - tx) + v(x1, y1) * tx) * ty
            };
            let a = bil(mask, 0) / 255.0;
            if a < 0.02 {
                continue;
            }
            let o = ((h - 1 - y) * w + x) * 3;
            for c in 0..3 {
                px[o + c] = (px[o + c] as f32 * (1.0 - a) + bil(hud, c) * a) as u8;
            }
        }
    }
}

/// Interactive first-person frame for walk mode: one BVH raycast per pixel,
/// parallel over rows: fast enough to stream. Same hillshade / sky / HUD /
/// crosshair language as the stills. Returns finished BMP bytes (no disk).
pub fn render_pov_bytes(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, w: usize, h: usize) -> Vec<u8> {
    use parry3d::query::{Ray, RayCast};
    use rayon::prelude::*;
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    let fwd = V3::new(cp * cy, cp * sy, sp);
    let right = V3::new(-sy, cy, 0.0);
    let up = fwd.cross(&right).normalize();
    let tan_h = (HFOV_DEG.to_radians() / 2.0).tan();
    let tan_v = tan_h * h as f32 / w as f32;
    let id = nalgebra::Isometry3::identity();
    let ntris = scene.mesh.indices().len() as u32;
    let mut px = vec![0u8; w * h * 3];
    px.par_chunks_mut(w * 3).enumerate().for_each(|(buf_y, rowbuf)| {
        let scr_y = h - 1 - buf_y; // BMP rows are bottom-up
        let sv = 1.0 - (scr_y as f32 + 0.5) / h as f32 * 2.0;
        for x in 0..w {
            let su = (x as f32 + 0.5) / w as f32 * 2.0 - 1.0;
            let dir = (fwd + right * (su * tan_h) + up * (sv * tan_v)).normalize();
            let ray = Ray::new(nalgebra::Point3::from(eye), dir);
            let o = x * 3;
            let (b, g, r) = match scene.mesh.cast_ray_and_get_normal(&id, &ray, 5.0e4, true) {
                Some(hit) => {
                    let n = hit.normal;
                    let lambert = 0.22 + 0.72 * n.dot(&V3::new(0.55, 0.45, 0.70)).abs();
                    let tri = match hit.feature {
                        parry3d::shape::FeatureId::Face(i) => i % ntris.max(1),
                        _ => 0,
                    };
                    let mat = scene.color_of(tri);
                    let fog = (hit.time_of_impact / 9000.0).min(0.75);
                    let l = lambert * (1.0 - fog);
                    (
                        (((mat[2] * 1.35).min(1.0) * l + 0.65 * fog).clamp(0.0, 1.0) * 255.0) as u8,
                        (((mat[1] * 1.35).min(1.0) * l + 0.60 * fog).clamp(0.0, 1.0) * 255.0) as u8,
                        (((mat[0] * 1.35).min(1.0) * l + 0.55 * fog).clamp(0.0, 1.0) * 255.0) as u8,
                    )
                }
                None => (235u8, 195u8, 140u8),
            };
            rowbuf[o] = b;
            rowbuf[o + 1] = g;
            rowbuf[o + 2] = r;
        }
    });
    // real ability-bar HUD (the aim reference) + crosshair, as in the stills
    overlay_hud(&mut px, w, h);
    for d in 0..14i32 {
        for (cx, cy2) in [(w as i32 / 2 + d - 7, h as i32 / 2), (w as i32 / 2, h as i32 / 2 + d - 7)] {
            if cx >= 0 && cx < w as i32 && cy2 >= 0 && cy2 < h as i32 {
                let o = ((h - 1 - cy2 as usize) * w + cx as usize) * 3;
                px[o] = 60;
                px[o + 1] = 255;
                px[o + 2] = 60;
            }
        }
    }
    let row = w * 3;
    let size = 54 + row * h;
    let mut bmp = Vec::with_capacity(size);
    bmp.extend_from_slice(b"BM");
    bmp.extend_from_slice(&(size as u32).to_le_bytes());
    bmp.extend_from_slice(&[0; 4]);
    bmp.extend_from_slice(&54u32.to_le_bytes());
    bmp.extend_from_slice(&40u32.to_le_bytes());
    bmp.extend_from_slice(&(w as i32).to_le_bytes());
    bmp.extend_from_slice(&(h as i32).to_le_bytes());
    bmp.extend_from_slice(&1u16.to_le_bytes());
    bmp.extend_from_slice(&24u16.to_le_bytes());
    bmp.extend_from_slice(&[0; 24]);
    bmp.extend_from_slice(&px);
    bmp
}

fn render_ex(
    scene: &Scene,
    eye: V3,
    yaw_deg: f32,
    pitch_deg: f32,
    path: &str,
    grid: bool,
    mark: Option<V3>,
    traj: Option<(&[V3], usize)>,
    w: usize,
    h: usize,
) {
    #[allow(non_snake_case)]
    let (W, H) = (w, h);
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    let fwd = V3::new(cp * cy, cp * sy, sp);
    let right = V3::new(-sy, cy, 0.0);
    let up = fwd.cross(&right).normalize(); // screen-up: +Z when the camera is level
    let tan_h = (HFOV_DEG.to_radians() / 2.0).tan();
    let tan_v = tan_h * H as f32 / W as f32;

    let mut depth = vec![f32::INFINITY; W * H];
    let mut shade = vec![[0.55f32; 3]; W * H]; // rgb, sky-ish base
    let mut is_sky = vec![true; W * H];

    let vtx = scene.mesh.vertices();
    for (tri_idx, tri) in scene.mesh.indices().iter().enumerate() {
        let mat = scene.color_of(tri_idx as u32);
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
        // early frustum reject: whole triangle outside one screen edge
        if cam.iter().all(|c| c.x > c.z * tan_h)
            || cam.iter().all(|c| c.x < -c.z * tan_h)
            || cam.iter().all(|c| c.y > c.z * tan_v)
            || cam.iter().all(|c| c.y < -c.z * tan_v)
        {
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
                    let l = lambert * (1.0 - fog);
                    // boost material color toward readable brightness
                    shade[idx] = [
                        (mat[0] * 1.35).min(1.0) * l + 0.55 * fog,
                        (mat[1] * 1.35).min(1.0) * l + 0.60 * fog,
                        (mat[2] * 1.35).min(1.0) * l + 0.65 * fog,
                    ];
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
                let m = if f500(wp.x) || f500(wp.y) {
                    0.55
                } else if f100(wp.x) || f100(wp.y) {
                    0.8
                } else {
                    1.0
                };
                for c in &mut shade[i] {
                    *c *= m;
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
                (
                    (shade[i][2].clamp(0.0, 1.0) * 255.0) as u8,
                    (shade[i][1].clamp(0.0, 1.0) * 255.0) as u8,
                    (shade[i][0].clamp(0.0, 1.0) * 255.0) as u8,
                )
            };
            px[o] = b;
            px[o + 1] = g;
            px[o + 2] = r;
        }
    }
    // world-point marker: green ring where the mark projects
    if let Some(m) = mark {
        let d = m - eye;
        let c = V3::new(d.dot(&right), d.dot(&up), d.dot(&fwd));
        if c.z > 1.0 {
            let mx = ((c.x / (c.z * tan_h) * 0.5 + 0.5) * W as f32) as i32;
            let my = ((0.5 - c.y / (c.z * tan_v) * 0.5) * H as f32) as i32;
            for a in 0..64 {
                let t = a as f32 / 64.0 * std::f32::consts::TAU;
                for r in [10.0f32, 11.0] {
                    let (px_, py_) = ((mx as f32 + t.cos() * r) as i32, (my as f32 + t.sin() * r) as i32);
                    if px_ >= 0 && px_ < W as i32 && py_ >= 0 && py_ < H as i32 {
                        let o = ((H - 1 - py_ as usize) * W + px_ as usize) * 3;
                        px[o] = 60;
                        px[o + 1] = 255;
                        px[o + 2] = 60;
                    }
                }
            }
        }
    }

    // flight trail overlay: project the trajectory, draw trail + molly dot
    if let Some((tr, upto)) = traj {
        let proj = |w: &V3| -> Option<(i32, i32)> {
            let d = w - eye;
            let c = V3::new(d.dot(&right), d.dot(&up), d.dot(&fwd));
            if c.z < 1.0 {
                return None;
            }
            Some((
                ((c.x / (c.z * tan_h) * 0.5 + 0.5) * W as f32) as i32,
                ((0.5 - c.y / (c.z * tan_v) * 0.5) * H as f32) as i32,
            ))
        };
        let mut put = |x: i32, y: i32, r: u8, g: u8, b: u8| {
            if x >= 0 && x < W as i32 && y >= 0 && y < H as i32 {
                let o = ((H - 1 - y as usize) * W + x as usize) * 3;
                px[o] = b;
                px[o + 1] = g;
                px[o + 2] = r;
            }
        };
        let end = upto.min(tr.len().saturating_sub(1));
        for w in tr[..=end].windows(2) {
            if let (Some(a), Some(b)) = (proj(&w[0]), proj(&w[1])) {
                let (dx, dy) = ((b.0 - a.0) as f32, (b.1 - a.1) as f32);
                let n = dx.abs().max(dy.abs()).max(1.0) as i32;
                for s in 0..=n {
                    let f = s as f32 / n as f32;
                    put(a.0 + (dx * f) as i32, a.1 + (dy * f) as i32, 255, 200, 40);
                }
            }
        }
        if let Some((mx, my)) = proj(&tr[end]) {
            for dy in -4i32..=4 {
                for dx in -4i32..=4 {
                    if dx * dx + dy * dy <= 16 {
                        put(mx + dx, my + dy, 255, 90, 30);
                    }
                }
            }
        }
    }

    // real HUD overlay (aim views only): the actual UI extracted from Henry's
    // own gameplay (cards/hud.bmp + hudmask.bmp) - lineups are aimed by
    // aligning world features against this static UI
    if mark.is_none() && !grid {
        overlay_hud(&mut px, W, H);
    }

    // crosshair: green cross at center (not on marked context shots)
    for d in if mark.is_none() { 0..14i32 } else { 0..0i32 } {
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
