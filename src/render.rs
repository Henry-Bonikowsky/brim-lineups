//! Synthetic lineup screenshots: first-person render of the map geometry from
//! the stand point looking along the crosshair direction. Matching the in-game
//! screen to this image reproduces the aim (Valorant fixed 103 deg horizontal FOV).

use crate::scene::{Scene, V3};

// 16:10 to match Henry's ACTUAL game resolution (1152x720): with a fixed
// 103 deg horizontal FOV the vertical FOV depends on aspect, so any UI
// reference off screen center only transfers if the aspect matches
const DEF_W: usize = 960;
const DEF_H: usize = 600;
const HFOV_DEG: f32 = 103.0;
/// Valorant holds VERTICAL FOV constant across aspect ratios (103 horizontal
/// is only true at 16:9). tan of the half-vertical-FOV; horizontal derives
/// from it per the render aspect.
fn tan_vh(w: usize, h: usize) -> (f32, f32) {
    let tan_v = (HFOV_DEG.to_radians() / 2.0).tan() * 9.0 / 16.0;
    (tan_v * w as f32 / h as f32, tan_v)
}

/// Real diffuse for a surface hit: the tri's material texture sampled at the
/// authored UVs (barycentric of the world point within the tri). Falls back
/// to the flat material color when the surface has no texture.
/// Sky for a view ray: vertical gradient plus a glow around the map's sun.
/// Shared by both renderers.
pub fn sky_color(dir: V3, sun: V3, sun_color: [f32; 3]) -> [f32; 3] {
    let t = dir.z.clamp(0.0, 1.0).powf(0.7);
    let mut c = [0.78 - 0.36 * t, 0.86 - 0.24 * t, 0.93 - 0.05 * t];
    let g = dir.dot(&sun).max(0.0).powf(200.0) * 1.2;
    for (i, ch) in c.iter_mut().enumerate() {
        *ch = (*ch + g * sun_color[i]).min(1.0);
    }
    c
}

/// THE lit surface color: albedo x (hemispheric ambient + the map's real sun
/// N.L, optionally shadow-rayed) + distance fog. The rasterizer, the
/// walk-mode raycaster and solve's corner luminance (shadow=false there,
/// references anchor on geometry, not our synthetic shadows) all shade
/// through this one function - keep them identical or the references drift
/// from the pictures.
pub fn lit(scene: &crate::scene::Scene, tri: u32, n: V3, wp: V3, eye: V3, detail: bool) -> [f32; 3] {
    use parry3d::query::{Ray, RayCast};
    let id = nalgebra::Isometry3::identity();
    let albedo = surface_color(scene, tri, n, wp);
    let v = wp - eye;
    let dist = v.norm();
    let nn = if n.dot(&v) > 0.0 { -n } else { n }; // orient toward the camera
    let mut sun = nn.dot(&scene.sun).max(0.0);
    if sun > 0.0 && detail {
        // offset off the surface so the ray does not re-hit its own tri
        let o = wp + nn * 2.0;
        if scene.mesh.cast_ray(&id, &Ray::new(nalgebra::Point3::from(o), scene.sun), 5.0e4, true).is_some() {
            sun = 0.0;
        }
    }
    // baked-GI stand-in: shade stays luminous in game, so ambient sits high
    let mut amb = 0.66 + 0.14 * nn.z;
    if detail {
        // 4-ray hemisphere occlusion (deterministic): corners, undersides and
        // contact points darken, which is what makes lighting read everywhere.
        // Steep rays + a grazing-hit floor avoid speckle from a surface's own
        // coplanar neighbors.
        let t1n = if nn.z.abs() < 0.9 { nn.cross(&V3::new(0.0, 0.0, 1.0)) } else { nn.cross(&V3::new(1.0, 0.0, 0.0)) }
            .normalize();
        let t2n = nn.cross(&t1n);
        let o = nalgebra::Point3::from(wp + nn * 4.0);
        let mut open = 0;
        for d in [t1n, -t1n, t2n, -t2n] {
            let dir = nn * 0.66 + d * 0.75;
            match scene.mesh.cast_ray(&id, &Ray::new(o, dir), 260.0, true) {
                Some(t) if t > 14.0 => {}
                Some(_) => open += 1, // grazing self-hit: not real cover
                None => open += 1,
            }
        }
        amb *= 0.70 + 0.30 * open as f32 / 4.0;
    }
    // cool skylight ambient, the map's warm sun, bluish distance haze.
    // Tone-map shoulder instead of a hard clamp: a clamp pushed every bright
    // albedo to 1.0 in sun AND shade, erasing the lighting on most surfaces.
    let ambc = [amb * 0.94, amb * 0.98, amb * 1.04];
    let fog = (dist / 20000.0).min(0.45);
    let fogc = [0.70, 0.78, 0.88];
    std::array::from_fn(|i| {
        let t = albedo[i] * (ambc[i] + 0.85 * sun * scene.sun_color[i]);
        ((t * 1.15 / (1.0 + 0.45 * t)) * (1.0 - fog) + fogc[i] * fog).clamp(0.0, 1.0)
    })
}

pub fn surface_color(scene: &crate::scene::Scene, tri: u32, _n: V3, wp: V3) -> [f32; 3] {
    let (Some(t), Some(uv3)) = (scene.tex_of(tri), scene.uvs.get(tri as usize)) else {
        return scene.color_of(tri);
    };
    let vtx = scene.mesh.vertices();
    let idx = scene.mesh.indices()[tri as usize];
    let p = |k: usize| V3::new(vtx[idx[k] as usize].x, vtx[idx[k] as usize].y, vtx[idx[k] as usize].z);
    let (a, b, c) = (p(0), p(1), p(2));
    let (v0, v1, v2) = (b - a, c - a, wp - a);
    let (d00, d01, d11, d20, d21) =
        (v0.dot(&v0), v0.dot(&v1), v1.dot(&v1), v2.dot(&v0), v2.dot(&v1));
    let den = d00 * d11 - d01 * d01;
    if den.abs() < 1e-9 {
        return scene.color_of(tri);
    }
    let bv = (d11 * d20 - d01 * d21) / den;
    let bw = (d00 * d21 - d01 * d20) / den;
    let bu = 1.0 - bv - bw;
    let u = uv3[0][0] * bu + uv3[1][0] * bv + uv3[2][0] * bw;
    let v = uv3[0][1] * bu + uv3[1][1] * bv + uv3[2][1] * bw;
    t.sample_uv_bilinear(u.rem_euclid(1.0), v.rem_euclid(1.0))
}

pub fn render(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, path: &str) {
    std::fs::write(path, render_bytes(scene, eye, yaw_deg, pitch_deg)).expect("write bmp")
}

pub fn render_bytes(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32) -> Vec<u8> {
    render_ex(scene, eye, yaw_deg, pitch_deg, false, None, None, DEF_W, DEF_H)
}

pub fn render_grid(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, path: &str) {
    std::fs::write(path, render_grid_bytes(scene, eye, yaw_deg, pitch_deg)).expect("write bmp")
}

pub fn render_grid_bytes(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32) -> Vec<u8> {
    render_ex(scene, eye, yaw_deg, pitch_deg, true, None, None, DEF_W, DEF_H)
}

/// Translucent yellow wash over a finished aim BMP: each (dyaw, dpitch) in
/// `hits` (deg, relative to the render's yaw/pitch) is an aim that still
/// lands the throw on the target - drawn as a step-sized cell so the wash
/// tiles without gaps. The player hunts references INSIDE the wash: any
/// texture or edge it covers lines up some working angle.
pub fn stamp_aim_region(bmp: &mut [u8], yaw_deg: f32, pitch_deg: f32, hits: &[(f32, f32)], step: f32) {
    if bmp.len() < 54 || hits.is_empty() {
        return;
    }
    let w = u32::from_le_bytes(bmp[18..22].try_into().unwrap()) as usize;
    let h = u32::from_le_bytes(bmp[22..26].try_into().unwrap()) as usize;
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    let fwd = V3::new(cp * cy, cp * sy, sp);
    let right = V3::new(-sy, cy, 0.0);
    let up = fwd.cross(&right).normalize();
    let (tan_h, tan_v) = tan_vh(w, h);
    // cell half-size in px from the sample step (center-of-screen scale)
    let half = ((step.to_radians() * w as f32 / (2.0 * tan_h)) * 0.5).ceil() as i32;
    let mut mask = vec![false; w * h];
    for &(dy, dp) in hits {
        let (jsy, jcy) = (yaw_deg + dy).to_radians().sin_cos();
        let (jsp, jcp) = (pitch_deg + dp).to_radians().sin_cos();
        let dir = V3::new(jcp * jcy, jcp * jsy, jsp);
        let cz = dir.dot(&fwd);
        if cz <= 0.01 {
            continue;
        }
        let px = ((dir.dot(&right) / cz / tan_h + 1.0) * 0.5 * w as f32) as i32;
        let py = ((1.0 - dir.dot(&up) / cz / tan_v) * 0.5 * h as f32) as i32;
        for yy in (py - half).max(0)..=(py + half).min(h as i32 - 1) {
            for xx in (px - half).max(0)..=(px + half).min(w as i32 - 1) {
                mask[yy as usize * w + xx as usize] = true;
            }
        }
    }
    for y in 0..h {
        for x in 0..w {
            if mask[y * w + x] {
                let o = 54 + ((h - 1 - y) * w + x) * 3;
                // 45% yellow (BGR), texture stays visible under the wash
                bmp[o] = (bmp[o] as f32 * 0.55) as u8;
                bmp[o + 1] = (bmp[o + 1] as f32 * 0.55 + 255.0 * 0.45) as u8;
                bmp[o + 2] = (bmp[o + 2] as f32 * 0.55 + 255.0 * 0.45) as u8;
            }
        }
    }
}

/// Wide context shot with a ring marking a world point (the stand spot).
pub fn render_marked(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, path: &str, mark: V3) {
    std::fs::write(path, render_marked_bytes(scene, eye, yaw_deg, pitch_deg, mark)).expect("write bmp")
}

pub fn render_marked_bytes(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, mark: V3) -> Vec<u8> {
    render_ex(scene, eye, yaw_deg, pitch_deg, true, Some(mark), None, DEF_W, DEF_H)
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
    std::fs::write(path, render_ex(scene, eye, yaw_deg, pitch_deg, false, Some(target), Some((traj, upto)), DEF_W, DEF_H))
        .expect("write bmp")
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

/// Landing camera for flight views: ring candidates around the rest point,
/// preferring the direction the molly ARRIVES from (so the bounce happens
/// toward us), first one that sees both the rest point and the first-bounce
/// point. Shared by the native flight video and the web keyframe stills.
pub fn land_cam(vscene: &Scene, target: V3, traj: &[V3], first_bounce: usize) -> V3 {
    use parry3d::query::{Ray, RayCast};
    let id = nalgebra::Isometry3::identity();
    let clear = |a: V3, b: V3| -> bool {
        let d = b - a;
        let n = d.norm().max(1.0);
        vscene.mesh.cast_ray(&id, &Ray::new(nalgebra::Point3::from(a), d / n), n - 40.0, true).is_none()
    };
    let arrive = {
        let a = traj[first_bounce.min(traj.len() - 1)] - traj[first_bounce.saturating_sub(8)];
        (-a.y).atan2(-a.x)
    };
    let fb_pos = traj[first_bounce.min(traj.len() - 1)];
    let mut cam = target + V3::new(0.0, 0.0, 900.0);
    'search: for (radius, height) in [(650.0f32, 330.0f32), (900.0, 480.0), (450.0, 240.0), (1200.0, 700.0)] {
        for k in 0..12 {
            // fan out from the arrival direction: 0, +-30, +-60... degrees
            let da = ((k + 1) / 2) as f32 * 30.0_f32.to_radians() * if k % 2 == 0 { 1.0 } else { -1.0 };
            let a = arrive + da;
            let c = target + V3::new(a.cos() * radius, a.sin() * radius, height);
            if clear(c, target + V3::new(0.0, 0.0, 60.0)) && clear(c, fb_pos + V3::new(0.0, 0.0, 60.0)) {
                cam = c;
                break 'search;
            }
        }
    }
    cam
}

/// Keyframe stills of one flight (the web build's flight video): a frame 0.5s
/// (60 steps) before the first bounce, then a frame ON each visible bounce and
/// one at the arc apex between bounces, ending at rest. Same cameras and
/// trail/dot/ring language as the video frames, 640x400.
pub fn flight_stills(vscene: &Scene, target: V3, traj: &[V3], first_bounce: usize, bounces: &[usize]) -> Vec<Vec<u8>> {
    let last = traj.len().saturating_sub(1);
    let fb = bounces.first().copied().unwrap_or(first_bounce).min(last);
    let mut idxs: Vec<usize> = vec![fb.saturating_sub(60)];
    for (k, &b) in bounces.iter().enumerate() {
        idxs.push(b.min(last));
        let next = bounces.get(k + 1).copied().unwrap_or(last).min(last);
        // apex of the arc between this bounce and the next (or the rest)
        if let Some(a) = (b.min(last) + 1..next).max_by(|&i, &j| traj[i].z.total_cmp(&traj[j].z)) {
            idxs.push(a);
        }
    }
    idxs.push(last);
    idxs.dedup();
    let lcam = land_cam(vscene, target, traj, fb);
    idxs.iter()
        .map(|&i| {
            let m = traj[i];
            // same camera switch as the video: over-the-shoulder until 1.6s
            // before the first bounce, then the verified landing viewpoint
            let cam = if i + 192 >= fb { lcam } else { traj[0] + V3::new(0.0, 0.0, 90.0) };
            let mut look = m - cam;
            if look.norm() < 150.0 {
                look = traj[(i + 8).min(last)] - cam;
            }
            let cam_yaw = look.y.atan2(look.x).to_degrees();
            let cam_pitch = (look.z / look.norm()).asin().to_degrees();
            render_ex(vscene, cam, cam_yaw, cam_pitch, false, Some(target), Some((traj, i)), 640, 400)
        })
        .collect()
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
    std::fs::write(path, render_ex(scene, eye, yaw_deg, pitch_deg, false, Some(target), Some((traj, upto)), w, h))
        .expect("write bmp")
}

#[allow(clippy::too_many_arguments)]
/// Interactive first-person frame for walk mode: one BVH raycast per pixel,
/// parallel over rows: fast enough to stream. Same hillshade / sky / HUD /
/// crosshair language as the stills. Returns finished BMP bytes (no disk).
pub fn render_pov_bytes(scene: &Scene, eye: V3, yaw_deg: f32, pitch_deg: f32, w: usize, h: usize) -> Vec<u8> {
    use crate::par::*;
    use parry3d::query::{Ray, RayCast};
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    let fwd = V3::new(cp * cy, cp * sy, sp);
    let right = V3::new(-sy, cy, 0.0);
    let up = fwd.cross(&right).normalize();
    let (tan_h, tan_v) = tan_vh(w, h);
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
                    let tri = match hit.feature {
                        parry3d::shape::FeatureId::Face(i) => i % ntris.max(1),
                        _ => 0,
                    };
                    let c = lit(scene, tri, hit.normal, eye + dir * hit.time_of_impact, eye, true);
                    ((c[2] * 255.0) as u8, (c[1] * 255.0) as u8, (c[0] * 255.0) as u8)
                }
                None => {
                    let c = sky_color(dir, scene.sun, scene.sun_color);
                    ((c[2] * 255.0) as u8, (c[1] * 255.0) as u8, (c[0] * 255.0) as u8)
                }
            };
            rowbuf[o] = b;
            rowbuf[o + 1] = g;
            rowbuf[o + 2] = r;
        }
    });
    // HUD is layered in the BROWSER at native resolution (cards/hud.png);
    // baking it here made it mushy. Only the crosshair stays baked.
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
    grid: bool,
    mark: Option<V3>,
    traj: Option<(&[V3], usize)>,
    w: usize,
    h: usize,
) -> Vec<u8> {
    #[allow(non_snake_case)]
    let (W, H) = (w, h);
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    let fwd = V3::new(cp * cy, cp * sy, sp);
    let right = V3::new(-sy, cy, 0.0);
    let up = fwd.cross(&right).normalize(); // screen-up: +Z when the camera is level
    let (tan_h, tan_v) = tan_vh(W, H);

    let mut depth = vec![f32::INFINITY; W * H];
    let mut shade = vec![[0.55f32; 3]; W * H]; // rgb, filled by the shading pass
    let mut is_sky = vec![true; W * H];
    let mut tri_of = vec![0u32; W * H];

    let vtx = scene.mesh.vertices();
    for (tri_idx, tri) in scene.mesh.indices().iter().enumerate() {
        let p: Vec<V3> = tri.iter().map(|&i| V3::new(vtx[i as usize].x, vtx[i as usize].y, vtx[i as usize].z)).collect();
        // camera space: x right, y up, z forward
        let cam: Vec<V3> = p
            .iter()
            .map(|w| {
                let d = w - eye;
                V3::new(d.dot(&right), d.dot(&up), d.dot(&fwd))
            })
            .collect();
        // near-plane clip (z >= 1): map surfaces are huge single triangles, so
        // a camera standing close to a wall/floor puts one vertex behind the
        // plane; dropping the whole tri (the old behavior) erased entire
        // surfaces from stills. Clip instead, lerping world coords in step.
        const NEAR: f32 = 1.0;
        let mut poly: Vec<(V3, V3)> = cam.iter().copied().zip(p.iter().copied()).collect();
        if poly.iter().all(|(c, _)| c.z < NEAR) {
            continue; // fully behind
        }
        if poly.iter().any(|(c, _)| c.z < NEAR) {
            let mut clipped: Vec<(V3, V3)> = Vec::with_capacity(4);
            for i in 0..poly.len() {
                let (a, aw) = poly[i];
                let (b, bw) = poly[(i + 1) % poly.len()];
                if a.z >= NEAR {
                    clipped.push((a, aw));
                }
                if (a.z >= NEAR) != (b.z >= NEAR) {
                    let t = (NEAR - a.z) / (b.z - a.z);
                    clipped.push((a + (b - a) * t, aw + (bw - aw) * t));
                }
            }
            poly = clipped;
            if poly.len() < 3 {
                continue;
            }
        }
        // early frustum reject: whole polygon outside one screen edge
        if poly.iter().all(|(c, _)| c.x > c.z * tan_h)
            || poly.iter().all(|(c, _)| c.x < -c.z * tan_h)
            || poly.iter().all(|(c, _)| c.y > c.z * tan_v)
            || poly.iter().all(|(c, _)| c.y < -c.z * tan_v)
        {
            continue;
        }
        // degenerate reject (shading itself happens in the post-pass)
        if (p[1] - p[0]).cross(&(p[2] - p[0])).norm() < 1e-3 {
            continue;
        }
        // fan-triangulate the clipped polygon (3 or 4 verts)
        for k in 1..poly.len() - 1 {
            let sub = [poly[0], poly[k], poly[k + 1]];
            let scr: Vec<(f32, f32, f32)> = sub
                .iter()
                .map(|(c, _)| {
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
                    let (w0, w1, w2) = (l0 / scr[0].2, l1 / scr[1].2, l2 / scr[2].2);
                    let z = 1.0 / (w0 + w1 + w2);
                    let idx = py * W + px;
                    if z < depth[idx] {
                        depth[idx] = z;
                        tri_of[idx] = tri_idx as u32;
                        is_sky[idx] = false;
                    }
                }
            }
        }
    }

    // shading pass: reconstruct the world point per pixel, light it through
    // the shared `lit` (real sun + shadow ray), sky through the gradient
    {
        use crate::par::*;
        let idxs = scene.mesh.indices();
        let (depth, is_sky, tri_of) = (&depth, &is_sky, &tri_of);
        shade.par_chunks_mut(W).enumerate().for_each(|(y, row)| {
            for (x, out) in row.iter_mut().enumerate() {
                let i = y * W + x;
                let sx = (x as f32 + 0.5) / W as f32 * 2.0 - 1.0;
                let sy2 = 1.0 - (y as f32 + 0.5) / H as f32 * 2.0;
                let dir = fwd + right * (sx * tan_h) + up * (sy2 * tan_v);
                if is_sky[i] {
                    *out = sky_color(dir.normalize(), scene.sun, scene.sun_color);
                } else {
                    let t = idxs[tri_of[i] as usize];
                    let p = |k: usize| V3::new(vtx[t[k] as usize].x, vtx[t[k] as usize].y, vtx[t[k] as usize].z);
                    let n = (p(1) - p(0)).cross(&(p(2) - p(0))).normalize();
                    // dir has forward-component 1, so t = stored z is exact
                    *out = lit(scene, tri_of[i], n, eye + dir * depth[i], eye, true);
                }
            }
        });
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
            let (b, g, r) = (
                (shade[i][2].clamp(0.0, 1.0) * 255.0) as u8,
                (shade[i][1].clamp(0.0, 1.0) * 255.0) as u8,
                (shade[i][0].clamp(0.0, 1.0) * 255.0) as u8,
            );
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

    // HUD is layered in the browser (cards/hud.png) for full sharpness

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
    bmp
}


