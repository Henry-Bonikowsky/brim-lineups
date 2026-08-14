//! Inverse solver: every navmesh stand point x analytic ballistic aim, refined
//! by simulation, ranked by time-to-land with an aim-forgiveness measure.

use crate::scene::{Scene, V3};
use crate::par::*;
use crate::sim::{fly, Cfg};

/// Wall-clock phase timer; a no-op on wasm (Instant panics there).
pub(crate) struct Timer(#[cfg(not(target_arch = "wasm32"))] std::time::Instant);
impl Timer {
    pub(crate) fn new() -> Self {
        Timer(#[cfg(not(target_arch = "wasm32"))] std::time::Instant::now())
    }
    pub(crate) fn secs(&self) -> f32 {
        #[cfg(not(target_arch = "wasm32"))]
        return self.0.elapsed().as_secs_f32();
        #[cfg(target_arch = "wasm32")]
        0.0
    }
}

/// UI landmarks usable as aiming references (screen fractions of the HUD):
/// pixel-true elements only. Lineups whose aim puts one of these ON a world
/// silhouette edge are replicable in game without guesswork.
const UI_ANCHORS: [(&str, f32, f32); 18] = [
    // measured PIXEL-EXACT from cards/hud.png (1999x1249) - the overlay the
    // site composites, so anchor and drawn pixel agree by construction.
    // Henry: ALL sharp tips in the UI are usable anchors. The mouse crescent
    // is NOT here - it is his tool for lining up on non-point features.
    ("crosshair", 0.5, 0.5),
    ("chevron point below the equip prompt", 0.4660, 0.8855),
    ("white dot above the equip prompt", 0.4665, 0.8078),
    ("top tip of the E flame icon", 0.4652, 0.9119),
    ("bottom tip of the E flame icon", 0.4667, 0.9424),
    ("bottom tip of the Q icon", 0.4010, 0.9440),
    ("top of the Q icon", 0.4012, 0.9207),
    ("top of the MB4 icon", 0.5318, 0.9167),
    ("bottom tip of the MB4 icon", 0.5328, 0.9416),
    ("top of the Z icon", 0.5975, 0.9087),
    // Z bottom dropped: 41% of its surroundings are covered by the icon's own
    // graphics - corners aligned there hide behind HUD art
    ("left end of the Q charge bar", 0.3802, 0.9644),
    ("right end of the Q charge bar", 0.4222, 0.9640),
    ("left end of the E charge bar", 0.4457, 0.9644),
    ("right end of the E charge bar", 0.4877, 0.9640),
    ("left end of the MB4 charge pips", 0.5113, 0.9640),
    ("right end of the MB4 charge pips", 0.5533, 0.9640),
    ("left end of the X charge bar", 0.5763, 0.9640),
    ("right end of the X charge bar", 0.6188, 0.9640),
];

/// A reference is Henry's rule verbatim: a CLEAR POINT of the UI perfectly on
/// top of a CLEAR POINT in the view. "Clear point in the view" is a property
/// of the rendered IMAGE, so this samples the exact shading the aim pictures
/// use (texture x lambert x fog) in a patch around each anchor and runs a
/// Harris corner detector on those pixels. The response IS visual clarity -
/// no geometry proxies. Foliage pixels poison a corner (leaves render nothing
/// like in game). Returns candidates best-first as
/// ((name, depth, grade, anchor fx, anchor fy), corner (fx, fy)).
fn ui_reference_candidates(
    scene: &Scene,
    eye: V3,
    yaw: f32,
    pitch: f32,
    cfg: &Cfg,
    _s: f32,
) -> Vec<((&'static str, f32, u8, f32, f32), (f32, f32))> {
    let mut cands: Vec<(f32, ((&'static str, f32, u8, f32, f32), (f32, f32)))> = Vec::new();
    for (name, ax0, ay0) in UI_ANCHORS {
        // apply the user's HUD calibration: scale about bottom-center + dy
        let (ax, ay) = if name == "crosshair" {
            (ax0, ay0) // crosshair is the aim point, never moved by UI scale
        } else {
            (
                0.5 + (ax0 - 0.5) * cfg.hud_scale,
                1.0 - (1.0 - ay0) * cfg.hud_scale + cfg.hud_dy / 100.0,
            )
        };
        if let Some((cx, cy, resp, dist)) = best_corner(scene, eye, yaw, pitch, ax, ay, 0.0143, 0.0004) {
            let grade = if ((cx - ax).powi(2) + (cy - ay).powi(2)).sqrt() < 0.005 { 2u8 } else { 1 };
            // the crosshair is the most natural anchor to line up
            let score = resp * if name == "crosshair" { 1.5 } else { 1.0 };
            cands.push((score, ((name, dist, grade, ax, ay), (cx, cy))));
        }
    }
    cands.sort_by(|a, b| b.0.total_cmp(&a.0));
    cands.into_iter().map(|(_, c)| c).collect()
}

/// Strongest visually-clear corner in a patch of the rendered view centered
/// on (ax, ay): sample luminance on a grid (`step` screen fractions per
/// pixel, `half` fractions half-width), Sobel gradients, Harris response
/// smoothed 5x5, foliage-poisoned windows excluded. Returns
/// (fx, fy, response, depth) of the best corner above the clarity floor.
/// Strongest visually-clear corner in a patch of the rendered view centered
/// on (ax, ay). A clear corner is where the silhouette BOUNDARY between the
/// near surface and its background (sky or much deeper geometry) forms a
/// wedge: on a straight edge a disc around a boundary cell is ~50% near; at
/// a corner it deviates hard (90 deg = 25%/75%). Requiring the deviation to
/// hold at two radii rejects short notches (only corners of LONG edges
/// survive), and the luminance step across the boundary must be visible.
/// Foliage anywhere near a corner poisons it. Returns
/// (fx, fy, score, depth) of the best corner.
fn best_corner(
    scene: &Scene,
    eye: V3,
    yaw: f32,
    pitch: f32,
    ax: f32,
    ay: f32,
    half: f32,
    step: f32,
) -> Option<(f32, f32, f32, f32)> {
    use parry3d::query::{Ray, RayCast};
    let (sy, cy) = yaw.to_radians().sin_cos();
    let (sp, cp) = pitch.to_radians().sin_cos();
    let fwd = V3::new(cp * cy, cp * sy, sp);
    let right = V3::new(-sy, cy, 0.0);
    let up = fwd.cross(&right).normalize();
    let tan_v = (103.0f32.to_radians() / 2.0).tan() * 9.0 / 16.0;
    let tan_h = tan_v * 1.6;
    let id = nalgebra::Isometry3::identity();
    let ntris = scene.mesh.indices().len() as u32;
    let n = ((half / step) as usize) * 2 + 1;
    let c0 = (n / 2) as f32;
    // one patch pixel: (luminance, foliage, depth)
    let lum_of = |c: [f32; 3]| 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2];
    let sample = |fx: f32, fy: f32| -> (f32, bool, f32) {
        let d = (fwd + right * ((fx * 2.0 - 1.0) * tan_h) + up * ((1.0 - fy * 2.0) * tan_v)).normalize();
        match scene.mesh.cast_ray_and_get_normal(&id, &Ray::new(nalgebra::Point3::from(eye), d), 5.0e4, true) {
            Some(h) => {
                let tri = match h.feature {
                    parry3d::shape::FeatureId::Face(f) => f % ntris.max(1),
                    _ => 0,
                };
                // the render's exact shading minus the shadow ray: references
                // anchor on geometry/material contrast, never on our
                // synthetic shadows (the game's baked ones differ)
                let c = crate::render::lit(scene, tri, h.normal, eye + d * h.time_of_impact, eye, false);
                (lum_of(c), scene.foliage_at(tri), h.time_of_impact)
            }
            None => (lum_of(crate::render::sky_color(d, scene.sun, scene.sun_color)), false, f32::INFINITY),
        }
    };
    // cheap 9x9 pre-gate: a mask (and thus a corner) needs sky in the patch
    // (>3.3% for the silhouette mask) or a luminance split >=0.12 (two-means
    // mask). A coarse scan finding NO sky and a spread under 0.05 means only
    // sub-percent features could flip that, and those fail the two-radii
    // long-edge rule anyway - skip the dense scan (73x73 rays) entirely.
    // Measured ~1.75x on refine (Ascent browse: solve 2.1s -> 1.2s).
    {
        let (mut lo, mut hi, mut sky) = (f32::MAX, f32::MIN, false);
        for j in 0..9 {
            for i in 0..9 {
                let (l, _, d) =
                    sample(ax + (i as f32 - 4.0) / 4.0 * half, ay + (j as f32 - 4.0) / 4.0 * half);
                sky |= !d.is_finite();
                lo = lo.min(l);
                hi = hi.max(l);
            }
        }
        if !sky && hi - lo < 0.05 {
            return None;
        }
    }
    // rendered luminance + foliage + depth per patch pixel
    let mut lum = vec![0.0f32; n * n];
    let mut leaf = vec![false; n * n];
    let mut dep = vec![f32::INFINITY; n * n];
    for j in 0..n {
        for i in 0..n {
            let (l, f, d) = sample(ax + (i as f32 - c0) * step, ay + (j as f32 - c0) * step);
            lum[j * n + i] = l;
            leaf[j * n + i] = f;
            dep[j * n + i] = d;
        }
    }
    // two masks, best corner wins across both:
    // 1) sky/solid from ray misses - pixel-EXACT for silhouettes (luminance
    //    clustering pulled bright wall pixels into the sky class and put tips
    //    pixels off);
    // 2) two-means luminance split - the mask the eye builds for interior
    //    contrast lines (two overlapping roofs are both "near" by depth but
    //    their contrast line is plain to see).
    let mut masks: Vec<Vec<bool>> = Vec::new();
    let sky_cells = dep.iter().filter(|d| !d.is_finite()).count();
    if sky_cells * 30 > n * n {
        masks.push(dep.iter().map(|d| !d.is_finite()).collect());
    }
    let mut t = 0.5f32;
    let (mut m_lo, mut m_hi) = (0.0f32, 0.0f32);
    let mut ok = true;
    for _ in 0..8 {
        let (mut slo, mut nlo, mut shi, mut nhi) = (0.0f32, 0i32, 0.0f32, 0i32);
        for &l in &lum {
            if l < t {
                slo += l;
                nlo += 1;
            } else {
                shi += l;
                nhi += 1;
            }
        }
        if nlo == 0 || nhi == 0 {
            ok = false;
            break;
        }
        (m_lo, m_hi) = (slo / nlo as f32, shi / nhi as f32);
        t = (m_lo + m_hi) * 0.5;
    }
    if ok && m_hi - m_lo >= 0.12 {
        masks.push(lum.iter().map(|&l| l >= t).collect());
    }
    if masks.is_empty() {
        return None; // uniform patch: nothing to line up on
    }
    let mut overall: Option<(f32, f32, f32, f32)> = None;
    for (mk, far) in masks.iter().enumerate() {
        // the sky/solid mask (index 0 when present) is pixel-exact: prefer
        // its corners over luminance-cluster ones at similar strength
        let mask_boost = if mk == 0 && sky_cells * 30 > n * n { 1.5 } else { 1.0 };
        let far = far.as_slice();
    let (r0, r1, r2) = (2i32, 5i32, 10i32);
    let mut best: Option<(f32, usize, usize)> = None;
    let ni = n as i32;
    for j in (r2 as usize)..n - r2 as usize {
        for i in (r2 as usize)..n - r2 as usize {
            let here = far[j * n + i];
            // boundary cells only - and the boundary must be a real visible
            // EDGE right here (hard local luminance step), not the soft line
            // where a shading gradient crosses the two-means threshold
            let mut local_step = 0.0f32;
            for (dx, dy) in [(1i32, 0i32), (-1, 0), (0, 1), (0, -1)] {
                let idx = ((j as i32 + dy) * ni + i as i32 + dx) as usize;
                if far[idx] != here {
                    local_step = local_step.max((lum[idx] - lum[j * n + i]).abs());
                }
            }
            if local_step < 0.10 {
                continue;
            }
            // wedge fractions at two radii + foliage poison + contrast
            let mut ok = true;
            let mut devs = [0.0f32; 3];
            let mut lum_near = (0.0f32, 0i32);
            let mut lum_far = (0.0f32, 0i32);
            for (k, r) in [r0, r1, r2].into_iter().enumerate() {
                let (mut nn, mut nt) = (0i32, 0i32);
                for dj in -r..=r {
                    for di in -r..=r {
                        if di * di + dj * dj > r * r {
                            continue;
                        }
                        let idx = ((j as i32 + dj) * ni + i as i32 + di) as usize;
                        if leaf[idx] {
                            ok = false;
                        }
                        nt += 1;
                        if !far[idx] {
                            nn += 1;
                            if k == 1 {
                                lum_near.0 += lum[idx];
                                lum_near.1 += 1;
                            }
                        } else if k == 1 {
                            lum_far.0 += lum[idx];
                            lum_far.1 += 1;
                        }
                    }
                }
                devs[k] = (nn as f32 / nt as f32 - 0.5).abs();
            }
            if !ok || devs[0] < 0.12 || devs[1] < 0.15 || devs[2] < 0.12 {
                continue; // straight, ROUNDED (soft up close), short, or leafy
            }
            let contrast = if lum_near.1 > 0 && lum_far.1 > 0 {
                (lum_near.0 / lum_near.1 as f32 - lum_far.0 / lum_far.1 as f32).abs()
            } else {
                0.0
            };
            if contrast < 0.12 {
                continue; // geometrically sharp, optically invisible
            }
            let score = (devs[1] + devs[2]) * contrast;
            if score > CORNER_FLOOR && best.is_none_or(|(bs, _, _)| score > bs) {
                best = Some((score, i, j));
            }
        }
    }
    let this = best.map(|(sc, i, j)| {
        // display depth: nearest solid hit around the corner
        let mut d = f32::INFINITY;
        for wj in j - 2..=j + 2 {
            for wi in i - 2..=i + 2 {
                d = d.min(dep[wj * n + wi]);
            }
        }
        // snap to the wedge TIP: the perceptual point is the extremum where
        // the minority region penetrates deepest (a pocket's deepest pixel, a
        // peak's outermost pixel), not the wedge centroid ("close" is not on)
        let (mut fi, mut fj) = (i as i32, j as i32);
        {
            // minority class around the corner
            let (mut nn, mut nt) = (0i32, 0i32);
            for dj in -r1..=r1 {
                for di in -r1..=r1 {
                    if di * di + dj * dj > r1 * r1 {
                        continue;
                    }
                    nt += 1;
                    if !far[((j as i32 + dj) * ni + i as i32 + di) as usize] {
                        nn += 1;
                    }
                }
            }
            let minority_far = nn * 2 > nt; // near majority -> minority is far
            let mut best_tip: Option<(f32, i32, i32)> = None;
            for dj in -r2..=r2 {
                for di in -r2..=r2 {
                    let (wi, wj) = (i as i32 + di, j as i32 + dj);
                    if wi < 3 || wj < 3 || wi >= ni - 3 || wj >= ni - 3 {
                        continue;
                    }
                    if far[(wj * ni + wi) as usize] != minority_far {
                        continue;
                    }
                    // minority fraction in two tight discs: tip = most
                    // surrounded; the r=2 disc breaks ties to the exact pixel
                    let (mut mn3, mut mt3, mut mn2, mut mt2) = (0i32, 0i32, 0i32, 0i32);
                    for tj in -3i32..=3 {
                        for ti in -3i32..=3 {
                            let rr = ti * ti + tj * tj;
                            if rr > 9 {
                                continue;
                            }
                            let m = far[((wj + tj) * ni + wi + ti) as usize] == minority_far;
                            mt3 += 1;
                            if m {
                                mn3 += 1;
                            }
                            if rr <= 4 {
                                mt2 += 1;
                                if m {
                                    mn2 += 1;
                                }
                            }
                        }
                    }
                    let frac = mn3 as f32 / mt3 as f32 + 0.1 * (mn2 as f32 / mt2 as f32);
                    if best_tip.is_none_or(|(bf, _, _)| frac < bf) {
                        best_tip = Some((frac, wi, wj));
                    }
                }
            }
            if let Some((_, wi, wj)) = best_tip {
                (fi, fj) = (wi, wj);
            }
        }
        (ax + (fi as f32 - c0) * step, ay + (fj as f32 - c0) * step, sc, d)
    });
    if let Some(mut c) = this {
        c.2 *= mask_boost;
        // atmospheric perspective: a corner at 60m is fog-washed and reads
        // borderline even when geometrically sharp - prefer near corners
        if c.3.is_finite() {
            c.2 /= 1.0 + c.3 / 6000.0;
        }
        if overall.is_none_or(|o| c.2 > o.2) {
            overall = Some(c);
        }
    }
    }
    overall
}

/// Minimum Harris response for a corner to count as CLEAR. Calibrated on
/// Henry's review verdicts: the approved rooftop-vs-sky corner scores far
/// above this; the rejected statue tip, gray-on-gray seam and lone tower
/// edge score below.
const CORNER_FLOOR: f32 = 0.05;

/// Nudge a covered lineup's aim by sub-degree steps until a UI anchor point
/// sits EXACTLY on a clear corner of the rendered view (the in-game "kiss the
/// corner" motion), keeping the throw only if it still covers `target`.
/// Candidates best-first; if the throw cannot afford the strongest corner's
/// nudge, fall back to the next. References are VISUAL: everything probes the
/// visual scene; only the throw physics uses collision.
fn align_reference(scene: &Scene, vis: Option<&Scene>, origin: V3, l: &mut Lineup, target: V3, tol: f32, cfg: &Cfg) -> bool {
    let vscene = vis.unwrap_or(scene);
    let tan_v = (103.0f32.to_radians() / 2.0).tan() * 9.0 / 16.0;
    let tan_h = tan_v * 1.6;
    for ((name, dist, _, ax, ay), (cx0, cy0)) in
        ui_reference_candidates(vscene, origin, l.yaw, l.pitch, cfg, 0.0).into_iter().take(6)
    {
        let (mut yaw, mut aim) = (l.yaw, l.pitch);
        let (mut cx, mut cy) = (cx0, cy0);
        // already lined up at the row's own angle: take it, zero physics risk
        if (cx0 - ax).abs() < 0.0003 && (cy0 - ay).abs() < 0.0003 {
            l.ui_ref = Some((name, dist, 2, ax, ay));
            return true;
        }
        let mut aligned = false;
        for it in 0..8 {
            let (dfx, dfy) = (cx - ax, cy - ay);
            if dfx.abs() < 0.0003 && dfy.abs() < 0.0003 {
                aligned = true; // within ~0.6 native px
                break;
            }
            // small-angle screen->camera: turning right shifts the world left
            yaw += (dfx * 2.0 * tan_h).to_degrees();
            aim += (-dfy * 2.0 * tan_v).to_degrees();
            // re-find the corner at the nudged aim. SAME native-pixel step as
            // detection: Harris at sub-pixel sampling hallucinates corners on
            // razor edges (full-contrast jumps between adjacent cells at any
            // step), so scale never changes; sub-pixel precision comes from
            // the edge-line intersection refinement instead
            let _ = it;
            let Some((bx, by, _, _)) = best_corner(vscene, origin, yaw, aim, ax, ay, 0.0143, 0.0004) else {
                break;
            };
            (cx, cy) = (bx, by);
        }
        // the covers re-check is the real safety net; the drift guard only
        // rejects runaway steering
        if !aligned || (yaw - l.yaw).abs() + (aim - l.pitch).abs() > 3.0 {
            continue;
        }
        let launch = crate::sim::launch_pitch(aim, cfg);
        let Some(o) = crate::sim::fly(scene, crate::sim::hand_origin(origin, yaw, cfg), dir_from(yaw, launch), cfg)
        else {
            continue;
        };
        let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
        let dz = (o.rest.z - target.z).abs();
        let err = if dz <= 220.0 { dxy } else { dxy + (dz - 220.0) * 3.0 };
        if !(err < tol && crate::sim::fire_covers(scene, o.rest, target)) {
            continue; // this reference costs too much aim: try the next
        }
        l.yaw = yaw;
        l.pitch = aim;
        l.rest = o.rest;
        l.time = o.time;
        l.bounces = o.bounces;
        l.err = err;
        l.ui_ref = Some((name, dist, 2, ax, ay));
        return true;
    }
    false
}

#[derive(Clone)]
pub struct Lineup {
    pub dist: f32, // stand-to-target range (lineups are long throws, not tosses)
    pub stand: V3,
    pub rest: V3,        // where the fire actually comes to rest
    pub yaw: f32,        // player aim yaw (deg, UE convention: atan2(y, x))
    pub pitch: f32,      // player aim pitch (deg; launch pitch minus arc knob)
    pub time: f32,
    pub bounces: u32,
    pub err: f32,        // rest-to-target distance
    pub covered: bool,   // fire actually SPREADS to the target (box-aware)
    pub forgive: f32,    // fraction of +-0.75 deg jitters still within tol
    pub spread: f32,     // worst landing deviation across those jitters (fragility)
    pub pos_forgive: f32, // fraction of ~75u stand shifts (same aim) still covering
    pub wedged: bool,    // stand is corner-pinned (always true for solver-picked stands)
    pub aim_ref: Option<(V3, f32)>, // crosshair reference: first geometry the aim ray hits
    /// UI landmark sitting on a world edge at this aim:
    /// (anchor, edge dist, grade, screen fx, screen fy)
    pub ui_ref: Option<(&'static str, f32, u8, f32, f32)>,
}

impl Lineup {
    /// 2 = corner + position-forgiving (rough standing works), 1 = press W
    /// into the corner first, then it is exact, 0 = free right-clicked stand
    /// with no corner in reach (stand exactly on the dot).
    pub fn pos_grade(&self) -> u8 {
        if !self.wedged {
            0
        } else if self.pos_forgive >= 0.75 {
            2
        } else {
            1
        }
    }
}

fn dir_from(yaw_deg: f32, pitch_deg: f32) -> V3 {
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    V3::new(cp * cy, cp * sy, sp)
}

/// Vacuum-ballistics launch angles for speed s, gravity g, horizontal dist d,
/// height diff h: tan(theta) = (s^2 +- sqrt(s^4 - g(g d^2 + 2 h s^2))) / (g d).
fn launch_angles(s: f32, g: f32, d: f32, h: f32) -> Vec<f32> {
    let disc = s.powi(4) - g * (g * d * d + 2.0 * h * s * s);
    if disc < 0.0 || d < 1.0 {
        return vec![];
    }
    let r = disc.sqrt();
    [(s * s - r) / (g * d), (s * s + r) / (g * d)]
        .iter()
        .map(|t| t.atan().to_degrees())
        .collect()
}

/// strict=true: full-map hunt (covered + hidden-from-site gates, one best per
/// stand, deduped, ranked by mid-range preference + time). strict=false
/// (paired mode, one locked stand): every distinct working ANGLE FAMILY from
/// that stand (pitch buckets), ranked by time.
/// browse=true (strict only): keep every lineup RESTING within tol of the
/// click even when its fire cannot reach the click itself - the user browses
/// nearby end locations and picks one. Sorted by landing distance.
pub fn solve(scene: &Scene, vis: Option<&Scene>, stands: &[V3], target: V3, tol: f32, min_dist: f32, strict: bool, browse: bool, cfg: &Cfg) -> Vec<Lineup> {
    solve_impl(scene, vis, stands, target, tol, min_dist, strict, browse, cfg, true)
}

/// `align_all=false` (refine pass): reference alignment - the expensive
/// per-anchor corner scans - runs only for the fastest covered family per
/// stand, since the caller keeps just that one. Everything else identical.
#[allow(clippy::too_many_arguments)]
fn solve_impl(scene: &Scene, vis: Option<&Scene>, stands: &[V3], target: V3, tol: f32, min_dist: f32, strict: bool, browse: bool, cfg: &Cfg, align_all: bool) -> Vec<Lineup> {
    let paired = !strict && stands.len() == 1;
    // POSITION RULE (strict/solver-picked stands only): a lineup stand must
    // sit against TWO faces (wall corner, angled walls, object against wall)
    // so pressing W into it stops the player on the same spot every time.
    // Snap every candidate to its capsule-pinned corner position; anything
    // with no wedge in reach is out. A RIGHT-CLICKED stand is the user's
    // choice: snap to a corner when one is in reach (repeatability for free),
    // otherwise take the exact spot and label it pos 0.
    let mut free_stand = false;
    let t_pin = Timer::new();
    let stands: Vec<V3> = if strict {
        let n_in = stands.len();
        let pinned: Vec<V3> = stands.par_iter().filter_map(|s| wedge_stand(scene, *s)).collect();
        let mut seen = std::collections::HashSet::new();
        let out: Vec<V3> = pinned
            .into_iter()
            .filter(|p| seen.insert(((p.x / 25.0).round() as i64, (p.y / 25.0).round() as i64)))
            .collect();
        if n_in > 1 {
            eprintln!("stands: {n_in} candidates -> {} wedge-pinned", out.len());
        }
        out
    } else {
        stands
            .iter()
            .map(|s| {
                wedge_stand(scene, *s).unwrap_or_else(|| {
                    free_stand = true;
                    *s
                })
            })
            .collect()
    };
    let wedged = !free_stand;
    if strict {
        eprintln!("[t] pin {:.2}s", t_pin.secs());
    }
    let t_sweep = Timer::new();
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    let (n_none, n_far, n_near) = (AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0));
    let max_range = cfg.speed * cfg.speed / cfg.gravity * 1.05;
    let mut all: Vec<Lineup> = stands
        .par_iter()
        .flat_map_iter(|stand| {
            let origin = stand + V3::new(0.0, 0.0, cfg.eye_z);
            let delta = target - origin;
            let d = (delta.x * delta.x + delta.y * delta.y).sqrt();
            if d > max_range || d < min_dist {
                return vec![].into_iter();
            }
            let yaw0 = delta.y.atan2(delta.x).to_degrees();
            // paired mode: ONE stand is cheap, so scan the full angle space
            // exhaustively (finds arch-threads and skims the ballistic anchor
            // misses) and keep the best lineup per ~8 deg pitch family
            let mut best: Option<Lineup> = None;
            let mut best_miss: Option<Lineup> = None;
            let mut families: std::collections::HashMap<i32, Lineup> = Default::default();
            // covered (time, yaw, aim pitch) alternates per family for the
            // reference-hunting fallback
            let mut alts: std::collections::HashMap<i32, Vec<(f32, f32, f32)>> = Default::default();
            let sweeps: Vec<(f32, f32)> = if paired {
                let mut v = Vec::new();
                let mut pitch = -35.0f32;
                while pitch <= 85.0 {
                    let mut dy = -6.0f32;
                    while dy <= 6.0 {
                        v.push((yaw0 + dy, pitch));
                        dy += 1.0;
                    }
                    pitch += 1.25;
                }
                v
            } else {
                let mut v = Vec::new();
                for (ai, base) in launch_angles(cfg.speed, cfg.gravity, d, delta.z).into_iter().enumerate() {
                    // refine around the vacuum solution: geometry + bounces move
                    // the landing; the LOW arc gets a wider downward sweep for
                    // fast skim-bounce throws
                    let lo = if ai == 0 { -10 } else { -4 };
                    for dp in lo..=4 {
                        for dy in -2..=2 {
                            v.push((yaw0 + dy as f32, base + dp as f32));
                        }
                    }
                }
                // broad coarse families: bounce-assisted and off-family throws
                // live far from the vacuum solutions and a narrow sweep never
                // DISCOVERS their stands at all ("strict in the wrong way").
                // The per-stand refine pass polishes whatever this finds
                let mut pitch = -25.0f32;
                while pitch <= 80.0 {
                    for dy in [-3.0f32, 0.0, 3.0] {
                        v.push((yaw0 + dy, pitch));
                    }
                    pitch += 5.0;
                }
                v
            };
            {
                for (yaw, pitch) in sweeps {
                    {
                        if pitch <= -89.0 || pitch >= 89.0 {
                            continue;
                        }
                        let Some(o) = fly(scene, crate::sim::hand_origin(origin, yaw, cfg), dir_from(yaw, pitch), cfg) else {
                            n_none.fetch_add(1, Relaxed);
                            continue;
                        };
                        // success = the FIRE covers the clicked spot: rest within
                        // the 450u patch radius horizontally and within the
                        // fire's vertical reach (ZLayerTolerance 200, StepUp 110
                        // / StepDown 210 from the patch files)
                        let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
                        let dz = (o.rest.z - target.z).abs();
                        let err = if dz <= 220.0 { dxy } else { dxy + (dz - 220.0) * 3.0 };
                        // distance alone is not success: the fire must SPREAD
                        // to the click (a >110u box between rest and click
                        // blocks it even at 2m)
                        // fire_covers BFS caps spread travel at 450u, so even
                        // with a wide browse tol `covered` stays honest
                        let covered = err < tol && crate::sim::fire_covers(scene, o.rest, target);
                        let sel = covered || (browse && err < tol);
                        if covered { &n_near } else { &n_far }.fetch_add(1, Relaxed);
                        // covered candidates get scored for a UI reference: a
                        // lineup you can replicate off a landmark beats a
                        // slightly faster one aimed at featureless sky
                        let ui_ref = None; // references come from align_reference on final rows
                        let cand = Lineup {
                            dist: d,
                            stand: *stand,
                            rest: o.rest,
                            yaw,
                            pitch: crate::sim::aim_pitch(pitch, cfg),
                            time: o.time,
                            bounces: o.bounces,
                            err,
                            covered,
                            forgive: 0.0,
                            spread: 0.0,
                            pos_forgive: 0.0,
                            wedged,
                            aim_ref: None,
                            ui_ref,
                        };
                        if !sel {
                            if paired && best_miss.as_ref().is_none_or(|b| cand.err < b.err) {
                                best_miss = Some(cand);
                            }
                            continue;
                        }
                        let grade = |l: &Lineup| l.ui_ref.map_or(0u8, |(_, _, g, _, _)| g);
                        if paired {
                            let key = (pitch / 8.0).round() as i32;
                            // remember covered alternates: when the fastest
                            // angle can't reach any reference, align walks
                            // these (a player scans working angles until a
                            // feature meets an anchor)
                            if cand.covered {
                                let a = alts.entry(key).or_default();
                                a.push((cand.time, cand.yaw, cand.pitch));
                                if a.len() > 16 {
                                    a.sort_by(|x, y| x.0.total_cmp(&y.0));
                                    a.truncate(12);
                                }
                            }
                            // a UI reference is worth at most 0.4s: beyond
                            // that, speed wins - an unconditional grade
                            // preference was discarding fast bounce-assisted
                            // throws in favor of slow aimable ones
                            let replace = match families.get(&key) {
                                Some(cur) => {
                                    if (cand.time - cur.time).abs() > 0.4 {
                                        cand.time < cur.time
                                    } else {
                                        grade(&cand) > grade(cur)
                                            || (grade(&cand) == grade(cur) && cand.time < cur.time)
                                    }
                                }
                                None => true,
                            };
                            if replace {
                                families.insert(key, cand);
                            }
                        } else if best.as_ref().is_none_or(|b| cand.time < b.time) {
                            best = Some(cand);
                        }
                    }
                }
            }
            if paired {
                let mut out: Vec<Lineup> = families.into_values().collect();
                if out.is_empty() {
                    // no throw lands within tolerance: report the closest miss so
                    // the user sees WHY (err > tol labels it)
                    out.extend(best_miss);
                }
                // align_all=false: only the fastest covered family gets the
                // (expensive) reference alignment - it is the only row the
                // refine caller keeps
                let align_idx: Option<usize> = if align_all {
                    None
                } else {
                    out.iter()
                        .enumerate()
                        .filter(|(_, l)| l.covered)
                        .min_by(|a, b| a.1.time.total_cmp(&b.1.time))
                        .map(|(i, _)| i)
                };
                for (fi, b) in out.iter_mut().enumerate() {
                    let want_ref = b.covered && (align_all || align_idx == Some(fi));
                    // snap the aim so the reference point sits exactly on its
                    // feature. A reference is only real when the ANGLE is
                    // perfectly aligned - if the snap fails (or the row is a
                    // miss), showing the near-miss detection would be a lie
                    let mut refd = want_ref && align_reference(scene, vis, origin, b, target, tol, cfg);
                    if !refd && want_ref {
                        // no reference at the fastest angle: walk the covered
                        // alternates of this family (nearest time first, max
                        // +0.5s) until one of them aligns onto a feature
                        let key = (crate::sim::launch_pitch(b.pitch, cfg) / 8.0).round() as i32;
                        if let Some(a) = alts.get_mut(&key) {
                            a.sort_by(|x, y| x.0.total_cmp(&y.0));
                            for &(t, y, p) in a.iter().filter(|(t, ..)| *t <= b.time + 0.8).take(16) {
                                if (y - b.yaw).abs() + (p - b.pitch).abs() < 0.05 {
                                    continue; // the angle we already tried
                                }
                                let mut c = b.clone();
                                (c.yaw, c.pitch, c.time) = (y, p, t);
                                if align_reference(scene, vis, origin, &mut c, target, tol, cfg) {
                                    *b = c;
                                    refd = true;
                                    break;
                                }
                            }
                        }
                    }
                    if !refd {
                        b.ui_ref = None;
                    }
                    finish(scene, target, tol, cfg, origin, b);
                }
                out.sort_by(|a, b| a.time.total_cmp(&b.time));
                return out.into_iter();
            }
            if let Some(b) = &mut best {
                // a LINEUP is thrown from cover: nobody standing anywhere around
                // the site may have line of sight to the thrower. Sample defender
                // eyes on rings around the target; ANY clear sightline kills it.
                use parry3d::query::{Ray, RayCast};
                let mut site_eyes = vec![target + V3::new(0.0, 0.0, 160.0)];
                for (ring, n) in [(350.0f32, 6usize), (650.0, 8)] {
                    for k in 0..n {
                        let a = k as f32 / n as f32 * std::f32::consts::TAU;
                        site_eyes.push(target + V3::new(a.cos() * ring, a.sin() * ring, 160.0));
                    }
                }
                let exposed = strict && site_eyes.iter().any(|se| {
                    let v = se - origin;
                    let d = v.norm();
                    let sight = Ray::new(nalgebra::Point3::from(origin), v / d);
                    scene
                        .mesh
                        .cast_ray(&nalgebra::Isometry3::identity(), &sight, d - 60.0, true)
                        .is_none()
                });
                if exposed {
                    return vec![].into_iter();
                }
                // browse: forgiveness/anchoring are about reliably hitting the
                // lineup's OWN landing spot (the user picked it by end
                // location), not about covering the original click
                if browse {
                    let rest = b.rest;
                    finish(scene, rest, 450.0, cfg, origin, b);
                } else {
                    finish(scene, target, tol, cfg, origin, b);
                }
            }
            best.into_iter().collect::<Vec<_>>().into_iter()
        })
        .collect();

    if strict || !paired {
        eprintln!(
            "flights: {} never stopped, {} landed far, {} within tol [t] sweep {:.2}s",
            n_none.load(Relaxed),
            n_far.load(Relaxed),
            n_near.load(Relaxed),
            t_sweep.secs()
        );
    }
    // a lineup the solver itself rates near-zero forgiveness is untrustworthy
    // (tiny aim error cascades, e.g. clipping a sloped roof); prefer sturdy ones
    // the all-or-nothing forgiveness cliff hides otherwise-valid rows the
    // moment one sturdy lineup exists; in browse the user picks from a list
    // with forgiveness shown per row, so show everything there
    let sturdy = |v: &mut Vec<Lineup>| {
        if !browse && v.iter().any(|l| l.forgive >= 0.25) {
            v.retain(|l| l.forgive >= 0.25);
        }
    };
    if paired {
        // angle families from the locked stand, fastest first; easy positions first
        sturdy(&mut all);
        all.sort_by(|a, b| (b.pos_grade(), a.time).partial_cmp(&(a.pos_grade(), b.time)).unwrap());
        return all;
    }
    // dedup: one lineup per 200u XY cell, keep the fastest
    let mut by_cell: std::collections::HashMap<(i64, i64), Lineup> = Default::default();
    for l in all.drain(..) {
        let key = ((l.stand.x / 200.0) as i64, (l.stand.y / 200.0) as i64);
        match by_cell.get(&key) {
            Some(cur) if cur.time <= l.time => {}
            _ => {
                by_cell.insert(key, l);
            }
        }
    }
    let mut out: Vec<Lineup> = by_cell.into_values().collect();
    sturdy(&mut out);
    // rank: browse = nearest landing first (user picks by end location);
    // else easy positions first, then mid-range preference (~3000u) + speed.
    // Stand coords break ties so repeat solves order identically (the n-th
    // row must be the same lineup when the picker re-requests it by index)
    let key = |l: &Lineup| {
        if browse {
            l.err
        } else {
            (l.dist - 3000.0).abs() / 1500.0 + l.time * 0.35 - l.pos_grade() as f32 * 10.0
        }
    };
    out.sort_by(|a, b| {
        key(a).total_cmp(&key(b)).then(a.stand.x.total_cmp(&b.stand.x)).then(a.stand.y.total_cmp(&b.stand.y))
    });
    // REFINE: the coarse per-stand sweep proves a stand WORKS, not that its
    // angle is optimal (walk mode found better angles from the same spot in
    // seconds). Re-run the exhaustive paired sweep from each surviving stand
    // and keep its true fastest covered throw; browse rows optimize hitting
    // their OWN landing spot
    out.truncate(24);
    let t_refine = Timer::new();
    let mut out: Vec<Lineup> = out
        .par_iter()
        .map(|l| {
            let (rt, rtol) = if browse { (l.rest, 450.0) } else { (target, tol) };
            let fams = solve_impl(scene, vis, &[l.stand], rt, rtol, 0.0, false, false, cfg, false);
            let mut cov: Vec<Lineup> = fams.into_iter().filter(|f| f.covered).collect();
            cov.sort_by(|a, b| a.time.total_cmp(&b.time));
            // the aligned family (the only one carrying a reference) wins the
            // usual <=0.4s reference preference over a barely-faster raw row
            let pick = cov
                .iter()
                .position(|f| f.ui_ref.is_some())
                .filter(|&i| cov[i].time - cov[0].time <= 0.4)
                .unwrap_or(0);
            match (!cov.is_empty()).then(|| cov.swap_remove(pick)) {
                Some(mut f) => {
                    // err stays relative to the original click for display/sort
                    let dxy = ((f.rest.x - target.x).powi(2) + (f.rest.y - target.y).powi(2)).sqrt();
                    let dz = (f.rest.z - target.z).abs();
                    f.err = if dz <= 220.0 { dxy } else { dxy + (dz - 220.0) * 3.0 };
                    f
                }
                None => l.clone(),
            }
        })
        .collect();
    eprintln!("[t] refine {:.2}s", t_refine.secs());
    out.sort_by(|a, b| {
        key(a).total_cmp(&key(b)).then(a.stand.x.total_cmp(&b.stand.x)).then(a.stand.y.total_cmp(&b.stand.y))
    });
    out
}

/// Post-processing shared by both modes: crosshair reference point + aim
/// forgiveness for a confirmed lineup.
fn finish(scene: &Scene, target: V3, tol: f32, cfg: &Cfg, origin: V3, b: &mut Lineup) {
    use parry3d::query::{Ray, RayCast};
    let aim_dir = dir_from(b.yaw, b.pitch);
    let ray = Ray::new(nalgebra::Point3::from(origin), aim_dir);
    b.aim_ref = scene
        .mesh
        .cast_ray(&nalgebra::Isometry3::identity(), &ray, 5.0e4, true)
        .map(|t| (origin + aim_dir * t, t));
    // no ui_ref backfill here: a reference is only real when align_reference
    // snapped the angle exactly onto it - near-miss detection is not a ref
    let mut ok = 0;
    let mut worst = 0.0f32;
    for (jy, jp) in
        [(0.75, 0.0), (-0.75, 0.0), (0.0, 0.75), (0.0, -0.75), (0.75, 0.75), (-0.75, 0.75), (0.75, -0.75), (-0.75, -0.75)]
    {
        let launch_pitch = crate::sim::launch_pitch(b.pitch + jp, cfg);
        if let Some(o) = fly(scene, crate::sim::hand_origin(origin, b.yaw + jy, cfg), dir_from(b.yaw + jy, launch_pitch), cfg) {
            let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
            let dz = (o.rest.z - target.z).abs();
            let dev = if dz <= 220.0 { dxy } else { dxy + (dz - 220.0) * 3.0 };
            worst = worst.max(dev);
            if dev < tol && crate::sim::fire_covers(scene, o.rest, target) {
                ok += 1;
            }
        } else {
            worst = worst.max(9999.0);
        }
    }
    b.forgive = ok as f32 / 8.0;
    b.spread = worst;

    // POSITION forgiveness: shift the stand ~75u with the SAME aim; if the fire
    // still covers, exact positioning does not matter ("easy position")
    let covers = |o: &crate::sim::Outcome| {
        let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
        let dz = (o.rest.z - target.z).abs();
        (if dz <= 220.0 { dxy } else { dxy + (dz - 220.0) * 3.0 }) < tol
            && crate::sim::fire_covers(scene, o.rest, target)
    };
    let launch = dir_from(b.yaw, crate::sim::launch_pitch(b.pitch, cfg));
    let mut pok = 0;
    for (ox, oy) in [(75.0f32, 0.0), (-75.0, 0.0), (0.0, 75.0), (0.0, -75.0), (55.0, 55.0), (-55.0, 55.0), (55.0, -55.0), (-55.0, -55.0)] {
        let o2 = crate::sim::hand_origin(origin, b.yaw, cfg) + V3::new(ox, oy, 0.0);
        if fly(scene, o2, launch, cfg).as_ref().map(&covers).unwrap_or(false) {
            pok += 1;
        }
    }
    b.pos_forgive = pok as f32 / 8.0;
}

/// The position rule: a reproducible stand presses W into a real CORNER -
/// two substantial near-vertical faces meeting at a clear angle (normals
/// 45..135 deg apart, a hard 90 is ideal: pathetically easy to nestle into).
/// Faces are probed at knee AND waist height so a low box against a wall
/// counts. Returns the capsule-pinned position the player is stopped at
/// every time, preferring the corner closest to 90 deg; None = no corner in
/// reach, so not a valid lineup stand.
pub fn wedge_stand(scene: &Scene, stand: V3) -> Option<V3> {
    use parry3d::query::{Ray, RayCast};
    const R: f32 = 42.0; // pawn CapsuleRadius from the files
    const REACH: f32 = 150.0;
    let id = nalgebra::Isometry3::identity();
    // wall faces around the stand: (hit point, horizontal unit normal facing
    // the player, probe height)
    let mut walls: Vec<(V3, V3, f32)> = Vec::new();
    for h in [45.0f32, 90.0] {
        let o = stand + V3::new(0.0, 0.0, h);
        for k in 0..16 {
            let a = k as f32 / 16.0 * std::f32::consts::TAU;
            let d = V3::new(a.cos(), a.sin(), 0.0);
            let ray = Ray::new(nalgebra::Point3::from(o), d);
            if let Some(hit) = scene.mesh.cast_ray_and_get_normal(&id, &ray, REACH, true) {
                let mut n = hit.normal;
                if n.dot(&d) > 0.0 {
                    n = -n;
                }
                let hz = V3::new(n.x, n.y, 0.0);
                if hz.norm() > 0.7 {
                    // wall-like, not ramp/floor
                    walls.push((o + d * hit.time_of_impact, hz.normalize(), h));
                }
            }
        }
    }
    // best corner = closest to 90 deg (|dot| ~ 0), then closest to the stand
    let mut best: Option<V3> = None;
    let mut best_key = f32::MAX;
    for (i, (p1, n1, h1)) in walls.iter().enumerate() {
        for (p2, n2, h2) in &walls[i + 1..] {
            let dot = n1.dot(n2);
            // a real corner: normals 45..135 deg apart. Shallower bends read
            // as one flat wall (you slide), near-opposite is a corridor
            if dot.abs() > 0.71 {
                continue;
            }
            // capsule center touching both planes: n_i . (c - p_i) = R
            // (normals are horizontal, so height drops out of the 2x2)
            let (r1, r2) = (R - n1.dot(&(stand - *p1)), R - n2.dot(&(stand - *p2)));
            let det = n1.x * n2.y - n1.y * n2.x;
            let m = V3::new((r1 * n2.y - r2 * n1.y) / det, (n1.x * r2 - n2.x * r1) / det, 0.0);
            let shift = V3::new(m.x, m.y, 0.0).norm();
            let key = dot.abs() * 400.0 + shift;
            if key >= best_key || shift > 140.0 {
                continue;
            }
            // both faces must be substantial and really extend to the touch
            // point: present at their probe height AND 25u lower (rejects
            // poles, rails, trim) - and reachable from the pinned spot
            // (rejects convex corners, where the spot hangs off an edge)
            let pxy = stand + m;
            let touch_ok = [(*n1, *h1), (*n2, *h2)].into_iter().all(|(n, h)| {
                [0.0f32, -25.0].into_iter().all(|dz| {
                    let o = V3::new(pxy.x, pxy.y, stand.z + h + dz);
                    let ray = Ray::new(nalgebra::Point3::from(o), -n);
                    scene.mesh.cast_ray(&id, &ray, R + 15.0, true).map_or(false, |t| t > 20.0)
                })
            });
            if !touch_ok {
                continue;
            }
            let Some(gz) = scene.ground_z(pxy.x, pxy.y) else { continue };
            if (gz - stand.z).abs() > 60.0 {
                continue;
            }
            best_key = key;
            best = Some(V3::new(pxy.x, pxy.y, gz));
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;
    use parry3d::shape::TriMesh;

    /// Ground quad plus vertical wall quads ([x0,y0,x1,y1], top_z) (z from 0).
    fn scene_with_walls(walls: &[([f32; 4], f32)]) -> Scene {
        let mut verts = vec![
            Point3::new(-2.0e4, -2.0e4, 0.0),
            Point3::new(2.0e4, -2.0e4, 0.0),
            Point3::new(2.0e4, 2.0e4, 0.0),
            Point3::new(-2.0e4, 2.0e4, 0.0),
        ];
        let mut tris = vec![[0u32, 1, 2], [0, 2, 3]];
        for (w, top) in walls {
            let b = verts.len() as u32;
            verts.push(Point3::new(w[0], w[1], 0.0));
            verts.push(Point3::new(w[2], w[3], 0.0));
            verts.push(Point3::new(w[2], w[3], *top));
            verts.push(Point3::new(w[0], w[1], *top));
            tris.push([b, b + 1, b + 2]);
            tris.push([b, b + 2, b + 3]);
        }
        Scene {
            mesh: TriMesh::new(verts, tris),
            stands: vec![],
            min_z: 0.0,
            sun: V3::new(0.55, 0.45, 0.70),
            sun_color: [1.0; 3],
            uvs: vec![],
            tri_owner: vec![(0, "ground".into())],
            tri_color: vec![(0, [0.6, 0.6, 0.6])],
            tri_tex: vec![],
            tri_foliage: vec![],
        }
    }

    /// A corner of two perpendicular walls pins the capsule 42u off each
    /// face; a knee-high box against a wall also counts; a single wall, a
    /// corridor, and open ground do not qualify.
    #[test]
    fn wedge_rule() {
        let corner = scene_with_walls(&[([200.0, 0.0, 200.0, 1000.0], 300.0), ([0.0, 200.0, 1000.0, 200.0], 300.0)]);
        let p = wedge_stand(&corner, V3::new(150.0, 150.0, 0.0)).expect("corner must pin");
        assert!(
            (p.x - 158.0).abs() < 2.0 && (p.y - 158.0).abs() < 2.0,
            "pinned 42u off both faces, got ({}, {})",
            p.x,
            p.y
        );

        // low box (60u) against a tall wall: only the knee probe sees the box
        let lowbox = scene_with_walls(&[([200.0, 0.0, 200.0, 1000.0], 300.0), ([0.0, 200.0, 1000.0, 200.0], 60.0)]);
        let p = wedge_stand(&lowbox, V3::new(150.0, 150.0, 0.0)).expect("wall + knee-high box must pin");
        assert!((p.x - 158.0).abs() < 2.0 && (p.y - 158.0).abs() < 2.0, "got ({}, {})", p.x, p.y);

        let wall = scene_with_walls(&[([200.0, -1000.0, 200.0, 1000.0], 300.0)]);
        assert!(wedge_stand(&wall, V3::new(150.0, 0.0, 0.0)).is_none(), "one wall = slide, not a pin");

        let corridor = scene_with_walls(&[([100.0, -1000.0, 100.0, 1000.0], 300.0), ([-100.0, -1000.0, -100.0, 1000.0], 300.0)]);
        assert!(wedge_stand(&corridor, V3::new(0.0, 0.0, 0.0)).is_none(), "corridor has no pin along its axis");

        let open = scene_with_walls(&[]);
        assert!(wedge_stand(&open, V3::new(0.0, 0.0, 0.0)).is_none(), "open ground has no pin");
    }
}

