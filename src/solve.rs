//! Inverse solver: every navmesh stand point x analytic ballistic aim, refined
//! by simulation, ranked by time-to-land with an aim-forgiveness measure.

use crate::scene::{Scene, V3};
use crate::par::*;
use crate::sim::{fly, Cfg};

/// UI landmarks usable as aiming references (screen fractions of the HUD):
/// pixel-true elements only. Lineups whose aim puts one of these ON a world
/// silhouette edge are replicable in game without guesswork.
const UI_ANCHORS: [(&str, f32, f32); 12] = [
    // measured PIXEL-EXACT from Henry's full native screenshot (1999x1249,
    // 16:10). Preference order: ties keep the earlier entry.
    ("crosshair", 0.5, 0.5),
    // Henry's go-to: the sharp chevron point under the equip prompt box
    // (y derived from his reference screenshot via the diamond/mouse spacing)
    ("chevron point below the equip prompt", 0.4722, 0.8963),
    ("mouse icon in the equip prompt", 0.4722, 0.8607),
    ("diamond tip above the equip prompt", 0.4722, 0.8055),
    ("left end of the Q charge bar", 0.3802, 0.9680),
    ("right end of the Q charge bar", 0.4217, 0.9680),
    ("left end of the E charge bar", 0.4457, 0.9680),
    ("right end of the E charge bar", 0.4872, 0.9680),
    ("left end of the MB4 charge pips", 0.5113, 0.9680),
    ("right end of the MB4 charge pips", 0.5528, 0.9680),
    ("left end of the X charge bar", 0.5768, 0.9680),
    ("right end of the X charge bar", 0.6183, 0.9680),
];

/// Does any UI anchor sit on a strong depth edge at this aim? Returns the
/// best (anchor name, edge distance, grade 1..2, screen fx, fy); grade 2 =
/// dead on.
fn ui_reference(scene: &Scene, eye: V3, yaw: f32, pitch: f32, cfg: &Cfg) -> Option<(&'static str, f32, u8, f32, f32)> {
    ui_reference_ex(scene, eye, yaw, pitch, cfg, 0.005).map(|(r, _)| r)
}

/// ui_reference plus the screen position of the feature-edge crossing nearest
/// the winning anchor (what align_reference steers onto). `s` is the probe
/// grid spacing in screen fractions (0.005 = the display default).
fn ui_reference_ex(
    scene: &Scene,
    eye: V3,
    yaw: f32,
    pitch: f32,
    cfg: &Cfg,
    s: f32,
) -> Option<((&'static str, f32, u8, f32, f32), (f32, f32))> {
    use parry3d::query::{Ray, RayCast};
    let (sy, cy) = yaw.to_radians().sin_cos();
    let (sp, cp) = pitch.to_radians().sin_cos();
    let fwd = V3::new(cp * cy, cp * sy, sp);
    let right = V3::new(-sy, cy, 0.0);
    let up = fwd.cross(&right).normalize();
    // Valorant is vertical-FOV-fixed across aspects: tan_v from 103 hFOV at
    // 16:9, horizontal derived for Henry's 16:10 screen
    let tan_v = (103.0f32.to_radians() / 2.0).tan() * 9.0 / 16.0;
    let tan_h = tan_v * 1.6;
    let id = nalgebra::Isometry3::identity();
    let depth_at = |fx: f32, fy: f32| -> (f32, V3) {
        let d = (fwd + right * ((fx * 2.0 - 1.0) * tan_h) + up * ((1.0 - fy * 2.0) * tan_v)).normalize();
        match scene.mesh.cast_ray_and_get_normal(&id, &Ray::new(nalgebra::Point3::from(eye), d), 5.0e4, true) {
            Some(h) => (h.time_of_impact, h.normal),
            None => (f32::INFINITY, V3::zeros()),
        }
    };
    let edgy = |a: f32, b: f32| -> bool {
        let near = a.min(b);
        near < 9000.0 && (a.max(b) / near > 1.7 || (a.is_infinite() != b.is_infinite()))
    };
    // a corner SEAM (two walls meeting) is the reference humans actually use:
    // no depth jump at all, but the surface normal snaps. Same-distance
    // neighbors on sharply angled faces count as a feature edge.
    let crease = |a: &(f32, V3), b: &(f32, V3)| -> bool {
        a.0.is_finite() && b.0.is_finite() && a.0.min(b.0) < 9000.0 && a.1.dot(&b.1).abs() < 0.6
    };
    // an EDGE is a line - the anchor can slide along it, so it locks only one
    // aim axis and is NOT a reference. A reference is a POINT: corner,
    // junction, tip. Ring-probe 12 rays around the candidate: a straight edge
    // splits the ring into two ~half arcs; a point feature shows >=3 surface
    // groups or one small arc (a tip against sky / an L-junction).
    let is_point = |cx: f32, cy: f32, s: f32| -> bool {
        let ring: Vec<(f32, V3)> = (0..12)
            .map(|k| {
                let a = k as f32 / 12.0 * std::f32::consts::TAU;
                depth_at(cx + a.cos() * s, cy + a.sin() * s)
            })
            .collect();
        let same = |a: &(f32, V3), b: &(f32, V3)| -> bool {
            if a.0.is_infinite() && b.0.is_infinite() {
                return true;
            }
            a.0.is_finite()
                && b.0.is_finite()
                && a.0.max(b.0) / a.0.min(b.0) < 1.3
                && a.1.dot(&b.1).abs() > 0.8
        };
        // contiguous same-surface arcs around the ring: (start slot, length)
        let breaks: Vec<usize> = (0..12).filter(|&k| !same(&ring[k], &ring[(k + 1) % 12])).collect();
        if breaks.len() < 2 {
            return false; // 0-1 groups: flat surface or lone glitch, no feature
        }
        let arcs: Vec<(usize, usize)> = breaks
            .iter()
            .enumerate()
            .map(|(n, &b)| {
                let start = (b + 1) % 12;
                let end = breaks[(n + 1) % breaks.len()];
                (start, (end + 12 - start) % 12 + 1)
            })
            .collect();
        // a LINE passes through the ring: the same surface (or sky) shows up
        // as two arcs on roughly opposite sides. Ropes, wires, rails and thin
        // trim all read this way - and none of them pin a point. Depth-only
        // comparison: a thin cylinder's normals differ side to side.
        let same_side = |a: (usize, usize), b: (usize, usize)| -> bool {
            let (ra, rb) = (&ring[a.0], &ring[b.0]);
            let both_inf = ra.0.is_infinite() && rb.0.is_infinite();
            let both_near = ra.0.is_finite() && rb.0.is_finite() && ra.0.max(rb.0) / ra.0.min(rb.0) < 1.3;
            if !(both_inf || both_near) {
                return false;
            }
            let ca = a.0 as f32 + a.1 as f32 / 2.0;
            let cb = b.0 as f32 + b.1 as f32 / 2.0;
            ((ca - cb).rem_euclid(12.0) - 6.0).abs() <= 2.0 // ~antipodal
        };
        for i in 0..arcs.len() {
            for j in i + 1..arcs.len() {
                if same_side(arcs[i], arcs[j]) {
                    return false; // the feature continues through: a line
                }
            }
        }
        // what's left: a junction of distinct surfaces, or a tip (small arc)
        arcs.len() >= 3 || arcs.iter().map(|a| a.1).min().unwrap_or(12) <= 4
    };
    let mut best: Option<((&'static str, f32, u8, f32, f32), (f32, f32))> = None;
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
        let mut d = [[(0.0f32, V3::zeros()); 5]; 5];
        for (j, dj) in (-2i32..=2).enumerate() {
            for (i, di) in (-2i32..=2).enumerate() {
                d[j][i] = depth_at(ax + di as f32 * s, ay + dj as f32 * s);
            }
        }
        // edge crossing adjacent to the center = grade 2; anywhere = grade 1.
        // dist must come from the SAME pair that tripped the edge test (the
        // old code min'd the horizontal pair for vertical edges - inf leak)
        // collect tripped feature-edge pairs, nearest the anchor first, then
        // keep the first one whose crossing is a POINT feature (ring test) -
        // a bare edge line never becomes a reference
        type Pair = (f32, (f32, f32, (f32, V3)), (f32, f32, (f32, V3)));
        let mut pairs: Vec<Pair> = Vec::new(); // (r, endpoint0, endpoint1)
        for j in 0..5 {
            for i in 0..5 {
                if i < 4 && (edgy(d[j][i].0, d[j][i + 1].0) || crease(&d[j][i], &d[j][i + 1])) {
                    let r = ((i as f32 + 0.5 - 2.0).abs()).max((j as f32 - 2.0).abs());
                    let (px, py) = (ax + (i as f32 - 2.0) * s, ay + (j as f32 - 2.0) * s);
                    pairs.push((r, (px, py, d[j][i]), (px + s, py, d[j][i + 1])));
                }
                if j < 4 && (edgy(d[j][i].0, d[j + 1][i].0) || crease(&d[j][i], &d[j + 1][i])) {
                    let r = ((i as f32 - 2.0).abs()).max((j as f32 + 0.5 - 2.0).abs());
                    let (px, py) = (ax + (i as f32 - 2.0) * s, ay + (j as f32 - 2.0) * s);
                    pairs.push((r, (px, py, d[j][i]), (px, py + s, d[j + 1][i])));
                }
            }
        }
        pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut grade = 0u8;
        let mut dist = f32::INFINITY;
        let mut cross = (ax, ay);
        for &(r, e0, e1) in pairs.iter().take(6) {
            // bisect the crossing ONTO the feature before ring-testing: with
            // an off-center probe a straight edge cuts the ring into a small
            // arc + big arc and masquerades as a tip
            let ((mut x0, mut y0, mut s0), (mut x1, mut y1, mut s1)) = (e0, e1);
            for _ in 0..5 {
                let (mx, my) = ((x0 + x1) * 0.5, (y0 + y1) * 0.5);
                let sm = depth_at(mx, my);
                if edgy(s0.0, sm.0) || crease(&s0, &sm) {
                    (x1, y1, s1) = (mx, my, sm);
                } else {
                    (x0, y0, s0) = (mx, my, sm);
                }
            }
            let (cx, cy) = ((x0 + x1) * 0.5, (y0 + y1) * 0.5);
            if is_point(cx, cy, s) {
                grade = if r <= 1.0 { 2 } else { 1 };
                dist = s0.0.min(s1.0);
                cross = (cx, cy);
                break;
            }
        }
        if grade > 0 {
            let better = match best {
                Some(((_, _, g, _, _), _)) => grade > g,
                None => true,
            };
            if better {
                best = Some(((name, dist, grade, ax, ay), cross));
            }
        }
        if matches!(best, Some(((_, _, 2, _, _), _))) && name == "crosshair" {
            break; // crosshair dead-on beats everything
        }
    }
    best
}

/// Nudge a covered lineup's aim by sub-degree steps until the winning UI
/// anchor sits EXACTLY on its feature edge (the in-game "kiss the corner"
/// motion), keeping the throw only if it still covers `target`. The coarse
/// sweep grid (1 deg) can never do this by itself - references it finds are
/// near-misses. Updates yaw/pitch/rest/time/bounces/err/ui_ref on success.
fn align_reference(scene: &Scene, origin: V3, l: &mut Lineup, target: V3, tol: f32, cfg: &Cfg) -> bool {
    let tan_v = (103.0f32.to_radians() / 2.0).tan() * 9.0 / 16.0;
    let tan_h = tan_v * 1.6;
    let (mut yaw, mut aim) = (l.yaw, l.pitch);
    let mut aligned = false;
    let mut s = 0.005;
    for _ in 0..6 {
        let Some(((_, _, _, ax, ay), (cx, cy))) = ui_reference_ex(scene, origin, yaw, aim, cfg, s) else {
            return false;
        };
        let (dfx, dfy) = (cx - ax, cy - ay);
        if dfx.abs() < 0.0008 && dfy.abs() < 0.0008 {
            aligned = true; // <1px at render scale: the point sits ON the anchor
            break;
        }
        // small-angle screen->camera: turning right shifts the world left
        yaw += (dfx * 2.0 * tan_h).to_degrees();
        aim += (-dfy * 2.0 * tan_v).to_degrees();
        s = (s * 0.5).max(0.0006); // finer probe grid as we close in
    }
    if !aligned || (yaw - l.yaw).abs() + (aim - l.pitch).abs() > 2.0 {
        return false; // never converged, or drifted implausibly far
    }
    let launch = crate::sim::launch_pitch(aim, cfg);
    let Some(o) = crate::sim::fly(scene, crate::sim::hand_origin(origin, yaw, cfg), dir_from(yaw, launch), cfg) else {
        return false;
    };
    let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
    let dz = (o.rest.z - target.z).abs();
    let err = if dz <= 220.0 { dxy } else { dxy + (dz - 220.0) * 3.0 };
    if !(err < tol && crate::sim::fire_covers(scene, o.rest, target)) {
        return false; // the aligned aim no longer lands the throw
    }
    l.yaw = yaw;
    l.pitch = aim;
    l.rest = o.rest;
    l.time = o.time;
    l.bounces = o.bounces;
    l.err = err;
    l.ui_ref = ui_reference(scene, origin, yaw, aim, cfg);
    true
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
pub fn solve(scene: &Scene, stands: &[V3], target: V3, tol: f32, min_dist: f32, strict: bool, browse: bool, cfg: &Cfg) -> Vec<Lineup> {
    let paired = !strict && stands.len() == 1;
    // POSITION RULE (strict/solver-picked stands only): a lineup stand must
    // sit against TWO faces (wall corner, angled walls, object against wall)
    // so pressing W into it stops the player on the same spot every time.
    // Snap every candidate to its capsule-pinned corner position; anything
    // with no wedge in reach is out. A RIGHT-CLICKED stand is the user's
    // choice: snap to a corner when one is in reach (repeatability for free),
    // otherwise take the exact spot and label it pos 0.
    let mut free_stand = false;
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
                        let ui_ref = if covered && paired { ui_reference(scene, origin, yaw, crate::sim::aim_pitch(pitch, cfg), cfg) } else { None };
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
                for b in &mut out {
                    // snap the aim so the reference point sits exactly on its
                    // feature (only kept when the nudged throw still covers)
                    if b.covered {
                        align_reference(scene, origin, b, target, tol, cfg);
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
            "flights: {} never stopped, {} landed far, {} within tol",
            n_none.load(Relaxed),
            n_far.load(Relaxed),
            n_near.load(Relaxed)
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
    let mut out: Vec<Lineup> = out
        .par_iter()
        .map(|l| {
            let (rt, rtol) = if browse { (l.rest, 450.0) } else { (target, tol) };
            let fams = solve(scene, &[l.stand], rt, rtol, 0.0, false, false, cfg);
            match fams.into_iter().filter(|f| f.covered).min_by(|a, b| a.time.total_cmp(&b.time)) {
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
    if b.ui_ref.is_none() {
        b.ui_ref = ui_reference(scene, origin, b.yaw, b.pitch, cfg);
    }
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
            tri_owner: vec![(0, "ground".into())],
            tri_color: vec![(0, [0.6, 0.6, 0.6])],
            tri_tex: vec![],
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

