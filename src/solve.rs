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

/// Henry's optimal-angle rule, final form: a throw resting within this of
/// the click IS "on the spot" (the fire patch is 450u), and among on-spot
/// throws from a position the FASTEST wins, full stop. Closeness beyond
/// this threshold buys nothing - a 20u lob never outranks a 100u throw
/// that gets there 4 seconds sooner.
const ON_TARGET: f32 = 150.0;

/// Throw preference: fewest bounces, then time. Forgiveness does NOT rank
/// (Henry: "forgiveness is obviously never gonna be high, it's a lineup" -
/// the forgiveness era picked soft compromise throws); it stays a label.
fn best(a: &Lineup, b: &Lineup) -> std::cmp::Ordering {
    a.bounces.cmp(&b.bounces).then(a.time.total_cmp(&b.time))
}

/// Walking distance (from the spike) beyond which a stand counts as fully
/// safe: nobody retaking will hike 40m+ before they can even see you.
const APPROACH_SAFE: f32 = 4000.0;

/// For each queried stand: the walking distance a retaker starting AT the
/// spike must cover before they first have line of sight to the thrower's
/// head. Henry: crow-flies distance says nothing - a 50m lineup one doorway
/// from site is easy to punish and a bad lineup. Dijkstra over the navmesh
/// stand graph (same <=130u step / <=420u neighbor rule the loader uses),
/// then per stand walk the nodes nearest-first until one sees it.
fn approach_dists(scene: &Scene, target: V3, queries: &[V3]) -> Vec<f32> {
    use parry3d::query::{Ray, RayCast};
    let nodes = &scene.stands;
    let n = nodes.len();
    if n == 0 {
        return vec![APPROACH_SAFE; queries.len()];
    }
    let mut adj: Vec<Vec<(u32, f32)>> = vec![Vec::new(); n];
    for i in 0..n {
        for j in i + 1..n {
            let d = nodes[j] - nodes[i];
            if d.z.abs() <= 130.0 {
                let dxy = (d.x * d.x + d.y * d.y).sqrt();
                if dxy <= 420.0 {
                    adj[i].push((j as u32, dxy));
                    adj[j].push((i as u32, dxy));
                }
            }
        }
    }
    let start = (0..n)
        .min_by(|&a, &b| (nodes[a] - target).norm().total_cmp(&(nodes[b] - target).norm()))
        .unwrap();
    let mut dist = vec![f32::INFINITY; n];
    dist[start] = 0.0;
    // positive-f32 bit patterns order like the floats: cheap Dijkstra keys
    let mut heap = std::collections::BinaryHeap::new();
    heap.push(std::cmp::Reverse((0u32, start)));
    while let Some(std::cmp::Reverse((dbits, i))) = heap.pop() {
        let dcur = f32::from_bits(dbits);
        if dcur > dist[i] || dcur > APPROACH_SAFE {
            continue;
        }
        for &(j, w) in &adj[i] {
            let nd = dcur + w;
            if nd < dist[j as usize] {
                dist[j as usize] = nd;
                heap.push(std::cmp::Reverse((nd.to_bits(), j as usize)));
            }
        }
    }
    let mut order: Vec<usize> = (0..n).filter(|&i| dist[i] <= APPROACH_SAFE).collect();
    order.sort_by(|&a, &b| dist[a].total_cmp(&dist[b]));
    let id = nalgebra::Isometry3::identity();
    queries
        .iter()
        .map(|s| {
            let head = s + V3::new(0.0, 0.0, 175.0);
            for &i in &order {
                let eye = nodes[i] + V3::new(0.0, 0.0, 160.0);
                let v = head - eye;
                let dl = v.norm();
                // -60 margin like the old site-exposure check: peeking around
                // the thrower's own cover corner doesn't count as seen
                if dl < 60.0
                    || scene
                        .mesh
                        .cast_ray(&id, &Ray::new(nalgebra::Point3::from(eye), v / dl), dl - 60.0, true)
                        .is_none()
                {
                    return dist[i];
                }
            }
            APPROACH_SAFE
        })
        .collect()
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
    pub exposed: bool,   // a retaker sees the thrower within 8m of leaving the spike
    pub graze: bool,     // flight scrapes a wall >25m out: model-fragile, ranked last
    pub razor: bool,     // >=2 aim jitters change the flight's bounce structure AND
                         // land far: the throw threads geometry (dropped outright)
    pub approach: f32,   // walking distance from the spike before the thrower is in view
    pub aim_ref: Option<(V3, f32)>, // crosshair reference: first geometry the aim ray hits
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
/// stand, deduped, capped at 55m, ranked near + time). strict=false
/// (paired mode, one locked stand): every distinct working ANGLE FAMILY from
/// that stand (pitch buckets), ranked by time. The angle per family/stand is
/// simply the fastest covered throw - no reference alignment of any kind.
/// browse=true (strict only): keep every lineup RESTING within tol of the
/// click even when its fire cannot reach the click itself - the user browses
/// nearby end locations and picks one. Sorted by landing distance.
pub fn solve(scene: &Scene, vis: Option<&Scene>, stands: &[V3], target: V3, tol: f32, min_dist: f32, strict: bool, browse: bool, cfg: &Cfg) -> Vec<Lineup> {
    // a right-clicked stand also tries WALL-PRESSED variants: "stand up
    // against that box" is the standard lineup stance (Henry's market
    // volley only exists pressed against the kiosk, 1.3m from his click)
    // and the corner pin only handles two-face corners
    let mut sv: Vec<V3> = stands.to_vec();
    if !strict && stands.len() == 1 {
        sv.extend(press_candidates(scene, vis, stands[0]));
    }
    let razor = |mut rows: Vec<Lineup>| -> Vec<Lineup> {
        // HARD drop, not a label (Henry 2026-08-16: "the lineup shown in the
        // video is impossible in game"): a row that CLAIMS success but fails
        // the inflated-radius clearance re-fly never appears. Near-miss
        // explanation rows (covered=false) stay - they promise nothing
        let n0 = rows.len();
        rows.retain(|l| !(l.razor && l.covered));
        if rows.len() < n0 {
            eprintln!("[razor] dropped {} geometry-threading rows", n0 - rows.len());
        }
        rows
    };
    if strict {
        return razor(solve_impl(scene, vis, &sv, target, tol, min_dist, strict, browse, cfg, true, true));
    }
    // paired: a right-click resolves to CORNERS (Henry's design; corners are
    // the reproducible stands) - EVERY corner in reach of the click and its
    // press variants, not just the geometrically best one (the squarest
    // corner near Henry's C-site click sits under the mid-birdhouse roof
    // where the lob dies, while the open ground beside it has it). Only when
    // no corner produces a working throw does the EXACT clicked spot run as
    // the pos-0 free stand, so a lineup that exists nowhere else still shows
    let mut pins: Vec<V3> = sv.iter().flat_map(|s| wedge_stands(scene, *s)).collect();
    let mut seen = std::collections::HashSet::new();
    pins.retain(|p| seen.insert(((p.x / 25.0).round() as i64, (p.y / 25.0).round() as i64)));
    let mut out = if pins.is_empty() {
        Vec::new()
    } else {
        razor(solve_impl(scene, vis, &pins, target, tol, min_dist, strict, browse, cfg, true, true))
    };
    if !out.iter().any(|l| l.covered) {
        out.extend(razor(solve_impl(scene, vis, &sv, target, tol, min_dist, strict, browse, cfg, true, false)));
    }
    out
}

/// Capsule-pressed positions against each distinct wall within reach of a
/// user-chosen stand (single face - the W-press stance the corner pin does
/// not cover). Ground- and headroom-validated, deduped, max 4.
fn press_candidates(scene: &Scene, vis: Option<&Scene>, stand: V3) -> Vec<V3> {
    use parry3d::query::{Ray, RayCast};
    let id = nalgebra::Isometry3::identity();
    let mut out: Vec<V3> = Vec::new();
    for k in 0..16 {
        if out.len() >= 4 {
            break;
        }
        let a = k as f32 / 16.0 * std::f32::consts::TAU;
        let d = V3::new(a.cos(), a.sin(), 0.0);
        let o = stand + V3::new(0.0, 0.0, 90.0);
        let Some(hit) = scene.mesh.cast_ray_and_get_normal(&id, &Ray::new(nalgebra::Point3::from(o), d), 260.0, true)
        else {
            continue;
        };
        let mut n = hit.normal;
        if n.dot(&d) > 0.0 {
            n = -n;
        }
        if V3::new(n.x, n.y, 0.0).norm() < 0.7 {
            continue; // ramp/floor, not a wall
        }
        let nh = V3::new(n.x, n.y, 0.0).normalize();
        let p = o + d * hit.time_of_impact + nh * 42.0;
        // Henry's rule: a wall press is only repeatable when the wall gives
        // you something to line up with. Geometric proxy: the face must END
        // or BREAK (edge, seam >10u, abutting object) within ~120u along the
        // wall on at least one side - a short box face passes, the middle of
        // a long featureless wall does not
        let along = V3::new(-nh.y, nh.x, 0.0);
        // Henry's rule, both halves: a press is repeatable if the wall
        // face ends/breaks nearby (geometric) OR carries a clear VISUAL
        // pattern to line up with (texture detail on the rendered face -
        // geometric-only turned every press into a corner press)
        let geo_anchor = [along, -along].iter().any(|dir| {
            (1..=4).any(|s| {
                // origin 12u off the face; a continuing flat face hits at ~12
                let so = p + *dir * (s as f32 * 30.0) - nh * 30.0;
                let back = Ray::new(nalgebra::Point3::from(so), -nh);
                match scene.mesh.cast_ray(&id, &back, 60.0, true) {
                    None => true,                    // wall ended
                    Some(t) => (t - 12.0).abs() > 10.0, // seam/step
                }
            })
        });
        if !geo_anchor && !vis.map(|v| wall_pattern(v, p, nh, stand.z)).unwrap_or(false) {
            continue;
        }
        // ground near the stand's level + standing headroom
        let down = Ray::new(nalgebra::Point3::new(p.x, p.y, stand.z + 50.0), V3::new(0.0, 0.0, -1.0));
        let Some(t) = scene.mesh.cast_ray(&id, &down, 170.0, true) else { continue };
        let gz = stand.z + 50.0 - t;
        let up = Ray::new(nalgebra::Point3::new(p.x, p.y, gz + 10.0), V3::new(0.0, 0.0, 1.0));
        if scene.mesh.cast_ray(&id, &up, 210.0, true).is_some() {
            continue;
        }
        let c = V3::new(p.x, p.y, gz);
        if out.iter().all(|q| (q - c).norm() > 30.0) {
            out.push(c);
        }
    }
    out
}


/// Does the rendered wall face around a press point carry visible DETAIL
/// (texture pattern, trim, panel lines) a player can line up with? Samples
/// a 3x3 luminance grid (+-45u) on the face from the VISUAL scene and looks
/// for contrast. Henry: "wall presses only work if theres a clear pattern
/// on the wall to line up with."
fn wall_pattern(vis: &Scene, press: V3, nh: V3, ground_z: f32) -> bool {
    use parry3d::query::{Ray, RayCast};
    if vis.uvs.is_empty() {
        return false;
    }
    let id = nalgebra::Isometry3::identity();
    let along = V3::new(-nh.y, nh.x, 0.0);
    let mut lums: Vec<f32> = Vec::with_capacity(9);
    for i in -1..=1 {
        for j in -1..=1 {
            let o = press + along * (i as f32 * 45.0) + V3::new(0.0, 0.0, 140.0 + j as f32 * 45.0 + (ground_z - press.z));
            let ray = Ray::new(nalgebra::Point3::from(o), -nh);
            let Some(hit) = vis.mesh.cast_ray_and_get_normal(&id, &ray, 120.0, true) else { continue };
            let tri = match hit.feature {
                parry3d::shape::FeatureId::Face(f) => f % vis.mesh.indices().len().max(1) as u32,
                _ => continue,
            };
            let q = o + (-nh) * hit.time_of_impact;
            let idxs = vis.mesh.indices()[tri as usize];
            let (a, b, c) = (
                vis.mesh.vertices()[idxs[0] as usize],
                vis.mesh.vertices()[idxs[1] as usize],
                vis.mesh.vertices()[idxs[2] as usize],
            );
            // barycentric weights of q in tri (projected)
            let (v0, v1, v2) = (b - a, c - a, nalgebra::Point3::from(q) - a);
            let (d00, d01, d11, d20, d21) =
                (v0.dot(&v0), v0.dot(&v1), v1.dot(&v1), v2.dot(&v0), v2.dot(&v1));
            let den = d00 * d11 - d01 * d01;
            if den.abs() < 1e-6 {
                continue;
            }
            let w1 = (d11 * d20 - d01 * d21) / den;
            let w2 = (d00 * d21 - d01 * d20) / den;
            let w0 = 1.0 - w1 - w2;
            let rgb = match (vis.tex_of(tri), vis.uvs.get(tri as usize)) {
                (Some(tex), Some(uv)) => {
                    let u = w0 * uv[0][0] + w1 * uv[1][0] + w2 * uv[2][0];
                    let v = w0 * uv[0][1] + w1 * uv[1][1] + w2 * uv[2][1];
                    tex.sample_uv(u, v)
                }
                _ => continue,
            };
            lums.push(0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2]);
        }
    }
    if lums.len() < 6 {
        return false;
    }
    let mean = lums.iter().sum::<f32>() / lums.len() as f32;
    let var = lums.iter().map(|l| (l - mean) * (l - mean)).sum::<f32>() / lums.len() as f32;
    var.sqrt() > 0.04
}

/// `finish_all=false` (refine pass): forgiveness (16 swept-sphere flights per
/// row) runs only for the fastest covered family per stand, since the caller
/// keeps just that one. Everything else identical.
#[allow(clippy::too_many_arguments)]
fn solve_impl(scene: &Scene, vis: Option<&Scene>, stands: &[V3], target: V3, tol: f32, min_dist: f32, strict: bool, browse: bool, cfg: &Cfg, finish_all: bool, pinned: bool) -> Vec<Lineup> {
    let paired = !strict;
    // POSITION RULE (strict/solver-picked stands only): a lineup stand must
    // sit against TWO faces (wall corner, angled walls, object against wall)
    // so pressing W into it stops the player on the same spot every time.
    // Snap every candidate to its capsule-pinned corner position; anything
    // with no wedge in reach is out. A RIGHT-CLICKED stand is the user's
    // choice: snap to a corner when one is in reach (repeatability for free),
    // otherwise take the exact spot and label it pos 0.
    let t_pin = Timer::new();
    let stands: Vec<(V3, bool)> = if strict {
        let n_in = stands.len();
        // corner pins PLUS anchored wall presses: pressing against a box or
        // wall with a feature to line up with is just as repeatable as a
        // corner (Henry's rule), and corner-only hunting never OFFERED the
        // behind-a-box stances he uses
        let pinned: Vec<V3> = stands
            .par_iter()
            .flat_map_iter(|s| {
                let mut v: Vec<V3> = wedge_stand(scene, *s).into_iter().collect();
                v.extend(press_candidates(scene, vis, *s));
                v.into_iter()
            })
            .collect();
        let mut seen = std::collections::HashSet::new();
        let out: Vec<V3> = pinned
            .into_iter()
            .filter(|p| seen.insert(((p.x / 25.0).round() as i64, (p.y / 25.0).round() as i64)))
            .collect();
        if n_in > 1 {
            eprintln!("stands: {n_in} candidates -> {} pinned/pressed", out.len());
        }
        out.into_iter().map(|s| (s, true)).collect()
    } else {
        // paired stands arrive resolved: solve() passes corner pins
        // (pinned=true) or the exact clicked spots as the pos-0 fallback
        stands.iter().map(|s| (*s, pinned)).collect()
    };
    if strict {
        eprintln!("[t] pin {:.2}s", t_pin.secs());
    }
    let t_sweep = Timer::new();
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    let (n_none, n_far, n_near) = (AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0));
    let (n_stand_any, n_stand_conf) = (AtomicUsize::new(0), AtomicUsize::new(0));
    // physics ceiling, and for the solver's own hunts Henry's rule: cap at
    // 60m (6000u, ~hearing range) - a longer lineup is not worth learning.
    // A user-locked stand (paired) is his choice and stays uncapped
    let mut max_range = cfg.speed * cfg.speed / cfg.gravity * 1.05;
    if strict {
        max_range = max_range.min(6000.0);
    }
    let mut all: Vec<Lineup> = stands
        .par_iter()
        .flat_map_iter(|&(stand, wedged)| {
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
            let mut best_miss: Option<Lineup> = None;
            // per family: the 5 fastest ray candidates, fastest first. The
            // sphere confirm walks them in order - a single angle that drifts
            // under the sphere must not kill a family whose neighbors pass
            let mut families: std::collections::HashMap<i32, Vec<Lineup>> = Default::default();
            // swept-sphere confirmation: re-fly at full radius, rescore
            // against the click; true = this row is real
            let confirm = |b: &mut Lineup| -> bool {
                let launch = crate::sim::launch_pitch(b.pitch, cfg);
                fly(scene, crate::sim::hand_origin(origin, b.yaw, cfg), dir_from(b.yaw, launch), cfg)
                    .filter(|o| !o.wall_carry)
                    .map(|o| {
                        let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
                        let dz = (o.rest.z - target.z).abs();
                        (b.rest, b.time, b.bounces, b.graze) = (o.rest, o.time, o.bounces, o.graze);
                        b.err = if dz <= 110.0 { dxy } else { dxy + (dz - 110.0) * 3.0 };
                        b.covered = b.err < tol && crate::sim::fire_covers(scene, o.rest, target);
                        b.covered || (browse && b.err < tol)
                    })
                    .unwrap_or(false)
            };
            let mut sweeps: Vec<(f32, f32)> = if paired {
                // WIDE yaw scan: wall-bounce throws aim at a WALL, up to
                // 60 deg off the direct line - a narrow +-6 window never
                // even discovered the fast bounce lineups Henry wants.
                // 2 x 2 deg cells; polish() walks the last mile continuously
                let mut v = Vec::new();
                let mut pitch = -35.0f32;
                while pitch <= 89.0 {
                    let mut dy = -60.0f32;
                    while dy <= 60.0 {
                        v.push((yaw0 + dy, pitch));
                        dy += 2.0;
                    }
                    // 1 deg pitch when the USER locked this stand (indoor
                    // corridors are 1-2 deg wide and fell between 2-deg
                    // rows); refine sub-solves keep 2 deg - they run per
                    // stand for every strict click and 1 deg tripled solves
                    pitch += if finish_all { 1.0 } else { 2.0 };
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
                // WIDE yaw (+-60): from most stands the direct line is
                // walled off and every real throw is a wall bounce or an
                // off-line lob - +-3 deg found 7 of 178 stands on an
                // enclosed site while a human improvises throws freely
                let mut pitch = -25.0f32;
                while pitch <= 88.0 {
                    let mut dy = -60.0f32;
                    while dy <= 60.0 {
                        v.push((yaw0 + dy, pitch));
                        dy += 5.0;
                    }
                    pitch += 5.0;
                }
                v
            };
            // pass 0: the normal sweep. Pass 1 (strict only, only when pass 0
            // found NOTHING from this stand): a dense grid - tight angular
            // windows (doorways, arch threads) slip between the coarse steps
            // and whole positions went missing
            for pass in 0..2 {
                let sw: Vec<(f32, f32)> = if pass == 0 {
                    std::mem::take(&mut sweeps)
                } else {
                    // dense rescue only where it can pay: nearby stands
                    // (tight windows matter there); 3.5-deg steps. The old
                    // 2.5-deg everywhere burned 1.9M flights per click
                    if paired || !families.is_empty() || d > 4000.0 {
                        break;
                    }
                    let mut v = Vec::new();
                    let mut pitch = -30.0f32;
                    while pitch <= 89.0 {
                        let mut dy = -60.0f32;
                        while dy <= 60.0 {
                            v.push((yaw0 + dy, pitch));
                            dy += 3.5;
                        }
                        pitch += 3.5;
                    }
                    v
                };
                for (yaw, pitch) in sw {
                    {
                        // sweeps iterate LAUNCH pitch: above the game's
                        // launch clamp the flight is fiction - skip
                        if pitch <= -89.0 || pitch > crate::sim::LAUNCH_MAX {
                            continue;
                        }
                        let hand = crate::sim::hand_origin(origin, yaw, cfg);
                        // discovery flies a cheap zero-width ray; anything that
                        // lands within tol is re-flown as the real swept sphere
                        // below and scored on THAT outcome (grazes the ray
                        // clears block the real molly)
                        // paired (incl. every refine sub-solve): discover at
                        // FULL radius - some throws exist only as sphere
                        // flights (a ray misses the container lip whose clip
                        // ricochet IS the lineup; Henry found one in game the
                        // thin sweep could never see). Strict's coarse stand
                        // hunt stays thin: it only proves stands, and refine
                        // re-derives every kept angle sphere-first
                        let thin = if paired { *cfg } else { crate::sim::Cfg { radius: 0.0, ..*cfg } };
                        let Some(o) = fly(scene, hand, dir_from(yaw, pitch), &thin) else {
                            n_none.fetch_add(1, Relaxed);
                            continue;
                        };
                        if o.wall_carry {
                            // long carry after a hard wall impact: not real
                            // in game (Henry) - never a candidate
                            n_far.fetch_add(1, Relaxed);
                            continue;
                        }
                        // success = the FIRE covers the clicked spot: rest within
                        // the 450u patch radius horizontally and within the
                        // fire's vertical reach (ZLayerTolerance 200, StepUp 110
                        // / StepDown 210 from the patch files)
                        let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
                        let dz = (o.rest.z - target.z).abs();
                        let err = if dz <= 110.0 { dxy } else { dxy + (dz - 110.0) * 3.0 };
                        // distance alone is not success: the fire must SPREAD
                        // to the click (a >110u box between rest and click
                        // blocks it even at 2m)
                        // fire_covers BFS caps spread travel at 450u, so even
                        // with a wide browse tol `covered` stays honest
                        let covered = err < tol && crate::sim::fire_covers(scene, o.rest, target);
                        let sel = covered || (browse && err < tol);
                        if covered { &n_near } else { &n_far }.fetch_add(1, Relaxed);
                        let cand = Lineup {
                            dist: d,
                            stand,
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
                            exposed: false,
                            graze: o.graze,
                            razor: false,
                            approach: 0.0,
                            aim_ref: None,
                        };
                        if !sel {
                            if paired && best_miss.as_ref().is_none_or(|b| cand.err < b.err) {
                                best_miss = Some(cand);
                            }
                            continue;
                        }
                        // candidates kept CLOSEST-first: optimal is the angle
                        // that lands nearest the click, time only breaks ties.
                        // Families split by pitch bucket AND bounce count: a
                        // wall-bounce throw (the class that kills flight time)
                        // must not be swallowed by a direct lob sharing its
                        // pitch bucket
                        let key = (pitch / 8.0).round() as i32 + o.bounces.min(3) as i32 * 1000;
                        let fam = families.entry(key).or_default();
                        let pos = fam.partition_point(|c| c.err <= cand.err);
                        if pos < 5 {
                            fam.insert(pos, cand);
                            fam.truncate(5);
                        }
                    }
                }
            }
            if paired {
                // families were discovered on the cheap zero-width flight:
                // sphere-confirm the candidates, then the family's rep is
                // Henry's optimal: the FASTEST throw that lands ON the spot
                // (err <= ON_TARGET); only if none is on the spot, the
                // closest confirmed
                let mut out: Vec<Lineup> = Vec::new();
                for cands in families.into_values() {
                    // confirm EVERY stored candidate (<=5/family): thin-ray
                    // and sphere flights can land in different places around
                    // lips/vents, so a candidate whose RAY err looks far can
                    // be the family's only on-target SPHERE throw (Henry's
                    // A-corner lineup was skipped by an early break here)
                    let mut ok: Vec<Lineup> = Vec::new();
                    for mut b in cands {
                        if confirm(&mut b) {
                            ok.push(b);
                        }
                    }
                    let rep = if ok.iter().any(|l| l.err <= ON_TARGET) {
                        ok.into_iter()
                            .filter(|l| l.err <= ON_TARGET)
                            .min_by(|a, b| a.time.total_cmp(&b.time))
                    } else {
                        ok.into_iter().min_by(|a, b| a.err.total_cmp(&b.err))
                    };
                    out.extend(rep);
                }
                if out.is_empty() {
                    // no throw lands within tolerance: report the closest miss so
                    // the user sees WHY (err > tol labels it)
                    out.extend(best_miss);
                }
                // Henry's tie-break: throws within 0.1s (same bounces) are
                // equal on speed - the most FORGIVING one wins. So the whole
                // 0.1s leading group gets polished+finished (forgiveness is
                // 16 flights/row, contenders only), not just the fastest;
                // finish_all (top-level paired) still does every row
                // clean hits before grazy hits before misses: a far-out
                // wall-scrape "works" only in a cm-perfect model
                let hit = |l: &Lineup| l.covered && l.err <= ON_TARGET;
                let mut order: Vec<usize> = (0..out.len()).collect();
                order.sort_by(|&i, &j| {
                    let (a, b) = (&out[i], &out[j]);
                    hit(b)
                        .cmp(&hit(a))
                        .then(a.graze.cmp(&b.graze))
                        .then(best(a, b))
                        .then(a.err.total_cmp(&b.err))
                });
                let lead = order.iter().position(|&i| hit(&out[i]));
                // polish the leader AND the best near-miss of every LOWER
                // bounce class: a clean lob confirming a hair off the spike
                // must get its polish nudge, or a bouncier throw that
                // happened to confirm on-spike steals the row (Henry's
                // corner: a working lob lost to a wall-bang)
                let lead_b = lead.map(|k| out[order[k]].bounces).unwrap_or(u32::MAX);
                let mut classes_done: Vec<u32> = Vec::new();
                for (k, &fi) in order.iter().enumerate() {
                    let contender = out[fi].bounces < lead_b
                        && out[fi].err <= 600.0
                        && !classes_done.contains(&out[fi].bounces);
                    if finish_all || lead == Some(k) || contender {
                        if contender {
                            classes_done.push(out[fi].bounces);
                        }
                        let b = &mut out[fi];
                        polish(scene, target, tol, cfg, origin, b);
                        finish(scene, target, tol, cfg, origin, b);
                    }
                }
                // RESCUE: no angle hits the spike, but the discovered pool
                // is coarse - polish can WALK a miss onto the click. Multi-
                // start: the on-target basin is often a NEIGHBOR of the
                // closest miss (ricochet corridors are non-convex; Henry's
                // A-corner lineup sat one basin over), so polish every
                // family rep, they are few and this path is rare
                if lead.is_none() {
                    for b in out.iter_mut() {
                        // dense local fan first: the on-target basin is often
                        // a NEIGHBOR of this miss (ricochet corridors are
                        // non-convex - Henry's A-corner lineup sat 1.4 deg
                        // from the coarse grid's best, across a roof band no
                        // hill-climb crosses). 0.5 deg steps, +-3 deg
                        let (y0, p0) = (b.yaw, b.pitch);
                        for dy in -10..=10 {
                            for dp in -10..=10 {
                                let (y, p) = (y0 + dy as f32 * 0.5, p0 + dp as f32 * 0.5);
                                let launch = crate::sim::launch_pitch(p, cfg);
                                let Some(o) =
                                    fly(scene, crate::sim::hand_origin(origin, y, cfg), dir_from(y, launch), cfg)
                                else {
                                    continue;
                                };
                                if o.wall_carry {
                                    continue;
                                }
                                let dxy =
                                    ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
                                let dz = (o.rest.z - target.z).abs();
                                let err = if dz <= 110.0 { dxy } else { dxy + (dz - 110.0) * 3.0 };
                                if err < b.err && crate::sim::fire_covers(scene, o.rest, target) {
                                    (b.yaw, b.pitch, b.rest, b.time, b.bounces, b.err, b.graze) =
                                        (y, p, o.rest, o.time, o.bounces, err, o.graze);
                                    b.covered = err < tol;
                                }
                            }
                        }
                        polish(scene, target, tol, cfg, origin, b);
                        finish(scene, target, tol, cfg, origin, b);
                    }
                }
                // final order: on-spot by (bounces, 0.1s time bucket,
                // forgiveness desc, time); misses trail by closeness
                out.sort_by(|a, b| {
                    hit(b)
                        .cmp(&hit(a))
                        .then(best(a, b))
                        .then(a.err.total_cmp(&b.err))
                });
                return out.into_iter();
            }
            // the ray-discovered angles must survive the real swept-sphere
            // flight: walk this stand's families closest-landing-first, and
            // each family's candidates closest-first - a stand only dies
            // when EVERY discovered angle grazes. The winner is provisional:
            // refine re-tunes the final angle with the full accuracy-first
            // rule
            let mut fams: Vec<Vec<Lineup>> = families.into_values().collect();
            if !fams.is_empty() {
                n_stand_any.fetch_add(1, Relaxed);
            }
            fams.sort_by(|a, b| a[0].err.total_cmp(&b[0].err));
            let mut best: Option<Lineup> = None;
            // budget: sphere-confirm at most the 12 closest candidates
            // overall - a stand whose 12 best ray landings all die under
            // the sphere isn't worth 100+ more flights (press stands
            // tripled the stand pool; unbudgeted confirms cost ~6s/click)
            let mut tries = 0;
            'fams: for cands in fams.iter_mut() {
                for b in cands.iter_mut() {
                    tries += 1;
                    if tries > 12 {
                        break 'fams;
                    }
                    if confirm(b) {
                        best = Some(b.clone());
                        break 'fams;
                    }
                }
            }
            if best.is_some() {
                n_stand_conf.fetch_add(1, Relaxed);
            } else {
                // RESCUE: the coarse candidates are ray-tuned and can ALL
                // drift under the sphere. Keep the stand's fastest candidate
                // flagged uncovered - the refine pass re-sweeps it densely
                // and sphere-confirms per angle; if even that finds nothing,
                // refine drops the row. A stand must not vanish just because
                // the coarse pool missed the sphere-correct aim
                best = fams.first().and_then(|c| c.first()).cloned().map(|mut b| {
                    b.covered = false;
                    b
                });
            }
            if let Some(b) = &mut best {
                // exposure/approach (how far a retaker walks before seeing
                // this stand) is computed once for the deduped survivors
                // after the sweep - see the approach_dists call below
                // browse: forgiveness/anchoring are about reliably hitting the
                // lineup's OWN landing spot (the user picked it by end
                // location), not about covering the original click.
                // Pre-refine forgiveness is only a RANKING signal (172 of 196
                // stands get truncated before refine, which recomputes it at
                // full radius) - fly it thin, not 16 swept spheres per stand
                let thin = crate::sim::Cfg { radius: 0.0, ..*cfg };
                if browse {
                    let rest = b.rest;
                    finish(scene, rest, 450.0, &thin, origin, b);
                } else {
                    finish(scene, target, tol, &thin, origin, b);
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
    // Henry's FINAL ranking rule: fewest bounces (a bouncy fast throw just
    // means a cleaner lineup exists), then least time. Nothing else ranks -
    // forgiveness/exposure/distance are labels the player judges. No
    // forgiveness culling either: every on-spike row shows with its numbers
    if paired {
        if finish_all {
            // top-level paired call (not a refine sub-solve): approach per
            // distinct stand (press candidates give paired several stands)
            let q: Vec<V3> = all.iter().map(|l| l.stand).collect();
            if !q.is_empty() {
                let ds = approach_dists(scene, target, &q);
                for (l, a) in all.iter_mut().zip(ds) {
                    l.approach = a;
                    l.exposed = a < 800.0;
                }
            }
        }
        // on-spike families by (bounces, time); a locked stand still shows
        // its closest miss (the user asked about THIS spot - explain why)
        all.sort_by(|a, b| {
            let (oa, ob) = (a.covered && a.err <= ON_TARGET, b.covered && b.err <= ON_TARGET);
            ob.cmp(&oa).then(best(a, b))
        });
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
    // approach = walking distance from the spike before a retaker SEES the
    // thrower (crow-flies says nothing: a 50m stand one doorway from site
    // is easy to punish). Drives ranking; <800u (8m) = the EXPOSED label
    let t_app = Timer::new();
    let q: Vec<V3> = out.iter().map(|l| l.stand).collect();
    for (l, a) in out.iter_mut().zip(approach_dists(scene, target, &q)) {
        l.approach = a;
        l.exposed = a < 800.0;
    }
    if strict {
        eprintln!(
            "[funnel] stands: any-covered-angle {} -> sphere-confirmed {} -> deduped {} ({} exposed) [t] approach {:.2}s",
            n_stand_any.load(Relaxed),
            n_stand_conf.load(Relaxed),
            out.len(),
            out.iter().filter(|l| l.exposed).count(),
            t_app.secs()
        );
    }
    // no sturdiness cull BEFORE refine: coarse-angle forgiveness says nothing
    // about the stand - refine re-tunes the angle entirely, and the real gate
    // runs on refine's output. Culling here was erasing rescuable positions
    // rank: browse = nearest landing first (user picks by end location).
    // else: (bounces, time) with an on-spike-first ordering pre-refine
    // (misses may still be rescued into on-spike rows by refine).
    // Stand coords break ties so repeat solves order identically (the n-th
    // row must be the same lineup when the picker re-requests it by index)
    // ONE rule everywhere, browse included (the picker's list IS browse):
    // tier 0 = HIDDEN on-spike lineups by (bounces, time) - a real lineup
    // is thrown from out of view; tier 1 = exposed on-spike rows, labeled,
    // after every hidden one; tier 2 = browse's near-miss rows by landing
    // distance (strict drops those post-refine)
    let rank = |v: &mut Vec<Lineup>| {
        // 0 hidden clean, 1 exposed clean, 2 hidden graze, 3 exposed graze,
        // 9 miss - a grazing thread never outranks any clean lineup
        let tier = |l: &Lineup| {
            if !(l.covered && l.err <= ON_TARGET) {
                9
            } else {
                (l.graze as u8) * 2 + l.exposed as u8
            }
        };
        v.sort_by(|a, b| {
            let (ta, tb) = (tier(a), tier(b));
            ta.cmp(&tb)
                .then(if ta == 9 { a.err.total_cmp(&b.err) } else { best(a, b) })
                .then(a.stand.x.total_cmp(&b.stand.x))
                .then(a.stand.y.total_cmp(&b.stand.y))
        });
    };
    rank(&mut out);
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
            // browse included: every row re-tunes onto the CLICK (the old
            // self-targeting "optimize your own landing" left the picker's
            // list full of never-tuned throws while strict looked fine)
            let fams = solve_impl(scene, vis, &[l.stand], target, tol, 0.0, false, false, cfg, false, true);
            // rows arrive in Henry's optimal order (closest-landing band,
            // then time) - the first covered row IS the optimal angle
            let mut cov: Vec<Lineup> = fams.into_iter().filter(|f| f.covered).collect();
            match (!cov.is_empty()).then(|| cov.swap_remove(0)) {
                Some(mut f) => {
                    // err stays relative to the original click for display/sort
                    let dxy = ((f.rest.x - target.x).powi(2) + (f.rest.y - target.y).powi(2)).sqrt();
                    let dz = (f.rest.z - target.z).abs();
                    f.err = if dz <= 110.0 { dxy } else { dxy + (dz - 110.0) * 3.0 };
                    // the paired sub-solve never computes exposure/approach
                    (f.exposed, f.approach) = (l.exposed, l.approach);
                    Some(f)
                }
                // even the dense sphere-confirmed re-sweep found no covered
                // throw at the click from this stand: strict drops it, but
                // browse keeps the coarse row as a near-miss option (Henry:
                // fewer rows is never the ask)
                None => browse.then(|| l.clone()),
            }
        })
        .flatten()
        .collect();
    eprintln!("[funnel] refine kept {} rows [t] refine {:.2}s", out.len(), t_refine.secs());
    // NO MATTER WHAT: not on the spike = not shown (Henry). Misses appear
    // ONLY when nothing lands at all - the closest few explain why the
    // click has no lineups instead of an empty list
    let on_spike = |l: &Lineup| l.covered && l.err <= ON_TARGET;
    if out.iter().any(on_spike) {
        out.retain(on_spike);
    } else {
        out.sort_by(|a, b| a.err.total_cmp(&b.err));
        out.truncate(3);
    }
    rank(&mut out);
    out
}

/// Continuous angle polish: the sweep grid is 1 x 1.25 deg, so its best
/// angle can sit a whole cell off the true optimum. Pattern-search descent
/// on (yaw, pitch) at full sphere physics - keep any move that lands CLOSER
/// and still covers - down to 0.05 deg steps. ~10-30 flights per row.
fn polish(scene: &Scene, target: V3, tol: f32, cfg: &Cfg, origin: V3, b: &mut Lineup) {
    // hard flight budget: a full-lob sphere flight costs 1-2ms, and chasing
    // 1u gains forever burned ~1s per row. Phase 1 walks the throw ON the
    // spot; phase 2 spends the rest cutting FLIGHT TIME while staying on it
    // (Henry's optimal: fastest that lands on it, closeness past ON_TARGET
    // buys nothing)
    let mut budget = 40u32;
    let probe = |b: &mut Lineup, y: f32, p: f32, by_time: bool| -> bool {
        let launch = crate::sim::launch_pitch(p, cfg);
        let Some(o) = fly(scene, crate::sim::hand_origin(origin, y, cfg), dir_from(y, launch), cfg)
        else {
            return false;
        };
        if o.wall_carry {
            return false;
        }
        let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
        let dz = (o.rest.z - target.z).abs();
        let err = if dz <= 110.0 { dxy } else { dxy + (dz - 110.0) * 3.0 };
        // covers is only meaningful (and only required) once the rest is
        // near the click: demanding it on FAR intermediate steps made
        // phase 1 unable to walk a distant throw closer at all
        let cov = err < tol && crate::sim::fire_covers(scene, o.rest, target);
        let better = if by_time {
            // never trade extra bounces for speed, never trade clean for
            // grazy: fewer bounces outranks
            err <= ON_TARGET && cov && o.bounces <= b.bounces && (o.graze <= b.graze) && o.time < b.time - 0.05
        } else {
            err < b.err - 2.0 && err < tol && (err > ON_TARGET || cov)
        };
        if better {
            (b.yaw, b.pitch, b.rest, b.time, b.bounces, b.err) = (y, p, o.rest, o.time, o.bounces, err);
            b.covered = cov;
            b.graze = o.graze;
            return true;
        }
        false
    };
    for by_time in [false, true] {
        if !by_time && b.err <= ON_TARGET {
            continue; // already on the spot: all budget goes to speed
        }
        let mut step = 0.6f32;
        while step > 0.05 && budget > 0 {
            let mut improved = false;
            for (dy, dp) in [(step, 0.0), (-step, 0.0), (0.0, step), (0.0, -step)] {
                if budget == 0 {
                    break;
                }
                budget -= 1;
                if probe(b, b.yaw + dy, b.pitch + dp, by_time) {
                    improved = true;
                    break;
                }
            }
            if !improved {
                step *= 0.5;
            }
        }
        if b.err > ON_TARGET {
            break; // never got on the spot: keep the closest, skip phase 2
        }
    }
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
    let mut ok = 0;
    let mut worst = 0.0f32;
    // razor evidence: a jitter that changes WHICH geometry the flight hits
    // (bounce structure differs) and lands far is not aim drift - the throw
    // threads geometry the model can't promise (Henry 2026-08-16: roof-skim
    // and facade-graze lineups, both impossible in game, both >=2 such
    // jitters; legit precise lineups only drift, same bounce structure)
    let mut ncat = 0;
    for (jy, jp) in
        [(0.75, 0.0), (-0.75, 0.0), (0.0, 0.75), (0.0, -0.75), (0.75, 0.75), (-0.75, 0.75), (0.75, -0.75), (-0.75, -0.75)]
    {
        let launch_pitch = crate::sim::launch_pitch(b.pitch + jp, cfg);
        if let Some(o) = fly(scene, crate::sim::hand_origin(origin, b.yaw + jy, cfg), dir_from(b.yaw + jy, launch_pitch), cfg) {
            let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
            let dz = (o.rest.z - target.z).abs();
            let dev = if dz <= 110.0 { dxy } else { dxy + (dz - 110.0) * 3.0 };
            worst = worst.max(dev);
            if o.bounces != b.bounces && dev > 800.0 {
                ncat += 1;
            }
            if !o.wall_carry && dev < tol && crate::sim::fire_covers(scene, o.rest, target) {
                ok += 1;
            }
        } else {
            worst = worst.max(9999.0);
            ncat += 1;
        }
    }
    b.forgive = ok as f32 / 8.0;
    b.spread = worst;
    b.razor = ncat >= 2;

    // POSITION forgiveness: shift the stand ~75u with the SAME aim; if the fire
    // still covers, exact positioning does not matter ("easy position")
    let covers = |o: &crate::sim::Outcome| {
        let dxy = ((o.rest.x - target.x).powi(2) + (o.rest.y - target.y).powi(2)).sqrt();
        let dz = (o.rest.z - target.z).abs();
        !o.wall_carry
            && (if dz <= 110.0 { dxy } else { dxy + (dz - 110.0) * 3.0 }) < tol
            && crate::sim::fire_covers(scene, o.rest, target)
    };
    let launch = dir_from(b.yaw, crate::sim::launch_pitch(b.pitch, cfg));
    // spatial clearance: re-fly the nominal throw with the radius fattened by
    // the model's error budget (30u). Clearing real geometry by less than that
    // is not a promise the model can keep (Henry's Foxtrot roof-skim
    // 2026-08-16: sim cleared a ridge by ~15u at 1km, the real molly hits the
    // roof every time). Start 60u out along the launch so the wall a pressed
    // stand leans on does not false-trip the fat sphere at the hand
    if !b.razor && b.covered {
        let fat = crate::sim::Cfg { radius: cfg.radius + 30.0, ..*cfg };
        let o0 = crate::sim::hand_origin(origin, b.yaw, cfg) + launch * 60.0;
        b.razor = !fly(scene, o0, launch, &fat).as_ref().map(&covers).unwrap_or(false);
    }
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
    wedge_stands(scene, stand).into_iter().next()
}

/// EVERY valid corner pin in reach, best (squarest, closest) first. Paired
/// mode sweeps them all: the geometric best corner can sit under a roof
/// (Haven mid birdhouse) that kills the very lob the user right-clicked
/// for, while a neighboring corner in the open has it.
pub fn wedge_stands(scene: &Scene, stand: V3) -> Vec<V3> {
    use parry3d::query::{Ray, RayCast};
    const R: f32 = 42.0; // pawn CapsuleRadius from the files
    // navmesh samples run ~190u apart: corners must be findable from HALF a
    // sample gap past the midpoint, or the ones between samples vanish
    // (Henry: "it's not finding all the corners")
    const REACH: f32 = 260.0;
    let id = nalgebra::Isometry3::identity();
    // wall faces around the stand: (hit point, horizontal unit normal facing
    // the player, probe height)
    let mut walls: Vec<(V3, V3, f32)> = Vec::new();
    for h in [45.0f32, 90.0] {
        let o = stand + V3::new(0.0, 0.0, h);
        for k in 0..24 {
            let a = k as f32 / 24.0 * std::f32::consts::TAU;
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
    // corner rank = closest to 90 deg (|dot| ~ 0), then closest to the stand
    let mut pins: Vec<(f32, V3)> = Vec::new();
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
            if shift > 250.0 {
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
            // ground at the pin, resolved RELATIVE TO THE STAND's level: the
            // navmesh snap picks the floor beside a climbable box and vetoed
            // every pin on its top (server paired mode lost box lineups the
            // CLI found)
            let down = Ray::new(nalgebra::Point3::new(pxy.x, pxy.y, stand.z + 50.0), V3::new(0.0, 0.0, -1.0));
            let Some(t) = scene.mesh.cast_ray(&id, &down, 170.0, true) else { continue };
            let gz = stand.z + 50.0 - t;
            if (gz - stand.z).abs() > 60.0 {
                continue;
            }
            // HEADROOM: a pin under an overhang (electrical box tops) put
            // the player inside geometry - aim views rendered black and
            // every throw died on the ceiling. A standing capsule needs
            // ~200u of clear air above the floor
            {
                let up = Ray::new(nalgebra::Point3::new(pxy.x, pxy.y, gz + 10.0), V3::new(0.0, 0.0, 1.0));
                if scene.mesh.cast_ray(&id, &up, 210.0, true).is_some() {
                    continue;
                }
            }
            pins.push((key, V3::new(pxy.x, pxy.y, gz)));
        }
    }
    // many wall-pair combos resolve to the same physical corner: keep the
    // best-ranked pin per 25u cell, best corners first
    pins.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut seen = std::collections::HashSet::new();
    pins.retain(|(_, p)| seen.insert(((p.x / 25.0).round() as i64, (p.y / 25.0).round() as i64)));
    pins.into_iter().map(|(_, p)| p).collect()
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

    /// A stand right behind a wall is 9m from the spike as the crow flies,
    /// but a retaker must walk the whole detour around the wall before they
    /// can see it: approach must reflect the WALK, not the straight line.
    #[test]
    fn approach_respects_walls() {
        let mut scene = scene_with_walls(&[([500.0, -2000.0, 500.0, 2000.0], 400.0)]);
        let mut nodes = vec![V3::new(0.0, 0.0, 0.0), V3::new(400.0, 0.0, 0.0)];
        for k in 1..=6 {
            nodes.push(V3::new(400.0, k as f32 * 400.0, 0.0));
        }
        nodes.push(V3::new(800.0, 2400.0, 0.0));
        for k in (0..=5).rev() {
            nodes.push(V3::new(900.0, k as f32 * 400.0, 0.0));
        }
        scene.stands = nodes;
        let target = V3::new(0.0, 0.0, 0.0);
        let a = approach_dists(&scene, target, &[V3::new(400.0, 0.0, 0.0), V3::new(900.0, 0.0, 0.0)]);
        assert!(a[0] < 100.0, "same-side stand is seen immediately, got {}", a[0]);
        assert!(a[1] > 2500.0 && a[1] <= 4000.0, "behind-wall stand needs the long detour, got {}", a[1]);
    }
}

