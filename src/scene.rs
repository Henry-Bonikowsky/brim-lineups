//! Load a map from a valo_dump output dir into a raycastable triangle scene
//! plus navmesh stand points.

use nalgebra::{Point3, Vector3};
use parry3d::shape::TriMesh;
use serde_json::Value;
use std::collections::HashMap;
use std::path::Path;

pub type V3 = Vector3<f32>;

/// Sublevels that are not live bomb-mode molly-blocking geometry: alt modes,
/// vistas, disabled props, event reskins, dev greybox, kill-volume triggers,
/// and the BV volumes (the molly ignores the Projectile channel and pawn
/// blockers per its BodyInstance responses; it collides with world statics).
const UMAP_BLACKLIST: [&str; 18] = [
    "Vista", "Skybox", "Lighting", "Inactive", "_Alt", "_FFA", "QuickSpike", "SiteRush",
    "SpikeRush", "Profiling", "BTIL", "Destruction", "ObserverCameras", "Greybox",
    "FortCollins", "KillVolumes", "BVProjectile", "BVPawn",
];

/// Soft decor that does not gate projectiles in game (same family the ValoBoard
/// sight bake prunes).
const MESH_BLACKLIST: [&str; 9] =
    ["Foliage", "Plant", "Sky", "Vista", "Wire", "Rope", "Paper", "Flag", "Floater"];

pub struct Scene {
    pub mesh: TriMesh,
    pub stands: Vec<V3>,
    pub min_z: f32,
    /// (first triangle index, source label) per placement, for debug attribution
    pub tri_owner: Vec<(u32, String)>,
}

impl Scene {
    pub fn owner_of(&self, tri: u32) -> &str {
        match self.tri_owner.binary_search_by_key(&tri, |(s, _)| *s) {
            Ok(i) => &self.tri_owner[i].1,
            Err(0) => "?",
            Err(i) => &self.tri_owner[i - 1].1,
        }
    }
}

fn rotm(pitch: f32, yaw: f32, roll: f32) -> [[f32; 3]; 3] {
    // UE FRotationMatrix rows = rotated local X/Y/Z axes (same math the
    // ValoBoard bake validated against the real map render).
    let (sp, cp) = pitch.to_radians().sin_cos();
    let (sy, cy) = yaw.to_radians().sin_cos();
    let (sr, cr) = roll.to_radians().sin_cos();
    [
        [cp * cy, cp * sy, sp],
        [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp],
        [-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp],
    ]
}

fn xyz(v: &Value, d: f32) -> [f32; 3] {
    let g = |k: &str| v.get(k).and_then(Value::as_f64).map(|x| x as f32).unwrap_or(d);
    [g("X"), g("Y"), g("Z")]
}

fn rot3(v: &Value) -> [f32; 3] {
    let g = |k: &str| v.get(k).and_then(Value::as_f64).map(|x| x as f32).unwrap_or(0.0);
    [g("Pitch"), g("Yaw"), g("Roll")]
}

/// Minimal OBJ reader for valo_dump meshes (v/f triangles only), local space.
fn load_obj(path: &Path) -> Option<(Vec<[f32; 3]>, Vec<[u32; 3]>)> {
    let text = std::fs::read_to_string(path).ok()?;
    let (mut vs, mut fs) = (Vec::new(), Vec::new());
    for line in text.lines() {
        let mut it = line.split_ascii_whitespace();
        match it.next() {
            Some("v") => {
                let mut p = [0f32; 3];
                for x in &mut p {
                    *x = it.next()?.parse().ok()?;
                }
                vs.push(p);
            }
            Some("f") => {
                let mut t = [0u32; 3];
                for x in &mut t {
                    *x = it.next()?.parse::<u32>().ok()? - 1;
                }
                fs.push(t);
            }
            _ => {}
        }
    }
    Some((vs, fs))
}

/// Rotation matrix (rows = rotated axes) from a UE quaternion.
fn quat_rotm(x: f32, y: f32, z: f32, w: f32) -> [[f32; 3]; 3] {
    let (x2, y2, z2) = (x + x, y + y, z + z);
    let (xx, yy, zz) = (x * x2, y * y2, z * z2);
    let (xy, xz, yz) = (x * y2, x * z2, y * z2);
    let (wx, wy, wz) = (w * x2, w * y2, w * z2);
    [
        [1.0 - (yy + zz), xy + wz, xz - wy],
        [xy - wz, 1.0 - (xx + zz), yz + wx],
        [xz + wy, yz - wx, 1.0 - (xx + yy)],
    ]
}

/// Collision scene for the flight sim (strict molly-blocking filter).
pub fn load(dir: &Path) -> Scene {
    load_ex(dir, false)
}

/// Visual scene for renders: everything the player sees (decorative and
/// no-collision meshes included; only backdrop and invisible volumes dropped).
pub fn load_visual(dir: &Path) -> Scene {
    load_ex(dir, true)
}

fn load_ex(dir: &Path, visual: bool) -> Scene {
    // --- placed meshes -> world triangles
    let inst: Vec<Value> = serde_json::from_str::<Value>(
        &std::fs::read_to_string(dir.join("instances.json")).expect("instances.json"),
    )
    .expect("instances json")
    .as_array()
    .expect("array")
    .clone();

    let mut objs: HashMap<String, Option<(Vec<[f32; 3]>, Vec<[u32; 3]>)>> = HashMap::new();
    let mut tris: Vec<[[f32; 3]; 3]> = Vec::new();
    let mut tri_owner: Vec<(u32, String)> = Vec::new();
    let (mut used, mut skipped_umap, mut skipped_inst) = (0usize, 0usize, 0usize);
    for i in &inst {
        let umap = i["umap"].as_str().unwrap_or("");
        let mesh = i["mesh"].as_str().unwrap_or("");
        // real world geometry is Environment art; Cube/BasicShape placements are
        // trigger volumes, barriers, and markup, never molly-blocking surfaces.
        // GameObjectMesh components are pickups (ult orbs), overlap-only in game.
        if !mesh.contains("/Environment/")
            || i["component"].as_str().unwrap_or("") == "GameObjectMesh"
            || UMAP_BLACKLIST.iter().any(|b| umap.contains(b))
        {
            skipped_umap += 1;
            continue;
        }
        if !visual && MESH_BLACKLIST.iter().any(|b| mesh.contains(b)) {
            skipped_umap += 1;
            continue;
        }
        if visual && (mesh.contains("Sky") || mesh.contains("Vista")) {
            skipped_umap += 1;
            continue;
        }
        // component collision override: cosmetics (glow columns, outlines, FX
        // shells) carry NoCollision and must not block the molly (but they ARE
        // visible, so the visual scene keeps them)
        let coll = &i["collision"];
        if !visual && !coll.is_null() {
            let enabled = coll["enabled"].as_str().unwrap_or("");
            let profile = coll["profile"].as_str().unwrap_or("");
            if enabled == "ECollisionEnabled::NoCollision"
                || profile == "NoCollision"
                || profile.starts_with("Overlap")
                || profile.starts_with("Trigger")
            {
                skipped_umap += 1;
                continue;
            }
        }
        let obj_name = mesh.replace('/', "_").replace('.', "_") + ".obj";
        let entry = objs
            .entry(obj_name.clone())
            .or_insert_with(|| load_obj(&dir.join("meshes").join(&obj_name)));
        let Some((vs, fs)) = entry else { continue };
        let loc = xyz(&i["location"], 0.0);
        let rot = rot3(&i["rotation"]);
        let scale = xyz(&i["scale"], 1.0);
        let m = rotm(rot[0], rot[1], rot[2]);
        let comp = |v: [f32; 3]| -> [f32; 3] {
            let l = [v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]];
            [
                loc[0] + l[0] * m[0][0] + l[1] * m[1][0] + l[2] * m[2][0],
                loc[1] + l[0] * m[0][1] + l[1] * m[1][1] + l[2] * m[2][1],
                loc[2] + l[0] * m[0][2] + l[1] * m[1][2] + l[2] * m[2][2],
            ]
        };
        tri_owner.push((tris.len() as u32, format!("{umap}:{}", i["component"].as_str().unwrap_or("?"))));
        if let Some(insts) = i["perInstance"].as_array() {
            // instanced meshes (benches, props, clutter): per-instance transform
            // relative to the component, then the component world transform
            for inst in insts {
                let q = &inst["Rotation"];
                let g = |k: &str| q.get(k).and_then(Value::as_f64).map(|x| x as f32).unwrap_or(0.0);
                let im = quat_rotm(g("X"), g("Y"), g("Z"), q.get("W").and_then(Value::as_f64).map(|x| x as f32).unwrap_or(1.0));
                let it = xyz(&inst["Translation"], 0.0);
                let is = xyz(&inst["Scale3D"], 1.0);
                let xf = |v: [f32; 3]| -> [f32; 3] {
                    let l = [v[0] * is[0], v[1] * is[1], v[2] * is[2]];
                    comp([
                        it[0] + l[0] * im[0][0] + l[1] * im[1][0] + l[2] * im[2][0],
                        it[1] + l[0] * im[0][1] + l[1] * im[1][1] + l[2] * im[2][1],
                        it[2] + l[0] * im[0][2] + l[1] * im[1][2] + l[2] * im[2][2],
                    ])
                };
                for f in fs.iter() {
                    tris.push([xf(vs[f[0] as usize]), xf(vs[f[1] as usize]), xf(vs[f[2] as usize])]);
                }
            }
            skipped_inst += insts.len(); // counted as expanded now
        } else {
            for f in fs.iter() {
                tris.push([comp(vs[f[0] as usize]), comp(vs[f[1] as usize]), comp(vs[f[2] as usize])]);
            }
        }
        used += 1;
    }

    // --- navmesh stand points (BasePawn): poly centroids + verts, deduped
    let nav: Value = serde_json::from_str(
        &std::fs::read_to_string(dir.join("navmesh.json")).expect("navmesh.json"),
    )
    .expect("navmesh json");
    let mut stands: Vec<V3> = Vec::new();
    let mut seen: std::collections::HashSet<(i64, i64, i64)> = std::collections::HashSet::new();
    for navset in nav["navs"].as_array().into_iter().flatten() {
        if !navset["nav"].as_str().unwrap_or("").contains("BasePawn") {
            continue;
        }
        for poly in navset["polys"].as_array().into_iter().flatten() {
            let verts: Vec<[f64; 3]> = poly
                .as_array()
                .unwrap()
                .iter()
                .map(|v| {
                    let a = v.as_array().unwrap();
                    [a[0].as_f64().unwrap(), a[1].as_f64().unwrap(), a[2].as_f64().unwrap()]
                })
                .collect();
            let n = verts.len() as f64;
            let c0 = verts.iter().fold([0.0; 3], |a, v| [a[0] + v[0], a[1] + v[1], a[2] + v[2]]);
            let c = [c0[0] / n, c0[1] / n, c0[2] / n];
            // poly verts sit flush against walls; inset by the player capsule
            // radius (BasePawn CapsuleRadius 42) so the eye is not inside a wall
            const INSET: f64 = 42.0;
            let inset = |v: &[f64; 3]| -> [f64; 3] {
                let d = [c[0] - v[0], c[1] - v[1]];
                let len = (d[0] * d[0] + d[1] * d[1]).sqrt();
                if len < INSET {
                    c
                } else {
                    [v[0] + d[0] / len * INSET, v[1] + d[1] / len * INSET, v[2]]
                }
            };
            for p in verts.iter().map(inset).chain(std::iter::once(c)) {
                let key = ((p[0] / 50.0) as i64, (p[1] / 50.0) as i64, (p[2] / 50.0) as i64);
                if seen.insert(key) {
                    stands.push(V3::new(p[0] as f32, p[1] as f32, p[2] as f32));
                }
            }
        }
    }

    // some maps (Rook, HURM_Helix) ship no real baked navmesh; fall back to
    // Riot's ability-targeting mesh (navmesh-derived), extracted as targeting.obj
    // via `valo_dump obj .../TargetingMeshes/<Map>_NavMesh_Targeting <dir>/targeting.obj`
    if stands.len() < 200 {
        if let Some((vs, fs)) = load_obj(&dir.join("targeting.obj")) {
            let before = stands.len();
            for f in &fs {
                let (a, b, c) = (vs[f[0] as usize], vs[f[1] as usize], vs[f[2] as usize]);
                let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                let w = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                let nz = u[0] * w[1] - u[1] * w[0]; // z of the cross product
                let area2 = ((u[1] * w[2] - u[2] * w[1]).powi(2)
                    + (u[2] * w[0] - u[0] * w[2]).powi(2)
                    + nz * nz)
                    .sqrt();
                if area2 < 1.0 || nz.abs() / area2 < 0.7 {
                    continue; // degenerate or too steep to stand on
                }
                let p = [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0, (a[2] + b[2] + c[2]) / 3.0];
                let key = ((p[0] / 50.0) as i64, (p[1] / 50.0) as i64, (p[2] / 50.0) as i64);
                if seen.insert(key) {
                    stands.push(V3::new(p[0], p[1], p[2]));
                }
            }
            eprintln!("navmesh sparse ({before} pts); targeting-mesh fallback added {}", stands.len() - before);
        }
    }

    let min_z = tris.iter().flat_map(|t| t.iter().map(|v| v[2])).fold(f32::MAX, f32::min);
    eprintln!(
        "scene[{}]: {} tris from {used} placements ({skipped_umap} filtered, {skipped_inst} instances expanded), {} stand points",
        if visual { "visual" } else { "collision" },
        tris.len(),
        stands.len()
    );

    let vertices: Vec<Point3<f32>> =
        tris.iter().flat_map(|t| t.iter().map(|v| Point3::new(v[0], v[1], v[2]))).collect();
    let indices: Vec<[u32; 3]> =
        (0..tris.len() as u32).map(|i| [i * 3, i * 3 + 1, i * 3 + 2]).collect();
    let mesh = TriMesh::new(vertices, indices);

    // reachability: Riot's navmesh includes crate-tops and AI/ability-only
    // perches. A player can only mount ~130u; keep a stand only if some nearby
    // floor sits within a jump of it (ground, ramps, and hoppable boxes pass;
    // isolated elevated islands fail).
    if !visual {
        use parry3d::query::{Ray, RayCast};
        let before = stands.len();
        stands.retain(|s| {
            (0..8).any(|k| {
                let a = k as f32 / 8.0 * std::f32::consts::TAU;
                let o = Point3::new(s.x + a.cos() * 150.0, s.y + a.sin() * 150.0, s.z + 200.0);
                let ray = Ray::new(o, V3::new(0.0, 0.0, -1.0));
                match mesh.cast_ray(&nalgebra::Isometry3::identity(), &ray, 800.0, true) {
                    Some(t) => {
                        let fz = o.z - t;
                        (fz - s.z).abs() <= 130.0
                    }
                    None => false,
                }
            })
        });
        eprintln!("stands: {} -> {} after reachability gate", before, stands.len());
    }

    Scene { mesh, stands, min_z, tri_owner }
}
