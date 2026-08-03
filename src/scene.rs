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
const UMAP_BLACKLIST: [&str; 17] = [
    "Vista", "Skybox", "Lighting", "Inactive", "_Alt", "_FFA", "QuickSpike", "SiteRush",
    "Profiling", "BTIL", "Destruction", "ObserverCameras", "Greybox", "FortCollins",
    "KillVolumes", "BVProjectile", "BVPawn",
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

pub fn load(dir: &Path) -> Scene {
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
        // trigger volumes, barriers, and markup, never molly-blocking surfaces
        if !mesh.contains("/Environment/")
            || UMAP_BLACKLIST.iter().any(|b| umap.contains(b))
            || MESH_BLACKLIST.iter().any(|b| mesh.contains(b))
        {
            skipped_umap += 1;
            continue;
        }
        if !i["perInstance"].is_null() {
            skipped_inst += 1; // instanced foliage/clutter; not handled in v1
            continue;
        }
        // component collision override: cosmetics (glow columns, outlines, FX
        // shells) carry NoCollision and must not block the molly
        let coll = &i["collision"];
        if !coll.is_null() {
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
        let xf = |v: [f32; 3]| -> [f32; 3] {
            let l = [v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]];
            [
                loc[0] + l[0] * m[0][0] + l[1] * m[1][0] + l[2] * m[2][0],
                loc[1] + l[0] * m[0][1] + l[1] * m[1][1] + l[2] * m[2][1],
                loc[2] + l[0] * m[0][2] + l[1] * m[1][2] + l[2] * m[2][2],
            ]
        };
        tri_owner.push((tris.len() as u32, format!("{umap}:{}", i["component"].as_str().unwrap_or("?"))));
        for f in fs.iter() {
            tris.push([xf(vs[f[0] as usize]), xf(vs[f[1] as usize]), xf(vs[f[2] as usize])]);
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
            // radius so the launch eye is not inside a wall face
            const INSET: f64 = 35.0;
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

    let min_z = tris.iter().flat_map(|t| t.iter().map(|v| v[2])).fold(f32::MAX, f32::min);
    eprintln!(
        "scene: {} tris from {used} placements ({skipped_umap} filtered, {skipped_inst} instanced skipped), {} stand points",
        tris.len(),
        stands.len()
    );

    let vertices: Vec<Point3<f32>> =
        tris.iter().flat_map(|t| t.iter().map(|v| Point3::new(v[0], v[1], v[2]))).collect();
    let indices: Vec<[u32; 3]> =
        (0..tris.len() as u32).map(|i| [i * 3, i * 3 + 1, i * 3 + 2]).collect();
    Scene { mesh: TriMesh::new(vertices, indices), stands, min_z, tri_owner }
}
