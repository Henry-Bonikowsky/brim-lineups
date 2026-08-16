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
/// and the pawn-blocker volumes. BVProjectile volumes ARE loaded: they are
/// BP_ProjectileBlockingVolume actors and the molly demonstrably bounces off
/// them in game (2026-08-03 clips: both real throws clipped the volume at the
/// Breeze courtyard tower that the bare art meshes do not cover).
const UMAP_BLACKLIST: [&str; 18] = [
    "Skybox", "Lighting", "Inactive", "_Alt", "_FFA", "QuickSpike", "SiteRush",
    "SpikeRush", "Profiling", "BTIL", "ObserverCameras", "Greybox",
    "FortCollins", "KillVolumes", "BVPawn", "VFX", "Working", "DesignChanges",
];
// "Vista" is NOT in the blacklist: vista sublevels are always-loaded scenery
// the game really shows (Lotus's mountains) and must RENDER for aim
// references against the skyline; they stay out of COLLISION only (no molly
// collision, handled per-mode below).
// NOTE: "Destruction" must NOT be blacklisted: on Haven the live C site IS
// Triad_Art_C_Destruction (the persistent level streams it always-loaded and
// does not stream the old Triad_Art_C at all).

/// Foliage renders nothing like in game (coarse blobs vs animated leaves), so
/// aim references must never sit on or against it. Shared by the native
/// loader and the pack writer (pack flag bit 4).
pub(crate) fn is_foliage_mesh(mesh: &str) -> bool {
    // "street" contains "tree": strip it before the keyword scan
    let ml = mesh.to_lowercase().replace("street", "");
    ["tree", "foliage", "leaf", "leaves", "bush", "ivy", "grass", "fern", "hedge", "canopy", "shrub", "vine", "frond", "flower"]
        .iter()
        .any(|k| ml.contains(k))
}

/// Real sun from the map's Lighting umap exports: the strongest
/// DirectionalLightComponent with a rotation (UE forward vector = the light's
/// travel direction; we store the opposite, pointing AT the sun) plus its
/// color. Juliett names the file Juliett_LightingTest.json, so scan every
/// *Lighting*.json in the dump dir.
pub(crate) fn sun_of(dir: &Path) -> Option<(V3, [f32; 3])> {
    let mut best: Option<(f64, V3, [f32; 3])> = None;
    for entry in std::fs::read_dir(dir).ok()? {
        let p = entry.ok()?.path();
        let name = p.file_name()?.to_str()?;
        if !name.contains("Lighting") || !name.ends_with(".json") {
            continue;
        }
        let Ok(txt) = std::fs::read_to_string(&p) else { continue };
        let Ok(v) = serde_json::from_str::<Value>(&txt) else { continue };
        for e in v.as_array().into_iter().flatten() {
            if e["Type"].as_str() != Some("DirectionalLightComponent") {
                continue;
            }
            let props = &e["Properties"];
            let rot = &props["RelativeRotation"];
            let (Some(pitch), Some(yaw)) = (rot["Pitch"].as_f64(), rot["Yaw"].as_f64()) else {
                continue;
            };
            let inten = props["Intensity"].as_f64().unwrap_or(0.0);
            if best.as_ref().is_some_and(|(bi, ..)| inten <= *bi) {
                continue;
            }
            let (sp, cp) = (pitch as f32).to_radians().sin_cos();
            let (sy, cy) = (yaw as f32).to_radians().sin_cos();
            // light forward = direction the light travels; sun sits opposite
            let to_sun = -V3::new(cp * cy, cp * sy, sp);
            let c = &props["LightColor"];
            let col = [
                c["R"].as_f64().unwrap_or(255.0) as f32 / 255.0,
                c["G"].as_f64().unwrap_or(255.0) as f32 / 255.0,
                c["B"].as_f64().unwrap_or(255.0) as f32 / 255.0,
            ];
            best = Some((inten, to_sun, col));
        }
    }
    best.map(|(_, d, c)| (d, c))
}

/// The authoritative filter: the persistent level's own streaming list.
/// Always-loaded sublevels plus the BombMode set are what the live bomb-mode
/// map consists of; everything else (old art variants, alt modes, dev levels)
/// is not in the game. Falls back to None (name blacklist only) if missing.
pub(crate) fn allowed_umaps(dir: &Path) -> Option<std::collections::HashSet<String>> {
    let name = dir.file_name()?.to_str()?;
    let txt = std::fs::read_to_string(dir.join(format!("{name}.json"))).ok()?;
    let v: Value = serde_json::from_str(&txt).ok()?;
    let mut set = std::collections::HashSet::new();
    set.insert(name.to_string()); // the persistent level itself
    let last = |s: &str| s.rsplit('.').next().unwrap_or(s).to_string();
    for it in v.as_array()? {
        let p = &it["Properties"];
        if it["Class"].as_str().is_some_and(|c| c.contains("LevelStreamingAlwaysLoaded")) {
            if let Some(w) = p["WorldAsset"]["AssetPathName"].as_str() {
                set.insert(last(w));
            }
        }
        if let Some(arr) = p["GameModeSpecificSublevelsByKey"].as_array() {
            for e in arr {
                if e["SublevelKey"].as_str().is_some_and(|k| k.contains("BombMode")) {
                    for s in e["Sublevels"].as_array().into_iter().flatten() {
                        if let Some(w) = s["AssetPathName"].as_str() {
                            set.insert(last(w));
                        }
                    }
                }
            }
        }
    }
    (set.len() > 1).then_some(set)
}

/// Soft decor that does not gate projectiles in game (same family the ValoBoard
/// sight bake prunes).
// "Vista" was here and WRONG: Sunset's A-alley freeway is a Vista-named
// bridge over playable space that the real molly bounces off (Henry's
// look-up lineup). Distant vista skyline is beyond throw range anyway.
const MESH_BLACKLIST: [&str; 12] = [
    "Foliage", "Plant", "Sky", "Wire", "Rope", "Paper", "Flag", "Floater",
    "Islands_", "FloatingChunk",
    // overhead radianite tube runs (Breeze def-spawn): BlockAll profile but no
    // real collision in game; Henry's lineup arcs straight through where the
    // sim deflected off them (2026-08-03 walk-mode test). Support pillars stay.
    "RadianiteTubeStraight", "RadianiteTubeElbow",
];

/// One parsed OBJ: positions, optional per-vertex UVs (parallel to `vs`),
/// faces, and material sections (first face index, texture key from usemtl).
pub struct ObjMesh {
    pub vs: Vec<[f32; 3]>,
    pub uvs: Vec<[f32; 2]>,
    pub fs: Vec<[u32; 3]>,
    pub sections: Vec<(u32, String)>,
}

/// Small diffuse texture (BGR, top-down rows) for ground sampling.
pub struct TexImg {
    pub w: usize,
    pub h: usize,
    pub px: Vec<u8>,
}

impl TexImg {
    pub fn sample_uv(&self, u: f32, v: f32) -> [f32; 3] {
        let x = ((u * self.w as f32) as usize).min(self.w - 1);
        let y = ((v * self.h as f32) as usize).min(self.h - 1);
        let o = (y * self.w + x) * 3;
        [self.px[o + 2] as f32 / 255.0, self.px[o + 1] as f32 / 255.0, self.px[o] as f32 / 255.0]
    }

    /// Bilinear sample, wrapping at the edges (textures tile).
    pub fn sample_uv_bilinear(&self, u: f32, v: f32) -> [f32; 3] {
        let fx = u * self.w as f32 - 0.5;
        let fy = v * self.h as f32 - 0.5;
        let (x0, y0) = (fx.floor(), fy.floor());
        let (tx, ty) = (fx - x0, fy - y0);
        let xi = |x: f32| (x.rem_euclid(self.w as f32) as usize).min(self.w - 1);
        let yi = |y: f32| (y.rem_euclid(self.h as f32) as usize).min(self.h - 1);
        let (x0u, x1u, y0u, y1u) = (xi(x0), xi(x0 + 1.0), yi(y0), yi(y0 + 1.0));
        let at = |x: usize, y: usize, ch: usize| self.px[(y * self.w + x) * 3 + ch] as f32 / 255.0;
        let mut out = [0.0f32; 3];
        for (i, ch) in [2usize, 1, 0].into_iter().enumerate() {
            let top = at(x0u, y0u, ch) * (1.0 - tx) + at(x1u, y0u, ch) * tx;
            let bot = at(x0u, y1u, ch) * (1.0 - tx) + at(x1u, y1u, ch) * tx;
            out[i] = top * (1.0 - ty) + bot * ty;
        }
        out
    }
}

/// Legacy oblique light direction: the default sun when a map ships no
/// DirectionalLight (test scenes).
pub const DEFAULT_SUN: [f32; 3] = [0.55, 0.45, 0.70];

pub struct Scene {
    pub mesh: TriMesh,
    pub stands: Vec<V3>,
    pub min_z: f32,
    /// Unit vector pointing TOWARD the map's sun (from its DirectionalLight)
    /// and the sun's color, 0..1 per channel.
    pub sun: V3,
    pub sun_color: [f32; 3],
    /// Per-triangle UV coords (parallel to the tri soup); empty for collision
    /// scenes (only renders need UVs).
    pub uvs: Vec<[[f32; 2]; 3]>,
    /// (first triangle index, source label) per placement, for debug attribution
    pub tri_owner: Vec<(u32, String)>,
    /// (first triangle index, material color) per placement
    pub tri_color: Vec<(u32, [f32; 3])>,
    /// (first triangle index, ground texture) per placement
    pub tri_tex: Vec<(u32, Option<std::sync::Arc<TexImg>>)>,
    /// (first triangle index, is-foliage) per placement: trees, bushes, ivy.
    /// Foliage renders nothing like in game, so aim references must never sit
    /// on or against it.
    pub tri_foliage: Vec<(u32, bool)>,
}

impl Scene {
    /// Ground surface under (x, y). Collects all stacked hits walking down and
    /// picks the one closest to the walkable height the navmesh reports nearby
    /// (under-map shadow/basement meshes must not win); falls back to the
    /// lowest hit when no navmesh is near.
    pub fn ground_z(&self, x: f32, y: f32) -> Option<f32> {
        use parry3d::query::{Ray, RayCast};
        let mut z_top = 6000.0f32;
        let mut hits: Vec<f32> = Vec::new();
        // walk ALL stacked layers down to the map floor: an 8-hit cap plus
        // bailing on sub-1u coplanar sandwiches (roof trim, stacked beams over
        // Bonsai mid) used to stop ~1500u above the real ground, resolving
        // clicks onto rooftops and killing every genuinely-near lineup
        for _ in 0..64 {
            let ray = Ray::new(Point3::new(x, y, z_top), V3::new(0.0, 0.0, -1.0));
            match self.mesh.cast_ray(&nalgebra::Isometry3::identity(), &ray, z_top - self.min_z + 100.0, true) {
                Some(t) => {
                    if t > 1.0 {
                        hits.push(z_top - t);
                    }
                    z_top -= t + 5.0;
                }
                None => break,
            }
        }
        if hits.is_empty() {
            return None;
        }
        let nav_z = self
            .stands
            .iter()
            .filter(|s| (s.x - x).powi(2) + (s.y - y).powi(2) < 400.0 * 400.0)
            .min_by(|a, b| {
                ((a.x - x).powi(2) + (a.y - y).powi(2)).total_cmp(&((b.x - x).powi(2) + (b.y - y).powi(2)))
            })
            .map(|s| s.z);
        match nav_z {
            Some(nz) => hits
                .iter()
                .copied()
                .min_by(|a, b| (a - nz).abs().total_cmp(&(b - nz).abs())),
            None => hits.last().copied(),
        }
    }

    /// Standing surface under (x, y) for a USER-CHOSEN stand: like ground_z,
    /// but a right-click ON a climbable box means "I stand on the box", so
    /// the highest surface within mounting reach (one stepup box + hop,
    /// ~260u above the navmesh height) wins over the ground beside it.
    /// Climbable box tops are filtered out of the navmesh samples, so the
    /// plain nav-snap resolved onto the floor and spawned INSIDE the box.
    /// Target clicks keep ground_z (clicks under roofs must stay on the
    /// floor).
    pub fn stand_z(&self, x: f32, y: f32) -> Option<f32> {
        let g = self.ground_z(x, y)?;
        use parry3d::query::{Ray, RayCast};
        let top = g + 260.0 + 15.0;
        let ray = Ray::new(Point3::new(x, y, top), V3::new(0.0, 0.0, -1.0));
        match self.mesh.cast_ray(&nalgebra::Isometry3::identity(), &ray, 260.0, true) {
            Some(t) => Some(top - t),
            None => Some(g),
        }
    }

    pub fn foliage_at(&self, tri: u32) -> bool {
        match self.tri_foliage.binary_search_by_key(&tri, |(s, _)| *s) {
            Ok(i) => self.tri_foliage[i].1,
            Err(0) => false,
            Err(i) => self.tri_foliage[i - 1].1,
        }
    }

    pub fn owner_of(&self, tri: u32) -> &str {
        match self.tri_owner.binary_search_by_key(&tri, |(s, _)| *s) {
            Ok(i) => &self.tri_owner[i].1,
            Err(0) => "?",
            Err(i) => &self.tri_owner[i - 1].1,
        }
    }

    pub fn color_of(&self, tri: u32) -> [f32; 3] {
        match self.tri_color.binary_search_by_key(&tri, |(s, _)| *s) {
            Ok(i) => self.tri_color[i].1,
            Err(0) => [0.62, 0.62, 0.62],
            Err(i) => self.tri_color[i - 1].1,
        }
    }

    pub fn tex_of(&self, tri: u32) -> Option<&TexImg> {
        let i = match self.tri_tex.binary_search_by_key(&tri, |(s, _)| *s) {
            Ok(i) => i,
            Err(0) => return None,
            Err(i) => i - 1,
        };
        self.tri_tex[i].1.as_deref()
    }
}

/// 24bpp BMP reader for the dumped ground textures (bottom-up rows -> top-down).
pub(crate) fn load_bmp(path: &Path) -> Option<TexImg> {
    let d = std::fs::read(path).ok()?;
    if d.len() < 54 || &d[0..2] != b"BM" {
        return None;
    }
    let off = u32::from_le_bytes(d[10..14].try_into().ok()?) as usize;
    let w = i32::from_le_bytes(d[18..22].try_into().ok()?) as usize;
    let h = i32::from_le_bytes(d[22..26].try_into().ok()?).unsigned_abs() as usize;
    // rows are normally 4-byte padded, but the uvtex writer emits unpadded
    // rows (only matters for widths not divisible by 4)
    let mut row = (w * 3 + 3) & !3;
    if d.len() < off + row * h {
        row = w * 3;
        if d.len() < off + row * h {
            return None;
        }
    }
    let mut px = vec![0u8; w * h * 3];
    for y in 0..h {
        let src = off + (h - 1 - y) * row;
        px[y * w * 3..(y + 1) * w * 3].copy_from_slice(&d[src..src + w * 3]);
    }
    Some(TexImg { w, h, px })
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

pub(crate) fn xyz(v: &Value, d: f32) -> [f32; 3] {
    let g = |k: &str| v.get(k).and_then(Value::as_f64).map(|x| x as f32).unwrap_or(d);
    [g("X"), g("Y"), g("Z")]
}

pub(crate) fn rot3(v: &Value) -> [f32; 3] {
    let g = |k: &str| v.get(k).and_then(Value::as_f64).map(|x| x as f32).unwrap_or(0.0);
    [g("Pitch"), g("Yaw"), g("Roll")]
}

/// Minimal OBJ reader for valo_dump meshes, local space. Handles plain
/// `v/f` files and the uvtex form (`vt`, `usemtl <texKey>`, `f a/a b/b c/c`
/// with vt parallel to v). `sections` = (first face index, texture key).
pub(crate) fn load_obj(path: &Path) -> Option<ObjMesh> {
    let text = std::fs::read_to_string(path).ok()?;
    let (mut vs, mut uvs, mut fs) = (Vec::new(), Vec::new(), Vec::new());
    let mut sections: Vec<(u32, String)> = Vec::new();
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
            Some("vt") => {
                let mut p = [0f32; 2];
                for x in &mut p {
                    *x = it.next()?.parse().ok()?;
                }
                uvs.push(p);
            }
            Some("usemtl") => {
                sections.push((fs.len() as u32, it.next().unwrap_or("none").to_string()));
            }
            Some("f") => {
                let mut t = [0u32; 3];
                for x in &mut t {
                    *x = it.next()?.split('/').next()?.parse::<u32>().ok()? - 1;
                }
                fs.push(t);
            }
            _ => {}
        }
    }
    if uvs.len() != vs.len() {
        uvs.clear(); // malformed or absent: treat as no UVs
    }
    Some(ObjMesh { vs, uvs, fs, sections })
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

#[derive(PartialEq, Clone, Copy)]
pub enum Mode {
    Collision,
    Visual,
    Everything,
}

/// The one filter decision for an instance, shared by load_ex and the pack
/// writer so the packed bundle contains exactly the triangles the native
/// loader would build.
pub(crate) fn keep_instance(
    i: &Value,
    allowed: &Option<std::collections::HashSet<String>>,
    mode: Mode,
) -> bool {
    let umap = i["umap"].as_str().unwrap_or("");
    let mesh = i["mesh"].as_str().unwrap_or("");
    // real world geometry is Environment art; Cube/BasicShape placements are
    // trigger volumes, barriers, and markup, never molly-blocking surfaces.
    // GameObjectMesh components are pickups (ult orbs), overlap-only in game.
    // Bombsite outline/glow markers are decals + a target-view column: they
    // render as thin overlays in game and block nothing.
    let comp = i["component"].as_str().unwrap_or("");
    // BVProjectile sublevels hold the molly-blocking stand-ins for props
    // whose art mesh is NoCollision: accurate per-prop *Collision shells.
    // The crude Box_For_Volumes cubes in the same sublevel do NOT block
    // mollies. Shells load, cubes never do.
    let bv_proj = umap.contains("BVProjectile");
    if mode != Mode::Everything
        && (allowed.as_ref().is_some_and(|a| !a.contains(umap))
            || !(mesh.contains("/Environment/") || bv_proj)
            || comp == "GameObjectMesh"
            || comp.starts_with("StaticMesh_Glow")
            || comp.contains("TargetViewMode")
            // marker meshes only, anchored at the basename (Bombsite_0_*
            // planes, BombsiteMarker_*, BombSite_Decal): the old bare
            // "BombSite"/"Bombsite_" substrings also ate real architecture -
            // Haven's C floor (Floor_13_CBombSite), Ascent's ABombsite_0_Arch,
            // Plummet's BombSite_ buildings - leaving "no ground at target"
            // holes across whole sites. A path anchor is not enough either:
            // Ascent's real B props live in a /Props/BombsiteB/ folder
            || {
                let base = mesh.rsplit('/').next().unwrap_or("");
                base.starts_with("Bombsite_") || base.starts_with("BombsiteMarker") || base.starts_with("BombSite_")
            }
            || mesh.contains("BVS_Bomb")
            || mesh.contains("Box_For_Volumes")
            // Vista sublevels DO collide: Sunset's A-alley freeway (an
            // overhead bridge in playable space) lives in Art_ASiteVista and
            // Henry's real molly bounces off its slope - excluding vistas
            // from collision made flights sail through it ("shooting
            // through a wall") and made every bounce-off-the-bridge lineup
            // impossible. Genuinely distant vista scenery is beyond the
            // throw ceiling anyway
            || UMAP_BLACKLIST.iter().any(|b| umap.contains(b)))
    {
        return false;
    }
    if mode == Mode::Collision && MESH_BLACKLIST.iter().any(|b| mesh.contains(b)) {
        return false;
    }
    if mode == Mode::Visual
        && (mesh.contains("Sky")
            // invisible-in-game utility meshes: per-prop collision shells
            // (BVProjectile stand-ins and Environment *_Col/*Collision*
            // meshes) and lighting blockers. They must keep COLLIDING where
            // the collision filters load them, but rendering them paints
            // gray walls over real art the player can see through.
            || !mesh.contains("/Environment/")
            || mesh.ends_with("_Col")
            || mesh.contains("Collision")
            || mesh.contains("LightBlocker")
            || mesh.contains("LightLeakBlocker"))
    {
        return false;
    }
    // component collision override: cosmetics (glow columns, outlines, FX
    // shells) carry NoCollision and must not block the molly (but they ARE
    // visible, so the visual scene keeps them)
    let coll = &i["collision"];
    if mode == Mode::Collision && !coll.is_null() {
        let enabled = coll["enabled"].as_str().unwrap_or("");
        let profile = coll["profile"].as_str().unwrap_or("");
        if enabled == "ECollisionEnabled::NoCollision"
            || profile == "NoCollision"
            || profile.starts_with("Overlap")
            || profile.starts_with("Trigger")
        {
            return false;
        }
    }
    true
}

/// One placement's sub-instances as plain floats: (quat xyzw, translation,
/// scale). Empty = a plain (non-instanced) placement.
pub(crate) type SubInsts = Vec<([f32; 4], [f32; 3], [f32; 3])>;

pub(crate) fn parse_subinsts(i: &Value) -> SubInsts {
    i["perInstance"]
        .as_array()
        .map(|insts| {
            insts
                .iter()
                .map(|inst| {
                    let q = &inst["Rotation"];
                    let g = |k: &str| q.get(k).and_then(Value::as_f64).map(|x| x as f32).unwrap_or(0.0);
                    (
                        [g("X"), g("Y"), g("Z"), q.get("W").and_then(Value::as_f64).map(|x| x as f32).unwrap_or(1.0)],
                        xyz(&inst["Translation"], 0.0),
                        xyz(&inst["Scale3D"], 1.0),
                    )
                })
                .collect()
        })
        .unwrap_or_default()
}

/// World-space triangle expansion for one placement: EXACTLY the float ops of
/// the original loader (both the native JSON path and the wasm pack path call
/// this, so their triangle soups are bit-identical). When `uvs_out` is given
/// (visual scenes), per-tri UVs are appended in the same order; meshes without
/// UVs contribute zero triples.
#[allow(clippy::too_many_arguments)]
pub(crate) fn place_tris(
    vs: &[[f32; 3]],
    fs: &[[u32; 3]],
    loc: [f32; 3],
    rot: [f32; 3],
    scale: [f32; 3],
    subinsts: &SubInsts,
    tris: &mut Vec<[[f32; 3]; 3]>,
    uvs_in: &[[f32; 2]],
    uvs_out: Option<&mut Vec<[[f32; 2]; 3]>>,
) {
    let m = rotm(rot[0], rot[1], rot[2]);
    let comp = |v: [f32; 3]| -> [f32; 3] {
        let l = [v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]];
        [
            loc[0] + l[0] * m[0][0] + l[1] * m[1][0] + l[2] * m[2][0],
            loc[1] + l[0] * m[0][1] + l[1] * m[1][1] + l[2] * m[2][1],
            loc[2] + l[0] * m[0][2] + l[1] * m[1][2] + l[2] * m[2][2],
        ]
    };
    if !subinsts.is_empty() {
        // instanced meshes (benches, props, clutter): per-instance transform
        // relative to the component, then the component world transform
        for (q, it, is) in subinsts {
            let im = quat_rotm(q[0], q[1], q[2], q[3]);
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
    } else {
        for f in fs.iter() {
            tris.push([comp(vs[f[0] as usize]), comp(vs[f[1] as usize]), comp(vs[f[2] as usize])]);
        }
    }
    if let Some(uv_out) = uvs_out {
        let reps = if subinsts.is_empty() { 1 } else { subinsts.len() };
        if uvs_in.is_empty() {
            uv_out.extend(std::iter::repeat([[0.0f32; 2]; 3]).take(reps * fs.len()));
        } else {
            for _ in 0..reps {
                for f in fs.iter() {
                    uv_out.push([uvs_in[f[0] as usize], uvs_in[f[1] as usize], uvs_in[f[2] as usize]]);
                }
            }
        }
    }
}

/// Collision scene for the flight sim (strict molly-blocking filter).
pub fn load(dir: &Path) -> Scene {
    load_ex(dir, Mode::Collision)
}

/// Visual scene for renders: everything the player sees (decorative and
/// no-collision meshes included; only backdrop and invisible volumes dropped).
pub fn load_visual(dir: &Path) -> Scene {
    load_ex(dir, Mode::Visual)
}

/// Diagnostic scene: no filters at all (every instance from every umap).
pub fn load_everything(dir: &Path) -> Scene {
    load_ex(dir, Mode::Everything)
}

fn load_ex(dir: &Path, mode: Mode) -> Scene {
    let visual = mode != Mode::Collision;
    // --- placed meshes -> world triangles
    let inst: Vec<Value> = serde_json::from_str::<Value>(
        &std::fs::read_to_string(dir.join("instances.json")).expect("instances.json"),
    )
    .expect("instances json")
    .as_array()
    .expect("array")
    .clone();

    let allowed = allowed_umaps(dir);
    // game-file collision truth (valo_dump colmesh): mesh -> "complex"
    // (render tris ARE the collision), "simple" (use meshes_col hull), or
    // "none" (does not block - the canopy/decor class real mollies fly
    // through). Absent file = old behavior for that map
    let colinfo: HashMap<String, String> = std::fs::read_to_string(dir.join("colinfo.json"))
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default();
    let mut objs: HashMap<String, Option<ObjMesh>> = HashMap::new();
    // textures deduped by usemtl key (one diffuse per material, shared by
    // every mesh section that uses it)
    let mut texs: HashMap<String, Option<std::sync::Arc<TexImg>>> = HashMap::new();
    let mut tri_tex: Vec<(u32, Option<std::sync::Arc<TexImg>>)> = Vec::new();
    let mut tris: Vec<[[f32; 3]; 3]> = Vec::new();
    let mut uv_soup: Vec<[[f32; 2]; 3]> = Vec::new();
    let mut tri_owner: Vec<(u32, String)> = Vec::new();
    let mut tri_foliage: Vec<(u32, bool)> = Vec::new();
    let mut tri_color: Vec<(u32, [f32; 3])> = Vec::new();
    // material colors disabled: flat averages muddied the renders more than
    // they helped (Henry's call); hillshade gray reads better
    let colors: HashMap<String, [f32; 3]> = HashMap::new();
    let (mut used, mut skipped_umap, mut skipped_inst) = (0usize, 0usize, 0usize);
    for i in &inst {
        if !keep_instance(i, &allowed, mode) {
            skipped_umap += 1;
            continue;
        }
        let mesh = i["mesh"].as_str().unwrap_or("");
        let umap = i["umap"].as_str().unwrap_or("");
        let cmode = colinfo.get(mesh).map(String::as_str);
        if mode == Mode::Collision && cmode == Some("none") {
            skipped_inst += 1;
            continue;
        }
        let simple = mode == Mode::Collision && cmode == Some("simple");
        let obj_name = mesh.replace('/', "_").replace('.', "_") + ".obj";
        let cache_key = if simple { format!("col:{obj_name}") } else { obj_name.clone() };
        let entry = objs.entry(cache_key).or_insert_with(|| {
            load_obj(&dir.join(if simple { "meshes_col" } else { "meshes" }).join(&obj_name))
        });
        let Some(om) = entry else { continue };
        let base = tris.len() as u32;
        tri_owner.push((base, format!("{umap}:{}", mesh.rsplit('/').next().unwrap_or("?"))));
        tri_foliage.push((base, is_foliage_mesh(mesh)));
        tri_color.push((
            base,
            colors.get(&obj_name.trim_end_matches(".obj").to_string()).copied().unwrap_or([0.62, 0.62, 0.62]),
        ));
        let subinsts = parse_subinsts(i);
        // per-section texture binding, repeated per sub-instance (faces are
        // emitted rep-major by place_tris)
        if om.sections.is_empty() {
            tri_tex.push((base, None));
        } else {
            let reps = if subinsts.is_empty() { 1 } else { subinsts.len() } as u32;
            let nf = om.fs.len() as u32;
            for rep in 0..reps {
                for (ff, key) in &om.sections {
                    let tex = texs
                        .entry(key.clone())
                        .or_insert_with(|| {
                            let t = (key != "none")
                                .then(|| load_bmp(&dir.join("textures").join(format!("{key}.bmp"))).map(std::sync::Arc::new))
                                .flatten();
                            if t.is_none() && key != "none" {
                                eprintln!("texture missing: {key}");
                            }
                            t
                        })
                        .clone();
                    tri_tex.push((base + rep * nf + ff, tex));
                }
            }
        }
        skipped_inst += subinsts.len(); // counted as expanded now
        place_tris(
            &om.vs,
            &om.fs,
            xyz(&i["location"], 0.0),
            rot3(&i["rotation"]),
            xyz(&i["scale"], 1.0),
            &subinsts,
            &mut tris,
            &om.uvs,
            visual.then_some(&mut uv_soup),
        );
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
        if let Some(om) = load_obj(&dir.join("targeting.obj")) {
            let (vs, fs) = (&om.vs, &om.fs);
            let before = stands.len();
            for f in fs {
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

        // keep every LARGE connected walkable network (<=130u steps, neighbors
        // within 420u): isolated perches, scaffold tops and ability-only
        // towers form small islands and vanish. Networks joined only by
        // one-way drops / rope ascenders (Bonsai splits into two ~1000-stand
        // halves) are all real playable ground - keeping only the single
        // largest one deleted half the map
        let n = stands.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn find(p: &mut Vec<usize>, i: usize) -> usize {
            let mut r = i;
            while p[r] != r {
                r = p[r];
            }
            let mut c = i;
            while p[c] != r {
                let nx = p[c];
                p[c] = r;
                c = nx;
            }
            r
        }
        for i in 0..n {
            for j in i + 1..n {
                let d = stands[j] - stands[i];
                if d.z.abs() <= 130.0 && d.x * d.x + d.y * d.y <= 420.0 * 420.0 {
                    let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                    if a != b {
                        parent[a] = b;
                    }
                }
            }
        }
        let mut counts: std::collections::HashMap<usize, usize> = Default::default();
        for i in 0..n {
            *counts.entry(find(&mut parent, i)).or_default() += 1;
        }
        if let Some(&max_count) = counts.values().max() {
            let min_keep = max_count / 10;
            let keep: Vec<bool> =
                (0..n).map(|i| counts[&find(&mut parent, i)] > min_keep).collect();
            let mut k = 0;
            stands.retain(|_| {
                k += 1;
                keep[k - 1]
            });
            eprintln!("stands: {} -> {} after main-network filter", n, stands.len());
        }
    }

    let (sun, sun_color) =
        sun_of(dir).unwrap_or((V3::new(DEFAULT_SUN[0], DEFAULT_SUN[1], DEFAULT_SUN[2]), [1.0; 3]));
    Scene { mesh, stands, min_z, sun, sun_color, uvs: uv_soup, tri_owner, tri_color, tri_tex, tri_foliage }
}

// ---- packed map bundles (web build) ----
// Format "BLP1", little-endian, written by pack::pack (native) and read here
// (both native for verification and wasm in the browser). The filters and the
// stand pipeline (reachability + main-network) already ran at pack time; the
// loader only re-expands placements through the SAME place_tris float ops, so
// the triangle soup is bit-identical to the native JSON loader's.

struct Rd<'a>(&'a [u8], usize);
impl<'a> Rd<'a> {
    fn u8(&mut self) -> u8 { let v = self.0[self.1]; self.1 += 1; v }
    fn u16(&mut self) -> u16 { let v = u16::from_le_bytes(self.0[self.1..self.1 + 2].try_into().unwrap()); self.1 += 2; v }
    fn u32(&mut self) -> u32 { let v = u32::from_le_bytes(self.0[self.1..self.1 + 4].try_into().unwrap()); self.1 += 4; v }
    fn f32(&mut self) -> f32 { f32::from_bits(self.u32()) }
    fn bytes(&mut self, n: usize) -> &'a [u8] { let v = &self.0[self.1..self.1 + n]; self.1 += n; v }
    fn f3(&mut self) -> [f32; 3] { [self.f32(), self.f32(), self.f32()] }
}

/// Load a packed map (already gunzipped) into (collision, visual) scenes.
pub fn load_pack(bytes: &[u8]) -> (Scene, Scene) {
    let mut r = Rd(bytes, 0);
    assert_eq!(r.bytes(4), b"BLP3", "bad pack magic");
    let sun = { let p = r.f3(); V3::new(p[0], p[1], p[2]) };
    let sun_color = r.f3();
    let ntex = r.u32() as usize;
    let mut texs: Vec<std::sync::Arc<TexImg>> = Vec::with_capacity(ntex);
    for _ in 0..ntex {
        let (w, h) = (r.u16() as usize, r.u16() as usize);
        texs.push(std::sync::Arc::new(TexImg { w, h, px: r.bytes(w * h * 3).to_vec() }));
    }
    let nmesh = r.u32() as usize;
    // (vs, uvs, fs, sections: (first face, texture id or -1))
    type PackMesh = (Vec<[f32; 3]>, Vec<[f32; 2]>, Vec<[u32; 3]>, Vec<(u32, i32)>);
    let mut meshes: Vec<PackMesh> = Vec::with_capacity(nmesh);
    for _ in 0..nmesh {
        let (nv, nuv, nf, nsec) = (r.u32() as usize, r.u32() as usize, r.u32() as usize, r.u32() as usize);
        let vs: Vec<[f32; 3]> = (0..nv).map(|_| r.f3()).collect();
        let uvs: Vec<[f32; 2]> = (0..nuv).map(|_| [r.f32(), r.f32()]).collect();
        let fs: Vec<[u32; 3]> = (0..nf).map(|_| [r.u32(), r.u32(), r.u32()]).collect();
        let secs: Vec<(u32, i32)> = (0..nsec).map(|_| (r.u32(), r.u32() as i32)).collect();
        meshes.push((vs, uvs, fs, secs));
    }
    let nplace = r.u32() as usize;
    struct Place {
        flags: u8,
        mesh: u32,
        loc: [f32; 3],
        rot: [f32; 3],
        scale: [f32; 3],
        subinsts: SubInsts,
    }
    let mut places = Vec::with_capacity(nplace);
    for _ in 0..nplace {
        let flags = r.u8();
        let mesh = r.u32();
        let (loc, rot, scale) = (r.f3(), r.f3(), r.f3());
        let ninst = r.u32() as usize;
        let subinsts: SubInsts = (0..ninst)
            .map(|_| ([r.f32(), r.f32(), r.f32(), r.f32()], r.f3(), r.f3()))
            .collect();
        places.push(Place { flags, mesh, loc, rot, scale, subinsts });
    }
    let nstand = r.u32() as usize;
    let stands: Vec<V3> = (0..nstand).map(|_| { let p = r.f3(); V3::new(p[0], p[1], p[2]) }).collect();

    let build = |bit: u8, stands: Vec<V3>| -> Scene {
        let visual = bit == 2;
        let mut tris: Vec<[[f32; 3]; 3]> = Vec::new();
        let mut uv_soup: Vec<[[f32; 2]; 3]> = Vec::new();
        let mut tri_tex: Vec<(u32, Option<std::sync::Arc<TexImg>>)> = Vec::new();
        let mut tri_foliage: Vec<(u32, bool)> = Vec::new();
        for p in &places {
            if p.flags & bit == 0 {
                continue;
            }
            let (vs, uvs, fs, secs) = &meshes[p.mesh as usize];
            let base = tris.len() as u32;
            if secs.is_empty() {
                tri_tex.push((base, None));
            } else {
                let reps = if p.subinsts.is_empty() { 1 } else { p.subinsts.len() } as u32;
                let nf = fs.len() as u32;
                for rep in 0..reps {
                    for (ff, t) in secs {
                        tri_tex.push((base + rep * nf + ff, (*t >= 0).then(|| texs[*t as usize].clone())));
                    }
                }
            }
            tri_foliage.push((base, p.flags & 4 != 0));
            place_tris(
                vs,
                fs,
                p.loc,
                p.rot,
                p.scale,
                &p.subinsts,
                &mut tris,
                uvs,
                visual.then_some(&mut uv_soup),
            );
        }
        let min_z = tris.iter().flat_map(|t| t.iter().map(|v| v[2])).fold(f32::MAX, f32::min);
        let vertices: Vec<Point3<f32>> =
            tris.iter().flat_map(|t| t.iter().map(|v| Point3::new(v[0], v[1], v[2]))).collect();
        let indices: Vec<[u32; 3]> =
            (0..tris.len() as u32).map(|i| [i * 3, i * 3 + 1, i * 3 + 2]).collect();
        Scene {
            mesh: TriMesh::new(vertices, indices),
            stands,
            min_z,
            sun,
            sun_color,
            uvs: uv_soup,
            tri_owner: vec![],
            tri_color: vec![],
            tri_tex,
            tri_foliage,
        }
    };
    (build(1, stands), build(2, vec![]))
}

