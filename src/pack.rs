//! `pack <dumpDir> <out>`: compact binary map bundle for the web build. All
//! scene filters run HERE (the wasm loader never parses OBJ/JSON); the bundle
//! stores deduped meshes + placement transforms + textures + the fully gated
//! stand list, gzipped. Before writing, the bundle is round-tripped through
//! scene::load_pack and compared bit-for-bit against the native loader.

use crate::scene::{self, Mode};
use serde_json::Value;
use std::collections::HashMap;
use std::io::Write as _;
use std::path::Path;

fn put_u32(b: &mut Vec<u8>, v: u32) {
    b.extend_from_slice(&v.to_le_bytes());
}
fn put_f32(b: &mut Vec<u8>, v: f32) {
    b.extend_from_slice(&v.to_le_bytes());
}
fn put_f3(b: &mut Vec<u8>, v: [f32; 3]) {
    for x in v {
        put_f32(b, x);
    }
}

pub fn pack(dir: &Path, out: &Path) {
    let inst: Vec<Value> = serde_json::from_str::<Value>(
        &std::fs::read_to_string(dir.join("instances.json")).expect("instances.json"),
    )
    .expect("instances json")
    .as_array()
    .expect("array")
    .clone();
    let allowed = scene::allowed_umaps(dir);

    // mesh + texture tables, deduped by obj name / usemtl key, in first-use order
    let mut mesh_ids: HashMap<String, Option<u32>> = HashMap::new();
    // (vs, uvs, fs, sections: (first face, texture id or -1))
    type PackMesh = (Vec<[f32; 3]>, Vec<[f32; 2]>, Vec<[u32; 3]>, Vec<(u32, i32)>);
    let mut meshes: Vec<PackMesh> = Vec::new();
    let mut tex_ids: HashMap<String, i32> = HashMap::new();
    let mut textures: Vec<scene::TexImg> = Vec::new();
    // placements in instances.json order: (flags, mesh, loc, rot, scale, subinsts)
    let mut places: Vec<(u8, u32, [f32; 3], [f32; 3], [f32; 3], scene::SubInsts)> = Vec::new();

    for i in &inst {
        let kc = scene::keep_instance(i, &allowed, Mode::Collision);
        let kv = scene::keep_instance(i, &allowed, Mode::Visual);
        if !kc && !kv {
            continue;
        }
        let mesh_name = i["mesh"].as_str().unwrap_or("");
        let obj_name = mesh_name.replace('/', "_").replace('.', "_") + ".obj";
        let id = mesh_ids
            .entry(obj_name.clone())
            .or_insert_with(|| {
                let om = scene::load_obj(&dir.join("meshes").join(&obj_name))?;
                let secs: Vec<(u32, i32)> = om
                    .sections
                    .iter()
                    .map(|(ff, key)| {
                        let id = *tex_ids.entry(key.clone()).or_insert_with(|| {
                            if key == "none" {
                                return -1;
                            }
                            match scene::load_bmp(&dir.join("textures").join(format!("{key}.bmp"))) {
                                Some(t) => {
                                    textures.push(t);
                                    textures.len() as i32 - 1
                                }
                                None => -1,
                            }
                        });
                        (*ff, id)
                    })
                    .collect();
                meshes.push((om.vs, om.uvs, om.fs, secs));
                Some(meshes.len() as u32 - 1)
            })
            .clone();
        let Some(id) = id else { continue }; // obj missing: native loader skips too
        let flags = (kc as u8) | ((kv as u8) << 1) | ((scene::is_foliage_mesh(mesh_name) as u8) << 2);
        places.push((
            flags,
            id,
            scene::xyz(&i["location"], 0.0),
            scene::rot3(&i["rotation"]),
            scene::xyz(&i["scale"], 1.0),
            scene::parse_subinsts(i),
        ));
    }

    // the fully gated stand list (navmesh + targeting fallback + reachability
    // + main-network) comes from the normal native load; also our reference
    // for the round-trip check below
    let cref = scene::load(dir);
    let vref = scene::load_visual(dir);

    let mut b: Vec<u8> = Vec::new();
    b.extend_from_slice(b"BLP2");
    put_u32(&mut b, textures.len() as u32);
    for t in &textures {
        b.extend_from_slice(&(t.w as u16).to_le_bytes());
        b.extend_from_slice(&(t.h as u16).to_le_bytes());
        b.extend_from_slice(&t.px);
    }
    put_u32(&mut b, meshes.len() as u32);
    for (vs, uvs, fs, secs) in &meshes {
        put_u32(&mut b, vs.len() as u32);
        put_u32(&mut b, uvs.len() as u32);
        put_u32(&mut b, fs.len() as u32);
        put_u32(&mut b, secs.len() as u32);
        for v in vs {
            put_f3(&mut b, *v);
        }
        for u in uvs {
            put_f32(&mut b, u[0]);
            put_f32(&mut b, u[1]);
        }
        for f in fs {
            for x in f {
                put_u32(&mut b, *x);
            }
        }
        for (ff, t) in secs {
            put_u32(&mut b, *ff);
            put_u32(&mut b, *t as u32);
        }
    }
    put_u32(&mut b, places.len() as u32);
    for (flags, mesh, loc, rot, scale, subinsts) in &places {
        b.push(*flags);
        put_u32(&mut b, *mesh);
        put_f3(&mut b, *loc);
        put_f3(&mut b, *rot);
        put_f3(&mut b, *scale);
        put_u32(&mut b, subinsts.len() as u32);
        for (q, t, s) in subinsts {
            for x in q {
                put_f32(&mut b, *x);
            }
            put_f3(&mut b, *t);
            put_f3(&mut b, *s);
        }
    }
    put_u32(&mut b, cref.stands.len() as u32);
    for s in &cref.stands {
        put_f3(&mut b, [s.x, s.y, s.z]);
    }

    // round-trip check: the packed scenes must be BIT-IDENTICAL to the native
    // loader's (same tri soup, same order, same stands, same min_z)
    let (pc, pv) = scene::load_pack(&b);
    for (name, p, r) in [("collision", &pc, &cref), ("visual", &pv, &vref)] {
        assert_eq!(p.mesh.vertices().len(), r.mesh.vertices().len(), "{name}: vertex count");
        assert!(
            p.mesh.vertices().iter().zip(r.mesh.vertices()).all(|(a, b)| {
                a.x.to_bits() == b.x.to_bits() && a.y.to_bits() == b.y.to_bits() && a.z.to_bits() == b.z.to_bits()
            }),
            "{name}: vertex soup differs"
        );
        assert_eq!(p.min_z.to_bits(), r.min_z.to_bits(), "{name}: min_z");
        assert_eq!(p.uvs.len(), r.uvs.len(), "{name}: uv count");
        assert!(
            p.uvs.iter().zip(&r.uvs).all(|(a, b)| {
                a.iter().zip(b).all(|(x, y)| x[0].to_bits() == y[0].to_bits() && x[1].to_bits() == y[1].to_bits())
            }),
            "{name}: uv soup differs"
        );
        assert_eq!(
            p.tri_tex.len(),
            r.tri_tex.len(),
            "{name}: tri_tex placement count"
        );
        assert!(
            p.tri_tex.iter().zip(&r.tri_tex).all(|(a, b)| a.0 == b.0
                && match (&a.1, &b.1) {
                    (None, None) => true,
                    (Some(x), Some(y)) => x.w == y.w && x.h == y.h && x.px == y.px,
                    _ => false,
                }),
            "{name}: tri_tex mapping differs"
        );
    }
    assert_eq!(pc.stands.len(), cref.stands.len(), "stand count");
    assert!(
        pc.stands.iter().zip(&cref.stands).all(|(a, b)| a == b),
        "stands differ"
    );

    let raw_len = b.len();
    let f = std::fs::File::create(out).expect("create out");
    let mut gz = flate2::write::GzEncoder::new(f, flate2::Compression::new(7));
    gz.write_all(&b).expect("gzip write");
    gz.finish().expect("gzip finish");
    let packed = std::fs::metadata(out).unwrap().len();
    eprintln!(
        "packed {} -> {} ({:.1} MB raw, {:.1} MB gz): {} meshes, {} textures, {} placements, {} stands [round-trip verified]",
        dir.display(),
        out.display(),
        raw_len as f64 / 1.0e6,
        packed as f64 / 1.0e6,
        meshes.len(),
        textures.len(),
        places.len(),
        cref.stands.len()
    );
}
