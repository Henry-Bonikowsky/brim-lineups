//! wasm-bindgen surface for the browser build: load a packed map, then the
//! same solve -> JSON contract as serve.rs /solve (byte-identical bodies),
//! plus renders as BMP byte buffers and the flight trajectory for the JS
//! canvas animation (no ffmpeg on the web).

use crate::scene::{self, Scene, V3};
use crate::sim::Cfg;
use crate::{api, render, solve};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn error(msg: &str);
}

#[derive(PartialEq, Clone, Copy)]
struct Key {
    tx: f32,
    ty: f32,
    stand: Option<(f32, f32)>,
    list: bool,
}

struct St {
    cscene: Scene,
    vscene: Scene,
    /// last solve, so picking row K to render does not redo the whole hunt
    /// (the native server re-solves per request; it has all cores, we don't)
    cache: Option<(Key, V3, Vec<solve::Lineup>)>,
}

thread_local! {
    static STATE: RefCell<Option<St>> = const { RefCell::new(None) };
}

#[wasm_bindgen]
pub fn load_map(pack: &[u8]) {
    std::panic::set_hook(Box::new(|p| error(&p.to_string())));
    let (cscene, vscene) = scene::load_pack(pack);
    STATE.with(|s| *s.borrow_mut() = Some(St { cscene, vscene, cache: None }));
}

/// Identical solve + row-selection logic to serve.rs /solve. Returns the same
/// JSON body (imgs always [] here; the browser fetches renders separately).
#[wasm_bindgen]
pub fn solve_json(tx: f32, ty: f32, list: bool, n: Option<u32>, sx: Option<f32>, sy: Option<f32>) -> String {
    STATE.with(|s| {
        let mut st = s.borrow_mut();
        let Some(st) = st.as_mut() else { return "{\"error\":\"no map loaded\"}".into() };
        let (target, count, rows_json) = match solved(st, tx, ty, list, sx, sy) {
            Ok(v) => v,
            Err(e) => return e,
        };
        let list_mode = list && sx.is_none();
        let nsel = n.map(|k| k as usize);
        let (_, _, lineups) = st.cache.as_ref().unwrap();
        let _ = rows_json;
        let row_idxs = api::row_indices(nsel, list_mode, lineups.len());
        let rows: Vec<String> = row_idxs.iter().map(|&i| api::row_json(i, &lineups[i], "[]")).collect();
        let _ = count;
        api::body_json(tx, ty, target.z, lineups.len(), &rows)
    })
}

/// Renders row `n` (1-based) of the identical solve: [aim, stand, wide] BMPs.
/// The aim image gets the UI-reference ring stamped, like the server does.
#[wasm_bindgen]
pub fn render_lineup(tx: f32, ty: f32, list: bool, n: u32, sx: Option<f32>, sy: Option<f32>) -> Result<js_sys::Array, JsValue> {
    STATE.with(|s| {
        let mut st = s.borrow_mut();
        let st = st.as_mut().ok_or("no map loaded")?;
        solved(st, tx, ty, list, sx, sy).map_err(JsValue::from)?;
        let (_, _, lineups) = st.cache.as_ref().unwrap();
        let l = lineups.get(n as usize - 1).ok_or("no such lineup")?;
        let cfg = Cfg::default();
        let eye = l.stand + V3::new(0.0, 0.0, cfg.eye_z);
        let aim = render::render_bytes(&st.vscene, eye, l.yaw, l.pitch);
        let stand = render::render_grid_bytes(&st.vscene, l.stand + V3::new(0.0, 0.0, 350.0), l.yaw, -89.0);
        let (wide_eye, wyaw, wpitch) = render::wide_cam(&st.vscene, l.stand, l.yaw);
        let wide = render::render_marked_bytes(&st.vscene, wide_eye, wyaw, wpitch, l.stand + V3::new(0.0, 0.0, 40.0));
        let arr = js_sys::Array::new();
        for b in [aim, stand, wide] {
            arr.push(&js_sys::Uint8Array::from(b.as_slice()));
        }
        Ok(arr)
    })
}

/// Keyframe stills of row `n`'s flight (the web stand-in for the native
/// flight video): 0.5s before the first bounce, then on each bounce and at
/// the apex between bounces, ending at rest. Returns 640x400 BMPs.
#[wasm_bindgen]
pub fn flight_stills(tx: f32, ty: f32, list: bool, n: u32, sx: Option<f32>, sy: Option<f32>) -> Result<js_sys::Array, JsValue> {
    STATE.with(|s| {
        let mut st = s.borrow_mut();
        let st = st.as_mut().ok_or("no map loaded")?;
        let (target, _, _) = solved(st, tx, ty, list, sx, sy).map_err(JsValue::from)?;
        let (_, _, lineups) = st.cache.as_ref().unwrap();
        let l = lineups.get(n as usize - 1).ok_or("no such lineup")?;
        let cfg = Cfg::default();
        let origin = crate::sim::hand_origin(l.stand + V3::new(0.0, 0.0, cfg.eye_z), l.yaw, &cfg);
        let lp = crate::sim::launch_pitch(l.pitch, &cfg);
        let (sy2, cy2) = l.yaw.to_radians().sin_cos();
        let (sp2, cp2) = lp.to_radians().sin_cos();
        let (_, traj, first_bounce, bounces) =
            crate::sim::fly_path_marks(&st.cscene, origin, V3::new(cp2 * cy2, cp2 * sy2, sp2), &cfg)
                .ok_or("flight failed")?;
        let arr = js_sys::Array::new();
        for b in render::flight_stills(&st.vscene, target, &traj, first_bounce, &bounces) {
            arr.push(&js_sys::Uint8Array::from(b.as_slice()));
        }
        Ok(arr)
    })
}

/// Flight trajectory of row `n` for the canvas animation:
/// {"traj":[[x,y,z],...],"first_bounce":K,"rest":[x,y,z],"time":T,"bounces":B}
#[wasm_bindgen]
pub fn traj_json(tx: f32, ty: f32, list: bool, n: u32, sx: Option<f32>, sy: Option<f32>) -> String {
    STATE.with(|s| {
        let mut st = s.borrow_mut();
        let Some(st) = st.as_mut() else { return "{\"error\":\"no map loaded\"}".into() };
        if let Err(e) = solved(st, tx, ty, list, sx, sy) {
            return e;
        }
        let (_, _, lineups) = st.cache.as_ref().unwrap();
        let Some(l) = lineups.get(n as usize - 1) else { return "{\"error\":\"no such lineup\"}".into() };
        let cfg = Cfg::default();
        let origin = crate::sim::hand_origin(l.stand + V3::new(0.0, 0.0, cfg.eye_z), l.yaw, &cfg);
        let lp = crate::sim::launch_pitch(l.pitch, &cfg);
        let (sy2, cy2) = l.yaw.to_radians().sin_cos();
        let (sp2, cp2) = lp.to_radians().sin_cos();
        let Some((out, traj, first_bounce)) =
            crate::sim::fly_path(&st.cscene, origin, V3::new(cp2 * cy2, cp2 * sy2, sp2), &cfg)
        else {
            return "{\"error\":\"flight failed\"}".into();
        };
        let pts: Vec<String> = traj.iter().map(|p| format!("[{:.0},{:.0},{:.0}]", p.x, p.y, p.z)).collect();
        format!(
            "{{\"traj\":[{}],\"first_bounce\":{first_bounce},\"rest\":[{:.0},{:.0},{:.0}],\"time\":{:.2},\"bounces\":{}}}",
            pts.join(","),
            out.rest.x, out.rest.y, out.rest.z, out.time, out.bounces
        )
    })
}

/// Ensure st.cache holds the solve for these params (serve.rs semantics:
/// ground-resolve the click, stands = locked pair or the whole map, browse
/// tol 1000 / paired 450). Returns (target, count, ()) or an error body.
fn solved(st: &mut St, tx: f32, ty: f32, list: bool, sx: Option<f32>, sy: Option<f32>) -> Result<(V3, usize, ()), String> {
    let stand = sx.zip(sy);
    let key = Key { tx, ty, stand, list };
    if let Some((k, t, l)) = st.cache.as_ref() {
        if *k == key {
            return Ok((*t, l.len(), ()));
        }
    }
    let list_mode = list && stand.is_none();
    let tol: f32 = if list_mode { 1000.0 } else { 450.0 };
    let cfg = Cfg::default();
    let Some(tz) = st.cscene.ground_z(tx, ty) else {
        return Err("{\"error\":\"no ground at target\"}".into());
    };
    let target = V3::new(tx, ty, tz);
    let stands_vec: Vec<V3> = if let Some((sx, sy)) = stand {
        let Some(sz) = st.cscene.stand_z(sx, sy) else {
            return Err("{\"error\":\"no ground at stand\"}".into());
        };
        vec![V3::new(sx, sy, sz)]
    } else {
        st.cscene.stands.clone()
    };
    let (min_dist, strict) = if stand.is_some() { (0.0, false) } else { (1800.0, true) };
    let lineups = solve::solve(&st.cscene, Some(&st.vscene), &stands_vec, target, tol, min_dist, strict, list_mode, &cfg);
    let count = lineups.len();
    st.cache = Some((key, target, lineups));
    Ok((target, count, ()))
}
