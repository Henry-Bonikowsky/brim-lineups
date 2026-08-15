//! The /solve JSON contract, shared verbatim by the native server (serve.rs)
//! and the wasm build (wasm.rs) so both emit byte-identical bodies.

use crate::scene::V3;
use crate::solve::Lineup;

/// Which rows to return / render: browse list = many rows, no renders;
/// n=K = that one row, rendered; default = top 5 rendered.
pub fn row_indices(nsel: Option<usize>, list_mode: bool, nlineups: usize) -> Vec<usize> {
    match nsel {
        Some(k) if k >= 1 && k <= nlineups => vec![k - 1],
        Some(_) => vec![],
        None if list_mode => (0..nlineups.min(20)).collect(),
        None => (0..nlineups.min(5)).collect(),
    }
}

pub fn row_json(i: usize, l: &Lineup, imgs: &str) -> String {
    let aim = l
        .aim_ref
        .map(|(p, d)| format!("[{:.0},{:.0},{:.0},{:.0}]", p.x, p.y, p.z, d))
        .unwrap_or("null".into());
    format!(
        "{{\"idx\":{},\"stand\":[{:.0},{:.0},{:.0}],\"rest\":[{:.0},{:.0},{:.0}],\"range\":{:.0},\"yaw\":{:.1},\"pitch\":{:.1},\"time\":{:.2},\"bounces\":{},\"err\":{:.0},\"covered\":{},\"forgive\":{:.2},\"spread\":{:.0},\"pos\":{},\"exposed\":{},\"graze\":{},\"approach\":{:.0},\"aim_ref\":{},\"imgs\":{imgs}}}",
        i + 1, l.stand.x, l.stand.y, l.stand.z, l.rest.x, l.rest.y, l.rest.z, l.dist, l.yaw, l.pitch, l.time, l.bounces, l.err, l.covered, l.forgive, l.spread, l.pos_grade(), l.exposed, l.graze, l.approach, aim
    )
}

pub fn body_json(tx: f32, ty: f32, tz: f32, count: usize, rows: &[String]) -> String {
    format!(
        "{{\"target\":[{tx:.0},{ty:.0},{tz:.0}],\"count\":{},\"lineups\":[{}]}}",
        count,
        rows.join(",")
    )
}

/// Suppress unused warning on non-wasm builds where V3 is only in signatures.
pub type _V3 = V3;
