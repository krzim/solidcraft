//! The toolpaths of the open job's last Generate in the 3D view (Manufacture workspace): one
//! colour per operation, the selected operation bright and the others dim (or only the selected
//! one, or none: *Show paths*), rapids dashed, and the mark of the last finding gone to. Part of
//! the model scene ([`draw`] from `viewport::build_scene`, keyed by [`scene_key`]); display
//! only, and drawn only while no simulation view is loaded (that layer draws the paths then).

use std::hash::{Hash, Hasher};

use solidcore_record::Vec3;
use solidcraft_engine::cam_job_state::OpPath;

use crate::SolveApp;
use crate::gpu::{BlockBuilder, GpuScene, SceneBlock};
use crate::manufacture_workspace::ShowPaths;
use crate::theme::Tokens;
use crate::viewport::c4;

/// Segments a scene block holds.
const CHUNK: usize = 4096;

/// Whether the layer draws: the workspace is shown, a job has a Generate, no simulation is loaded.
fn shown(app: &SolveApp) -> bool {
    crate::manufacture_workspace::active(app)
        && app.session.cam_view.is_none()
        && !app.cam_player.away()
        && app.session.cam_job.as_deref().is_some_and(|j| j.generated.is_some())
}

/// What the layer looks like, for the scene's key (0: nothing drawn).
pub(crate) fn scene_key(app: &SolveApp) -> u64 {
    let mark = app.manufacture.mark.map(|m| m.map(f64::to_bits));
    if !shown(app) {
        return mark.map_or(0, |m| hash_of(("mark", m)));
    }
    let Some(g) = app.session.cam_job.as_deref().and_then(|j| j.generated.as_ref()) else { return 0 };
    let points: usize = g.paths.iter().map(|p| p.points.len()).sum();
    hash_of(("paths", g.revision, g.model_revision, g.paths.len(), points, app.manufacture.selected, show_key(app.manufacture.show_paths), mark))
}

fn show_key(s: ShowPaths) -> u8 {
    match s {
        ShowPaths::All => 0,
        ShowPaths::Selected => 1,
        ShowPaths::None => 2,
    }
}

fn hash_of(parts: impl Hash) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    "manufacture-paths".hash(&mut h);
    parts.hash(&mut h);
    h.finish()
}

/// Whether operation `op`'s path is drawn, and bright.
pub(crate) fn path_look(selected: Option<usize>, show: ShowPaths, op: usize) -> Option<bool> {
    match (show, selected) {
        (ShowPaths::None, _) => None,
        (ShowPaths::Selected, Some(s)) if s != op => None,
        (ShowPaths::Selected, None) => None,
        (_, None) => Some(true),
        (_, Some(s)) => Some(s == op),
    }
}

/// The segments of a path: (from, to, rapid).
pub(crate) fn segments(path: &OpPath) -> Vec<([f32; 3], [f32; 3], bool)> {
    path.points.windows(2).zip(path.rapid.iter().skip(1)).map(|(w, rapid)| (w[0], w[1], *rapid)).collect()
}

pub(crate) fn draw(app: &SolveApp, sc: &mut GpuScene) {
    let t = Tokens::view();
    if let Some(m) = app.manufacture.mark.filter(|_| crate::manufacture_workspace::active(app)) {
        let mut bb = BlockBuilder::default();
        let c = sc.color(c4(t.error));
        bb.marker([m[0] as f32, m[1] as f32, m[2] as f32], c, 10.0, false);
        sc.blocks.push(SceneBlock { key: hash_of(("mark-block", m.map(f64::to_bits))), bytes: bb.freeze() });
    }
    if !shown(app) {
        return;
    }
    let Some(g) = app.session.cam_job.as_deref().and_then(|j| j.generated.as_ref()) else { return };
    let dim = sc.color(c4(t.text_dim.gamma_multiply(0.6)));
    let rapid = sc.color(c4(t.error.gamma_multiply(0.8)));
    let size = extent(&g.paths).max(1.0);
    let (dash, gap) = (size / 150.0, size / 200.0);
    // An indexed pattern plans one path per angle, all of one operation: each is a block of its own.
    for (n, path) in g.paths.iter().enumerate() {
        let Some(bright) = path_look(app.manufacture.selected, app.manufacture.show_paths, path.op) else { continue };
        let lit = sc.color(c4(crate::cam_view_layer::category(&t, path.op)));
        let (col, width) = if bright { (lit, 1.6) } else { (dim, 1.0) };
        let segs = segments(path);
        for (k, chunk) in segs.chunks(CHUNK).enumerate() {
            let mut bb = BlockBuilder::default();
            for (a, b, is_rapid) in chunk {
                if *is_rapid {
                    if !bright {
                        continue;
                    }
                    let pts = [v3(*a), v3(*b)];
                    for (x, y) in crate::viewport::dashes(&pts, &[dash, gap]) {
                        bb.line(f3(x), f3(y), rapid, 1.0);
                    }
                } else {
                    bb.line(*a, *b, col, width);
                }
            }
            if let (Some(first), true) = (path.points.first(), k == 0 && bright) {
                bb.marker(*first, lit, 6.0, false);
            }
            sc.blocks.push(SceneBlock { key: hash_of(("block", g.revision, n, path.op, k, bright)), bytes: bb.freeze() });
        }
    }
}

fn v3(p: [f32; 3]) -> Vec3 {
    Vec3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))
}

fn f3(p: Vec3) -> [f32; 3] {
    [p.x as f32, p.y as f32, p.z as f32]
}

/// The largest side of the box round every path (mm).
fn extent(paths: &[OpPath]) -> f64 {
    let mut lo = [f32::INFINITY; 3];
    let mut hi = [f32::NEG_INFINITY; 3];
    for p in paths.iter().flat_map(|p| &p.points) {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    (0..3).map(|k| f64::from(hi[k] - lo[k])).filter(|d| d.is_finite()).fold(0.0, f64::max)
}
