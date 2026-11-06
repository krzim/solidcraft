//! The CAM simulation view in the 3D view (state in `solidcraft_engine::cam_view`, commands
//! `cam.view.*`): the stock as cut so far, the toolpaths (one colour per operation or tool, the
//! part already played bright and the rest dim, rapids dashed, plunges marked), the tool with
//! its shank and holder and the spindle's parts at the tool's pose, the workholding see-through,
//! and the findings so far as marks. It is part of the model scene ([`draw`] from
//! `viewport::build_scene`), keyed by [`scene_key`]; it never handles input (the viewport's
//! `navigation::handle` stays the only camera path).
//!
//! What a frame of playback changes is small, and only that is built and uploaded again:
//! - the stock is drawn in tiles (`solidcraft_cam::stock_mesh`), grouped [`GROUP`] × [`GROUP`]
//!   into one scene block each; a block is kept while its tiles are as they were. The tiles
//!   themselves are built off the window thread by the player (`cam_view_player`) after each
//!   step, into [`Tiles`]; what is still missing here (a headless session) is built at once;
//! - the toolpaths are blocks of [`CHUNK`] moves: only the block the tool is in changes;
//! - the tool, holder and spindle parts are instanced meshes: their triangles stay on the GPU,
//!   only their placements change.
//!
//! Display only: nothing here decides anything about the model (findings come from the exact
//! simulation in `solidcraft_cam::playback`).

use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use std::sync::{Arc, Mutex};

use egui::Color32;
use solidcore_record::Vec3;
use solidcraft_engine::cam::findings::Severity;
use solidcraft_engine::cam::machine_model::{MachineModel, Mount, ObstacleKind, PartShape, nose_above_tip};
use solidcraft_engine::cam::playback::{MoveKind, Playback};
use solidcraft_engine::cam::sim::{Grid, SimPlan, SimState};
use solidcraft_engine::cam::stock_mesh::{display_step, tile_mesh};
use solidcraft_engine::cam_view::CamView;
use solidcraft_engine::geom::Mesh;
use solidcraft_engine::geom::prelude::*;
use solidcraft_engine::schema::cam_view::ColourBy;

use crate::SolveApp;
use crate::gpu::{BlockBuilder, GpuScene, InstanceLook, InstancedMesh, PaletteIndex, SceneBlock, SharedTris, TriClass};
use crate::theme::Tokens;
use crate::viewport::c4;

/// Display cells along the stock's longer side at most (the tiles' step follows).
pub(crate) const MAX_SIDE: usize = 384;
/// Tiles a stock block spans along each side.
pub(crate) const GROUP: usize = 4;
/// Moves a toolpath block holds.
pub(crate) const CHUNK: usize = 1024;
/// Points a move's drawn path has at most.
const MAX_SAMPLES: usize = 64;

/// Which loaded view a cache belongs to: its plan's address and size (a new load is a new plan).
pub(crate) fn view_id(v: &CamView) -> u64 {
    v.playback().map_or(0, |p| plan_id(p.plan()))
}

fn plan_id(plan: &SimPlan) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    (plan as *const SimPlan as usize).hash(&mut h);
    plan.moves().hash(&mut h);
    plan.cells().hash(&mut h);
    h.finish()
}

/// The stock's tiles as display meshes, kept between frames and shared with the player's worker.
#[derive(Default)]
pub(crate) struct Tiles {
    view: u64,
    generation: u64,
    step: usize,
    /// Changes whenever a tile is built again.
    pub(crate) changed: u64,
    tiles: HashMap<(usize, usize), ([u32; 3], Arc<Mesh>)>,
}

impl Tiles {
    fn mesh(&self, view: u64, generation: u64, step: usize, at: (usize, usize), key: [u32; 3]) -> Option<&Arc<Mesh>> {
        if (self.view, self.generation, self.step) != (view, generation, step) {
            return None;
        }
        self.tiles.get(&at).filter(|(k, _)| *k == key).map(|(_, m)| m)
    }
}

/// A tile's key: its own version and those of the tiles it draws walls against (+X and +Y, or
/// round on a 4th axis).
fn tile_key(plan: &SimPlan, st: &SimState, ti: usize, tj: usize) -> [u32; 3] {
    let (tx, ty) = plan.grid().tiles();
    let radial = matches!(plan.grid(), Grid::Radii { .. });
    let x = if ti + 1 < tx { st.tile_version(ti + 1, tj) } else { 0 };
    let y = if tj + 1 < ty {
        st.tile_version(ti, tj + 1)
    } else if radial {
        st.tile_version(ti, 0)
    } else {
        0
    };
    [st.tile_version(ti, tj), x, y]
}

/// Build again the tiles of `v`'s stock that changed since they were last built, into `tiles`.
/// Returns how many were built. Called by the player's worker after each step, so the window
/// thread finds them ready.
pub(crate) fn refresh_tiles(v: &CamView, tiles: &Mutex<Tiles>) -> usize {
    let Some(p) = v.playback() else { return 0 };
    let plan = p.plan();
    let st = p.state();
    let (view, generation, step) = (view_id(v), p.generation(), display_step(plan, MAX_SIDE));
    let (tx, ty) = plan.grid().tiles();
    let wanted: Vec<((usize, usize), [u32; 3])> =
        (0..ty).flat_map(|tj| (0..tx).map(move |ti| (ti, tj))).map(|(ti, tj)| ((ti, tj), tile_key(plan, st, ti, tj))).collect();
    let stale: Vec<((usize, usize), [u32; 3])> = {
        let Ok(t) = tiles.lock() else { return 0 };
        wanted.into_iter().filter(|(at, key)| t.mesh(view, generation, step, *at, *key).is_none()).collect()
    };
    if stale.is_empty() {
        return 0;
    }
    let built: Vec<((usize, usize), [u32; 3], Arc<Mesh>)> =
        stale.into_iter().map(|(at, key)| (at, key, Arc::new(tile_mesh(plan, st, at.0, at.1, step)))).collect();
    let n = built.len();
    let Ok(mut t) = tiles.lock() else { return 0 };
    if (t.view, t.generation, t.step) != (view, generation, step) {
        t.tiles.clear();
        (t.view, t.generation, t.step) = (view, generation, step);
    }
    for (at, key, mesh) in built {
        t.tiles.insert(at, (key, mesh));
    }
    t.changed = t.changed.wrapping_add(1);
    n
}

/// What the layer drew last, kept so that unchanged blocks keep their bytes (and their GPU
/// buffers) and so that it can be drawn again while the view is away on the player's worker.
#[derive(Default)]
pub(crate) struct LayerCache {
    view: u64,
    /// Blocks by key, as last drawn.
    blocks: HashMap<u64, SceneBlock>,
    /// Each tool's and machine part's triangles, by (tool number or part, piece).
    tris: HashMap<(u32, u8), Arc<SharedTris>>,
    /// The last frame's drawing, for frames while the view is away.
    last: Option<Drawn>,
}

#[derive(Clone, Default)]
struct Drawn {
    blocks: Vec<SceneBlock>,
    instanced: Vec<(u64, Arc<SharedTris>, Vec<([[f32; 4]; 3], [u8; 4])>)>,
    overlay: Vec<([f32; 3], [f32; 3], [u8; 4], f32)>,
}

/// Whether the design's own bodies are drawn: not while a CAM view hides them (its `part` layer
/// off), the stock being cut where the part will be.
pub(crate) fn part_shown(app: &SolveApp) -> bool {
    match app.session.cam_view.as_deref() {
        Some(v) => v.layers.part,
        None => !app.cam_player.away() || app.cam_player.held_part(),
    }
}

/// What the model scene's key takes from the CAM view: anything it draws changing.
pub(crate) fn scene_key(app: &SolveApp) -> u64 {
    let Some(v) = app.session.cam_view.as_deref() else {
        return if app.cam_player.away() { app.cam_player.held_key() } else { 0 };
    };
    key_of(v, &app.cam_player.tiles)
}

/// The key of view `v` as it is now (the player keeps it while the view is away).
pub(crate) fn key_of(v: &CamView, tiles: &Mutex<Tiles>) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    view_id(v).hash(&mut h);
    v.frame.hash(&mut h);
    format!("{:?}{:?}", v.layers, v.colour_by).hash(&mut h);
    if let Some(p) = v.playback() {
        p.generation().hash(&mut h);
        let (m, f) = p.at();
        (m, f.to_bits()).hash(&mut h);
    }
    tiles.lock().map_or(0, |t| t.changed).hash(&mut h);
    h.finish()
}

/// The theme's colours for categories (operations, tools), in order.
pub(crate) fn series(t: &Tokens) -> [Color32; 8] {
    [t.accent, t.axis_y, t.warning, t.surface_accent, t.mesh_accent, t.render_accent, t.form_accent, t.sketch_accent]
}

/// The colour of operation `op` (by place) or tool slot `slot` (by its order of use), as the
/// view colours paths.
pub(crate) fn category(t: &Tokens, k: usize) -> Color32 {
    let s = series(t);
    s.get(k % s.len()).copied().unwrap_or(t.accent)
}

/// Each tool slot's place in the order the program uses them.
pub(crate) fn slot_order(p: &Playback) -> Vec<u8> {
    let tl = p.timeline();
    let mut used: Vec<u8> = Vec::new();
    for m in tl.tool_changes.iter().filter_map(|c| tl.moves.get(c.at_move)).chain(tl.moves.first()) {
        if !used.contains(&m.slot) {
            used.push(m.slot);
        }
    }
    used
}

fn order_of(used: &[u8], slot: u8) -> usize {
    used.iter().position(|u| *u == slot).unwrap_or(0)
}

/// A theme colour at opacity `alpha` (of 255), for see-through faces.
fn see_through(c: Color32, alpha: u8) -> [u8; 4] {
    let [r, g, b, _] = c4(c);
    [r, g, b, alpha]
}

fn f3(p: Vec3) -> [f32; 3] {
    [p.x as f32, p.y as f32, p.z as f32]
}

fn hash_of(parts: impl Hash) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    "cam-view".hash(&mut h);
    parts.hash(&mut h);
    h.finish()
}

/// Draw the CAM view into the model scene `sc`.
pub(crate) fn draw(app: &SolveApp, sc: &mut GpuScene) {
    let Ok(mut cache) = app.cam_player.layer.lock() else { return };
    let Some(v) = app.session.cam_view.as_deref() else {
        // Away on the worker: what was drawn last, as it was.
        if app.cam_player.away()
            && let Some(last) = cache.last.clone()
        {
            emit(sc, &last);
        } else {
            *cache = LayerCache::default();
        }
        return;
    };
    let id = view_id(v);
    if cache.view != id {
        *cache = LayerCache { view: id, ..Default::default() };
    }
    let Some(p) = v.playback() else { return };
    let t = Tokens::view();
    let mut drawn = Drawn::default();
    let mut blocks = std::mem::take(&mut cache.blocks);
    let mut kept: HashMap<u64, SceneBlock> = HashMap::new();
    let mut take = |key: u64, build: &mut dyn FnMut() -> Option<BlockBuilder>| -> Option<SceneBlock> {
        let b = match blocks.remove(&key) {
            Some(b) => b,
            None => SceneBlock { key, bytes: build()?.freeze() },
        };
        kept.insert(key, b.clone());
        Some(b)
    };
    let origin = v.origin;
    if v.layers.stock {
        stock_blocks(app, v, p, sc, &t, &mut take, &mut drawn);
    }
    if v.layers.toolpaths || v.layers.rapids {
        path_blocks(v, p, sc, &t, &mut take, &mut drawn);
    }
    if v.layers.fixtures {
        fixture_block(p, sc, &t, origin, &mut take, &mut drawn);
    }
    tool_parts(v, p, &t, &mut cache.tris, &mut drawn);
    if v.layers.gouges || v.layers.collisions {
        marks(v, p, &t, &mut drawn);
    }
    cache.blocks = kept;
    emit(sc, &drawn);
    cache.last = Some(drawn);
}

/// Put `d` into the scene.
fn emit(sc: &mut GpuScene, d: &Drawn) {
    sc.blocks.extend(d.blocks.iter().cloned());
    for (key, tris, places) in &d.instanced {
        let mut m =
            InstancedMesh { key: *key, tris: tris.clone(), instances: Vec::new(), look: InstanceLook::Opaque, levels: Vec::new(), closed: true };
        for (rows, rgba) in places {
            let c = sc.color(*rgba);
            m.place(*rows, c);
        }
        sc.instanced.push(m);
    }
    for (a, b, rgba, w) in &d.overlay {
        let c = sc.color(*rgba);
        sc.line(*a, *b, c, *w, true);
    }
}

type Take<'a> = dyn FnMut(u64, &mut dyn FnMut() -> Option<BlockBuilder>) -> Option<SceneBlock> + 'a;

/// The stock, a block per [`GROUP`] × [`GROUP`] tiles, coloured by the tool that cut it last.
fn stock_blocks(app: &SolveApp, v: &CamView, p: &Playback, sc: &mut GpuScene, t: &Tokens, take: &mut Take<'_>, drawn: &mut Drawn) {
    let plan = p.plan();
    let st = p.state();
    let used = slot_order(p);
    // Colour classes: uncut, then each slot.
    let mut classes: Vec<PaletteIndex> = vec![sc.color(c4(t.body))];
    for slot in 0..=u8::MAX.saturating_sub(1) {
        if plan.tool_number(slot).is_none() {
            break;
        }
        classes.push(sc.color(c4(category(t, order_of(&used, slot)))));
    }
    let signature: Vec<u32> = classes.iter().map(|c| c.0).collect();
    let (view, generation, step) = (view_id(v), p.generation(), display_step(plan, MAX_SIDE));
    let (tx, ty) = plan.grid().tiles();
    let origin = v.origin;
    for gj in (0..ty).step_by(GROUP) {
        for gi in (0..tx).step_by(GROUP) {
            let members: Vec<((usize, usize), [u32; 3])> = (gj..(gj + GROUP).min(ty))
                .flat_map(|tj| (gi..(gi + GROUP).min(tx)).map(move |ti| (ti, tj)))
                .map(|(ti, tj)| ((ti, tj), tile_key(plan, st, ti, tj)))
                .collect();
            let key = hash_of(("stock", view, generation, step, gi, gj, &members, &signature));
            let block = take(key, &mut || {
                let mut meshes = Vec::with_capacity(members.len());
                for (at, k) in &members {
                    let cached = app.cam_player.tiles.lock().ok().and_then(|tl| tl.mesh(view, generation, step, *at, *k).cloned());
                    // Not built by the worker (a headless session): built here.
                    meshes.push(cached.unwrap_or_else(|| Arc::new(tile_mesh(plan, st, at.0, at.1, step))));
                }
                let mut b = BlockBuilder::default();
                b.reserve_triangles(meshes.iter().map(|m| m.triangles.len()).sum());
                b.begin_body();
                for m in &meshes {
                    for (k, tri) in m.triangles.iter().enumerate() {
                        let [Some(a), Some(bb), Some(c)] = tri.map(|i| m.positions.get(i as usize).copied()) else { continue };
                        let n = (bb - a).cross(c - a).normalized().unwrap_or(Vec3::new(0.0, 0.0, 1.0));
                        let class = m.tri_face.get(k).copied().unwrap_or(0) as usize;
                        let col = classes.get(class).or_else(|| classes.first()).copied().unwrap_or(PaletteIndex(0));
                        b.triangle([f3(a + origin), f3(bb + origin), f3(c + origin)], [f3(n); 3], col, TriClass::Opaque);
                    }
                }
                b.end_body();
                Some(b)
            });
            if let Some(b) = block {
                drawn.blocks.push(b);
            }
        }
    }
}

/// The part of move `k` from fraction `f0` to `f1`, as points (work coordinates).
fn move_points(plan: &SimPlan, k: usize, f0: f64, f1: f64, spacing: f64) -> Vec<Vec3> {
    let samples = plan.samples(k, spacing, MAX_SAMPLES);
    let n = samples.len();
    if n < 2 {
        return samples.into_iter().map(|q| q.tip).collect();
    }
    let mut out = Vec::with_capacity(n + 2);
    if let Some(q) = plan.pose(k, f0) {
        out.push(q.tip);
    }
    for (i, q) in samples.iter().enumerate() {
        let at = i as f64 / (n - 1) as f64;
        if at > f0 && at < f1 {
            out.push(q.tip);
        }
    }
    if let Some(q) = plan.pose(k, f1) {
        out.push(q.tip);
    }
    out
}

/// The toolpaths, a block per [`CHUNK`] moves: done bright in the operation's (or the tool's)
/// colour, to come dim, rapids dashed, each plunge's foot marked.
fn path_blocks(v: &CamView, p: &Playback, sc: &mut GpuScene, t: &Tokens, take: &mut Take<'_>, drawn: &mut Drawn) {
    let plan = p.plan();
    let tl = p.timeline();
    let (now, frac) = p.at();
    let n = tl.moves.len();
    let used = slot_order(p);
    let by_op = v.colour_by == ColourBy::Operation;
    let ops = tl.ops.len().max(1);
    let cats: Vec<PaletteIndex> = (0..series(t).len()).map(|k| sc.color(c4(category(t, k)))).collect();
    let rapid = sc.color(c4(t.error));
    let rest = sc.color(c4(t.text_dim.gamma_multiply(0.7)));
    let plunge = sc.color(c4(t.warning));
    let signature: Vec<u32> = cats.iter().map(|c| c.0).chain([rapid.0, rest.0, plunge.0]).collect();
    let cell = plan.cell().max(1e-3);
    let spacing = (cell * 8.0).max(0.2);
    // Dashes a fixed share of the stock's size, so they read the same on any part.
    let size = {
        let g = plan.grid();
        (g.columns().max(g.rows()) as f64 * cell).max(1.0)
    };
    let (dash, gap) = (size / 120.0, size / 160.0);
    let layers = (v.layers.toolpaths, v.layers.rapids, v.layers.remaining_path);
    let origin = v.origin;
    for a in (0..n).step_by(CHUNK) {
        let b = (a + CHUNK).min(n);
        // Where the tool is, within this chunk: all done, none, or split.
        let state: (u8, usize, u64) = if now >= b {
            (2, 0, 0)
        } else if now < a {
            (0, 0, 0)
        } else {
            (1, now, frac.to_bits())
        };
        let key = hash_of(("path", view_id(v), a, state, layers, by_op, ops, &signature));
        let block = take(key, &mut || {
            let mut bb = BlockBuilder::default();
            for k in a..b {
                let Some(m) = tl.moves.get(k) else { continue };
                let is_rapid = m.kind == MoveKind::Rapid;
                if (is_rapid && !v.layers.rapids) || (!is_rapid && !v.layers.toolpaths) {
                    continue;
                }
                let cat = if by_op { m.op.unwrap_or(0) } else { order_of(&used, m.slot) };
                let lit = cats.get(cat % cats.len().max(1)).copied().unwrap_or(rest);
                let pieces: Vec<(f64, f64, bool)> = if k < now {
                    vec![(0.0, 1.0, true)]
                } else if k > now {
                    vec![(0.0, 1.0, false)]
                } else {
                    vec![(0.0, frac, true), (frac, 1.0, false)]
                };
                for (f0, f1, done) in pieces {
                    if f1 <= f0 || (!done && !v.layers.remaining_path) {
                        continue;
                    }
                    let pts: Vec<Vec3> = move_points(plan, k, f0, f1, spacing).into_iter().map(|q| q + origin).collect();
                    let (col, width) = match (done, is_rapid) {
                        (true, true) => (rapid, 1.0),
                        (true, false) => (lit, 1.8),
                        (false, _) => (rest, 1.0),
                    };
                    if is_rapid {
                        for (x, y) in crate::viewport::dashes(&pts, &[dash, gap]) {
                            bb.line(f3(x), f3(y), col, width);
                        }
                    } else {
                        for w in pts.windows(2) {
                            if let [x, y] = w {
                                bb.line(f3(*x), f3(*y), col, width);
                            }
                        }
                    }
                    if m.kind == MoveKind::Plunge
                        && f1 >= 1.0
                        && let Some(foot) = pts.last()
                    {
                        bb.marker(f3(*foot), if done { plunge } else { rest }, 5.0, false);
                    }
                }
            }
            Some(bb)
        });
        if let Some(bl) = block {
            drawn.blocks.push(bl);
        }
    }
}

/// A box as triangles (outward) and its twelve edges, its corners moved by `at`.
fn box_into(b: &mut BlockBuilder, min: Vec3, max: Vec3, at: &dyn Fn(Vec3) -> Vec3, fill: PaletteIndex, edge: PaletteIndex) {
    let corner = |k: u32| {
        at(Vec3::new(if k & 1 == 0 { min.x } else { max.x }, if k & 2 == 0 { min.y } else { max.y }, if k & 4 == 0 { min.z } else { max.z }))
    };
    for [a, bb, c, d] in [[0u32, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]] {
        let (pa, pb, pc, pd) = (corner(a), corner(bb), corner(c), corner(d));
        let n = f3((pb - pa).cross(pc - pa).normalized().unwrap_or(Vec3::new(0.0, 0.0, 1.0)));
        b.triangle([f3(pa), f3(pb), f3(pc)], [n; 3], fill, TriClass::Faded);
        b.triangle([f3(pa), f3(pc), f3(pd)], [n; 3], fill, TriClass::Faded);
    }
    for [a, c] in [[0u32, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]] {
        b.line(f3(corner(a)), f3(corner(c)), edge, 1.0);
    }
}

/// The workholding and the table, see-through with their edges; on a 4th axis turned with the
/// work (by -A about X).
fn fixture_block(p: &Playback, sc: &mut GpuScene, t: &Tokens, origin: Vec3, take: &mut Take<'_>, drawn: &mut Drawn) {
    let Some(scene) = p.setup().scene.clone() else { return };
    let plan = p.plan();
    let (now, frac) = p.at();
    let rotary = matches!(plan.grid(), Grid::Radii { .. });
    let a = if rotary { plan.joints(now, frac).and_then(|j| j.rotary.first().copied()).unwrap_or(0.0) } else { 0.0 };
    let fixture = sc.color(see_through(t.construction_geom, 70));
    let table = sc.color(see_through(t.grid_major, 45));
    let edge = sc.color(c4(t.construction_geom));
    let key =
        hash_of(("fixtures", plan_id(plan), a.to_bits(), fixture.0, table.0, edge.0, origin.x.to_bits(), origin.y.to_bits(), origin.z.to_bits()));
    let block = take(key, &mut || {
        let (sa, ca) = (-a).sin_cos();
        let turn = move |q: Vec3| if rotary { Vec3::new(q.x, q.y * ca - q.z * sa, q.y * sa + q.z * ca) + origin } else { q + origin };
        let mut b = BlockBuilder::default();
        for o in &scene.obstacles {
            let fill = if o.kind == ObstacleKind::Table { table } else { fixture };
            box_into(&mut b, o.min, o.max, &turn, fill, edge);
        }
        Some(b)
    });
    if let Some(b) = block {
        drawn.blocks.push(b);
    }
}

/// Two unit vectors square to `a` and to each other (as the engine's picture places the tool).
fn frame_of(a: Vec3) -> (Vec3, Vec3) {
    let pick = if a.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let u = a.cross(pick).normalized().unwrap_or(Vec3::new(1.0, 0.0, 0.0));
    (u, a.cross(u))
}

/// The rows of the rigid move that puts a part's origin at `at` and its +Z along `axis`.
pub(crate) fn placement(at: Vec3, axis: Vec3) -> [[f32; 4]; 3] {
    let a = axis.normalized().unwrap_or(Vec3::new(0.0, 0.0, 1.0));
    let (u, v) = frame_of(a);
    [
        [u.x as f32, v.x as f32, a.x as f32, at.x as f32],
        [u.y as f32, v.y as f32, a.y as f32, at.y as f32],
        [u.z as f32, v.z as f32, a.z as f32, at.z as f32],
    ]
}

/// `m` with a normal at every corner: the corners of faces less than about 35 degrees apart
/// share one (round surfaces read round), sharper edges keep both faces' own. Degenerate
/// triangles are left out.
pub(crate) fn shaded(m: &Mesh) -> Mesh {
    let normal = |t: &[u32; 3]| -> Option<Vec3> {
        let [a, b, c] = t.map(|i| m.positions.get(i as usize).copied());
        let (a, b, c) = (a?, b?, c?);
        (b - a).cross(c - a).normalized()
    };
    let faces: Vec<Option<Vec3>> = m.triangles.iter().map(normal).collect();
    let mut around: HashMap<u32, Vec<usize>> = HashMap::new();
    for (k, t) in m.triangles.iter().enumerate() {
        if faces.get(k).copied().flatten().is_some() {
            for i in t {
                around.entry(*i).or_default().push(k);
            }
        }
    }
    let mut out = Mesh::default();
    let cos = 35f64.to_radians().cos();
    for (k, t) in m.triangles.iter().enumerate() {
        let Some(n) = faces.get(k).copied().flatten() else { continue };
        let Ok(base) = u32::try_from(out.positions.len()) else { break };
        for i in t {
            let Some(p) = m.positions.get(*i as usize) else { continue };
            let mut sum = Vec3::new(0.0, 0.0, 0.0);
            for o in around.get(i).into_iter().flatten() {
                if let Some(on) = faces.get(*o).copied().flatten()
                    && on.dot(n) >= cos
                {
                    sum += on;
                }
            }
            out.positions.push(*p);
            out.normals.push(sum.normalized().unwrap_or(n));
        }
        if out.positions.len() == base as usize + 3 {
            out.triangles.push([base, base + 1, base + 2]);
        } else {
            out.positions.truncate(base as usize);
            out.normals.truncate(base as usize);
        }
    }
    out
}

/// A closed upright cylinder (a machine part) from `z0` to `z1` about (`cx`, `cy`).
fn cylinder(cx: f64, cy: f64, r: f64, z0: f64, z1: f64) -> Mesh {
    let n = 32u32;
    let mut m = Mesh::default();
    for z in [z0, z1] {
        m.positions.push(Vec3::new(cx, cy, z));
        for s in 0..n {
            let (sa, ca) = (f64::from(s) * std::f64::consts::TAU / f64::from(n)).sin_cos();
            m.positions.push(Vec3::new(cx + r * ca, cy + r * sa, z));
        }
    }
    let top = n + 1;
    for s in 0..n {
        let (a, b) = (1 + s, 1 + (s + 1) % n);
        m.triangles.push([0, b, a]);
        m.triangles.push([top, top + a, top + b]);
        m.triangles.push([a, b, top + b]);
        m.triangles.push([a, top + b, top + a]);
    }
    m
}

fn box_mesh(min: Vec3, max: Vec3) -> Mesh {
    let mut m = Mesh::default();
    for k in 0..8u32 {
        m.positions.push(Vec3::new(
            if k & 1 == 0 { min.x } else { max.x },
            if k & 2 == 0 { min.y } else { max.y },
            if k & 4 == 0 { min.z } else { max.z },
        ));
    }
    for [a, b, c, d] in [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]] {
        m.triangles.push([a, b, c]);
        m.triangles.push([a, c, d]);
    }
    m
}

/// A spindle-mounted part of the machine model as a mesh in the spindle frame (nose at the
/// origin, up +Z).
fn part_mesh(shape: &PartShape) -> Mesh {
    match shape {
        PartShape::Box { min, max } => box_mesh(Vec3::new(min[0], min[1], min[2]), Vec3::new(max[0], max[1], max[2])),
        PartShape::Cylinder { diameter, z, center } => {
            let c = center.unwrap_or([0.0, 0.0]);
            cylinder(c[0], c[1], diameter * 0.5, z[0], z[1])
        }
    }
}

/// Pieces of a tool's drawing, for the triangle cache's key.
const CUTTING: u8 = 0;
const SHANK: u8 = 1;
const HOLDER: u8 = 2;
/// Machine parts: by their place in the list, after these.
const MACHINE: u8 = 8;

fn cached_tris(cache: &mut HashMap<(u32, u8), Arc<SharedTris>>, key: (u32, u8), mesh: impl FnOnce() -> Mesh) -> Option<Arc<SharedTris>> {
    if let Some(t) = cache.get(&key) {
        return Some(t.clone());
    }
    let m = shaded(&mesh());
    if m.triangles.is_empty() {
        return None;
    }
    let t = Arc::new(SharedTris::new(&m));
    cache.insert(key, t.clone());
    Some(t)
}

/// The tool, its shank and holder, and the machine's spindle parts at the pose now.
fn tool_parts(v: &CamView, p: &Playback, t: &Tokens, cache: &mut HashMap<(u32, u8), Arc<SharedTris>>, drawn: &mut Drawn) {
    let Some(pose) = p.pose_now() else { return };
    let plan = p.plan();
    let (now, _) = p.at();
    let Some(tool) = plan.step(now).and_then(|s| p.tool(s.slot)) else { return };
    let at = pose.tip + v.origin;
    let rows = placement(at, pose.axis);
    let steel = c4(t.text_dim);
    let holder = c4(t.text.gamma_multiply(0.75).to_opaque());
    if let Some(meshes) = v.tool_mesh(tool.number) {
        let pieces: [(u8, &Mesh, [u8; 4], bool); 3] = [
            (CUTTING, &meshes.cutting, steel, v.layers.tool),
            (SHANK, &meshes.shank, steel, v.layers.tool),
            (HOLDER, &meshes.holder, holder, v.layers.holder),
        ];
        for (piece, mesh, rgba, on) in pieces {
            if !on || mesh.triangles.is_empty() {
                continue;
            }
            if let Some(tris) = cached_tris(cache, (tool.number, piece), || mesh.clone()) {
                drawn.instanced.push((hash_of(("tool", tool.number, piece)), tris, vec![(rows, rgba)]));
            }
        }
    }
    if !v.layers.machine {
        return;
    }
    let Some(model) = p.setup().model.clone() else { return };
    let nose = at + pose.axis * nose_above_tip(tool, Some(model.as_ref()));
    let nose_rows = placement(nose, pose.axis);
    spindle_parts(v, &model, t, cache, nose_rows, drawn);
}

fn spindle_parts(
    v: &CamView,
    model: &MachineModel,
    t: &Tokens,
    cache: &mut HashMap<(u32, u8), Arc<SharedTris>>,
    rows: [[f32; 4]; 3],
    drawn: &mut Drawn,
) {
    let meshes: Vec<(usize, &solidcraft_engine::cam::machine_model::MeshPart)> =
        v.machine_meshes.iter().enumerate().filter(|(_, m)| m.mount == Mount::Spindle).collect();
    if meshes.is_empty() {
        let grey = c4(t.text_dim.gamma_multiply(0.6).to_opaque());
        for (k, part) in model.parts.iter().enumerate().filter(|(_, q)| q.mount == Mount::Spindle) {
            let piece = MACHINE.saturating_add(u8::try_from(k).unwrap_or(u8::MAX - MACHINE));
            if let Some(tris) = cached_tris(cache, (u32::MAX, piece), || part_mesh(&part.shape)) {
                drawn.instanced.push((hash_of(("machine", k)), tris, vec![(rows, grey)]));
            }
        }
        return;
    }
    for (k, mp) in meshes {
        let piece = MACHINE.saturating_add(u8::try_from(k).unwrap_or(u8::MAX - MACHINE));
        // The machine file's own colour for its part (data, not the theme's).
        let byte = |x: f64| (x.clamp(0.0, 1.0) * 255.0).round() as u8;
        let rgba = [byte(mp.color[0]), byte(mp.color[1]), byte(mp.color[2]), 255];
        if let Some(tris) = cached_tris(cache, (u32::MAX - 1, piece), || mp.mesh.clone()) {
            drawn.instanced.push((hash_of(("machine-mesh", k)), tris, vec![(rows, rgba)]));
        }
    }
}

/// The findings so far as crosses over everything, in their severity's colour.
fn marks(v: &CamView, p: &Playback, t: &Tokens, drawn: &mut Drawn) {
    let (now, frac) = p.at();
    let end = if frac >= 1.0 { now + 1 } else { now };
    let size = (p.plan().cell().max(0.2) * 6.0) as f32;
    for e in p.findings().until(end) {
        let Some(at) = e.at else { continue };
        let gouge = matches!(e.code.as_str(), "gouge" | "rapid-in-stock" | "cut-spindle-off" | "shank-rubs");
        if (gouge && !v.layers.gouges) || (!gouge && !v.layers.collisions) {
            continue;
        }
        let rgba = c4(if e.severity == Severity::Error { t.error } else { t.warning });
        let c = f3(at + v.origin);
        for d in [[size, 0.0, 0.0], [0.0, size, 0.0], [0.0, 0.0, size]] {
            drawn.overlay.push(([c[0] - d[0], c[1] - d[1], c[2] - d[2]], [c[0] + d[0], c[1] + d[1], c[2] + d[2]], rgba, 2.6));
        }
    }
}

#[cfg(test)]
#[path = "cam_view_layer_tests.rs"]
pub(crate) mod tests;
