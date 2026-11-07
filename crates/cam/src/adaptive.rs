//! Adaptive clearing: an engagement-limited way of clearing a region at a level, for machines
//! that cannot take a full-width cut (a small, not very rigid mill above all).
//!
//! The material still to cut at a level is kept as a raster of cells over the region: a cell is
//! material while no tool position so far has come within the tool's radius of its centre. The
//! cutter's centre is confined to where it may stand (inside a pocket's walls by the radius and
//! the stock to leave, or wherever the drop-cutter says it clears the part at the level), and
//! only material within reach of that is kept, so corners a round tool cannot reach are not
//! chased. The path is walked in short steps: at each step a fan of headings either side of the
//! current one (turning no more than a set angle per step, so the path stays smooth) is tried,
//! the engagement each would give (the share of the tool's circumference in material there) is
//! estimated from the raster, and the heading whose engagement is nearest the optimal load, with
//! the material on the side climb (or conventional) milling wants, is taken. A heading that
//! would engage the tool more than the limit is never taken. When no heading cuts any more, the
//! walk ends; the next starts where cleared space meets material again, reached by a feed through
//! cleared material when one leads there with the tool down, or by lifting to the retract height
//! otherwise. Material no walk can reach from cleared space is entered with a helix where it has
//! room (or from open air outside the stock), and refused otherwise.
//!
//! The planner's raster is its own; [`engagement`] measures a finished path against the region
//! itself (exact containment, and exact distances to the moves already made), so the report does
//! not depend on the model the path was planned with.

use std::collections::VecDeque;
use std::f64::consts::{PI, TAU};

use serde::{Deserialize, Serialize};
use solidcore_record::{Region2, Vec2, Vec3};
use solidcraft_geom::Ring;
use solidcraft_geom::prelude::*;

use crate::check::verify;
use crate::cutter::Cutter;
use crate::dropcutter::drop_z;
use crate::engagement::SKIN;
use crate::offset::Shape;
use crate::ops::{Cut, Feeds, Heights};
use crate::path::{MAX_MOVES, Move, Toolpath, arc_sweep};
use crate::piece::Piece;
use crate::surface::Surface;
use crate::surface_ops::{Boundary, Budget, check_leave, check_stepdown, check_tolerance, descent_feed, levels, tool_cutter};
use crate::tool::Tool;
use crate::{CamError, Result, invalid};

/// Most cells of one level's material raster.
pub const MAX_CELLS: usize = 6_000_000;
/// Most steps of the walks of one operation, every level together.
const MAX_STEPS: usize = 3_000_000;
/// Most walks of one level.
const MAX_PASSES: usize = 50_000;
/// Most cell visits of one 2D adaptive operation (rasters, distance transforms, searches,
/// clearing): it plans one level and repeats it.
const MAX_WORK: f64 = 6.0e9;
/// Most cell visits of one level of an adaptive roughing, per cell of its raster: some 500
/// walks' searches and refreshes. A level's work does not depend on how many levels there
/// are, so a larger stepdown (fewer levels) is never refused where a smaller one is planned.
pub(crate) const LEVEL_WORK_PER_CELL: f64 = 8000.0;
/// Cell visits a step of a walk takes, about, per cell of the raster its sweep looks at (the
/// sweep itself, the engagement round the fan of headings, the exact distances near the cut).
const STEP_WORK_PER_SWEEP_CELL: f64 = 3.0;
/// How many times the steps a level is expected to take its budget allows.
const STEP_SLACK: f64 = 3.0;
/// Fewest cell visits a level of an adaptive roughing is allowed (a small raster).
const MIN_LEVEL_WORK: f64 = 2.0e8;
/// About how many cell visits a level of an adaptive roughing takes per cell of its raster (a
/// plate round a boss: some 3000).
pub(crate) const TYPICAL_LEVEL_WORK_PER_CELL: f64 = 3000.0;
/// Material at most this thick (mm) left above a level by earlier cuts is not chased by rest
/// machining (the operation's tolerance when that is larger).
pub const REST_TOLERANCE: f64 = 0.02;
/// Most cell visits an adaptive roughing is expected to take, all its levels together (a few
/// minutes of planning); a job expected to take more is refused before any is done.
pub(crate) const MAX_ROUGHING_WORK: f64 = 3.0e10;
/// Points round the tool the planner's engagement estimate looks at.
const RING: usize = 120;
/// Headings tried either side of the current one.
const FAN: i32 = 6;
/// Largest turn of the heading in one step (radians).
const MAX_TURN: f64 = 25.0 * PI / 180.0;
/// How far below the limit the planner keeps its own estimate (radians), for the raster's error.
const PLAN_MARGIN: f64 = 4.0 * PI / 180.0;
/// Most half turns of one entry helix.
const MAX_HALF_TURNS: f64 = 20_000.0;
/// Largest coordinate or height (mm).
const MAX_COORD: f64 = 1.0e6;
/// Engagement at or above this counts as a full slot (radians).
pub const FULL_SLOT: f64 = 170.0 * PI / 180.0;

/// What a move of an adaptive path is for, so a report can tell the entries from the clearing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MoveRole {
    /// Rapids and feeds through air: positioning, lifting, going down outside the material.
    Position,
    /// A helix into the material (and the circle at its foot).
    Entry,
    /// Clearing at the level.
    Cut,
    /// A feed at the level through material already cleared.
    Link,
}

/// How an adaptive clearing cuts.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AdaptiveParams {
    /// The radial width of cut the path aims at (mm), above 0 and at most the tool's diameter:
    /// on a straight stretch along a straight edge of material the tool takes this much.
    pub optimal_load: f64,
    /// Engagement below which a stretch counts as too light (radians): the walk steers towards
    /// the material harder there.
    pub min_engagement: f64,
    /// Engagement the path never exceeds outside an entry helix (radians), 5° to 180°.
    pub max_engagement: f64,
    /// Material left on the walls (mm); in 3D roughing, on the part all round.
    pub stock_to_leave: f64,
    /// Chord error allowed on curved boundaries, and how far the path may be simplified (mm).
    pub tolerance: f64,
    /// Diameter of the circle the tool's centre follows down an entry helix (mm), above 0 and at
    /// most the tool's diameter (so no core is left standing in the middle).
    pub helix_diameter: f64,
    /// Descent angle of the entry helix (radians), 0.5° to 45°.
    pub ramp_angle: f64,
    /// Climb milling (the material on the right of the tool's travel); otherwise conventional.
    pub climb: bool,
}

/// 3D adaptive roughing of a stock box in levels.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AdaptiveRoughingParams {
    /// `top` is the top of the stock, `bottom` the lowest level.
    pub heights: Heights,
    pub feeds: Feeds,
    /// Most material per level (mm).
    pub stepdown: f64,
    /// The stock's extent seen from above (left out: the surface's extent). The tool may go
    /// outside it (into open air) to enter and to turn.
    #[serde(default)]
    pub boundary: Boundary,
    /// Add a level at every flat area of the part (raised by the stock to leave) between the
    /// regular levels, so a flat is roughed to an even skin.
    #[serde(default)]
    pub flats: bool,
    pub adaptive: AdaptiveParams,
    /// What earlier cuts left of the stock (rest machining): the levels clear only what is
    /// still above them there. Left out, the stock box is whole.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rest: Option<HeightMap>,
}

/// An adaptive toolpath and what each of its moves is for.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AdaptivePath {
    pub path: Toolpath,
    /// One per move of `path`.
    pub roles: Vec<MoveRole>,
    /// Levels the path cuts at.
    pub levels: usize,
    /// Entry helices.
    pub helices: usize,
    /// Times the tool lifts to the retract height between cuts at a level.
    pub lifts: usize,
    /// Area of material within the tool's reach the walks could not clear, summed over the
    /// levels (mm², to the raster's resolution).
    pub left_area: f64,
    pub warnings: Vec<String>,
}

impl AdaptiveParams {
    fn validate(&self, tool: &Tool) -> Result<()> {
        if !(self.optimal_load > 0.0) || !(self.optimal_load <= tool.diameter) {
            return invalid("the optimal load must be above 0 and at most the tool diameter");
        }
        let deg = PI / 180.0;
        if !(self.max_engagement >= 5.0 * deg) || !(self.max_engagement <= PI) {
            return invalid("the largest engagement must be from 5 to 180 degrees");
        }
        if !(self.min_engagement >= 0.0) || !(self.min_engagement < self.max_engagement) {
            return invalid("the smallest engagement must be 0 or more and below the largest");
        }
        check_leave(self.stock_to_leave)?;
        check_tolerance(self.tolerance)?;
        if !(self.helix_diameter > 0.0) || !(self.helix_diameter <= tool.diameter) {
            return invalid("the helix diameter must be above 0 and at most the tool diameter");
        }
        if !(self.ramp_angle >= 0.5 * deg) || !(self.ramp_angle <= 45.0 * deg) {
            return invalid("the helix ramp angle must be from 0.5 to 45 degrees");
        }
        Ok(())
    }
}

/// Adaptive clearing mills with the tool's side and enters along a helix of `ramp_angle`.
fn mills(tool: &Tool, ramp_angle: f64) -> Result<()> {
    tool.validate()?;
    crate::ops::mills(tool, ramp_angle)
}

/// Refuses levels deeper apart than the tool's flutes reach: below them the shank would rub on
/// the material the flutes leave standing (the deep, light cut adaptive clearing is for makes
/// that easy to ask for).
fn check_flutes(tool: &Tool, top: f64, zs: &[f64]) -> Result<()> {
    let mut prev = top;
    for &z in zs {
        if prev - z > tool.flute_length + 1e-9 {
            return invalid(format!(
                "a level {:.3} mm deep is more than the tool's flute length {:.3} mm: use a smaller stepdown",
                prev - z,
                tool.flute_length
            ));
        }
        prev = z;
    }
    Ok(())
}

/// Counts cell visits: those of the level being planned against its own budget, and all of
/// them against the sum of the budgets so far.
struct Work {
    total: std::cell::Cell<f64>,
    level: std::cell::Cell<f64>,
    limit: f64,
    level_limit: f64,
}

impl Work {
    /// Before any level: `setup` visits for rasterising the region.
    fn new(setup: f64) -> Work {
        Work { total: std::cell::Cell::new(0.0), level: std::cell::Cell::new(0.0), limit: setup, level_limit: setup }
    }

    /// A new level starts, allowed `budget` visits of its own.
    fn start_level(&mut self, budget: f64) {
        self.level.set(0.0);
        self.level_limit = budget;
        self.limit += budget;
    }

    fn add(&mut self, n: f64) -> Result<()> {
        self.add_shared(n)
    }

    fn add_shared(&self, n: f64) -> Result<()> {
        self.total.set(self.total.get() + n);
        self.level.set(self.level.get() + n);
        if self.level.get() > self.level_limit || self.total.get() > self.limit {
            return Err(CamError::TooLarge(
                "the adaptive clearing needs far more work at one level than its area and load call for (the walks keep missing the material); use a larger optimal load or engagement limit, or split the region"
                    .into(),
            ));
        }
        Ok(())
    }
}

/// The most cell visits one level over a raster of `cells` may take when its walks are expected
/// to take `steps` steps of `step_work` visits each: the searches between the walks (scaled to
/// the raster) and the walks themselves (scaled to the area and the load), each with room to
/// spare.
pub(crate) fn level_work_limit(cells: usize, steps: f64, step_work: f64) -> f64 {
    (cells as f64 * LEVEL_WORK_PER_CELL).max(MIN_LEVEL_WORK) + STEP_SLACK * steps * step_work
}

/// What an adaptive clearing at `knobs` is expected to take: steps per level over `area` mm².
fn expected_steps(knobs: &Knobs, area: f64) -> f64 {
    area / (knobs.w * knobs.step)
}

/// The smallest optimal load (mm) at which `fits` holds for the knobs it gives, the rest of
/// `params` kept but for the largest engagement, raised with the load as a job raises it by
/// default (one and a half times the load's, and at least 15° more): `None` when not even the
/// tool's diameter does.
fn smallest_load(tool: &Tool, params: &AdaptiveParams, fits: &dyn Fn(&Knobs) -> bool) -> Option<f64> {
    let r = tool.radius();
    let knobs = |w: f64| {
        let opt = (1.0 - w / r).clamp(-1.0, 1.0).acos();
        let most = params.max_engagement.max((opt * 1.5).max(opt + 15.0 * PI / 180.0).min(PI));
        Knobs::new(tool, &AdaptiveParams { optimal_load: w, max_engagement: most, ..*params })
    };
    let (mut lo, mut hi) = (params.optimal_load, tool.diameter);
    if !fits(&knobs(hi)) {
        return None;
    }
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if fits(&knobs(mid)) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    Some(hi)
}

/// What to say when a clearing at `params` would take too many steps: the smallest load that
/// would plan, or that none does.
fn load_hint(tool: &Tool, params: &AdaptiveParams, fits: &dyn Fn(&Knobs) -> bool) -> String {
    match smallest_load(tool, params, fits) {
        // Rounded up to the next 0.01 mm, so the load named does plan.
        Some(w) => {
            format!("use an optimal load of at least {:.2} mm ({:.1}% of the tool's diameter)", (w * 100.0).ceil() / 100.0, w / tool.diameter * 100.0)
        }
        None => "no optimal load up to the tool's diameter plans it: use a larger tool, or split the region".into(),
    }
}

/// Refuses, before any is done, an adaptive roughing of `levels` levels over a raster of
/// `cells` whose walks are expected to take `steps` steps of `step_work` visits a level, when
/// that would take minutes of planning ([`MAX_ROUGHING_WORK`] as it typically goes), more steps
/// a level than a walk may take, or more drop-cutter samples than an operation may take: with
/// the numbers and `hint`, what to change.
pub(crate) fn check_roughing_work(levels: usize, cells: usize, steps: f64, step_work: f64, hint: &dyn Fn() -> String) -> Result<()> {
    let expected = levels as f64 * (cells as f64 * TYPICAL_LEVEL_WORK_PER_CELL + steps * step_work);
    if !(expected <= MAX_ROUGHING_WORK) {
        return Err(CamError::TooLarge(format!(
            "the adaptive roughing has {levels} levels over a raster of {cells} cells, some {steps:.2e} steps a level: some {expected:.1e} cell visits, minutes of planning (at most {MAX_ROUGHING_WORK:.0e}); use a larger stepdown (fewer levels) or a smaller boundary, or {}",
            hint()
        )));
    }
    if !(steps <= MAX_STEPS as f64) {
        return Err(CamError::TooLarge(format!(
            "the adaptive roughing needs some {steps:.2e} steps a level, more than {MAX_STEPS}; {}, or use a smaller boundary",
            hint()
        )));
    }
    if !(levels as f64 * cells as f64 <= crate::surface_ops::MAX_DROPS as f64) {
        return Err(CamError::TooLarge(format!(
            "the adaptive roughing has {levels} levels over a raster of {cells} cells, more drop-cutter samples than {}; use a larger stepdown (fewer levels), a larger tool (a coarser raster), or a smaller boundary",
            crate::surface_ops::MAX_DROPS
        )));
    }
    Ok(())
}

/// A raster of square cells.
#[derive(Clone, Copy, Debug)]
struct Grid {
    origin: Vec2,
    h: f64,
    nx: usize,
    ny: usize,
}

impl Grid {
    fn new(lo: Vec2, hi: Vec2, h: f64) -> Result<Grid> {
        if ![lo.x, lo.y, hi.x, hi.y].iter().all(|v| v.is_finite() && v.abs() <= MAX_COORD) || !(h > 0.0) {
            return invalid("the region must lie within 1000 m");
        }
        let fx = ((hi.x - lo.x) / h).ceil() + 1.0;
        let fy = ((hi.y - lo.y) / h).ceil() + 1.0;
        if !(fx >= 1.0 && fy >= 1.0) || !(fx * fy <= MAX_CELLS as f64) {
            return Err(CamError::TooLarge(format!(
                "the region needs more than {MAX_CELLS} cells for this tool, load and engagement limit; use a larger tool, optimal load or engagement limit, or split the region"
            )));
        }
        Ok(Grid { origin: lo, h, nx: fx as usize, ny: fy as usize })
    }

    fn len(&self) -> usize {
        self.nx * self.ny
    }

    fn center(&self, i: usize, j: usize) -> Vec2 {
        Vec2::new(self.origin.x + i as f64 * self.h, self.origin.y + j as f64 * self.h)
    }

    fn center_of(&self, k: usize) -> Vec2 {
        self.center(k % self.nx.max(1), k / self.nx.max(1))
    }

    /// The cell whose centre is nearest `p`.
    fn cell(&self, p: Vec2) -> Option<(usize, usize)> {
        let fi = ((p.x - self.origin.x) / self.h).round();
        let fj = ((p.y - self.origin.y) / self.h).round();
        if fi >= 0.0 && fj >= 0.0 && fi < self.nx as f64 && fj < self.ny as f64 { Some((fi as usize, fj as usize)) } else { None }
    }

    fn index(&self, p: Vec2) -> Option<usize> {
        self.cell(p).map(|(i, j)| j * self.nx + i)
    }

    /// The cells whose centres may be within `r` of the box `lo..hi`, as index ranges.
    fn span(&self, lo: Vec2, hi: Vec2, r: f64) -> (std::ops::Range<usize>, std::ops::Range<usize>) {
        let clamp = |v: f64, n: usize| -> usize {
            if !(v > 0.0) {
                0
            } else if v >= n as f64 {
                n
            } else {
                v as usize
            }
        };
        let i0 = clamp(((lo.x - r - self.origin.x) / self.h).floor(), self.nx);
        let i1 = clamp(((hi.x + r - self.origin.x) / self.h).ceil() + 1.0, self.nx);
        let j0 = clamp(((lo.y - r - self.origin.y) / self.h).floor(), self.ny);
        let j1 = clamp(((hi.y + r - self.origin.y) / self.h).ceil() + 1.0, self.ny);
        (i0..i1, j0..j1)
    }
}

const FAR: f64 = f64::INFINITY;

/// Squared distances along one line to the nearest site (where `f` is 0) by the lower envelope
/// of the parabolas rooted at the sites.
fn envelope(f: &[f64], d: &mut [f64], v: &mut Vec<usize>, z: &mut Vec<f64>) {
    v.clear();
    z.clear();
    for (q, &fq) in f.iter().enumerate() {
        if !fq.is_finite() {
            continue;
        }
        let qf = q as f64;
        loop {
            match (v.last().copied(), z.last().copied()) {
                (Some(p), Some(zp)) => {
                    let pf = p as f64;
                    let fp = f.get(p).copied().unwrap_or(FAR);
                    let s = ((fq + qf * qf) - (fp + pf * pf)) / (2.0 * (qf - pf));
                    if s <= zp {
                        v.pop();
                        z.pop();
                        continue;
                    }
                    v.push(q);
                    z.push(s);
                }
                _ => {
                    v.push(q);
                    z.push(f64::NEG_INFINITY);
                }
            }
            break;
        }
    }
    let mut k = 0usize;
    for (q, out) in d.iter_mut().enumerate() {
        if v.is_empty() {
            *out = FAR;
            continue;
        }
        while k + 1 < v.len() && z.get(k + 1).is_some_and(|zk| *zk < q as f64) {
            k += 1;
        }
        let p = v.get(k).copied().unwrap_or(0);
        let dq = q as f64 - p as f64;
        *out = dq * dq + f.get(p).copied().unwrap_or(FAR);
    }
}

/// Distance (mm) from each cell's centre to the nearest cell's centre where `site` holds;
/// infinite where there is none.
fn distance_field(g: &Grid, site: &dyn Fn(usize) -> bool, work: &mut Work) -> Result<Vec<f64>> {
    distance_window(g, site, Window { i0: 0, i1: g.nx, j0: 0, j1: g.ny }, work)
}

/// A rectangle of a raster's cells: columns `i0..i1`, rows `j0..j1`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Window {
    i0: usize,
    i1: usize,
    j0: usize,
    j1: usize,
}

impl Window {
    /// Grown by `m` cells on every side, within the raster.
    fn grown(&self, m: usize, g: &Grid) -> Window {
        Window { i0: self.i0.saturating_sub(m), i1: (self.i1 + m).min(g.nx), j0: self.j0.saturating_sub(m), j1: (self.j1 + m).min(g.ny) }
    }

    /// Also covering cell `(i, j)`.
    fn with(w: Option<Window>, i: usize, j: usize) -> Window {
        match w {
            None => Window { i0: i, i1: i + 1, j0: j, j1: j + 1 },
            Some(w) => Window { i0: w.i0.min(i), i1: w.i1.max(i + 1), j0: w.j0.min(j), j1: w.j1.max(j + 1) },
        }
    }
}

/// [`distance_field`] over the cells of `win` only, from the sites inside it, indexed from its
/// corner row by row.
fn distance_window(g: &Grid, site: &dyn Fn(usize) -> bool, win: Window, work: &mut Work) -> Result<Vec<f64>> {
    let (nx, ny) = (win.i1.saturating_sub(win.i0), win.j1.saturating_sub(win.j0));
    work.add(3.0 * (nx * ny) as f64)?;
    let mut out = vec![FAR; nx * ny];
    let n = nx.max(ny);
    let (mut f, mut d, mut v, mut z) = (vec![FAR; n], vec![FAR; n], Vec::with_capacity(n), Vec::with_capacity(n + 1));
    for j in 0..ny {
        for (i, fi) in f.iter_mut().enumerate().take(nx) {
            *fi = if site((win.j0 + j) * g.nx + win.i0 + i) { 0.0 } else { FAR };
        }
        if let (Some(fr), Some(dr)) = (f.get(..nx), d.get_mut(..nx)) {
            envelope(fr, dr, &mut v, &mut z);
        }
        if let (Some(row), Some(dr)) = (out.get_mut(j * nx..(j + 1) * nx), d.get(..nx)) {
            row.copy_from_slice(dr);
        }
    }
    for i in 0..nx {
        for (j, fj) in f.iter_mut().enumerate().take(ny) {
            *fj = out.get(j * nx + i).copied().unwrap_or(FAR);
        }
        if let (Some(fc), Some(dc)) = (f.get(..ny), d.get_mut(..ny)) {
            envelope(fc, dc, &mut v, &mut z);
        }
        for (j, dj) in d.iter().enumerate().take(ny) {
            if let Some(o) = out.get_mut(j * nx + i) {
                *o = if dj.is_finite() { dj.sqrt() * g.h } else { FAR };
            }
        }
    }
    Ok(out)
}

/// Where the cutter's centre may stand at a level, exactly (with the planner's margin).
trait Domain {
    fn allowed(&self, p: Vec2) -> Result<bool>;
}

/// Inside a pocket's walls.
struct Walls {
    shape: Shape,
    /// The tool radius, the stock to leave and the margin.
    wall: f64,
}

impl Domain for Walls {
    fn allowed(&self, p: Vec2) -> Result<bool> {
        Ok(p.is_finite() && self.shape.clear(p, self.wall))
    }
}

/// Clear of a surface at a level, inside a rectangle.
struct Clear<'a> {
    surface: &'a Surface,
    /// The cutter grown by the stock to leave and the margin.
    cutter: Cutter,
    /// How far the grown cutter's tip is below the real one.
    grown_by: f64,
    z: f64,
    lo: Vec2,
    hi: Vec2,
    budget: &'a Budget,
}

impl Domain for Clear<'_> {
    fn allowed(&self, p: Vec2) -> Result<bool> {
        if !(p.x >= self.lo.x && p.x <= self.hi.x && p.y >= self.lo.y && p.y <= self.hi.y) {
            return Ok(false);
        }
        self.budget.spend()?;
        Ok(match drop_z(self.surface, &self.cutter, p.x, p.y) {
            Some(zg) => zg + self.grown_by <= self.z + 1e-9,
            None => true,
        })
    }
}

/// A heightfield of what earlier cuts left: cell `(i, j)` is the column over the square of
/// side `dx` by `dy` about `(x0 + (i + ½)·dx, y0 + (j + ½)·dy)`, its top at `heights[j·nx + i]`
/// (the simulation's [`crate::sim::StockGrid::Heights`]).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HeightMap {
    pub x0: f64,
    pub y0: f64,
    pub dx: f64,
    pub dy: f64,
    pub nx: usize,
    pub ny: usize,
    pub heights: Vec<f64>,
}

/// Most cells of a height map.
pub const MAX_MAP_CELLS: usize = 64_000_000;

impl HeightMap {
    pub fn validate(&self) -> Result<()> {
        if ![self.x0, self.y0, self.dx, self.dy].iter().all(|v| v.is_finite() && v.abs() <= MAX_COORD) || !(self.dx > 0.0) || !(self.dy > 0.0) {
            return invalid("the stock's height map needs finite corners and cells above 0 mm");
        }
        if self.nx == 0 || self.ny == 0 || self.nx.checked_mul(self.ny).is_none_or(|n| n > MAX_MAP_CELLS || n != self.heights.len()) {
            return invalid(format!("the stock's height map needs nx × ny heights, at most {MAX_MAP_CELLS}"));
        }
        if !self.heights.iter().all(|z| z.is_finite() && z.abs() <= MAX_COORD) {
            return invalid("the stock's heights must be finite numbers within 1000 m");
        }
        Ok(())
    }

    /// The top of the cell `q` is in, as the simulation that made the map has it (a cell is cut
    /// where the tool covered its centre); `None` off the map.
    pub fn top_at(&self, q: Vec2) -> Option<f64> {
        let fi = (q.x - self.x0) / self.dx;
        let fj = (q.y - self.y0) / self.dy;
        if !(fi >= 0.0 && fj >= 0.0 && fi < self.nx as f64 && fj < self.ny as f64) {
            return None;
        }
        self.heights.get(fj as usize * self.nx + fi as usize).copied()
    }

    /// This map with each cell no higher than `other` has it at the cell's centre: what is
    /// left after both the cuts this map holds and those `other` holds.
    pub fn lower(&self, other: &HeightMap) -> HeightMap {
        let heights = (0..self.heights.len())
            .map(|k| {
                let (i, j) = (k % self.nx.max(1), k / self.nx.max(1));
                let c = Vec2::new(self.x0 + (i as f64 + 0.5) * self.dx, self.y0 + (j as f64 + 0.5) * self.dy);
                let own = self.heights.get(k).copied().unwrap_or(f64::NEG_INFINITY);
                other.top_at(c).map_or(own, |t| own.min(t))
            })
            .collect();
        HeightMap { heights, ..self.clone() }
    }

    /// The highest top over the disc of radius `rad` about `c` (and a cell round it); `None`
    /// where the disc is off the map.
    fn top_over(&self, c: Vec2, rad: f64) -> Option<f64> {
        let i0 = ((c.x - rad - self.x0) / self.dx - 1.0).floor().max(0.0);
        let i1 = ((c.x + rad - self.x0) / self.dx + 1.0).ceil().min(self.nx as f64 - 1.0);
        let j0 = ((c.y - rad - self.y0) / self.dy - 1.0).floor().max(0.0);
        let j1 = ((c.y + rad - self.y0) / self.dy + 1.0).ceil().min(self.ny as f64 - 1.0);
        if !(i0 <= i1 && j0 <= j1) {
            return None;
        }
        let mut most = f64::NEG_INFINITY;
        let reach = rad + self.dx.hypot(self.dy);
        for j in j0 as usize..=j1 as usize {
            for i in i0 as usize..=i1 as usize {
                let p = Vec2::new(self.x0 + (i as f64 + 0.5) * self.dx, self.y0 + (j as f64 + 0.5) * self.dy);
                if p.dist(c) <= reach
                    && let Some(z) = self.heights.get(j * self.nx + i)
                {
                    most = most.max(*z);
                }
            }
        }
        most.is_finite().then_some(most)
    }
}

/// What an adaptive clearing knows of the stock beyond its own region (2D) or boundary (3D):
/// where the stock is seen from above, how high it reaches, and what earlier cuts left of it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct StockModel {
    /// The stock's low and high corners seen from above; `None`: it reaches past the region
    /// everywhere (a 2D clearing then enters with a helix).
    #[serde(default)]
    pub rect: Option<[Vec2; 2]>,
    /// The stock's top; `None`: the operation's top.
    #[serde(default)]
    pub top: Option<f64>,
    /// What earlier operations (or setups) left: where the map has a cell, the material ends at
    /// its top. Rest machining plans on it.
    #[serde(default)]
    pub heights: Option<HeightMap>,
    /// Material left above a level no thicker than this is taken as cut (mm): the cusps and
    /// the cells of the map are not chased.
    #[serde(default)]
    pub rest_tolerance: f64,
}

impl StockModel {
    fn validate(&self) -> Result<()> {
        if let Some([lo, hi]) = self.rect
            && (![lo.x, lo.y, hi.x, hi.y].iter().all(|v| v.is_finite() && v.abs() <= MAX_COORD) || !(lo.x < hi.x) || !(lo.y < hi.y))
        {
            return invalid("the stock's corners must be finite, the low one below and left of the high one");
        }
        if self.top.is_some_and(|t| !(t.is_finite() && t.abs() <= MAX_COORD)) {
            return invalid("the stock's top must be a finite height");
        }
        if !(self.rest_tolerance >= 0.0 && self.rest_tolerance <= 100.0) {
            return invalid("the rest tolerance must be from 0 to 100 mm");
        }
        if let Some(m) = &self.heights {
            m.validate()?;
        }
        Ok(())
    }

    fn in_rect(&self, q: Vec2) -> bool {
        self.rect.is_none_or(|[lo, hi]| q.x >= lo.x && q.x <= hi.x && q.y >= lo.y && q.y <= hi.y)
    }

    /// Whether there is stock at `q` at the level `z`.
    fn material_at(&self, q: Vec2, z: f64) -> bool {
        self.in_rect(q)
            && self.top.is_none_or(|t| t > z + self.rest_tolerance)
            && self.heights.as_ref().and_then(|m| m.top_at(q)).is_none_or(|t| t > z + self.rest_tolerance)
    }

    /// The highest the stock reaches over the disc of radius `rad` about `c`, at most `cap`.
    fn top_over(&self, c: Vec2, rad: f64, cap: f64) -> f64 {
        let t = self.top.map_or(cap, |t| t.min(cap));
        match self.heights.as_ref().and_then(|m| m.top_over(c, rad)) {
            Some(h) => h.min(t),
            None => t,
        }
    }
}

/// Where the cutter's centre may stand in a 2D clearing when the stock's extent is known: inside
/// the regions clear of their walls as [`Walls`] has it, and also past a region's boundary where
/// the tool meets no stock outside the regions (open air beyond the stock's edge, or a region
/// drawn round the stock).
struct Fence {
    shape: Shape,
    lo: Vec2,
    hi: Vec2,
    /// The boundary of the stock outside the regions: the regions' boundary inside the stock and
    /// the stock's edges outside the regions.
    walls: Vec<Piece>,
    wall: f64,
}

impl Fence {
    fn new(shape: Shape, lo: Vec2, hi: Vec2, wall: f64) -> Fence {
        let corners = [lo, Vec2::new(hi.x, lo.y), hi, Vec2::new(lo.x, hi.y)];
        let edges: Vec<Piece> = (0..4).filter_map(|i| Some(Piece::Line { a: *corners.get(i)?, b: *corners.get((i + 1) % 4)? })).collect();
        let size = (hi - lo).len().max(1.0);
        let eps = 1e-9 * size;
        let nudge = 1e-6 * size;
        let strictly_in = |q: Vec2| q.x > lo.x + eps && q.x < hi.x - eps && q.y > lo.y + eps && q.y < hi.y - eps;
        // The stretches of `piece` between where it crosses `others`.
        let split = |piece: &Piece, others: &mut dyn Iterator<Item = &Piece>| -> Vec<(f64, f64)> {
            let mut ts = vec![0.0, 1.0];
            let mut hits = Vec::new();
            for o in others {
                hits.clear();
                piece.intersections(o, &mut hits);
                ts.extend(hits.iter().map(|h| h.0));
            }
            ts.sort_by(f64::total_cmp);
            ts.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
            ts.windows(2).filter_map(|w| Some((*w.first()?, *w.get(1)?))).filter(|(a, b)| b - a > 1e-12).collect()
        };
        let mut walls = Vec::new();
        for piece in shape.loops().iter().flatten() {
            for (t0, t1) in split(piece, &mut edges.iter()) {
                if strictly_in(piece.point_at(0.5 * (t0 + t1))) {
                    walls.push(piece.sub(t0, t1));
                }
            }
        }
        for e in &edges {
            let inward = e.tangent_at(0.5).perp();
            for (t0, t1) in split(e, &mut shape.loops().iter().flatten()) {
                let m = e.point_at(0.5 * (t0 + t1)) + inward * nudge;
                if !shape.inside(m, nudge) {
                    walls.push(e.sub(t0, t1));
                }
            }
        }
        Fence { shape, lo, hi, walls, wall }
    }

    /// Whether a tool of radius `wall` centred on `p` stays off the stock outside the regions.
    fn clear(&self, p: Vec2, wall: f64) -> bool {
        if !p.is_finite() {
            return false;
        }
        if self.shape.clear(p, wall) {
            return true;
        }
        let in_stock = p.x >= self.lo.x && p.x <= self.hi.x && p.y >= self.lo.y && p.y <= self.hi.y;
        if in_stock && !self.shape.inside(p, wall.max(1e-6)) {
            return false;
        }
        self.walls.iter().all(|w| w.dist(p) >= wall - 1e-7)
    }
}

impl Domain for Fence {
    fn allowed(&self, p: Vec2) -> Result<bool> {
        Ok(self.clear(p, self.wall))
    }
}

/// The planner's knobs, derived from the tool and the parameters.
#[derive(Clone, Copy, Debug)]
struct Knobs {
    r: f64,
    /// Cell size.
    h: f64,
    /// Step length of a walk.
    step: f64,
    /// The radial width of cut aimed at.
    w: f64,
    /// Margin kept from where the centre may not go.
    margin: f64,
    /// Engagement the walk aims at, the most it takes, and the least it wants.
    target: f64,
    limit: f64,
    least: f64,
    climb: bool,
    helix_radius: f64,
    ramp: f64,
    /// How far the path may be simplified.
    simplify: f64,
}

impl Knobs {
    fn new(tool: &Tool, p: &AdaptiveParams) -> Knobs {
        let r = tool.radius();
        let limit = (p.max_engagement - PLAN_MARGIN).max(p.max_engagement * 0.5);
        let load = p.optimal_load.min(2.0 * r);
        let target = (1.0 - load / r).clamp(-1.0, 1.0).acos().min(limit * 0.92);
        // The width actually aimed at: the load's, or less where the limit holds the engagement
        // below the load's. The raster and the steps follow it, so a step into material (a
        // concave wall above all) stays within the limit.
        let w = (r * (1.0 - target.cos())).min(load);
        // The raster follows the tool, not the load: where the path is near the edge of what
        // it has cut, the engagement is settled by exact distances to the moves, so a light
        // load needs no finer cells (only more steps).
        let h = r / 16.0;
        let step = (w * 0.5).clamp(h, r * 0.25);
        // Only points in line with their neighbours are left out: the walk's engagement holds for
        // the points it took, and a corner cut short near the edge of a cut can bite much deeper.
        let simplify = (p.tolerance * 0.01).min(1e-4);
        Knobs {
            r,
            h,
            step,
            w,
            margin: step * step / (8.0 * r) + simplify + 1e-4 * r.max(1.0),
            target,
            limit,
            least: p.min_engagement.min(target),
            climb: p.climb,
            helix_radius: p.helix_diameter * 0.5,
            ramp: p.ramp_angle,
            simplify,
        }
    }
}

impl Knobs {
    /// How far a walk may go without cutting before it ends: a half turn at its tightest turn
    /// and a quarter more, enough to round a corner of the material and come back to it, too
    /// little to wander off through what is already cleared.
    fn max_air(&self) -> f64 {
        1.25 * PI * self.step / MAX_TURN
    }

    /// How far from material the distances to it are kept exact: past where a walk starts
    /// and where it steers through air.
    fn near(&self) -> f64 {
        2.0 * self.r + 2.0 * self.step + 4.0 * self.h + self.max_air()
    }

    /// About how many cell visits a step of a walk takes: the cells its sweep looks at.
    fn step_work(&self) -> f64 {
        let span = 2.0 * (self.r + self.h) / self.h + 2.0 + self.step / self.h;
        span * span * STEP_WORK_PER_SWEEP_CELL
    }
}

/// How a level's path gets from one cut to the next.
#[derive(Clone, Debug, PartialEq)]
enum Step {
    /// From the retract height straight down to the level at the point (cleared or open air).
    Drop(Vec2),
    /// A helix about `center` from where the material starts down to the level, then a circle.
    Helix { center: Vec2, from: Option<f64> },
    /// A feed at the level through cleared material.
    Link(Vec<Vec2>),
    /// Clearing at the level along the points (from where the tool is).
    Cut(Vec<Vec2>),
}

fn rotate(v: Vec2, a: f64) -> Vec2 {
    let (s, c) = a.sin_cos();
    Vec2::new(c * v.x - s * v.y, s * v.x + c * v.y)
}

fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let d = b - a;
    let l2 = d.len2();
    let t = if l2 > 0.0 { ((p - a).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
    p.dist(a + d * t)
}

/// One level's material raster and what the walks need.
struct Level<'a> {
    g: Grid,
    k: Knobs,
    domain: &'a dyn Domain,
    allow: Vec<bool>,
    /// Whether a point is stock, exactly.
    stock_at: &'a dyn Fn(Vec2) -> bool,
    /// Distance from each cell's centre to the path so far (where it is within reach of it).
    dpath: Vec<f64>,
    /// The path so far, for exact distances.
    segs: Vec<(Vec2, Vec2)>,
    /// For each cell not yet settled by the raster (its centre neither well inside nor well
    /// outside the radius of the path), the moves that come within the radius and the slack of
    /// its centre: all the moves that may cut a point the cell settles.
    band: std::collections::HashMap<usize, Vec<u32>>,
    /// Material still there: stock farther than the radius from the path (all of it, so the
    /// engagement sees what the tool cannot reach too).
    mat: Vec<bool>,
    /// Cells within reach of where the centre may stand: the material the walks clear.
    reach: Vec<bool>,
    /// Distance from each cell to the nearest material within reach (refreshed between walks).
    dmat: Vec<f64>,
    /// Distance from each cell to the nearest material of any kind (refreshed with `dmat`).
    /// Both are exact up to [`Knobs::near`]; a larger distance only says the material is
    /// farther than that.
    dall: Vec<f64>,
    /// Whether `dmat` and `dall` have been worked out once.
    fresh: bool,
    /// The cells whose material the walks cleared since the distances were last refreshed.
    dirty: Option<Window>,
    /// Cells of material within reach still there.
    left: usize,
    /// The links' search: the cell each cell was reached from, and the search that reached it.
    parent: Vec<u32>,
    seen: Vec<u32>,
    search: u32,
    /// Distance from each cell to the nearest cell the centre may not stand at.
    room: Vec<f64>,
    tried: Vec<bool>,
    ring: Vec<Vec2>,
    steps: &'a mut usize,
    work: &'a mut Work,
}

impl<'a> Level<'a> {
    /// The level's raster: where the centre may stand (`allow`, exact at each cell's centre) and
    /// the stock (`stock_cell` at each cell's centre, `stock_at` for any point the tool's edge
    /// reaches).
    fn new(
        g: Grid,
        k: Knobs,
        domain: &'a dyn Domain,
        allow: Vec<bool>,
        stock_cell: &dyn Fn(Vec2) -> bool,
        stock_at: &'a dyn Fn(Vec2) -> bool,
        steps: &'a mut usize,
        work: &'a mut Work,
    ) -> Result<Level<'a>> {
        let mat: Vec<bool> = (0..g.len()).map(|c| stock_cell(g.center_of(c))).collect();
        // The level's budget: its raster and the walks its material is expected to take.
        let area = mat.iter().filter(|m| **m).count() as f64 * g.h * g.h;
        work.start_level(level_work_limit(g.len(), expected_steps(&k, area), k.step_work()));
        let reach = distance_field(&g, &|c| allow.get(c).copied().unwrap_or(false), work)?;
        let room = distance_field(&g, &|c| !allow.get(c).copied().unwrap_or(false), work)?;
        let reach: Vec<bool> = reach.iter().map(|d| *d <= k.r - 0.5 * k.h).collect();
        let left = mat.iter().zip(&reach).filter(|(m, r)| **m && **r).count();
        let rho = k.r * (1.0 - SKIN);
        let ring = (0..RING).map(|i| Vec2::from_angle(TAU * i as f64 / RING as f64) * rho).collect();
        let n = g.len();
        Ok(Level {
            g,
            k,
            domain,
            allow,
            stock_at,
            dpath: vec![FAR; n],
            segs: Vec::new(),
            band: std::collections::HashMap::new(),
            mat,
            reach,
            dmat: vec![FAR; n],
            dall: vec![FAR; n],
            fresh: false,
            dirty: None,
            left,
            parent: vec![0; n],
            seen: vec![0; n],
            search: 0,
            room,
            tried: vec![false; n],
            ring,
            steps,
            work,
        })
    }

    fn grid_allows(&self, p: Vec2) -> bool {
        self.g.index(p).and_then(|c| self.allow.get(c).copied()).unwrap_or(false)
    }

    fn dmat_at(&self, p: Vec2) -> f64 {
        self.g.index(p).and_then(|c| self.dmat.get(c).copied()).unwrap_or(FAR)
    }

    /// Material within reach at cell `c`.
    fn target(&self, c: usize) -> bool {
        self.mat.get(c).copied().unwrap_or(false) && self.reach.get(c).copied().unwrap_or(false)
    }

    fn material_cells(&self) -> usize {
        self.left
    }

    /// Whether the point is still material: stock not within the radius of the path. The raster
    /// settles it where the path is well inside or well outside the radius, exact distances to
    /// the path do in between.
    fn material_at(&self, q: Vec2) -> Result<bool> {
        let Some(c) = self.g.index(q) else { return Ok(false) };
        let d = self.dpath.get(c).copied().unwrap_or(FAR);
        let slack = self.g.h * std::f64::consts::FRAC_1_SQRT_2 + 1e-9;
        if d <= self.k.r - slack {
            return Ok(false);
        }
        if !(self.stock_at)(q) {
            return Ok(false);
        }
        if d > self.k.r + slack {
            return Ok(true);
        }
        let Some(near) = self.band.get(&c) else { return Ok(true) };
        self.work.add_shared(near.len() as f64)?;
        let r = self.k.r;
        Ok(!near.iter().any(|id| self.segs.get(*id as usize).is_some_and(|(a, b)| seg_dist(q, *a, *b) < r)))
    }

    /// The engagement at `q` (radians of the circumference in material, at most a half turn)
    /// and where the material lies (the sum of the directions to it).
    fn engage(&self, q: Vec2) -> Result<(f64, Vec2)> {
        let mut n = 0usize;
        let mut dir = Vec2::ZERO;
        for u in &self.ring {
            if self.material_at(q + *u)? {
                n += 1;
                dir += *u;
            }
        }
        Ok(((TAU * n as f64 / RING as f64).min(PI), dir))
    }

    /// Records the tool's centre moving from `a` to `b`; how many cells of material it cleared.
    fn sweep(&mut self, a: Vec2, b: Vec2) -> Result<usize> {
        let id = self.segs.len() as u32;
        self.segs.push((a, b));
        let slack = self.g.h * std::f64::consts::FRAC_1_SQRT_2 + 1e-9;
        let (settled, unsettled) = (self.k.r - slack, self.k.r + slack);
        let lo = Vec2::new(a.x.min(b.x), a.y.min(b.y));
        let hi = Vec2::new(a.x.max(b.x), a.y.max(b.y));
        let (is, js) = self.g.span(lo, hi, self.k.r + self.g.h);
        self.work.add((is.len() * js.len()) as f64)?;
        let mut n = 0;
        for j in js {
            for i in is.clone() {
                let c = j * self.g.nx + i;
                let d = seg_dist(self.g.center(i, j), a, b);
                let Some(dp) = self.dpath.get_mut(c) else { continue };
                if *dp <= settled {
                    continue;
                }
                if d < *dp {
                    *dp = d;
                    if d <= self.k.r
                        && let Some(m) = self.mat.get_mut(c)
                        && *m
                    {
                        *m = false;
                        n += 1;
                        if self.reach.get(c).copied().unwrap_or(false) {
                            self.left = self.left.saturating_sub(1);
                        }
                        self.dirty = Some(Window::with(self.dirty, i, j));
                    }
                }
                if *dp <= settled {
                    self.band.remove(&c);
                } else if d < unsettled {
                    self.band.entry(c).or_default().push(id);
                }
            }
        }
        Ok(n)
    }

    /// Records the tool's centre going round the circle of radius `rad` about `c`.
    fn bore(&mut self, c: Vec2, rad: f64) -> Result<()> {
        let n = ((TAU * rad / self.k.step).ceil() as usize).clamp(16, 4096);
        let mut prev = c + Vec2::new(rad, 0.0);
        for i in 1..=n {
            let next = c + Vec2::from_angle(TAU * i as f64 / n as f64) * rad;
            self.sweep(prev, next)?;
            prev = next;
        }
        Ok(())
    }

    /// Brings the distances to the material up to date with what the walks cleared: all of
    /// them the first time, then only round the cells cleared since. Material is only ever
    /// cleared, so a distance grows only where the material nearest it went: within
    /// [`Knobs::near`] of a cell cleared. Those cells are worked out again from the material
    /// within that much more round them; the rest keep theirs.
    fn refresh(&mut self) -> Result<()> {
        let full = Window { i0: 0, i1: self.g.nx, j0: 0, j1: self.g.ny };
        let (outer, inner) = if self.fresh {
            let Some(d) = self.dirty.take() else { return Ok(()) };
            let m = (self.k.near() / self.g.h).ceil() as usize + 1;
            (d.grown(2 * m, &self.g), d.grown(m, &self.g))
        } else {
            self.fresh = true;
            self.dirty = None;
            (full, full)
        };
        let (mat, reach) = (&self.mat, &self.reach);
        let dmat = distance_window(&self.g, &|c| mat.get(c).copied().unwrap_or(false) && reach.get(c).copied().unwrap_or(false), outer, self.work)?;
        let dall = distance_window(&self.g, &|c| mat.get(c).copied().unwrap_or(false), outer, self.work)?;
        let wnx = outer.i1 - outer.i0;
        for j in inner.j0..inner.j1 {
            for i in inner.i0..inner.i1 {
                let (c, w) = (j * self.g.nx + i, (j - outer.j0) * wnx + (i - outer.i0));
                if let (Some(d), Some(v)) = (self.dmat.get_mut(c), dmat.get(w)) {
                    *d = *v;
                }
                if let (Some(d), Some(v)) = (self.dall.get_mut(c), dall.get(w)) {
                    *d = *v;
                }
            }
        }
        Ok(())
    }

    /// A cell the tool can stand in with (next to) no material: links run through these.
    fn free(&self, c: usize) -> bool {
        self.allow.get(c).copied().unwrap_or(false) && self.dall.get(c).is_some_and(|d| *d >= self.k.r + self.k.h)
    }

    /// A free cell next to material, not started from before.
    fn frontier(&self, c: usize) -> bool {
        self.free(c)
            && !self.tried.get(c).copied().unwrap_or(true)
            && self.dmat.get(c).is_some_and(|d| *d <= self.k.r + 2.0 * self.k.step + 2.0 * self.k.h)
    }

    /// The direction from cell `c` towards the nearest material.
    fn towards_material(&self, c: usize) -> Vec2 {
        let (nx, i, j) = (self.g.nx, c % self.g.nx.max(1), c / self.g.nx.max(1));
        let at = |i: usize, j: usize| self.dmat.get(j * nx + i).copied().filter(|d| d.is_finite());
        let here = at(i, j).unwrap_or(0.0);
        let dx = at(i + 1, j).unwrap_or(here) - if i > 0 { at(i - 1, j).unwrap_or(here) } else { here };
        let dy = at(i, j + 1).unwrap_or(here) - if j > 0 { at(i, j - 1).unwrap_or(here) } else { here };
        (-Vec2::new(dx, dy)).normalized().unwrap_or(Vec2::new(1.0, 0.0))
    }

    /// The heading that puts the material on the side the milling wants.
    fn heading_for(&self, towards: Vec2) -> Vec2 {
        if self.k.climb { towards.perp() } else { -towards.perp() }
    }

    /// The nearest frontier cell reachable from `from` through free cells, and the link there.
    fn linked_frontier(&mut self, from: Vec2) -> Result<Option<(Vec<Vec2>, usize)>> {
        let Some(start) = self.g.index(from) else { return Ok(None) };
        let n = self.g.len();
        self.search = self.search.wrapping_add(1);
        if self.search == 0 {
            self.seen.fill(0);
            self.search = 1;
        }
        let search = self.search;
        let mut queue = VecDeque::new();
        if let (Some(p), Some(s)) = (self.parent.get_mut(start), self.seen.get_mut(start)) {
            *p = start as u32;
            *s = search;
        }
        queue.push_back(start);
        let (nx, ny) = (self.g.nx as isize, self.g.ny as isize);
        let mut visited = 0usize;
        let mut found = None;
        while let Some(c) = queue.pop_front() {
            visited += 1;
            if self.frontier(c) {
                found = Some(c);
                break;
            }
            let (ci, cj) = ((c % self.g.nx) as isize, (c / self.g.nx) as isize);
            for (di, dj) in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)] {
                let (i, j) = (ci + di, cj + dj);
                if i < 0 || j < 0 || i >= nx || j >= ny {
                    continue;
                }
                let d = (j * nx + i) as usize;
                if self.seen.get(d).is_some_and(|s| *s != search) && self.free(d) {
                    if let (Some(p), Some(s)) = (self.parent.get_mut(d), self.seen.get_mut(d)) {
                        *p = c as u32;
                        *s = search;
                    }
                    queue.push_back(d);
                }
            }
        }
        self.work.add(visited as f64 * 8.0)?;
        let Some(end) = found else { return Ok(None) };
        let mut cells = vec![end];
        let mut c = end;
        while c != start {
            let Some(p) = self.parent.get(c).map(|p| *p as usize) else { break };
            if p == c || cells.len() > n {
                break;
            }
            cells.push(p);
            c = p;
        }
        cells.reverse();
        let mut pts: Vec<Vec2> = vec![from];
        pts.extend(cells.iter().skip(1).map(|c| self.g.center_of(*c)));
        Ok(Some((self.shortcut(pts)?, end)))
    }

    /// Whether the straight feed from `a` to `b` runs through free cells only.
    fn free_line(&self, a: Vec2, b: Vec2) -> bool {
        let n = (a.dist(b) / (0.5 * self.k.h)).ceil().clamp(1.0, 1.0e6) as usize;
        (1..=n).all(|i| self.g.index(a.lerp(b, i as f64 / n as f64)).is_some_and(|c| self.free(c)))
    }

    /// Whether the straight feed from `a` to `b` stays exactly where the centre may go.
    fn exact_line(&self, a: Vec2, b: Vec2) -> Result<bool> {
        let m = (a.dist(b) / self.k.step).ceil().clamp(1.0, 1.0e6) as usize;
        for i in 1..=m {
            if !self.domain.allowed(a.lerp(b, i as f64 / m as f64))? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// The link's corners, with the corners a straight feed can skip left out. Consecutive
    /// corners are cell centres (where the centre may stand) a cell apart, so the link through
    /// all of them is always safe; a shortcut is taken only when it is too.
    fn shortcut(&self, pts: Vec<Vec2>) -> Result<Vec<Vec2>> {
        let Some(&first) = pts.first() else { return Ok(pts) };
        let mut out = vec![first];
        let mut i = 0usize;
        while i + 1 < pts.len() {
            let a = pts.get(i).copied().unwrap_or(first);
            let mut j = i + 1;
            while j + 1 < pts.len() && j < i + 2000 && pts.get(j + 1).is_some_and(|b| self.free_line(a, *b)) {
                j += 1;
            }
            if j > i + 1 && !pts.get(j).map_or(Ok(false), |b| self.exact_line(a, *b))? {
                j = i + 1;
            }
            if let Some(b) = pts.get(j) {
                out.push(*b);
            }
            i = j;
        }
        Ok(out)
    }

    /// The frontier cell nearest `near` anywhere (reached by lifting).
    fn any_frontier(&self, near: Option<Vec2>) -> Option<usize> {
        let mut best: Option<(usize, f64)> = None;
        for c in 0..self.g.len() {
            if !self.frontier(c) {
                continue;
            }
            let d = near.map_or(0.0, |p| self.g.center_of(c).dist(p));
            if best.is_none_or(|(_, b)| d < b) {
                best = Some((c, d));
                if near.is_none() {
                    break;
                }
            }
        }
        best.map(|(c, _)| c)
    }

    /// Whether some material is out of reach of every free cell (it needs an entry of its own).
    fn untouched(&mut self) -> Result<Option<Vec<f64>>> {
        let free: Vec<bool> = (0..self.g.len()).map(|c| self.free(c)).collect();
        let dfree = distance_field(&self.g, &|c| free.get(c).copied().unwrap_or(false), self.work)?;
        let reach = self.k.r + 3.0 * self.k.step + 2.0 * self.k.h;
        let any = dfree.iter().enumerate().any(|(c, d)| self.target(c) && *d > reach);
        Ok(any.then_some(dfree))
    }

    /// Where an entry helix goes into untouched material: the cell with the most room round it.
    fn helix_site(&self, dfree: &[f64], near: Option<Vec2>) -> Result<std::result::Result<Vec2, f64>> {
        let need = self.k.helix_radius + self.k.h;
        let reach = self.k.r + 3.0 * self.k.step + 2.0 * self.k.h;
        let mut best: Option<(usize, f64, f64)> = None;
        let mut most = 0.0f64;
        for c in 0..self.g.len() {
            let untouched = self.target(c) && dfree.get(c).is_some_and(|d| *d > reach);
            if !untouched || !self.allow.get(c).copied().unwrap_or(false) {
                continue;
            }
            let room = self.room.get(c).copied().unwrap_or(0.0);
            most = most.max(room);
            if room < need {
                continue;
            }
            // Of the cells with room, the one nearest the tool (the most room when it is up).
            let d = near.map_or(-room, |p| self.g.center_of(c).dist(p) - room * 1e-3);
            if best.is_none_or(|(_, _, b)| d < b) {
                best = Some((c, room, d));
            }
        }
        let Some((c, _, _)) = best else { return Ok(Err(most)) };
        let center = self.g.center_of(c);
        let rad = self.k.helix_radius;
        let n = ((TAU * rad / self.k.step).ceil() as usize).clamp(16, 4096);
        for i in 0..n {
            if !self.domain.allowed(center + Vec2::from_angle(TAU * i as f64 / n as f64) * rad)? {
                return Ok(Err(most.min(rad)));
            }
        }
        Ok(Ok(center))
    }

    /// Walks from `start` heading `heading`, clearing as it goes; the points from `start` on,
    /// ending at the last one that cut.
    fn walk(&mut self, start: Vec2, heading: Vec2) -> Result<Vec<Vec2>> {
        let k = self.k;
        let mut pts = vec![start];
        let (mut pos, mut dir) = (start, heading.normalized().unwrap_or(Vec2::new(1.0, 0.0)));
        let mut air = 0.0;
        let mut last_cut = 0usize;
        let max_air = k.max_air();
        let side = if k.climb { 1.0 } else { -1.0 };
        // Steps through air not yet recorded: they are recorded when the walk cuts again, and
        // dropped with the end of the walk otherwise, so the model holds only what is cut.
        let mut pending: Vec<(Vec2, Vec2)> = Vec::new();
        let mut fan: Vec<(f64, Vec2, Vec2, f64)> = Vec::with_capacity(4 * FAN as usize + 2);
        loop {
            if *self.steps >= MAX_STEPS {
                return Err(CamError::TooLarge(format!(
                    "the adaptive clearing needs more than {MAX_STEPS} steps; use a larger optimal load or engagement limit, or a smaller region"
                )));
            }
            if pts.len() >= MAX_MOVES / 4 {
                break;
            }
            let mut next = None;
            // The usual fan first; when no heading in it keeps within the limit, sharper turns
            // with shorter steps (turning away from the material lightens the cut).
            for (spread, length) in [(MAX_TURN, k.step), (4.0 * MAX_TURN, 0.5 * k.step)] {
                fan.clear();
                for i in -FAN..=FAN {
                    let turn = spread * i as f64 / FAN as f64;
                    let d = rotate(dir, turn);
                    let q = pos + d * length;
                    if !self.grid_allows(q) {
                        continue;
                    }
                    let (phi, mdir) = self.engage(q)?;
                    if phi > k.limit {
                        continue;
                    }
                    let score = if phi > 0.0 {
                        // Material on the wrong side of the travel costs as much as missing the load.
                        let wrong = (d.cross(mdir) * side).max(0.0) / mdir.len().max(1e-12);
                        let light = (k.least - phi).max(0.0);
                        (phi - k.target).abs() + light + wrong * k.target + 0.05 * turn.abs()
                    } else {
                        // Cutting air: towards the nearest material.
                        10.0 + self.dmat_at(q).min(1.0e6) / k.r + 0.05 * turn.abs()
                    };
                    fan.push((score, q, d, phi));
                }
                fan.sort_by(|a, b| a.0.total_cmp(&b.0));
                for &(_, q, d, phi) in &fan {
                    if self.domain.allowed(q)? {
                        next = Some((q, d, phi));
                        break;
                    }
                }
                if next.is_some() {
                    break;
                }
            }
            let Some((q, d, phi)) = next else { break };
            *self.steps += 1;
            let cleared = if phi > 0.0 {
                for (a, b) in std::mem::take(&mut pending) {
                    self.sweep(a, b)?;
                }
                self.sweep(pos, q)?
            } else {
                pending.push((pos, q));
                0
            };
            pts.push(q);
            pos = q;
            dir = d;
            if phi > 0.0 {
                // Recorded in the model: the path must keep it.
                last_cut = pts.len() - 1;
            }
            // Material in the tool's circle at the new point is cut by the move there: the
            // walk is cutting, whether or not a cell's centre was cleared (a load lighter than
            // the raster's cell clears few of them).
            if cleared > 0 || phi > 0.0 {
                air = 0.0;
            } else {
                air += pos.dist(pts.get(pts.len().saturating_sub(2)).copied().unwrap_or(pos)).max(1e-9);
                if air > max_air {
                    break;
                }
            }
        }
        pts.truncate(last_cut + 1);
        Ok(pts)
    }

    /// Plans the level: the steps from wherever the tool is (`None`: up) to every cut.
    fn plan(&mut self, mut pos: Option<Vec2>, floor_at: &dyn Fn(Vec2, f64) -> Option<f64>) -> Result<(Vec<Step>, usize)> {
        let mut steps = Vec::new();
        let mut helices = 0usize;
        for _ in 0..MAX_PASSES {
            self.refresh()?;
            if self.material_cells() == 0 {
                break;
            }
            let linked = match pos {
                Some(p) => self.linked_frontier(p)?,
                None => None,
            };
            let (lead, start, heading) = if let Some((link, c)) = linked {
                if let Some(t) = self.tried.get_mut(c) {
                    *t = true;
                }
                let at = self.g.center_of(c);
                (Some(Step::Link(link)), at, self.heading_for(self.towards_material(c)))
            } else if let Some(c) = self.any_frontier(pos) {
                if let Some(t) = self.tried.get_mut(c) {
                    *t = true;
                }
                let at = self.g.center_of(c);
                (Some(Step::Drop(at)), at, self.heading_for(self.towards_material(c)))
            } else if let Some(dfree) = self.untouched()? {
                let center = match self.helix_site(&dfree, pos)? {
                    Ok(c) => c,
                    Err(most) => {
                        return invalid(format!(
                            "material the tool cannot reach from the cleared area within the engagement limit has no room for the entry helix of diameter {:.3} mm (the most room is about {:.3} mm across); use a smaller helix diameter, a smaller tool or a larger engagement limit",
                            2.0 * self.k.helix_radius,
                            2.0 * most
                        ));
                    }
                };
                let rad = self.k.helix_radius;
                let from = floor_at(center, rad + self.k.r);
                steps.push(Step::Helix { center, from });
                helices += 1;
                self.bore(center, rad)?;
                let at = center + Vec2::new(rad, 0.0);
                pos = Some(at);
                let tangent = if self.k.climb { Vec2::new(0.0, 1.0) } else { Vec2::new(0.0, -1.0) };
                // The bore's outside is material: walk round it from where the helix ends.
                self.refresh()?;
                let pts = self.walk(at, tangent)?;
                if pts.len() >= 2 {
                    pos = pts.last().copied();
                    steps.push(Step::Cut(pts));
                }
                continue;
            } else {
                break;
            };
            let pts = self.walk(start, heading)?;
            if pts.len() >= 2 {
                if let Some(l) = lead {
                    steps.push(l);
                }
                pos = pts.last().copied();
                steps.push(Step::Cut(pts));
            }
        }
        Ok((steps, helices))
    }
}

/// Points of a polyline within `tol` of it, its ends kept (Douglas-Peucker, without recursion).
fn simplify(pts: &[Vec2], tol: f64) -> Vec<Vec2> {
    let n = pts.len();
    if n <= 2 || !(tol > 0.0) {
        return pts.to_vec();
    }
    let mut keep = vec![false; n];
    if let Some(k) = keep.first_mut() {
        *k = true;
    }
    if let Some(k) = keep.last_mut() {
        *k = true;
    }
    let mut stack = vec![(0usize, n - 1)];
    while let Some((a, b)) = stack.pop() {
        if b <= a + 1 {
            continue;
        }
        let (Some(pa), Some(pb)) = (pts.get(a).copied(), pts.get(b).copied()) else { continue };
        let mut worst = (a, 0.0f64);
        for (i, p) in pts.iter().enumerate().take(b).skip(a + 1) {
            let d = seg_dist(*p, pa, pb);
            if d > worst.1 {
                worst = (i, d);
            }
        }
        if worst.1 > tol {
            if let Some(k) = keep.get_mut(worst.0) {
                *k = true;
            }
            stack.push((a, worst.0));
            stack.push((worst.0, b));
        }
    }
    pts.iter().zip(&keep).filter(|(_, k)| **k).map(|(p, _)| *p).collect()
}

/// Collects moves with their roles, keeping the tool's position.
struct Emit {
    moves: Vec<Move>,
    roles: Vec<MoveRole>,
    pos: Option<Vec3>,
    h: Heights,
    feeds: Feeds,
    lifts: usize,
}

impl Emit {
    fn new(h: Heights, feeds: Feeds) -> Emit {
        Emit { moves: Vec::new(), roles: Vec::new(), pos: None, h, feeds, lifts: 0 }
    }

    fn push(&mut self, m: Move, role: MoveRole) -> Result<()> {
        if self.moves.len() >= MAX_MOVES {
            return Err(CamError::TooLarge(format!("the toolpath has more than {MAX_MOVES} moves")));
        }
        if let Some(to) = m.target() {
            self.pos = Some(to);
        }
        self.moves.push(m);
        self.roles.push(role);
        Ok(())
    }

    /// Up to the retract height if below it.
    fn up(&mut self) -> Result<()> {
        match self.pos {
            Some(q) if q.z < self.h.retract => self.push(Move::Rapid { to: Vec3::new(q.x, q.y, self.h.retract) }, MoveRole::Position),
            _ => Ok(()),
        }
    }

    /// Over `p` at the retract height (from the clearance height for the first move).
    fn over(&mut self, p: Vec2) -> Result<()> {
        match self.pos {
            None => {
                self.push(Move::Rapid { to: p.extend(self.h.clearance) }, MoveRole::Position)?;
                self.push(Move::Rapid { to: p.extend(self.h.retract) }, MoveRole::Position)
            }
            Some(q) => {
                if q.z < self.h.retract {
                    self.lifts += 1;
                }
                self.up()?;
                if Vec2::new(q.x, q.y).dist(p) > 1e-9 { self.push(Move::Rapid { to: p.extend(self.h.retract) }, MoveRole::Position) } else { Ok(()) }
            }
        }
    }

    /// Straight down to `z` at the plunge feed (through air).
    fn down(&mut self, z: f64) -> Result<()> {
        match self.pos {
            Some(q) if q.z > z + 1e-9 => self.push(Move::Feed { to: Vec3::new(q.x, q.y, z), feed: self.feeds.plunge }, MoveRole::Position),
            _ => Ok(()),
        }
    }

    fn feed_to(&mut self, p: Vec2, z: f64, role: MoveRole) -> Result<()> {
        if self.pos.is_some_and(|q| Vec2::new(q.x, q.y).dist(p) <= 1e-9 && (q.z - z).abs() <= 1e-9) {
            return Ok(());
        }
        self.push(Move::Feed { to: p.extend(z), feed: self.feeds.cut }, role)
    }

    /// Down a helix of radius `rad` about `c` from `from` to `z`, starting and ending at
    /// `c + (rad, 0)` after a last circle at `z`; counter-clockwise when `ccw`.
    fn helix(&mut self, c: Vec2, rad: f64, from: f64, z: f64, ramp: f64, ccw: bool) -> Result<()> {
        let start = c + Vec2::new(rad, 0.0);
        self.over(start)?;
        self.down(from.max(z))?;
        let drop = from - z;
        let per_half = PI * rad * ramp.tan();
        let halves = if drop > 1e-9 { (drop / per_half - 1e-9).ceil().max(1.0) } else { 0.0 };
        if !(halves <= MAX_HALF_TURNS) {
            return invalid("the entry helix needs too many turns; use a steeper ramp angle or a larger helix diameter");
        }
        // An even number of half turns, so the helix ends where it started.
        let halves = (halves / 2.0).ceil() as usize * 2;
        let sign = if ccw { 1.0 } else { -1.0 };
        let total = halves + 2;
        for k in 1..=total {
            let p = c + Vec2::from_angle(sign * PI * k as f64) * rad;
            let zk = if k >= halves { z } else { from - drop * k as f64 / halves.max(1) as f64 };
            let dz = self.pos.map_or(0.0, |q| q.z - zk);
            let feed = descent_feed(&self.feeds, (PI * rad).hypot(dz), dz);
            self.push(Move::Arc { to: p.extend(zk), center: c, ccw, feed }, MoveRole::Entry)?;
        }
        Ok(())
    }

    fn finish(mut self) -> Result<(Toolpath, Vec<MoveRole>, usize)> {
        if let Some(q) = self.pos
            && q.z < self.h.clearance
        {
            self.push(Move::Rapid { to: Vec3::new(q.x, q.y, self.h.clearance) }, MoveRole::Position)?;
        }
        let path = Toolpath { moves: self.moves };
        verify(&path, &self.h)?;
        Ok((path, self.roles, self.lifts))
    }
}

/// Writes one level's steps at `z`; `slab_top`: where the material starts above the level.
fn emit_level(e: &mut Emit, steps: &[Step], z: f64, slab_top: f64, k: &Knobs) -> Result<()> {
    for step in steps {
        match step {
            Step::Drop(p) => {
                e.over(*p)?;
                e.down(z)?;
            }
            Step::Helix { center, from } => {
                let top = from.unwrap_or(slab_top).max(z).min(e.h.retract);
                e.helix(*center, k.helix_radius, top, z, k.ramp, k.climb)?;
            }
            Step::Link(pts) => {
                for p in pts.iter().skip(1) {
                    e.feed_to(*p, z, MoveRole::Link)?;
                }
            }
            Step::Cut(pts) => {
                for p in simplify(pts, k.simplify).iter().skip(1) {
                    e.feed_to(*p, z, MoveRole::Cut)?;
                }
            }
        }
    }
    e.up()
}

/// Points along a path's cutting moves, at most `spacing` apart (arcs followed).
fn cutting_samples(path: &Toolpath, spacing: f64) -> Vec<Vec3> {
    let mut out = Vec::new();
    let mut pos: Option<Vec3> = None;
    for m in &path.moves {
        match *m {
            Move::Feed { to, .. } => {
                if let Some(p) = pos {
                    let n = (p.dist(to) / spacing).ceil().clamp(1.0, 1.0e5) as usize;
                    out.extend((1..=n).map(|i| p.lerp(to, i as f64 / n as f64)));
                }
            }
            Move::Arc { to, center, ccw, .. } => {
                if let Some(p) = pos {
                    let start = Vec2::new(p.x, p.y) - center;
                    let (r, sweep) = (start.len(), arc_sweep(p, to, center, ccw));
                    let n = ((r * sweep.abs()).hypot(to.z - p.z) / spacing).ceil().clamp(1.0, 1.0e5) as usize;
                    for i in 1..=n {
                        let t = i as f64 / n as f64;
                        out.push((center + Vec2::from_angle(start.angle() + sweep * t) * r).extend(p.z + (to.z - p.z) * t));
                    }
                }
            }
            Move::PlaneArc { to, center, plane, ccw, .. } => {
                if let Some(from) = pos {
                    let arc = crate::path::PlaneArc { from, to, center, plane, ccw };
                    let n = (arc.length() / spacing).ceil().clamp(1.0, 1.0e5) as usize;
                    out.extend((1..=n).map(|i| arc.at(i as f64 / n as f64)));
                }
            }
            _ => {}
        }
        if let Some(to) = m.target() {
            pos = Some(to);
        }
    }
    out
}

/// Adaptive clearing of `regions` (holes are islands) in depth passes, with nothing known of
/// the stock but that it fills the regions: [`adaptive_in`] with the default [`StockModel`].
pub fn adaptive(regions: &[Region2], tool: &Tool, params: &AdaptiveParams, cut: &Cut) -> Result<AdaptivePath> {
    adaptive_in(regions, tool, params, cut, &StockModel::default())
}

/// Adaptive clearing of `regions` (holes are islands) in depth passes, cleared out to the walls
/// (less the stock to leave), of the material `stock` says is there.
///
/// With the stock's extent, the material is the regions' part of it, and the tool may also
/// stand past a region's boundary where it meets no stock outside the regions: it goes down in
/// the open air beside the stock and cuts in from there, and never cuts air where a region
/// reaches past the stock. Material no walk can reach from open or cleared space is entered with
/// a helix, refused when the helix has no room. Levels at or above the stock's top are left
/// out. Without a map of earlier cuts every level is the same and is planned once; with one,
/// each level is planned on what the map leaves at it (rest machining).
pub fn adaptive_in(regions: &[Region2], tool: &Tool, params: &AdaptiveParams, cut: &Cut, stock: &StockModel) -> Result<AdaptivePath> {
    mills(tool, params.ramp_angle)?;
    params.validate(tool)?;
    cut.heights.validate()?;
    cut.feeds.validate()?;
    check_stepdown(cut.stepdown)?;
    stock.validate()?;
    let all = levels(cut.heights.top, cut.heights.bottom, cut.stepdown, false)?;
    check_flutes(tool, cut.heights.top, &all)?;
    let k = Knobs::new(tool, params);
    let shape = Shape::from_regions(regions, params.tolerance)?;
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for piece in shape.loops().iter().flatten() {
        let (a, b) = piece.bounds();
        lo = Vec2::new(lo.x.min(a.x), lo.y.min(a.y));
        hi = Vec2::new(hi.x.max(b.x), hi.y.max(b.y));
    }
    let wall = k.r + params.stock_to_leave + k.margin;
    // With the stock's extent the tool may go past the regions' boundary into open air: the
    // raster reaches as far round them as an adaptive roughing's round its stock.
    let grow = if stock.rect.is_some() { k.r + 2.0 * k.step + 4.0 * k.h } else { 2.0 * k.h };
    let g = Grid::new(lo - Vec2::new(grow, grow), hi + Vec2::new(grow, grow), k.h)?;
    let pieces = shape.loops().iter().map(Vec::len).sum::<usize>().max(1);
    let fence = stock.rect.map(|[slo, shi]| Fence::new(shape.clone(), slo, shi, wall));
    let walls = Walls { shape, wall };
    let domain: &dyn Domain = match &fence {
        Some(f) => f,
        None => &walls,
    };
    let fence_pieces = fence.as_ref().map_or(0, |f| f.walls.len());
    let top = stock.top.map_or(cut.heights.top, |t| t.min(cut.heights.top));
    let zs: Vec<f64> = all.iter().copied().filter(|z| *z < top - 1e-9).collect();
    let per_level = stock.heights.is_some();
    let planned_levels = if per_level { zs.len() } else { zs.len().min(1) };
    let mut work = Work::new(MAX_WORK);
    work.add(g.len() as f64 * (pieces + fence_pieces) as f64 * 0.05)?;
    let mut allow = Vec::with_capacity(g.len());
    for c in 0..g.len() {
        allow.push(domain.allowed(g.center_of(c))?);
    }
    let room = allow.iter().filter(|a| **a).count();
    if room == 0 {
        return invalid("the tool does not fit the region");
    }
    // Refused at once where the walks would take too many steps: the area the centre may
    // cover and the tool's reach round it, at every level planned.
    let area = room as f64 * k.h * k.h + (hi.x - lo.x + hi.y - lo.y) * 2.0 * k.r;
    let fits = |kn: &Knobs| expected_steps(kn, area) * planned_levels as f64 <= MAX_STEPS as f64;
    if !fits(&k) {
        return Err(CamError::TooLarge(format!(
            "the adaptive clearing at an optimal load of {} mm needs some {:.2e} steps over {area:.0} mm² and {planned_levels} planned levels, more than {MAX_STEPS}; {}",
            params.optimal_load,
            expected_steps(&k, area) * planned_levels as f64,
            load_hint(tool, params, &fits)
        )));
    }
    let mut e = Emit::new(cut.heights, cut.feeds);
    let mut slab_top = top;
    let (mut helices, mut left, mut cut_levels) = (0usize, 0usize, 0usize);
    let mut once: Option<(Vec<Step>, usize, usize)> = None;
    for &z in &zs {
        let (plan, n_helix, n_left) = match &once {
            Some(p) if !per_level => p.clone(),
            _ => {
                let mut steps_taken = 0usize;
                let cell_stock = |p: Vec2| walls.shape.inside(p, k.h) && stock.material_at(p, z);
                let at = |p: Vec2| stock.material_at(p, z);
                let cap = slab_top;
                // A plan made once is written at every level: its helices start from the level
                // above where they are written (`None`), cleared by then. A level planned on its
                // own starts them where the map leaves material over them.
                let floor_at = |c: Vec2, rad: f64| per_level.then(|| stock.top_over(c, rad, cap).max(z));
                let mut lv = Level::new(g, k, domain, allow.clone(), &cell_stock, &at, &mut steps_taken, &mut work)?;
                let (plan, n_helix) = lv.plan(None, &floor_at)?;
                let n_left = lv.material_cells();
                (plan, n_helix, n_left)
            }
        };
        if !per_level && once.is_none() {
            once = Some((plan.clone(), n_helix, n_left));
        }
        if !plan.is_empty() {
            cut_levels += 1;
        }
        helices += n_helix;
        left += n_left;
        emit_level(&mut e, &plan, z, slab_top, &k)?;
        slab_top = z;
    }
    let (path, roles, lifts) = e.finish()?;
    // The path's own check: every cutting point stays inside the walls (off the stock outside
    // the regions).
    let wall = k.r + params.stock_to_leave;
    for p in cutting_samples(&path, k.r * 0.25) {
        let q = Vec2::new(p.x, p.y);
        let ok = match &fence {
            Some(f) => f.clear(q, wall - 1e-6),
            None => walls.shape.clear(q, wall - 1e-6),
        };
        if !ok {
            return Err(CamError::Unsafe(format!("the adaptive path comes closer than the tool radius to a wall at ({:.4}, {:.4})", p.x, p.y)));
        }
    }
    let left_area = left as f64 * k.h * k.h;
    let mut warnings = Vec::new();
    if cut_levels == 0 {
        warnings.push("there is no stock to clear in the regions between the operation's top and bottom".to_string());
    }
    if left > 0 {
        warnings.push(format!("about {left_area:.2} mm² of material within reach was left (narrow corners the walk did not get into)"));
    }
    Ok(AdaptivePath { path, roles, levels: cut_levels, helices, lifts, left_area, warnings })
}

/// The stock rectangle of an adaptive roughing.
fn stock_rect(b: &Boundary, surface: &Surface) -> Result<(Vec2, Vec2)> {
    let (lo, hi) = match b.rect {
        Some([a, c]) => (a, c),
        None => {
            let bb = surface.bounds();
            if bb.is_empty() {
                return invalid("the surface has no triangles: give the boundary's rectangle");
            }
            (Vec2::new(bb.min.x, bb.min.y), Vec2::new(bb.max.x, bb.max.y))
        }
    };
    if ![lo.x, lo.y, hi.x, hi.y].iter().all(|v| v.is_finite() && v.abs() <= MAX_COORD) {
        return invalid("the boundary's corners must be finite numbers within 1000 m");
    }
    if !(lo.x < hi.x) || !(lo.y < hi.y) {
        return invalid("the boundary's low corner must be below and left of its high corner");
    }
    Ok((lo, hi))
}

/// The heights of the flat, upward-facing areas of the surface over the rectangle.
fn flat_heights(surface: &Surface, lo: Vec2, hi: Vec2) -> Vec<f64> {
    let mut zs: Vec<f64> = Vec::new();
    for (i, t) in surface.triangles().iter().enumerate() {
        let Some(n) = surface.facet_normal(i) else { continue };
        if n.z < 0.9999 {
            continue;
        }
        let (minx, maxx) = (t[0].x.min(t[1].x).min(t[2].x), t[0].x.max(t[1].x).max(t[2].x));
        let (miny, maxy) = (t[0].y.min(t[1].y).min(t[2].y), t[0].y.max(t[1].y).max(t[2].y));
        if maxx < lo.x || minx > hi.x || maxy < lo.y || miny > hi.y {
            continue;
        }
        let z = t[0].z.max(t[1].z).max(t[2].z);
        if !zs.iter().any(|q| (q - z).abs() < 1e-6) {
            zs.push(z);
            if zs.len() > 10_000 {
                break;
            }
        }
    }
    zs
}

/// 3D adaptive roughing: the stock box from `heights.top` down to `heights.bottom` in levels at
/// most a stepdown apart (and at the flats, with `flats`), each cleared with the engagement-limited
/// walk where the dropped cutter (grown by the stock to leave) is at or below the level. The
/// tool enters each level from open air outside the stock where it can, and with a helix where
/// the material is enclosed. Every cutting point is checked with the drop-cutter before the path
/// is returned.
pub fn adaptive_roughing(surface: &Surface, tool: &Tool, p: &AdaptiveRoughingParams) -> Result<AdaptivePath> {
    mills(tool, p.adaptive.ramp_angle)?;
    let cutter = tool_cutter(tool)?;
    p.adaptive.validate(tool)?;
    p.heights.validate()?;
    p.feeds.validate()?;
    check_stepdown(p.stepdown)?;
    let k = Knobs::new(tool, &p.adaptive);
    let leave = p.adaptive.stock_to_leave;
    let (slo, shi) = stock_rect(&p.boundary, surface)?;
    let fence: Option<(Ring<Vec2>, Vec<Ring<Vec2>>)> =
        p.boundary.region.as_ref().map(|r| {
            (r.outer.polyline(p.adaptive.tolerance.min(1e-3)), r.holes.iter().map(|h| h.polyline(p.adaptive.tolerance.min(1e-3))).collect())
        });
    let in_stock = |q: Vec2| {
        q.x >= slo.x
            && q.x <= shi.x
            && q.y >= slo.y
            && q.y <= shi.y
            && fence.as_ref().is_none_or(|(o, hs)| o.contains(q) && !hs.iter().any(|h| h.contains(q)))
    };
    let grow = k.r + 2.0 * k.step + 4.0 * k.h;
    let (rlo, rhi) = (slo - Vec2::new(grow, grow), shi + Vec2::new(grow, grow));
    let g = Grid::new(rlo - Vec2::new(k.h, k.h), rhi + Vec2::new(k.h, k.h), k.h)?;
    // Rapids run at the retract height: it must clear the part over the stock and round it.
    let reach = k.r + leave;
    let mut top = f64::NEG_INFINITY;
    for t in surface.triangles() {
        let (minx, maxx) = (t[0].x.min(t[1].x).min(t[2].x), t[0].x.max(t[1].x).max(t[2].x));
        let (miny, maxy) = (t[0].y.min(t[1].y).min(t[2].y), t[0].y.max(t[1].y).max(t[2].y));
        if maxx >= rlo.x - reach && minx <= rhi.x + reach && maxy >= rlo.y - reach && miny <= rhi.y + reach {
            top = top.max(t[0].z.max(t[1].z).max(t[2].z));
        }
    }
    if top.is_finite() && p.heights.retract < top + leave - 1e-9 {
        return invalid(format!(
            "the retract height {} is below the top of the surface ({}, stock to leave included)",
            p.heights.retract,
            top + leave
        ));
    }
    let mut zs = levels(p.heights.top, p.heights.bottom, p.stepdown, false)?;
    if p.flats {
        for f in flat_heights(surface, rlo, rhi) {
            let z = f + leave;
            if z > p.heights.bottom + 1e-6 && z < p.heights.top - 1e-6 && !zs.iter().any(|q| (q - z).abs() < p.adaptive.tolerance.max(1e-3)) {
                zs.push(z);
            }
        }
        zs.sort_by(|a, b| b.total_cmp(a));
    }
    check_flutes(tool, p.heights.top, &zs)?;
    // Grown by the stock to leave all round and by the margin sideways only, so a floor at the
    // stock to leave is a level the centre may stand at.
    let grown = {
        let g = cutter.grown(leave)?;
        let c = Cutter { radius: g.radius + k.margin, ..g };
        c.validate()?;
        c
    };
    if let Some(m) = &p.rest {
        m.validate()?;
    }
    // What earlier cuts left (rest machining): material thinner than this above a level is
    // not chased.
    let left_by = StockModel { rect: None, top: None, heights: p.rest.clone(), rest_tolerance: p.adaptive.tolerance.max(REST_TOLERANCE) };
    // Refused at once where the levels over this raster would take minutes or their walks too
    // many steps, with the smallest load that would plan; each level is then held to a budget
    // of its own.
    let area = (shi.x - slo.x) * (shi.y - slo.y);
    let fits = |kn: &Knobs| {
        let steps = expected_steps(kn, area);
        zs.len() as f64 * (g.len() as f64 * TYPICAL_LEVEL_WORK_PER_CELL + steps * kn.step_work()) <= MAX_ROUGHING_WORK && steps <= MAX_STEPS as f64
    };
    check_roughing_work(zs.len(), g.len(), expected_steps(&k, area), k.step_work(), &|| load_hint(tool, &p.adaptive, &fits))?;
    let budget = Budget::new();
    let mut work = Work::new(0.0);
    // How far down each cell has been cleared by the levels so far (or by earlier cuts).
    let mut floor: Vec<f64> = (0..g.len()).map(|c| left_by.top_over(g.center_of(c), 0.0, p.heights.top)).collect();
    let mut e = Emit::new(p.heights, p.feeds);
    let (mut helices, mut cut_levels, mut left) = (0usize, 0usize, 0usize);
    for &z in &zs {
        // Each level's work and walk steps are its own.
        let mut steps_taken = 0usize;
        let domain = Clear { surface, cutter: grown, grown_by: leave, z, lo: rlo, hi: rhi, budget: &budget };
        let mut allow = Vec::with_capacity(g.len());
        for c in 0..g.len() {
            allow.push(domain.allowed(g.center_of(c))?);
        }
        let before: Vec<bool>;
        let (plan, n_helix, n_left) = {
            let here = |q: Vec2| in_stock(q) && left_by.material_at(q, z);
            let mut lv = Level::new(g, k, &domain, allow, &here, &here, &mut steps_taken, &mut work)?;
            before = lv.mat.clone();
            let floor_ref = &floor;
            let floor_at = |c: Vec2, rad: f64| -> Option<f64> {
                let (is, js) = g.span(c, c, rad);
                let mut most = z;
                for j in js {
                    for i in is.clone() {
                        if g.center(i, j).dist(c) <= rad {
                            most = most.max(floor_ref.get(j * g.nx + i).copied().unwrap_or(p.heights.top));
                        }
                    }
                }
                Some(most)
            };
            let (plan, n_helix) = lv.plan(None, &floor_at)?;
            let n_left = lv.material_cells();
            for (c, (was, now)) in before.iter().zip(&lv.mat).enumerate() {
                if *was
                    && !*now
                    && let Some(f) = floor.get_mut(c)
                {
                    *f = f.min(z);
                }
            }
            (plan, n_helix, n_left)
        };
        if plan.is_empty() {
            continue;
        }
        cut_levels += 1;
        helices += n_helix;
        left += n_left;
        emit_level(&mut e, &plan, z, p.heights.top, &k)?;
    }
    let (path, roles, lifts) = e.finish()?;
    // The path's own check: no cutting point is below where the cutter rests on the surface.
    let rest = cutter.grown(leave)?;
    for q in cutting_samples(&path, k.r * 0.25) {
        budget.spend()?;
        if let Some(zg) = drop_z(surface, &rest, q.x, q.y)
            && zg + leave > q.z + 1e-6
        {
            return Err(CamError::Unsafe(format!("the adaptive path cuts into the part at ({:.4}, {:.4}, {:.4})", q.x, q.y, q.z)));
        }
    }
    let left_area = left as f64 * k.h * k.h;
    let mut warnings = Vec::new();
    if p.boundary.rect.is_none() {
        // The tool goes down outside the stock as open air: a blank larger than the part would be
        // cut into from the side at the level's full depth.
        warnings.push(
            "the stock was taken as the part's extent seen from above, and the tool goes down to each level just outside it: if the blank is larger, give the boundary's rectangle as the blank's extent"
                .into(),
        );
    }
    if left > 0 {
        warnings
            .push(format!("about {left_area:.2} mm² of material within reach was left over the levels (narrow corners the walk did not get into)"));
    }
    Ok(AdaptivePath { path, roles, levels: cut_levels, helices, lifts, left_area, warnings })
}
