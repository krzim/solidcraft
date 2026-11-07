//! Stock-removal simulation: an independent check of a written program. The G-code is read
//! with [`parse5`] as the controller would run it, the tool is swept through a model of the
//! stock along every block, and what is left is held against the part.
//!
//! Two stock models, both dexels (one column of material per cell of a grid):
//!
//! - **3-axis** ([`StockShape::Box`]): a heightfield over the stock box, one top height per
//!   cell, sampled at the cell's centre. Each block lowers every cell to the tool's lower
//!   envelope along it, computed exactly per cell rather than at points along the move: along
//!   a straight move the height of the tool above a cell is a convex function of the position
//!   along the move, so its least value is found in closed form (flat, ball, drill point) or by
//!   bisection on its slope to rounding (bull nose). A level arc is exact too (the nearest
//!   point of the arc to the cell). An arc that also moves in Z (a helix) is followed as
//!   chords at most a quarter of the tolerance from it.
//! - **4th axis** ([`StockShape::Cylinder`], or a [`StockShape::Box`] bar the axis runs
//!   through, on a rotary about work X through work zero): a grid over (x, θ) holding the radius
//!   the material reaches along the ray from the axis at that x and angle (for a box bar, from
//!   where the ray leaves its rectangle). A block that turns the rotary while X moves is a helix in the work frame, so
//!   the tool is placed at points along every block (every axis moving evenly from the block's
//!   start to its end, as written), each close enough to the last that the cusp the sampling
//!   leaves is at most half the tolerance: for a tool with a round corner of radius r the
//!   points are at most √(4·r·tol) apart (the tip's travel, and the travel of the cutting end
//!   as the axis turns), for a flat end mill √(4·R·tol) (the scallop its rim leaves sideways),
//!   for a drill point tol / (its slope). At each point the ray of every cell near the tool is
//!   cut where it first enters the tool (closed form for flat and ball, bisection for the
//!   others). A ray holds one stretch of material, so a cut that crosses a ray inside the
//!   material without reaching out of it (beside the ray, as an indexed pocket in the part's
//!   side does to the rays at a slant through it) is left out of that ray, and counted in a
//!   warning: that ray neither loses the material beyond the cut nor is checked for it.
//!
//! The tool is its cutting end with an endless cylinder of its diameter above (a cut deeper
//! than the flutes still removes material). A block before which the position is not known
//! (the program's first, or the first after a return home, `G28`/`G30`/`G53`) is taken as
//! coming straight down the tool axis from above the stock, which is where those moves leave
//! the tool. Rapids sweep a straight line like cuts.
//!
//! The grid is split into bands of rows (of angles on a 4th axis), one per core up to sixteen,
//! and every band runs the whole program in order on a thread of its own (one thread on the
//! web): a cell is only ever changed by its own band, so the result is the same as on one
//! thread, and what each block removed is the sum over the bands.
//!
//! What is left is compared with the part (a [`Surface`]): each cell's part height (radius) is
//! where a ray from above (from outside) first meets the part. A cell cut below the part by
//! more than the tolerance is measured again in 3D (the distance from the bottom and from the
//! middle of the cut column to the part's surface), so a cut beside a steep wall that stays
//! within the tolerance of the wall is not a gouge. Leftover is the height (radius) of the
//! stock above the part, along Z (along the ray), over the cells that were cut: material left
//! beside a steep wall counts its full height.
//!
//! [`simulate`] runs in three parts that can also be driven one at a time, for a view that
//! plays the program move by move ([`crate::playback`]): [`plan_sim`] reads the program into
//! steps (one per motion block) and the grid; [`SimState`] is the stock part way through, and
//! [`SimState::advance`] cuts it on to a later step, or part of the way through one (lowering a
//! cell is a minimum over the tool's positions, so cutting the first part of a move and then the
//! rest leaves what cutting all of it does, to rounding); [`SimState::finish`] holds what is left
//! against the part. Advancing whole steps in any number of calls leaves the stock bit for bit
//! as one call does. The stock keeps, per tile of [`TILE`] × [`TILE`] cells, an upper bound of
//! its heights (radii) for quick rejects and a version that changes whenever a cell of the tile
//! is cut, and per cell the tool that cut it last, for a view.

use std::cell::Cell;
use std::f64::consts::{PI, TAU};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use solidcore_record::{Vec2, Vec3};
use solidcraft_geom::prelude::*;
use solidcraft_geom::{Aabb3, Mesh};

use crate::gcode::words;
use crate::gcode5::{JointWords, parse5};
use crate::kinematics::{Joints, Machine, forward, tool_axis};
use crate::multiaxis::Pose;
use crate::path::arc_sweep;
use crate::surface::{Surface, nearest_on_triangle};
use crate::tool::{Tool, ToolKind};
use crate::{CamError, Result, invalid};

/// Most cells a stock grid may have.
pub const MAX_CELLS: usize = 20_000_000;
/// Most cells the grid has when no resolution is given.
pub const DEFAULT_CELLS: usize = 4_000_000;
/// The cell size when no resolution is given, in multiples of the tolerance (coarser when the
/// stock is too large for [`DEFAULT_CELLS`]).
pub const DEFAULT_CELL_TOLERANCES: f64 = 5.0;
/// The point angle a drill is simulated with (radians): 118°.
pub const DRILL_POINT_ANGLE: f64 = 118.0 * PI / 180.0;
/// Most cell updates one simulation makes: beyond it the simulation is refused.
const MAX_VISITS: u64 = 4_000_000_000;
/// The share of [`MAX_VISITS`] a simulation without a resolution is fitted within.
const FIT_SHARE: f64 = 0.5;
/// Most tool positions along one block on a 4th axis, and chords of one arc.
const MAX_BLOCK_POINTS: usize = 1_000_000;
/// Most cells measured again in 3D for the gouge depth; beyond, the vertical depth is used.
const MAX_REFINED: usize = 1_000_000;
/// Most tool positions on a 4th axis over the whole program.
const MAX_STAMPS: usize = 50_000_000;
/// Most threads one simulation runs on, and the fewest rows (angles) each one takes.
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
const MAX_THREADS: usize = 16;
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
const MIN_BAND: usize = 16;
/// Iterations of the bisections (each halves the interval).
const BISECTIONS: usize = 60;
/// Largest coordinate accepted (mm).
const MAX_COORD: f64 = 1.0e6;
/// A slope taken as vertical (a tool's flank at its rim).
const STEEP: f64 = 1.0e12;
/// Cells along a side of a tile of the stock (its quick-reject maxima and versions).
pub const TILE: usize = 16;
/// The tool slot of a cell no tool has cut.
pub const UNCUT: u8 = 255;
/// Estimated cell visits above which one advance runs its bands on threads.
const THREADED_VISITS: f64 = 250_000.0;

/// The stock before the program runs, in work coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StockShape {
    /// A box (3-axis).
    Box { min: Vec3, max: Vec3 },
    /// A cylinder about work X from `x_min` to `x_max` (a 4th axis about X).
    Cylinder { x_min: f64, x_max: f64, radius: f64 },
}

/// How finely and how strictly to simulate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimParams {
    /// Depths up to this count as touching the part, and the cusps the 4th-axis sampling may
    /// leave are half of it (mm, 0.0001 to 1).
    pub tolerance: f64,
    /// The cell size (mm); `None`: [`DEFAULT_CELL_TOLERANCES`] times the tolerance, coarser
    /// when that would be more than [`DEFAULT_CELLS`] cells.
    pub resolution: Option<f64>,
}

/// The stock the program leaves.
#[derive(Clone, Debug, PartialEq)]
pub enum StockGrid {
    /// Heights over the box's XY: cell `(i, j)` has its centre at `(x0 + (i + ½)·dx, y0 + (j + ½)·dy)`
    /// and its height at `heights[j·nx + i]`.
    Heights { x0: f64, y0: f64, dx: f64, dy: f64, nx: usize, ny: usize, bottom: f64, top: f64, heights: Vec<f64> },
    /// Radii over (x, θ): cell `(i, k)` has its centre at `x = x0 + (i + ½)·dx`, `θ = (k + ½)·dθ`
    /// (`dθ = 2π / nt`) and its radius at `radii[k·nx + i]`.
    Radii { x0: f64, dx: f64, nx: usize, nt: usize, radius: f64, radii: Vec<f64> },
}

/// One motion block as simulated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BlockRemoval {
    /// The G-code line, from 1.
    pub line: usize,
    pub rapid: bool,
    /// Material it removed (mm³).
    pub volume: f64,
    /// The deepest it cut into the stock as it was (mm, along Z or the radius).
    pub depth: f64,
    /// A cut at the tool's full width (a slot): material on both sides of the tool and under
    /// it ahead, halfway along the move.
    pub slot: bool,
}

/// What the simulation found.
#[derive(Clone, Debug, PartialEq)]
pub struct SimReport {
    /// The cell size used (mm; on a 4th axis, along X and round the stock's surface).
    pub cell: f64,
    pub cells: usize,
    /// The stock as the program leaves it.
    pub stock: StockGrid,
    /// Motion blocks simulated (0: the program has none whose position is known, as when a 4th
    /// axis is never written, so nothing was checked).
    pub moves: usize,
    /// Every motion block that removed material, in program order.
    pub removals: Vec<BlockRemoval>,
    /// Rapids that cut more than the tolerance deep: a crash on the machine.
    pub rapids_in_material: Vec<BlockRemoval>,
    pub removed_volume: f64,
    /// The deepest cut into the part beyond the tolerance (mm; 0: none) and the cells cut so.
    pub gouge_worst: f64,
    pub gouge_cells: usize,
    /// Material left above the part over the cells the program cut (mm).
    pub leftover_max: f64,
    pub leftover_mean: f64,
    pub machined_cells: usize,
    /// Time at feed as the controller times it, dwells included (minutes).
    pub minutes: f64,
    pub feed_length: f64,
    pub rapid_length: f64,
    pub warnings: Vec<String>,
}

/// The stock to start from when none is given: the part's box (3-axis), or the cylinder about
/// work X through the part's farthest point from the axis over the part's extent along X (a
/// 4th axis about X).
pub fn stock_around(part: &Surface, machine: &Machine) -> Result<StockShape> {
    if part.is_empty() {
        return invalid("the part has no surface");
    }
    let b = part.bounds();
    match Kind::of(machine)? {
        Kind::Three => Ok(StockShape::Box { min: b.min, max: b.max }),
        Kind::Rotary => {
            let radius = part.triangles().iter().flatten().map(|p| p.y.hypot(p.z)).fold(0.0, f64::max);
            Ok(StockShape::Cylinder { x_min: b.min.x, x_max: b.max.x, radius })
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Kind {
    Three,
    Rotary,
}

impl Kind {
    fn of(machine: &Machine) -> Result<Kind> {
        machine.validate()?;
        match machine.rotaries.as_slice() {
            [] => Ok(Kind::Three),
            [r] => {
                let dir = r.direction.normalized().unwrap_or(Vec3::ZERO);
                let at = r.center - machine.work_origin;
                if dir.x.abs() < 1.0 - 1e-9 {
                    return Err(CamError::Unsupported("the simulation handles a 4th axis about X only".into()));
                }
                if at.y.abs() > 1e-9 || at.z.abs() > 1e-9 {
                    return Err(CamError::Unsupported("the simulation needs the rotary axis through work zero".into()));
                }
                Ok(Kind::Rotary)
            }
            _ => Err(CamError::Unsupported("the simulation handles 3-axis machines and a 4th axis about X".into())),
        }
    }
}

/// The cutting end's shape for the simulation.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Shape {
    Flat,
    Ball,
    Bull,
    /// A point: height `slope·d` at distance `d` from the axis.
    Cone(f64),
    /// A flat tip of radius `tip`, then a cone: height `slope·(d - tip)` beyond it (a chamfer
    /// mill).
    Taper {
        slope: f64,
        tip: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Profile {
    pub(crate) radius: f64,
    pub(crate) corner: f64,
    shape: Shape,
}

impl Profile {
    pub(crate) fn of(tool: &Tool) -> Result<Profile> {
        tool.validate()?;
        let r = tool.radius();
        Ok(match tool.kind {
            // A thread mill is checked as its whole radius, as the gouge check does.
            ToolKind::FlatEndmill | ToolKind::ThreadMill => Profile { radius: r, corner: 0.0, shape: Shape::Flat },
            ToolKind::BallEndmill => Profile { radius: r, corner: r, shape: Shape::Ball },
            ToolKind::BullNose | ToolKind::FaceMill => {
                let c = tool.corner_radius.unwrap_or(0.0).clamp(0.0, r);
                if c <= 0.0 {
                    Profile { radius: r, corner: 0.0, shape: Shape::Flat }
                } else if c >= r {
                    Profile { radius: r, corner: r, shape: Shape::Ball }
                } else {
                    Profile { radius: r, corner: c, shape: Shape::Bull }
                }
            }
            ToolKind::Drill => Profile { radius: r, corner: 0.0, shape: Shape::Cone(1.0 / (crate::drill::point_angle(tool) * 0.5).tan()) },
            ToolKind::Chamfer => {
                let half = tool.angle_deg.unwrap_or(90.0).to_radians() * 0.5;
                let tip = (tool.tip_diameter.unwrap_or(0.0) * 0.5).clamp(0.0, r);
                Profile { radius: r, corner: 0.0, shape: Shape::Taper { slope: 1.0 / half.tan(), tip } }
            }
        })
    }

    /// Height of the cutting end above the tip at distance `d` from the axis (`d` taken within
    /// the radius).
    pub(crate) fn height(&self, d: f64) -> f64 {
        let d = d.clamp(0.0, self.radius);
        match self.shape {
            Shape::Flat => 0.0,
            Shape::Cone(k) => k * d,
            Shape::Taper { slope, tip } => slope * (d - tip).max(0.0),
            Shape::Ball | Shape::Bull => {
                let a = self.radius - self.corner;
                if d <= a {
                    return 0.0;
                }
                let e = (d - a).min(self.corner);
                self.corner - (self.corner * self.corner - e * e).max(0.0).sqrt()
            }
        }
    }

    /// The slope of [`Profile::height`] at `d`, [`STEEP`] at the rim.
    fn slope(&self, d: f64) -> f64 {
        match self.shape {
            Shape::Flat => 0.0,
            Shape::Cone(k) => k,
            Shape::Taper { slope, tip } => {
                if d <= tip {
                    0.0
                } else {
                    slope
                }
            }
            Shape::Ball | Shape::Bull => {
                let e = d - (self.radius - self.corner);
                if e <= 0.0 {
                    return 0.0;
                }
                let q = self.corner * self.corner - e * e;
                if q <= 0.0 { STEEP } else { (e / q.sqrt()).min(STEEP) }
            }
        }
    }

    /// Largest distance between tool positions on a 4th axis that leaves cusps of at most half
    /// of `tol` (module documentation).
    pub(crate) fn step(&self, tol: f64) -> f64 {
        let s = match self.shape {
            Shape::Flat => (4.0 * self.radius * tol).sqrt(),
            Shape::Ball | Shape::Bull => (4.0 * self.corner * tol).sqrt(),
            Shape::Cone(k) | Shape::Taper { slope: k, .. } => tol / k.max(1e-9),
        };
        s.clamp(1e-3, self.radius.max(1e-3))
    }

    /// The least height of the tool's lower envelope over a cell as the tip moves straight from
    /// `z0` (at 0) to `z0 + dz` (at `len`) along a line, the cell at `u` along it and `v` (≥ 0)
    /// to its side; `None` when the tool never covers the cell. Exact: the height is convex
    /// along the move, its least value on the covered stretch is where its slope is zero.
    pub(crate) fn along_line(&self, u: f64, v: f64, len: f64, z0: f64, dz: f64) -> Option<f64> {
        let r = self.radius;
        if v > r {
            return None;
        }
        if len <= 1e-12 {
            let d = hyp(u, v);
            return (d <= r).then(|| z0.min(z0 + dz) + self.height(d));
        }
        let w = (r * r - v * v).max(0.0).sqrt();
        let (lo, hi) = ((u - w).max(0.0), (u + w).min(len));
        if lo > hi {
            return None;
        }
        let m = dz / len;
        let s = if hi - lo <= 1e-12 {
            lo
        } else {
            match self.shape {
                Shape::Flat => {
                    if m > 0.0 {
                        lo
                    } else {
                        hi
                    }
                }
                Shape::Ball => (u - dz * w / hyp(len, dz)).clamp(lo, hi),
                Shape::Cone(k) => {
                    if m.abs() >= k {
                        if m > 0.0 { lo } else { hi }
                    } else {
                        let c = -m / k;
                        (u + c * v / (1.0 - c * c).sqrt()).clamp(lo, hi)
                    }
                }
                Shape::Bull | Shape::Taper { .. } => {
                    let slope = |s: f64| {
                        let t = s - u;
                        let d = hyp(t, v);
                        if d <= 1e-15 { m } else { m + self.slope(d) * t / d }
                    };
                    if slope(lo) >= 0.0 {
                        lo
                    } else if slope(hi) <= 0.0 {
                        hi
                    } else {
                        let (mut a, mut b) = (lo, hi);
                        for _ in 0..BISECTIONS {
                            let mid = 0.5 * (a + b);
                            if slope(mid) < 0.0 {
                                a = mid;
                            } else {
                                b = mid;
                            }
                        }
                        0.5 * (a + b)
                    }
                }
            }
        };
        Some(z0 + m * s + self.height(hyp(s - u, v)))
    }

    /// The least radius along the ray `(x, ρ·s, ρ·c)` (ρ from 0 to `cap`) inside the tool, its
    /// tip at `(x - dx, y, z)` and its axis +Z; `None` when the ray does not enter it below `cap`.
    pub(crate) fn ray_entry(&self, dx: f64, y: f64, z: f64, s: f64, c: f64, cap: f64) -> Option<f64> {
        let r = self.radius;
        if dx.abs() > r {
            return None;
        }
        let w = (r * r - dx * dx).max(0.0).sqrt();
        // Where the ray is within the tool's radius, seen along the axis.
        let (lo, hi) = if s.abs() < 1e-15 {
            if y.abs() > w {
                return None;
            }
            (0.0, cap)
        } else {
            let (a, b) = ((y - w) / s, (y + w) / s);
            (a.min(b).max(0.0), a.max(b).min(cap))
        };
        if !(lo <= hi) {
            return None;
        }
        // The flat bottom (or a plane at height `h` above the tip) and above it.
        let above = |h: f64| -> Option<f64> {
            if lo * c >= z + h {
                Some(lo)
            } else if c > 0.0 {
                let at = (z + h) / c;
                (at <= hi).then_some(at.max(lo))
            } else {
                None
            }
        };
        match self.shape {
            Shape::Flat => above(0.0),
            Shape::Ball => {
                // The sphere about (dx, y, z + r) seen from the ray's start at the axis.
                let b = s * y + c * (z + r);
                let q = dx * dx + y * y + (z + r) * (z + r) - r * r;
                let disc = b * b - q;
                let sphere = (disc >= 0.0)
                    .then(|| {
                        let root = disc.sqrt();
                        let (r1, r2) = (b - root, b + root);
                        let at = r1.max(lo);
                        (at <= r2.min(hi)).then_some(at)
                    })
                    .flatten();
                match (sphere, above(r)) {
                    (Some(a), Some(b)) => Some(a.min(b)),
                    (a, b) => a.or(b),
                }
            }
            Shape::Bull | Shape::Cone(_) | Shape::Taper { .. } => {
                // Inside where g ≥ 0, g concave along the ray.
                let g = |p: f64| p * c - z - self.height(hyp(dx, p * s - y));
                let dg = |p: f64| {
                    let e = p * s - y;
                    let d = hyp(dx, e);
                    if d <= 1e-15 { c } else { c - self.slope(d) * s * e / d }
                };
                if g(lo) >= 0.0 {
                    return Some(lo);
                }
                let top = if dg(lo) <= 0.0 {
                    lo
                } else if dg(hi) >= 0.0 {
                    hi
                } else {
                    let (mut a, mut b) = (lo, hi);
                    for _ in 0..BISECTIONS {
                        let mid = 0.5 * (a + b);
                        if dg(mid) > 0.0 {
                            a = mid;
                        } else {
                            b = mid;
                        }
                    }
                    0.5 * (a + b)
                };
                if g(top) < 0.0 {
                    return None;
                }
                let (mut a, mut b) = (lo, top);
                for _ in 0..BISECTIONS {
                    let mid = 0.5 * (a + b);
                    if g(mid) < 0.0 {
                        a = mid;
                    } else {
                        b = mid;
                    }
                }
                Some(b)
            }
        }
    }
}

/// What one sweep removed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Removed {
    /// Material removed (mm³).
    pub volume: f64,
    /// The deepest it cut into the stock as it was (mm, along Z or the radius), and where (the
    /// bottom of that cut).
    pub depth: f64,
    pub at: Vec3,
    /// The most material that stood above the cutting length where it cut (mm: above the tip
    /// plus the flute length; the shank rubbing), and where.
    pub above: f64,
    pub above_at: Vec3,
    /// The deepest it cut below the part's surface along Z (along the ray) where a part floor
    /// is given ([`SimState::set_floor`]), and where; a gouge only once measured in 3D, along
    /// the column from `gouge_from` (just outside the part's surface) to `gouge_at`
    /// ([`column_depth`]).
    pub gouge: f64,
    pub gouge_at: Vec3,
    pub gouge_from: Vec3,
}

impl Default for Removed {
    fn default() -> Removed {
        Removed {
            volume: 0.0,
            depth: 0.0,
            at: Vec3::ZERO,
            above: 0.0,
            above_at: Vec3::ZERO,
            gouge: 0.0,
            gouge_at: Vec3::ZERO,
            gouge_from: Vec3::ZERO,
        }
    }
}

impl Removed {
    fn add(&mut self, o: Removed) {
        self.volume += o.volume;
        if o.depth > self.depth {
            self.depth = o.depth;
            self.at = o.at;
        }
        if o.above > self.above {
            self.above = o.above;
            self.above_at = o.above_at;
        }
        if o.gouge > self.gouge {
            self.gouge = o.gouge;
            self.gouge_at = o.gouge_at;
            self.gouge_from = o.gouge_from;
        }
    }
}

/// Per tile of [`TILE`] × [`TILE`] cells of a band: an upper bound of the values in it (exact
/// unless stale), whether a cell was cut since it was found, and a version that changes with
/// every cut.
#[derive(Clone, Debug)]
struct Tiles {
    tx: usize,
    max: Vec<f64>,
    stale: Vec<bool>,
    version: Vec<u32>,
}

impl Tiles {
    fn new(nu: usize, nv: usize, value: f64) -> Tiles {
        let n = nu.div_ceil(TILE).max(1) * nv.div_ceil(TILE).max(1);
        Tiles { tx: nu.div_ceil(TILE).max(1), max: vec![value; n], stale: vec![false; n], version: vec![0; n] }
    }

    /// Cell `(i, j)` of the band (`j` from the band's first row) was cut.
    fn mark(&mut self, i: usize, j: usize) {
        let t = (j / TILE) * self.tx + i / TILE;
        if let Some(s) = self.stale.get_mut(t) {
            *s = true;
        }
        if let Some(v) = self.version.get_mut(t) {
            *v = v.wrapping_add(1);
        }
    }

    /// The highest of `values` (`nu` a row) over the cells `i0..=i1`, `j0..=j1`, by whole tiles:
    /// at least the highest of those cells (stale tiles found again first).
    fn max_in(&mut self, values: &[f64], nu: usize, (i0, i1): (usize, usize), (j0, j1): (usize, usize)) -> f64 {
        let mut m = f64::NEG_INFINITY;
        let nv = values.len() / nu.max(1);
        for tj in j0 / TILE..=j1 / TILE {
            for ti in i0 / TILE..=i1 / TILE {
                let t = tj * self.tx + ti;
                if self.stale.get(t) == Some(&true) {
                    let mut best = f64::NEG_INFINITY;
                    for j in tj * TILE..((tj + 1) * TILE).min(nv) {
                        for i in ti * TILE..((ti + 1) * TILE).min(nu) {
                            if let Some(&v) = values.get(j * nu + i) {
                                best = best.max(v);
                            }
                        }
                    }
                    if let (Some(slot), Some(s)) = (self.max.get_mut(t), self.stale.get_mut(t)) {
                        *slot = best;
                        *s = false;
                    }
                }
                m = m.max(self.max.get(t).copied().unwrap_or(f64::NEG_INFINITY));
            }
        }
        m
    }
}

/// What a cut of one cell from `old` down to `new` (along Z or the ray) means beyond the
/// material: the shank above the flutes (`flute` above the cut), and the part's floor `floor`
/// (NaN: none) passed by more than `tol`. `point(v)` is the cell's point at height (radius) `v`.
fn note_cut(out: &mut Removed, old: f64, new: f64, flute: f64, floor: f64, tol: f64, point: impl Fn(f64) -> Vec3) {
    let cut = old - new;
    if cut > out.depth {
        out.depth = cut;
        out.at = point(new);
    }
    let above = old - (new + flute);
    if flute > 0.0 && above > tol && above > out.above {
        out.above = above;
        out.above_at = point(new + flute);
    }
    let below = floor - new;
    if below > tol && below > out.gouge {
        out.gouge = below;
        out.gouge_at = point(new);
        out.gouge_from = point(floor + tol);
    }
}

/// `√(a² + b²)`, without the care (and the cost) of `f64::hypot`.
fn hyp(a: f64, b: f64) -> f64 {
    (a * a + b * b).sqrt()
}

/// `v` as an index from 0 to `n - 1` (rounding down; out of range values clamped).
fn index(v: f64, n: usize) -> usize {
    let top = n.saturating_sub(1) as f64;
    if v.is_nan() { 0 } else { v.floor().clamp(0.0, top) as usize }
}

/// Counts cell updates against [`MAX_VISITS`], over every band.
fn count(visits: &AtomicU64, n: u64) -> Result<()> {
    if visits.fetch_add(n, Ordering::Relaxed).saturating_add(n) > MAX_VISITS {
        return Err(CamError::TooLarge("the program sweeps too much stock to simulate at this resolution: give a coarser resolution".into()));
    }
    Ok(())
}

/// The heightfield stock (3-axis), or one band of its rows: rows `j0..j0 + ny` of the whole
/// grid, which starts at `y0`.
#[derive(Clone)]
struct Heights {
    x0: f64,
    y0: f64,
    dx: f64,
    dy: f64,
    nx: usize,
    j0: usize,
    ny: usize,
    bottom: f64,
    top: f64,
    h: Vec<f64>,
    visits: Arc<AtomicU64>,
    tiles: Tiles,
    /// The tool slot that cut each cell last ([`UNCUT`]: none).
    last: Vec<u8>,
    /// The part's height over every cell of the whole grid (NaN: no part there), when gouges
    /// are watched for as the stock is cut.
    floor: Option<Arc<Vec<f64>>>,
    tol: f64,
    /// The tool cutting now: its slot and flute length.
    slot: u8,
    flute: f64,
}

impl Heights {
    /// The centre of the band's cell `(i, j)`.
    fn centre(&self, i: usize, j: usize) -> (f64, f64) {
        (self.x0 + (i as f64 + 0.5) * self.dx, self.row_y(j))
    }

    fn row_y(&self, j: usize) -> f64 {
        self.y0 + ((self.j0 + j) as f64 + 0.5) * self.dy
    }

    /// Cells whose centres are within `[lo, hi]` along X, as an index range (empty when none).
    fn columns(&self, lo: f64, hi: f64) -> Option<(usize, usize)> {
        let a = ((lo - self.x0) / self.dx - 0.5).ceil();
        let b = ((hi - self.x0) / self.dx - 0.5).floor();
        if !(a <= b) || b < 0.0 || a > (self.nx as f64 - 1.0) {
            return None;
        }
        Some((index(a, self.nx), index(b, self.nx)))
    }

    /// The band's rows whose centres are within `[lo, hi]` along Y.
    fn rows(&self, lo: f64, hi: f64) -> Option<(usize, usize)> {
        let a = ((lo - self.y0) / self.dy - 0.5).ceil() - self.j0 as f64;
        let b = ((hi - self.y0) / self.dy - 0.5).floor() - self.j0 as f64;
        if !(a <= b) || b < 0.0 || a > (self.ny as f64 - 1.0) {
            return None;
        }
        Some((index(a, self.ny), index(b, self.ny)))
    }

    fn count(&self, n: usize) -> Result<()> {
        count(&self.visits, n as u64)
    }

    /// Lowers cell `c` to `z` (not below the stock's bottom).
    fn lower(&mut self, c: usize, z: f64, out: &mut Removed) {
        let area = self.dx * self.dy;
        let bottom = self.bottom;
        let Some(h) = self.h.get_mut(c) else { return };
        let z = z.max(bottom);
        if !(z < *h) {
            return;
        }
        let old = *h;
        out.volume += (old - z) * area;
        *h = z;
        let nx = self.nx.max(1);
        let (i, j) = (c % nx, c / nx);
        self.tiles.mark(i, j);
        if let Some(l) = self.last.get_mut(c) {
            *l = self.slot;
        }
        let floor = self.floor.as_ref().and_then(|f| f.get(self.j0 * self.nx + c).copied()).unwrap_or(f64::NAN);
        let (x, y) = self.centre(i, j);
        note_cut(out, old, z, self.flute, floor, self.tol, |v| Vec3::new(x, y, v));
    }

    /// The highest material over the cells whose centres are within `[x0, x1] × [y0, y1]` and
    /// satisfy `inside`, above `z`: the first such cell's top, or `None`.
    fn above(&mut self, (x0, x1): (f64, f64), (y0, y1): (f64, f64), z: f64, inside: &dyn Fn(f64, f64) -> bool) -> Option<Vec3> {
        if z >= self.top {
            return None;
        }
        let ((i0, i1), (j0, j1)) = (self.columns(x0, x1)?, self.rows(y0, y1)?);
        if self.tiles.max_in(&self.h, self.nx, (i0, i1), (j0, j1)) <= z {
            return None;
        }
        for j in j0..=j1 {
            for i in i0..=i1 {
                let Some(&h) = self.h.get(j * self.nx + i) else { continue };
                let (x, y) = self.centre(i, j);
                if h > z && h > self.bottom && inside(x, y) {
                    return Some(Vec3::new(x, y, h));
                }
            }
        }
        None
    }

    /// The tool swept straight from tip `a` to tip `b`.
    fn sweep_line(&mut self, p: &Profile, a: Vec3, b: Vec3) -> Result<Removed> {
        let mut out = Removed::default();
        let r = p.radius;
        let d = Vec2::new(b.x - a.x, b.y - a.y);
        let len = d.len();
        let dir = if len > 1e-12 { d * (1.0 / len) } else { Vec2::new(1.0, 0.0) };
        let nrm = dir.perp();
        let (z0, dz) = (a.z, b.z - a.z);
        if a.z.min(b.z) >= self.top {
            return Ok(out);
        }
        let Some((j0, j1)) = self.rows(a.y.min(b.y) - r, a.y.max(b.y) + r) else { return Ok(out) };
        for j in j0..=j1 {
            let y = self.row_y(j);
            // The capsule round the move on this row.
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for e in [a, b] {
                let ey = y - e.y;
                if ey.abs() <= r {
                    let w = (r * r - ey * ey).sqrt();
                    lo = lo.min(e.x - w);
                    hi = hi.max(e.x + w);
                }
            }
            if len > 1e-12 && dir.y.abs() > 1e-12 {
                for side in [-r, r] {
                    let s = (y - a.y - side * nrm.y) / dir.y;
                    if (0.0..=len).contains(&s) {
                        let x = a.x + side * nrm.x + s * dir.x;
                        lo = lo.min(x);
                        hi = hi.max(x);
                    }
                }
            }
            let Some((i0, i1)) = self.columns(lo, hi) else { continue };
            self.count(i1 - i0 + 1)?;
            for i in i0..=i1 {
                let c = j * self.nx + i;
                let Some(&h) = self.h.get(c) else { continue };
                let x = self.x0 + (i as f64 + 0.5) * self.dx;
                let rel = Vec2::new(x - a.x, y - a.y);
                let (u, v) = if len > 1e-12 { (rel.dot(dir), rel.cross(dir).abs()) } else { (rel.len(), 0.0) };
                // Nothing to do where the stock is already below anything the tool reaches.
                let outside = (-u).max(u - len).max(0.0);
                if h <= z0.min(z0 + dz) + p.height(hyp(outside, v)) {
                    continue;
                }
                if let Some(z) = p.along_line(u, v, len, z0, dz) {
                    self.lower(c, z, &mut out);
                }
            }
        }
        Ok(out)
    }

    /// The tool swept along an arc in XY about `center` from tip `a` to tip `b`, turning `sweep`.
    /// A helix is followed as chords; `part` (fractions from and to) cuts only the chords of that
    /// part of it, so that cutting it in parts leaves what cutting it whole does.
    fn sweep_arc(&mut self, p: &Profile, a: Vec3, b: Vec3, center: Vec2, sweep: f64, part: (f64, f64)) -> Result<Removed> {
        let tol = self.tol;
        let rho = Vec2::new(a.x, a.y).dist(center);
        let r = p.radius;
        if chorded(a, b, center, sweep) {
            // A helix (or a degenerate arc): chords at most a quarter of the tolerance inside it.
            let e = (tol * 0.25).min(rho);
            let step = if rho > 1e-9 { 2.0 * (1.0 - e / rho).clamp(-1.0, 1.0).acos() } else { TAU };
            let n = (sweep.abs() / step.max(1e-9)).ceil().max(1.0);
            if !(n <= MAX_BLOCK_POINTS as f64) {
                return Err(CamError::TooLarge("an arc needs too many chords to simulate".into()));
            }
            let n = n as usize;
            let start = Vec2::new(a.x - center.x, a.y - center.y).angle();
            let at = |k: usize| {
                let t = k as f64 / n as f64;
                if k == n { b } else { (center + Vec2::from_angle(start + sweep * t) * rho).extend(a.z + (b.z - a.z) * t) }
            };
            let (k0, k1) = (done(n, part.0), done(n, part.1));
            let mut out = Removed::default();
            let mut prev = if k0 == 0 { a } else { at(k0) };
            for k in k0 + 1..=k1 {
                let q = at(k);
                out.add(self.sweep_line(p, prev, q)?);
                prev = q;
            }
            return Ok(out);
        }
        let mut out = Removed::default();
        if a.z >= self.top {
            return Ok(out);
        }
        let start = Vec2::new(a.x - center.x, a.y - center.y).angle();
        // The arc's box: its ends and the quarter points it passes.
        let (mut lo, mut hi) = (Vec2::new(a.x.min(b.x), a.y.min(b.y)), Vec2::new(a.x.max(b.x), a.y.max(b.y)));
        for q in 0..4 {
            let ang = q as f64 * PI * 0.5;
            if within_sweep(ang, start, sweep) {
                let pt = center + Vec2::from_angle(ang) * rho;
                lo = Vec2::new(lo.x.min(pt.x), lo.y.min(pt.y));
                hi = Vec2::new(hi.x.max(pt.x), hi.y.max(pt.y));
            }
        }
        let (Some((i0, i1)), Some((j0, j1))) = (self.columns(lo.x - r, hi.x + r), self.rows(lo.y - r, hi.y + r)) else { return Ok(out) };
        let (pa, pb) = (Vec2::new(a.x, a.y), Vec2::new(b.x, b.y));
        for j in j0..=j1 {
            self.count(i1 - i0 + 1)?;
            for i in i0..=i1 {
                let c = j * self.nx + i;
                let Some(&h) = self.h.get(c) else { continue };
                if h <= a.z {
                    continue;
                }
                let (x, y) = self.centre(i, j);
                let q = Vec2::new(x, y);
                let rel = q - center;
                let rq = rel.len();
                if (rq - rho).abs() > r {
                    continue;
                }
                let d = if rq > 1e-12 && within_sweep(rel.angle(), start, sweep) { (rq - rho).abs() } else { q.dist(pa).min(q.dist(pb)) };
                if d <= r {
                    self.lower(c, a.z + p.height(d), &mut out);
                }
            }
        }
        Ok(out)
    }

    /// Whether `p` is in the stock, when it is over this band (`None` when it is not).
    fn owns(&self, p: Vec3) -> Option<bool> {
        let (fx, fy) = ((p.x - self.x0) / self.dx, (p.y - self.y0) / self.dy - self.j0 as f64);
        if !(fy >= 0.0 && fy < self.ny as f64) {
            return None;
        }
        if !(fx >= 0.0 && fx < self.nx as f64) {
            return Some(false);
        }
        let c = index(fy, self.ny) * self.nx + index(fx, self.nx);
        Some(self.h.get(c).is_some_and(|&h| p.z < h && p.z > self.bottom))
    }
}

/// Whether an arc from `a` to `b` about `center` turning `sweep` is followed as chords (a helix,
/// or a degenerate arc).
fn chorded(a: Vec3, b: Vec3, center: Vec2, sweep: f64) -> bool {
    (b.z - a.z).abs() > 1e-12 || Vec2::new(a.x, a.y).dist(center) <= 1e-9 || sweep.abs() <= 1e-12
}

/// The turn of an arc from `from` to `to` about `center` as the controller runs it (signed,
/// radians): an arc whose end is written at its start is a whole turn, not none.
fn arc_turn(from: Vec3, to: Vec3, center: Vec2, ccw: bool) -> f64 {
    let (a, b) = (Vec2::new(from.x, from.y), Vec2::new(to.x, to.y));
    if a.dist(b) <= 1e-9 && a.dist(center) > 1e-9 {
        return if ccw { TAU } else { -TAU };
    }
    arc_sweep(from, to, center, ccw)
}

/// Whether the angle `ang` is on an arc starting at `start` and turning `sweep` (signed).
fn within_sweep(ang: f64, start: f64, sweep: f64) -> bool {
    if sweep >= 0.0 { (ang - start).rem_euclid(TAU) <= sweep + 1e-12 } else { (start - ang).rem_euclid(TAU) <= -sweep + 1e-12 }
}

/// The radial stock (a 4th axis about X), or one band of its angles: angles `k0..k0 + nt` of
/// the `nt_all` round.
#[derive(Clone)]
struct Radii {
    x0: f64,
    dx: f64,
    nx: usize,
    k0: usize,
    nt: usize,
    nt_all: usize,
    /// The farthest the stock reaches from the axis.
    radius: f64,
    rho: Vec<f64>,
    /// How far the stock reaches along each of the band's angles before it is cut.
    start: Vec<f64>,
    /// sin and cos of the band's angles.
    sin: Vec<f64>,
    cos: Vec<f64>,
    visits: Arc<AtomicU64>,
    tol: f64,
    /// Cells a tool position cut beside their ray, inside the material (not modelled).
    tunnels: u64,
    tiles: Tiles,
    last: Vec<u8>,
    /// The part's radius along every ray of the whole grid (NaN: none).
    floor: Option<Arc<Vec<f64>>>,
    slot: u8,
    flute: f64,
}

impl Radii {
    fn dtheta(&self) -> f64 {
        TAU / self.nt_all as f64
    }

    /// The tool at `pose` cuts every ray it enters.
    fn stamp(&mut self, p: &Profile, pose: &Pose) -> Result<Removed> {
        let mut out = Removed::default();
        let r = p.radius;
        // The frame turned so that the tool axis is +Z: work angle θ is at θ - α in it.
        let alpha = pose.axis.y.atan2(pose.axis.z);
        let (sa, ca) = alpha.sin_cos();
        let t = pose.tip;
        let (x, y, z) = (t.x, t.y * ca - t.z * sa, t.y * sa + t.z * ca);
        if z >= self.radius {
            return Ok(out);
        }
        let i0 = ((x - r - self.x0) / self.dx - 0.5).ceil();
        let i1 = ((x + r - self.x0) / self.dx - 0.5).floor();
        if !(i0 <= i1) || i1 < 0.0 || i0 > self.nx as f64 - 1.0 {
            return Ok(out);
        }
        let (i0, i1) = (index(i0, self.nx), index(i1, self.nx));
        // The angles the tool's box (sideways ±r, from its tip up to the stock's radius) is
        // seen at from the axis.
        let dt = self.dtheta();
        let (k0, k1) = if y - r <= 0.0 && y + r >= 0.0 && z <= 0.0 {
            (0i64, self.nt_all as i64 - 1)
        } else {
            let corners = [(y - r, z), (y + r, z), (y - r, self.radius), (y + r, self.radius)];
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for (cy, cz) in corners {
                let a = cy.atan2(cz);
                lo = lo.min(a);
                hi = hi.max(a);
            }
            let k0 = ((alpha + lo) / dt - 0.5).floor();
            let k1 = ((alpha + hi) / dt - 0.5).ceil();
            if !(k0.is_finite() && k1.is_finite()) || k1 - k0 > 4.0 * self.nt_all as f64 {
                return Ok(out);
            }
            (k0 as i64, (k1 as i64).min(k0 as i64 + self.nt_all as i64 - 1))
        };
        let nt = self.nt_all as i64;
        let area = self.dx * dt * 0.5;
        let band = self.k0..self.k0 + self.nt;
        let mine = (k0..=k1).filter(|k| band.contains(&(k.rem_euclid(nt) as usize))).count();
        count(&self.visits, (mine as u64) * ((i1 - i0 + 1) as u64))?;
        for k in k0..=k1 {
            let Some(kk) = (k.rem_euclid(nt) as usize).checked_sub(self.k0).filter(|kk| *kk < self.nt) else { continue };
            let (Some(&st), Some(&ct)) = (self.sin.get(kk), self.cos.get(kk)) else { continue };
            // sin and cos of θ - α.
            let (s, c) = (st * ca - ct * sa, ct * ca + st * sa);
            if c <= 0.0 && z > 0.0 {
                continue;
            }
            for i in i0..=i1 {
                let cell = kk * self.nx + i;
                let Some(&have) = self.rho.get(cell) else { continue };
                if c > 0.0 && have * c <= z {
                    continue;
                }
                let cx = self.x0 + (i as f64 + 0.5) * self.dx;
                if let Some(at) = p.ray_entry(cx - x, y, z, s, c, have) {
                    let at = at.max(0.0);
                    // The tool must reach out to (within half the tolerance of) the material's
                    // end along the ray: a cut that passes beside the ray inside the material
                    // (an indexed feature in the side of the part) is not one dexel can hold.
                    let outer = (have - 0.5 * self.tol).max(at);
                    let d = hyp(cx - x, outer * s - y);
                    if !(d <= p.radius + 1e-9 && outer * c >= z + p.height(d) - 1e-9) {
                        self.tunnels += 1;
                        continue;
                    }
                    if at < have {
                        out.volume += (have * have - at * at) * area;
                        if let Some(slot) = self.rho.get_mut(cell) {
                            *slot = at;
                        }
                        self.tiles.mark(i, kk);
                        if let Some(l) = self.last.get_mut(cell) {
                            *l = self.slot;
                        }
                        let floor = self.floor.as_ref().and_then(|f| f.get(self.k0 * self.nx + cell).copied()).unwrap_or(f64::NAN);
                        note_cut(&mut out, have, at, self.flute, floor, self.tol, |v| Vec3::new(cx, v * st, v * ct));
                    }
                }
            }
        }
        Ok(out)
    }

    /// Whether `p` is in the stock, when its angle is in this band (`None` when it is not).
    fn owns(&self, p: Vec3) -> Option<bool> {
        let th = p.y.atan2(p.z).rem_euclid(TAU);
        let k = index(th / self.dtheta(), self.nt_all).checked_sub(self.k0).filter(|k| *k < self.nt)?;
        let fx = (p.x - self.x0) / self.dx;
        if !(fx >= 0.0 && fx < self.nx as f64) {
            return Some(false);
        }
        Some(self.rho.get(k * self.nx + index(fx, self.nx)).is_some_and(|&r| p.y.hypot(p.z) < r))
    }
}

#[derive(Clone)]
enum Model {
    Heights(Heights),
    Radii(Radii),
}

impl Model {
    fn owns(&self, p: Vec3) -> Option<bool> {
        match self {
            Model::Heights(h) => h.owns(p),
            Model::Radii(r) => r.owns(p),
        }
    }

    fn set_visits(&mut self, v: &Arc<AtomicU64>) {
        match self {
            Model::Heights(h) => h.visits = Arc::clone(v),
            Model::Radii(r) => r.visits = Arc::clone(v),
        }
    }

    fn set_floor(&mut self, f: &Arc<Vec<f64>>) {
        match self {
            Model::Heights(h) => h.floor = Some(Arc::clone(f)),
            Model::Radii(r) => r.floor = Some(Arc::clone(f)),
        }
    }

    fn set_tool(&mut self, slot: u8, flute: f64) {
        match self {
            Model::Heights(h) => (h.slot, h.flute) = (slot, flute),
            Model::Radii(r) => (r.slot, r.flute) = (slot, flute),
        }
    }

    /// The band's first row (angle) and its rows (angles).
    fn span(&self) -> (usize, usize) {
        match self {
            Model::Heights(h) => (h.j0, h.ny),
            Model::Radii(r) => (r.k0, r.nt),
        }
    }

    fn values(&self) -> &[f64] {
        match self {
            Model::Heights(h) => &h.h,
            Model::Radii(r) => &r.rho,
        }
    }

    fn last(&self) -> &[u8] {
        match self {
            Model::Heights(h) => &h.last,
            Model::Radii(r) => &r.last,
        }
    }

    fn tiles(&self) -> &Tiles {
        match self {
            Model::Heights(h) => &h.tiles,
            Model::Radii(r) => &r.tiles,
        }
    }

    fn bytes(&self) -> usize {
        let t = self.tiles();
        self.values().len() * 9 + t.max.len() * 13
    }
}

/// The cell size for a stock whose grid spans `area` mm².
fn cell_size(area: f64, params: &SimParams) -> Result<f64> {
    match params.resolution {
        Some(r) => {
            if !(1e-4..=1e4).contains(&r) {
                return invalid("the resolution must be from 0.0001 to 10000 mm");
            }
            Ok(r)
        }
        None => Ok((params.tolerance * DEFAULT_CELL_TOLERANCES).max((area / DEFAULT_CELLS as f64).sqrt())),
    }
}

fn cells_for(span: f64, cell: f64) -> Result<usize> {
    let n = (span / cell).ceil().max(1.0);
    if !(n <= MAX_CELLS as f64) {
        return Err(CamError::TooLarge(format!("the stock needs more than {MAX_CELLS} cells at this resolution: give a coarser one")));
    }
    Ok(n as usize)
}

fn too_many(n: usize) -> CamError {
    CamError::TooLarge(format!("the stock needs {n} cells at this resolution, more than {MAX_CELLS}: give a coarser one"))
}

/// The stock's grid before it is split into bands: a heightfield over XY (cell `(i, j)` centred
/// at `(x0 + (i + ½)·dx, y0 + (j + ½)·dy)`, its top `top` to start with), or radii over (x, θ)
/// (cell `(i, k)` at `x0 + (i + ½)·dx`, `θ = (k + ½)·2π / nt`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Grid {
    Heights {
        x0: f64,
        y0: f64,
        dx: f64,
        dy: f64,
        nx: usize,
        ny: usize,
        bottom: f64,
        top: f64,
    },
    /// `radius` is the farthest the stock reaches from the axis; `section`, a box bar's
    /// `[y0, y1, z0, z1]` round the axis (each ray starting where it leaves that rectangle).
    Radii {
        x0: f64,
        dx: f64,
        nx: usize,
        nt: usize,
        radius: f64,
        section: Option<[f64; 4]>,
    },
}

/// How far the ray from the axis along `(0, s, c)` reaches inside the rectangle `[y0, y1, z0,
/// z1]` round the axis.
fn section_reach(sec: [f64; 4], s: f64, c: f64) -> f64 {
    let [y0, y1, z0, z1] = sec;
    let along = |d: f64, lo: f64, hi: f64| {
        if d > 1e-15 {
            hi / d
        } else if d < -1e-15 {
            lo / d
        } else {
            f64::INFINITY
        }
    };
    along(s, y0, y1).min(along(c, z0, z1))
}

impl Grid {
    /// Rows (angles) the grid is split along.
    fn lines(&self) -> usize {
        match *self {
            Grid::Heights { ny, .. } => ny,
            Grid::Radii { nt, .. } => nt,
        }
    }

    /// Cells along a row (along X).
    pub fn columns(&self) -> usize {
        match *self {
            Grid::Heights { nx, .. } | Grid::Radii { nx, .. } => nx,
        }
    }

    /// Rows (angles round).
    pub fn rows(&self) -> usize {
        self.lines()
    }

    /// Tiles along a row and rows of tiles.
    pub fn tiles(&self) -> (usize, usize) {
        (self.columns().div_ceil(TILE).max(1), self.lines().div_ceil(TILE).max(1))
    }

    /// The point at height (radius) `v` over the centre of cell `(i, j)`, in work coordinates.
    pub fn point(&self, i: usize, j: usize, v: f64) -> Vec3 {
        match *self {
            Grid::Heights { x0, y0, dx, dy, .. } => Vec3::new(x0 + (i as f64 + 0.5) * dx, y0 + (j as f64 + 0.5) * dy, v),
            Grid::Radii { x0, dx, nt, .. } => {
                let (s, c) = ((j as f64 + 0.5) * TAU / nt.max(1) as f64).sin_cos();
                Vec3::new(x0 + (i as f64 + 0.5) * dx, v * s, v * c)
            }
        }
    }

    /// The value every cell starts at: the stock's top, or its radius.
    pub fn initial(&self) -> f64 {
        match *self {
            Grid::Heights { top, .. } => top,
            Grid::Radii { radius, .. } => radius,
        }
    }

    /// Rows (angles) `from..to` of the stock as it starts.
    fn band(&self, from: usize, to: usize, visits: &Arc<AtomicU64>, tol: f64) -> Model {
        let n = to.saturating_sub(from);
        match *self {
            Grid::Heights { x0, y0, dx, dy, nx, bottom, top, .. } => Model::Heights(Heights {
                x0,
                y0,
                dx,
                dy,
                nx,
                j0: from,
                ny: n,
                bottom,
                top,
                h: vec![top; nx * n],
                visits: Arc::clone(visits),
                tiles: Tiles::new(nx, n, top),
                last: vec![UNCUT; nx * n],
                floor: None,
                tol,
                slot: 0,
                flute: 0.0,
            }),
            Grid::Radii { x0, dx, nx, nt, radius, section } => {
                let dt = TAU / nt as f64;
                let (sin, cos): (Vec<f64>, Vec<f64>) = (from..to).map(|k| ((k as f64 + 0.5) * dt).sin_cos()).unzip();
                let start: Vec<f64> =
                    sin.iter().zip(&cos).map(|(&s, &c)| section.map_or(radius, |sec| section_reach(sec, s, c).min(radius))).collect();
                let rho = start.iter().flat_map(|&r| std::iter::repeat_n(r, nx)).collect();
                Model::Radii(Radii {
                    x0,
                    dx,
                    nx,
                    k0: from,
                    nt: n,
                    nt_all: nt,
                    radius,
                    rho,
                    start,
                    sin,
                    cos,
                    visits: Arc::clone(visits),
                    tol,
                    tunnels: 0,
                    tiles: Tiles::new(nx, n, radius),
                    last: vec![UNCUT; nx * n],
                    floor: None,
                    slot: 0,
                    flute: 0.0,
                })
            }
        }
    }
}

fn grid_for(stock: &StockShape, kind: Kind, params: &SimParams) -> Result<(Grid, f64)> {
    let finite = |v: f64| v.is_finite() && v.abs() <= MAX_COORD;
    match *stock {
        StockShape::Box { min, max } if kind == Kind::Rotary => {
            if ![min.x, min.y, min.z, max.x, max.y, max.z].into_iter().all(finite) {
                return invalid("the stock's box must be finite numbers within 1000 m");
            }
            if !(max.x > min.x && min.y < 0.0 && max.y > 0.0 && min.z < 0.0 && max.z > 0.0) {
                return invalid("a box stock on a 4th axis must have a length and the A axis inside it");
            }
            let radius = min.y.abs().max(max.y.abs()).hypot(min.z.abs().max(max.z.abs()));
            let (x_min, x_max) = (min.x, max.x);
            let cell = cell_size((x_max - x_min) * TAU * radius, params)?;
            let (nx, nt) = (cells_for(x_max - x_min, cell)?, cells_for(TAU * radius, cell)?.max(8));
            nx.checked_mul(nt).filter(|n| *n <= MAX_CELLS).ok_or_else(|| too_many(nx.saturating_mul(nt)))?;
            let section = Some([min.y, max.y, min.z, max.z]);
            Ok((Grid::Radii { x0: x_min, dx: (x_max - x_min) / nx as f64, nx, nt, radius, section }, cell))
        }
        StockShape::Box { min, max } => {
            if ![min.x, min.y, min.z, max.x, max.y, max.z].into_iter().all(finite) {
                return invalid("the stock's box must be finite numbers within 1000 m");
            }
            if !(max.x > min.x && max.y > min.y && max.z >= min.z) {
                return invalid("the stock's box must have a width and a depth");
            }
            let cell = cell_size((max.x - min.x) * (max.y - min.y), params)?;
            let (nx, ny) = (cells_for(max.x - min.x, cell)?, cells_for(max.y - min.y, cell)?);
            nx.checked_mul(ny).filter(|n| *n <= MAX_CELLS).ok_or_else(|| too_many(nx.saturating_mul(ny)))?;
            let grid = Grid::Heights {
                x0: min.x,
                y0: min.y,
                dx: (max.x - min.x) / nx as f64,
                dy: (max.y - min.y) / ny as f64,
                nx,
                ny,
                bottom: min.z,
                top: max.z,
            };
            Ok((grid, cell))
        }
        StockShape::Cylinder { x_min, x_max, radius } => {
            if ![x_min, x_max, radius].into_iter().all(finite) {
                return invalid("the stock's cylinder must be finite numbers within 1000 m");
            }
            if !(x_max > x_min && radius > 0.0) {
                return invalid("the stock's cylinder must have a length and a radius");
            }
            let cell = cell_size((x_max - x_min) * TAU * radius, params)?;
            let (nx, nt) = (cells_for(x_max - x_min, cell)?, cells_for(TAU * radius, cell)?.max(8));
            nx.checked_mul(nt).filter(|n| *n <= MAX_CELLS).ok_or_else(|| too_many(nx.saturating_mul(nt)))?;
            Ok((Grid::Radii { x0: x_min, dx: (x_max - x_min) / nx as f64, nx, nt, radius, section: None }, cell))
        }
    }
}

/// Where the axes are after a block.
#[derive(Clone)]
struct State {
    linear: Vec3,
    rotary: Vec<f64>,
    tcp: bool,
    line: usize,
}

fn pose_at(machine: &Machine, linear: Vec3, rotary: &[f64], tcp: bool) -> Pose {
    if tcp { Pose { tip: linear, axis: tool_axis(machine, rotary) } } else { forward(machine, &Joints { linear, rotary: rotary.to_vec() }) }
}

/// What the simulation reads of a program beside [`parse5`]'s motion.
#[derive(Default)]
struct Scan {
    /// The tool loaded at each line (`T` with `M6`), in line order.
    changes: Vec<(usize, u32)>,
    /// The lines that lose the position (a return home or a move in machine coordinates).
    gaps: Vec<usize>,
    /// Lines in the XZ or YZ plane (`G18`, `G19`), with their `I J K` words (0 when not
    /// written, in the program's units), in line order.
    planes: Vec<(usize, Plane, [f64; 3])>,
    /// The spindle after each line that changes it (`S`, `M3`, `M4`, `M5`, `M2`, `M30`), in line
    /// order: its speed when it turns, 0 when it is stopped. Before the first `M3`/`M4`/`M5` it is
    /// not known (a program that never says is not taken as cutting with it stopped).
    spindle: Vec<(usize, f64)>,
}

impl Scan {
    fn of(text: &str) -> Result<Scan> {
        let mut scan = Scan::default();
        let mut pending: Option<u32> = None;
        let mut plane = Plane::Xy;
        let (mut rpm, mut on): (f64, Option<bool>) = (0.0, None);
        for (i, line) in text.lines().enumerate() {
            let n = i + 1;
            let ws = words(line, n)?;
            let mut m6 = false;
            let mut ijk = [0.0; 3];
            let was = (rpm, on);
            for &(c, v) in &ws {
                match c {
                    'S' if v.is_finite() && v >= 0.0 => rpm = v,
                    'M' if [3.0, 4.0].iter().any(|m| (v - m).abs() < 1e-9) => on = Some(true),
                    'M' if [5.0, 2.0, 30.0].iter().any(|m| (v - m).abs() < 1e-9) => on = Some(false),
                    'T' => {
                        if !((0.0..=1.0e6).contains(&v) && v.fract() == 0.0) {
                            return Err(CamError::Parse(n, "a tool number must be a whole number".into()));
                        }
                        pending = Some(v as u32);
                    }
                    'M' if (v - 6.0).abs() < 1e-9 => m6 = true,
                    'G' => match (v * 10.0).round() as i64 {
                        280 | 300 | 530 => scan.gaps.push(n),
                        170 => plane = Plane::Xy,
                        180 => plane = Plane::Zx,
                        190 => plane = Plane::Yz,
                        _ => {}
                    },
                    'I' => ijk[0] = v,
                    'J' => ijk[1] = v,
                    'K' => ijk[2] = v,
                    _ => {}
                }
            }
            if plane != Plane::Xy {
                scan.planes.push((n, plane, ijk));
            }
            if (rpm, on) != was
                && let Some(turning) = on
            {
                scan.spindle.push((n, if turning { rpm } else { 0.0 }));
            }
            if m6 {
                let Some(t) = pending else { return Err(CamError::Parse(n, "a tool change (M6) without a tool number (T)".into())) };
                scan.changes.push((n, t));
            }
        }
        Ok(scan)
    }

    /// The spindle's speed at `line` (0: stopped), when the program has said.
    fn rpm_at(&self, line: usize) -> Option<f64> {
        let k = self.spindle.partition_point(|(l, _)| *l <= line).checked_sub(1)?;
        self.spindle.get(k).map(|(_, r)| *r)
    }

    /// The plane and `I J K` of `line`, when it is not XY.
    fn plane_at(&self, line: usize) -> Option<(Plane, [f64; 3])> {
        self.planes.binary_search_by_key(&line, |(n, _, _)| *n).ok().and_then(|k| self.planes.get(k)).map(|(_, p, ijk)| (*p, *ijk))
    }
}

/// The tool in the spindle at `line`: its slot in `tools`.
fn tool_at(changes: &[(usize, u32)], tools: &[(u32, Profile)], line: usize) -> Result<usize> {
    let loaded = changes.partition_point(|(l, _)| *l <= line).checked_sub(1).and_then(|k| changes.get(k)).map(|(_, t)| *t);
    match loaded {
        Some(t) => tools
            .iter()
            .position(|(n, _)| *n == t)
            .ok_or_else(|| CamError::Parse(line, format!("the program uses tool {t}, which is not among the tools given"))),
        None => match tools {
            [_] => Ok(0),
            _ => Err(CamError::Parse(line, "a move before any tool change (T… M6), with more than one tool given".into())),
        },
    }
}

/// How the tool goes through one block.
enum How {
    /// Straight in the work frame from the start pose to the end (down the tool axis from
    /// above, when the position was not known).
    Straight,
    /// Every axis evenly from `from` to `to`.
    Joint { from: Joints, to: Joints, tcp: bool },
    /// Round an arc of the linear axes (rotaries held) about `center`, turning `sweep`;
    /// `work_center` is the centre in work coordinates (3-axis).
    Arc { from: Vec3, to: Vec3, center: Vec2, work_center: Vec2, sweep: f64, rotary: Vec<f64>, tcp: bool },
    /// An arc in the XZ (`G18`) or YZ (`G19`) plane, the rotaries held, as `chords` chords on a
    /// 3-axis machine.
    Tilted { arc: PlaneArc, chords: usize, rotary: Vec<f64>, tcp: bool },
}

/// The plane an arc is in (`G17`, `G18`, `G19`).
#[derive(Clone, Copy, Debug, PartialEq)]
enum Plane {
    Xy,
    Zx,
    Yz,
}

impl Plane {
    /// The plane's first and second axes and its normal (indices into X, Y, Z): turning from
    /// the first towards the second is counter-clockwise (`G3`) seen from the normal's +end.
    fn axes(self) -> (usize, usize, usize) {
        match self {
            Plane::Xy => (0, 1, 2),
            Plane::Zx => (2, 0, 1),
            Plane::Yz => (1, 2, 0),
        }
    }
}

fn comp(v: Vec3, i: usize) -> f64 {
    match i {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

fn with(mut v: Vec3, i: usize, value: f64) -> Vec3 {
    match i {
        0 => v.x = value,
        1 => v.y = value,
        _ => v.z = value,
    }
    v
}

/// An arc of the linear axes in a plane of them, moving evenly along the plane's normal.
#[derive(Clone, Copy, Debug)]
struct PlaneArc {
    from: Vec3,
    to: Vec3,
    center: Vec3,
    plane: Plane,
    radius: f64,
    start: f64,
    sweep: f64,
}

impl PlaneArc {
    /// The arc from `from` to `to` about `center` (only its in-plane coordinates count), as
    /// the controller runs it: an end at the start is a whole turn.
    fn new(from: Vec3, to: Vec3, center: Vec3, plane: Plane, ccw: bool) -> PlaneArc {
        let (u, v, _) = plane.axes();
        let s = Vec2::new(comp(from, u) - comp(center, u), comp(from, v) - comp(center, v));
        let e = Vec2::new(comp(to, u) - comp(center, u), comp(to, v) - comp(center, v));
        let sweep = if s.dist(e) <= 1e-9 {
            if s.len() > 1e-9 { if ccw { TAU } else { -TAU } } else { 0.0 }
        } else {
            let delta = (e.angle() - s.angle()).rem_euclid(TAU);
            let delta = if delta.is_finite() { delta } else { 0.0 };
            if ccw { delta } else { delta - TAU }
        };
        PlaneArc { from, to, center, plane, radius: s.len(), start: s.angle(), sweep }
    }

    /// The point a fraction `t` of the way along.
    fn at(&self, t: f64) -> Vec3 {
        if t >= 1.0 {
            return self.to;
        }
        let (u, v, w) = self.plane.axes();
        let p = Vec2::from_angle(self.start + self.sweep * t) * self.radius;
        let q = with(self.from, u, comp(self.center, u) + p.x);
        let q = with(q, v, comp(self.center, v) + p.y);
        with(q, w, comp(self.from, w) + (comp(self.to, w) - comp(self.from, w)) * t)
    }

    fn length(&self) -> f64 {
        let (_, _, w) = self.plane.axes();
        (self.radius * self.sweep.abs()).hypot(comp(self.to, w) - comp(self.from, w))
    }

    /// Chords at most `e` inside the arc.
    fn chords(&self, e: f64) -> Result<usize> {
        let e = e.min(self.radius);
        let step = if self.radius > 1e-9 { 2.0 * (1.0 - e / self.radius).clamp(-1.0, 1.0).acos() } else { TAU };
        let n = (self.sweep.abs() / step.max(1e-9)).ceil().max(1.0);
        if !(n <= MAX_BLOCK_POINTS as f64) {
            return Err(CamError::TooLarge("an arc needs too many chords to simulate".into()));
        }
        Ok(n as usize)
    }
}

/// One block as the simulation runs it.
struct Step {
    profile: Profile,
    start: Pose,
    end: Pose,
    how: How,
    /// Tool positions along it on a 4th axis.
    points: usize,
    /// Where the material must be for a cut at the tool's full width: under the tool and at
    /// both sides of it, halfway along.
    probes: Option<[Vec3; 3]>,
    line: usize,
    rapid: bool,
    /// The tool's slot among the tools given, and its flute length.
    slot: u8,
    flute: f64,
    /// The spindle's speed (0: stopped), when the program has said.
    rpm: Option<f64>,
    /// The axes at its end, and at its start when known.
    to: Joints,
    from: Option<Joints>,
    /// The distance the tool travels (mm; with the rotaries' sweep on a 4th axis).
    travel: f64,
    /// About how many cells it visits.
    cost: f64,
}

impl Step {
    /// The pose a fraction `t` of the way along, as the 4th axis's tool positions are placed.
    fn pose_frac(&self, machine: &Machine, t: f64) -> Pose {
        match &self.how {
            How::Straight => Pose { tip: self.start.tip.lerp(self.end.tip, t), axis: self.end.axis },
            How::Joint { from, to, tcp } => {
                let j = from.lerp(to, t);
                pose_at(machine, j.linear, &j.rotary, *tcp)
            }
            How::Arc { from, to, center, sweep, rotary, tcp, .. } => pose_at(machine, arc_linear(*from, *to, *center, *sweep, t), rotary, *tcp),
            How::Tilted { arc, rotary, tcp, .. } => pose_at(machine, arc.at(t), rotary, *tcp),
        }
    }

    /// The axes a fraction `t` of the way along (a block from an unknown position: its end's,
    /// shifted with the tip).
    fn joints_frac(&self, machine: &Machine, t: f64) -> Joints {
        match &self.how {
            How::Joint { from, to, .. } => from.lerp(to, t),
            How::Arc { from, to, center, sweep, rotary, .. } => Joints { linear: arc_linear(*from, *to, *center, *sweep, t), rotary: rotary.clone() },
            How::Tilted { arc, rotary, .. } => Joints { linear: arc.at(t), rotary: rotary.clone() },
            How::Straight => Joints { linear: self.to.linear + (self.pose_frac(machine, t).tip - self.end.tip), rotary: self.to.rotary.clone() },
        }
    }

    /// The 4th axis's tool positions: indices `first..=last` of [`Step::pose_frac`] at `k / n`.
    fn pose_range(&self) -> (usize, usize) {
        let n = self.points.max(1);
        match self.how {
            How::Straight => (0, n),
            _ => (1, n),
        }
    }
}

/// The linear axes a fraction `t` round an arc of them from `from` to `to` about `center`.
fn arc_linear(from: Vec3, to: Vec3, center: Vec2, sweep: f64, t: f64) -> Vec3 {
    if t >= 1.0 {
        return to;
    }
    let st = Vec2::new(from.x, from.y) - center;
    (center + Vec2::from_angle(st.angle() + sweep * t) * st.len()).extend(from.z + (to.z - from.z) * t)
}

/// Of `total` pieces of work, those done a fraction `f` of the way.
fn done(total: usize, f: f64) -> usize {
    if f >= 1.0 { total } else { ((f.max(0.0) * total as f64).floor() as usize).min(total) }
}

/// The point a fraction `f` round a level arc (or helix) from `a` to `b` about `center`.
fn arc_point(a: Vec3, b: Vec3, center: Vec2, sweep: f64, f: f64) -> Vec3 {
    if f <= 0.0 {
        return a;
    }
    arc_linear(a, b, center, sweep, f)
}

/// The tool positions of `step` on a 4th axis with indices `ks` (at `k / n`), one at a time (a
/// block may have up to [`MAX_BLOCK_POINTS`], and every band walks them).
fn each_pose(machine: &Machine, step: &Step, ks: std::ops::RangeInclusive<usize>, mut f: impl FnMut(&Pose) -> Result<()>) -> Result<()> {
    let n = step.points.max(1);
    for k in ks {
        f(&step.pose_frac(machine, k as f64 / n as f64))?;
    }
    Ok(())
}

/// Cuts the part of `step` from fraction `f0` to `f1` of the way along through one band.
fn cut(model: &mut Model, step: &Step, machine: &Machine, f0: f64, f1: f64) -> Result<Removed> {
    model.set_tool(step.slot, step.flute);
    let whole = f0 <= 0.0 && f1 >= 1.0;
    Ok(match model {
        Model::Heights(h) => match &step.how {
            How::Arc { work_center, sweep, .. } => {
                let (a, b) = (step.start.tip, step.end.tip);
                if whole || chorded(a, b, *work_center, *sweep) {
                    h.sweep_arc(&step.profile, a, b, *work_center, *sweep, (f0, f1))?
                } else {
                    let (p, q) = (arc_point(a, b, *work_center, *sweep, f0), arc_point(a, b, *work_center, *sweep, f1));
                    h.sweep_arc(&step.profile, p, q, *work_center, *sweep * (f1 - f0), (0.0, 1.0))?
                }
            }
            How::Tilted { arc, chords, rotary, tcp } => {
                let (k0, k1) = (done(*chords, f0), done(*chords, f1));
                let n = (*chords).max(1) as f64;
                let mut out = Removed::default();
                let mut prev = if k0 == 0 { step.start.tip } else { pose_at(machine, arc.at(k0 as f64 / n), rotary, *tcp).tip };
                for k in k0 + 1..=k1 {
                    let tip = pose_at(machine, arc.at(k as f64 / n), rotary, *tcp).tip;
                    out.add(h.sweep_line(&step.profile, prev, tip)?);
                    prev = tip;
                }
                out
            }
            _ => {
                let a = if f0 <= 0.0 { step.start.tip } else { step.start.tip.lerp(step.end.tip, f0) };
                let b = if f1 >= 1.0 { step.end.tip } else { step.start.tip.lerp(step.end.tip, f1) };
                h.sweep_line(&step.profile, a, b)?
            }
        },
        Model::Radii(r) => {
            let (first, last) = step.pose_range();
            let total = last + 1 - first;
            let (d0, d1) = (done(total, f0), done(total, f1));
            let mut out = Removed::default();
            if d1 > d0 {
                each_pose(machine, step, first + d0..=first + d1 - 1, |pose| {
                    out.add(r.stamp(&step.profile, pose)?);
                    Ok(())
                })?;
            }
            out
        }
    })
}

/// A piece of work: step `k` from fraction `f0` to `f1` of the way along.
type Work = (usize, f64, f64);

/// Runs `work` through one band: what each piece removed, and which probes were in the stock
/// before its step started.
fn band_advance(model: &mut Model, plan: &SimPlan, work: &[Work]) -> Result<Vec<(Removed, u8)>> {
    let mut out = Vec::with_capacity(work.len());
    for &(k, f0, f1) in work {
        let Some(step) = plan.steps.get(k) else { continue };
        let mut bits = 0u8;
        if f0 <= 0.0
            && let Some(ps) = &step.probes
        {
            for (b, p) in ps.iter().enumerate() {
                if model.owns(*p) == Some(true) {
                    bits |= 1 << b;
                }
            }
        }
        out.push((cut(model, step, &plan.machine, f0, f1)?, bits));
    }
    Ok(out)
}

/// Bands to split `lines` rows (angles) into: one per core up to [`MAX_THREADS`], each at least
/// [`MIN_BAND`] rows; one on the web, which has no threads.
fn band_count(lines: usize) -> usize {
    #[cfg(target_arch = "wasm32")]
    {
        let _ = lines;
        1
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let cores = std::thread::available_parallelism().map_or(1, |n| n.get());
        cores.min(MAX_THREADS).min(lines / MIN_BAND).max(1)
    }
}

/// `f` run for every item, each on a thread of its own when `threaded` and one can be had (else
/// here), the results in order.
fn each_threaded<T: Send, R: Send>(items: Vec<T>, threaded: bool, f: impl Fn(T) -> Result<R> + Sync) -> Result<Vec<R>> {
    if !threaded || items.len() <= 1 {
        return items.into_iter().map(&f).collect();
    }
    let f = &f;
    let lost = || CamError::Unsupported("a simulation thread lost its work".into());
    std::thread::scope(|scope| {
        let mut pending = Vec::with_capacity(items.len());
        for (b, item) in items.into_iter().enumerate() {
            // The item goes through a slot the thread takes it from, so that it is still here
            // when no thread can be had.
            let slot = Arc::new(Mutex::new(Some(item)));
            let mine = Arc::clone(&slot);
            let job = move || match mine.lock().ok().and_then(|mut s| s.take()) {
                Some(item) => f(item),
                None => Err(lost()),
            };
            match std::thread::Builder::new().name(format!("cam-sim-{b}")).spawn_scoped(scope, job) {
                Ok(h) => pending.push(Err(h)),
                Err(_) => pending.push(Ok(match slot.lock().ok().and_then(|mut s| s.take()) {
                    Some(item) => f(item),
                    None => Err(lost()),
                })),
            }
        }
        pending
            .into_iter()
            .map(|p| match p {
                Ok(done) => done,
                Err(h) => h.join().unwrap_or_else(|_| Err(CamError::Unsupported("a simulation thread stopped".into()))),
            })
            .collect()
    })
}

/// A program read for the simulation: its steps (one per motion block whose position is known),
/// the grid of the stock and what times it. See [`plan_sim`].
pub struct SimPlan {
    machine: Machine,
    steps: Vec<Step>,
    grid: Grid,
    cell: f64,
    tol: f64,
    initial: f64,
    minutes: f64,
    feed_length: f64,
    rapid_length: f64,
    /// The tools by slot: number.
    tools: Vec<u32>,
    /// The rows (angles) of each band, from and to (multiples of [`TILE`], the last excepted).
    bands: Vec<(usize, usize)>,
}

/// One step of a [`SimPlan`], as a view shows it.
#[derive(Clone, Debug, PartialEq)]
pub struct StepView {
    /// The G-code line, from 1.
    pub line: usize,
    pub rapid: bool,
    /// An arc (in any plane).
    pub arc: bool,
    pub tool: u32,
    pub slot: u8,
    /// The spindle's speed (0: stopped), when the program has said.
    pub rpm: Option<f64>,
    pub start: Pose,
    pub end: Pose,
    /// Whether the position before it was known (otherwise it comes down the tool axis from
    /// above the stock).
    pub known: bool,
    /// The distance it travels, mm.
    pub travel: f64,
}

impl SimPlan {
    /// Steps: motion blocks simulated.
    pub fn moves(&self) -> usize {
        self.steps.len()
    }

    pub fn step(&self, k: usize) -> Option<StepView> {
        let s = self.steps.get(k)?;
        Some(StepView {
            line: s.line,
            rapid: s.rapid,
            arc: matches!(s.how, How::Arc { .. } | How::Tilted { .. }),
            tool: self.tools.get(usize::from(s.slot)).copied().unwrap_or(0),
            slot: s.slot,
            rpm: s.rpm,
            start: s.start,
            end: s.end,
            known: s.from.is_some(),
            travel: s.travel,
        })
    }

    /// The tool's pose a fraction `f` (0 to 1) along step `k`.
    pub fn pose(&self, k: usize, f: f64) -> Option<Pose> {
        let f = if f.is_finite() { f.clamp(0.0, 1.0) } else { 0.0 };
        self.steps.get(k).map(|s| s.pose_frac(&self.machine, f))
    }

    /// The axes a fraction `f` along step `k`.
    pub fn joints(&self, k: usize, f: f64) -> Option<Joints> {
        let f = if f.is_finite() { f.clamp(0.0, 1.0) } else { 0.0 };
        self.steps.get(k).map(|s| s.joints_frac(&self.machine, f))
    }

    /// Poses along step `k` at most `spacing` mm of its travel apart (its start and end
    /// included; at most `max` + 1; a straight move of a 3-axis machine is its two ends).
    pub fn samples(&self, k: usize, spacing: f64, max: usize) -> Vec<Pose> {
        let Some(s) = self.steps.get(k) else { return Vec::new() };
        let spacing = if spacing.is_finite() && spacing > 1e-6 { spacing } else { 1.0 };
        let straight = match &s.how {
            How::Straight => true,
            How::Joint { from, to, .. } => from.rotary == to.rotary,
            _ => false,
        };
        let n = if straight { 1 } else { ((s.travel / spacing).ceil() as usize).clamp(1, max.max(1)) };
        (0..=n).map(|i| s.pose_frac(&self.machine, i as f64 / n as f64)).collect()
    }

    pub fn grid(&self) -> &Grid {
        &self.grid
    }

    /// The cell size (mm).
    pub fn cell(&self) -> f64 {
        self.cell
    }

    pub fn cells(&self) -> usize {
        self.grid.columns().saturating_mul(self.grid.rows())
    }

    pub fn tolerance(&self) -> f64 {
        self.tol
    }

    pub fn machine(&self) -> &Machine {
        &self.machine
    }

    /// The number of the tool in `slot`.
    pub fn tool_number(&self, slot: u8) -> Option<u32> {
        self.tools.get(usize::from(slot)).copied()
    }

    /// Time at feed as the controller times it, dwells included (minutes).
    pub fn minutes(&self) -> f64 {
        self.minutes
    }

    /// About how many cells cutting `work` visits.
    fn cost(&self, work: &[Work]) -> f64 {
        work.iter().map(|&(k, f0, f1)| self.steps.get(k).map_or(0.0, |s| s.cost * (f1 - f0).max(0.0))).sum()
    }

    /// About how many cells cutting step `k` visits.
    pub fn step_cost(&self, k: usize) -> f64 {
        self.steps.get(k).map_or(0.0, |s| s.cost)
    }

    /// The part's height over every cell (its radius along every ray on a 4th axis) where a
    /// ray from above (from outside) first meets it; NaN where none does. For gouges found as
    /// the stock is cut ([`SimState::set_floor`]).
    pub fn part_floor(&self, part: &Surface) -> Vec<f64> {
        let (nu, nv) = (self.grid.columns(), self.grid.rows());
        let grid = self.grid;
        let threaded = !part.is_empty() && nu.saturating_mul(nv) > 100_000;
        let rows = each_threaded(self.bands.clone(), threaded, |(from, to)| {
            let mut out = Vec::with_capacity(nu * to.saturating_sub(from));
            for j in from..to {
                for i in 0..nu {
                    out.push(if part.is_empty() { f64::NAN } else { floor_at(&grid, part, i, j) });
                }
            }
            Ok(out)
        });
        let mut all = Vec::with_capacity(nu * nv);
        for r in rows.unwrap_or_default() {
            all.extend(r);
        }
        all.resize(nu * nv, f64::NAN);
        all
    }
}

/// The part's height (radius) over cell `(i, j)` of `grid`, NaN when the ray misses it.
fn floor_at(grid: &Grid, part: &Surface, i: usize, j: usize) -> f64 {
    match *grid {
        Grid::Heights { top, bottom, .. } => {
            let span = (top - bottom).max(0.0) + 2.0;
            let p = grid.point(i, j, top + 1.0);
            part.raycast(p, Vec3::new(0.0, 0.0, -span)).map_or(f64::NAN, |(t, _)| top + 1.0 - t * span)
        }
        Grid::Radii { radius, .. } => {
            let out = radius + 1.0;
            let p = grid.point(i, j, out);
            part.raycast(p, Vec3::new(p.x, 0.0, 0.0) - p).map_or(f64::NAN, |(t, _)| out * (1.0 - t))
        }
    }
}

/// An outline seen along the tool axis (from above on a 3-axis machine), work coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Footprint {
    Circle { x: f64, y: f64, r: f64 },
    Rect { x0: f64, x1: f64, y0: f64, y1: f64 },
}

impl Footprint {
    pub fn bounds(self) -> (f64, f64, f64, f64) {
        match self {
            Footprint::Circle { x, y, r } => (x - r, x + r, y - r, y + r),
            Footprint::Rect { x0, x1, y0, y1 } => (x0, x1, y0, y1),
        }
    }

    pub fn contains(self, px: f64, py: f64) -> bool {
        match self {
            Footprint::Circle { x, y, r } => (px - x).powi(2) + (py - y).powi(2) <= r * r,
            Footprint::Rect { x0, x1, y0, y1 } => px >= x0 && px <= x1 && py >= y0 && py <= y1,
        }
    }
}

/// The stock part way through a program: every band of it, and what each step removed so far.
/// Cloning it (a keyframe) copies the stock.
pub struct SimState {
    bands: Vec<Model>,
    /// What each step removed, every band's share added in band order.
    removed: Vec<Removed>,
    probes: Vec<u8>,
    /// Steps before `next` are done, and `frac` of step `next`.
    next: usize,
    frac: f64,
    visits: Arc<AtomicU64>,
    failed: bool,
}

impl Clone for SimState {
    fn clone(&self) -> SimState {
        let visits = Arc::new(AtomicU64::new(0));
        let mut bands = self.bands.clone();
        for b in &mut bands {
            b.set_visits(&visits);
        }
        SimState { bands, removed: self.removed.clone(), probes: self.probes.clone(), next: self.next, frac: self.frac, visits, failed: self.failed }
    }
}

impl SimState {
    /// The stock before the program.
    pub fn new(plan: &SimPlan) -> SimState {
        let visits = Arc::new(AtomicU64::new(0));
        let bands = plan.bands.iter().map(|&(from, to)| plan.grid.band(from, to, &visits, plan.tol)).collect();
        SimState { bands, removed: Vec::new(), probes: Vec::new(), next: 0, frac: 0.0, visits, failed: false }
    }

    /// Watches for cuts below `floor` (from [`SimPlan::part_floor`]) as the stock is cut: each
    /// step's [`Removed::gouge`].
    pub fn set_floor(&mut self, floor: &Arc<Vec<f64>>) {
        for b in &mut self.bands {
            b.set_floor(floor);
        }
    }

    /// How far it is: every step before the first number is cut, and the second's fraction of
    /// that step.
    pub fn at(&self) -> (usize, f64) {
        (self.next, self.frac)
    }

    /// What each step cut so far, in step order (the step under way included).
    pub fn removed(&self) -> &[Removed] {
        &self.removed
    }

    /// Cuts on to step `to`, and a fraction `fraction` of it (steps past the program's end are
    /// its end). A position before the state's own is refused: start again from an earlier state.
    pub fn advance(&mut self, plan: &SimPlan, to: usize, fraction: f64) -> Result<()> {
        if self.failed {
            return invalid("this simulation state stopped part way through a step: start again from an earlier one");
        }
        let n = plan.steps.len();
        let (mut to, mut fraction) = (to, if fraction.is_finite() { fraction.clamp(0.0, 1.0) } else { 0.0 });
        if fraction >= 1.0 {
            to = to.saturating_add(1);
            fraction = 0.0;
        }
        if to >= n {
            to = n;
            fraction = 0.0;
        }
        if to < self.next || (to == self.next && fraction < self.frac) {
            return invalid("the stock cannot be cut back to an earlier move: start again from an earlier state");
        }
        let mut work: Vec<Work> = Vec::new();
        let (mut k, mut f0) = (self.next, self.frac);
        while k < to {
            work.push((k, f0, 1.0));
            k += 1;
            f0 = 0.0;
        }
        if k < n && fraction > f0 {
            work.push((k, f0, fraction));
        }
        if work.is_empty() {
            return Ok(());
        }
        self.visits.store(0, Ordering::Relaxed);
        let threaded = plan.cost(&work) > THREADED_VISITS;
        let bands: Vec<&mut Model> = self.bands.iter_mut().collect();
        let results = match each_threaded(bands, threaded, |m| band_advance(m, plan, &work)) {
            Ok(r) => r,
            Err(e) => {
                self.failed = true;
                return Err(e);
            }
        };
        for (w, &(k, _, _)) in work.iter().enumerate() {
            while self.removed.len() <= k {
                self.removed.push(Removed::default());
                self.probes.push(0);
            }
            for band in &results {
                if let Some(&(r, bits)) = band.get(w) {
                    if let Some(slot) = self.removed.get_mut(k) {
                        slot.add(r);
                    }
                    if let Some(p) = self.probes.get_mut(k) {
                        *p |= bits;
                    }
                }
            }
        }
        (self.next, self.frac) = (to, fraction);
        Ok(())
    }

    /// The stock as it is now.
    pub fn grid(&self, plan: &SimPlan) -> StockGrid {
        let mut values = Vec::with_capacity(plan.cells());
        for b in &self.bands {
            values.extend_from_slice(b.values());
        }
        stock_grid(&plan.grid, values)
    }

    /// The band holding row (angle) `j`, and the row within it.
    fn band_of(&self, j: usize) -> Option<(&Model, usize)> {
        let b = self.bands.partition_point(|m| m.span().0 <= j).checked_sub(1)?;
        let m = self.bands.get(b)?;
        let (j0, n) = m.span();
        (j < j0 + n).then_some((m, j - j0))
    }

    /// The height (radius) of cell `(i, j)` of the whole grid.
    pub fn value(&self, plan: &SimPlan, i: usize, j: usize) -> Option<f64> {
        let nx = plan.grid.columns();
        if i >= nx {
            return None;
        }
        let (m, jj) = self.band_of(j)?;
        m.values().get(jj * nx + i).copied()
    }

    /// The slot of the tool that cut cell `(i, j)` last ([`UNCUT`]: none).
    pub fn last_tool(&self, plan: &SimPlan, i: usize, j: usize) -> u8 {
        let nx = plan.grid.columns();
        if i >= nx {
            return UNCUT;
        }
        self.band_of(j).and_then(|(m, jj)| m.last().get(jj * nx + i).copied()).unwrap_or(UNCUT)
    }

    /// The version of tile `(ti, tj)`: it changes whenever a cell of the tile is cut.
    pub fn tile_version(&self, ti: usize, tj: usize) -> u32 {
        let Some((m, jj)) = self.band_of(tj * TILE) else { return 0 };
        let t = m.tiles();
        if ti >= t.tx {
            return 0;
        }
        t.version.get((jj / TILE) * t.tx + ti).copied().unwrap_or(0)
    }

    /// Every tile's version, row of tiles after row ([`Grid::tiles`]).
    pub fn tile_versions(&self) -> Vec<u32> {
        let mut out = Vec::new();
        for b in &self.bands {
            out.extend_from_slice(&b.tiles().version);
        }
        out
    }

    /// Material higher than `z` inside `fp` (3-axis stock; `None` on a 4th axis): a point on its
    /// top. Exact over the cells' centres; tiles whose highest cell is below `z` are skipped.
    pub fn material_above(&mut self, fp: Footprint, z: f64) -> Option<Vec3> {
        let (x0, x1, y0, y1) = fp.bounds();
        if ![x0, x1, y0, y1, z].iter().all(|v| v.is_finite()) {
            return None;
        }
        let inside = |x: f64, y: f64| fp.contains(x, y);
        for b in &mut self.bands {
            if let Model::Heights(h) = b
                && let Some(p) = h.above((x0, x1), (y0, y1), z, &inside)
            {
                return Some(p);
            }
        }
        None
    }

    /// The highest of the cells `i0..=i1` by `j0..=j1` by whole tiles (at least their highest).
    pub fn max_in(&mut self, plan: &SimPlan, (i0, i1): (usize, usize), (j0, j1): (usize, usize)) -> f64 {
        let nx = plan.grid.columns();
        let mut m = f64::NEG_INFINITY;
        for b in &mut self.bands {
            let (b0, n) = b.span();
            let (lo, hi) = (j0.max(b0), j1.min(b0 + n.saturating_sub(1)));
            if n == 0 || lo > hi || i0 > i1 || i0 >= nx {
                continue;
            }
            let (values, tiles) = match b {
                Model::Heights(h) => (&h.h, &mut h.tiles),
                Model::Radii(r) => (&r.rho, &mut r.tiles),
            };
            m = m.max(tiles.max_in(values, nx, (i0, i1.min(nx - 1)), (lo - b0, hi - b0)));
        }
        m
    }

    /// Whether `p` (work coordinates) is inside the stock.
    pub fn contains(&self, p: Vec3) -> bool {
        self.bands.iter().find_map(|b| b.owns(p)).unwrap_or(false)
    }

    /// The memory the stock takes, about (bytes).
    pub fn bytes(&self) -> usize {
        self.bands.iter().map(Model::bytes).sum::<usize>() + self.removed.len() * (std::mem::size_of::<Removed>() + 1)
    }

    /// What is left held against `part`, and the report [`simulate`] gives.
    pub fn finish(self, plan: &SimPlan, part: &Surface) -> SimReport {
        let (tol, initial) = (plan.tol, plan.initial);
        let refine = (MAX_REFINED / self.bands.len().max(1)).max(1);
        let threaded = self.bands.len() > 1 && plan.cells() > 100_000;
        let models: Vec<&Model> = self.bands.iter().collect();
        let cmps = each_threaded(models, threaded, |m| Ok(compare(m, part, tol, initial, refine))).unwrap_or_default();
        let mut removals = Vec::new();
        for (k, step) in plan.steps.iter().enumerate() {
            let r = self.removed.get(k).copied().unwrap_or_default();
            let slot = self.probes.get(k).copied().unwrap_or(0) == 0b111;
            if r.volume > 0.0 || slot {
                removals.push(BlockRemoval { line: step.line, rapid: step.rapid, volume: r.volume, depth: r.depth, slot });
            }
        }
        let rapids_in_material: Vec<BlockRemoval> = removals.iter().filter(|r| r.rapid && r.depth > tol).copied().collect();
        let removed_volume = removals.iter().map(|r| r.volume).sum();
        let mut warnings = Vec::new();
        if plan.steps.is_empty() {
            warnings.push(
                "no move of the program was simulated: none comes after every axis of the machine (its rotary included) is written, so \
                 nothing was checked"
                    .into(),
            );
        }
        let slots: Vec<usize> = removals.iter().filter(|r| r.slot).map(|r| r.line).collect();
        if !slots.is_empty() {
            let shown: Vec<String> = slots.iter().take(10).map(usize::to_string).collect();
            warnings.push(format!(
                "{} cutting moves take the tool's full width (a slot), lines {}{}: on a light machine keep their depth and feed low",
                slots.len(),
                shown.join(", "),
                if slots.len() > shown.len() { ", …" } else { "" }
            ));
        }
        let mut cmp = Comparison::default();
        for c in &cmps {
            cmp.add(c);
        }
        if cmp.unrefined > 0 {
            warnings.push(format!("{} gouged cells were measured along Z (or the radius) only: too many to measure in 3D", cmp.unrefined));
        }
        let tunnels: u64 = self.bands.iter().map(|b| if let Model::Radii(m) = b { m.tunnels } else { 0 }).sum();
        if tunnels > 0 {
            warnings.push(format!(
                "{tunnels} times the tool cut beside a ray from the A axis without reaching out of the material along it (a feature in \
                 the side of the part): the round stock model leaves those cuts out, so they are not checked here (the job's gouge check \
                 covers them)"
            ));
        }
        let mut values = Vec::with_capacity(plan.cells());
        for b in self.bands {
            match b {
                Model::Heights(h) => values.extend(h.h),
                Model::Radii(r) => values.extend(r.rho),
            }
        }
        let stock = stock_grid(&plan.grid, values);
        SimReport {
            cell: plan.cell,
            cells: match &stock {
                StockGrid::Heights { heights, .. } => heights.len(),
                StockGrid::Radii { radii, .. } => radii.len(),
            },
            stock,
            moves: plan.steps.len(),
            removals,
            rapids_in_material,
            removed_volume,
            gouge_worst: cmp.worst,
            gouge_cells: cmp.gouged,
            leftover_max: cmp.leftover_max,
            leftover_mean: if cmp.machined > 0 { cmp.leftover_sum / cmp.machined as f64 } else { 0.0 },
            machined_cells: cmp.machined,
            minutes: plan.minutes,
            feed_length: plan.feed_length,
            rapid_length: plan.rapid_length,
            warnings,
        }
    }
}

fn stock_grid(grid: &Grid, values: Vec<f64>) -> StockGrid {
    match *grid {
        Grid::Heights { x0, y0, dx, dy, nx, ny, bottom, top } => StockGrid::Heights { x0, y0, dx, dy, nx, ny, bottom, top, heights: values },
        Grid::Radii { x0, dx, nx, nt, radius, .. } => StockGrid::Radii { x0, dx, nx, nt, radius, radii: values },
    }
}

/// Simulates `text` (G-code for `machine`, in absolute coordinates) cutting `stock` with
/// `tools` (by number), and compares what is left with `part`. See the module documentation.
///
/// Without a resolution in `params`, a program whose cuts would visit more cells at the default
/// cell than the simulation allows (a long 4th-axis program) is simulated at a cell coarse
/// enough to fit, with a warning saying so; the cell used is in the report.
pub fn simulate(text: &str, machine: &Machine, tools: &[Tool], stock: &StockShape, part: &Surface, params: &SimParams) -> Result<SimReport> {
    let mut plan = plan_sim(text, machine, tools, stock, params)?;
    let mut coarser = None;
    if params.resolution.is_none() {
        let visits: f64 = (0..plan.moves()).map(|k| plan.step_cost(k)).sum();
        let budget = FIT_SHARE * MAX_VISITS as f64;
        if visits > budget {
            // The cells a tool position visits go as the square of the cell's size.
            let cell = plan.cell() * (visits / budget).sqrt();
            plan = plan_sim(text, machine, tools, stock, &SimParams { resolution: Some(cell), ..*params })?;
            coarser = Some(visits);
        }
    }
    let mut state = SimState::new(&plan);
    state.advance(&plan, plan.moves(), 0.0)?;
    let mut report = state.finish(&plan, part);
    if let Some(visits) = coarser {
        report.warnings.push(format!(
            "the program would visit about {visits:.3e} cells at the default resolution, more than the simulation allows: it was simulated \
             at a cell of {:.3} mm instead (give `resolution` to choose one)",
            plan.cell()
        ));
    }
    Ok(report)
}

/// Reads `text` (G-code for `machine`, in absolute coordinates) into the steps that cut `stock`
/// with `tools` (by number): the first part of [`simulate`].
pub fn plan_sim(text: &str, machine: &Machine, tools: &[Tool], stock: &StockShape, params: &SimParams) -> Result<SimPlan> {
    let tol = params.tolerance;
    if !(1e-4..=1.0).contains(&tol) {
        return invalid("the tolerance must be from 0.0001 to 1 mm");
    }
    let kind = Kind::of(machine)?;
    match (kind, stock) {
        (Kind::Three, StockShape::Box { .. }) | (Kind::Rotary, StockShape::Cylinder { .. } | StockShape::Box { .. }) => {}
        (Kind::Three, _) => return invalid("a 3-axis machine's stock is a box"),
    }
    if tools.is_empty() {
        return invalid("the simulation needs the program's tools");
    }
    let mut profiles: Vec<(u32, Profile)> = Vec::with_capacity(tools.len());
    for t in tools {
        if profiles.iter().any(|(n, _)| *n == t.number) {
            return invalid(format!("two tools are number {}", t.number));
        }
        profiles.push((t.number, Profile::of(t)?));
    }
    let blocks = parse5(text, &machine.letters())?;
    let scan = Scan::of(text)?;
    let (grid, cell) = grid_for(stock, kind, params)?;
    let (reach, initial) = match (*stock, grid) {
        (StockShape::Box { min, max }, Grid::Radii { radius, .. }) => ((max.x - min.x).hypot(2.0 * radius), radius),
        (StockShape::Box { min, max }, _) => ((max - min).len(), max.z),
        (StockShape::Cylinder { x_min, x_max, radius }, _) => ((x_max - x_min).hypot(2.0 * radius), radius),
    };
    let lever_extra = machine.work_origin.len() + machine.rotaries.iter().map(|r| r.center.len()).sum::<f64>();
    let area = match grid {
        Grid::Heights { dx, dy, .. } => dx * dy,
        Grid::Radii { dx, nt, radius, .. } => dx * (TAU * radius / nt.max(1) as f64),
    };

    // The blocks as steps, in order.
    let mut steps: Vec<Step> = Vec::with_capacity(blocks.len());
    let (mut minutes, mut feed_length, mut rapid_length) = (0.0, 0.0, 0.0);
    let mut stamps = 0usize;
    let mut state: Option<State> = None;
    for b in &blocks {
        let (line, linear, rotary, tcp, rapid) = match b {
            JointWords::Rapid { line, linear, rotary, tcp } => (*line, *linear, rotary.clone(), *tcp, true),
            JointWords::Feed { line, linear, rotary, tcp, .. } | JointWords::Arc { line, linear, rotary, tcp, .. } => {
                (*line, *linear, rotary.clone(), *tcp, false)
            }
            JointWords::Dwell { seconds, .. } => {
                minutes += seconds / 60.0;
                continue;
            }
        };
        let slot = tool_at(&scan.changes, &profiles, line)?;
        let Some(&(_, profile)) = profiles.get(slot) else { return Err(CamError::Parse(line, "a tool that is not among the tools given".into())) };
        let flute = tools.get(slot).map_or(0.0, |t| t.flute_length);
        // The position before this block, unless the program lost it since the last one.
        let gaps = &scan.gaps;
        let lost = |since: usize| gaps.partition_point(|g| *g <= since) < gaps.partition_point(|g| *g <= line);
        let from = state.as_ref().filter(|s| s.tcp == tcp && !lost(s.line)).cloned();
        let to = State { linear, rotary, tcp, line };
        let end = pose_at(machine, to.linear, &to.rotary, tcp);
        if !(end.tip.is_finite() && end.axis.is_finite()) {
            return Err(CamError::Parse(line, "a position that is not a finite number".into()));
        }
        // The first pose: the block's start, or straight down the tool axis from above the stock.
        let start = match &from {
            Some(f) => pose_at(machine, f.linear, &f.rotary, tcp),
            None => Pose { tip: end.tip + end.axis * (end.tip.len() + reach + 1.0), axis: end.axis },
        };
        let length = start.tip.dist(end.tip);
        match b {
            JointWords::Rapid { .. } => rapid_length += length,
            JointWords::Feed { f, inverse_time, mm_per_unit, .. } => {
                let lin = from.as_ref().map_or(0.0, |s| s.linear.dist(to.linear));
                let turn = from.as_ref().map_or(0.0, |s| s.rotary.iter().zip(&to.rotary).map(|(a, b)| (b - a).abs()).fold(0.0, f64::max));
                minutes += if *inverse_time {
                    1.0 / f
                } else if tcp || lin > 1e-9 {
                    lin / (f * mm_per_unit)
                } else {
                    turn.to_degrees() / f
                };
                feed_length += length;
            }
            JointWords::Arc { .. } => feed_length += length,
            JointWords::Dwell { .. } => {}
        }
        let (how, travel) = match (b, &from, scan.plane_at(line)) {
            // An arc in the XZ or YZ plane: the reader takes `I J` as in XY, so its centre and
            // turn are found again from the block's own words.
            (JointWords::Arc { ccw, f, mm_per_unit, .. }, Some(f0), Some((plane, ijk))) => {
                let center = f0.linear + Vec3::new(ijk[0], ijk[1], ijk[2]) * *mm_per_unit;
                let arc = PlaneArc::new(f0.linear, to.linear, center, plane, *ccw);
                let travel = arc.length();
                minutes += travel / (f * mm_per_unit);
                let chords = if kind == Kind::Three { arc.chords(tol * 0.25)? } else { 0 };
                (How::Tilted { arc, chords, rotary: to.rotary.clone(), tcp }, travel)
            }
            (JointWords::Arc { center, ccw, f, mm_per_unit, .. }, Some(f0), None) => {
                let st = Vec2::new(f0.linear.x, f0.linear.y) - *center;
                let sweep = arc_turn(f0.linear, to.linear, *center, *ccw);
                let travel = (st.len() * sweep).hypot(to.linear.z - f0.linear.z);
                minutes += travel / (f * mm_per_unit);
                let offset = end.tip - to.linear;
                let how = How::Arc {
                    from: f0.linear,
                    to: to.linear,
                    center: *center,
                    work_center: *center + Vec2::new(offset.x, offset.y),
                    sweep,
                    rotary: to.rotary.clone(),
                    tcp,
                };
                (how, travel)
            }
            (JointWords::Arc { .. }, None, _) => return Err(CamError::Parse(line, "an arc right after the position was lost".into())),
            (_, Some(f0), _) => {
                let (a, c) = (Joints { linear: f0.linear, rotary: f0.rotary.clone() }, Joints { linear: to.linear, rotary: to.rotary.clone() });
                let lin = a.linear.dist(c.linear);
                let lever = start.tip.len().max(end.tip.len()) + lin + lever_extra + profile.radius;
                let travel = length.max(lin) + a.rotary_change(&c) * lever;
                (How::Joint { from: a, to: c, tcp }, travel)
            }
            (_, None, _) => (How::Straight, length),
        };
        // Tool positions along the block: only the 4th axis's model places the tool at points
        // (a heightfield sweeps each block exactly), so only it is limited by them.
        let points = if kind == Kind::Rotary {
            let n = points(travel / profile.step(tol))?;
            stamps = stamps.saturating_add(n);
            if stamps > MAX_STAMPS {
                return Err(CamError::TooLarge("the program needs too many tool positions to simulate: give a larger tolerance".into()));
            }
            n
        } else {
            0
        };
        // Where the material must be for a cut at the tool's full width.
        let probes = (!rapid && matches!(how, How::Joint { .. })).then(|| {
            let mid = Pose { tip: start.tip.lerp(end.tip, 0.5), axis: (start.axis + end.axis).normalized().unwrap_or(end.axis) };
            let along = end.tip - start.tip;
            let along = along - mid.axis * along.dot(mid.axis);
            along.normalized().map(|t| {
                let side = mid.axis.cross(t);
                let up = tol.max(0.05 * profile.radius);
                let w = 0.9 * profile.radius;
                let rim = mid.tip + mid.axis * (profile.height(w) + up);
                [mid.tip + mid.axis * up, rim + side * w, rim - side * w]
            })
        });
        let d = 2.0 * profile.radius;
        let cost = if kind == Kind::Rotary { points as f64 * (d * d / area + 1.0) } else { (travel.min(reach * 4.0) + d) * d / area + 1.0 };
        let from_joints = from.as_ref().map(|f| Joints { linear: f.linear, rotary: f.rotary.clone() });
        let to_joints = Joints { linear: to.linear, rotary: to.rotary.clone() };
        steps.push(Step {
            profile,
            start,
            end,
            how,
            points,
            probes: probes.flatten(),
            line,
            rapid,
            slot: u8::try_from(slot).unwrap_or(UNCUT - 1).min(UNCUT - 1),
            flute,
            rpm: scan.rpm_at(line),
            to: to_joints,
            from: from_joints,
            travel,
            cost,
        });
        state = Some(to);
    }
    // Bands of whole tiles, so that the tiles of the bands are the tiles of the grid.
    let lines = grid.lines();
    let count = band_count(lines);
    let tiles = lines.div_ceil(TILE).max(1);
    let bands = (0..count).map(|b| ((b * tiles / count * TILE).min(lines), ((b + 1) * tiles / count * TILE).min(lines))).collect();
    Ok(SimPlan {
        machine: machine.clone(),
        steps,
        grid,
        cell,
        tol,
        initial,
        minutes,
        feed_length,
        rapid_length,
        tools: profiles.iter().map(|(n, _)| *n).collect(),
        bands,
    })
}

/// Tool positions for a travel of `steps` sample steps.
fn points(steps: f64) -> Result<usize> {
    let n = steps.ceil().max(1.0);
    if !(n <= MAX_BLOCK_POINTS as f64) {
        return Err(CamError::TooLarge("a block needs too many tool positions to simulate: give a larger tolerance".into()));
    }
    Ok(n as usize)
}

#[derive(Default)]
struct Comparison {
    worst: f64,
    gouged: usize,
    unrefined: usize,
    leftover_max: f64,
    leftover_sum: f64,
    machined: usize,
}

impl Comparison {
    fn add(&mut self, o: &Comparison) {
        self.worst = self.worst.max(o.worst);
        self.gouged += o.gouged;
        self.unrefined += o.unrefined;
        self.leftover_max = self.leftover_max.max(o.leftover_max);
        self.leftover_sum += o.leftover_sum;
        self.machined += o.machined;
    }
}

/// The distance from `p` to `surface`, or `cap` when that is nearer.
pub(crate) fn distance_within(surface: &Surface, p: Vec3, cap: f64) -> f64 {
    let best = Cell::new(cap);
    surface.visit(
        |b: &Aabb3| {
            let q = p.max(b.min).min(b.max);
            p.dist(q) < best.get()
        },
        |_, t| {
            let d = p.dist(nearest_on_triangle(p, t));
            if d < best.get() {
                best.set(d);
            }
        },
    );
    best.get()
}

/// Most crossings of the part's surface followed along one column when measuring a gouge.
const MAX_CROSSINGS: usize = 64;

/// How deep a cut along the straight column from `from` (outside the part) to `to` (where the
/// cut ends) goes into the part, at most `cap`. The part's material along the column is found
/// from where it crosses the part's surface: a triangle facing against the column starts
/// material, one facing along it ends it (a crossing that does neither, as where the column
/// grazes an edge, is passed over). Each stretch of material the column runs through is
/// measured from its middle to the part's surface, and the column's end when it is inside the
/// material. Air between the part's surfaces, as down a hole or a pocket, is no gouge however
/// far it is from them; a stretch of material, however thin, is as deep as its middle.
pub(crate) fn column_depth(part: &Surface, from: Vec3, to: Vec3, cap: f64) -> f64 {
    let d = to - from;
    let len = d.len();
    if !(len > 1e-12) || !len.is_finite() {
        return 0.0;
    }
    let step = 1e-9 / len;
    let measure = |a: f64, b: f64| distance_within(part, from + d * (0.5 * (a + b)), cap);
    let (mut t0, mut inside, mut start, mut worst) = (0.0f64, false, 0.0f64, 0.0f64);
    for _ in 0..MAX_CROSSINGS {
        let Some((t, i)) = part.raycast(from + d * t0, d) else { break };
        let at = t0 + t;
        if !(at <= 1.0) {
            break;
        }
        let facing = part.triangle(i).map_or(0.0, |tri| (tri[1] - tri[0]).cross(tri[2] - tri[0]).dot(d));
        if facing < 0.0 && !inside {
            (inside, start) = (true, at);
        } else if facing > 0.0 && inside {
            inside = false;
            worst = worst.max(measure(start, at));
        }
        t0 = at + step;
    }
    if inside {
        worst = worst.max(measure(start, 1.0)).max(distance_within(part, to, cap));
    }
    worst.min(cap)
}

/// The stock left in `model` (a band) against the part; cells the program cut are those below
/// `initial` (the stock's top or radius). At most `refine` gouged cells are measured in 3D.
fn compare(model: &Model, part: &Surface, tol: f64, initial: f64, refine: usize) -> Comparison {
    let mut cmp = Comparison::default();
    let (mut refined, mut unrefined) = (0usize, 0usize);
    // The depth into the part of a column cut from outside the part (`point(outer)`) in to
    // `at`, `gap` deep below the part's surface along it: `point(v)` is the column's point at
    // height (or radius) `v`. Measured in 3D over the material the column runs through
    // ([`column_depth`]).
    let mut depth = |gap: f64, at: f64, outer: f64, point: &dyn Fn(f64) -> Vec3| -> f64 {
        if refined >= refine {
            unrefined += 1;
            return gap;
        }
        refined += 1;
        column_depth(part, point(outer), point(at), gap)
    };
    match model {
        Model::Heights(h) => {
            let span = (h.top - h.bottom).max(0.0) + 2.0;
            for j in 0..h.ny {
                for i in 0..h.nx {
                    let Some(&z) = h.h.get(j * h.nx + i) else { continue };
                    let (x, y) = h.centre(i, j);
                    let from = Vec3::new(x, y, h.top + 1.0);
                    let Some((t, _)) = part.raycast(from, Vec3::new(0.0, 0.0, -span)) else { continue };
                    let surface = h.top + 1.0 - t * span;
                    let gap = z - surface;
                    if gap < -tol {
                        let d = depth(-gap, z, h.top + 1.0, &|v| Vec3::new(x, y, v));
                        if d > tol {
                            cmp.gouged += 1;
                            cmp.worst = cmp.worst.max(d);
                        }
                    }
                    if z < initial - 1e-9 {
                        cmp.machined += 1;
                        // No stock below the stock's bottom to leave.
                        let left = (z - surface.max(h.bottom)).max(0.0);
                        cmp.leftover_sum += left;
                        cmp.leftover_max = cmp.leftover_max.max(left);
                    }
                }
            }
        }
        Model::Radii(r) => {
            let out = r.radius + 1.0;
            for k in 0..r.nt {
                let (Some(&s), Some(&c)) = (r.sin.get(k), r.cos.get(k)) else { continue };
                let dir = Vec3::new(0.0, s, c);
                for i in 0..r.nx {
                    let Some(&rho) = r.rho.get(k * r.nx + i) else { continue };
                    let x = r.x0 + (i as f64 + 0.5) * r.dx;
                    let from = Vec3::new(x, 0.0, 0.0) + dir * out;
                    let Some((t, _)) = part.raycast(from, dir * -out) else { continue };
                    let surface = out * (1.0 - t);
                    let gap = rho - surface;
                    if gap < -tol {
                        let d = depth(-gap, rho, out, &|v| Vec3::new(x, 0.0, 0.0) + dir * v);
                        if d > tol {
                            cmp.gouged += 1;
                            cmp.worst = cmp.worst.max(d);
                        }
                    }
                    if rho < r.start.get(k).copied().unwrap_or(initial) - 1e-9 {
                        cmp.machined += 1;
                        let left = gap.max(0.0);
                        cmp.leftover_sum += left;
                        cmp.leftover_max = cmp.leftover_max.max(left);
                    }
                }
            }
        }
    }
    cmp.unrefined = unrefined;
    cmp
}

impl StockGrid {
    /// The stock's surface as triangles (top heights, or the radial surface), every `step`-th
    /// cell so that a side has at most `max_side` points; for a preview.
    pub fn mesh(&self, max_side: usize) -> Mesh {
        let max_side = max_side.max(2);
        let mut m = Mesh::default();
        let (nu, nv, point): (usize, usize, Box<dyn Fn(usize, usize) -> Option<Vec3> + '_>) = match self {
            StockGrid::Heights { x0, y0, dx, dy, nx, ny, heights, .. } => (
                *nx,
                *ny,
                Box::new(move |i, j| heights.get(j * nx + i).map(|&z| Vec3::new(x0 + (i as f64 + 0.5) * dx, y0 + (j as f64 + 0.5) * dy, z))),
            ),
            StockGrid::Radii { x0, dx, nx, nt, radii, .. } => (
                *nx,
                *nt,
                Box::new(move |i, k| {
                    let th = (k as f64 + 0.5) * TAU / *nt as f64;
                    radii.get(k * nx + i).map(|&r| Vec3::new(x0 + (i as f64 + 0.5) * dx, r * th.sin(), r * th.cos()))
                }),
            ),
        };
        let su = nu.div_ceil(max_side).max(1);
        let sv = nv.div_ceil(max_side).max(1);
        let us: Vec<usize> = (0..nu).step_by(su).chain((nu > 0 && (nu - 1) % su != 0).then(|| nu - 1)).collect();
        let vs: Vec<usize> = (0..nv).step_by(sv).chain((nv > 0 && (nv - 1) % sv != 0).then(|| nv - 1)).collect();
        for &v in &vs {
            for &u in &us {
                m.positions.push(point(u, v).unwrap_or(Vec3::ZERO));
            }
        }
        let wrap = matches!(self, StockGrid::Radii { .. });
        let (cu, cv) = (us.len(), vs.len());
        let rows = if wrap { cv } else { cv.saturating_sub(1) };
        for b in 0..rows {
            let b1 = (b + 1) % cv.max(1);
            for a in 0..cu.saturating_sub(1) {
                let at = |a: usize, b: usize| u32::try_from(b * cu + a).ok();
                if let (Some(p), Some(q), Some(r), Some(s)) = (at(a, b), at(a + 1, b), at(a + 1, b1), at(a, b1)) {
                    m.triangles.push([p, q, r]);
                    m.triangles.push([p, r, s]);
                }
            }
        }
        m
    }
}
