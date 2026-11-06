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
//! - **4th axis** ([`StockShape::Cylinder`], a rotary about work X through work zero): a grid
//!   over (x, θ) holding the radius the material reaches along the ray from the axis at that x
//!   and angle. A block that turns the rotary while X moves is a helix in the work frame, so
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

use std::cell::Cell;
use std::f64::consts::{PI, TAU};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

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
/// Most tool positions along one block on a 4th axis, and chords of one arc.
const MAX_BLOCK_POINTS: usize = 1_000_000;
/// Most cells measured again in 3D for the gouge depth; beyond, the vertical depth is used.
const MAX_REFINED: usize = 1_000_000;
/// Most tool positions on a 4th axis over the whole program.
const MAX_STAMPS: usize = 50_000_000;
/// Most threads one simulation runs on, and the fewest rows (angles) each one takes.
const MAX_THREADS: usize = 16;
const MIN_BAND: usize = 16;
/// Iterations of the bisections (each halves the interval).
const BISECTIONS: usize = 60;
/// Largest coordinate accepted (mm).
const MAX_COORD: f64 = 1.0e6;
/// A slope taken as vertical (a tool's flank at its rim).
const STEEP: f64 = 1.0e12;

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
            ToolKind::FlatEndmill => Profile { radius: r, corner: 0.0, shape: Shape::Flat },
            ToolKind::BallEndmill => Profile { radius: r, corner: r, shape: Shape::Ball },
            ToolKind::BullNose => {
                let c = tool.corner_radius.unwrap_or(0.0).clamp(0.0, r);
                if c <= 0.0 {
                    Profile { radius: r, corner: 0.0, shape: Shape::Flat }
                } else if c >= r {
                    Profile { radius: r, corner: r, shape: Shape::Ball }
                } else {
                    Profile { radius: r, corner: c, shape: Shape::Bull }
                }
            }
            ToolKind::Drill => Profile { radius: r, corner: 0.0, shape: Shape::Cone(1.0 / (DRILL_POINT_ANGLE * 0.5).tan()) },
        })
    }

    /// Height of the cutting end above the tip at distance `d` from the axis (`d` taken within
    /// the radius).
    pub(crate) fn height(&self, d: f64) -> f64 {
        let d = d.clamp(0.0, self.radius);
        match self.shape {
            Shape::Flat => 0.0,
            Shape::Cone(k) => k * d,
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
            Shape::Cone(k) => tol / k.max(1e-9),
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
                Shape::Bull => {
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
            Shape::Bull | Shape::Cone(_) => {
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
#[derive(Clone, Copy, Debug, Default)]
struct Removed {
    volume: f64,
    depth: f64,
}

impl Removed {
    fn add(&mut self, o: Removed) {
        self.volume += o.volume;
        self.depth = self.depth.max(o.depth);
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
        if let Some(h) = self.h.get_mut(c) {
            let z = z.max(bottom);
            if z < *h {
                out.volume += (*h - z) * area;
                out.depth = out.depth.max(*h - z);
                *h = z;
            }
        }
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
    fn sweep_arc(&mut self, p: &Profile, a: Vec3, b: Vec3, center: Vec2, sweep: f64, tol: f64) -> Result<Removed> {
        let rho = Vec2::new(a.x, a.y).dist(center);
        let r = p.radius;
        if (b.z - a.z).abs() > 1e-12 || rho <= 1e-9 || sweep.abs() <= 1e-12 {
            // A helix (or a degenerate arc): chords at most a quarter of the tolerance inside it.
            let e = (tol * 0.25).min(rho);
            let step = if rho > 1e-9 { 2.0 * (1.0 - e / rho).clamp(-1.0, 1.0).acos() } else { TAU };
            let n = (sweep.abs() / step.max(1e-9)).ceil().max(1.0);
            if !(n <= MAX_BLOCK_POINTS as f64) {
                return Err(CamError::TooLarge("an arc needs too many chords to simulate".into()));
            }
            let n = n as usize;
            let start = Vec2::new(a.x - center.x, a.y - center.y).angle();
            let mut out = Removed::default();
            let mut prev = a;
            for k in 1..=n {
                let t = k as f64 / n as f64;
                let q = if k == n { b } else { (center + Vec2::from_angle(start + sweep * t) * rho).extend(a.z + (b.z - a.z) * t) };
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

/// Whether the angle `ang` is on an arc starting at `start` and turning `sweep` (signed).
fn within_sweep(ang: f64, start: f64, sweep: f64) -> bool {
    if sweep >= 0.0 { (ang - start).rem_euclid(TAU) <= sweep + 1e-12 } else { (start - ang).rem_euclid(TAU) <= -sweep + 1e-12 }
}

/// The radial stock (a 4th axis about X), or one band of its angles: angles `k0..k0 + nt` of
/// the `nt_all` round.
struct Radii {
    x0: f64,
    dx: f64,
    nx: usize,
    k0: usize,
    nt: usize,
    nt_all: usize,
    radius: f64,
    rho: Vec<f64>,
    /// sin and cos of the band's angles.
    sin: Vec<f64>,
    cos: Vec<f64>,
    visits: Arc<AtomicU64>,
    tol: f64,
    /// Cells a tool position cut beside their ray, inside the material (not modelled).
    tunnels: u64,
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
                        out.depth = out.depth.max(have - at);
                        if let Some(slot) = self.rho.get_mut(cell) {
                            *slot = at;
                        }
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

/// The stock's grid before it is split into bands.
#[derive(Clone, Copy, Debug)]
enum Grid {
    Heights { x0: f64, y0: f64, dx: f64, dy: f64, nx: usize, ny: usize, bottom: f64, top: f64 },
    Radii { x0: f64, dx: f64, nx: usize, nt: usize, radius: f64 },
}

impl Grid {
    /// Rows (angles) the grid is split along.
    fn lines(&self) -> usize {
        match *self {
            Grid::Heights { ny, .. } => ny,
            Grid::Radii { nt, .. } => nt,
        }
    }

    /// Rows (angles) `from..to` of the stock as it starts.
    fn band(&self, from: usize, to: usize, visits: &Arc<AtomicU64>, tol: f64) -> Model {
        let n = to.saturating_sub(from);
        match *self {
            Grid::Heights { x0, y0, dx, dy, nx, bottom, top, .. } => {
                Model::Heights(Heights { x0, y0, dx, dy, nx, j0: from, ny: n, bottom, top, h: vec![top; nx * n], visits: Arc::clone(visits) })
            }
            Grid::Radii { x0, dx, nx, nt, radius } => {
                let dt = TAU / nt as f64;
                let (sin, cos): (Vec<f64>, Vec<f64>) = (from..to).map(|k| ((k as f64 + 0.5) * dt).sin_cos()).unzip();
                Model::Radii(Radii {
                    x0,
                    dx,
                    nx,
                    k0: from,
                    nt: n,
                    nt_all: nt,
                    radius,
                    rho: vec![radius; nx * n],
                    sin,
                    cos,
                    visits: Arc::clone(visits),
                    tol,
                    tunnels: 0,
                })
            }
        }
    }
}

fn grid_for(stock: &StockShape, params: &SimParams) -> Result<(Grid, f64)> {
    let finite = |v: f64| v.is_finite() && v.abs() <= MAX_COORD;
    match *stock {
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
            Ok((Grid::Radii { x0: x_min, dx: (x_max - x_min) / nx as f64, nx, nt, radius }, cell))
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

/// The tool loaded at each line (`T` with `M6`), and the lines that lose the position (a
/// return home or a move in machine coordinates), both in line order.
fn tool_changes_and_gaps(text: &str) -> Result<(Vec<(usize, u32)>, Vec<usize>)> {
    let (mut changes, mut gaps) = (Vec::new(), Vec::new());
    let mut pending: Option<u32> = None;
    for (i, line) in text.lines().enumerate() {
        let n = i + 1;
        let ws = words(line, n)?;
        let mut m6 = false;
        for &(c, v) in &ws {
            match c {
                'T' => {
                    if !((0.0..=1.0e6).contains(&v) && v.fract() == 0.0) {
                        return Err(CamError::Parse(n, "a tool number must be a whole number".into()));
                    }
                    pending = Some(v as u32);
                }
                'M' if (v - 6.0).abs() < 1e-9 => m6 = true,
                'G' => {
                    let g = (v * 10.0).round() as i64;
                    if matches!(g, 280 | 300 | 530) {
                        gaps.push(n);
                    }
                }
                _ => {}
            }
        }
        if m6 {
            let Some(t) = pending else { return Err(CamError::Parse(n, "a tool change (M6) without a tool number (T)".into())) };
            changes.push((n, t));
        }
    }
    Ok((changes, gaps))
}

/// The tool in the spindle at `line`.
fn tool_at<'a>(changes: &[(usize, u32)], tools: &'a [(u32, Profile)], line: usize) -> Result<&'a Profile> {
    let loaded = changes.partition_point(|(l, _)| *l <= line).checked_sub(1).and_then(|k| changes.get(k)).map(|(_, t)| *t);
    match loaded {
        Some(t) => tools
            .iter()
            .find(|(n, _)| *n == t)
            .map(|(_, p)| p)
            .ok_or_else(|| CamError::Parse(line, format!("the program uses tool {t}, which is not among the tools given"))),
        None => match tools {
            [(_, p)] => Ok(p),
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
}

/// What one band found of every step: material removed, and which probes were in the stock.
struct BandRun {
    model: Model,
    removed: Vec<Removed>,
    probes: Vec<u8>,
    cmp: Comparison,
}

/// The tool positions of `step` on a 4th axis, after its start.
fn step_poses(machine: &Machine, step: &Step) -> Vec<Pose> {
    let n = step.points.max(1);
    let t = |k: usize| k as f64 / n as f64;
    match &step.how {
        How::Straight => (0..=n).map(|k| Pose { tip: step.start.tip.lerp(step.end.tip, t(k)), axis: step.end.axis }).collect(),
        How::Joint { from, to, tcp } => (1..=n)
            .map(|k| {
                let j = from.lerp(to, t(k));
                pose_at(machine, j.linear, &j.rotary, *tcp)
            })
            .collect(),
        How::Arc { from, to, center, sweep, rotary, tcp, .. } => {
            let st = Vec2::new(from.x, from.y) - *center;
            let rad = st.len();
            (1..=n)
                .map(|k| {
                    let q = if k == n {
                        *to
                    } else {
                        (*center + Vec2::from_angle(st.angle() + sweep * t(k)) * rad).extend(from.z + (to.z - from.z) * t(k))
                    };
                    pose_at(machine, q, rotary, *tcp)
                })
                .collect()
        }
    }
}

/// Runs every step through one band of the stock, then holds the band against the part.
fn run_band(mut model: Model, steps: &[Step], machine: &Machine, part: &Surface, tol: f64, initial: f64, refine: usize) -> Result<BandRun> {
    let mut removed = Vec::with_capacity(steps.len());
    let mut probes = Vec::with_capacity(steps.len());
    for step in steps {
        let mut bits = 0u8;
        if let Some(ps) = &step.probes {
            for (b, p) in ps.iter().enumerate() {
                if model.owns(*p) == Some(true) {
                    bits |= 1 << b;
                }
            }
        }
        probes.push(bits);
        let out = match &mut model {
            Model::Heights(h) => match &step.how {
                How::Arc { work_center, sweep, .. } => h.sweep_arc(&step.profile, step.start.tip, step.end.tip, *work_center, *sweep, tol)?,
                _ => h.sweep_line(&step.profile, step.start.tip, step.end.tip)?,
            },
            Model::Radii(r) => {
                let mut out = Removed::default();
                for pose in step_poses(machine, step) {
                    out.add(r.stamp(&step.profile, &pose)?);
                }
                out
            }
        };
        removed.push(out);
    }
    let cmp = compare(&model, part, tol, initial, refine);
    Ok(BandRun { model, removed, probes, cmp })
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

/// Every band run, each on a thread of its own where one can be had, in band order.
fn run_bands(grid: &Grid, steps: &[Step], machine: &Machine, part: &Surface, tol: f64, initial: f64) -> Result<Vec<BandRun>> {
    let lines = grid.lines();
    let bands = band_count(lines);
    let visits = Arc::new(AtomicU64::new(0));
    let refine = (MAX_REFINED / bands).max(1);
    let range = |b: usize| (b * lines / bands, (b + 1) * lines / bands);
    if bands <= 1 {
        let (from, to) = range(0);
        return Ok(vec![run_band(grid.band(from, to, &visits, tol), steps, machine, part, tol, initial, refine)?]);
    }
    std::thread::scope(|scope| {
        let mut pending = Vec::with_capacity(bands);
        for b in 0..bands {
            let (from, to) = range(b);
            let v = Arc::clone(&visits);
            let job = move || run_band(grid.band(from, to, &v, tol), steps, machine, part, tol, initial, refine);
            match std::thread::Builder::new().name(format!("cam-sim-{b}")).spawn_scoped(scope, job) {
                Ok(h) => pending.push(Err(h)),
                // No thread to be had: this band runs here.
                Err(_) => pending.push(Ok(run_band(grid.band(from, to, &visits, tol), steps, machine, part, tol, initial, refine))),
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

/// Simulates `text` (G-code for `machine`, in absolute coordinates) cutting `stock` with
/// `tools` (by number), and compares what is left with `part`. See the module documentation.
pub fn simulate(text: &str, machine: &Machine, tools: &[Tool], stock: &StockShape, part: &Surface, params: &SimParams) -> Result<SimReport> {
    let tol = params.tolerance;
    if !(1e-4..=1.0).contains(&tol) {
        return invalid("the tolerance must be from 0.0001 to 1 mm");
    }
    let kind = Kind::of(machine)?;
    match (kind, stock) {
        (Kind::Three, StockShape::Box { .. }) | (Kind::Rotary, StockShape::Cylinder { .. }) => {}
        (Kind::Three, _) => return invalid("a 3-axis machine's stock is a box"),
        (Kind::Rotary, _) => return invalid("a 4th axis's stock is a cylinder about X"),
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
    let (changes, gaps) = tool_changes_and_gaps(text)?;
    let (grid, cell) = grid_for(stock, params)?;
    let (reach, initial) = match *stock {
        StockShape::Box { min, max } => ((max - min).len(), max.z),
        StockShape::Cylinder { x_min, x_max, radius } => ((x_max - x_min).hypot(2.0 * radius), radius),
    };
    let lever_extra = machine.work_origin.len() + machine.rotaries.iter().map(|r| r.center.len()).sum::<f64>();

    // The blocks as steps, in order.
    let mut steps: Vec<Step> = Vec::with_capacity(blocks.len());
    let mut lines: Vec<(usize, bool)> = Vec::with_capacity(blocks.len());
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
        let profile = *tool_at(&changes, &profiles, line)?;
        // The position before this block, unless the program lost it since the last one.
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
        let step_len = profile.step(tol);
        let (how, points) = match (b, &from) {
            (JointWords::Arc { center, ccw, f, mm_per_unit, .. }, Some(f0)) => {
                let st = Vec2::new(f0.linear.x, f0.linear.y) - *center;
                let sweep = arc_sweep(f0.linear, to.linear, *center, *ccw);
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
                (how, points(travel / step_len)?)
            }
            (JointWords::Arc { .. }, None) => return Err(CamError::Parse(line, "an arc right after the position was lost".into())),
            (_, Some(f0)) => {
                let (a, c) = (Joints { linear: f0.linear, rotary: f0.rotary.clone() }, Joints { linear: to.linear, rotary: to.rotary.clone() });
                let lin = a.linear.dist(c.linear);
                let lever = start.tip.len().max(end.tip.len()) + lin + lever_extra + profile.radius;
                let travel = length.max(lin) + a.rotary_change(&c) * lever;
                (How::Joint { from: a, to: c, tcp }, points(travel / step_len)?)
            }
            (_, None) => (How::Straight, points(length / step_len)?),
        };
        if kind == Kind::Rotary {
            stamps = stamps.saturating_add(points);
            if stamps > MAX_STAMPS {
                return Err(CamError::TooLarge("the program needs too many tool positions to simulate: give a larger tolerance".into()));
            }
        }
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
        steps.push(Step { profile, start, end, how, points, probes: probes.flatten() });
        lines.push((line, rapid));
        state = Some(to);
    }

    let runs = run_bands(&grid, &steps, machine, part, tol, initial)?;
    let mut removals = Vec::new();
    for (k, (line, rapid)) in lines.iter().enumerate() {
        let mut r = Removed::default();
        let mut bits = 0u8;
        for run in &runs {
            if let Some(o) = run.removed.get(k) {
                r.add(*o);
            }
            bits |= run.probes.get(k).copied().unwrap_or(0);
        }
        let slot = bits == 0b111;
        if r.volume > 0.0 || slot {
            removals.push(BlockRemoval { line: *line, rapid: *rapid, volume: r.volume, depth: r.depth, slot });
        }
    }
    let rapids_in_material: Vec<BlockRemoval> = removals.iter().filter(|r| r.rapid && r.depth > tol).copied().collect();
    let removed_volume = removals.iter().map(|r| r.volume).sum();
    let mut warnings = Vec::new();
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
    for run in &runs {
        cmp.add(&run.cmp);
    }
    if cmp.unrefined > 0 {
        warnings.push(format!("{} gouged cells were measured along Z (or the radius) only: too many to measure in 3D", cmp.unrefined));
    }
    let tunnels: u64 = runs.iter().map(|r| if let Model::Radii(m) = &r.model { m.tunnels } else { 0 }).sum();
    if tunnels > 0 {
        warnings.push(format!(
            "{tunnels} times the tool cut beside a ray from the A axis without reaching out of the material along it (a feature in \
             the side of the part): the round stock model leaves those cuts out, so they are not checked here (the job's gouge check \
             covers them)"
        ));
    }
    let stock_grid = match grid {
        Grid::Heights { x0, y0, dx, dy, nx, ny, bottom, top } => {
            let mut heights = Vec::with_capacity(nx * ny);
            for run in runs {
                if let Model::Heights(h) = run.model {
                    heights.extend(h.h);
                }
            }
            StockGrid::Heights { x0, y0, dx, dy, nx, ny, bottom, top, heights }
        }
        Grid::Radii { x0, dx, nx, nt, radius } => {
            let mut radii = Vec::with_capacity(nx * nt);
            for run in runs {
                if let Model::Radii(r) = run.model {
                    radii.extend(r.rho);
                }
            }
            StockGrid::Radii { x0, dx, nx, nt, radius, radii }
        }
    };
    Ok(SimReport {
        cell,
        cells: match &stock_grid {
            StockGrid::Heights { heights, .. } => heights.len(),
            StockGrid::Radii { radii, .. } => radii.len(),
        },
        stock: stock_grid,
        removals,
        rapids_in_material,
        removed_volume,
        gouge_worst: cmp.worst,
        gouge_cells: cmp.gouged,
        leftover_max: cmp.leftover_max,
        leftover_mean: if cmp.machined > 0 { cmp.leftover_sum / cmp.machined as f64 } else { 0.0 },
        machined_cells: cmp.machined,
        minutes,
        feed_length,
        rapid_length,
        warnings,
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
fn distance_within(surface: &Surface, p: Vec3, cap: f64) -> f64 {
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

/// The stock left in `model` (a band) against the part; cells the program cut are those below
/// `initial` (the stock's top or radius). At most `refine` gouged cells are measured in 3D.
fn compare(model: &Model, part: &Surface, tol: f64, initial: f64, refine: usize) -> Comparison {
    let mut cmp = Comparison::default();
    let (mut refined, mut unrefined) = (0usize, 0usize);
    // The depth into the part of a column cut from `outer` (the part's surface along the
    // column) in to `at`, `gap` deep along it: `point(v)` is the column's point at height (or
    // radius) `v`. Measured from the column's bottom and middle to the part's surface.
    let mut depth = |gap: f64, at: f64, outer: f64, point: &dyn Fn(f64) -> Vec3| -> f64 {
        if refined >= refine {
            unrefined += 1;
            return gap;
        }
        refined += 1;
        let bottom = distance_within(part, point(at), gap);
        let middle = distance_within(part, point(0.5 * (at + outer)), gap * 0.5);
        bottom.max(middle)
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
                        let d = depth(-gap, z, surface, &|v| Vec3::new(x, y, v));
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
                        let d = depth(-gap, rho, surface, &|v| Vec3::new(x, 0.0, 0.0) + dir * v);
                        if d > tol {
                            cmp.gouged += 1;
                            cmp.worst = cmp.worst.max(d);
                        }
                    }
                    if rho < initial - 1e-9 {
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
