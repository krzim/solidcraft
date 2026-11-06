//! Stock and facing: the raw material a job starts from, and the operation that faces its top
//! flat.
//!
//! A [`Stock`] is a box or a cylinder about work X (round bar on a 4th axis), in work
//! coordinates. It is what the program may not rapid through: [`Stock::intrusion`] says how
//! deep a cutter, taken as a cylinder of its radius endless along its axis above the tip (the
//! shank and spindle are there too), is inside it. Nothing tracks what earlier cuts removed, so
//! the stock as given is the only safe bound.
//!
//! [`face`] cuts the stock's top (or any rectangle) down to a depth in levels, in straight
//! passes along an angle a stepover apart. Every pass starts and ends with the whole cutter
//! clear of the area by [`FACE_LEAD`], so the tool goes down to each level beside the stock and
//! enters it at depth, never plunging into material; the outermost passes reach past the edges
//! by the overhang, so the edges come out clean. Zigzag passes link at depth outside the area;
//! one-way passes rise to the retract height between passes.

use serde::{Deserialize, Serialize};
use solidcore_record::{Vec2, Vec3};
use solidcraft_geom::Aabb3;
use solidcraft_geom::prelude::*;

use crate::check::verify;
use crate::multiaxis::Pose;
use crate::ops::{Feeds, Heights};
use crate::path::{MAX_MOVES, Move, Toolpath};
use crate::tool::{Tool, ToolKind};
use crate::{CamError, Result, invalid};

/// Largest coordinate, size or margin accepted (mm).
const MAX_COORD: f64 = 1.0e6;
/// How far every facing pass starts and ends beyond the cutter touching the area (mm).
pub const FACE_LEAD: f64 = 1.0;
/// What the default overhang adds to the tool radius (mm): the outermost passes' centres lie
/// this far outside the edges.
pub const FACE_MARGIN: f64 = 0.5;
/// Most passes of one facing level, and most levels.
const MAX_PASSES: usize = 100_000;
const MAX_LEVELS: usize = 10_000;
/// Points a tilted cutter's axis is sampled at against a stock it is not square to.
const AXIS_SAMPLES: usize = 64;

/// The raw material, in work coordinates (mm).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Stock {
    /// A box from its lowest to its highest corner.
    Box { min: Vec3, max: Vec3 },
    /// A round bar about work X (the A axis of a 4th axis set up the usual way), from `x_min`
    /// to `x_max`.
    Cylinder { radius: f64, x_min: f64, x_max: f64 },
}

fn finite(v: f64) -> bool {
    v.is_finite() && v.abs() <= MAX_COORD
}

impl Stock {
    pub fn validate(&self) -> Result<()> {
        match *self {
            Stock::Box { min, max } => {
                if ![min.x, min.y, min.z, max.x, max.y, max.z].into_iter().all(finite) {
                    return invalid("the stock's corners must be finite numbers within 1000 m");
                }
                if !(min.x < max.x && min.y < max.y && min.z < max.z) {
                    return invalid("the stock box's `min` must be below its `max` along every axis");
                }
            }
            Stock::Cylinder { radius, x_min, x_max } => {
                if !(radius > 0.0) || !finite(radius) {
                    return invalid("the stock cylinder's radius must be above 0 and at most 1000 m");
                }
                if !finite(x_min) || !finite(x_max) || !(x_min < x_max) {
                    return invalid("the stock cylinder's `x_min` must be below its `x_max`, both finite");
                }
            }
        }
        Ok(())
    }

    /// The box round `part` (a non-empty extent) grown by `margin` on every side.
    pub fn around(part: &Aabb3, margin: Vec3) -> Result<Stock> {
        if ![margin.x, margin.y, margin.z].iter().all(|m| m.is_finite() && *m >= 0.0 && *m <= 1.0e4) {
            return invalid("the stock margin must be from 0 to 10 m along each axis");
        }
        if part.is_empty() || !part.min.is_finite() || !part.max.is_finite() {
            return invalid("the part has no extent to put the stock round");
        }
        let stock = Stock::Box { min: part.min - margin, max: part.max + margin };
        // A flat part with no margin has no height: give it the thinnest box there is.
        if let Stock::Box { min, max } = stock
            && !(min.x < max.x && min.y < max.y && min.z < max.z)
        {
            return invalid("the part is flat along an axis: give the stock a margin along it");
        }
        stock.validate()?;
        Ok(stock)
    }

    /// The highest Z of the stock at A = 0.
    pub fn top(&self) -> f64 {
        match *self {
            Stock::Box { max, .. } => max.z,
            Stock::Cylinder { radius, .. } => radius,
        }
    }

    /// The low and high corners of the stock seen from above (work XY).
    pub fn footprint(&self) -> [Vec2; 2] {
        match *self {
            Stock::Box { min, max } => [Vec2::new(min.x, min.y), Vec2::new(max.x, max.y)],
            Stock::Cylinder { radius, x_min, x_max } => [Vec2::new(x_min, -radius), Vec2::new(x_max, radius)],
        }
    }

    /// The stock's extent along X.
    pub fn x_range(&self) -> (f64, f64) {
        match *self {
            Stock::Box { min, max } => (min.x, max.x),
            Stock::Cylinder { x_min, x_max, .. } => (x_min, x_max),
        }
    }

    /// The farthest the stock reaches from the line through `center` along X: what it sweeps
    /// as the A axis turns it.
    pub fn radius_about_x(&self, center: Vec2) -> f64 {
        match *self {
            Stock::Box { min, max } => [(min.y, min.z), (min.y, max.z), (max.y, min.z), (max.y, max.z)]
                .iter()
                .map(|&(y, z)| (y - center.x).hypot(z - center.y))
                .fold(0.0, f64::max),
            Stock::Cylinder { radius, .. } => radius + center.len(),
        }
    }

    /// Whether `b` lies inside the stock, within `tol`.
    pub fn holds(&self, b: &Aabb3, tol: f64) -> bool {
        if b.is_empty() {
            return true;
        }
        match *self {
            Stock::Box { min, max } => {
                b.min.x >= min.x - tol
                    && b.min.y >= min.y - tol
                    && b.min.z >= min.z - tol
                    && b.max.x <= max.x + tol
                    && b.max.y <= max.y + tol
                    && b.max.z <= max.z + tol
            }
            Stock::Cylinder { radius, x_min, x_max } => {
                let r = [(b.min.y, b.min.z), (b.min.y, b.max.z), (b.max.y, b.min.z), (b.max.y, b.max.z)]
                    .iter()
                    .map(|&(y, z)| y.hypot(z))
                    .fold(0.0, f64::max);
                b.min.x >= x_min - tol && b.max.x <= x_max + tol && r <= radius + tol
            }
        }
    }

    /// How deep the cutter at `pose` (a cylinder of `radius` from the tip along its axis
    /// without end) is in the stock, where it is deeper than `tol`: the least it would have to
    /// move to clear it, along its axis or sideways. `None` when it is clear.
    pub fn intrusion(&self, pose: &Pose, radius: f64, tol: f64) -> Option<f64> {
        match *self {
            Stock::Box { min, max } => box_intrusion(min + Vec3::new(tol, tol, tol), max - Vec3::new(tol, tol, tol), pose, radius),
            Stock::Cylinder { radius: big, x_min, x_max } => axis_intrusion(pose, radius, Vec2::new(0.0, 0.0), big - tol, x_min + tol, x_max - tol),
        }
    }
}

fn coord(v: Vec3, k: usize) -> f64 {
    match k {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

/// The distance from `p` to the box `lo`–`hi` (0 inside).
fn point_box(p: Vec3, lo: Vec3, hi: Vec3) -> f64 {
    let d = |v: f64, a: f64, b: f64| (a - v).max(0.0).max(v - b);
    Vec3::new(d(p.x, lo.x, hi.x), d(p.y, lo.y, hi.y), d(p.z, lo.z, hi.z)).len()
}

/// [`Stock::intrusion`] of the box `lo`–`hi` (already shrunk by the tolerance).
fn box_intrusion(lo: Vec3, hi: Vec3, pose: &Pose, radius: f64) -> Option<f64> {
    if !(lo.x < hi.x && lo.y < hi.y && lo.z < hi.z) {
        return None;
    }
    let (p, a) = (pose.tip, pose.axis);
    // A cutter along a work axis: exact.
    for k in 0..3 {
        let s = coord(a, k);
        if s.abs() < 1.0 - 1e-9 {
            continue;
        }
        let along = if s > 0.0 { coord(hi, k) - coord(p, k) } else { coord(p, k) - coord(lo, k) };
        if !(along > 0.0) {
            return None;
        }
        let (i, j) = ((k + 1) % 3, (k + 2) % 3);
        let side = |c: usize| (coord(lo, c) - coord(p, c)).max(0.0).max(coord(p, c) - coord(hi, c));
        let gap = side(i).hypot(side(j));
        return (gap < radius).then(|| along.min(radius - gap));
    }
    // A tilted cutter: its axis sampled, each point grown by the radius (a little larger than
    // the cutter around its tip, so on the safe side).
    let reach = point_box(p, lo, hi) + (hi - lo).len() + radius;
    let mut nearest = f64::INFINITY;
    for k in 0..=AXIS_SAMPLES {
        let q = p + a * (reach * k as f64 / AXIS_SAMPLES as f64);
        nearest = nearest.min(point_box(q, lo, hi));
    }
    (nearest < radius).then_some(radius - nearest)
}

/// How deep the cutter at `pose` (a cylinder of `radius` from the tip along its axis without
/// end) reaches inside the round region within `big` of the line along X through `(center.x,
/// center.y)` in Y and Z, from `x_lo` to `x_hi`: `big` less the cutter's nearest approach to
/// the line, where that is positive and the cutter is level with the region along X.
pub(crate) fn axis_intrusion(pose: &Pose, radius: f64, center: Vec2, big: f64, x_lo: f64, x_hi: f64) -> Option<f64> {
    if !(x_lo < x_hi) || !(big > 0.0) {
        return None;
    }
    let (p, a) = (pose.tip, pose.axis);
    let q = Vec2::new(p.y - center.x, p.z - center.y);
    let nearest = if a.x.abs() < 1e-9 {
        // Square to the line: the cutter's cross-section in YZ is a strip `radius` either side
        // of its axis, from the tip outwards.
        let along = Vec2::new(a.y, a.z).normalized()?;
        let across = along.perp();
        if !(p.x - radius < x_hi && p.x + radius > x_lo) {
            return None;
        }
        q.dot(along).max(0.0).hypot((q.dot(across).abs() - radius).max(0.0))
    } else if a.x.abs() > 1.0 - 1e-9 {
        // Along the line, from the tip one way.
        let level = if a.x > 0.0 { p.x < x_hi } else { p.x > x_lo };
        if !level {
            return None;
        }
        (q.len() - radius).max(0.0)
    } else {
        // Neither: the axis sampled, each point grown by the radius.
        let reach = q.len() + big + (x_hi - x_lo) + radius;
        let mut best = f64::INFINITY;
        for k in 0..=AXIS_SAMPLES {
            let s = p + a * (reach * k as f64 / AXIS_SAMPLES as f64);
            if s.x + radius > x_lo && s.x - radius < x_hi {
                best = best.min(((s.y - center.x).hypot(s.z - center.y) - radius).max(0.0));
            }
        }
        best
    };
    (nearest < big).then_some(big - nearest)
}

/// How a [`face`] operation cuts.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FaceParams {
    /// The low and high corners of the area to face (work XY): the stock's footprint.
    pub area: [Vec2; 2],
    /// `top` is where the first level starts from, `bottom` the faced height.
    pub heights: Heights,
    pub feeds: Feeds,
    /// Most material per level (mm).
    pub stepdown: f64,
    /// Distance between passes (mm), at most the flat of the tool's end.
    pub stepover: f64,
    /// Direction of the passes, radians counter-clockwise from +X.
    pub angle: f64,
    /// How far the cutter reaches past the area's edges on the outermost passes (mm).
    pub overhang: f64,
    /// Every other pass runs back, linked at depth outside the area; otherwise every pass runs
    /// the same way and the tool rises to the retract height between them.
    pub zigzag: bool,
}

/// The diameter of the flat of `tool`'s end: what leaves a flat floor.
pub fn flat_diameter(tool: &Tool) -> f64 {
    match tool.kind {
        ToolKind::BullNose => tool.diameter - 2.0 * tool.corner_radius.unwrap_or(0.0),
        _ => tool.diameter,
    }
}

/// The default stepover of a facing operation: 70% of the flat of the tool's end.
pub fn default_face_stepover(tool: &Tool) -> f64 {
    0.7 * flat_diameter(tool)
}

/// The default overhang of a facing operation: half the tool's diameter and [`FACE_MARGIN`],
/// which puts the outermost passes' centres just outside the edges.
pub fn default_face_overhang(tool: &Tool) -> f64 {
    tool.radius() + FACE_MARGIN
}

/// Depth levels from `top` down to `bottom`, evenly spaced, at most `stepdown` apart.
fn levels(top: f64, bottom: f64, stepdown: f64) -> Result<Vec<f64>> {
    let n = ((top - bottom) / stepdown - 1e-9).ceil().max(1.0);
    if !(n <= MAX_LEVELS as f64) {
        return Err(CamError::TooLarge(format!("more than {MAX_LEVELS} facing levels")));
    }
    let n = n as usize;
    Ok((1..=n).map(|k| if k == n { bottom } else { top - (top - bottom) * k as f64 / n as f64 }).collect())
}

struct Path {
    moves: Vec<Move>,
}

impl Path {
    fn push(&mut self, m: Move) -> Result<()> {
        if self.moves.len() >= MAX_MOVES {
            return Err(CamError::TooLarge(format!("the toolpath has more than {MAX_MOVES} moves")));
        }
        self.moves.push(m);
        Ok(())
    }

    fn pos(&self) -> Option<Vec3> {
        self.moves.iter().rev().find_map(Move::target)
    }
}

/// Faces the area of `p` with `tool` (a flat end mill or a bull nose, as a face mill is) down
/// to `p.heights.bottom`; see the module notes.
pub fn face(tool: &Tool, p: &FaceParams) -> Result<Toolpath> {
    tool.validate()?;
    if !matches!(tool.kind, ToolKind::FlatEndmill | ToolKind::BullNose) {
        return invalid("facing needs a flat end mill or a bull nose (a face mill is either)");
    }
    p.heights.validate()?;
    p.feeds.validate()?;
    let h = p.heights;
    if !(h.top > h.bottom) {
        return invalid("the top must be above the bottom: there is nothing to face");
    }
    let [lo, hi] = p.area;
    if ![lo.x, lo.y, hi.x, hi.y].into_iter().all(finite) || !(lo.x < hi.x && lo.y < hi.y) {
        return invalid("the area to face must be [[low x, low y], [high x, high y]], finite numbers");
    }
    if !(p.stepdown > 0.0) || !finite(p.stepdown) {
        return invalid("the stepdown must be above 0");
    }
    let flat = flat_diameter(tool);
    if !(p.stepover > 0.0) || !(p.stepover <= flat + 1e-9) {
        return invalid(format!("the stepover must be above 0 and at most the flat of the tool's end ({flat} mm)"));
    }
    if !(p.angle.is_finite() && p.angle.abs() <= 1.0e4) {
        return invalid("the pass angle must be a finite angle");
    }
    if !(p.overhang >= 0.0) || !(p.overhang <= 10.0 * tool.diameter) {
        return invalid("the overhang must be from 0 to ten times the tool's diameter");
    }
    let r = tool.radius();
    let (c, s) = (p.angle.cos(), p.angle.sin());
    let local = |q: Vec2| Vec2::new(q.x * c + q.y * s, -q.x * s + q.y * c);
    let corners = [lo, Vec2::new(hi.x, lo.y), hi, Vec2::new(lo.x, hi.y)].map(local);
    let (mut umin, mut umax, mut vmin, mut vmax) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for q in corners {
        (umin, umax, vmin, vmax) = (umin.min(q.x), umax.max(q.x), vmin.min(q.y), vmax.max(q.y));
    }
    // Every pass starts and ends on a line beyond the whole area: the cutter clears it there,
    // and the links between passes run along those lines.
    let (u_start, u_end) = (umin - r - FACE_LEAD, umax + r + FACE_LEAD);
    let (v_first, v_last) = (vmin + r - p.overhang, vmax - r + p.overhang);
    let passes: Vec<f64> = if v_last <= v_first + 1e-9 {
        vec![0.5 * (vmin + vmax)]
    } else {
        let n = ((v_last - v_first) / p.stepover - 1e-9).ceil().max(1.0);
        if !(n < MAX_PASSES as f64) {
            return Err(CamError::TooLarge(format!("more than {MAX_PASSES} facing passes")));
        }
        let n = n as usize;
        (0..=n).map(|k| v_first + (v_last - v_first) * k as f64 / n as f64).collect()
    };
    let world = |u: f64, v: f64, z: f64| Vec3::new(u * c - v * s, u * s + v * c, z);

    let mut path = Path { moves: Vec::new() };
    let mut at_start_side = true;
    for (li, z) in levels(h.top, h.bottom, p.stepdown)?.into_iter().enumerate() {
        // Each level runs the passes the other way round from the last, so a zigzag carries on
        // from where the last level ended.
        let order: Vec<f64> = if p.zigzag && li % 2 == 1 { passes.iter().rev().copied().collect() } else { passes.clone() };
        for (k, v) in order.into_iter().enumerate() {
            if !p.zigzag {
                at_start_side = true;
            }
            let (ua, ub) = if at_start_side { (u_start, u_end) } else { (u_end, u_start) };
            let (a, b) = (world(ua, v, z), world(ub, v, z));
            let pos = path.pos();
            let carry_on = p.zigzag && pos.is_some_and(|q| (k > 0 && (q.z - z).abs() <= 1e-9) || (k == 0 && q.x == a.x && q.y == a.y));
            if carry_on {
                // Along the line outside the area, or down beside it to the next level.
                let feed = if k == 0 { p.feeds.plunge } else { p.feeds.cut };
                path.push(Move::Feed { to: a, feed })?;
            } else {
                match pos {
                    None => path.push(Move::Rapid { to: world(ua, v, h.clearance) })?,
                    Some(q) => {
                        if q.z < h.retract {
                            path.push(Move::Rapid { to: Vec3::new(q.x, q.y, h.retract) })?;
                        }
                        path.push(Move::Rapid { to: world(ua, v, h.retract.max(q.z)) })?;
                    }
                }
                if path.pos().is_some_and(|q| q.z > h.retract) {
                    path.push(Move::Rapid { to: world(ua, v, h.retract) })?;
                }
                path.push(Move::Feed { to: a, feed: p.feeds.plunge })?;
            }
            path.push(Move::Feed { to: b, feed: p.feeds.cut })?;
            at_start_side = !at_start_side;
        }
    }
    if let Some(q) = path.pos() {
        path.push(Move::Rapid { to: Vec3::new(q.x, q.y, h.clearance) })?;
    }
    let out = Toolpath { moves: path.moves };
    verify(&out, &h)?;
    Ok(out)
}
