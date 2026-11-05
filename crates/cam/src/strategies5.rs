//! Simultaneous 5-axis toolpaths: the tool axis set along the way from the surface, a point, a
//! line or a ruled wall.
//!
//! A surface finish follows a [`DrivePattern`]: points on the surface, with the surface's normal
//! there, in passes. At each point the tool axis comes from an [`AxisControl`], and the tip is
//! placed where the tool, coming in along that axis, first touches the surface
//! ([`drop_along`]), so no pose cuts into the surface whatever the cutter's end. Each pass is
//! sampled more finely wherever the tip strays from the straight move between two poses by more
//! than the tolerance, and where the axis still turns more than the step allowed in one move,
//! poses turned evenly between the two are put in, each placed the same way. Every move is then
//! checked finely for the cutter cutting into the surface between its poses (a feature narrower
//! than the samples' spacing): such a move is halved, a pose placed in between, and where that
//! does not help, the pass is split there and the tool lifted over.
//!
//! Passes are linked the same way every time. The tool leaves along its own axis (the way a
//! placed tool came in, so no part of the cutter can touch the surface) until its tip reaches
//! the clearance plane or has left the surface's bounding box grown by the tool radius for good,
//! then rises straight up (+Z) to the clearance plane, crosses to the next pass at or above it,
//! and comes down the same way; the last stretch, from the retract distance up the axis to the
//! surface, is fed at the plunge feed. The links are checked for gouges too. A pass that would
//! end with the tool axis pointing below the horizontal cannot be left like this and is refused.
//!
//! Swarf (flank) milling cuts a ruled wall, given as two rails of matching points, with the side
//! of the tool: the axis along each ruling, the tool beside the wall. A wall that is not
//! developable (a twisted one) cannot be touched by a cylinder along a whole ruling without the
//! cylinder cutting into it or staying off it a little nearby; [`swarf`] measures by how much.

use std::cell::Cell;

use serde::{Deserialize, Serialize};
use solidcore_record::{Vec2, Vec3};
use solidcraft_geom::prelude::*;
use solidcraft_geom::{Aabb3, Bvh};

use crate::collision::{GougeTest, Moves, gouges_in};
use crate::cutter::Cutter;
use crate::dropcutter::drop_along;
use crate::frame::{Frame, rotate, unit};
use crate::multiaxis::{Move5, Pose, Toolpath5, slerp, verify5};
use crate::ops::Feeds;
use crate::path::MAX_MOVES;
use crate::surface::{MAX_COORD, Surface, nearest_on_triangle, nearest_segments};
use crate::tool::Tool;
use crate::{CamError, Result, invalid};

/// Most passes of one drive pattern.
const MAX_PASSES: usize = 100_000;
/// Most points one drive pattern or one finish samples on the surface.
const MAX_SAMPLES: usize = 4_000_000;
/// Most times a move between two samples is halved.
const MAX_DEPTH: u32 = 20;
/// Smallest axis step accepted (radians): it bounds the poses put in to turn the axis.
const MIN_AXIS_STEP: f64 = 1e-4;
/// Smallest cosine between the tool axis and the surface normal at a contact: an axis at or past
/// the tangent plane cannot reach the point from outside the material.
const MIN_REACH: f64 = 1e-6;
/// Most rulings of a swarf wall.
const MAX_RULINGS: usize = 100_000;
/// Most depth passes of a swarf cut.
const MAX_LEVELS: u32 = 1000;
/// Points across a wall (bottom to top) at which a swarf cut's deviation is measured, less one.
const ACROSS: usize = 16;
/// Most spans a swarf cut's deviation is measured against, over the whole cut.
const MAX_LOOKS: usize = 10_000_000;

/// How the tool axis is set at each point of a surface finish. The axis points from the tip up
/// the tool. Angles are radians.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum AxisControl {
    /// The same axis everywhere.
    Fixed { axis: Vec3 },
    /// The surface's normal, turned by `lead` about the side direction (positive: the spindle
    /// leans forward, towards the feed direction) and then by `tilt` about the feed direction
    /// (positive: the spindle leans to the left of the feed, seen from outside the material).
    /// Each is less than a right angle.
    Normal { lead: f64, tilt: f64 },
    /// From the contact point towards `point`.
    TowardPoint { point: Vec3 },
    /// From `point` through the contact point: for convex shapes around it.
    AwayFromPoint { point: Vec3 },
    /// From the contact point straight towards the line through `point` along `direction`.
    TowardLine { point: Vec3, direction: Vec3 },
    /// Straight out from the line through `point` along `direction` through the contact point.
    AwayFromLine { point: Vec3, direction: Vec3 },
}

fn finite_point(p: Vec3, what: &str) -> Result<()> {
    if !p.is_finite() || !(p.x.abs() <= MAX_COORD && p.y.abs() <= MAX_COORD && p.z.abs() <= MAX_COORD) {
        return invalid(format!("{what} must be a finite point within 1000 m"));
    }
    Ok(())
}

/// The unit direction along `v` in the plane square to the unit `n`; `None` when `v` is (close
/// to) along `n`.
fn across(v: Vec3, n: Vec3) -> Option<Vec3> {
    let w = v - n * n.dot(v);
    (w.len() > 1e-9 * (1.0 + v.len())).then(|| w.normalized()).flatten()
}

impl AxisControl {
    pub fn validate(&self) -> Result<()> {
        match *self {
            AxisControl::Fixed { axis } => unit(axis, "the fixed tool axis").map(|_| ()),
            AxisControl::Normal { lead, tilt } => {
                let half = std::f64::consts::FRAC_PI_2;
                if !(lead.abs() < half) || !(tilt.abs() < half) {
                    return invalid("the lead and tilt angles must be less than a right angle either way");
                }
                Ok(())
            }
            AxisControl::TowardPoint { point } | AxisControl::AwayFromPoint { point } => finite_point(point, "the point the tool axis is set by"),
            AxisControl::TowardLine { point, direction } | AxisControl::AwayFromLine { point, direction } => {
                finite_point(point, "a point of the line the tool axis is set by")?;
                unit(direction, "the direction of the line the tool axis is set by").map(|_| ())
            }
        }
    }

    /// The axis at the contact point `at` with the unit surface normal `n`, the tool moving along
    /// `feed`; `None` where it is not defined (the contact on the point or line).
    fn axis(&self, at: Vec3, n: Vec3, feed: Vec3) -> Option<Vec3> {
        match *self {
            AxisControl::Fixed { axis } => axis.normalized(),
            AxisControl::Normal { lead, tilt } => {
                let f = across(feed, n).unwrap_or_else(|| n.any_perp());
                let left = n.cross(f);
                let a = rotate(n, left, lead);
                rotate(a, f, -tilt).normalized()
            }
            AxisControl::TowardPoint { point } => (point - at).normalized(),
            AxisControl::AwayFromPoint { point } => (at - point).normalized(),
            AxisControl::TowardLine { point, direction } | AxisControl::AwayFromLine { point, direction } => {
                let d = direction.normalized()?;
                let foot = point + d * d.dot(at - point);
                let to_line = (foot - at).normalized()?;
                Some(if matches!(self, AxisControl::TowardLine { .. }) { to_line } else { -to_line })
            }
        }
    }
}

/// Where a surface finish touches the surface: points with the surface's normal, in passes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum DrivePattern {
    /// Straight, evenly spaced lines in the plane square to `direction` (left out, straight
    /// down, −Z), projected onto the surface along it; the passes run to and fro. `angle`
    /// (radians) turns the lines from that plane's X axis (the world X when projecting down),
    /// `stepover` is the most between lines and `step` the most between points along a line
    /// (mm). `boundary`, the corners `[min, max]` of a rectangle in that plane's coordinates
    /// (the world XY when projecting down), limits the lines; left out, the surface's extent.
    Parallel {
        angle: f64,
        stepover: f64,
        step: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        boundary: Option<[Vec2; 2]>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        direction: Option<Vec3>,
    },
    /// Polylines on the surface, each one pass: every point is taken to the nearest point of
    /// the surface, whose normal there is used. `step` (mm), when given, is the most between
    /// points along a curve; longer segments are divided.
    Along {
        curves: Vec<Vec<Vec3>>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        step: Option<f64>,
    },
}

/// A point on the surface the tool is to touch.
#[derive(Clone, Copy, Debug)]
struct Contact {
    at: Vec3,
    /// The surface's unit normal at `at`, out of the material.
    normal: Vec3,
    /// What the point was found from: a ray's start, or a point of a curve.
    seed: Vec3,
}

/// How points between samples are found on the surface.
#[derive(Clone, Copy, Debug)]
enum Projector {
    /// Cast along the unit direction.
    Ray(Vec3),
    /// The nearest point of the surface.
    Nearest,
}

impl Projector {
    fn project(&self, surface: &Surface, seed: Vec3) -> Option<Contact> {
        match *self {
            Projector::Ray(d) => {
                let (t, i) = surface.raycast(seed, d)?;
                let at = seed + d * t;
                let normal = surface.normal_at(i, at)?;
                // The back of a surface seen from where the rays come from is not machined.
                (normal.dot(-d) > 0.0).then_some(Contact { at, normal, seed })
            }
            Projector::Nearest => {
                let (at, i) = nearest_point(surface, seed)?;
                let normal = surface.normal_at(i, at)?;
                Some(Contact { at, normal, seed })
            }
        }
    }
}

/// The squared distance from `p` to the box.
fn box_dist2(b: &Aabb3, p: Vec3) -> f64 {
    let d = |v: f64, lo: f64, hi: f64| {
        if v < lo {
            lo - v
        } else if v > hi {
            v - hi
        } else {
            0.0
        }
    };
    let (x, y, z) = (d(p.x, b.min.x, b.max.x), d(p.y, b.min.y, b.max.y), d(p.z, b.min.z, b.max.z));
    x * x + y * y + z * z
}

/// The squared distance between the segment `a–b` and the box.
fn segment_box_dist2(a: Vec3, b: Vec3, bx: &Aabb3) -> f64 {
    let d = b - a;
    let axes = [(a.x, d.x, bx.min.x, bx.max.x), (a.y, d.y, bx.min.y, bx.max.y), (a.z, d.z, bx.min.z, bx.max.z)];
    // Where the segment crosses the planes of the box's faces, it falls into pieces along each
    // of which every coordinate stays below, within or above the box: the squared distance is a
    // quadratic on each piece.
    let mut cuts = [0.0f64; 8];
    let mut n = 0;
    let mut cut = |t: f64| {
        if let Some(c) = cuts.get_mut(n) {
            *c = t;
            n += 1;
        }
    };
    cut(0.0);
    cut(1.0);
    for (o, dv, lo, hi) in axes {
        if dv.abs() > 1e-300 {
            for v in [lo, hi] {
                let t = (v - o) / dv;
                if t > 0.0 && t < 1.0 {
                    cut(t);
                }
            }
        }
    }
    let cuts = cuts.get_mut(..n).unwrap_or(&mut []);
    cuts.sort_by(f64::total_cmp);
    let at = |t: f64| box_dist2(bx, a + d * t);
    let mut best = at(0.0).min(at(1.0));
    for w in cuts.windows(2) {
        let [t0, t1] = *w else { continue };
        let mid = a + d * ((t0 + t1) * 0.5);
        let (mut qa, mut qb) = (0.0, 0.0);
        for ((o, dv, lo, hi), m) in axes.into_iter().zip([mid.x, mid.y, mid.z]) {
            let bound = if m < lo {
                lo
            } else if m > hi {
                hi
            } else {
                continue;
            };
            qa += dv * dv;
            qb += dv * (o - bound);
        }
        if qa > 0.0 {
            best = best.min(at((-qb / qa).clamp(t0, t1)));
        }
    }
    best
}

/// The nearest point of the surface to `p` and its triangle.
fn nearest_point(surface: &Surface, p: Vec3) -> Option<(Vec3, usize)> {
    let best = Cell::new(f64::INFINITY);
    let found: Cell<Option<(Vec3, usize)>> = Cell::new(None);
    surface.visit(
        |b| {
            let d = best.get();
            box_dist2(b, p) <= d * d
        },
        |i, t| {
            let q = nearest_on_triangle(p, t);
            let d = q.dist(p);
            if d < best.get() {
                best.set(d);
                found.set(Some((q, i)));
            }
        },
    );
    found.get()
}

/// Counts samples against [`MAX_SAMPLES`].
struct Budget(usize);

impl Budget {
    fn take(&mut self, n: usize) -> Result<()> {
        self.0 = self.0.saturating_add(n);
        if self.0 > MAX_SAMPLES {
            return Err(CamError::TooLarge(format!("the finish samples the surface at more than {MAX_SAMPLES} points")));
        }
        Ok(())
    }
}

/// Number of equal pieces of at most `step` that `len` divides into, at least 1.
fn pieces(len: f64, step: f64) -> Result<usize> {
    let n = (len / step - 1e-9).ceil().max(1.0);
    if !(n <= MAX_SAMPLES as f64) {
        return Err(CamError::TooLarge(format!("more than {MAX_SAMPLES} points along one line")));
    }
    Ok(n as usize)
}

fn positive(v: f64, what: &str) -> Result<f64> {
    if !(v > 0.0) || !(v <= MAX_COORD) {
        return invalid(format!("{what} must be above 0 and at most 1000 m"));
    }
    Ok(v)
}

impl DrivePattern {
    /// The passes on the surface, and how points between their samples are found.
    fn passes(&self, surface: &Surface, budget: &mut Budget) -> Result<(Projector, Vec<Vec<Contact>>)> {
        match self {
            DrivePattern::Parallel { angle, stepover, step, boundary, direction } => {
                if !angle.is_finite() || !(angle.abs() <= 1.0e3) {
                    return invalid("the angle of the parallel lines must be a finite number");
                }
                let (stepover, step) = (positive(*stepover, "the stepover")?, positive(*step, "the step along a line")?);
                let d = unit(direction.unwrap_or(Vec3::new(0.0, 0.0, -1.0)), "the projection direction")?;
                let frame = Frame::from_z(Vec3::ZERO, -d)?;
                let b = surface.bounds();
                let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
                let mut top = f64::NEG_INFINITY;
                for i in 0..8 {
                    let c = Vec3::new(
                        if i & 1 == 0 { b.min.x } else { b.max.x },
                        if i & 2 == 0 { b.min.y } else { b.max.y },
                        if i & 4 == 0 { b.min.z } else { b.max.z },
                    );
                    let l = frame.to_local(c);
                    lo = Vec2::new(lo.x.min(l.x), lo.y.min(l.y));
                    hi = Vec2::new(hi.x.max(l.x), hi.y.max(l.y));
                    top = top.max(l.z);
                }
                if let Some([a, c]) = boundary {
                    if !a.is_finite()
                        || !c.is_finite()
                        || !(a.x < c.x && a.y < c.y)
                        || !(a.x.abs().max(c.x.abs()).max(a.y.abs()).max(c.y.abs()) <= MAX_COORD)
                    {
                        return invalid("the boundary must be the finite corners [min, max] of a rectangle, min below max");
                    }
                    (lo, hi) = (*a, *c);
                }
                let (along, side) = (Vec2::from_angle(*angle), Vec2::from_angle(*angle).perp());
                let (mut u0, mut u1, mut w0, mut w1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
                for c in [lo, hi, Vec2::new(lo.x, hi.y), Vec2::new(hi.x, lo.y)] {
                    (u0, u1) = (u0.min(c.dot(along)), u1.max(c.dot(along)));
                    (w0, w1) = (w0.min(c.dot(side)), w1.max(c.dot(side)));
                }
                let (lines, points) = (pieces(w1 - w0, stepover)?, pieces(u1 - u0, step)?);
                if lines + 1 > MAX_PASSES {
                    return Err(CamError::TooLarge(format!("more than {MAX_PASSES} passes")));
                }
                budget.take((lines + 1).saturating_mul(points + 1))?;
                let slack = 1e-9 * (1.0 + hi.x.abs().max(hi.y.abs()).max(lo.x.abs()).max(lo.y.abs()));
                let inside = |q: Vec2| q.x >= lo.x - slack && q.x <= hi.x + slack && q.y >= lo.y - slack && q.y <= hi.y + slack;
                let projector = Projector::Ray(d);
                let mut passes = Vec::new();
                for k in 0..=lines {
                    let w = w0 + (w1 - w0) * k as f64 / lines as f64;
                    let mut line: Vec<Option<Contact>> = Vec::with_capacity(points + 1);
                    for j in 0..=points {
                        let u = u0 + (u1 - u0) * j as f64 / points as f64;
                        let q = along * u + side * w;
                        line.push(if inside(q) { projector.project(surface, frame.to_world(Vec3::new(q.x, q.y, top + 1.0))) } else { None });
                    }
                    if k % 2 == 1 {
                        line.reverse();
                    }
                    split_runs(line, &mut passes);
                }
                Ok((projector, passes))
            }
            DrivePattern::Along { curves, step } => {
                let step = step.map(|s| positive(s, "the step along a curve")).transpose()?;
                if curves.len() > MAX_PASSES {
                    return Err(CamError::TooLarge(format!("more than {MAX_PASSES} curves")));
                }
                let projector = Projector::Nearest;
                let mut passes = Vec::new();
                for curve in curves {
                    budget.take(curve.len())?;
                    let mut line: Vec<Option<Contact>> = Vec::new();
                    for (k, p) in curve.iter().enumerate() {
                        finite_point(*p, "a point of a curve")?;
                        if let (Some(s), Some(prev)) = (step, k.checked_sub(1).and_then(|j| curve.get(j))) {
                            let n = pieces(prev.dist(*p), s)?;
                            budget.take(n)?;
                            for i in 1..n {
                                line.push(projector.project(surface, prev.lerp(*p, i as f64 / n as f64)));
                            }
                        }
                        line.push(projector.project(surface, *p));
                    }
                    split_runs(line, &mut passes);
                }
                Ok((projector, passes))
            }
        }
    }
}

/// Appends the runs of found points of `line` (two or more points each) to `passes`.
fn split_runs(line: Vec<Option<Contact>>, passes: &mut Vec<Vec<Contact>>) {
    let mut run = Vec::new();
    for c in line {
        match c {
            Some(c) => run.push(c),
            None => {
                if run.len() >= 2 {
                    passes.push(std::mem::take(&mut run));
                } else {
                    run.clear();
                }
            }
        }
    }
    if run.len() >= 2 {
        passes.push(run);
    }
}

/// Cutting data of a simultaneous 5-axis surface finish.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Finish5Params {
    pub feeds: Feeds,
    /// Most the tip may stray from the straight move between two poses (mm).
    pub tolerance: f64,
    /// Most the tool axis may turn in one cutting move (radians).
    pub max_axis_step: f64,
    /// Material left on the surface (mm).
    #[serde(default)]
    pub stock_to_leave: f64,
    /// How far up the tool axis from the surface the plunge feed starts and a pass is left (mm).
    pub retract: f64,
    /// Height of the plane the tool crosses between passes (work Z, mm): at least the tool
    /// radius above the surface's highest point.
    pub clearance: f64,
    /// Most the tool axis may lean from `tilt_reference` (radians); an axis leaning further is
    /// turned back towards it on the great circle between them. Left out, no limit.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_tilt: Option<f64>,
    /// The direction `max_tilt` is measured from; left out, +Z.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tilt_reference: Option<Vec3>,
}

fn check_axis_step(step: f64) -> Result<f64> {
    if !(step >= MIN_AXIS_STEP) || !(step <= std::f64::consts::PI) {
        return invalid("the largest axis step must be from 0.0001 rad to π");
    }
    Ok(step)
}

fn check_links(retract: f64, clearance: f64) -> Result<()> {
    if !(retract > 0.0) || !(retract <= 1.0e4) {
        return invalid("the retract distance must be above 0 and at most 10 m");
    }
    if !clearance.is_finite() || !(clearance.abs() <= MAX_COORD) {
        return invalid("the clearance height must be a finite number");
    }
    Ok(())
}

impl Finish5Params {
    pub fn validate(&self) -> Result<()> {
        self.feeds.validate()?;
        if !(self.tolerance > 0.0) || !(self.tolerance <= 100.0) {
            return invalid("the tolerance must be above 0 and at most 100 mm");
        }
        check_axis_step(self.max_axis_step)?;
        if !(self.stock_to_leave >= 0.0) || !(self.stock_to_leave <= 1.0e3) {
            return invalid("the stock to leave must be from 0 to 1000 mm");
        }
        check_links(self.retract, self.clearance)?;
        if let Some(m) = self.max_tilt
            && (!(m > 0.0) || !(m <= std::f64::consts::PI))
        {
            return invalid("the largest tilt must be above 0 and at most π");
        }
        if let Some(r) = self.tilt_reference {
            unit(r, "the tilt reference")?;
        }
        Ok(())
    }
}

/// `a` turned back towards the unit `reference` on the great circle until it leans at most `max`.
fn clamp_tilt(a: Vec3, reference: Vec3, max: f64) -> Vec3 {
    let angle = a.cross(reference).len().atan2(a.dot(reference));
    if angle <= max { a } else { slerp(reference, a, max / angle) }
}

/// A pose with the surface point it was placed for.
#[derive(Clone, Copy, Debug)]
struct Placed {
    c: Contact,
    pose: Pose,
}

/// Places the tip of `cutter` with the unit axis `a` so that, coming in along the axis, it stops
/// against the surface, starting from the line through the point that touches `c` on a cutter
/// of that shape: for a ball, tip = contact + r·n − r·a. `None` when nothing is in the way.
fn place(surface: &Surface, cutter: &Cutter, c: &Contact, a: Vec3) -> Result<Option<Pose>> {
    let n = c.normal;
    // The corner's centre is `corner` off the surface along the normal, and the axis is the flat
    // radius from it, towards where the normal leans (seen along the axis).
    let mut line = c.at + n * cutter.corner;
    if let Some(lean) = across(n, a) {
        line += lean * cutter.flat_radius();
    }
    let start = line - a * cutter.corner;
    Ok(drop_along(surface, cutter, start, a)?.map(|t| Pose::new(start + a * t, a)))
}

/// The pose with axis `a` turned evenly between two placed poses, placed against the surface; in
/// the air when nothing is in the way.
fn between(surface: &Surface, cutter: &Cutter, p: &Pose, q: &Pose, t: f64) -> Result<Pose> {
    let a = slerp(p.axis, q.axis, t);
    // A ball turns about its centre; the others from the tip.
    let start = if cutter.is_ball() {
        let r = cutter.radius;
        p.up(r).lerp(q.up(r), t) - a * r
    } else {
        p.tip.lerp(q.tip, t)
    };
    Ok(match drop_along(surface, cutter, start, a)? {
        Some(s) => Pose::new(start + a * s, a),
        None => Pose::new(start, a),
    })
}

/// Fills in poses turned evenly between neighbours whose axes differ by more than `max_step`.
fn fill_turns(surface: &Surface, cutter: &Cutter, poses: &[Pose], max_step: f64, budget: &mut Budget) -> Result<Vec<Pose>> {
    let mut out = Vec::with_capacity(poses.len());
    for (k, q) in poses.iter().enumerate() {
        if let Some(p) = k.checked_sub(1).and_then(|j| poses.get(j)) {
            let angle = p.axis_angle(q);
            if angle > max_step {
                let n = (angle / max_step).ceil() as usize;
                budget.take(n)?;
                for i in 1..n {
                    out.push(between(surface, cutter, p, q, i as f64 / n as f64)?);
                }
            }
        }
        out.push(*q);
    }
    Ok(out)
}

struct Finisher<'a> {
    surface: &'a Surface,
    /// The cutter grown by the stock to leave.
    cutter: Cutter,
    control: AxisControl,
    limit: Option<(Vec3, f64)>,
    projector: Projector,
    tol: f64,
    max_step: f64,
    budget: Budget,
    /// The test for the grown cutter cutting into the surface by more than the tolerance.
    gouge: GougeTest,
    /// How finely each move is checked with it (mm).
    check_step: f64,
}

impl Finisher<'_> {
    /// The pose for the contact `c`, the tool moving along `feed`; `None` where the axis cannot
    /// reach the point from outside or nothing stops the tool.
    fn sample(&self, c: Contact, feed: Vec3) -> Result<Option<Placed>> {
        let Some(mut a) = self.control.axis(c.at, c.normal, feed) else { return Ok(None) };
        if let Some((reference, max)) = self.limit {
            a = clamp_tilt(a, reference, max);
        }
        if !(a.dot(c.normal) > MIN_REACH) {
            return Ok(None);
        }
        Ok(place(self.surface, &self.cutter, &c, a)?.map(|pose| Placed { c, pose }))
    }

    /// The poses of one pass, in the pieces it falls apart into where points are missing.
    fn pass(&mut self, contacts: &[Contact]) -> Result<Vec<Vec<Pose>>> {
        let mut placed: Vec<Option<Placed>> = Vec::with_capacity(contacts.len());
        for (k, c) in contacts.iter().enumerate() {
            let prev = k.checked_sub(1).and_then(|j| contacts.get(j)).unwrap_or(c);
            let next = contacts.get(k + 1).unwrap_or(c);
            placed.push(self.sample(*c, next.at - prev.at)?);
        }
        let mut pieces: Vec<Vec<Pose>> = Vec::new();
        let mut last: Option<Placed> = None;
        for p in placed {
            match (p, last) {
                (Some(b), Some(a)) => self.refine(&a, &b, MAX_DEPTH, &mut pieces)?,
                (Some(b), None) => pieces.push(vec![b.pose]),
                (None, _) => {}
            }
            last = p;
        }
        let mut out = Vec::with_capacity(pieces.len());
        for piece in pieces {
            if piece.len() >= 2 {
                let turned = fill_turns(self.surface, &self.cutter, &piece, self.max_step, &mut self.budget)?;
                out.extend(self.clear(&turned)?);
            }
        }
        Ok(out)
    }

    /// The piece with every move along which the cutter cuts into the surface (a feature between
    /// its poses that no sample landed on) halved, a pose placed between; where halving does not
    /// help, the move is left out and the piece split there, for the tool to be lifted over.
    fn clear(&mut self, piece: &[Pose]) -> Result<Vec<Vec<Pose>>> {
        let mut out = Vec::new();
        let mut run: Vec<Pose> = Vec::with_capacity(piece.len());
        let mut last: Option<Pose> = None;
        for q in piece {
            match last {
                Some(p) => self.clear_move(&p, q, MAX_DEPTH, &mut run, &mut out)?,
                None => run.push(*q),
            }
            last = Some(*q);
        }
        if run.len() >= 2 {
            out.push(run);
        }
        Ok(out)
    }

    /// Adds the move from `p` (the last pose of `run`) to `q` to `run`, as [`Finisher::clear`]
    /// does.
    fn clear_move(&mut self, p: &Pose, q: &Pose, depth: u32, run: &mut Vec<Pose>, out: &mut Vec<Vec<Pose>>) -> Result<()> {
        if !self.cuts_in(p, q)? {
            run.push(*q);
            return Ok(());
        }
        if depth == 0 {
            let done = std::mem::replace(run, vec![*q]);
            if done.len() >= 2 {
                out.push(done);
            }
            return Ok(());
        }
        self.budget.take(1)?;
        let m = between(self.surface, &self.cutter, p, q, 0.5)?;
        self.clear_move(p, &m, depth - 1, run, out)?;
        self.clear_move(&m, q, depth - 1, run, out)
    }

    /// Whether the cutter cuts into the surface by more than the tolerance between the placed
    /// poses `p` and `q`, the tip moving straight and the axis turning evenly.
    fn cuts_in(&mut self, p: &Pose, q: &Pose) -> Result<bool> {
        let n = pieces(p.tip.dist(q.tip) + p.axis_angle(q) * self.cutter.radius, self.check_step)?.max(2);
        self.budget.take(n)?;
        for k in 1..n {
            let t = k as f64 / n as f64;
            if self.gouge.depth(self.surface, &Pose::new(p.tip.lerp(q.tip, t), slerp(p.axis, q.axis, t)))?.is_some() {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Adds `b` to the last piece, first putting in points between `a` and `b` where the tip
    /// strays from the straight move or the axis turns too far; starts a new piece at `b` where
    /// the surface between them cannot be followed.
    fn refine(&mut self, a: &Placed, b: &Placed, depth: u32, pieces: &mut Vec<Vec<Pose>>) -> Result<()> {
        let push = |pieces: &mut Vec<Vec<Pose>>, p: Pose| match pieces.last_mut() {
            Some(last) => last.push(p),
            None => pieces.push(vec![p]),
        };
        // Points this close together are not divided further: the axis is turned between them
        // afterwards, and the tip moves along it.
        if a.c.at.dist(b.c.at) <= self.tol * 0.5 {
            push(pieces, b.pose);
            return Ok(());
        }
        self.budget.take(1)?;
        let mid = self.projector.project(self.surface, a.c.seed.lerp(b.c.seed, 0.5));
        let Some(m) = (match mid {
            Some(c) => self.sample(c, b.c.at - a.c.at)?,
            None => None,
        }) else {
            pieces.push(vec![b.pose]);
            return Ok(());
        };
        let strays = m.pose.tip.dist(a.pose.tip.lerp(b.pose.tip, 0.5)) > self.tol;
        let turns = a.pose.axis_angle(&b.pose) > self.max_step;
        if !strays && !turns {
            push(pieces, b.pose);
            return Ok(());
        }
        if depth == 0 {
            if strays {
                pieces.push(vec![b.pose]);
            } else {
                push(pieces, b.pose);
            }
            return Ok(());
        }
        self.refine(a, &m, depth - 1, pieces)?;
        self.refine(&m, b, depth - 1, pieces)
    }
}

/// How far along the ray from `o` along `d` it leaves the box for good, also when `o` is
/// outside it and the ray only meets the box farther on; 0 when the ray does not meet it ahead.
fn ray_exit(b: &Aabb3, o: Vec3, d: Vec3) -> f64 {
    let (mut t0, mut t1) = (0.0f64, f64::INFINITY);
    for (o, d, lo, hi) in [(o.x, d.x, b.min.x, b.max.x), (o.y, d.y, b.min.y, b.max.y), (o.z, d.z, b.min.z, b.max.z)] {
        if d.abs() <= 1e-15 {
            if o < lo || o > hi {
                return 0.0;
            }
            continue;
        }
        let (u, v) = ((lo - o) / d, (hi - o) / d);
        t0 = t0.max(u.min(v));
        t1 = t1.min(u.max(v));
    }
    if t0 <= t1 && t1.is_finite() { t1.max(0.0) } else { 0.0 }
}

/// Builds the moves of passes and the links between them (see the module's documentation).
struct Linker {
    /// What the tool must have left for good before it rises straight up: the bounds of the
    /// surface, grown by the tool radius and 1 mm. Past it along the axis (which points up or
    /// level), every point of the cutter, endless above its end, is beside or above the bounds,
    /// and stays so rising straight up.
    zone: Aabb3,
    clearance: f64,
    retract: f64,
    feeds: Feeds,
    moves: Vec<Move5>,
}

impl Linker {
    fn new(bounds: Aabb3, radius: f64, clearance: f64, retract: f64, feeds: Feeds) -> Result<Linker> {
        if !(clearance >= bounds.max.z + radius) {
            return invalid("the clearance plane must be at least the tool radius above the highest point of what is cut");
        }
        Ok(Linker { zone: bounds.inflated(radius + 1.0), clearance, retract, feeds, moves: Vec::new() })
    }

    /// Where the tool, leaving `p` along its axis, stops before rising straight up: on the
    /// clearance plane (where the cutter is above the bounds) or past the zone, whichever comes
    /// first, and never before the retract distance. The tool came in along this line, so it is
    /// clear of the surface all the way.
    fn exit(&self, p: &Pose) -> Result<Vec3> {
        let a = p.axis;
        if a.z < -1e-9 {
            return Err(CamError::Unsupported(
                "a pass ends with the tool axis pointing below the horizontal, which it cannot be lifted out along".into(),
            ));
        }
        let to_plane = if a.z > 1e-12 { (self.clearance - p.tip.z) / a.z } else { f64::INFINITY };
        let s = to_plane.min(ray_exit(&self.zone, p.tip, a)).max(self.retract);
        let e = p.up(s);
        if !e.is_finite() || !(e.x.abs() <= MAX_COORD && e.y.abs() <= MAX_COORD && e.z.abs() <= MAX_COORD) {
            return Err(CamError::TooLarge("the way out of a pass along the tool axis leaves the 1000 m work space".into()));
        }
        Ok(e)
    }

    fn push(&mut self, m: Move5) -> Result<()> {
        if self.moves.len() >= MAX_MOVES {
            return Err(CamError::TooLarge(format!("the toolpath has more than {MAX_MOVES} moves")));
        }
        self.moves.push(m);
        Ok(())
    }

    fn pass(&mut self, poses: &[Pose]) -> Result<()> {
        let (Some(first), Some(last)) = (poses.first(), poses.last()) else { return Ok(()) };
        let into = self.exit(first)?;
        let a = first.axis;
        self.push(Move5::Rapid { to: Pose::new(Vec3::new(into.x, into.y, into.z.max(self.clearance)), a) })?;
        if into.z < self.clearance {
            self.push(Move5::Rapid { to: Pose::new(into, a) })?;
        }
        let near = first.up(self.retract);
        if into.dist(near) > 1e-9 {
            self.push(Move5::Rapid { to: Pose::new(near, a) })?;
        }
        self.push(Move5::Feed { to: *first, feed: self.feeds.plunge })?;
        for p in poses.iter().skip(1) {
            self.push(Move5::Feed { to: *p, feed: self.feeds.cut })?;
        }
        let out = self.exit(last)?;
        let near = last.up(self.retract);
        self.push(Move5::Rapid { to: Pose::new(near, last.axis) })?;
        if out.dist(near) > 1e-9 {
            self.push(Move5::Rapid { to: Pose::new(out, last.axis) })?;
        }
        if out.z < self.clearance {
            self.push(Move5::Rapid { to: Pose::new(Vec3::new(out.x, out.y, self.clearance), last.axis) })?;
        }
        Ok(())
    }
}

/// A simultaneous 5-axis finish of `surface` along `pattern`, the axis set by `control` (and
/// held within `params.max_tilt`), every pose placed against the surface by the drop-cutter so
/// that none cuts into it, and every move checked not to cut into it by more than the tolerance
/// between its poses (see the module's documentation). Checked with [`verify5`], and its links
/// for gouges, before it is returned; a link that would gouge is refused with
/// [`CamError::Unsafe`].
pub fn surface_finish(surface: &Surface, tool: &Tool, pattern: &DrivePattern, control: &AxisControl, params: &Finish5Params) -> Result<Toolpath5> {
    tool.validate()?;
    params.validate()?;
    control.validate()?;
    if surface.is_empty() {
        return invalid("the surface to finish has no triangles");
    }
    let cutter = tool.cutter()?.grown(params.stock_to_leave)?;
    let reference = unit(params.tilt_reference.unwrap_or(Vec3::new(0.0, 0.0, 1.0)), "the tilt reference")?;
    let mut budget = Budget(0);
    let (projector, passes) = pattern.passes(surface, &mut budget)?;
    // Fine enough that a sample lands on anything the cutter would cut between two poses, and
    // that the cutters between two samples sweep within the tolerance of their outline.
    let check_step = (cutter.radius * 0.5).min((8.0 * cutter.radius * params.tolerance).sqrt());
    let mut f = Finisher {
        surface,
        cutter,
        control: *control,
        limit: params.max_tilt.map(|m| (reference, m)),
        projector,
        tol: params.tolerance,
        max_step: params.max_axis_step,
        budget,
        gouge: GougeTest::new(&cutter, params.tolerance)?,
        check_step,
    };
    let mut links = Linker::new(surface.bounds(), cutter.radius, params.clearance, params.retract, params.feeds)?;
    let stock = params.stock_to_leave;
    for pass in &passes {
        for piece in f.pass(pass)? {
            // The real tool's tip is the stock above the grown one's, up its axis.
            let real: Vec<Pose> = piece.iter().map(|p| Pose::new(p.up(stock), p.axis)).collect();
            links.pass(&real)?;
        }
    }
    if links.moves.is_empty() {
        return invalid("the drive pattern does not reach the surface anywhere the tool axis can come from");
    }
    let path = Toolpath5 { moves: links.moves };
    verify5(&path, params.max_axis_step)?;
    // The links, checked as the passes were: the grown cutter, its tip the stock lower.
    let lower = |p: Pose| Pose::new(p.up(-stock), p.axis);
    let grown = Toolpath5 {
        moves: path
            .moves
            .iter()
            .map(|m| match *m {
                Move5::Rapid { to } => Move5::Rapid { to: lower(to) },
                Move5::Feed { to, feed } => Move5::Feed { to: lower(to), feed },
                other => other,
            })
            .collect(),
    };
    if let Some(h) = gouges_in(surface, &cutter, &grown, params.tolerance, check_step, Moves::Links)?.first() {
        return Err(CamError::Unsafe(format!("move {}: a link between passes cuts {:.4} mm into the surface", h.index, h.distance)));
    }
    Ok(path)
}

/// Which side of a swarf wall the tool runs on, walking along the rails from their first points
/// with the rulings pointing up.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WallSide {
    Left,
    Right,
}

/// Cutting data of a swarf cut.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SwarfParams {
    pub feeds: Feeds,
    /// Material left on the wall (mm).
    #[serde(default)]
    pub stock_to_leave: f64,
    /// How far below the bottom rail the tip runs, down the ruling (mm); negative: above it.
    #[serde(default)]
    pub bottom_offset: f64,
    /// Depth passes, evenly spaced down the rulings, the last at full depth (1: one pass).
    pub levels: u32,
    /// Most the tool axis may turn in one move (radians).
    pub max_axis_step: f64,
    /// How far up the tool axis the plunge feed starts and a pass is left (mm).
    pub retract: f64,
    /// Height of the plane the tool crosses between passes (work Z, mm): at least the tool
    /// radius above the wall's highest point.
    pub clearance: f64,
}

/// A swarf cut and how far its tool strays from the wall.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Swarf {
    pub path: Toolpath5,
    /// The larger of `max_overcut` and `max_undercut` (mm).
    pub max_deviation: f64,
    /// Deepest the tool's cylinder reaches into the wall (offset by the stock) at any pose of the
    /// full-depth pass, or halfway along a move of it (mm).
    pub max_overcut: f64,
    /// Farthest the cylinder stays off the ruling it is meant to touch (mm).
    pub max_undercut: f64,
    /// The overcut at each ruling of the rails, in order (mm).
    pub overcut: Vec<f64>,
}

/// A station of a swarf cut: a ruling, at `lambda` of the way along span `span` of the rails.
#[derive(Clone, Copy, Debug)]
struct Station {
    span: usize,
    lambda: f64,
    /// Index of the rails' ruling when the station is one.
    ruling: Option<usize>,
    bottom: Vec3,
    up: Vec3,
    length: f64,
    /// Where the tool's axis is off the ruling, per mm of the tool's radius, towards the tool:
    /// the wall's unit normal, or at a concave corner the way to where the offsets of the two
    /// faces meet.
    side: Vec3,
}

/// The ruled wall between two rails.
struct Wall<'a> {
    bottom: &'a [Vec3],
    top: &'a [Vec3],
    /// The box of each span (its four corners), and a hierarchy over them.
    boxes: Vec<Aabb3>,
    bvh: Bvh,
    /// Spans measured so far, against [`MAX_LOOKS`].
    looks: Cell<usize>,
}

impl<'a> Wall<'a> {
    fn new(bottom: &'a [Vec3], top: &'a [Vec3]) -> Wall<'a> {
        let boxes: Vec<Aabb3> = bottom
            .windows(2)
            .zip(top.windows(2))
            .map(|(b, t)| {
                let mut bx = Aabb3::EMPTY;
                for p in b.iter().chain(t) {
                    bx.add(*p);
                }
                bx
            })
            .collect();
        let bvh = Bvh::new(&boxes);
        Wall { bottom, top, boxes, bvh, looks: Cell::new(0) }
    }

    fn spans(&self) -> usize {
        self.bottom.len().saturating_sub(1)
    }

    /// The ruling at `lambda` along span `k`.
    fn ruling(&self, k: usize, lambda: f64) -> Option<(Vec3, Vec3)> {
        let (b0, b1, t0, t1) = (self.bottom.get(k)?, self.bottom.get(k + 1)?, self.top.get(k)?, self.top.get(k + 1)?);
        Some((b0.lerp(*b1, lambda), t0.lerp(*t1, lambda)))
    }

    /// The straight line across span `k` at `v` of the way up the wall.
    fn level(&self, k: usize, v: f64) -> Option<(Vec3, Vec3)> {
        let (b0, b1, t0, t1) = (self.bottom.get(k)?, self.bottom.get(k + 1)?, self.top.get(k)?, self.top.get(k + 1)?);
        Some((b0.lerp(*t0, v), b1.lerp(*t1, v)))
    }

    /// The overcut and undercut of the tool at `pose` (its cylinder `radius`, `reach` long) meant
    /// to touch the ruling at `lambda` along span `span`.
    fn compare(&self, pose: &Pose, span: usize, lambda: f64, radius: f64, reach: f64) -> Result<(f64, f64)> {
        let (a0, a1) = (pose.tip, pose.up(reach));
        let (mut over, mut under) = (0.0f64, 0.0f64);
        if let Some((b, t)) = self.ruling(span, lambda) {
            for i in 0..=ACROSS {
                let d = b.lerp(t, i as f64 / ACROSS as f64).dist_to_segment(a0, a1);
                over = over.max(radius - d);
                under = under.max(d - radius);
            }
        }
        // Every span whose box comes within the radius of the cylinder's axis: no other can
        // reach into the cylinder.
        let r2 = radius * radius;
        let mut looked = 0usize;
        self.bvh.visit(
            |bx| segment_box_dist2(a0, a1, bx) < r2,
            |k| {
                let k = k as usize;
                if !self.boxes.get(k).is_some_and(|bx| segment_box_dist2(a0, a1, bx) < r2) {
                    return;
                }
                looked += 1;
                for i in 0..=ACROSS {
                    if let Some((p, q)) = self.level(k, i as f64 / ACROSS as f64) {
                        let (x, y) = nearest_segments(p, q, a0, a1);
                        over = over.max(radius - x.dist(y));
                    }
                }
            },
        );
        let looks = self.looks.get().saturating_add(looked);
        if looks > MAX_LOOKS {
            return Err(CamError::TooLarge(format!("measuring the swarf cut's deviation looks at more than {MAX_LOOKS} spans")));
        }
        self.looks.set(looks);
        Ok((over, under))
    }
}

/// The unit direction of `b - a`, or zero when they coincide.
fn dir_or_zero(a: Vec3, b: Vec3) -> Vec3 {
    (b - a).normalized().unwrap_or(Vec3::ZERO)
}

/// Flank milling of the ruled wall between the rails `bottom` and `top` (matching points; each
/// pair a straight ruling, bottom to top): the tool axis along each ruling, the tool on `side`
/// of the wall, its axis the tool radius plus the stock off the ruling along the wall's normal
/// there (taken square to the ruling and to the span's own direction along the rails, so that
/// the tool stays parallel to a planar face), its tip `params.bottom_offset` below the bottom
/// rail. Where two spans meet at an angle, the tool rolls round the ruling outside the corner
/// (in steps of at most the axis step) and stops where it touches both faces inside it. Where
/// the axis would turn more than the step allowed between rulings, rulings in between are cut
/// too. Refused when a ruling is longer than the flutes. The deviation of the tool's cylinder
/// from the wall is measured at every pose of the full-depth pass and halfway along every move
/// of it.
pub fn swarf(bottom: &[Vec3], top: &[Vec3], tool: &Tool, side: WallSide, params: &SwarfParams) -> Result<Swarf> {
    tool.validate()?;
    params.feeds.validate()?;
    let max_step = check_axis_step(params.max_axis_step)?;
    check_links(params.retract, params.clearance)?;
    if !(params.stock_to_leave >= 0.0) || !(params.stock_to_leave <= 1.0e3) {
        return invalid("the stock to leave must be from 0 to 1000 mm");
    }
    if !params.bottom_offset.is_finite() || !(params.bottom_offset.abs() <= 1.0e4) {
        return invalid("the bottom offset must be a finite number of at most 10 m");
    }
    if !(1..=MAX_LEVELS).contains(&params.levels) {
        return invalid(format!("a swarf cut has 1 to {MAX_LEVELS} depth passes"));
    }
    if bottom.len() != top.len() {
        return invalid("the two rails of a swarf wall must have the same number of points");
    }
    if bottom.len() < 2 {
        return invalid("a swarf wall needs at least two rulings");
    }
    if bottom.len() > MAX_RULINGS {
        return Err(CamError::TooLarge(format!("a swarf wall has more than {MAX_RULINGS} rulings")));
    }
    for p in bottom.iter().chain(top) {
        finite_point(*p, "a rail point")?;
    }
    let radius = tool.cutter()?.radius + params.stock_to_leave;
    let sign = if side == WallSide::Left { 1.0 } else { -1.0 };
    let wall = Wall::new(bottom, top);
    for (i, (b, t)) in bottom.iter().zip(top).enumerate() {
        if !(b.dist(*t) > 1e-6) {
            return invalid(format!("ruling {i} of the swarf wall has no length"));
        }
        if !(b.dist(*t) + params.bottom_offset.max(0.0) <= tool.flute_length + 1e-9) {
            return invalid(format!("ruling {i} of the swarf wall is longer than the tool's flutes"));
        }
    }
    // Each span's own direction along the rails: a span of a wall made of planar faces is one
    // of them, and the tool must stay parallel to it all along.
    let along: Vec<Vec3> = bottom
        .windows(2)
        .zip(top.windows(2))
        .map(|(b, t)| match (b, t) {
            ([b0, b1], [t0, t1]) => dir_or_zero(*b0, *b1) + dir_or_zero(*t0, *t1),
            _ => Vec3::ZERO,
        })
        .collect();
    // The unit direction of span `span` along the rails, square to `up`.
    let span_dir = |span: usize, up: Vec3, near: usize| -> Result<Vec3> {
        match along.get(span).and_then(|e| across(*e, up)) {
            Some(e) => Ok(e),
            None => invalid(format!("the swarf wall runs along its rulings near ruling {near}")),
        }
    };
    let station = |span: usize, lambda: f64, ruling: Option<usize>| -> Result<Station> {
        let Some((b, t)) = wall.ruling(span, lambda) else {
            return invalid("a swarf station lies outside the rails");
        };
        let up = unit(t - b, "a ruling of the swarf wall")?;
        let e = span_dir(span, up, ruling.unwrap_or(span))?;
        Ok(Station { span, lambda, ruling, bottom: b, up, length: b.dist(t), side: up.cross(e) * sign })
    };
    // The stations at the ruling where the span of `end` meets that of `start`, `end` and
    // `start` being their stations there, and where the offsets of the two faces meet when the
    // corner is concave (seen from the tool).
    let corner = |end: &Station, start: &Station| -> Result<(Vec<Station>, Option<Vec3>)> {
        let (n0, n1, up) = (end.side, start.side, start.up);
        let phi = n0.cross(n1).len().atan2(n0.dot(n1));
        if phi <= 1e-9 {
            return Ok((vec![*start], None));
        }
        let near = start.ruling.unwrap_or(start.span);
        let (t0, t1) = (span_dir(end.span, up, near)?, span_dir(start.span, up, near)?);
        if n1.dot(t0) - n0.dot(t1) > 0.0 {
            // Outside the corner: the tool rolls round the ruling, its axis the radius off it,
            // the side turning about the ruling the way the rails go on.
            let turn = if up.cross(n0).dot(t0) >= 0.0 { phi } else { -phi };
            // At most π / MIN_AXIS_STEP stations.
            let m = (phi / max_step).ceil().max(1.0);
            if m <= 1.0 {
                return Ok((vec![Station { side: rotate(n0, up, turn * 0.5), ..*start }], None));
            }
            let m = m as usize;
            let roll = (0..=m).map(|j| Station { side: if j == m { n1 } else { rotate(n0, up, turn * j as f64 / m as f64) }, ..*start }).collect();
            Ok((roll, None))
        } else {
            // Inside it: where the two faces' offsets meet, the radius from each.
            let d = 1.0 + n0.dot(n1);
            if !(d > 1e-6) {
                return invalid(format!("the swarf wall folds back on itself at ruling {near}"));
            }
            let side = (n0 + n1) * (1.0 / d);
            Ok((vec![Station { side, ..*start }], Some(start.bottom + side * radius)))
        }
    };
    // Stations: every ruling (several round a convex corner), and as many evenly between as the
    // axis steps need.
    let spans = wall.spans();
    let ends: Vec<(Station, Station)> = (0..spans).map(|k| Ok((station(k, 0.0, Some(k))?, station(k, 1.0, Some(k + 1))?))).collect::<Result<_>>()?;
    let mut stations: Vec<Station> = Vec::new();
    let mut before: Option<Vec3> = None;
    for (span, (s0, s1)) in ends.iter().enumerate() {
        match span.checked_sub(1).and_then(|k| ends.get(k)) {
            Some((_, end)) => {
                let (at, concave) = corner(end, s0)?;
                if stations.len().saturating_add(at.len()) > MAX_SAMPLES {
                    return Err(CamError::TooLarge(format!("a swarf cut of more than {MAX_SAMPLES} rulings")));
                }
                stations.extend(at);
                before = concave;
            }
            None => stations.push(*s0),
        }
        let after = match ends.get(span + 1) {
            Some((next, _)) => corner(s1, next)?.1,
            None => None,
        };
        let turn = s0.up.cross(s1.up).len().atan2(s0.up.dot(s1.up)).max(s0.side.cross(s1.side).len().atan2(s0.side.dot(s1.side)));
        let mut m = ((turn / max_step).ceil() as usize).max(1);
        let mut inner: Vec<Station>;
        loop {
            if stations.len().saturating_add(m) > MAX_SAMPLES {
                return Err(CamError::TooLarge(format!("a swarf cut of more than {MAX_SAMPLES} rulings")));
            }
            inner = (1..m).map(|k| station(span, k as f64 / m as f64, None)).collect::<Result<_>>()?;
            let ok = std::iter::once(s0).chain(&inner).chain(std::iter::once(s1)).collect::<Vec<_>>().windows(2).all(|w| match w {
                [p, q] => p.up.cross(q.up).len().atan2(p.up.dot(q.up)) <= max_step,
                _ => true,
            });
            if ok {
                break;
            }
            m *= 2;
        }
        // Next to a concave corner, the tool stops where it meets the other face's offset.
        let e = s0.up.cross(s0.side) * -sign;
        inner.retain(|s| {
            let at = s.bottom + s.side * radius;
            before.is_none_or(|c| (at - c).dot(e) > 0.0) && after.is_none_or(|c| (at - c).dot(e) < 0.0)
        });
        stations.extend(inner);
        if span + 1 == spans {
            stations.push(*s1);
        }
    }
    let n = params.levels;
    let pose = |s: &Station, level: u32| -> Pose {
        let raise = s.length * f64::from(n - level) / f64::from(n);
        Pose::new(s.bottom + s.side * radius + s.up * (raise - params.bottom_offset), s.up)
    };
    let mut bounds = Aabb3::EMPTY;
    for p in bottom.iter().chain(top) {
        bounds.add(*p);
    }
    let mut links = Linker::new(bounds, radius, params.clearance, params.retract, params.feeds)?;
    for level in 1..=n {
        let poses: Vec<Pose> = stations.iter().map(|s| pose(s, level)).collect();
        links.pass(&poses)?;
    }
    // How far the full-depth pass strays from the wall.
    let reach = tool.flute_length;
    let (mut max_over, mut max_under) = (0.0f64, 0.0f64);
    let mut overcut = vec![0.0f64; bottom.len()];
    let mut prev: Option<(&Station, Pose)> = None;
    for s in &stations {
        let p = pose(s, n);
        let (over, under) = wall.compare(&p, s.span, s.lambda, radius, reach)?;
        if let Some(slot) = s.ruling.and_then(|i| overcut.get_mut(i)) {
            *slot = slot.max(over);
        }
        (max_over, max_under) = (max_over.max(over), max_under.max(under));
        if let Some((ps, pp)) = prev {
            let mid = Pose::new(pp.tip.lerp(p.tip, 0.5), slerp(pp.axis, p.axis, 0.5));
            let lambda = if s.span == ps.span { (ps.lambda + s.lambda) * 0.5 } else { (ps.lambda + 1.0) * 0.5 };
            let (over, under) = wall.compare(&mid, ps.span, lambda, radius, reach)?;
            (max_over, max_under) = (max_over.max(over), max_under.max(under));
        }
        prev = Some((s, p));
    }
    let path = Toolpath5 { moves: links.moves };
    verify5(&path, max_step)?;
    Ok(Swarf { path, max_deviation: max_over.max(max_under), max_overcut: max_over, max_undercut: max_under, overcut })
}
