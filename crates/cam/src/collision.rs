//! Collision checks for multi-axis toolpaths: the tool's shank and holder against the part and
//! fixtures, and tilting the tool away where they would touch.
//!
//! The checks work on any [`Toolpath5`], whatever the tool axis: 3-axis paths turned into one,
//! indexed and rotary ones, and simultaneous 5-axis ones. Each cutting move is sampled finely
//! enough that neither the tip nor any point of the tool up to the top of what is checked moves
//! more than the sample step between samples: the tip on its straight line, the axis turning
//! evenly between the two directions, as the move is cut. [`gouges`] and [`holder_collisions`]
//! do not sample rapid moves: they are the links' business, planned clear of the part.
//! [`tilt_away`], which changes the axis on the rapids as well, checks every move it returns.
//!
//! A gouge is the cutter deeper in the surface than the tolerance: the cutter shrunk by the
//! tolerance must not meet it. A tool that only touches a wall beside it, or is within the
//! tolerance of it sideways, does not gouge it.

use std::cell::Cell;

use serde::{Deserialize, Serialize};
use solidcore_record::Vec3;
use solidcraft_geom::prelude::*;

use crate::cutter::Cutter;
use crate::dropcutter::drop_along;
use crate::multiaxis::{Move5, Pose, Toolpath5, slerp, verify5};
use crate::path::MAX_MOVES;
use crate::surface::{Surface, capsule_meets_box, nearest_segments, segment_triangle_distance};
use crate::tool::{Envelope, Tool};
use crate::{CamError, Result, invalid};

/// Most samples one check takes along a toolpath.
const MAX_SAMPLES: usize = 20_000_000;
/// Smallest tilt increment accepted (radians).
const MIN_TILT_STEP: f64 = 1e-4;

/// Where a toolpath makes the tool touch what it must not.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Hit {
    /// The move (index into the path's moves) along which it happens.
    pub index: usize,
    /// The worst pose found along that move.
    pub pose: Pose,
    /// Which of [`Tool::envelopes`] touches; `None` for the cutter itself (a gouge).
    pub envelope: Option<usize>,
    /// For a gouge, how much farther out along the axis the tool would have to be (mm): for the
    /// cutter shrunk by the tolerance to clear the surface, plus the tolerance. For an
    /// envelope, the gap between its cylinder and the nearest triangle (mm), below the clearance
    /// asked for; 0 or less where they touch or cross.
    pub distance: f64,
}

fn check_step(step: f64) -> Result<()> {
    if !(step > 0.0) || !(step <= 1.0e6) {
        return invalid("the sample step must be above 0 and at most 1000 m");
    }
    Ok(())
}

fn check_path(path: &Toolpath5) -> Result<()> {
    if path.moves.len() > MAX_MOVES {
        return Err(CamError::TooLarge(format!("the toolpath has more than {MAX_MOVES} moves")));
    }
    for (i, m) in path.moves.iter().enumerate() {
        if m.target().is_some_and(|p| !p.is_valid()) {
            return invalid(format!("move {i}: a pose is not finite or its tool axis is not a unit direction"));
        }
    }
    Ok(())
}

/// Number of samples along a move from `from` to `to` for points up to `reach` up the tool.
fn samples_along(from: &Pose, to: &Pose, reach: f64, step: f64) -> Result<usize> {
    let len = from.tip.dist(to.tip) + from.axis_angle(to) * reach;
    let n = (len / step - 1e-9).ceil().max(1.0);
    if !(n <= MAX_SAMPLES as f64) {
        return Err(CamError::TooLarge(format!("more than {MAX_SAMPLES} samples along one move")));
    }
    Ok(n as usize)
}

/// Which moves of a toolpath a check samples.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Moves {
    /// The cutting moves.
    Cutting,
    /// The links: the rapid moves, and each cutting move that ends one.
    Links,
    /// Every move.
    All,
}

/// Calls `f` with the move index and pose of every sample along the moves `moves` selects,
/// each from the pose before it. The start of a run of such moves is sampled too, as the first
/// move's.
fn for_each_sample(path: &Toolpath5, reach: f64, step: f64, moves: Moves, mut f: impl FnMut(usize, Pose) -> Result<()>) -> Result<()> {
    let mut prev: Option<Pose> = None;
    // Whether the move to `prev` was sampled, and whether it was a rapid.
    let (mut sampled, mut after_rapid) = (false, false);
    let mut total = 0usize;
    for (i, m) in path.moves.iter().enumerate() {
        let (to, rapid) = match *m {
            Move5::Feed { to, .. } => (to, false),
            Move5::Rapid { to } => (to, true),
            Move5::Dwell { .. } => continue,
        };
        let checked = match moves {
            Moves::Cutting => !rapid,
            Moves::Links => rapid || after_rapid,
            Moves::All => true,
        };
        if checked {
            match prev {
                Some(from) => {
                    let n = samples_along(&from, &to, reach, step)?;
                    total = total.saturating_add(n + 1);
                    if total > MAX_SAMPLES {
                        return Err(CamError::TooLarge(format!("more than {MAX_SAMPLES} samples along the toolpath")));
                    }
                    for k in usize::from(sampled)..=n {
                        let t = k as f64 / n as f64;
                        f(i, Pose::new(from.tip.lerp(to.tip, t), slerp(from.axis, to.axis, t)))?;
                    }
                }
                None => f(i, to)?,
            }
        }
        (sampled, after_rapid, prev) = (checked, rapid, Some(to));
    }
    Ok(())
}

/// What a gouge is: the cutter shrunk by the tolerance, which the surface meets only where the
/// cutter is deeper in it than that.
#[derive(Clone, Copy, Debug)]
pub(crate) struct GougeTest {
    /// The cutter shrunk all round, its tip `lift` up the axis.
    inner: Cutter,
    lift: f64,
    tol: f64,
}

impl GougeTest {
    pub(crate) fn new(cutter: &Cutter, tol: f64) -> Result<GougeTest> {
        cutter.validate()?;
        if !(tol >= 0.0) || !(tol <= 1.0e3) {
            return invalid("the gouge tolerance must be from 0 to 1000 mm");
        }
        // Sideways by at most half the radius, so that a tolerance as large as the cutter still
        // finds it deep in the surface along its axis; the shrunk cutter, raised by the whole
        // tolerance (a tapered one's side moved in by it), stays inside the cutter.
        let (inner, lift) = cutter.shrunk(tol)?;
        Ok(GougeTest { inner, lift, tol })
    }

    /// How far the cutter at `pose` is in the surface, where deeper than the tolerance: how much
    /// farther out along the axis it would have to be (see [`Hit::distance`]).
    pub(crate) fn depth(&self, surface: &Surface, pose: &Pose) -> Result<Option<f64>> {
        Ok(drop_along(surface, &self.inner, pose.up(self.lift), pose.axis)?.filter(|d| *d > 0.0).map(|d| d + self.tol))
    }
}

/// The poses along the cutting moves of `path` where `cutter` (as a cylinder of its radius,
/// endless above its end) cuts into the surface by more than `tol`: where the cutter shrunk by
/// `tol` all round (sideways by at most half its radius) and raised by `tol` along its axis
/// still meets the surface. A cutter touching a wall beside it, or within `tol` of it sideways,
/// does not gouge it. One hit per move, at its deepest sample.
pub fn gouges(surface: &Surface, cutter: &Cutter, path: &Toolpath5, tol: f64, sample_step: f64) -> Result<Vec<Hit>> {
    gouges_in(surface, cutter, path, tol, sample_step, Moves::Cutting)
}

/// [`gouges`] along the moves `moves` selects.
pub(crate) fn gouges_in(surface: &Surface, cutter: &Cutter, path: &Toolpath5, tol: f64, sample_step: f64, moves: Moves) -> Result<Vec<Hit>> {
    let test = GougeTest::new(cutter, tol)?;
    check_step(sample_step)?;
    check_path(path)?;
    let mut hits: Vec<Hit> = Vec::new();
    for_each_sample(path, cutter.radius, sample_step, moves, |index, pose| {
        if let Some(depth) = test.depth(surface, &pose)? {
            let hit = Hit { index, pose, envelope: None, distance: depth };
            match hits.last_mut() {
                Some(h) if h.index == index => {
                    if depth > h.distance {
                        *h = hit;
                    }
                }
                _ => hits.push(hit),
            }
        }
        Ok(())
    })?;
    Ok(hits)
}

/// A convex polygon of at most eight corners, without allocating.
struct Poly {
    p: [Vec3; 8],
    n: usize,
}

impl Poly {
    fn push(&mut self, v: Vec3) {
        if let Some(slot) = self.p.get_mut(self.n) {
            *slot = v;
            self.n += 1;
        }
    }

    fn points(&self) -> &[Vec3] {
        self.p.get(..self.n).unwrap_or(&[])
    }

    /// The part of the polygon where `h(p) ≥ 0`, `h` linear.
    fn clip(&self, h: impl Fn(Vec3) -> f64) -> Poly {
        let mut out = Poly { p: [Vec3::ZERO; 8], n: 0 };
        let pts = self.points();
        for (k, &a) in pts.iter().enumerate() {
            let b = pts.get(k + 1).or(pts.first()).copied().unwrap_or(a);
            let (ha, hb) = (h(a), h(b));
            if ha >= 0.0 {
                out.push(a);
            }
            if (ha >= 0.0) != (hb >= 0.0) {
                out.push(a + (b - a) * (ha / (ha - hb)));
            }
        }
        out
    }
}

/// The gap between the envelope's cylinder at `pose` and the triangle; `None` when no part of the
/// triangle is level with the cylinder (between its ends, along the axis).
fn cylinder_gap(pose: &Pose, e: &Envelope, t: &[Vec3; 3]) -> Option<f64> {
    let (a, o) = (pose.axis, pose.tip);
    let tri = Poly { p: [t[0], t[1], t[2], Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO], n: 3 };
    // Within the slab between the cylinder's ends, the distance to its axis segment is the
    // distance to its axis line, so the gap is exact.
    let slab = tri.clip(|p| a.dot(p - o) - e.bottom).clip(|p| e.top - a.dot(p - o));
    let (p0, p1) = (pose.up(e.bottom), pose.up(e.top));
    let d = match slab.points() {
        [] => return None,
        [q] => q.dist_to_segment(p0, p1),
        [q, r] => {
            let (x, y) = nearest_segments(*q, *r, p0, p1);
            x.dist(y)
        }
        [first, rest @ ..] => rest
            .windows(2)
            .map(|w| match w {
                [b, c] => segment_triangle_distance(p0, p1, &[*first, *b, *c]),
                _ => f64::INFINITY,
            })
            .fold(f64::INFINITY, f64::min),
    };
    Some(d - e.radius)
}

/// The smallest gap between the envelope at `pose` and the surface, when it is below `limit`.
fn envelope_gap(surface: &Surface, pose: &Pose, e: &Envelope, limit: f64) -> Option<f64> {
    let (p0, p1) = (pose.up(e.bottom), pose.up(e.top));
    let reach = e.radius + limit;
    let best: Cell<Option<f64>> = Cell::new(None);
    surface.visit(
        |b| capsule_meets_box(p0, p1, reach, b),
        |_, t| {
            if let Some(gap) = cylinder_gap(pose, e, t)
                && gap < limit
                && best.get().is_none_or(|b| gap < b)
            {
                best.set(Some(gap));
            }
        },
    );
    best.get()
}

/// The poses along the cutting moves of `path` where a part of `tool` above its flutes (each of
/// [`Tool::envelopes`]: the shank, then the holder's sections, cylinders about the axis) comes
/// closer to the surface than `clearance`, measured square to the axis. One hit per move and
/// envelope, at its closest sample.
pub fn holder_collisions(surface: &Surface, tool: &Tool, path: &Toolpath5, clearance: f64, sample_step: f64) -> Result<Vec<Hit>> {
    holder_collisions_in(surface, tool, path, clearance, sample_step, Moves::Cutting)
}

/// [`holder_collisions`] along the moves `moves` selects.
pub(crate) fn holder_collisions_in(
    surface: &Surface,
    tool: &Tool,
    path: &Toolpath5,
    clearance: f64,
    sample_step: f64,
    moves: Moves,
) -> Result<Vec<Hit>> {
    tool.validate()?;
    if !(clearance >= 0.0) || !(clearance <= 1.0e4) {
        return invalid("the holder clearance must be from 0 to 10 m");
    }
    check_step(sample_step)?;
    check_path(path)?;
    let envelopes = tool.envelopes();
    let reach = envelopes.iter().map(|e| e.top).fold(0.0, f64::max);
    let mut hits: Vec<Hit> = Vec::new();
    if envelopes.is_empty() {
        return Ok(hits);
    }
    for_each_sample(path, reach, sample_step, moves, |index, pose| {
        for (k, e) in envelopes.iter().enumerate() {
            let Some(gap) = envelope_gap(surface, &pose, e, clearance) else { continue };
            let hit = Hit { index, pose, envelope: Some(k), distance: gap };
            match hits.iter_mut().rev().take_while(|h| h.index == index).find(|h| h.envelope == Some(k)) {
                Some(h) => {
                    if gap < h.distance {
                        *h = hit;
                    }
                }
                None => hits.push(hit),
            }
        }
        Ok(())
    })?;
    Ok(hits)
}

/// Which way [`tilt_away`] leans the tool.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum TiltAbout {
    /// Sideways, about the feed direction, to whichever side (the same along each run of cutting
    /// moves) clears with the smaller tilt.
    FeedSide,
    /// The spindle towards `point`.
    TowardPoint { point: Vec3 },
    /// The spindle away from `point`.
    AwayFromPoint { point: Vec3 },
}

/// How [`tilt_away`] tilts a toolpath clear.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TiltOptions {
    /// Most the tool may be tilted from the path's own axis (radians, at most a right angle).
    pub max_tilt: f64,
    /// The tilt is tried in increments of this (radians).
    pub step: f64,
    pub about: TiltAbout,
    /// Least gap between the shank or holder and the surface (mm).
    pub clearance: f64,
    /// Most the tool axis may turn in one cutting move (radians).
    pub max_axis_step: f64,
    /// The path is divided into moves of about this length and checked this finely (mm).
    pub sample_step: f64,
    /// How far the cutter may cut into the surface before the result is refused (mm).
    pub tolerance: f64,
}

impl TiltOptions {
    pub fn validate(&self) -> Result<()> {
        if !(self.max_tilt > 0.0) || !(self.max_tilt <= std::f64::consts::FRAC_PI_2) {
            return invalid("the largest tilt must be above 0 and at most a right angle");
        }
        if !(self.step >= MIN_TILT_STEP) || !(self.step <= self.max_tilt) {
            return invalid("the tilt increment must be from 0.0001 rad to the largest tilt");
        }
        if !(self.max_axis_step >= MIN_TILT_STEP) || !(self.max_axis_step <= std::f64::consts::PI) {
            return invalid("the largest axis step must be from 0.0001 rad to π");
        }
        if !(self.clearance >= 0.0) || !(self.clearance <= 1.0e4) {
            return invalid("the holder clearance must be from 0 to 10 m");
        }
        if !(self.tolerance >= 0.0) || !(self.tolerance <= 1.0e3) {
            return invalid("the gouge tolerance must be from 0 to 1000 mm");
        }
        if let TiltAbout::TowardPoint { point } | TiltAbout::AwayFromPoint { point } = self.about
            && !point.is_finite()
        {
            return invalid("the point the tool is tilted by must be finite");
        }
        check_step(self.sample_step)
    }
}

/// One move of the path being tilted.
#[derive(Clone, Copy, Debug)]
enum Node {
    Rapid { pose: Pose, from: usize },
    Feed { pose: Pose, feed: f64, from: usize },
    Dwell { seconds: f64, from: usize },
}

impl Node {
    fn pose(&self) -> Option<Pose> {
        match *self {
            Node::Rapid { pose, .. } | Node::Feed { pose, .. } => Some(pose),
            Node::Dwell { .. } => None,
        }
    }

    fn from(&self) -> usize {
        match *self {
            Node::Rapid { from, .. } | Node::Feed { from, .. } | Node::Dwell { from, .. } => from,
        }
    }
}

/// The path with every cutting move divided into pieces of at most `step` (tip and the top of
/// the tool).
fn divide(path: &Toolpath5, reach: f64, step: f64) -> Result<Vec<Node>> {
    let mut out = Vec::with_capacity(path.moves.len());
    let mut prev: Option<Pose> = None;
    for (i, m) in path.moves.iter().enumerate() {
        match *m {
            Move5::Rapid { to } => out.push(Node::Rapid { pose: to, from: i }),
            Move5::Feed { to, feed } => {
                let n = match prev {
                    Some(from) => samples_along(&from, &to, reach, step)?,
                    None => 1,
                };
                if out.len().saturating_add(n) > MAX_MOVES {
                    return Err(CamError::TooLarge(format!("the divided toolpath has more than {MAX_MOVES} moves")));
                }
                for k in 1..=n {
                    let t = k as f64 / n as f64;
                    let pose = match prev {
                        Some(from) if k < n => Pose::new(from.tip.lerp(to.tip, t), slerp(from.axis, to.axis, t)),
                        _ => to,
                    };
                    out.push(Node::Feed { pose, feed, from: i });
                }
            }
            Move5::Dwell { seconds } => out.push(Node::Dwell { seconds, from: i }),
        }
        if let Some(to) = m.target() {
            prev = Some(to);
        }
    }
    Ok(out)
}

struct Tilter<'a> {
    surface: &'a Surface,
    cutter: Cutter,
    envelopes: Vec<Envelope>,
    clearance: f64,
    /// How far the cutter may cut into the surface (mm).
    tol: f64,
}

impl Tilter<'_> {
    fn collides(&self, pose: &Pose) -> bool {
        self.envelopes.iter().any(|e| envelope_gap(self.surface, pose, e, self.clearance).is_some())
    }

    /// `pose` with its axis leant by `angle` towards the unit `d` (square to the axis), keeping
    /// the cutter where it was: a ball turns about its centre, which keeps it touching the same
    /// points; another end turns about its tip and is pushed out along the new axis as far as it
    /// would cut in (never pulled in, so a pose in the air stays in the air). `None` where that
    /// would lift it off what it cuts ([`Tilter::turned`]).
    fn lean(&self, pose: &Pose, d: Vec3, angle: f64) -> Result<Option<Pose>> {
        if angle <= 0.0 {
            return Ok(Some(*pose));
        }
        let a = (pose.axis * angle.cos() + d * angle.sin()).normalized().unwrap_or(pose.axis);
        self.turned(pose, a)
    }

    /// `pose` with its axis `a`, the cutter kept as [`Tilter::lean`] does. `None` where an end
    /// other than a ball would have to be pushed out farther than turning it can ask of the
    /// surface it touches: its endless side has then met something beside it (a wall it runs
    /// along), and pushed out of that, it would no longer cut where it should, but somewhere up
    /// the wall or in the air.
    fn turned(&self, pose: &Pose, a: Vec3) -> Result<Option<Pose>> {
        if self.cutter.is_ball() {
            let r = self.cutter.radius;
            return Ok(Some(Pose::new(pose.up(r) - a * r, a)));
        }
        let out = drop_along(self.surface, &self.cutter, pose.tip, a)?.filter(|t| *t > 0.0).unwrap_or(0.0);
        // Turned by θ about the tip, no point of the end (none farther from the tip than
        // hypot(radius, rim height)) moves by more than 2·hypot·sin(θ/2); pushing it out along the
        // new axis clears that from a surface up to 60° from the axis within twice as far.
        let theta = a.cross(pose.axis).len().atan2(a.dot(pose.axis));
        let limit = 4.0 * self.cutter.radius.hypot(self.cutter.rim_height()) * (theta * 0.5).sin() + self.tol + 1e-9;
        Ok((out <= limit).then(|| Pose::new(pose.tip + a * out, a)))
    }

    /// The least tilt towards `d` in whole increments (the last one the largest tilt) that clears
    /// `pose`, and one increment more when the largest tilt allows it: the moves between poses,
    /// where the direction leant towards changes, are then clear too. `None` when none clears.
    fn needed(&self, pose: &Pose, d: Vec3, opts: &TiltOptions) -> Result<Option<f64>> {
        let n = (opts.max_tilt / opts.step - 1e-9).ceil().max(1.0) as usize;
        for k in 1..=n {
            let angle = (k as f64 * opts.step).min(opts.max_tilt);
            if self.lean(pose, d, angle)?.is_some_and(|p| !self.collides(&p)) {
                return Ok(Some((angle + opts.step).min(opts.max_tilt)));
            }
        }
        Ok(None)
    }
}

/// The unit direction of `v` square to the unit axis `a`, if it has one.
fn square_to(v: Vec3, a: Vec3) -> Option<Vec3> {
    let w = v - a * a.dot(v);
    (w.len() > 1e-9).then(|| w.normalized()).flatten()
}

/// The toolpath `path` (for example a 3-axis one through [`Toolpath5::from_3axis`]) made clear
/// of the tool's shank and holder by tilting it: at every pose where they come closer to the
/// surface than the clearance, the axis is leant ([`TiltAbout`]) in increments until they are
/// clear, keeping the cutter on the same points (a ball turns about its centre; other ends turn
/// about the tip, pushed out as far as they would cut in, and a lean that would push them out
/// farther, off what they cut, does not count as clear). The tilt is then spread along the path
/// so that it grows before a collision and dies away after it, the axis never turning more than
/// the step allowed in one move. A rapid move does not turn the axis on its way: the axis turns
/// where the tool stands, at the end of a rapid away from the part or at the start of one
/// towards it. The result is checked along every move, the rapids too: no gouge beyond the
/// tolerance, no holder collision, and [`verify5`]. Refused with [`CamError::Unsafe`], naming
/// the move of `path`, where even the largest tilt does not clear or the result fails a check.
pub fn tilt_away(surface: &Surface, tool: &Tool, path: &Toolpath5, opts: &TiltOptions) -> Result<Toolpath5> {
    tool.validate()?;
    opts.validate()?;
    check_path(path)?;
    let t = Tilter { surface, cutter: tool.cutter()?, envelopes: tool.envelopes(), clearance: opts.clearance, tol: opts.tolerance };
    if t.envelopes.is_empty() {
        return Ok(path.clone());
    }
    let reach = t.envelopes.iter().map(|e| e.top).fold(t.cutter.radius, f64::max);
    let nodes = divide(path, reach, opts.sample_step)?;
    // Runs of moves the tilt is spread along: each starts at a rapid and holds its cutting moves.
    let mut runs: Vec<Vec<usize>> = Vec::new();
    for (k, node) in nodes.iter().enumerate() {
        match node {
            Node::Rapid { .. } => runs.push(vec![k]),
            Node::Feed { .. } => match runs.last_mut() {
                Some(run) => run.push(k),
                None => runs.push(vec![k]),
            },
            Node::Dwell { .. } => {}
        }
    }
    let mut leant: Vec<Option<Pose>> = nodes.iter().map(Node::pose).collect();
    let spread = opts.max_axis_step * 0.5;
    for run in &runs {
        let poses: Vec<(usize, Pose)> = run.iter().filter_map(|&k| nodes.get(k).and_then(|n| n.pose().map(|p| (k, p)))).collect();
        let hit: Vec<bool> = poses.iter().map(|(_, p)| t.collides(p)).collect();
        if !hit.contains(&true) {
            continue;
        }
        // The direction to lean each pose towards.
        let feed_dir = |j: usize| -> Vec3 {
            let at = |i: usize| poses.get(i).map(|(_, p)| *p);
            let Some(p) = at(j) else { return Vec3::new(1.0, 0.0, 0.0) };
            let way = |i: usize, k: usize| match (at(i), at(k)) {
                (Some(a), Some(b)) => square_to(b.tip - a.tip, p.axis),
                _ => None,
            };
            let near = j.checked_sub(1).and_then(|i| way(i, j)).or_else(|| way(j, j + 1));
            let later = || (j + 1..poses.len()).find_map(|i| way(i - 1, i));
            let earlier = || (1..j).rev().find_map(|i| way(i - 1, i));
            near.or_else(later).or_else(earlier).unwrap_or_else(|| p.axis.any_perp())
        };
        let dirs = |sign: f64| -> Vec<Vec3> {
            poses
                .iter()
                .enumerate()
                .map(|(j, (_, p))| {
                    let d = match opts.about {
                        TiltAbout::FeedSide => square_to(p.axis.cross(feed_dir(j)) * sign, p.axis),
                        TiltAbout::TowardPoint { point } => square_to(point - p.tip, p.axis),
                        TiltAbout::AwayFromPoint { point } => square_to(p.tip - point, p.axis),
                    };
                    d.unwrap_or_else(|| p.axis.any_perp())
                })
                .collect()
        };
        let needs = |d: &[Vec3]| -> Result<std::result::Result<Vec<f64>, usize>> {
            let mut out = Vec::with_capacity(poses.len());
            for (j, ((k, p), dir)) in poses.iter().zip(d).enumerate() {
                if !hit.get(j).copied().unwrap_or(false) {
                    out.push(0.0);
                    continue;
                }
                match t.needed(p, *dir, opts)? {
                    Some(a) => out.push(a),
                    None => return Ok(Err(*k)),
                }
            }
            Ok(Ok(out))
        };
        let signs: &[f64] = if opts.about == TiltAbout::FeedSide { &[1.0, -1.0] } else { &[1.0] };
        let mut best: Option<(f64, Vec<Pose>)> = None;
        let mut stuck = 0;
        'signs: for &sign in signs {
            let d = dirs(sign);
            let mut angles = match needs(&d)? {
                Ok(angles) => angles,
                Err(k) => {
                    stuck = k;
                    continue;
                }
            };
            // Spread the tilt: lean neighbours early and ease off after.
            for j in 1..angles.len() {
                let before = angles.get(j - 1).copied().unwrap_or(0.0);
                if let Some(a) = angles.get_mut(j) {
                    *a = a.max(before - spread);
                }
            }
            for j in (0..angles.len().saturating_sub(1)).rev() {
                let after = angles.get(j + 1).copied().unwrap_or(0.0);
                if let Some(a) = angles.get_mut(j) {
                    *a = a.max(after - spread);
                }
            }
            let worst = angles.iter().copied().fold(0.0, f64::max);
            if best.as_ref().is_some_and(|(w, _)| *w <= worst) {
                continue;
            }
            // Every pose of the run, the spread ones too, must take its lean.
            let mut out = Vec::with_capacity(poses.len());
            for (((k, p), dir), angle) in poses.iter().zip(&d).zip(&angles) {
                match t.lean(p, *dir, *angle)? {
                    Some(q) => out.push(q),
                    None => {
                        stuck = *k;
                        continue 'signs;
                    }
                }
            }
            best = Some((worst, out));
        }
        let Some((_, out)) = best else {
            let from = nodes.get(stuck).map_or(0, Node::from);
            return Err(CamError::Unsafe(format!("move {from}: the holder cannot be tilted clear of the surface within the largest tilt")));
        };
        for ((k, _), q) in poses.iter().zip(out) {
            if let Some(slot) = leant.get_mut(*k) {
                *slot = Some(q);
            }
        }
    }
    // The moves, with poses turned evenly in between where the axis still turns too far.
    let mut moves: Vec<Move5> = Vec::with_capacity(nodes.len());
    let mut from: Vec<usize> = Vec::with_capacity(nodes.len());
    let mut prev: Option<(Pose, Pose)> = None;
    for (node, new) in nodes.iter().zip(&leant) {
        let (Some(orig), Some(new)) = (node.pose(), *new) else {
            if let Node::Dwell { seconds, from: f } = *node {
                moves.push(Move5::Dwell { seconds });
                from.push(f);
            }
            continue;
        };
        if let (Node::Feed { feed, from: f, .. }, Some((po, pn))) = (*node, prev) {
            let angle = pn.axis_angle(&new);
            if angle > opts.max_axis_step {
                let n = (angle / opts.max_axis_step).ceil() as usize;
                if moves.len().saturating_add(n) > MAX_MOVES {
                    return Err(CamError::TooLarge(format!("the tilted toolpath has more than {MAX_MOVES} moves")));
                }
                for i in 1..n {
                    let s = i as f64 / n as f64;
                    let base = Pose::new(po.tip.lerp(orig.tip, s), slerp(po.axis, orig.axis, s));
                    let Some(to) = t.turned(&base, slerp(pn.axis, new.axis, s))? else {
                        return Err(CamError::Unsafe(format!("move {f}: turning the tool there would lift the cutter off what it cuts")));
                    };
                    moves.push(Move5::Feed { to, feed });
                    from.push(f);
                }
            }
        }
        // A rapid turns the axis where the tool stands, never on its way: at its end when it
        // leaves the part (along the axes), at its start when it comes towards it.
        if let (Node::Rapid { .. }, Some((_, pn))) = (*node, prev)
            && pn.axis_angle(&new) > 1e-12
            && pn.tip.dist(new.tip) > 1e-12
        {
            if moves.len() >= MAX_MOVES {
                return Err(CamError::TooLarge(format!("the tilted toolpath has more than {MAX_MOVES} moves")));
            }
            let away = (new.tip - pn.tip).dot(pn.axis + new.axis) >= 0.0;
            moves.push(Move5::Rapid { to: if away { Pose::new(new.tip, pn.axis) } else { Pose::new(pn.tip, new.axis) } });
            from.push(node.from());
        }
        if moves.len() >= MAX_MOVES {
            return Err(CamError::TooLarge(format!("the tilted toolpath has more than {MAX_MOVES} moves")));
        }
        moves.push(match *node {
            Node::Feed { feed, .. } => Move5::Feed { to: new, feed },
            _ => Move5::Rapid { to: new },
        });
        from.push(node.from());
        prev = Some((orig, new));
    }
    let out = Toolpath5 { moves };
    let named = |i: usize| from.get(i).copied().unwrap_or(0);
    verify5(&out, opts.max_axis_step).map_err(|e| CamError::Unsafe(format!("the tilted toolpath fails its check: {e}")))?;
    if let Some(h) = gouges_in(surface, &t.cutter, &out, opts.tolerance, opts.sample_step, Moves::All)?.first() {
        return Err(CamError::Unsafe(format!("move {}: the tilted tool cuts {:.4} mm into the surface", named(h.index), h.distance)));
    }
    if let Some(h) = holder_collisions_in(surface, tool, &out, opts.clearance, opts.sample_step, Moves::All)?.first() {
        return Err(CamError::Unsafe(format!("move {}: the tilted holder still comes within {:.4} mm of the surface", named(h.index), h.distance)));
    }
    Ok(out)
}
