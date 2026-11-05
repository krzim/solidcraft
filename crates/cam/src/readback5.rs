//! Multi-axis programs as written, read back through the machine's kinematics and held against
//! the toolpaths they were written from, so what leaves for the machine is checked rather than
//! what was planned.
//!
//! The written blocks are read with [`parse5`], and the machine is moved through each one as
//! the controller would: every axis evenly from the block's start to its end (the tip evenly
//! in work coordinates with tool centre point control; around the arc for `G2`/`G3`). Points
//! along the way, through [`forward`], give the tip and tool axis against the work, and these
//! are compared both ways round with the planned cutting moves: the tip with the planned line,
//! the axis with the planned turn of the nearest one. Rounding to the post's decimals, the
//! pieces the cuts were broken into, and anything lost between the two show up as deviation.
//! The draw out of the cut and the plunge back of a post's rewinds are not in the toolpaths:
//! of a move that may be one, only its end in the cut is held against them (the planned moves
//! are still all looked for in what was written), and it counts in the cut length and time as
//! written like any other feed.

use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

use serde::{Deserialize, Serialize};
use solidcore_record::{Vec2, Vec3};
use solidcraft_geom::prelude::*;

use crate::gcode5::{JointWords, MachineOperation, MachineProgram, Post5, parse5};
use crate::kinematics::{Carrier, Joints, Machine, arc_distance, forward, tool_axis};
use crate::multiaxis::{Move5, Pose, Program5, Toolpath5, slerp};
use crate::path::arc_sweep;
use crate::{CamError, Result};

/// Most grid entries the comparison builds.
const MAX_ENTRIES: usize = 20_000_000;
/// Most points the written program is followed at.
const MAX_SAMPLES: usize = 10_000_000;
/// Largest rotary turn between two points a written block is followed at (radians).
const SAMPLE_TURN: f64 = 0.002;
/// Most points one written block is followed at.
const MAX_BLOCK_SAMPLES: usize = 512;

/// The written program against its toolpaths.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Readback5 {
    /// Largest distance between the tip as written and the planned cutting moves, either way
    /// round (mm). Distances beyond [`Readback5::reach`] are reported as `reach`.
    pub max_tip_deviation: f64,
    /// Largest angle between the tool axis as written and the planned one at the nearest
    /// cutting move (radians); π where no planned move is within reach.
    pub max_axis_deviation: f64,
    /// How far the post's precision and tolerances let the tips differ (mm): see [`allowance5`].
    pub allowed_tip: f64,
    /// How far they let the axes differ (radians).
    pub allowed_axis: f64,
    /// Farthest the comparison looks (mm).
    pub reach: f64,
    /// Distance the tip travels at feed as written and as planned (mm).
    pub cut_length: f64,
    pub planned_cut_length: f64,
    /// Time at feed as written (as the controller times it) and as planned, dwells included (minutes).
    pub cut_minutes: f64,
    pub planned_cut_minutes: f64,
}

impl Readback5 {
    /// Whether the written program stays within the post's precision and tolerances of its toolpaths.
    pub fn within(&self) -> bool {
        self.max_tip_deviation <= self.allowed_tip && self.max_axis_deviation <= self.allowed_axis
    }
}

/// The planned moves of every operation in work coordinates.
fn planned_paths(program: &MachineProgram, post: &Post5, machine: &Machine) -> Result<Vec<Toolpath5>> {
    program
        .operations
        .iter()
        .map(|op| match op {
            MachineOperation::Continuous(o) => Ok(o.path.clone()),
            MachineOperation::Indexed(o) => o.poses(machine, post.base.arc_tolerance),
        })
        .collect()
}

/// The largest deviation of tip (mm) and axis (radians) between a written and a planned
/// cutting move that `post`'s precision and tolerances account for, on `machine` for
/// `program`: half the last written digit on each linear axis (three times over, for an arc's
/// end and centre), half the last angle digit on each rotary times the farthest the tip gets
/// from the rotaries' lines, half again the linearization tolerances (the deviation between a
/// piece's ends peaks near, not exactly at, its middle), and the chord error of arcs twice.
pub fn allowance5(post: &Post5, machine: &Machine, program: &MachineProgram) -> Result<(f64, f64)> {
    let paths = planned_paths(program, post, machine)?;
    let reach = paths.iter().flat_map(Toolpath5::poses).map(|p| p.tip.len()).fold(0.0, f64::max);
    let lever = reach + machine.work_origin.len() + machine.rotaries.iter().map(|r| r.center.len()).sum::<f64>() + 1.0;
    let digit = 0.5 * 10f64.powi(-i32::from(post.base.decimals)) * post.base.units.mm();
    let angle_digit = (0.5 * 10f64.powi(-i32::from(post.angle_decimals))).to_radians();
    let n = machine.rotaries.len() as f64;
    let tip = 3.0 * digit * 3f64.sqrt() + n * angle_digit * lever + 1.5 * post.tolerance + 2.0 * post.base.arc_tolerance + 1e-6;
    let axis = 1.5 * post.angle_tolerance + 2.0 * n * angle_digit + 1e-9;
    Ok((tip, axis))
}

/// Reads `text` (written by [`crate::gcode5::emit5`] from `program` through `post` for
/// `machine`) back and measures how far its cutting moves are from the program's toolpaths.
pub fn readback5(text: &str, program: &Program5, post: &Post5, machine: &Machine) -> Result<Readback5> {
    readback_program(text, &MachineProgram::from(program), post, machine)
}

/// Where the axes are after a written block.
#[derive(Clone)]
struct State {
    linear: Vec3,
    rotary: Vec<f64>,
    tcp: bool,
}

/// Whether a cut from `from` to `to` at `feed` (mm/min) is the draw out of the cut or the plunge
/// back of one of the post's rewinds ([`crate::gcode5::Rewind`]): in machine coordinates on a
/// machine whose rotaries all turn the table (so the tool is along Z), straight along Z by at
/// most the rewind's distance with every other axis held, at the rewind's retract or plunge
/// feed as written. Such a move only passes through the room the tool leaves above a point of
/// the cut, so that point is all that is checked of it against the toolpaths.
fn rewind_move(post: &Post5, machine: &Machine, from: &State, to: &State, feed: f64) -> bool {
    let Some(rewind) = post.rewind else { return false };
    if from.tcp || to.tcp || machine.rotaries.iter().any(|r| r.carrier == Carrier::Head) {
        return false;
    }
    let digit = 0.5 * 10f64.powi(-i32::from(post.base.decimals)) * post.base.units.mm();
    let feed_digit = 0.5 * 10f64.powi(-i32::from(post.base.feed_decimals)) * post.base.units.mm() + 1e-9;
    from.rotary == to.rotary
        && from.linear.x == to.linear.x
        && from.linear.y == to.linear.y
        && (to.linear.z - from.linear.z).abs() <= rewind.distance + 2.0 * digit
        && [rewind.retract_feed, rewind.plunge_feed].iter().any(|r| (feed - r).abs() <= feed_digit)
}

fn pose_at(machine: &Machine, linear: Vec3, rotary: &[f64], tcp: bool) -> Pose {
    if tcp { Pose { tip: linear, axis: tool_axis(machine, rotary) } } else { forward(machine, &Joints { linear, rotary: rotary.to_vec() }) }
}

/// [`readback5`] for a program that may hold indexed operations
/// ([`crate::gcode5::emit_program`]).
pub fn readback_program(text: &str, program: &MachineProgram, post: &Post5, machine: &Machine) -> Result<Readback5> {
    post.validate()?;
    machine.validate()?;
    let written = parse5(text, &machine.letters())?;
    let (allowed_tip, allowed_axis) = allowance5(post, machine, program)?;
    let reach = (allowed_tip * 10.0).max(1.0);

    // The written cutting moves, followed point by point.
    let mut samples: Vec<Pose> = Vec::new();
    let mut written_segments: Vec<(Pose, Pose)> = Vec::new();
    let (mut cut_length, mut cut_minutes) = (0.0, 0.0);
    let mut state: Option<State> = None;
    for w in &written {
        let next = match w {
            JointWords::Rapid { linear, rotary, tcp, .. } => Some(State { linear: *linear, rotary: rotary.clone(), tcp: *tcp }),
            JointWords::Feed { linear, rotary, tcp, f, inverse_time, mm_per_unit, .. } => {
                let to = State { linear: *linear, rotary: rotary.clone(), tcp: *tcp };
                if let Some(from) = state.as_ref().filter(|s| s.tcp == *tcp) {
                    // Of a move that may be a rewind's, only its end in the cut is held
                    // against the toolpaths.
                    let rewind = !*inverse_time && rewind_move(post, machine, from, &to, *f * *mm_per_unit);
                    if rewind {
                        let low = if from.linear.z <= to.linear.z { from } else { &to };
                        samples.push(pose_at(machine, low.linear, &low.rotary, false));
                    }
                    let a = Joints { linear: from.linear, rotary: from.rotary.clone() };
                    let b = Joints { linear: to.linear, rotary: to.rotary.clone() };
                    let turn = a.rotary_change(&b);
                    let lin = from.linear.dist(to.linear);
                    cut_minutes += if *inverse_time {
                        1.0 / f
                    } else if *tcp || lin > 1e-9 {
                        lin / (f * mm_per_unit)
                    } else {
                        turn.to_degrees() / f
                    };
                    let n = ((turn / SAMPLE_TURN).ceil() as usize).clamp(1, MAX_BLOCK_SAMPLES);
                    let mut prev = pose_at(machine, a.linear, &a.rotary, *tcp);
                    for k in 1..=n {
                        let j = a.lerp(&b, k as f64 / n as f64);
                        let p = pose_at(machine, j.linear, &j.rotary, *tcp);
                        cut_length += prev.tip.dist(p.tip);
                        written_segments.push((prev, p));
                        if !rewind {
                            samples.push(p);
                        }
                        prev = p;
                    }
                }
                Some(to)
            }
            JointWords::Arc { linear, center, ccw, rotary, tcp, f, mm_per_unit, .. } => {
                let to = State { linear: *linear, rotary: rotary.clone(), tcp: *tcp };
                if let Some(from) = state.as_ref() {
                    let start = Vec2::new(from.linear.x, from.linear.y) - *center;
                    let (r, sweep) = (start.len(), arc_sweep(from.linear, to.linear, *center, *ccw));
                    let length = (r * sweep).hypot(to.linear.z - from.linear.z);
                    cut_minutes += length / (f * mm_per_unit);
                    let tol = 1e-4;
                    let step = if tol < r { 2.0 * (1.0 - tol / r).acos() } else { TAU / 8.0 };
                    let n = ((sweep.abs() / step.max(1e-3)).ceil() as usize).clamp(1, 4096);
                    let mut prev = pose_at(machine, from.linear, rotary, *tcp);
                    for k in 1..=n {
                        let t = k as f64 / n as f64;
                        let q = if k == n {
                            to.linear
                        } else {
                            (*center + Vec2::from_angle(start.angle() + sweep * t) * r).extend(from.linear.z + (to.linear.z - from.linear.z) * t)
                        };
                        let p = pose_at(machine, q, rotary, *tcp);
                        cut_length += prev.tip.dist(p.tip);
                        written_segments.push((prev, p));
                        samples.push(p);
                        prev = p;
                    }
                }
                Some(to)
            }
            JointWords::Dwell { seconds, .. } => {
                cut_minutes += seconds / 60.0;
                None
            }
        };
        if let Some(s) = next {
            state = Some(s);
        }
        if samples.len() > MAX_SAMPLES {
            return Err(CamError::TooLarge("the program is too large to read back and check".into()));
        }
    }

    // The planned cutting moves.
    let mut planned_segments: Vec<(Pose, Pose)> = Vec::new();
    let mut planned_points: Vec<Pose> = Vec::new();
    let (mut planned_cut_length, mut planned_cut_minutes) = (0.0, 0.0);
    for (op, path) in program.operations.iter().zip(planned_paths(program, post, machine)?) {
        let mut pos: Option<Pose> = None;
        for m in &path.moves {
            if let (Move5::Feed { to, .. }, Some(from)) = (m, pos) {
                planned_segments.push((from, *to));
                planned_points.extend([from, Pose { tip: from.tip.lerp(to.tip, 0.5), axis: slerp(from.axis, to.axis, 0.5) }, *to]);
            }
            if let Some(to) = m.target() {
                pos = Some(to);
            }
        }
        let (length, minutes) = match op {
            MachineOperation::Continuous(_) => {
                let s = path.stats();
                (s.cut_length, s.cut_minutes)
            }
            MachineOperation::Indexed(o) => {
                let s = o.path.stats();
                (s.cut_length, s.cut_minutes)
            }
        };
        planned_cut_length += length;
        planned_cut_minutes += minutes;
    }

    let (t1, a1) = directed(&samples, &planned_segments, reach, allowed_tip, allowed_axis)?;
    let (t2, a2) = directed(&planned_points, &written_segments, reach, allowed_tip, allowed_axis)?;
    Ok(Readback5 {
        max_tip_deviation: t1.max(t2),
        max_axis_deviation: a1.max(a2),
        allowed_tip,
        allowed_axis,
        reach,
        cut_length,
        planned_cut_length,
        cut_minutes,
        planned_cut_minutes,
    })
}

/// The largest deviation of a pose of `from` from the piece of `to` that explains it best (the
/// least of tip distance over `allowed_tip` and axis angle over `allowed_axis`, the larger of
/// the two counting): its tip from the piece's line and its axis from the piece's turn. Tip
/// distances are capped at `reach`.
fn directed(from: &[Pose], to: &[(Pose, Pose)], reach: f64, allowed_tip: f64, allowed_axis: f64) -> Result<(f64, f64)> {
    if from.is_empty() {
        return Ok((0.0, 0.0));
    }
    if to.is_empty() {
        return Ok((reach, PI));
    }
    // Pieces of `to` filed by the XY cells they cross (each split into bits at most a cell
    // long, filed by the cells of the bit's box, so a long diagonal files about its length in
    // cells, not its box's area); a point looks in its cell and the eight around it, so any
    // piece within `cell` of it in XY is seen.
    let (mut lo, mut hi) =
        (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
    for p in to.iter().flat_map(|(a, b)| [a.tip, b.tip]).chain(from.iter().map(|p| p.tip)) {
        lo = lo.min(p);
        hi = hi.max(p);
    }
    if !(lo.is_finite() && hi.is_finite()) {
        return Err(CamError::Unsafe("a toolpath has a coordinate that is not a finite number".into()));
    }
    let cell = reach.max((hi.x - lo.x).max(hi.y - lo.y) / 4096.0);
    let index = |v: f64, o: f64| ((v - o) / cell).floor() as i64;
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    let mut entries = 0usize;
    for (k, (a, b)) in to.iter().enumerate() {
        let (pa, pb) = (Vec2::new(a.tip.x, a.tip.y), Vec2::new(b.tip.x, b.tip.y));
        // The extent bounds the length, so at most about 5800 bits.
        let bits = ((pa.dist(pb) / cell).ceil() as usize).clamp(1, 8192);
        for s in 0..bits {
            let (p, q) = (pa.lerp(pb, s as f64 / bits as f64), pa.lerp(pb, (s + 1) as f64 / bits as f64));
            let (i0, i1) = (index(p.x.min(q.x), lo.x), index(p.x.max(q.x), lo.x));
            let (j0, j1) = (index(p.y.min(q.y), lo.y), index(p.y.max(q.y), lo.y));
            for i in i0..=i1 {
                for j in j0..=j1 {
                    let filed = grid.entry((i, j)).or_default();
                    if filed.last() != Some(&k) {
                        filed.push(k);
                        entries += 1;
                    }
                }
            }
            if entries > MAX_ENTRIES {
                return Err(CamError::TooLarge("the program is too large to read back and check".into()));
            }
        }
    }
    let (mut worst_tip, mut worst_axis): (f64, f64) = (0.0, 0.0);
    for p in from {
        let (pi, pj) = (index(p.tip.x, lo.x), index(p.tip.y, lo.y));
        let mut best: Option<(f64, f64, f64)> = None;
        for i in pi - 1..=pi + 1 {
            for j in pj - 1..=pj + 1 {
                for &k in grid.get(&(i, j)).map(Vec::as_slice).unwrap_or_default() {
                    let Some((a, b)) = to.get(k) else { continue };
                    let d = p.tip.dist_to_segment(a.tip, b.tip);
                    if d > reach {
                        continue;
                    }
                    let e = arc_distance(p.axis, a.axis, b.axis);
                    let score = (d / allowed_tip).max(e / allowed_axis);
                    if best.is_none_or(|(s, _, _)| score < s) {
                        best = Some((score, d, e));
                    }
                }
            }
        }
        let (d, e) = best.map_or((reach, PI), |(_, d, e)| (d, e));
        worst_tip = worst_tip.max(d);
        worst_axis = worst_axis.max(e);
    }
    Ok((worst_tip, worst_axis))
}
