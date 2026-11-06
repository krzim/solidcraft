//! A written program timed as the machine runs it, and the controller settings a post must
//! agree with to be read right.
//!
//! A controller moves no axis faster than its top rate: a rapid takes as long as its slowest
//! axis needs, every axis at its own rapid rate, and a cut written faster than an axis can go
//! (a feed above the machine's top feed, or a cut that turns a rotary faster than it can turn,
//! in inverse time, `G93`, or not) is slowed until every axis keeps within its rate, so the cut
//! takes longer than its `F` says. [`machine_time`] reads a program with [`parse5`] and times every
//! block that way against a machine's [`MachineRates`]: the cut time as written and as run, the
//! rapid time, the spindle's start-up waits, and the blocks the controller would slow, with a
//! warning for each kind. Acceleration is left out (a controller blending moves at constant
//! velocity rarely stops between them), so the times are a lower bound for short blocks; moves
//! to and from the machine's home (`G28`) are in machine coordinates and are not timed.
//!
//! A dwell's `P` is read in the unit the post writes it in ([`DwellUnit`]), which must be the
//! unit the controller is set to read (some read `G4 P` in seconds, others in milliseconds).

use serde::{Deserialize, Serialize};
use solidcore_record::Vec3;
use solidcraft_geom::prelude::*;

use crate::gcode::{number, words};
use crate::gcode5::{JointWords, parse5};
use crate::kinematics::AxisLetter;
use crate::path::{Move, arc_sweep};
use crate::post::Post;
use crate::{Result, invalid};

/// Largest rate accepted (mm/min or degrees a minute).
const MAX_RATE: f64 = 1.0e6;
/// Largest travel accepted (mm).
const MAX_TRAVEL: f64 = 1.0e5;
/// Longest spindle start accepted (seconds).
const MAX_SPINDLE_START: f64 = 3600.0;
/// Most spindle speeds out of range listed in a report.
const MAX_LISTED: usize = 20;
/// Share by which a block may run longer than written before it counts as slowed.
const SLOWED: f64 = 1e-6;

/// How a post's dwell template states a dwell's length.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DwellUnit {
    Seconds,
    Milliseconds,
}

impl DwellUnit {
    /// The unit `post`'s dwell template writes: `{milliseconds}` or `{seconds}`, never both, and
    /// never neither (a dwell would then be written at a fixed length, whatever was planned).
    pub fn of(post: &Post) -> Result<DwellUnit> {
        match (post.dwell.contains("{seconds}"), post.dwell.contains("{milliseconds}")) {
            (true, false) => Ok(DwellUnit::Seconds),
            (false, true) => Ok(DwellUnit::Milliseconds),
            (true, true) => invalid("the post's dwell template must give the length once: `{seconds}` or `{milliseconds}`, not both"),
            (false, false) => invalid("the post's dwell template must give the length: `{seconds}` or `{milliseconds}`"),
        }
    }

    /// A dwell's `P` (or `X`) as written, in seconds.
    pub fn seconds(self, written: f64) -> f64 {
        match self {
            DwellUnit::Seconds => written,
            DwellUnit::Milliseconds => written / 1000.0,
        }
    }
}

/// The dwells of read motion, as [`crate::parse`] and [`parse5`] read them (`P` in seconds), in
/// seconds for a post that writes them in `unit`.
pub fn dwells_in(unit: DwellUnit, moves: Vec<Move>) -> Vec<Move> {
    moves
        .into_iter()
        .map(|m| match m {
            Move::Dwell { seconds } => Move::Dwell { seconds: unit.seconds(seconds) },
            other => other,
        })
        .collect()
}

/// [`dwells_in`] for multi-axis motion.
pub fn joint_dwells_in(unit: DwellUnit, words: Vec<JointWords>) -> Vec<JointWords> {
    words
        .into_iter()
        .map(|w| match w {
            JointWords::Dwell { line, seconds } => JointWords::Dwell { line, seconds: unit.seconds(seconds) },
            other => other,
        })
        .collect()
}

/// A machine's top rates, from its controller's settings. Every linear rate is mm/min, every
/// rotary rate degrees a minute; a rate left out is not limited (and a rapid of that axis is
/// not timed).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MachineRates {
    /// Rapid rate of X and Y (and of Z, unless `rapid_z_mm_min` is given).
    pub rapid_xyz_mm_min: f64,
    /// Rapid rate of Z, where it differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rapid_z_mm_min: Option<f64>,
    /// Rapid rate of every rotary.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rapid_a_deg_min: Option<f64>,
    /// Top feed of X and Y (and of Z, unless `max_feed_z_mm_min` is given).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_feed_mm_min: Option<f64>,
    /// Top feed of Z, where it differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_feed_z_mm_min: Option<f64>,
    /// Top rate of every rotary while cutting.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_a_deg_min: Option<f64>,
    /// The spindle's lowest and highest speed (rpm): the controller runs any other `S` at the
    /// nearest end.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_rpm: Option<[f64; 2]>,
    /// How long the controller waits for the spindle to come up to speed at each start (`M3`,
    /// `M4`), seconds.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_start_seconds: Option<f64>,
    /// The travel of X, Y and Z (mm): no program can span more.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub travel_mm: Option<[f64; 3]>,
}

fn rate_ok(v: f64) -> bool {
    v > 0.0 && v <= MAX_RATE
}

impl MachineRates {
    pub fn validate(&self) -> Result<()> {
        let rates = [self.rapid_z_mm_min, self.rapid_a_deg_min, self.max_feed_mm_min, self.max_feed_z_mm_min, self.max_a_deg_min];
        if !rate_ok(self.rapid_xyz_mm_min) || rates.iter().flatten().any(|&r| !rate_ok(r)) {
            return invalid(format!("every machine rate must be above 0 and at most {} a minute", number(MAX_RATE, 0)));
        }
        if let Some([lo, hi]) = self.spindle_rpm
            && !(lo >= 0.0 && lo <= hi && hi <= MAX_RATE)
        {
            return invalid("the spindle range must be [lowest, highest] rpm, the lowest first, at most 1000000");
        }
        if self.spindle_start_seconds.is_some_and(|s| !(0.0..=MAX_SPINDLE_START).contains(&s)) {
            return invalid("the spindle's start must be 0 to 3600 seconds");
        }
        if self.travel_mm.is_some_and(|t| t.iter().any(|&v| !(v > 0.0 && v <= MAX_TRAVEL))) {
            return invalid("the travel of X, Y and Z must be above 0 and at most 100 m");
        }
        Ok(())
    }

    fn rapid_z(&self) -> f64 {
        self.rapid_z_mm_min.unwrap_or(self.rapid_xyz_mm_min)
    }

    fn feed_z(&self) -> Option<f64> {
        self.max_feed_z_mm_min.or(self.max_feed_mm_min)
    }
}

/// A program timed against a machine's rates ([`machine_time`]).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MachineTime {
    /// Time at feed as the machine runs it, every cut held to the axes' top rates, dwells
    /// included (minutes).
    pub cut_minutes: f64,
    /// Time at feed as written (the `F` words alone), dwells included (minutes).
    pub written_cut_minutes: f64,
    /// Time at rapid, each move as long as its slowest axis takes (minutes).
    pub rapid_minutes: f64,
    /// Time waiting for the spindle to start (minutes).
    pub spindle_minutes: f64,
    /// All of it (minutes).
    pub total_minutes: f64,
    /// Cutting blocks the controller runs slower than written.
    pub slowed_blocks: usize,
    /// Of those, blocks that ask a rotary for more than its top rate.
    pub rotary_over: usize,
    /// Of those, blocks that ask a linear axis for more than its top feed.
    pub feed_over: usize,
    /// The highest rotary rate a cut asks for (degrees a minute) and its line.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub peak_rotary_deg_min: Option<(usize, f64)>,
    /// The highest feed of the linear axes a cut asks for (mm/min) and its line.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub peak_feed_mm_min: Option<(usize, f64)>,
    /// Spindle speeds outside the machine's range: `(line, rpm)`, the first few.
    pub spindle_out_of_range: Vec<(usize, f64)>,
    /// How far the program's positions reach along X, Y and Z (mm).
    pub span_mm: [f64; 3],
    /// What the controller will do differently from what is written.
    pub warnings: Vec<String>,
}

/// Where the axes are after a block (mm, radians).
struct At {
    linear: Vec3,
    rotary: Vec<f64>,
}

/// The largest turn of any rotary between two positions, degrees.
fn turn_deg(from: &[f64], to: &[f64]) -> f64 {
    from.iter().zip(to).map(|(a, b)| (b - a).abs().to_degrees()).fold(0.0, f64::max)
}

/// Minutes the slowest axis needs to move `d` (mm) and turn `turn` (degrees) at `xy`, `z` and
/// `a` (a rate per minute; `None`: unlimited).
fn axis_minutes(d: Vec3, turn: f64, xy: Option<f64>, z: Option<f64>, a: Option<f64>) -> f64 {
    let per = |dist: f64, rate: Option<f64>| rate.map_or(0.0, |r| dist / r);
    per(d.x.abs(), xy).max(per(d.y.abs(), xy)).max(per(d.z.abs(), z)).max(per(turn, a))
}

fn peak(slot: &mut Option<(usize, f64)>, line: usize, v: f64) {
    if slot.is_none_or(|(_, p)| v > p) {
        *slot = Some((line, v));
    }
}

/// Times `text` (G-code in absolute coordinates, rotaries `letters`, dwells written in `dwell`)
/// as a machine with `rates` runs it.
pub fn machine_time(text: &str, letters: &[AxisLetter], rates: &MachineRates, dwell: DwellUnit) -> Result<MachineTime> {
    rates.validate()?;
    let written = parse5(text, letters)?;
    let mut t = MachineTime::default();
    let (xy, z, a) = (rates.max_feed_mm_min, rates.feed_z(), rates.max_a_deg_min);
    let mut at: Option<At> = None;
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    let mut untimed_turns = 0usize;
    let (mut first_rotary_over, mut first_feed_over) = (None, None);
    // The fastest feed among the blocks slowed for a linear axis: the feed the warning names
    // (the fastest of all may be a diagonal no single axis is too slow for).
    let mut worst_feed_over: Option<(usize, f64)> = None;
    for w in &written {
        let (line, linear, rotary) = match w {
            JointWords::Dwell { seconds, .. } => {
                let minutes = dwell.seconds(*seconds) / 60.0;
                t.cut_minutes += minutes;
                t.written_cut_minutes += minutes;
                continue;
            }
            JointWords::Rapid { line, linear, rotary, .. }
            | JointWords::Feed { line, linear, rotary, .. }
            | JointWords::Arc { line, linear, rotary, .. } => (*line, *linear, rotary),
        };
        for (k, v) in [linear.x, linear.y, linear.z].into_iter().enumerate() {
            if let (Some(lo), Some(hi)) = (low.get_mut(k), high.get_mut(k)) {
                *lo = lo.min(v);
                *hi = hi.max(v);
            }
        }
        // The first block whose every axis is known only says where the axes are.
        let Some(from) = at.replace(At { linear, rotary: rotary.clone() }) else { continue };
        let d = linear - from.linear;
        let turn = turn_deg(&from.rotary, rotary);
        match w {
            JointWords::Rapid { .. } => {
                if turn > 0.0 && rates.rapid_a_deg_min.is_none() {
                    untimed_turns += 1;
                }
                t.rapid_minutes += axis_minutes(d, turn, Some(rates.rapid_xyz_mm_min), Some(rates.rapid_z()), rates.rapid_a_deg_min);
            }
            JointWords::Feed { f, inverse_time, mm_per_unit, .. } => {
                let len = d.len();
                let nominal = if *inverse_time {
                    1.0 / f
                } else if len > 1e-12 {
                    len / (f * mm_per_unit)
                } else {
                    turn / f
                };
                if !(nominal.is_finite() && nominal > 0.0) {
                    continue;
                }
                // Every cut that turns a rotary asks it for a rate, whether its `F` is inverse
                // time or along the linear axes (or, for a turn alone, in degrees a minute).
                if turn > 0.0 {
                    peak(&mut t.peak_rotary_deg_min, line, turn / nominal);
                }
                if len > 1e-12 {
                    peak(&mut t.peak_feed_mm_min, line, len / nominal);
                }
                let run = nominal.max(axis_minutes(d, turn, xy, z, a));
                t.written_cut_minutes += nominal;
                t.cut_minutes += run;
                if run > nominal * (1.0 + SLOWED) {
                    t.slowed_blocks += 1;
                    if a.is_some_and(|a| turn / nominal > a * (1.0 + SLOWED)) {
                        t.rotary_over += 1;
                        first_rotary_over.get_or_insert(line);
                    }
                    if axis_minutes(d, 0.0, xy, z, None) > nominal * (1.0 + SLOWED) {
                        t.feed_over += 1;
                        first_feed_over.get_or_insert(line);
                        peak(&mut worst_feed_over, line, len / nominal);
                    }
                }
            }
            JointWords::Arc { center, ccw, f, mm_per_unit, .. } => {
                let r = (from.linear.x - center.x).hypot(from.linear.y - center.y);
                let across = r * arc_sweep(from.linear, linear, *center, *ccw).abs();
                let len = across.hypot(d.z);
                let nominal = len / (f * mm_per_unit);
                if !(nominal.is_finite() && nominal > 0.0) {
                    continue;
                }
                peak(&mut t.peak_feed_mm_min, line, len / nominal);
                // Round an arc each linear axis in turn moves as fast as the tool does.
                let per = |dist: f64, rate: Option<f64>| rate.map_or(0.0, |r| dist / r);
                let run = nominal.max(per(across, xy)).max(per(d.z.abs(), z));
                t.written_cut_minutes += nominal;
                t.cut_minutes += run;
                if run > nominal * (1.0 + SLOWED) {
                    t.slowed_blocks += 1;
                    t.feed_over += 1;
                    first_feed_over.get_or_insert(line);
                    peak(&mut worst_feed_over, line, len / nominal);
                }
            }
            JointWords::Dwell { .. } => {}
        }
    }
    if low.iter().all(|v| v.is_finite()) {
        t.span_mm = [high[0] - low[0], high[1] - low[1], high[2] - low[2]];
    }

    // The spindle: its starts, and speeds outside its range.
    let mut out_of_range = 0usize;
    let mut on = false;
    let mut starts = 0usize;
    for (i, text_line) in text.lines().enumerate() {
        let n = i + 1;
        let ws = words(text_line, n)?;
        let code = |letter: char, v: i64| ws.iter().any(|&(c, x)| c == letter && (x * 10.0).round() as i64 == v * 10);
        if let (Some([lo, hi]), Some(&(_, s))) = (rates.spindle_rpm, ws.iter().find(|(c, _)| *c == 'S'))
            && !(s >= lo && s <= hi)
        {
            out_of_range += 1;
            if t.spindle_out_of_range.len() < MAX_LISTED {
                t.spindle_out_of_range.push((n, s));
            }
        }
        if code('M', 3) || code('M', 4) {
            if !on {
                starts += 1;
            }
            on = true;
        }
        if code('M', 5) || code('M', 2) || code('M', 30) {
            on = false;
        }
    }
    t.spindle_minutes = rates.spindle_start_seconds.unwrap_or(0.0) * starts as f64 / 60.0;
    t.total_minutes = t.cut_minutes + t.rapid_minutes + t.spindle_minutes;

    let rotary = match letters {
        [one] => one.char().to_string(),
        _ => "a rotary".into(),
    };
    let fmt = |v: f64| number(v, 0);
    if t.rotary_over > 0
        && let (Some(max), Some((l, p)), Some(first)) = (a, t.peak_rotary_deg_min, first_rotary_over)
    {
        t.warnings.push(format!(
            "{} cutting blocks (the first at line {first}) ask {rotary} to turn faster than its {}°/min (up to {}°/min at line {l}): \
             the controller slows them, so they take longer than written and cut at a lower feed",
            t.rotary_over,
            fmt(max),
            fmt(p)
        ));
    }
    if t.feed_over > 0
        && let Some(first) = first_feed_over
    {
        let peak = worst_feed_over.map_or(String::new(), |(l, p)| format!(" (up to {} mm/min at line {l})", fmt(p)));
        t.warnings.push(format!(
            "{} cutting blocks (the first at line {first}) ask a linear axis for more than its top feed{peak}: the controller \
             slows them, so they take longer than written",
            t.feed_over
        ));
    }
    if untimed_turns > 0 {
        t.warnings.push(format!("{untimed_turns} rapids turn {rotary}, which is not timed: the machine's rates give no `rapid_a_deg_min`"));
    }
    if let (Some([lo, hi]), Some(&(l, s))) = (rates.spindle_rpm, t.spindle_out_of_range.first()) {
        t.warnings.push(format!(
            "{out_of_range} spindle speeds are outside the machine's {}-{} rpm (S{} at line {l}): the controller runs them at the \
             nearest end of its range",
            fmt(lo),
            fmt(hi),
            fmt(s)
        ));
    }
    if let Some(travel) = rates.travel_mm {
        for ((axis, span), limit) in ['X', 'Y', 'Z'].into_iter().zip(t.span_mm).zip(travel) {
            if span > limit + 1e-6 {
                t.warnings.push(format!(
                    "the program spans {} mm along {axis}, more than the machine's {} mm of travel",
                    number(span, 3),
                    number(limit, 3)
                ));
            }
        }
    }
    Ok(t)
}
