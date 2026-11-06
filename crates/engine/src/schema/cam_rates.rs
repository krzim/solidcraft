//! Contracts and result types for a CAM job's machine rates (`machine_rates` or
//! `machine_rates_path` in `cam.job`, docs/cam-jobs.md): the machine's top rates from its
//! controller's settings, which time the written program as the machine runs it and warn about
//! the blocks its controller would slow.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// A machine's top rates. Linear rates are mm/min, rotary rates degrees a minute; a rate left
/// out is not limited.
#[derive(Clone, Debug, Default, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MachineRatesSpec {
    /// Rapid rate of X and Y, and of Z unless `rapid_z_mm_min` is given.
    pub rapid_xyz_mm_min: f64,
    /// Rapid rate of Z, where it differs from X and Y.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rapid_z_mm_min: Option<f64>,
    /// Rapid rate of the rotaries. Without it a turn at rapid is not timed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rapid_a_deg_min: Option<f64>,
    /// Top feed of X and Y (and of Z unless `max_feed_z_mm_min` is given): a cut asking more is
    /// slowed by the controller, and the reply warns.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_feed_mm_min: Option<f64>,
    /// Top feed of Z, where it differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_feed_z_mm_min: Option<f64>,
    /// Top rate of the rotaries while cutting: a cut asking more, in inverse time (`G93`) or
    /// not, is slowed by the controller, and the reply warns.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_a_deg_min: Option<f64>,
    /// The spindle's lowest and highest speed, rpm: the controller runs any other `S` at the
    /// nearest end, and the reply warns.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_rpm: Option<[f64; 2]>,
    /// Seconds the controller waits at each spindle start for it to come up to speed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_start_seconds: Option<f64>,
    /// The travel of X, Y and Z, mm: the reply warns when the program spans more.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub travel_mm: Option<[f64; 3]>,
}

/// A spindle speed outside the machine's range.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct SpindleSpeedReport {
    /// The program line.
    pub line: usize,
    pub rpm: f64,
}

/// The written program timed against the machine's rates.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct MachineTimeReport {
    /// Time at feed as the machine runs it, every cut held to the axes' top rates, dwells
    /// included, minutes.
    pub cut_minutes: f64,
    /// Time at feed as written (the `F` words alone), dwells included, minutes.
    pub written_cut_minutes: f64,
    /// Time at rapid, each move as long as its slowest axis takes, minutes. Moves to and from
    /// the machine's home (`G28`) are not timed.
    pub rapid_minutes: f64,
    /// Time waiting for the spindle to start, minutes.
    pub spindle_minutes: f64,
    /// All of it, minutes (acceleration left out).
    pub total_minutes: f64,
    /// Cutting blocks the controller runs slower than written.
    pub slowed_blocks: usize,
    /// Of those, blocks asking a rotary for more than `max_a_deg_min`.
    pub rotary_over: usize,
    /// Of those, blocks asking a linear axis for more than its top feed.
    pub feed_over: usize,
    /// The highest rotary rate a cut asks for, degrees a minute.
    pub peak_rotary_deg_min: Option<f64>,
    /// Its program line.
    pub peak_rotary_line: Option<usize>,
    /// The highest feed of the linear axes a cut asks for, mm/min.
    pub peak_feed_mm_min: Option<f64>,
    /// Its program line.
    pub peak_feed_line: Option<usize>,
    /// Spindle speeds outside the machine's range (the first few).
    pub spindle_out_of_range: Vec<SpindleSpeedReport>,
    /// How far the program's positions reach along X, Y and Z, mm.
    pub span_mm: [f64; 3],
    /// What the controller will do differently from what is written (also in the reply's notes).
    pub warnings: Vec<String>,
}
