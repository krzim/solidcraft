//! Contracts for the agent-facing side of `cam.job` (`cmd/cam_agent.rs`): machine profiles,
//! tools asked for by what they are rather than by number, heights left out to be worked out
//! from the part, the policies a profile sets for both, structured refusals with a fix, and the
//! compact reply. docs/cam-jobs.md ("Profiles, defaults and refusals") describes them.

use std::fmt;

use schemars::JsonSchema;
use serde::de::Error as _;
use serde::{Deserialize, Deserializer, Serialize};
use serde_json::{Map, Value};

use super::cam::{CamJobReply, MachineSpec, Outputs};

/// The tool an operation cuts with: a tool's `number`, or what it must be (the best match in the
/// job's tools is picked, and the reply's `tool_choices` says which and why).
#[derive(Clone, Debug, JsonSchema)]
#[serde(untagged)]
pub enum ToolRef {
    /// A tool's `number`.
    Number(u32),
    /// What the tool must be.
    Spec(ToolSpec),
}

/// Read by hand rather than untagged, so a misspelt key of a spec is named in the error.
impl<'de> Deserialize<'de> for ToolRef {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let v = Value::deserialize(d)?;
        match v {
            Value::Number(n) => n
                .as_u64()
                .and_then(|n| u32::try_from(n).ok())
                .map(ToolRef::Number)
                .ok_or_else(|| D::Error::custom("a tool number is a whole number from 0")),
            Value::Object(_) => ToolSpec::deserialize(v).map(ToolRef::Spec).map_err(D::Error::custom),
            _ => Err(D::Error::custom("must be a tool's number or what the tool must be: {kind?, diameter?, max_diameter?, min_flute_length?, …}")),
        }
    }
}

impl fmt::Display for ToolRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ToolRef::Number(n) => write!(f, "{n}"),
            ToolRef::Spec(s) => write!(f, "{s}"),
        }
    }
}

/// What a tool must be. Every key is optional; a tool matches when it meets all of them. Among
/// the matches the pick is deterministic: one of exactly `diameter` if given, else the largest
/// diameter (fewest passes), then the shortest flutes that still meet `min_flute_length` (the
/// stiffest), then the lowest number. A pocket or an inside contour also needs the tool to fit
/// its profiles.
#[derive(Clone, Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ToolSpec {
    /// `flat_endmill`, `ball_endmill`, `bull_nose` or `drill`. Default: any kind that mills.
    #[serde(default)]
    pub kind: Option<String>,
    /// Exactly this cutting diameter, mm.
    #[serde(default)]
    pub diameter: Option<f64>,
    /// At most this cutting diameter, mm.
    #[serde(default)]
    pub max_diameter: Option<f64>,
    /// At least this cutting diameter, mm.
    #[serde(default)]
    pub min_diameter: Option<f64>,
    /// Flutes at least this long, mm.
    #[serde(default)]
    pub min_flute_length: Option<f64>,
    /// Standing out of the holder at least this far, mm (a tool without `stickout` stands out its
    /// flute length).
    #[serde(default)]
    pub min_stickout: Option<f64>,
}

impl fmt::Display for ToolSpec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut parts: Vec<String> = Vec::new();
        if let Some(k) = &self.kind {
            parts.push(k.clone());
        }
        let n = |v: f64| format!("{}", (v * 1000.0).round() / 1000.0);
        if let Some(d) = self.diameter {
            parts.push(format!("{} mm", n(d)));
        }
        if let Some(d) = self.min_diameter {
            parts.push(format!("at least {} mm", n(d)));
        }
        if let Some(d) = self.max_diameter {
            parts.push(format!("at most {} mm", n(d)));
        }
        if let Some(l) = self.min_flute_length {
            parts.push(format!("flutes of at least {} mm", n(l)));
        }
        if let Some(l) = self.min_stickout {
            parts.push(format!("standing out at least {} mm", n(l)));
        }
        if parts.is_empty() { write!(f, "any milling tool") } else { write!(f, "{}", parts.join(", ")) }
    }
}

/// The Z levels of a 3-axis operation, in the operation's coordinates. Every one left out is
/// worked out from the part: `top` the part's top, `bottom` the lowest point of the region (a
/// pocket's floor; the part's bottom for an outside contour and the 3D operations), `retract`
/// `top` + the job's `default_heights.retract_above` (2), `clearance` the higher of `top` and
/// the fixtures' top + `default_heights.clearance_above` (10).
#[derive(Clone, Copy, Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct HeightsInput {
    /// Where the tool travels between cuts. Default: above the part and fixtures.
    #[serde(default)]
    pub clearance: Option<f64>,
    /// Where rapid motion towards the material stops and feed begins. Default: just above `top`.
    #[serde(default)]
    pub retract: Option<f64>,
    /// Top of the material to cut. Default: the part's top.
    #[serde(default)]
    pub top: Option<f64>,
    /// Final depth (for a parallel finish: the floor the tool never goes below). Default: the
    /// lowest point of the region.
    #[serde(default)]
    pub bottom: Option<f64>,
}

/// How far above the part the heights that are left out go.
#[derive(Clone, Copy, Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct HeightsPolicy {
    /// `clearance` this far above the part's (and fixtures') top, mm. Default: 10.
    #[serde(default)]
    pub clearance_above: Option<f64>,
    /// `retract` this far above `top`, mm. Default: 2.
    #[serde(default)]
    pub retract_above: Option<f64>,
}

/// Spindle speed and feed rates of an operation that gives none.
#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FallbackFeeds {
    /// The cutting feed, mm/min.
    pub cut: f64,
    /// The plunge feed, mm/min.
    pub plunge: f64,
    /// The spindle speed, rev/min.
    pub spindle_rpm: f64,
}

/// What feeds and speeds an operation takes when it gives none, and the machine's limits. An
/// operation's own `feeds` and `spindle_rpm` above a limit are refused; a tool's are scaled and
/// then held to the limits, and the reply's notes say so.
#[derive(Clone, Copy, Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FeedsPolicy {
    /// The tool's cutting feed times this (0.05 to 2). Default: 1.
    #[serde(default)]
    pub cut_scale: Option<f64>,
    /// The tool's plunge feed times this (0.05 to 2). Default: 1.
    #[serde(default)]
    pub plunge_scale: Option<f64>,
    /// The fastest cutting feed the machine takes, mm/min.
    #[serde(default)]
    pub max_cut: Option<f64>,
    /// The fastest plunge feed the machine takes, mm/min.
    #[serde(default)]
    pub max_plunge: Option<f64>,
    /// The spindle's slowest and fastest speed, [min, max] rev/min.
    #[serde(default)]
    pub spindle_rpm_range: Option<[f64; 2]>,
    /// For a tool without cutting data: these. Default: such an operation must give its own.
    #[serde(default)]
    pub fallback: Option<FallbackFeeds>,
}

/// A machine profile: a JSON file that bundles what every job on one machine shares. A job
/// names it with `profile_path`; every key the job gives itself wins over the profile's (a job's
/// `machine` or `machine_path` replaces the profile's machine with its A-axis options, a job's
/// `post` or `post_path` its post, a job's `tools`, `tool_library_path` or `tool_library_paths`
/// its tools). Relative paths in it are read from its own folder.
#[derive(Clone, Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamProfile {
    /// What the profile is for (not written anywhere).
    #[serde(default)]
    pub name: Option<String>,
    /// More words on it (not written anywhere).
    #[serde(default)]
    pub description: Option<String>,
    /// The machine: `3axis`, `4axis_a` or the CAM crate's `Machine` JSON.
    #[serde(default)]
    pub machine: Option<MachineSpec>,
    /// The machine as a `Machine` JSON file.
    #[serde(default)]
    pub machine_path: Option<String>,
    /// With `4axis_a`: the controller's positive A turns the table the other way.
    #[serde(default)]
    pub a_reversed: Option<bool>,
    /// With `4axis_a`: the lowest and highest A, degrees.
    #[serde(default)]
    pub a_limits_deg: Option<[f64; 2]>,
    /// A built-in post by name.
    #[serde(default)]
    pub post: Option<String>,
    /// A post as a `Post5` JSON file.
    #[serde(default)]
    pub post_path: Option<String>,
    /// 1 to 6 (G54 to G59).
    #[serde(default)]
    pub work_offset: Option<u8>,
    /// The machining tolerance, mm.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// The design point that is work zero, [x, y, z].
    #[serde(default)]
    pub origin: Option<[f64; 3]>,
    /// Fixture bodies by name (vises, clamps) the designs on this machine have.
    #[serde(default)]
    pub fixtures: Option<Vec<String>>,
    /// The tools as a JSON array (as a job's `tools`).
    #[serde(default)]
    pub tools: Option<Vec<Map<String, Value>>>,
    /// One tool library file.
    #[serde(default)]
    pub tool_library_path: Option<String>,
    /// Several tool library files. A number found in two of them with different tools is refused
    /// unless the job picks one library with `tool_library`.
    #[serde(default)]
    pub tool_library_paths: Option<Vec<String>>,
    /// Which of `tool_library_paths` to take the tools from (its path, file name or file name
    /// without `.json`).
    #[serde(default)]
    pub tool_library: Option<String>,
    /// How far above the part left-out heights go.
    #[serde(default)]
    pub default_heights: Option<HeightsPolicy>,
    /// Feeds and speeds of operations that give none, and the machine's limits.
    #[serde(default)]
    pub default_feeds: Option<FeedsPolicy>,
}

/// An operation, by its place in the job (from 0) and its name.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct OpRef {
    /// Its place in `operations`, from 0.
    pub index: usize,
    pub name: String,
}

/// Why `cam.job` refused, or a check that failed: what, where, and what to change.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct Refusal {
    /// What is wrong.
    pub error: String,
    /// The operation (null: the job as a whole).
    pub op: Option<OpRef>,
    /// The check: `heights`, `clearance`, `tool`, `feeds`, `fit`, `region`, `plan`, `gouge`,
    /// `holder`, `readback`, `profile`, `tools`.
    pub check: String,
    /// Where, [x, y, z] in work coordinates (null: no one place).
    pub at: Option<[f64; 3]>,
    /// The line of the written program (from 1), when there is one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub line: Option<usize>,
    /// What to change.
    pub fix: String,
}

/// The tool an operation cuts with, and why that one.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct ToolChoice {
    /// The operation's place in the job, from 0.
    pub op_index: usize,
    pub op: String,
    /// The tool's number.
    pub tool: u32,
    /// Its size and kind, and its description.
    pub description: String,
    /// `given by number`, or why the spec picked it.
    pub why: String,
}

/// The heights an operation ran with when some were worked out from the part.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct DerivedHeights {
    /// The operation's place in the job, from 0.
    pub op_index: usize,
    pub op: String,
    pub clearance: f64,
    pub retract: f64,
    pub top: f64,
    pub bottom: f64,
    /// The ones worked out (the rest were given).
    pub derived: Vec<String>,
    /// Where each derived one came from.
    pub why: String,
}

/// `cam.job` with `compact: true`: what an agent needs to go on.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamJobCompactReply {
    /// Always true: this is the compact reply.
    pub compact: bool,
    /// Whether every check passed.
    pub passed: bool,
    /// Whether this was a dry run (nothing written).
    pub dry_run: bool,
    /// The checks that failed, each with its place and fix.
    pub failures: Vec<Refusal>,
    /// One line per operation: index, name, kind, tool, moves, minutes, heights.
    pub operations: Vec<String>,
    /// The tool of every operation and why.
    pub tools: Vec<ToolChoice>,
    /// Time at feed of the whole program as the controller times it, minutes.
    pub total_minutes: f64,
    /// Lines of G-code.
    pub program_lines: usize,
    /// The files written.
    pub outputs: Outputs,
    /// The notes that are news: tools held to the machine's limits or left out of a library,
    /// a body named like a fixture machined as part of the part, and the like (not how the
    /// bodies were tessellated, which every job says).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub notes: Vec<String>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// What a refused `cam.job`'s message ends with, before its refusals as a JSON array.
pub const REFUSALS_TAG: &str = "\nrefusals: ";

/// The refusals at the end of a `cam.job` error message, as `{"cam_refusals": [...]}`: what the
/// MCP server returns as the error's structured content.
pub fn refusals_in(text: &str) -> Option<Value> {
    let (_, tail) = text.rsplit_once(REFUSALS_TAG)?;
    match serde_json::from_str::<Value>(tail.trim()) {
        Ok(Value::Array(a)) if a.iter().all(Value::is_object) => Some(serde_json::json!({ "cam_refusals": a })),
        _ => None,
    }
}

/// What `cam.job` returns: the full report, or with `compact: true` the compact one.
#[derive(Debug, Serialize, JsonSchema)]
#[serde(untagged)]
pub enum CamJobOutput {
    Full(Box<CamJobReply>),
    Compact(Box<CamJobCompactReply>),
}
