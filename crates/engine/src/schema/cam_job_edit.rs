//! Contracts and replies of the open CAM job's commands (`cmd/cam_job_edit.rs` and
//! `cmd/cam_plan.rs`): `cam.job.new`, `open`, `save`, `reload`, `close`, `get`, `edit`, `undo`,
//! `redo`, `cam.library.list`, `cam.op.*`, `cam.generate`, `cam.findings`, `cam.finding.go`,
//! `cam.post` and `view.manufacture_stage`. docs/cam-jobs.md, "The open job", describes them.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use super::cam::Outputs;
use super::cam_agent::OpRef;

/// Start a new job for the design, kept open in the session; nothing is written until it is
/// saved (`cam.job.save`).
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamJobNewParams {
    /// Where the job will be saved (a `.json` file). Default: `<design name>.cam.json` beside
    /// the saved design; none for an unsaved design (then `cam.job.save {path}`).
    #[serde(default)]
    pub path: Option<String>,
    /// The program's name. Default: the design's.
    #[serde(default)]
    pub name: Option<String>,
    /// A machine profile file for the job's `profile_path` (machine, post, tool libraries).
    #[serde(default)]
    pub profile_path: Option<String>,
    /// Drop an open job's unsaved changes. Default: false (refused when there are some).
    #[serde(default)]
    pub discard: Option<bool>,
}

/// Open a job file, kept open in the session for `cam.op.*`, `cam.generate` and the
/// Manufacture workspace.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamJobOpenParams {
    /// The job file (`.json`).
    pub path: String,
    /// Drop an open job's unsaved changes. Default: false (refused when there are some).
    #[serde(default)]
    pub discard: Option<bool>,
}

/// Write the open job to its file, keys in the job's order and the file's own indent.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamJobSaveParams {
    /// Save it as this file instead (`.json`), which it then is. Default: its own file.
    #[serde(default)]
    pub path: Option<String>,
    /// Write even when the file was changed outside SolidCraft since it was read, or when
    /// another file is at `path`. Default: false (refused).
    #[serde(default)]
    pub overwrite: Option<bool>,
}

/// Read the open job's file again (an undoable step).
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamJobReloadParams {
    /// Drop unsaved changes. Default: false (refused when there are some).
    #[serde(default)]
    pub discard: Option<bool>,
}

/// Close the open job.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamJobCloseParams {
    /// Drop unsaved changes. Default: false (refused when there are some).
    #[serde(default)]
    pub discard: Option<bool>,
}

/// The open job's state.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamJobGetParams {
    /// Include the job's JSON. Default: false.
    #[serde(default)]
    pub job: Option<bool>,
}

/// Set or remove keys of the open job (one undoable step). Every edit is read with the job's
/// schema before it is kept: one that does not parse is refused and changes nothing.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamJobEditParams {
    /// Keys to set: job keys, or dotted paths into objects (`stock.box.max`, `default_heights.clearance_above`).
    #[serde(default)]
    pub set: Option<Map<String, Value>>,
    /// Keys (or dotted paths) to remove, so they take their defaults again.
    #[serde(default)]
    pub unset: Option<Vec<String>>,
}

/// Undo or redo a step of the open job.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamJobHistoryParams {}

/// What a library folder holds for jobs.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamLibraryListParams {
    /// The folder (with `profiles/`, `posts/`, `tools/` and `machines/`). Default: the open
    /// job's folder's parent.
    #[serde(default)]
    pub folder: Option<String>,
}

/// An operation of the open job: its place (from 0) or its name.
#[derive(Clone, Debug, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum OpSelect {
    Index(usize),
    Name(String),
}

/// Add an operation to the open job (one undoable step).
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamOpAddParams {
    /// The operation: `type` and any of its keys (docs/cam-jobs.md). Keys left out stay out
    /// (their defaults); a `tool` left out is `{}` (the best tool of the job's for it), and keys
    /// the type needs are given starting values. Default name: its type's and a number.
    pub op: Map<String, Value>,
    /// Where it goes (from 0). Default: last.
    #[serde(default)]
    pub at: Option<usize>,
    /// Index it to this A angle, degrees (an `indexed` operation; 0: not indexed).
    #[serde(default)]
    pub a_deg: Option<f64>,
}

/// Change an operation of the open job (one undoable step).
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamOpEditParams {
    pub op: OpSelect,
    /// Keys to set, dotted paths into objects (`heights.bottom`, `feeds.cut`); in an indexed
    /// operation they are its inner operation's.
    #[serde(default)]
    pub set: Option<Map<String, Value>>,
    /// Keys (or dotted paths) to remove, so they take their defaults again.
    #[serde(default)]
    pub unset: Option<Vec<String>>,
    /// A new name.
    #[serde(default)]
    pub rename: Option<String>,
    /// Turn A to: one angle (0: not indexed), several (`indexed_pattern`) or `{count, start_deg?}`.
    #[serde(default)]
    pub a_deg: Option<Value>,
}

/// An operation of the open job.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamOpParams {
    pub op: OpSelect,
}

/// Move an operation of the open job.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamOpMoveParams {
    pub op: OpSelect,
    /// Its new place, from 0.
    pub to: usize,
}

/// Copy an operation of the open job.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamOpDuplicateParams {
    pub op: OpSelect,
    /// Where the copy goes, from 0. Default: right after it.
    #[serde(default)]
    pub to: Option<usize>,
}

/// Turn an operation of the open job off or on.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamOpEnableParams {
    pub op: OpSelect,
    pub enabled: bool,
}

/// Plan and check the open job (a dry run of `cam.job`) and keep its findings and toolpaths.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamGenerateParams {
    /// Also simulate the program on the stock. Default: false.
    #[serde(default)]
    pub simulate: Option<bool>,
    /// With `simulate`: the cell size, mm (0.0001 to 10000).
    #[serde(default)]
    pub resolution: Option<f64>,
}

/// The findings of the open job's last Generate.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamFindingsParams {}

/// Go to a finding of the last Generate: the window selects its operation and frames its place.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamFindingGoParams {
    /// The finding's `index`.
    pub index: usize,
}

/// Write the open job's program (and setup sheet), planned and checked again unless the last
/// Generate is of the job and design as they are.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamPostParams {
    /// The G-code file. Default: the job file's name with the post's extension, beside it.
    #[serde(default)]
    pub output: Option<String>,
    /// A setup sheet (`.html` or `.md`). Default: none.
    #[serde(default)]
    pub setup_sheet: Option<String>,
    /// An HTML preview page. Default: none.
    #[serde(default)]
    pub preview: Option<String>,
    /// Write even when a check fails. Default: false.
    #[serde(default)]
    pub allow_unsafe: Option<bool>,
    /// Also simulate the program on the stock. Default: false.
    #[serde(default)]
    pub simulate: Option<bool>,
}

/// The Manufacture workspace's stage.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ManufactureStageParams {
    /// `setup`, `operations`, `check` or `program`.
    pub stage: String,
}

/// An operation's state in the job tree.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum OpStatus {
    /// Planned and checked by the last Generate, nothing found.
    Ok,
    /// The job or the design changed since the last Generate.
    Stale,
    /// The last Generate found a warning.
    Warning,
    /// It could not be planned, or a check failed.
    Error,
    /// Turned off (`enabled: false`).
    Off,
    /// Not generated yet.
    Unplanned,
}

/// An operation in a [`JobSummary`].
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct OpSummary {
    pub index: usize,
    pub name: String,
    /// Its `type` (`indexed` and `indexed_pattern`: the inner operation's).
    #[serde(rename = "type")]
    pub kind: String,
    /// The UI's message id for its type.
    pub label_id: String,
    /// Its tool's number, when it names one (or the last Generate picked one).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool: Option<u32>,
    /// The A angles it is indexed to.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub a_deg: Vec<f64>,
    pub enabled: bool,
    pub status: OpStatus,
    /// Time at feed from the last Generate, minutes.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub minutes: Option<f64>,
}

/// The open job in brief.
#[derive(Clone, Debug, Default, Serialize, JsonSchema)]
pub struct JobSummary {
    /// Its file (none: not saved anywhere yet).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    pub name: String,
    /// It differs from its file.
    pub dirty: bool,
    /// Counts up with every change.
    pub revision: u64,
    /// The machine (`3axis`, `4axis_a`, a file, a profile's or none).
    pub machine: String,
    /// The post (a built-in's name, a file, a profile's or none).
    pub post: String,
    /// The machine profile, when it uses one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub profile: Option<String>,
    /// The tools, or the libraries they come from.
    pub tools: String,
    /// The stock in words.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stock: Option<String>,
    /// Work zero, design coordinates.
    pub origin: [f64; 3],
    /// The workholding (a file, inline, or none).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub holding: Option<String>,
    /// The part's bodies (empty: every visible body not a fixture).
    pub bodies: Vec<String>,
    pub fixtures: Vec<String>,
    pub operations: Vec<OpSummary>,
}

/// What `cam.job.get` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamJobGetReply {
    /// Whether a job is open.
    pub open: bool,
    /// The file against what was last read or written: `none`, `changed` or `gone`.
    pub outside: String,
    /// The labels of the steps undo and redo would take, latest first.
    pub undo: Vec<String>,
    pub redo: Vec<String>,
    /// Whether the last Generate is of the job and design as they are.
    pub generated: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub summary: Option<JobSummary>,
    /// The job's JSON (with `job: true`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub job: Option<Value>,
}

/// What `cam.job.save` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamJobSaveReply {
    pub path: String,
    pub bytes: usize,
    /// Whether the file was written (false: it already held the job).
    pub written: bool,
}

/// What `cam.job.reload` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamJobReloadReply {
    pub summary: JobSummary,
    /// What changed, one line each.
    pub changes: Vec<String>,
}

/// What `cam.job.close` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamJobCloseReply {
    pub closed: bool,
}

/// What `cam.job.undo` and `cam.job.redo` return.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamJobHistoryReply {
    /// The step's label (none: nothing to undo or redo).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub step: Option<String>,
    pub summary: JobSummary,
}

/// A profile in a library folder.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct LibraryProfile {
    pub path: String,
    pub name: String,
    pub description: String,
    pub machine: String,
}

/// A file in a library folder, with its name.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct LibraryFile {
    pub path: String,
    pub name: String,
}

/// A tool library in a library folder.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct LibraryTools {
    pub path: String,
    /// Tools in it.
    pub tools: usize,
    /// Tools this build cannot read, by number, with why.
    pub left_out: Vec<String>,
}

/// What `cam.library.list` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamLibraryListReply {
    pub folder: String,
    pub profiles: Vec<LibraryProfile>,
    /// The built-in posts, then post files.
    pub posts: Vec<LibraryFile>,
    pub tool_libraries: Vec<LibraryTools>,
    pub machines: Vec<LibraryFile>,
    pub workholdings: Vec<LibraryFile>,
    pub machine_models: Vec<LibraryFile>,
}

/// What `cam.op.*` return.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamOpReply {
    /// The operation's place (none: removed).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub index: Option<usize>,
    pub name: String,
    pub summary: JobSummary,
}

/// A finding of a Generate: a failed check, a refusal or a warning, with its operation, place
/// and fix.
#[derive(Clone, Debug, Default, Serialize, JsonSchema)]
pub struct Finding {
    /// Its place in the list, for `cam.finding.go`.
    pub index: usize,
    /// `error`, `warning` or `note`.
    pub severity: String,
    /// The check (`heights`, `clearance`, `tool`, `gouge`, `holder`, `readback`, `rapids`,
    /// `travel`, `below_stock`, `simulation`, `plan`, `warning`...).
    pub check: String,
    /// The operation (none: the job as a whole).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub op: Option<OpRef>,
    /// Where, work coordinates.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub at: Option<[f64; 3]>,
    /// Where, design coordinates.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub at_design: Option<[f64; 3]>,
    /// The program line, from 1.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub line: Option<usize>,
    /// The move of the operation's toolpath, from 0.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub move_index: Option<usize>,
    pub message: String,
    /// What to change (empty: nothing to change).
    pub fix: String,
}

/// An operation in a Generate's reply.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct GeneratedOp {
    pub index: usize,
    pub name: String,
    pub tool: u32,
    pub moves: usize,
    pub minutes: f64,
    pub status: Option<OpStatus>,
}

/// What `cam.generate` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamGenerateReply {
    /// Whether every check passed.
    pub passed: bool,
    pub findings: Vec<Finding>,
    pub operations: Vec<GeneratedOp>,
    /// Time at feed of the program, minutes.
    pub total_minutes: f64,
    /// Lines of the program.
    pub program_lines: usize,
    /// The results are of the job and design as they are.
    pub stale: bool,
    pub notes: Vec<String>,
}

/// What `cam.findings` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamFindingsReply {
    pub findings: Vec<Finding>,
    /// The job or the design changed since they were found (or nothing was generated).
    pub stale: bool,
    /// The job's revision they were found for.
    pub generated_at: u64,
}

/// What `cam.finding.go` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamFindingGoReply {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub op: Option<OpRef>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub at_design: Option<[f64; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub line: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub move_index: Option<usize>,
}

/// What `cam.post` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamPostReply {
    pub passed: bool,
    pub outputs: Outputs,
    pub findings: Vec<Finding>,
    pub program_lines: usize,
    pub program_head: Vec<String>,
    pub total_minutes: f64,
}

/// What `view.manufacture_stage` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct ManufactureStageReply {
    pub stage: String,
}
