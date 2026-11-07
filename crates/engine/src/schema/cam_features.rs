//! Contracts and result types for the commands in `cmd/cam_features.rs` and `cmd/cam_suggest.rs`:
//! `cam.features` (the machinable features of a part, found on its exact B-rep) and `cam.suggest`
//! (a whole CAM job proposed from them). docs/cam-jobs.md describes both.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

/// What a part is made of, for picking tools.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum MaterialSpec {
    #[serde(alias = "aluminum")]
    Aluminium,
    Steel,
    Stainless,
    Brass,
    Plastic,
    Wood,
}

/// The direction(s) the tool comes from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum AxisSpec {
    /// A 3-axis mill: the tool comes down along work -Z only.
    #[default]
    Z,
    /// A 4th axis about work X (the A axis): every face square to X can be turned up under the
    /// tool, each at the A angle that brings its normal up.
    A,
}

/// List the machinable features of a part, found on the exact B-rep of its bodies: holes,
/// pockets, planar faces facing the tool, outer profiles, fillets and chamfers on top edges,
/// other 3D surfaces and (with `axis: "a"`) turned diameters, each with a stable id
/// (`hole:0`, `pocket:1`…) a `cam.job` operation can name instead of a sketch or polygon.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamFeaturesParams {
    /// The bodies of the part, by name. Default: every visible body.
    #[serde(default)]
    pub bodies: Option<Vec<String>>,
    /// The design point that is work zero, [x, y, z] mm (as a job's `origin`). Default: the
    /// design's origin.
    #[serde(default)]
    pub origin: Option<[f64; 3]>,
    /// `z` (3-axis, the default) or `a` (a 4th axis about work X).
    #[serde(default)]
    pub axis: Option<AxisSpec>,
    /// How close two sizes or positions must be to count as equal, mm, 0.0001 to 1 (as a job's
    /// `tolerance`). Default: 0.01.
    #[serde(default)]
    pub tolerance: Option<f64>,
}

/// What `cam.features` returns. Every coordinate of a feature is in its setup's frame: work
/// coordinates (the design less `origin`) turned by the setup's A angle, so its Z is the tool
/// axis and its numbers go straight into an operation of that setup.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamFeaturesReply {
    /// `z` or `a`.
    pub axis: AxisSpec,
    /// Work zero in design coordinates.
    pub origin: [f64; 3],
    /// The bodies looked at.
    pub bodies: Vec<String>,
    pub tolerance: f64,
    /// One setup per tool direction: `a_deg` 0 first (with `axis: "z"` the only one).
    pub setups: Vec<SetupReport>,
    pub holes: Vec<HoleReport>,
    /// Holes of the same diameter, depth and kind, for one drilling operation each.
    pub hole_groups: Vec<HoleGroup>,
    pub pockets: Vec<PocketReport>,
    /// Planar faces facing the tool.
    pub faces: Vec<FaceReport>,
    /// Outer profiles to contour round.
    pub profiles: Vec<ProfileReport>,
    /// Rounds on top edges.
    pub fillets: Vec<EdgeBreakReport>,
    /// Chamfers on top edges.
    pub chamfers: Vec<EdgeBreakReport>,
    /// Other curved areas the tool reaches from above (3D finishing).
    pub surfaces: Vec<SurfaceReport>,
    /// Diameters turned about the A axis (`axis: "a"` only).
    pub turned: Vec<TurnedReport>,
    /// What was not recognised, or not reachable along the axis.
    pub notes: Vec<String>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// One tool direction.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct SetupReport {
    /// The A angle that brings this side up, degrees (0 with `axis: "z"`).
    pub a_deg: f64,
    /// The tool axis (pointing from the part to the spindle) in work coordinates.
    pub tool_axis: [f64; 3],
    /// The part's highest Z in this setup's frame (an upper bound on curved surfaces).
    pub top_z: f64,
    /// The part's lowest Z in this setup's frame.
    pub bottom_z: f64,
    /// The part's extent in this frame's XY: [[low x, low y], [high x, high y]].
    pub bounds: [[f64; 2]; 2],
    /// With `axis: "a"`: the largest distance of the part from the A axis.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub swing_radius: Option<f64>,
    /// The ids of the features machined from this side.
    pub features: Vec<String>,
}

/// A round hole along the tool axis.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct HoleReport {
    pub id: String,
    /// The setup's A angle (with `axis: "a"`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub a_deg: Option<f64>,
    pub body: String,
    /// The axis's position in the setup's XY.
    pub center: [f64; 2],
    /// Where the axis meets the hole's top, in work coordinates.
    pub center_work: [f64; 3],
    /// The tool axis of the setup in work coordinates.
    pub axis_work: [f64; 3],
    pub diameter: f64,
    /// Where the round wall starts (the face it opens in).
    pub top_z: f64,
    /// Where the round wall ends.
    pub bottom_z: f64,
    /// `top_z - bottom_z`.
    pub depth: f64,
    /// It comes out on the other side.
    pub through: bool,
    /// `through`, `flat` (a flat floor), `cone` (a drill point) or `round` (the wall runs on into
    /// a curved floor within its radius, a round at its foot or a ball end, below `bottom_z`).
    pub floor: String,
    /// The drill point's tip (with `floor: "cone"`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tip_z: Option<f64>,
    /// The larger hole above this one that is its counterbore.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub counterbore: Option<String>,
    /// The smaller hole this one is the counterbore of.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub counterbore_of: Option<String>,
    /// A countersink at the top: its largest diameter and included angle.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub countersink: Option<Countersink>,
    /// The body's faces (as `model.faces` numbers them) that make the round wall.
    pub faces: Vec<usize>,
}

#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct Countersink {
    pub diameter: f64,
    pub angle_deg: f64,
}

#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct HoleGroup {
    pub diameter: f64,
    pub depth: f64,
    pub through: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub a_deg: Option<f64>,
    pub count: usize,
    pub ids: Vec<String>,
}

/// A closed pocket: a planar floor below the faces round it, walled all round.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct PocketReport {
    pub id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub a_deg: Option<f64>,
    pub body: String,
    /// The floor's face (as `model.faces` numbers it), also listed under `faces` as `face_id`.
    pub floor_face: usize,
    pub face_id: String,
    pub floor_z: f64,
    /// The top of its highest wall.
    pub top_z: f64,
    pub depth: f64,
    /// The floor's outline, counter-clockwise, curves within the tolerance.
    pub outline: Vec<[f64; 2]>,
    pub area: f64,
    pub bounds: [[f64; 2]; 2],
    /// The smallest radius of the outline's inside corners (where the tool must fit); 0 when
    /// a corner is sharp.
    pub min_corner_radius: f64,
    /// Sharp inside corners (a cutter leaves its own radius there).
    pub sharp_corners: usize,
    /// Every wall is square to the floor (no draft or round at its foot): the outline is the
    /// pocket's all the way up. When false the outline is the floor's, inside the walls: a 2D
    /// pocket or contour on it leaves the walls, and `cam.suggest` says so under `check`.
    pub walls_vertical: bool,
    /// Bosses standing in it.
    pub islands: Vec<IslandReport>,
}

#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct IslandReport {
    pub outline: Vec<[f64; 2]>,
    pub top_z: f64,
    pub area: f64,
}

/// A planar face facing the tool.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct FaceReport {
    pub id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub a_deg: Option<f64>,
    pub body: String,
    /// The face (as `model.faces` numbers it).
    pub face: usize,
    pub z: f64,
    pub area: f64,
    pub bounds: [[f64; 2]; 2],
    /// `top` (the highest), `plateau` (everything round it falls away), `step` (open on some
    /// sides), `pocket_floor`, `hole_floor` (a blind hole's floor or a counterbore's step).
    pub role: String,
    /// The pocket whose floor it is.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pocket: Option<String>,
    /// The face's outward normal in work coordinates.
    pub normal_work: [f64; 3],
}

/// The outline of a face whose walls fall straight away all round: what a contour runs
/// outside of, from `top_z` down to `bottom_z`.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct ProfileReport {
    pub id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub a_deg: Option<f64>,
    pub body: String,
    pub top_z: f64,
    pub bottom_z: f64,
    pub depth: f64,
    pub outline: Vec<[f64; 2]>,
    pub area: f64,
    pub bounds: [[f64; 2]; 2],
    /// The smallest radius of the outline's notches (where a tool running outside must fit);
    /// absent when it has none.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_concave_radius: Option<f64>,
    /// The face it is the outline of (`face:k`).
    pub face_id: String,
}

/// A fillet or chamfer on top edges.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct EdgeBreakReport {
    pub id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub a_deg: Option<f64>,
    pub body: String,
    pub faces: Vec<usize>,
    /// A fillet's radius.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub radius: Option<f64>,
    /// A chamfer's angle from the tool axis, degrees.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub angle_deg: Option<f64>,
    pub top_z: f64,
    pub bottom_z: f64,
    pub bounds: [[f64; 2]; 2],
}

/// Curved faces for 3D finishing.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct SurfaceReport {
    pub id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub a_deg: Option<f64>,
    pub body: String,
    pub faces: Vec<usize>,
    /// The kinds of surface: `sphere`, `torus`, `cylinder`, `cone`, `freeform`, `plane`
    /// (sloped).
    pub kinds: Vec<String>,
    /// The smallest radius of curvature of its analytic faces (a ball's radius should not
    /// exceed it in the concave ones); absent for free-form faces only.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_radius: Option<f64>,
    pub top_z: f64,
    pub bottom_z: f64,
    pub bounds: [[f64; 2]; 2],
}

/// A diameter turned about the A axis.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct TurnedReport {
    pub id: String,
    pub body: String,
    pub diameter: f64,
    pub x_min: f64,
    pub x_max: f64,
    pub faces: Vec<usize>,
}

/// Propose a whole CAM job for a part from its machinable features (`cam.features`): facing,
/// pockets roughed and their walls finished, holes, outer profiles and 3D finishing, with tools
/// picked from the library by size. The job is returned, not run: pass it to `cam.job`.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamSuggestParams {
    /// The bodies of the part. Default: every visible body.
    #[serde(default)]
    pub bodies: Option<Vec<String>>,
    /// Work zero, [x, y, z] in design coordinates. Default: the design's origin.
    #[serde(default)]
    pub origin: Option<[f64; 3]>,
    /// `z` or `a`. Default: `a` when the profile's machine has an A axis, else `z`.
    #[serde(default)]
    pub axis: Option<AxisSpec>,
    /// The job's tolerance, mm. Default: 0.01.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// A machine profile: the same file `cam.job` takes as its `profile_path` (machine, post,
    /// tool libraries, default heights and feeds; relative paths are read from the file's
    /// folder), whose path the proposed job names; or a JSON array of tools (a tool library).
    /// Give this, `tools` or `tool_library_path`.
    #[serde(default)]
    pub profile_path: Option<String>,
    /// The tools to choose from, as a job's `tools`.
    #[serde(default)]
    pub tools: Option<Vec<Map<String, Value>>>,
    /// A JSON file holding an array of tools.
    #[serde(default)]
    pub tool_library_path: Option<String>,
    /// A built-in post (`cam.posts`). Default: the profile's, else `linuxcnc_4axis`.
    #[serde(default)]
    pub post: Option<String>,
    /// Material above the part's top to face off, mm, 0 to 5 (0: no facing). Default: 0.5.
    #[serde(default)]
    pub stock_top: Option<f64>,
    /// What the part is made of: tools whose description names it (`Al`, `aluminium`,
    /// `steel`, …) are picked first, and for metals carbide before HSS. Default: none (carbide
    /// before HSS).
    #[serde(default)]
    pub material: Option<MaterialSpec>,
    /// Also run the proposed job through `cam.job` as a dry run and return its report under
    /// `dry_run`. Default: false.
    #[serde(default)]
    pub check: Option<bool>,
}

/// What `cam.suggest` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamSuggestReply {
    /// The proposed job, ready for `cam.job {job, dry_run: true}` and then `{job, output}`.
    pub job: Value,
    /// Each operation of the job, with the features it machines and why it was chosen.
    pub operations: Vec<SuggestedOperation>,
    /// The tools the job uses and what for.
    pub tools: Vec<ToolChoice>,
    /// What the proposal assumes (stock, work holding, work zero…).
    pub assumptions: Vec<String>,
    /// What to check before running the program on the machine.
    pub check: Vec<String>,
    /// Features the job does not machine, and why.
    pub skipped: Vec<String>,
    /// With `check: true`: the `cam.job` dry run's report.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dry_run: Option<Value>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct SuggestedOperation {
    pub name: String,
    /// The operation's `type` (an indexed one: `indexed <type>`).
    #[serde(rename = "type")]
    pub kind: String,
    pub tool: u32,
    pub features: Vec<String>,
    pub why: String,
}

#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct ToolChoice {
    pub number: u32,
    pub kind: String,
    pub diameter: f64,
    pub used_for: Vec<String>,
}

#[cfg(test)]
#[path = "cam_features_tests.rs"]
mod tests;
