//! Picks: the pieces of the design a machining operation works on, stored by their persistent
//! names (`cmd/cam_picks.rs` finds them again), heights that follow a face, how work zero was
//! set (`origin_from`), and the replies of `cam.picks.resolve`, `cam.op.defaults`,
//! `cam.setup.zero` and `cam.op.suggest`. docs/cam-jobs.md, "Picks", describes them.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use super::cam_agent::OpRef;
use super::cam_job_edit::{JobSummary, OpSelect};

/// A piece of the design, found again by its persistent name: a face or an edge of a body (with
/// a point on it, the fallback when the name no longer finds it), or a sketch's profile, curve or
/// point.
#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Pick {
    /// The body the face or edge is on, by name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub body: Option<String>,
    /// The face's persistent name (as `model.faces` gives it).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub face: Option<String>,
    /// The edge's persistent name (as `model.edges` gives it).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub edge: Option<String>,
    /// A point on the face or edge, design coordinates (mm): where it is looked for when the name
    /// finds nothing.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub at: Option<[f64; 3]>,
    /// A sketch, by id or name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sketch: Option<String>,
    /// With `sketch`: a profile of it, by index.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub profile: Option<usize>,
    /// With `sketch`: a curve of it, by id.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub curve: Option<String>,
    /// With `sketch`: a point of it, by id.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub point: Option<String>,
}

/// What a height follows: a picked planar face, or one of `stock_top`, `part_top`,
/// `part_bottom`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(untagged)]
pub enum HeightSource {
    Named(String),
    Face(Pick),
}

/// A height that follows the model: `{from, offset?}`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct HeightRef {
    /// A planar face square to the tool axis (a pick), or `stock_top`, `part_top`, `part_bottom`.
    pub from: HeightSource,
    /// Added to the height it follows, mm (negative: below it). Default: 0.
    #[serde(default)]
    pub offset: Option<f64>,
}

/// A height: a number (mm, in the operation's frame), or `{from, offset?}`, a height that
/// follows a face of the model (worked out each time the job is planned).
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(untagged)]
pub enum HeightValue {
    Number(f64),
    From(HeightRef),
}

/// One of three places along an axis of the stock.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum GridPlace {
    Low,
    Middle,
    High,
    /// Z only: the stock's top.
    Top,
    /// Z only: the stock's bottom.
    Bottom,
}

/// A place on the stock: X and Y low, middle or high, Z top, middle or bottom.
#[derive(Clone, Copy, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StockGrid {
    pub x: GridPlace,
    pub y: GridPlace,
    pub z: GridPlace,
}

/// Where along the A axis work zero is: a picked face or edge, or an end of the stock.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(untagged)]
pub enum XEnd {
    /// `stock_low` or `stock_high`.
    Stock(String),
    Pick(Pick),
}

/// How work zero was set, kept beside the numbers in `origin` so a change of the model or the
/// stock that moves it is reported (never followed by itself).
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum OriginFrom {
    /// A picked vertex, a circular edge's centre, or a point on a face.
    Pick(Pick),
    /// A place on the stock.
    Stock(StockGrid),
    /// On a 4th axis: the axis of a picked cylinder face, at `x_end` along it.
    Axis { axis: Pick, x_end: XEnd },
}

/// How one pick of an operation was found again.
#[derive(Clone, Debug, Default, Serialize, JsonSchema)]
pub struct PickResolved {
    /// The operation (none: the job's work zero, `origin_from`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub op: Option<OpRef>,
    /// Which pick of the operation, from 0.
    pub pick: usize,
    /// `name` (by its persistent name), `point` (by the point stored with it) or `lost`.
    pub by: String,
    /// Its body.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub body: Option<String>,
    /// The face found (as `model.faces` numbers it).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub face: Option<usize>,
    /// The edge found (as `model.edges` numbers it).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub edge: Option<usize>,
    /// The `cam.features` id it belongs to.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feature: Option<String>,
    /// What to look at: found by position rather than name, or not found.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

/// Find picks again and say what they are and which operations suit them.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamPicksResolveParams {
    /// The picks. Give these or `from_selection`.
    #[serde(default)]
    pub picks: Option<Vec<Pick>>,
    /// Take the faces and edges selected in the design instead. Default: false.
    #[serde(default)]
    pub from_selection: Option<bool>,
    /// Say how this operation type reads them (`pocket`, `drill`...). Default: every type.
    #[serde(default, rename = "for")]
    pub for_type: Option<String>,
}

/// One pick as `cam.picks.resolve` read it.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct PickRead {
    /// The pick (as an operation stores it).
    pub pick: Pick,
    /// `name`, `point` or `lost`.
    pub by: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub face: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub edge: Option<usize>,
    /// `planar`, `cylinder`, `cone`, `sphere`, `torus`, `other`, `edge`, `circle`, `sketch`.
    pub kind: String,
    /// The `cam.features` id it belongs to.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub feature: Option<String>,
    /// What it is, in words: "pocket 16 x 10 mm, 5 deep".
    pub reads: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

/// An operation that suits the picks.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct Suit {
    /// The operation's `type`.
    #[serde(rename = "type")]
    pub kind: String,
    /// Its label, as the Manufacture workspace shows it.
    pub label: String,
    /// The tool it would take (none when the job has none that fits).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool: Option<u32>,
    /// Why it suits, and why the tool.
    pub why: String,
}

/// What `cam.picks.resolve` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamPicksResolveReply {
    pub picks: Vec<PickRead>,
    /// The operations that suit them, best first.
    pub suits: Vec<Suit>,
}

/// Set work zero (one job undo step), by numbers, a pick, a place on the stock or a cylinder's
/// axis; with `refresh`, again from the way it was set (`origin_from`).
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamSetupZeroParams {
    /// Work zero at this design point [x, y, z], mm.
    #[serde(default)]
    pub at: Option<[f64; 3]>,
    /// Work zero at a pick: a vertex, a circular edge's centre, or the point on a face.
    #[serde(default)]
    pub pick: Option<Pick>,
    /// Work zero on the stock: `{x: low|middle|high, y: …, z: top|middle|bottom}`.
    #[serde(default)]
    pub stock: Option<StockGrid>,
    /// On a 4th axis: work zero on the axis of this cylinder face, at `x_end`.
    #[serde(default)]
    pub axis: Option<Pick>,
    /// With `axis`: `stock_low`, `stock_high` or a picked face or edge. Default: `stock_low`.
    #[serde(default)]
    pub x_end: Option<XEnd>,
    /// Work zero again from the job's `origin_from`, after the model or the stock moved it.
    #[serde(default)]
    pub refresh: Option<bool>,
    /// Keep how it was set (`origin_from`) in the job. Default: true.
    #[serde(default)]
    pub keep_ref: Option<bool>,
}

/// What `cam.setup.zero` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamSetupZeroReply {
    /// Work zero, design coordinates.
    pub origin: [f64; 3],
    /// How it was set (absent: by numbers).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub origin_from: Option<Value>,
    /// How far it moved, mm.
    pub moved: [f64; 3],
    /// What else changed with it (a box or bar stock kept where it was).
    pub notes: Vec<String>,
    pub summary: JobSummary,
}

/// The values an operation runs with and where each comes from.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamOpDefaultsParams {
    /// An operation of the open job. Give this or `draft`.
    #[serde(default)]
    pub op: Option<OpSelect>,
    /// An operation not in the job yet (or a changed copy of one): `{type, …}`.
    #[serde(default)]
    pub draft: Option<Map<String, Value>>,
    /// Plan it to work out the tool, heights and feeds (slower). Default: true.
    #[serde(default)]
    pub plan: Option<bool>,
}

/// One value of an operation.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct FieldDefault {
    /// The key, a dotted path (`heights.bottom`, `feeds.cut`).
    pub path: String,
    /// The value it runs with (absent: worked out when it is planned, see `why`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value: Option<Value>,
    /// `given` (in the job), `tool`, `cutting_data`, `profile`, `model`, `stock`, `feature`,
    /// `pick` or `rule`.
    pub source: String,
    /// Where it comes from, in words.
    pub why: String,
}

/// A tool of the job as an operation sees it.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct ToolFit {
    pub number: u32,
    /// "T6 6 mm flat end mill, 20 mm flutes".
    pub label: String,
    /// The operation takes this kind of tool and it fits the geometry.
    pub fits: bool,
    /// Why it does not fit.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub why_not: Option<String>,
}

/// What `cam.op.defaults` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamOpDefaultsReply {
    /// The operation's `type` (an indexed one: its inner type).
    #[serde(rename = "type")]
    pub kind: String,
    pub fields: Vec<FieldDefault>,
    /// The tool it cuts with and why.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_why: Option<String>,
    /// The job's tools, those it can take first.
    pub tools: Vec<ToolFit>,
    pub picks_resolved: Vec<PickResolved>,
    /// Why it could not be planned (the values then come from the rules alone).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refused: Option<String>,
}

/// Add the operations `cam.suggest` proposes for the part to the open job (one job undo step),
/// their geometry stored as picks.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamOpSuggestParams {
    /// Replace the job's operations instead of adding after them. Default: false.
    #[serde(default)]
    pub replace: Option<bool>,
    /// How much material is on the part's top to face off, mm. Default: 0.5.
    #[serde(default)]
    pub stock_top: Option<f64>,
}

/// What `cam.op.suggest` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamOpSuggestReply {
    /// The operations added, by name, with why each.
    pub added: Vec<String>,
    /// What it could not suggest an operation for.
    pub skipped: Vec<String>,
    /// What to check.
    pub check: Vec<String>,
    pub summary: JobSummary,
}

/// The JSON schema of operation type `ty`'s own keys (`type` aside), as an agent reads it: what
/// the Manufacture workspace's operation form must show (`None`: not an operation type).
pub fn op_schema(ty: &str) -> Option<Value> {
    use super::cam::*;
    use super::cam_adaptive::{AdaptiveOp, AdaptiveRoughingOp};
    use super::cam_stock::FaceOp;
    let schema = match ty {
        "adaptive" => schemars::schema_for!(AdaptiveOp),
        "adaptive_roughing" => schemars::schema_for!(AdaptiveRoughingOp),
        "contour" => schemars::schema_for!(ContourOp),
        "pocket" => schemars::schema_for!(PocketOp),
        "parallel" => schemars::schema_for!(ParallelOp),
        "roughing" => schemars::schema_for!(RoughingOp),
        "waterline" => schemars::schema_for!(WaterlineOp),
        "drill" => schemars::schema_for!(DrillOp),
        "scallop" => schemars::schema_for!(ScallopOp),
        "steep_shallow" => schemars::schema_for!(SteepShallowOp),
        "pencil" => schemars::schema_for!(PencilOp),
        "rest_finish" => schemars::schema_for!(RestFinishOp),
        "rotary_finish" => schemars::schema_for!(RotaryFinishOp),
        "rotary_roughing" => schemars::schema_for!(RotaryRoughingOp),
        "wrap" => schemars::schema_for!(WrapOp),
        "bore" => schemars::schema_for!(BoreOp),
        "thread_mill" => schemars::schema_for!(ThreadMillOp),
        "face" => schemars::schema_for!(FaceOp),
        "chamfer" => schemars::schema_for!(ChamferOp),
        "wrap_pocket" => schemars::schema_for!(WrapPocketOp),
        "wrap_contour" => schemars::schema_for!(WrapContourOp),
        "indexed" => schemars::schema_for!(IndexedOp),
        "indexed_pattern" => schemars::schema_for!(IndexedPatternOp),
        _ => return None,
    };
    serde_json::to_value(schema).ok()
}

/// Operation type `ty`'s own keys and each one's description, in the schema's order.
pub fn op_keys(ty: &str) -> Vec<(String, String)> {
    let Some(schema) = op_schema(ty) else { return Vec::new() };
    let Some(props) = schema.get("properties").and_then(Value::as_object) else { return Vec::new() };
    props.iter().map(|(k, v)| (k.clone(), v.get("description").and_then(Value::as_str).unwrap_or_default().to_string())).collect()
}

/// The "Default: …" sentence of a key's description (empty when it has none).
pub fn default_rule(description: &str) -> String {
    match description.find("Default:") {
        Some(i) => description[i + "Default:".len()..].split_whitespace().collect::<Vec<_>>().join(" ").trim_end_matches('.').to_string(),
        None => String::new(),
    }
}
