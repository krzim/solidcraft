//! The operation form's descriptor table (Manufacture workspace): for every key an operation of
//! a job can have, the form section it is in, what kind of value it takes and its label. The
//! form shows the keys of the selected operation's type in the order the engine's schema gives
//! them (`schema::cam_picks::op_keys`). A key an agent can write that the form does not show is
//! on [`AGENT_ONLY_KEYS`] with the reason; `manufacture_op_tests` holds every key of every
//! operation type's schema to one or the other, so a new engine key fails until the form shows it.

/// The form's sections, in the order they are drawn.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum Section {
    Geometry,
    Tool,
    Heights,
    Cutting,
    Entry,
    Specific,
    Advanced,
}

impl Section {
    pub(crate) const ALL: [Section; 7] =
        [Section::Geometry, Section::Tool, Section::Heights, Section::Cutting, Section::Entry, Section::Specific, Section::Advanced];

    pub(crate) fn label(self) -> &'static str {
        match self {
            Section::Geometry => "ui.manufacture.section.geometry",
            Section::Tool => "ui.manufacture.section.tool",
            Section::Heights => "ui.manufacture.section.heights",
            Section::Cutting => "ui.manufacture.section.cutting",
            Section::Entry => "ui.manufacture.section.entry",
            Section::Specific => "ui.manufacture.section.specific",
            Section::Advanced => "ui.manufacture.section.advanced",
        }
    }
}

/// What a key's value is, and so how its row edits it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Kind {
    /// A number (mm, degrees, rev/min…).
    Number,
    /// A whole number.
    Count,
    /// On or off.
    Flag,
    /// One of these words.
    Choice(&'static [&'static str]),
    /// A height: a number, or `{from, offset?}` following a face.
    Height,
    /// A name (a sketch, a curve, a feature id).
    Text,
    /// A list or an object, typed as JSON.
    Json,
    /// The tool: the tool list.
    ToolList,
    /// The geometry picked in the view.
    Picks,
    /// Edited elsewhere in the form (the name, on/off).
    Header,
}

/// One key of the form.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Field {
    /// The key, or a dotted path into `heights` or `feeds`.
    pub key: &'static str,
    pub section: Section,
    pub kind: Kind,
}

const fn f(key: &'static str, section: Section, kind: Kind) -> Field {
    Field { key, section, kind }
}

use Kind::*;
use Section::*;

/// Every key the form shows. `heights` and `feeds` are shown key by key.
pub(crate) const FIELDS: &[Field] = &[
    f("name", Geometry, Header),
    f("enabled", Geometry, Header),
    f("picks", Geometry, Picks),
    f("sketch", Geometry, Text),
    f("profiles", Geometry, Json),
    f("polygon", Geometry, Json),
    f("feature", Geometry, Text),
    f("curve", Geometry, Text),
    f("points", Geometry, Json),
    f("holes", Geometry, Json),
    f("centers", Geometry, Json),
    f("circles", Geometry, Flag),
    f("sketch_points", Geometry, Flag),
    f("diameter_range", Geometry, Json),
    f("radius", Geometry, Number),
    f("boundary", Geometry, Json),
    f("range", Geometry, Json),
    f("rect", Geometry, Json),
    f("tool", Tool, ToolList),
    f("heights.clearance", Heights, Height),
    f("heights.retract", Heights, Height),
    f("heights.top", Heights, Height),
    f("heights.bottom", Heights, Height),
    f("clearance", Heights, Number),
    f("retract", Heights, Number),
    f("top", Heights, Height),
    f("bottom", Heights, Height),
    f("floor", Heights, Number),
    f("depth", Heights, Number),
    f("stock_radius", Heights, Number),
    f("stepdown", Cutting, Number),
    f("stepover", Cutting, Number),
    f("optimal_load", Cutting, Number),
    f("scallop_height", Cutting, Number),
    f("stock_to_leave", Cutting, Number),
    f("side", Cutting, Choice(&["outside", "inside", "on"])),
    f("climb", Cutting, Flag),
    f("angle_deg", Cutting, Number),
    f("zigzag", Cutting, Flag),
    f("direction", Cutting, Choice(&["inward", "outward"])),
    f("spindle_rpm", Cutting, Number),
    f("feeds.cut", Cutting, Number),
    f("feeds.plunge", Cutting, Number),
    f("coolant", Cutting, Flag),
    f("ramp_angle_deg", Entry, Number),
    f("helix_diameter", Entry, Number),
    f("entry", Entry, Json),
    f("lead", Entry, Json),
    f("vertical_lead", Entry, Number),
    f("lead_radius", Entry, Number),
    f("overhang", Entry, Number),
    f("linking", Entry, Choice(&["clearance", "stay_down"])),
    f("tabs", Entry, Json),
    f("finish_passes", Entry, Count),
    f("finish_stepover", Entry, Number),
    f("spring_pass", Entry, Flag),
    f("cycle", Specific, Choice(&["simple", "dwell", "peck", "chip_break"])),
    f("peck", Specific, Number),
    f("peck_clearance", Specific, Number),
    f("back_off", Specific, Number),
    f("dwell_seconds", Specific, Number),
    f("to_full_diameter", Specific, Flag),
    f("stay_at_retract", Specific, Flag),
    f("pitch", Specific, Number),
    f("thread_diameter", Specific, Number),
    f("thread_depth", Specific, Number),
    f("internal", Specific, Flag),
    f("right_hand", Specific, Flag),
    f("radial_passes", Specific, Count),
    f("width", Specific, Number),
    f("tip_offset", Specific, Number),
    f("pattern", Specific, Json),
    f("previous_tool", Specific, Count),
    f("threshold", Specific, Number),
    f("threshold_deg", Specific, Number),
    f("overlap", Specific, Number),
    f("shallow", Specific, Choice(&["scallop", "parallel"])),
    f("min_angle_deg", Specific, Number),
    f("passes", Specific, Count),
    f("min_length", Specific, Number),
    f("min_engagement_deg", Advanced, Number),
    f("max_engagement_deg", Advanced, Number),
    f("flats", Advanced, Flag),
    f("tolerance", Advanced, Number),
    f("step", Advanced, Number),
    f("descent_limit_deg", Advanced, Number),
    f("max_angle_step_deg", Advanced, Number),
    f("gouge_check", Advanced, Flag),
];

/// Keys of an operation the form does not show, each with why: an agent writes them.
pub(crate) const AGENT_ONLY_KEYS: &[(&str, &str)] = &[
    ("type", "the operation's kind, chosen when it is added (a new kind is a new operation)"),
    ("compare", "plans a pocket beside the light-load clearing to compare the two: a check of the planner, not of the job"),
];

/// The field of `key` (a top-level key or a dotted `heights.`/`feeds.` path).
pub(crate) fn field(key: &str) -> Option<&'static Field> {
    FIELDS.iter().find(|f| f.key == key)
}

/// The form's rows for operation type `ty`, in the schema's order, `heights` and `feeds` taken
/// apart (a chamfer's heights have no bottom).
pub(crate) fn rows(ty: &str) -> Vec<&'static Field> {
    let mut out = Vec::new();
    for (key, _) in solidcraft_engine::schema::cam_picks::op_keys(ty) {
        match key.as_str() {
            "heights" => {
                let keys: &[&str] = if ty == "chamfer" { &["clearance", "retract", "top"] } else { &["clearance", "retract", "top", "bottom"] };
                out.extend(keys.iter().filter_map(|k| field(&format!("heights.{k}"))));
            }
            "feeds" => out.extend(["feeds.cut", "feeds.plunge"].iter().filter_map(|k| field(k))),
            other => out.extend(field(other)),
        }
    }
    out
}

/// The label message of a key.
pub(crate) fn label_id(key: &str) -> String {
    let key = key.strip_prefix("heights.").unwrap_or(key);
    format!("ui.manufacture.field.{}", key.replace('.', "_"))
}

/// Operation types that can be turned to an A angle (the 3-axis kinds an indexed operation holds).
pub(crate) const INDEXABLE: &[&str] = &[
    "adaptive",
    "adaptive_roughing",
    "contour",
    "pocket",
    "parallel",
    "roughing",
    "waterline",
    "drill",
    "chamfer",
    "scallop",
    "steep_shallow",
    "pencil",
    "rest_finish",
];

#[cfg(test)]
#[path = "manufacture_op_tests.rs"]
mod tests;
