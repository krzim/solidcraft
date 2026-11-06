//! Contracts and result types for the commands in `cmd/cam.rs`: `cam.job`, `cam.posts` and
//! `cam.check`. The job is the CAM setup of a design (machine, post, tools, operations), kept in
//! a JSON file next to the design rather than in it; docs/cam-jobs.md describes it in full.
//!
//! The handler reads its parameters through these same types, so what the contract accepts is
//! what the command accepts. Every object of the job refuses keys it does not know, so a
//! misspelt key is an error rather than a setting silently left at its default. The objects the
//! CAM crate defines (tools, a machine, a post) are taken as JSON here and read by the handler,
//! which refuses their unknown keys the same way.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use super::cam_rates::{MachineRatesSpec, MachineTimeReport};
use super::types::SketchRef;

/// Plan and check a CAM job and write its G-code: toolpaths for every operation on the design's
/// bodies, the program written through the job's post for its machine, read back and compared
/// with the toolpaths, and checked for gouges and holder collisions. Refuses, writing nothing,
/// when a check fails, unless `allow_unsafe`. The job is described in docs/cam-jobs.md.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamJobParams {
    /// The job itself (see docs/cam-jobs.md). Give this or `job_path`.
    #[serde(default)]
    pub job: Option<CamJob>,
    /// A `.json` file holding the job. Relative `post_path` and `tool_library_path` in it are
    /// read from the file's folder.
    #[serde(default)]
    pub job_path: Option<String>,
    /// Where to write the G-code. Required unless `dry_run`.
    #[serde(default)]
    pub output: Option<String>,
    /// Where to write a self-contained HTML page showing the part and every toolpath (no
    /// network access). Default: no page.
    #[serde(default)]
    pub preview: Option<String>,
    /// Plan and check the job, and report, but write no file; the report says whether the
    /// checks pass instead of refusing. Default: false.
    #[serde(default)]
    pub dry_run: Option<bool>,
    /// Write the program even when the read-back, gouge or holder check fails (the report says
    /// which). Default: false.
    #[serde(default)]
    pub allow_unsafe: Option<bool>,
}

/// A CAM job: what is machined, on which machine, through which post, with which tools.
#[derive(Clone, Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamJob {
    /// The program's name, written in its header. Default: the design's name.
    #[serde(default)]
    pub name: Option<String>,
    /// `3axis`, `4axis_a` (a rotary table about +X, the A axis, through work zero), or a whole
    /// machine as the CAM crate's `Machine` JSON.
    pub machine: MachineSpec,
    /// With `4axis_a`: the controller's positive A turns the table the other way. Default: false.
    #[serde(default)]
    pub a_reversed: Option<bool>,
    /// With `4axis_a`: the lowest and highest A the table reaches, in degrees. Default: it turns
    /// without end.
    #[serde(default)]
    pub a_limits_deg: Option<[f64; 2]>,
    /// A built-in post by name (`cam.posts` lists them). Give this or `post_path`.
    #[serde(default)]
    pub post: Option<String>,
    /// A post as a JSON file (the CAM crate's `Post5`, as `cam.posts {name}` returns one).
    #[serde(default)]
    pub post_path: Option<String>,
    /// The work offset the program runs in, 1 to 6 (G54 to G59), for posts that write one.
    /// Default: 1.
    #[serde(default)]
    pub work_offset: Option<u8>,
    /// The design point that is work zero, [x, y, z] in mm. On a 4th axis the rotary axis is
    /// work X through work zero. Default: the design's origin.
    #[serde(default)]
    pub origin: Option<[f64; 3]>,
    /// The bodies that make up the part, by name. Default: every visible body not in `fixtures`.
    #[serde(default)]
    pub bodies: Option<Vec<String>>,
    /// Bodies the tool must stay clear of (vises, clamps): checked for gouges and holder
    /// collisions, never machined. Default: none.
    #[serde(default)]
    pub fixtures: Option<Vec<String>>,
    /// The machining tolerance in mm, 0.0001 to 1: how far a toolpath may be from the part, and
    /// how deep a cut into it counts as a gouge. The bodies are tessellated at half of it.
    /// Default: 0.01.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// The tools, as the CAM crate's `Tool` JSON (`number`, `kind`, `diameter`, `flute_length`,
    /// ...). Give this or `tool_library_path`.
    #[serde(default)]
    pub tools: Option<Vec<Map<String, Value>>>,
    /// A JSON file holding an array of tools, as `tools`.
    #[serde(default)]
    pub tool_library_path: Option<String>,
    /// The operations, run and written in this order.
    pub operations: Vec<OperationSpec>,
    /// The machine's top rates (from its controller's settings): the reply then times the
    /// program as the machine runs it and warns about blocks the controller would slow. Give
    /// this or `machine_rates_path`. Default: none.
    #[serde(default)]
    pub machine_rates: Option<MachineRatesSpec>,
    /// A JSON file holding `machine_rates`, read from the job file's folder when relative.
    #[serde(default)]
    pub machine_rates_path: Option<String>,
}

/// A machine: a preset's name or the CAM crate's `Machine` JSON.
#[derive(Clone, Debug, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum MachineSpec {
    /// `3axis` or `4axis_a`.
    Preset(String),
    /// The CAM crate's `Machine` JSON: `name`, `rotaries`, `linear_limits?`, `work_origin?`.
    Custom(Map<String, Value>),
}

/// The Z levels of an operation, in the operation's coordinates (work coordinates, or the
/// indexed frame of an indexed operation), mm.
#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct HeightsSpec {
    /// Where the tool travels between cuts.
    pub clearance: f64,
    /// Where rapid motion towards the material stops and feed begins.
    pub retract: f64,
    /// Top of the material to cut.
    pub top: f64,
    /// Final depth (for a parallel finish: the floor the tool never goes below).
    pub bottom: f64,
}

/// Feed rates in mm/min.
#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FeedsSpec {
    /// The cutting feed.
    pub cut: f64,
    /// The feed going down into the material.
    pub plunge: f64,
}

/// Where a 3D operation's cutter centre may go, seen along the tool axis, in the operation's
/// coordinates.
#[derive(Clone, Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BoundarySpec {
    /// The low and high corners [[x, y], [x, y]] of a rectangle. Default: the part's extent.
    #[serde(default)]
    pub rect: Option<[[f64; 2]; 2]>,
    /// A closed polygon [[x, y], ...] the centre must also stay inside.
    #[serde(default)]
    pub polygon: Option<Vec<[f64; 2]>>,
}

/// Which side of a profile a contour runs on.
#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum SideSpec {
    /// Around the profile: the profile is the part that stays.
    Outside,
    /// Inside the profile: the profile is the opening.
    Inside,
    /// The tool centre on the profile.
    On,
}

/// How the passes of a rotary operation run.
#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum PatternSpec {
    /// Passes along X, at most `angle_step_deg` apart round the axis.
    Lines {
        /// Degrees between passes round the axis.
        angle_step_deg: f64,
        /// Every other pass runs back. Default: true.
        #[serde(default)]
        zigzag: Option<bool>,
    },
    /// One pass going round and round, `pitch` mm along +X per turn.
    Helix {
        /// Advance along X per turn, mm.
        pitch: f64,
    },
}

/// Declares an operation's struct with the fields every operation has, then its own.
macro_rules! operation {
    ($(#[$doc:meta])* $name:ident { $($(#[$fdoc:meta])* pub $field:ident : $ty:ty,)* }) => {
        $(#[$doc])*
        #[derive(Clone, Debug, Deserialize, JsonSchema)]
        #[serde(deny_unknown_fields)]
        pub struct $name {
            /// The operation's name, written as a comment in the program.
            pub name: String,
            /// The `number` of the tool it cuts with.
            pub tool: u32,
            /// Spindle speed, rev/min. Default: the tool's `spindle_rpm`.
            #[serde(default)]
            pub spindle_rpm: Option<f64>,
            /// Feed rates. Default: the tool's `feed` and `plunge_feed`.
            #[serde(default)]
            pub feeds: Option<FeedsSpec>,
            /// Coolant on while it cuts. Default: false.
            #[serde(default)]
            pub coolant: Option<bool>,
            /// Check this operation for gouges into the part. Turn it off only for a cut the
            /// model does not show (an engraving); the holder and read-back checks still run.
            /// Default: true.
            #[serde(default)]
            pub gouge_check: Option<bool>,
            $($(#[$fdoc])* pub $field: $ty,)*
        }

        impl $name {
            pub fn common(&self) -> Common<'_> {
                Common {
                    name: &self.name,
                    tool: self.tool,
                    spindle_rpm: self.spindle_rpm,
                    feeds: self.feeds,
                    coolant: self.coolant.unwrap_or(false),
                    gouge_check: self.gouge_check.unwrap_or(true),
                }
            }
        }
    };
}

/// What every operation has.
#[derive(Clone, Copy, Debug)]
pub struct Common<'a> {
    pub name: &'a str,
    pub tool: u32,
    pub spindle_rpm: Option<f64>,
    pub feeds: Option<FeedsSpec>,
    pub coolant: bool,
    pub gouge_check: bool,
}

operation! {
    /// A 2D contour round closed profiles, in depth passes.
    ContourOp {
        /// The sketch whose closed profiles are cut round (on a plane square to the tool axis,
        /// facing it). Give this or `polygon`.
        #[serde(default)]
        pub sketch: Option<SketchRef>,
        /// With `sketch`: which of its profiles, by index. Default: every drawn profile.
        #[serde(default)]
        pub profiles: Option<Vec<usize>>,
        /// One closed polygon [[x, y], ...] in the operation's XY.
        #[serde(default)]
        pub polygon: Option<Vec<[f64; 2]>>,
        /// The Z levels.
        pub heights: HeightsSpec,
        /// Most depth per pass, mm. Default: the tool's `stepdown`, else half its diameter.
        #[serde(default)]
        pub stepdown: Option<f64>,
        /// Which side the tool runs on. Default: outside.
        #[serde(default)]
        pub side: Option<SideSpec>,
        /// Climb milling. Default: true.
        #[serde(default)]
        pub climb: Option<bool>,
        /// Material left on the wall, mm. Default: 0.
        #[serde(default)]
        pub stock_to_leave: Option<f64>,
        /// Steepest ramp into each pass along the profile, degrees; 0 plunges. Default: 0.
        #[serde(default)]
        pub ramp_angle_deg: Option<f64>,
        /// Chord error on curved profiles, mm. Default: the job's tolerance.
        #[serde(default)]
        pub tolerance: Option<f64>,
    }
}

operation! {
    /// A 2D pocket: clears the inside of closed profiles, in depth passes.
    PocketOp {
        /// The sketch whose closed profiles are pocketed (on a plane square to the tool axis,
        /// facing it). Give this or `polygon`.
        #[serde(default)]
        pub sketch: Option<SketchRef>,
        /// With `sketch`: which of its profiles, by index. Default: every drawn profile.
        #[serde(default)]
        pub profiles: Option<Vec<usize>>,
        /// One closed polygon [[x, y], ...] in the operation's XY.
        #[serde(default)]
        pub polygon: Option<Vec<[f64; 2]>>,
        /// The Z levels.
        pub heights: HeightsSpec,
        /// Most depth per pass, mm. Default: the tool's `stepdown`, else half its diameter.
        #[serde(default)]
        pub stepdown: Option<f64>,
        /// Distance between passes, mm, at most the tool diameter. Default: the tool's
        /// `stepover`, else 40% of its diameter.
        #[serde(default)]
        pub stepover: Option<f64>,
        /// Climb milling. Default: true.
        #[serde(default)]
        pub climb: Option<bool>,
        /// Material left on the walls, mm. Default: 0.
        #[serde(default)]
        pub stock_to_leave: Option<f64>,
        /// Steepest ramp into uncut material, degrees; 0 plunges. Default: 0.
        #[serde(default)]
        pub ramp_angle_deg: Option<f64>,
        /// Chord error on curved profiles, mm. Default: the job's tolerance.
        #[serde(default)]
        pub tolerance: Option<f64>,
    }
}

operation! {
    /// 3D raster finishing: passes side by side following the part's surface.
    ParallelOp {
        /// The Z levels: `bottom` is the floor, `top` is not used.
        pub heights: HeightsSpec,
        /// Distance between passes, mm. Default: the tool's `stepover`, else 10% of its diameter.
        #[serde(default)]
        pub stepover: Option<f64>,
        /// Direction of the passes, degrees counter-clockwise from +X. Default: 0.
        #[serde(default)]
        pub angle_deg: Option<f64>,
        /// Where the cutter centre may go. Default: the part's extent.
        #[serde(default)]
        pub boundary: Option<BoundarySpec>,
        /// Material left on the surface, mm. Default: 0.
        #[serde(default)]
        pub stock_to_leave: Option<f64>,
        /// Distance between samples along a pass before refinement, mm. Default: a quarter of
        /// the tool's diameter.
        #[serde(default)]
        pub step: Option<f64>,
        /// Largest distance from the surface, mm. Default: the job's tolerance.
        #[serde(default)]
        pub tolerance: Option<f64>,
        /// Every other pass runs back. Default: true.
        #[serde(default)]
        pub zigzag: Option<bool>,
    }
}

operation! {
    /// 3D roughing of a stock box in levels, staying `stock_to_leave` clear of the part.
    RoughingOp {
        /// The Z levels: `top` is the stock's top, `bottom` the lowest level.
        pub heights: HeightsSpec,
        /// Most material per level, mm. Default: the tool's `stepdown`, else half its diameter.
        #[serde(default)]
        pub stepdown: Option<f64>,
        /// Distance between passes, mm. Default: the tool's `stepover`, else 40% of its diameter.
        #[serde(default)]
        pub stepover: Option<f64>,
        /// Direction of the passes, degrees counter-clockwise from +X. Default: 0.
        #[serde(default)]
        pub angle_deg: Option<f64>,
        /// The stock's extent as the cutter centre may reach it. Default: the part's extent.
        #[serde(default)]
        pub boundary: Option<BoundarySpec>,
        /// Material left on the part, mm. Default: 0.25.
        #[serde(default)]
        pub stock_to_leave: Option<f64>,
        /// Distance between samples along a pass before refinement, mm. Default: a quarter of
        /// the tool's diameter.
        #[serde(default)]
        pub step: Option<f64>,
        /// Largest distance from the surface, mm. Default: the job's tolerance.
        #[serde(default)]
        pub tolerance: Option<f64>,
        /// Every other pass runs back. Default: true.
        #[serde(default)]
        pub zigzag: Option<bool>,
    }
}

operation! {
    /// 3D z-level finishing: round the part at each level, for steep walls.
    WaterlineOp {
        /// The Z levels: from `top` down to `bottom`, both included.
        pub heights: HeightsSpec,
        /// Most distance between levels, mm. Default: the tool's `stepdown`, else a quarter of
        /// its diameter.
        #[serde(default)]
        pub stepdown: Option<f64>,
        /// Where the cutter centre may go. Default: the part's extent and a little round it.
        #[serde(default)]
        pub boundary: Option<BoundarySpec>,
        /// Material left on the walls, mm. Default: 0.
        #[serde(default)]
        pub stock_to_leave: Option<f64>,
        /// Spacing of the grid the levels are traced on, mm. Default: a quarter of the tool's
        /// diameter.
        #[serde(default)]
        pub step: Option<f64>,
        /// How close to each level the loops are found, mm. Default: the job's tolerance.
        #[serde(default)]
        pub tolerance: Option<f64>,
        /// Climb milling. Default: true.
        #[serde(default)]
        pub climb: Option<bool>,
    }
}

operation! {
    /// 4th-axis finishing: the part turns on the A axis while the tool, radial, follows it.
    RotaryFinishOp {
        /// Lines along X or one helix.
        pub pattern: PatternSpec,
        /// Radius from the rotary axis rapids go round at, mm.
        pub clearance: f64,
        /// Radius the tool feeds in from, mm; must clear the part.
        pub retract: f64,
        /// Smallest radius the tip may reach, mm. Default: 0.
        #[serde(default)]
        pub floor: Option<f64>,
        /// From where to where along X, [min, max] mm. Default: the part's extent.
        #[serde(default)]
        pub range: Option<[f64; 2]>,
        /// Distance between samples along a pass before refinement, mm. Default: a quarter of
        /// the tool's diameter.
        #[serde(default)]
        pub step: Option<f64>,
        /// Largest distance from the surface, mm. Default: the job's tolerance.
        #[serde(default)]
        pub tolerance: Option<f64>,
        /// Material left on the surface, mm. Default: 0.
        #[serde(default)]
        pub stock_to_leave: Option<f64>,
        /// Largest turn of the tool axis in one cutting move, degrees. Default: 2.
        #[serde(default)]
        pub max_angle_step_deg: Option<f64>,
    }
}

operation! {
    /// 4th-axis roughing of round stock down to the part, in levels of radius.
    RotaryRoughingOp {
        /// Lines along X or one helix.
        pub pattern: PatternSpec,
        /// The stock's radius, mm.
        pub stock_radius: f64,
        /// Radius from the rotary axis rapids go round at, mm.
        pub clearance: f64,
        /// Radius the tool feeds in from, mm; at or outside the stock.
        pub retract: f64,
        /// The last level's radius, mm. Default: 0.
        #[serde(default)]
        pub floor: Option<f64>,
        /// Most material per level, in radius, mm. Default: the tool's `stepdown`, else half its
        /// diameter.
        #[serde(default)]
        pub stepdown: Option<f64>,
        /// From where to where along X, [min, max] mm. Default: the part's extent.
        #[serde(default)]
        pub range: Option<[f64; 2]>,
        /// Distance between samples along a pass before refinement, mm. Default: a quarter of
        /// the tool's diameter.
        #[serde(default)]
        pub step: Option<f64>,
        /// Largest distance from the surface, mm. Default: the job's tolerance.
        #[serde(default)]
        pub tolerance: Option<f64>,
        /// Material left on the part, mm. Default: 0.25.
        #[serde(default)]
        pub stock_to_leave: Option<f64>,
        /// Largest turn of the tool axis in one cutting move, degrees. Default: 2.
        #[serde(default)]
        pub max_angle_step_deg: Option<f64>,
    }
}

operation! {
    /// A flat path wrapped round the rotary axis (engraving, slots round a cylinder).
    WrapOp {
        /// The radius of the cylinder the path is drawn on, mm.
        pub radius: f64,
        /// The flat path [[x, y, z], ...]: x along the axis, y the arc length round it at
        /// `radius` (mm), z the depth from the cylinder (0 on it, negative into it). Give this
        /// or `sketch` and `curve`.
        #[serde(default)]
        pub points: Option<Vec<[f64; 3]>>,
        /// A sketch whose curve `curve` is the path, its sketch x along the axis and y the arc
        /// length round it.
        #[serde(default)]
        pub sketch: Option<SketchRef>,
        /// With `sketch`: the curve's id.
        #[serde(default)]
        pub curve: Option<String>,
        /// With `sketch`: the depth of the whole path from the cylinder, mm (negative into it).
        /// Default: 0.
        #[serde(default)]
        pub depth: Option<f64>,
        /// How far above the cylinder the tool travels between cuts, mm. Default: 5.
        #[serde(default)]
        pub clearance: Option<f64>,
        /// How far above the cylinder feed begins, mm. Default: 1.
        #[serde(default)]
        pub retract: Option<f64>,
        /// Largest turn of the tool axis in one move, degrees. Default: 2.
        #[serde(default)]
        pub max_angle_step_deg: Option<f64>,
    }
}

/// A 3-axis operation, as an indexed operation runs it.
#[derive(Clone, Debug, Deserialize, JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Op3Spec {
    Contour(ContourOp),
    Pocket(PocketOp),
    Parallel(ParallelOp),
    Roughing(RoughingOp),
    Waterline(WaterlineOp),
}

/// A 3-axis operation run with the A axis held at `a_deg` (3+1): its heights, boundary and
/// polygon are in the frame that angle brings under the tool.
#[derive(Clone, Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct IndexedOp {
    /// The A angle, degrees.
    pub a_deg: f64,
    /// The 3-axis operation (`contour`, `pocket`, `parallel`, `roughing` or `waterline`).
    pub operation: Box<Op3Spec>,
}

/// One operation of a job, by `type`.
#[derive(Clone, Debug, Deserialize, JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum OperationSpec {
    Contour(ContourOp),
    Pocket(PocketOp),
    Parallel(ParallelOp),
    Roughing(RoughingOp),
    Waterline(WaterlineOp),
    Indexed(IndexedOp),
    RotaryFinish(RotaryFinishOp),
    RotaryRoughing(RotaryRoughingOp),
    Wrap(WrapOp),
}

/// What `cam.job` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamJobReply {
    /// The program's name.
    pub name: String,
    /// The machine's name.
    pub machine: String,
    /// The post's name.
    pub post: String,
    /// Every operation, in program order.
    pub operations: Vec<OperationReport>,
    /// Lines of G-code in the program.
    pub program_lines: usize,
    /// The first lines of the program.
    pub program_head: Vec<String>,
    /// The program read back and compared with the toolpaths.
    pub readback: ReadbackReport,
    /// Time at feed of the whole program, dwells included, minutes: as the machine runs it with
    /// the job's `machine_rates`, otherwise as written.
    pub cut_minutes: f64,
    /// Time at rapid with the machine's rapid rates, minutes; `null` without `machine_rates`.
    pub rapid_minutes: Option<f64>,
    /// The program timed against the job's `machine_rates`; `null` without them.
    pub machine_time: Option<MachineTimeReport>,
    /// The gouge check against the part and fixtures.
    pub gouge: GougeReport,
    /// Moves along which a tool's shank or holder comes closer to the part or fixtures than
    /// zero (tools with holder data only); `null` when no tool has holder data.
    pub holder_collisions: Option<usize>,
    /// Whether every check passed.
    pub passed: bool,
    /// The checks that failed.
    pub failures: Vec<String>,
    /// Whether this was a dry run (nothing written).
    pub dry_run: bool,
    /// The files written.
    pub outputs: Outputs,
    /// How the bodies were tessellated, and anything else worth knowing.
    pub notes: Vec<String>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// One operation of a `cam.job` reply.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct OperationReport {
    pub name: String,
    /// Its `type` (an indexed operation: `indexed <type>`).
    pub kind: String,
    /// The tool's number.
    pub tool: u32,
    /// Moves in its toolpath.
    pub moves: usize,
    /// Distance at feed, mm.
    pub cut_length: f64,
    /// Time at feed, minutes.
    pub cut_minutes: f64,
    /// Largest angle between the tool axis and work +Z, degrees.
    pub max_tilt_deg: f64,
    /// The A angle it is indexed at, degrees (indexed operations only).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub a_deg: Option<f64>,
    /// The deepest gouge found, mm beyond the tolerance (0: none).
    pub gouge: f64,
    /// Moves with a holder collision.
    pub holder_collisions: usize,
    pub warnings: Vec<String>,
}

/// The program as written against its toolpaths.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct ReadbackReport {
    /// Largest distance between the tip as written and the toolpaths, mm.
    pub max_tip_deviation: f64,
    /// Largest angle between the tool axis as written and the toolpaths, degrees.
    pub max_axis_deviation_deg: f64,
    /// The deviation the post's precision accounts for, mm.
    pub allowed_tip: f64,
    /// The angle the post's precision accounts for, degrees.
    pub allowed_axis_deg: f64,
    /// Time at feed as the controller times the written program, minutes.
    pub cut_minutes: f64,
    pub pass: bool,
}

/// The gouge check.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct GougeReport {
    /// The deepest gouge of any operation, mm (0: none).
    pub worst: f64,
    /// Depths up to this count as touching, mm.
    pub tolerance: f64,
    /// Moves with a gouge.
    pub hits: usize,
    /// Operations not checked (`gouge_check: false`).
    pub skipped: Vec<String>,
    pub pass: bool,
}

/// The files `cam.job` wrote.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct Outputs {
    /// The G-code file.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gcode: Option<String>,
    /// The HTML preview.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preview: Option<String>,
}

/// List the built-in multi-axis posts, or return one as JSON to copy into a post file.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamPostsParams {
    /// A post's name: its JSON is returned. Default: the list of posts.
    #[serde(default)]
    pub name: Option<String>,
}

/// What `cam.posts` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamPostsReply {
    /// Every built-in post (without `name`).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub posts: Vec<PostEntry>,
    /// The post asked for, as the CAM crate's `Post5` JSON (with `name`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub post: Option<Value>,
    /// The name it was asked by (with `name`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// A built-in post.
#[derive(Debug, Serialize, JsonSchema)]
pub struct PostEntry {
    pub name: String,
    pub description: String,
    /// Other names it is found by.
    pub aliases: Vec<String>,
}

/// Read a G-code file and report what it does: lengths, time, the rotary range, inverse-time
/// blocks, and the first error the reader finds. Reads the subset of G-code SolidCraft's posts
/// write (absolute coordinates); other programs are read on a best-effort basis.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamCheckParams {
    /// The G-code file.
    pub path: String,
    /// The rapid rate in mm/min, to estimate time at rapid. Default: rapids are not timed.
    #[serde(default)]
    pub rapid_rate: Option<f64>,
}

/// What `cam.check` returns.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamCheckReply {
    pub path: String,
    /// Lines in the file.
    pub lines: usize,
    /// Motion blocks read (rapids, cuts, arcs, dwells).
    pub motion_blocks: usize,
    /// Distance of the linear axes at rapid, mm.
    pub rapid_length: f64,
    /// Distance of the linear axes at feed, mm.
    pub feed_length: f64,
    /// Time at feed, dwells included, minutes, as the controller times it.
    pub cut_minutes: f64,
    /// Time at rapid at `rapid_rate`, minutes (with `rapid_rate`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rapid_minutes: Option<f64>,
    /// Arcs (G2, G3).
    pub arcs: usize,
    /// Cuts in inverse time (G93).
    pub g93_blocks: usize,
    /// The rotary axes the program moves, each with its lowest and highest angle, degrees.
    pub rotary_ranges: Vec<RotaryRange>,
    /// Lowest and highest X, Y, Z reached, mm (absent with no motion).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bounds: Option<[[f64; 3]; 2]>,
    /// What the reader could not read (it stops at the first).
    pub errors: Vec<ParseErrorReport>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// A rotary axis's range in a program.
#[derive(Debug, Serialize, JsonSchema)]
pub struct RotaryRange {
    pub letter: String,
    pub min_deg: f64,
    pub max_deg: f64,
}

/// A line the reader could not read.
#[derive(Debug, Serialize, JsonSchema)]
pub struct ParseErrorReport {
    /// The line, from 1 (0: not tied to a line).
    pub line: usize,
    pub message: String,
}

#[cfg(test)]
#[path = "cam_tests.rs"]
mod tests;
