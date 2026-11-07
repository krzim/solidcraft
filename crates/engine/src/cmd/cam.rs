//! CAM (`cam.job`, `cam.posts`, `cam.check`): toolpaths for a design's bodies and G-code for a
//! machine, through the `solidcraft-cam` crate. The job (machine, post, tools, operations) is a
//! JSON file next to the design, never part of it, so nothing new is saved in a design
//! (docs/cam-jobs.md). The simulation view's commands (`cam.view.*`) are in `cam_view.rs`.
//!
//! `cam.job` tessellates the part's bodies from their exact surfaces at a quarter of the job's
//! tolerance into a surface in work coordinates (the design less the job's origin), runs every
//! operation on it within what the tessellation leaves of the tolerance, writes the program
//! through the job's post for its machine, reads that back and compares it with the toolpaths,
//! and checks every cutting move for gouges against the part and fixtures and, for tools with
//! holder data, for shank and holder collisions. A program that fails a check is not written
//! unless the call allows it. Every path is checked against the
//! session's folders (`vfs`, which MCP's `--allow-path` confines) before anything is computed or
//! written.

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use serde::de::DeserializeOwned;
use serde_json::{Value, json};
use solidcore_record::{Loop2, Region2, Seg2, Vec2, Vec3};
use solidcraft_cam::collision::{Hit, holder_collisions_within};
use solidcraft_cam::drill::tool_gouges;
use solidcraft_cam::gcode5::{JointWords, MachineOperation, MachineProgram, Post5, Rewind, Tcp, emit_program_report, parse5};
use solidcraft_cam::indexed::{IndexedOperation, frame_for_index};
use solidcraft_cam::kinematics::{AxisLetter, Carrier, Machine, RotaryAxis};
use solidcraft_cam::readback5::readback_program;
use solidcraft_cam::rotary::{RotaryParams, RotaryPattern, RotaryRoughingParams, rotary_finish, rotary_roughing, wrap_with_step};
use solidcraft_cam::surface_ops::{Boundary, ParallelParams, RoughingParams, WaterlineParams, parallel, roughing, waterline};
use solidcraft_cam::{
    CamError, Compaction, ContourParams, Cut, DEFAULT_DESCENT_LIMIT, DEFAULT_MAX_ANGLE_STEP, Feeds, Frame, Heights, HolderSection, LineNumbers, Move,
    Operation5, PocketParams, SafeRetract, Side, Strategy, Surface, Tool, ToolKind, Toolpath, Toolpath5, band_gouges, compact, contour_with,
    lift_entries, limit_descents, pocket_with,
};
use solidcraft_geom::Mesh;
use solidcraft_geom::prelude::*;

use super::CommandSpec;
use crate::params::bad;
use crate::schema::cam::{
    BoundarySpec, CamCheckParams, CamCheckReply, CamJob, CamJobParams, CamJobReply, CamPostsParams, CamPostsReply, Common, GougeAt, GougeReport,
    HeightsSpec, MachineSpec, Op3Spec, OperationReport, OperationSpec, Outputs, ParseErrorReport, PatternSpec, PostEntry, ReadbackReport,
    RotaryRange, SideSpec,
};
use crate::schema::{TypedContract, output_schema_for};
use crate::{EngineError, Result, Session};

const JOB_CONTRACT: TypedContract<CamJobParams> = TypedContract::new();
const POSTS_CONTRACT: TypedContract<CamPostsParams> = TypedContract::new();
const CHECK_CONTRACT: TypedContract<CamCheckParams> = TypedContract::new();

pub(crate) use view::COMMANDS as VIEW_COMMANDS;

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.job", "CAM Job", crate::checked!("cam.job", job))
        .noundo()
        .contract(&JOB_CONTRACT)
        .output(output_schema_for::<crate::schema::cam_agent::CamJobOutput>)
        .params(
            "job: {…} | job_path: .json (the CAM job, docs/cam-jobs.md: profile_path?: a machine profile (machine, post, tool libraries, \
             default heights and feeds; the job's keys win), name?, machine: \"3axis\" | \"4axis_a\" | Machine JSON, a_reversed?, \
             a_limits_deg?: [min, max], post: built-in name (cam.posts) | post_path: Post5 JSON file, work_offset?: 1-6, origin?: [x,y,z] \
             (the design point that is work zero; on a 4th axis the A axis is work X through it), bodies?: [names] (default every visible \
             body), fixtures?: [names] (checked against, never cut), tolerance?: mm (default 0.01; bodies are tessellated at a quarter of it), \
             stock?: {box: {min, max}} | {cylinder: {radius, x_min, x_max}} | {around_part: {margin: [x,y,z]}} (work coordinates; no \
             rapid may pass through it), tools: [Tool JSON] | tool_library_path | tool_library_paths (+ tool_library to pick one), \
             machine_rates?: {rapid_xyz_mm_min, rapid_z_mm_min?, \
             rapid_a_deg_min?, max_feed_mm_min?, max_feed_z_mm_min?, max_a_deg_min?, spindle_rpm?: [min, max], spindle_start_seconds?, \
             travel_mm?: [x, y, z], home_z_mm?} | machine_rates_path (the machine's top rates: times the program as it runs and warns about blocks \
             the controller slows), operations: [{name, type: contour | pocket | parallel | \
             roughing | waterline | scallop | steep_shallow | pencil | rest_finish | drill | chamfer {heights: {clearance, retract, top}, \
             width, tip_offset?, side?} | indexed {a_deg, operation, name?} | indexed_pattern {a_deg: [..] | count, start_deg?, operation} | \
             rotary_finish | rotary_roughing (pattern: lines | helix | rings) | wrap | wrap_pocket | wrap_contour | bore | thread_mill | face \
             {bottom, top?, retract?, clearance?, stepdown?, stepover?, angle_deg?, zigzag?, overhang?, rect?}, tool: number | {kind?, diameter?, \
             max_diameter?, min_flute_length?, …} (the best match is picked), heights?: {clearance?, retract?, top?, bottom?} (contour, \
             pocket, parallel, roughing, waterline: left out, worked out from the part and stock), spindle_rpm?, feeds?: {cut, plunge}, coolant?, \
             gouge_check? (false: not against the part, still against fixtures), …}]; unknown keys are refused); output: G-code path \
             (required unless dry_run); preview?: HTML path, another file than output (the part \
             and toolpaths, self-contained); setup_sheet?: .html | .md path (the operator's sheet: work zero, A convention, stock, \
             tools with Z min, operations, checks, warnings); dry_run?: plan and check, write nothing; allow_unsafe?: write even when a \
             check fails; compact?: a short reply (passed, failures with fixes, one line per operation, outputs, tools, total_minutes); \
             simulate?: also run the written program through cam.simulate's stock model (a gouge or a rapid in material \
             fails); simulate_resolution?: its cell mm (default 5 × tolerance, coarser to fit a large stock or a long program) → \
             operations: [{name, kind, tool, moves, cut_length, cut_minutes (rewinds' feeds included), rewinds, max_tilt_deg, \
             a_deg?, gouge, gouge_at?, holder_collisions, \
             warnings}], program_lines, program_head, readback: {max_tip_deviation, max_axis_deviation_deg, allowed_tip, allowed_axis_deg, \
             cut_minutes, pass}, cut_minutes, rapid_minutes?, machine_time?: {cut_minutes, written_cut_minutes, rapid_minutes, home_returns, \
             spindle_minutes, total_minutes, slowed_blocks, rotary_over, feed_over, peak_rotary_deg_min?, peak_feed_mm_min?, \
             spindle_out_of_range, span_mm, warnings}, gouge: {worst, tolerance, tessellation, allowed, hits, at?: {operation, move_index, line?, \
             xyz}, skipped, pass}, holder_collisions, rapids: \
             {checked, hits, worst, pass, from_unknown, first: [{line, operation?, kind, depth, at}]}, travel: {checked, violation?, pass} \
             | null, stock, passed, failures, dry_run, outputs: {gcode?, preview?, setup_sheet?}, notes, simulation? (as cam.simulate), \
             summary: {work_zero, part_extent, stock?, work_offset, rotary, tools: [{number, kind, diameter, flute_length, stickout?, \
             holder?, z_min?, operations}], operations: [{name, kind, tool, spindle_rpm, feed, plunge_feed, rotary?, cut_length, \
             cut_minutes}], totals, checks, warnings}, failure_details: [{error, op: {index, name}, check, at: [x,y,z], line?, fix}], tool_choices, \
             derived_heights. Without dry_run or allow_unsafe a failed check refuses and nothing is written; every refusal names the \
             operation, check, place and a fix, and ends `refusals: [...]` (JSON)",
        ),
    CommandSpec::new("cam.posts", "CAM Posts", crate::checked!("cam.posts", posts))
        .noundo()
        .doc_only()
        .contract(&POSTS_CONTRACT)
        .output(output_schema_for::<CamPostsReply>)
        .params(
            "name?: a built-in post → {name, post: its Post5 JSON, to copy into a post file for a job's post_path}; without name → \
             posts: [{name, description, aliases}]",
        ),
    CommandSpec::new("cam.check", "Check G-code", crate::checked!("cam.check", check))
        .noundo()
        .doc_only()
        .contract(&CHECK_CONTRACT)
        .output(output_schema_for::<CamCheckReply>)
        .params(
            "path: G-code file; rapid_rate?: mm/min (to time the rapids) → lines, motion_blocks, rapid_length, feed_length, cut_minutes, \
             rapid_minutes?, arcs, g93_blocks, rotary_ranges: [{letter, min_deg, max_deg}], bounds?: [[x,y,z] low, [x,y,z] high], \
             errors: [{line, message}] (the first line the reader cannot read; it reads the G-code SolidCraft's posts write, others on a \
             best-effort basis)",
        ),
    CommandSpec::new("cam.simulate", "Simulate G-code", crate::checked!("cam.simulate", sim::simulate))
        .noundo()
        .contract(&sim::CONTRACT)
        .output(output_schema_for::<crate::schema::cam_sim::CamSimulateReply>)
        .params(
            "path: G-code file; job: {…} | job_path: .json (the job it was written for: machine, tools, bodies, origin, tolerance); \
             resolution?: cell mm (default 5 × tolerance, coarser to fit a large stock or a long program); stock_stl?: STL path for the stock left → stock: {kind: \
             box | cylinder, min?, max?, x_range?, radius?, derived}, resolution, cells, gouge: {worst, cells}, rapids_in_material: \
             [{line, volume, depth}], leftover: {max, mean, cells}, removed_volume, minutes, slot_moves, passed, failures, warnings, \
             notes, stock_stl?. The stock is swept by every block (3-axis: a heightfield; 4th axis: radii over x and A) and compared \
             with the part; a failure is reported, not refused",
        ),
];

/// Largest job, post or tool library file read (bytes).
const MAX_JSON_BYTES: usize = 16 << 20;
/// Largest G-code file `cam.check` reads (bytes).
const MAX_GCODE_BYTES: usize = 256 << 20;
/// Most operations in a job.
const MAX_OPERATIONS: usize = 200;
/// Most operations of a job planned side by side: each may hold a lot of memory while it is
/// planned.
const PLAN_THREADS: usize = 8;
/// Most tools in a job.
const MAX_TOOLS: usize = 1000;
/// Most bodies named in a job.
const MAX_BODY_NAMES: usize = 1000;
/// Most points of a polygon or a wrapped path.
const MAX_POINTS: usize = 100_000;
/// Largest coordinate accepted (mm).
const MAX_COORD: f64 = 1.0e6;
/// Largest angle accepted (degrees).
const MAX_DEGREES: f64 = 1.0e5;
/// The job's tolerance range (mm).
const TOLERANCE_RANGE: (f64, f64) = (1e-4, 1.0);
/// The share of the job's tolerance the bodies are tessellated within. The facets of a convex
/// surface are inside it and those of a hollow one outside, by up to that, so a toolpath right
/// on the true surface of a hole seems to cut that far into the facets: a quarter leaves the
/// gouge check room to tell that from a gouge.
const CHORD_SHARE: f64 = 0.25;
/// The share of a 3D operation's part of the tolerance its moves may move by when they are
/// merged into fewer (lines and arcs); the rest is the operation's own.
const COMPACT_SHARE: f64 = 0.2;
/// The share of the gouge check's allowance arcs are followed within for the checks.
const CHECK_ARC_SHARE: f64 = 0.05;
/// Most program lines looked through for the line one move ends on.
const LINE_WINDOW: usize = 20_000;
/// Most triangles and poses each scene of the preview carries.
const PREVIEW_TRIANGLES: usize = 60_000;
const PREVIEW_POSES: usize = 40_000;
/// Lines of the program the reply and the preview show.
const HEAD_LINES: usize = 60;
const PREVIEW_GCODE_LINES: usize = 200;

/// The viewer page; `/*DATA*/null` is replaced with the scenes.
const PREVIEW_TEMPLATE: &str = include_str!("cam_preview.html");

/// The built-in posts: the name a job gives, a description, other names, and the post.
const POSTS: &[(&str, &str, &[&str], fn() -> Post5)] = &[
    (
        "mach3_4axis",
        "Mach3-style controllers with a 4th axis (A): G90 G91.1 G94 G17 header, M6 T with G43 H, M3 S, incremental arc centres, \
         inverse time (G93) for cuts that turn A, a rise to clearance before A turns; .tap files",
        &["mach3", "mach3-4axis"],
        Post5::mach3_4axis,
    ),
    (
        "linuxcnc_4axis",
        "LinuxCNC with a 4th axis: RS-274/NGC with incremental arc centres, the work offset in the header, inverse time for cuts that \
         turn A; .ngc files",
        &["linuxcnc", "linuxcnc-4axis"],
        Post5::linuxcnc_4axis,
    ),
    (
        "grblhal_4axis",
        "grblHAL with a 4th axis: ; comments, no tool length offsets, a manual tool change (M0 pause), inverse time for cuts that turn A; \
         .nc files",
        &["grblhal", "grblhal-4axis"],
        Post5::grblhal_4axis,
    ),
    (
        "rs274_5axis",
        "RS-274/NGC controllers without tool centre point control: machine axis positions, inverse time for cuts that turn a rotary, \
         G53 G0 Z0 before a rotary turns; .ngc files",
        &["rs274", "rs274-5axis"],
        Post5::rs274_5axis,
    ),
    (
        "rs274_tcp",
        "RS-274/NGC controllers with tool centre point control (G43.4): the tip in work coordinates and the rotary angles; .ngc files",
        &["rs274-tcp", "g43.4"],
        Post5::tcp_g43_4,
    ),
];

/// The built-in post called `name` (a job's name or an alias, in any case) and its job name.
fn builtin_post(name: &str) -> Option<(&'static str, Post5)> {
    let n = name.trim().to_ascii_lowercase();
    POSTS.iter().find(|(id, _, aliases, _)| *id == n || aliases.contains(&n.as_str())).map(|(id, _, _, make)| (*id, make()))
}

fn post_names() -> String {
    POSTS.iter().map(|(id, ..)| *id).collect::<Vec<_>>().join(", ")
}

/// `p` read into `T`, a failure naming the key path where it went wrong.
fn parse<T: DeserializeOwned>(cmd: &str, p: &Value, what: &str) -> Result<T> {
    let empty = json!({});
    let v = if p.is_null() { &empty } else { p };
    serde_path_to_error::deserialize(v).map_err(|e| {
        let path = e.path().to_string();
        let at = if path == "." {
            what.to_string()
        } else if what.is_empty() {
            format!("`{path}`")
        } else {
            format!("{what} `{path}`")
        };
        if at.is_empty() { bad(cmd, e.into_inner().to_string()) } else { bad(cmd, format!("{at}: {}", e.into_inner())) }
    })
}

fn cam_err(cmd: &str, what: &str, e: CamError) -> EngineError {
    if what.is_empty() { bad(cmd, e.to_string()) } else { bad(cmd, format!("{what}: {e}")) }
}

/// Refuses keys of `given` that `sample` (a value with every field set) does not have, at any
/// depth: the CAM crate's types read unknown keys as nothing, so a misspelt key would otherwise
/// be a setting silently left at its default. An array's items are held against the sample's
/// first item.
fn known_keys(given: &Value, sample: &Value, at: &str) -> std::result::Result<(), String> {
    match (given, sample) {
        (Value::Object(g), Value::Object(s)) => {
            for (k, v) in g {
                let Some(sv) = s.get(k) else {
                    let known: Vec<&str> = s.keys().map(String::as_str).collect();
                    return Err(format!("{at}: unknown key `{k}` (known: {})", known.join(", ")));
                };
                known_keys(v, sv, &format!("{at}.{k}"))?;
            }
            Ok(())
        }
        (Value::Array(g), Value::Array(s)) => match s.first() {
            Some(first) => g.iter().enumerate().try_for_each(|(i, v)| known_keys(v, first, &format!("{at}[{i}]"))),
            None => Ok(()),
        },
        _ => Ok(()),
    }
}

/// A tool with every field set, for [`known_keys`].
fn tool_sample() -> Value {
    let tool = Tool {
        description: "sample".into(),
        spindle_rpm: Some(1.0),
        feed: Some(1.0),
        plunge_feed: Some(1.0),
        stepdown: Some(1.0),
        stepover: Some(1.0),
        corner_radius: Some(1.0),
        tip_diameter: Some(1.0),
        angle_deg: Some(1.0),
        shank_diameter: Some(1.0),
        stickout: Some(1.0),
        holder: vec![HolderSection { diameter: 1.0, length: 1.0 }],
        point_angle_deg: Some(118.0),
        thread_pitch: Some(1.0),
        tooth_height: Some(1.0),
        surface_speed: Some(1.0),
        chip_load: Some(1.0),
        cutting_data: vec![solidcraft_cam::CuttingData {
            strategy: Strategy::Adaptive,
            spindle_rpm: Some(1.0),
            feed: Some(1.0),
            plunge_feed: Some(1.0),
            stepdown: Some(1.0),
            stepover: Some(1.0),
            surface_speed: Some(1.0),
            chip_load: Some(1.0),
        }],
        ..Tool::new(1, ToolKind::BullNose, 1.0, 1.0)
    };
    serde_json::to_value(tool).unwrap_or(Value::Null)
}

/// A machine with every field set, for [`known_keys`].
fn machine_sample() -> Value {
    let axis = RotaryAxis::new(AxisLetter::A, Vec3::X, Vec3::ZERO, Carrier::Table, Some((-1.0, 1.0))).reversed(true);
    let machine = Machine { name: "sample".into(), rotaries: vec![axis], linear_limits: Some((Vec3::ZERO, Vec3::ZERO)), work_origin: Vec3::ZERO };
    serde_json::to_value(machine).unwrap_or(Value::Null)
}

/// A post with every field set, for [`known_keys`].
fn post_sample() -> Value {
    let mut post = Post5::mach3_4axis();
    post.tcp = Some(Tcp { on: vec![String::new()], off: vec![String::new()] });
    post.clearance = Some(1.0);
    post.safe_rotate = vec![String::new()];
    post.base.line_numbers = Some(LineNumbers { start: 1, step: 1 });
    post.base.upper_case_comments = true;
    post.base.comment_chars = Some(String::new());
    post.base.tool_list = true;
    post.base.operation_name_only = true;
    post.base.blank_lines = true;
    post.base.operation_start = vec![String::new()];
    post.base.operation_end = vec![String::new()];
    post.base.safe_retract = Some(SafeRetract { lines: vec![String::new()], at_start: true, at_tool_change: true, at_end: true });
    post.base.tool_length_on_first_z = true;
    post.base.modal_motion = true;
    post.safe_rotate_first = true;
    post.coolant_after_turn = true;
    post.inverse_time_decimals = Some(4);
    post.rewind = Some(Rewind { distance: 1.0, retract_feed: 1.0, plunge_feed: 1.0 });
    serde_json::to_value(post).unwrap_or(Value::Null)
}

/// `value` as a `T`, its keys held against `sample` first.
fn crate_value<T: DeserializeOwned>(cmd: &str, value: Value, sample: &Value, what: &str) -> Result<T> {
    known_keys(&value, sample, what).map_err(|e| bad(cmd, e))?;
    serde_path_to_error::deserialize(&value).map_err(|e| {
        let path = e.path().to_string();
        if path == "." { bad(cmd, format!("{what}: {}", e.into_inner())) } else { bad(cmd, format!("{what}.{path}: {}", e.into_inner())) }
    })
}

fn read_json(cmd: &str, path: &str, what: &str) -> Result<Value> {
    let bytes = solidcraft_io::vfs::read(path, MAX_JSON_BYTES).map_err(|e| bad(cmd, format!("{what} {path}: {e}")))?;
    serde_json::from_slice(&bytes).map_err(|e| bad(cmd, format!("{what} {path}: not valid JSON: {e}")))
}

/// `path` read from `base` (the job file's folder) when it is relative and the job came from a file.
fn beside(base: Option<&Path>, path: &str) -> String {
    match base {
        Some(b) if Path::new(path).is_relative() => b.join(path).to_string_lossy().to_string(),
        _ => path.to_string(),
    }
}

fn finite_point(v: &[f64], what: &str) -> std::result::Result<(), String> {
    if v.iter().all(|x| x.is_finite() && x.abs() <= MAX_COORD) { Ok(()) } else { Err(format!("{what} must be finite numbers within 1000 m")) }
}

fn radians(deg: f64, what: &str) -> std::result::Result<f64, String> {
    if deg.is_finite() && deg.abs() <= MAX_DEGREES { Ok(deg.to_radians()) } else { Err(format!("{what} must be a finite angle in degrees")) }
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

/// The job's machine.
fn machine_of(cmd: &str, job: &CamJob) -> Result<Machine> {
    let a_options = job.a_reversed.is_some() || job.a_limits_deg.is_some();
    let Some(spec) = &job.machine else {
        return Err(bad(cmd, "the job needs a `machine` (`3axis`, `4axis_a` or Machine JSON), a `machine_path` or a `profile_path` that gives one"));
    };
    let machine = match spec {
        MachineSpec::Preset(name) => match name.trim().to_ascii_lowercase().replace('-', "_").as_str() {
            "3axis" | "3_axis" | "three_axis" => {
                if a_options {
                    return Err(bad(cmd, "`a_reversed` and `a_limits_deg` are for the `4axis_a` machine"));
                }
                Machine::three_axis()
            }
            "4axis_a" | "4axis" | "4_axis_a" | "four_axis_a" => {
                let limits = match job.a_limits_deg {
                    None => None,
                    Some([lo, hi]) => {
                        let lo = radians(lo, "`a_limits_deg`").map_err(|e| bad(cmd, e))?;
                        let hi = radians(hi, "`a_limits_deg`").map_err(|e| bad(cmd, e))?;
                        if !(lo < hi) {
                            return Err(bad(cmd, "`a_limits_deg` must be [lowest, highest], the lowest first"));
                        }
                        Some((lo, hi))
                    }
                };
                Machine::four_axis_a_with(Vec3::ZERO, job.a_reversed.unwrap_or(false), limits)
            }
            other => return Err(bad(cmd, format!("unknown machine `{other}`: `3axis`, `4axis_a` or a Machine JSON object"))),
        },
        MachineSpec::Custom(m) => {
            if a_options {
                return Err(bad(cmd, "`a_reversed` and `a_limits_deg` are for the `4axis_a` machine; set them in the machine's JSON"));
            }
            crate_value(cmd, Value::Object(m.clone()), &machine_sample(), "`machine`")?
        }
    };
    machine.validate().map_err(|e| cam_err(cmd, "`machine`", e))?;
    Ok(machine)
}

/// The job's post and its name.
fn post_of(cmd: &str, job: &CamJob, base: Option<&Path>) -> Result<(String, Post5)> {
    let post = match (&job.post, &job.post_path) {
        (Some(_), Some(_)) => return Err(bad(cmd, "give the job `post` or `post_path`, not both")),
        (None, None) => return Err(bad(cmd, format!("the job needs a `post` ({}) or a `post_path`", post_names()))),
        (Some(name), None) => match builtin_post(name) {
            Some((id, post)) => (id.to_string(), post),
            None => return Err(bad(cmd, format!("unknown post `{name}` (built in: {})", post_names()))),
        },
        (None, Some(path)) => {
            let path = beside(base, path);
            let post: Post5 = crate_value(cmd, read_json(cmd, &path, "post_path")?, &post_sample(), "the post")?;
            (post.base.name.clone(), post)
        }
    };
    post.1.validate().map_err(|e| cam_err(cmd, "the post", e))?;
    Ok(post)
}

/// The job's tools, each checked.
fn tools_of(cmd: &str, job: &CamJob, base: Option<&Path>) -> Result<Vec<Tool>> {
    let raw: Vec<Value> = match (&job.tools, &job.tool_library_path) {
        (Some(_), Some(_)) => return Err(bad(cmd, "give the job `tools` or `tool_library_path`, not both")),
        (None, None) => return Err(bad(cmd, "the job needs `tools` or a `tool_library_path`")),
        (Some(t), None) => t.iter().cloned().map(Value::Object).collect(),
        (None, Some(path)) => match read_json(cmd, &beside(base, path), "tool_library_path")? {
            Value::Array(a) => a,
            _ => return Err(bad(cmd, "a tool library file holds a JSON array of tools")),
        },
    };
    if raw.is_empty() || raw.len() > MAX_TOOLS {
        return Err(bad(cmd, format!("a job has 1 to {MAX_TOOLS} tools")));
    }
    let sample = tool_sample();
    let mut tools: Vec<Tool> = Vec::with_capacity(raw.len());
    for (i, v) in raw.into_iter().enumerate() {
        let what = format!("`tools[{i}]`");
        let tool: Tool = crate_value(cmd, v, &sample, &what)?;
        tool.validate().map_err(|e| cam_err(cmd, &what, e))?;
        if tools.iter().any(|t| t.number == tool.number) {
            return Err(bad(cmd, format!("two tools are number {}", tool.number)));
        }
        tools.push(tool);
    }
    Ok(tools)
}

/// The part, the fixtures and the collision surface (part and fixtures) in work coordinates.
#[derive(Clone)]
struct Surfaces {
    part: Surface,
    /// The fixtures alone (`None` without any), which an operation that skips the gouge check
    /// against the part is still checked against.
    fixtures: Option<Surface>,
    all: Surface,
    notes: Vec<String>,
}

/// How a job places the design: turned about `origin` by its `orientation`, then moved so
/// `origin` is work zero. The turned design is still in design units and place, so everything
/// that takes `origin` off a design point works on [`Placement::turn`] of it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Placement {
    pub origin: Vec3,
    /// The work axes' directions in the design, when the part is turned.
    pub axes: Option<[Vec3; 3]>,
}

impl Placement {
    /// The design point `p` turned about `origin`.
    pub fn turn(&self, p: Vec3) -> Vec3 {
        self.origin + self.turn_dir(p - self.origin)
    }

    /// The design direction `d` turned.
    pub fn turn_dir(&self, d: Vec3) -> Vec3 {
        match self.axes {
            None => d,
            Some([x, y, z]) => Vec3::new(d.dot(x), d.dot(y), d.dot(z)),
        }
    }

    /// The turned point `p` back in the design.
    pub fn unturn(&self, p: Vec3) -> Vec3 {
        match self.axes {
            None => p,
            Some([x, y, z]) => {
                let d = p - self.origin;
                self.origin + x * d.x + y * d.y + z * d.z
            }
        }
    }

    /// The design point `p` in work coordinates.
    pub fn in_work(&self, p: Vec3) -> Vec3 {
        self.turn(p) - self.origin
    }

    /// The turn as the kernel's matrix (row vectors: the point times the 3 × 3 part, then the
    /// last row added), when there is one.
    pub fn matrix(&self) -> Option<[[f64; 4]; 4]> {
        let [x, y, z] = self.axes?;
        // p' = origin + R (p - origin), R's rows x, y, z: p'_j = Σ_i p_i R[j][i] + t_j.
        let t = self.origin - self.turn_dir(self.origin);
        Some([[x.x, y.x, z.x, 0.0], [x.y, y.y, z.y, 0.0], [x.z, y.z, z.z, 0.0], [t.x, t.y, t.z, 1.0]])
    }
}

/// The job's placement: its `origin` and its `orientation`, checked.
pub(super) fn placement_of(cmd: &str, s: &Session, job: &CamJob) -> Result<Placement> {
    use crate::schema::cam::OrientationSpec;
    let origin = job.origin.unwrap_or([0.0; 3]);
    finite_point(&origin, "`origin`").map_err(|e| bad(cmd, e))?;
    let origin = v3(origin);
    let unit = |v: Vec3, what: &str| -> Result<Vec3> {
        let len = v.dot(v).sqrt();
        if v.x.is_finite() && v.y.is_finite() && v.z.is_finite() && len > 1e-12 {
            Ok(Vec3::new(v.x / len, v.y / len, v.z / len))
        } else {
            Err(bad(cmd, format!("{what} must be a finite, non-zero direction")))
        }
    };
    let axes = match &job.orientation {
        None => None,
        Some(OrientationSpec::Named(n)) => match n.as_str() {
            "flip_x" => Some([Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, -1.0)]),
            "flip_y" => Some([Vec3::new(-1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, -1.0)]),
            "none" => None,
            other => {
                return Err(bad(
                    cmd,
                    format!("`orientation` is \"flip_x\", \"flip_y\", {{\"axis\", \"angle_deg\"}} or {{\"sketch\"}}, not \"{other}\""),
                ));
            }
        },
        Some(OrientationSpec::Turn { axis, angle_deg }) => {
            finite_point(axis, "`orientation.axis`").map_err(|e| bad(cmd, e))?;
            let a = unit(v3(*axis), "`orientation.axis`")?;
            if !(angle_deg.is_finite() && angle_deg.abs() <= 3600.0) {
                return Err(bad(cmd, "`orientation.angle_deg` must be a finite angle"));
            }
            let t = -angle_deg.to_radians();
            // The work axes in the design: the design's own axes turned back.
            Some([Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)].map(|e| solidcraft_cam::rotate(e, a, t)))
        }
        Some(OrientationSpec::Sketch { sketch }) => {
            let id = match s.doc.find_feature(sketch) {
                Some(f) if matches!(f.kind, solidcraft_doc::FeatureKind::Sketch { .. }) => f.id,
                _ => return Err(bad(cmd, format!("`orientation`: no sketch `{sketch}`"))),
            };
            let st = s.model.state();
            let Some(ss) = st.sketch(id) else { return Err(bad(cmd, format!("`orientation`: sketch `{sketch}` has not been solved"))) };
            let (x, y) = (ss.plane.x, ss.plane.y);
            let z = unit(x.cross(y), "the sketch's normal")?;
            let x = unit(x, "the sketch's X")?;
            Some([x, z.cross(x), z])
        }
    };
    Ok(Placement { origin, axes })
}

/// Every body of `names` tessellated within `chord` into one mesh, turned and moved to work
/// coordinates by `place`.
fn merged_mesh(cmd: &str, st: &solidcraft_doc::ModelState, names: &[String], place: &Placement, chord: f64, notes: &mut Vec<String>) -> Result<Mesh> {
    let mut out = Mesh::default();
    for name in names {
        let Some(b) = st.bodies.iter().find(|b| &b.name == name) else { return Err(bad(cmd, format!("no body `{name}`"))) };
        if b.body.is_mesh() {
            notes.push(format!("body `{name}` is a mesh: its own triangles are used, which may be coarser than the job's tolerance"));
        }
        let mesh = b.body.mesh(&solidcraft_kernel::MeshTolerance::chord(chord))?;
        let Ok(start) = u32::try_from(out.positions.len()) else { return Err(bad(cmd, "the part has too many triangles")) };
        let smooth = mesh.normals.len() == mesh.positions.len();
        out.positions.extend(mesh.positions.iter().map(|p| place.in_work(*p)));
        if smooth {
            out.normals.extend(mesh.normals.iter().map(|n| place.turn_dir(*n)));
        } else {
            // Not directions: the surface takes each triangle's own normal instead.
            out.normals.extend(std::iter::repeat_n(Vec3::ZERO, mesh.positions.len()));
        }
        for t in &mesh.triangles {
            let shifted = t.map(|i| i.checked_add(start));
            let [Some(a), Some(b), Some(c)] = shifted else { return Err(bad(cmd, "the part has too many triangles")) };
            out.triangles.push([a, b, c]);
        }
        if out.triangles.len() > solidcraft_cam::surface_ops::MAX_SAMPLES {
            return Err(bad(cmd, "the part has too many triangles at this tolerance: raise the job's `tolerance`"));
        }
    }
    Ok(out)
}

fn surfaces_of(cmd: &str, s: &Session, job: &CamJob, origin: Vec3, tol: f64) -> Result<Surfaces> {
    let place = Placement { origin, ..placement_of(cmd, s, job)? };
    let st = s.world_state()?;
    let fixtures: Vec<String> = job.fixtures.clone().unwrap_or_default();
    let part: Vec<String> = match &job.bodies {
        Some(b) => b.clone(),
        None => st.bodies.iter().map(|b| b.name.clone()).filter(|n| !fixtures.contains(n) && s.body_visible(n)).collect(),
    };
    if part.len() > MAX_BODY_NAMES || fixtures.len() > MAX_BODY_NAMES {
        return Err(bad(cmd, format!("a job names at most {MAX_BODY_NAMES} bodies")));
    }
    if let Some(both) = part.iter().find(|n| fixtures.contains(n)) {
        return Err(bad(cmd, format!("body `{both}` is in both `bodies` and `fixtures`")));
    }
    if part.is_empty() {
        return Err(bad(cmd, "the job has no bodies to machine: the design has no visible body, or give `bodies`"));
    }
    let chord = tol * CHORD_SHARE;
    let mut notes = vec![format!(
        "{TESSELLATION_NOTE} from their exact surfaces at a chord tolerance of {} mm (a quarter of the job's tolerance), in work coordinates \
         (the design less origin [{}, {}, {}])",
        num(chord),
        num(origin.x),
        num(origin.y),
        num(origin.z)
    )];
    if place.axes.is_some() {
        notes.push("the design is turned by the job's `orientation` about its origin before it is machined".into());
    }
    let part_mesh = merged_mesh(cmd, &st, &part, &place, chord, &mut notes)?;
    let part_surface = Surface::from_mesh(&part_mesh).map_err(|e| cam_err(cmd, "the part", e))?;
    if part_surface.is_empty() {
        return Err(bad(cmd, "the part's bodies have no surface to machine"));
    }
    let (all, fixture_surface) = if fixtures.is_empty() {
        (part_surface.clone(), None)
    } else {
        let mut both = part_mesh;
        let fix = merged_mesh(cmd, &st, &fixtures, &place, chord, &mut notes)?;
        let fixture_surface = Surface::from_mesh(&fix).map_err(|e| cam_err(cmd, "the fixtures", e))?;
        let Ok(start) = u32::try_from(both.positions.len()) else { return Err(bad(cmd, "the part has too many triangles")) };
        both.positions.extend(fix.positions);
        both.normals.extend(fix.normals);
        for t in fix.triangles {
            let [Some(a), Some(b), Some(c)] = t.map(|i| i.checked_add(start)) else { return Err(bad(cmd, "the part has too many triangles")) };
            both.triangles.push([a, b, c]);
        }
        (Surface::from_mesh(&both).map_err(|e| cam_err(cmd, "the fixtures", e))?, (!fixture_surface.is_empty()).then_some(fixture_surface))
    };
    Ok(Surfaces { part: part_surface, fixtures: fixture_surface, all, notes })
}

/// How the note on the bodies' tessellation starts (the compact reply leaves it out).
const TESSELLATION_NOTE: &str = "bodies tessellated";

/// A number as the notes and messages show it.
fn num(v: f64) -> String {
    let s = format!("{v:.4}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.to_string() }
}

/// The closed polygon `pts` as a region, counter-clockwise.
fn polygon_region(cmd: &str, what: &str, pts: &[[f64; 2]]) -> Result<Region2> {
    if pts.len() > MAX_POINTS {
        return Err(bad(cmd, format!("{what} has more than {MAX_POINTS} points")));
    }
    let mut v: Vec<Vec2> = Vec::with_capacity(pts.len());
    for p in pts {
        finite_point(p, what).map_err(|e| bad(cmd, e))?;
        let q = Vec2::new(p[0], p[1]);
        if v.last().is_none_or(|l| l.dist(q) > 1e-9) {
            v.push(q);
        }
    }
    if v.len() > 1 && v.first().zip(v.last()).is_some_and(|(a, b)| a.dist(*b) <= 1e-9) {
        v.pop();
    }
    if v.len() < 3 {
        return Err(bad(cmd, format!("{what} needs at least 3 distinct points")));
    }
    let area: f64 = v.iter().zip(v.iter().cycle().skip(1)).map(|(a, b)| a.x * b.y - b.x * a.y).sum::<f64>() * 0.5;
    if !(area.abs() > 1e-9) {
        return Err(bad(cmd, format!("{what} encloses no area")));
    }
    if area < 0.0 {
        v.reverse();
    }
    Ok(Region2 { outer: Loop2::polygon(&v), holes: Vec::new() })
}

/// The closed profiles of sketch `key` in the coordinates of `frame` (inside work coordinates,
/// the design less `origin`): the sketch's plane must be square to the frame's Z and face it.
fn sketch_regions(cmd: &str, s: &Session, key: &str, profiles: Option<&[usize]>, place: &Placement, frame: &Frame) -> Result<Vec<Region2>> {
    let id = match s.doc.find_feature(key) {
        Some(f) if matches!(f.kind, solidcraft_doc::FeatureKind::Sketch { .. }) => f.id,
        _ => return Err(bad(cmd, format!("no sketch `{key}`"))),
    };
    let st = s.model.state();
    let Some(ss) = st.sketch(id) else { return Err(bad(cmd, format!("sketch `{key}` has not been solved"))) };
    let sel = match profiles {
        None => solidcraft_doc::ProfileSel::All,
        Some(ix) => solidcraft_doc::ProfileSel::Indices { indices: ix.to_vec() },
    };
    let regions = solidcraft_doc::select_profiles(ss, &sel)?;
    let plane = &ss.plane;
    let normal = frame.dir_to_local(place.turn_dir(plane.x.cross(plane.y)));
    if !(normal.z > 1.0 - 1e-6) {
        return Err(bad(
            cmd,
            format!(
                "sketch `{key}` is not on a plane square to the operation's tool axis and facing it (draw it on a plane facing +Z of the operation)"
            ),
        ));
    }
    let lx = frame.dir_to_local(place.turn_dir(plane.x));
    let turn = lx.y.atan2(lx.x);
    let map = |q: Vec2| {
        let l = frame.to_local(place.in_work(plane.origin + plane.x * q.x + plane.y * q.y));
        Vec2::new(l.x, l.y)
    };
    let seg = |s: &Seg2| match *s {
        Seg2::Line { a, b } => Seg2::Line { a: map(a), b: map(b) },
        Seg2::Arc { center, radius, start, sweep } => Seg2::Arc { center: map(center), radius, start: start + turn, sweep },
        Seg2::Cubic { p0, p1, p2, p3 } => Seg2::Cubic { p0: map(p0), p1: map(p1), p2: map(p2), p3: map(p3) },
        Seg2::Conic { a, apex, b, w } => Seg2::Conic { a: map(a), apex: map(apex), b: map(b), w },
        Seg2::Bezier { n, p } => Seg2::Bezier { n, p: p.map(map) },
    };
    let lp = |l: &Loop2| Loop2 { segs: l.segs.iter().map(seg).collect() };
    Ok(regions.iter().map(|r| Region2 { outer: lp(&r.outer), holes: r.holes.iter().map(lp).collect() }).collect())
}

/// What the operations of a job share while they are planned, side by side or not: a
/// [`Ctx`] each, with an agent of its own ([`Shared::ctx`]).
#[derive(Clone, Copy)]
struct Shared<'a> {
    cmd: &'a str,
    s: &'a Session,
    machine: &'a Machine,
    tools: &'a [Tool],
    part: &'a Surface,
    all: &'a Surface,
    origin: Vec3,
    place: Placement,
    tol: f64,
    stock: Option<&'a solidcraft_cam::stock::Stock>,
    start: Option<&'a solidcraft_cam::adaptive::HeightMap>,
    chord: f64,
    post: &'a Post5,
    features: &'a features::Lazy,
}

impl<'a> Shared<'a> {
    fn ctx<'b>(&self, agent: &'b agent::Agent) -> Ctx<'b>
    where
        'a: 'b,
    {
        Ctx {
            adaptive: Default::default(),
            band: Default::default(),
            rest: None,
            cmd: self.cmd,
            s: self.s,
            machine: self.machine,
            tools: self.tools,
            part: self.part,
            all: self.all,
            origin: self.origin,
            place: self.place,
            tol: self.tol,
            stock: self.stock,
            start: self.start,
            chord: self.chord,
            post: self.post,
            agent,
            features: self.features,
        }
    }
}

/// What every operation needs, resolved.
struct Resolved {
    tool: Tool,
    rpm: f64,
    feeds: Feeds,
    /// `feeds.descent`, for the 3D finishes.
    descent: Option<f64>,
}

/// Everything an operation is planned with.
struct Ctx<'a> {
    /// What an adaptive operation adds to its report, handed from its planning to the report.
    adaptive: std::cell::RefCell<Option<(crate::schema::cam_adaptive::AdaptiveReport, Vec<String>)>>,
    /// The band a chamfer mill is told to cut into what the model shows (a chamfer, a V-trace,
    /// a spotted hole), handed from its planning to [`Planned::band`].
    band: std::cell::Cell<Option<f64>>,
    /// For an operation with `rest_from`: what the earlier operations it names left of the
    /// stock, in work coordinates, and how many they were.
    rest: Option<(std::sync::Arc<solidcraft_cam::adaptive::HeightMap>, usize)>,
    cmd: &'a str,
    s: &'a Session,
    machine: &'a Machine,
    tools: &'a [Tool],
    part: &'a Surface,
    /// The part and the fixtures.
    all: &'a Surface,
    origin: Vec3,
    /// The design's placement: `origin` and the job's `orientation`.
    place: Placement,
    tol: f64,
    stock: Option<&'a solidcraft_cam::stock::Stock>,
    /// What an earlier setup left of the stock (`stock: {simulated}`), in work coordinates.
    start: Option<&'a solidcraft_cam::adaptive::HeightMap>,
    /// The chord tolerance the part was tessellated within (mm).
    chord: f64,
    post: &'a Post5,
    /// Profiles, defaults, tool picks and structured refusals (`cam_agent.rs`).
    agent: &'a agent::Agent,
    /// The part's machinable features, found when an operation names one (`cam_features.rs`).
    features: &'a features::Lazy,
}

impl Ctx<'_> {
    fn bad(&self, op: &str, why: impl std::fmt::Display) -> EngineError {
        self.agent.name_op(op);
        bad(self.cmd, format!("operation `{op}`: {why}"))
    }

    fn resolve(&self, c: &Common<'_>) -> Result<Resolved> {
        agent::resolve(self, c, None)
    }

    fn tolerance(&self, op: &str, t: Option<f64>) -> Result<f64> {
        match t {
            None => Ok(self.tol),
            Some(t) if t.is_finite() && (TOLERANCE_RANGE.0..=TOLERANCE_RANGE.1).contains(&t) => Ok(t),
            Some(_) => Err(self.bad(op, "the tolerance must be from 0.0001 to 1 mm")),
        }
    }

    /// What a 3D operation planned on the tessellated part may take of its tolerance `t` (the
    /// job's by default): the tolerance less the tessellation's chord error, at least half of it.
    fn surface_budget(&self, op: &str, t: Option<f64>) -> Result<f64> {
        let t = self.tolerance(op, t)?;
        Ok((t - self.chord).max(0.5 * t))
    }

    fn degrees(&self, op: &str, deg: f64, what: &str) -> Result<f64> {
        radians(deg, what).map_err(|e| self.bad(op, e))
    }

    /// The regions of a 2D operation in `frame`'s coordinates.
    fn regions(
        &self,
        op: &str,
        sketch: Option<&crate::schema::SketchRef>,
        profiles: Option<&[usize]>,
        polygon: Option<&[[f64; 2]]>,
        frame: &Frame,
    ) -> Result<Vec<Region2>> {
        match (sketch, polygon) {
            (Some(_), Some(_)) => Err(self.bad(op, "give `sketch` or `polygon`, not both")),
            (None, None) => Err(self.bad(op, "give a `sketch` or a `polygon` to cut")),
            (None, Some(_)) if profiles.is_some() => Err(self.bad(op, "`profiles` goes with `sketch`")),
            (Some(k), None) => sketch_regions(self.cmd, self.s, &k.0, profiles, &self.place, frame).map_err(|e| match e {
                EngineError::BadParams { msg, .. } => self.bad(op, msg),
                other => self.bad(op, other),
            }),
            (None, Some(p)) => polygon_region(self.cmd, &format!("operation `{op}`: `polygon`"), p).map(|r| vec![r]),
        }
    }

    fn boundary(&self, op: &str, b: Option<&BoundarySpec>) -> Result<Boundary> {
        let Some(b) = b else { return Ok(Boundary::default()) };
        let rect = match b.rect {
            None => None,
            Some([lo, hi]) => {
                finite_point(&[lo[0], lo[1], hi[0], hi[1]], "`boundary.rect`").map_err(|e| self.bad(op, e))?;
                if !(lo[0] < hi[0] && lo[1] < hi[1]) {
                    return Err(self.bad(op, "`boundary.rect` must be [[low x, low y], [high x, high y]]"));
                }
                Some([Vec2::new(lo[0], lo[1]), Vec2::new(hi[0], hi[1])])
            }
        };
        let region = match &b.polygon {
            None => None,
            Some(p) => Some(polygon_region(self.cmd, &format!("operation `{op}`: `boundary.polygon`"), p)?),
        };
        Ok(Boundary { rect, region })
    }
}

fn heights(h: &HeightsSpec) -> Heights {
    Heights { clearance: h.clearance, retract: h.retract, top: h.top, bottom: h.bottom }
}

/// The operation's stepdown, stepover or step, or its default from the tool.
fn or_tool(v: Option<f64>, tool: Option<f64>, default: f64) -> f64 {
    v.or(tool).unwrap_or(default)
}

/// A 3-axis operation planned in `frame` on `surface` (the part in that frame), its runs of
/// moves merged where a 3D operation leaves room for that, and its slowed descents worked out
/// again from the coordinates as the post writes them.
fn plan3(ctx: &Ctx<'_>, op: &Op3Spec, frame: &Frame, surface: &Surface) -> Result<(Toolpath, Resolved, &'static str)> {
    let (path, r, kind, merge) = plan3_raw(ctx, op, frame, surface)?;
    let name = common3(op).name;
    let mut path = match merge {
        Some((tolerance, planned)) => {
            let b = &ctx.post.base;
            // An arc may bow out from the moves it replaces as far as they were allowed to cut
            // across a curve when they were planned.
            let how = Compaction { tolerance: tolerance.min(b.arc_tolerance.max(1e-6)), arcs: b.arcs, helical: b.helical_arcs, bow: planned };
            compact(&path, &how).map_err(|e| ctx.bad(name, e))?
        }
        None => path,
    };
    let b = &ctx.post.base;
    let feed_step = 10f64.powi(-i32::from(b.feed_decimals)) * b.units.mm();
    // The ways down at rapid stop above where the material may reach, not only above the
    // operation's own top.
    if let Some(top) = material_top(ctx, frame) {
        lift_entries(&mut path, top);
    }
    // A 3D finish's moves down a wall go down at its descent feed, its entries (straight down)
    // at the plunge feed: the larger of the two bounds both, and a straight entry keeps its own.
    let takes_descent =
        matches!(op, Op3Spec::Parallel(_) | Op3Spec::Scallop(_) | Op3Spec::SteepShallow(_) | Op3Spec::Pencil(_) | Op3Spec::RestFinish(_));
    let down = match r.descent {
        Some(d) if takes_descent => d.max(r.feeds.plunge),
        _ => r.feeds.plunge,
    };
    limit_descents(&mut path, down, r.feeds.cut, b.decimals, b.units.mm(), feed_step).map_err(|e| ctx.bad(name, e))?;
    Ok((path, r, kind))
}

/// Where the material may reach in `frame` (the highest Z of the operation's coordinates): the
/// stock's top seen from there; on a machine with rotaries without a bar stock, the round bar
/// through the part's farthest point from the A axis too (as the simulation takes the stock);
/// otherwise not known (the operations' heights say).
fn material_top(ctx: &Ctx<'_>, frame: &Frame) -> Option<f64> {
    use solidcraft_cam::stock::Stock;
    let highest = |lo: Vec3, hi: Vec3| -> f64 {
        let mut top = f64::NEG_INFINITY;
        for i in 0..8 {
            let c = Vec3::new(if i & 1 == 0 { lo.x } else { hi.x }, if i & 2 == 0 { lo.y } else { hi.y }, if i & 4 == 0 { lo.z } else { hi.z });
            top = top.max(frame.to_local(c).z);
        }
        top
    };
    // A round bar about work X: its radius where the frame looks square to X.
    let bar = |radius: f64, x0: f64, x1: f64| -> f64 {
        if frame.z.x.abs() <= 1e-9 {
            radius - frame.origin.dot(frame.z)
        } else {
            highest(Vec3::new(x0, -radius, -radius), Vec3::new(x1, radius, radius))
        }
    };
    // The round bar through the part's farthest point from work X.
    let part_bar = || {
        let (mut reach, mut x0, mut x1) = (0.0f64, f64::INFINITY, f64::NEG_INFINITY);
        for t in ctx.part.triangles() {
            for p in t {
                reach = reach.max(p.y.hypot(p.z));
                x0 = x0.min(p.x);
                x1 = x1.max(p.x);
            }
        }
        (x0 <= x1).then(|| bar(reach, x0, x1))
    };
    let rotary = !ctx.machine.rotaries.is_empty();
    let top = match ctx.stock {
        Some(Stock::Box { min, max }) if !rotary => highest(*min, *max),
        // On a 4th axis the simulation models the round bar through the part's farthest point,
        // not a box: above both.
        Some(Stock::Box { min, max }) => part_bar().map_or(highest(*min, *max), |b| b.max(highest(*min, *max))),
        Some(Stock::Cylinder { radius, x_min, x_max }) => bar(*radius, *x_min, *x_max),
        None if rotary => part_bar()?,
        None => return None,
    };
    top.is_finite().then_some(top)
}

/// The radius a `rotary_roughing` starts from: its own `stock_radius`, else the job's stock's
/// (a cylinder's radius, or how far a box's farthest edge is from the A axis).
fn roughing_stock_radius(ctx: &Ctx<'_>, o: &crate::schema::cam::RotaryRoughingOp) -> Option<f64> {
    use solidcraft_cam::stock::Stock;
    o.stock_radius.or_else(|| match ctx.stock? {
        Stock::Cylinder { radius, .. } => Some(*radius),
        Stock::Box { min, max } => Some(min.y.abs().max(max.y.abs()).hypot(min.z.abs().max(max.z.abs()))),
    })
}

/// The share of a 3D operation's budget it plans within, and the share its moves may then move
/// by when merged (with the share it plans within again).
fn split_budget(budget: f64) -> (f64, Option<(f64, f64)>) {
    let planned = budget * (1.0 - COMPACT_SHARE);
    (planned, Some((budget * COMPACT_SHARE, planned)))
}

/// A 3-axis operation as planned, its tool, its kind, and the tolerance its moves may be merged
/// within and the one it was planned within (3D finishing only).
type Planned3 = (Toolpath, Resolved, &'static str, Option<(f64, f64)>);

/// A 3-axis operation as planned (see [`Planned3`]).
fn plan3_raw(ctx: &Ctx<'_>, op: &Op3Spec, frame: &Frame, surface: &Surface) -> Result<Planned3> {
    let unmerged = |(path, r, kind): (Toolpath, Resolved, &'static str)| (path, r, kind, None);
    match op {
        Op3Spec::Adaptive(o) => adaptive::plan(ctx, o, frame).map(unmerged),
        Op3Spec::AdaptiveRoughing(o) => adaptive::plan_roughing(ctx, o, frame, surface).map(unmerged),
        Op3Spec::Contour(o) => {
            let c = o.common();
            let (leave, tol) = (o.stock_to_leave.unwrap_or(0.0), ctx.tolerance(c.name, o.tolerance)?);
            let side_of = |s: Option<SideSpec>| match s.unwrap_or(SideSpec::Outside) {
                SideSpec::Outside => Side::Outside,
                SideSpec::Inside => Side::Inside,
                SideSpec::On => Side::On,
            };
            // On a feature: its outline and heights (the side follows its kind). Else the
            // sketch or polygon, with the heights and tool worked out from the part.
            let (regions, h, r, side) = if o.feature.is_some() {
                let r = ctx.resolve(&c)?;
                let two = features::op_cut(
                    ctx,
                    c.name,
                    o.sketch.as_ref(),
                    o.profiles.as_deref(),
                    o.polygon.as_deref(),
                    o.feature.as_deref(),
                    o.heights.as_ref(),
                    frame,
                    None,
                )?;
                (two.regions, two.heights, r, side_of(o.side.or(two.side)))
            } else {
                let regions = ctx.regions(c.name, o.sketch.as_ref(), o.profiles.as_deref(), o.polygon.as_deref(), frame)?;
                let side = side_of(o.side);
                let fit = |t: &Tool| agent::fits(&regions, t, leave, tol);
                let region = if side == Side::Outside { agent::Region::Part } else { agent::Region::Floor(&regions) };
                let h = agent::heights(ctx, c.name, o.heights.as_ref(), region, frame, tol)?;
                let r = agent::resolve_for(ctx, &c, if side == Side::Inside { Some(&fit) } else { None }, Some(h.top - h.bottom))?;
                (regions, h, r, side)
            };
            let cut = Cut { heights: h, feeds: r.feeds, stepdown: or_tool(o.stepdown, r.tool.stepdown, r.tool.diameter * 0.5) };
            let params = ContourParams {
                side,
                climb: o.climb.unwrap_or(true),
                stock_to_leave: leave,
                tolerance: tol,
                ramp_angle: ctx.degrees(c.name, o.ramp_angle_deg.unwrap_or(0.0), "`ramp_angle_deg`")?,
            };
            if r.tool.kind == ToolKind::Chamfer {
                // A V-trace: the groove down to its depth is its own.
                ctx.band.set(Some(h.top - h.bottom));
            }
            let opts = leads::contour_options(ctx, c.name, o, &r.tool)?;
            let above = leads::clear_above(surface, &r.tool);
            let path = contour_with(&regions, &r.tool, &params, &opts, &cut, &above)
                .map_err(|e| agent::fit_error(ctx, c.name, &regions, &r.tool, leave, tol, e))?;
            Ok((path, r, "contour", None))
        }
        Op3Spec::Pocket(o) => {
            let c = o.common();
            let (leave, tol) = (o.stock_to_leave.unwrap_or(0.0), ctx.tolerance(c.name, o.tolerance)?);
            let (regions, h, r) = if o.feature.is_some() {
                let r = ctx.resolve(&c)?;
                let two = features::op_cut(
                    ctx,
                    c.name,
                    o.sketch.as_ref(),
                    o.profiles.as_deref(),
                    o.polygon.as_deref(),
                    o.feature.as_deref(),
                    o.heights.as_ref(),
                    frame,
                    Some(r.tool.radius()),
                )?;
                (two.regions, two.heights, r)
            } else {
                let regions = ctx.regions(c.name, o.sketch.as_ref(), o.profiles.as_deref(), o.polygon.as_deref(), frame)?;
                let fit = |t: &Tool| agent::fits(&regions, t, leave, tol);
                let h = agent::heights(ctx, c.name, o.heights.as_ref(), agent::Region::Floor(&regions), frame, tol)?;
                let r = agent::resolve_for(ctx, &c, Some(&fit), Some(h.top - h.bottom))?;
                (regions, h, r)
            };
            let cut = Cut { heights: h, feeds: r.feeds, stepdown: or_tool(o.stepdown, r.tool.stepdown, r.tool.diameter * 0.5) };
            let params = PocketParams {
                stepover: or_tool(o.stepover, r.tool.stepover, r.tool.diameter * 0.4),
                climb: o.climb.unwrap_or(true),
                stock_to_leave: leave,
                tolerance: tol,
                ramp_angle: ctx.degrees(c.name, o.ramp_angle_deg.unwrap_or(0.0), "`ramp_angle_deg`")?,
            };
            let opts = leads::pocket_options(ctx, c.name, o, &r.tool)?;
            let above = leads::clear_above(surface, &r.tool);
            let path = pocket_with(&regions, &r.tool, &params, &opts, &cut, &above)
                .map_err(|e| agent::fit_error(ctx, c.name, &regions, &r.tool, leave, tol, e))?;
            Ok((path, r, "pocket", None))
        }
        Op3Spec::Parallel(o) => {
            let c = o.common();
            let r = ctx.resolve(&c)?;
            let (tolerance, merge) = split_budget(ctx.surface_budget(c.name, o.tolerance)?);
            let descent_limit = match o.descent_limit_deg {
                None => DEFAULT_DESCENT_LIMIT,
                Some(d) if d.is_finite() && (0.0..=90.0).contains(&d) => d.to_radians(),
                Some(_) => return Err(ctx.bad(c.name, "`descent_limit_deg` must be from 0 to 90 degrees")),
            };
            let tol = ctx.tolerance(c.name, o.tolerance)?;
            let params = ParallelParams {
                heights: agent::heights(ctx, c.name, o.heights.as_ref(), agent::Region::Part, frame, tol)?,
                feeds: r.feeds,
                descent_feed: r.descent,
                stepover: or_tool(o.stepover, r.tool.stepover, r.tool.diameter * 0.1),
                angle: ctx.degrees(c.name, o.angle_deg.unwrap_or(0.0), "`angle_deg`")?,
                boundary: ctx.boundary(c.name, o.boundary.as_ref())?,
                stock_to_leave: o.stock_to_leave.unwrap_or(0.0),
                step: o.step.unwrap_or(r.tool.diameter * 0.25),
                tolerance,
                zigzag: o.zigzag.unwrap_or(true),
                descent_limit,
            };
            let path = parallel(surface, &r.tool, &params).map_err(|e| ctx.bad(c.name, e))?;
            Ok((path, r, "parallel", merge))
        }
        Op3Spec::Roughing(o) => {
            let c = o.common();
            let r = ctx.resolve(&c)?;
            let (tolerance, merge) = split_budget(ctx.surface_budget(c.name, o.tolerance)?);
            let tol = ctx.tolerance(c.name, o.tolerance)?;
            let params = RoughingParams {
                heights: agent::heights(ctx, c.name, o.heights.as_ref(), agent::Region::Part, frame, tol)?,
                feeds: r.feeds,
                stepdown: or_tool(o.stepdown, r.tool.stepdown, r.tool.diameter * 0.5),
                stepover: or_tool(o.stepover, r.tool.stepover, r.tool.diameter * 0.4),
                angle: ctx.degrees(c.name, o.angle_deg.unwrap_or(0.0), "`angle_deg`")?,
                boundary: ctx.boundary(c.name, o.boundary.as_ref())?,
                stock_to_leave: o.stock_to_leave.unwrap_or(0.25),
                step: o.step.unwrap_or(r.tool.diameter * 0.25),
                tolerance,
                zigzag: o.zigzag.unwrap_or(true),
            };
            let path = roughing(surface, &r.tool, &params).map_err(|e| ctx.bad(c.name, e))?;
            Ok((path, r, "roughing", merge))
        }
        Op3Spec::Chamfer(o) => {
            let (path, r) = chamfer::plan(ctx, o, frame)?;
            Ok((path, r, "chamfer", None))
        }
        Op3Spec::Waterline(o) => {
            let c = o.common();
            let r = ctx.resolve(&c)?;
            let (tolerance, merge) = split_budget(ctx.surface_budget(c.name, o.tolerance)?);
            let tol = ctx.tolerance(c.name, o.tolerance)?;
            let params = WaterlineParams {
                heights: agent::heights(ctx, c.name, o.heights.as_ref(), agent::Region::Part, frame, tol)?,
                feeds: r.feeds,
                stepdown: or_tool(o.stepdown, r.tool.stepdown, r.tool.diameter * 0.25),
                boundary: ctx.boundary(c.name, o.boundary.as_ref())?,
                stock_to_leave: o.stock_to_leave.unwrap_or(0.0),
                step: o.step.unwrap_or(r.tool.diameter * 0.25),
                tolerance,
                climb: o.climb.unwrap_or(true),
            };
            let path = waterline(surface, &r.tool, &params).map_err(|e| ctx.bad(c.name, e))?;
            Ok((path, r, "waterline", merge))
        }
        Op3Spec::Drill(o) => {
            let (path, r) = drill::plan(ctx, o, frame)?;
            Ok((path, r, "drill", None))
        }
        Op3Spec::Scallop(o) => finish3d::plan_scallop(ctx, o, surface),
        Op3Spec::SteepShallow(o) => finish3d::plan_steep_shallow(ctx, o, surface),
        Op3Spec::Pencil(o) => finish3d::plan_pencil(ctx, o, surface),
        Op3Spec::RestFinish(o) => finish3d::plan_rest_finish(ctx, o, surface),
    }
}

fn common3(op: &Op3Spec) -> Common<'_> {
    match op {
        Op3Spec::Adaptive(o) => o.common(),
        Op3Spec::AdaptiveRoughing(o) => o.common(),
        Op3Spec::Contour(o) => o.common(),
        Op3Spec::Pocket(o) => o.common(),
        Op3Spec::Parallel(o) => o.common(),
        Op3Spec::Roughing(o) => o.common(),
        Op3Spec::Waterline(o) => o.common(),
        Op3Spec::Drill(o) => o.common(),
        Op3Spec::Chamfer(o) => o.common(),
        Op3Spec::Scallop(o) => o.common(),
        Op3Spec::SteepShallow(o) => o.common(),
        Op3Spec::Pencil(o) => o.common(),
        Op3Spec::RestFinish(o) => o.common(),
    }
}

fn name3_mut(op: &mut Op3Spec) -> &mut String {
    match op {
        Op3Spec::Adaptive(o) => &mut o.name,
        Op3Spec::AdaptiveRoughing(o) => &mut o.name,
        Op3Spec::Contour(o) => &mut o.name,
        Op3Spec::Pocket(o) => &mut o.name,
        Op3Spec::Parallel(o) => &mut o.name,
        Op3Spec::Roughing(o) => &mut o.name,
        Op3Spec::Waterline(o) => &mut o.name,
        Op3Spec::Drill(o) => &mut o.name,
        Op3Spec::Chamfer(o) => &mut o.name,
        Op3Spec::Scallop(o) => &mut o.name,
        Op3Spec::SteepShallow(o) => &mut o.name,
        Op3Spec::Pencil(o) => &mut o.name,
        Op3Spec::RestFinish(o) => &mut o.name,
    }
}

/// An adaptive operation's name and `rest_from`, when it has one.
fn rest_from(spec: &OperationSpec) -> Option<(&str, &crate::schema::cam_adaptive::RestFromSpec)> {
    match spec {
        OperationSpec::Adaptive(o) => o.rest_from.as_ref().map(|r| (o.name.as_str(), r)),
        OperationSpec::AdaptiveRoughing(o) => o.rest_from.as_ref().map(|r| (o.name.as_str(), r)),
        _ => None,
    }
}

/// The name of an operation that is not `indexed` (an `indexed_pattern` names its operations
/// after its inner one).
fn spec_name(spec: &OperationSpec) -> Option<&str> {
    Some(match spec {
        OperationSpec::Adaptive(o) => &o.name,
        OperationSpec::AdaptiveRoughing(o) => &o.name,
        OperationSpec::Contour(o) => &o.name,
        OperationSpec::Pocket(o) => &o.name,
        OperationSpec::Parallel(o) => &o.name,
        OperationSpec::Roughing(o) => &o.name,
        OperationSpec::Waterline(o) => &o.name,
        OperationSpec::Drill(o) => &o.name,
        OperationSpec::Scallop(o) => &o.name,
        OperationSpec::SteepShallow(o) => &o.name,
        OperationSpec::Pencil(o) => &o.name,
        OperationSpec::RestFinish(o) => &o.name,
        OperationSpec::RotaryFinish(o) => &o.name,
        OperationSpec::RotaryRoughing(o) => &o.name,
        OperationSpec::Wrap(o) => &o.name,
        OperationSpec::Bore(o) => &o.name,
        OperationSpec::ThreadMill(o) => &o.name,
        OperationSpec::Face(o) => &o.name,
        OperationSpec::Chamfer(o) => &o.name,
        OperationSpec::WrapPocket(o) => &o.name,
        OperationSpec::WrapContour(o) => &o.name,
        OperationSpec::IndexedPattern(o) => common3(&o.operation).name,
        OperationSpec::Indexed(_) => return None,
    })
}

/// One operation planned: what the program holds, its poses in work coordinates (for the
/// checks and the preview), and its report.
#[derive(Clone)]
struct Planned {
    op: MachineOperation,
    poses: Toolpath5,
    /// For each move of `poses`, the move of the operation's own toolpath it comes from.
    sources: Vec<usize>,
    tool: Tool,
    feeds: Feeds,
    gouge_check: bool,
    /// How far up its axis a chamfer mill's cut is its own (mm): the chamfer, the V-trace or the
    /// spotted hole it is told to cut, which the model need not show. Its gouge check holds the
    /// cone against the part less that band ([`band_gouges`]), and the rapid check lets it
    /// retract straight up through it.
    band: Option<f64>,
    report: OperationReport,
    axes: String,
}

fn pattern(ctx: &Ctx<'_>, op: &str, p: PatternSpec) -> Result<RotaryPattern> {
    Ok(match p {
        PatternSpec::Lines { angle_step_deg, zigzag } => {
            RotaryPattern::Lines { angle_step: ctx.degrees(op, angle_step_deg, "`angle_step_deg`")?, zigzag: zigzag.unwrap_or(true) }
        }
        PatternSpec::Helix { pitch } => RotaryPattern::Helix { pitch },
        PatternSpec::Rings { stepover, zigzag } => RotaryPattern::Rings { stepover, zigzag: zigzag.unwrap_or(true) },
    })
}

fn angle_step(ctx: &Ctx<'_>, op: &str, deg: Option<f64>) -> Result<f64> {
    match deg {
        None => Ok(DEFAULT_MAX_ANGLE_STEP),
        Some(d) => ctx.degrees(op, d, "`max_angle_step_deg`"),
    }
}

fn needs_rotary(ctx: &Ctx<'_>, op: &str) -> Result<()> {
    if ctx.machine.rotaries.is_empty() {
        return Err(ctx.bad(op, "a rotary operation needs a machine with a rotary axis (`machine: \"4axis_a\"`)"));
    }
    Ok(())
}

/// The flat path of a wrap operation: across at the clearance height, down to the retract
/// height, plunged to the first point, along the points, and up again.
fn wrap_path(ctx: &Ctx<'_>, op: &str, pts: &[Vec3], clearance: f64, retract: f64, feeds: Feeds) -> Result<Toolpath> {
    if !(clearance.is_finite() && retract.is_finite() && clearance > retract && retract >= 0.0 && clearance <= 1000.0) {
        return Err(ctx.bad(op, "the clearance must be above the retract height, both at or above the cylinder (0) and at most 1000 mm"));
    }
    let (Some(first), Some(last)) = (pts.first(), pts.last()) else { return Err(ctx.bad(op, "the path needs at least 2 points")) };
    if pts.len() < 2 || pts.len() > MAX_POINTS {
        return Err(ctx.bad(op, format!("the path needs 2 to {MAX_POINTS} points")));
    }
    if pts.iter().any(|p| !p.is_finite() || p.x.abs() > MAX_COORD || p.y.abs() > MAX_COORD || p.z > retract || p.z < -MAX_COORD) {
        return Err(ctx.bad(op, "the path's points must be finite, at or below the retract height"));
    }
    let mut moves = vec![
        Move::Rapid { to: Vec3::new(first.x, first.y, clearance) },
        Move::Rapid { to: Vec3::new(first.x, first.y, retract) },
        Move::Feed { to: *first, feed: feeds.plunge },
    ];
    moves.extend(pts.iter().skip(1).map(|p| Move::Feed { to: *p, feed: feeds.cut }));
    moves.push(Move::Rapid { to: Vec3::new(last.x, last.y, clearance) });
    Ok(Toolpath { moves })
}

/// The points of a wrap operation.
fn wrap_points(ctx: &Ctx<'_>, o: &crate::schema::cam::WrapOp) -> Result<Vec<Vec3>> {
    let op = o.name.as_str();
    match (&o.points, &o.sketch) {
        (Some(_), Some(_)) => Err(ctx.bad(op, "give `points` or `sketch` and `curve`, not both")),
        (None, None) => Err(ctx.bad(op, "give `points` or `sketch` and `curve`")),
        (Some(p), None) => {
            if o.curve.is_some() || o.depth.is_some() {
                return Err(ctx.bad(op, "`curve` and `depth` go with `sketch`"));
            }
            Ok(p.iter().map(|q| v3(*q)).collect())
        }
        (None, Some(k)) => {
            let Some(curve) = &o.curve else { return Err(ctx.bad(op, "with `sketch`, give the `curve` id")) };
            let depth = o.depth.unwrap_or(0.0);
            let id = match ctx.s.doc.find_feature(&k.0) {
                Some(f) if matches!(f.kind, solidcraft_doc::FeatureKind::Sketch { .. }) => f.id,
                _ => return Err(ctx.bad(op, format!("no sketch `{}`", k.0))),
            };
            let st = ctx.s.model.state();
            let Some(ss) = st.sketch(id) else { return Err(ctx.bad(op, format!("sketch `{}` has not been solved", k.0))) };
            let Some(ci) = ss.sketch.curve_index(curve) else { return Err(ctx.bad(op, format!("sketch `{}` has no curve `{curve}`", k.0))) };
            Ok(ss.sketch.polyline(ci).into_iter().map(|q| Vec3::new(q.x, q.y, depth)).collect())
        }
    }
}

/// The strategy whose cutting data an operation's tool takes ([`solidcraft_cam::Tool::for_strategy`]).
fn strategy_of(spec: &OperationSpec) -> Option<Strategy> {
    let of3 = |op: &Op3Spec| match op {
        Op3Spec::Adaptive(_) | Op3Spec::AdaptiveRoughing(_) => Some(Strategy::Adaptive),
        Op3Spec::Contour(_) | Op3Spec::Pocket(_) | Op3Spec::Roughing(_) => Some(Strategy::Slot),
        Op3Spec::Parallel(_)
        | Op3Spec::Waterline(_)
        | Op3Spec::Scallop(_)
        | Op3Spec::SteepShallow(_)
        | Op3Spec::Pencil(_)
        | Op3Spec::RestFinish(_) => Some(Strategy::Finish),
        Op3Spec::Drill(_) => Some(Strategy::Drill),
        Op3Spec::Chamfer(_) => Some(Strategy::Chamfer),
    };
    match spec {
        OperationSpec::Adaptive(_) | OperationSpec::AdaptiveRoughing(_) => Some(Strategy::Adaptive),
        OperationSpec::Contour(_)
        | OperationSpec::Pocket(_)
        | OperationSpec::Roughing(_)
        | OperationSpec::RotaryRoughing(_)
        | OperationSpec::Bore(_)
        | OperationSpec::WrapPocket(_)
        | OperationSpec::WrapContour(_) => Some(Strategy::Slot),
        OperationSpec::Parallel(_)
        | OperationSpec::Waterline(_)
        | OperationSpec::Scallop(_)
        | OperationSpec::SteepShallow(_)
        | OperationSpec::Pencil(_)
        | OperationSpec::RestFinish(_)
        | OperationSpec::RotaryFinish(_)
        | OperationSpec::Wrap(_) => Some(Strategy::Finish),
        OperationSpec::Drill(_) => Some(Strategy::Drill),
        OperationSpec::Chamfer(_) => Some(Strategy::Chamfer),
        OperationSpec::Face(_) => Some(Strategy::Face),
        OperationSpec::ThreadMill(_) => None,
        OperationSpec::Indexed(o) => of3(&o.operation),
        OperationSpec::IndexedPattern(o) => of3(&o.operation),
    }
}

fn plan(ctx: &Ctx<'_>, spec: &OperationSpec, arc_tol: f64) -> Result<Planned> {
    ctx.agent.set_strategy(strategy_of(spec));
    if spec_name(spec).is_some_and(|n| n.trim().is_empty()) {
        return Err(bad(ctx.cmd, "every operation needs a `name` (an `indexed` one may carry it on itself or on its inner `operation`)"));
    }
    let machine = ctx.machine;
    let rotaries = machine.rotaries.len();
    let indexed = |op3: &Op3Spec, rotary: Vec<f64>, a_deg: Option<f64>| -> Result<Planned> {
        let c = common3(op3);
        let frame = frame_for_index(machine, &rotary).map_err(|e| ctx.bad(c.name, e))?;
        let turned;
        let surface = if frame == Frame::IDENTITY {
            ctx.part
        } else {
            turned = ctx.part.in_frame(&frame).map_err(|e| ctx.bad(c.name, e))?;
            &turned
        };
        ctx.band.set(None);
        let (path, r, kind) = plan3(ctx, op3, &frame, surface)?;
        let band = ctx.band.take();
        let stats = path.stats();
        let io = IndexedOperation { name: c.name.into(), tool: r.tool.clone(), spindle_rpm: r.rpm, coolant: c.coolant, rotary, path };
        let (poses, sources) = io.poses_mapped(machine, arc_tol).map_err(|e| ctx.bad(c.name, e))?;
        let report = OperationReport {
            name: c.name.into(),
            kind: if a_deg.is_some() { format!("indexed {kind}") } else { kind.into() },
            tool: r.tool.number,
            moves: io.path.moves.len(),
            cut_length: stats.cut_length,
            cut_minutes: stats.cut_minutes,
            max_tilt_deg: poses.stats().max_tilt.to_degrees(),
            a_deg,
            ..OperationReport::default()
        };
        let mut report = report;
        if let Some((extra, warnings)) = ctx.adaptive.borrow_mut().take() {
            report.adaptive = Some(extra);
            report.warnings.extend(warnings);
        }
        let axes = match a_deg {
            Some(a) => format!("3+1 A{}°", num(a)),
            None => "3-axis".into(),
        };
        Ok(Planned {
            op: MachineOperation::Indexed(io),
            poses,
            sources,
            feeds: r.feeds,
            tool: r.tool,
            gouge_check: c.gouge_check,
            band,
            report,
            axes,
        })
    };
    let continuous = |c: Common<'_>, r: Resolved, kind: &str, path: Toolpath5| -> Planned {
        let stats = path.stats();
        let report = OperationReport {
            name: c.name.into(),
            kind: kind.into(),
            tool: r.tool.number,
            moves: path.moves.len(),
            cut_length: stats.cut_length,
            cut_minutes: stats.cut_minutes,
            max_tilt_deg: stats.max_tilt.to_degrees(),
            ..OperationReport::default()
        };
        let op = Operation5 {
            name: c.name.into(),
            tool: r.tool.clone(),
            spindle_rpm: r.rpm,
            coolant: c.coolant,
            path: path.clone(),
            plunge_feed: Some(r.feeds.plunge),
        };
        let sources = (0..path.moves.len()).collect();
        Planned {
            op: MachineOperation::Continuous(op),
            poses: path,
            sources,
            feeds: r.feeds,
            tool: r.tool,
            gouge_check: c.gouge_check,
            band: None,
            report,
            axes: "4-axis".into(),
        }
    };
    match spec {
        OperationSpec::Adaptive(o) => match features::op_index(ctx, &o.name, o.feature.as_deref())? {
            Some((rotary, a)) => indexed(&Op3Spec::Adaptive(o.clone()), rotary, Some(a)),
            None => indexed(&Op3Spec::Adaptive(o.clone()), vec![0.0; rotaries], None),
        },
        OperationSpec::AdaptiveRoughing(o) => indexed(&Op3Spec::AdaptiveRoughing(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Contour(o) => match features::op_index(ctx, &o.name, o.feature.as_deref())? {
            Some((rotary, a)) => indexed(&Op3Spec::Contour(o.clone()), rotary, Some(a)),
            None => indexed(&Op3Spec::Contour(o.clone()), vec![0.0; rotaries], None),
        },
        OperationSpec::Pocket(o) => match features::op_index(ctx, &o.name, o.feature.as_deref())? {
            Some((rotary, a)) => indexed(&Op3Spec::Pocket(o.clone()), rotary, Some(a)),
            None => indexed(&Op3Spec::Pocket(o.clone()), vec![0.0; rotaries], None),
        },
        OperationSpec::Parallel(o) => indexed(&Op3Spec::Parallel(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Roughing(o) => indexed(&Op3Spec::Roughing(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Waterline(o) => indexed(&Op3Spec::Waterline(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Drill(o) => indexed(&Op3Spec::Drill(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Chamfer(o) => indexed(&Op3Spec::Chamfer(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Scallop(o) => indexed(&Op3Spec::Scallop(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::SteepShallow(o) => indexed(&Op3Spec::SteepShallow(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Pencil(o) => indexed(&Op3Spec::Pencil(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::RestFinish(o) => indexed(&Op3Spec::RestFinish(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Indexed(o) => {
            // The inner operation's name, else the wrapper's.
            let mut inner = (*o.operation).clone();
            let inner_name = name3_mut(&mut inner);
            if inner_name.trim().is_empty() {
                match o.name.as_deref().filter(|n| !n.trim().is_empty()) {
                    Some(n) => n.clone_into(inner_name),
                    None => return Err(bad(ctx.cmd, "an `indexed` operation needs a `name`, on itself or on its inner `operation`")),
                }
            }
            let name = common3(&inner).name;
            if rotaries != 1 {
                return Err(ctx.bad(name, "an indexed operation's `a_deg` needs a machine with one rotary axis (`machine: \"4axis_a\"`)"));
            }
            let a = ctx.degrees(name, o.a_deg, "`a_deg`")?;
            indexed(&inner, vec![a], Some(o.a_deg))
        }
        OperationSpec::RotaryFinish(o) => {
            let c = o.common();
            needs_rotary(ctx, c.name)?;
            let r = ctx.resolve(&c)?;
            let params = RotaryParams {
                pattern: pattern(ctx, c.name, o.pattern)?,
                clearance: o.clearance,
                retract: o.retract,
                floor: o.floor.unwrap_or(0.0),
                feeds: r.feeds,
                range: o.range,
                step: o.step.unwrap_or(r.tool.diameter * 0.25),
                tolerance: ctx.tolerance(c.name, o.tolerance)?,
                stock_to_leave: o.stock_to_leave.unwrap_or(0.0),
                max_angle_step: angle_step(ctx, c.name, o.max_angle_step_deg)?,
            };
            let path = rotary_finish(ctx.part, &r.tool, &params).map_err(|e| ctx.bad(c.name, e))?;
            Ok(continuous(c, r, "rotary_finish", path))
        }
        OperationSpec::RotaryRoughing(o) => {
            let c = o.common();
            needs_rotary(ctx, c.name)?;
            let r = ctx.resolve(&c)?;
            let params = RotaryRoughingParams {
                pattern: pattern(ctx, c.name, o.pattern)?,
                stock_radius: roughing_stock_radius(ctx, o)
                    .ok_or_else(|| ctx.bad(c.name, "give `stock_radius`, or the job a `stock` (a `cylinder` about X or a `box`) to take it from"))?,
                clearance: o.clearance,
                retract: o.retract,
                floor: o.floor.unwrap_or(0.0),
                stepdown: or_tool(o.stepdown, r.tool.stepdown, r.tool.diameter * 0.5),
                feeds: r.feeds,
                range: o.range,
                step: o.step.unwrap_or(r.tool.diameter * 0.25),
                tolerance: ctx.tolerance(c.name, o.tolerance)?,
                stock_to_leave: o.stock_to_leave.unwrap_or(0.25),
                max_angle_step: angle_step(ctx, c.name, o.max_angle_step_deg)?,
            };
            let path = rotary_roughing(ctx.part, &r.tool, &params).map_err(|e| ctx.bad(c.name, e))?;
            Ok(continuous(c, r, "rotary_roughing", path))
        }
        OperationSpec::Face(o) => cam_stock::plan_face(ctx, o, arc_tol),
        OperationSpec::Wrap(o) => {
            let c = o.common();
            needs_rotary(ctx, c.name)?;
            let r = ctx.resolve(&c)?;
            let pts = wrap_points(ctx, o)?;
            let flat = wrap_path(ctx, c.name, &pts, o.clearance.unwrap_or(5.0), o.retract.unwrap_or(1.0), r.feeds)?;
            let tol = ctx.tolerance(c.name, None)?;
            let step = angle_step(ctx, c.name, o.max_angle_step_deg)?;
            let path = wrap_with_step(&flat, o.radius, tol, step).map_err(|e| ctx.bad(c.name, e))?;
            Ok(continuous(c, r, "wrap", path))
        }
        OperationSpec::Bore(o) => helical::bore_op(ctx, o, arc_tol),
        OperationSpec::ThreadMill(o) => helical::thread_mill_op(ctx, o, arc_tol),
        OperationSpec::WrapPocket(o) => xrotary::plan_wrap_pocket(ctx, o),
        OperationSpec::WrapContour(o) => xrotary::plan_wrap_contour(ctx, o),
        // `xrotary::plan_op` plans a pattern as its indexed operations.
        OperationSpec::IndexedPattern(o) => Err(ctx.bad(common3(&o.operation).name, "an indexed pattern is planned as its indexed operations")),
    }
}

/// A job planned and its program written in memory: what `cam.job` checks and writes, and what
/// `cam.view.load` plays.
#[derive(Clone)]
struct PlannedJob {
    name: String,
    machine: Machine,
    post_name: String,
    post: Post5,
    tools: Vec<Tool>,
    machine_rates: Option<solidcraft_cam::machine_time::MachineRates>,
    surfaces: Surfaces,
    notes: Vec<String>,
    stock: Option<solidcraft_cam::stock::Stock>,
    planned: Vec<Planned>,
    program: MachineProgram,
    text: String,
    tol: f64,
    origin: Vec3,
    place: Placement,
    /// The chord tolerance the part was tessellated within (mm).
    chord: f64,
    /// How deep a cut into the tessellated part is allowed (mm).
    allowed: f64,
    /// The job's operation each planned one comes from (an indexed pattern plans several).
    op_of: Vec<usize>,
    agent: agent::Agent,
}

/// Plans every operation of the job `loaded` (its profile, machine file and tool libraries read)
/// and writes its program in memory through its post. An operation that cannot be planned
/// refuses the whole job, every such operation listed.
fn plan_job(cmd: &str, s: &Session, loaded: &agent::Loaded) -> Result<PlannedJob> {
    let job = &loaded.job;
    let base = loaded.base.as_deref();
    if job.operations.is_empty() || job.operations.len() > MAX_OPERATIONS {
        return Err(bad(cmd, format!("a job has 1 to {MAX_OPERATIONS} operations")));
    }
    let tol = job.tolerance.unwrap_or(0.01);
    if !(tol.is_finite() && (TOLERANCE_RANGE.0..=TOLERANCE_RANGE.1).contains(&tol)) {
        return Err(bad(cmd, "the job's `tolerance` must be from 0.0001 to 1 mm"));
    }
    let place = placement_of(cmd, s, job)?;
    let origin = place.origin;
    if place.axes.is_some() && !machine_of(cmd, job)?.rotaries.is_empty() {
        return Err(bad(cmd, "`orientation` turns the part on a 3-axis machine; a 4th axis turns it with its own angles"));
    }
    if job.work_offset.is_some_and(|w| !(1..=6).contains(&w)) {
        return Err(bad(cmd, "`work_offset` must be 1 to 6 (G54 to G59)"));
    }
    let name = match &job.name {
        Some(n) if n.chars().count() > 200 || n.chars().any(char::is_control) => {
            return Err(bad(cmd, "the job's `name` must be at most 200 characters, without control characters"));
        }
        Some(n) => n.clone(),
        None => s.doc.name.clone(),
    };
    let machine = machine_of(cmd, job)?;
    let (post_name, post) = post_of(cmd, job, base)?;
    let tools = tools_of(cmd, job, base)?;
    let machine_rates = rates::rates_of(cmd, job, base)?;
    let surfaces = surfaces_of(cmd, s, job, origin, tol)?;
    let mut notes = surfaces.notes.clone();
    let (stock, start) = cam_stock::stock_and_start(cmd, s, job, base, &surfaces, tol, &mut notes)?;
    notes.extend(loaded.notes.iter().cloned());
    let geometry = agent::Geometry::of(s, job, &place, &mut notes)?;
    let agent = agent::Agent::new(cmd, job, geometry, loaded, &machine, &post, machine_rates.as_ref())?;
    if job.work_offset.is_some() && !post.base.writes_work_offset() {
        notes.push(format!("post `{post_name}` writes no work offset: the program runs in whichever the controller has active"));
    }

    let found = features::Lazy::new(job.bodies.clone(), job.fixtures.clone().unwrap_or_default(), origin, tol, &machine, place.axes.is_some());
    let chord = tol * CHORD_SHARE;
    // How deep a cut into the tessellated part is allowed: the tolerance less what the
    // tessellation itself may be off by.
    let allowed = tol - chord;
    let shared = Shared {
        cmd,
        s,
        machine: &machine,
        tools: &tools,
        part: &surfaces.part,
        all: &surfaces.all,
        origin,
        place,
        tol,
        stock: stock.as_ref(),
        start: start.as_ref(),
        chord,
        post: &post,
        features: &found,
    };
    let ctx = shared.ctx(&agent);
    let mut planned: Vec<Planned> = Vec::with_capacity(job.operations.len());
    // Arcs are followed far closer than the post's tolerance for the checks, so their chords
    // are not taken for cuts into the part.
    let pose_tol = post.base.arc_tolerance.min(allowed * CHECK_ARC_SHARE).max(1e-6);
    let mut op_of: Vec<usize> = Vec::with_capacity(job.operations.len());
    let mut refused = Vec::new();
    // Every operation planned on its own with a fork of the agent, side by side natively; then,
    // in order, what each recorded taken back and its set-up checked against the one before:
    // the same plan, notes and refusals as one after the other. Only the operations that fit
    // in the job whatever those before them plan are planned side by side; the rest one after
    // the other, each refused before it is planned when the job has no room for it.
    // An operation with `rest_from` plans on what those before it left: it and those after it
    // are planned one after the other.
    let side_by_side = xrotary::fitting(&job.operations).min(job.operations.iter().position(|op| rest_from(op).is_some()).unwrap_or(usize::MAX));
    let work: Vec<(usize, &OperationSpec, std::sync::Mutex<Option<agent::Agent>>)> =
        job.operations.iter().take(side_by_side).enumerate().map(|(i, spec)| (i, spec, std::sync::Mutex::new(Some(agent.fork())))).collect();
    let scope = solidcraft_io::vfs::current_scope();
    let results = solidcraft_kernel::map_on_threads(&work, PLAN_THREADS, |(i, spec, slot)| {
        let fork = slot.lock().ok().and_then(|mut f| f.take())?;
        solidcraft_io::vfs::scoped(scope.clone(), || {
            fork.start(*i);
            let r = xrotary::plan_op(&shared.ctx(&fork), spec, pose_tol, 0);
            Some((fork, r))
        })
    });
    let mut results = results.into_iter().map(Option::flatten);
    for (i, spec) in job.operations.iter().enumerate() {
        let result = if i < side_by_side {
            results.next().flatten()
        } else {
            let fork = agent.fork();
            fork.start(i);
            let mut c = shared.ctx(&fork);
            let r = match rest_from(spec) {
                Some((name, from)) => adaptive::rest_map(&c, job, name, from, &planned, &op_of).map(|m| c.rest = Some(m)),
                None => Ok(()),
            };
            let r = r.and_then(|()| xrotary::plan_op(&c, spec, pose_tol, planned.len()));
            Some((fork, r))
        };
        let r = match result {
            Some((fork, r)) => {
                agent.absorb(fork);
                r.and_then(|ops| xrotary::add_checked(&ctx, spec, &post, &mut planned, ops))
            }
            None => {
                agent.start(i);
                Err(EngineError::Other(format!("operation {i}: its planning stopped")))
            }
        };
        match r {
            Ok(()) => op_of.resize(planned.len(), i),
            Err(e) => refused.push(agent.refusal_of(e)),
        }
    }
    if !refused.is_empty() {
        return Err(agent::refuse(cmd, "the job was not planned", &refused, "Nothing was written."));
    }

    let program = MachineProgram { name: name.clone(), operations: planned.iter().map(|p| p.op.clone()).collect(), work_offset: job.work_offset };
    let (text, written) = emit_program_report(&program, &post, &machine).map_err(|e| cam_err(cmd, "writing the program", e))?;
    // The rewinds' feeds are time at feed of their operations too, so the operations' minutes add
    // up to the program's as read back.
    for (p, w) in planned.iter_mut().zip(&written) {
        p.report.cut_minutes += w.rewind_minutes;
        p.report.rewinds = w.rewinds;
    }
    Ok(PlannedJob {
        name,
        machine,
        post_name,
        post,
        tools,
        machine_rates,
        surfaces,
        notes,
        stock,
        planned,
        program,
        text,
        tol,
        origin,
        place,
        chord,
        allowed,
        op_of,
        agent,
    })
}

/// The last job planned in a session, kept so that the same job run again (a write after its
/// dry run, a picture after the write) is not planned again ([`plan_cached`]).
pub(crate) struct PlanCache {
    key: PlanKey,
    plan: PlannedJob,
    /// How often it was reused.
    pub(crate) hits: usize,
}

/// What a planned job depends on: the design (the same document, unchanged since), the job as
/// loaded (its profile, machine and tool libraries merged in), the folder its files are read
/// from, and the files planning reads (the post, a tool library, the machine's rates) by their
/// time and size.
struct PlanKey {
    doc: std::sync::Arc<solidcraft_doc::Document>,
    revision: u64,
    job: String,
    base: Option<PathBuf>,
    files: Vec<(String, Option<(std::time::SystemTime, u64)>)>,
}

impl PlanKey {
    fn of(s: &Session, loaded: &agent::Loaded) -> PlanKey {
        let job = &loaded.job;
        let base = loaded.base.as_deref();
        let paths = [&job.post_path, &job.tool_library_path, &job.machine_rates_path].into_iter().flatten();
        let files = paths
            .chain(job.tool_library_paths.iter().flatten())
            .map(|p| beside(base, p))
            .map(|p| (p.clone(), solidcraft_io::vfs::stamp(&p)))
            .collect();
        PlanKey { doc: std::sync::Arc::clone(&s.doc), revision: s.revision, job: format!("{job:?}"), base: loaded.base.clone(), files }
    }

    fn same(&self, other: &PlanKey) -> bool {
        std::sync::Arc::ptr_eq(&self.doc, &other.doc)
            && self.revision == other.revision
            && self.job == other.job
            && self.base == other.base
            && self.files == other.files
    }
}

/// [`plan_job`], or the session's last plan of the same job on the same design: a write after
/// a dry run reuses the dry run's plan. A file planning reads that changed since, or any change
/// to the design, plans again.
fn plan_cached(cmd: &str, s: &Session, loaded: &agent::Loaded) -> Result<PlannedJob> {
    let key = PlanKey::of(s, loaded);
    if let Ok(mut cache) = s.cam_plan.lock() {
        match cache.as_mut() {
            Some(c) if c.key.same(&key) => {
                c.hits += 1;
                return Ok(c.plan.clone());
            }
            _ => *cache = None,
        }
    }
    let plan = plan_job(cmd, s, loaded)?;
    if let Ok(mut cache) = s.cam_plan.lock() {
        *cache = Some(Box::new(PlanCache { key, plan: plan.clone(), hits: 0 }));
    }
    Ok(plan)
}

/// `cam.job`.
pub(super) fn job(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job";
    let params: CamJobParams = parse(cmd, p, "")?;
    let compact = params.compact.unwrap_or(false);
    let reply = run_job(s, cmd, p, params, None)?;
    agent::reply_value(reply, compact)
}

/// `cam.job` (or `cam.view`, as `cmd`) with `params`, read from `raw` (a profile is merged into
/// its job as given); with `only`, its pictures draw that operation alone.
fn run_job(s: &mut Session, cmd: &str, raw: &Value, params: CamJobParams, only: Option<&str>) -> Result<CamJobReply> {
    let dry_run = params.dry_run.unwrap_or(false);
    let allow_unsafe = params.allow_unsafe.unwrap_or(false);

    // Every path is checked before anything is computed or written.
    let output = match (&params.output, dry_run) {
        (Some(o), _) => Some(o.clone()),
        (None, true) => None,
        (None, false) => return Err(bad(cmd, "give `output` (the G-code file to write) or `dry_run: true`")),
    };
    let sheet_format = sheet::format_of(cmd, params.setup_sheet.as_deref())?;
    if !dry_run {
        for path in output.iter().chain(&params.preview).chain(&params.setup_sheet) {
            if path.trim().is_empty() {
                return Err(bad(cmd, "a file path must not be empty"));
            }
            solidcraft_io::vfs::check(path).map_err(|e| EngineError::Other(e.to_string()))?;
        }
        if let (Some(o), Some(p)) = (&output, &params.preview)
            && same_file(o, p)
        {
            return Err(bad(cmd, format!("`output` and `preview` must be different files (`{o}` and `{p}` are the same file)")));
        }
        sheet::distinct(cmd, output.as_deref(), params.preview.as_deref(), params.setup_sheet.as_deref())?;
    }
    let wanted = pictures::Request::of(cmd, &params, only)?;
    let loaded = agent::load_job(cmd, raw, &params)?;
    let job: &CamJob = &loaded.job;
    let PlannedJob {
        name,
        machine,
        post_name,
        post,
        tools,
        machine_rates,
        surfaces,
        mut notes,
        stock,
        mut planned,
        program,
        text,
        tol,
        origin,
        place,
        chord,
        allowed,
        op_of,
        agent,
    } = plan_cached(cmd, s, &loaded)?;
    let rb = readback_program(&text, &program, &post, &machine).map_err(|e| cam_err(cmd, "reading the program back", e))?;
    let machine_time = match &machine_rates {
        Some(r) => Some(rates::time_report(cmd, &text, &machine, &post, r)?),
        None => None,
    };
    if let Some(t) = &machine_time {
        notes.extend(t.warnings.iter().cloned());
    }

    // Gouges and holder collisions, along every cutting move.
    let mut failures: Vec<String> = Vec::new();
    let mut details = Vec::new();
    let has_fixtures = job.fixtures.as_ref().is_some_and(|f| !f.is_empty());
    let mut gouge = GougeReport { tolerance: tol, tessellation: chord, allowed, ..GougeReport::default() };
    let mut holder_total: Option<usize> = None;
    // The operations' own toolpaths, as written (indexed ones), to find the line of a move.
    let own_paths: Vec<Option<&Toolpath>> = planned
        .iter()
        .map(|p| match &p.op {
            MachineOperation::Indexed(io) => Some(&io.path),
            MachineOperation::Continuous(_) => None,
        })
        .collect();
    let b = &post.base;
    let lines = move_lines(&text, &own_paths, b.units.mm(), 10f64.powi(-i32::from(b.decimals)) * b.units.mm());
    for (k, p) in planned.iter_mut().enumerate() {
        let i = op_of.get(k).copied().unwrap_or(k);
        let step = (p.tool.radius() * 0.5).clamp(0.1, 2.0);
        let op = p.report.name.clone();
        // The moves of the operation's own toolpath hits are along.
        let moves_of = |hits: &[Hit]| hits.iter().filter_map(|h| p.sources.get(h.index)).collect::<BTreeSet<_>>().len();
        // A drill is checked as its own shape, point and all; a chamfer mill less the band it
        // is told to cut (its tip, beside the wall, as it runs for a chamfer). The band is cut
        // into the part only: the fixtures are held against the whole tool.
        let tip_clear = p.report.kind.ends_with("chamfer");
        let check = |s: &Surface| match p.band {
            Some(band) => band_gouges(s, &p.tool, &p.poses, band, tip_clear, allowed, step),
            None => tool_gouges(s, &p.tool, &p.poses, allowed, step),
        };
        let check_all = || match (p.band, &surfaces.fixtures) {
            (Some(_), Some(fixtures)) => {
                let mut hits = check(&surfaces.part)?;
                for h in tool_gouges(fixtures, &p.tool, &p.poses, allowed, step)? {
                    match hits.iter_mut().find(|k| k.index == h.index) {
                        Some(k) if h.distance > k.distance => *k = h,
                        Some(_) => {}
                        None => hits.push(h),
                    }
                }
                hits.sort_by_key(|h| h.index);
                Ok(hits)
            }
            _ => check(&surfaces.all),
        };
        if p.gouge_check {
            let hits = check_all().map_err(|e| cam_err(cmd, &format!("operation `{op}`: gouge check"), e))?;
            pictures::note_problems(&mut p.report, "gouge", &hits);
            let moves = moves_of(&hits);
            if let Some(worst) = hits.iter().max_by(|a, b| a.distance.total_cmp(&b.distance)) {
                let move_index = p.sources.get(worst.index).copied().unwrap_or(worst.index);
                let line = lines.get(k).and_then(|l| l.get(move_index)).copied().flatten();
                let tip = worst.pose.tip;
                let at = GougeAt { operation: op.clone(), move_index, line, xyz: [tip.x, tip.y, tip.z] };
                failures.push(format!(
                    "gouge: operation `{op}` cuts {} mm into the part or a fixture as tessellated (the check allows {} mm: the {} mm tolerance \
                     less the tessellation's {} mm), up to {} mm into the exact part, along {moves} moves; deepest at move {move_index}{} at \
                     X{} Y{} Z{} (work coordinates)",
                    num(worst.distance),
                    num(allowed),
                    num(tol),
                    num(chord),
                    num(worst.distance + chord),
                    line.map_or(String::new(), |l| format!(" (G-code line {l})")),
                    num(tip.x),
                    num(tip.y),
                    num(tip.z)
                ));
                p.report.gouge = worst.distance;
                if worst.distance > gouge.worst {
                    gouge.worst = worst.distance;
                    gouge.at = Some(GougeAt { operation: op.clone(), ..at.clone() });
                }
                p.report.gouge_at = Some(at);
                let on_part = !has_fixtures || !matches!(check(&surfaces.part), Ok(h) if h.is_empty());
                details.push(agent.gouge_refusal(i, p, &hits, on_part, &text, &machine));
            }
            gouge.hits += moves;
        } else {
            // Not against the part, but a fixture is never cut.
            gouge.skipped.push(op.clone());
            p.report.warnings.push("not checked for gouges into the part (`gouge_check: false`); checked against the fixtures".into());
            if let Some(fixtures) = &surfaces.fixtures {
                let hits = tool_gouges(fixtures, &p.tool, &p.poses, allowed, step)
                    .map_err(|e| cam_err(cmd, &format!("operation `{op}`: fixture check"), e))?;
                let moves = moves_of(&hits);
                if let Some(worst) = hits.iter().max_by(|a, b| a.distance.total_cmp(&b.distance)) {
                    let move_index = p.sources.get(worst.index).copied().unwrap_or(worst.index);
                    let line = lines.get(k).and_then(|l| l.get(move_index)).copied().flatten();
                    let tip = worst.pose.tip;
                    let at = GougeAt { operation: op.clone(), move_index, line, xyz: [tip.x, tip.y, tip.z] };
                    failures.push(format!(
                        "gouge: operation `{op}` cuts {} mm into a fixture along {moves} moves; deepest at move {move_index}{} at X{} Y{} Z{} (work coordinates)",
                        num(worst.distance),
                        line.map_or(String::new(), |l| format!(" (G-code line {l})")),
                        num(tip.x),
                        num(tip.y),
                        num(tip.z)
                    ));
                    p.report.gouge = worst.distance;
                    if worst.distance > gouge.worst {
                        gouge.worst = worst.distance;
                        gouge.at = Some(at.clone());
                    }
                    p.report.gouge_at = Some(at);
                }
                gouge.hits += moves;
            }
        }
        let (checked, least) = agent.holder_tool(i, &p.tool);
        if !checked.envelopes().is_empty() {
            let hits = holder_collisions_within(&surfaces.all, &checked, &p.poses, 0.0, allowed, step)
                .map_err(|e| cam_err(cmd, &format!("operation `{op}`: holder check"), e))?;
            pictures::note_problems(&mut p.report, "holder", &hits);
            let moves = moves_of(&hits);
            p.report.holder_collisions = moves;
            *holder_total.get_or_insert(0) += moves;
            if moves > 0 {
                failures.push(format!("holder: operation `{op}`: the tool's shank or holder touches the part or a fixture along {moves} moves"));
                details.push(agent.holder_refusal(i, p, &checked, least, &hits, &text, &machine));
            }
        }
    }
    gouge.pass = gouge.hits == 0;
    // Every operation's lowest cut against the stock's bottom.
    if let Some(st) = stock.as_ref() {
        cam_stock::check_below_stock(&planned, st, agent.geo.fixture_low(), &machine, tol, &mut failures);
    }
    // Every rapid of the program as written, and every position against the machine's travel.
    let unchecked: Vec<bool> = planned.iter().map(|p| !p.gouge_check || p.band.is_some()).collect();
    let (rapids, travel) =
        cam_stock::check_motion(cmd, job, &text, &program, &post, &machine, &surfaces, stock.as_ref(), &unchecked, tol, &mut failures)?;
    let simulation = if params.simulate.unwrap_or(false) {
        // Every cut into the part is a failure, but those of an operation without a gouge check
        // and, where the program's blocks can be put under their operations, a band's own.
        let deliberate = if planned.iter().any(|p| !p.gouge_check) {
            sim::Deliberate::All
        } else if planned.iter().any(|p| p.band.is_some()) {
            let blocks = parse5(&text, &machine.letters()).map_err(|e| cam_err(cmd, "reading the program back", e))?;
            match solidcraft_cam::rapids::owners(&program, &post, &machine, &blocks) {
                Some(owner) => sim::Deliberate::Bands {
                    ops: planned.iter().map(|p| (p.report.name.clone(), p.band.is_some())).collect(),
                    op_of_line: blocks.iter().zip(owner).map(|(b, k)| (b.line(), k)).collect(),
                },
                None => sim::Deliberate::All,
            }
        } else {
            sim::Deliberate::None
        };
        // A thread mill cuts in the simulation as the core its operation's gouge check took.
        let cores: Vec<Tool> = planned
            .iter()
            .map(|p| p.tool.clone())
            .filter(|c| c.kind != ToolKind::ThreadMill && tools.iter().any(|t| t.number == c.number && t.kind == ToolKind::ThreadMill))
            .collect();
        let sim_tools = sim::with_thread_cores(&tools, &cores);
        match sim::check_program(cmd, &text, &machine, &sim_tools, &surfaces.part, stock.as_ref(), tol, params.simulate_resolution, &deliberate) {
            Ok((reply, _)) => {
                failures.extend(reply.failures.iter().cloned());
                Some(reply)
            }
            Err(e) => {
                failures.push(format!("simulation: {e}"));
                None
            }
        }
    } else {
        None
    };
    let readback = ReadbackReport {
        max_tip_deviation: rb.max_tip_deviation,
        max_axis_deviation_deg: rb.max_axis_deviation.to_degrees(),
        allowed_tip: rb.allowed_tip,
        allowed_axis_deg: rb.allowed_axis.to_degrees(),
        cut_minutes: rb.cut_minutes,
        pass: rb.within(),
    };
    if !readback.pass {
        failures.insert(
            0,
            format!(
                "read-back: the written program is up to {} mm and {}° from its toolpaths (the post's precision allows {} mm and {}°)",
                num(readback.max_tip_deviation),
                num(readback.max_axis_deviation_deg),
                num(readback.allowed_tip),
                num(readback.allowed_axis_deg)
            ),
        );
        details.insert(0, agent::readback_refusal(&readback, &post));
    }
    // The checks without a structured refusal of their own (rapids, travel, the stock's bottom,
    // the simulation) are listed in the refusal too.
    let names: Vec<(usize, &str)> = planned.iter().enumerate().map(|(k, p)| (op_of.get(k).copied().unwrap_or(k), p.report.name.as_str())).collect();
    for f in &failures {
        if !["gouge", "holder", "read-back"].iter().any(|c| f.starts_with(c)) {
            details.push(agent::failure_refusal(f, &names));
        }
    }
    let passed = failures.is_empty();
    if !passed && !dry_run && !allow_unsafe {
        return Err(agent::refuse(
            cmd,
            "nothing was written: checks failed",
            &details,
            "Fix the job, run it with `dry_run: true` to see the report, or pass `allow_unsafe: true`.",
        ));
    }

    pictures::extents(&machine, &post, &program, &mut planned);
    // Said in a dry run too, before anything is written.
    if let Some(path) = &output
        && let Some(ext) = Path::new(path).extension().map(|e| e.to_string_lossy().to_ascii_lowercase())
        && ext != post.base.extension.to_ascii_lowercase()
    {
        notes.push(format!("post `{post_name}` writes .{} files; this one is .{ext}", post.base.extension));
    }
    let setup = sheet::build(&sheet::Inputs {
        name: &name,
        machine: &machine,
        post_name: &post_name,
        post: &post,
        work_offset: job.work_offset,
        origin,
        part: &surfaces.part,
        planned: &planned,
        text: &text,
        readback: &readback,
        gouge: &gouge,
        holder_checked: holder_total.is_some(),
        failures: &failures,
        notes: &notes,
        program_file: if dry_run { None } else { output.as_deref() },
        stock: stock.as_ref(),
        fixtures: sheet::fixture_words(s, job.fixtures.as_deref().unwrap_or_default(), &place),
        attention: agent.limits().into_iter().chain(machine_time.iter().flat_map(|t| t.limits.iter().cloned())).collect(),
    });
    let summary = sheet::summary(&setup);

    let mut outputs = Outputs::default();
    if !dry_run {
        if let Some(path) = &output {
            write_file(path, text.as_bytes()).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
            outputs.gcode = Some(path.clone());
        }
        if let Some(path) = &params.preview {
            let page = preview_page(&name, &surfaces.all, &planned, &text);
            write_file(path, page.as_bytes()).map_err(|e| {
                let written = outputs.gcode.as_deref().map_or(String::new(), |g| format!(" (the G-code was written to {g})"));
                EngineError::Other(format!("{path}: {e}{written}"))
            })?;
            outputs.preview = Some(path.clone());
        }
        if let (Some(path), Some(format)) = (&params.setup_sheet, sheet_format) {
            write_file(path, setup.render(format).as_bytes()).map_err(|e| {
                let written: Vec<&str> = outputs.gcode.iter().chain(&outputs.preview).map(String::as_str).collect();
                let written = if written.is_empty() { String::new() } else { format!(" ({} written)", written.join(" and ")) };
                EngineError::Other(format!("{path}: {e}{written}"))
            })?;
            outputs.setup_sheet = Some(path.clone());
        }
    }
    let cut_minutes = machine_time.as_ref().map_or(readback.cut_minutes, |t| t.cut_minutes);
    let rapid_minutes = machine_time.as_ref().map(|t| t.rapid_minutes);
    let setting = pictures::Setting { machine: &machine, origin, place, surface: &surfaces.all, tol, name: &name };
    let images = pictures::write(cmd, s, &wanted, &setting, &planned).map_err(|e| match (e, &outputs.gcode) {
        (EngineError::Other(m), Some(g)) => EngineError::Other(format!("{m} (the G-code was written to {g})")),
        (e, _) => e,
    })?;

    let mut reply = CamJobReply {
        name,
        machine: machine.name.clone(),
        post: post_name,
        operations: planned.into_iter().map(|p| p.report).collect(),
        program_lines: text.lines().count(),
        program_head: text.lines().take(HEAD_LINES).map(str::to_string).collect(),
        readback,
        cut_minutes,
        rapid_minutes,
        machine_time,
        gouge,
        holder_collisions: holder_total,
        rapids,
        travel,
        stock: stock.as_ref().map(cam_stock::stock_report),
        passed,
        failures,
        dry_run,
        outputs,
        notes,
        simulation,
        summary,
        images,
        param_warnings: Vec::new(),
        failure_details: details,
        op_of,
        ..CamJobReply::default()
    };
    agent.finish(&mut reply);
    Ok(reply)
}

/// Writes `bytes` to `path` whole: checked against the folders the caller may use, then to a
/// temporary file beside it that is renamed over it, so a reader (the controller's share, a
/// viewer) never sees half a program and a failed write leaves the old file as it was. In the
/// browser the host stores files, and one write is whole there already.
pub(super) fn write_file(path: &str, bytes: &[u8]) -> std::io::Result<()> {
    solidcraft_io::vfs::check(path)?;
    solidcraft_io::write_atomic(Path::new(path), bytes, false)
}

/// `path` as one file is named whatever its spelling: absolute, its folder resolved (links,
/// `.` and `..`) or the whole path if the file is there, `/` between parts, and on Windows
/// without the dots and spaces it drops from the end of a name and in lower case, as its file
/// names are not case-sensitive.
fn file_key(path: &str) -> String {
    let p = Path::new(path.trim());
    let abs = if p.is_relative() { std::env::current_dir().map(|d| d.join(p)).unwrap_or_else(|_| p.to_path_buf()) } else { p.to_path_buf() };
    let lexical = |q: &Path| {
        let mut out = PathBuf::new();
        for c in q.components() {
            match c {
                std::path::Component::CurDir => {}
                std::path::Component::ParentDir => {
                    out.pop();
                }
                other => out.push(other),
            }
        }
        out
    };
    let abs = lexical(&abs);
    let key = match (abs.parent(), abs.file_name()) {
        (Some(dir), Some(name)) => {
            // Windows drops the dots and spaces at the end of a name (`a.tap.` opens `a.tap`).
            let name = name.to_string_lossy();
            let name = if cfg!(windows) { name.trim_end_matches(['.', ' ']) } else { &name };
            // A file that is there already is named by itself (a link to it, a short name).
            let joined = dir.join(name);
            std::fs::canonicalize(&joined).unwrap_or_else(|_| std::fs::canonicalize(dir).unwrap_or_else(|_| dir.to_path_buf()).join(name))
        }
        _ => abs,
    };
    let text = key.to_string_lossy().replace('\\', "/");
    let text = text.strip_prefix("//?/").map(str::to_string).unwrap_or(text);
    if cfg!(windows) { text.to_lowercase() } else { text }
}

/// Whether two paths name the same file ([`file_key`]).
fn same_file(a: &str, b: &str) -> bool {
    file_key(a) == file_key(b)
}

/// For each operation whose toolpath `paths` gives in the program's own coordinates, the line
/// (counted from 1) of `text` each of its moves ends on, where it is found: the next line that
/// leaves the axes where the move ends (to within `digit` mm, the last written digit), looking
/// on from the move before. A move written as no line (too short to change a word) ends on the
/// line before. Words are in the post's units, `unit` mm each; lines in incremental
/// coordinates or that home the axes leave them unknown.
fn move_lines(text: &str, paths: &[Option<&Toolpath>], unit: f64, digit: f64) -> Vec<Vec<Option<usize>>> {
    // Every line that leaves all three axes known, and where it leaves them.
    let mut ends: Vec<(usize, Vec3)> = Vec::new();
    let mut at: [Option<f64>; 3] = [None; 3];
    let mut incremental = false;
    for (no, raw) in text.lines().enumerate() {
        let mut clean = String::with_capacity(raw.len());
        let mut depth = 0usize;
        for c in raw.chars() {
            match c {
                '(' => depth += 1,
                ')' => depth = depth.saturating_sub(1),
                ';' if depth == 0 => break,
                c if depth == 0 => clean.push(c.to_ascii_uppercase()),
                _ => {}
            }
        }
        let mut words: Vec<(char, f64)> = Vec::new();
        let mut chars = clean.chars().peekable();
        while let Some(c) = chars.next() {
            if !c.is_ascii_alphabetic() {
                continue;
            }
            let mut v = String::new();
            while let Some(&d) = chars.peek() {
                if d.is_ascii_digit() || d == '.' || d == '-' || d == '+' || d == ' ' {
                    if d != ' ' {
                        v.push(d);
                    }
                    chars.next();
                } else {
                    break;
                }
            }
            if let Ok(x) = v.parse::<f64>() {
                words.push((c, x));
            }
        }
        let g = |code: f64| words.iter().any(|&(c, v)| c == 'G' && (v - code).abs() < 1e-6);
        if g(91.0) {
            incremental = true;
        }
        if g(90.0) {
            incremental = false;
        }
        let homing = g(28.0) || g(53.0);
        let mut moved = false;
        for (k, letter) in ['X', 'Y', 'Z'].into_iter().enumerate() {
            if let Some(&(_, v)) = words.iter().find(|w| w.0 == letter)
                && let Some(slot) = at.get_mut(k)
            {
                *slot = if homing || incremental { None } else { Some(v * unit) };
                moved = true;
            }
        }
        if moved && let [Some(x), Some(y), Some(z)] = at {
            ends.push((no + 1, Vec3::new(x, y, z)));
        }
    }
    let near = 0.5 * digit + 1e-9;
    let mut cursor = 0usize;
    paths
        .iter()
        .map(|path| {
            let Some(path) = path else { return Vec::new() };
            path.moves
                .iter()
                .map(|m| {
                    let to = m.target()?;
                    let from = cursor.saturating_sub(1);
                    let found = ends
                        .get(from..ends.len().min(cursor.saturating_add(LINE_WINDOW)))?
                        .iter()
                        .position(|(_, p)| (p.x - to.x).abs() <= near && (p.y - to.y).abs() <= near && (p.z - to.z).abs() <= near)?;
                    let k = from + found;
                    cursor = k + 1;
                    ends.get(k).map(|(line, _)| *line)
                })
                .collect()
        })
        .collect()
}

/// `text` safe to put inside HTML.
fn escape_html(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            c => out.push(c),
        }
    }
    out
}

fn round(v: f64, k: f64) -> f64 {
    (v * k).round() / k
}

fn pose_rows(path: &Toolpath5) -> Vec<[f64; 7]> {
    path.points()
        .into_iter()
        .map(|(p, cut)| {
            [
                round(p.tip.x, 1000.0),
                round(p.tip.y, 1000.0),
                round(p.tip.z, 1000.0),
                round(p.axis.x, 1000.0),
                round(p.axis.y, 1000.0),
                round(p.axis.z, 1000.0),
                if cut { 1.0 } else { 0.0 },
            ]
        })
        .collect()
}

/// At most `max` of `rows`, evenly picked.
fn thin<T: Copy>(rows: &[T], max: usize) -> Vec<T> {
    let every = rows.len().div_ceil(max.max(1)).max(1);
    rows.iter().step_by(every).copied().collect()
}

/// The preview page: one scene per operation and one for the whole job.
fn preview_page(name: &str, surface: &Surface, planned: &[Planned], text: &str) -> String {
    let tris: Vec<f64> = thin(surface.triangles(), PREVIEW_TRIANGLES)
        .iter()
        .flat_map(|t| t.iter().flat_map(|p| [round(p.x, 100.0), round(p.y, 100.0), round(p.z, 100.0)]))
        .collect();
    let gcode: String = text.lines().take(PREVIEW_GCODE_LINES).fold(String::new(), |mut acc, l| {
        let _ = writeln!(acc, "{l}");
        acc
    });
    let stats = |r: &OperationReport| {
        format!("{} moves · cut {} mm · {} min · tilt up to {}°", r.moves, num(r.cut_length.round()), num(r.cut_minutes), num(r.max_tilt_deg))
    };
    let scene = |title: &str, axes: &str, short: &str, desc: &str, poses: Vec<[f64; 7]>, stats: String, tool: Option<&Tool>| {
        json!({
            "name": escape_html(title),
            "axes": escape_html(axes),
            "short": escape_html(short),
            "desc": escape_html(desc),
            "tris": tris,
            "poses": thin(&poses, PREVIEW_POSES),
            "gcode": gcode,
            "stats": stats,
            "tool_radius": tool.map_or(2.0, Tool::radius),
            "tool_length": tool.map_or(25.0, |t| t.stickout.unwrap_or(t.flute_length).max(t.flute_length)),
        })
    };
    let mut scenes = Vec::with_capacity(planned.len() + 1);
    let all: Vec<[f64; 7]> = planned.iter().flat_map(|p| pose_rows(&p.poses)).collect();
    let (moves, length, minutes) =
        planned.iter().fold((0, 0.0, 0.0), |(m, l, t), p| (m + p.report.moves, l + p.report.cut_length, t + p.report.cut_minutes));
    let job_axes: BTreeSet<&str> = planned.iter().map(|p| p.axes.as_str()).collect();
    scenes.push(scene(
        name,
        &job_axes.into_iter().collect::<Vec<_>>().join(", "),
        &format!("the whole job, {} operations", planned.len()),
        &format!("Every operation of the job, in program order: {}.", planned.iter().map(|p| p.report.name.as_str()).collect::<Vec<_>>().join(", ")),
        all,
        format!("{moves} moves · cut {} mm · {} min", num(length.round()), num(minutes)),
        planned.first().map(|p| &p.tool),
    ));
    for p in planned {
        let t = &p.tool;
        let desc = format!(
            "{} with tool {} ({} mm {}{}).",
            p.report.kind,
            t.number,
            num(t.diameter),
            t.kind.label(),
            if t.description.is_empty() { String::new() } else { format!(", {}", t.description) }
        );
        scenes.push(scene(&p.report.name, &p.axes, &p.report.kind, &desc, pose_rows(&p.poses), stats(&p.report), Some(t)));
    }
    // `</` would end the script early; `<\/` is the same string in JSON and JavaScript.
    let data = json!({ "scenes": scenes }).to_string().replace("</", "<\\/");
    PREVIEW_TEMPLATE.replacen("/*DATA*/null", &data, 1)
}

/// `cam.posts`.
fn posts(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.posts";
    let params: CamPostsParams = parse(cmd, p, "")?;
    let reply = match params.name {
        Some(name) => {
            let Some((id, post)) = builtin_post(&name) else { return Err(bad(cmd, format!("unknown post `{name}` (built in: {})", post_names()))) };
            CamPostsReply {
                post: Some(serde_json::to_value(post).map_err(|e| EngineError::Other(e.to_string()))?),
                name: Some(id.into()),
                ..CamPostsReply::default()
            }
        }
        None => CamPostsReply {
            posts: POSTS
                .iter()
                .map(|(id, description, aliases, _)| PostEntry {
                    name: (*id).into(),
                    description: (*description).into(),
                    aliases: aliases.iter().map(|a| (*a).to_string()).collect(),
                })
                .collect(),
            ..CamPostsReply::default()
        },
    };
    serde_json::to_value(reply).map_err(|e| EngineError::Other(e.to_string()))
}

/// The motion of `text`, read with every rotary letter it uses: [`parse5`] refuses a rotary
/// word the machine lacks, so each refusal adds that letter and reads again.
fn parse_any(text: &str) -> (Vec<AxisLetter>, std::result::Result<Vec<JointWords>, CamError>) {
    let mut letters: Vec<AxisLetter> = Vec::new();
    loop {
        match parse5(text, &letters) {
            Err(CamError::Parse(line, msg)) if msg.starts_with("the machine has no ") => {
                let letter = match msg.trim_start_matches("the machine has no ").chars().next() {
                    Some('A') => AxisLetter::A,
                    Some('B') => AxisLetter::B,
                    Some('C') => AxisLetter::C,
                    _ => return (letters, Err(CamError::Parse(line, msg))),
                };
                if letters.contains(&letter) {
                    return (letters, Err(CamError::Parse(line, msg)));
                }
                letters.push(letter);
                letters.sort();
            }
            other => return (letters, other),
        }
    }
}

/// `cam.check`.
fn check(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.check";
    let params: CamCheckParams = parse(cmd, p, "")?;
    if params.rapid_rate.is_some_and(|r| !(r > 0.0 && r <= 1.0e6)) {
        return Err(bad(cmd, "`rapid_rate` must be above 0 and at most 1000000 mm/min"));
    }
    let bytes = solidcraft_io::vfs::read(&params.path, MAX_GCODE_BYTES).map_err(|e| EngineError::Other(format!("{}: {e}", params.path)))?;
    let text = String::from_utf8_lossy(&bytes);
    let mut reply = CamCheckReply { path: params.path.clone(), lines: text.lines().count(), ..CamCheckReply::default() };
    let (letters, parsed) = parse_any(&text);
    // What the reader takes up to its first error, when there is one.
    let blocks = match parsed {
        Ok(b) => b,
        Err(e) => {
            let (line, message) = match e {
                CamError::Parse(line, msg) => (line, msg),
                other => (0, other.to_string()),
            };
            reply.errors.push(ParseErrorReport { line, message });
            let upto: String = text.lines().take(line.saturating_sub(1)).fold(String::new(), |mut acc, l| {
                acc.push_str(l);
                acc.push('\n');
                acc
            });
            parse5(&upto, &letters).unwrap_or_default()
        }
    };
    let mut ranges: Vec<(f64, f64)> = vec![(f64::INFINITY, f64::NEG_INFINITY); letters.len()];
    let (mut lo, mut hi) =
        (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
    let mut last: Option<(Vec3, Vec<f64>)> = None;
    for b in &blocks {
        reply.motion_blocks += 1;
        let (linear, rotary) = match b {
            JointWords::Rapid { linear, rotary, .. } => {
                if let Some((from, _)) = &last {
                    reply.rapid_length += from.dist(*linear);
                }
                (*linear, rotary.clone())
            }
            JointWords::Feed { linear, rotary, f, inverse_time, mm_per_unit, .. } => {
                let (len, turn) = last
                    .as_ref()
                    .map_or((0.0, 0.0), |(from, r)| (from.dist(*linear), r.iter().zip(rotary).map(|(a, b)| (b - a).abs()).fold(0.0, f64::max)));
                reply.feed_length += len;
                if *inverse_time {
                    reply.g93_blocks += 1;
                    reply.cut_minutes += 1.0 / f;
                } else if len > 1e-9 {
                    reply.cut_minutes += len / (f * mm_per_unit);
                } else {
                    reply.cut_minutes += turn.to_degrees() / f;
                }
                (*linear, rotary.clone())
            }
            JointWords::Arc { linear, rotary, f, mm_per_unit, .. } | JointWords::PlaneArc { linear, rotary, f, mm_per_unit, .. } => {
                reply.arcs += 1;
                // Measured round the arc in its own plane (G17, G18 or G19).
                if let Some((from, _)) = &last
                    && let Some(arc) = b.arc_from(*from)
                {
                    let len = arc.length();
                    reply.feed_length += len;
                    reply.cut_minutes += len / (f * mm_per_unit);
                    // A vertical arc can reach past its ends along Z.
                    if matches!(b, JointWords::PlaneArc { .. }) {
                        for q in arc.points(1e-3) {
                            lo = lo.min(q);
                            hi = hi.max(q);
                        }
                    }
                }
                (*linear, rotary.clone())
            }
            JointWords::Dwell { seconds, .. } => {
                reply.cut_minutes += seconds / 60.0;
                continue;
            }
        };
        lo = lo.min(linear);
        hi = hi.max(linear);
        for (range, a) in ranges.iter_mut().zip(&rotary) {
            range.0 = range.0.min(*a);
            range.1 = range.1.max(*a);
        }
        last = Some((linear, rotary));
    }
    reply.rotary_ranges = letters
        .iter()
        .zip(&ranges)
        .filter(|(_, r)| r.0 <= r.1)
        .map(|(l, r)| RotaryRange { letter: l.char().to_string(), min_deg: r.0.to_degrees(), max_deg: r.1.to_degrees() })
        .collect();
    if lo.is_finite() && hi.is_finite() {
        reply.bounds = Some([[lo.x, lo.y, lo.z], [hi.x, hi.y, hi.z]]);
    }
    reply.rapid_minutes = params.rapid_rate.map(|r| reply.rapid_length / r);
    serde_json::to_value(reply).map_err(|e| EngineError::Other(e.to_string()))
}

#[path = "cam_adaptive.rs"]
mod adaptive;
#[path = "cam_agent.rs"]
mod agent;
#[path = "cam_stock.rs"]
mod cam_stock;
#[path = "cam_chamfer.rs"]
mod chamfer;
#[path = "cam_drill.rs"]
mod drill;
#[path = "cam_features.rs"]
pub(super) mod features;
#[path = "cam_finish3d.rs"]
mod finish3d;
#[path = "cam_helical.rs"]
mod helical;
#[path = "cam_leads.rs"]
mod leads;
#[path = "cam_image.rs"]
pub(super) mod pictures;
#[path = "cam_rates.rs"]
mod rates;
#[path = "cam_sheet.rs"]
mod sheet;
#[path = "cam_sim.rs"]
mod sim;
#[path = "cam_suggest.rs"]
pub(super) mod suggest;
#[path = "cam_view.rs"]
mod view;
#[path = "cam_xrotary.rs"]
mod xrotary;

#[cfg(test)]
#[path = "cam_tests.rs"]
mod tests;
