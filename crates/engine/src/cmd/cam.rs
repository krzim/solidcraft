//! CAM (`cam.job`, `cam.posts`, `cam.check`): toolpaths for a design's bodies and G-code for a
//! machine, through the `solidcraft-cam` crate. The job (machine, post, tools, operations) is a
//! JSON file next to the design, never part of it, so nothing new is saved in a design
//! (docs/cam-jobs.md).
//!
//! `cam.job` tessellates the part's bodies from their exact surfaces at half the job's
//! tolerance into a surface in work coordinates (the design less the job's origin), runs every
//! operation on it, writes the program through the job's post for its machine, reads that back
//! and compares it with the toolpaths, and checks every cutting move for gouges against the part
//! and fixtures and, for tools with holder data, for shank and holder collisions. A program that
//! fails a check is not written unless the call allows it. Every path is checked against the
//! session's folders (`vfs`, which MCP's `--allow-path` confines) before anything is computed or
//! written.

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use serde::de::DeserializeOwned;
use serde_json::{Value, json};
use solidcore_record::{Loop2, Region2, Seg2, Vec2, Vec3};
use solidcraft_cam::collision::{gouges, holder_collisions};
use solidcraft_cam::gcode5::{JointWords, MachineOperation, MachineProgram, Post5, Rewind, Tcp, emit_program, parse5};
use solidcraft_cam::indexed::{IndexedOperation, frame_for_index};
use solidcraft_cam::kinematics::{AxisLetter, Carrier, Machine, RotaryAxis};
use solidcraft_cam::readback5::readback_program;
use solidcraft_cam::rotary::{RotaryParams, RotaryPattern, RotaryRoughingParams, rotary_finish, rotary_roughing, wrap_with_step};
use solidcraft_cam::surface_ops::{Boundary, ParallelParams, RoughingParams, WaterlineParams, parallel, roughing, waterline};
use solidcraft_cam::{
    CamError, ContourParams, Cut, DEFAULT_MAX_ANGLE_STEP, Feeds, Frame, Heights, HolderSection, LineNumbers, Move, Operation5, PocketParams,
    SafeRetract, Side, Surface, Tool, ToolKind, Toolpath, Toolpath5, arc_sweep, contour, pocket,
};
use solidcraft_geom::Mesh;
use solidcraft_geom::prelude::*;

use super::CommandSpec;
use crate::params::bad;
use crate::schema::cam::{
    BoundarySpec, CamCheckParams, CamCheckReply, CamJob, CamJobParams, CamJobReply, CamPostsParams, CamPostsReply, Common, GougeReport, HeightsSpec,
    MachineSpec, Op3Spec, OperationReport, OperationSpec, Outputs, ParseErrorReport, PatternSpec, PostEntry, ReadbackReport, RotaryRange, SideSpec,
};
use crate::schema::{TypedContract, output_schema_for};
use crate::{EngineError, Result, Session};

const JOB_CONTRACT: TypedContract<CamJobParams> = TypedContract::new();
const POSTS_CONTRACT: TypedContract<CamPostsParams> = TypedContract::new();
const CHECK_CONTRACT: TypedContract<CamCheckParams> = TypedContract::new();

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.job", "CAM Job", crate::checked!("cam.job", job))
        .noundo()
        .contract(&JOB_CONTRACT)
        .output(output_schema_for::<CamJobReply>)
        .params(
            "job: {…} | job_path: .json (the CAM job, docs/cam-jobs.md: name?, machine: \"3axis\" | \"4axis_a\" | Machine JSON, a_reversed?, \
             a_limits_deg?: [min, max], post: built-in name (cam.posts) | post_path: Post5 JSON file, work_offset?: 1-6, origin?: [x,y,z] \
             (the design point that is work zero; on a 4th axis the A axis is work X through it), bodies?: [names] (default every visible \
             body), fixtures?: [names] (checked against, never cut), tolerance?: mm (default 0.01; bodies are tessellated at half of it), \
             tools: [Tool JSON] | tool_library_path, operations: [{name, type: contour | pocket | parallel | roughing | waterline | chamfer {heights: {clearance, retract, top}, width, tip_offset?, side?} | indexed \
             {a_deg, operation} | rotary_finish | rotary_roughing | wrap, tool: number, spindle_rpm?, feeds?: {cut, plunge}, coolant?, \
             gouge_check?, …}]; unknown keys are refused); output: G-code path (required unless dry_run); preview?: HTML path (the part \
             and toolpaths, self-contained); dry_run?: plan and check, write nothing; allow_unsafe?: write even when a check fails → \
             operations: [{name, kind, tool, moves, cut_length, cut_minutes, max_tilt_deg, a_deg?, gouge, holder_collisions, warnings}], \
             program_lines, program_head, readback: {max_tip_deviation, max_axis_deviation_deg, allowed_tip, allowed_axis_deg, \
             cut_minutes, pass}, gouge: {worst, tolerance, hits, skipped, pass}, holder_collisions, passed, failures, dry_run, outputs: \
             {gcode?, preview?}, notes. Without dry_run or allow_unsafe a failed check refuses and nothing is written",
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
];

/// Largest job, post or tool library file read (bytes).
const MAX_JSON_BYTES: usize = 16 << 20;
/// Largest G-code file `cam.check` reads (bytes).
const MAX_GCODE_BYTES: usize = 256 << 20;
/// Most operations in a job.
const MAX_OPERATIONS: usize = 200;
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
    let machine = match &job.machine {
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

/// The part and the collision surface (part and fixtures) in work coordinates.
struct Surfaces {
    part: Surface,
    all: Surface,
    notes: Vec<String>,
}

/// Every body of `names` tessellated within `chord` into one mesh, moved by `-origin`.
fn merged_mesh(cmd: &str, st: &solidcraft_doc::ModelState, names: &[String], origin: Vec3, chord: f64, notes: &mut Vec<String>) -> Result<Mesh> {
    let mut out = Mesh::default();
    for name in names {
        let Some(b) = st.bodies.iter().find(|b| &b.name == name) else { return Err(bad(cmd, format!("no body `{name}`"))) };
        if b.body.is_mesh() {
            notes.push(format!("body `{name}` is a mesh: its own triangles are used, which may be coarser than the job's tolerance"));
        }
        let mesh = b.body.mesh(&solidcraft_kernel::MeshTolerance::chord(chord))?;
        let Ok(start) = u32::try_from(out.positions.len()) else { return Err(bad(cmd, "the part has too many triangles")) };
        let smooth = mesh.normals.len() == mesh.positions.len();
        out.positions.extend(mesh.positions.iter().map(|p| *p - origin));
        if smooth {
            out.normals.extend(mesh.normals.iter().copied());
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
    let chord = tol * 0.5;
    let mut notes = vec![format!(
        "bodies tessellated from their exact surfaces at a chord tolerance of {} mm (half the job's tolerance), in work coordinates \
         (the design less origin [{}, {}, {}])",
        num(chord),
        num(origin.x),
        num(origin.y),
        num(origin.z)
    )];
    let part_mesh = merged_mesh(cmd, &st, &part, origin, chord, &mut notes)?;
    let part_surface = Surface::from_mesh(&part_mesh).map_err(|e| cam_err(cmd, "the part", e))?;
    if part_surface.is_empty() {
        return Err(bad(cmd, "the part's bodies have no surface to machine"));
    }
    let all = if fixtures.is_empty() {
        part_surface.clone()
    } else {
        let mut both = part_mesh;
        let fix = merged_mesh(cmd, &st, &fixtures, origin, chord, &mut notes)?;
        let Ok(start) = u32::try_from(both.positions.len()) else { return Err(bad(cmd, "the part has too many triangles")) };
        both.positions.extend(fix.positions);
        both.normals.extend(fix.normals);
        for t in fix.triangles {
            let [Some(a), Some(b), Some(c)] = t.map(|i| i.checked_add(start)) else { return Err(bad(cmd, "the part has too many triangles")) };
            both.triangles.push([a, b, c]);
        }
        Surface::from_mesh(&both).map_err(|e| cam_err(cmd, "the fixtures", e))?
    };
    Ok(Surfaces { part: part_surface, all, notes })
}

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
fn sketch_regions(cmd: &str, s: &Session, key: &str, profiles: Option<&[usize]>, origin: Vec3, frame: &Frame) -> Result<Vec<Region2>> {
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
    let normal = frame.dir_to_local(plane.x.cross(plane.y));
    if !(normal.z > 1.0 - 1e-6) {
        return Err(bad(
            cmd,
            format!(
                "sketch `{key}` is not on a plane square to the operation's tool axis and facing it (draw it on a plane facing +Z of the operation)"
            ),
        ));
    }
    let lx = frame.dir_to_local(plane.x);
    let turn = lx.y.atan2(lx.x);
    let map = |q: Vec2| {
        let l = frame.to_local(plane.origin + plane.x * q.x + plane.y * q.y - origin);
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

/// What every operation needs, resolved.
struct Resolved {
    tool: Tool,
    rpm: f64,
    feeds: Feeds,
}

/// Everything an operation is planned with.
struct Ctx<'a> {
    cmd: &'a str,
    s: &'a Session,
    machine: &'a Machine,
    tools: &'a [Tool],
    part: &'a Surface,
    origin: Vec3,
    tol: f64,
}

impl Ctx<'_> {
    fn bad(&self, op: &str, why: impl std::fmt::Display) -> EngineError {
        bad(self.cmd, format!("operation `{op}`: {why}"))
    }

    fn resolve(&self, c: &Common<'_>) -> Result<Resolved> {
        let Some(tool) = self.tools.iter().find(|t| t.number == c.tool) else {
            return Err(self.bad(c.name, format!("no tool number {}", c.tool)));
        };
        let rpm =
            c.spindle_rpm.or(tool.spindle_rpm).ok_or_else(|| self.bad(c.name, format!("give `spindle_rpm` (tool {} has none)", tool.number)))?;
        if !(rpm > 0.0 && rpm <= 200_000.0) {
            return Err(self.bad(c.name, "the spindle speed must be above 0 and at most 200000 rev/min"));
        }
        let feeds = match (c.feeds, tool.feed, tool.plunge_feed) {
            (Some(f), ..) => Feeds { cut: f.cut, plunge: f.plunge },
            (None, Some(cut), Some(plunge)) => Feeds { cut, plunge },
            (None, Some(cut), None) => Feeds { cut, plunge: cut },
            _ => return Err(self.bad(c.name, format!("give `feeds` (tool {} has no feed)", tool.number))),
        };
        feeds.validate().map_err(|e| self.bad(c.name, e))?;
        Ok(Resolved { tool: tool.clone(), rpm, feeds })
    }

    fn tolerance(&self, op: &str, t: Option<f64>) -> Result<f64> {
        match t {
            None => Ok(self.tol),
            Some(t) if t.is_finite() && (TOLERANCE_RANGE.0..=TOLERANCE_RANGE.1).contains(&t) => Ok(t),
            Some(_) => Err(self.bad(op, "the tolerance must be from 0.0001 to 1 mm")),
        }
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
            (Some(k), None) => sketch_regions(self.cmd, self.s, &k.0, profiles, self.origin, frame).map_err(|e| match e {
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

/// A 3-axis operation planned in `frame` on `surface` (the part in that frame).
fn plan3(ctx: &Ctx<'_>, op: &Op3Spec, frame: &Frame, surface: &Surface) -> Result<(Toolpath, Resolved, &'static str)> {
    match op {
        Op3Spec::Contour(o) => {
            let c = o.common();
            let r = ctx.resolve(&c)?;
            let regions = ctx.regions(c.name, o.sketch.as_ref(), o.profiles.as_deref(), o.polygon.as_deref(), frame)?;
            let cut = Cut { heights: heights(&o.heights), feeds: r.feeds, stepdown: or_tool(o.stepdown, r.tool.stepdown, r.tool.diameter * 0.5) };
            let params = ContourParams {
                side: match o.side.unwrap_or(SideSpec::Outside) {
                    SideSpec::Outside => Side::Outside,
                    SideSpec::Inside => Side::Inside,
                    SideSpec::On => Side::On,
                },
                climb: o.climb.unwrap_or(true),
                stock_to_leave: o.stock_to_leave.unwrap_or(0.0),
                tolerance: ctx.tolerance(c.name, o.tolerance)?,
                ramp_angle: ctx.degrees(c.name, o.ramp_angle_deg.unwrap_or(0.0), "`ramp_angle_deg`")?,
            };
            let path = contour(&regions, &r.tool, &params, &cut).map_err(|e| ctx.bad(c.name, e))?;
            Ok((path, r, "contour"))
        }
        Op3Spec::Pocket(o) => {
            let c = o.common();
            let r = ctx.resolve(&c)?;
            let regions = ctx.regions(c.name, o.sketch.as_ref(), o.profiles.as_deref(), o.polygon.as_deref(), frame)?;
            let cut = Cut { heights: heights(&o.heights), feeds: r.feeds, stepdown: or_tool(o.stepdown, r.tool.stepdown, r.tool.diameter * 0.5) };
            let params = PocketParams {
                stepover: or_tool(o.stepover, r.tool.stepover, r.tool.diameter * 0.4),
                climb: o.climb.unwrap_or(true),
                stock_to_leave: o.stock_to_leave.unwrap_or(0.0),
                tolerance: ctx.tolerance(c.name, o.tolerance)?,
                ramp_angle: ctx.degrees(c.name, o.ramp_angle_deg.unwrap_or(0.0), "`ramp_angle_deg`")?,
            };
            let path = pocket(&regions, &r.tool, &params, &cut).map_err(|e| ctx.bad(c.name, e))?;
            Ok((path, r, "pocket"))
        }
        Op3Spec::Parallel(o) => {
            let c = o.common();
            let r = ctx.resolve(&c)?;
            let params = ParallelParams {
                heights: heights(&o.heights),
                feeds: r.feeds,
                stepover: or_tool(o.stepover, r.tool.stepover, r.tool.diameter * 0.1),
                angle: ctx.degrees(c.name, o.angle_deg.unwrap_or(0.0), "`angle_deg`")?,
                boundary: ctx.boundary(c.name, o.boundary.as_ref())?,
                stock_to_leave: o.stock_to_leave.unwrap_or(0.0),
                step: o.step.unwrap_or(r.tool.diameter * 0.25),
                tolerance: ctx.tolerance(c.name, o.tolerance)?,
                zigzag: o.zigzag.unwrap_or(true),
            };
            let path = parallel(surface, &r.tool, &params).map_err(|e| ctx.bad(c.name, e))?;
            Ok((path, r, "parallel"))
        }
        Op3Spec::Roughing(o) => {
            let c = o.common();
            let r = ctx.resolve(&c)?;
            let params = RoughingParams {
                heights: heights(&o.heights),
                feeds: r.feeds,
                stepdown: or_tool(o.stepdown, r.tool.stepdown, r.tool.diameter * 0.5),
                stepover: or_tool(o.stepover, r.tool.stepover, r.tool.diameter * 0.4),
                angle: ctx.degrees(c.name, o.angle_deg.unwrap_or(0.0), "`angle_deg`")?,
                boundary: ctx.boundary(c.name, o.boundary.as_ref())?,
                stock_to_leave: o.stock_to_leave.unwrap_or(0.25),
                step: o.step.unwrap_or(r.tool.diameter * 0.25),
                tolerance: ctx.tolerance(c.name, o.tolerance)?,
                zigzag: o.zigzag.unwrap_or(true),
            };
            let path = roughing(surface, &r.tool, &params).map_err(|e| ctx.bad(c.name, e))?;
            Ok((path, r, "roughing"))
        }
        Op3Spec::Chamfer(o) => {
            let (path, r) = chamfer::plan(ctx, o, frame)?;
            Ok((path, r, "chamfer"))
        }
        Op3Spec::Waterline(o) => {
            let c = o.common();
            let r = ctx.resolve(&c)?;
            let params = WaterlineParams {
                heights: heights(&o.heights),
                feeds: r.feeds,
                stepdown: or_tool(o.stepdown, r.tool.stepdown, r.tool.diameter * 0.25),
                boundary: ctx.boundary(c.name, o.boundary.as_ref())?,
                stock_to_leave: o.stock_to_leave.unwrap_or(0.0),
                step: o.step.unwrap_or(r.tool.diameter * 0.25),
                tolerance: ctx.tolerance(c.name, o.tolerance)?,
                climb: o.climb.unwrap_or(true),
            };
            let path = waterline(surface, &r.tool, &params).map_err(|e| ctx.bad(c.name, e))?;
            Ok((path, r, "waterline"))
        }
    }
}

fn common3(op: &Op3Spec) -> Common<'_> {
    match op {
        Op3Spec::Contour(o) => o.common(),
        Op3Spec::Pocket(o) => o.common(),
        Op3Spec::Parallel(o) => o.common(),
        Op3Spec::Roughing(o) => o.common(),
        Op3Spec::Waterline(o) => o.common(),
        Op3Spec::Chamfer(o) => o.common(),
    }
}

/// One operation planned: what the program holds, its poses in work coordinates (for the
/// checks and the preview), and its report.
struct Planned {
    op: MachineOperation,
    poses: Toolpath5,
    tool: Tool,
    gouge_check: bool,
    report: OperationReport,
    axes: String,
}

fn pattern(ctx: &Ctx<'_>, op: &str, p: PatternSpec) -> Result<RotaryPattern> {
    Ok(match p {
        PatternSpec::Lines { angle_step_deg, zigzag } => {
            RotaryPattern::Lines { angle_step: ctx.degrees(op, angle_step_deg, "`angle_step_deg`")?, zigzag: zigzag.unwrap_or(true) }
        }
        PatternSpec::Helix { pitch } => RotaryPattern::Helix { pitch },
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

fn plan(ctx: &Ctx<'_>, spec: &OperationSpec, arc_tol: f64) -> Result<Planned> {
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
        let (path, r, kind) = plan3(ctx, op3, &frame, surface)?;
        let stats = path.stats();
        let io = IndexedOperation { name: c.name.into(), tool: r.tool.clone(), spindle_rpm: r.rpm, coolant: c.coolant, rotary, path };
        let poses = io.poses(machine, arc_tol).map_err(|e| ctx.bad(c.name, e))?;
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
        let axes = match a_deg {
            Some(a) => format!("3+1 A{}°", num(a)),
            None => "3-axis".into(),
        };
        Ok(Planned { op: MachineOperation::Indexed(io), poses, tool: r.tool, gouge_check: c.gouge_check, report, axes })
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
        let op = Operation5 { name: c.name.into(), tool: r.tool.clone(), spindle_rpm: r.rpm, coolant: c.coolant, path: path.clone() };
        Planned { op: MachineOperation::Continuous(op), poses: path, tool: r.tool, gouge_check: c.gouge_check, report, axes: "4-axis".into() }
    };
    match spec {
        OperationSpec::Contour(o) => indexed(&Op3Spec::Contour(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Pocket(o) => indexed(&Op3Spec::Pocket(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Parallel(o) => indexed(&Op3Spec::Parallel(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Roughing(o) => indexed(&Op3Spec::Roughing(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Waterline(o) => indexed(&Op3Spec::Waterline(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Chamfer(o) => indexed(&Op3Spec::Chamfer(o.clone()), vec![0.0; rotaries], None),
        OperationSpec::Indexed(o) => {
            let name = common3(&o.operation).name;
            if rotaries != 1 {
                return Err(ctx.bad(name, "an indexed operation's `a_deg` needs a machine with one rotary axis (`machine: \"4axis_a\"`)"));
            }
            let a = ctx.degrees(name, o.a_deg, "`a_deg`")?;
            indexed(&o.operation, vec![a], Some(o.a_deg))
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
                stock_radius: o.stock_radius,
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
    }
}

/// `cam.job`.
fn job(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job";
    let params: CamJobParams = parse(cmd, p, "")?;
    let dry_run = params.dry_run.unwrap_or(false);
    let allow_unsafe = params.allow_unsafe.unwrap_or(false);

    // Every path is checked before anything is computed or written.
    let output = match (&params.output, dry_run) {
        (Some(o), _) => Some(o.clone()),
        (None, true) => None,
        (None, false) => return Err(bad(cmd, "give `output` (the G-code file to write) or `dry_run: true`")),
    };
    if !dry_run {
        for path in output.iter().chain(&params.preview) {
            if path.trim().is_empty() {
                return Err(bad(cmd, "a file path must not be empty"));
            }
            solidcraft_io::vfs::check(path).map_err(|e| EngineError::Other(e.to_string()))?;
        }
        if output.is_some() && output == params.preview {
            return Err(bad(cmd, "`output` and `preview` must be different files"));
        }
    }
    let (job, base): (CamJob, Option<PathBuf>) = match (params.job, &params.job_path) {
        (Some(_), Some(_)) => return Err(bad(cmd, "give `job` or `job_path`, not both")),
        (None, None) => return Err(bad(cmd, "give the `job` (an object) or a `job_path` (a .json file)")),
        (Some(j), None) => (j, None),
        (None, Some(path)) => {
            let v = read_json(cmd, path, "job_path")?;
            (parse(cmd, &v, &format!("job_path {path}"))?, Path::new(path).parent().map(Path::to_path_buf))
        }
    };
    let base = base.as_deref();
    if job.operations.is_empty() || job.operations.len() > MAX_OPERATIONS {
        return Err(bad(cmd, format!("a job has 1 to {MAX_OPERATIONS} operations")));
    }
    let tol = job.tolerance.unwrap_or(0.01);
    if !(tol.is_finite() && (TOLERANCE_RANGE.0..=TOLERANCE_RANGE.1).contains(&tol)) {
        return Err(bad(cmd, "the job's `tolerance` must be from 0.0001 to 1 mm"));
    }
    let origin = job.origin.unwrap_or([0.0; 3]);
    finite_point(&origin, "`origin`").map_err(|e| bad(cmd, e))?;
    let origin = v3(origin);
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
    let machine = machine_of(cmd, &job)?;
    let (post_name, post) = post_of(cmd, &job, base)?;
    let tools = tools_of(cmd, &job, base)?;
    let surfaces = surfaces_of(cmd, s, &job, origin, tol)?;
    let mut notes = surfaces.notes.clone();
    if job.work_offset.is_some() && !post.base.writes_work_offset() {
        notes.push(format!("post `{post_name}` writes no work offset: the program runs in whichever the controller has active"));
    }

    let ctx = Ctx { cmd, s, machine: &machine, tools: &tools, part: &surfaces.part, origin, tol };
    let mut planned: Vec<Planned> = Vec::with_capacity(job.operations.len());
    for spec in &job.operations {
        planned.push(plan(&ctx, spec, post.base.arc_tolerance)?);
    }

    let program = MachineProgram { name: name.clone(), operations: planned.iter().map(|p| p.op.clone()).collect(), work_offset: job.work_offset };
    let text = emit_program(&program, &post, &machine).map_err(|e| cam_err(cmd, "writing the program", e))?;
    let rb = readback_program(&text, &program, &post, &machine).map_err(|e| cam_err(cmd, "reading the program back", e))?;

    // Gouges and holder collisions, along every cutting move.
    let mut failures: Vec<String> = Vec::new();
    let mut gouge = GougeReport { tolerance: tol, ..GougeReport::default() };
    let mut holder_total: Option<usize> = None;
    for p in &mut planned {
        let step = (p.tool.radius() * 0.5).clamp(0.1, 2.0);
        let op = p.report.name.clone();
        if p.gouge_check {
            let cutter = p.tool.cutter().map_err(|e| cam_err(cmd, &format!("operation `{op}`"), e))?;
            let hits = gouges(&surfaces.all, &cutter, &p.poses, tol, step).map_err(|e| cam_err(cmd, &format!("operation `{op}`: gouge check"), e))?;
            let worst = hits.iter().map(|h| h.distance).fold(0.0, f64::max);
            p.report.gouge = worst;
            gouge.worst = gouge.worst.max(worst);
            gouge.hits += hits.len();
            if !hits.is_empty() {
                failures.push(format!("gouge: operation `{op}` cuts up to {} mm into the part or a fixture along {} moves", num(worst), hits.len()));
            }
        } else {
            gouge.skipped.push(op.clone());
            p.report.warnings.push("not checked for gouges (`gouge_check: false`)".into());
        }
        if !p.tool.envelopes().is_empty() {
            let hits = holder_collisions(&surfaces.all, &p.tool, &p.poses, 0.0, step)
                .map_err(|e| cam_err(cmd, &format!("operation `{op}`: holder check"), e))?;
            p.report.holder_collisions = hits.len();
            *holder_total.get_or_insert(0) += hits.len();
            if !hits.is_empty() {
                failures
                    .push(format!("holder: operation `{op}`: the tool's shank or holder touches the part or a fixture along {} moves", hits.len()));
            }
        }
    }
    gouge.pass = gouge.hits == 0;
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
    }
    let passed = failures.is_empty();
    if !passed && !dry_run && !allow_unsafe {
        return Err(bad(
            cmd,
            format!(
                "nothing was written: {}. Fix the job, run it with `dry_run: true` to see the report, or pass `allow_unsafe: true`",
                failures.join("; ")
            ),
        ));
    }

    let mut outputs = Outputs::default();
    if !dry_run {
        if let Some(path) = &output {
            if let Some(ext) = Path::new(path).extension().map(|e| e.to_string_lossy().to_ascii_lowercase())
                && ext != post.base.extension.to_ascii_lowercase()
            {
                notes.push(format!("post `{post_name}` writes .{} files; this one is .{ext}", post.base.extension));
            }
            solidcraft_io::vfs::write(path, text.as_bytes()).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
            outputs.gcode = Some(path.clone());
        }
        if let Some(path) = &params.preview {
            let page = preview_page(&name, &surfaces.all, &planned, &text);
            solidcraft_io::vfs::write(path, page.as_bytes()).map_err(|e| {
                let written = outputs.gcode.as_deref().map_or(String::new(), |g| format!(" (the G-code was written to {g})"));
                EngineError::Other(format!("{path}: {e}{written}"))
            })?;
            outputs.preview = Some(path.clone());
        }
    }
    let reply = CamJobReply {
        name,
        machine: machine.name.clone(),
        post: post_name,
        operations: planned.into_iter().map(|p| p.report).collect(),
        program_lines: text.lines().count(),
        program_head: text.lines().take(HEAD_LINES).map(str::to_string).collect(),
        readback,
        gouge,
        holder_collisions: holder_total,
        passed,
        failures,
        dry_run,
        outputs,
        notes,
        param_warnings: Vec::new(),
    };
    serde_json::to_value(reply).map_err(|e| EngineError::Other(e.to_string()))
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
            JointWords::Arc { linear, center, ccw, rotary, f, mm_per_unit, .. } => {
                reply.arcs += 1;
                if let Some((from, _)) = &last {
                    let r = Vec2::new(from.x, from.y).dist(*center);
                    let len = (r * arc_sweep(*from, *linear, *center, *ccw)).hypot(linear.z - from.z);
                    reply.feed_length += len;
                    reply.cut_minutes += len / (f * mm_per_unit);
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

#[path = "cam_chamfer.rs"]
mod chamfer;

#[cfg(test)]
#[path = "cam_tests.rs"]
mod tests;
