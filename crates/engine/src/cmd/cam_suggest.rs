//! `cam.suggest`: a whole CAM job proposed from a part's machinable features (`cam.features`),
//! returned for `cam.job` to plan and check, never run on its own. The operations, in order:
//!
//! 1. face the flat top (`stock_top` mm of material assumed above it) with a `face` operation:
//!    a face mill, else the largest flat end mill;
//! 2. with curved areas, 3D adaptive roughing of the whole part (largest flat end mill, 0.25 mm
//!    left);
//! 3. every pocket cleared by adaptive clearing on the feature with the largest flat end mill
//!    that fits it, 0.2 mm left on the walls, islands below the rim in bands of their own;
//! 4. every hole a drill of its diameter matches drilled to its full diameter (pecking when
//!    deeper than three diameters), the others milled as round pockets with the largest flat end
//!    mill of at most 80% of its diameter (counterbores before the holes they lead into);
//! 5. every pocket's walls finished with a contour by the largest flat end mill whose radius is
//!    at most the pocket's smallest inside corner radius (the smallest that fits when a corner is
//!    sharp);
//! 6. curved areas and rounds finished with a parallel 3D finish by the largest ball end mill
//!    whose radius is at most their smallest radius (else the smallest ball, else a bull nose or
//!    flat end mill);
//! 7. on a 4th axis, turned diameters finished with a rotary finish;
//! 8. the outer profiles, with the largest flat end mill that fits their notches, flutes long
//!    enough for the depth preferred;
//! 9. modelled chamfers along a profile, a pocket's rim or a hole cut by the chamfer operation
//!    with a chamfer mill of their angle.
//!
//! Features on other sides of a 4th-axis part are indexed to their A angle. The machine, post
//! and tools come from the same profile `cam.job` reads, which the proposed job names. Tools are
//! taken from the library only when they have a spindle speed and a feed, and of the tools that
//! fit, those made for the part's `material` come first, then carbide, then HSS. Each tool is
//! tried with the operation itself (`solidcraft_cam::pocket` and `contour` on the feature's exact
//! outline) before it is picked, so a pick is one the operation accepts. Every pick, assumption
//! and skipped feature is in the reply.

use std::path::Path;

use serde_json::{Map, Value, json};
use solidcore_record::{Loop2, Region2, Vec2};
use solidcraft_cam::kinematics::Machine;
use solidcraft_cam::{ContourParams, Cut, Feeds, Heights, PocketParams, Side, Tool, ToolKind, contour, pocket};

use super::features::{FeatureSet, Geo, Kind, Lazy, part_names, recognize};
use super::{beside, crate_value, finite_point, machine_of, num, parse, read_json, tool_sample, v3};
use crate::cmd::CommandSpec;
use crate::params::bad;
use crate::schema::cam::{CamJob, MachineSpec};
use crate::schema::cam_features::{AxisSpec, CamSuggestParams, CamSuggestReply, MaterialSpec, SuggestedOperation, ToolChoice};
use crate::schema::{TypedContract, output_schema_for};
use crate::{EngineError, Result, Session};

const SUGGEST_CONTRACT: TypedContract<CamSuggestParams> = TypedContract::new();

pub static COMMANDS: &[CommandSpec] = &[CommandSpec::new("cam.suggest", "Suggest CAM Job", crate::checked!("cam.suggest", suggest))
    .noundo()
    .contract(&SUGGEST_CONTRACT)
    .output(output_schema_for::<CamSuggestReply>)
    .params(
        "profile_path: the machine profile cam.job reads (or a tool library array) | tools: [Tool JSON] | tool_library_path; bodies?; \
         origin?: [x,y,z]; axis?: \"z\" | \"a\" (default: a when the machine has an A axis); tolerance?; post?: built-in (default the \
         profile's, else linuxcnc_4axis); stock_top?: mm faced off the top (default 0.5); material?: aluminium | steel | stainless | \
         brass | plastic | wood (tools made for it first; carbide before HSS); check?: bool (also run cam.job as a dry run) → job (a \
         complete cam.job job naming the profile, operations naming features: face, adaptive clearing, drill, pockets, wall finish, \
         3D finish, turned, profile, chamfer), operations: [{name, type, \
         tool, features, why}], tools: [{number, kind, diameter, used_for}], assumptions, check (what to verify), skipped (features left \
         and why), dry_run? (the cam.job report). Nothing is run or written: pass `job` to cam.job {job, dry_run: true}, then {job, output}",
    )];

/// Pockets leave this on their walls for the finishing contour (mm).
const WALL_STOCK: f64 = 0.2;
/// 3D roughing leaves this (mm).
const ROUGH_STOCK: f64 = 0.25;
/// The largest share of a hole's diameter the tool milling it may have.
const HOLE_TOOL_SHARE: f64 = 0.8;
/// Most material faced off the top (mm).
const MAX_STOCK_TOP: f64 = 5.0;
/// Ramp into material, degrees.
const RAMP_DEG: f64 = 3.0;

/// A usable tool and its JSON as given.
#[derive(Clone)]
struct Pick {
    tool: Tool,
    json: Map<String, Value>,
}

/// The entry helix of adaptive clearing, as a share of the tool's diameter: room for it is
/// found where a larger one (the operations' default, 80%) is refused.
const HELIX_SHARE: f64 = 0.5;
/// Holes this close to a drill's diameter are drilled with it (mm).
const DRILL_MATCH: f64 = 0.05;
/// Holes deeper than this many diameters are drilled in pecks of a diameter.
const PECK_DEPTHS: f64 = 3.0;
/// How far a chamfer mill's half angle may be from a modelled chamfer's for it to cut it
/// (degrees).
const CHAMFER_ANGLE: f64 = 1.0;

/// How well `t` suits `material`: 0 made for it, 1 carbide (or any tool for a soft material),
/// 2 HSS, from what its description says.
fn suits(t: &Tool, material: Option<MaterialSpec>) -> u8 {
    let d = format!(" {} ", t.description.to_ascii_lowercase());
    let hss = d.contains("hss") || d.contains("high speed steel");
    let names = |words: &[&str]| words.iter().any(|w| d.contains(w));
    let made_for = match material {
        Some(MaterialSpec::Aluminium) => names(&[" al ", " al-", "(al)", "alu", "dlc", "zrn"]),
        Some(MaterialSpec::Steel) => names(&["steel", "tialn", "altin", "hrc6"]),
        Some(MaterialSpec::Stainless) => names(&["stainless", "inox", "tialn", "altin"]),
        Some(MaterialSpec::Brass) => names(&["brass", " al ", "uncoated"]),
        Some(MaterialSpec::Plastic) => names(&["plastic", "acrylic", "o flute", "single flute"]),
        Some(MaterialSpec::Wood) => names(&["wood", "plastic"]),
        None => false,
    };
    let soft = matches!(material, Some(MaterialSpec::Plastic | MaterialSpec::Wood));
    match (made_for, hss && !soft) {
        (true, false) => 0,
        (_, true) => 2,
        (false, false) => 1,
    }
}

/// What a profile file or the call gives besides the tools.
#[derive(Default)]
struct Profile {
    /// A machine profile as `cam.job` reads it (the proposed job names it), and the job it
    /// makes on its own.
    path: Option<String>,
    job: Option<CamJob>,
    tools: Vec<Value>,
    machine: Option<Value>,
    post: Option<String>,
    post_path: Option<String>,
    a_reversed: Option<Value>,
    a_limits_deg: Option<Value>,
    work_offset: Option<Value>,
}

/// Keys only a profile as `cam.job` reads it has: such a file is read the way `cam.job` reads it.
const JOB_PROFILE_KEYS: &[&str] =
    &["default_feeds", "default_heights", "description", "name", "tool_library_paths", "tolerance", "fixtures", "origin"];

const PROFILE_KEYS: &[&str] =
    &["a_limits_deg", "a_reversed", "machine", "machine_path", "post", "post_path", "tool_library_path", "tools", "work_offset"];

fn load_profile(cmd: &str, p: &CamSuggestParams) -> Result<Profile> {
    let given = [p.profile_path.is_some(), p.tools.is_some(), p.tool_library_path.is_some()].iter().filter(|x| **x).count();
    if given != 1 {
        return Err(bad(
            cmd,
            "give the tools to choose from, one way: `profile_path` (a machine profile or tool library .json), `tools` or `tool_library_path`",
        ));
    }
    let library = |path: &str| -> Result<Vec<Value>> {
        match read_json(cmd, path, "tool_library_path")? {
            Value::Array(a) => Ok(a),
            _ => Err(bad(cmd, format!("{path}: a tool library holds a JSON array of tools"))),
        }
    };
    if let Some(t) = &p.tools {
        return Ok(Profile { tools: t.iter().cloned().map(Value::Object).collect(), ..Profile::default() });
    }
    if let Some(path) = &p.tool_library_path {
        return Ok(Profile { tools: library(path)?, ..Profile::default() });
    }
    let Some(path) = &p.profile_path else { return Ok(Profile::default()) };
    let base = Path::new(path).parent().map(Path::to_path_buf);
    match read_json(cmd, path, "profile_path")? {
        Value::Array(a) => Ok(Profile { tools: a, ..Profile::default() }),
        // A machine profile as cam.job reads it: its machine, post and tool libraries.
        Value::Object(o) if o.keys().any(|k| JOB_PROFILE_KEYS.contains(&k.as_str())) => {
            let probe = json!({"job": {"profile_path": path, "operations": []}});
            let loaded = super::agent::load_job_of(cmd, &probe, true, None)?;
            let tools = loaded.job.tools.clone().unwrap_or_default().into_iter().map(Value::Object).collect();
            Ok(Profile { path: Some(path.clone()), job: Some(loaded.job), tools, ..Profile::default() })
        }
        Value::Object(o) => {
            if let Some(k) = o.keys().find(|k| !PROFILE_KEYS.contains(&k.as_str())) {
                return Err(bad(cmd, format!("profile_path {path}: unknown key `{k}` (known: {})", PROFILE_KEYS.join(", "))));
            }
            let tools = match (o.get("tools"), o.get("tool_library_path")) {
                (Some(Value::Array(a)), None) => a.clone(),
                (None, Some(Value::String(lib))) => library(&beside(base.as_deref(), lib))?,
                _ => return Err(bad(cmd, format!("profile_path {path}: give `tools` (an array) or `tool_library_path` (a file), one of them"))),
            };
            let text = |k: &str| -> Result<Option<String>> {
                match o.get(k) {
                    None => Ok(None),
                    Some(Value::String(v)) => Ok(Some(v.clone())),
                    Some(_) => Err(bad(cmd, format!("profile_path {path}: `{k}` must be a string"))),
                }
            };
            let machine = match (o.get("machine"), o.get("machine_path")) {
                (Some(_), Some(_)) => return Err(bad(cmd, format!("profile_path {path}: give `machine` or `machine_path`, not both"))),
                (m, None) => m.cloned(),
                (None, Some(Value::String(mp))) => {
                    let file = beside(base.as_deref(), mp);
                    match read_json(cmd, &file, "machine_path")? {
                        m @ Value::Object(_) => Some(m),
                        _ => return Err(bad(cmd, format!("machine_path {file}: a machine file holds the CAM crate's `Machine` JSON object"))),
                    }
                }
                (None, Some(_)) => return Err(bad(cmd, format!("profile_path {path}: `machine_path` must be a string"))),
            };
            Ok(Profile {
                tools,
                machine,
                post: text("post")?,
                post_path: text("post_path")?.map(|pp| beside(base.as_deref(), &pp)),
                a_reversed: o.get("a_reversed").cloned(),
                a_limits_deg: o.get("a_limits_deg").cloned(),
                work_offset: o.get("work_offset").cloned(),
                ..Profile::default()
            })
        }
        _ => Err(bad(cmd, format!("profile_path {path}: a machine profile is a JSON object or an array of tools"))),
    }
}

/// The tools that can be used, and why the others cannot.
fn usable(cmd: &str, raw: &[Value], skipped: &mut Vec<String>) -> Vec<Pick> {
    let sample = tool_sample();
    let mut out: Vec<Pick> = Vec::new();
    for (i, v) in raw.iter().enumerate().take(super::MAX_TOOLS) {
        let what = format!("tool {}", v.get("number").map_or_else(|| format!("[{i}]"), Value::to_string));
        let tool: Tool = match crate_value(cmd, v.clone(), &sample, &what) {
            Ok(t) => t,
            Err(e) => {
                skipped.push(format!("{what} not used: {}", message(e)));
                continue;
            }
        };
        if let Err(e) = tool.validate() {
            skipped.push(format!("{what} not used: {e}"));
            continue;
        }
        if tool.spindle_rpm.is_none() || tool.feed.is_none() {
            skipped.push(format!("{what} not used: it has no `spindle_rpm` or `feed`"));
            continue;
        }
        if out.iter().any(|p| p.tool.number == tool.number) {
            skipped.push(format!("{what} not used: another tool has its number"));
            continue;
        }
        let Value::Object(json) = v.clone() else { continue };
        out.push(Pick { tool, json });
    }
    out
}

fn message(e: EngineError) -> String {
    match e {
        EngineError::BadParams { msg, .. } => msg,
        other => other.to_string(),
    }
}

/// The smallest of `tools` among those that suit `material` best ([`suits`]): never an HSS tool
/// where a carbide one would do.
fn smallest<'a>(tools: impl Iterator<Item = &'a Pick>, material: Option<MaterialSpec>) -> Option<&'a Pick> {
    tools.min_by(|a, b| {
        suits(&a.tool, material)
            .cmp(&suits(&b.tool, material))
            .then(a.tool.diameter.total_cmp(&b.tool.diameter))
            .then(a.tool.number.cmp(&b.tool.number))
    })
}

/// `tools` of `kind`, those that suit `material` best first ([`suits`]), then the largest (the
/// lower number first among equals).
fn largest(tools: &[Pick], kind: ToolKind, material: Option<MaterialSpec>) -> Vec<&Pick> {
    let mut v: Vec<&Pick> = tools.iter().filter(|p| p.tool.kind == kind).collect();
    v.sort_by(|a, b| {
        suits(&a.tool, material)
            .cmp(&suits(&b.tool, material))
            .then(b.tool.diameter.total_cmp(&a.tool.diameter))
            .then(a.tool.number.cmp(&b.tool.number))
    });
    v
}

fn feeds(t: &Tool) -> Feeds {
    let cut = t.feed.unwrap_or(100.0);
    Feeds { cut, plunge: t.plunge_feed.unwrap_or(cut) }
}

/// Whether a pocket of `regions` from `top` to `bottom` takes `t`.
fn pocket_takes(t: &Tool, regions: &[Region2], top: f64, bottom: f64, leave: f64, tol: f64) -> bool {
    let cut = Cut { heights: Heights { clearance: top + 10.0, retract: top + 3.0, top, bottom }, feeds: feeds(t), stepdown: (top - bottom).max(0.1) };
    let params = PocketParams { stepover: t.diameter * 0.4, climb: true, stock_to_leave: leave, tolerance: tol, ramp_angle: 0.0 };
    pocket(regions, t, &params, &cut).is_ok_and(|p| !p.moves.is_empty())
}

/// Whether a contour of `regions` on `side` takes `t`.
fn contour_takes(t: &Tool, regions: &[Region2], side: Side, top: f64, bottom: f64, tol: f64) -> bool {
    let cut = Cut { heights: Heights { clearance: top + 10.0, retract: top + 3.0, top, bottom }, feeds: feeds(t), stepdown: (top - bottom).max(0.1) };
    let params = ContourParams { side, climb: true, stock_to_leave: 0.0, tolerance: tol, ramp_angle: 0.0 };
    contour(regions, t, &params, &cut).is_ok_and(|p| !p.moves.is_empty())
}

/// Whether adaptive clearing with `t` has room in `regions`: a tool's diameter clear between the
/// walls its centre keeps off (its radius and the stock left), for its entry and its turns.
fn adaptive_room(t: &Tool, regions: &[Region2], tol: f64) -> bool {
    solidcraft_cam::offset_regions(regions, t.radius() + WALL_STOCK + t.radius(), tol).is_ok_and(|l| !l.is_empty())
}

/// The regions a 2D operation on `g` cuts down to `bottom` (islands below it left out).
fn regions_of(g: &Geo, bottom: f64, tol: f64) -> Vec<Region2> {
    let outer = match (g.kind, &g.outer, g.hole) {
        (Kind::Hole, _, Some((c, d, _))) => Loop2::circle(c, d * 0.5),
        (_, Some(o), _) => o.clone(),
        _ => return Vec::new(),
    };
    let holes = g.islands.iter().filter(|(_, top)| *top > bottom + tol).map(|(l, _)| l.clone()).collect();
    vec![Region2 { outer, holes }]
}

/// The stepdown of a roughing pass with `t`: half its diameter, at most its own.
fn rough_step(t: &Tool) -> f64 {
    let half = t.diameter * 0.5;
    t.stepdown.map_or(half, |s| s.min(half))
}

/// The stepdown of a finishing pass with `t` over `depth`: its diameter, at most its own.
fn finish_step(t: &Tool, depth: f64) -> f64 {
    t.stepdown.unwrap_or(t.diameter).min(t.diameter).min(depth.max(0.1))
}

struct Plan {
    ops: Vec<Value>,
    report: Vec<SuggestedOperation>,
    used: Vec<(u32, String)>,
    assumptions: Vec<String>,
    check: Vec<String>,
    skipped: Vec<String>,
}

impl Plan {
    fn add(&mut self, op: Value, kind: &str, tool: &Tool, features: Vec<String>, why: String) {
        let name = op
            .get("name")
            .and_then(Value::as_str)
            .map(str::to_string)
            .or_else(|| op.get("operation").and_then(|o| o.get("name")).and_then(Value::as_str).map(str::to_string));
        let name = name.unwrap_or_default();
        self.used.push((tool.number, name.clone()));
        self.report.push(SuggestedOperation { name, kind: kind.into(), tool: tool.number, features, why });
        self.ops.push(op);
    }
}

/// `op` as the job runs it: wrapped in an `indexed` operation at `a_deg` unless that is 0.
fn at_side(op: Value, a_deg: Option<f64>) -> (Value, bool) {
    match a_deg {
        Some(a) if a.abs() > 1e-9 => (json!({"type": "indexed", "a_deg": a, "operation": op}), true),
        _ => (op, false),
    }
}

/// `cam.suggest`.
fn suggest(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.suggest";
    let params: CamSuggestParams = parse(cmd, p, "")?;
    let tol = params.tolerance.unwrap_or(0.01);
    if !(tol.is_finite() && (super::TOLERANCE_RANGE.0..=super::TOLERANCE_RANGE.1).contains(&tol)) {
        return Err(bad(cmd, "the `tolerance` must be from 0.0001 to 1 mm"));
    }
    let origin = params.origin.unwrap_or([0.0; 3]);
    finite_point(&origin, "`origin`").map_err(|e| bad(cmd, e))?;
    let stock_top = params.stock_top.unwrap_or(0.5);
    if !(stock_top.is_finite() && (0.0..=MAX_STOCK_TOP).contains(&stock_top)) {
        return Err(bad(cmd, format!("`stock_top` must be from 0 to {MAX_STOCK_TOP} mm")));
    }
    let profile = load_profile(cmd, &params)?;
    let mut skipped: Vec<String> = Vec::new();
    let tools = usable(cmd, &profile.tools, &mut skipped);
    let material = params.material;
    let flats = largest(&tools, ToolKind::FlatEndmill, material);
    if flats.is_empty() {
        return Err(bad(
            cmd,
            format!(
                "no flat end mill to mill with: give at least one tool with `kind: \"flat_endmill\"`, `spindle_rpm` and `feed`{}",
                if skipped.is_empty() { String::new() } else { format!(" ({})", skipped.join("; ")) }
            ),
        ));
    }

    // The machine: the profile's, else one for the axis asked for.
    let machine_value = match (&profile.machine, params.axis) {
        (Some(m), _) => m.clone(),
        (None, Some(AxisSpec::A)) => json!("4axis_a"),
        (None, _) => json!("3axis"),
    };
    let mut head = Map::new();
    let machine: Machine = match &profile.job {
        // A machine profile as cam.job reads it: the job names it, and takes its machine,
        // post, work offset and default heights and feeds from it.
        Some(pj) => {
            if let Some(path) = &profile.path {
                head.insert("profile_path".into(), json!(path));
            }
            if pj.machine.is_none() {
                head.insert("machine".into(), machine_value.clone());
            }
            let mut probe = pj.clone();
            if probe.machine.is_none() {
                probe.machine = Some(MachineSpec::Preset(machine_value.as_str().unwrap_or("3axis").to_string()));
            }
            machine_of(cmd, &probe)?
        }
        None => {
            head.insert("machine".into(), machine_value.clone());
            for (k, v) in [("a_reversed", &profile.a_reversed), ("a_limits_deg", &profile.a_limits_deg)] {
                if let Some(v) = v {
                    head.insert(k.into(), v.clone());
                }
            }
            let mut probe = Value::Object(head.clone());
            if let Some(o) = probe.as_object_mut() {
                o.insert("operations".into(), json!([]));
            }
            let probe_job: CamJob = parse(cmd, &probe, "the profile's machine")?;
            machine_of(cmd, &probe_job)?
        }
    };
    let job_axis = Lazy::axis_of(&machine);
    let axis = params.axis.unwrap_or(job_axis);
    if axis == AxisSpec::A && job_axis != AxisSpec::A {
        return Err(bad(
            cmd,
            "`axis: \"a\"` needs a machine with one rotary axis about X: give a profile whose `machine` is \"4axis_a\", or leave `axis` out",
        ));
    }
    let names = part_names(cmd, s, params.bodies.as_deref(), &[])?;
    let set = recognize(cmd, s, &names, v3(origin), job_axis, tol, &machine)?;

    let profile_post = profile.job.as_ref().is_some_and(|j| j.post.is_some() || j.post_path.is_some());
    let post = match (&params.post, &profile.post, &profile.post_path) {
        (Some(p), ..) => json!({"post": p}),
        _ if profile_post => json!({}),
        (None, Some(p), _) => json!({"post": p}),
        (None, None, Some(pp)) => json!({"post_path": pp}),
        (None, None, None) => json!({"post": "linuxcnc_4axis"}),
    };
    let mut plan = Plan { ops: Vec::new(), report: Vec::new(), used: Vec::new(), assumptions: Vec::new(), check: Vec::new(), skipped };
    operations(&mut plan, &set, &tools, &flats, axis, stock_top, tol, material);
    if plan.ops.is_empty() {
        return Err(bad(cmd, format!("nothing to machine was found that these tools can cut: {}", plan.skipped.join("; "))));
    }

    let mut job = Map::new();
    job.insert("name".into(), json!(s.doc.name.clone()));
    for (k, v) in head {
        job.insert(k, v);
    }
    if let Value::Object(p) = post {
        job.extend(p);
    }
    if let Some(w) = &profile.work_offset {
        job.insert("work_offset".into(), w.clone());
    }
    job.insert("origin".into(), json!(origin));
    job.insert("bodies".into(), json!(names));
    job.insert("tolerance".into(), json!(tol));
    let mut used: Vec<u32> = plan.used.iter().map(|(n, _)| *n).collect();
    used.sort_unstable();
    used.dedup();
    job.insert("tools".into(), Value::Array(tools.iter().filter(|t| used.contains(&t.tool.number)).map(|t| Value::Object(t.json.clone())).collect()));
    job.insert("operations".into(), Value::Array(plan.ops.clone()));
    let job = Value::Object(job);

    let top = set.setups.first().map_or(0.0, |su| su.top);
    plan.assumptions.splice(
        0..0,
        [
            format!(
                "work zero is the design point [{}, {}, {}]; the part's top is at Z {} in work coordinates{}",
                num(origin[0]),
                num(origin[1]),
                num(origin[2]),
                num(top),
                if job_axis == AxisSpec::A { " and the A axis runs along work X through work zero" } else { "" }
            ),
            format!("the stock is the part's own envelope with {} mm on top; nothing else is modelled", num(stock_top)),
            "no work holding is modelled: give the vise or clamps to the job as `fixtures` to have them checked".into(),
            "feeds and speeds are each tool's own, as the library gives them (cam.job takes each operation's own cutting data)".into(),
            "holes a drill of their diameter matches are drilled to their full diameter, the others milled as round pockets".into(),
        ],
    );
    plan.check.splice(
        0..0,
        [
            "run cam.job with this job and `dry_run: true`: `passed` must be true and `failures` empty".to_string(),
            "write a `preview` page with cam.job and look at every operation against the part".into(),
            "check each tool's stickout and holder against the deepest feature it cuts (the holder check runs only for tools with holder data)"
                .into(),
            "set the work offset on the machine at the work zero given".into(),
        ],
    );
    let mut tool_choices: Vec<ToolChoice> = Vec::new();
    for (n, op) in &plan.used {
        let Some(t) = tools.iter().find(|t| t.tool.number == *n) else { continue };
        match tool_choices.iter_mut().find(|c| c.number == *n) {
            Some(c) => c.used_for.push(op.clone()),
            None => {
                tool_choices.push(ToolChoice { number: *n, kind: t.tool.kind.label().into(), diameter: t.tool.diameter, used_for: vec![op.clone()] })
            }
        }
    }
    tool_choices.sort_by_key(|c| c.number);
    let dry_run = if params.check.unwrap_or(false) {
        Some(match super::job(s, &json!({"job": job.clone(), "dry_run": true})) {
            Ok(r) => r,
            Err(e) => json!({"refused": message(e)}),
        })
    } else {
        None
    };
    let reply = CamSuggestReply {
        job,
        operations: plan.report,
        tools: tool_choices,
        assumptions: plan.assumptions,
        check: plan.check,
        skipped: plan.skipped,
        dry_run,
        param_warnings: Vec::new(),
    };
    serde_json::to_value(reply).map_err(|e| EngineError::Other(e.to_string()))
}

/// The partial heights a feature operation gives when the stock rises past the default retract.
fn lift(set: &FeatureSet, g: &Geo, stock_top: f64) -> Option<Value> {
    let su = set.setup(g)?;
    (stock_top > 2.0).then(|| json!({"clearance": su.top + stock_top + 10.0, "retract": su.top + stock_top + 2.0}))
}

#[allow(clippy::too_many_arguments)]
fn operations(
    plan: &mut Plan,
    set: &FeatureSet,
    tools: &[Pick],
    flats: &[&Pick],
    axis: AxisSpec,
    stock_top: f64,
    tol: f64,
    material: Option<MaterialSpec>,
) {
    let r = &set.reply;
    let in_axis = |a: Option<f64>| axis == AxisSpec::A || a.is_none_or(|a| a.abs() <= 1e-9);
    let first = set.setups.first();
    let (top, bottom) = first.map_or((0.0, 0.0), |su| (su.top, su.bottom));
    let bounds = r.setups.first().map_or([[0.0; 2]; 2], |su| su.bounds);
    let three = axis == AxisSpec::Z || set.setups.len() == 1;

    // 1. Face the top.
    let flat_top = r.faces.iter().any(|f| f.role == "top" && f.a_deg.is_none_or(|a| a.abs() <= 1e-9));
    if stock_top > 0.0 && flat_top && three {
        let margin = 1.0;
        let rect = [[bounds[0][0] - margin, bounds[0][1] - margin], [bounds[1][0] + margin, bounds[1][1] + margin]];
        let poly = vec![
            Vec2::new(rect[0][0], rect[0][1]),
            Vec2::new(rect[1][0], rect[0][1]),
            Vec2::new(rect[1][0], rect[1][1]),
            Vec2::new(rect[0][0], rect[1][1]),
        ];
        let region = vec![Region2 { outer: Loop2::polygon(&poly), holes: Vec::new() }];
        // A face mill (made for the material first), else the largest flat end mill that fits.
        let faces = largest(tools, ToolKind::FaceMill, material);
        let pick = faces.iter().copied().find(|t| t.tool.flute_length >= stock_top).or_else(|| {
            flats.iter().copied().find(|t| t.tool.flute_length >= stock_top && pocket_takes(&t.tool, &region, top + stock_top, top, 0.0, tol))
        });
        match pick {
            Some(t) => {
                let op = json!({
                    "name": "face top", "type": "face", "tool": t.tool.number,
                    "top": top + stock_top, "bottom": top, "stepdown": stock_top, "rect": rect
                });
                let what = if t.tool.kind == ToolKind::FaceMill { "a face mill" } else { "the largest flat end mill (no face mill in the library)" };
                plan.add(
                    op,
                    "face",
                    &t.tool,
                    Vec::new(),
                    format!("faces {} mm off the top with {what}, over the part's extent and 1 mm round it", num(stock_top)),
                );
            }
            None => plan.skipped.push("facing: no face mill or flat end mill with flutes as long as the stock on top".into()),
        }
    }

    // 2. 3D roughing where there are curved areas.
    let curved: Vec<(&str, f64, [[f64; 2]; 2], Option<f64>)> = r
        .surfaces
        .iter()
        .filter(|x| in_axis(x.a_deg))
        .map(|x| (x.id.as_str(), x.bottom_z, x.bounds, x.min_radius))
        .chain(
            r.fillets
                .iter()
                .filter(|x| in_axis(x.a_deg) && x.a_deg.is_none_or(|a| a.abs() <= 1e-9))
                .map(|x| (x.id.as_str(), x.bottom_z, x.bounds, x.radius)),
        )
        .collect();
    if !three && !curved.is_empty() {
        plan.skipped.push("curved areas on a 4th-axis part: finish them with a rotary or indexed 3D operation of your own".into());
    }
    let curved = if three { curved } else { Vec::new() };
    if !r.surfaces.is_empty() && three {
        // No lower than the part's bottom (a curved face's reported extent may run past its edges).
        let floor = curved.iter().map(|c| c.1).fold(top, f64::min).max(bottom);
        // A tool that stands out of its holder deeper than the roughing goes, else the largest.
        let depth = top + stock_top - floor;
        let reach = |t: &&&Pick| t.tool.stickout.unwrap_or(t.tool.flute_length) >= depth + 1.0;
        if let Some(t) = flats.iter().find(reach).or(flats.first()) {
            let floor = within_reach(plan, &t.tool, top + stock_top, floor, "rough 3D");
            let m = t.tool.radius() + 1.0;
            let op = json!({
                "name": "rough 3D", "type": "adaptive_roughing", "tool": t.tool.number,
                "heights": {"clearance": top + stock_top + 10.0, "retract": top + stock_top + 2.0, "top": top + stock_top, "bottom": floor},
                "stock_to_leave": ROUGH_STOCK, "helix_diameter": t.tool.diameter * HELIX_SHARE,
                "boundary": {"rect": [[bounds[0][0] - m, bounds[0][1] - m], [bounds[1][0] + m, bounds[1][1] + m]]}
            });
            let ids = r.surfaces.iter().map(|x| x.id.clone()).collect();
            plan.add(
                op,
                "adaptive_roughing",
                &t.tool,
                ids,
                format!(
                    "adaptive roughing of the curved areas down to Z {} leaving {} mm, with the largest flat end mill and its adaptive cutting data",
                    num(floor),
                    num(ROUGH_STOCK)
                ),
            );
        }
    }

    // 3. Rough every pocket.
    let mut finishes: Vec<(Value, String, &Pick, Vec<String>, String)> = Vec::new();
    for pk in r.pockets.iter().filter(|p| in_axis(p.a_deg)) {
        let Ok(g) = set.get(&pk.id) else { continue };
        let regions = regions_of(g, g.bottom, tol);
        let fits = |t: &Pick| pocket_takes(&t.tool, &regions, g.top, g.bottom, WALL_STOCK, tol);
        // The largest tool adaptive clearing has room for (flutes as long as the depth first),
        // else the largest the pocket takes.
        let roomy = |t: &&Pick| fits(t) && adaptive_room(&t.tool, &regions, tol);
        let long = |t: &&Pick| t.tool.flute_length >= pk.depth;
        let rough = flats
            .iter()
            .copied()
            .filter(long)
            .find(roomy)
            .or_else(|| flats.iter().copied().find(roomy))
            .or_else(|| flats.iter().copied().filter(long).find(|t| fits(t)))
            .or_else(|| flats.iter().copied().find(|t| fits(t)));
        let Some(rough) = rough else {
            plan.skipped.push(format!("{}: no flat end mill fits it (its smallest inside corner radius is {} mm)", pk.id, num(pk.min_corner_radius)));
            continue;
        };
        if rough.tool.flute_length < pk.depth {
            plan.check.push(format!(
                "{}: tool {} has {} mm of flutes for a {} mm deep pocket",
                pk.id,
                rough.tool.number,
                num(rough.tool.flute_length),
                num(pk.depth)
            ));
        }
        // Islands below the rim are cut round in bands: above each island's top it is cleared.
        let mut levels: Vec<f64> = pk.islands.iter().map(|i| i.top_z).filter(|z| *z < g.top - tol && *z > g.bottom + tol).collect();
        levels.sort_by(|a, b| b.total_cmp(a));
        levels.dedup_by(|a, b| (*a - *b).abs() <= tol);
        let mut bands: Vec<(f64, f64)> = Vec::new();
        let mut upper = g.top;
        for z in levels.iter().chain(std::iter::once(&g.bottom)) {
            bands.push((upper, *z));
            upper = *z;
        }
        let banded = bands.len() > 1;
        for (k, (hi, lo)) in bands.iter().enumerate() {
            let name = if banded { format!("rough {} band {}", pk.id, k + 1) } else { format!("rough {}", pk.id) };
            // Adaptive clearing where it has room to enter and turn (a diameter clear between the
            // walls the tool leaves); a narrow pocket is cut as a plain pocket.
            let mut op = if adaptive_room(&rough.tool, &regions, tol) {
                json!({
                    "name": name, "type": "adaptive", "tool": rough.tool.number, "feature": pk.id, "stock_to_leave": WALL_STOCK,
                    "helix_diameter": rough.tool.diameter * HELIX_SHARE
                })
            } else {
                json!({
                    "name": name, "type": "pocket", "tool": rough.tool.number, "feature": pk.id, "stepdown": rough_step(&rough.tool),
                    "stepover": rough.tool.diameter * 0.4, "stock_to_leave": WALL_STOCK, "ramp_angle_deg": RAMP_DEG
                })
            };
            let kind = if op["type"] == "adaptive" { "adaptive" } else { "pocket" };
            if banded {
                op["heights"] = json!({"top": hi, "bottom": lo});
            }
            if let Some(h) = lift(set, g, stock_top) {
                let mut merged = op.get("heights").cloned().unwrap_or_else(|| json!({}));
                if let (Some(m), Value::Object(l)) = (merged.as_object_mut(), h) {
                    m.extend(l);
                }
                op["heights"] = merged;
            }
            let (op, indexed) = at_side(op, pk.a_deg);
            let why = format!(
                "{} with the largest flat end mill the pocket fits ({} mm), {} mm left on the walls{}",
                if kind == "adaptive" { "adaptive clearing" } else { "a pocket (too narrow for adaptive clearing to enter)" },
                num(rough.tool.diameter),
                num(WALL_STOCK),
                if banded { format!(", from Z {} to {}", num(*hi), num(*lo)) } else { String::new() }
            );
            plan.add(op, &if indexed { format!("indexed {kind}") } else { kind.to_string() }, &rough.tool, vec![pk.id.clone()], why);
        }
        if !pk.walls_vertical {
            plan.check.push(format!(
                "{}: its walls are not upright (rounded, chamfered or drafted where they meet the floor): its pocket and wall finish follow \
                 the floor's outline, inside the walls, and leave them; finish the walls with a `waterline` or `parallel` operation over \
                 its bounds [[{}, {}], [{}, {}]]",
                pk.id,
                num(pk.bounds[0][0]),
                num(pk.bounds[0][1]),
                num(pk.bounds[1][0]),
                num(pk.bounds[1][1])
            ));
        }
        // The finishing contour round its walls, added after the holes.
        let corner = pk.min_corner_radius;
        let takes = |t: &Pick| contour_takes(&t.tool, &regions, Side::Inside, g.top, g.bottom, tol);
        let finish = flats
            .iter()
            .copied()
            .filter(|t| t.tool.radius() <= corner + 1e-9)
            .find(|t| takes(t))
            .or_else(|| smallest(flats.iter().copied().filter(|t| takes(t)), material));
        let Some(finish) = finish else {
            plan.skipped.push(format!("{}: no flat end mill fits round its walls for a finishing pass", pk.id));
            continue;
        };
        let why = if finish.tool.radius() <= corner + 1e-9 {
            format!("the largest flat end mill whose radius ({} mm) fits the smallest inside corner ({} mm)", num(finish.tool.radius()), num(corner))
        } else {
            plan.check.push(format!(
                "{}: {} sharp inside corners keep tool {}'s radius of {} mm",
                pk.id,
                pk.sharp_corners,
                finish.tool.number,
                num(finish.tool.radius())
            ));
            format!("the smallest flat end mill that fits ({} mm): the pocket has sharp inside corners", num(finish.tool.diameter))
        };
        let mut op = json!({
            "name": format!("finish {} walls", pk.id), "type": "contour", "tool": finish.tool.number, "feature": pk.id,
            "side": "inside", "stepdown": finish_step(&finish.tool, pk.depth)
        });
        if let Some(h) = lift(set, g, stock_top) {
            op["heights"] = h;
        }
        let (op, indexed) = at_side(op, pk.a_deg);
        finishes.push((op, if indexed { "indexed contour".into() } else { "contour".into() }, finish, vec![pk.id.clone()], why));
    }

    // 4. Holes, counterbores first.
    let mut holes: Vec<&crate::schema::cam_features::HoleReport> = r.holes.iter().filter(|h| in_axis(h.a_deg)).collect();
    holes.sort_by_key(|h| (h.counterbore_of.is_none(), h.id.clone()));
    let drills = largest(tools, ToolKind::Drill, material);
    // Holes drilled with one drill between the same heights on the same side, one operation.
    let mut drilled: Vec<(u32, [f64; 4], Option<f64>, bool, Vec<[f64; 2]>, Vec<String>, f64)> = Vec::new();
    for h in holes {
        let Ok(g) = set.get(&h.id) else { continue };
        if !(h.depth > tol) {
            plan.skipped.push(format!(
                "{} (diameter {} mm): no depth to cut from its setup's side (its wall starts and ends at Z {}): drill or mill it with an operation of your own",
                h.id,
                num(h.diameter),
                num(h.top_z)
            ));
            continue;
        }
        let regions = regions_of(g, g.bottom, tol);
        // A drill of its diameter, where a drill's point is what the hole ends in (through, or a
        // drill point) and its flutes reach.
        let drillable = h.counterbore_of.is_none() && (h.through || h.floor == "cone");
        let drill =
            drills.iter().find(|t| drillable && (t.tool.diameter - h.diameter).abs() <= DRILL_MATCH && t.tool.flute_length >= h.depth).copied();
        if let Some(d) = drill {
            let su_top = set.setup(g).map_or(top, |su| su.top);
            let heights = [su_top + stock_top + 10.0, su_top + stock_top + 2.0, h.top_z, h.bottom_z];
            match drilled.iter_mut().find(|e| {
                e.0 == d.tool.number
                    && e.1.iter().zip(&heights).all(|(a, b)| (a - b).abs() <= tol)
                    && e.2.unwrap_or(0.0) == h.a_deg.unwrap_or(0.0)
                    && e.3 == h.through
            }) {
                Some(e) => {
                    e.4.push(h.center);
                    e.5.push(h.id.clone());
                }
                None => drilled.push((d.tool.number, heights, h.a_deg, h.through, vec![h.center], vec![h.id.clone()], h.diameter)),
            }
            continue;
        }
        let pick = flats
            .iter()
            .filter(|t| t.tool.diameter <= h.diameter * HOLE_TOOL_SHARE + 1e-9)
            .find(|t| pocket_takes(&t.tool, &regions, g.top, g.bottom, 0.0, tol));
        let Some(t) = pick else {
            plan.skipped.push(format!(
                "{} (diameter {} mm): no drill of its diameter, and no flat end mill of at most {}% of its diameter ({} mm) fits; add \
                 one to the tools to drill it or to mill it as a round pocket, {{\"type\": \"pocket\", \"feature\": \"{}\", \"tool\": <its number>}}",
                h.id,
                num(h.diameter),
                num(HOLE_TOOL_SHARE * 100.0),
                num(h.diameter * HOLE_TOOL_SHARE),
                h.id,
            ));
            continue;
        };
        if t.tool.flute_length < h.depth {
            plan.check.push(format!(
                "{}: tool {} has {} mm of flutes for a {} mm deep hole",
                h.id,
                t.tool.number,
                num(t.tool.flute_length),
                num(h.depth)
            ));
        }
        let mut op = json!({
            "name": format!("mill {}", h.id), "type": "pocket", "tool": t.tool.number, "feature": h.id,
            "stepdown": rough_step(&t.tool), "stepover": t.tool.diameter * 0.4, "ramp_angle_deg": RAMP_DEG
        });
        if let Some(lh) = lift(set, g, stock_top) {
            op["heights"] = lh;
        }
        let (op, indexed) = at_side(op, h.a_deg);
        let why = format!(
            "a round pocket: the largest flat end mill of at most {}% of the {} mm diameter{}",
            num(HOLE_TOOL_SHARE * 100.0),
            num(h.diameter),
            if h.through { ", down to the part's bottom face" } else { "" }
        );
        plan.add(op, if indexed { "indexed pocket" } else { "pocket" }, &t.tool, vec![h.id.clone()], why);
    }
    for (number, [clearance, retract, top_z, bottom_z], a_deg, through, points, ids, diameter) in drilled {
        let Some(t) = tools.iter().find(|t| t.tool.number == number) else { continue };
        let depth = top_z - bottom_z;
        let mut op = json!({
            "name": format!("drill {}", ids.join(", ")), "type": "drill", "tool": number, "points": points,
            "heights": {"clearance": clearance, "retract": retract, "top": top_z, "bottom": bottom_z}, "to_full_diameter": true
        });
        let pecks = depth > PECK_DEPTHS * diameter;
        if pecks {
            op["cycle"] = json!("peck");
            op["peck"] = json!(diameter);
        }
        let (op, indexed) = at_side(op, a_deg);
        let why = format!(
            "drill {} ({} mm) has the holes' diameter: drilled to its full diameter at {}{}",
            number,
            num(t.tool.diameter),
            if through { "the part's bottom face" } else { "the holes' depth" },
            if pecks { format!(", pecking {} mm (deeper than {} diameters)", num(diameter), num(PECK_DEPTHS)) } else { String::new() }
        );
        plan.add(op, if indexed { "indexed drill" } else { "drill" }, &t.tool, ids, why);
    }
    if r.holes.iter().any(|h| h.through) {
        plan.assumptions
            .push("through holes are cut down to the part's bottom face exactly: lift the part or add breakthrough if it sits on the table".into());
    }

    // 5. Finish the pockets' walls.
    for (op, kind, t, ids, why) in finishes {
        plan.add(op, &kind, &t.tool, ids, why);
    }

    // 6. 3D finishing.
    if !curved.is_empty() {
        let limit = curved.iter().filter_map(|c| c.3).fold(f64::INFINITY, f64::min);
        let balls = largest(tools, ToolKind::BallEndmill, material);
        let pick = balls
            .iter()
            .copied()
            .find(|t| t.tool.radius() <= limit + 1e-9)
            .or_else(|| smallest(balls.iter().copied(), material))
            .or_else(|| smallest(largest(tools, ToolKind::BullNose, material).into_iter(), material))
            .or_else(|| smallest(flats.iter().copied(), material));
        if let Some(t) = pick {
            if t.tool.kind != ToolKind::BallEndmill {
                plan.check.push(format!(
                    "no ball end mill in the library: the curved areas are finished with tool {} ({}), which leaves steps",
                    t.tool.number,
                    t.tool.kind.label()
                ));
            }
            let m = t.tool.radius() + 0.5;
            let lo = curved.iter().map(|c| c.2[0]).fold([f64::INFINITY; 2], |a, b| [a[0].min(b[0]), a[1].min(b[1])]);
            let hi = curved.iter().map(|c| c.2[1]).fold([f64::NEG_INFINITY; 2], |a, b| [a[0].max(b[0]), a[1].max(b[1])]);
            let floor = curved.iter().map(|c| c.1).fold(top, f64::min).max(bottom);
            let floor = within_reach(plan, &t.tool, top, floor, "finish 3D");
            let op = json!({
                "name": "finish 3D", "type": "parallel", "tool": t.tool.number,
                "heights": {"clearance": top + stock_top + 10.0, "retract": top + stock_top + 2.0, "top": top, "bottom": floor},
                "stepover": t.tool.stepover.unwrap_or(t.tool.diameter * 0.1).min(t.tool.diameter * 0.15),
                "boundary": {"rect": [[lo[0] - m, lo[1] - m], [hi[0] + m, hi[1] + m]]}
            });
            let ids = curved.iter().map(|c| c.0.to_string()).collect();
            let why = if limit.is_finite() {
                format!("the largest ball end mill whose radius is at most the smallest radius of the curved areas ({} mm)", num(limit))
            } else {
                "a ball end mill over the curved areas".to_string()
            };
            plan.add(op, "parallel", &t.tool, ids, why);
        }
    }

    // 7. Turned diameters on a 4th axis.
    if axis == AxisSpec::A && !r.turned.is_empty() {
        let balls = largest(tools, ToolKind::BallEndmill, material);
        let pick = smallest(balls.iter().copied(), material).or_else(|| smallest(flats.iter().copied(), material));
        let swing = first.and_then(|su| su.swing).unwrap_or(top);
        if let Some(t) = pick {
            let x0 = r.turned.iter().map(|x| x.x_min).fold(f64::INFINITY, f64::min);
            let x1 = r.turned.iter().map(|x| x.x_max).fold(f64::NEG_INFINITY, f64::max);
            let step = (t.tool.diameter * 0.25 / swing.max(1.0)).to_degrees().clamp(2.0, 15.0);
            let op = json!({
                "name": "finish turned", "type": "rotary_finish", "tool": t.tool.number, "tolerance": tol * 0.5,
                "pattern": {"type": "lines", "angle_step_deg": step},
                "clearance": swing + 10.0, "retract": swing + 3.0, "range": [x0, x1]
            });
            let ids = r.turned.iter().map(|x| x.id.clone()).collect();
            plan.add(op, "rotary_finish", &t.tool, ids, format!("passes along X {} degrees apart round the turned diameters", num(step)));
        }
    }

    // 8. Outer profiles, last.
    if three {
        for pr in r.profiles.iter().filter(|p| p.a_deg.is_none_or(|a| a.abs() <= 1e-9)) {
            let Ok(g) = set.get(&pr.id) else { continue };
            let regions = regions_of(g, g.bottom, tol);
            let notch = pr.min_concave_radius.unwrap_or(f64::INFINITY);
            let ok = |t: &Pick| t.tool.radius() <= notch + 1e-9 && contour_takes(&t.tool, &regions, Side::Outside, g.top, g.bottom, tol);
            let pick =
                flats.iter().copied().filter(|t| t.tool.flute_length >= pr.depth).find(|t| ok(t)).or_else(|| flats.iter().copied().find(|t| ok(t)));
            let Some(t) = pick else {
                plan.skipped.push(format!("{}: no flat end mill fits its notches (radius {} mm)", pr.id, num(notch)));
                continue;
            };
            if t.tool.flute_length < pr.depth {
                plan.check.push(format!(
                    "{}: tool {} has {} mm of flutes for a {} mm deep profile",
                    pr.id,
                    t.tool.number,
                    num(t.tool.flute_length),
                    num(pr.depth)
                ));
            }
            let mut op = json!({
                "name": format!("profile {}", pr.id), "type": "contour", "tool": t.tool.number, "feature": pr.id,
                "side": "outside", "stepdown": rough_step(&t.tool), "ramp_angle_deg": RAMP_DEG
            });
            if let Some(h) = lift(set, g, stock_top) {
                op["heights"] = h;
            }
            let why = format!(
                "the largest flat end mill{} round the outline, from Z {} to {}",
                if notch.is_finite() { " that fits its notches" } else { "" },
                num(pr.top_z),
                num(pr.bottom_z)
            );
            plan.add(op, "contour", &t.tool, vec![pr.id.clone()], why);
            if (pr.bottom_z - bottom).abs() <= tol {
                plan.check.push(format!("{} cuts through to the part's bottom: hold the part (tabs or a fixture) before it comes free", pr.id));
            }
        }
    } else if !r.profiles.is_empty() {
        plan.skipped.push("outer profiles on a 4th-axis part: the stock is assumed to be the bar the part is turned from".into());
    }
    for f in r.faces.iter().filter(|f| f.role == "step" && in_axis(f.a_deg)) {
        plan.skipped.push(format!(
            "{} is a step open on some sides: cut it with `{{\"type\": \"pocket\", \"feature\": \"{}\"}}` and a tool of your choice (the \
             pocket stays inside the face's outline, so it leaves a tool's radius along the open sides: for those, give a `polygon` \
             running past them instead, with `heights`)",
            f.id, f.id
        ));
    }
    chamfers(plan, set, tools, in_axis, stock_top, tol, material);
    plan.assumptions.push(format!(
        "pockets are cleared by adaptive clearing leaving {} mm on their walls and finished with a contour; islands lower than a pocket's rim are cleared above their tops",
        num(WALL_STOCK)
    ));
    let _ = bottom;
}

/// `floor`, or as deep below `top` as `t` stands out of its holder (less a millimetre) where
/// that is higher, with a check line saying what is left: the holder would touch the part.
fn within_reach(plan: &mut Plan, t: &Tool, top: f64, floor: f64, what: &str) -> f64 {
    let reach = t.stickout.unwrap_or(t.flute_length) - 1.0;
    if top - floor <= reach {
        return floor;
    }
    let lowest = top - reach.max(0.0);
    plan.check.push(format!(
        "{what}: tool {} stands out {} mm from its holder, so it goes down to Z {} only (the curved areas reach Z {}): finish below with a \
         longer tool, or give it more stickout and its `stickout`",
        t.number,
        num(t.stickout.unwrap_or(t.flute_length)),
        num(lowest),
        num(floor)
    ));
    lowest
}

/// The points of a circle within `tol`, for a polygon round a hole.
fn circle_points(c: [f64; 2], d: f64, tol: f64) -> Vec<[f64; 2]> {
    let r = d * 0.5;
    let step = if tol < r { 2.0 * (1.0 - tol / r).acos() } else { std::f64::consts::FRAC_PI_4 };
    let n = ((std::f64::consts::TAU / step.max(1e-3)).ceil() as usize).clamp(8, 720);
    (0..n)
        .map(|k| {
            let a = std::f64::consts::TAU * k as f64 / n as f64;
            [c[0] + r * a.cos(), c[1] + r * a.sin()]
        })
        .collect()
}

/// `a` grown by `by` on every side, against `b`, within `tol`.
fn grown_is(a: [[f64; 2]; 2], by: f64, b: [[f64; 2]; 2], tol: f64) -> bool {
    (a[0][0] - by - b[0][0]).abs() <= tol
        && (a[0][1] - by - b[0][1]).abs() <= tol
        && (a[1][0] + by - b[1][0]).abs() <= tol
        && (a[1][1] + by - b[1][1]).abs() <= tol
}

/// Step 9, every modelled chamfer along a profile, a pocket's rim or a hole: the chamfer operation
/// with a chamfer mill of its angle, on the edge before it was chamfered.
fn chamfers(
    plan: &mut Plan,
    set: &FeatureSet,
    tools: &[Pick],
    in_axis: impl Fn(Option<f64>) -> bool,
    stock_top: f64,
    tol: f64,
    material: Option<MaterialSpec>,
) {
    let r = &set.reply;
    let mills = largest(tools, ToolKind::Chamfer, material);
    for c in r.chamfers.iter().filter(|c| in_axis(c.a_deg)) {
        let Some(angle) = c.angle_deg else { continue };
        let height = c.top_z - c.bottom_z;
        let width = height * angle.to_radians().tan();
        if !(width > tol && height > tol) {
            continue;
        }
        let near = (width * 0.05).max(tol * 10.0);
        // The edge the chamfer was cut from: a profile's outline (outside), a pocket's or a
        // hole's (inside), each where it reaches the chamfer's foot.
        let edge = r
            .profiles
            .iter()
            .filter(|p| p.a_deg == c.a_deg && grown_is(p.bounds, 0.0, c.bounds, near))
            .map(|p| (p.outline.clone(), "outside"))
            .chain(
                r.pockets.iter().filter(|p| p.a_deg == c.a_deg && grown_is(p.bounds, width, c.bounds, near)).map(|p| (p.outline.clone(), "inside")),
            )
            .chain(r.holes.iter().filter(|h| h.a_deg == c.a_deg).filter_map(|h| {
                let rad = h.diameter * 0.5;
                let b = [[h.center[0] - rad, h.center[1] - rad], [h.center[0] + rad, h.center[1] + rad]];
                grown_is(b, width, c.bounds, near).then(|| (circle_points(h.center, h.diameter, tol), "inside"))
            }))
            .next();
        let Some((outline, side)) = edge else {
            plan.skipped.push(format!(
                "{}: a chamfer {} mm wide not along a profile, pocket or hole found: cut it with a `chamfer` operation on a `polygon` of its edge",
                c.id,
                num(width)
            ));
            continue;
        };
        // A chamfer mill of the chamfer's angle whose cone reaches that wide.
        let fits = |t: &&&Pick| {
            t.tool.angle_deg.is_some_and(|a| (a * 0.5 - angle).abs() <= CHAMFER_ANGLE)
                && solidcraft_cam::chamfer_geometry(&t.tool, width, solidcraft_cam::DEFAULT_TIP_OFFSET).is_ok()
        };
        let Some(t) = mills.iter().find(fits) else {
            plan.skipped.push(format!(
                "{}: no chamfer mill of {}° (included) whose cone reaches a chamfer {} mm wide",
                c.id,
                num(angle * 2.0),
                num(width)
            ));
            continue;
        };
        let su_top = set.get(&c.id).ok().and_then(|g| set.setup(g)).map_or(c.top_z, |su| su.top);
        let op = json!({
            "name": format!("chamfer {}", c.id), "type": "chamfer", "tool": t.tool.number, "polygon": outline, "side": side,
            "width": width, "heights": {"clearance": su_top + stock_top + 10.0, "retract": su_top + stock_top + 2.0, "top": c.top_z}
        });
        let (op, indexed) = at_side(op, c.a_deg);
        let why = format!("a chamfer mill of the chamfer's {}° angle, {} mm wide {} the edge", num(angle * 2.0), num(width), side);
        plan.add(op, if indexed { "indexed chamfer" } else { "chamfer" }, &t.tool, vec![c.id.clone()], why);
    }
}
