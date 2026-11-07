//! The agent-facing side of `cam.job`: what lets an agent go from a modelled part to checked
//! G-code in a few calls without working anything out by hand (docs/cam-jobs.md, "Profiles,
//! defaults and refusals").
//!
//! - **Profiles.** A job's `profile_path` names a JSON file with the machine, the post, tool
//!   libraries (several, with clashing numbers refused unless the job picks one), the work
//!   offset, the tolerance and the default heights and feeds. The job's own keys win.
//! - **Defaults.** Heights left out are worked out from the part's exact geometry: `top` from
//!   the bodies' exact boxes, `bottom` of a pocket from point classification of the B-rep down
//!   the region (the floor where the material starts), `retract` and `clearance` above them by
//!   the profile's offsets. A tool asked for by what it is (`{kind, diameter}` or
//!   `{max_diameter, min_flute_length}`) is picked from the job's tools by a fixed order, and the
//!   reply says which one and why.
//! - **Refusals.** Every refusal names the operation (index and name), the check, the place in
//!   work coordinates and the program line where there is one, and a fix; the message ends with
//!   them as JSON (`refusals: [...]`), which the MCP server returns as structured content.
//! - **Compact replies** (`compact: true`).
//!
//! Model decisions here use the exact B-rep (`Body::exact_bounds`, `Body::planar_face`,
//! `Body::classify_point`); the toolpaths and their checks stay on the CAM crate's surface.

use std::cell::{Cell, RefCell};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use serde_json::{Map, Value};
use solidcore_record::{Region2, Seg2, Vec2, Vec3};
use solidcraft_cam::collision::Hit;
use solidcraft_cam::gcode5::{JointWords, Post5, parse5};
use solidcraft_cam::kinematics::{Carrier, Joints, Machine, forward};
use solidcraft_cam::{Feeds, Frame, Heights, HolderSection, Tool, ToolKind, offset_regions};
use solidcraft_geom::prelude::*;
use solidcraft_kernel::{Body, PointClass};

use super::{Ctx, Planned, Resolved, beside, crate_value, num, read_json, tool_sample};
use crate::params::bad;
use crate::schema::cam::{CamJob, CamJobParams, CamJobReply, Common, HeightsSpec, OperationReport, ReadbackReport};
use crate::schema::cam_agent::{
    CamJobCompactReply, CamJobOutput, CamProfile, DerivedHeights, FeedsPolicy, HeightsInput, OpRef, REFUSALS_TAG, Refusal, ToolChoice, ToolRef,
    ToolSpec,
};
use crate::{EngineError, Result, Session};

/// Most tool library files a job or profile names.
const MAX_LIBRARIES: usize = 64;
/// Most points a region's floor is classified at.
const MAX_FLOOR_SAMPLES: usize = 400;
/// Most refusals one message lists.
const MAX_REFUSALS: usize = 50;
/// Body names that suggest a fixture machined by mistake.
const FIXTURE_WORDS: &[&str] = &["vise", "vice", "clamp", "fixture", "jaw", "chuck"];

/// The job as read: its profile merged in, its machine file and tool libraries loaded.
pub(super) struct Loaded {
    pub job: CamJob,
    pub base: Option<PathBuf>,
    pub profile: Option<String>,
    pub notes: Vec<String>,
    /// Library tools left out (a kind or key this build does not know), by number.
    pub left_out: Vec<(u32, String)>,
    /// Whether an operation of the job has `gouge_check: false`.
    pub unchecked: bool,
}

/// A refusal of the job as a whole.
fn job_refusal(cmd: &str, check: &str, error: impl Into<String>, fix: impl Into<String>) -> EngineError {
    let r = Refusal { error: error.into(), op: None, check: check.into(), at: None, line: None, fix: fix.into() };
    refuse(cmd, "the job was refused", &[r], "")
}

fn str_key<'a>(cmd: &str, o: &'a Map<String, Value>, key: &str) -> Result<Option<&'a str>> {
    match o.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) if !s.trim().is_empty() => Ok(Some(s.as_str())),
        Some(_) => Err(bad(cmd, format!("`{key}` must be a file path"))),
    }
}

/// `path` relative to `dir` when it is relative (a profile's paths are read from its folder).
fn rebase(dir: Option<&Path>, path: &str) -> String {
    match dir {
        Some(d) if Path::new(path).is_relative() && !d.as_os_str().is_empty() => d.join(path).to_string_lossy().to_string(),
        _ => path.to_string(),
    }
}

/// The profile's `key` with its paths read from `dir`.
fn rebased(key: &str, v: Value, dir: Option<&Path>) -> Value {
    match (key, v) {
        ("machine_path" | "post_path" | "tool_library_path", Value::String(s)) => Value::String(rebase(dir, &s)),
        ("tool_library_paths", Value::Array(a)) => {
            Value::Array(a.into_iter().map(|p| if let Value::String(s) = p { Value::String(rebase(dir, &s)) } else { p }).collect())
        }
        (_, v) => v,
    }
}

/// The profile's keys the job does not give, put into the job.
fn merge_profile(job: &mut Map<String, Value>, profile: Map<String, Value>, dir: Option<&Path>) {
    let has = |job: &Map<String, Value>, keys: &[&str]| keys.iter().any(|k| job.get(*k).is_some_and(|v| !v.is_null()));
    for (k, v) in profile {
        let skip = match k.as_str() {
            "name" | "description" => true,
            "machine" | "machine_path" => has(job, &["machine", "machine_path"]),
            "a_reversed" | "a_limits_deg" => has(job, &["machine", "machine_path", &k]),
            "post" | "post_path" => has(job, &["post", "post_path"]),
            "tools" | "tool_library_path" | "tool_library_paths" => has(job, &["tools", "tool_library_path", "tool_library_paths"]),
            "default_heights" | "default_feeds" => {
                if let (Some(Value::Object(mine)), Value::Object(theirs)) = (job.get_mut(&k), &v) {
                    for (f, x) in theirs {
                        mine.entry(f.clone()).or_insert_with(|| x.clone());
                    }
                    true
                } else {
                    has(job, &[&k])
                }
            }
            _ => has(job, &[&k]),
        };
        if !skip {
            let v = rebased(&k, v, dir);
            job.insert(k, v);
        }
    }
}

/// The name a library is picked by: its file name without `.json`.
fn library_name(path: &str) -> String {
    let file = Path::new(path).file_name().map_or_else(|| path.to_string(), |f| f.to_string_lossy().to_string());
    file.strip_suffix(".json").map_or(file.clone(), str::to_string)
}

/// The tools of `paths` (read from `base`), or of the one `pick` names.
fn library_tools(cmd: &str, base: Option<&Path>, paths: &[String], pick: Option<&str>, left_out: &mut Vec<(u32, String)>) -> Result<Vec<Value>> {
    if paths.is_empty() || paths.len() > MAX_LIBRARIES {
        return Err(bad(cmd, format!("`tool_library_paths` names 1 to {MAX_LIBRARIES} files")));
    }
    let names: Vec<String> = paths.iter().map(|p| library_name(p)).collect();
    let chosen: Vec<usize> = match pick {
        None => (0..paths.len()).collect(),
        Some(want) => {
            let w = want.trim().to_lowercase();
            let w = w.strip_suffix(".json").unwrap_or(&w).to_string();
            let hit = paths.iter().zip(&names).position(|(p, n)| {
                let p = p.to_lowercase();
                p == w || p.strip_suffix(".json") == Some(w.as_str()) || n.to_lowercase() == w
            });
            match hit {
                Some(i) => vec![i],
                None => {
                    return Err(job_refusal(
                        cmd,
                        "tools",
                        format!("`tool_library` `{want}` is none of the tool libraries"),
                        format!("give `tool_library` one of: {}", names.iter().map(|n| format!("\"{n}\"")).collect::<Vec<_>>().join(", ")),
                    ));
                }
            }
        }
    };
    let sample = tool_sample();
    let mut by_number: BTreeMap<u32, (usize, Value)> = BTreeMap::new();
    let mut clashes: BTreeMap<(usize, usize), Vec<u32>> = BTreeMap::new();
    for &li in &chosen {
        let (Some(path), Some(lib)) = (paths.get(li), names.get(li)) else { continue };
        let Value::Array(tools) = read_json(cmd, &beside(base, path), "tool library")? else {
            return Err(bad(cmd, format!("tool library {path}: a tool library file holds a JSON array of tools")));
        };
        for (i, v) in tools.into_iter().enumerate() {
            let what = format!("tool library `{lib}` [{i}]");
            let number = v.get("number").and_then(Value::as_u64).and_then(|n| u32::try_from(n).ok());
            let read = crate_value::<Tool>(cmd, v.clone(), &sample, &what).and_then(|t| t.validate().map_err(|e| bad(cmd, format!("{what}: {e}"))));
            let Some(number) = number else {
                left_out.push((0, format!("{what} was left out: it has no `number`")));
                continue;
            };
            if let Err(e) = read {
                let why = match e {
                    EngineError::BadParams { msg, .. } => msg,
                    other => other.to_string(),
                };
                left_out.push((number, format!("tool {number} of `{lib}` was left out: {why}")));
                continue;
            }
            match by_number.get(&number) {
                Some((other, existing)) if *existing != v => clashes.entry((*other, li)).or_default().push(number),
                Some(_) => {}
                None => {
                    by_number.insert(number, (li, v));
                }
            }
        }
    }
    if !clashes.is_empty() {
        let list: Vec<String> = clashes
            .iter()
            .map(|((a, b), nums)| {
                let a = names.get(*a).map_or("?", String::as_str);
                let b = names.get(*b).map_or("?", String::as_str);
                format!("{} in both `{a}` and `{b}`", nums.iter().map(|n| format!("T{n}")).collect::<Vec<_>>().join(", "))
            })
            .collect();
        return Err(job_refusal(
            cmd,
            "tools",
            format!("tool numbers clash between the tool libraries: {}", list.join("; ")),
            format!(
                "add `tool_library` to the job to take every tool from one library (one of: {}), or renumber the tools so no number is in two libraries",
                names.iter().map(|n| format!("\"{n}\"")).collect::<Vec<_>>().join(", ")
            ),
        ));
    }
    if by_number.is_empty() {
        return Err(bad(cmd, "the tool libraries hold no tool this build can read"));
    }
    Ok(by_number.into_values().map(|(_, v)| v).collect())
}

/// The job of a `cam.job` call: inline or from `job_path`, its profile merged in, a machine
/// file and tool libraries read. Relative paths of the job are left relative to its file's
/// folder (`base`); a profile's are made relative to the job's folder through the profile's.
pub(super) fn load_job(cmd: &str, p: &Value, params: &CamJobParams) -> Result<Loaded> {
    load_job_of(cmd, p, params.job.is_some(), params.job_path.as_ref())
}

/// [`load_job`] for any command that takes a `job` (in `p`, when `has_job`) or a `job_path`.
pub(super) fn load_job_of(cmd: &str, p: &Value, has_job: bool, job_path: Option<&String>) -> Result<Loaded> {
    let (raw, base, what) = match (has_job, job_path) {
        (true, Some(_)) => return Err(bad(cmd, "give `job` or `job_path`, not both")),
        (false, None) => return Err(bad(cmd, "give the `job` (an object) or a `job_path` (a .json file)")),
        (true, None) => (p.get("job").cloned().unwrap_or(Value::Null), None, String::new()),
        (false, Some(path)) => (read_json(cmd, path, "job_path")?, Path::new(path).parent().map(Path::to_path_buf), format!("job_path {path}")),
    };
    let Value::Object(mut job) = raw else {
        return Err(bad(cmd, format!("{what}: the job must be a JSON object").trim_start_matches(": ").to_string()));
    };
    let based = base.as_deref();
    let mut profile = None;
    if let Some(given) = str_key(cmd, &job, "profile_path")?.map(str::to_string) {
        let path = beside(based, &given);
        let v = read_json(cmd, &path, "profile_path")?;
        let _: CamProfile = super::parse(cmd, &v, &format!("profile {path}"))?;
        if let Value::Object(o) = v {
            merge_profile(&mut job, o, Path::new(&given).parent());
        }
        job.remove("profile_path");
        profile = Some(path);
    }
    if let Some(mp) = str_key(cmd, &job, "machine_path")?.map(str::to_string) {
        if job.get("machine").is_some_and(|m| !m.is_null()) {
            return Err(bad(cmd, "give the job `machine` or `machine_path`, not both"));
        }
        let m = read_json(cmd, &beside(based, &mp), "machine_path")?;
        if !m.is_object() {
            return Err(bad(cmd, format!("machine_path {mp}: a machine file holds the CAM crate's Machine JSON object")));
        }
        job.insert("machine".into(), m);
        job.remove("machine_path");
    }
    let mut left_out = Vec::new();
    let pick = job.remove("tool_library");
    let pick = match &pick {
        None | Some(Value::Null) => None,
        Some(Value::String(s)) => Some(s.as_str()),
        Some(_) => return Err(bad(cmd, "`tool_library` must be the name of one of `tool_library_paths`")),
    };
    match job.remove("tool_library_paths") {
        None | Some(Value::Null) => {
            if pick.is_some() {
                return Err(job_refusal(
                    cmd,
                    "tools",
                    "`tool_library` picks one of `tool_library_paths`, and there are none",
                    "leave `tool_library` out, or list the libraries in `tool_library_paths`",
                ));
            }
        }
        Some(Value::Array(list)) => {
            if job.get("tools").is_some_and(|t| !t.is_null()) || job.get("tool_library_path").is_some_and(|t| !t.is_null()) {
                return Err(bad(cmd, "give the job one of `tools`, `tool_library_path` and `tool_library_paths`"));
            }
            let paths: Vec<String> = list.iter().filter_map(|v| v.as_str().map(str::to_string)).collect();
            if paths.len() != list.len() {
                return Err(bad(cmd, "`tool_library_paths` is a list of file paths"));
            }
            let tools = library_tools(cmd, based, &paths, pick, &mut left_out)?;
            job.insert("tools".into(), Value::Array(tools));
        }
        Some(_) => return Err(bad(cmd, "`tool_library_paths` is a list of file paths")),
    }
    let unchecked = job.get("operations").is_some_and(super::sim::unchecked);
    let job: CamJob = super::parse(cmd, &Value::Object(job), &what)?;
    let notes = left_out.iter().map(|(_, n)| n.clone()).collect();
    Ok(Loaded { job, base, profile, notes, left_out, unchecked })
}

/// The part's and the fixtures' exact geometry, in design coordinates.
pub(super) struct Geometry {
    part: Vec<Body>,
    fixtures: Vec<Body>,
    part_box: std::result::Result<[Vec3; 2], String>,
    fixture_box: Option<[Vec3; 2]>,
    origin: Vec3,
    /// How far from a face a classified point is taken to be off it (mm).
    delta: f64,
}

fn exact_box(bodies: &[&Body]) -> std::result::Result<[Vec3; 2], String> {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for b in bodies {
        let bb = b.exact_bounds().map_err(|e| e.to_string())?;
        lo = lo.min(bb.min);
        hi = hi.max(bb.max);
    }
    if lo.is_finite() && hi.is_finite() { Ok([lo, hi]) } else { Err("the part has no exact box".into()) }
}

impl Geometry {
    /// The bodies of `job` (as `surfaces_of` picks them). `notes` gets a hint for a part body
    /// whose name says it holds the part.
    pub fn of(s: &Session, job: &CamJob, origin: Vec3, notes: &mut Vec<String>) -> Result<Geometry> {
        let st = s.world_state()?;
        let fixtures: Vec<String> = job.fixtures.clone().unwrap_or_default();
        let names: Vec<String> = match &job.bodies {
            Some(b) => b.clone(),
            None => st.bodies.iter().map(|b| b.name.clone()).filter(|n| !fixtures.contains(n) && s.body_visible(n)).collect(),
        };
        for n in &names {
            let low = n.to_lowercase();
            if FIXTURE_WORDS.iter().any(|w| low.contains(w)) {
                notes.push(format!("body `{n}` is machined as part of the part: if it holds the part, add it to `fixtures`"));
            }
        }
        let part: Vec<Body> = names.iter().filter_map(|n| st.bodies.iter().find(|b| &b.name == n)).map(|b| b.body.clone()).collect();
        let fixture_bodies: Vec<Body> = fixtures.iter().filter_map(|n| st.bodies.iter().find(|b| &b.name == n)).map(|b| b.body.clone()).collect();
        let part_box = exact_box(&part.iter().collect::<Vec<_>>());
        let fixture_box = if fixture_bodies.is_empty() { None } else { exact_box(&fixture_bodies.iter().collect::<Vec<_>>()).ok() };
        let tol = part.iter().map(|b| b.record().tolerance).fold(0.0, f64::max);
        let delta = (tol * 4.0).clamp(1e-3, 0.05);
        Ok(Geometry { part, fixtures: fixture_bodies, part_box, fixture_box, origin, delta })
    }

    /// The highest program Z the part and the fixtures reach while the machine's table rotaries
    /// turn them through any angle, with the rotaries' name; `None` on a machine without
    /// rotaries or with a rotary head (whose program Z is not the table's). On a table machine
    /// the program's coordinates are the machine's, so this is where a tool must be for the
    /// part to turn under it. An upper bound from the exact boxes' corners, made tight on one
    /// rotary with the exact boxes of the bodies turned every 10° (a point's height between two
    /// of those is at most 1/cos 5° of its height above the axis there).
    fn turn_top(&self, machine: &Machine) -> Option<(f64, String)> {
        let rotaries = &machine.rotaries;
        if rotaries.is_empty() || rotaries.iter().any(|r| r.carrier != Carrier::Table) {
            return None;
        }
        let o = machine.work_origin;
        let boxes: Vec<[Vec3; 2]> = self.part_box.clone().ok().into_iter().chain(self.fixture_box).collect();
        if boxes.is_empty() {
            return None;
        }
        // The boxes' corners where they are in the machine with every rotary at zero.
        let corners: Vec<Vec3> = boxes
            .iter()
            .flat_map(|[lo, hi]| {
                (0..8).map(move |i| {
                    Vec3::new(if i & 1 == 0 { lo.x } else { hi.x }, if i & 2 == 0 { lo.y } else { hi.y }, if i & 4 == 0 { lo.z } else { hi.z })
                })
            })
            .map(|p| p - self.origin + o)
            .collect();
        let (top, name) = match rotaries.as_slice() {
            [r] => {
                let d = r.direction.normalized()?;
                let c = r.center;
                let side = (1.0 - d.z * d.z).max(0.0).sqrt();
                let along = |q: Vec3| c.z + (q - c).dot(d) * d.z;
                let radial = |q: Vec3| ((q - c) - d * (q - c).dot(d)).len();
                let corner = corners.iter().map(|&q| along(q) + radial(q) * side).fold(f64::NEG_INFINITY, f64::max);
                let low_along = corners.iter().map(|&q| along(q)).fold(f64::INFINITY, f64::min);
                // The exact boxes of the bodies turned about the axis (in design coordinates:
                // through `c - o + origin`), as program Z.
                let pivot = c - o + self.origin;
                let lift = o.z - self.origin.z;
                let mut sampled = Some(f64::NEG_INFINITY);
                'turns: for k in 0..36 {
                    let angle = (k as f64 * 10.0).to_radians();
                    for b in self.part.iter().chain(&self.fixtures) {
                        match solidcraft_kernel::transform(b, Vec3::ZERO, pivot, d, angle).and_then(|t| t.exact_bounds()) {
                            Ok(bb) if bb.max.z.is_finite() => sampled = sampled.map(|s| s.max(bb.max.z + lift)),
                            _ => {
                                sampled = None;
                                break 'turns;
                            }
                        }
                    }
                }
                let half = 5f64.to_radians().cos();
                let tight = sampled.filter(|s| s.is_finite()).map(|s| s / half - low_along * (1.0 / half - 1.0));
                (tight.map_or(corner, |t| t.min(corner)), format!("{} axis", r.letter.char()))
            }
            rs => {
                // Turning about a line through c_k keeps a point within |p - c_k| of it, so
                // within |p - c_0| + 2 |c_k - c_0| of the first centre after every turn.
                let c0 = rs.first()?.center;
                let spread: f64 = rs.iter().map(|r| 2.0 * (r.center - c0).len()).sum();
                let reach = corners.iter().map(|&q| (q - c0).len()).fold(f64::NEG_INFINITY, f64::max);
                let letters: String = rs.iter().map(|r| r.letter.char()).collect();
                (c0.z + reach + spread, format!("{letters} axes"))
            }
        };
        top.is_finite().then_some((top, name))
    }

    /// The highest and lowest Z of `bounds` in `frame` (inside work coordinates), and whether
    /// that is exact: the box's corners turned into a frame whose axes are not along the
    /// design's give a Z range at least as wide as the part's.
    fn z_range(&self, bounds: [Vec3; 2], frame: &Frame) -> (f64, f64, bool) {
        let [lo, hi] = bounds;
        let mut top = f64::NEG_INFINITY;
        let mut low = f64::INFINITY;
        for i in 0..8 {
            let c = Vec3::new(if i & 1 == 0 { lo.x } else { hi.x }, if i & 2 == 0 { lo.y } else { hi.y }, if i & 4 == 0 { lo.z } else { hi.z });
            let z = frame.to_local(c - self.origin).z;
            top = top.max(z);
            low = low.min(z);
        }
        let along = |v: Vec3| [v.x, v.y, v.z].iter().all(|c| c.abs() < 1e-9 || (c.abs() - 1.0).abs() < 1e-9);
        (top, low, along(frame.x) && along(frame.y) && along(frame.z))
    }

    /// The part's (top, bottom, exact) in `frame`.
    fn part_range(&self, frame: &Frame) -> std::result::Result<(f64, f64, bool), String> {
        self.part_box.clone().map(|b| self.z_range(b, frame))
    }

    /// The fixtures' top in `frame`, when there are fixtures.
    fn fixture_top(&self, frame: &Frame) -> Option<f64> {
        self.fixture_box.map(|b| self.z_range(b, frame).0)
    }

    /// Whether the frame point `p` is in the part's material or on its boundary.
    fn solid_at(&self, frame: &Frame, p: Vec3) -> std::result::Result<bool, String> {
        let q = frame.to_world(p) + self.origin;
        for b in &self.part {
            match b.classify_point(q).map_err(|e| e.to_string())? {
                PointClass::Inside | PointClass::OnBoundary => return Ok(true),
                PointClass::Outside => {}
            }
        }
        Ok(false)
    }

    /// The heights in `frame` of the part's flat faces that face up its Z, highest first.
    fn up_levels(&self, frame: &Frame, top: f64, low: f64) -> Vec<f64> {
        let mut levels: Vec<f64> = Vec::new();
        for b in &self.part {
            if b.face_count() > 20_000 {
                continue;
            }
            for i in 0..b.face_count() {
                let Some(pl) = b.planar_face(i) else { continue };
                let n = frame.dir_to_local(pl.x.cross(pl.y));
                let len = n.len();
                if !(len > 0.0) || n.z / len < 1.0 - 1e-9 {
                    continue;
                }
                let h = frame.to_local(pl.origin - self.origin).z;
                if h.is_finite() && h < top - 1e-9 && h > low + 1e-9 {
                    levels.push(h);
                }
            }
        }
        levels.push(low);
        levels.sort_by(|a, b| b.total_cmp(a));
        levels.dedup_by(|a, b| (*a - *b).abs() < 1e-7);
        levels
    }

    /// Where the material starts going down the column at (x, y) of `frame` from `top`: a flat
    /// floor exactly, a curved one within `tol` from above; `None` where the column has none.
    fn column_floor(&self, frame: &Frame, x: f64, y: f64, top: f64, levels: &[f64], tol: f64) -> std::result::Result<Option<f64>, String> {
        let d = self.delta;
        let at = |z: f64| self.solid_at(frame, Vec3::new(x, y, z));
        if at(top + d)? {
            return Ok(Some(top));
        }
        let mut out = top + d;
        for &h in levels {
            if h >= out {
                continue;
            }
            if h + d < out && at(h + d)? {
                // The material starts between this level and the last clear point: a curved
                // floor, found by halving the gap, kept on the clear side.
                let (mut inside, mut clear) = (h + d, out);
                for _ in 0..60 {
                    if clear - inside <= tol * 0.25 {
                        break;
                    }
                    let mid = 0.5 * (inside + clear);
                    if at(mid)? { inside = mid } else { clear = mid }
                }
                return Ok(Some(clear.min(top)));
            }
            if at(h - d)? {
                return Ok(Some(h));
            }
            out = h - d;
        }
        Ok(None)
    }

    /// The lowest floor of the part inside `regions` (in `frame`'s XY), at or below `top`, over
    /// the columns that meet material: a hole through the part in the region (a bolt hole in a
    /// pocket's floor) is no floor, and is reported apart. `lowest` is `None` where every column
    /// goes through.
    fn floor(&self, frame: &Frame, regions: &[Region2], top: f64, low: f64, tol: f64) -> std::result::Result<Floor, String> {
        let samples = region_samples(regions, tol)?;
        if samples.is_empty() {
            return Err("the region is too narrow to find its floor".into());
        }
        let levels = self.up_levels(frame, top, low);
        let mut floor = Floor { lowest: None, through_at: None };
        for q in samples {
            match self.column_floor(frame, q.x, q.y, top, &levels, tol)? {
                Some(z) => floor.lowest = Some(floor.lowest.map_or(z, |f: f64| f.min(z))),
                None => {
                    floor.through_at.get_or_insert(q);
                }
            }
        }
        Ok(floor)
    }
}

/// What [`Geometry::floor`] finds down a region.
struct Floor {
    /// The lowest height the material starts at, over the columns that meet any.
    lowest: Option<f64>,
    /// A point (frame XY) where the region goes through the part.
    through_at: Option<Vec2>,
}

/// The point a fraction `t` along `s` (lines and arcs; other curves by their ends).
fn seg_point(s: &Seg2, t: f64) -> Vec2 {
    match *s {
        Seg2::Line { a, b } => Vec2::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t),
        Seg2::Arc { center, radius, start, sweep } => {
            let a = start + sweep * t;
            Vec2::new(center.x + radius * a.cos(), center.y + radius * a.sin())
        }
        Seg2::Cubic { p0, p3, .. } => Vec2::new(p0.x + (p3.x - p0.x) * t, p0.y + (p3.y - p0.y) * t),
        Seg2::Conic { a, b, .. } => Vec2::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t),
        Seg2::Bezier { n, p } => {
            let (Some(a), Some(b)) = (p.first(), p.get(usize::from(n))) else { return Vec2::new(0.0, 0.0) };
            Vec2::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
        }
    }
}

fn seg_length(s: &Seg2) -> f64 {
    match *s {
        Seg2::Arc { radius, sweep, .. } => (radius * sweep).abs(),
        _ => seg_point(s, 0.0).dist(seg_point(s, 1.0)),
    }
}

/// Points spread over `regions`: on rings moved into them, from just inside the boundary to
/// the middle, at most [`MAX_FLOOR_SAMPLES`].
fn region_samples(regions: &[Region2], tol: f64) -> std::result::Result<Vec<Vec2>, String> {
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for r in regions {
        for lp in r.loops() {
            for s in &lp.segs {
                for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
                    let q = seg_point(s, t);
                    lo = Vec2::new(lo.x.min(q.x), lo.y.min(q.y));
                    hi = Vec2::new(hi.x.max(q.x), hi.y.max(q.y));
                }
            }
        }
    }
    let size = (hi.x - lo.x).max(hi.y - lo.y);
    if !(size.is_finite() && size > 0.0) {
        return Ok(Vec::new());
    }
    let spacing = (size / 20.0).max(0.05);
    let margin = (tol * 2.0).max(1e-3).min(spacing * 0.5);
    let mut pts: Vec<Vec2> = Vec::new();
    for k in 0..40 {
        let loops = offset_regions(regions, margin + spacing * k as f64, tol.max(1e-4)).map_err(|e| e.to_string())?;
        if loops.is_empty() {
            break;
        }
        for lp in &loops {
            for s in &lp.segs {
                let n = (seg_length(s) / spacing).ceil().clamp(1.0, 200.0) as usize;
                pts.extend((0..n).map(|i| seg_point(s, i as f64 / n as f64)));
            }
        }
        if pts.len() > MAX_FLOOR_SAMPLES * 20 {
            break;
        }
    }
    let every = pts.len().div_ceil(MAX_FLOOR_SAMPLES).max(1);
    Ok(pts.into_iter().step_by(every).collect())
}

/// What the agent side keeps while a job is planned: the policies, the operation being
/// planned, and what the reply reports.
pub(super) struct Agent {
    pub geo: Geometry,
    clearance_above: f64,
    retract_above: f64,
    feeds: FeedsPolicy,
    left_out: Vec<(u32, String)>,
    profile: Option<String>,
    index: Cell<usize>,
    name: RefCell<Option<String>>,
    pending: RefCell<Option<Refusal>>,
    choices: RefCell<Vec<ToolChoice>>,
    derived: RefCell<Vec<DerivedHeights>>,
    used: RefCell<BTreeMap<usize, (Heights, Frame)>>,
    notes: RefCell<Vec<String>>,
    /// The height the part and fixtures reach as the table turns, and the rotaries' name, when
    /// the post turns the rotaries at the operations' clearance (no `safe_rotate` to the
    /// machine's top, no post `clearance` above it).
    turn: Option<(f64, String)>,
    /// The job's tolerance (mm).
    tol: f64,
    /// The operations whose heights or tool were worked out here: their tools are checked with
    /// at least the holder they must have.
    auto: RefCell<BTreeSet<usize>>,
}

/// How much wider than its shank a holder is taken to be when a tool gives none (mm), and how
/// far up from the holder's face it is checked.
const LEAST_HOLDER: (f64, f64) = (1.0, 50.0);

fn positive(v: Option<f64>, max: f64) -> bool {
    v.is_none_or(|x| x.is_finite() && x > 0.0 && x <= max)
}

impl Agent {
    pub fn new(cmd: &str, job: &CamJob, geo: Geometry, loaded: &Loaded, machine: &Machine, post: &Post5) -> Result<Agent> {
        let h = job.default_heights.unwrap_or_default();
        let clearance_above = h.clearance_above.unwrap_or(10.0);
        let retract_above = h.retract_above.unwrap_or(2.0);
        if !(clearance_above.is_finite() && retract_above.is_finite() && (0.0..=1000.0).contains(&retract_above) && clearance_above >= retract_above)
            || clearance_above > 1000.0
        {
            return Err(bad(cmd, "`default_heights` must have 0 ≤ retract_above ≤ clearance_above ≤ 1000 mm"));
        }
        let f = job.default_feeds.unwrap_or_default();
        let scale_ok = |s: Option<f64>| s.is_none_or(|x| x.is_finite() && (0.05..=2.0).contains(&x));
        let range_ok = f.spindle_rpm_range.is_none_or(|[lo, hi]| lo.is_finite() && hi.is_finite() && lo > 0.0 && lo <= hi && hi <= 200_000.0);
        let fallback_ok =
            f.fallback.is_none_or(|b| positive(Some(b.cut), 1e6) && positive(Some(b.plunge), 1e6) && positive(Some(b.spindle_rpm), 200_000.0));
        if !(scale_ok(f.cut_scale) && scale_ok(f.plunge_scale) && positive(f.max_cut, 1e6) && positive(f.max_plunge, 1e6) && range_ok && fallback_ok)
        {
            return Err(bad(
                cmd,
                "`default_feeds` must have scales from 0.05 to 2, positive feed limits, a spindle range [lowest, highest] above 0 and a positive fallback",
            ));
        }
        // A post that goes to the machine's top before a turn, or to its own clearance above
        // the part's reach, turns the part clear of the tool whatever the operations' heights.
        let turn = if post.safe_rotate.is_empty() {
            geo.turn_top(machine).filter(|(top, _)| !(post.clearance_retract && post.clearance.is_some_and(|c| c > *top)))
        } else {
            None
        };
        Ok(Agent {
            geo,
            turn,
            tol: job.tolerance.unwrap_or(0.01),
            auto: RefCell::new(BTreeSet::new()),
            clearance_above,
            retract_above,
            feeds: f,
            left_out: loaded.left_out.clone(),
            profile: loaded.profile.clone(),
            index: Cell::new(0),
            name: RefCell::new(None),
            pending: RefCell::new(None),
            choices: RefCell::new(Vec::new()),
            derived: RefCell::new(Vec::new()),
            used: RefCell::new(BTreeMap::new()),
            notes: RefCell::new(Vec::new()),
        })
    }

    /// Operation `index` is planned next.
    pub fn start(&self, index: usize) {
        self.index.set(index);
        self.name.replace(None);
        self.pending.replace(None);
    }

    /// The operation being planned is called `name` (every refusal of an operation names it).
    pub fn name_op(&self, name: &str) {
        self.name.replace(Some(name.to_string()));
    }

    fn op(&self, name: &str) -> OpRef {
        OpRef { index: self.index.get(), name: name.to_string() }
    }

    fn note(&self, n: String) {
        self.notes.borrow_mut().push(n);
    }

    /// The structured refusal of a failed operation: the one [`refuse_op`] recorded, else one
    /// read from the message.
    pub fn refusal_of(&self, e: EngineError) -> Refusal {
        if let Some(r) = self.pending.replace(None) {
            return r;
        }
        let name = self.name.borrow().clone();
        let msg = match e {
            EngineError::BadParams { msg, .. } => msg,
            other => other.to_string(),
        };
        let why = match &name {
            Some(n) => msg.strip_prefix(&format!("operation `{n}`: ")).map_or(msg.clone(), str::to_string),
            None => msg.clone(),
        };
        let (check, fix) = fix_for(&why);
        Refusal {
            error: why,
            op: Some(OpRef { index: self.index.get(), name: name.unwrap_or_else(|| format!("operation {}", self.index.get())) }),
            check: check.into(),
            at: None,
            line: None,
            fix,
        }
    }

    /// What the reply carries from here.
    pub fn finish(&self, reply: &mut CamJobReply) {
        reply.tool_choices = self.choices.borrow().clone();
        reply.derived_heights = self.derived.borrow().clone();
        reply.profile = self.profile.clone();
        reply.notes.extend(self.notes.borrow().iter().cloned());
    }

    /// The heights operation `index` was planned with, and its frame.
    fn used(&self, index: usize) -> Option<(Heights, Frame)> {
        self.used.borrow().get(&index).copied()
    }
}

/// Records a structured refusal of the operation being planned and returns its error.
fn refuse_op(ctx: &Ctx<'_>, op: &str, check: &str, error: String, at: Option<[f64; 3]>, fix: String) -> EngineError {
    let e = ctx.bad(op, format!("{error}. Fix: {fix}"));
    ctx.agent.pending.replace(Some(Refusal { error, op: Some(ctx.agent.op(op)), check: check.into(), at, line: None, fix }));
    e
}

/// The check and a fix for an operation's error message from the CAM crate or the handler.
fn fix_for(why: &str) -> (&'static str, String) {
    let w = why.to_lowercase();
    let fix = |s: &str| s.to_string();
    // The setup check: A turns into the part where the tool is.
    if w.contains("swing into the tool") {
        let raise = why.find("Raise the clearance").map_or(why, |i| why.get(i..).unwrap_or(why));
        return ("clearance", raise.to_string());
    }
    if let Some((_, after)) = why.split_once(". Fix: ") {
        return ("plan", after.to_string());
    }
    if w.contains("heights must be ordered") {
        ("heights", fix("give `heights` with clearance ≥ retract ≥ top ≥ bottom, or leave `heights` out to have them worked out from the part"))
    } else if w.contains("retract height") && w.contains("below the top") {
        (
            "heights",
            fix("raise `heights.retract` and `heights.clearance` above the part's top, or leave them out to have them worked out from the part"),
        )
    } else if w.contains("stepover must be above 0") {
        ("plan", fix("give `stepover` above 0 and at most the tool's diameter"))
    } else if w.contains("stepdown must be above 0") {
        ("plan", fix("give `stepdown` above 0"))
    } else if w.contains("a drill cannot mill") {
        ("tool", fix("cut with a milling tool: `tool: {kind: \"flat_endmill\"}` (or ball_endmill, bull_nose) picks one"))
    } else if w.contains("does not fit") {
        ("fit", fix("use a smaller tool (`tool: {max_diameter: …}`) or a wider profile"))
    } else if w.contains("no sketch") {
        ("region", fix("give a sketch id or name that `inspect_design` lists, or a `polygon`"))
    } else if w.contains("give a `sketch` or a `polygon`") {
        ("region", fix("give `sketch` (and `profiles`) or `polygon: [[x, y], …]` in the operation's coordinates"))
    } else if w.contains("not on a plane square") {
        ("region", fix("draw the sketch on a plane facing the operation's tool axis (for a 3-axis operation, an XY plane), or give a `polygon`"))
    } else if w.contains("rotary") && w.contains("needs a machine") {
        ("plan", fix("use `machine: \"4axis_a\"` (or a profile with a 4th axis)"))
    } else if w.contains("feed") {
        ("feeds", fix("give `feeds: {cut, plunge}` above 0, or a tool with `feed` and `plunge_feed`"))
    } else if w.contains("spindle") {
        ("feeds", fix("give `spindle_rpm` above 0 and at most 200000, or a tool with `spindle_rpm`"))
    } else if w.contains("tolerance") {
        ("plan", fix("give `tolerance` from 0.0001 to 1 mm"))
    } else if let Some((_, hint)) = why.rsplit_once("; ") {
        ("plan", hint.to_string())
    } else if w.contains("more than") {
        ("plan", fix("use a larger step, stepover or tolerance, or a smaller boundary"))
    } else {
        ("plan", fix("change the operation as the error says (docs/cam-jobs.md lists every key and its default)"))
    }
}

/// Where a refusal is, for the message.
fn place(r: &Refusal) -> String {
    let mut s = match &r.op {
        Some(op) => format!("operation {} `{}` ({})", op.index, op.name, r.check),
        None => format!("the job ({})", r.check),
    };
    if let Some([x, y, z]) = r.at {
        s.push_str(&format!(" at ({}, {}, {})", num(x), num(y), num(z)));
    }
    if let Some(l) = r.line {
        s.push_str(&format!(", program line {l}"));
    }
    s
}

/// The error of a refused call: a line per refusal with its fix, `tail`, then the refusals as
/// JSON after [`REFUSALS_TAG`].
pub(super) fn refuse(cmd: &str, headline: &str, refusals: &[Refusal], tail: &str) -> EngineError {
    let shown = refusals.get(..MAX_REFUSALS).unwrap_or(refusals);
    let mut text = format!("{headline}: {} problem{}.", refusals.len(), if refusals.len() == 1 { "" } else { "s" });
    for (i, r) in shown.iter().enumerate() {
        text.push_str(&format!("\n{}. {}: {} Fix: {}", i + 1, place(r), r.error.trim_end_matches('.').to_string() + ".", r.fix));
    }
    if refusals.len() > shown.len() {
        text.push_str(&format!("\n… and {} more", refusals.len() - shown.len()));
    }
    if !tail.is_empty() {
        text.push('\n');
        text.push_str(tail);
    }
    text.push_str(REFUSALS_TAG);
    text.push_str(&serde_json::to_string(shown).unwrap_or_else(|_| "[]".into()));
    bad(cmd, text)
}

/// Which part of an operation's region its `bottom` comes from.
pub(super) enum Region<'a> {
    /// The lowest floor inside these profiles (a pocket, an inside contour).
    Floor(&'a [Region2]),
    /// The part's bottom (an outside contour, the 3D operations).
    Part,
}

/// The heights of a 3-axis operation in `frame`: those given, the rest worked out from the
/// part, then checked (ordered, and clear of the part and fixtures when that is exact).
pub(super) fn heights(ctx: &Ctx<'_>, op: &str, given: Option<&HeightsInput>, region: Region<'_>, frame: &Frame, tol: f64) -> Result<Heights> {
    let ag = ctx.agent;
    ag.name_op(op);
    let g = given.copied().unwrap_or_default();
    let range = ag.geo.part_range(frame);
    let need = |what: &str| -> Result<(f64, f64, bool)> {
        range.clone().map_err(|e| {
            refuse_op(
                ctx,
                op,
                "heights",
                format!("the part's top and bottom could not be worked out ({e})"),
                None,
                format!("give `heights.{what}` (and the other heights) yourself"),
            )
        })
    };
    // Given heights are checked before anything is worked out from them.
    if [g.clearance, g.retract, g.top, g.bottom].iter().flatten().any(|v| !(v.is_finite() && v.abs() <= super::MAX_COORD)) {
        return Err(refuse_op(ctx, op, "heights", "the heights must be finite numbers within 1000 m".into(), None, "give finite heights".into()));
    }
    let mut derived: Vec<String> = Vec::new();
    let mut why: Vec<String> = Vec::new();
    let top = match g.top {
        Some(t) => t,
        None => {
            let (t, _, exact) = need("top")?;
            derived.push("top".into());
            // A stock that stands above the part is material to cut from its top down, and the
            // retract and clearance must clear it.
            match ctx.stock.and_then(|_| super::material_top(ctx, frame)).filter(|s| *s > t + 1e-9) {
                Some(st) => {
                    why.push(format!("top {}: the stock's top (above the part's, {})", num(st), num(t)));
                    st
                }
                None => {
                    why.push(format!(
                        "top {}: the part's top{}",
                        num(t),
                        if exact { "" } else { " (its exact box turned into the operation's frame: at or above the part)" }
                    ));
                    t
                }
            }
        }
    };
    let bottom = match g.bottom {
        Some(b) => b,
        None => {
            let (_, low, _) = need("bottom")?;
            derived.push("bottom".into());
            match region {
                Region::Part => {
                    why.push(format!("bottom {}: the part's bottom", num(low)));
                    low
                }
                Region::Floor(regions) => match ag.geo.floor(frame, regions, top, low, tol) {
                    Ok(Floor { lowest: Some(z), through_at }) if z < top - 1e-6 => {
                        why.push(format!("bottom {}: the lowest floor of the part inside the region", num(z)));
                        if let Some(q) = through_at {
                            why.push(format!(
                                "the region also goes through the part (at {}, {}): a hole there is not cut through by this operation",
                                num(q.x),
                                num(q.y)
                            ));
                        }
                        z
                    }
                    Ok(Floor { lowest: Some(z), through_at: None }) => {
                        return Err(refuse_op(
                            ctx,
                            op,
                            "heights",
                            format!("the part is solid up to the top ({}) everywhere in the region: there is nothing to cut", num(z)),
                            None,
                            "give `heights.bottom`, or draw the region over a pocket the model has".into(),
                        ));
                    }
                    // No floor below the top, and somewhere no material at all: the region
                    // goes through the part (or off its edge).
                    Ok(Floor { .. }) => {
                        why.push(format!("bottom {}: the part's bottom (the region goes through the part)", num(low)));
                        low
                    }
                    Err(e) => {
                        return Err(refuse_op(
                            ctx,
                            op,
                            "heights",
                            format!("the region's floor could not be worked out ({e})"),
                            None,
                            "give `heights.bottom`".into(),
                        ));
                    }
                },
            }
        }
    };
    let part_top = range.as_ref().map_or(top, |r| r.0);
    let fixture_top = ag.geo.fixture_top(frame);
    let obstacle = fixture_top.map_or(part_top, |f| f.max(part_top));
    let retract = match g.retract {
        Some(r) => r,
        None => {
            derived.push("retract".into());
            why.push(format!("retract {}: top + {}", num(top + ag.retract_above), num(ag.retract_above)));
            top + ag.retract_above
        }
    };
    let clearance = match g.clearance {
        Some(c) => c,
        None => {
            let c = (top.max(obstacle) + ag.clearance_above).max(retract);
            derived.push("clearance".into());
            match &ag.turn {
                Some((reach, axis)) if reach + ag.clearance_above > c => {
                    why.push(format!(
                        "clearance {}: {} above the part's reach from the {axis} ({}), where the post turns the part under the tool",
                        num(reach + ag.clearance_above),
                        num(ag.clearance_above),
                        num(*reach)
                    ));
                    reach + ag.clearance_above
                }
                _ => {
                    why.push(format!(
                        "clearance {}: {} above the {}",
                        num(c),
                        num(ag.clearance_above),
                        if fixture_top.is_some_and(|f| f > part_top) { "fixtures' top" } else { "part's top" }
                    ));
                    c
                }
            }
        }
    };
    if !([clearance, retract, top, bottom].iter().all(|v| v.is_finite() && v.abs() <= super::MAX_COORD)) {
        return Err(refuse_op(ctx, op, "heights", "the heights must be finite numbers within 1000 m".into(), None, "give finite heights".into()));
    }
    if top < bottom {
        return Err(refuse_op(
            ctx,
            op,
            "heights",
            format!("the bottom ({}) is above the top ({})", num(bottom), num(top)),
            None,
            format!("give `heights.bottom` at most {} or `heights.top` at least {}", num(top), num(bottom)),
        ));
    }
    if retract < top {
        return Err(refuse_op(
            ctx,
            op,
            "heights",
            format!("the retract height ({}) is below the top ({})", num(retract), num(top)),
            None,
            format!("raise `heights.retract` to at least {} (top + {}), or leave it out", num(top + ag.retract_above), num(ag.retract_above)),
        ));
    }
    if clearance < retract {
        return Err(refuse_op(
            ctx,
            op,
            "heights",
            format!("the clearance ({}) is below the retract height ({})", num(clearance), num(retract)),
            None,
            format!("raise `heights.clearance` to at least {}, or leave it out", num(retract)),
        ));
    }
    // A clearance at or below the part's top would rapid across the part. One below a fixture's
    // top or inside what the part reaches as the table turns is left to the job's own checks,
    // which hold each rapid against the fixtures and each turn against the part where the tool
    // is (a clamp off to one side, or a slender section, is no reason to refuse).
    if let Ok((_, _, true)) = range
        && clearance <= part_top + 1e-6
    {
        let at = frame.to_world(Vec3::new(0.0, 0.0, part_top));
        return Err(refuse_op(
            ctx,
            op,
            "clearance",
            format!("the clearance height ({}) does not clear the part's top ({}): rapids there would run into it", num(clearance), num(part_top)),
            Some([at.x, at.y, at.z]),
            format!(
                "raise `heights.clearance` to at least {} (the part's top is {}), or leave it out ({} is the default)",
                num(part_top + ag.retract_above),
                num(part_top),
                num(obstacle.max(top) + ag.clearance_above)
            ),
        ));
    }
    let h = super::heights(&HeightsSpec { clearance, retract, top, bottom });
    ag.used.borrow_mut().insert(ag.index.get(), (h, *frame));
    if !derived.is_empty() {
        ag.auto.borrow_mut().insert(ag.index.get());
        ag.derived.borrow_mut().push(DerivedHeights {
            op_index: ag.index.get(),
            op: op.to_string(),
            clearance,
            retract,
            top,
            bottom,
            derived,
            why: why.join("; "),
        });
    }
    Ok(h)
}

/// The widest tool (diameter, mm) that fits inside `regions` with `leave` on the walls.
fn widest_fit(regions: &[Region2], leave: f64, tol: f64) -> Option<f64> {
    let fits = |r: f64| offset_regions(regions, r + leave, tol).is_ok_and(|l| !l.is_empty());
    if !fits(1e-4) {
        return None;
    }
    let (mut lo, mut hi) = (1e-4, 1.0);
    while fits(hi) && hi < 1e6 {
        lo = hi;
        hi *= 2.0;
    }
    for _ in 0..50 {
        if hi - lo < 1e-4 {
            break;
        }
        let mid = 0.5 * (lo + hi);
        if fits(mid) { lo = mid } else { hi = mid }
    }
    Some(2.0 * lo)
}

/// Whether `tool` fits inside `regions` with `leave` on the walls.
pub(super) fn fits(regions: &[Region2], tool: &Tool, leave: f64, tol: f64) -> bool {
    offset_regions(regions, tool.radius() + leave, tol).is_ok_and(|l| !l.is_empty())
}

/// The error of a pocket or inside contour whose tool does not fit: the widest that does.
pub(super) fn fit_error(ctx: &Ctx<'_>, op: &str, regions: &[Region2], tool: &Tool, leave: f64, tol: f64, e: solidcraft_cam::CamError) -> EngineError {
    if !e.to_string().contains("does not fit") {
        return ctx.bad(op, e);
    }
    let fix = match widest_fit(regions, leave, tol) {
        Some(d) => format!(
            "use a tool of at most {} mm across (the widest circle that fits the profile{} is {} mm): `tool: {{max_diameter: {}}}` picks one",
            num((d * 100.0).floor() / 100.0),
            if leave > 0.0 { " with `stock_to_leave`" } else { "" },
            num((d * 1000.0).round() / 1000.0),
            num((d * 100.0).floor() / 100.0)
        ),
        None => "the profile has no room for any tool: check the polygon or sketch".into(),
    };
    refuse_op(ctx, op, "fit", format!("tool {} ({} mm) does not fit the profile", tool.number, num(tool.diameter)), None, fix)
}

/// A tool as replies show it.
fn tool_label(t: &Tool) -> String {
    let mut s = format!("T{} {} mm {}, {} mm flutes", t.number, num(t.diameter), t.kind.label(), num(t.flute_length));
    if !t.description.is_empty() {
        s.push_str(&format!(" ({})", t.description));
    }
    s
}

/// The job's tools as a list for a fix.
fn tool_list(tools: &[Tool]) -> String {
    let mut v: Vec<String> = tools.iter().take(30).map(|t| format!("{} ({} mm {})", t.number, num(t.diameter), t.kind.label())).collect();
    if tools.len() > 30 {
        v.push(format!("… {} more", tools.len() - 30));
    }
    v.join(", ")
}

fn kind_of(name: &str) -> Option<ToolKind> {
    serde_json::from_value(Value::String(name.trim().to_ascii_lowercase())).ok()
}

/// How deep below the material beside it `t` can cut before its holder's face gets there: its
/// stickout (its flutes when it gives none), or its flutes when its shank is wider than its cut.
fn reach(t: &Tool) -> f64 {
    if t.shank_diameter.is_some_and(|s| s > t.diameter + 1e-9) { t.flute_length } else { t.stickout.unwrap_or(t.flute_length).max(t.flute_length) }
}

/// The tool `spec` picks from the job's tools, and why. With `depth` (a 2D operation's cut, top
/// to bottom), the tools that reach it come first.
fn pick<'t>(ctx: &'t Ctx<'_>, op: &str, spec: &ToolSpec, fit: Option<&dyn Fn(&Tool) -> bool>, depth: Option<f64>) -> Result<(&'t Tool, String)> {
    let nums = [spec.diameter, spec.max_diameter, spec.min_diameter, spec.min_flute_length, spec.min_stickout];
    if !nums.iter().all(|v| v.is_none_or(|x| x.is_finite() && x > 0.0 && x <= 10_000.0)) {
        return Err(refuse_op(ctx, op, "tool", "a tool spec's sizes must be above 0".into(), None, "give sizes in mm above 0".into()));
    }
    let kind = match &spec.kind {
        None => None,
        Some(k) => Some(kind_of(k).ok_or_else(|| {
            refuse_op(
                ctx,
                op,
                "tool",
                format!("unknown tool kind `{k}`"),
                None,
                "give `kind` as flat_endmill, ball_endmill, bull_nose or drill".into(),
            )
        })?),
    };
    let sized = |t: &Tool| {
        kind.map_or(t.kind != ToolKind::Drill, |k| t.kind == k)
            && spec.diameter.is_none_or(|d| (t.diameter - d).abs() <= 1e-6)
            && spec.max_diameter.is_none_or(|d| t.diameter <= d + 1e-9)
            && spec.min_diameter.is_none_or(|d| t.diameter >= d - 1e-9)
            && spec.min_flute_length.is_none_or(|l| t.flute_length >= l - 1e-9)
            && spec.min_stickout.is_none_or(|l| t.stickout.unwrap_or(t.flute_length) >= l - 1e-9)
    };
    let sized_tools: Vec<&Tool> = ctx.tools.iter().filter(|t| sized(t)).collect();
    let mut matches: Vec<&Tool> = sized_tools.iter().copied().filter(|t| fit.is_none_or(|f| f(t))).collect();
    let reaches = |t: &Tool| depth.is_none_or(|d| reach(t) + 1e-9 >= d);
    matches.sort_by(|a, b| {
        reaches(b)
            .cmp(&reaches(a))
            .then(b.diameter.total_cmp(&a.diameter))
            .then(a.flute_length.total_cmp(&b.flute_length))
            .then(a.number.cmp(&b.number))
    });
    let Some(first) = matches.first().copied() else {
        let (error, fix) = if sized_tools.is_empty() {
            let same_kind: Vec<Tool> = ctx.tools.iter().filter(|t| kind.is_none_or(|k| t.kind == k)).cloned().collect();
            (
                format!("no tool of the job is {spec}"),
                if same_kind.is_empty() {
                    format!("ask for another kind, or add such a tool; the job's tools: {}", tool_list(ctx.tools))
                } else {
                    format!("loosen the spec or add a tool; the job's tools of that kind: {}", tool_list(&same_kind))
                },
            )
        } else {
            let n: Vec<String> = sized_tools.iter().map(|t| format!("T{} ({} mm)", t.number, num(t.diameter))).collect();
            (
                format!("the tools that are {spec} ({}) do not fit the profile", n.join(", ")),
                "ask for a smaller `max_diameter`, or widen the profile".into(),
            )
        };
        return Err(refuse_op(ctx, op, "tool", error, None, fix));
    };
    let mut why = if spec.diameter.is_some() {
        format!("the {} of exactly {} mm with the shortest flutes", first.kind.label(), num(first.diameter))
    } else {
        format!("the largest {} that is {spec}", if kind.is_some() { first.kind.label() } else { "milling tool" })
    };
    if fit.is_some() {
        why.push_str(" and fits the profile");
    }
    if let Some(d) = depth {
        if reaches(first) {
            why.push_str(&format!(", reaching the {} mm deep cut", num(d)));
        } else {
            why.push_str(&format!(
                "; none of the matches reaches the {} mm deep cut (this one stands out {} mm): give a longer tool, or cut less deep",
                num(d),
                num(reach(first))
            ));
        }
    }
    let others: Vec<String> = matches.iter().skip(1).take(10).map(|t| format!("T{}", t.number)).collect();
    if !others.is_empty() {
        why.push_str(&format!("; also matched: {}", others.join(", ")));
    }
    Ok((first, why))
}

/// `c`'s tool, spindle speed and feeds: the tool by number or spec (`fit`: whether a tool fits
/// the operation's profiles), its cutting data with the job's feeds policy.
pub(super) fn resolve(ctx: &Ctx<'_>, c: &Common<'_>, fit: Option<&dyn Fn(&Tool) -> bool>) -> Result<Resolved> {
    resolve_for(ctx, c, fit, None)
}

/// [`resolve`] for a cut `depth` deep (a 2D operation's top to bottom): a tool asked for by
/// what it is is one that reaches it where the job has one.
pub(super) fn resolve_for(ctx: &Ctx<'_>, c: &Common<'_>, fit: Option<&dyn Fn(&Tool) -> bool>, depth: Option<f64>) -> Result<Resolved> {
    let ag = ctx.agent;
    ag.name_op(c.name);
    let (tool, why) = match c.tool {
        ToolRef::Number(n) => match ctx.tools.iter().find(|t| t.number == *n) {
            Some(t) => (t, "given by number".to_string()),
            None => {
                let left = ag.left_out.iter().find(|(m, _)| m == n).map_or(String::new(), |(_, why)| format!(" ({why})"));
                return Err(refuse_op(
                    ctx,
                    c.name,
                    "tool",
                    format!("no tool number {n}{left}"),
                    None,
                    format!(
                        "use one of the job's tools: {}; or ask by what it is, e.g. `tool: {{kind: \"flat_endmill\", max_diameter: 6}}`",
                        tool_list(ctx.tools)
                    ),
                ));
            }
        },
        ToolRef::Spec(spec) => {
            ag.auto.borrow_mut().insert(ag.index.get());
            pick(ctx, c.name, spec, fit, depth)?
        }
    };
    let pol = ag.feeds;
    let op = c.name;
    let prefix = format!("operation {} `{op}`", ag.index.get());
    // The spindle speed.
    let rpm = match c.spindle_rpm {
        Some(r) => {
            if let Some([lo, hi]) = pol.spindle_rpm_range
                && !(lo..=hi).contains(&r)
            {
                return Err(refuse_op(
                    ctx,
                    op,
                    "feeds",
                    format!("`spindle_rpm` {} is outside the spindle's range {} to {}", num(r), num(lo), num(hi)),
                    None,
                    format!("give `spindle_rpm` from {} to {}, or leave it out to take the tool's", num(lo), num(hi)),
                ));
            }
            r
        }
        None => {
            let r = match (tool.spindle_rpm, pol.fallback) {
                (Some(r), _) => r,
                (None, Some(f)) => {
                    ag.note(format!(
                        "{prefix}: tool {} has no spindle speed: the profile's fallback {} rev/min is used",
                        tool.number,
                        num(f.spindle_rpm)
                    ));
                    f.spindle_rpm
                }
                (None, None) => {
                    return Err(refuse_op(
                        ctx,
                        op,
                        "feeds",
                        format!("tool {} has no spindle speed", tool.number),
                        None,
                        "give the operation `spindle_rpm`, the tool `spindle_rpm`, or the profile `default_feeds.fallback`".into(),
                    ));
                }
            };
            match pol.spindle_rpm_range {
                Some([lo, hi]) if !(lo..=hi).contains(&r) => {
                    let held = r.clamp(lo, hi);
                    ag.note(format!("{prefix}: tool {}'s {} rev/min is held to the spindle's range: {}", tool.number, num(r), num(held)));
                    held
                }
                _ => r,
            }
        }
    };
    if !(rpm > 0.0 && rpm <= 200_000.0) {
        return Err(refuse_op(
            ctx,
            op,
            "feeds",
            "the spindle speed must be above 0 and at most 200000 rev/min".into(),
            None,
            "give `spindle_rpm` above 0 and at most 200000".into(),
        ));
    }
    // The feeds.
    let feeds = match c.feeds {
        Some(f) => {
            for (v, max, key, limit) in [(f.cut, pol.max_cut, "feeds.cut", "max_cut"), (f.plunge, pol.max_plunge, "feeds.plunge", "max_plunge")] {
                if let Some(m) = max
                    && v > m
                {
                    return Err(refuse_op(
                        ctx,
                        op,
                        "feeds",
                        format!("`{key}` {} is above the machine's {}", num(v), num(m)),
                        None,
                        format!("lower `{key}` to at most {} (the profile's `default_feeds.{limit}`)", num(m)),
                    ));
                }
            }
            Feeds { cut: f.cut, plunge: f.plunge }
        }
        None => {
            let base = match (tool.feed, tool.plunge_feed, pol.fallback) {
                (Some(cut), Some(plunge), _) => Feeds { cut, plunge },
                (Some(cut), None, _) => Feeds { cut, plunge: cut },
                (None, _, Some(f)) => {
                    ag.note(format!("{prefix}: tool {} has no feed: the profile's fallback feeds are used", tool.number));
                    Feeds { cut: f.cut, plunge: f.plunge }
                }
                (None, _, None) => {
                    return Err(refuse_op(
                        ctx,
                        op,
                        "feeds",
                        format!("tool {} has no feed", tool.number),
                        None,
                        "give the operation `feeds: {cut, plunge}`, the tool `feed` and `plunge_feed`, or the profile `default_feeds.fallback`"
                            .into(),
                    ));
                }
            };
            let mut f = Feeds { cut: base.cut * pol.cut_scale.unwrap_or(1.0), plunge: base.plunge * pol.plunge_scale.unwrap_or(1.0) };
            if let Some(m) = pol.max_cut
                && f.cut > m
            {
                ag.note(format!("{prefix}: the cutting feed {} is held to the machine's {}", num(f.cut), num(m)));
                f.cut = m;
            }
            if let Some(m) = pol.max_plunge
                && f.plunge > m
            {
                ag.note(format!("{prefix}: the plunge feed {} is held to the machine's {}", num(f.plunge), num(m)));
                f.plunge = m;
            }
            f
        }
    };
    if let Err(e) = feeds.validate() {
        return Err(refuse_op(ctx, op, "feeds", e.to_string(), None, "give `feeds: {cut, plunge}` above 0".into()));
    }
    ag.choices.borrow_mut().push(ToolChoice { op_index: ag.index.get(), op: op.to_string(), tool: tool.number, description: tool_label(tool), why });
    Ok(Resolved { tool: tool.clone(), rpm, feeds })
}

/// The distance from `p` to the segment `a`–`b`.
fn segment_distance(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b - a;
    let l2 = ab.dot(ab);
    let t = if l2 > 0.0 { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
    (a + ab * t - p).len()
}

/// The program line of the cutting move that passes nearest `tip` (work coordinates), within 1 mm.
fn line_near(text: &str, machine: &Machine, tip: Vec3) -> Option<usize> {
    let blocks = parse5(text, &machine.letters()).ok()?;
    let mut prev: Option<Vec3> = None;
    let mut best: Option<(f64, usize)> = None;
    for b in &blocks {
        let (line, linear, rotary, tcp, cut) = match b {
            JointWords::Rapid { line, linear, rotary, tcp } => (*line, *linear, rotary, *tcp, false),
            JointWords::Feed { line, linear, rotary, tcp, .. } | JointWords::Arc { line, linear, rotary, tcp, .. } => {
                (*line, *linear, rotary, *tcp, true)
            }
            JointWords::Dwell { .. } => continue,
        };
        let p = if tcp { linear } else { forward(machine, &Joints { linear, rotary: rotary.clone() }).tip };
        if cut && let Some(a) = prev {
            let d = segment_distance(tip, a, p);
            if best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, line));
            }
        }
        prev = Some(p);
    }
    best.filter(|(d, _)| *d <= 1.0).map(|(_, l)| l)
}

fn point(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

impl Agent {
    /// The refusal of operation `index`'s gouges (`on_part`: the cutter meets the part, not only
    /// a fixture).
    pub fn gouge_refusal(&self, index: usize, p: &Planned, hits: &[Hit], on_part: bool, text: &str, machine: &Machine) -> Refusal {
        let worst = hits.iter().copied().max_by(|a, b| a.distance.total_cmp(&b.distance));
        let tip = worst.map(|h| h.pose.tip);
        let d = worst.map_or(0.0, |h| h.distance);
        let r = &p.report;
        let fix = if !on_part {
            "the tool runs into a fixture, not the part: keep the operation's polygon or boundary clear of it, raise the clearance, or move the fixture"
                .to_string()
        } else {
            let mut f = match self.used(index) {
                Some((h, _)) => format!(
                    "the tool must stay {} mm further out along its axis here: raise `heights.bottom` to at least {} if the cut goes below the floor here, leave `stock_to_leave: {}`, or use a smaller tool where it meets a wall",
                    num(d),
                    num(h.bottom + d),
                    num((d * 100.0).ceil() / 100.0)
                ),
                None => format!(
                    "the tool must stay {} mm further out along its axis here: leave `stock_to_leave: {}`, check the operation's heights or radii, or use a smaller tool",
                    num(d),
                    num((d * 100.0).ceil() / 100.0)
                ),
            };
            f.push_str("; `gouge_check: false` only for an engraving the model does not show");
            f
        };
        Refusal {
            error: format!(
                "the {} cuts up to {} mm into the {} along {} moves",
                r.kind,
                num(d),
                if on_part { "part" } else { "fixtures" },
                hits.len()
            ),
            op: Some(OpRef { index, name: r.name.clone() }),
            check: "gouge".into(),
            at: tip.map(point),
            line: tip.and_then(|t| line_near(text, machine, t)),
            fix,
        }
    }

    /// The tool operation `index`'s shank and holder are checked with, and whether its holder is
    /// the least one it can have: where the operation's heights or tool were worked out and the
    /// tool gives no `holder`, one [`LEAST_HOLDER`] wider than its shank from its stickout up
    /// (a real one is wider still), so a cut worked out deeper than the tool reaches is seen.
    ///
    /// A shank no wider than the cut rides in the slot its flutes cut: it is checked shrunk by
    /// the job's tolerance, as the cutter is in the gouge check, so a shank as wide as the
    /// flutes (a tool that gives no `shank_diameter`) running down the wall it has just cut is
    /// no collision.
    pub fn holder_tool(&self, index: usize, tool: &Tool) -> (Tool, bool) {
        let mut t = tool.clone();
        let shank = tool.shank_diameter.unwrap_or(tool.diameter);
        if shank <= tool.diameter + 1e-9 {
            t.shank_diameter = Some((shank - 2.0 * self.tol).max(shank * 0.5));
        }
        if !tool.holder.is_empty() || !self.auto.borrow().contains(&index) {
            return (t, false);
        }
        t.holder = vec![HolderSection { diameter: shank + LEAST_HOLDER.0, length: LEAST_HOLDER.1 }];
        (t, true)
    }

    /// The refusal of operation `index`'s holder or shank collisions, `t` the tool as checked
    /// (`least`: with the least holder of [`Agent::holder_tool`]).
    #[allow(clippy::too_many_arguments)]
    pub fn holder_refusal(&self, index: usize, p: &Planned, t: &Tool, least: bool, hits: &[Hit], text: &str, machine: &Machine) -> Refusal {
        // The deepest hit: how far the tool must reach.
        let frame = self.used(index).map(|(_, f)| f);
        let top = frame.and_then(|f| self.geo.part_range(&f).ok().map(|(top, _, _)| top));
        let depth_of = |h: &Hit| match (frame, top) {
            (Some(f), Some(top)) => Some(top - f.to_local(h.pose.tip).z),
            _ => None,
        };
        let first = hits.iter().copied().max_by(|a, b| depth_of(a).unwrap_or(0.0).total_cmp(&depth_of(b).unwrap_or(0.0))).or(hits.first().copied());
        let tip = first.map(|h| h.pose.tip);
        let envelopes = t.envelopes();
        let env = first.and_then(|h| h.envelope).and_then(|i| envelopes.get(i)).copied();
        let stickout = t.stickout.unwrap_or(t.flute_length).max(t.flute_length);
        let shank = env.is_some_and(|e| e.top <= stickout + 1e-9);
        let depth = first.as_ref().and_then(depth_of);
        let fix = match (shank, depth) {
            (false, Some(d)) if least => format!(
                "tool {} gives no `holder`, so it is checked with the least it can have (1 mm wider than its shank, from where it stands out: {} mm{}), and the cut goes {} mm below the part's top: use a tool standing out at least {} mm, e.g. `tool: {{min_stickout: {}}}`, give tool {} its real `stickout` and `holder` if it stands out further, or cut less deep (`heights.bottom`)",
                t.number,
                num(stickout),
                if p.tool.stickout.is_none() { ", its flutes, as it gives no `stickout`" } else { "" },
                num(d),
                num(d + 1.0),
                num((d + 1.0).ceil()),
                t.number
            ),
            (true, Some(d)) => format!(
                "the shank meets the part: use a tool whose flutes reach at least {} mm (tool {} has {} mm), e.g. `tool: {{min_flute_length: {}}}`, or cut less deep here",
                num(d + 0.5),
                t.number,
                num(t.flute_length),
                num((d + 0.5).ceil())
            ),
            (false, Some(d)) => format!(
                "the holder meets the part: use a tool standing out at least {} mm (tool {} stands out {} mm), e.g. `tool: {{min_stickout: {}}}`, or a slimmer holder",
                num(d + 1.0),
                t.number,
                num(stickout),
                num((d + 1.0).ceil())
            ),
            (true, None) => "use a tool with longer flutes or a slimmer shank, or cut less deep here".into(),
            (false, None) => "use a tool that stands out further, or a slimmer holder".into(),
        };
        Refusal {
            error: format!(
                "tool {}'s {} touches the part or a fixture along {} moves",
                t.number,
                if shank {
                    "shank"
                } else if least {
                    "holder (the least it can have: none is given)"
                } else {
                    "holder"
                },
                hits.len()
            ),
            op: Some(OpRef { index, name: p.report.name.clone() }),
            check: "holder".into(),
            at: tip.map(point),
            line: tip.and_then(|t| line_near(text, machine, t)),
            fix,
        }
    }
}

/// The refusal of a failed check of the job's own (rapids, travel, the stock's bottom, the
/// simulation): its check is what the failure starts with, its operation the first one of
/// `ops` (job index and name) it names, its fix what follows the last `: ` of it.
pub(super) fn failure_refusal(failure: &str, ops: &[(usize, &str)]) -> Refusal {
    let check = failure.split(':').next().unwrap_or("check").trim().to_string();
    let op = ops.iter().find(|(_, n)| failure.contains(&format!("`{n}`"))).map(|(i, n)| OpRef { index: *i, name: (*n).to_string() });
    let fix = failure.rsplit_once(": ").map_or_else(|| "see the error".to_string(), |(_, f)| f.to_string());
    Refusal { error: failure.to_string(), op, check, at: None, line: None, fix }
}

/// The refusal of a failed read-back.
pub(super) fn readback_refusal(rb: &ReadbackReport, post: &Post5) -> Refusal {
    Refusal {
        error: format!(
            "the written program is up to {} mm and {}° from its toolpaths (the post's precision allows {} mm and {}°)",
            num(rb.max_tip_deviation),
            num(rb.max_axis_deviation_deg),
            num(rb.allowed_tip),
            num(rb.allowed_axis_deg)
        ),
        op: None,
        check: "readback".into(),
        at: None,
        line: None,
        fix: format!(
            "raise the post's `base.decimals` (now {}) or `angle_decimals` (now {}); if they are enough, the post does not match the machine: \
             compare `cam.check` of the program with the job's reply",
            post.base.decimals, post.angle_decimals
        ),
    }
}

/// One line for an operation of the compact reply.
fn op_line(i: usize, r: &OperationReport, derived: Option<&DerivedHeights>, choice: Option<&ToolChoice>) -> String {
    let mut s = format!("{i} `{}`: {}", r.name, r.kind);
    if let Some(a) = r.a_deg {
        s.push_str(&format!(" at A{}", num(a)));
    }
    match choice {
        Some(c) => s.push_str(&format!(", {}", c.description.split(" (").next().unwrap_or(&c.description))),
        None => s.push_str(&format!(", T{}", r.tool)),
    }
    s.push_str(&format!(", {} moves, {} min", r.moves, num(r.cut_minutes)));
    if let Some(d) = derived {
        s.push_str(&format!(
            ", heights {}/{}/{}/{} (worked out: {})",
            num(d.clearance),
            num(d.retract),
            num(d.top),
            num(d.bottom),
            d.derived.join(", ")
        ));
    }
    if r.gouge > 0.0 {
        s.push_str(&format!(", gouge {} mm", num(r.gouge)));
    }
    if r.holder_collisions > 0 {
        s.push_str(&format!(", {} holder collisions", r.holder_collisions));
    }
    for w in &r.warnings {
        s.push_str(&format!(", {w}"));
    }
    s
}

/// The reply of `cam.job`: the full one, or the compact one.
pub(super) fn reply_value(reply: CamJobReply, compact: bool) -> Result<Value> {
    let out = if compact {
        let operations = reply
            .operations
            .iter()
            .enumerate()
            .map(|(k, r)| {
                // Numbered, and looked up, by the job operation it comes from.
                let i = reply.op_of.get(k).copied().unwrap_or(k);
                op_line(i, r, reply.derived_heights.iter().find(|d| d.op_index == i), reply.tool_choices.iter().find(|c| c.op_index == i))
            })
            .collect();
        CamJobOutput::Compact(Box::new(CamJobCompactReply {
            compact: true,
            passed: reply.passed,
            dry_run: reply.dry_run,
            failures: reply.failure_details,
            operations,
            tools: reply.tool_choices,
            total_minutes: reply.readback.cut_minutes,
            program_lines: reply.program_lines,
            outputs: reply.outputs,
            // Everything but how the bodies were tessellated, which every job says.
            notes: reply.notes.into_iter().filter(|n| !n.starts_with(super::TESSELLATION_NOTE)).collect(),
            param_warnings: reply.param_warnings,
        }))
    } else {
        CamJobOutput::Full(Box::new(reply))
    };
    serde_json::to_value(out).map_err(|e| EngineError::Other(e.to_string()))
}
