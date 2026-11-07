//! The open CAM job (a child of `cmd/cam.rs`; state in `crate::cam_job_state`): `cam.job.new`,
//! `open`, `save`, `reload`, `close`, `get`, `edit`, `undo` and `redo`, `cam.op.add`, `edit`,
//! `remove`, `move`, `duplicate` and `enable`, `cam.library.list` and `view.manufacture_stage`.
//!
//! A session keeps at most one job open. Its edits are its own history, never the design's
//! (`.noundo()`): every successful edit is one step, and every edit is read with the job's
//! schema before it is kept, so a person cannot make a job an agent cannot run. A command that
//! would drop unsaved edits refuses without `discard`; one that would write over a file changed
//! outside SolidCraft refuses without `overwrite`. None opens a dialog.

use std::path::Path;

use serde_json::{Map, Value, json};

use super::{CommandSpec, MAX_OPERATIONS, agent, parse};
use crate::cam_job_state::{self, Disk, OP_TYPES, OpenJob, Outside, STAGES, op_enabled, op_name, op_types};
use crate::params::bad;
use crate::schema::cam::CamJob;
use crate::schema::cam_job_edit::{
    CamJobCloseParams, CamJobCloseReply, CamJobEditParams, CamJobGetParams, CamJobGetReply, CamJobHistoryParams, CamJobHistoryReply, CamJobNewParams,
    CamJobOpenParams, CamJobReloadParams, CamJobReloadReply, CamJobSaveParams, CamJobSaveReply, CamLibraryListParams, CamLibraryListReply,
    CamOpAddParams, CamOpDuplicateParams, CamOpEditParams, CamOpEnableParams, CamOpMoveParams, CamOpParams, CamOpReply, JobSummary, LibraryFile,
    LibraryProfile, LibraryTools, ManufactureStageParams, ManufactureStageReply, OpSelect, OpSummary,
};
use crate::schema::{TypedContract, output_schema_for};
use crate::{EngineError, Result, Session};

const NEW: TypedContract<CamJobNewParams> = TypedContract::new();
const OPEN: TypedContract<CamJobOpenParams> = TypedContract::new();
const SAVE: TypedContract<CamJobSaveParams> = TypedContract::new();
const RELOAD: TypedContract<CamJobReloadParams> = TypedContract::new();
const CLOSE: TypedContract<CamJobCloseParams> = TypedContract::new();
const GET: TypedContract<CamJobGetParams> = TypedContract::new();
const EDIT: TypedContract<CamJobEditParams> = TypedContract::new();
const HISTORY: TypedContract<CamJobHistoryParams> = TypedContract::new();
const LIBRARY: TypedContract<CamLibraryListParams> = TypedContract::new();
const OP_ADD: TypedContract<CamOpAddParams> = TypedContract::new();
const OP_EDIT: TypedContract<CamOpEditParams> = TypedContract::new();
const OP: TypedContract<CamOpParams> = TypedContract::new();
const OP_MOVE: TypedContract<CamOpMoveParams> = TypedContract::new();
const OP_DUPLICATE: TypedContract<CamOpDuplicateParams> = TypedContract::new();
const OP_ENABLE: TypedContract<CamOpEnableParams> = TypedContract::new();
const STAGE: TypedContract<ManufactureStageParams> = TypedContract::new();

/// Longest name of a job or an operation (characters).
const MAX_NAME: usize = 200;
/// Deepest nesting of a job's JSON.
const MAX_DEPTH: usize = 32;
/// Most keys one edit sets or unsets.
const MAX_KEYS: usize = 200;
/// Most files of each kind a library folder lists.
const MAX_LISTED: usize = 200;
/// The refusal of a command that needs an open job.
pub(super) const NO_JOB: &str = "open or create a job first (cam.job.open, cam.job.new)";
/// The 3-axis types an indexed operation can hold.
const INDEXABLE: &[&str] = &[
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

macro_rules! job_cmd {
    ($id:literal, $label:literal, $run:path, $contract:expr, $reply:ty, $doc:literal) => {
        CommandSpec::new($id, $label, crate::checked!($id, $run))
            .noundo()
            .doc_only()
            .contract($contract)
            .output(output_schema_for::<$reply>)
            .params($doc)
    };
}

pub(crate) static COMMANDS: &[CommandSpec] = &[
    job_cmd!(
        "cam.job.new",
        "New Machining Job",
        new_job,
        &NEW,
        JobSummary,
        "path?: .json (default <design>.cam.json beside the saved design); name?; profile_path?: machine profile; discard?: drop an open \
         job's unsaved edits → the job summary: {path?, name, dirty, revision, machine, post, profile?, tools, stock?, origin, holding?, \
         bodies, fixtures, operations: [{index, name, type, label_id, tool?, a_deg, enabled, status, minutes?}]}. Nothing is written \
         until cam.job.save; refused when a file is already at path (open it instead)"
    ),
    job_cmd!(
        "cam.job.open",
        "Open Machining Job",
        open_job,
        &OPEN,
        JobSummary,
        "path: job .json; discard?: drop an open job's unsaved edits → the job summary (as cam.job.new). A file that does not read as a \
         job is refused with the key and line"
    ),
    job_cmd!(
        "cam.job.save",
        "Save Machining Job",
        save_job,
        &SAVE,
        CamJobSaveReply,
        "path?: save as this .json; overwrite?: write over a file changed outside SolidCraft (or another file at path) → path, bytes, \
         written. Keys in the job's order, the file's own indent, a final newline"
    ),
    job_cmd!(
        "cam.job.reload",
        "Reload Machining Job",
        reload_job,
        &RELOAD,
        CamJobReloadReply,
        "discard?: drop unsaved edits → summary, changes: [what changed, one line each]. One job undo step"
    ),
    job_cmd!("cam.job.close", "Close Machining Job", close_job, &CLOSE, CamJobCloseReply, "discard?: drop unsaved edits → closed"),
    job_cmd!(
        "cam.job.get",
        "Machining Job State",
        get_job,
        &GET,
        CamJobGetReply,
        "job?: include the JSON → open, outside: none | changed | gone, undo: [labels], redo: [labels], generated, summary?, job?"
    ),
    job_cmd!(
        "cam.job.edit",
        "Edit Machining Job",
        edit_job,
        &EDIT,
        JobSummary,
        "set?: {key or dotted path: value} (job keys: machine, post, post_path, origin, stock, tool_library_path, tolerance, …); unset?: \
         [key or dotted path] → the summary. One job undo step; refused, changing nothing, when the job would not read"
    ),
    job_cmd!("cam.job.undo", "Undo Machining Job Edit", undo_job, &HISTORY, CamJobHistoryReply, "→ step: the label undone, summary"),
    job_cmd!("cam.job.redo", "Redo Machining Job Edit", redo_job, &HISTORY, CamJobHistoryReply, "→ step: the label redone, summary"),
    job_cmd!(
        "cam.library.list",
        "Machining Library",
        library_list,
        &LIBRARY,
        CamLibraryListReply,
        "folder?: a library folder (profiles/, posts/, tools/, machines/; default: found above the open job's folder) → folder, \
         profiles: [{path, name, description, machine}], posts: [{path, name}] (built-ins first, path empty), tool_libraries: [{path, \
         tools, left_out}], machines, workholdings, machine_models: [{path, name}]"
    ),
    job_cmd!(
        "cam.op.add",
        "Add Machining Operation",
        op_add,
        &OP_ADD,
        CamOpReply,
        "op: {type, …} (tool left out: {}, the job's best tool for it; required keys get starting values; name default: the type's and a \
         number); at?: index; a_deg?: index to this A angle → index, name, summary. One job undo step"
    ),
    job_cmd!(
        "cam.op.edit",
        "Edit Machining Operation",
        op_edit,
        &OP_EDIT,
        CamOpReply,
        "op: index | name; set?: {dotted path: value} (an indexed operation's inner keys); unset?: [dotted path]; rename?; a_deg?: \
         angle (0: not indexed) | [angles] | {count, start_deg?} → index, name, summary. One job undo step; refused, changing nothing, \
         when the job would not read"
    ),
    job_cmd!("cam.op.remove", "Remove Machining Operation", op_remove, &OP, CamOpReply, "op: index | name → name, summary"),
    job_cmd!("cam.op.move", "Move Machining Operation", op_move, &OP_MOVE, CamOpReply, "op: index | name; to: index → index, name, summary"),
    job_cmd!(
        "cam.op.duplicate",
        "Duplicate Machining Operation",
        op_duplicate,
        &OP_DUPLICATE,
        CamOpReply,
        "op: index | name; to?: index (default right after it) → index, name (`<name> copy`), summary"
    ),
    job_cmd!(
        "cam.op.enable",
        "Turn Machining Operation On or Off",
        op_enable,
        &OP_ENABLE,
        CamOpReply,
        "op: index | name; enabled: bool → index, name, summary. Off keeps it in the job, neither planned nor written"
    ),
    job_cmd!(
        "view.manufacture_stage",
        "Manufacture Stage",
        stage,
        &STAGE,
        ManufactureStageReply,
        "stage: setup | operations | check | program → stage. The Manufacture workspace shows that stage"
    ),
];

/// The open job, or the refusal.
pub(super) fn need<'a>(s: &'a mut Session, cmd: &str) -> Result<&'a mut OpenJob> {
    s.cam_job.as_deref_mut().ok_or_else(|| bad(cmd, NO_JOB))
}

fn to_value<T: serde::Serialize>(v: T) -> Result<Value> {
    serde_json::to_value(v).map_err(|e| EngineError::Other(e.to_string()))
}

/// Refused when a job is open with unsaved edits and the call does not say to drop them.
fn may_drop(s: &Session, cmd: &str, discard: Option<bool>) -> Result<()> {
    match s.cam_job.as_deref() {
        Some(open) if open.dirty() && !discard.unwrap_or(false) => {
            Err(bad(cmd, "the open job has unsaved changes: save it (cam.job.save) or give `discard: true` to drop them"))
        }
        _ => Ok(()),
    }
}

/// A job file's path: not empty, a `.json` file, in the folders the session may use.
fn job_path(cmd: &str, path: &str) -> Result<String> {
    let path = path.trim();
    if path.is_empty() || path.chars().count() > 4096 || path.chars().any(char::is_control) {
        return Err(bad(cmd, "`path` must be a file path"));
    }
    if !path.to_ascii_lowercase().ends_with(".json") {
        return Err(bad(cmd, "a job file is a `.json` file"));
    }
    solidcraft_io::vfs::check(path).map_err(|e| bad(cmd, format!("{path}: {e}")))?;
    Ok(path.to_string())
}

fn check_name(cmd: &str, what: &str, name: &str) -> Result<()> {
    if name.trim().is_empty() || name.chars().count() > MAX_NAME || name.chars().any(char::is_control) {
        return Err(bad(cmd, format!("{what} must be 1 to {MAX_NAME} characters, without control characters")));
    }
    Ok(())
}

/// A job read from `path`: its JSON, what the file held and its indent. Refused, with the key
/// and line, when it does not read as a job.
fn read_job(cmd: &str, path: &str) -> Result<(Value, Disk, usize)> {
    let bytes = solidcraft_io::vfs::read(path, cam_job_state::MAX_FILE_BYTES).map_err(|e| bad(cmd, format!("{path}: {e}")))?;
    let text = std::str::from_utf8(&bytes).map_err(|_| bad(cmd, format!("{path}: not UTF-8 text")))?;
    let value: Value = serde_json::from_str(text).map_err(|e| bad(cmd, format!("{path}: not valid JSON: {e}")))?;
    if !value.is_object() {
        return Err(bad(cmd, format!("{path}: a job is a JSON object")));
    }
    if super::picks::has_refs(&value) {
        // Picks and heights from faces are read without the model first.
        let plain = super::picks::placeholders(&value).map_err(|e| bad(cmd, format!("{path}: not a job (docs/cam-jobs.md): {e}")))?;
        if let Err(e) = serde_path_to_error::deserialize::<_, CamJob>(&plain) {
            let problem = job_problem(&plain, &e.path().to_string(), &e.inner().to_string());
            return Err(bad(cmd, format!("{path}: not a job (docs/cam-jobs.md): {problem}")));
        }
    } else {
        let mut de = serde_json::Deserializer::from_str(text);
        if let Err(e) = serde_path_to_error::deserialize::<_, CamJob>(&mut de) {
            let at = format!("line {} column {}", e.inner().line(), e.inner().column());
            let problem = job_problem(&value, &e.path().to_string(), &e.inner().to_string());
            return Err(bad(cmd, format!("{path}: not a job (docs/cam-jobs.md), {at}: {problem}")));
        }
    }
    check_job(cmd, &value)?;
    let disk = Disk { stamp: solidcraft_io::vfs::stamp(path), hash: cam_job_state::hash_bytes(&bytes) };
    Ok((value, disk, cam_job_state::detect_indent(text)))
}

/// Why operation `op` does not read, down to its key (the job's error stops at the operation,
/// as an operation is read by its `type`).
fn op_problem(op: &Value) -> Option<(String, String)> {
    use crate::schema::cam::*;
    use crate::schema::cam_adaptive::{AdaptiveOp, AdaptiveRoughingOp};
    use crate::schema::cam_stock::FaceOp;
    let ty = op.get("type")?.as_str()?;
    let mut body = op.as_object()?.clone();
    body.remove("type");
    let body = Value::Object(body);
    macro_rules! read_as {
        ($($name:literal => $t:ty),* $(,)?) => {
            match ty {
                $($name => serde_path_to_error::deserialize::<_, $t>(&body).err().map(|e| (e.path().to_string(), e.into_inner().to_string())),)*
                _ => None,
            }
        };
    }
    let (path, why) = read_as!(
        "adaptive" => AdaptiveOp, "adaptive_roughing" => AdaptiveRoughingOp, "contour" => ContourOp, "pocket" => PocketOp,
        "parallel" => ParallelOp, "roughing" => RoughingOp, "waterline" => WaterlineOp, "drill" => DrillOp, "scallop" => ScallopOp,
        "steep_shallow" => SteepShallowOp, "pencil" => PencilOp, "rest_finish" => RestFinishOp, "indexed" => IndexedOp,
        "rotary_finish" => RotaryFinishOp, "rotary_roughing" => RotaryRoughingOp, "wrap" => WrapOp, "bore" => BoreOp,
        "thread_mill" => ThreadMillOp, "face" => FaceOp, "chamfer" => ChamferOp, "wrap_pocket" => WrapPocketOp,
        "wrap_contour" => WrapContourOp, "indexed_pattern" => IndexedPatternOp,
    )?;
    // Inside a wrapper, the operation it holds says more.
    if path.starts_with("operation")
        && let Some((deeper, why)) = op.get("operation").and_then(op_problem)
    {
        return Some((if deeper.is_empty() { "operation".into() } else { format!("operation.{deeper}") }, why));
    }
    Some((if path == "." { String::new() } else { path }, why))
}

/// The job's read error `path: why`, taken down to the key inside an operation.
fn job_problem(value: &Value, path: &str, why: &str) -> String {
    let k = path.strip_prefix("operations[").and_then(|r| r.strip_suffix(']')).and_then(|k| k.parse::<usize>().ok());
    match k.and_then(|k| value.get("operations")?.get(k)).and_then(op_problem) {
        Some((key, why)) if !key.is_empty() => format!("`{path}.{key}`: {why}"),
        Some((_, why)) => format!("`{path}`: {why}"),
        None => format!("`{path}`: {why}"),
    }
}

/// Refused when `value` is not a job that reads, or is too large.
fn check_job(cmd: &str, value: &Value) -> Result<()> {
    if cam_job_state::depth(value, MAX_DEPTH + 1) > MAX_DEPTH {
        return Err(bad(cmd, format!("the job is nested more than {MAX_DEPTH} levels deep")));
    }
    let n = value.get("operations").and_then(Value::as_array).map_or(0, Vec::len);
    if n > MAX_OPERATIONS {
        return Err(bad(cmd, format!("a job has at most {MAX_OPERATIONS} operations")));
    }
    if value.to_string().len() > cam_job_state::MAX_FILE_BYTES {
        return Err(bad(cmd, "the job would be larger than 16 MB"));
    }
    let plain;
    let value = if super::picks::has_refs(value) {
        plain = super::picks::placeholders(value).map_err(|e| bad(cmd, format!("the job would not read, so nothing changed: {e}")))?;
        &plain
    } else {
        value
    };
    if let Err(e) = serde_path_to_error::deserialize::<_, CamJob>(value) {
        let problem = job_problem(value, &e.path().to_string(), &e.inner().to_string());
        return Err(bad(cmd, format!("the job would not read, so nothing changed: {problem}")));
    }
    Ok(())
}

/// Keep `next` as the open job, one undo step `label`, once it reads as a job.
pub(super) fn commit(s: &mut Session, cmd: &str, label: String, next: Value) -> Result<()> {
    check_job(cmd, &next)?;
    let open = need(s, cmd)?;
    if open.value != next {
        open.change(label, next);
    }
    Ok(())
}

/// The design's file name without its extension, for a new job's name and file.
fn design_stem(s: &Session) -> Option<(std::path::PathBuf, String)> {
    let path = Path::new(s.path.as_deref()?);
    let stem = path.file_stem()?.to_string_lossy().to_string();
    Some((path.parent()?.to_path_buf(), stem))
}

fn new_job(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job.new";
    let params: CamJobNewParams = parse(cmd, p, "")?;
    may_drop(s, cmd, params.discard)?;
    let path = match &params.path {
        Some(given) => Some(job_path(cmd, given)?),
        None => design_stem(s).map(|(dir, stem)| dir.join(format!("{stem}.cam.json")).to_string_lossy().to_string()),
    };
    if let Some(path) = &path
        && solidcraft_io::vfs::exists(path)
    {
        return Err(bad(cmd, format!("a file is already at {path}: open it (cam.job.open) or give another `path`")));
    }
    let mut job = Map::new();
    if let Some(name) = &params.name {
        check_name(cmd, "`name`", name)?;
        job.insert("name".into(), json!(name));
    }
    if let Some(profile) = &params.profile_path {
        if profile.trim().is_empty() || profile.chars().any(char::is_control) {
            return Err(bad(cmd, "`profile_path` must be a file path"));
        }
        // Kept as the job will read it: beside the job's file when relative.
        job.insert("profile_path".into(), json!(profile.trim()));
    }
    job.insert("operations".into(), json!([]));
    let value = Value::Object(job);
    check_job(cmd, &value)?;
    s.cam_job = Some(Box::new(OpenJob::new(path, value, false, None, cam_job_state::DEFAULT_INDENT)));
    to_value(summary(s))
}

fn open_job(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job.open";
    let params: CamJobOpenParams = parse(cmd, p, "")?;
    let path = job_path(cmd, &params.path)?;
    may_drop(s, cmd, params.discard)?;
    let (value, disk, indent) = read_job(cmd, &path)?;
    s.cam_job = Some(Box::new(OpenJob::new(Some(path), value, true, Some(disk), indent)));
    to_value(summary(s))
}

fn save_job(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job.save";
    let params: CamJobSaveParams = parse(cmd, p, "")?;
    let overwrite = params.overwrite.unwrap_or(false);
    let given = params.path.as_deref().map(|p| job_path(cmd, p)).transpose()?;
    let open = need(s, cmd)?;
    let path = match (&given, &open.path) {
        (Some(p), _) | (None, Some(p)) => p.clone(),
        (None, None) => return Err(bad(cmd, "the job has no file yet: give `path`")),
    };
    let same = open.path.as_deref().is_some_and(|own| super::same_file(own, &path));
    if same {
        if open.outside() == Outside::Changed && !overwrite {
            return Err(bad(
                cmd,
                "the job file was changed outside SolidCraft since it was read: reload it (cam.job.reload {discard: true}) or give \
                 `overwrite: true` to keep these edits",
            ));
        }
    } else if solidcraft_io::vfs::exists(&path) && !overwrite {
        return Err(bad(cmd, format!("a file is already at {path}: give `overwrite: true` to replace it")));
    }
    let text = cam_job_state::write_job(&open.value, open.indent);
    let unchanged = same && !open.dirty() && open.disk.as_ref().is_some_and(|d| d.hash == cam_job_state::hash_bytes(text.as_bytes()));
    if !unchanged {
        super::write_file(&path, text.as_bytes()).map_err(|e| bad(cmd, format!("{path}: {e}")))?;
    }
    open.path = Some(path.clone());
    open.mark_saved(text.as_bytes());
    to_value(CamJobSaveReply { path, bytes: text.len(), written: !unchanged })
}

fn reload_job(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job.reload";
    let params: CamJobReloadParams = parse(cmd, p, "")?;
    may_drop(s, cmd, params.discard)?;
    let open = need(s, cmd)?;
    let Some(path) = open.path.clone() else { return Err(bad(cmd, "the job has no file to read again")) };
    let (value, disk, indent) = read_job(cmd, &path)?;
    let changes = cam_job_state::changes(&open.value, &value);
    if open.value != value {
        open.change("Reloaded: changed outside SolidCraft", value);
    }
    open.indent = indent;
    open.mark_saved(&[]);
    open.disk = Some(disk);
    to_value(CamJobReloadReply { summary: summary(s), changes })
}

fn close_job(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job.close";
    let params: CamJobCloseParams = parse(cmd, p, "")?;
    may_drop(s, cmd, params.discard)?;
    let closed = s.cam_job.take().is_some();
    to_value(CamJobCloseReply { closed })
}

fn get_job(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job.get";
    let params: CamJobGetParams = parse(cmd, p, "")?;
    let revision = s.model_stamp();
    let Some(open) = s.cam_job.as_deref_mut() else { return to_value(CamJobGetReply { outside: "none".into(), ..Default::default() }) };
    let outside = open.outside().key().to_string();
    let generated = open.generated.as_ref().is_some_and(|g| g.revision == open.revision && g.model_revision == revision);
    let reply = CamJobGetReply {
        open: true,
        outside,
        undo: open.undo.iter().rev().map(|st| st.label.clone()).collect(),
        redo: open.redo.iter().rev().map(|st| st.label.clone()).collect(),
        generated,
        job: params.job.unwrap_or(false).then(|| open.value.clone()),
        summary: None,
    };
    to_value(CamJobGetReply { summary: Some(summary(s)), ..reply })
}

/// Checked dotted paths: each part a key (or, into an array, an index).
fn path_parts<'a>(cmd: &str, path: &'a str) -> Result<Vec<&'a str>> {
    let parts: Vec<&str> = path.split('.').collect();
    if path.is_empty() || path.len() > 256 || parts.len() > MAX_DEPTH || parts.iter().any(|p| p.is_empty() || p.chars().any(char::is_control)) {
        return Err(bad(cmd, format!("`{}` is not a key or a dotted path of keys", path.chars().take(64).collect::<String>())));
    }
    Ok(parts)
}

/// Sets `path` in `root` to `v`, making the objects on the way.
fn set_path(cmd: &str, root: &mut Value, path: &str, v: Value) -> Result<()> {
    let parts = path_parts(cmd, path)?;
    let mut at = root;
    for (i, part) in parts.iter().enumerate() {
        let last = i + 1 == parts.len();
        at = match at {
            Value::Object(map) => {
                if last {
                    map.insert((*part).to_string(), v);
                    return Ok(());
                }
                map.entry((*part).to_string()).or_insert_with(|| Value::Object(Map::new()))
            }
            Value::Array(items) => {
                let k: usize = part.parse().map_err(|_| bad(cmd, format!("`{path}`: `{part}` is not a place in a list")))?;
                let len = items.len();
                let slot = items.get_mut(k).ok_or_else(|| bad(cmd, format!("`{path}`: the list has {len} items")))?;
                if last {
                    *slot = v;
                    return Ok(());
                }
                slot
            }
            _ => return Err(bad(cmd, format!("`{path}`: `{part}` is inside a value that is not an object"))),
        };
    }
    Ok(())
}

/// Removes `path` from `root` (nothing when it is not there).
fn unset_path(cmd: &str, root: &mut Value, path: &str) -> Result<()> {
    let parts = path_parts(cmd, path)?;
    let Some((last, way)) = parts.split_last() else { return Ok(()) };
    let mut at = root;
    for part in way {
        at = match at {
            Value::Object(map) => match map.get_mut(*part) {
                Some(v) => v,
                None => return Ok(()),
            },
            Value::Array(items) => match part.parse::<usize>().ok().and_then(|k| items.get_mut(k)) {
                Some(v) => v,
                None => return Ok(()),
            },
            _ => return Ok(()),
        };
    }
    if let Value::Object(map) = at {
        map.remove(*last);
    }
    Ok(())
}

/// `set` and `unset` applied to `target`.
fn apply_edits(cmd: &str, target: &mut Value, set: Option<&Map<String, Value>>, unset: Option<&[String]>, guard: &[&str]) -> Result<()> {
    let n = set.map_or(0, Map::len) + unset.map_or(0, <[String]>::len);
    if n > MAX_KEYS {
        return Err(bad(cmd, format!("one edit sets or unsets at most {MAX_KEYS} keys")));
    }
    for (path, v) in set.into_iter().flatten() {
        if guard.iter().any(|g| path.split('.').next() == Some(*g)) {
            return Err(bad(cmd, format!("`{path}` is not edited here")));
        }
        set_path(cmd, target, path, v.clone())?;
    }
    for path in unset.into_iter().flatten() {
        if guard.iter().any(|g| path.split('.').next() == Some(*g)) {
            return Err(bad(cmd, format!("`{path}` is not edited here")));
        }
        unset_path(cmd, target, path)?;
    }
    Ok(())
}

fn edit_job(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job.edit";
    let params: CamJobEditParams = parse(cmd, p, "")?;
    let open = need(s, cmd)?;
    let mut next = open.value.clone();
    apply_edits(cmd, &mut next, params.set.as_ref(), params.unset.as_deref(), &["operations"])?;
    let keys: Vec<String> = params.set.iter().flat_map(Map::keys).cloned().chain(params.unset.iter().flatten().cloned()).collect();
    let label = match keys.as_slice() {
        [one] => format!("Set {one}"),
        _ => "Edit the job".to_string(),
    };
    commit(s, cmd, label, next)?;
    to_value(summary(s))
}

fn history(s: &mut Session, cmd: &str, redo: bool) -> Result<Value> {
    let open = need(s, cmd)?;
    let step = if redo { open.redo() } else { open.undo() };
    to_value(CamJobHistoryReply { step, summary: summary(s) })
}

fn undo_job(s: &mut Session, p: &Value) -> Result<Value> {
    let _: CamJobHistoryParams = parse("cam.job.undo", p, "")?;
    history(s, "cam.job.undo", false)
}

fn redo_job(s: &mut Session, p: &Value) -> Result<Value> {
    let _: CamJobHistoryParams = parse("cam.job.redo", p, "")?;
    history(s, "cam.job.redo", true)
}

fn stage(_s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "view.manufacture_stage";
    let params: ManufactureStageParams = parse(cmd, p, "")?;
    if !STAGES.contains(&params.stage.as_str()) {
        return Err(bad(cmd, "`stage` must be setup, operations, check or program"));
    }
    to_value(ManufactureStageReply { stage: params.stage })
}

/// The operation `sel` names in `ops`: its place. A name two operations have is refused.
pub(super) fn select(cmd: &str, ops: &[Value], sel: &OpSelect) -> Result<usize> {
    match sel {
        OpSelect::Index(i) if *i < ops.len() => Ok(*i),
        OpSelect::Index(i) => Err(bad(cmd, format!("operation {i} is not in the job: it has {} (from 0)", ops.len()))),
        OpSelect::Name(name) => {
            let hits: Vec<usize> = ops.iter().enumerate().filter(|(_, o)| op_name(o) == *name).map(|(i, _)| i).collect();
            match hits.as_slice() {
                [one] => Ok(*one),
                [] => Err(bad(cmd, format!("no operation is named `{}`", name.chars().take(MAX_NAME).collect::<String>()))),
                _ => Err(bad(cmd, format!("{} operations are named `{name}`: give its index", hits.len()))),
            }
        }
    }
}

fn ops_of(v: &Value) -> Vec<Value> {
    v.get("operations").and_then(Value::as_array).cloned().unwrap_or_default()
}

fn with_ops(mut v: Value, ops: Vec<Value>) -> Value {
    if let Value::Object(map) = &mut v {
        map.insert("operations".into(), Value::Array(ops));
    }
    v
}

/// `base`, or `base 2`, `base 3`... whichever no operation of `ops` is named.
fn unique_name(ops: &[Value], base: &str) -> String {
    let taken = |n: &str| ops.iter().any(|o| op_name(o) == n);
    if !taken(base) {
        return base.to_string();
    }
    (2..).map(|k| format!("{base} {k}")).find(|n| !taken(n)).unwrap_or_else(|| base.to_string())
}

/// The English label of an operation type.
pub fn type_label(ty: &str) -> &str {
    OP_TYPES.iter().find(|(t, _, _)| *t == ty).map_or(ty, |(_, label, _)| *label)
}

/// The starting heights of an operation that needs all four.
const STARTING_HEIGHTS: [(&str, i32); 4] = [("clearance", 10), ("retract", 3), ("top", 0), ("bottom", -1)];

/// Starting values for the keys `ty` cannot do without, where `op` leaves them out. Everything
/// else stays out, so it takes its default. With picks, the heights (and a face's bottom) come
/// from the picked geometry instead.
fn starting_values(ty: &str, op: &mut Map<String, Value>) {
    let picked = op.get("picks").is_some();
    let mut put = |k: &str, v: Value| {
        if picked && matches!(k, "heights" | "bottom" | "width") {
            return;
        }
        op.entry(k.to_string()).or_insert(v);
    };
    put("tool", json!({}));
    let heights = Value::Object(STARTING_HEIGHTS.iter().map(|(k, v)| ((*k).to_string(), json!(v))).collect());
    match ty {
        "face" => put("bottom", json!(0)),
        "adaptive" | "adaptive_roughing" | "drill" | "bore" | "scallop" | "steep_shallow" | "pencil" => put("heights", heights),
        "rest_finish" => {
            put("heights", heights);
            put("previous_tool", json!(0));
        }
        "thread_mill" => {
            put("heights", heights);
            put("pitch", json!(1));
            put("thread_diameter", json!(6));
        }
        "chamfer" => {
            put("heights", json!({"clearance": 10, "retract": 3, "top": 0}));
            put("width", json!(0.5));
        }
        "rotary_finish" | "rotary_roughing" => {
            put("pattern", json!({"type": "lines", "angle_step_deg": 5}));
            put("clearance", json!(30));
            put("retract", json!(25));
        }
        "wrap" => put("radius", json!(10)),
        "wrap_pocket" | "wrap_contour" => {
            put("radius", json!(10));
            put("bottom", json!(-0.5));
        }
        _ => {}
    }
}

/// `inner` (a 3-axis operation, with its name) indexed by `a`: one angle (0: plain), a list
/// (`indexed_pattern`), or `{count, start_deg?}`. `enabled` stays on the outside.
fn index_to(cmd: &str, mut inner: Map<String, Value>, a: &Value) -> Result<Value> {
    let ty = inner.get("type").and_then(Value::as_str).unwrap_or_default().to_string();
    let enabled = inner.remove("enabled");
    let finite = |x: f64| x.is_finite() && x.abs() <= 1.0e5;
    let plain = matches!(a, Value::Number(n) if n.as_f64() == Some(0.0));
    if !plain && !INDEXABLE.contains(&ty.as_str()) {
        return Err(bad(cmd, format!("a `{ty}` operation cannot be indexed: turning A to an angle takes a 3-axis operation")));
    }
    let mut outer = Map::new();
    match a {
        _ if plain => {
            if let Some(e) = enabled {
                inner.insert("enabled".into(), e);
            }
            return Ok(Value::Object(inner));
        }
        Value::Number(n) => {
            let deg = n.as_f64().filter(|d| finite(*d)).ok_or_else(|| bad(cmd, "`a_deg` must be a finite angle in degrees"))?;
            if let Some(name) = inner.remove("name") {
                outer.insert("name".into(), name);
            }
            outer.insert("type".into(), json!("indexed"));
            outer.insert("a_deg".into(), json!(deg));
        }
        Value::Array(list) => {
            let degs: Vec<f64> = list.iter().filter_map(Value::as_f64).filter(|d| finite(*d)).collect();
            if degs.len() != list.len() || degs.is_empty() || degs.len() > MAX_OPERATIONS {
                return Err(bad(cmd, format!("`a_deg` lists 1 to {MAX_OPERATIONS} finite angles")));
            }
            outer.insert("type".into(), json!("indexed_pattern"));
            outer.insert("a_deg".into(), json!(degs));
        }
        Value::Object(o) => {
            let count = o.get("count").and_then(Value::as_u64).filter(|c| (1..=MAX_OPERATIONS as u64).contains(c));
            let Some(count) = count else { return Err(bad(cmd, format!("`a_deg.count` must be 1 to {MAX_OPERATIONS}"))) };
            outer.insert("type".into(), json!("indexed_pattern"));
            outer.insert("count".into(), json!(count));
            if let Some(start) = o.get("start_deg") {
                let start = start.as_f64().filter(|d| finite(*d)).ok_or_else(|| bad(cmd, "`a_deg.start_deg` must be a finite angle"))?;
                outer.insert("start_deg".into(), json!(start));
            }
            if o.keys().any(|k| k != "count" && k != "start_deg") {
                return Err(bad(cmd, "`a_deg` as an object takes `count` and `start_deg`"));
            }
        }
        _ => return Err(bad(cmd, "`a_deg` is an angle, a list of angles or {count, start_deg?}")),
    }
    if let Some(e) = enabled {
        outer.insert("enabled".into(), e);
    }
    outer.insert("operation".into(), Value::Object(inner));
    Ok(Value::Object(outer))
}

/// An operation as its 3-axis inside, with its name and `enabled`: an indexed one unwrapped.
fn unwrap_indexed(op: &Value) -> Map<String, Value> {
    let (ty, _) = op_types(op);
    let mut out = match (ty.as_str(), op.get("operation")) {
        ("indexed" | "indexed_pattern", Some(Value::Object(inner))) => inner.clone(),
        _ => return op.as_object().cloned().unwrap_or_default(),
    };
    if let Some(name) = op.get("name").filter(|n| n.as_str().is_some_and(|n| !n.is_empty())) {
        out.insert("name".into(), name.clone());
    }
    if let Some(e) = op.get("enabled") {
        out.insert("enabled".into(), e.clone());
    }
    out
}

fn op_reply(s: &Session, index: Option<usize>, name: String) -> Result<Value> {
    to_value(CamOpReply { index, name, summary: summary(s) })
}

fn op_add(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.add";
    let params: CamOpAddParams = parse(cmd, p, "")?;
    need(s, cmd)?;
    let mut op = params.op;
    let Some(ty) = op.get("type").and_then(Value::as_str).map(str::to_string) else {
        return Err(bad(cmd, "`op` needs its `type` (face, pocket, contour, drill, …)"));
    };
    let picks = match (&params.picks, params.from_selection.unwrap_or(false)) {
        (None, false) => None,
        (p, sel) => Some(super::picks::picks_param(cmd, s, p.as_ref(), sel)?),
    };
    let mut a_deg = params.a_deg;
    if let Some(picks) = &picks {
        if super::picks::GEOMETRY_KEYS.iter().any(|k| op.contains_key(*k)) {
            return Err(bad(cmd, "give the operation `picks` or its own geometry (sketch, polygon, feature…), not both"));
        }
        // Picks on another side turn A there (a pocket or contour on a feature turns itself).
        if a_deg.is_none() && !matches!(ty.as_str(), "pocket" | "contour") {
            a_deg = super::picks::picks_side(cmd, s, picks);
        }
        op.insert("picks".into(), json!(picks));
    }
    let open = need(s, cmd)?;
    let mut ops = ops_of(&open.value);
    if ops.len() >= MAX_OPERATIONS {
        return Err(bad(cmd, format!("a job has at most {MAX_OPERATIONS} operations")));
    }
    if ty == "indexed" || ty == "indexed_pattern" {
        if op.get("operation").is_none() {
            return Err(bad(cmd, format!("an `{ty}` operation holds its `operation`")));
        }
    } else {
        starting_values(&ty, &mut op);
    }
    let name = match op.get("name").and_then(Value::as_str) {
        Some(n) => {
            check_name(cmd, "the operation's `name`", n)?;
            if ops.iter().any(|o| op_name(o) == n) {
                return Err(bad(cmd, format!("an operation is already named `{n}`: give another name")));
            }
            n.to_string()
        }
        None => {
            let n = unique_name(&ops, type_label(&ty));
            op.insert("name".into(), json!(n));
            n
        }
    };
    let at = params.at.unwrap_or(ops.len());
    if at > ops.len() {
        return Err(bad(cmd, format!("`at` must be 0 to {}", ops.len())));
    }
    let value = match a_deg {
        Some(a) if a != 0.0 => index_to(cmd, op, &json!(a))?,
        _ => Value::Object(op),
    };
    ops.insert(at, value);
    let next = with_ops(open.value.clone(), ops);
    commit(s, cmd, format!("Add {} '{name}'", type_label(&ty)), next)?;
    op_reply(s, Some(at), name)
}

fn op_edit(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.edit";
    let params: CamOpEditParams = parse(cmd, p, "")?;
    let open = need(s, cmd)?;
    let mut ops = ops_of(&open.value);
    let i = select(cmd, &ops, &params.op)?;
    let old_name = op_name(&ops[i]);
    let (ty, _) = op_types(&ops[i]);
    let wrapped = ty == "indexed" || ty == "indexed_pattern";
    // Edits go to the operation inside a wrapper; its name and on/off stay with it.
    let mut inner = Value::Object(unwrap_indexed(&ops[i]));
    apply_edits(cmd, &mut inner, params.set.as_ref(), params.unset.as_deref(), &["type", "operation"])?;
    // New picks take the place of the geometry it had, and of the starting heights it was
    // added with (the picks give heights of their own).
    if let (Some(set), Value::Object(m)) = (&params.set, &mut inner)
        && set.contains_key("picks")
    {
        for k in super::picks::GEOMETRY_KEYS {
            if !set.contains_key(*k) {
                m.remove(*k);
            }
        }
        let starting = Value::Object(STARTING_HEIGHTS.iter().map(|(k, v)| ((*k).to_string(), json!(v))).collect());
        if m.get("heights") == Some(&starting) && !set.contains_key("heights") {
            m.remove("heights");
        }
    }
    let mut name = old_name.clone();
    if let Some(new) = &params.rename {
        check_name(cmd, "`rename`", new)?;
        if ops.iter().enumerate().any(|(k, o)| k != i && op_name(o) == *new) {
            return Err(bad(cmd, format!("an operation is already named `{new}`: give another name")));
        }
        set_path(cmd, &mut inner, "name", json!(new))?;
        name = new.clone();
    }
    // A name given through `set` or `unset` is held to what `rename` is.
    let edited = inner.get("name").and_then(Value::as_str).unwrap_or_default().to_string();
    if edited != name {
        check_name(cmd, "the operation's `name`", &edited)?;
        if ops.iter().enumerate().any(|(k, o)| k != i && op_name(o) == edited) {
            return Err(bad(cmd, format!("an operation is already named `{edited}`: give another name")));
        }
        name = edited;
    }
    let inner = inner.as_object().cloned().unwrap_or_default();
    let a = match (&params.a_deg, wrapped) {
        (Some(a), _) => Some(a.clone()),
        // Keep the wrapper as it was.
        (None, true) => {
            let o = &ops[i];
            Some(match (o.get("a_deg"), o.get("count")) {
                (Some(a), _) => a.clone(),
                (None, Some(c)) => {
                    let mut m = Map::new();
                    m.insert("count".into(), c.clone());
                    if let Some(st) = o.get("start_deg") {
                        m.insert("start_deg".into(), st.clone());
                    }
                    Value::Object(m)
                }
                (None, None) => json!(0),
            })
        }
        (None, false) => None,
    };
    ops[i] = match a {
        Some(a) => index_to(cmd, inner, &a)?,
        None => Value::Object(inner),
    };
    let next = with_ops(open.value.clone(), ops);
    let label = match &params.rename {
        Some(new) if params.set.is_none() && params.unset.is_none() && params.a_deg.is_none() => format!("Rename '{old_name}' to '{new}'"),
        _ => format!("Edit '{old_name}'"),
    };
    commit(s, cmd, label, next)?;
    op_reply(s, Some(i), name)
}

fn op_remove(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.remove";
    let params: CamOpParams = parse(cmd, p, "")?;
    let open = need(s, cmd)?;
    let mut ops = ops_of(&open.value);
    let i = select(cmd, &ops, &params.op)?;
    let name = op_name(&ops.remove(i));
    let next = with_ops(open.value.clone(), ops);
    commit(s, cmd, format!("Remove '{name}'"), next)?;
    op_reply(s, None, name)
}

fn op_move(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.move";
    let params: CamOpMoveParams = parse(cmd, p, "")?;
    let open = need(s, cmd)?;
    let mut ops = ops_of(&open.value);
    let i = select(cmd, &ops, &params.op)?;
    if params.to >= ops.len() {
        return Err(bad(cmd, format!("`to` must be 0 to {}", ops.len() - 1)));
    }
    let op = ops.remove(i);
    let name = op_name(&op);
    ops.insert(params.to, op);
    let next = with_ops(open.value.clone(), ops);
    commit(s, cmd, format!("Move '{name}'"), next)?;
    op_reply(s, Some(params.to), name)
}

fn op_duplicate(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.duplicate";
    let params: CamOpDuplicateParams = parse(cmd, p, "")?;
    let open = need(s, cmd)?;
    let mut ops = ops_of(&open.value);
    if ops.len() >= MAX_OPERATIONS {
        return Err(bad(cmd, format!("a job has at most {MAX_OPERATIONS} operations")));
    }
    let i = select(cmd, &ops, &params.op)?;
    let to = params.to.unwrap_or(i + 1);
    if to > ops.len() {
        return Err(bad(cmd, format!("`to` must be 0 to {}", ops.len())));
    }
    let old = op_name(&ops[i]);
    let name = unique_name(&ops, &format!("{old} copy"));
    let mut copy = ops[i].clone();
    // An indexed pattern keeps its name on the operation inside.
    let inside = !copy.get("name").and_then(Value::as_str).is_some_and(|n| !n.is_empty()) && copy.get("operation").is_some();
    let target = if inside { copy.get_mut("operation") } else { Some(&mut copy) };
    if let Some(Value::Object(m)) = target {
        m.insert("name".into(), json!(name));
    }
    ops.insert(to, copy);
    let next = with_ops(open.value.clone(), ops);
    commit(s, cmd, format!("Duplicate '{old}'"), next)?;
    op_reply(s, Some(to), name)
}

fn op_enable(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.enable";
    let params: CamOpEnableParams = parse(cmd, p, "")?;
    let open = need(s, cmd)?;
    let mut ops = ops_of(&open.value);
    let i = select(cmd, &ops, &params.op)?;
    let name = op_name(&ops[i]);
    if let Value::Object(m) = &mut ops[i] {
        // On is the default: the key goes rather than being written true.
        if params.enabled {
            m.remove("enabled");
            // Off inside an indexed operation is off too.
            if let Some(Value::Object(inner)) = m.get_mut("operation") {
                inner.remove("enabled");
            }
        } else {
            m.insert("enabled".into(), json!(false));
        }
    }
    let next = with_ops(open.value.clone(), ops);
    commit(s, cmd, format!("{} '{name}'", if params.enabled { "Turn on" } else { "Turn off" }), next)?;
    op_reply(s, Some(i), name)
}

/// The open job in brief (an empty summary without one).
pub fn summary(s: &Session) -> JobSummary {
    let Some(open) = s.cam_job.as_deref() else { return JobSummary::default() };
    let v = &open.value;
    let text = |k: &str| v.get(k).and_then(Value::as_str).map(str::to_string);
    let file = |p: String| Path::new(&p).file_name().map_or(p.clone(), |f| f.to_string_lossy().to_string());
    let profile = text("profile_path");
    let from_profile = || if profile.is_some() { "from the profile".to_string() } else { "none".to_string() };
    let machine = match v.get("machine") {
        Some(Value::String(m)) => m.clone(),
        Some(Value::Object(m)) => m.get("name").and_then(Value::as_str).unwrap_or("custom").to_string(),
        _ => text("machine_path").map(file).unwrap_or_else(from_profile),
    };
    let post = text("post").or_else(|| text("post_path").map(file)).unwrap_or_else(from_profile);
    let tools = match (v.get("tools").and_then(Value::as_array), text("tool_library_path"), v.get("tool_library_paths").and_then(Value::as_array)) {
        (Some(t), _, _) => format!("{} tools", t.len()),
        (None, Some(p), _) => file(p),
        (None, None, Some(list)) => list.iter().filter_map(Value::as_str).map(|p| file(p.to_string())).collect::<Vec<_>>().join(", "),
        _ => from_profile(),
    };
    let num = |x: f64| super::num(x);
    let stock = v.get("stock").map(|st| {
        let arr = |x: &Value| x.as_array().map(|a| a.iter().filter_map(Value::as_f64).collect::<Vec<_>>()).unwrap_or_default();
        if let Some(b) = st.get("box") {
            let (lo, hi) = (arr(&b["min"]), arr(&b["max"]));
            if lo.len() == 3 && hi.len() == 3 {
                return format!("block {} x {} x {} mm", num(hi[0] - lo[0]), num(hi[1] - lo[1]), num(hi[2] - lo[2]));
            }
        }
        if let Some(c) = st.get("cylinder") {
            return format!(
                "bar R{} mm, X {} to {}",
                num(c["radius"].as_f64().unwrap_or(0.0)),
                num(c["x_min"].as_f64().unwrap_or(0.0)),
                num(c["x_max"].as_f64().unwrap_or(0.0))
            );
        }
        if let Some(a) = st.get("around_part") {
            let m = arr(&a["margin"]);
            return format!("around the part, margins {}", m.iter().map(|x| num(*x)).collect::<Vec<_>>().join(", "));
        }
        "given".to_string()
    });
    let origin = v.get("origin").and_then(Value::as_array).map_or([0.0; 3], |a| {
        let g = |k: usize| a.get(k).and_then(Value::as_f64).unwrap_or(0.0);
        [g(0), g(1), g(2)]
    });
    let holding = text("workholding_path").map(file).or_else(|| v.get("workholding").map(|_| "in the job".to_string()));
    let names =
        |k: &str| v.get(k).and_then(Value::as_array).map(|a| a.iter().filter_map(Value::as_str).map(str::to_string).collect()).unwrap_or_default();
    let generated = open.generated.as_ref();
    let operations = open
        .operations()
        .iter()
        .enumerate()
        .map(|(index, op)| {
            let (ty, inner) = op_types(op);
            let kind = match ty.as_str() {
                "indexed" | "indexed_pattern" => inner.unwrap_or_default(),
                _ => ty.clone(),
            };
            let a_deg = match (ty.as_str(), op.get("a_deg")) {
                ("indexed", Some(a)) => a.as_f64().into_iter().collect(),
                ("indexed_pattern", Some(Value::Array(a))) => a.iter().filter_map(Value::as_f64).collect(),
                _ => Vec::new(),
            };
            let own_tool = op.get("tool").or_else(|| op.get("operation").and_then(|o| o.get("tool"))).and_then(Value::as_u64);
            let tool = own_tool.and_then(|t| u32::try_from(t).ok()).or_else(|| generated.and_then(|g| g.tools.get(&index).copied()));
            OpSummary {
                index,
                name: op_name(op),
                label_id: format!("ui.manufacture.op.{kind}"),
                kind,
                tool,
                a_deg,
                enabled: op_enabled(op),
                status: open.status(index, s.model_stamp()),
                minutes: generated.and_then(|g| g.minutes.get(&index).copied()),
            }
        })
        .collect();
    JobSummary {
        path: open.path.clone(),
        name: text("name").unwrap_or_else(|| s.doc.name.clone()),
        dirty: open.dirty(),
        revision: open.revision,
        machine,
        post,
        profile,
        tools,
        stock,
        origin,
        holding,
        bodies: names("bodies"),
        fixtures: names("fixtures"),
        operations,
    }
}

/// The library folder: the one asked for, else the nearest folder above the open job's that
/// holds `profiles/` or `tools/`.
fn library_folder(s: &Session, cmd: &str, folder: Option<&str>) -> Result<String> {
    if let Some(f) = folder {
        if f.trim().is_empty() || f.chars().any(char::is_control) {
            return Err(bad(cmd, "`folder` must be a folder path"));
        }
        return Ok(f.trim().to_string());
    }
    let start = s.cam_job.as_deref().and_then(OpenJob::folder).ok_or_else(|| bad(cmd, "give `folder`: no job file is open to look beside"))?;
    let mut at = Some(start.as_path());
    for _ in 0..4 {
        let Some(dir) = at else { break };
        if ["profiles", "tools"].iter().any(|sub| !list_dir(&dir.join(sub).to_string_lossy()).is_empty()) {
            return Ok(dir.to_string_lossy().to_string());
        }
        at = dir.parent();
    }
    Err(bad(cmd, "no library folder (with profiles/ or tools/) was found above the job's folder: give `folder`"))
}

/// The files in `dir`, by name (none when it cannot be read or is outside the session's folders).
pub(crate) fn list_dir(dir: &str) -> Vec<String> {
    if solidcraft_io::vfs::check(dir).is_err() {
        return Vec::new();
    }
    #[cfg(not(target_arch = "wasm32"))]
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .map(|it| {
            it.filter_map(|e| e.ok()).filter(|e| e.path().is_file()).map(|e| e.path().to_string_lossy().to_string()).take(MAX_LISTED * 4).collect()
        })
        .unwrap_or_default();
    #[cfg(target_arch = "wasm32")]
    let mut names: Vec<String> = {
        let prefix = format!("{}/", dir.trim_end_matches('/'));
        solidcraft_io::vfs::list(&prefix).into_iter().map(|(p, _)| p).filter(|p| !p[prefix.len()..].contains('/')).collect()
    };
    names.sort();
    names
}

fn library_list(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.library.list";
    let params: CamLibraryListParams = parse(cmd, p, "")?;
    let folder = library_folder(s, cmd, params.folder.as_deref())?;
    let root = Path::new(&folder);
    let files = |sub: &str, suffix: &str| -> Vec<String> {
        list_dir(&root.join(sub).to_string_lossy()).into_iter().filter(|f| f.to_lowercase().ends_with(suffix)).take(MAX_LISTED).collect()
    };
    let stem = |path: &str, suffix: &str| {
        let name = Path::new(path).file_name().map_or(path.to_string(), |f| f.to_string_lossy().to_string());
        name.strip_suffix(suffix).map_or(name.clone(), str::to_string)
    };
    let read = |path: &str| solidcraft_io::vfs::read(path, cam_job_state::MAX_FILE_BYTES).ok().and_then(|b| serde_json::from_slice::<Value>(&b).ok());
    let named = |path: &str, suffix: &str| {
        let name = read(path).and_then(|v| v.get("name").and_then(Value::as_str).map(str::to_string)).unwrap_or_else(|| stem(path, suffix));
        LibraryFile { path: path.to_string(), name }
    };
    let profiles = files("profiles", ".profile.json")
        .into_iter()
        .map(|path| {
            let v = read(&path).unwrap_or(Value::Null);
            let machine = match v.get("machine") {
                Some(Value::String(m)) => m.clone(),
                Some(_) => "custom".into(),
                None => v.get("machine_path").and_then(Value::as_str).map_or_else(String::new, |m| stem(m, ".machine.json")),
            };
            LibraryProfile {
                name: v.get("name").and_then(Value::as_str).map_or_else(|| stem(&path, ".profile.json"), str::to_string),
                description: v.get("description").and_then(Value::as_str).unwrap_or_default().to_string(),
                machine,
                path,
            }
        })
        .collect();
    let mut posts: Vec<LibraryFile> = super::POSTS.iter().map(|(name, ..)| LibraryFile { path: String::new(), name: (*name).to_string() }).collect();
    posts.extend(files("posts", ".post.json").iter().map(|p| named(p, ".post.json")));
    let tool_libraries = files("tools", ".json")
        .into_iter()
        .map(|path| {
            let mut left_out = Vec::new();
            let tools = agent::library_tools(cmd, None, std::slice::from_ref(&path), None, &mut left_out).map_or(0, |t| t.len());
            LibraryTools { path, tools, left_out: left_out.into_iter().map(|(n, why)| format!("T{n}: {why}")).collect() }
        })
        .collect();
    let machines_dir = |suffix: &str| -> Vec<LibraryFile> { files("machines", suffix).iter().map(|p| named(p, suffix)).collect() };
    let reply = CamLibraryListReply {
        machines: machines_dir(".machine.json"),
        workholdings: machines_dir(".workholding.json"),
        machine_models: machines_dir(".machine-model.json"),
        folder,
        profiles,
        posts,
        tool_libraries,
    };
    to_value(reply)
}

#[cfg(test)]
#[path = "cam_job_edit_tests.rs"]
mod tests;
