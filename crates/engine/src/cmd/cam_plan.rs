//! Generate, findings and output for the open job (a child of `cmd/cam.rs`): `cam.generate`
//! plans and checks the open job as `cam.job {dry_run}` does and keeps what it found as a flat
//! list of findings (each with its operation, place, program line and fix) and every
//! operation's toolpath for the window to draw; `cam.findings` lists them, `cam.finding.go`
//! points the window at one, and `cam.post` writes the program (and setup sheet) from a plan of
//! the job and design as they are now, never from stale results.

use std::collections::BTreeMap;
use std::path::Path;

use serde_json::{Map, Value, json};
use solidcraft_cam::Move5;

use super::job_edit::{NO_JOB, need};
use super::{CommandSpec, PlanCache, agent, parse, post_of, run_job};
use crate::cam_job_state::{Generated, OpPath};
use crate::params::bad;
use crate::schema::cam::{CamJobParams, CamJobReply};
use crate::schema::cam_agent::{OpRef, REFUSALS_TAG};
use crate::schema::cam_job_edit::{
    CamFindingGoParams, CamFindingGoReply, CamFindingsParams, CamFindingsReply, CamGenerateParams, CamGenerateReply, CamPostParams, CamPostReply,
    Finding, GeneratedOp,
};
use crate::schema::{TypedContract, output_schema_for};
use crate::{EngineError, Result, Session};

const GENERATE: TypedContract<CamGenerateParams> = TypedContract::new();
const FINDINGS: TypedContract<CamFindingsParams> = TypedContract::new();
const GO: TypedContract<CamFindingGoParams> = TypedContract::new();
const POST: TypedContract<CamPostParams> = TypedContract::new();

/// Most toolpath points kept for drawing, over every operation.
const MAX_POINTS: usize = 400_000;
/// Most findings kept.
const MAX_FINDINGS: usize = 1000;

pub(crate) static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.generate", "Generate Toolpaths", crate::checked!("cam.generate", generate))
        .noundo()
        .contract(&GENERATE)
        .output(output_schema_for::<CamGenerateReply>)
        .params(
            "simulate?: also simulate the program on the stock; resolution?: its cell mm → passed, findings: [{index, severity: error | \
             warning | note, check, op?: {index, name}, at?: [x,y,z] work, at_design?, line?, move_index?, message, fix}], operations: \
             [{index, name, tool, moves, minutes, status}], total_minutes, program_lines, stale: false, notes. Plans and checks the open \
             job as cam.job {dry_run} and keeps the findings and toolpaths (a job that cannot be planned is findings too, not a refusal)",
        ),
    CommandSpec::new("cam.findings", "Machining Findings", crate::checked!("cam.findings", findings))
        .noundo()
        .doc_only()
        .contract(&FINDINGS)
        .output(output_schema_for::<CamFindingsReply>)
        .params("→ findings (as cam.generate), stale: the job or design changed since, generated_at: the job revision"),
    CommandSpec::new("cam.finding.go", "Go to Finding", crate::checked!("cam.finding.go", go))
        .noundo()
        .doc_only()
        .contract(&GO)
        .output(output_schema_for::<CamFindingGoReply>)
        .params("index: a finding's → op?, at_design?, line?, move_index?. The window selects the operation and frames the place"),
    CommandSpec::new("cam.post", "Write Program", crate::checked!("cam.post", post))
        .noundo()
        .contract(&POST)
        .output(output_schema_for::<CamPostReply>)
        .params(
            "output?: G-code file (default: the job file's name with the post's extension, beside it); setup_sheet?: .html | .md; \
             preview?: .html; allow_unsafe?: write even when a check fails; simulate? → passed, outputs: {gcode?, preview?, \
             setup_sheet?}, findings, program_lines, program_head, total_minutes. Plans and checks the open job as it is now first (the \
             last Generate's plan is reused when nothing changed); a failed check refuses and writes nothing unless allow_unsafe",
        ),
];

/// A finding from one of a reply's refusals (`failure_details`, or those of a refusal's text).
fn from_refusal(r: &Value, origin: [f64; 3]) -> Finding {
    let at = r.get("at").and_then(Value::as_array).and_then(|a| {
        let v: Vec<f64> = a.iter().filter_map(Value::as_f64).collect();
        (v.len() == 3 && v.iter().all(|x| x.is_finite())).then(|| [v[0], v[1], v[2]])
    });
    let op = r.get("op").filter(|o| o.is_object()).map(|o| OpRef {
        index: o.get("index").and_then(Value::as_u64).and_then(|i| usize::try_from(i).ok()).unwrap_or(0),
        name: o.get("name").and_then(Value::as_str).unwrap_or_default().to_string(),
    });
    Finding {
        severity: "error".into(),
        check: r.get("check").and_then(Value::as_str).unwrap_or("job").to_string(),
        op,
        at,
        at_design: at.map(|a| [a[0] + origin[0], a[1] + origin[1], a[2] + origin[2]]),
        line: r.get("line").and_then(Value::as_u64).and_then(|l| usize::try_from(l).ok()),
        message: r.get("error").and_then(Value::as_str).unwrap_or_default().to_string(),
        fix: r.get("fix").and_then(Value::as_str).unwrap_or_default().to_string(),
        ..Finding::default()
    }
}

/// The findings of a reply: its refusals, its failures without details, each operation's
/// warnings and the simulation's.
fn reply_findings(reply: &CamJobReply, origin: [f64; 3]) -> Vec<Finding> {
    let mut out: Vec<Finding> = Vec::new();
    for r in &reply.failure_details {
        if let Ok(v) = serde_json::to_value(r) {
            out.push(from_refusal(&v, origin));
        }
    }
    if reply.failure_details.is_empty() {
        out.extend(reply.failures.iter().map(|f| Finding {
            severity: "error".into(),
            check: "check".into(),
            message: f.clone(),
            ..Finding::default()
        }));
    }
    if let Some(g) = &reply.gouge.at {
        // The deepest gouge's move, on the finding that names it.
        if let Some(f) = out.iter_mut().find(|f| f.check == "gouge" && f.op.as_ref().is_some_and(|o| o.name == g.operation)) {
            f.move_index = Some(g.move_index);
            f.line = f.line.or(g.line);
        }
    }
    for (k, op) in reply.operations.iter().enumerate() {
        let index = reply.op_of.get(k).copied().unwrap_or(k);
        for w in &op.warnings {
            out.push(Finding {
                severity: "warning".into(),
                check: "warning".into(),
                op: Some(OpRef { index, name: op.name.clone() }),
                message: w.clone(),
                ..Finding::default()
            });
        }
    }
    if let Some(sim) = &reply.simulation {
        let sim = serde_json::to_value(sim).unwrap_or(Value::Null);
        for (key, severity) in [("failures", "error"), ("warnings", "warning")] {
            for m in sim.get(key).and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_str) {
                out.push(Finding { severity: severity.into(), check: "simulation".into(), message: m.to_string(), ..Finding::default() });
            }
        }
    }
    number(out)
}

fn number(mut out: Vec<Finding>) -> Vec<Finding> {
    out.truncate(MAX_FINDINGS);
    for (i, f) in out.iter_mut().enumerate() {
        f.index = i;
    }
    out
}

/// The findings of a refusal: its refusals when it lists them, else the refusal itself.
fn refusal_findings(e: &EngineError, origin: [f64; 3]) -> Vec<Finding> {
    let text = e.to_string();
    let listed = text.find(REFUSALS_TAG).and_then(|at| serde_json::from_str::<Vec<Value>>(&text[at + REFUSALS_TAG.len()..]).ok());
    match listed {
        Some(list) if !list.is_empty() => number(list.iter().map(|r| from_refusal(r, origin)).collect()),
        _ => {
            let message = text.split(REFUSALS_TAG).next().unwrap_or(&text).trim().to_string();
            number(vec![Finding { severity: "error".into(), check: "job".into(), message, ..Finding::default() }])
        }
    }
}

/// Every operation's toolpath of the session's last plan, in design coordinates, at most
/// [`MAX_POINTS`] points in all.
fn paths_of(cache: Option<&PlanCache>) -> (Vec<OpPath>, BTreeMap<usize, u32>) {
    let mut tools = BTreeMap::new();
    let Some(c) = cache else { return (Vec::new(), tools) };
    let plan = &c.plan;
    let o = plan.origin;
    let total: usize = plan.planned.iter().map(|p| p.poses.moves.len()).sum();
    let stride = total.div_ceil(MAX_POINTS).max(1);
    let paths = plan
        .planned
        .iter()
        .enumerate()
        .map(|(k, p)| {
            let op = plan.op_of.get(k).copied().unwrap_or(k);
            tools.entry(op).or_insert(p.tool.number);
            let mut path = OpPath { op, name: p.report.name.clone(), tool: p.tool.number, ..OpPath::default() };
            let mut rapid_since = false;
            let n = p.poses.moves.len();
            for (i, m) in p.poses.moves.iter().enumerate() {
                let (to, rapid) = match m {
                    Move5::Rapid { to } => (to, true),
                    Move5::Feed { to, .. } => (to, false),
                    Move5::Dwell { .. } => continue,
                };
                rapid_since |= rapid;
                if i % stride != 0 && i + 1 != n && !rapid {
                    continue;
                }
                let t = to.tip;
                path.points.push([(t.x + o.x) as f32, (t.y + o.y) as f32, (t.z + o.z) as f32]);
                path.rapid.push(rapid_since);
                rapid_since = false;
            }
            path
        })
        .collect();
    (paths, tools)
}

/// Keeps what a run of the open job found (and drew) on it.
fn keep(s: &mut Session, revision: u64, reply: Option<&CamJobReply>, findings: Vec<Finding>, written: Vec<String>) -> Generated {
    let model_revision = s.model_stamp();
    let (paths, mut tools) =
        if reply.is_some() { s.cam_plan.lock().ok().map(|c| paths_of(c.as_deref())).unwrap_or_default() } else { Default::default() };
    let mut minutes = BTreeMap::new();
    if let Some(r) = reply {
        for (k, op) in r.operations.iter().enumerate() {
            let i = r.op_of.get(k).copied().unwrap_or(k);
            *minutes.entry(i).or_insert(0.0) += op.cut_minutes;
            tools.entry(i).or_insert(op.tool);
        }
    }
    let generated = Generated {
        revision,
        model_revision,
        passed: reply.is_some_and(|r| r.passed) && !findings.iter().any(|f| f.severity == "error"),
        findings,
        minutes,
        tools,
        paths,
        total_minutes: reply.map_or(0.0, |r| r.readback.cut_minutes),
        written,
        program_head: reply.map(|r| r.program_head.clone()).unwrap_or_default(),
    };
    if let Some(open) = s.cam_job.as_deref_mut()
        && open.revision == revision
    {
        open.generated = Some(generated.clone());
    }
    generated
}

fn to_value<T: serde::Serialize>(v: T) -> Result<Value> {
    serde_json::to_value(v).map_err(|e| EngineError::Other(e.to_string()))
}

fn origin_of(s: &Session) -> [f64; 3] {
    s.cam_job.as_deref().and_then(|o| o.value.get("origin")).and_then(Value::as_array).map_or([0.0; 3], |a| {
        let g = |k: usize| a.get(k).and_then(Value::as_f64).filter(|x| x.is_finite()).unwrap_or(0.0);
        [g(0), g(1), g(2)]
    })
}

fn generate(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.generate";
    let params: CamGenerateParams = parse(cmd, p, "")?;
    let revision = need(s, cmd)?.revision;
    if params.resolution.is_some_and(|r| !(r.is_finite() && (1e-4..=1e4).contains(&r))) {
        return Err(bad(cmd, "`resolution` must be from 0.0001 to 10000 mm"));
    }
    let mut raw = Map::new();
    raw.insert("dry_run".into(), json!(true));
    if params.simulate.unwrap_or(false) {
        raw.insert("simulate".into(), json!(true));
        if let Some(r) = params.resolution {
            raw.insert("simulate_resolution".into(), json!(r));
        }
    }
    let raw = Value::Object(raw);
    let job_params: CamJobParams = parse(cmd, &raw, "")?;
    let origin = origin_of(s);
    let (reply, generated) = match run_job(s, cmd, &raw, job_params, None) {
        Ok(reply) => {
            let findings = reply_findings(&reply, origin);
            let g = keep(s, revision, Some(&reply), findings, Vec::new());
            (Some(reply), g)
        }
        Err(e) => {
            let findings = refusal_findings(&e, origin);
            (None, keep(s, revision, None, findings, Vec::new()))
        }
    };
    let status = |i: usize| s.cam_job.as_deref().map(|o| o.status(i, s.model_stamp()));
    let operations = match &reply {
        Some(r) => r
            .operations
            .iter()
            .enumerate()
            .map(|(k, op)| {
                let index = r.op_of.get(k).copied().unwrap_or(k);
                GeneratedOp { index, name: op.name.clone(), tool: op.tool, moves: op.moves, minutes: op.cut_minutes, status: status(index) }
            })
            .collect(),
        None => Vec::new(),
    };
    to_value(CamGenerateReply {
        passed: generated.passed,
        findings: generated.findings,
        operations,
        total_minutes: generated.total_minutes,
        program_lines: reply.as_ref().map_or(0, |r| r.program_lines),
        stale: false,
        notes: reply.map(|r| r.notes).unwrap_or_default(),
    })
}

fn findings(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.findings";
    let _: CamFindingsParams = parse(cmd, p, "")?;
    let model_revision = s.model_stamp();
    let open = s.cam_job.as_deref().ok_or_else(|| bad(cmd, NO_JOB))?;
    let reply = match &open.generated {
        Some(g) => CamFindingsReply {
            findings: g.findings.clone(),
            stale: g.revision != open.revision || g.model_revision != model_revision,
            generated_at: g.revision,
        },
        None => CamFindingsReply { findings: Vec::new(), stale: true, generated_at: 0 },
    };
    to_value(reply)
}

fn go(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.finding.go";
    let params: CamFindingGoParams = parse(cmd, p, "")?;
    let open = need(s, cmd)?;
    let Some(g) = &open.generated else { return Err(bad(cmd, "nothing was generated yet: run cam.generate first")) };
    let Some(f) = g.findings.get(params.index).cloned() else {
        return Err(bad(cmd, format!("finding {} is not there: the last Generate found {}", params.index, g.findings.len())));
    };
    open.go.seq += 1;
    open.go.op = f.op.as_ref().map(|o| o.index);
    open.go.at_design = f.at_design;
    open.go.line = f.line;
    open.go.move_index = f.move_index;
    to_value(CamFindingGoReply { op: f.op, at_design: f.at_design, line: f.line, move_index: f.move_index })
}

/// A file path given to `cam.post`: not empty, in the session's folders.
fn out_path(cmd: &str, path: &str) -> Result<String> {
    let path = path.trim();
    if path.is_empty() || path.chars().any(char::is_control) {
        return Err(bad(cmd, "a file path must not be empty"));
    }
    solidcraft_io::vfs::check(path).map_err(|e| bad(cmd, format!("{path}: {e}")))?;
    Ok(path.to_string())
}

fn post(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.post";
    let params: CamPostParams = parse(cmd, p, "")?;
    let open = need(s, cmd)?;
    let revision = open.revision;
    let job_file = open.path.clone();
    let output = match &params.output {
        Some(o) => out_path(cmd, o)?,
        None => {
            let Some(job_file) = &job_file else { return Err(bad(cmd, "the job has no file yet: give `output` (or save the job first)")) };
            let loaded = agent::load_job_of(cmd, s, &json!({}), false, None)?;
            let (_, post) = post_of(cmd, &loaded.job, loaded.base.as_deref())?;
            let path = Path::new(job_file);
            let name = path.file_name().map(|f| f.to_string_lossy().to_string()).unwrap_or_default();
            let stem = name.strip_suffix(".json").unwrap_or(&name);
            let stem = stem.strip_suffix(".cam").unwrap_or(stem);
            let ext = post.base.extension.trim_start_matches('.');
            let ext = if ext.is_empty() { "nc" } else { ext };
            path.with_file_name(format!("{stem}.{ext}")).to_string_lossy().to_string()
        }
    };
    let mut raw = Map::new();
    raw.insert("output".into(), json!(output));
    for (key, v) in [("setup_sheet", &params.setup_sheet), ("preview", &params.preview)] {
        if let Some(v) = v {
            raw.insert(key.into(), json!(out_path(cmd, v)?));
        }
    }
    for (key, v) in [("allow_unsafe", params.allow_unsafe), ("simulate", params.simulate)] {
        if let Some(v) = v {
            raw.insert(key.into(), json!(v));
        }
    }
    let raw = Value::Object(raw);
    let job_params: CamJobParams = parse(cmd, &raw, "")?;
    let origin = origin_of(s);
    match run_job(s, cmd, &raw, job_params, None) {
        Ok(reply) => {
            let findings = reply_findings(&reply, origin);
            let written: Vec<String> =
                [&reply.outputs.gcode, &reply.outputs.setup_sheet, &reply.outputs.preview].into_iter().flatten().cloned().collect();
            keep(s, revision, Some(&reply), findings.clone(), written);
            to_value(CamPostReply {
                passed: reply.passed,
                findings,
                program_lines: reply.program_lines,
                program_head: reply.program_head,
                total_minutes: reply.readback.cut_minutes,
                outputs: reply.outputs,
            })
        }
        Err(e) => {
            // What refused it is kept as the findings, so the window lists it.
            keep(s, revision, None, refusal_findings(&e, origin), Vec::new());
            Err(e)
        }
    }
}
