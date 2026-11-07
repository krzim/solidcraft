//! `cam.op.defaults` and `cam.op.suggest` (a child of `cmd/cam.rs`): the values an operation of
//! the open job runs with and where each comes from, for the Manufacture workspace's operation
//! form (a key left out of the job takes its default, worked out here: from the tool, its
//! cutting data, the profile, the model, the stock, a pick or the rule the schema states); and
//! the operations `cam.suggest` proposes for the part, added to the open job as picks.

use serde_json::{Map, Value, json};
use solidcraft_cam::gcode5::MachineOperation;

use super::{CommandSpec, num, parse};
use crate::params::bad;
use crate::schema::cam_picks::{
    CamOpDefaultsParams, CamOpDefaultsReply, CamOpSuggestParams, CamOpSuggestReply, FieldDefault, ToolFit, default_rule, op_keys,
};
use crate::schema::{TypedContract, output_schema_for};
use crate::{EngineError, Result, Session};

const DEFAULTS: TypedContract<CamOpDefaultsParams> = TypedContract::new();
const SUGGEST: TypedContract<CamOpSuggestParams> = TypedContract::new();

pub(crate) static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.op.defaults", "Machining Operation Defaults", crate::checked!("cam.op.defaults", defaults))
        .noundo()
        .doc_only()
        .contract(&DEFAULTS)
        .output(output_schema_for::<CamOpDefaultsReply>)
        .params(
            "op?: index | name of the open job's | draft?: {type, …} (an operation not in the job); plan?: work out the tool, heights and \
             feeds by planning it (default true) → type, fields: [{path, value?, source: given | tool | cutting_data | profile | model | \
             stock | pick | rule, why}], tool?, tool_why?, tools: [{number, label, fits, why_not?}], picks_resolved, refused?. Changes \
             nothing",
        ),
    CommandSpec::new("cam.op.suggest", "Suggest Machining Operations", crate::checked!("cam.op.suggest", suggest))
        .noundo()
        .doc_only()
        .contract(&SUGGEST)
        .output(output_schema_for::<CamOpSuggestReply>)
        .params(
            "replace?: instead of adding after the job's operations; stock_top?: mm to face off the top (default 0.5) → added: [name: \
             why], skipped, check, summary. The operations cam.suggest proposes for the part with the open job's machine and tools, \
             their geometry as picks; one job undo step",
        ),
];

fn to_value<T: serde::Serialize>(v: T) -> Result<Value> {
    serde_json::to_value(v).map_err(|e| EngineError::Other(e.to_string()))
}

/// The operation an indexed one holds (its name and on/off kept), or the operation itself.
fn inner_of(op: &Value) -> (String, Value) {
    let ty = op.get("type").and_then(Value::as_str).unwrap_or_default();
    match (ty, op.get("operation")) {
        ("indexed" | "indexed_pattern", Some(inner)) => {
            let t = inner.get("type").and_then(Value::as_str).unwrap_or_default().to_string();
            (t, inner.clone())
        }
        _ => (ty.to_string(), op.clone()),
    }
}

/// The value at dotted `path` of `v`.
fn at<'a>(v: &'a Value, path: &str) -> Option<&'a Value> {
    path.split('.').try_fold(v, |v, k| v.get(k))
}

/// The keys of type `ty` the form shows, `heights` and `feeds` taken apart.
fn paths_of(ty: &str) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for (k, d) in op_keys(ty) {
        match k.as_str() {
            "name" | "type" => {}
            "heights" => {
                let keys: &[&str] = if ty == "chamfer" { &["clearance", "retract", "top"] } else { &["clearance", "retract", "top", "bottom"] };
                for h in keys {
                    out.push((format!("heights.{h}"), d.clone()));
                }
            }
            "feeds" => {
                out.push(("feeds.cut".into(), d.clone()));
                out.push(("feeds.plunge".into(), d.clone()));
            }
            _ => out.push((k, d)),
        }
    }
    out
}

fn defaults(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.defaults";
    let params: CamOpDefaultsParams = parse(cmd, p, "")?;
    let open = s.cam_job.as_deref().ok_or_else(|| bad(cmd, super::job_edit::NO_JOB))?;
    let op = match (&params.op, &params.draft) {
        (Some(_), Some(_)) => return Err(bad(cmd, "give `op` or `draft`, not both")),
        (None, None) => return Err(bad(cmd, "give `op` (an operation of the open job) or a `draft`")),
        (Some(sel), None) => {
            let ops = open.operations();
            let i = super::job_edit::select(cmd, ops, sel)?;
            ops.get(i).cloned().unwrap_or(Value::Null)
        }
        (None, Some(d)) => {
            let ty = d.get("type").and_then(Value::as_str).unwrap_or_default();
            if !crate::cam_job_state::OP_TYPES.iter().any(|(t, _, _)| *t == ty) && !matches!(ty, "indexed" | "indexed_pattern") {
                return Err(bad(cmd, "a `draft` is an operation: `{type, …}` with a type such as face, pocket or drill"));
            }
            let mut d = d.clone();
            d.entry("name").or_insert(json!("draft"));
            Value::Object(d)
        }
    };
    let base = open.folder();
    let merged = super::agent::merged_value(cmd, open.value.clone(), base.as_deref())?;
    let (ty, inner) = inner_of(&op);
    let mut reply = CamOpDefaultsReply { kind: ty.clone(), ..CamOpDefaultsReply::default() };

    // The job's tools, those that fit first.
    let m = super::picks::model_of(cmd, s, &merged, base.as_deref()).ok();
    let set = m.as_ref().and_then(|m| m.features.get(cmd, s).ok());
    // The features the operation's picks are on.
    let mut ids: Vec<String> = Vec::new();
    if let (Some(m), Some(picks)) =
        (&m, inner.get("picks").and_then(|p| serde_json::from_value::<Vec<crate::schema::cam_picks::Pick>>(p.clone()).ok()))
    {
        for p in picks.iter().take(super::picks::MAX_PICKS) {
            if let (Ok(h), Some(set)) = (super::picks::find(&m.st, &m.part, p, m.tol), set) {
                let Some(mb) = m.st.bodies.get(h.body) else { continue };
                let faces = super::features::BodyFaces::new(&mb.name, &mb.body);
                if let Some(f) = h.face {
                    for id in super::picks::features_of_face(set, &faces, &mb.name, f, m.origin, m.tol) {
                        if !ids.contains(&id) {
                            ids.push(id);
                        }
                    }
                }
            }
        }
    }
    if let Some(f) = inner.get("feature").and_then(Value::as_str) {
        ids.push(f.to_string());
    }
    if let Some(m) = &m {
        let mut fits: Vec<ToolFit> = m
            .tools
            .iter()
            .map(|t| {
                let why_not = super::picks::misfit(set, &ids, &ty, t);
                let mut label = format!("T{} {} mm {}, {} mm flutes", t.number, num(t.diameter), t.kind.label(), num(t.flute_length));
                if let Some(so) = t.stickout {
                    label.push_str(&format!(", {} stickout", num(so)));
                }
                if !t.description.is_empty() {
                    label.push_str(&format!(" ({})", t.description));
                }
                ToolFit { number: t.number, label, fits: why_not.is_none(), why_not }
            })
            .collect();
        fits.sort_by_key(|f| (!f.fits, f.number));
        reply.tools = fits;
    }

    // Plan it alone, with the job's setup, to find what it runs with.
    let mut planned_values: Map<String, Value> = Map::new();
    let mut sources: Map<String, Value> = Map::new();
    if params.plan.unwrap_or(true) {
        let mut one = merged.clone();
        let mut op1 = op.clone();
        if let Value::Object(o) = &mut op1 {
            o.remove("enabled");
            if let Some(Value::Object(inner)) = o.get_mut("operation") {
                inner.remove("enabled");
            }
        }
        if let Value::Object(j) = &mut one {
            j.insert("operations".into(), json!([op1]));
        }
        let mut run = || -> Result<()> {
            let loaded = super::agent::load_value(cmd, s, one.clone(), base.clone(), String::new())?;
            reply.picks_resolved = loaded.picks.clone();
            let plan = super::plan_job(cmd, s, &loaded)?;
            let (choices, derived, notes) = plan.agent.record();
            if let Some(c) = choices.first() {
                reply.tool = Some(c.tool);
                reply.tool_why = Some(c.why.clone());
            }
            let first = plan.planned.first();
            if let Some(p) = first {
                reply.tool = reply.tool.or(Some(p.tool.number));
                let rpm = match &p.op {
                    MachineOperation::Indexed(io) => io.spindle_rpm,
                    MachineOperation::Continuous(o) => o.spindle_rpm,
                };
                let speed_src = if notes.iter().any(|n| n.contains("held to the spindle")) {
                    ("profile", "the tool's, held to the machine's spindle range")
                } else if notes.iter().any(|n| n.contains("surface speed")) {
                    ("tool", "worked out from the tool's surface speed")
                } else if notes.iter().any(|n| n.contains("fallback")) {
                    ("profile", "the profile's fallback")
                } else if notes.iter().any(|n| n.contains("cutting data")) {
                    ("cutting_data", "the tool's cutting data for this kind of cut")
                } else {
                    ("tool", "the tool's")
                };
                planned_values.insert("spindle_rpm".into(), json!(num(rpm).parse::<f64>().unwrap_or(rpm)));
                sources.insert("spindle_rpm".into(), json!(speed_src));
                let feed_src = if notes.iter().any(|n| n.contains("chip load")) {
                    ("tool", "worked out from the tool's chip load")
                } else if notes.iter().any(|n| n.contains("cutting data")) {
                    ("cutting_data", "the tool's cutting data for this kind of cut")
                } else {
                    ("tool", "the tool's feed and plunge feed")
                };
                for (k, v) in [("feeds.cut", p.feeds.cut), ("feeds.plunge", p.feeds.plunge)] {
                    planned_values.insert(k.into(), json!(v));
                    sources.insert(k.into(), json!(feed_src));
                }
                let t = &p.tool;
                let d = t.diameter;
                for (k, own, share, what) in [
                    ("stepdown", t.stepdown, if ty == "waterline" { 0.25 } else { 0.5 }, "of its diameter"),
                    (
                        "stepover",
                        t.stepover,
                        match ty.as_str() {
                            "parallel" | "scallop" | "pencil" | "steep_shallow" => 0.1,
                            _ => 0.4,
                        },
                        "of its diameter",
                    ),
                ] {
                    let v = own.unwrap_or(d * share);
                    planned_values.insert(k.into(), json!(super::round(v, 1e4)));
                    let why = match own {
                        Some(_) => format!("T{}'s own {k}", t.number),
                        None => format!("{}% {what} ({} mm)", num(share * 100.0), num(d)),
                    };
                    sources.insert(k.into(), json!([if own.is_some() { "tool" } else { "rule" }, why]));
                }
            }
            if let Some(h) = derived.first() {
                for (k, v) in [("clearance", h.clearance), ("retract", h.retract), ("top", h.top), ("bottom", h.bottom)] {
                    let path = format!("heights.{k}");
                    planned_values.insert(path.clone(), json!(super::round(v, 1e4)));
                    if h.derived.iter().any(|d| d == k) {
                        let why: String = h.why.split(';').map(str::trim).find(|w| w.starts_with(k)).unwrap_or(&h.why).to_string();
                        let src = if why.contains("stock") { "stock" } else { "model" };
                        sources.insert(path, json!([src, why]));
                    }
                }
            }
            Ok(())
        };
        if let Err(e) = run() {
            reply.refused = Some(refusal_words(&e));
        }
    }
    // The heights the picks give, where the job does not give them.
    let resolved = resolved_op(cmd, s, &merged, base.as_deref(), &op);
    let picked = |k: &str| resolved.as_ref().and_then(|r| at(r, &format!("heights.{k}"))).is_some_and(|v| !v.is_null());
    // An operation on a feature: the clearance and retract worked out above it (unless its picks
    // give them: those are what it runs with).
    if !planned_values.contains_key("heights.clearance")
        && let (Some(set), Some(id)) = (set, ids.first())
        && let Ok(g) = set.get(id)
        && let Some(su) = set.setup(g)
        && let Ok(h) = super::features::feature_heights(su, g.top, g.bottom, None)
    {
        for (k, v, why) in [
            ("clearance", h.clearance, "10 mm above the material, 5 above the retract height, clear of the fixtures"),
            ("retract", h.retract, "3 mm above the material"),
        ] {
            if picked(k) {
                continue;
            }
            planned_values.insert(format!("heights.{k}"), json!(super::round(v, 1e4)));
            sources.insert(format!("heights.{k}"), json!(["model", why]));
        }
    }

    for (path, description) in paths_of(&ty) {
        let given = at(&inner, &path).cloned();
        let field = if let Some(v) = given {
            FieldDefault { path, value: Some(v), source: "given".into(), why: "set here".into() }
        } else if let Some(v) = planned_values.get(&path) {
            let (source, why) = match sources.get(&path) {
                Some(Value::Array(a)) => (
                    a.first().and_then(Value::as_str).unwrap_or("rule").to_string(),
                    a.get(1).and_then(Value::as_str).unwrap_or_default().to_string(),
                ),
                Some(Value::String(src)) => (src.clone(), default_rule(&description)),
                _ => ("rule".to_string(), default_rule(&description)),
            };
            FieldDefault { path, value: Some(v.clone()), source, why }
        } else if let Some(v) = resolved.as_ref().and_then(|r| at(r, &path)).filter(|v| !v.is_null()) {
            FieldDefault { path, value: Some(v.clone()), source: "pick".into(), why: "from the picked geometry".into() }
        } else if let Some((v, why)) = path.strip_prefix("heights.").and_then(|k| feature_height(set, &ids, k)) {
            FieldDefault { path, value: Some(json!(v)), source: "feature".into(), why }
        } else if path == "tool" {
            let why = reply.tool_why.clone().unwrap_or_else(|| "the job's best tool for it".into());
            FieldDefault { path, value: reply.tool.map(|t| json!(t)), source: "tool".into(), why }
        } else if path == "tolerance" {
            let tol = merged.get("tolerance").and_then(Value::as_f64).unwrap_or(0.01);
            FieldDefault { path, value: Some(json!(tol)), source: "profile".into(), why: "the job's tolerance".into() }
        } else {
            let rule = default_rule(&description);
            FieldDefault { path, value: rule_value(&rule), source: "rule".into(), why: if rule.is_empty() { "none".into() } else { rule } }
        };
        reply.fields.push(field);
    }
    to_value(reply)
}

/// Why planning refused, in words for the form: each refused operation's error, without the
/// wrapping an agent reads (the command, the count, the fix for a call, the `refusals` list).
fn refusal_words(e: &EngineError) -> String {
    let text = match e {
        EngineError::BadParams { msg, .. } => msg.clone(),
        other => other.to_string(),
    };
    let tag = crate::schema::cam_agent::REFUSALS_TAG;
    let listed = text.find(tag).and_then(|at| text.get(at + tag.len()..)).and_then(|t| serde_json::from_str::<Vec<Value>>(t).ok());
    let errors: Vec<String> = listed.iter().flatten().filter_map(|r| r.get("error").and_then(Value::as_str)).map(str::to_string).collect();
    if errors.is_empty() { text.lines().take(4).collect::<Vec<_>>().join(" ") } else { errors.join("; ") }
}

/// The value a rule states outright ("true", "false", a number), when it does.
fn rule_value(rule: &str) -> Option<Value> {
    let first = rule.split([' ', ',', ';', '(']).next().unwrap_or_default();
    match first {
        "true" => Some(json!(true)),
        "false" => Some(json!(false)),
        _ => first.parse::<f64>().ok().filter(|x| x.is_finite()).map(|x| json!(x)),
    }
}

/// Height `k` (`top` or `bottom`) of the first of features `ids`, and what it is.
fn feature_height(set: Option<&super::features::FeatureSet>, ids: &[String], k: &str) -> Option<(f64, String)> {
    let id = ids.first()?;
    let g = set?.get(id).ok()?;
    match k {
        "top" => Some((g.top, format!("the top of {id}"))),
        "bottom" => Some((g.bottom, format!("the bottom of {id}"))),
        _ => None,
    }
}

/// `op` as the planners read it once its picks are found (none when they cannot be).
fn resolved_op(cmd: &str, s: &Session, merged: &Value, base: Option<&std::path::Path>, op: &Value) -> Option<Value> {
    let mut one = merged.clone();
    if let Value::Object(j) = &mut one {
        j.insert("operations".into(), json!([op]));
    }
    if !super::picks::has_refs(&one) {
        return None;
    }
    let mut m = super::picks::model_of(cmd, s, &one, base).ok()?;
    let Value::Object(map) = &mut one else { return None };
    super::picks::resolve_job(&mut m, map).ok()?;
    let first = map.get("operations")?.get(0)?.clone();
    Some(inner_of(&first).1)
}

fn suggest(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.suggest";
    let params: CamOpSuggestParams = parse(cmd, p, "")?;
    let open = s.cam_job.as_deref().ok_or_else(|| bad(cmd, super::job_edit::NO_JOB))?;
    let stored = open.value.clone();
    let base = open.folder();
    let merged = super::agent::merged_value(cmd, stored.clone(), base.as_deref())?;
    let probe_value = super::picks::placeholders(&merged).map_err(|e| bad(cmd, e))?;
    let probe: crate::schema::cam::CamJob = parse(cmd, &probe_value, "")?;
    let machine = super::machine_of(cmd, &probe)?;
    let tools = super::tools_of(cmd, &probe, base.as_deref())?;
    let axis = if super::features::Lazy::axis_of(&machine) == crate::schema::cam_features::AxisSpec::A { "a" } else { "z" };
    let mut q = Map::new();
    if let Some(b) = &probe.bodies {
        q.insert("bodies".into(), json!(b));
    }
    q.insert("origin".into(), json!(probe.origin.unwrap_or([0.0; 3])));
    q.insert("axis".into(), json!(axis));
    if let Some(t) = probe.tolerance {
        q.insert("tolerance".into(), json!(t));
    }
    let tool_values: Vec<Value> = tools.iter().filter_map(|t| serde_json::to_value(t).ok()).collect();
    q.insert("tools".into(), Value::Array(tool_values));
    if let Some(st) = params.stock_top {
        q.insert("stock_top".into(), json!(st));
    }
    q.insert("as_picks".into(), json!(true));
    let got = super::suggest::suggest(s, &Value::Object(q))?;
    let ops: Vec<Value> = got.get("job").and_then(|j| j.get("operations")).and_then(Value::as_array).cloned().unwrap_or_default();
    let open = super::job_edit::need(s, cmd)?;
    let mut next = open.value.clone();
    let mut list: Vec<Value> = if params.replace.unwrap_or(false) { Vec::new() } else { open.operations().to_vec() };
    let mut added = Vec::new();
    let why: Vec<(String, String)> = got
        .get("operations")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .map(|o| {
            (
                o.get("name").and_then(Value::as_str).unwrap_or_default().to_string(),
                o.get("why").and_then(Value::as_str).unwrap_or_default().to_string(),
            )
        })
        .collect();
    for mut op in ops {
        if list.len() >= super::MAX_OPERATIONS {
            break;
        }
        // The library's tools are the job's own: a suggestion names them by number.
        let old = crate::cam_job_state::op_name(&op);
        let name = unique_name(&list, &old);
        let target = if op.get("operation").is_some() && op.get("name").is_none() { op.get_mut("operation") } else { Some(&mut op) };
        if let Some(Value::Object(m)) = target {
            m.insert("name".into(), json!(name));
        }
        let reason = why.iter().find(|(n, _)| *n == old).map_or(String::new(), |(_, w)| w.clone());
        added.push(if reason.is_empty() { name } else { format!("{name}: {reason}") });
        list.push(op);
    }
    if let Value::Object(m) = &mut next {
        m.insert("operations".into(), Value::Array(list));
    }
    let n = added.len();
    super::job_edit::commit(s, cmd, format!("Suggest {n} operation{}", if n == 1 { "" } else { "s" }), next)?;
    let strings = |k: &str| -> Vec<String> {
        got.get(k).and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_str).map(str::to_string).collect()
    };
    to_value(CamOpSuggestReply { added, skipped: strings("skipped"), check: strings("check"), summary: super::job_edit::summary(s) })
}

/// `base`, or `base 2`, `base 3`... whichever no operation of `ops` is named.
fn unique_name(ops: &[Value], base: &str) -> String {
    let taken = |n: &str| ops.iter().any(|o| crate::cam_job_state::op_name(o) == n);
    if !taken(base) {
        return base.to_string();
    }
    (2..1000).map(|k| format!("{base} {k}")).find(|n| !taken(n)).unwrap_or_else(|| base.to_string())
}
