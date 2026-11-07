//! Operations by hand in the Manufacture workspace (the stage card's Operations stage, the job
//! tree's *Found in the part* rows and the Setup form's work-zero rows): the operation form built
//! from the descriptor table (`manufacture_op_fields`) with each value's default and where it
//! comes from (`cam.op.defaults`), picks taken from the design's selection, the tool list with why
//! a tool does not fit, *Turn A to*, the Add menu grouped by what the cut does, *Machine this*
//! (`cam.picks.resolve` on the selection: the operations that suit it), *Suggest operations*
//! (`cam.op.suggest`), the part's features (`cam.features`) and work zero by a pick, a place on
//! the stock or a bar's axis (`cam.setup.zero`). Every change is a command; this module only builds
//! its parameters. The slower questions (defaults, features) run off the window thread where
//! there are threads, and here at once on the web and headless.

use std::sync::mpsc::{Receiver, TryRecvError};

use egui::Ui;
use serde_json::{Map, Value, json};
use solidcraft_engine::Sel;
use solidcraft_engine::cam_job_state::{OP_TYPES, op_enabled, op_name, op_types};

use crate::SolveApp;
use crate::manufacture_op_fields::{Field, INDEXABLE, Kind, Section, label_id, rows};
use crate::manufacture_workspace::Stage;
use crate::ui_kit::{self, BackingTooltipExt as _};

/// The groups of the Add menu, in order, and their heading messages.
const GROUPS: &[&str] = &["remove", "walls", "holes", "surfaces", "rotary"];

/// A question asked of the engine off the window thread.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Ask {
    Defaults,
    Features,
    Resolve,
}

/// An answer, for the key it was asked with.
#[derive(Clone, Debug)]
pub(crate) struct Got {
    pub key: String,
    pub value: Result<Value, String>,
}

struct Away {
    ask: Ask,
    key: String,
    rx: Receiver<Result<Value, String>>,
}

/// The state of operations by hand (not part of the job).
#[derive(Default)]
pub struct OpState {
    /// `cam.op.defaults` of the selected operation.
    pub(crate) defaults: Option<Got>,
    /// `cam.features` of the job's part.
    pub(crate) features: Option<Got>,
    /// *Machine this*: `cam.picks.resolve` of the selection.
    pub(crate) resolve: Option<Got>,
    /// The Add menu is open, and what it is filtered by.
    pub add_open: bool,
    pub add_filter: String,
    away: Vec<Away>,
}

fn tr(app: &SolveApp, id: &str) -> String {
    app.tr(id).to_string()
}

/// Each frame: take back the answers that came.
pub(crate) fn collect(app: &mut SolveApp) {
    let mut done = Vec::new();
    for (k, a) in app.manufacture.ops.away.iter().enumerate() {
        match a.rx.try_recv() {
            Ok(v) => done.push((k, Some(v))),
            Err(TryRecvError::Disconnected) => done.push((k, None)),
            Err(TryRecvError::Empty) => {}
        }
    }
    for (k, v) in done.into_iter().rev() {
        let a = app.manufacture.ops.away.remove(k);
        let got = Got { key: a.key, value: v.unwrap_or_else(|| Err(app.tr("ui.manufacture.worker_lost").to_string())) };
        store(app, a.ask, got);
    }
}

fn store(app: &mut SolveApp, ask: Ask, got: Got) {
    let slot = match ask {
        Ask::Defaults => &mut app.manufacture.ops.defaults,
        Ask::Features => &mut app.manufacture.ops.features,
        Ask::Resolve => &mut app.manufacture.ops.resolve,
    };
    *slot = Some(got);
}

/// Whether there is work away.
pub(crate) fn busy(app: &SolveApp) -> bool {
    !app.manufacture.ops.away.is_empty()
}

/// Ask `id` with `p` for `key`, unless that is what is held or asked already.
fn ask(app: &mut SolveApp, what: Ask, key: String, id: &str, p: Value) {
    let held = match what {
        Ask::Defaults => &app.manufacture.ops.defaults,
        Ask::Features => &app.manufacture.ops.features,
        Ask::Resolve => &app.manufacture.ops.resolve,
    };
    if held.as_ref().is_some_and(|g| g.key == key) || app.manufacture.ops.away.iter().any(|a| a.ask == what && a.key == key) {
        return;
    }
    if !crate::manufacture_workspace::threads(app) {
        let value = app.session.execute(id, &p).map_err(|e| e.to_string());
        store(app, what, Got { key, value });
        return;
    }
    let mut copy = Box::new(app.session.scratch());
    let (tx, rx) = std::sync::mpsc::channel();
    let scope = solidcraft_engine::io::vfs::current_scope();
    let id = id.to_string();
    let spawned = std::thread::Builder::new().name("manufacture-ask".into()).spawn(move || {
        let r = solidcraft_engine::io::vfs::scoped(scope, || copy.execute(&id, &p)).map_err(|e| e.to_string());
        let _ = tx.send(r);
    });
    if spawned.is_ok() {
        app.manufacture.ops.away.push(Away { ask: what, key, rx });
    }
}

/// The open job's operations.
fn ops(app: &SolveApp) -> Vec<Value> {
    app.session.cam_job.as_deref().map(|j| j.operations().to_vec()).unwrap_or_default()
}

thread_local! {
    /// Machine and profile files read for whether they have an A axis, by path.
    static AXES: std::cell::RefCell<Vec<(String, bool)>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// Whether machine value `m` (a preset's name or a machine object) has one rotary axis.
fn machine_value_has_a(m: &Value) -> bool {
    m.as_str() == Some("4axis_a") || m.get("rotaries").and_then(Value::as_array).is_some_and(|r| r.len() == 1)
}

/// Whether the machine file or profile at `path` (beside the job) has an A axis: read once.
fn file_has_a(path: &str, key: &str) -> bool {
    if let Some(hit) = AXES.with(|c| c.borrow().iter().find(|(p, _)| p == path).map(|(_, a)| *a)) {
        return hit;
    }
    let v: Option<Value> = solidcraft_engine::io::vfs::read(path, 4 << 20).ok().and_then(|b| serde_json::from_slice(&b).ok());
    let a = v.as_ref().is_some_and(|v| match key {
        "machine" => machine_value_has_a(v),
        // A profile: its machine, or its machine file (beside the profile).
        _ => match (v.get("machine"), v.get("machine_path").and_then(Value::as_str)) {
            (Some(m), _) => machine_value_has_a(m),
            (None, Some(mp)) => {
                let dir = std::path::Path::new(path).parent().map(std::path::Path::to_path_buf).unwrap_or_default();
                let full = if std::path::Path::new(mp).is_relative() { dir.join(mp).to_string_lossy().to_string() } else { mp.to_string() };
                full != path && file_has_a(&full, "machine")
            }
            _ => false,
        },
    });
    AXES.with(|c| {
        let mut c = c.borrow_mut();
        if c.len() > 32 {
            c.clear();
        }
        c.push((path.to_string(), a));
    });
    a
}

/// Whether the job's machine has an A axis: its `machine`, machine file or profile's machine.
pub(crate) fn four_axis(app: &SolveApp) -> bool {
    let Some(j) = app.session.cam_job.as_deref() else { return false };
    if let Some(m) = j.value.get("machine") {
        return machine_value_has_a(m);
    }
    let beside = |p: &str| match j.folder() {
        Some(dir) if std::path::Path::new(p).is_relative() => dir.join(p).to_string_lossy().to_string(),
        _ => p.to_string(),
    };
    if let Some(p) = j.value.get("machine_path").and_then(Value::as_str) {
        return file_has_a(&beside(p), "machine");
    }
    j.value.get("profile_path").and_then(Value::as_str).is_some_and(|p| file_has_a(&beside(p), "profile"))
}

/// Run an edit; its refusal goes to the stage card.
fn report(app: &mut SolveApp, id: &str, p: Value) -> Option<Value> {
    match app.run(id, p) {
        Ok(v) => {
            app.manufacture.error = None;
            Some(v)
        }
        Err(e) => {
            app.manufacture.error = Some(e);
            None
        }
    }
}

/// The faces and edges selected in the design.
fn picked(app: &SolveApp) -> Vec<Sel> {
    app.session.selection.iter().filter(|s| matches!(s, Sel::Face { .. } | Sel::Edge { .. })).cloned().collect()
}

/// The selection as picks (persistent names), through `cam.picks.resolve`.
fn selection_picks(app: &mut SolveApp) -> Option<Vec<Value>> {
    let r = report(app, "cam.picks.resolve", json!({"from_selection": true}))?;
    let picks: Vec<Value> = r.get("picks").and_then(Value::as_array).into_iter().flatten().filter_map(|p| p.get("pick").cloned()).collect();
    (!picks.is_empty()).then_some(picks)
}

/// The type label of `ty` in the person's language.
fn type_label(app: &SolveApp, ty: &str) -> String {
    let id = format!("ui.manufacture.op.{ty}");
    let text = tr(app, &id);
    if text == id { OP_TYPES.iter().find(|(t, _, _)| *t == ty).map_or(ty.to_string(), |(_, l, _)| (*l).to_string()) } else { text }
}

// ---- keys and the context menu -----------------------------------------------------------

/// The workspace's own keys: `M` (Machine this) and `A` (the Add menu). `true` when taken.
pub(crate) fn key(app: &mut SolveApp, key: &str) -> bool {
    if !crate::manufacture_workspace::active(app) || app.session.cam_job.is_none() {
        return false;
    }
    match key {
        "M" => {
            machine_this(app);
            true
        }
        "A" => {
            app.manufacture.ops.add_open = !app.manufacture.ops.add_open;
            if app.manufacture.stage != Stage::Operations {
                let _ = app.run("view.manufacture_stage", json!({"stage": "operations"}));
            }
            true
        }
        _ => false,
    }
}

/// *Machine this*: what the selection is and which operations suit it, in the Operations stage.
pub(crate) fn machine_this(app: &mut SolveApp) {
    let sel = picked(app);
    if sel.is_empty() {
        app.set_status(tr(app, "ui.manufacture.machine_this_hint"), false);
        return;
    }
    let key = format!("{:?}:{}", sel, app.session.model_stamp());
    ask(app, Ask::Resolve, key, "cam.picks.resolve", json!({"from_selection": true}));
    if app.manufacture.stage != Stage::Operations {
        let _ = app.run("view.manufacture_stage", json!({"stage": "operations"}));
    }
}

// ---- the Operations stage ----------------------------------------------------------------

/// The Operations stage: Add, Suggest and Machine this, then the selected operation's form.
pub(crate) fn operations_form(app: &mut SolveApp, ui: &mut Ui) {
    ui.horizontal_wrapped(|ui| {
        let add = ui_kit::button(
            ui,
            tr(app, "ui.manufacture.add_operation"),
            if app.manufacture.ops.add_open { ui_kit::Button::SmallSelected } else { ui_kit::Button::Small },
        );
        crate::scenario::publish_target("manufacture:add_open", add.rect);
        if add.clicked() {
            app.manufacture.ops.add_open = !app.manufacture.ops.add_open;
        }
        let sug = ui_kit::button(ui, tr(app, "ui.manufacture.suggest"), ui_kit::Button::Small);
        crate::scenario::publish_target("manufacture:suggest", sug.rect);
        if sug.clicked()
            && let Some(r) = report(app, "cam.op.suggest", json!({}))
        {
            let n = r.get("added").and_then(Value::as_array).map_or(0, Vec::len);
            app.set_status(crate::i18n::format("ui.manufacture.suggested", &[("count", &n.to_string())]), false);
        }
        let has = !picked(app).is_empty();
        let m = ui_kit::button(ui, tr(app, "ui.manufacture.machine_this"), ui_kit::Button::Enabled(has));
        crate::scenario::publish_target("manufacture:machine_this", m.rect);
        if m.clicked() && has {
            machine_this(app);
        }
    });
    if app.manufacture.ops.add_open {
        add_menu(app, ui);
    }
    machine_this_card(app, ui);
    ui_kit::divider(ui, "manufacture:ops:rule");
    let Some(i) = app.manufacture.selected.filter(|i| *i < ops(app).len()) else {
        ui_kit::text(ui, app.tr("ui.manufacture.select_operation"), ui_kit::Text::Hint("manufacture:ops:none"));
        return;
    };
    op_form(app, ui, i);
}

/// The Add menu: the operation types grouped by what the cut does, filtered as typed.
fn add_menu(app: &mut SolveApp, ui: &mut Ui) {
    let four = four_axis(app);
    let mut filter = app.manufacture.ops.add_filter.clone();
    let hint = tr(app, "ui.manufacture.add_filter");
    let r = ui_kit::menu_filter(ui, egui::Id::unique("manufacture_add_filter"), &mut filter, &hint);
    crate::scenario::publish_target("manufacture:add_filter", r.rect);
    app.manufacture.ops.add_filter = filter.clone();
    let words = filter.trim().to_lowercase();
    let from_selection = !picked(app).is_empty();
    for group in GROUPS.iter().filter(|g| four || **g != "rotary") {
        let items: Vec<(&str, String)> = OP_TYPES
            .iter()
            .filter(|(_, _, g)| g == group)
            .map(|(ty, _, _)| (*ty, type_label(app, ty)))
            .filter(|(ty, label)| words.is_empty() || label.to_lowercase().contains(&words) || ty.contains(&words))
            .collect();
        if items.is_empty() {
            continue;
        }
        ui_kit::text(ui, app.tr(&format!("ui.manufacture.group.{group}")), ui_kit::Text::Heading(group));
        for (ty, label) in items {
            let r = ui_kit::tree_row(
                ui,
                ui.scope_id().with(("mf_add", ty)),
                &ui_kit::Row { depth: 1, compact: true, label: &label, ..Default::default() },
            );
            if let Some(rect) = r.rect {
                crate::scenario::publish_target(&format!("manufacture:add:{ty}"), rect);
            }
            if r.clicked {
                add(app, ty, from_selection, None);
            }
        }
    }
}

/// Add an operation of type `ty` (with the selection as its picks), and select it.
fn add(app: &mut SolveApp, ty: &str, from_selection: bool, tool: Option<u64>) {
    let n = ops(app).len();
    let at = app.manufacture.selected.map(|i| i + 1).unwrap_or(n).min(n);
    let mut op = Map::new();
    op.insert("type".into(), json!(ty));
    if let Some(t) = tool {
        op.insert("tool".into(), json!(t));
    }
    let mut p = json!({"op": op, "at": at});
    if from_selection {
        p["from_selection"] = json!(true);
    }
    if let Some(r) = report(app, "cam.op.add", p) {
        app.manufacture.selected = r.get("index").and_then(Value::as_u64).and_then(|i| usize::try_from(i).ok());
        app.manufacture.ops.add_open = false;
        app.manufacture.ops.add_filter.clear();
        app.manufacture.ops.resolve = None;
    }
}

/// *Machine this*: what the selection reads as, and the operations that suit it, each with the
/// tool it would take.
fn machine_this_card(app: &mut SolveApp, ui: &mut Ui) {
    let Some(got) = app.manufacture.ops.resolve.clone() else {
        if app.manufacture.ops.away.iter().any(|a| a.ask == Ask::Resolve) {
            ui_kit::text(ui, app.tr("ui.manufacture.working"), ui_kit::Text::Hint("manufacture:resolve:working"));
        }
        return;
    };
    ui_kit::divider(ui, "manufacture:machine_this:rule");
    ui_kit::text(ui, app.tr("ui.manufacture.machine_this"), ui_kit::Text::Heading("manufacture:machine_this"));
    let r = match got.value {
        Ok(r) => r,
        Err(e) => {
            ui_kit::text(ui, &e, ui_kit::Text::Error("manufacture:resolve:error"));
            return;
        }
    };
    for (k, p) in r.get("picks").and_then(Value::as_array).into_iter().flatten().enumerate() {
        let reads = p.get("reads").and_then(Value::as_str).unwrap_or_default();
        let line = crate::i18n::format("ui.manufacture.selected_reads", &[("what", reads)]);
        ui_kit::text(ui, &line, ui_kit::Text::Note(&format!("manufacture:resolve:pick:{k}")));
        if let Some(note) = p.get("note").and_then(Value::as_str) {
            ui_kit::text(ui, note, ui_kit::Text::Hint(&format!("manufacture:resolve:note:{k}")));
        }
    }
    let suits: Vec<Value> = r.get("suits").and_then(Value::as_array).cloned().unwrap_or_default();
    for (k, s) in suits.iter().enumerate() {
        let ty = s.get("type").and_then(Value::as_str).unwrap_or_default().to_string();
        let tool = s.get("tool").and_then(Value::as_u64);
        let label = match tool {
            Some(t) => format!("{} \u{b7} T{t}", type_label(app, &ty)),
            None => type_label(app, &ty),
        };
        let why = s.get("why").and_then(Value::as_str).unwrap_or_default().to_string();
        let row = ui_kit::tree_row(
            ui,
            ui.scope_id().with(("mf_suit", k)),
            &ui_kit::Row { depth: 1, compact: true, label: &label, tip: Some(&why), selected: k == 0, ..Default::default() },
        );
        if let Some(rect) = row.rect {
            crate::scenario::publish_target(&format!("manufacture:suit:{k}"), rect);
            crate::scenario::publish_target(&format!("manufacture:suit:{ty}"), rect);
        }
        if row.clicked {
            add(app, &ty, true, tool);
            return;
        }
    }
    if ui_kit::button(ui, app.tr_label("Cancel"), ui_kit::Button::Small).clicked() {
        app.manufacture.ops.resolve = None;
    }
}

/// The operation `op` (an indexed one's inner operation, with its name) and its type.
fn inner_of(op: &Value) -> (String, Value) {
    let (ty, inner) = op_types(op);
    match (ty.as_str(), inner, op.get("operation")) {
        ("indexed" | "indexed_pattern", Some(t), Some(o)) => (t, o.clone()),
        _ => (ty, op.clone()),
    }
}

/// The value at dotted `path` of `v`.
fn at<'a>(v: &'a Value, path: &str) -> Option<&'a Value> {
    path.split('.').try_fold(v, |v, k| v.get(k))
}

/// A value as a field shows it: numbers short, words plain, the rest as JSON.
fn shown(v: &Value) -> String {
    match v {
        Value::Number(n) => n.as_f64().map_or_else(|| n.to_string(), num),
        Value::String(s) => s.clone(),
        Value::Bool(b) => b.to_string(),
        Value::Null => String::new(),
        other => other.to_string(),
    }
}

fn num(x: f64) -> String {
    let r = (x * 1000.0).round() / 1000.0;
    if r == r.trunc() && r.abs() < 1e15 { format!("{}", r as i64) } else { format!("{r}") }
}

/// The defaults of operation `i`, asked for when the job, the model or the operation changed.
fn defaults_of(app: &mut SolveApp, i: usize) -> Option<Value> {
    let rev = app.session.cam_job.as_deref().map_or(0, |j| j.revision);
    let key = format!("{i}:{rev}:{}", app.session.model_stamp());
    ask(app, Ask::Defaults, key.clone(), "cam.op.defaults", json!({"op": i}));
    let got = app.manufacture.ops.defaults.as_ref()?;
    // The last answer stands in while a newer one is worked out, for the same operation.
    if got.key != key && !got.key.starts_with(&format!("{i}:")) {
        return None;
    }
    got.value.as_ref().ok().cloned()
}

/// The form of operation `i`.
fn op_form(app: &mut SolveApp, ui: &mut Ui, i: usize) {
    let op = ops(app)[i].clone();
    let (ty, inner) = inner_of(&op);
    ui_kit::text(ui, &type_label(app, &ty), ui_kit::Text::Heading("manufacture:op:type"));
    if let Some(name) = value_row(app, ui, app.tr("ui.manufacture.name"), &format!("op{i}name"), &op_name(&op), "") {
        report(app, "cam.op.edit", json!({"op": i, "rename": name}));
    }
    let mut on = op_enabled(&op);
    let r = ui_kit::checkbox(ui, app.tr("ui.manufacture.field.enabled"), &mut on);
    crate::scenario::publish_target("manufacture:field:enabled", r.rect);
    if r.changed() {
        report(app, "cam.op.enable", json!({"op": i, "enabled": on}));
    }
    let defaults = defaults_of(app, i);
    if defaults.is_none() {
        ui_kit::text(ui, app.tr("ui.manufacture.working_out"), ui_kit::Text::Hint("manufacture:op:working"));
    }
    if let Some(why) = defaults.as_ref().and_then(|d| d.get("refused")).and_then(Value::as_str) {
        let line = crate::i18n::format("ui.manufacture.not_planned", &[("why", why)]);
        ui_kit::text(ui, &line, ui_kit::Text::Error("manufacture:op:refused"));
    }
    let fields = rows(&ty);
    for section in Section::ALL {
        let here: Vec<&Field> = fields.iter().copied().filter(|f| f.section == section && f.kind != Kind::Header).collect();
        if here.is_empty() && section != Section::Geometry {
            continue;
        }
        heading(ui, app.tr(section.label()));
        match section {
            Section::Geometry => geometry_rows(app, ui, i, &inner, defaults.as_ref()),
            Section::Tool => tool_rows(app, ui, i, &inner, defaults.as_ref()),
            _ => {}
        }
        // With picks, the other ways of giving the geometry are shown only when the job gives them.
        let picked = inner.get("picks").is_some();
        let alternative = |f: &Field| {
            matches!(f.key, "sketch" | "profiles" | "polygon" | "feature" | "curve" | "points" | "holes" | "centers" | "circles" | "sketch_points")
                && at(&inner, f.key).is_none()
        };
        for f in here.iter().filter(|f| !matches!(f.kind, Kind::Picks | Kind::ToolList) && !(picked && alternative(f))) {
            field_row(app, ui, i, f, &inner, defaults.as_ref());
        }
        if section == Section::Tool && four_axis(app) && INDEXABLE.contains(&ty.as_str()) {
            heading(ui, app.tr("ui.manufacture.section.a_axis"));
            turn_a(app, ui, i, &op);
        }
    }
    // Keys an agent wrote that the form does not edit: shown as they are, with why.
    for (k, why) in crate::manufacture_op_fields::AGENT_ONLY_KEYS.iter().filter(|(k, _)| *k != "type") {
        if let Some(v) = inner.get(*k) {
            ui_kit::text(ui, &format!("{k}: {}", shown(v)), ui_kit::Text::MonoNote(&format!("manufacture:op:agent:{k}"))).backing_on_hover_text(*why);
        }
    }
}

fn heading(ui: &mut Ui, text: &str) {
    ui.add_space(crate::theme::Tokens::get().space_4);
    ui_kit::text(ui, text, ui_kit::Text::Heading(text));
}

/// A value row whose text is kept while it is typed and given back on Enter (`hint`: the default,
/// dim). A draft is dropped when the job changes under it.
fn value_row(app: &SolveApp, ui: &mut Ui, label: &str, key: &str, current: &str, hint: &str) -> Option<String> {
    let revision = app.session.cam_job.as_deref().map_or(0, |j| j.revision);
    let id = egui::Id::unique(("manufacture_op_draft", key));
    let mut draft =
        ui.ctx().data(|d| d.get_temp::<(String, u64)>(id)).filter(|(_, r)| *r == revision).map_or_else(|| current.to_string(), |(d, _)| d);
    let (enter, changed) = ui_kit::text_row(ui, label, &mut draft, hint);
    if enter {
        ui.ctx().data_mut(|d| d.remove::<(String, u64)>(id));
        return (draft != current).then_some(draft);
    }
    if changed {
        ui.ctx().data_mut(|d| d.insert_temp(id, (draft, revision)));
    }
    None
}

/// The default of `path` from `cam.op.defaults`: (value, source, why).
fn default_of(defaults: Option<&Value>, path: &str) -> Option<(Option<Value>, String, String)> {
    let f = defaults?.get("fields")?.as_array()?.iter().find(|f| f.get("path").and_then(Value::as_str) == Some(path))?;
    Some((
        f.get("value").cloned().filter(|v| !v.is_null()),
        f.get("source").and_then(Value::as_str).unwrap_or("rule").to_string(),
        f.get("why").and_then(Value::as_str).unwrap_or_default().to_string(),
    ))
}

/// What a typed value is for a field of `kind` (`None`: not a value of that kind).
fn parse(kind: Kind, text: &str) -> Option<Value> {
    let t = text.trim();
    match kind {
        Kind::Number | Kind::Height => t.parse::<f64>().ok().filter(|x| x.is_finite()).map(|x| json!(x)),
        Kind::Count => t.parse::<u64>().ok().map(|x| json!(x)),
        Kind::Text => (!t.is_empty()).then(|| json!(t)),
        Kind::Json => serde_json::from_str::<Value>(t).ok(),
        _ => None,
    }
}

/// One key's row: its value (dim: the default), where it comes from, and Reset when set here.
fn field_row(app: &mut SolveApp, ui: &mut Ui, i: usize, f: &Field, inner: &Value, defaults: Option<&Value>) {
    let label = tr(app, &label_id(f.key));
    let given = at(inner, f.key).cloned();
    let (def, source, why) = default_of(defaults, f.key).unwrap_or((None, "rule".into(), String::new()));
    let edit = |app: &mut SolveApp, v: Option<Value>| match v {
        Some(v) => {
            let mut set = Map::new();
            set.insert(f.key.to_string(), v);
            report(app, "cam.op.edit", json!({"op": i, "set": set}));
        }
        None => {
            report(app, "cam.op.edit", json!({"op": i, "unset": [f.key]}));
        }
    };
    let target = format!("manufacture:field:{}", f.key);
    match f.kind {
        Kind::Flag => {
            let mut on = given.as_ref().or(def.as_ref()).and_then(Value::as_bool).unwrap_or(false);
            let r = ui_kit::checkbox(ui, &label, &mut on);
            crate::scenario::publish_target(&target, r.rect);
            if r.changed() {
                edit(app, Some(json!(on)));
            }
        }
        Kind::Choice(words) => {
            let options: Vec<String> = words.iter().map(|w| tr(app, &format!("ui.manufacture.choice.{w}"))).collect();
            let now = given.as_ref().or(def.as_ref()).and_then(Value::as_str).and_then(|w| words.iter().position(|x| *x == w)).unwrap_or(0);
            let mut sel = now;
            let refs: Vec<&str> = options.iter().map(String::as_str).collect();
            let r = ui_kit::choice_row(ui, &label, &target, &mut sel, &refs);
            crate::scenario::publish_target(&target, r.rect);
            if sel != now
                && let Some(w) = words.get(sel)
            {
                edit(app, Some(json!(w)));
            }
        }
        _ => {
            let follows = given.as_ref().filter(|g| g.is_object() && g.get("from").is_some());
            let current = match (&given, follows) {
                (_, Some(r)) => {
                    let off = r.get("offset").and_then(Value::as_f64).unwrap_or(0.0);
                    crate::i18n::format("ui.manufacture.follows_face", &[("offset", &num(off))])
                }
                (Some(g), None) => shown(g),
                (None, None) => String::new(),
            };
            let hint = def.as_ref().map(shown).unwrap_or_else(|| why.clone());
            if let Some(t) = value_row(app, ui, &label, &format!("op{i}:{}", f.key), &current, &hint) {
                if t.trim().is_empty() {
                    edit(app, None);
                } else if let Some(v) = parse(f.kind, &t) {
                    edit(app, Some(v));
                } else {
                    app.manufacture.error = Some(crate::i18n::format("ui.manufacture.bad_value", &[("field", &label)]));
                }
            }
        }
    }
    // Reset, a height from a face, and where the value comes from.
    ui.horizontal_wrapped(|ui| {
        let src = if given.is_some() { "given" } else { source.as_str() };
        let chip = tr(app, &format!("ui.manufacture.source.{src}"));
        let tip = if given.is_some() { chip.clone() } else { why.clone() };
        if given.is_some() {
            let r = ui_kit::button(ui, tr(app, "ui.manufacture.reset"), ui_kit::Button::Small);
            crate::scenario::publish_target(&format!("{target}:reset"), r.rect);
            if r.clicked() {
                edit(app, None);
            }
        }
        if f.kind == Kind::Height {
            let face = picked(app).iter().any(|s| matches!(s, Sel::Face { .. }));
            let r = ui_kit::button(ui, tr(app, "ui.manufacture.from_face"), ui_kit::Button::Enabled(face));
            crate::scenario::publish_target(&format!("{target}:from_face"), r.rect);
            if r.clicked()
                && face
                && let Some(p) = selection_picks(app).and_then(|p| p.into_iter().next())
            {
                edit(app, Some(json!({"from": p})));
            }
        }
        ui_kit::text(ui, &chip, ui_kit::Text::Hint(&format!("{target}:source"))).backing_on_hover_text(&tip);
    });
}

/// The Geometry section: the picks and how each was found, and taking the selection as picks.
fn geometry_rows(app: &mut SolveApp, ui: &mut Ui, i: usize, inner: &Value, defaults: Option<&Value>) {
    let picks: Vec<Value> = inner.get("picks").and_then(Value::as_array).cloned().unwrap_or_default();
    // This operation's picks (work zero's `origin_from` is reported there too, without an operation).
    let resolved: Vec<Value> = defaults
        .and_then(|d| d.get("picks_resolved"))
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter(|r| r.get("op").is_some_and(Value::is_object))
        .cloned()
        .collect();
    if picks.is_empty() {
        ui_kit::text(ui, app.tr("ui.manufacture.picks_none"), ui_kit::Text::Hint("manufacture:picks:none"));
    }
    for (k, p) in picks.iter().enumerate() {
        let r = resolved.iter().find(|r| r.get("pick").and_then(Value::as_u64) == Some(k as u64));
        let what = r
            .and_then(|r| r.get("feature").and_then(Value::as_str))
            .map(str::to_string)
            .or_else(|| p.get("sketch").and_then(Value::as_str).map(str::to_string))
            .unwrap_or_else(|| p.get("face").or_else(|| p.get("edge")).map(shown).unwrap_or_default());
        let by = r.and_then(|r| r.get("by")).and_then(Value::as_str).unwrap_or("name");
        let how = tr(app, &format!("ui.manufacture.found_by.{by}"));
        let line = crate::i18n::format("ui.manufacture.pick_line", &[("n", &(k + 1).to_string()), ("what", &what), ("how", &how)]);
        let style = match (by, r.and_then(|r| r.get("note"))) {
            ("lost", _) => ui_kit::Text::Error("manufacture:pick"),
            (_, Some(_)) => ui_kit::Text::Note("manufacture:pick"),
            _ => ui_kit::Text::Hint("manufacture:pick"),
        };
        let resp = ui_kit::text(ui, &line, style);
        if let Some(note) = r.and_then(|r| r.get("note")).and_then(Value::as_str) {
            resp.backing_on_hover_text(note);
        }
    }
    ui.horizontal_wrapped(|ui| {
        let has = !picked(app).is_empty();
        let r = ui_kit::button(ui, tr(app, "ui.manufacture.use_selection"), ui_kit::Button::Enabled(has));
        crate::scenario::publish_target("manufacture:use_selection", r.rect);
        if r.clicked()
            && has
            && let Some(list) = selection_picks(app)
        {
            report(app, "cam.op.edit", json!({"op": i, "set": {"picks": list}}));
        }
        let show = ui_kit::button(ui, tr(app, "ui.manufacture.show_picks"), ui_kit::Button::Enabled(!resolved.is_empty()));
        crate::scenario::publish_target("manufacture:show_picks", show.rect);
        if show.clicked() {
            let items: Vec<Value> = resolved
                .iter()
                .zip(&picks)
                .filter_map(|(r, p)| {
                    let body = r.get("body").and_then(Value::as_str)?;
                    let index = r.get("face").and_then(Value::as_u64)?;
                    let point = p.get("at").cloned().unwrap_or(json!([0, 0, 0]));
                    Some(json!({"type": "face", "body": body, "index": index, "point": point}))
                })
                .collect();
            if !items.is_empty() {
                let _ = app.run("select.set", json!({"items": items}));
            }
        }
    });
}

/// The Tool section: the tool it cuts with and why, and the job's tools, those that do not fit
/// dim with the reason.
fn tool_rows(app: &mut SolveApp, ui: &mut Ui, i: usize, inner: &Value, defaults: Option<&Value>) {
    let given = inner.get("tool").and_then(Value::as_u64);
    let chosen = given.or_else(|| defaults.and_then(|d| d.get("tool")).and_then(Value::as_u64));
    let why = if given.is_some() {
        tr(app, "ui.manufacture.source.given")
    } else {
        defaults.and_then(|d| d.get("tool_why")).and_then(Value::as_str).unwrap_or_default().to_string()
    };
    if let Some(t) = chosen {
        let line = crate::i18n::format("ui.manufacture.tool_chosen", &[("tool", &format!("T{t}")), ("why", &why)]);
        ui_kit::text(ui, &line, ui_kit::Text::Note("manufacture:tool:chosen"));
    }
    let tools: Vec<Value> = defaults.and_then(|d| d.get("tools")).and_then(Value::as_array).cloned().unwrap_or_default();
    // The list folds: the tools that fit first, then the rest dim with why.
    let fit = tools.iter().filter(|t| t.get("fits").and_then(Value::as_bool) == Some(true)).count();
    let id = egui::Id::unique(("manufacture_tool_list", i));
    let mut open = ui.ctx().data(|d| d.get_temp::<bool>(id)).unwrap_or(false);
    let label = crate::i18n::format("ui.manufacture.tool_list", &[("fit", &fit.to_string()), ("other", &(tools.len() - fit).to_string())]);
    let r = ui_kit::tree_row(
        ui,
        ui.scope_id().with(("mf_tool_list", i)),
        &ui_kit::Row { depth: 1, compact: true, fold: Some(open), label: &label, ..Default::default() },
    );
    if let Some(rect) = r.rect {
        crate::scenario::publish_target("manufacture:tool:list", rect);
    }
    if r.clicked || r.fold {
        open = !open;
        ui.ctx().data_mut(|d| d.insert_temp(id, open));
    }
    for t in tools.iter().filter(|_| open) {
        let n = t.get("number").and_then(Value::as_u64).unwrap_or(0);
        let fits = t.get("fits").and_then(Value::as_bool).unwrap_or(false);
        let label = t.get("label").and_then(Value::as_str).unwrap_or_default().to_string();
        let why_not = t.get("why_not").and_then(Value::as_str).unwrap_or_default().to_string();
        let r = ui_kit::tree_row(
            ui,
            ui.scope_id().with(("mf_tool_fit", n)),
            &ui_kit::Row {
                depth: 2,
                compact: true,
                label: &label,
                subtitle: &why_not,
                dim: !fits,
                selected: chosen == Some(n),
                tip: Some(&why_not),
                ..Default::default()
            },
        );
        if let Some(rect) = r.rect {
            crate::scenario::publish_target(&format!("manufacture:tool:{n}"), rect);
        }
        if r.clicked && fits && given != Some(n) {
            report(app, "cam.op.edit", json!({"op": i, "set": {"tool": n}}));
        }
    }
    let r = ui_kit::button(ui, tr(app, "ui.manufacture.pick_tool"), ui_kit::Button::Small);
    crate::scenario::publish_target("manufacture:tool:auto", r.rect);
    if r.clicked() {
        report(app, "cam.op.edit", json!({"op": i, "set": {"tool": {}}}));
    }
}

/// *Turn A to*: one angle, several, or a number of turns evenly round from a first angle.
fn turn_a(app: &mut SolveApp, ui: &mut Ui, i: usize, op: &Value) {
    let ty = op.get("type").and_then(Value::as_str).unwrap_or_default();
    let mode = match ty {
        "indexed_pattern" if op.get("count").is_some() => 2,
        "indexed_pattern" => 1,
        _ => 0,
    };
    let id = egui::Id::unique(("manufacture_turn_mode", i, ty));
    let mut pick = ui.ctx().data(|d| d.get_temp::<usize>(id)).unwrap_or(mode);
    let options = [tr(app, "ui.manufacture.turn_one"), tr(app, "ui.manufacture.turn_list"), tr(app, "ui.manufacture.turn_even")];
    let refs: Vec<&str> = options.iter().map(String::as_str).collect();
    if ui_kit::segmented(
        ui,
        &mut pick,
        &refs,
        ui_kit::Segments { targets: &["manufacture:turn:one", "manufacture:turn:list", "manufacture:turn:even"], dense: true, ..Default::default() },
    ) {
        ui.ctx().data_mut(|d| d.insert_temp(id, pick));
    }
    match pick {
        0 => {
            let now = if ty == "indexed" { op.get("a_deg").map(shown).unwrap_or_default() } else { "0".into() };
            if let Some(t) = value_row(app, ui, app.tr("ui.manufacture.turn_a"), &format!("op{i}:a"), &now, "0")
                && let Ok(a) = t.trim().parse::<f64>()
            {
                report(app, "cam.op.edit", json!({"op": i, "a_deg": a}));
            }
        }
        1 => {
            let now = op.get("a_deg").and_then(Value::as_array).map(|a| a.iter().map(shown).collect::<Vec<_>>().join(", ")).unwrap_or_default();
            if let Some(t) = value_row(app, ui, app.tr("ui.manufacture.turn_angles"), &format!("op{i}:alist"), &now, "0, 90, 180, 270") {
                let angles: Vec<f64> = t.split(',').filter_map(|x| x.trim().parse::<f64>().ok()).collect();
                if !angles.is_empty() {
                    report(app, "cam.op.edit", json!({"op": i, "a_deg": angles}));
                }
            }
        }
        _ => {
            let count = op.get("count").map(shown).unwrap_or_default();
            let start = op.get("start_deg").map(shown).unwrap_or_default();
            let c = value_row(app, ui, app.tr("ui.manufacture.turn_count"), &format!("op{i}:count"), &count, "4");
            let s = value_row(app, ui, app.tr("ui.manufacture.turn_start"), &format!("op{i}:start"), &start, "0");
            if c.is_some() || s.is_some() {
                let n = c.as_deref().unwrap_or(&count).trim().parse::<u64>().unwrap_or(4);
                let st = s.as_deref().unwrap_or(&start).trim().parse::<f64>().unwrap_or(0.0);
                report(app, "cam.op.edit", json!({"op": i, "a_deg": {"count": n, "start_deg": st}}));
            }
        }
    }
}

// ---- Found in the part -------------------------------------------------------------------

/// What `cam.features` is asked for the job's part.
fn features_params(app: &SolveApp) -> Option<Value> {
    let j = app.session.cam_job.as_deref()?;
    let v = &j.value;
    let mut p = Map::new();
    if let Some(b) = v.get("bodies") {
        p.insert("bodies".into(), b.clone());
    }
    if let Some(o) = v.get("origin") {
        p.insert("origin".into(), o.clone());
    }
    if let Some(t) = v.get("tolerance") {
        p.insert("tolerance".into(), t.clone());
    }
    p.insert("axis".into(), json!(if four_axis(app) { "a" } else { "z" }));
    p.insert("picks".into(), json!(true));
    Some(Value::Object(p))
}

/// One row of *Found in the part*: its feature ids, words, and the faces it selects.
struct FoundRow {
    id: String,
    label: String,
    faces: Vec<(String, u64)>,
}

fn found_rows_of(r: &Value) -> Vec<(&'static str, Vec<FoundRow>)> {
    let list = |k: &str| r.get(k).and_then(Value::as_array).cloned().unwrap_or_default();
    let n = |v: &Value, k: &str| v.get(k).and_then(Value::as_f64).map_or_else(|| "?".into(), num);
    let s = |v: &Value, k: &str| v.get(k).and_then(Value::as_str).unwrap_or_default().to_string();
    let faces_of = |v: &Value| -> Vec<(String, u64)> {
        let body = s(v, "body");
        let mut f: Vec<u64> = v.get("faces").and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_u64).collect();
        f.extend(v.get("floor_face").and_then(Value::as_u64));
        f.extend(v.get("face").and_then(Value::as_u64));
        f.into_iter().map(|x| (body.clone(), x)).collect()
    };
    let side = |v: &Value| v.get("a_deg").and_then(Value::as_f64).filter(|a| a.abs() > 1e-9).map_or(String::new(), |a| format!(", A{}", num(a)));
    let holes = list("holes");
    let mut out = Vec::new();
    // Holes by group: "4 x 5 mm through".
    let groups: Vec<FoundRow> = list("hole_groups")
        .iter()
        .map(|g| {
            let ids: Vec<String> =
                g.get("ids").and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_str).map(str::to_string).collect();
            let depth =
                if g.get("through").and_then(Value::as_bool) == Some(true) { "through".to_string() } else { format!("{} deep", n(g, "depth")) };
            let faces = holes.iter().filter(|h| ids.contains(&s(h, "id"))).flat_map(&faces_of).collect();
            FoundRow { id: ids.join(","), label: format!("{} \u{d7} {} mm {depth}{}", n(g, "count"), n(g, "diameter"), side(g)), faces }
        })
        .collect();
    out.push(("holes", groups));
    let pockets = list("pockets")
        .iter()
        .map(|p| {
            let b = &p["bounds"];
            let w = b[1][0].as_f64().unwrap_or(0.0) - b[0][0].as_f64().unwrap_or(0.0);
            let h = b[1][1].as_f64().unwrap_or(0.0) - b[0][1].as_f64().unwrap_or(0.0);
            let corner = p.get("min_corner_radius").and_then(Value::as_f64).filter(|r| *r > 0.0).map_or(String::new(), |r| format!(", R{}", num(r)));
            FoundRow { id: s(p, "id"), label: format!("{} \u{d7} {}, {} deep{corner}{}", num(w), num(h), n(p, "depth"), side(p)), faces: faces_of(p) }
        })
        .collect();
    out.push(("pockets", pockets));
    let faces = list("faces")
        .iter()
        .filter(|f| !matches!(f.get("role").and_then(Value::as_str), Some("pocket_floor" | "hole_floor")))
        .map(|f| FoundRow {
            id: s(f, "id"),
            label: format!("{} Z {}{}", crate::i18n::tr(&format!("ui.manufacture.role.{}", s(f, "role"))), n(f, "z"), side(f)),
            faces: faces_of(f),
        })
        .collect();
    out.push(("faces", faces));
    let outlines = list("profiles")
        .iter()
        .map(|p| FoundRow { id: s(p, "id"), label: format!("Z {} \u{2013} {}{}", n(p, "top_z"), n(p, "bottom_z"), side(p)), faces: Vec::new() })
        .collect();
    out.push(("outlines", outlines));
    let rounds = list("fillets")
        .iter()
        .chain(list("chamfers").iter())
        .map(|x| {
            let what = x.get("radius").and_then(Value::as_f64).map_or_else(|| s(x, "id"), |r| format!("R{}", num(r)));
            FoundRow { id: s(x, "id"), label: format!("{what}{}", side(x)), faces: faces_of(x) }
        })
        .collect();
    out.push(("rounds", rounds));
    let surfaces = list("surfaces")
        .iter()
        .map(|x| FoundRow { id: s(x, "id"), label: format!("Z {} \u{2013} {}{}", n(x, "bottom_z"), n(x, "top_z"), side(x)), faces: faces_of(x) })
        .collect();
    out.push(("surfaces", surfaces));
    let turned = list("turned")
        .iter()
        .map(|x| FoundRow {
            id: s(x, "id"),
            label: format!("\u{2300}{} X {} \u{2013} {}", n(x, "diameter"), n(x, "x_min"), n(x, "x_max")),
            faces: faces_of(x),
        })
        .collect();
    out.push(("turned", turned));
    out
}

/// *Found in the part*: the part's features, a row each (holes by group); a click selects its
/// faces and says what suits them (Machine this).
pub(crate) fn found_rows(app: &mut SolveApp, ui: &mut Ui) {
    let Some(p) = features_params(app) else { return };
    let key = format!("{}:{p}", app.session.model_stamp());
    ask(app, Ask::Features, key.clone(), "cam.features", p);
    ui.add_space(crate::theme::Tokens::get().space_4);
    ui_kit::text(ui, app.tr("ui.manufacture.found_in_part"), ui_kit::Text::Heading("manufacture:found"));
    let Some(got) = app.manufacture.ops.features.clone() else {
        ui_kit::text(ui, app.tr("ui.manufacture.working"), ui_kit::Text::Hint("manufacture:found:working"));
        return;
    };
    let r = match got.value {
        Ok(r) => r,
        Err(e) => {
            ui_kit::text(ui, &e, ui_kit::Text::Hint("manufacture:found:error"));
            return;
        }
    };
    let picks = r.get("picks").cloned().unwrap_or(Value::Null);
    let mut any = false;
    for (group, list) in found_rows_of(&r) {
        if list.is_empty() {
            continue;
        }
        any = true;
        let heading = tr(app, &format!("ui.manufacture.found_{group}"));
        ui_kit::tree_row(
            ui,
            ui.scope_id().with(("mf_found_group", group)),
            &ui_kit::Row { depth: 1, compact: true, label: &heading, dim: true, ..Default::default() },
        );
        for row in list {
            let resp = ui_kit::tree_row(
                ui,
                ui.scope_id().with(("mf_found", &row.id)),
                &ui_kit::Row { depth: 2, compact: true, label: &row.label, secondary: &row.id, ..Default::default() },
            );
            if let Some(rect) = resp.rect {
                crate::scenario::publish_target(&format!("manufacture:found:{}", row.id), rect);
            }
            if resp.clicked && !row.faces.is_empty() {
                let items: Vec<Value> = row
                    .faces
                    .iter()
                    .enumerate()
                    .map(|(k, (body, f))| {
                        let id = row.id.split(',').nth(k.min(row.id.split(',').count().saturating_sub(1))).unwrap_or(&row.id);
                        let point = picks.get(id).and_then(|l| l.get(0)).and_then(|p| p.get("at")).cloned().unwrap_or(json!([0, 0, 0]));
                        json!({"type": "face", "body": body, "index": f, "point": point})
                    })
                    .collect();
                if app.run("select.set", json!({"items": items})).is_ok() {
                    machine_this(app);
                }
            }
        }
    }
    if !any {
        ui_kit::text(ui, app.tr("ui.manufacture.found_none"), ui_kit::Text::Hint("manufacture:found:none"));
    }
}

// ---- work zero ---------------------------------------------------------------------------

/// The Setup form's work-zero rows: at the selection, on the stock, on a bar's axis, and again
/// from the way it was set.
pub(crate) fn zero_rows(app: &mut SolveApp, ui: &mut Ui) {
    let Some(job) = app.session.cam_job.as_deref().map(|j| j.value.clone()) else { return };
    if let Some(from) = job.get("origin_from") {
        let how = match from {
            Value::Object(m) if m.contains_key("pick") => "pick",
            Value::Object(m) if m.contains_key("stock") => "stock",
            _ => "axis",
        };
        let line = crate::i18n::format("ui.manufacture.zero_how", &[("how", &tr(app, &format!("ui.manufacture.zero_by.{how}")))]);
        ui_kit::text(ui, &line, ui_kit::Text::Hint("manufacture:zero:how"));
        let drift = app
            .session
            .cam_job
            .as_deref()
            .and_then(|j| j.generated.as_ref())
            .and_then(|g| g.findings.iter().find(|f| f.check == "work_zero").map(|f| f.message.clone()));
        if let Some(d) = drift {
            ui_kit::text(ui, &d, ui_kit::Text::Error("manufacture:zero:drift"));
        }
        let r = ui_kit::button(ui, tr(app, "ui.manufacture.zero_refresh"), ui_kit::Button::Small);
        crate::scenario::publish_target("manufacture:zero:refresh", r.rect);
        if r.clicked() {
            report(app, "cam.setup.zero", json!({"refresh": true}));
        }
    }
    let sel = picked(app);
    ui.horizontal_wrapped(|ui| {
        let r = ui_kit::button(ui, tr(app, "ui.manufacture.zero_selection"), ui_kit::Button::Enabled(sel.len() == 1));
        crate::scenario::publish_target("manufacture:zero:selection", r.rect);
        if r.clicked()
            && let Some(p) = selection_picks(app).and_then(|p| p.into_iter().next())
        {
            report(app, "cam.setup.zero", json!({"pick": p}));
        }
        if four_axis(app) {
            let face = sel.iter().any(|s| matches!(s, Sel::Face { .. }));
            let r = ui_kit::button(ui, tr(app, "ui.manufacture.zero_axis"), ui_kit::Button::Enabled(face));
            crate::scenario::publish_target("manufacture:zero:axis", r.rect);
            if r.clicked()
                && let Some(p) = selection_picks(app).and_then(|p| p.into_iter().next())
            {
                report(app, "cam.setup.zero", json!({"axis": p, "x_end": "stock_low"}));
            }
        }
    });
    if sel.is_empty() {
        ui_kit::text(ui, app.tr("ui.manufacture.zero_select_hint"), ui_kit::Text::Hint("manufacture:zero:hint"));
    }
    // On the stock: X and Y low, middle or high, Z top, middle or bottom.
    if job.get("stock").is_some() {
        let grid = job.get("origin_from").and_then(|o| o.get("stock")).cloned().unwrap_or(json!({"x": "low", "y": "low", "z": "top"}));
        let mut place = [0usize; 3];
        let words: [&[&str]; 3] = [&["low", "middle", "high"], &["low", "middle", "high"], &["top", "middle", "bottom"]];
        for (k, axis) in ["x", "y", "z"].iter().enumerate() {
            place[k] = grid.get(*axis).and_then(Value::as_str).and_then(|w| words[k].iter().position(|x| *x == w)).unwrap_or(0);
        }
        let id = egui::Id::unique("manufacture_zero_grid");
        let mut chosen: [usize; 3] = ui.ctx().data(|d| d.get_temp(id)).unwrap_or(place);
        for (k, axis) in ["X", "Y", "Z"].iter().enumerate() {
            let options: Vec<String> = words[k].iter().map(|w| tr(app, &format!("ui.manufacture.grid.{w}"))).collect();
            let refs: Vec<&str> = options.iter().map(String::as_str).collect();
            let label = crate::i18n::format("ui.manufacture.zero_grid_axis", &[("axis", axis)]);
            let targets: Vec<String> = words[k].iter().map(|w| format!("manufacture:zero:{}:{w}", axis.to_lowercase())).collect();
            let target_refs: Vec<&str> = targets.iter().map(String::as_str).collect();
            ui_kit::row(ui, &label, |ui| {
                ui_kit::segmented(
                    ui,
                    &mut chosen[k],
                    &refs,
                    ui_kit::Segments { targets: &target_refs, dense: true, intrinsic: true, ..Default::default() },
                );
            });
        }
        ui.ctx().data_mut(|d| d.insert_temp(id, chosen));
        let r = ui_kit::button(ui, tr(app, "ui.manufacture.zero_stock"), ui_kit::Button::Small);
        crate::scenario::publish_target("manufacture:zero:stock", r.rect);
        if r.clicked() {
            let p = json!({"stock": {"x": words[0][chosen[0].min(2)], "y": words[1][chosen[1].min(2)], "z": words[2][chosen[2].min(2)]}});
            report(app, "cam.setup.zero", p);
        }
    }
}

/// Every field label the form can show, for the catalog test.
#[cfg(test)]
pub(crate) fn label_ids() -> Vec<String> {
    let mut v: Vec<String> = crate::manufacture_op_fields::FIELDS.iter().map(|f| label_id(f.key)).collect();
    v.sort();
    v.dedup();
    v
}
