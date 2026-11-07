//! The Manufacture workspace's chrome (state and lifecycle in `manufacture_workspace`): the
//! stage bar in place of the ribbon, the job tree card, and the stage card with the Setup form,
//! the selected operation, the Check list of findings and the Program form. Every change is a
//! command (`cam.job.edit`, `cam.op.*`, `cam.generate`, `cam.finding.go`, `cam.post`); these
//! cards only build their parameters.

use egui::{Rect, Ui};
use serde_json::{Map, Value, json};
use solidcraft_engine::cam_job_state::{OP_TYPES, op_enabled, op_name, op_types};
use solidcraft_engine::schema::cam_job_edit::OpStatus;

use crate::SolveApp;
use crate::manufacture_workspace::{Banner, ShowPaths, Stage, run_heavy};
use crate::ui_kit::{self, BackingTooltipExt as _};

/// Drops the workspace's cards from the shell's layout (on leaving).
pub(crate) fn reset_shell(app: &mut SolveApp) {
    app.shell.workspace_panels.retain(|id, _| !id.starts_with("manufacture:"));
}

/// The stage bar: four numbered stages, Generate and Simulate, and the stage's tools.
pub(crate) fn stage_bar(app: &mut SolveApp, ui: &mut Ui, rect: Rect) {
    crate::scenario::publish_rect("manufacture:stage_bar", rect);
    let labels: Vec<String> = Stage::ALL.iter().map(|s| app.tr(s.label()).to_string()).collect();
    let selected = Stage::ALL.iter().position(|s| *s == app.manufacture.stage).unwrap_or(0);
    let open = app.session.cam_job.is_some();
    let busy = app.manufacture.busy().is_some();
    let why = if !open {
        Some(app.tr("ui.manufacture.no_job").to_string())
    } else if busy {
        Some(app.tr("ui.manufacture.working").to_string())
    } else {
        None
    };
    let (generate, simulate) = (app.tr("ui.manufacture.generate").to_string(), app.tr("ui.manufacture.simulate").to_string());
    let actions = [
        ui_kit::StageAction { icon: Some("play"), label: &generate, enabled: why.is_none(), primary: true, reason: why.as_deref() },
        ui_kit::StageAction { icon: None, label: &simulate, enabled: why.is_none(), primary: false, reason: why.as_deref() },
    ];
    let choice = ui_kit::StageBar::row(ui, rect, false, |ui| {
        ui_kit::StageBar::header(ui, selected, &labels.iter().map(String::as_str).collect::<Vec<_>>(), &actions)
    });
    if let Some(choice) = choice {
        for (stage, chip) in Stage::ALL.iter().zip(&choice.chips) {
            crate::scenario::publish_target(&format!("manufacture:stage:{}", stage.key()), *chip);
        }
        for (name, r) in ["generate", "simulate"].iter().zip(&choice.actions) {
            crate::scenario::publish_target(&format!("manufacture:{name}"), *r);
        }
        if let Some(stage) = choice.stage.and_then(|i| Stage::ALL.get(i)) {
            let _ = app.run("view.manufacture_stage", json!({"stage": stage.key()}));
        }
        match choice.action {
            Some(0) => generate_now(app),
            Some(1) => crate::manufacture_workspace::simulate(app),
            _ => {}
        }
    }
    ui_kit::StageBar::row(ui, rect, true, |ui| stage_tools(app, ui));
}

/// Generate the open job (with the simulation when asked).
pub(crate) fn generate_now(app: &mut SolveApp) {
    let p = if app.manufacture.also_simulate { json!({"simulate": true}) } else { json!({}) };
    run_heavy(app, "cam.generate", p);
}

fn tool_button(ui: &mut Ui, label: &str, icon: &str, enabled: bool, target: &str) -> bool {
    let r = ui_kit::compact_icon_label_button(ui, label, icon, enabled);
    crate::scenario::publish_target(target, r.rect);
    r.clicked() && enabled
}

fn stage_tools(app: &mut SolveApp, ui: &mut Ui) {
    let open = app.session.cam_job.is_some();
    if let Some(what) = app.manufacture.busy() {
        let label = if what == "cam.post" { "ui.manufacture.writing" } else { "ui.manufacture.generating" };
        ui_kit::StageBar::note(ui, app.tr(label));
        return;
    }
    match app.manufacture.stage {
        Stage::Setup => {
            let dirty = app.session.cam_job.as_deref().is_some_and(|j| j.dirty());
            if tool_button(ui, app.tr("ui.manufacture.save"), "save", open && dirty, "manufacture:save") {
                report(app, "cam.job.save", json!({}));
            }
            if tool_button(ui, app.tr("ui.manufacture.reload"), "refresh", open, "manufacture:reload") {
                report(app, "cam.job.reload", json!({}));
            }
            if tool_button(ui, app.tr("ui.manufacture.library_refresh"), "folder", open, "manufacture:library") {
                crate::manufacture_workspace::refresh_library(app);
            }
        }
        Stage::Operations => {
            let sel = app.manufacture.selected.filter(|i| *i < ops(app).len());
            let n = ops(app).len();
            let on = sel.and_then(|i| ops(app).get(i).map(op_enabled)).unwrap_or(true);
            if tool_button(ui, app.tr("ui.manufacture.duplicate"), "copy", sel.is_some(), "manufacture:duplicate")
                && let Some(i) = sel
            {
                op_command(app, "cam.op.duplicate", json!({"op": i}), Some(i + 1));
            }
            if tool_button(ui, app.tr("ui.manufacture.move_up"), "arrow_up", sel.is_some_and(|i| i > 0), "manufacture:up")
                && let Some(i) = sel
            {
                op_command(app, "cam.op.move", json!({"op": i, "to": i - 1}), Some(i - 1));
            }
            if tool_button(ui, app.tr("ui.manufacture.move_down"), "arrow_down", sel.is_some_and(|i| i + 1 < n), "manufacture:down")
                && let Some(i) = sel
            {
                op_command(app, "cam.op.move", json!({"op": i, "to": i + 1}), Some(i + 1));
            }
            let toggle = if on { "ui.manufacture.turn_off" } else { "ui.manufacture.turn_on" };
            if tool_button(ui, app.tr(toggle), "eye", sel.is_some(), "manufacture:enable")
                && let Some(i) = sel
            {
                op_command(app, "cam.op.enable", json!({"op": i, "enabled": !on}), Some(i));
            }
            if tool_button(ui, app.tr("ui.manufacture.delete"), "delete", sel.is_some(), "manufacture:delete")
                && let Some(i) = sel
            {
                op_command(app, "cam.op.remove", json!({"op": i}), None);
            }
            ui_kit::StageBar::tool_gap(ui);
            ui_kit::StageBar::caption(ui, app.tr("ui.manufacture.show_paths"));
            let mut shown = match app.manufacture.show_paths {
                ShowPaths::All => 0,
                ShowPaths::Selected => 1,
                ShowPaths::None => 2,
            };
            let options = [app.tr("ui.manufacture.paths_all"), app.tr("ui.manufacture.paths_selected"), app.tr("ui.manufacture.paths_none")];
            if ui_kit::segmented(
                ui,
                &mut shown,
                &options,
                ui_kit::Segments {
                    targets: &["manufacture:paths:all", "manufacture:paths:selected", "manufacture:paths:none"],
                    dense: true,
                    intrinsic: true,
                    ..Default::default()
                },
            ) {
                app.manufacture.show_paths = [ShowPaths::All, ShowPaths::Selected, ShowPaths::None][shown.min(2)];
            }
        }
        Stage::Check => {
            let mut also = app.manufacture.also_simulate;
            if ui_kit::checkbox(ui, app.tr("ui.manufacture.also_simulate"), &mut also).changed() {
                app.manufacture.also_simulate = also;
            }
            ui_kit::StageBar::tool_gap(ui);
            let mut which = usize::from(app.manufacture.errors_only);
            let options = [app.tr("ui.manufacture.findings_all"), app.tr("ui.manufacture.findings_errors")];
            if ui_kit::segmented(
                ui,
                &mut which,
                &options,
                ui_kit::Segments {
                    targets: &["manufacture:findings:all", "manufacture:findings:errors"],
                    dense: true,
                    intrinsic: true,
                    ..Default::default()
                },
            ) {
                app.manufacture.errors_only = which == 1;
            }
        }
        Stage::Program => {
            if tool_button(ui, app.tr("ui.manufacture.write_program"), "save", open, "manufacture:write_tool") {
                write_program(app);
            }
            let sheet = written(app).into_iter().find(|p| p.ends_with(".html") || p.ends_with(".md"));
            if tool_button(ui, app.tr("ui.manufacture.open_sheet"), "list", sheet.is_some(), "manufacture:open_sheet")
                && let Some(path) = sheet
            {
                open_file(ui, &path);
            }
        }
    }
}

/// The open job's operations.
fn ops(app: &SolveApp) -> Vec<Value> {
    app.session.cam_job.as_deref().map(|j| j.operations().to_vec()).unwrap_or_default()
}

/// The files the last Write program wrote.
fn written(app: &SolveApp) -> Vec<String> {
    app.session.cam_job.as_deref().and_then(|j| j.generated.as_ref()).map(|g| g.written.clone()).unwrap_or_default()
}

fn open_file(ui: &Ui, path: &str) {
    let url = format!("file:///{}", path.replace('\\', "/").trim_start_matches('/'));
    ui.ctx().open_url(egui::OpenUrl::new_tab(url));
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

fn op_command(app: &mut SolveApp, id: &str, p: Value, select: Option<usize>) {
    if report(app, id, p).is_some() {
        app.manufacture.selected = select;
    }
}

/// The two cards: the job tree and the stage's card.
pub(crate) fn panels(app: &mut SolveApp, ui: &mut Ui) {
    let bounds = crate::shell::panel_bounds(app, ui.available_rect_before_wrap());
    let t = crate::theme::Tokens::get();
    if app.manufacture.tree_width <= 0.0 {
        app.manufacture.tree_width = t.form_card_width;
        app.manufacture.stage_width = t.form_card_width * 1.2;
    }
    let mut side = "left".to_string();
    let mut width = app.manufacture.tree_width;
    let title = job_title(app);
    let tree = ui_kit::Panel::workspace(
        ui,
        ui_kit::WorkspaceOptions {
            id: "manufacture:tree",
            title: &title,
            icon: "list",
            bounds,
            fill_height: true,
            compact: false,
            editor: false,
            enabled: true,
            header_actions: None,
        },
        &mut side,
        &mut width,
        |ui| tree_card(app, ui),
    );
    app.manufacture.tree_width = width;
    let tree_rect = tree.map(|c| c.response.rect);
    app.shell.browser = tree_rect;
    crate::shell::register_panel(app, "manufacture:tree", tree_rect, Some(ui_kit::Edge::Left));
    if let Some(r) = tree_rect {
        crate::scenario::publish_rect("manufacture:tree", r);
    }
    let mut side = "right".to_string();
    let mut width = app.manufacture.stage_width;
    let title = app.tr(app.manufacture.stage.label()).to_string();
    let card = ui_kit::Panel::workspace(
        ui,
        ui_kit::WorkspaceOptions {
            id: "manufacture:stage",
            title: &title,
            icon: "settings",
            bounds,
            fill_height: true,
            compact: false,
            editor: false,
            enabled: true,
            header_actions: None,
        },
        &mut side,
        &mut width,
        |ui| stage_card(app, ui),
    );
    app.manufacture.stage_width = width;
    let card_rect = card.map(|c| c.response.rect);
    crate::shell::register_panel(app, "manufacture:stage", card_rect, Some(ui_kit::Edge::Right));
    if let Some(r) = card_rect {
        crate::scenario::publish_rect("manufacture:stage", r);
    }
}

fn job_title(app: &SolveApp) -> String {
    match app.session.cam_job.as_deref() {
        Some(j) => {
            let name = j.value.get("name").and_then(Value::as_str).map_or_else(|| app.session.doc.name.clone(), str::to_string);
            if j.dirty() { format!("{name} \u{2022}") } else { name }
        }
        None => app.tr("ui.manufacture.job").to_string(),
    }
}

fn scroll(ui: &mut Ui, id: &str, add: impl FnOnce(&mut Ui)) {
    egui::ScrollArea::vertical().id_salt(id).auto_shrink([false, false]).show(ui, add);
}

/// The job tree: the job's file and banner, its setup, its operations with their state, the
/// tools they use; without a job, how to get one.
fn tree_card(app: &mut SolveApp, ui: &mut Ui) {
    if app.session.cam_job.is_none() {
        empty_state(app, ui);
        return;
    }
    banner(app, ui);
    let path = app.session.cam_job.as_deref().and_then(|j| j.path.clone());
    let file = path.as_deref().and_then(|p| std::path::Path::new(p).file_name()).map(|f| f.to_string_lossy().to_string());
    ui_kit::text(ui, file.as_deref().unwrap_or(app.tr("ui.manufacture.not_saved")), ui_kit::Text::Hint("manufacture:file"));
    scroll(ui, "manufacture_tree", |ui| {
        heading_row(ui, app.tr("ui.manufacture.setup"));
        let rows = setup_rows(app);
        for (k, (label, value)) in rows.iter().enumerate() {
            let r = ui_kit::tree_row(
                ui,
                ui.scope_id().with(("mf_setup", k)),
                &ui_kit::Row { depth: 1, compact: true, label, secondary: value, ..Default::default() },
            );
            if r.clicked {
                let _ = app.run("view.manufacture_stage", json!({"stage": "setup"}));
            }
        }
        heading_row(ui, app.tr("ui.manufacture.operations"));
        operation_rows(app, ui);
        let tools = tools_used(app);
        if !tools.is_empty() {
            heading_row(ui, app.tr("ui.manufacture.tools_used"));
            for (k, line) in tools.iter().enumerate() {
                ui_kit::tree_row(ui, ui.scope_id().with(("mf_tool", k)), &ui_kit::Row { depth: 1, compact: true, label: line, ..Default::default() });
            }
        }
    });
}

fn heading_row(ui: &mut Ui, text: &str) {
    ui.add_space(crate::theme::Tokens::get().space_4);
    ui_kit::text(ui, text, ui_kit::Text::Heading(text));
}

/// The setup in words, one row each.
fn setup_rows(app: &SolveApp) -> Vec<(String, String)> {
    let Some(j) = app.session.cam_job.as_deref() else { return Vec::new() };
    let v = &j.value;
    let s = |k: &str| v.get(k).and_then(Value::as_str).map(file_name);
    let profile = s("profile_path");
    let from_profile =
        || if profile.is_some() { app.tr("ui.manufacture.from_profile").to_string() } else { app.tr("ui.manufacture.none").to_string() };
    let machine = match v.get("machine") {
        Some(Value::String(m)) if m == "3axis" => app.tr("ui.manufacture.machine_3axis").to_string(),
        Some(Value::String(m)) if m == "4axis_a" => app.tr("ui.manufacture.machine_4axis").to_string(),
        Some(Value::String(m)) => m.clone(),
        Some(_) => app.tr("ui.manufacture.machine_custom").to_string(),
        None => s("machine_path").unwrap_or_else(from_profile),
    };
    let post = s("post").or_else(|| s("post_path")).unwrap_or_else(from_profile);
    let tools = match (v.get("tools").and_then(Value::as_array), s("tool_library_path")) {
        (Some(t), _) => crate::i18n::format("ui.manufacture.tools_n", &[("count", &t.len().to_string())]),
        (None, Some(p)) => p,
        _ => v
            .get("tool_library_paths")
            .and_then(Value::as_array)
            .map_or_else(from_profile, |l| l.iter().filter_map(Value::as_str).map(file_name).collect::<Vec<_>>().join(", ")),
    };
    let origin = v
        .get("origin")
        .and_then(Value::as_array)
        .map_or_else(|| "0, 0, 0".to_string(), |a| a.iter().map(|x| x.as_f64().map_or_else(|| "?".into(), num)).collect::<Vec<_>>().join(", "));
    let holding = s("workholding_path").or_else(|| v.get("workholding").map(|_| app.tr("ui.manufacture.in_the_job").to_string()));
    let names = |k: &str| v.get(k).and_then(Value::as_array).map(|a| a.iter().filter_map(Value::as_str).collect::<Vec<_>>().join(", "));
    let mut rows = vec![(app.tr("ui.manufacture.machine").to_string(), machine)];
    if let Some(p) = profile.clone() {
        rows.push((app.tr("ui.manufacture.profile").to_string(), p));
    }
    rows.push((app.tr("ui.manufacture.post").to_string(), post));
    rows.push((app.tr("ui.manufacture.tools").to_string(), tools));
    rows.push((app.tr("ui.manufacture.stock").to_string(), stock_words(app, v.get("stock"))));
    rows.push((app.tr("ui.manufacture.work_zero").to_string(), origin));
    rows.push((app.tr("ui.manufacture.holding").to_string(), holding.unwrap_or_else(|| app.tr("ui.manufacture.none").to_string())));
    rows.push((app.tr("ui.manufacture.part").to_string(), names("bodies").unwrap_or_else(|| app.tr("ui.manufacture.every_body").to_string())));
    if let Some(f) = names("fixtures") {
        rows.push((app.tr("ui.manufacture.fixtures").to_string(), f));
    }
    rows
}

fn file_name(p: &str) -> String {
    std::path::Path::new(p).file_name().map_or_else(|| p.to_string(), |f| f.to_string_lossy().to_string())
}

fn num(x: f64) -> String {
    let r = (x * 1000.0).round() / 1000.0;
    if r == r.trunc() { format!("{}", r as i64) } else { format!("{r}") }
}

fn stock_words(app: &SolveApp, stock: Option<&Value>) -> String {
    let Some(st) = stock else { return app.tr("ui.manufacture.none").to_string() };
    let arr = |x: &Value| x.as_array().map(|a| a.iter().filter_map(Value::as_f64).collect::<Vec<_>>()).unwrap_or_default();
    if let Some(b) = st.get("box") {
        let (lo, hi) = (arr(&b["min"]), arr(&b["max"]));
        if let ([a, b2, c], [d, e, f]) = (lo.as_slice(), hi.as_slice()) {
            let size = format!("{} x {} x {}", num(d - a), num(e - b2), num(f - c));
            return crate::i18n::format("ui.manufacture.stock_block", &[("size", &size)]);
        }
    }
    if let Some(c) = st.get("cylinder") {
        let g = |k: &str| c.get(k).and_then(Value::as_f64).map_or_else(|| "?".into(), num);
        return crate::i18n::format("ui.manufacture.stock_bar", &[("radius", &g("radius")), ("from", &g("x_min")), ("to", &g("x_max"))]);
    }
    if st.get("around_part").is_some() {
        return app.tr("ui.manufacture.stock_around").to_string();
    }
    app.tr("ui.manufacture.none").to_string()
}

fn status_word(app: &SolveApp, status: OpStatus) -> &'static str {
    let _ = app;
    match status {
        OpStatus::Ok => "ui.manufacture.status_ok",
        OpStatus::Stale => "ui.manufacture.status_stale",
        OpStatus::Warning => "ui.manufacture.status_warning",
        OpStatus::Error => "ui.manufacture.status_error",
        OpStatus::Off => "ui.manufacture.status_off",
        OpStatus::Unplanned => "ui.manufacture.status_unplanned",
    }
}

/// The type label an operation shows (an indexed one: its inner type's).
fn type_label(app: &SolveApp, op: &Value) -> String {
    let (ty, inner) = op_types(op);
    let kind = if ty.starts_with("indexed") { inner.unwrap_or(ty) } else { ty };
    let id = format!("ui.manufacture.op.{kind}");
    let text = crate::i18n::Locale::for_language(&app.preferences.language).text(&id).to_string();
    if text == id { kind } else { text }
}

fn operation_rows(app: &mut SolveApp, ui: &mut Ui) {
    let Some(job) = app.session.cam_job.as_deref() else { return };
    let revision = app.session.model_stamp();
    let generated = job.generated.clone();
    let rows: Vec<(String, String, OpStatus, bool)> = job
        .operations()
        .iter()
        .enumerate()
        .map(|(i, op)| {
            let status = job.status(i, revision);
            let tool = op
                .get("tool")
                .or_else(|| op.get("operation").and_then(|o| o.get("tool")))
                .and_then(Value::as_u64)
                .or_else(|| generated.as_ref().and_then(|g| g.tools.get(&i)).map(|t| u64::from(*t)));
            let mut parts = vec![type_label(app, op)];
            if let Some(a) = op.get("a_deg").filter(|_| op_types(op).0.starts_with("indexed")) {
                parts.push(format!("A{}", a.as_f64().map_or_else(|| a.to_string(), num)));
            }
            if let Some(t) = tool {
                parts.push(format!("T{t}"));
            }
            if let Some(m) = generated.as_ref().and_then(|g| g.minutes.get(&i)).filter(|_| status != OpStatus::Stale) {
                parts.push(crate::cam_view_panel::clock(m * 60.0));
            }
            (format!("{}  {}", i + 1, op_name(op)), parts.join(" \u{b7} "), status, op_enabled(op))
        })
        .collect();
    if rows.is_empty() {
        ui_kit::text(ui, app.tr("ui.manufacture.no_operations"), ui_kit::Text::Hint("manufacture:no_ops"));
    }
    for (i, (label, secondary, status, enabled)) in rows.iter().enumerate() {
        let badge = app.tr(status_word(app, *status)).to_string();
        let badges = [badge.as_str()];
        let flag = match status {
            OpStatus::Error => Some(ui_kit::Status::Error),
            OpStatus::Warning => Some(ui_kit::Status::Warning),
            _ => None,
        };
        let tip = app.tr(if *enabled { "ui.manufacture.turn_off" } else { "ui.manufacture.turn_on" }).to_string();
        let r = ui_kit::tree_row(
            ui,
            ui.scope_id().with(("mf_op", i)),
            &ui_kit::Row {
                depth: 1,
                compact: true,
                label,
                secondary,
                selected: app.manufacture.selected == Some(i),
                dim: !enabled,
                eye: Some(*enabled),
                tip: Some(&tip),
                status: flag,
                badges: if matches!(status, OpStatus::Ok | OpStatus::Warning | OpStatus::Error) { &[] } else { &badges },
                ..Default::default()
            },
        );
        if let Some(rect) = r.rect {
            crate::scenario::publish_target(&format!("manufacture:op:{i}"), rect);
        }
        if r.eye {
            op_command(app, "cam.op.enable", json!({"op": i, "enabled": !enabled}), Some(i));
        } else if r.clicked {
            app.manufacture.selected = Some(i);
            if app.manufacture.stage != Stage::Operations {
                let _ = app.run("view.manufacture_stage", json!({"stage": "operations"}));
            }
        }
    }
}

/// The tools the operations use, by number: "T6 · 3 operations".
fn tools_used(app: &SolveApp) -> Vec<String> {
    let Some(job) = app.session.cam_job.as_deref() else { return Vec::new() };
    let mut count: std::collections::BTreeMap<u64, usize> = std::collections::BTreeMap::new();
    for (i, op) in job.operations().iter().enumerate() {
        let own = op.get("tool").or_else(|| op.get("operation").and_then(|o| o.get("tool"))).and_then(Value::as_u64);
        let picked = job.generated.as_ref().and_then(|g| g.tools.get(&i)).map(|t| u64::from(*t));
        if let Some(t) = own.or(picked) {
            *count.entry(t).or_default() += 1;
        }
    }
    count
        .into_iter()
        .map(|(t, n)| crate::i18n::format("ui.manufacture.tool_used", &[("tool", &format!("T{t}")), ("count", &n.to_string())]))
        .collect()
}

/// The header's banner: the file changed under unsaved edits, or is gone.
fn banner(app: &mut SolveApp, ui: &mut Ui) {
    let Some(b) = app.manufacture.banner.clone() else { return };
    match b {
        Banner::Changed(lines) => {
            ui_kit::text(ui, app.tr("ui.manufacture.changed_outside"), ui_kit::Text::Error("manufacture:banner"));
            for (k, l) in lines.iter().take(4).enumerate() {
                ui_kit::text(ui, &format!("  {l}"), ui_kit::Text::Report(&format!("manufacture:banner:{k}"), false));
            }
            ui.horizontal(|ui| {
                let load = ui_kit::button(ui, app.tr("ui.manufacture.load_file"), ui_kit::Button::Small);
                crate::scenario::publish_target("manufacture:banner:load", load.rect);
                if load.clicked() {
                    report(app, "cam.job.reload", json!({"discard": true}));
                }
                let keep = ui_kit::button(ui, app.tr("ui.manufacture.keep_mine"), ui_kit::Button::Small);
                crate::scenario::publish_target("manufacture:banner:keep", keep.rect);
                if keep.clicked() {
                    report(app, "cam.job.save", json!({"overwrite": true}));
                }
            });
        }
        Banner::Gone => {
            ui_kit::text(ui, app.tr("ui.manufacture.file_gone"), ui_kit::Text::Error("manufacture:banner"));
            if ui_kit::button(ui, app.tr("ui.manufacture.save_again"), ui_kit::Button::Small).clicked() {
                report(app, "cam.job.save", json!({"overwrite": true}));
            }
        }
    }
    ui_kit::divider(ui, "manufacture:banner:rule");
}

/// No job open: start one, open one, or one of those beside the design.
fn empty_state(app: &mut SolveApp, ui: &mut Ui) {
    ui_kit::text(ui, app.tr("ui.manufacture.empty"), ui_kit::Text::Note("manufacture:empty"));
    if app.session.path.is_none() {
        ui_kit::text(ui, app.tr("ui.manufacture.save_design_first"), ui_kit::Text::Hint("manufacture:unsaved"));
    }
    let new = ui_kit::button(ui, app.tr("ui.manufacture.new_job"), ui_kit::Button::Primary);
    crate::scenario::publish_target("manufacture:new", new.rect);
    if new.clicked() {
        report(app, "cam.job.new", json!({}));
    }
    let open = ui_kit::button(ui, app.tr("ui.manufacture.open_job"), ui_kit::Button::Normal);
    crate::scenario::publish_target("manufacture:open", open.rect);
    if open.clicked() {
        app.pick_open_for(&["json"], |app, path| {
            if let Some(path) = path {
                report(app, "cam.job.open", json!({"path": path}));
            }
        });
    }
    let beside = app.manufacture.beside.clone();
    if !beside.is_empty() {
        heading_row(ui, app.tr("ui.manufacture.jobs_beside"));
        for (k, path) in beside.iter().enumerate() {
            let name = file_name(path);
            let r = ui_kit::tree_row(ui, ui.scope_id().with(("mf_beside", k)), &ui_kit::Row { compact: true, label: &name, ..Default::default() });
            if r.clicked {
                report(app, "cam.job.open", json!({"path": path}));
            }
        }
    }
    if let Some(e) = app.manufacture.error.clone() {
        ui_kit::text(ui, &e, ui_kit::Text::Error("manufacture:error"));
    }
}

fn stage_card(app: &mut SolveApp, ui: &mut Ui) {
    if let Some(e) = app.manufacture.error.clone() {
        ui_kit::text(ui, &e, ui_kit::Text::Error("manufacture:error"));
    }
    if app.session.cam_job.is_none() {
        ui_kit::text(ui, app.tr("ui.manufacture.no_job"), ui_kit::Text::Hint("manufacture:stage:none"));
        return;
    }
    scroll(ui, "manufacture_stage", |ui| match app.manufacture.stage {
        Stage::Setup => setup_form(app, ui),
        Stage::Operations => operations_form(app, ui),
        Stage::Check => check_form(app, ui),
        Stage::Program => program_form(app, ui),
    });
}

/// A value row whose text is kept while it is typed and given back on Enter. A draft is
/// dropped when the job changes under it.
fn value_row(app: &SolveApp, ui: &mut Ui, label: &str, key: &str, current: &str) -> Option<String> {
    let revision = app.session.cam_job.as_deref().map_or(0, |j| j.revision);
    let id = egui::Id::unique(("manufacture_draft", key));
    let mut draft =
        ui.ctx().data(|d| d.get_temp::<(String, u64)>(id)).filter(|(_, r)| *r == revision).map_or_else(|| current.to_string(), |(d, _)| d);
    let (enter, changed) = ui_kit::text_row(ui, label, &mut draft, "");
    if enter {
        ui.ctx().data_mut(|d| d.remove::<(String, u64)>(id));
        return (draft != current).then_some(draft);
    }
    if changed {
        ui.ctx().data_mut(|d| d.insert_temp(id, (draft, revision)));
    }
    None
}

/// Set (or with `None`, unset) job keys as one step.
fn edit_job(app: &mut SolveApp, set: &[(&str, Option<Value>)], unset_too: &[&str]) {
    let mut s = Map::new();
    let mut u: Vec<String> = unset_too.iter().map(|k| (*k).to_string()).collect();
    for (k, v) in set {
        match v {
            Some(v) => {
                s.insert((*k).to_string(), v.clone());
            }
            None => u.push((*k).to_string()),
        }
    }
    let present: Vec<String> = u.into_iter().filter(|k| app.session.cam_job.as_deref().is_some_and(|j| j.value.get(k).is_some())).collect();
    report(app, "cam.job.edit", json!({"set": s, "unset": present}));
}

/// `path` as the job file reads it: relative to the job's folder when it is below the same root.
fn relative(app: &SolveApp, path: &str) -> String {
    let Some(base) = app.session.cam_job.as_deref().and_then(|j| j.folder()) else { return path.to_string() };
    let target = std::path::Path::new(path);
    let (a, b): (Vec<_>, Vec<_>) = (base.components().collect(), target.components().collect());
    let common = a.iter().zip(&b).take_while(|(x, y)| x == y).count();
    if common == 0 {
        return path.to_string();
    }
    let mut out = std::path::PathBuf::new();
    for _ in common..a.len() {
        out.push("..");
    }
    for c in &b[common..] {
        out.push(c.as_os_str());
    }
    out.to_string_lossy().replace('\\', "/")
}

fn lib_list(app: &SolveApp, key: &str) -> Vec<(String, String)> {
    app.manufacture
        .library
        .as_ref()
        .and_then(|l| l.get(key))
        .and_then(Value::as_array)
        .map(|a| {
            a.iter()
                .map(|e| {
                    let path = e.get("path").and_then(Value::as_str).unwrap_or_default().to_string();
                    let name = e.get("name").and_then(Value::as_str).map_or_else(|| file_name(&path), str::to_string);
                    (name, path)
                })
                .collect()
        })
        .unwrap_or_default()
}

/// A drop-down of `options` (the first: "none" or "from the profile") for job key(s).
fn choose(app: &mut SolveApp, ui: &mut Ui, label: &str, id: &str, options: &[String], current: usize) -> Option<usize> {
    let mut sel = current.min(options.len().saturating_sub(1));
    let refs: Vec<&str> = options.iter().map(String::as_str).collect();
    let r = ui_kit::choice_row(ui, label, id, &mut sel, &refs);
    crate::scenario::publish_target(&format!("manufacture:{id}"), r.rect);
    let _ = app;
    (sel != current).then_some(sel)
}

fn setup_form(app: &mut SolveApp, ui: &mut Ui) {
    let Some(v) = app.session.cam_job.as_deref().map(|j| j.value.clone()) else { return };
    let text = |k: &str| v.get(k).and_then(Value::as_str).map(str::to_string);
    // The library folder and saving.
    let folder = app.preferences.cam_library_folder.clone();
    let shown = app.manufacture.library.as_ref().and_then(|l| l.get("folder")).and_then(Value::as_str).unwrap_or(&folder).to_string();
    if let Some(f) = value_row(app, ui, app.tr("ui.manufacture.library_folder"), "library", &shown) {
        app.preferences.cam_library_folder = f;
        crate::manufacture_workspace::refresh_library(app);
    }
    let mut autosave = app.preferences.cam_job_autosave;
    if ui_kit::checkbox(ui, app.tr("ui.manufacture.autosave"), &mut autosave).changed() {
        app.preferences.cam_job_autosave = autosave;
    }
    ui_kit::divider(ui, "manufacture:setup:machine");
    // Profile.
    let none = app.tr("ui.manufacture.none").to_string();
    let profiles = lib_list(app, "profiles");
    let mut options = vec![none.clone()];
    options.extend(profiles.iter().map(|(n, _)| n.clone()));
    let current = text("profile_path")
        .map_or(0, |p| profiles.iter().position(|(_, path)| relative(app, path) == p || *path == p).map_or(options.len(), |i| i + 1));
    if let Some(i) = choose(app, ui, app.tr("ui.manufacture.profile"), "profile", &options, current) {
        let path = profiles.get(i.wrapping_sub(1)).map(|(_, p)| json!(relative(app, p)));
        edit_job(app, &[("profile_path", path)], &[]);
    }
    // Machine.
    let from = app.tr("ui.manufacture.from_profile").to_string();
    let machines = [from.clone(), app.tr("ui.manufacture.machine_3axis").to_string(), app.tr("ui.manufacture.machine_4axis").to_string()];
    let current = match v.get("machine").and_then(Value::as_str) {
        Some("3axis") => 1,
        Some("4axis_a") => 2,
        _ => 0,
    };
    if let Some(i) = choose(app, ui, app.tr("ui.manufacture.machine"), "machine", &machines, current) {
        let m = match i {
            1 => Some(json!("3axis")),
            2 => Some(json!("4axis_a")),
            _ => None,
        };
        edit_job(app, &[("machine", m)], &["machine_path"]);
    }
    // Post: built-ins, then files.
    let posts = lib_list(app, "posts");
    let mut options = vec![from.clone()];
    options.extend(posts.iter().map(|(n, p)| if p.is_empty() { n.clone() } else { format!("{n} ({})", file_name(p)) }));
    let current = match (text("post"), text("post_path")) {
        (Some(p), _) => posts.iter().position(|(n, path)| path.is_empty() && *n == p).map_or(options.len(), |i| i + 1),
        (None, Some(pp)) => {
            posts.iter().position(|(_, path)| !path.is_empty() && (relative(app, path) == pp || *path == pp)).map_or(options.len(), |i| i + 1)
        }
        _ => 0,
    };
    // The job's own post when the library does not list it.
    if current == options.len() {
        options.push(text("post").or_else(|| text("post_path").map(|p| file_name(&p))).unwrap_or_default());
    }
    if let Some(i) = choose(app, ui, app.tr("ui.manufacture.post"), "post", &options, current).filter(|i| *i <= posts.len()) {
        match posts.get(i.wrapping_sub(1)) {
            Some((n, p)) if p.is_empty() => edit_job(app, &[("post", Some(json!(n)))], &["post_path"]),
            Some((_, p)) => {
                let rel = relative(app, p);
                edit_job(app, &[("post_path", Some(json!(rel)))], &["post"]);
            }
            None => edit_job(app, &[], &["post", "post_path"]),
        }
    }
    // Tool library.
    let libs: Vec<(String, String)> = app
        .manufacture
        .library
        .as_ref()
        .and_then(|l| l.get("tool_libraries"))
        .and_then(Value::as_array)
        .map(|a| {
            a.iter()
                .map(|e| {
                    let path = e.get("path").and_then(Value::as_str).unwrap_or_default().to_string();
                    let n = e.get("tools").and_then(Value::as_u64).unwrap_or(0);
                    let out = e.get("left_out").and_then(Value::as_array).map_or(0, Vec::len);
                    let label = crate::i18n::format(
                        "ui.manufacture.library_entry",
                        &[("name", &file_name(&path)), ("count", &n.to_string()), ("left_out", &out.to_string())],
                    );
                    (label, path)
                })
                .collect()
        })
        .unwrap_or_default();
    let mut options = vec![from.clone()];
    options.extend(libs.iter().map(|(l, _)| l.clone()));
    let current =
        text("tool_library_path").map_or(0, |t| libs.iter().position(|(_, p)| relative(app, p) == t || *p == t).map_or(options.len(), |i| i + 1));
    if current == options.len() {
        options.push(text("tool_library_path").map(|p| file_name(&p)).unwrap_or_default());
    }
    if let Some(i) = choose(app, ui, app.tr("ui.manufacture.tools"), "tools", &options, current).filter(|i| *i <= libs.len()) {
        let path = libs.get(i.wrapping_sub(1)).map(|(_, p)| json!(relative(app, p)));
        edit_job(app, &[("tool_library_path", path)], &["tools", "tool_library_paths", "tool_library"]);
    }
    // Work offset and tolerance.
    let offsets: Vec<String> = (54..=59).map(|g| format!("G{g}")).collect();
    let current = v.get("work_offset").and_then(Value::as_u64).map_or(0, |w| w.saturating_sub(1) as usize);
    if let Some(i) = choose(app, ui, app.tr("ui.manufacture.work_offset"), "work_offset", &offsets, current) {
        edit_job(app, &[("work_offset", if i == 0 { None } else { Some(json!(i + 1)) })], &[]);
    }
    let tol = v.get("tolerance").and_then(Value::as_f64).map_or_else(String::new, num);
    if let Some(t) = value_row(app, ui, app.tr("ui.manufacture.tolerance"), "tolerance", &tol) {
        let value = t.trim().parse::<f64>().ok().map(|x| json!(x));
        edit_job(app, &[("tolerance", value)], &[]);
    }
    ui_kit::divider(ui, "manufacture:setup:zero");
    // Work zero, design coordinates.
    let origin: Vec<f64> =
        v.get("origin").and_then(Value::as_array).map_or_else(|| vec![0.0; 3], |a| a.iter().map(|x| x.as_f64().unwrap_or(0.0)).collect());
    for (k, axis) in ["X", "Y", "Z"].iter().enumerate() {
        let label = crate::i18n::format("ui.manufacture.work_zero_axis", &[("axis", axis)]);
        let now = origin.get(k).copied().map_or_else(String::new, num);
        if let Some(t) = value_row(app, ui, &label, &format!("origin{k}"), &now)
            && let Ok(x) = t.trim().parse::<f64>()
        {
            let mut o = origin.clone();
            o.resize(3, 0.0);
            o[k] = x;
            edit_job(app, &[("origin", Some(json!(o)))], &[]);
        }
    }
    ui_kit::divider(ui, "manufacture:setup:stock");
    stock_rows(app, ui, v.get("stock"));
    ui_kit::divider(ui, "manufacture:setup:holding");
    // Workholding.
    let holds = lib_list(app, "workholdings");
    let mut options = vec![none.clone()];
    options.extend(holds.iter().map(|(n, p)| format!("{n} ({})", file_name(p))));
    let current = match (text("workholding_path"), v.get("workholding")) {
        (Some(w), _) => holds.iter().position(|(_, p)| relative(app, p) == w || *p == w).map_or(options.len(), |i| i + 1),
        (None, Some(_)) => options.len(),
        _ => 0,
    };
    if current == options.len() {
        options.push(text("workholding_path").map_or_else(|| app.tr("ui.manufacture.in_the_job").to_string(), |w| file_name(&w)));
    }
    if let Some(i) = choose(app, ui, app.tr("ui.manufacture.holding"), "holding", &options, current)
        && i <= holds.len()
    {
        let path = holds.get(i.wrapping_sub(1)).map(|(_, p)| json!(relative(app, p)));
        edit_job(app, &[("workholding_path", path)], &["workholding"]);
    }
    let names =
        |k: &str| v.get(k).and_then(Value::as_array).map(|a| a.iter().filter_map(Value::as_str).collect::<Vec<_>>().join(", ")).unwrap_or_default();
    for (key, label) in [("bodies", "ui.manufacture.part"), ("fixtures", "ui.manufacture.fixtures")] {
        if let Some(t) = value_row(app, ui, app.tr(label), key, &names(key)) {
            let list: Vec<String> = t.split(',').map(|s| s.trim().to_string()).filter(|s| !s.is_empty()).collect();
            edit_job(app, &[(key, (!list.is_empty()).then(|| json!(list)))], &[]);
        }
    }
    ui_kit::text(ui, app.tr("ui.manufacture.bodies_hint"), ui_kit::Text::Hint("manufacture:bodies_hint"));
}

fn stock_rows(app: &mut SolveApp, ui: &mut Ui, stock: Option<&Value>) {
    let kinds = [
        app.tr("ui.manufacture.none").to_string(),
        app.tr("ui.manufacture.stock_kind_around").to_string(),
        app.tr("ui.manufacture.stock_kind_block").to_string(),
        app.tr("ui.manufacture.stock_kind_bar").to_string(),
    ];
    let current = match stock {
        Some(s) if s.get("around_part").is_some() => 1,
        Some(s) if s.get("box").is_some() => 2,
        Some(s) if s.get("cylinder").is_some() => 3,
        _ => 0,
    };
    if let Some(i) = choose(app, ui, app.tr("ui.manufacture.stock"), "stock", &kinds, current) {
        let value = match i {
            1 => Some(json!({"around_part": {"margin": [2, 2, 1], "bottom": 0}})),
            2 => Some(json!({"box": {"min": [-50, -50, -20], "max": [50, 50, 1]}})),
            3 => Some(json!({"cylinder": {"radius": 20, "x_min": 0, "x_max": 60}})),
            _ => None,
        };
        edit_job(app, &[("stock", value)], &[]);
        return;
    }
    let Some(stock) = stock else { return };
    let field = |app: &mut SolveApp, ui: &mut Ui, label: &str, path: &str, now: Option<f64>| {
        if let Some(t) = value_row(app, ui, label, path, &now.map_or_else(String::new, num)) {
            let v = t.trim().parse::<f64>().ok().map(|x| json!(x));
            edit_job(app, &[(path, v)], &[]);
        }
    };
    let get = |path: &[&str]| -> Option<f64> {
        let mut at = stock;
        for p in path {
            at = match p.parse::<usize>() {
                Ok(k) => at.get(k)?,
                Err(_) => at.get(*p)?,
            };
        }
        at.as_f64()
    };
    let axes = ["X", "Y", "Z"];
    if stock.get("around_part").is_some() {
        for (k, a) in axes.iter().enumerate() {
            let label = crate::i18n::format("ui.manufacture.stock_margin", &[("axis", a)]);
            field(app, ui, &label, &format!("stock.around_part.margin.{k}"), get(&["around_part", "margin", &k.to_string()]));
        }
        field(app, ui, app.tr("ui.manufacture.stock_top"), "stock.around_part.top", get(&["around_part", "top"]));
        field(app, ui, app.tr("ui.manufacture.stock_bottom"), "stock.around_part.bottom", get(&["around_part", "bottom"]));
    } else if stock.get("box").is_some() {
        for (corner, label_id) in [("min", "ui.manufacture.stock_min"), ("max", "ui.manufacture.stock_max")] {
            for (k, a) in axes.iter().enumerate() {
                let label = crate::i18n::format(label_id, &[("axis", a)]);
                field(app, ui, &label, &format!("stock.box.{corner}.{k}"), get(&["box", corner, &k.to_string()]));
            }
        }
    } else if stock.get("cylinder").is_some() {
        field(app, ui, app.tr("ui.manufacture.stock_radius"), "stock.cylinder.radius", get(&["cylinder", "radius"]));
        field(app, ui, app.tr("ui.manufacture.stock_x_min"), "stock.cylinder.x_min", get(&["cylinder", "x_min"]));
        field(app, ui, app.tr("ui.manufacture.stock_x_max"), "stock.cylinder.x_max", get(&["cylinder", "x_max"]));
    }
}

/// The selected operation in brief (its parameters are edited in a form in a later stage), and
/// adding one from the list of types.
fn operations_form(app: &mut SolveApp, ui: &mut Ui) {
    let four = app.session.cam_job.as_deref().is_some_and(|j| j.value.get("machine").and_then(Value::as_str) == Some("4axis_a"));
    let labels: Vec<String> = OP_TYPES
        .iter()
        .filter(|(_, _, group)| four || *group != "rotary")
        .map(|(ty, label, group)| {
            let id = format!("ui.manufacture.op.{ty}");
            let text = crate::i18n::Locale::for_language(&app.preferences.language).text(&id).to_string();
            let group = app.tr(&format!("ui.manufacture.group.{group}")).to_string();
            format!("{group} \u{b7} {}", if text == id { (*label).to_string() } else { text })
        })
        .collect();
    let types: Vec<&str> = OP_TYPES.iter().filter(|(_, _, group)| four || *group != "rotary").map(|(t, _, _)| *t).collect();
    let mut add = app.manufacture.add_type.min(types.len().saturating_sub(1));
    let refs: Vec<&str> = labels.iter().map(String::as_str).collect();
    ui_kit::choice_row(ui, app.tr("ui.manufacture.add_operation"), "manufacture_add", &mut add, &refs);
    app.manufacture.add_type = add;
    let b = ui_kit::button(ui, app.tr("ui.manufacture.add"), ui_kit::Button::Primary);
    crate::scenario::publish_target("manufacture:add", b.rect);
    if b.clicked()
        && let Some(ty) = types.get(add)
    {
        let at = app.manufacture.selected.map(|i| i + 1).unwrap_or(ops(app).len()).min(ops(app).len());
        if let Some(r) = report(app, "cam.op.add", json!({"op": {"type": ty}, "at": at})) {
            app.manufacture.selected = r.get("index").and_then(Value::as_u64).and_then(|i| usize::try_from(i).ok());
        }
    }
    ui_kit::divider(ui, "manufacture:ops:rule");
    let Some(i) = app.manufacture.selected.filter(|i| *i < ops(app).len()) else {
        ui_kit::text(ui, app.tr("ui.manufacture.select_operation"), ui_kit::Text::Hint("manufacture:ops:none"));
        return;
    };
    let op = ops(app)[i].clone();
    ui_kit::text(ui, &type_label(app, &op), ui_kit::Text::Heading("manufacture:op:type"));
    if let Some(name) = value_row(app, ui, app.tr("ui.manufacture.name"), &format!("op{i}name"), &op_name(&op)) {
        report(app, "cam.op.edit", json!({"op": i, "rename": name}));
    }
    if four {
        let a = match op_types(&op).0.as_str() {
            "indexed" => op.get("a_deg").and_then(Value::as_f64).map_or_else(String::new, num),
            "indexed_pattern" => op
                .get("a_deg")
                .and_then(Value::as_array)
                .map_or_else(String::new, |a| a.iter().filter_map(Value::as_f64).map(num).collect::<Vec<_>>().join(", ")),
            _ => "0".into(),
        };
        if let Some(t) = value_row(app, ui, app.tr("ui.manufacture.turn_a"), &format!("op{i}a"), &a) {
            let angles: Vec<f64> = t.split(',').filter_map(|x| x.trim().parse::<f64>().ok()).collect();
            let a_deg = if angles.len() == 1 { json!(angles[0]) } else { json!(angles) };
            if !angles.is_empty() {
                report(app, "cam.op.edit", json!({"op": i, "a_deg": a_deg}));
            }
        }
    }
    // Its keys as the job holds them: what is not here takes its default.
    ui_kit::text(ui, app.tr("ui.manufacture.keys_given"), ui_kit::Text::Note("manufacture:op:keys"));
    let body = if op_types(&op).0.starts_with("indexed") { op.get("operation").cloned().unwrap_or(Value::Null) } else { op.clone() };
    if let Some(map) = body.as_object() {
        for (k, v) in map.iter().filter(|(k, _)| !matches!(k.as_str(), "name" | "type")) {
            let mut text = v.to_string();
            if text.chars().count() > 60 {
                text = format!("{}\u{2026}", text.chars().take(60).collect::<String>());
            }
            ui_kit::text(ui, &format!("{k}: {text}"), ui_kit::Text::MonoNote(&format!("manufacture:op:key:{k}")));
        }
    }
}

/// The findings of the last Generate, grouped by operation, each going to its place.
fn check_form(app: &mut SolveApp, ui: &mut Ui) {
    let busy = app.manufacture.busy().is_some();
    let g = ui_kit::button(ui, app.tr("ui.manufacture.generate"), if busy { ui_kit::Button::Enabled(false) } else { ui_kit::Button::Primary });
    crate::scenario::publish_target("manufacture:check:generate", g.rect);
    if g.clicked() && !busy {
        generate_now(app);
    }
    let revision = app.session.model_stamp();
    let Some(job) = app.session.cam_job.as_deref() else { return };
    let Some(last) = job.generated.clone() else {
        ui_kit::text(ui, app.tr("ui.manufacture.not_generated"), ui_kit::Text::Hint("manufacture:check:none"));
        return;
    };
    let stale = last.revision != job.revision || last.model_revision != revision;
    let errors = last.findings.iter().filter(|f| f.severity == "error").count();
    let warnings = last.findings.iter().filter(|f| f.severity == "warning").count();
    let summary = if errors == 0 && warnings == 0 {
        app.tr("ui.manufacture.passed").to_string()
    } else {
        crate::i18n::format("ui.manufacture.found", &[("errors", &errors.to_string()), ("warnings", &warnings.to_string())])
    };
    ui_kit::text(
        ui,
        &summary,
        if errors > 0 { ui_kit::Text::Error("manufacture:check:summary") } else { ui_kit::Text::Note("manufacture:check:summary") },
    );
    if last.total_minutes > 0.0 {
        let total = crate::i18n::format("ui.manufacture.total_time", &[("time", &crate::cam_view_panel::clock(last.total_minutes * 60.0))]);
        ui_kit::text(ui, &total, ui_kit::Text::Hint("manufacture:check:time"));
    }
    if stale {
        ui_kit::text(ui, app.tr("ui.manufacture.stale_results"), ui_kit::Text::Hint("manufacture:check:stale"));
    }
    let mut last_group: Option<Option<usize>> = None;
    let errors_only = app.manufacture.errors_only;
    for f in last.findings.iter().filter(|f| !errors_only || f.severity == "error") {
        let group = f.op.as_ref().map(|o| o.index);
        if last_group != Some(group) {
            last_group = Some(group);
            let title = f.op.as_ref().map_or_else(|| app.tr("ui.manufacture.whole_job").to_string(), |o| format!("{}  {}", o.index + 1, o.name));
            heading_row(ui, &title);
        }
        let flag = match f.severity.as_str() {
            "error" => Some(ui_kit::Status::Error),
            "warning" => Some(ui_kit::Status::Warning),
            _ => None,
        };
        let fix = if f.fix.is_empty() { String::new() } else { crate::i18n::format("ui.manufacture.fix", &[("fix", &f.fix)]) };
        let place = f.line.map(|l| crate::i18n::format("ui.manufacture.line", &[("line", &l.to_string())])).unwrap_or_default();
        let r = ui_kit::tree_row(
            ui,
            ui.scope_id().with(("mf_finding", f.index)),
            &ui_kit::Row {
                depth: 1,
                label: &f.message,
                secondary: &place,
                subtitle: &fix,
                status: flag,
                tip: Some(&f.message),
                ..Default::default()
            },
        );
        if let Some(rect) = r.rect {
            crate::scenario::publish_target(&format!("manufacture:finding:{}", f.index), rect);
        }
        if r.clicked {
            report(app, "cam.finding.go", json!({"index": f.index}));
        }
    }
}

/// Start writing the program, asking first when it would be written despite failed checks.
fn write_program(app: &mut SolveApp) {
    if app.manufacture.allow_unsafe {
        app.manufacture.confirm_unsafe = true;
        return;
    }
    run_heavy(app, "cam.post", post_params(app, false));
}

pub(crate) fn post_params(app: &SolveApp, allow_unsafe: bool) -> Value {
    let mut p = Map::new();
    let out = app.manufacture.output.trim();
    if !out.is_empty() {
        p.insert("output".into(), json!(out));
    }
    if app.manufacture.sheet
        && let Some(path) = app.session.cam_job.as_deref().and_then(|j| j.path.clone())
    {
        let stem = path.strip_suffix(".json").unwrap_or(&path);
        let stem = stem.strip_suffix(".cam").unwrap_or(stem);
        let ext = if app.manufacture.sheet_markdown { "md" } else { "html" };
        p.insert("setup_sheet".into(), json!(format!("{stem}.setup.{ext}")));
    }
    if allow_unsafe {
        p.insert("allow_unsafe".into(), json!(true));
    }
    Value::Object(p)
}

fn program_form(app: &mut SolveApp, ui: &mut Ui) {
    let mut out = app.manufacture.output.clone();
    let hint = app.tr("ui.manufacture.output_hint").to_string();
    let (_, changed) = ui_kit::text_row(ui, app.tr("ui.manufacture.output"), &mut out, &hint);
    if changed {
        app.manufacture.output = out;
    }
    let mut sheet = app.manufacture.sheet;
    let r = ui_kit::checkbox(ui, app.tr("ui.manufacture.write_sheet"), &mut sheet);
    crate::scenario::publish_target("manufacture:write_sheet", r.rect);
    if r.changed() {
        app.manufacture.sheet = sheet;
    }
    if app.manufacture.sheet {
        let mut md = app.manufacture.sheet_markdown;
        if ui_kit::checkbox(ui, app.tr("ui.manufacture.sheet_markdown"), &mut md).changed() {
            app.manufacture.sheet_markdown = md;
        }
    }
    let mut unsafe_ok = app.manufacture.allow_unsafe;
    if ui_kit::checkbox(ui, app.tr("ui.manufacture.allow_unsafe"), &mut unsafe_ok).changed() {
        app.manufacture.allow_unsafe = unsafe_ok;
    }
    let busy = app.manufacture.busy().is_some();
    let w = ui_kit::button(ui, app.tr("ui.manufacture.write_program"), if busy { ui_kit::Button::Enabled(false) } else { ui_kit::Button::Primary });
    crate::scenario::publish_target("manufacture:write", w.rect);
    if w.clicked() && !busy {
        write_program(app);
    }
    let Some(reply) = app.manufacture.written.clone() else { return };
    ui_kit::divider(ui, "manufacture:program:rule");
    let passed = reply.get("passed").and_then(Value::as_bool).unwrap_or(false);
    ui_kit::text(
        ui,
        app.tr(if passed { "ui.manufacture.written_passed" } else { "ui.manufacture.written_unsafe" }),
        if passed { ui_kit::Text::Note("manufacture:program:state") } else { ui_kit::Text::Error("manufacture:program:state") },
    );
    for (k, key) in ["gcode", "setup_sheet", "preview"].iter().enumerate() {
        let Some(path) = reply.get("outputs").and_then(|o| o.get(*key)).and_then(Value::as_str) else { continue };
        ui.horizontal(|ui| {
            ui_kit::text(ui, &file_name(path), ui_kit::Text::Value(&format!("manufacture:out:{k}"), false)).backing_on_hover_text(path);
            if *key != "gcode" && ui_kit::button(ui, app.tr("ui.manufacture.open"), ui_kit::Button::Small).clicked() {
                open_file(ui, path);
            }
        });
    }
    let minutes = reply.get("total_minutes").and_then(Value::as_f64).unwrap_or(0.0);
    let lines = reply.get("program_lines").and_then(Value::as_u64).unwrap_or(0);
    let totals = crate::i18n::format(
        "ui.manufacture.program_totals",
        &[("lines", &lines.to_string()), ("time", &crate::cam_view_panel::clock(minutes * 60.0))],
    );
    ui_kit::text(ui, &totals, ui_kit::Text::Hint("manufacture:program:totals"));
    let head: Vec<&str> = reply.get("program_head").and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_str).take(20).collect();
    if !head.is_empty() {
        ui_kit::text(ui, &head.join("\n"), ui_kit::Text::MonoNote("manufacture:program:head"));
    }
}

/// Asks before writing a program whose checks failed (Program stage, "Write even when a check fails").
pub(crate) fn confirm(app: &mut SolveApp, ctx: &egui::Context) {
    if !app.manufacture.confirm_unsafe {
        return;
    }
    let title = app.tr("ui.manufacture.confirm_unsafe_title").to_string();
    let body = app.tr("ui.manufacture.confirm_unsafe").to_string();
    let (closed, action) = ui_kit::Confirm::new(egui::Id::unique("sc_manufacture_unsafe"), "status_warning", &title)
        .tone(ui_kit::Tone::Danger)
        .actions(app.tr("ui.manufacture.write_anyway"), app.tr_label("Cancel"), None)
        .show(ctx, true, |ui| {
            ui_kit::text(ui, &body, ui_kit::Text::Report("manufacture:unsafe", true));
        });
    if closed {
        app.manufacture.confirm_unsafe = false;
        if action == ui_kit::Action::Ok {
            let p = post_params(app, true);
            run_heavy(app, "cam.post", p);
        }
    }
}
