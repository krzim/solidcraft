//! The Manufacture workspace: the open CAM job (`solidcraft_engine::cam_job_state`) and the
//! stages a person takes it through, from setup to a written program, on the same commands an
//! agent uses (`cam.job.*`, `cam.op.*`, `cam.generate`, `cam.finding.go`, `cam.post`).
//!
//! This module holds the workspace's state and lifecycle: entering and leaving (the design's
//! camera kept and given back, as Render does), finding the job beside the design, looking for
//! edits made to the job file outside SolidCraft, saving after changes, undo routed to the
//! job, and Generate and Write program run off the window thread (inline on the web and
//! headless). What it draws is in `manufacture_ui` and `manufacture_plan_layer`.

use std::sync::mpsc::{Receiver, TryRecvError};

use serde_json::{Value, json};
use solidcraft_engine::Session;
use solidcraft_engine::cam_job_state::{Generated, Outside};
use solidcraft_engine::render::Camera;

use crate::SolveApp;

/// The workspace's ribbon tab: it owns its chrome, as Render does.
pub const TAB: &str = "MANUFACTURE";
/// How often the job file is looked at while the workspace shows (ms).
const POLL_MS: f64 = 2000.0;
/// How long after the last change the job is saved (ms): a burst of edits saves once.
const SAVE_AFTER_MS: f64 = 1000.0;

/// The stages, in order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Stage {
    #[default]
    Setup,
    Operations,
    Check,
    Program,
}

impl Stage {
    pub const ALL: [Stage; 4] = [Stage::Setup, Stage::Operations, Stage::Check, Stage::Program];

    pub fn key(self) -> &'static str {
        match self {
            Stage::Setup => "setup",
            Stage::Operations => "operations",
            Stage::Check => "check",
            Stage::Program => "program",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Stage::Setup => "ui.manufacture.stage_setup",
            Stage::Operations => "ui.manufacture.stage_operations",
            Stage::Check => "ui.manufacture.stage_check",
            Stage::Program => "ui.manufacture.stage_program",
        }
    }

    pub fn of(key: &str) -> Option<Stage> {
        Stage::ALL.into_iter().find(|s| s.key() == key)
    }
}

/// Which toolpaths the view draws.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ShowPaths {
    #[default]
    All,
    Selected,
    None,
}

/// The job file changed under unsaved edits, or went: the header's banner.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Banner {
    Changed(Vec<String>),
    Gone,
}

/// A Generate or Write program running on the worker.
struct Away {
    rx: Receiver<Back>,
    command: String,
    /// The job's and the design's revisions it started from.
    revision: u64,
    model_revision: u64,
}

struct Back {
    result: Result<Value, String>,
    generated: Option<Generated>,
    copy: Box<Session>,
}

/// The workspace's state (not part of the design or the job).
#[derive(Default)]
pub struct Manufacture {
    pub stage: Stage,
    /// The operation selected in the job tree.
    pub selected: Option<usize>,
    pub show_paths: ShowPaths,
    /// Findings: errors only.
    pub errors_only: bool,
    /// Generate also simulates.
    pub also_simulate: bool,
    /// The camera and projection of the design, given back on leaving.
    design_camera: Option<(Camera, bool)>,
    /// When the job file was last looked at (ms).
    polled: f64,
    /// When the job is to be saved (ms), and the revision that was for.
    save_at: Option<(f64, u64)>,
    pub banner: Option<Banner>,
    /// The last finding gone to: its place in the view.
    pub mark: Option<[f64; 3]>,
    away: Option<Away>,
    /// The last refusal or message for the stage card.
    pub error: Option<String>,
    /// Job files beside the design, when there are several.
    pub beside: Vec<String>,
    /// The design path the job was looked for beside.
    looked_for: Option<String>,
    /// The library folder's listing (`cam.library.list`).
    pub library: Option<Value>,
    /// The Program stage's form.
    pub output: String,
    pub sheet: bool,
    pub sheet_markdown: bool,
    pub allow_unsafe: bool,
    /// Asking whether to write despite failed checks.
    pub confirm_unsafe: bool,
    /// The last program written: its reply.
    pub written: Option<Value>,
    /// The Add operation choice (index into `OP_TYPES`).
    pub add_type: usize,
    /// The cards' widths.
    pub tree_width: f32,
    pub stage_width: f32,
    /// Operations by hand: the form's defaults, the part's features, Machine this.
    pub ops: crate::manufacture_op::OpState,
}

impl Manufacture {
    /// Whether a Generate or Write program is running.
    pub fn busy(&self) -> Option<&str> {
        self.away.as_ref().map(|a| a.command.as_str())
    }
}

pub fn active(app: &SolveApp) -> bool {
    app.ui.tab == TAB
}

/// Enter (`on`) or leave the workspace. Refused while a sketch, a tool or a dialog is open.
pub fn enter(app: &mut SolveApp, on: bool) -> Result<(), String> {
    if on {
        if app.session.active_sketch.is_some() || app.tool.is_some() || app.dialog.is_some() || app.file_load.busy() {
            return Err(app.tr("ui.manufacture.error_finish_operation").to_string());
        }
        if crate::render_workspace::active(app) {
            crate::render_workspace::enter(app, false)?;
        }
        if !active(app) {
            app.manufacture.design_camera = Some((app.cam, app.ui.perspective));
            app.ui.tab = TAB.into();
        }
        discover(app);
        app.manufacture.polled = 0.0;
    } else if active(app) {
        if let Some((camera, perspective)) = app.manufacture.design_camera.take() {
            app.cam = camera;
            app.ui.up_axis = camera.up;
            app.ui.perspective = perspective;
        }
        app.ui.tab = "MODEL".into();
        crate::manufacture_ui::reset_shell(app);
    }
    app.viewport.hover = None;
    app.cam_anim = None;
    Ok(())
}

/// Switch to `workspace`: `design`, `render` or `manufacture`.
pub fn switch(app: &mut SolveApp, workspace: &str) -> Result<(), String> {
    match workspace {
        "manufacture" => enter(app, true),
        "render" => {
            if active(app) {
                enter(app, false)?;
            }
            crate::render_workspace::enter(app, true)
        }
        _ if active(app) => enter(app, false),
        _ => crate::render_workspace::enter(app, false),
    }
}

/// The workspace shown: `design`, `render` or `manufacture`.
pub fn current(app: &SolveApp) -> &'static str {
    if active(app) {
        "manufacture"
    } else if crate::render_workspace::active(app) {
        "render"
    } else {
        "design"
    }
}

/// The job beside the design, opened when there is one: `<design>.cam.json`, else the one
/// `<design>.*.cam.json`; several are listed for the Setup stage to open one.
pub fn discover(app: &mut SolveApp) {
    if app.session.cam_job.is_some() {
        return;
    }
    let Some(design) = app.session.path.clone() else { return };
    if app.manufacture.looked_for.as_deref() == Some(design.as_str()) {
        return;
    }
    app.manufacture.looked_for = Some(design.clone());
    app.manufacture.beside = jobs_beside(&design);
    if let [one] = app.manufacture.beside.as_slice() {
        let path = one.clone();
        if let Err(e) = app.run("cam.job.open", json!({"path": path})) {
            app.manufacture.error = Some(e);
        }
    }
}

/// The job files beside design `path`: `<stem>.cam.json` alone when it is there, else every
/// `<stem>.*.cam.json`.
pub fn jobs_beside(path: &str) -> Vec<String> {
    let p = std::path::Path::new(path);
    let (Some(dir), Some(stem)) = (p.parent(), p.file_stem().map(|s| s.to_string_lossy().to_string())) else { return Vec::new() };
    let own = dir.join(format!("{stem}.cam.json")).to_string_lossy().to_string();
    if solidcraft_engine::io::vfs::is_file(&own) {
        return vec![own];
    }
    let prefix = format!("{stem}.").to_lowercase();
    let mut found: Vec<String> = list_files(&dir.to_string_lossy())
        .into_iter()
        .filter(|f| {
            let name = std::path::Path::new(f).file_name().map(|n| n.to_string_lossy().to_lowercase()).unwrap_or_default();
            name.starts_with(&prefix) && name.ends_with(".cam.json")
        })
        .collect();
    found.sort();
    found
}

fn list_files(dir: &str) -> Vec<String> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        std::fs::read_dir(dir)
            .map(|it| it.filter_map(|e| e.ok()).filter(|e| e.path().is_file()).map(|e| e.path().to_string_lossy().to_string()).take(500).collect())
            .unwrap_or_default()
    }
    #[cfg(target_arch = "wasm32")]
    {
        let prefix = format!("{}/", dir.trim_end_matches('/'));
        solidcraft_engine::io::vfs::list(&prefix).into_iter().map(|(p, _)| p).filter(|p| !p[prefix.len()..].contains('/')).collect()
    }
}

/// Show `key`'s stage (`view.manufacture_stage` applied).
pub fn select_stage(app: &mut SolveApp, key: &str) {
    if let Some(stage) = Stage::of(key) {
        app.manufacture.stage = stage;
        if stage == Stage::Setup && app.manufacture.library.is_none() {
            refresh_library(app);
        }
    }
}

/// Read the library folder (the preference, else found above the job's folder).
pub fn refresh_library(app: &mut SolveApp) {
    let folder = app.preferences.cam_library_folder.trim().to_string();
    let p = if folder.is_empty() { json!({}) } else { json!({"folder": folder}) };
    app.manufacture.library = app.session.execute("cam.library.list", &p).ok();
}

/// After a command ran through `SolveApp::run`: what the workspace follows of it.
pub fn apply_command(app: &mut SolveApp, id: &str, reply: &Value) {
    match id {
        "view.manufacture_stage" => {
            if let Some(stage) = reply.get("stage").and_then(Value::as_str) {
                select_stage(app, stage);
            }
        }
        "cam.finding.go" => go(app, reply),
        "cam.job.open" | "cam.job.new" | "cam.job.close" => {
            app.manufacture.selected = None;
            app.manufacture.banner = None;
            app.manufacture.mark = None;
            app.manufacture.written = None;
            app.manufacture.error = None;
            app.manufacture.save_at = None;
            app.manufacture.ops = Default::default();
            if id != "cam.job.close" {
                refresh_library(app);
            }
        }
        "cam.job.reload" | "cam.job.save" => app.manufacture.banner = None,
        _ => {}
    }
}

/// Follow a finding gone to: select its operation, frame and mark its place, and seek the
/// simulation to its line when one is loaded.
fn go(app: &mut SolveApp, reply: &Value) {
    if let Some(i) = reply.get("op").and_then(|o| o.get("index")).and_then(Value::as_u64) {
        app.manufacture.selected = usize::try_from(i).ok();
    }
    let at = reply.get("at_design").and_then(Value::as_array).and_then(|a| {
        let v: Vec<f64> = a.iter().filter_map(Value::as_f64).collect();
        (v.len() == 3 && v.iter().all(|x| x.is_finite())).then(|| [v[0], v[1], v[2]])
    });
    app.manufacture.mark = at;
    if let Some(p) = at {
        let r = 8.0;
        let bounds = solidcraft_engine::geom::Aabb3 {
            min: solidcore_record::Vec3::new(p[0] - r, p[1] - r, p[2] - r),
            max: solidcore_record::Vec3::new(p[0] + r, p[1] + r, p[2] + r),
        };
        app.fit_bounds(bounds);
    }
    if let Some(line) = reply.get("line").and_then(Value::as_u64)
        && app.session.cam_view.is_some()
    {
        app.cam_player.ask("cam.view.seek", json!({"line": line}));
    }
}

/// Undo and redo in the workspace are the job's: `true` when `id` was taken here.
pub fn takes_command(app: &mut SolveApp, id: &str) -> bool {
    if !active(app) || app.session.cam_job.is_none() {
        return false;
    }
    let job_id = match id {
        "edit.undo" => "cam.job.undo",
        "edit.redo" => "cam.job.redo",
        _ => return false,
    };
    match app.run(job_id, json!({})) {
        Ok(r) => {
            if let Some(step) = r.get("step").and_then(Value::as_str) {
                let what = crate::i18n::format(if id == "edit.undo" { "ui.manufacture.undone" } else { "ui.manufacture.redone" }, &[("step", step)]);
                app.set_status(what, false);
            }
        }
        Err(e) => app.set_status(e, true),
    }
    true
}

/// Whether this build runs heavy work off the window thread.
pub(crate) fn threads(app: &SolveApp) -> bool {
    cfg!(not(target_arch = "wasm32")) && app.apply_off_thread.is_some()
}

/// Run `cam.generate` or `cam.post`: on the worker with a copy of the session when there are
/// threads, else here at once.
pub fn run_heavy(app: &mut SolveApp, command: &str, params: Value) {
    if app.manufacture.away.is_some() || app.session.cam_job.is_none() {
        return;
    }
    app.manufacture.error = None;
    // The last finding's mark is of the results this replaces.
    app.manufacture.mark = None;
    if !threads(app) {
        let r = app.run(command, params);
        finished(app, command, r);
        return;
    }
    let Some(revision) = app.session.cam_job.as_deref().map(|j| j.revision) else { return };
    let mut copy = Box::new(app.session.scratch());
    let (tx, rx) = std::sync::mpsc::channel();
    let scope = solidcraft_engine::io::vfs::current_scope();
    let id = command.to_string();
    let spawned = std::thread::Builder::new().name("manufacture".into()).spawn(move || {
        let result = solidcraft_engine::io::vfs::scoped(scope, || copy.execute(&id, &params)).map_err(|e| e.to_string());
        let generated = copy.cam_job.as_deref().and_then(|j| j.generated.clone());
        let _ = tx.send(Back { result, generated, copy });
    });
    match spawned {
        Ok(_) => app.manufacture.away = Some(Away { rx, command: command.into(), revision, model_revision: app.session.model_stamp() }),
        Err(e) => app.manufacture.error = Some(e.to_string()),
    }
}

fn finished(app: &mut SolveApp, command: &str, result: Result<Value, String>) {
    match result {
        Ok(reply) => {
            if command == "cam.post" {
                app.manufacture.written = Some(reply);
                app.manufacture.stage = Stage::Program;
            } else {
                let passed = reply.get("passed").and_then(Value::as_bool).unwrap_or(false);
                if !passed {
                    app.manufacture.stage = Stage::Check;
                }
            }
        }
        Err(e) => {
            app.manufacture.error = Some(e);
            if command == "cam.post" {
                app.manufacture.stage = Stage::Check;
            }
        }
    }
}

/// Each frame: take back the worker's result, look at the job file, save when due.
pub fn tick(app: &mut SolveApp, ctx: &egui::Context) {
    collect(app);
    crate::manufacture_op::collect(app);
    if app.manufacture.away.is_some() || crate::manufacture_op::busy(app) {
        ctx.request_repaint_after(std::time::Duration::from_millis(50));
    }
    keep_up(app, crate::now_ms());
    if active(app) {
        ctx.request_repaint_after(std::time::Duration::from_millis(POLL_MS as u64));
    } else if app.manufacture.save_at.is_some_and(|(at, _)| at.is_finite()) {
        ctx.request_repaint_after(std::time::Duration::from_millis(SAVE_AFTER_MS as u64));
    }
}

/// What the workspace keeps up with at `now` (ms): while it shows, the job beside the design
/// and edits made to its file outside SolidCraft; always, the save after the last change (the
/// job stays open when the workspace is left, and agents edit it from anywhere).
pub(crate) fn keep_up(app: &mut SolveApp, now: f64) {
    if active(app) {
        discover(app);
        if now - app.manufacture.polled >= POLL_MS {
            app.manufacture.polled = now;
            poll(app);
        }
    }
    autosave(app, now);
}

fn collect(app: &mut SolveApp) {
    let Some(away) = &app.manufacture.away else { return };
    let back = match away.rx.try_recv() {
        Ok(b) => b,
        Err(TryRecvError::Empty) => return,
        Err(TryRecvError::Disconnected) => {
            app.manufacture.away = None;
            app.manufacture.error = Some(app.tr("ui.manufacture.worker_lost").to_string());
            return;
        }
    };
    let Some(away) = app.manufacture.away.take() else { return };
    let same = app.session.cam_job.as_deref().is_some_and(|j| j.revision == away.revision) && app.session.model_stamp() == away.model_revision;
    if same {
        if let Some(job) = app.session.cam_job.as_deref_mut() {
            job.generated = back.generated;
        }
        // The plan the worker made is the session's too: a write after it plans nothing again.
        let mut copy = back.copy;
        app.session.adopt_cam_plan(&mut copy);
    }
    finished(app, &away.command, back.result);
}

/// Look at the job file: a change under a clean job is reloaded (one undo step, a toast of
/// what changed); under unsaved edits, or a file gone, the header shows a banner.
pub fn poll(app: &mut SolveApp) {
    let Some(job) = app.session.cam_job.as_deref_mut() else {
        app.manufacture.banner = None;
        return;
    };
    let outside = job.outside();
    let dirty = job.dirty();
    let theirs = || {
        let job = app.session.cam_job.as_deref()?;
        let text = solidcraft_engine::io::vfs::read_to_string(job.path.as_deref()?, 16 << 20).ok()?;
        let v: Value = serde_json::from_str(&text).ok()?;
        Some(solidcraft_engine::cam_job_state::changes(&job.value, &v))
    };
    match outside {
        Outside::None => {
            if app.manufacture.banner == Some(Banner::Gone) {
                app.manufacture.banner = None;
            }
        }
        Outside::Changed if !dirty => match app.run("cam.job.reload", json!({})) {
            Ok(r) => {
                let lines: Vec<&str> = r["changes"].as_array().into_iter().flatten().filter_map(Value::as_str).take(3).collect();
                let what = crate::i18n::format("ui.manufacture.reloaded", &[("changes", &lines.join("; "))]);
                app.set_status(what, false);
            }
            Err(e) => app.manufacture.error = Some(e),
        },
        Outside::Changed => {
            if !matches!(app.manufacture.banner, Some(Banner::Changed(_))) {
                app.manufacture.banner = Some(Banner::Changed(theirs().unwrap_or_default()));
            }
        }
        Outside::Gone => app.manufacture.banner = Some(Banner::Gone),
    }
}

/// Saves the job a moment after its last change, when the preference asks for it. A save that
/// is refused (the file changed outside, a folder that cannot be written) is not tried again
/// until the job changes.
fn autosave(app: &mut SolveApp, now: f64) {
    let Some(job) = app.session.cam_job.as_deref() else { return };
    if !app.preferences.cam_job_autosave || !job.dirty() || job.path.is_none() || app.manufacture.banner.is_some() {
        app.manufacture.save_at = None;
        return;
    }
    let revision = job.revision;
    match app.manufacture.save_at {
        Some((_, r)) if r != revision => app.manufacture.save_at = Some((now + SAVE_AFTER_MS, revision)),
        None => app.manufacture.save_at = Some((now + SAVE_AFTER_MS, revision)),
        Some((at, _)) if now >= at => {
            app.manufacture.save_at = None;
            if let Err(e) = app.run("cam.job.save", json!({})) {
                app.manufacture.error = Some(e);
                app.manufacture.save_at = Some((f64::INFINITY, revision));
            }
        }
        Some(_) => {}
    }
}

/// Load the open job into the simulation view.
pub fn simulate(app: &mut SolveApp) {
    if app.session.cam_job.is_none() {
        return;
    }
    crate::cam_view_panel::open(app);
    if let (Some(panel), Some(path)) = (app.cam_panel.as_mut(), app.session.cam_job.as_deref().and_then(|j| j.path.clone())) {
        panel.job = path;
    }
    app.cam_player.ask("cam.view.load", json!({"colour_by": "operation"}));
}

#[cfg(test)]
#[path = "manufacture_workspace_tests.rs"]
mod tests;
