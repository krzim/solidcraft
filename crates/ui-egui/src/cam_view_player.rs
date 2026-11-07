//! Plays the CAM simulation view (`solidcraft_engine::cam_view`) in the window, without cutting
//! the stock on the window thread.
//!
//! Every action is a `cam.view.*` command, whether it comes from the panel
//! (`cam_view_panel`), from playing on, or from an agent over the control channel
//! ([`intercept`]); they wait in one queue and run in order. The ones that cut the stock or read
//! files (`load`, `seek`, `export_stock`) run on a worker thread: the view
//! ([`solidcraft_engine::cam_view::CamView`]) moves to a copy of the session there, the command
//! runs, the stock tiles it changed are built there too (`cam_view_layer::refresh_tiles`), and
//! the view comes back. Meanwhile the window draws the view as it was last drawn and the panel
//! shows the last state ([`Snapshot`]). The others run here at once. Without threads (the web)
//! and headless, everything runs here, at once.
//!
//! Playing advances the tool by the time since the last step times the speed (or by moves a
//! second), as `cam.view.seek {time_s}`; a slow step makes the next one longer, so playback
//! keeps to time. A seek the panel asks for while another waits replaces it (scrubbing keeps
//! only the latest). When nothing else is asked and the program has not been checked to its end,
//! the worker plays it to the end in slices and goes back ([`prerun`]), so every finding is
//! known and marked on the timeline; anything asked meanwhile stops it.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, TryRecvError};
use std::sync::{Arc, Mutex};

use serde_json::{Value, json};
use solidcraft_engine::cam::findings::Severity;
use solidcraft_engine::cam_view::CamView;
use solidcraft_engine::schema::cam_view::{CamViewStateReply, FindingsListing};

use crate::SolveApp;
use crate::cam_view_layer::{LayerCache, Tiles, key_of, refresh_tiles};
use crate::control::ControlRequest;

/// Commands that cut the stock or read or write files: they run on the worker.
const HEAVY: &[&str] = &["cam.view.load", "cam.view.seek", "cam.view.export_stock"];
/// Cells cut per slice of the pre-run (a stop asked for is seen between slices).
const PRERUN_SLICE: f64 = 20.0e6;
/// The longest step of play (s): after a stall the tool does not leap.
const MAX_STEP_S: f64 = 0.5;
/// How long nothing must be asked before the pre-run starts again (ms).
const QUIET_MS: f64 = 1500.0;
/// How long a command runs before the panel says it is working (ms).
const BUSY_MS: f64 = 400.0;

/// Who asked for a command, and where its answer goes.
enum Asker {
    Panel,
    /// Playing on: dropped when play stops.
    Play,
    Agent(ControlRequest),
}

struct Request {
    command: String,
    params: Value,
    asker: Asker,
}

/// What the worker sends back.
struct Back {
    view: Option<Box<CamView>>,
    result: Result<Value, String>,
    command: String,
    asker: Asker,
}

struct Away {
    rx: Receiver<Back>,
    label: String,
    stop: Option<Arc<AtomicBool>>,
    /// When it went (ms).
    since: f64,
}

/// An operation on the timeline.
pub(crate) struct OpBand {
    pub index: usize,
    pub name: String,
    pub start_s: f64,
    pub end_s: f64,
}

/// A finding on the timeline and in the list.
pub(crate) struct FindingMark {
    pub index: usize,
    pub time_s: f64,
    pub line: usize,
    pub code: String,
    pub error: bool,
    pub message: String,
}

/// The view as the panel shows it: the last state, kept while the view is away.
pub(crate) struct Snapshot {
    pub state: CamViewStateReply,
    pub ops: Vec<OpBand>,
    /// Tool changes: time and tool number.
    pub changes: Vec<(f64, u32)>,
    pub findings: Vec<FindingMark>,
    pub duration: f64,
    /// Moves checked for findings so far, of all.
    pub checked: usize,
    /// Paths coloured by operation (else by tool).
    pub by_op: bool,
}

/// The player (see the module notes).
#[derive(Default)]
pub struct CamPlayer {
    queue: VecDeque<Request>,
    away: Option<Away>,
    /// The scene key and whether the part was shown, as when the view went away.
    held: (u64, bool),
    pub(crate) tiles: Arc<Mutex<Tiles>>,
    pub(crate) layer: Mutex<LayerCache>,
    pub(crate) snapshot: Option<Snapshot>,
    /// When play last stepped (ms).
    clock: Option<f64>,
    /// The last refusal of a command the panel asked for.
    pub(crate) error: Option<String>,
    /// How far the pre-run has checked (moves).
    prerun_at: Arc<AtomicUsize>,
    /// The pre-run gave up (an error): not started again for this view.
    prerun_failed: u64,
    /// When a command was last asked for (ms).
    asked_at: f64,
}

impl CamPlayer {
    /// Whether the view is on the worker.
    pub(crate) fn away(&self) -> bool {
        self.away.is_some()
    }

    pub(crate) fn held_key(&self) -> u64 {
        self.held.0
    }

    pub(crate) fn held_part(&self) -> bool {
        self.held.1
    }

    /// What the worker is doing, for the panel, once it has taken a moment (a step of play does
    /// not flash it): its label, and the pre-run's progress (moves).
    pub(crate) fn busy(&self) -> Option<(&str, Option<usize>)> {
        self.away
            .as_ref()
            .filter(|a| a.stop.is_some() || crate::now_ms() - a.since > BUSY_MS)
            .map(|a| (a.label.as_str(), a.stop.as_ref().map(|_| self.prerun_at.load(Ordering::Relaxed))))
    }

    /// Ask for `command`; a seek replaces a seek the panel or play asked for that still waits.
    pub(crate) fn ask(&mut self, command: &str, params: Value) {
        if command == "cam.view.seek" {
            self.queue.retain(|r| !(r.command == "cam.view.seek" && matches!(r.asker, Asker::Panel | Asker::Play)));
        }
        self.error = None;
        self.queue.push_back(Request { command: command.into(), params, asker: Asker::Panel });
        self.stop_prerun();
    }

    fn stop_prerun(&mut self) {
        self.asked_at = crate::now_ms();
        if let Some(stop) = self.away.as_ref().and_then(|a| a.stop.as_ref()) {
            stop.store(true, Ordering::Relaxed);
        }
    }

    /// Whether anything waits to run.
    pub(crate) fn pending(&self) -> bool {
        !self.queue.is_empty()
    }
}

/// Whether this build has threads to run commands on.
fn threads() -> bool {
    cfg!(not(target_arch = "wasm32"))
}

/// The command an `engine.execute` request runs, when it is a `cam.view.*` one.
fn cam_command(req: &ControlRequest) -> Option<(String, Value)> {
    if !matches!(req.method.as_str(), "engine.execute" | "command") {
        return None;
    }
    let command = req.params.get("command").or_else(|| req.params.get("id")).and_then(Value::as_str)?;
    let id = solidcraft_engine::find_command(command).map(|c| c.id)?;
    id.starts_with("cam.view.").then(|| (id.to_string(), req.params.get("params").cloned().filter(|v| !v.is_null()).unwrap_or(json!({}))))
}

/// Take an agent's `cam.view.*` request into the player's queue (in the app, with threads), so
/// that it waits for a view that is away and cuts the stock off the window thread. Returns the
/// request when it should run as any other.
pub(crate) fn intercept(app: &mut SolveApp, req: ControlRequest) -> Option<ControlRequest> {
    if app.apply_off_thread.is_none() || !threads() {
        return Some(req);
    }
    let Some((command, params)) = cam_command(&req) else { return Some(req) };
    app.cam_player.stop_prerun();
    app.cam_player.queue.push_back(Request { command, params, asker: Asker::Agent(req) });
    None
}

/// Each frame: take back a view the worker is done with, step play, run what waits.
pub(crate) fn tick(app: &mut SolveApp, ctx: &egui::Context) {
    collect(app);
    if app.cam_player.away() {
        ctx.request_repaint_after(std::time::Duration::from_millis(16));
        return;
    }
    if app.cam_player.queue.is_empty() {
        play_step(app);
    }
    if let Some(req) = app.cam_player.queue.pop_front() {
        dispatch(app, req);
    } else if threads() && app.apply_off_thread.is_some() {
        prerun(app);
    }
    let playing = app.session.cam_view.as_deref().is_some_and(|v| v.playing);
    if playing || app.cam_player.away() || app.cam_player.pending() {
        ctx.request_repaint_after(std::time::Duration::from_millis(16));
    }
    if !playing {
        app.cam_player.clock = None;
    }
}

/// The view is back from the worker: in the session again, the asker answered.
fn collect(app: &mut SolveApp) {
    let Some(away) = &app.cam_player.away else { return };
    let back = match away.rx.try_recv() {
        Ok(b) => b,
        Err(TryRecvError::Empty) => return,
        Err(TryRecvError::Disconnected) => {
            // The worker ended without a word (it panicked): the view is lost with it.
            app.cam_player.away = None;
            app.cam_player.error = Some(crate::i18n::tr("ui.cam_view.lost").to_string());
            return;
        }
    };
    app.cam_player.away = None;
    if back.command == "prerun" {
        // The pre-run found a fault in the program it could not play past: not again.
        if back.result.is_err()
            && let Some(v) = back.view.as_deref()
        {
            app.cam_player.prerun_failed = crate::cam_view_layer::view_id(v);
        }
        app.session.cam_view = back.view;
        refresh_snapshot(app);
        return;
    }
    if back.view.is_some() {
        if back.command == "cam.view.load" {
            // A new view: the caches and the old view go.
            *app.cam_player.layer.lock().unwrap_or_else(|e| e.into_inner()) = LayerCache::default();
            app.cam_player.prerun_failed = 0;
        }
        app.session.cam_view = back.view;
    }
    answer(app, back.asker, &back.command, back.result);
    refresh_snapshot(app);
}

/// Answer whoever asked.
fn answer(app: &mut SolveApp, asker: Asker, command: &str, result: Result<Value, String>) {
    match asker {
        Asker::Agent(req) => {
            let reply = match result {
                Ok(v) => json!({"ok": true, "result": v}),
                Err(e) => json!({"ok": false, "error": e}),
            };
            let _ = req.reply.send(reply);
        }
        Asker::Panel => {
            if let Err(e) = result {
                app.cam_player.error = Some(e);
            } else if command == "cam.view.load" {
                app.cam_player.error = None;
            }
        }
        Asker::Play => {
            if let Err(e) = result {
                app.cam_player.error = Some(e);
                if let Some(v) = app.session.cam_view.as_deref_mut() {
                    v.playing = false;
                    v.bump();
                }
            }
        }
    }
}

/// Run `req`: on the worker when it is heavy and there are threads, else here.
fn dispatch(app: &mut SolveApp, req: Request) {
    if matches!(req.asker, Asker::Play) && !app.session.cam_view.as_deref().is_some_and(|v| v.playing) {
        return;
    }
    if let Asker::Agent(r) = &req.asker
        && r.refuse_cancelled_chat()
    {
        return;
    }
    let heavy = HEAVY.contains(&req.command.as_str());
    let has_view = app.session.cam_view.is_some();
    if !(heavy && threads() && app.apply_off_thread.is_some()) || (!has_view && req.command != "cam.view.load") {
        let result = app.run(&req.command, req.params.clone());
        answer(app, req.asker, &req.command, result);
        refresh_snapshot(app);
        return;
    }
    let scope = match &req.asker {
        Asker::Agent(r) => match crate::control::request_scope(app, &r.params) {
            Ok(s) => s,
            Err(e) => {
                answer(app, req.asker, &req.command, Err(e));
                return;
            }
        },
        _ => None,
    };
    let mut copy = app.session.scratch();
    if req.command != "cam.view.load" {
        if let Some(v) = app.session.cam_view.as_deref() {
            app.cam_player.held = (key_of(v, &app.cam_player.tiles), v.layers.part);
        }
        copy.cam_view = app.session.cam_view.take();
    }
    let label = solidcraft_engine::find_command(&req.command).map_or(req.command.as_str(), |c| c.label).to_string();
    let tiles = app.cam_player.tiles.clone();
    let (job_tx, job_rx) = std::sync::mpsc::channel::<(solidcraft_engine::Session, Request)>();
    let (tx, rx) = std::sync::mpsc::channel::<Back>();
    let spawned = std::thread::Builder::new().name("cam-view".into()).spawn(move || {
        let Ok((mut copy, req)) = job_rx.recv() else { return };
        let result = solidcraft_engine::io::vfs::scoped(scope, || copy.execute(&req.command, &req.params).map_err(|e| e.to_string()));
        if let Some(v) = copy.cam_view.as_deref() {
            refresh_tiles(v, &tiles);
        }
        let _ = tx.send(Back { view: copy.cam_view.take(), result, command: req.command, asker: req.asker });
    });
    if spawned.is_ok() {
        if job_tx.send((copy, req)).is_ok() {
            app.cam_player.away = Some(Away { rx, label, stop: None, since: crate::now_ms() });
        }
        return;
    }
    // No thread after all: here, now.
    app.session.cam_view = copy.cam_view.take();
    let result = app.run(&req.command, req.params.clone());
    answer(app, req.asker, &req.command, result);
    refresh_snapshot(app);
}

/// Step play on: a seek to where the tool is after the time since the last step.
fn play_step(app: &mut SolveApp) {
    let now = crate::now_ms();
    let Some(v) = app.session.cam_view.as_deref() else { return };
    if !v.playing {
        return;
    }
    let Some(p) = v.playback() else { return };
    let tl = p.timeline();
    let dt = app.cam_player.clock.map_or(0.0, |c| ((now - c) / 1000.0).clamp(0.0, MAX_STEP_S));
    app.cam_player.clock = Some(now);
    if dt <= 0.0 {
        return;
    }
    let t = p.time_now();
    let target = match v.moves_per_s {
        Some(mps) => {
            let (m, f) = p.at();
            let pos = m as f64 + f + dt * mps;
            let k = pos.floor().max(0.0) as usize;
            if k >= tl.moves.len() { f64::INFINITY } else { tl.time_at(k, pos - k as f64) }
        }
        None => t + dt * v.speed,
    };
    let at_end = target >= tl.duration || (p.at().0 + 1 >= tl.moves.len() && p.at().1 >= 1.0);
    let params = if at_end {
        if v.looping {
            json!({"percent": 0})
        } else {
            // The end: stop there.
            app.cam_player.queue.push_back(Request { command: "cam.view.seek".into(), params: json!({"percent": 100}), asker: Asker::Play });
            app.cam_player.queue.push_back(Request { command: "cam.view.pause".into(), params: json!({}), asker: Asker::Play });
            return;
        }
    } else {
        json!({"time_s": target})
    };
    app.cam_player.queue.push_back(Request { command: "cam.view.seek".into(), params, asker: Asker::Play });
}

/// Check the program to its end on the worker, in slices, then go back to where the tool was;
/// stopped by anything asked meanwhile (see the module notes).
fn prerun(app: &mut SolveApp) {
    let Some(v) = app.session.cam_view.as_deref() else { return };
    if v.playing {
        return;
    }
    let Some(p) = v.playback() else { return };
    let id = crate::cam_view_layer::view_id(v);
    if p.checked() >= p.plan().moves() || app.cam_player.prerun_failed == id || crate::now_ms() - app.cam_player.asked_at < QUIET_MS {
        return;
    }
    app.cam_player.held = (key_of(v, &app.cam_player.tiles), v.layers.part);
    let Some(mut view) = app.session.cam_view.take() else { return };
    let stop = Arc::new(AtomicBool::new(false));
    let progress = app.cam_player.prerun_at.clone();
    progress.store(p_checked(&view), Ordering::Relaxed);
    let (tx, rx) = std::sync::mpsc::channel::<Back>();
    let (job_tx, job_rx) = std::sync::mpsc::channel::<Box<CamView>>();
    let flag = stop.clone();
    let spawned = std::thread::Builder::new().name("cam-view-check".into()).spawn(move || {
        let Ok(mut view) = job_rx.recv() else { return };
        let result = check_to_end(&mut view, &flag, &progress);
        let _ = tx.send(Back { view: Some(view), result, command: "prerun".into(), asker: Asker::Play });
    });
    match spawned {
        Ok(_) => match job_tx.send(view) {
            Ok(()) => app.cam_player.away = Some(Away { rx, label: "prerun".into(), stop: Some(stop), since: crate::now_ms() }),
            Err(e) => {
                view = e.0;
                app.session.cam_view = Some(view);
            }
        },
        Err(_) => {
            app.cam_player.prerun_failed = id;
            view.bump();
            app.session.cam_view = Some(view);
        }
    }
}

fn p_checked(v: &CamView) -> usize {
    v.playback().map_or(0, |p| p.checked())
}

/// The pre-run's work on the worker: on to the end in slices while not stopped, then back.
fn check_to_end(view: &mut CamView, stop: &AtomicBool, progress: &AtomicUsize) -> Result<Value, String> {
    let Some(p) = view.playback_mut() else { return Ok(json!({})) };
    let at = p.at();
    let mut out = Ok(json!({}));
    while !stop.load(Ordering::Relaxed) {
        match p.seek(usize::MAX, 1.0, Some(PRERUN_SLICE)) {
            Ok(r) => {
                progress.store(p.checked(), Ordering::Relaxed);
                if r.done {
                    break;
                }
            }
            Err(e) => {
                out = Err(e.to_string());
                break;
            }
        }
    }
    if let Err(e) = p.seek(at.0, at.1, None) {
        out = Err(e.to_string());
    }
    view.bump();
    out
}

/// The panel's picture of the view, from the view in the session.
pub(crate) fn refresh_snapshot(app: &mut SolveApp) {
    let Some(v) = app.session.cam_view.as_deref() else {
        if !app.cam_player.away() {
            app.cam_player.snapshot = None;
        }
        return;
    };
    let Some(p) = v.playback() else { return };
    let Ok(state) = v.state_reply(FindingsListing::Summary, 100) else { return };
    let tl = p.timeline();
    let time_of = |m: usize, f: f64| tl.time_at(m, f);
    let ops = tl
        .ops
        .iter()
        .map(|o| OpBand { index: o.index, name: o.name.clone(), start_s: time_of(o.first, 0.0), end_s: time_of(o.end.saturating_sub(1), 1.0) })
        .collect();
    let changes = tl.tool_changes.iter().map(|c| (time_of(c.at_move, 0.0), c.tool)).collect();
    let findings = p
        .findings()
        .events()
        .iter()
        .enumerate()
        .take(2000)
        .map(|(index, e)| FindingMark {
            index,
            time_s: e.at_move.map_or(0.0, |m| time_of(m, 1.0)),
            line: e.line,
            code: e.code.clone(),
            error: e.severity == Severity::Error,
            message: e.message.clone(),
        })
        .collect();
    let by_op = v.colour_by == solidcraft_engine::schema::cam_view::ColourBy::Operation;
    app.cam_player.snapshot = Some(Snapshot { state, ops, changes, findings, duration: tl.duration, checked: p.checked(), by_op });
}

/// Whether a panel or an agent made the view change this frame and it is home: the snapshot
/// follows commands that ran elsewhere (the command line, a script).
pub(crate) fn follow(app: &mut SolveApp) {
    let frame = app.session.cam_view.as_deref().map(|v| v.frame);
    let shown = app.cam_player.snapshot.as_ref().map(|s| s.state.frame);
    if frame.is_some() && frame != shown && !app.cam_player.away() {
        refresh_snapshot(app);
    }
    if frame.is_none() && !app.cam_player.away() && app.cam_player.snapshot.is_some() {
        app.cam_player.snapshot = None;
    }
}

#[cfg(test)]
pub(crate) fn queued(app: &SolveApp) -> Vec<(String, Value)> {
    app.cam_player.queue.iter().map(|r| (r.command.clone(), r.params.clone())).collect()
}
