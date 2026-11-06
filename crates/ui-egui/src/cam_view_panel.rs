//! The CAM simulation panel: load a job, play it, step it (by move, operation, tool change or
//! finding), scrub its timeline (operations as bands, tool changes as ticks, findings as marks,
//! the part already checked under them), read where the tool is (line and G-code, operation,
//! tool, position, feed, spindle, time, material removed), list the findings (click one to go
//! there), and pick what the 3D view shows. Every action is a `cam.view.*` command, asked of the
//! player (`cam_view_player`), which runs it in order and off the window thread where it cuts
//! the stock. Opened by `ui.start` of `cam.view.load`, and shown whenever a view is loaded (an
//! agent's load shows it too); closing it closes the view.

use egui::{Align2, Pos2, Rect, RichText, Sense, Stroke, pos2, vec2};
use serde_json::{Value, json};

use crate::SolveApp;
use crate::cam_view_layer::category;
use crate::cam_view_player::{FindingMark, OpBand, Snapshot};
use crate::dialogs_motion::{Transport, transport};
use crate::i18n::{format, tr};
use crate::theme::Tokens;
use crate::ui_kit::BackingTooltipExt as _;

/// Playback speeds offered, times the program's estimated real time.
pub(crate) const SPEEDS: [f64; 9] = [0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 64.0, 256.0];
/// The speed play starts at.
const DEFAULT_SPEED: usize = 6;
/// Findings listed at most.
const LISTED: usize = 200;
/// The panel's width, in form cards.
const CARDS: f32 = 0.8;
/// A drop-down's width in the panel's rows.
const PICK_WIDTH: f32 = 130.0;

/// A drop-down `PICK_WIDTH` wide; whether an option was picked.
fn pick(ui: &mut egui::Ui, id: &str, selected: &mut usize, options: &[&str]) -> bool {
    ui.allocate_ui(vec2(PICK_WIDTH, crate::ui_kit::row_height()), |ui| {
        ui.set_width(PICK_WIDTH);
        crate::ui_kit::dropdown_picked(ui, id, selected, options)
    })
    .inner
}

/// What a step of the transport's back and forward buttons is.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StepBy {
    Move,
    Op,
    ToolChange,
    Finding,
}

impl StepBy {
    const ALL: [StepBy; 4] = [StepBy::Move, StepBy::Op, StepBy::ToolChange, StepBy::Finding];

    fn label(self) -> &'static str {
        match self {
            StepBy::Move => "ui.cam_view.step_move",
            StepBy::Op => "ui.cam_view.step_op",
            StepBy::ToolChange => "ui.cam_view.step_tool",
            StepBy::Finding => "ui.cam_view.step_finding",
        }
    }

    /// `cam.view.seek`'s parameters for a step forward (or back).
    pub(crate) fn params(self, forward: bool) -> Value {
        let by = if forward { 1 } else { -1 };
        match self {
            StepBy::Move => json!({"by": by}),
            StepBy::Op => json!({"by": by, "until": "op"}),
            StepBy::ToolChange => json!({"by": by, "until": "tool_change"}),
            StepBy::Finding => json!({"by": by, "until": "finding"}),
        }
    }
}

/// The panel's own state; the view's is in the engine and in the player's snapshot.
pub struct CamPanel {
    pub job: String,
    pub program: String,
    pub machine: String,
    pub holding: String,
    /// The load rows shown while a view is loaded.
    pub show_load: bool,
    pub speed: usize,
    pub looping: bool,
    pub step: StepBy,
}

impl Default for CamPanel {
    fn default() -> CamPanel {
        CamPanel {
            job: String::new(),
            program: String::new(),
            machine: String::new(),
            holding: String::new(),
            show_load: false,
            speed: DEFAULT_SPEED,
            looping: false,
            step: StepBy::Move,
        }
    }
}

/// Open the panel (`ui.start` of `cam.view.load`).
pub fn open(app: &mut SolveApp) {
    if app.cam_panel.is_none() {
        app.cam_panel = Some(CamPanel::default());
    }
}

/// `cam.view.load`'s parameters from the load rows.
pub(crate) fn load_params(d: &CamPanel) -> Value {
    let mut p = json!({"job_path": d.job.trim(), "colour_by": "operation"});
    for (key, v) in [("program", &d.program), ("machine_model_path", &d.machine), ("workholding_path", &d.holding)] {
        if !v.trim().is_empty() {
            p[key] = json!(v.trim());
        }
    }
    p
}

/// `x` of time `t` on a strip from `left`, `width` wide, of a program `duration` long.
pub(crate) fn time_to_x(t: f64, duration: f64, left: f32, width: f32) -> f32 {
    if !(duration.is_finite() && duration > 0.0 && t.is_finite()) {
        return left;
    }
    left + width * (t / duration).clamp(0.0, 1.0) as f32
}

/// The time at `x` on such a strip.
pub(crate) fn x_to_time(x: f32, duration: f64, left: f32, width: f32) -> f64 {
    if !(duration.is_finite() && duration > 0.0 && width > 0.0 && x.is_finite()) {
        return 0.0;
    }
    (f64::from((x - left) / width)).clamp(0.0, 1.0) * duration
}

/// The finding nearest time `t` within `slack` seconds.
pub(crate) fn finding_near(findings: &[FindingMark], t: f64, slack: f64) -> Option<&FindingMark> {
    findings.iter().filter(|f| (f.time_s - t).abs() <= slack).min_by(|a, b| (a.time_s - t).abs().total_cmp(&(b.time_s - t).abs()))
}

/// The operation running at time `t`.
pub(crate) fn op_at(ops: &[OpBand], t: f64) -> Option<&OpBand> {
    ops.iter().find(|o| t >= o.start_s && t <= o.end_s)
}

/// A time as minutes and seconds (hours when long).
pub(crate) fn clock(s: f64) -> String {
    let s = if s.is_finite() { s.max(0.0).round() as u64 } else { 0 };
    if s >= 3600 { format!("{}:{:02}:{:02}", s / 3600, (s / 60) % 60, s % 60) } else { format!("{:02}:{:02}", s / 60, s % 60) }
}

/// Draw the panel when it is open or a view is loaded. Called once per frame.
pub fn show(app: &mut SolveApp, ctx: &egui::Context) {
    let loaded = app.session.cam_view.is_some() || app.cam_player.away();
    if app.cam_panel.is_none() && loaded {
        open(app);
    }
    let Some(mut d) = app.cam_panel.take() else { return };
    let mut asks: Vec<(&'static str, Value)> = Vec::new();
    let screen = ctx.content_rect();
    let at = pos2(screen.right() - Tokens::get().space_16, screen.top() + Tokens::get().form_header_height * 3.0);
    let app_ref: &SolveApp = app;
    let closed = crate::ui_kit::Window::new(egui::Id::unique("sc_cam_view_panel"), "list", tr("ui.cam_view.title"))
        .cards(CARDS)
        .placed(Align2::RIGHT_TOP, at)
        .keep_open_on_escape()
        .show_body(ctx, |ui| body(app_ref, &mut d, ui, &mut asks));
    for (command, params) in asks {
        app.cam_player.ask(command, params);
    }
    if closed {
        if loaded {
            app.cam_player.ask("cam.view.close", json!({}));
        }
        app.cam_panel = None;
    } else {
        app.cam_panel = Some(d);
    }
}

fn body(app: &SolveApp, d: &mut CamPanel, ui: &mut egui::Ui, asks: &mut Vec<(&'static str, Value)>) {
    // The card's width: rows that take all the room there is (a grid, a drop-down) stay in it.
    let width = (Tokens::get().form_card_width * CARDS - 2.0 * Tokens::get().space_12).max(200.0);
    ui.vertical(|ui| {
        ui.set_width(width);
        rows(app, d, ui, asks);
    });
}

fn rows(app: &SolveApp, d: &mut CamPanel, ui: &mut egui::Ui, asks: &mut Vec<(&'static str, Value)>) {
    let t = Tokens::get();
    if let Some((label, pre)) = app.cam_player.busy() {
        let text = match (pre, app.cam_player.snapshot.as_ref()) {
            (Some(done), Some(s)) if s.state.moves > 0 => {
                let pc = format!("{:.0}", 100.0 * done as f64 / s.state.moves as f64);
                format("ui.cam_view.checking", &[("percent", pc.as_str())])
            }
            _ => format("ui.cam_view.working", &[("command", crate::i18n::tr_label(label))]),
        };
        ui.horizontal(|ui| {
            ui.spinner();
            ui.label(RichText::new(text).color(t.text_dim));
        });
    }
    if let Some(e) = &app.cam_player.error {
        ui.label(RichText::new(e.as_str()).color(t.error));
    }
    let Some(s) = app.cam_player.snapshot.as_ref() else {
        load_rows(d, ui, asks);
        return;
    };
    transport_row(d, s, ui, asks);
    timeline_strip(s, ui, asks, &t);
    readout(s, ui, &t);
    findings(s, ui, asks, &t);
    layers(s, ui, asks);
    ui.horizontal(|ui| {
        let label = tr("ui.cam_view.load_other");
        if crate::ui_kit::button(ui, label, crate::ui_kit::Button::Small).clicked() {
            d.show_load = !d.show_load;
        }
    });
    if d.show_load {
        load_rows(d, ui, asks);
    }
}

fn load_rows(d: &mut CamPanel, ui: &mut egui::Ui, asks: &mut Vec<(&'static str, Value)>) {
    crate::ui_kit::text_row(ui, tr("ui.cam_view.job"), &mut d.job, tr("ui.cam_view.job_hint"));
    crate::ui_kit::text_row(ui, tr("ui.cam_view.program"), &mut d.program, tr("ui.cam_view.optional"));
    crate::ui_kit::text_row(ui, tr("ui.cam_view.machine"), &mut d.machine, tr("ui.cam_view.optional"));
    crate::ui_kit::text_row(ui, tr("ui.cam_view.holding"), &mut d.holding, tr("ui.cam_view.optional"));
    let ready = !d.job.trim().is_empty();
    if crate::ui_kit::button(ui, tr("ui.cam_view.load"), crate::ui_kit::Button::Enabled(ready)).clicked() && ready {
        asks.push(("cam.view.load", load_params(d)));
        d.show_load = false;
    }
}

fn transport_row(d: &mut CamPanel, s: &Snapshot, ui: &mut egui::Ui, asks: &mut Vec<(&'static str, Value)>) {
    let playing = s.state.playing;
    ui.horizontal(|ui| {
        if transport(ui, Transport::Start, tr("ui.cam_view.start")).clicked() {
            asks.push(("cam.view.seek", json!({"percent": 0})));
        }
        if transport(ui, Transport::Back, tr("ui.cam_view.back")).clicked() {
            asks.push(("cam.view.seek", d.step.params(false)));
        }
        let (what, tip) = if playing { (Transport::Stop, "ui.cam_view.pause") } else { (Transport::Play, "ui.cam_view.play") };
        if transport(ui, what, tr(tip)).clicked() {
            if playing {
                asks.push(("cam.view.pause", json!({})));
            } else {
                let speed = SPEEDS.get(d.speed).copied().unwrap_or(1.0);
                // From the start again when played at the end.
                if s.state.percent >= 100.0 {
                    asks.push(("cam.view.seek", json!({"percent": 0})));
                }
                asks.push(("cam.view.play", json!({"speed": speed, "loop": d.looping})));
            }
        }
        if transport(ui, Transport::Forward, tr("ui.cam_view.forward")).clicked() {
            asks.push(("cam.view.seek", d.step.params(true)));
        }
        if transport(ui, Transport::End, tr("ui.cam_view.end")).clicked() {
            asks.push(("cam.view.seek", json!({"percent": 100})));
        }
        ui.separator();
        let steps: Vec<&str> = StepBy::ALL.iter().map(|k| tr(k.label())).collect();
        let mut k = StepBy::ALL.iter().position(|x| *x == d.step).unwrap_or(0);
        ui.label(tr("ui.cam_view.step"));
        if pick(ui, "sc_cam_step", &mut k, &steps) {
            d.step = StepBy::ALL.get(k).copied().unwrap_or(StepBy::Move);
        }
    });
    ui.horizontal(|ui| {
        let speeds: Vec<String> = SPEEDS.iter().map(|x| format!("×{x}")).collect();
        let names: Vec<&str> = speeds.iter().map(String::as_str).collect();
        ui.label(tr("ui.cam_view.speed"));
        if pick(ui, "sc_cam_speed", &mut d.speed, &names) && playing {
            asks.push(("cam.view.play", json!({"speed": SPEEDS.get(d.speed).copied().unwrap_or(1.0)})));
        }
        let was = d.looping;
        crate::ui_kit::checkbox(ui, tr("ui.cam_view.loop"), &mut d.looping);
        if d.looping != was && playing {
            asks.push(("cam.view.play", json!({"loop": d.looping})));
        }
    });
}

/// The timeline: operations, tool changes, findings, the checked part and the playhead; a click
/// or a drag goes there.
fn timeline_strip(s: &Snapshot, ui: &mut egui::Ui, asks: &mut Vec<(&'static str, Value)>, t: &Tokens) {
    let width = ui.available_width().max(120.0);
    let (r, resp) = ui.allocate_exact_size(vec2(width, 34.0), Sense::click_and_drag());
    let p = ui.painter_at(r);
    p.rect_filled(r, t.radius_sm, t.field);
    let inner = r.shrink2(vec2(4.0, 3.0));
    let x = |time: f64| time_to_x(time, s.duration, inner.left(), inner.width());
    let band = Rect::from_min_max(pos2(inner.left(), inner.top()), pos2(inner.right(), inner.top() + 14.0));
    for o in &s.ops {
        let (x0, x1) = (x(o.start_s), x(o.end_s).max(x(o.start_s) + 1.0));
        p.rect_filled(Rect::from_min_max(pos2(x0, band.top()), pos2(x1, band.bottom())), 0.0, category(t, o.index).gamma_multiply(0.85));
    }
    for (time, _) in &s.changes {
        let cx = x(*time);
        p.line_segment([pos2(cx, inner.top()), pos2(cx, inner.bottom())], Stroke::new(1.0, t.text_dim));
    }
    // How far the program has been checked for findings.
    let checked = s.checked as f64 / s.state.moves.max(1) as f64;
    let cy = inner.bottom() - 1.0;
    p.line_segment([pos2(inner.left(), cy), pos2(inner.left() + inner.width() * checked.clamp(0.0, 1.0) as f32, cy)], Stroke::new(2.0, t.text_dim));
    for f in &s.findings {
        let fx = x(f.time_s);
        let c = if f.error { t.error } else { t.warning };
        p.add(egui::Shape::convex_polygon(
            vec![pos2(fx, inner.bottom() - 12.0), pos2(fx + 4.0, inner.bottom() - 4.0), pos2(fx - 4.0, inner.bottom() - 4.0)],
            c,
            Stroke::NONE,
        ));
    }
    let px = x(s.state.time_s);
    p.line_segment([pos2(px, r.top()), pos2(px, r.bottom())], Stroke::new(2.0, t.text));
    p.add(egui::Shape::convex_polygon(vec![pos2(px - 5.0, r.top()), pos2(px + 5.0, r.top()), pos2(px, r.top() + 6.0)], t.text, Stroke::NONE));
    let pointer: Option<Pos2> = resp.interact_pointer_pos().or(resp.hover_pos());
    if (resp.clicked() || resp.dragged())
        && let Some(at) = resp.interact_pointer_pos()
    {
        asks.push(("cam.view.seek", json!({"time_s": x_to_time(at.x, s.duration, inner.left(), inner.width())})));
    }
    if let Some(at) = pointer.filter(|_| resp.hovered()) {
        let time = x_to_time(at.x, s.duration, inner.left(), inner.width());
        let mut tip = clock(time);
        if let Some(o) = op_at(&s.ops, time) {
            tip = format!("{tip}  {}", o.name);
        }
        let slack = s.duration * 6.0 / f64::from(inner.width().max(1.0));
        if let Some(f) = finding_near(&s.findings, time, slack) {
            tip = format!("{tip}\n{} {}: {}", tr("ui.cam_view.line"), f.line, f.message);
        }
        resp.backing_on_hover_text(tip);
    }
}

/// Where the tool is.
fn readout(s: &Snapshot, ui: &mut egui::Ui, t: &Tokens) {
    let st = &s.state;
    let dim = |x: &str| RichText::new(x.to_string()).color(t.text_dim);
    egui::Grid::new("sc_cam_readout").num_columns(2).min_col_width(90.0).spacing([12.0, 4.0]).show(ui, |ui| {
        ui.label(dim(tr("ui.cam_view.line")));
        ui.label(RichText::new(format!("{}  {}", st.line, st.gcode)).monospace());
        ui.end_row();
        ui.label(dim(tr("ui.cam_view.operation")));
        ui.label(st.op.as_ref().map_or_else(|| "–".to_string(), |o| o.name.clone()));
        ui.end_row();
        ui.label(dim(tr("ui.cam_view.tool")));
        ui.label(st.tool.as_ref().map_or_else(|| "–".to_string(), |k| format!("T{}  {}  Ø{:.3}", k.number, k.kind, k.diameter)));
        ui.end_row();
        ui.label(dim(tr("ui.cam_view.position")));
        let m = &st.position.machine;
        let mut pos = format!("X {:.2}  Y {:.2}  Z {:.2}", m.x, m.y, m.z);
        if let Some(a) = m.a_deg {
            pos = format!("{pos}  A {a:.1}°");
        }
        ui.label(RichText::new(pos).monospace());
        ui.end_row();
        ui.label(dim(tr("ui.cam_view.feed")));
        let kind = tr(match st.kind.as_str() {
            "rapid" => "ui.cam_view.kind_rapid",
            "arc" => "ui.cam_view.kind_arc",
            "plunge" => "ui.cam_view.kind_plunge",
            _ => "ui.cam_view.kind_feed",
        });
        ui.label(st.feed.map_or_else(|| kind.to_string(), |f| format!("{f:.0} mm/min  ({kind})")));
        ui.end_row();
        ui.label(dim(tr("ui.cam_view.spindle")));
        ui.label(st.rpm.map_or_else(|| "–".to_string(), |r| format!("{r:.0} rpm")));
        ui.end_row();
        ui.label(dim(tr("ui.cam_view.time")));
        ui.label(format!("{} / {}  ({:.1} %)", clock(st.time_s), clock(st.duration_s), st.percent));
        ui.end_row();
        ui.label(dim(tr("ui.cam_view.removed")));
        ui.label(format!("{:.0} mm³", st.removed_volume));
        ui.end_row();
    });
}

fn findings(s: &Snapshot, ui: &mut egui::Ui, asks: &mut Vec<(&'static str, Value)>, t: &Tokens) {
    let so_far = &s.state.findings_so_far;
    let (e, w, i) = (so_far.errors.to_string(), so_far.warnings.to_string(), so_far.infos.to_string());
    ui.label(RichText::new(format("ui.cam_view.findings_count", &[("errors", &e), ("warnings", &w), ("infos", &i)])).strong());
    let now = s.state.time_s;
    let shown: Vec<&FindingMark> = s.findings.iter().take(LISTED).collect();
    if shown.is_empty() {
        ui.label(RichText::new(tr("ui.cam_view.no_findings")).color(t.text_dim));
        return;
    }
    egui::ScrollArea::vertical().id_salt("sc_cam_findings").max_height(120.0).show(ui, |ui| {
        for f in shown {
            ui.horizontal(|ui| {
                let (dot, _) = ui.allocate_exact_size(vec2(10.0, 10.0), Sense::hover());
                ui.painter().circle_filled(dot.center(), 4.0, if f.error { t.error } else { t.warning });
                let ahead = f.time_s > now + 1e-9;
                let text = format!("{} {}  {}", tr("ui.cam_view.line"), f.line, f.code);
                let key = format!("sc_cam_finding_{}", f.index);
                let picked = crate::ui_kit::list_picker(ui, &key, &text, false);
                let picked = picked.backing_on_hover_text(f.message.clone());
                if picked.clicked() {
                    asks.push(("cam.view.seek", json!({"finding": f.index})));
                }
                if ahead {
                    ui.label(RichText::new(clock(f.time_s)).color(t.text_dim));
                }
            });
        }
    });
}

/// The layers, each a `cam.view.show` switch.
fn layers(s: &Snapshot, ui: &mut egui::Ui, asks: &mut Vec<(&'static str, Value)>) {
    let l = s.state.layers;
    let mut v = [l.toolpaths, l.rapids, l.remaining_path, l.stock, l.part, l.tool, l.holder, l.machine, l.fixtures, l.gouges, l.collisions];
    const KEYS: [&str; 11] =
        ["toolpaths", "rapids", "remaining_path", "stock", "part", "tool", "holder", "machine", "fixtures", "gouges", "collisions"];
    let before = v;
    let labels: Vec<String> = KEYS.iter().map(|k| tr(&format!("ui.cam_view.layer_{k}")).to_string()).collect();
    ui.label(RichText::new(tr("ui.cam_view.layers")).strong());
    {
        let mut items: Vec<(&str, &mut bool)> = labels.iter().map(String::as_str).zip(v.iter_mut()).collect();
        crate::ui_kit::checkbox_grid(ui, &mut items, 3);
    }
    let mut changed = serde_json::Map::new();
    for ((k, a), b) in KEYS.iter().zip(before).zip(v) {
        if a != b {
            changed.insert((*k).into(), json!(b));
        }
    }
    let by_op = usize::from(s.by_op);
    let mut choice = by_op;
    ui.horizontal(|ui| {
        ui.label(tr("ui.cam_view.colour_by"));
        let options = [tr("ui.cam_view.by_tool"), tr("ui.cam_view.by_operation")];
        if pick(ui, "sc_cam_colour_by", &mut choice, &options) && choice != by_op {
            changed.insert("colour_by".into(), json!(if choice == 1 { "operation" } else { "tool" }));
        }
    });
    if !changed.is_empty() {
        asks.push(("cam.view.show", Value::Object(changed)));
    }
}

#[cfg(test)]
#[path = "cam_view_panel_tests.rs"]
mod tests;
