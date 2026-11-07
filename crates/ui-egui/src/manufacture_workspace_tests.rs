//! The Manufacture workspace: entering and leaving, the job found beside the design, the tree
//! and its states, Undo routed to the job, a finding clicked, Simulate from the open job,
//! saving after a burst of edits, edits made outside, and which toolpaths the view draws.

use std::path::PathBuf;
use std::time::Duration;

use serde_json::{Value, json};
use solidcraft_engine::cam_job_state::OpPath;
use solidcraft_engine::schema::cam_job_edit::OpStatus;

use super::*;
use crate::scenario::Harness;

/// A folder of its own, removed afterwards.
struct Dir(PathBuf);

impl Dir {
    fn new(tag: &str) -> Dir {
        static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let n = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let d = std::env::temp_dir().join(format!("solidcraft-mf-ui-{tag}-{}-{n}", std::process::id()));
        std::fs::create_dir_all(&d).unwrap();
        Dir(d)
    }

    fn file(&self, name: &str) -> String {
        self.0.join(name).to_string_lossy().into_owned()
    }
}

impl Drop for Dir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

const POCKET: [[f64; 2]; 4] = [[-8.0, -5.0], [8.0, -5.0], [8.0, 5.0], [-8.0, 5.0]];

fn job(low_clearance: bool) -> Value {
    let mut ops = vec![
        json!({"name": "pocket", "type": "pocket", "tool": 4, "polygon": POCKET, "stepover": 1.5}),
        json!({"name": "outline", "type": "contour", "tool": 4, "polygon": [[-20, -15], [20, -15], [20, 15], [-20, 15]], "heights": {"bottom": 10}}),
    ];
    if low_clearance {
        ops.push(json!({"name": "low clearance", "type": "pocket", "tool": 4, "polygon": POCKET, "heights": {"clearance": 19, "retract": 19, "top": 18, "bottom": 15}}));
    }
    json!({
        "name": "block", "machine": "3axis", "post": "mach3_4axis",
        "tools": [{"number": 4, "kind": "flat_endmill", "diameter": 4, "flute_length": 12, "spindle_rpm": 12000, "feed": 800, "plunge_feed": 200}],
        "operations": ops
    })
}

/// The block with its pocket, "saved" as `block.sc` in `dir`, its job beside it.
fn harness(dir: &Dir, low_clearance: bool) -> Harness {
    let mut h = Harness::new();
    h.app.run("solid.box", json!({"length": 40, "width": 30, "height": 20, "corner": [-20, -15, 0], "body_name": "Block"})).unwrap();
    h.app.run("solid.box", json!({"length": 16, "width": 10, "height": 5, "corner": [-8, -5, 15], "operation": "cut"})).unwrap();
    h.app.session.path = Some(dir.file("block.sc"));
    std::fs::write(dir.file("block.cam.json"), serde_json::to_string_pretty(&job(low_clearance)).unwrap()).unwrap();
    h
}

fn ops(h: &Harness) -> Vec<Value> {
    h.app.session.cam_job.as_ref().map(|j| j.operations().to_vec()).unwrap_or_default()
}

#[test]
fn entering_finds_the_job_beside_the_design_and_leaving_gives_the_camera_back() {
    let dir = Dir::new("enter");
    let mut h = harness(&dir, false);
    h.frames(2);
    let camera = h.app.cam;
    let tab = h.app.ui.tab.clone();
    switch(&mut h.app, "manufacture").unwrap();
    assert!(active(&h.app));
    assert_eq!(current(&h.app), "manufacture");
    assert_eq!(h.app.session.cam_job.as_ref().and_then(|j| j.path.clone()), Some(dir.file("block.cam.json")), "opened from beside the design");
    h.frames(3);
    assert!(crate::workspace::visible_tabs(&h.app).is_empty(), "the stage bar takes the ribbon's place");
    h.app.cam.distance *= 3.0;
    switch(&mut h.app, "design").unwrap();
    assert_eq!((h.app.cam, h.app.ui.tab.clone()), (camera, tab));
    assert!(h.app.shell.workspace_panels.keys().all(|k| !k.starts_with("manufacture:")));
    // Refused while a sketch is open.
    h.app.run("sketch.create", json!({"plane": "XY"})).unwrap();
    assert!(switch(&mut h.app, "manufacture").is_err());
    assert!(!active(&h.app));
}

#[test]
fn several_jobs_beside_a_design_are_listed_not_opened() {
    let dir = Dir::new("several");
    let mut h = harness(&dir, false);
    std::fs::remove_file(dir.file("block.cam.json")).unwrap();
    for side in ["side1", "side2"] {
        std::fs::write(dir.file(&format!("block.{side}.cam.json")), serde_json::to_string(&job(false)).unwrap()).unwrap();
    }
    std::fs::write(dir.file("other.cam.json"), "{}").unwrap();
    switch(&mut h.app, "manufacture").unwrap();
    assert!(h.app.session.cam_job.is_none());
    assert_eq!(h.app.manufacture.beside, [dir.file("block.side1.cam.json"), dir.file("block.side2.cam.json")]);
    h.frames(2);
}

#[test]
fn the_tree_shows_each_operations_state_and_a_finding_click_goes_to_it() {
    let dir = Dir::new("tree");
    let mut h = harness(&dir, true);
    switch(&mut h.app, "manufacture").unwrap();
    h.frames(2);
    let status = |h: &Harness, i: usize| h.app.session.cam_job.as_ref().unwrap().status(i, h.app.session.model_stamp());
    assert_eq!(status(&h, 0), OpStatus::Unplanned);
    // Headless, Generate runs at once; a failure takes the stage to Check.
    crate::manufacture_ui::generate_now(&mut h.app);
    assert_eq!(h.app.manufacture.stage, Stage::Check);
    // The job was refused: the operation at fault is an error, the others were not planned.
    assert_eq!((status(&h, 0), status(&h, 2)), (OpStatus::Unplanned, OpStatus::Error));
    h.frames(3);
    let findings = h.app.session.cam_job.as_ref().unwrap().generated.as_ref().unwrap().findings.clone();
    let error = findings.iter().find(|f| f.severity == "error").unwrap();
    // Clicking it in the Check list selects its operation and marks its place.
    h.step(&json!({"click": {"handle": format!("manufacture:finding:{}", error.index)}})).unwrap();
    assert_eq!(h.app.manufacture.selected, Some(2));
    assert_eq!(h.app.manufacture.mark.map(|m| m[2]), Some(20.0));
    // Turning an operation off from its row's eye.
    h.app.manufacture.stage = Stage::Operations;
    h.frames(2);
    h.app.run("cam.op.enable", json!({"op": 1, "enabled": false})).unwrap();
    assert_eq!(status(&h, 1), OpStatus::Off);
    assert_eq!(status(&h, 0), OpStatus::Stale, "the job changed since the Generate");
    h.frames(2);
}

#[test]
fn undo_in_the_workspace_is_the_jobs_and_outside_it_the_designs() {
    let dir = Dir::new("undo");
    let mut h = harness(&dir, false);
    switch(&mut h.app, "manufacture").unwrap();
    let design_steps = h.app.session.undo.len();
    h.app.run("cam.op.remove", json!({"op": "outline"})).unwrap();
    assert_eq!(ops(&h).len(), 1);
    assert!(takes_command(&mut h.app, "edit.undo"));
    assert_eq!(ops(&h).len(), 2, "the job's step undone");
    assert_eq!(h.app.session.undo.len(), design_steps, "the design's history untouched");
    assert!(takes_command(&mut h.app, "edit.redo"));
    assert_eq!(ops(&h).len(), 1);
    switch(&mut h.app, "design").unwrap();
    assert!(!takes_command(&mut h.app, "edit.undo"), "outside the workspace Undo is the design's");
    h.app.run("edit.undo", json!({})).unwrap();
    assert_eq!(h.app.session.undo.len(), design_steps - 1);
    assert_eq!(ops(&h).len(), 1);
}

#[test]
fn a_burst_of_edits_is_saved_once_a_moment_after_the_last() {
    let dir = Dir::new("autosave");
    let mut h = harness(&dir, false);
    switch(&mut h.app, "manufacture").unwrap();
    let path = dir.file("block.cam.json");
    let before = std::fs::read_to_string(&path).unwrap();
    h.app.preferences.cam_job_autosave = true;
    h.app.run("cam.op.edit", json!({"op": 0, "rename": "first"})).unwrap();
    autosave(&mut h.app, 0.0);
    h.app.run("cam.op.edit", json!({"op": 0, "rename": "second"})).unwrap();
    autosave(&mut h.app, 600.0);
    autosave(&mut h.app, 1200.0);
    assert_eq!(std::fs::read_to_string(&path).unwrap(), before, "not yet: the last edit was 600 ms before");
    autosave(&mut h.app, 1700.0);
    let saved = std::fs::read_to_string(&path).unwrap();
    assert!(saved.contains("\"second\"") && !h.app.session.cam_job.as_ref().unwrap().dirty(), "{saved}");
    // Off: the job stays unsaved.
    h.app.preferences.cam_job_autosave = false;
    h.app.run("cam.op.edit", json!({"op": 0, "rename": "third"})).unwrap();
    autosave(&mut h.app, 5000.0);
    autosave(&mut h.app, 9000.0);
    assert!(h.app.session.cam_job.as_ref().unwrap().dirty());
}

/// The job stays open when the workspace is left, and agents edit it from anywhere: an edit
/// made just before leaving, or while another workspace shows, is saved all the same (it used
/// to wait, unsaved, until the workspace was shown again). A save the file refuses is not tried
/// again every second, only after the next change.
#[test]
fn the_open_job_is_saved_after_an_edit_made_while_another_workspace_shows() {
    let dir = Dir::new("autosave-away");
    let mut h = harness(&dir, false);
    switch(&mut h.app, "manufacture").unwrap();
    h.app.preferences.cam_job_autosave = true;
    let path = dir.file("block.cam.json");
    h.app.run("cam.op.edit", json!({"op": 0, "rename": "left behind"})).unwrap();
    switch(&mut h.app, "design").unwrap();
    keep_up(&mut h.app, 0.0);
    h.app.run("cam.op.edit", json!({"op": 1, "rename": "by an agent"})).unwrap();
    keep_up(&mut h.app, 500.0);
    keep_up(&mut h.app, 1600.0);
    let saved = std::fs::read_to_string(&path).unwrap();
    assert!(saved.contains("left behind") && saved.contains("by an agent"), "{saved}");
    assert!(!h.app.session.cam_job.as_ref().unwrap().dirty());
    // The file changed outside under an unsaved edit: the save is refused once, and not again
    // until the job changes.
    h.app.run("cam.op.edit", json!({"op": 0, "rename": "mine"})).unwrap();
    std::fs::write(&path, serde_json::to_string_pretty(&job(false)).unwrap()).unwrap();
    keep_up(&mut h.app, 2000.0);
    keep_up(&mut h.app, 3100.0);
    assert!(h.app.manufacture.error.take().is_some_and(|e| e.contains("changed outside")));
    keep_up(&mut h.app, 4200.0);
    keep_up(&mut h.app, 9000.0);
    assert_eq!(h.app.manufacture.error, None, "not tried again before the next change");
    assert!(std::fs::read_to_string(&path).unwrap().contains("\"pocket\""), "their file is kept");
}

#[test]
fn an_edit_made_outside_reloads_a_clean_job_and_shows_a_banner_over_unsaved_edits() {
    let dir = Dir::new("outside");
    let mut h = harness(&dir, false);
    switch(&mut h.app, "manufacture").unwrap();
    let path = dir.file("block.cam.json");
    let mut theirs = job(false);
    theirs["operations"][0]["stepover"] = json!(2);
    std::fs::write(&path, serde_json::to_string_pretty(&theirs).unwrap()).unwrap();
    poll(&mut h.app);
    assert_eq!(ops(&h)[0]["stepover"], 2, "reloaded");
    assert!(h.app.manufacture.banner.is_none());
    h.app.preferences.cam_job_autosave = false;
    h.app.run("cam.op.edit", json!({"op": 0, "rename": "mine"})).unwrap();
    theirs["name"] = json!("theirs!");
    std::fs::write(&path, serde_json::to_string_pretty(&theirs).unwrap()).unwrap();
    poll(&mut h.app);
    assert!(
        matches!(&h.app.manufacture.banner, Some(Banner::Changed(lines)) if lines.iter().any(|l| l.contains("name"))),
        "{:?}",
        h.app.manufacture.banner
    );
    assert_eq!(op_name0(&h), "mine", "nothing dropped");
    h.frames(2);
    h.step(&json!({"click": {"handle": "manufacture:banner:keep"}})).unwrap();
    assert!(h.app.manufacture.banner.is_none());
    let kept: Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!(kept["operations"][0]["name"], "mine");
    std::fs::remove_file(&path).unwrap();
    poll(&mut h.app);
    assert_eq!(h.app.manufacture.banner, Some(Banner::Gone));
}

fn op_name0(h: &Harness) -> String {
    solidcraft_engine::cam_job_state::op_name(&ops(h)[0])
}

#[test]
fn simulate_loads_the_open_job_into_the_simulation_view() {
    let dir = Dir::new("simulate");
    let mut h = harness(&dir, false);
    switch(&mut h.app, "manufacture").unwrap();
    simulate(&mut h.app);
    for _ in 0..2000 {
        h.frame();
        if h.app.session.cam_view.is_some() && !h.app.cam_player.away() && !h.app.cam_player.pending() {
            break;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    assert!(h.app.session.cam_view.is_some(), "{:?}", h.app.cam_player.error);
    assert_eq!(h.app.cam_panel.as_ref().map(|p| p.job.clone()), Some(dir.file("block.cam.json")));
}

#[test]
fn write_program_runs_off_the_window_thread_and_lists_what_it_wrote() {
    let dir = Dir::new("write");
    let mut h = harness(&dir, false);
    switch(&mut h.app, "manufacture").unwrap();
    h.app.apply_off_thread = Some(Duration::ZERO);
    h.app.manufacture.sheet = true;
    let params = crate::manufacture_ui::post_params(&h.app, false);
    run_heavy(&mut h.app, "cam.post", params);
    assert_eq!(h.app.manufacture.busy(), Some("cam.post"));
    for _ in 0..3000 {
        h.frame();
        if h.app.manufacture.busy().is_none() {
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(h.app.manufacture.error.is_none(), "{:?}", h.app.manufacture.error);
    let written = h.app.manufacture.written.clone().unwrap();
    assert_eq!(written["passed"], true, "{written}");
    assert_eq!(written["outputs"]["gcode"], dir.file("block.tap"));
    assert_eq!(written["outputs"]["setup_sheet"], dir.file("block.setup.html"));
    assert!(std::path::Path::new(&dir.file("block.setup.html")).exists());
    assert_eq!(h.app.manufacture.stage, Stage::Program);
    let g = h.app.session.cam_job.as_ref().unwrap().generated.as_ref().unwrap();
    assert_eq!(g.paths.len(), 2, "the worker's results came back");
    h.frames(2);
}

#[test]
fn the_view_draws_every_path_the_selected_bright_or_only_the_selected() {
    use crate::manufacture_plan_layer::{path_look, segments};
    assert_eq!(path_look(None, ShowPaths::All, 3), Some(true));
    assert_eq!(path_look(Some(1), ShowPaths::All, 3), Some(false));
    assert_eq!(path_look(Some(3), ShowPaths::All, 3), Some(true));
    assert_eq!(path_look(Some(1), ShowPaths::Selected, 3), None);
    assert_eq!(path_look(Some(3), ShowPaths::Selected, 3), Some(true));
    assert_eq!(path_look(None, ShowPaths::Selected, 3), None);
    assert_eq!(path_look(Some(3), ShowPaths::None, 3), None);
    let p = OpPath { op: 0, points: vec![[0.0; 3], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]], rapid: vec![false, true, false], ..OpPath::default() };
    assert_eq!(segments(&p), vec![([0.0; 3], [1.0, 0.0, 0.0], true), ([1.0, 0.0, 0.0], [1.0, 1.0, 0.0], false)]);
}

#[test]
fn every_path_of_an_indexed_pattern_is_drawn_as_its_own_block() {
    // An indexed pattern plans one toolpath per angle, all of the same job operation.
    let dir = Dir::new("pattern");
    let mut h = harness(&dir, false);
    switch(&mut h.app, "manufacture").unwrap();
    let path = |x: f32| OpPath { op: 0, points: vec![[x, 0.0, 0.0], [x + 1.0, 0.0, 0.0]], rapid: vec![false, false], ..OpPath::default() };
    let stamp = h.app.session.model_stamp();
    let job = h.app.session.cam_job.as_mut().unwrap();
    let revision = job.revision;
    job.generated = Some(solidcraft_engine::cam_job_state::Generated {
        revision,
        model_revision: stamp,
        paths: vec![path(0.0), path(5.0), path(10.0), path(15.0)],
        ..Default::default()
    });
    let mut sc = crate::gpu::GpuScene::default();
    crate::manufacture_plan_layer::draw(&h.app, &mut sc);
    let mut keys: Vec<u64> = sc.blocks.iter().map(|b| b.key).collect();
    keys.sort_unstable();
    keys.dedup();
    assert_eq!(keys.len(), 5, "one block per angle's path, and work zero's triad");
    // A new Generate takes the last finding's mark away.
    h.app.manufacture.mark = Some([1.0, 2.0, 3.0]);
    crate::manufacture_ui::generate_now(&mut h.app);
    assert_eq!(h.app.manufacture.mark, None);
}
