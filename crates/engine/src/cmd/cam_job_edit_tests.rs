//! Tests of the open job's commands: open, edit, save and undo through `cam.job.*` and
//! `cam.op.*`, edits made outside SolidCraft, Generate's findings, going to one, and writing
//! the program; hostile parameters refused.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};

use serde_json::{Value, json};

use crate::{EngineError, Session};

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    s.execute(id, &p).unwrap_or_else(|e| panic!("{id} {p}: {e}"))
}

fn refused(s: &mut Session, id: &str, p: Value) -> String {
    match s.execute(id, &p) {
        Err(e @ EngineError::BadParams { .. }) => e.to_string(),
        other => panic!("{id} {p}: expected a refusal, got {other:?}"),
    }
}

/// A folder of its own under the temporary folder, removed afterwards.
struct Dir(PathBuf);

impl Dir {
    fn new(tag: &str) -> Dir {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let d =
            solidcraft_io::vfs::temp_dir().join(format!("solidcraft-camjob-{tag}-{}-{}", std::process::id(), NEXT.fetch_add(1, Ordering::Relaxed)));
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

/// A 40 × 30 × 20 block, its top at Z 20, with a 16 × 10 pocket 5 deep in it.
fn block() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 20, "corner": [-20, -15, 0], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 16, "width": 10, "height": 5, "corner": [-8, -5, 15], "operation": "cut"}));
    s
}

const POCKET: [[f64; 2]; 4] = [[-8.0, -5.0], [8.0, -5.0], [8.0, 5.0], [-8.0, 5.0]];

fn job() -> Value {
    json!({
        "name": "block",
        "machine": "3axis",
        "post": "mach3_4axis",
        "tools": [{"number": 4, "kind": "flat_endmill", "diameter": 4, "flute_length": 12, "spindle_rpm": 12000, "feed": 800, "plunge_feed": 200}],
        "operations": [
            {"name": "pocket", "type": "pocket", "tool": 4, "polygon": POCKET, "stepover": 1.5},
            {"name": "outline", "type": "contour", "tool": 4, "polygon": [[-20, -15], [20, -15], [20, 15], [-20, 15]], "heights": {"bottom": 10}}
        ]
    })
}

/// The job written by a script: one space a level, keys as the script put them.
fn write_script_job(path: &str, job: &Value) {
    let text = serde_json::to_string_pretty(job).unwrap().replace("\n  ", "\n ");
    std::fs::write(path, text).unwrap();
}

fn ops(s: &Session) -> Vec<Value> {
    s.cam_job.as_ref().unwrap().value["operations"].as_array().unwrap().clone()
}

#[test]
fn a_job_is_opened_edited_saved_and_read_back_the_same() {
    let dir = Dir::new("edit");
    let path = dir.file("block.cam.json");
    write_script_job(&path, &job());
    let mut s = block();
    let sum = run(&mut s, "cam.job.open", json!({"path": path}));
    assert_eq!((sum["name"].as_str(), sum["dirty"].as_bool(), sum["machine"].as_str()), (Some("block"), Some(false), Some("3axis")), "{sum}");
    assert_eq!(sum["operations"][1]["type"], "contour");
    assert_eq!(sum["operations"][1]["status"], "unplanned");
    run(&mut s, "cam.job.edit", json!({"set": {"tolerance": 0.02, "stock.box.min": [-21, -16, 0], "stock.box.max": [21, 16, 21]}}));
    run(&mut s, "cam.op.edit", json!({"op": "pocket", "set": {"heights.bottom": 16}, "rename": "boss pocket"}));
    let saved = run(&mut s, "cam.job.save", json!({}));
    assert_eq!(saved["written"], true, "{saved}");
    let text = std::fs::read_to_string(&path).unwrap();
    // Kept the file's one-space indent and the job's key order.
    assert!(text.starts_with("{\n \"name\": \"block\",\n \"machine\""), "{text}");
    assert!(
        text.find("\"tolerance\"").unwrap() < text.find("\"stock\"").unwrap() && text.find("\"stock\"").unwrap() < text.find("\"tools\"").unwrap()
    );
    let read: Value = serde_json::from_str(&text).unwrap();
    assert_eq!(read, s.cam_job.as_ref().unwrap().value);
    assert_eq!(read["operations"][0]["heights"], json!({"bottom": 16}), "only the key set was written");
    // Saved and unchanged: nothing more to write; opened again it is byte for byte the same file.
    assert_eq!(run(&mut s, "cam.job.save", json!({}))["written"], false);
    run(&mut s, "cam.job.open", json!({"path": path}));
    run(&mut s, "cam.job.save", json!({"overwrite": true}));
    assert_eq!(std::fs::read_to_string(&path).unwrap(), text);
    // Save as: the job is that file from then on.
    let other = dir.file("copy.cam.json");
    run(&mut s, "cam.job.save", json!({"path": other}));
    assert_eq!(s.cam_job.as_ref().unwrap().path.as_deref(), Some(other.as_str()));
    assert!(refused(&mut s, "cam.job.save", json!({"path": path})).contains("overwrite"));
}

#[test]
fn an_added_operation_writes_only_what_it_was_given() {
    let mut s = block();
    run(&mut s, "cam.job.new", json!({"name": "fresh"}));
    let r = run(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "tool": 4}}));
    assert_eq!((r["index"].as_u64(), r["name"].as_str()), (Some(0), Some("Clear Pocket")), "{r}");
    assert_eq!(ops(&s)[0], json!({"name": "Clear Pocket", "type": "pocket", "tool": 4}));
    // The second of a type is numbered; a tool left out is the job's best for it; a type that
    // cannot do without a key gets a starting value for it.
    let r = run(&mut s, "cam.op.add", json!({"op": {"type": "pocket"}, "at": 0}));
    assert_eq!(r["name"], "Clear Pocket 2");
    assert_eq!(ops(&s)[0], json!({"name": "Clear Pocket 2", "type": "pocket", "tool": {}}));
    run(&mut s, "cam.op.add", json!({"op": {"type": "face"}}));
    assert_eq!(ops(&s)[2], json!({"name": "Face Stock", "type": "face", "tool": {}, "bottom": 0}));
    assert!(refused(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "name": "Face Stock"}})).contains("already named"));
    assert!(refused(&mut s, "cam.op.add", json!({"op": {"tool": 4}})).contains("type"));
    assert!(refused(&mut s, "cam.op.add", json!({"op": {"type": "no_such_cut"}})).contains("nothing changed"));
    assert!(refused(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "stepovr": 1}})).contains("stepovr"));
    assert_eq!(ops(&s).len(), 3, "a refused edit changes nothing");
}

#[test]
fn turning_a_to_an_angle_wraps_and_unwraps_the_operation() {
    let mut s = block();
    run(&mut s, "cam.job.new", json!({}));
    run(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "name": "side", "tool": 4, "stepover": 1}}));
    run(&mut s, "cam.op.edit", json!({"op": "side", "a_deg": 90}));
    assert_eq!(ops(&s)[0], json!({"name": "side", "type": "indexed", "a_deg": 90.0, "operation": {"type": "pocket", "tool": 4, "stepover": 1}}));
    // Keys are the inner operation's; the wrapper stays.
    run(&mut s, "cam.op.edit", json!({"op": 0, "set": {"stepover": 1.5, "heights.bottom": -3}}));
    assert_eq!(ops(&s)[0]["operation"]["stepover"], 1.5);
    assert_eq!(ops(&s)[0]["operation"]["heights"]["bottom"], -3);
    assert_eq!(ops(&s)[0]["a_deg"], 90.0);
    // Several angles: an indexed pattern, its name inside; then an evenly spread count.
    run(&mut s, "cam.op.edit", json!({"op": "side", "a_deg": [0, 180]}));
    assert_eq!(ops(&s)[0]["type"], "indexed_pattern");
    assert_eq!(ops(&s)[0]["a_deg"], json!([0.0, 180.0]));
    assert_eq!(ops(&s)[0]["operation"]["name"], "side");
    run(&mut s, "cam.op.edit", json!({"op": "side", "a_deg": {"count": 4, "start_deg": 45}}));
    assert_eq!((ops(&s)[0]["count"].as_u64(), ops(&s)[0]["start_deg"].as_f64()), (Some(4), Some(45.0)));
    // Off stays on the outside; 0 unwraps to the plain operation.
    run(&mut s, "cam.op.enable", json!({"op": "side", "enabled": false}));
    assert_eq!(ops(&s)[0]["enabled"], false);
    run(&mut s, "cam.op.edit", json!({"op": "side", "a_deg": 0}));
    assert_eq!(ops(&s)[0], json!({"name": "side", "type": "pocket", "tool": 4, "stepover": 1.5, "heights": {"bottom": -3}, "enabled": false}));
    run(&mut s, "cam.op.enable", json!({"op": "side", "enabled": true}));
    assert!(ops(&s)[0].get("enabled").is_none(), "on is the default: the key goes");
    // Only a 3-axis operation turns; a refused edit says why and changes nothing.
    run(&mut s, "cam.op.add", json!({"op": {"type": "wrap", "name": "engrave"}}));
    assert!(refused(&mut s, "cam.op.edit", json!({"op": "engrave", "a_deg": 30})).contains("3-axis"));
    let before = ops(&s);
    let e = refused(&mut s, "cam.op.edit", json!({"op": "side", "set": {"stepover": "wide"}}));
    assert!(e.contains("`operations[0].stepover`: invalid type") && e.contains("nothing changed"), "{e}");
    assert_eq!(ops(&s), before);
    // Unset brings a default back.
    run(&mut s, "cam.op.edit", json!({"op": "side", "unset": ["stepover", "heights.bottom"]}));
    assert_eq!(ops(&s)[0], json!({"name": "side", "type": "pocket", "tool": 4, "heights": {}}));
}

#[test]
fn operations_move_duplicate_and_go_and_every_edit_undoes() {
    let dir = Dir::new("undo");
    let path = dir.file("block.cam.json");
    write_script_job(&path, &job());
    let mut s = block();
    run(&mut s, "cam.job.open", json!({"path": path}));
    let opened = s.cam_job.as_ref().unwrap().value.clone();
    let r = run(&mut s, "cam.op.duplicate", json!({"op": "pocket"}));
    assert_eq!((r["index"].as_u64(), r["name"].as_str()), (Some(1), Some("pocket copy")));
    run(&mut s, "cam.op.move", json!({"op": "outline", "to": 0}));
    let names: Vec<String> = ops(&s).iter().map(crate::cam_job_state::op_name).collect();
    assert_eq!(names, ["outline", "pocket", "pocket copy"]);
    run(&mut s, "cam.op.remove", json!({"op": 1}));
    run(&mut s, "cam.op.edit", json!({"op": "outline", "rename": "edge"}));
    run(&mut s, "cam.job.edit", json!({"unset": ["name"]}));
    let got = run(&mut s, "cam.job.get", json!({}));
    assert_eq!(got["undo"][0], "Set name");
    assert_eq!(got["undo"][1], "Rename 'outline' to 'edge'");
    assert_eq!(got["undo"].as_array().unwrap().len(), 5);
    for _ in 0..5 {
        run(&mut s, "cam.job.undo", json!({}));
    }
    assert_eq!(s.cam_job.as_ref().unwrap().value, opened, "five undos give the job as opened");
    assert_eq!(run(&mut s, "cam.job.undo", json!({}))["step"], Value::Null);
    assert!(!s.cam_job.as_ref().unwrap().dirty());
    assert_eq!(run(&mut s, "cam.job.redo", json!({}))["step"], "Duplicate 'pocket'");
    // The design's own history is not touched by any of it.
    assert!(s.execute("edit.undo", &json!({})).is_ok(), "the design's boxes are still its undo steps");
    assert_eq!(ops(&s).len(), 3, "a design undo leaves the job alone");
    // Selecting by a name two operations have, or a place that is not there, is refused.
    run(&mut s, "cam.op.edit", json!({"op": "pocket copy", "rename": "twin"}));
    s.cam_job.as_mut().unwrap().value["operations"][2]["name"] = json!("twin");
    assert!(refused(&mut s, "cam.op.remove", json!({"op": "twin"})).contains("give its index"));
    assert!(refused(&mut s, "cam.op.remove", json!({"op": 9})).contains("not in the job"));
    assert!(refused(&mut s, "cam.op.move", json!({"op": 0, "to": 3})).contains("`to`"));
    // A name set as a key is checked as a rename is: not another's, not empty, no control characters.
    let before = ops(&s);
    assert!(refused(&mut s, "cam.op.edit", json!({"op": 0, "set": {"name": "twin"}})).contains("already named"));
    for name in [json!(""), json!("a\nG0 Z-50"), json!("x".repeat(201))] {
        assert!(refused(&mut s, "cam.op.edit", json!({"op": 0, "set": {"name": name}})).contains("characters"), "{name}");
    }
    assert_eq!(ops(&s), before, "a refused name changes nothing");
    // Open and close clear the history; unsaved edits are not dropped without `discard`.
    assert!(refused(&mut s, "cam.job.close", json!({})).contains("discard"));
    assert!(refused(&mut s, "cam.job.open", json!({"path": path})).contains("discard"));
    run(&mut s, "cam.job.open", json!({"path": path, "discard": true}));
    assert!(s.cam_job.as_ref().unwrap().undo.is_empty());
    assert_eq!(run(&mut s, "cam.job.close", json!({}))["closed"], true);
    assert!(refused(&mut s, "cam.op.add", json!({"op": {"type": "pocket"}})).contains("cam.job.open"));
}

#[test]
fn edits_made_outside_are_seen_reloaded_and_undone() {
    let dir = Dir::new("outside");
    let path = dir.file("block.cam.json");
    write_script_job(&path, &job());
    let mut s = block();
    run(&mut s, "cam.job.open", json!({"path": path}));
    assert_eq!(run(&mut s, "cam.job.get", json!({}))["outside"], "none");
    // An agent changes the file: a clean job sees it and reloads with what changed, one step.
    let mut theirs = job();
    theirs["operations"][0]["stepover"] = json!(2);
    theirs["operations"].as_array_mut().unwrap().push(
        json!({"name": "spot", "type": "drill", "tool": 4, "heights": {"clearance": 25, "retract": 22, "top": 20, "bottom": 18}, "points": [[0, 0]]}),
    );
    write_script_job(&path, &theirs);
    assert_eq!(run(&mut s, "cam.job.get", json!({}))["outside"], "changed");
    let r = run(&mut s, "cam.job.reload", json!({}));
    let changes: Vec<&str> = r["changes"].as_array().unwrap().iter().filter_map(Value::as_str).collect();
    assert!(changes.contains(&"operation 1 'pocket': stepover 1.5 -> 2") && changes.contains(&"added operation 3 'spot'"), "{changes:?}");
    assert_eq!(run(&mut s, "cam.job.get", json!({}))["outside"], "none");
    assert_eq!(run(&mut s, "cam.job.undo", json!({}))["step"], "Reloaded: changed outside SolidCraft");
    assert_eq!(ops(&s).len(), 2);
    // With unsaved edits here: save and reload refuse until told which to keep.
    write_script_job(&path, &job());
    run(&mut s, "cam.op.edit", json!({"op": "pocket", "set": {"stepover": 1.2}}));
    let mut again = job();
    again["name"] = json!("theirs");
    write_script_job(&path, &again);
    assert!(refused(&mut s, "cam.job.save", json!({})).contains("changed outside"));
    assert!(refused(&mut s, "cam.job.reload", json!({})).contains("discard"));
    run(&mut s, "cam.job.save", json!({"overwrite": true}));
    let kept: Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!((kept["name"].as_str(), kept["operations"][0]["stepover"].as_f64()), (Some("block"), Some(1.2)));
    // Without file times (the web) the bytes are compared.
    s.cam_job.as_mut().unwrap().disk.as_mut().unwrap().stamp = None;
    assert_eq!(run(&mut s, "cam.job.get", json!({}))["outside"], "none", "same bytes");
    s.cam_job.as_mut().unwrap().disk.as_mut().unwrap().stamp = None;
    write_script_job(&path, &again);
    assert_eq!(run(&mut s, "cam.job.get", json!({}))["outside"], "changed");
    // Gone.
    std::fs::remove_file(&path).unwrap();
    assert_eq!(run(&mut s, "cam.job.get", json!({}))["outside"], "gone");
}

#[test]
fn a_file_that_is_not_a_job_is_refused_with_its_key_and_line() {
    let dir = Dir::new("bad");
    let path = dir.file("bad.cam.json");
    std::fs::write(&path, "{\n \"operations\": [\n  {\"name\": \"p\", \"type\": \"pocket\", \"tool\": 4, \"stepovr\": 2}\n ]\n}\n").unwrap();
    let mut s = block();
    let e = refused(&mut s, "cam.job.open", json!({"path": path}));
    assert!(e.contains("`operations[0].stepovr`: unknown field") && e.contains("line 4"), "{e}");
    std::fs::write(&path, "{ not json").unwrap();
    assert!(refused(&mut s, "cam.job.open", json!({"path": path})).contains("not valid JSON"));
    assert!(s.cam_job.is_none());
    assert!(refused(&mut s, "cam.job.open", json!({"path": dir.file("job.txt")})).contains(".json"));
    // A new job is not written over a file already there.
    std::fs::write(dir.file("taken.cam.json"), "{}").unwrap();
    assert!(refused(&mut s, "cam.job.new", json!({"path": dir.file("taken.cam.json")})).contains("cam.job.open"));
}

#[test]
fn generate_finds_each_problem_with_its_place_and_post_writes_from_a_fresh_plan() {
    let dir = Dir::new("generate");
    let path = dir.file("block.cam.json");
    let mut j = job();
    j["operations"].as_array_mut().unwrap().push(
        json!({"name": "low clearance", "type": "pocket", "tool": 4, "polygon": POCKET, "heights": {"clearance": 19, "retract": 19, "top": 18, "bottom": 15}}),
    );
    write_script_job(&path, &j);
    let mut s = block();
    run(&mut s, "cam.job.open", json!({"path": path}));
    let g = run(&mut s, "cam.generate", json!({}));
    assert_eq!(g["passed"], false, "{g}");
    let errors: Vec<&Value> = g["findings"].as_array().unwrap().iter().filter(|f| f["severity"] == "error").collect();
    assert_eq!(errors.len(), 1, "{g}");
    let f = errors[0];
    assert_eq!(
        (f["check"].as_str(), f["op"]["index"].as_u64(), f["op"]["name"].as_str()),
        (Some("clearance"), Some(2), Some("low clearance")),
        "{f}"
    );
    assert_eq!(f["at"][2].as_f64(), Some(20.0), "the part's top: {f}");
    assert!(f["fix"].as_str().unwrap().contains("at least 22"), "{f}");
    let summary = crate::cmd::cam::job_edit::summary(&s);
    let status: Vec<_> = summary.operations.iter().map(|o| o.status).collect();
    use crate::schema::cam_job_edit::OpStatus;
    assert_eq!(status[2], OpStatus::Error);
    // The job was refused, so nothing of it was planned: the others are not "ok" (they were).
    assert_eq!(status[..2], [OpStatus::Unplanned; 2]);
    // Going to it: its operation and place, for the window to follow.
    let index = f["index"].clone();
    let go = run(&mut s, "cam.finding.go", json!({"index": index}));
    assert_eq!((go["op"]["index"].as_u64(), go["at_design"][2].as_f64()), (Some(2), Some(20.0)), "{go}");
    assert_eq!(s.cam_job.as_ref().unwrap().go.seq, 1);
    assert!(refused(&mut s, "cam.finding.go", json!({"index": 99})).contains("not there"));
    // The fix applied: it passes, its toolpaths drawn, every operation ok.
    run(&mut s, "cam.op.edit", json!({"op": "low clearance", "set": {"heights.clearance": 25, "heights.retract": 22}}));
    assert_eq!(crate::cmd::cam::job_edit::summary(&s).operations[0].status, OpStatus::Stale);
    let g = run(&mut s, "cam.generate", json!({}));
    assert_eq!(g["passed"], true, "{g}");
    let open = s.cam_job.as_ref().unwrap();
    let generated = open.generated.as_ref().unwrap();
    assert_eq!(generated.paths.len(), 3);
    assert!(generated.paths.iter().all(|p| p.points.len() > 4 && p.points.len() == p.rapid.len()));
    // The pocket's tip stays inside the pocket's outline grown by the tool's radius, in design
    // coordinates (work zero is the design's origin).
    assert!(generated.paths[0].points.iter().all(|p| p[0].abs() <= 8.0 + 1e-3 && p[1].abs() <= 5.0 + 1e-3), "pocket path");
    let status: Vec<_> = crate::cmd::cam::job_edit::summary(&s).operations.iter().map(|o| o.status).collect();
    assert_eq!(status, [OpStatus::Ok; 3]);
    // A program is written from the job and design as they are: after a design edit it is
    // planned again (the operations go stale first), next to the job file by default.
    run(&mut s, "solid.box", json!({"length": 4, "width": 4, "height": 2, "corner": [14, 10, 20], "operation": "join"}));
    assert_eq!(crate::cmd::cam::job_edit::summary(&s).operations[0].status, OpStatus::Stale);
    let sheet = dir.file("block-sheet.html");
    let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
        s.execute("cam.post", &json!({"setup_sheet": sheet}))
    });
    let r = r.unwrap_or_else(|e| panic!("cam.post: {e}"));
    assert_eq!(r["passed"], true, "{r}");
    let gcode = r["outputs"]["gcode"].as_str().unwrap();
    assert_eq!(PathBuf::from(gcode), dir.0.join("block.tap"));
    assert!(std::fs::read_to_string(gcode).unwrap().contains("low clearance"));
    assert!(std::fs::read_to_string(&sheet).unwrap().contains("<html"), "the setup sheet");
    let status: Vec<_> = crate::cmd::cam::job_edit::summary(&s).operations.iter().map(|o| o.status).collect();
    assert_eq!(status, [OpStatus::Ok; 3], "planned again for the design as it is");
    assert_eq!(s.cam_job.as_ref().unwrap().generated.as_ref().unwrap().written.len(), 2);
}

#[test]
fn an_operation_turned_off_is_neither_planned_nor_written() {
    let dir = Dir::new("off");
    let path = dir.file("block.cam.json");
    write_script_job(&path, &job());
    let mut s = block();
    run(&mut s, "cam.job.open", json!({"path": path}));
    run(&mut s, "cam.op.enable", json!({"op": "outline", "enabled": false}));
    let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || run(&mut s, "cam.post", json!({})));
    assert_eq!(r["passed"], true, "{r}");
    let gcode = r["outputs"]["gcode"].as_str().unwrap().to_string();
    assert!(gcode.ends_with("block.tap"), "beside the job, the post's extension: {gcode}");
    let text = std::fs::read_to_string(&gcode).unwrap();
    assert!(text.contains("pocket") && !text.contains("outline"), "{text}");
    let summary = crate::cmd::cam::job_edit::summary(&s);
    assert_eq!(summary.operations[1].status, crate::schema::cam_job_edit::OpStatus::Off);
    // Every operation off: refused with that reason.
    run(&mut s, "cam.op.enable", json!({"op": "pocket", "enabled": false}));
    let g = run(&mut s, "cam.generate", json!({}));
    assert_eq!(g["passed"], false);
    assert!(g["findings"][0]["message"].as_str().unwrap().contains("turned off"), "{g}");
}

/// An operation turned off counts for nothing: its `gouge_check: false` does not make a gouge
/// the simulation finds in the program a mere warning, and `enabled: false` on the operation
/// inside an indexed one turns all of it off (it used to be planned and written regardless).
#[test]
fn an_operation_turned_off_changes_no_check_and_is_off_inside_a_wrapper_too() {
    let dir = Dir::new("off-checks");
    let path = dir.file("block.cam.json");
    write_script_job(&path, &job());
    let mut s = block();
    run(&mut s, "cam.job.open", json!({"path": path}));
    // A plunge 5 mm into the pocket's floor (Z 15).
    let bad = dir.file("bad.tap");
    std::fs::write(&bad, "G21\nT4 M6\nG0 X0. Y0. Z30.\nG1 Z10. F200\nG0 Z30.\n").unwrap();
    let simulate = |s: &mut Session| run(s, "cam.simulate", json!({"path": bad, "resolution": 0.5}));
    let c = simulate(&mut s);
    assert_eq!(c["passed"], false, "{c}");
    run(&mut s, "cam.op.add", json!({"op": {"name": "engrave", "type": "contour", "tool": 4, "polygon": POCKET, "gouge_check": false}}));
    run(&mut s, "cam.op.enable", json!({"op": "engrave", "enabled": false}));
    let c = simulate(&mut s);
    assert_eq!(c["passed"], false, "an operation turned off does not excuse a gouge: {c}");
    assert!(c["failures"].as_array().is_some_and(|f| f.iter().any(|m| m.as_str().is_some_and(|m| m.contains("into the part")))), "{c}");
    // Off inside an indexed operation (as a script may write it): the whole of it is off.
    let mut j = job();
    j["operations"][1] = json!({"type": "indexed", "a_deg": 90, "operation": {
        "name": "side", "type": "pocket", "tool": 4, "polygon": POCKET, "enabled": false
    }});
    write_script_job(&path, &j);
    run(&mut s, "cam.job.open", json!({"path": path, "discard": true}));
    let summary = crate::cmd::cam::job_edit::summary(&s);
    assert!(!summary.operations[1].enabled, "{:?}", summary.operations[1]);
    assert_eq!(summary.operations[1].status, crate::schema::cam_job_edit::OpStatus::Off);
    let r = run(&mut s, "cam.job", json!({"dry_run": true, "compact": true}));
    let planned = r["operations"].as_array().unwrap();
    assert_eq!(r["passed"], true, "{r}");
    assert!(planned.len() == 1 && planned[0].as_str().is_some_and(|o| o.contains("`pocket`")), "{r}");
}

#[test]
fn the_open_job_is_what_the_other_cam_commands_read() {
    let dir = Dir::new("default");
    let path = dir.file("block.cam.json");
    write_script_job(&path, &job());
    let mut s = block();
    assert!(refused(&mut s, "cam.job", json!({"dry_run": true})).contains("open a job"));
    run(&mut s, "cam.job.open", json!({"path": path}));
    let r = run(&mut s, "cam.job", json!({"dry_run": true, "compact": true}));
    assert_eq!(r["operations"].as_array().map(Vec::len), Some(2), "{r}");
    // Unsaved edits are what is planned.
    run(&mut s, "cam.op.remove", json!({"op": "outline"}));
    let r = run(&mut s, "cam.job", json!({"dry_run": true, "compact": true}));
    assert_eq!(r["operations"].as_array().map(Vec::len), Some(1), "{r}");
    let v = run(&mut s, "cam.view.load", json!({"resolution": 0.5}));
    assert_eq!(v["operations"].as_array().map(Vec::len), Some(1), "{v}");
}

#[test]
fn a_library_folder_lists_its_profiles_posts_and_tools() {
    let dir = Dir::new("library");
    for sub in ["profiles", "posts", "tools", "machines", "jobs/a"] {
        std::fs::create_dir_all(dir.0.join(sub)).unwrap();
    }
    std::fs::write(dir.0.join("profiles/mill.profile.json"), r#"{"name": "My mill", "description": "three axes", "machine": "3axis"}"#).unwrap();
    std::fs::write(dir.0.join("posts/mine.post.json"), r#"{"name": "my post"}"#).unwrap();
    std::fs::write(
        dir.0.join("tools/lib.json"),
        r#"[{"number": 4, "kind": "flat_endmill", "diameter": 4, "flute_length": 12}, {"number": 9, "kind": "laser_beam", "diameter": 1, "flute_length": 1}]"#,
    )
    .unwrap();
    std::fs::write(dir.0.join("machines/vise.workholding.json"), r#"{"holding": "vise"}"#).unwrap();
    let job_file = dir.0.join("jobs/a/x.cam.json").to_string_lossy().to_string();
    write_script_job(&job_file, &job());
    let mut s = block();
    run(&mut s, "cam.job.open", json!({"path": job_file}));
    let r = run(&mut s, "cam.library.list", json!({}));
    assert_eq!(r["folder"].as_str().map(PathBuf::from), Some(dir.0.clone()), "found above the job's folder: {r}");
    assert_eq!(r["profiles"][0]["name"], "My mill");
    assert_eq!(r["profiles"][0]["machine"], "3axis");
    assert!(r["posts"].as_array().unwrap().iter().any(|p| p["name"] == "my post"), "{r}");
    assert!(r["posts"].as_array().unwrap().iter().any(|p| p["name"] == "mach3_4axis" && p["path"] == ""), "{r}");
    assert_eq!((r["tool_libraries"][0]["tools"].as_u64(), r["tool_libraries"][0]["left_out"].as_array().map(Vec::len)), (Some(1), Some(1)), "{r}");
    assert_eq!(r["workholdings"][0]["name"], "vise");
    assert!(refused(&mut s, "cam.library.list", json!({"folder": ""})).contains("folder"));
}

#[test]
fn hostile_job_edits_are_refused_and_change_nothing() {
    let mut s = block();
    for id in
        ["cam.job.save", "cam.job.reload", "cam.job.edit", "cam.job.undo", "cam.op.add", "cam.op.remove", "cam.generate", "cam.post", "cam.findings"]
    {
        let p = if id == "cam.op.add" {
            json!({"op": {"type": "pocket"}})
        } else if id == "cam.op.remove" {
            json!({"op": 0})
        } else {
            json!({})
        };
        assert!(refused(&mut s, id, p).contains("cam.job.open"), "{id}");
    }
    run(&mut s, "cam.job.new", json!({}));
    let before = s.cam_job.as_ref().unwrap().value.clone();
    // Not a number where one goes, not finite, too deep, too many, a path outside the folders.
    for set in [
        json!({"origin": [0, "x", 0]}),
        json!({"tolerance": "fine"}),
        json!({"operations": []}),
        json!({"stock.box": 5}),
        json!({"no_such_key": 1}),
        json!({"": 1}),
        json!({"a..b": 1}),
    ] {
        assert!(s.execute("cam.job.edit", &json!({"set": set})).is_err(), "{set}");
    }
    let mut deep = json!(1);
    for _ in 0..40 {
        deep = json!({"x": deep});
    }
    assert!(refused(&mut s, "cam.job.edit", json!({"set": {"machine": deep}})).contains("nested"));
    assert!(s.execute("cam.job.edit", &json!({"set": {"origin": [f64::NAN, 0, 0]}})).is_err());
    assert_eq!(s.cam_job.as_ref().unwrap().value, before);
    for _ in 0..crate::cmd::cam::MAX_OPERATIONS {
        s.cam_job.as_mut().unwrap().value["operations"].as_array_mut().unwrap().push(json!({"name": "x", "type": "pocket", "tool": 1}));
    }
    assert!(refused(&mut s, "cam.op.add", json!({"op": {"type": "pocket"}})).contains("at most"));
    assert!(refused(&mut s, "cam.op.duplicate", json!({"op": 0})).contains("at most"));
    assert!(refused(&mut s, "cam.op.add", json!({"op": {"type": "pocket"}, "at": 9999})).contains("at most"));
    let scope = solidcraft_io::vfs::PathScope::new([Dir::new("scope").0.clone()]);
    let outside = Dir::new("elsewhere").file("x.cam.json");
    solidcraft_io::vfs::scoped(Some(scope), || {
        assert!(s.execute("cam.job.save", &json!({"path": outside})).is_err());
        assert!(s.execute("cam.job.open", &json!({"path": outside})).is_err());
    });
    assert!(!std::path::Path::new(&outside).exists());
    assert!(refused(&mut s, "view.manufacture_stage", json!({"stage": "toolpaths"})).contains("setup"));
    assert_eq!(run(&mut s, "view.manufacture_stage", json!({"stage": "check"}))["stage"], "check");
}
