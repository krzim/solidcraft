//! Tests of the CAM commands: a 3D finish, a 3+1 pocket and a 4th-axis finish planned on
//! designs built with commands, written only inside an agent's folders, read back and checked
//! for gouges; unknown keys and hostile parameters refused.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};

use serde_json::{Value, json};
use solidcraft_cam::indexed::frame_for_index;
use solidcraft_cam::kinematics::Machine;

use crate::{EngineError, Session};

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    s.execute(id, &p).unwrap_or_else(|e| panic!("{id} {p}: {e}"))
}

/// A folder of its own under the temporary folder, removed afterwards.
struct Dir(PathBuf);

impl Dir {
    fn new(tag: &str) -> Dir {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let d = solidcraft_io::vfs::temp_dir().join(format!("solidcraft-cam-{tag}-{}-{}", std::process::id(), NEXT.fetch_add(1, Ordering::Relaxed)));
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

fn ball(number: u32, diameter: f64) -> Value {
    json!({"number": number, "kind": "ball_endmill", "diameter": diameter, "flute_length": 12, "spindle_rpm": 12000, "feed": 900, "plunge_feed": 250})
}

fn flat(number: u32, diameter: f64) -> Value {
    json!({"number": number, "kind": "flat_endmill", "diameter": diameter, "flute_length": 15, "spindle_rpm": 12000, "feed": 800, "plunge_feed": 200})
}

/// A 40 mm square block, its top at Z 0, with a dome of radius 12 rising 7 mm out of it.
fn dome() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 40, "height": 20, "corner": [-20, -20, -20], "body_name": "Block"}));
    run(&mut s, "solid.sphere", json!({"radius": 12, "center": [0, 0, -5], "operation": "join"}));
    s
}

fn dome_job() -> Value {
    json!({
        "name": "dome",
        "machine": "3axis",
        "post": "mach3_4axis",
        "tools": [ball(2, 6.0)],
        "operations": [{
            "name": "finish", "type": "parallel", "tool": 2,
            "heights": {"clearance": 30, "retract": 15, "top": 7, "bottom": -1},
            "stepover": 1.5, "boundary": {"rect": [[-14, -14], [14, 14]]}
        }]
    })
}

/// A block 40 × 30 × 30 about the X axis with a 12 × 12 pocket 5 mm deep in its +Y face.
fn pocketed_block() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 30, "corner": [-20, -15, -15], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 12, "width": 10, "height": 12, "corner": [-6, 10, -6], "operation": "cut"}));
    s
}

fn side_pocket_job() -> Value {
    json!({
        "machine": "4axis_a",
        "post": "mach3_4axis",
        "tools": [flat(1, 4.0)],
        "operations": [{
            "type": "indexed", "a_deg": 90,
            "operation": {
                "name": "side pocket", "type": "pocket", "tool": 1,
                "polygon": [[-6, -6], [6, -6], [6, 6], [-6, 6]],
                "heights": {"clearance": 40, "retract": 20, "top": 15, "bottom": 10},
                "stepdown": 2.5, "stepover": 1.5
            }
        }]
    })
}

/// A cylinder of radius 10 along X from -20 to 20.
fn bar() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.cylinder", json!({"radius": 10, "height": 40, "axis": "X", "base": [-20, 0, 0], "body_name": "Bar"}));
    s
}

fn barrel_job() -> Value {
    json!({
        "machine": "4axis_a",
        "post": "mach3_4axis",
        "tools": [ball(3, 4.0)],
        "operations": [{
            "name": "barrel", "type": "rotary_finish", "tool": 3,
            "pattern": {"type": "lines", "angle_step_deg": 15},
            "clearance": 25, "retract": 15, "range": [-15, 15]
        }]
    })
}

fn assert_clean(r: &Value) {
    assert_eq!(r["passed"], true, "{r}");
    assert_eq!(r["readback"]["pass"], true, "{r}");
    assert_eq!(r["gouge"]["pass"], true, "{r}");
    assert_eq!(r["gouge"]["worst"].as_f64(), Some(0.0), "{r}");
    assert!(r["failures"].as_array().is_some_and(Vec::is_empty), "{r}");
}

#[test]
fn a_parallel_finish_is_written_only_inside_the_agents_folders() {
    let mut s = dome();
    let dir = Dir::new("dome");
    let root = dir.0.join("allowed");
    std::fs::create_dir_all(&root).unwrap();
    let inside = root.join("dome.tap").to_string_lossy().into_owned();
    let page = root.join("dome.html").to_string_lossy().into_owned();
    let outside = dir.file("dome.tap");
    let scope = Some(solidcraft_io::vfs::PathScope::new([root.clone()]));
    solidcraft_io::vfs::scoped(scope, || {
        let r = run(&mut s, "cam.job", json!({"job": dome_job(), "output": inside, "preview": page}));
        assert_clean(&r);
        assert_eq!(r["outputs"]["gcode"], inside.as_str(), "{r}");
        let op = &r["operations"][0];
        assert_eq!(op["kind"], "parallel", "{r}");
        assert!(op["moves"].as_u64().is_some_and(|m| m > 100) && op["cut_length"].as_f64().is_some_and(|l| l > 300.0), "{r}");
        assert_eq!(op["max_tilt_deg"].as_f64(), Some(0.0), "{r}");
        // Outside the folders nothing is computed or written, the preview included.
        let e = s.execute("cam.job", &json!({"job": dome_job(), "output": outside, "preview": dir.file("dome.html")})).unwrap_err();
        assert!(e.to_string().contains("outside the folders this agent may use"), "{e}");
        let e = s.execute("cam.job", &json!({"job": dome_job(), "output": dir.file("in-a-missing-folder/x.tap")})).unwrap_err();
        assert!(!e.to_string().is_empty());
    });
    assert!(!PathBuf::from(&outside).exists() && !dir.0.join("dome.html").exists());
    let text = std::fs::read_to_string(&inside).unwrap();
    // The dome's top is 7 mm up; the ball's tip reaches it, never below the floor.
    assert!(text.starts_with("(dome)") || text.contains("G90 G91.1 G94 G17"), "{text}");
    assert!(!text.contains(" A"), "a 3-axis program writes no A: {text}");
    let html = std::fs::read_to_string(&page).unwrap();
    assert!(html.contains("\"scenes\"") && !html.contains("/*DATA*/null") && html.contains("finish"), "the page holds the scenes");
    // The page loads nothing from the network.
    assert!(!html.contains("http://") && !html.contains("https://") && !html.contains("<script src"), "self-contained");
    // What was written reads back as the reply says.
    let c = run(&mut s, "cam.check", json!({"path": inside, "rapid_rate": 3000}));
    assert!(c["errors"].as_array().is_some_and(Vec::is_empty), "{c}");
    assert!(c["feed_length"].as_f64().is_some_and(|l| l > 300.0) && c["rapid_minutes"].as_f64().is_some(), "{c}");
    assert_eq!(c["g93_blocks"], 0, "{c}");
    let top = c["bounds"][1][2].as_f64().unwrap();
    assert!((14.999..=30.001).contains(&top), "{c}");
}

/// The frame A = 90° brings under the tool has its Z along work +Y, as docs/cam-jobs.md says.
#[test]
fn a_pocket_indexed_at_90_degrees_cuts_the_side_and_keeps_its_arcs() {
    let frame = frame_for_index(&Machine::four_axis_a(), &[90f64.to_radians()]).unwrap();
    assert!((frame.z.y - 1.0).abs() < 1e-9, "{frame:?}");
    let mut s = pocketed_block();
    let r = run(&mut s, "cam.job", json!({"job": side_pocket_job(), "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["dry_run"], true);
    assert!(r["outputs"]["gcode"].is_null(), "{r}");
    let op = &r["operations"][0];
    assert_eq!(op["kind"], "indexed pocket", "{r}");
    assert_eq!(op["a_deg"].as_f64(), Some(90.0), "{r}");
    assert!((op["max_tilt_deg"].as_f64().unwrap() - 90.0).abs() < 1e-6, "the tool comes in along work +Y: {r}");
    let head: Vec<&str> = r["program_head"].as_array().unwrap().iter().filter_map(Value::as_str).collect();
    assert!(head.iter().any(|l| l.contains("A90")), "{head:?}");
    // The same pocket on the solid face (no pocket in the model) cuts into the part: refused,
    // and with dry_run reported instead.
    let mut solid = Session::default();
    run(&mut solid, "solid.box", json!({"length": 40, "width": 30, "height": 30, "corner": [-20, -15, -15]}));
    let dir = Dir::new("pocket");
    let out = dir.file("side.tap");
    let e = solid.execute("cam.job", &json!({"job": side_pocket_job(), "output": out})).unwrap_err();
    assert!(matches!(e, EngineError::BadParams { .. }) && e.to_string().contains("gouge"), "{e}");
    assert!(!PathBuf::from(&out).exists(), "nothing was written");
    let r = run(&mut solid, "cam.job", json!({"job": side_pocket_job(), "dry_run": true}));
    assert_eq!(r["passed"], false, "{r}");
    assert!(r["gouge"]["worst"].as_f64().is_some_and(|w| w > 4.0), "the pocket is 5 mm into the face: {r}");
    // `allow_unsafe` writes it anyway.
    let r = run(&mut solid, "cam.job", json!({"job": side_pocket_job(), "output": out, "allow_unsafe": true}));
    assert_eq!(r["passed"], false);
    assert!(PathBuf::from(&out).exists());
}

#[test]
fn a_rotary_finish_on_a_bar_reads_back_and_turns_a_full_turn() {
    let mut s = bar();
    let dir = Dir::new("bar");
    let out = dir.file("bar.tap");
    let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
        run(&mut s, "cam.job", json!({"job": barrel_job(), "output": out}))
    });
    assert_clean(&r);
    let op = &r["operations"][0];
    assert_eq!(op["kind"], "rotary_finish", "{r}");
    assert!(op["max_tilt_deg"].as_f64().is_some_and(|t| t > 170.0), "the tool goes all round: {r}");
    let c = run(&mut s, "cam.check", json!({"path": out}));
    assert!(c["errors"].as_array().is_some_and(Vec::is_empty), "{c}");
    assert!(c["g93_blocks"].as_u64().is_some_and(|n| n > 0), "cuts that turn A are in inverse time: {c}");
    let a = &c["rotary_ranges"][0];
    assert_eq!(a["letter"], "A", "{c}");
    assert!(a["max_deg"].as_f64().unwrap() - a["min_deg"].as_f64().unwrap() >= 330.0, "{c}");
    // The tip stays on the bar: never inside radius 10 less the tolerance, in machine Z.
    let low = c["bounds"][0][2].as_f64().unwrap();
    assert!(low > 9.9, "{c}");
}

#[test]
fn a_job_file_reads_its_post_and_tools_beside_it() {
    let mut s = bar();
    let dir = Dir::new("files");
    let post = run(&mut s, "cam.posts", json!({"name": "mach3"}));
    assert_eq!(post["name"], "mach3_4axis");
    let mut post = post["post"].clone();
    post["base"]["header"] = json!(["(my mill)", "G90 G91.1 G94 G17 {units} G40 G49 G80"]);
    // The program-layout switches are known keys too, not refused as misspellings.
    for (k, v) in [
        ("upper_case_comments", json!(true)),
        ("comment_chars", json!(" .,=_-")),
        ("tool_list", json!(true)),
        ("operation_name_only", json!(true)),
        ("blank_lines", json!(true)),
        ("operation_start", json!(["G17 G90 G94", "{work_offset}"])),
        ("safe_retract", json!({"lines": ["G28 G91 Z0.", "G90"], "at_start": true, "at_tool_change": true, "at_end": true})),
        ("tool_length_on_first_z", json!(true)),
        ("modal_motion", json!(true)),
    ] {
        post["base"][k] = v;
    }
    post["safe_rotate_first"] = json!(true);
    post["inverse_time_decimals"] = json!(4);
    post["rewind"] = json!({"distance": 25.0, "retract_feed": 500.0, "plunge_feed": 250.0});
    std::fs::write(dir.0.join("my.post.json"), post.to_string()).unwrap();
    std::fs::write(dir.0.join("tools.json"), json!([ball(3, 4.0)]).to_string()).unwrap();
    let mut job = barrel_job();
    let o = job.as_object_mut().unwrap();
    o.remove("post");
    o.remove("tools");
    o.insert("post_path".into(), json!("my.post.json"));
    o.insert("tool_library_path".into(), json!("tools.json"));
    let job_path = dir.file("bar.cam.json");
    std::fs::write(&job_path, job.to_string()).unwrap();
    let r = run(&mut s, "cam.job", json!({"job_path": job_path, "dry_run": true}));
    assert_clean(&r);
    assert!(r["program_head"].as_array().unwrap().iter().any(|l| l == "(my mill)"), "{r}");
    let list = run(&mut s, "cam.posts", json!({}));
    let names: Vec<&str> = list["posts"].as_array().unwrap().iter().filter_map(|p| p["name"].as_str()).collect();
    assert_eq!(names, ["mach3_4axis", "linuxcnc_4axis", "grblhal_4axis", "rs274_5axis", "rs274_tcp"]);
    // A misspelt key in the post file is refused, not ignored.
    post["base"]["decimls"] = json!(3);
    std::fs::write(dir.0.join("my.post.json"), post.to_string()).unwrap();
    let e = s.execute("cam.job", &json!({"job_path": job_path, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("decimls"), "{e}");
}

#[test]
fn unknown_keys_are_refused_at_every_level() {
    let mut s = dome();
    let with = |f: &dyn Fn(&mut Value)| {
        let mut job = dome_job();
        f(&mut job);
        json!({"job": job, "dry_run": true})
    };
    let cases = [
        (json!({"job": dome_job(), "dry_rn": true}), "dry_rn"),
        (with(&|j| j["tolerence"] = json!(0.01)), "tolerence"),
        (with(&|j| j["operations"][0]["stepovr"] = json!(1)), "stepovr"),
        (with(&|j| j["operations"][0]["heights"]["clear"] = json!(1)), "clear"),
        (with(&|j| j["operations"][0]["boundary"]["rectangle"] = json!(1)), "rectangle"),
        (with(&|j| j["tools"][0]["stickot"] = json!(20)), "stickot"),
        (with(&|j| j["tools"][0]["holder"] = json!([{"diameter": 20, "lenght": 5}])), "lenght"),
        (with(&|j| j["operations"][0]["type"] = json!("paralel")), "paralel"),
        (with(&|j| j["machine"] = json!({"name": "m", "rotaries": [], "linear_limit": null})), "linear_limit"),
        (with(&|j| j["machine"] = json!("5axis")), "5axis"),
        (with(&|j| j["post"] = json!("fanuc")), "fanuc"),
    ];
    for (p, word) in cases {
        let e = s.execute("cam.job", &p).unwrap_err();
        assert!(matches!(e, EngineError::BadParams { .. }) && e.to_string().contains(word), "{p}: {e}");
    }
    let mut pocket = side_pocket_job();
    pocket["operations"][0]["operation"]["tol"] = json!(0.1);
    let e = pocketed_block().execute("cam.job", &json!({"job": pocket, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("tol"), "{e}");
    let e = s.execute("cam.posts", &json!({"nme": "mach3"})).unwrap_err();
    assert!(e.to_string().contains("nme"), "{e}");
    let e = s.execute("cam.check", &json!({"path": "x.nc", "rapid": 1})).unwrap_err();
    assert!(e.to_string().contains("rapid"), "{e}");
}

#[test]
fn hostile_jobs_are_refused_without_panics() {
    let mut s = dome();
    let dir = Dir::new("hostile");
    let job = |f: &dyn Fn(&mut Value)| {
        let mut j = dome_job();
        f(&mut j);
        json!({"job": j, "dry_run": true})
    };
    let many: Vec<Value> = (0..201)
        .map(|i| json!({"name": format!("o{i}"), "type": "parallel", "tool": 2, "heights": {"clearance": 30, "retract": 15, "top": 7, "bottom": -1}}))
        .collect();
    let cases = vec![
        json!({}),
        Value::Null,
        json!({"dry_run": true}),
        json!({"job": 5, "dry_run": true}),
        json!({"job": {}, "dry_run": true}),
        json!({"job": dome_job()}),
        json!({"job": dome_job(), "job_path": "x.json", "dry_run": true}),
        json!({"job_path": dir.file("missing.json"), "dry_run": true}),
        json!({"job": dome_job(), "output": ""}),
        json!({"job": dome_job(), "output": dir.file("a.tap"), "preview": dir.file("a.tap")}),
        job(&|j| j["operations"] = json!([])),
        job(&|j| j["operations"] = json!(many)),
        job(&|j| j["tolerance"] = json!(-1)),
        job(&|j| j["tolerance"] = json!(1e308)),
        job(&|j| j["origin"] = json!([1e308, 0, 0])),
        job(&|j| j["work_offset"] = json!(9)),
        job(&|j| j["work_offset"] = json!(-1)),
        job(&|j| j["name"] = json!("a\nb")),
        job(&|j| j["tools"] = json!([])),
        job(&|j| j["tools"][0]["diameter"] = json!(-6)),
        job(&|j| j["tools"][0]["diameter"] = json!(1e308)),
        job(&|j| j["tools"] = json!([ball(2, 6.0), ball(2, 4.0)])),
        job(&|j| j["tools"][0]["kind"] = json!("drill")),
        job(&|j| j["bodies"] = json!(["Nope"])),
        job(&|j| j["bodies"] = json!([])),
        job(&|j| j["fixtures"] = json!(["Block"])),
        job(&|j| j["a_limits_deg"] = json!([0, 90])),
        job(&|j| j["machine"] = json!({"name": "m", "rotaries": [{"letter": "A", "direction": [0, 0, 0], "center": [0, 0, 0], "carrier": "table"}]})),
        job(&|j| j["machine"] = json!(42)),
        job(&|j| j["operations"][0]["tool"] = json!(7)),
        job(&|j| j["operations"][0]["stepover"] = json!(0)),
        job(&|j| j["operations"][0]["stepover"] = json!(1e308)),
        job(&|j| j["operations"][0]["angle_deg"] = json!(1e308)),
        job(&|j| j["operations"][0]["heights"] = json!({"clearance": -30, "retract": 15, "top": 7, "bottom": -1})),
        job(&|j| j["operations"][0]["heights"]["retract"] = json!(1)),
        job(&|j| j["operations"][0]["boundary"] = json!({"rect": [[5, 5], [-5, -5]]})),
        job(&|j| j["operations"][0]["boundary"] = json!({"polygon": [[0, 0], [1, 1]]})),
        job(&|j| j["operations"][0]["feeds"] = json!({"cut": 0, "plunge": 100})),
        job(&|j| j["operations"][0]["spindle_rpm"] = json!(-5)),
        job(&|j| j["operations"][0]["step"] = json!(1e-12)),
        job(&|j| j["operations"][0]["type"] = json!("rotary_finish")),
        job(
            &|j| j["operations"][0] = json!({"name": "r", "type": "rotary_finish", "tool": 2, "pattern": {"type": "lines", "angle_step_deg": 10}, "clearance": 30, "retract": 20}),
        ),
        job(
            &|j| j["operations"][0] = json!({"type": "indexed", "a_deg": 90, "operation": {"name": "p", "type": "parallel", "tool": 2, "heights": {"clearance": 30, "retract": 15, "top": 7, "bottom": -1}}}),
        ),
        job(&|j| j["operations"][0] = json!({"name": "w", "type": "wrap", "tool": 2, "radius": 10, "points": [[0, 0, 0]]})),
        job(
            &|j| j["operations"][0] = json!({"name": "c", "type": "contour", "tool": 2, "sketch": "Nope", "heights": {"clearance": 30, "retract": 15, "top": 0, "bottom": -1}}),
        ),
        job(&|j| {
            j["operations"][0] =
                json!({"name": "c", "type": "contour", "tool": 2, "heights": {"clearance": 30, "retract": 15, "top": 0, "bottom": -1}})
        }),
        job(
            &|j| j["operations"][0] = json!({"name": "c", "type": "pocket", "tool": 2, "polygon": [[0, 0], [1e308, 0], [0, 1]], "heights": {"clearance": 30, "retract": 15, "top": 0, "bottom": -1}}),
        ),
        json!({"job": dome_job(), "dry_run": "yes"}),
        json!({"job": dome_job(), "output": 5}),
    ];
    for p in &cases {
        match s.execute("cam.job", p) {
            Err(EngineError::Internal(_, msg)) => panic!("{p}: panicked: {msg}"),
            Err(_) => {}
            Ok(v) => panic!("{p}: accepted: {v}"),
        }
    }
    // Junk G-code is reported, never a panic; a missing file is an error.
    let junk = dir.file("junk.nc");
    std::fs::write(&junk, "G1 X1 Y2 Z3 F100\nG2 X5 Y5\n\u{0}\u{1}garbage QQQ\nG91 G1 X1\n").unwrap();
    let c = run(&mut s, "cam.check", json!({"path": junk}));
    assert!(c["errors"].as_array().is_some_and(|e| e.len() == 1), "{c}");
    assert!(s.execute("cam.check", &json!({"path": dir.file("missing.nc")})).is_err());
    assert!(s.execute("cam.check", &json!({"path": junk, "rapid_rate": -1})).is_err());
    assert!(s.execute("cam.posts", &json!({"name": "nope"})).is_err());
}

// Profiles, heights and tools worked out for an agent, and refusals with a fix (cam_agent.rs).

/// A block 40 × 30 × 20 on Z 0 (its top at 20) with a 16 × 10 pocket 5 mm deep in its top.
fn block_with_pocket() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 20, "corner": [-20, -15, 0], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 16, "width": 10, "height": 5, "corner": [-8, -5, 15], "operation": "cut"}));
    s
}

const POCKET: [[f64; 2]; 4] = [[-8.0, -5.0], [8.0, -5.0], [8.0, 5.0], [-8.0, 5.0]];

fn flat_tool(number: u32, diameter: f64, flute: f64) -> Value {
    json!({"number": number, "kind": "flat_endmill", "diameter": diameter, "flute_length": flute, "spindle_rpm": 12000, "feed": 800, "plunge_feed": 200})
}

fn agent_tools() -> Value {
    json!([
        flat_tool(2, 2.0, 6.0),
        flat_tool(4, 4.0, 12.0),
        flat_tool(6, 6.0, 18.0),
        flat_tool(8, 8.0, 20.0),
        flat_tool(10, 10.0, 22.0),
        ball(9, 4.0)
    ])
}

/// A 3-axis job on `block_with_pocket` with these operations.
fn agent_job(operations: Value) -> Value {
    json!({"machine": "3axis", "post": "mach3_4axis", "tools": agent_tools(), "operations": operations})
}

fn pocket_op(tool: Value) -> Value {
    json!({"name": "pocket", "type": "pocket", "tool": tool, "polygon": POCKET, "stepover": 1.5})
}

fn with_key(mut job: Value, key: &str, v: Value) -> Value {
    job[key] = v;
    job
}

/// The refusals a failed `cam.job` carries at the end of its message.
fn refusals(e: &EngineError) -> Vec<Value> {
    let text = e.to_string();
    let v = crate::schema::cam_agent::refusals_in(&text).unwrap_or_else(|| panic!("no refusals in: {text}"));
    v["cam_refusals"].as_array().cloned().unwrap_or_default()
}

#[test]
fn a_job_with_only_operations_and_a_profile_writes_gcode() {
    let mut s = block_with_pocket();
    let dir = Dir::new("profile");
    let post = run(&mut s, "cam.posts", json!({"name": "linuxcnc"}))["post"].clone();
    for sub in ["posts", "tools", "profiles"] {
        std::fs::create_dir_all(dir.0.join(sub)).unwrap();
    }
    std::fs::write(dir.0.join("posts/mill.post.json"), post.to_string()).unwrap();
    // A tool of a kind this build does not read is left out with a note, not a refusal.
    let mut library = agent_tools().as_array().cloned().unwrap();
    library.push(json!({"number": 30, "kind": "engraver", "diameter": 3, "flute_length": 2}));
    std::fs::write(dir.0.join("tools/mill.tools.json"), Value::Array(library).to_string()).unwrap();
    let profile = json!({
        "name": "test mill",
        "machine": "3axis",
        "post_path": "../posts/mill.post.json",
        "tool_library_paths": ["../tools/mill.tools.json"],
        "work_offset": 2,
        "tolerance": 0.02,
        "default_heights": {"clearance_above": 5},
        "default_feeds": {"max_cut": 700, "spindle_rpm_range": [10000, 24000]}
    });
    std::fs::write(dir.0.join("profiles/mill.profile.json"), profile.to_string()).unwrap();
    let job = json!({"profile_path": "profiles/mill.profile.json", "operations": [pocket_op(json!({"kind": "flat_endmill", "max_diameter": 6}))]});
    let job_path = dir.file("part.cam.json");
    std::fs::write(&job_path, job.to_string()).unwrap();
    let out = dir.file("part.tap");
    let r = run(&mut s, "cam.job", json!({"job_path": job_path, "output": out}));
    assert_clean(&r);
    assert!(r["profile"].as_str().is_some_and(|p| p.ends_with("mill.profile.json")), "{r}");
    assert_eq!(r["post"], post["base"]["name"], "{r}");
    assert_eq!(r["tool_choices"][0]["tool"], 6, "{r}");
    let notes = r["notes"].to_string();
    assert!(notes.contains("tool 30") && notes.contains("left out"), "{notes}");
    assert!(notes.contains("held to the machine's 700"), "the tool's 800 mm/min is held to the profile's limit: {notes}");
    assert_eq!(r["derived_heights"][0]["clearance"].as_f64(), Some(25.0), "clearance_above 5 from the profile: {r}");
    let text = std::fs::read_to_string(&out).unwrap();
    assert!(text.contains("G55"), "the profile's work offset: {}", &text[..text.len().min(400)]);
    assert!(text.contains("F700") && !text.contains("F800"), "the held feed is written");
    // The job's own keys win over the profile's.
    let own = with_key(with_key(job.clone(), "work_offset", json!(3)), "tools", json!([flat_tool(5, 4.0, 12.0)]));
    std::fs::write(&job_path, own.to_string()).unwrap();
    let r = run(&mut s, "cam.job", json!({"job_path": job_path, "dry_run": true}));
    assert_eq!(r["tool_choices"][0]["tool"], 5, "{r}");
    assert!(r["program_head"].to_string().contains("G56"), "{r}");
    // A misspelt profile key is refused with the key named.
    std::fs::write(dir.0.join("profiles/mill.profile.json"), json!({"machine": "3axis", "post_pth": "x"}).to_string()).unwrap();
    std::fs::write(&job_path, job.to_string()).unwrap();
    let e = s.execute("cam.job", &json!({"job_path": job_path, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("post_pth"), "{e}");
}

#[test]
fn omitted_heights_are_worked_out_from_the_part() {
    let mut s = block_with_pocket();
    let ops = json!([
        pocket_op(json!(4)),
        {"name": "outline", "type": "contour", "tool": 10, "polygon": [[-20, -15], [20, -15], [20, 15], [-20, 15]], "stepdown": 5},
        // Only the top given: the rest follow it.
        {"name": "shallow", "type": "pocket", "tool": 4, "polygon": POCKET, "heights": {"top": 18}, "stepover": 1.5, "gouge_check": false},
        {"name": "finish", "type": "parallel", "tool": 9, "stepover": 2, "boundary": {"rect": [[-10, -6], [10, 6]]}}
    ]);
    let r = run(&mut s, "cam.job", json!({"job": agent_job(ops), "dry_run": true}));
    assert_clean(&r);
    let d = r["derived_heights"].as_array().unwrap();
    let h = |i: usize| {
        let e = d.iter().find(|e| e["op_index"] == i).unwrap_or_else(|| panic!("{r}"));
        ["clearance", "retract", "top", "bottom"].map(|k| e[k].as_f64().unwrap())
    };
    let near = |a: [f64; 4], b: [f64; 4]| a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-6);
    // The pocket: from the part's top (20) to its floor (15), retract 2 and clearance 10 above.
    assert!(near(h(0), [30.0, 22.0, 20.0, 15.0]), "{:?} {r}", h(0));
    assert_eq!(d[0]["derived"], json!(["top", "bottom", "retract", "clearance"]), "{r}");
    assert!(d[0]["why"].as_str().unwrap().contains("floor"), "{r}");
    // The outside contour goes down to the part's bottom.
    assert!(near(h(1), [30.0, 22.0, 20.0, 0.0]), "{:?}", h(1));
    assert!(near(h(2), [30.0, 20.0, 18.0, 15.0]), "{:?}", h(2));
    assert!(near(h(3), [30.0, 22.0, 20.0, 0.0]), "{:?}", h(3));
    // A region over solid material has nothing to cut: refused with a fix.
    let ops = json!([{"name": "nothing", "type": "pocket", "tool": 4, "polygon": [[10, -4], [18, -4], [18, 4], [10, 4]]}]);
    let e = s.execute("cam.job", &json!({"job": agent_job(ops), "dry_run": true})).unwrap_err();
    let r = refusals(&e);
    assert_eq!(r[0]["check"], "heights", "{e}");
    assert!(r[0]["fix"].as_str().unwrap().contains("heights.bottom"), "{e}");
    // A pocket that goes through (here: partly off the part) goes down to the part's bottom.
    let ops = json!([{"name": "through", "type": "pocket", "tool": 4, "polygon": [[15, -4], [30, -4], [30, 4], [15, 4]], "gouge_check": false}]);
    let r = run(&mut s, "cam.job", json!({"job": agent_job(ops), "dry_run": true}));
    assert_eq!(r["derived_heights"][0]["bottom"].as_f64(), Some(0.0), "{r}");
}

#[test]
fn a_tool_spec_picks_the_best_match_and_says_why() {
    let mut s = block_with_pocket();
    let pick = |s: &mut Session, tool: Value| {
        let r = run(s, "cam.job", json!({"job": agent_job(json!([pocket_op(tool)])), "dry_run": true}));
        (r["tool_choices"][0]["tool"].as_u64().unwrap_or(0), r["tool_choices"][0]["why"].as_str().unwrap_or("").to_string())
    };
    // The largest flat end mill that fits the 10 mm wide pocket: 8 (10 does not fit).
    let (n, why) = pick(&mut s, json!({"kind": "flat_endmill"}));
    assert_eq!(n, 8, "{why}");
    assert!(why.contains("fits the profile") && why.contains("T6"), "{why}");
    assert_eq!(pick(&mut s, json!({"kind": "flat_endmill", "max_diameter": 5})).0, 4);
    assert_eq!(pick(&mut s, json!({"diameter": 4, "kind": "ball_endmill"})).0, 9);
    // Same diameter, any kind, flutes as long: the lowest number.
    assert_eq!(pick(&mut s, json!({"diameter": 4})).0, 4);
    assert_eq!(pick(&mut s, json!({"min_flute_length": 19, "max_diameter": 8})).0, 8);
    // Given by number, it says so.
    assert_eq!(pick(&mut s, json!(2)), (2, "given by number".to_string()));
    // No match: refused with the tools there are.
    let e = s.execute("cam.job", &json!({"job": agent_job(json!([pocket_op(json!({"kind": "bull_nose"}))])), "dry_run": true})).unwrap_err();
    let r = refusals(&e);
    assert_eq!((r[0]["check"].as_str(), r[0]["op"]["index"].as_u64()), (Some("tool"), Some(0)), "{e}");
    assert!(r[0]["fix"].as_str().unwrap().contains("4 (4 mm flat end mill)"), "{e}");
}

#[test]
fn every_refusal_names_the_operation_check_place_and_fix() {
    let mut s = block_with_pocket();
    let refused = |s: &mut Session, job: Value| -> Vec<Value> {
        let e = s.execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
        assert!(matches!(e, EngineError::BadParams { .. }), "{e}");
        let r = refusals(&e);
        for x in &r {
            assert!(x["fix"].as_str().is_some_and(|f| f.len() > 10), "{x}");
            assert!(x["error"].as_str().is_some_and(|f| !f.is_empty()), "{x}");
            assert!(x.get("at").is_some() && x.get("check").is_some() && x.get("op").is_some(), "{x}");
        }
        r
    };
    // Every operation that cannot be planned is reported at once, each with its index.
    let ops = json!([
        pocket_op(json!(7)),
        {"name": "low clearance", "type": "pocket", "tool": 4, "polygon": POCKET, "heights": {"clearance": 19, "retract": 19, "top": 18, "bottom": 15}},
        pocket_op(json!(10)),
        {"name": "order", "type": "contour", "tool": 4, "polygon": POCKET, "heights": {"clearance": 30, "retract": 25, "top": 15, "bottom": 18}},
        {"name": "fast", "type": "pocket", "tool": 4, "polygon": POCKET, "feeds": {"cut": 9000, "plunge": 100}}
    ]);
    let r = refused(&mut s, with_key(agent_job(ops), "default_feeds", json!({"max_cut": 5000})));
    let checks: Vec<(u64, &str)> = r.iter().map(|x| (x["op"]["index"].as_u64().unwrap(), x["check"].as_str().unwrap())).collect();
    assert_eq!(checks, [(0, "tool"), (1, "clearance"), (2, "fit"), (3, "heights"), (4, "feeds")], "{r:?}");
    assert!(r[0]["fix"].as_str().unwrap().contains("use one of the job's tools"), "{r:?}");
    assert!(r[1]["fix"].as_str().unwrap().contains("at least 22 (the part's top is 20)"), "{r:?}");
    assert_eq!(r[1]["at"][2].as_f64(), Some(20.0), "{r:?}");
    assert!(r[2]["fix"].as_str().unwrap().contains("at most 9.99 mm"), "the pocket is 10 mm wide: {r:?}");
    assert!(r[4]["fix"].as_str().unwrap().contains("at most 5000"), "{r:?}");
    // A gouge: where, on which program line, and what to change.
    let ops = json!([{"name": "too deep", "type": "pocket", "tool": 4, "polygon": POCKET, "heights": {"bottom": 12}, "stepover": 1.5}]);
    let r = refused_written(&mut s, agent_job(ops.clone()));
    assert_eq!(r[0]["check"], "gouge", "{r:?}");
    let at = r[0]["at"].as_array().unwrap();
    assert!(at[2].as_f64().is_some_and(|z| z < 15.0), "the gouge is below the floor: {r:?}");
    assert!(r[0]["line"].as_u64().is_some_and(|l| l > 1), "{r:?}");
    assert!(r[0]["fix"].as_str().unwrap().contains("raise `heights.bottom` to at least"), "{r:?}");
    // The dry run's reply has the same.
    let ok = run(&mut s, "cam.job", json!({"job": agent_job(ops), "dry_run": true}));
    assert_eq!(ok["failure_details"][0]["check"], "gouge", "{ok}");
    // A holder that comes down onto the block round the pocket.
    let job = with_key(
        agent_job(json!([{"name": "short tool", "type": "pocket", "tool": 3, "polygon": POCKET, "stepover": 1.5}])),
        "tools",
        json!([{"number": 3, "kind": "flat_endmill", "diameter": 4, "flute_length": 4, "stickout": 4.5, "shank_diameter": 3, "spindle_rpm": 12000, "feed": 800,
                "plunge_feed": 200, "holder": [{"diameter": 30, "length": 20}]}]),
    );
    let r = refused_written(&mut s, job);
    assert_eq!(r[0]["check"], "holder", "{r:?}");
    assert!(r[0]["fix"].as_str().unwrap().contains("standing out at least 6 mm"), "{r:?}");
    // Tool numbers that clash between libraries: refused with the clash, unless one is picked.
    let dir = Dir::new("clash");
    std::fs::write(dir.0.join("a.json"), json!([flat_tool(1, 4.0, 12.0), flat_tool(2, 2.0, 6.0)]).to_string()).unwrap();
    std::fs::write(dir.0.join("b.json"), json!([flat_tool(1, 6.0, 18.0), flat_tool(2, 2.0, 6.0)]).to_string()).unwrap();
    let job = json!({"machine": "3axis", "post": "mach3", "tool_library_paths": [dir.file("a.json"), dir.file("b.json")], "operations": [pocket_op(json!(1))]});
    let r = refused(&mut s, job.clone());
    assert_eq!((r[0]["check"].as_str(), r[0]["op"].is_null()), (Some("tools"), true), "{r:?}");
    assert!(r[0]["error"].as_str().unwrap().contains("T1 in both `a` and `b`"), "T2 is the same tool in both: {r:?}");
    assert!(r[0]["fix"].as_str().unwrap().contains("tool_library"), "{r:?}");
    let ok = run(&mut s, "cam.job", json!({"job": with_key(job, "tool_library", json!("b")), "dry_run": true}));
    assert!(ok["tool_choices"][0]["description"].as_str().unwrap().contains("6 mm"), "{ok}");
}

/// A job whose checks fail, refused when it would be written.
fn refused_written(s: &mut Session, job: Value) -> Vec<Value> {
    let dir = Dir::new("refused");
    let out = dir.file("x.tap");
    let e = s.execute("cam.job", &json!({"job": job, "output": out})).unwrap_err();
    assert!(!PathBuf::from(&out).exists(), "nothing was written");
    let r = refusals(&e);
    assert!(!r.is_empty() && r.iter().all(|x| x["fix"].as_str().is_some_and(|f| f.len() > 10)), "{e}");
    r
}

#[test]
fn the_compact_reply_keeps_what_an_agent_needs() {
    let mut s = block_with_pocket();
    let ops = json!([
        pocket_op(json!({"kind": "flat_endmill", "max_diameter": 6})),
        {"name": "outline", "type": "contour", "tool": 10, "polygon": [[-20, -15], [20, -15], [20, 15], [-20, 15]], "stepdown": 5}
    ]);
    let r = run(&mut s, "cam.job", json!({"job": agent_job(ops), "dry_run": true, "compact": true}));
    let mut keys: Vec<&str> = r.as_object().unwrap().keys().map(String::as_str).collect();
    keys.sort_unstable();
    assert_eq!(keys, ["compact", "dry_run", "failures", "operations", "outputs", "passed", "program_lines", "tools", "total_minutes"], "{r}");
    assert_eq!((r["compact"].as_bool(), r["passed"].as_bool()), (Some(true), Some(true)), "{r}");
    let lines: Vec<&str> = r["operations"].as_array().unwrap().iter().filter_map(Value::as_str).collect();
    assert!(lines[0].starts_with("0 `pocket`: pocket, T6 6 mm flat end mill") && lines[0].contains("heights 30/22/20/15"), "{lines:?}");
    assert!(lines[1].starts_with("1 `outline`: contour, T10"), "{lines:?}");
    assert!(r["total_minutes"].as_f64().is_some_and(|m| m > 0.0), "{r}");
    assert_eq!(r["tools"][1]["why"], "given by number", "{r}");
    // The compact reply of a failing dry run carries the failures with their fixes.
    let ops = json!([{"name": "too deep", "type": "pocket", "tool": 4, "polygon": POCKET, "heights": {"bottom": 12}, "stepover": 1.5}]);
    let r = run(&mut s, "cam.job", json!({"job": agent_job(ops), "dry_run": true, "compact": true}));
    assert_eq!(r["passed"], false, "{r}");
    assert!(r["failures"][0]["fix"].as_str().is_some_and(|f| f.contains("heights.bottom")), "{r}");
}

/// The new keys are as strict as the rest: a misspelt key in a tool spec, partial heights, a
/// default policy or the call is refused with the key named; hostile values never panic.
#[test]
fn agent_keys_are_strict() {
    let mut s = block_with_pocket();
    let job = || agent_job(json!([pocket_op(json!(4))]));
    let cases = [
        (agent_job(json!([pocket_op(json!({"kind": "flat_endmill", "diametr": 6}))])), "diametr"),
        (agent_job(json!([{"name": "p", "type": "pocket", "tool": 4, "polygon": POCKET, "heights": {"botom": 15}}])), "botom"),
        (with_key(job(), "default_heights", json!({"clearance": 5})), "clearance"),
        (with_key(job(), "default_feeds", json!({"max_feed": 5})), "max_feed"),
    ];
    for (job, word) in cases {
        let e = s.execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
        assert!(e.to_string().contains(word), "{word}: {e}");
    }
    let e = s.execute("cam.job", &json!({"job": job(), "dry_run": true, "compakt": true})).unwrap_err();
    assert!(e.to_string().contains("compakt"), "{e}");
    for bad in [
        with_key(job(), "default_feeds", json!({"cut_scale": 1e308})),
        with_key(job(), "default_heights", json!({"clearance_above": -1})),
        agent_job(json!([pocket_op(json!({"max_diameter": -3}))])),
        agent_job(json!([pocket_op(json!({"kind": "laser"}))])),
        agent_job(json!([pocket_op(json!({"diameter": 1e308}))])),
        json!({"profile_path": 5, "operations": []}),
        json!({"operations": [pocket_op(json!(4))]}),
        json!({"machine": "3axis", "machine_path": "x.json", "post": "mach3", "tools": agent_tools(), "operations": [pocket_op(json!(4))]}),
        json!({"machine": "3axis", "post": "mach3", "tool_library_paths": [], "operations": [pocket_op(json!(4))]}),
        json!({"machine": "3axis", "post": "mach3", "tool_library": "a", "tools": agent_tools(), "operations": [pocket_op(json!(4))]}),
    ] {
        match s.execute("cam.job", &json!({"job": bad, "dry_run": true})) {
            Err(EngineError::Internal(_, m)) => panic!("{bad}: panicked: {m}"),
            Err(_) => {}
            Ok(v) => panic!("{bad}: accepted: {v}"),
        }
    }
}

// Review of the agent-facing defaults: what they work out must never turn a safe job into a
// crash the checks do not see, and must not walk an agent into a refusal it could have avoided.

/// A block 40 × 60 × 60 about the X axis, its corners 42.4 mm from it, with a 12 × 12 pocket 5 mm
/// deep in its top (+Z) and another in its +Y face.
fn tall_block_on_a() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 60, "height": 60, "corner": [-20, -30, -30], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 12, "width": 12, "height": 5, "corner": [-6, -6, 25], "operation": "cut"}));
    run(&mut s, "solid.box", json!({"length": 12, "width": 5, "height": 12, "corner": [-6, 25, -6], "operation": "cut"}));
    s
}

fn turning_job(post: &str, heights: Option<Value>) -> Value {
    let square = json!([[-6, -6], [6, -6], [6, 6], [-6, 6]]);
    let mut top = json!({"name": "top pocket", "type": "pocket", "tool": 1, "polygon": square, "stepover": 1.5});
    let mut side = json!({"name": "side pocket", "type": "pocket", "tool": 1, "polygon": square, "stepover": 1.5});
    if let Some(h) = heights {
        top["heights"] = h.clone();
        side["heights"] = h;
    }
    json!({"machine": "4axis_a", "post": post, "tools": [flat(1, 4.0)], "operations": [top, {"type": "indexed", "a_deg": 90, "operation": side}]})
}

/// The lowest Z (program coordinates) at which the program turns its A axis, and how many turns.
fn lowest_turn(text: &str) -> (f64, usize) {
    let (mut z, mut a): (Option<f64>, Option<f64>) = (None, None);
    let (mut lowest, mut turns) = (f64::INFINITY, 0);
    for line in text.lines() {
        let code = line.split('(').next().unwrap_or("");
        for w in code.split_whitespace() {
            let (letter, value) = w.split_at(1);
            let Ok(v) = value.parse::<f64>() else { continue };
            match letter {
                "Z" => z = Some(v),
                "A" => {
                    if a.is_none_or(|old| (old - v).abs() > 1e-9) {
                        turns += 1;
                        lowest = lowest.min(z.unwrap_or(f64::NEG_INFINITY));
                    }
                    a = Some(v);
                }
                _ => {}
            }
        }
    }
    (lowest, turns)
}

/// On a 4th axis the part turns under the tool between operations: a clearance worked out from
/// the part's top in one frame (here 30 + 10) is below the corners' 42.4 mm reach from the A axis,
/// and the built-in 4-axis posts turn A at the operation's clearance. Nothing checks rapids, so
/// the worked-out clearance itself must clear the part as it turns, and a given one that does
/// not is refused with the height that does.
#[test]
fn a_worked_out_clearance_clears_the_part_as_the_rotary_turns() {
    let mut s = tall_block_on_a();
    let reach = 30.0 * 2f64.sqrt();
    let dir = Dir::new("turn");
    let out = dir.file("turn.tap");
    let r = run(&mut s, "cam.job", json!({"job": turning_job("mach3_4axis", None), "output": out}));
    assert_clean(&r);
    for d in r["derived_heights"].as_array().unwrap() {
        let c = d["clearance"].as_f64().unwrap();
        assert!(c >= reach + 10.0 - 1e-6 && c < reach + 10.5, "{d}");
        assert!(d["why"].as_str().unwrap().contains("A axis"), "{d}");
    }
    let (lowest, turns) = lowest_turn(&std::fs::read_to_string(&out).unwrap());
    assert!(turns >= 2 && lowest > reach, "A turns at Z {lowest}, the part reaches {reach}");
    // A clearance given above the part's top but inside its reach is refused with the fix.
    let job = turning_job("mach3_4axis", Some(json!({"clearance": 40, "retract": 32})));
    let e = s.execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
    let r = refusals(&e);
    assert_eq!(r[0]["check"], "clearance", "{e}");
    let fix = r[0]["fix"].as_str().unwrap();
    assert!(fix.contains("A axis") && fix.contains("at least 44.4"), "{fix}");
    // A post that goes to the machine's top before turning needs no more than the part's top.
    let r = run(&mut s, "cam.job", json!({"job": turning_job("rs274_5axis", None), "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["derived_heights"][0]["clearance"].as_f64(), Some(40.0), "{r}");
}

/// A bolt hole through a pocket's floor is no reason to take the pocket through the part: the
/// worked-out bottom stays on the floor (it went down to the part's bottom, and every such job
/// was refused for a gouge through the floor).
#[test]
fn a_hole_through_a_pocket_floor_leaves_the_bottom_on_the_floor() {
    let mut s = block_with_pocket();
    run(&mut s, "solid.cylinder", json!({"radius": 2.5, "height": 20, "axis": "Z", "base": [0, 0, 0], "operation": "cut"}));
    let r = run(&mut s, "cam.job", json!({"job": agent_job(json!([pocket_op(json!(4))])), "dry_run": true}));
    assert_clean(&r);
    let d = &r["derived_heights"][0];
    assert_eq!(d["bottom"].as_f64(), Some(15.0), "{r}");
    assert!(d["why"].as_str().unwrap().contains("through"), "{d}");
}

/// A block 40 × 30 × 20 with a 16 × 10 pocket 15 mm deep in its top.
fn deep_pocket() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 20, "corner": [-20, -15, 0], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 16, "width": 10, "height": 15, "corner": [-8, -5, 5], "operation": "cut"}));
    s
}

/// A tool picked by what it is must reach the depth worked out for it, and where heights or the
/// tool were worked out, a tool without holder data is checked with the least holder it can
/// have (just wider than its shank, at its stickout): its holder would otherwise come down into
/// the part with no check to see it.
#[test]
fn a_worked_out_cut_never_takes_the_holder_into_the_part() {
    let mut s = deep_pocket();
    let tools = json!([flat_tool(8, 8.0, 10.0), flat_tool(6, 6.0, 18.0)]);
    let job = |op: Value| json!({"machine": "3axis", "post": "mach3_4axis", "tools": tools, "operations": [op]});
    // The 8 mm tool fits but its 10 mm flutes do not reach the 15 mm floor: the 6 mm one is picked.
    let r = run(&mut s, "cam.job", json!({"job": job(pocket_op(json!({"kind": "flat_endmill"}))), "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["tool_choices"][0]["tool"], 6, "{r}");
    assert!(r["tool_choices"][0]["why"].as_str().unwrap().contains("reach"), "{r}");
    // Given by number, the worked-out 15 mm cut with 10 mm of tool is refused, with the fix.
    let dir = Dir::new("reach");
    let e = s.execute("cam.job", &json!({"job": job(pocket_op(json!(8))), "output": dir.file("x.tap")})).unwrap_err();
    let r = refusals(&e);
    assert_eq!(r[0]["check"], "holder", "{e}");
    let fix = r[0]["fix"].as_str().unwrap();
    assert!(fix.contains("stickout") && fix.contains("min_stickout: 16"), "{fix}");
    // An outside contour down the whole 20 mm block: the same.
    let outline = json!({"name": "outline", "type": "contour", "tool": 6, "polygon": [[-20, -15], [20, -15], [20, 15], [-20, 15]], "stepdown": 5});
    let r = run(&mut s, "cam.job", json!({"job": job(outline.clone()), "dry_run": true}));
    assert_eq!(r["failure_details"][0]["check"], "holder", "{r}");
    // A tool standing out far enough passes.
    let long = json!([{"number": 6, "kind": "flat_endmill", "diameter": 6, "flute_length": 18, "stickout": 26, "spindle_rpm": 12000, "feed": 800, "plunge_feed": 200}]);
    let r = run(&mut s, "cam.job", json!({"job": with_key(job(outline), "tools", long), "dry_run": true}));
    assert_clean(&r);
}

/// The compact reply keeps the notes that matter: here, a body named like a vise that is about
/// to be machined as part of the part.
#[test]
fn the_compact_reply_keeps_the_notes_that_matter() {
    let mut s = block_with_pocket();
    run(&mut s, "solid.box", json!({"length": 10, "width": 10, "height": 10, "corner": [30, 30, 0], "body_name": "Vise jaw"}));
    let r = run(&mut s, "cam.job", json!({"job": agent_job(json!([pocket_op(json!(4))])), "dry_run": true, "compact": true}));
    let notes = r["notes"].to_string();
    assert!(notes.contains("Vise jaw") && notes.contains("fixtures"), "{r}");
    assert!(!notes.contains("tessellated"), "the tessellation note is not news: {r}");
}

/// Heights given out of range are refused before anything is worked out from them.
#[test]
fn hostile_heights_are_refused_before_the_floor_is_searched() {
    let mut s = block_with_pocket();
    for h in [json!({"top": 1e300}), json!({"bottom": -1e300}), json!({"clearance": 1e300})] {
        let op = json!({"name": "p", "type": "pocket", "tool": 4, "polygon": POCKET, "heights": h});
        let e = s.execute("cam.job", &json!({"job": agent_job(json!([op])), "dry_run": true})).unwrap_err();
        assert_eq!(refusals(&e)[0]["check"], "heights", "{h}: {e}");
    }
}
