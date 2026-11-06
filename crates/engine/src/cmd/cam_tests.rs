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

/// A plate 40 × 30 × 10, its top at Z 0, with a ⌀10 hole 6 deep at (-10, 0) and a ⌀5 hole
/// through at (10, 0) (an M6 tap drill), and a sketch `Holes` of both circles on its top.
fn holed_plate() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 10, "corner": [-20, -15, -10], "body_name": "Plate"}));
    run(&mut s, "solid.cylinder", json!({"radius": 5, "height": 7, "base": [-10, 0, -6], "operation": "cut", "targets": ["Plate"]}));
    run(&mut s, "solid.cylinder", json!({"radius": 2.5, "height": 12, "base": [10, 0, -11], "operation": "cut", "targets": ["Plate"]}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Holes"}));
    run(&mut s, "sketch.circle.center", json!({"center": [-10, 0], "radius": 5}));
    run(&mut s, "sketch.circle.center", json!({"center": [10, 0], "radius": 2.5}));
    run(&mut s, "sketch.finish", json!({}));
    s
}

fn thread_mill_tool(number: u32, diameter: f64) -> Value {
    json!({"number": number, "kind": "thread_mill", "diameter": diameter, "flute_length": 12, "spindle_rpm": 12000, "feed": 300, "plunge_feed": 100})
}

fn bore_job(op: Value) -> Value {
    let mut op = op;
    for (k, v) in [("name", json!("bore")), ("type", json!("bore")), ("tool", json!(1))] {
        op[k] = v;
    }
    op["heights"] = json!({"clearance": 20, "retract": 5, "top": 0, "bottom": -6});
    json!({"machine": "3axis", "post": "mach3_4axis", "tools": [flat(1, 6.0)], "operations": [op]})
}

fn m6_job() -> Value {
    json!({
        "machine": "4axis_a",
        "post": "mach3_4axis",
        "tools": [thread_mill_tool(2, 4.5)],
        "operations": [{
            "name": "M6 thread", "type": "thread_mill", "tool": 2, "centers": [[10, 0]],
            "heights": {"clearance": 20, "retract": 5, "top": 0, "bottom": -8},
            "pitch": 1.0, "thread_diameter": 6.0
        }]
    })
}

/// The Z of every helical arc of `text` (a G2/G3 block with Z), with its motion code: the
/// motion is modal, so a block without G2/G3 continues the last one.
fn helix_z(text: &str) -> Vec<(f64, u8)> {
    let mut out = Vec::new();
    let mut motion = 0u8;
    for line in text.lines().filter(|l| !l.starts_with('(')) {
        let words: Vec<&str> = line.split_whitespace().collect();
        for w in &words {
            match *w {
                "G0" | "G00" => motion = 0,
                "G1" | "G01" => motion = 1,
                "G2" | "G02" => motion = 2,
                "G3" | "G03" => motion = 3,
                _ => {}
            }
        }
        let z = words.iter().find_map(|w| w.strip_prefix('Z')).and_then(|v| v.parse::<f64>().ok());
        if let (Some(z), true, true) = (z, words.iter().any(|w| w.starts_with('I')), motion >= 2) {
            out.push((z, motion));
        }
    }
    out
}

#[test]
fn a_bore_reads_back_with_its_helical_arcs_and_clears_the_hole_it_is_checked_against() {
    let mut s = holed_plate();
    let dir = Dir::new("bore");
    let out = dir.file("bore.tap");
    let job = bore_job(json!({"holes": [{"center": [-10, 0], "diameter": 10}], "pitch": 0.5}));
    let r = run(&mut s, "cam.job", json!({"job": job, "output": out}));
    assert_clean(&r);
    let op = &r["operations"][0];
    assert_eq!(op["kind"], "bore", "{r}");
    assert!(op["warnings"].as_array().is_some_and(Vec::is_empty), "{r}");
    let text = std::fs::read_to_string(&out).unwrap();
    // Down a quarter of a millimetre each half turn, exactly, counter-clockwise (climb), to the floor.
    let z = helix_z(&text);
    assert_eq!(z.len(), 24, "{text}");
    assert!(z.iter().all(|(_, g)| *g == 3), "{z:?}");
    for (k, (zk, _)) in z.iter().enumerate() {
        assert!((zk - (-0.25 * (k + 1) as f64)).abs() < 1e-9, "{z:?}");
    }
    let c = run(&mut s, "cam.check", json!({"path": out}));
    assert!(c["errors"].as_array().is_some_and(Vec::is_empty) && c["arcs"].as_u64().is_some_and(|n| n >= 28), "{c}");
    assert!((c["bounds"][0][2].as_f64().unwrap() + 6.0).abs() < 1e-9, "{c}");
    // A bore wider than the hole in the model cuts into the part: the gouge check says so.
    let wide = bore_job(json!({"holes": [{"center": [-10, 0], "diameter": 12}], "pitch": 0.5}));
    let r = run(&mut s, "cam.job", json!({"job": wide, "dry_run": true}));
    assert_eq!(r["passed"], false, "{r}");
    assert!(r["gouge"]["worst"].as_f64().is_some_and(|w| w > 0.5), "{r}");
}

#[test]
fn a_bore_takes_the_sketch_circles_in_its_diameter_range() {
    let mut s = holed_plate();
    let job = bore_job(json!({"sketch": "Holes", "diameter_range": [8, 12], "ramp_angle_deg": 2, "finish_passes": 1}));
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    assert!(r["operations"][0]["moves"].as_u64().is_some_and(|m| m > 30), "{r}");
    let none = bore_job(json!({"sketch": "Holes", "diameter_range": [20, 30]}));
    let e = s.execute("cam.job", &json!({"job": none, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("no circles"), "{e}");
    // The ⌀5 circle is smaller than the ⌀6 tool: refused, the hole named.
    let all = bore_job(json!({"sketch": "Holes"}));
    let e = s.execute("cam.job", &json!({"job": all, "dry_run": true})).unwrap_err();
    assert!(matches!(e, EngineError::BadParams { .. }) && e.to_string().contains("hole 2 at (10, 0)"), "{e}");
}

#[test]
fn an_undersize_hole_is_refused_and_a_tight_one_warned_of() {
    let mut s = holed_plate();
    let small = bore_job(json!({"holes": [{"center": [10, 0], "diameter": 5}], "pitch": 0.5}));
    let e = s.execute("cam.job", &json!({"job": small, "dry_run": true})).unwrap_err();
    let msg = e.to_string();
    assert!(msg.contains("hole 1 at (10, 0)") && msg.contains("too small") && msg.contains("`bore`"), "{msg}");
    let tight = bore_job(json!({"holes": [{"center": [-10, 0], "diameter": 7}], "pitch": 0.5}));
    let r = run(&mut s, "cam.job", json!({"job": tight, "dry_run": true}));
    assert_clean(&r);
    let warnings = r["operations"][0]["warnings"].to_string();
    assert!(warnings.contains("chips"), "{r}");
}

#[test]
fn an_m6x1_internal_thread_is_written_one_pitch_a_turn_and_passes_the_checks() {
    let mut s = holed_plate();
    let dir = Dir::new("thread");
    let out = dir.file("m6.tap");
    let r = run(&mut s, "cam.job", json!({"job": m6_job(), "output": out}));
    assert_clean(&r);
    assert_eq!(r["operations"][0]["kind"], "thread_mill", "{r}");
    let text = std::fs::read_to_string(&out).unwrap();
    let z = helix_z(&text);
    // Eight turns up from -8, counter-clockwise: each half turn exactly half a pitch.
    assert_eq!(z.len(), 16, "{text}");
    assert!(z.iter().all(|(_, g)| *g == 3), "{z:?}");
    let mut last = -8.0;
    for (zk, _) in &z {
        assert!((zk - last - 0.5).abs() < 1e-9, "{z:?}");
        last = *zk;
    }
    let turns: Vec<f64> = z.chunks(2).filter_map(|t| t.last().map(|(zk, _)| *zk)).collect();
    assert!(turns.windows(2).all(|w| (w[1] - w[0] - 1.0).abs() < 1e-9), "one pitch per turn: {turns:?}");
    // A thread mill too big for the minor diameter, or an end mill, is refused.
    let mut big = m6_job();
    big["tools"][0]["diameter"] = json!(5.2);
    let e = s.execute("cam.job", &json!({"job": big, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("minor diameter"), "{e}");
    let mut end_mill = m6_job();
    end_mill["tools"] = json!([flat(2, 4.0)]);
    let e = s.execute("cam.job", &json!({"job": end_mill, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("thread mill"), "{e}");
    // A thread mill in a pocket is refused: only its core would be checked.
    let mut pocket = m6_job();
    pocket["operations"][0] = json!({"name": "p", "type": "pocket", "tool": 2, "polygon": [[0, -5], [5, -5], [5, 5], [0, 5]], "heights": {"clearance": 20, "retract": 5, "top": 0, "bottom": -1}});
    assert!(s.execute("cam.job", &json!({"job": pocket, "dry_run": true})).unwrap_err().to_string().contains("thread mill"));
}

#[test]
fn hostile_bores_and_threads_are_refused_without_panics() {
    let mut s = holed_plate();
    let bore = |f: &dyn Fn(&mut Value)| {
        let mut j = bore_job(json!({"holes": [{"center": [-10, 0], "diameter": 10}], "pitch": 0.5}));
        f(&mut j["operations"][0]);
        json!({"job": j, "dry_run": true})
    };
    let thread = |f: &dyn Fn(&mut Value)| {
        let mut j = m6_job();
        f(&mut j);
        json!({"job": j, "dry_run": true})
    };
    let unset = |o: &mut Value, k: &str| {
        if let Some(m) = o.as_object_mut() {
            m.remove(k);
        }
    };
    let cases = vec![
        bore(&|o| o["holes"] = json!([])),
        bore(&|o| o["holes"] = json!([{"center": [1e308, 0], "diameter": 10}])),
        bore(&|o| o["holes"] = json!([{"center": [0, 0], "diameter": -10}])),
        bore(&|o| o["holes"] = json!([{"center": [0, 0], "diameter": 1e308}])),
        bore(&|o| o["holes"] = json!([{"center": [0, 0]}])),
        bore(&|o| o["holes"] = json!([{"center": [0, 0], "diameter": 10, "depth": 3}])),
        bore(&|o| o["pich"] = json!(1)),
        bore(&|o| o["pitch"] = json!(-1)),
        bore(&|o| o["pitch"] = json!(0)),
        bore(&|o| o["pitch"] = json!(1e308)),
        bore(&|o| o["pitch"] = json!(1e-9)),
        bore(&|o| o["ramp_angle_deg"] = json!(2)),
        bore(&|o| o["stepdown"] = json!(1)),
        bore(&|o| o["sketch"] = json!("Holes")),
        bore(&|o| o["diameter_range"] = json!([1, 2])),
        bore(&|o| {
            unset(o, "pitch");
            o["ramp_angle_deg"] = json!(90);
        }),
        bore(&|o| {
            unset(o, "holes");
            o["sketch"] = json!("Holes");
            o["diameter_range"] = json!([12, 8]);
        }),
        bore(&|o| {
            unset(o, "holes");
            o["sketch"] = json!("Nope");
        }),
        bore(&|o| o["stepover"] = json!(0)),
        bore(&|o| o["stepover"] = json!(100)),
        bore(&|o| o["stock_to_leave"] = json!(-1)),
        bore(&|o| o["stock_to_leave"] = json!(4)),
        bore(&|o| o["finish_passes"] = json!(1000)),
        bore(&|o| o["finish_passes"] = json!(-1)),
        bore(&|o| o["heights"]["bottom"] = json!(-1e9)),
        bore(&|o| o["heights"]["bottom"] = json!(0)),
        bore(&|o| o["tool"] = json!(9)),
        thread(&|j| j["operations"][0]["pitch"] = json!(0)),
        thread(&|j| j["operations"][0]["pitch"] = json!(1e308)),
        thread(&|j| j["operations"][0]["thread_diameter"] = json!(-6)),
        thread(&|j| j["operations"][0]["thread_diameter"] = json!(1e308)),
        thread(&|j| j["operations"][0]["thread_depth"] = json!(0)),
        thread(&|j| j["operations"][0]["thread_depth"] = json!(5)),
        thread(&|j| j["operations"][0]["radial_passes"] = json!(0)),
        thread(&|j| j["operations"][0]["radial_passes"] = json!(1000)),
        thread(&|j| j["operations"][0]["centers"] = json!([])),
        thread(&|j| j["operations"][0]["centers"] = json!([[1e308, 0]])),
        thread(&|j| j["operations"][0]["center"] = json!([0, 0])),
        thread(&|j| j["operations"][0]["internal"] = json!("yes")),
        thread(&|j| j["operations"][0]["heights"]["bottom"] = json!(-1e5)),
        thread(&|j| j["tools"][0]["tooth_height"] = json!(3)),
        thread(&|j| j["tools"][0]["tooth_height"] = json!(0.2)),
        thread(&|j| j["tools"][0]["thread_pitch"] = json!(1.5)),
        thread(&|j| j["tools"][0]["thread_pitch"] = json!(-1)),
        thread(&|j| j["tools"][0]["thread_pich"] = json!(1)),
        thread(&|j| j["tools"][0]["kind"] = json!("drill")),
        thread(&|j| j["operations"][0]["thread_diameter"] = json!(4)),
    ];
    for p in &cases {
        match s.execute("cam.job", p) {
            Err(EngineError::Internal(_, msg)) => panic!("{p}: panicked: {msg}"),
            Err(_) => {}
            Ok(v) => panic!("{p}: accepted: {v}"),
        }
    }
}
