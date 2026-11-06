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

// Stock, facing and rapid safety (docs/cam-jobs.md, "Stock, facing and rapid safety").

/// A 60 × 20 block, its top at Z 0, with a 13 × 12 pocket 1 mm deep at each end, and a clamp
/// across its middle rising to Z 12.
fn clamped_block() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 20, "height": 10, "corner": [-30, -10, -10], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 13, "width": 12, "height": 1, "corner": [-25, -6, -1], "operation": "cut"}));
    run(&mut s, "solid.box", json!({"length": 13, "width": 12, "height": 1, "corner": [12, -6, -1], "operation": "cut"}));
    run(&mut s, "solid.box", json!({"length": 6, "width": 20, "height": 12, "corner": [-3, -10, 0], "operation": "new", "body_name": "Clamp"}));
    s
}

fn two_pockets_job(clearance: f64) -> Value {
    let pocket = |name: &str, x0: f64, x1: f64| {
        json!({"name": name, "type": "pocket", "tool": 1, "polygon": [[x0, -6], [x1, -6], [x1, 6], [x0, 6]],
               "heights": {"clearance": clearance, "retract": 2, "top": 0, "bottom": -1}, "stepover": 2})
    };
    json!({
        "machine": "3axis", "post": "mach3_4axis", "bodies": ["Block"], "fixtures": ["Clamp"],
        "tools": [flat(1, 6.0)],
        "operations": [pocket("left", -25.0, -12.0), pocket("right", 12.0, 25.0)]
    })
}

/// The link between the two pockets crosses the clamp at the clearance height: at 8 through
/// it, refused; at 20 over it.
#[test]
fn a_rapid_through_a_fixture_is_refused() {
    let mut s = clamped_block();
    let dir = Dir::new("rapid-fixture");
    let out = dir.file("pockets.tap");
    let e = s.execute("cam.job", &json!({"job": two_pockets_job(8.0), "output": out})).unwrap_err();
    assert!(matches!(e, EngineError::BadParams { .. }) && e.to_string().contains("rapids:"), "{e}");
    assert!(!PathBuf::from(&out).exists(), "nothing was written");
    let r = run(&mut s, "cam.job", json!({"job": two_pockets_job(8.0), "dry_run": true}));
    assert_eq!(r["passed"], false, "{r}");
    assert_eq!(r["gouge"]["pass"], true, "the cuts themselves are clean: {r}");
    assert_eq!(r["rapids"]["pass"], false, "{r}");
    let hit = &r["rapids"]["first"][0];
    assert_eq!(hit["kind"], "part", "{r}");
    assert_eq!(hit["operation"], "right", "the link belongs to the operation it leads into: {r}");
    assert!(hit["depth"].as_f64().is_some_and(|d| d > 3.9) && hit["at"][0].as_f64().is_some_and(|x| x.abs() < 6.1), "{r}");
    let r = run(&mut s, "cam.job", json!({"job": two_pockets_job(20.0), "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["rapids"]["pass"], true, "{r}");
    assert!(r["rapids"]["checked"].as_u64().is_some_and(|n| n > 4), "{r}");
    assert!(r["travel"].is_null() && r["stock"].is_null(), "{r}");
}

/// A square bar 20 mm across about X (corners 14.1 mm from the A axis) with a pocket on its
/// top and one on its +Y side, each at a clearance of `clearance` in its own frame.
fn square_bar_job(clearance: f64) -> Value {
    let pocket = |name: &str| {
        json!({"name": name, "type": "pocket", "tool": 1, "polygon": [[-6, -4], [6, -4], [6, 4], [-6, 4]],
               "heights": {"clearance": clearance, "retract": 11, "top": 10, "bottom": 9}, "stepover": 1.5})
    };
    json!({
        "machine": "4axis_a", "post": "mach3_4axis", "tools": [flat(1, 4.0)],
        "operations": [{"type": "indexed", "a_deg": 0, "operation": pocket("top")}, {"type": "indexed", "a_deg": 90, "operation": pocket("side")}]
    })
}

fn square_bar() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 20, "height": 20, "corner": [-20, -10, -10], "body_name": "Bar"}));
    run(&mut s, "solid.box", json!({"length": 12, "width": 8, "height": 1, "corner": [-6, -4, 9], "operation": "cut"}));
    run(&mut s, "solid.box", json!({"length": 12, "width": 1, "height": 8, "corner": [-6, 9, -4], "operation": "cut"}));
    s
}

/// The post rises to the operation's clearance before A turns: at 12 the bar's corners sweep
/// through the tool, refused; at 25 the turn is clear.
#[test]
fn an_a_turn_low_beside_a_square_bar_is_refused() {
    let mut s = square_bar();
    let e = s.execute("cam.job", &json!({"job": square_bar_job(12.0), "output": Dir::new("turn").file("bar.tap")})).unwrap_err();
    assert!(e.to_string().contains("rapids:") && e.to_string().contains("operation `side`"), "{e}");
    let r = run(&mut s, "cam.job", json!({"job": square_bar_job(12.0), "dry_run": true}));
    assert_eq!(r["gouge"]["pass"], true, "{r}");
    let hits = r["rapids"]["first"].as_array().unwrap();
    let turn = hits.iter().find(|h| h["kind"] == "turn").unwrap_or_else(|| panic!("{r}"));
    assert_eq!(turn["operation"], "side", "{r}");
    assert!(turn["depth"].as_f64().is_some_and(|d| (d - (200f64.sqrt() + 0.01 - 12.0)).abs() < 0.05), "{r}");
    let r = run(&mut s, "cam.job", json!({"job": square_bar_job(25.0), "dry_run": true}));
    assert_clean(&r);
    // Round stock of radius 30 reaches past a clearance of 25: the rapids are in it, the turn too.
    let mut job = square_bar_job(25.0);
    job["stock"] = json!({"cylinder": {"radius": 30, "x_min": -25, "x_max": 25}});
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_eq!(r["stock"]["kind"], "cylinder", "{r}");
    assert_eq!(r["rapids"]["pass"], false, "{r}");
    assert!(r["failures"].as_array().unwrap().iter().any(|f| f.as_str().is_some_and(|f| f.contains("through the stock"))), "{r}");
}

/// A 50 mm square block, its top at Z -1, under 1 mm of stock: faced with a 32 mm face mill
/// down to the block's top. Every descent is beside the stock with the whole cutter clear.
#[test]
fn facing_a_box_stock_enters_beside_it_and_reads_back() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 50, "width": 50, "height": 10, "corner": [-25, -25, -11], "body_name": "Block"}));
    let mill = json!({"number": 5, "kind": "flat_endmill", "diameter": 32, "flute_length": 6, "spindle_rpm": 9000, "feed": 1500, "plunge_feed": 400});
    let job = |stock: Value| {
        json!({
            "machine": "3axis", "post": "mach3_4axis", "tools": [mill.clone()], "stock": stock,
            "operations": [{"name": "face", "type": "face", "tool": 5, "bottom": -1, "stepdown": 0.5}]
        })
    };
    let dir = Dir::new("face");
    let out = dir.file("face.tap");
    let r = run(&mut s, "cam.job", json!({"job": job(json!({"box": {"min": [-25, -25, -11], "max": [25, 25, 0]}})), "output": out}));
    assert_clean(&r);
    assert_eq!(r["rapids"]["pass"], true, "{r}");
    assert_eq!(r["operations"][0]["kind"], "face", "{r}");
    assert_eq!(r["stock"]["max"], json!([25.0, 25.0, 0.0]), "{r}");
    let text = std::fs::read_to_string(&out).unwrap();
    let blocks = solidcraft_cam::gcode5::parse5(&text, &[]).unwrap();
    let mut last: Option<solidcore_record::Vec3> = None;
    let (mut descents, mut at_bottom) = (0, 0);
    for b in &blocks {
        let (p, cut) = match b {
            solidcraft_cam::gcode5::JointWords::Feed { linear, .. } => (*linear, true),
            solidcraft_cam::gcode5::JointWords::Rapid { linear, .. } => (*linear, false),
            _ => continue,
        };
        let gap = ((p.x.abs() - 25.0).max(0.0)).hypot((p.y.abs() - 25.0).max(0.0));
        if last.is_some_and(|q| p.z < q.z - 1e-9) || (!cut && p.z < 0.0) {
            descents += 1;
            assert!(gap >= 16.0, "a descent onto the stock at {p:?}");
        }
        if cut && (p.z + 1.0).abs() < 1e-9 {
            at_bottom += 1;
        }
        last = Some(p);
    }
    assert!(descents >= 2 && at_bottom > 4, "{descents} {at_bottom}");
    // The same stock as a margin round the part.
    let r = run(&mut s, "cam.job", json!({"job": job(json!({"around_part": {"margin": [0, 0, 1]}})), "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["stock"]["min"], json!([-25.0, -25.0, -12.0]), "{r}");
    assert_eq!(r["stock"]["max"], json!([25.0, 25.0, 0.0]), "{r}");
    // A face below the block's top cuts into it.
    let mut deep = job(json!({"box": {"min": [-25, -25, -11], "max": [25, 25, 0]}}));
    deep["operations"][0]["bottom"] = json!(-2);
    let r = run(&mut s, "cam.job", json!({"job": deep, "dry_run": true}));
    assert_eq!(r["gouge"]["pass"], false, "{r}");
}

/// A finish whose retract height is below the stock's top rapids through the stock: refused.
/// A machine whose travel ends below the clearance height: refused, naming the line.
#[test]
fn rapids_through_the_stock_and_beyond_the_travel_are_refused() {
    let mut s = dome();
    let mut job = dome_job();
    job["stock"] = json!({"box": {"min": [-20, -20, -20], "max": [20, 20, 18]}});
    let e = s.execute("cam.job", &json!({"job": job, "output": Dir::new("stock").file("d.tap")})).unwrap_err();
    assert!(e.to_string().contains("through the stock"), "{e}");
    job["stock"] = json!({"box": {"min": [-20, -20, -20], "max": [20, 20, 8]}});
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    // A planned move beyond the travel is refused as the program is written.
    let at = solidcore_record::Vec3::new;
    let mut m = Machine::three_axis();
    m.linear_limits = Some((at(-200.0, -200.0, -100.0), at(200.0, 200.0, 20.0)));
    let mut job = dome_job();
    job["machine"] = serde_json::to_value(&m).unwrap();
    let e = s.execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("beyond its travel"), "{e}");
    // The post's clearance, which the program rises to before A turns, is no planned move: the
    // travel check finds it.
    let mut s = bar();
    let dir = Dir::new("travel");
    let mut post = run(&mut s, "cam.posts", json!({"name": "mach3"}))["post"].clone();
    post["clearance"] = json!(60.0);
    let post_file = dir.file("tall.post.json");
    std::fs::write(&post_file, post.to_string()).unwrap();
    let mut m = Machine::four_axis_a();
    m.linear_limits = Some((at(-200.0, -200.0, -100.0), at(200.0, 200.0, 50.0)));
    let mut job = barrel_job();
    job["machine"] = serde_json::to_value(&m).unwrap();
    job.as_object_mut().unwrap().remove("post");
    job["post_path"] = json!(post_file);
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_eq!(r["readback"]["pass"], true, "{r}");
    assert_eq!(r["travel"]["pass"], false, "{r}");
    assert!(r["travel"]["violation"].as_str().is_some_and(|v| v.contains("Z at 60")), "{r}");
    assert!(r["failures"].as_array().unwrap().iter().any(|f| f.as_str().is_some_and(|f| f.starts_with("travel:"))), "{r}");
    m.linear_limits = Some((at(-200.0, -200.0, -100.0), at(200.0, 200.0, 80.0)));
    job["machine"] = serde_json::to_value(&m).unwrap();
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["travel"]["pass"], true, "{r}");
}

#[test]
fn hostile_stock_and_face_jobs_are_refused_without_panics() {
    let mut s = dome();
    let job = |f: &dyn Fn(&mut Value)| {
        let mut j = dome_job();
        f(&mut j);
        json!({"job": j, "dry_run": true})
    };
    // A face with a flat tool (number 7) and `extra` keys.
    let face = |extra: Value| {
        let mut j = dome_job();
        j["tools"] = json!([ball(2, 6.0), flat(7, 10.0)]);
        let mut op = json!({"name": "f", "type": "face", "tool": 7, "top": 7, "bottom": 6});
        if let (Some(o), Value::Object(e)) = (op.as_object_mut(), extra) {
            o.extend(e);
        }
        j["operations"] = json!([op]);
        json!({"job": j, "dry_run": true})
    };
    let cases = vec![
        job(&|j| j["stock"] = json!({"box": {"min": [0, 0, 0], "max": [0, 1, 1]}})),
        job(&|j| j["stock"] = json!({"box": {"min": [0, 0, 0], "max": [1e308, 1, 1]}})),
        job(&|j| j["stock"] = json!({"box": {"min": [0, 0], "max": [1, 1, 1]}})),
        job(&|j| j["stock"] = json!({"box": {"min": [0, 0, 0], "max": [1, 1, 1], "maxx": 1}})),
        job(&|j| j["stock"] = json!({"cylinder": {"radius": -1, "x_min": 0, "x_max": 1}})),
        job(&|j| j["stock"] = json!({"cylinder": {"radius": 5, "x_min": 1, "x_max": 1}})),
        job(&|j| j["stock"] = json!({"around_part": {"margin": [-1, 0, 0]}})),
        job(&|j| j["stock"] = json!({"around_part": {"margin": [1e308, 0, 0]}})),
        job(&|j| j["stock"] = json!({"sphere": {"radius": 5}})),
        job(&|j| j["stock"] = json!({"box": {"min": [0, 0, 0], "max": [1, 1, 1]}, "cylinder": {"radius": 1, "x_min": 0, "x_max": 1}})),
        job(&|j| j["stock"] = json!("box")),
        // A face without a stock or a top, with a ball, with hostile numbers or keys.
        job(&|j| j["operations"] = json!([{"name": "f", "type": "face", "tool": 2, "bottom": 6}])),
        job(&|j| j["operations"] = json!([{"name": "f", "type": "face", "tool": 2, "top": 7, "bottom": 6}])),
        face(json!({"top": null})),
        face(json!({"stepover": 1e308})),
        face(json!({"stepover": 1e-12})),
        face(json!({"stepdown": 1e-300})),
        face(json!({"bottom": 1e308})),
        face(json!({"bottom": 8})),
        face(json!({"rect": [[5, 5], [-5, -5]]})),
        face(json!({"rect": [[-1e300, 5], [5, 5]]})),
        face(json!({"overhang": -1})),
        face(json!({"angle_deg": 1e308})),
        face(json!({"retract": 1})),
        face(json!({"clearance": 1e308})),
        face(json!({"stepdwn": 1})),
        job(&|j| {
            j["operations"] = json!([{"type": "indexed", "a_deg": 0, "operation": {"name": "f", "type": "face", "tool": 2, "top": 7, "bottom": 6}}])
        }),
    ];
    for p in &cases {
        match s.execute("cam.job", p) {
            Err(EngineError::Internal(_, msg)) => panic!("{p}: panicked: {msg}"),
            Err(_) => {}
            Ok(v) => panic!("{p}: accepted: {v}"),
        }
    }
    // The face that the hostile ones start from is fine (it faces above the dome's top).
    let r = run(&mut s, "cam.job", face(json!({"top": 9, "bottom": 7.5})));
    assert_eq!(r["passed"], true, "{r}");
}

/// A `rect` inside the stock along the passes would take the face mill down into the stock
/// (at the plunge feed, a full-width plunge), and a bottom below the stock's would cut what
/// holds it: both refused, naming the operation. A strip reaching past the stock along the
/// passes is faced.
#[test]
fn facing_down_into_the_stock_or_below_it_is_refused() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 50, "width": 50, "height": 10, "corner": [-25, -25, -11], "body_name": "Block"}));
    let job = |op: Value| {
        let mut face = json!({"name": "face", "type": "face", "tool": 1, "bottom": -1, "stepdown": 0.5});
        if let (Some(o), Value::Object(e)) = (face.as_object_mut(), op) {
            o.extend(e);
        }
        json!({"job": {
            "machine": "3axis", "post": "mach3_4axis", "tools": [flat(1, 10.0)],
            "stock": {"box": {"min": [-25, -25, -11], "max": [25, 25, 0]}}, "operations": [face]
        }, "dry_run": true})
    };
    let r = run(&mut s, "cam.job", job(json!({})));
    assert_clean(&r);
    for bad in
        [json!({"rect": [[-15, -25], [15, 25]]}), json!({"rect": [[-25, -5], [25, 5]], "angle_deg": 90}), json!({"bottom": -12, "stepdown": 3})]
    {
        let e = s.execute("cam.job", &job(bad.clone())).unwrap_err();
        assert!(matches!(e, EngineError::BadParams { .. }) && e.to_string().contains("face"), "{bad}: {e}");
        assert!(e.to_string().contains("into the stock") || e.to_string().contains("below the stock's bottom"), "{bad}: {e}");
    }
    let r = run(&mut s, "cam.job", job(json!({"rect": [[-25, -5], [25, 5]]})));
    assert_clean(&r);
}

/// Faced all over from Z 0 down to the block's top at -1, the stock is lower for the pocket
/// after it: the pocket's retract height of -0.5, below the stock's first top, is above what is
/// left. Without the face, or with a face over half the top, its rapids run through the stock.
#[test]
fn rapids_after_a_face_are_held_against_the_faced_stock() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 50, "width": 50, "height": 10, "corner": [-25, -25, -11], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 10, "width": 10, "height": 1, "corner": [-5, -5, -2], "operation": "cut"}));
    let face = json!({"name": "face", "type": "face", "tool": 1, "bottom": -1, "stepdown": 0.5});
    let pocket = json!({"name": "pocket", "type": "pocket", "tool": 2, "polygon": [[-5, -5], [5, -5], [5, 5], [-5, 5]],
                        "heights": {"clearance": 5, "retract": -0.5, "top": -1, "bottom": -2}, "stepover": 2});
    let job = |ops: Value| {
        json!({"job": {
            "machine": "3axis", "post": "mach3_4axis", "tools": [flat(1, 10.0), flat(2, 4.0)],
            "stock": {"box": {"min": [-25, -25, -11], "max": [25, 25, 0]}}, "operations": ops
        }, "dry_run": true})
    };
    let r = run(&mut s, "cam.job", job(json!([face, pocket])));
    assert_clean(&r);
    assert_eq!(r["rapids"]["pass"], true, "{r}");
    let r = run(&mut s, "cam.job", job(json!([pocket])));
    assert_eq!(r["rapids"]["pass"], false, "{r}");
    assert!(r["rapids"]["first"].as_array().unwrap().iter().all(|h| h["kind"] == "stock" && h["operation"] == "pocket"), "{r}");
    let mut half = face.clone();
    half["rect"] = json!([[-25, -25], [0, 25]]);
    half["angle_deg"] = json!(90);
    let r = run(&mut s, "cam.job", job(json!([half, pocket])));
    assert_eq!(r["rapids"]["pass"], false, "{r}");
    // An overhang of its own may leave a strip along the edges: the stock is kept.
    let mut own = face.clone();
    own["overhang"] = json!(1);
    let r = run(&mut s, "cam.job", job(json!([own, pocket])));
    assert_eq!(r["rapids"]["pass"], false, "{r}");
}
