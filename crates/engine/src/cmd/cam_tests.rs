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

// Machinable features (`cam.features`) and job suggestions (`cam.suggest`).

/// A plate 80 × 60 × 10, its top at Z 0, with a 40 × 24 pocket 5 deep whose corners are
/// rounded to 4 mm and a round island of radius 4 standing in it up to the top, two through
/// holes of diameter 6 at y = -22 and two blind holes of diameter 4, 6 deep, at y = 22.
fn feature_plate() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 80, "width": 60, "height": 10, "corner": [-40, -30, -10], "body_name": "Plate"}));
    run(&mut s, "solid.box", json!({"length": 40, "width": 24, "height": 6, "corner": [-20, -12, -5], "operation": "cut"}));
    run(&mut s, "solid.fillet", json!({"edges": [[20, 12, -2.5], [-20, 12, -2.5], [-20, -12, -2.5], [20, -12, -2.5]], "radius": 4}));
    run(&mut s, "solid.cylinder", json!({"radius": 4, "height": 5, "base": [0, 0, -5], "operation": "join"}));
    for x in [-32.0, 32.0] {
        run(&mut s, "solid.cylinder", json!({"radius": 3, "height": 12, "base": [x, -22, -11], "operation": "cut"}));
        run(&mut s, "solid.cylinder", json!({"radius": 2, "height": 7, "base": [x, 22, -6], "operation": "cut"}));
    }
    s
}

/// A bar 60 × 20 × 20 along X about the X axis with a 12 × 8 pocket 4 deep in each of its
/// +Z, +Y, -Z and -Y faces, at different places along X.
fn pocketed_bar() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 20, "height": 20, "corner": [-30, -10, -10], "body_name": "Bar"}));
    for (corner, size) in [
        ([-26.0, -4.0, 6.0], [12.0, 8.0, 5.0]),
        ([-12.0, 6.0, -4.0], [12.0, 5.0, 8.0]),
        ([2.0, -4.0, -11.0], [12.0, 8.0, 5.0]),
        ([16.0, -11.0, -4.0], [12.0, 5.0, 8.0]),
    ] {
        run(&mut s, "solid.box", json!({"length": size[0], "width": size[1], "height": size[2], "corner": corner, "operation": "cut"}));
    }
    s
}

fn feature_tools() -> Value {
    json!([flat(6, 6.0), flat(4, 4.0), flat(2, 2.0), ball(12, 4.0)])
}

fn near(v: &Value, want: f64) -> bool {
    v.as_f64().is_some_and(|x| (x - want).abs() < 1e-6)
}

fn ids(list: &Value) -> Vec<String> {
    list.as_array().unwrap().iter().map(|f| f["id"].as_str().unwrap().to_string()).collect()
}

#[test]
fn features_of_a_plate_are_read_exactly_from_its_faces() {
    let mut s = feature_plate();
    let f = run(&mut s, "cam.features", json!({}));
    // Holes, by position: the two through holes of 6 mm first (y = -22), then the blind 4 mm ones.
    let holes = f["holes"].as_array().unwrap();
    assert_eq!(holes.len(), 4, "{f}");
    let want = [(-32.0, -22.0, 6.0, true), (32.0, -22.0, 6.0, true), (-32.0, 22.0, 4.0, false), (32.0, 22.0, 4.0, false)];
    for (h, (x, y, d, through)) in holes.iter().zip(want) {
        assert!(near(&h["center"][0], x) && near(&h["center"][1], y), "{h}");
        assert!(near(&h["diameter"], d) && near(&h["top_z"], 0.0), "{h}");
        assert_eq!(h["through"], through, "{h}");
        if through {
            assert!(near(&h["bottom_z"], -10.0) && near(&h["depth"], 10.0) && h["floor"] == "through", "{h}");
        } else {
            assert!(near(&h["bottom_z"], -6.0) && near(&h["depth"], 6.0) && h["floor"] == "flat", "{h}");
        }
        assert!(h.get("counterbore").is_none() && h.get("a_deg").is_none(), "{h}");
    }
    assert_eq!(ids(&f["holes"]), ["hole:0", "hole:1", "hole:2", "hole:3"]);
    let groups = f["hole_groups"].as_array().unwrap();
    assert_eq!(groups.len(), 2, "{f}");
    assert!(near(&groups[0]["diameter"], 6.0) && groups[0]["count"] == 2 && groups[0]["ids"] == json!(["hole:0", "hole:1"]), "{f}");
    assert!(near(&groups[1]["diameter"], 4.0) && groups[1]["count"] == 2 && groups[1]["through"] == false, "{f}");
    // The pocket: its floor, depth, rounded corners and island.
    let pockets = f["pockets"].as_array().unwrap();
    assert_eq!(pockets.len(), 1, "{f}");
    let p = &pockets[0];
    assert_eq!(p["id"], "pocket:0");
    assert!(near(&p["floor_z"], -5.0) && near(&p["top_z"], 0.0) && near(&p["depth"], 5.0), "{p}");
    assert!(near(&p["min_corner_radius"], 4.0) && p["sharp_corners"] == 0 && p["walls_vertical"] == true, "{p}");
    assert_eq!(p["bounds"], json!([[-20.0, -12.0], [20.0, 12.0]]), "{p}");
    let islands = p["islands"].as_array().unwrap();
    assert_eq!(islands.len(), 1, "{p}");
    assert!(near(&islands[0]["top_z"], 0.0), "{p}");
    let island_area = std::f64::consts::PI * 16.0;
    assert!((islands[0]["area"].as_f64().unwrap() - island_area).abs() < 0.05, "{p}");
    // The rounded rectangle less the island.
    let want = 40.0 * 24.0 - (4.0 - std::f64::consts::PI) * 16.0 - island_area;
    assert!((p["area"].as_f64().unwrap() - want).abs() < 0.1, "{p}");
    // The faces facing up, and the one outer profile (the island's top is not one).
    let faces = f["faces"].as_array().unwrap();
    let role = |r: &str| faces.iter().filter(|x| x["role"] == r).count();
    assert_eq!((role("top"), role("island_top"), role("pocket_floor"), role("hole_floor")), (1, 1, 1, 2), "{f}");
    let floor = faces.iter().find(|x| x["role"] == "pocket_floor").unwrap();
    assert_eq!(floor["pocket"], "pocket:0");
    assert_eq!(p["face_id"], floor["id"]);
    let profiles = f["profiles"].as_array().unwrap();
    assert_eq!(profiles.len(), 1, "{f}");
    assert!(near(&profiles[0]["top_z"], 0.0) && near(&profiles[0]["bottom_z"], -10.0), "{f}");
    assert_eq!(profiles[0]["bounds"], json!([[-40.0, -30.0], [40.0, 30.0]]), "{f}");
    assert!(profiles[0].get("min_concave_radius").is_none(), "a rectangle has no notches: {f}");
    assert!(f["fillets"].as_array().unwrap().is_empty() && f["surfaces"].as_array().unwrap().is_empty(), "{f}");
    let setup = &f["setups"][0];
    assert!(near(&setup["top_z"], 0.0) && near(&setup["bottom_z"], -10.0) && near(&setup["a_deg"], 0.0), "{f}");
    // The same part gives the same ids and numbers, run again or built again.
    assert_eq!(f, run(&mut s, "cam.features", json!({})));
    assert_eq!(f, run(&mut feature_plate(), "cam.features", json!({})));
}

#[test]
fn features_of_a_bar_on_a_4th_axis_carry_the_a_angle_that_brings_them_up() {
    let mut s = pocketed_bar();
    let f = run(&mut s, "cam.features", json!({"axis": "a"}));
    let pockets = f["pockets"].as_array().unwrap();
    assert_eq!(pockets.len(), 4, "{f}");
    // Each pocket's floor is 6 from the axis in its own setup, 4 below the face round it.
    let mut by_x: Vec<(f64, f64)> = Vec::new();
    for p in pockets {
        assert!(near(&p["floor_z"], 6.0) && near(&p["top_z"], 10.0) && near(&p["depth"], 4.0), "{p}");
        assert!(near(&p["min_corner_radius"], 0.0) && p["sharp_corners"] == 4, "{p}");
        let x = (p["bounds"][0][0].as_f64().unwrap() + p["bounds"][1][0].as_f64().unwrap()) * 0.5;
        by_x.push((x, p["a_deg"].as_f64().unwrap()));
    }
    by_x.sort_by(|a, b| a.0.total_cmp(&b.0));
    let angles: Vec<f64> = by_x.iter().map(|(_, a)| *a).collect();
    assert!(angles[0].abs() < 1e-6, "+Z: {by_x:?}");
    assert!((angles[1] - 90.0).abs() < 1e-6, "+Y: {by_x:?}");
    assert!((angles[2].abs() - 180.0).abs() < 1e-6, "-Z: {by_x:?}");
    assert!((angles[3] + 90.0).abs() < 1e-6, "-Y: {by_x:?}");
    // The A = 0 setup is first; every setup lists its pocket.
    let setups = f["setups"].as_array().unwrap();
    assert!(near(&setups[0]["a_deg"], 0.0), "{f}");
    for p in pockets {
        let a = p["a_deg"].as_f64().unwrap();
        let su = setups.iter().find(|su| near(&su["a_deg"], a)).unwrap();
        assert!(su["features"].as_array().unwrap().contains(&p["id"]), "{su}");
        assert!(near(&su["top_z"], 10.0) && (su["swing_radius"].as_f64().unwrap() - 200f64.sqrt()).abs() < 1e-6, "{su}");
    }
    // A 3-axis look sees the +Z pocket only, with the id it has on the 4th axis.
    let z = run(&mut s, "cam.features", json!({}));
    assert_eq!(ids(&z["pockets"]), ["pocket:0"]);
    assert_eq!(z["pockets"][0]["bounds"], pockets[0]["bounds"]);
    assert_eq!(f, run(&mut s, "cam.features", json!({"axis": "a"})));
}

#[test]
fn a_pocket_names_its_feature_and_takes_its_heights_from_it() {
    let mut s = feature_plate();
    let job =
        |ops: Value| json!({"job": {"machine": "3axis", "post": "linuxcnc_4axis", "tools": feature_tools(), "operations": ops}, "dry_run": true});
    // What an agent writes: the feature and a tool, nothing else.
    let ops = json!([
        {"name": "pocket", "type": "pocket", "feature": "pocket:0", "tool": 6, "stepover": 2},
        {"name": "walls", "type": "contour", "feature": "pocket:0", "tool": 6},
        {"name": "hole", "type": "pocket", "feature": "hole:0", "tool": 4}
    ]);
    let r = run(&mut s, "cam.job", job(ops));
    assert_clean(&r);
    assert!(r["operations"].as_array().unwrap().iter().all(|o| o["moves"].as_u64().is_some_and(|m| m > 4)), "{r}");
    // A pocket cut from a polygon still needs all its heights, and says which are missing.
    let poly = json!([{"name": "p", "type": "pocket", "tool": 6, "polygon": [[-5, -5], [5, -5], [5, 5]], "heights": {"top": 0}}]);
    let e = s.execute("cam.job", &job(poly)).unwrap_err();
    assert!(e.to_string().contains("`clearance`, `retract`, `bottom`"), "{e}");
    // A feature that is not there says which there are.
    let e = s.execute("cam.job", &job(json!([{"name": "p", "type": "pocket", "feature": "pocket:4", "tool": 6}]))).unwrap_err();
    assert!(e.to_string().contains("this part has only `pocket:0`"), "{e}");
    let e = s.execute("cam.job", &job(json!([{"name": "p", "type": "pocket", "feature": "fillet:0", "tool": 6}]))).unwrap_err();
    assert!(e.to_string().contains("ids are kind:number"), "{e}");
    let e = s.execute("cam.job", &job(json!([{"name": "p", "type": "pocket", "feature": "face:0", "tool": 6}]))).unwrap_err();
    assert!(e.to_string().contains("give `heights.top`"), "the top face has no depth: {e}");
    let both = json!([{"name": "p", "type": "pocket", "feature": "pocket:0", "polygon": [[0, 0], [1, 0], [0, 1]], "tool": 6}]);
    let e = s.execute("cam.job", &job(both)).unwrap_err();
    assert!(e.to_string().contains("one of them"), "{e}");
    // On the bar, a pocket on the +Y side is indexed to A = 90° by itself; inside an `indexed`
    // operation at another angle it is refused with the angle to use.
    let mut bar = pocketed_bar();
    let f = run(&mut bar, "cam.features", json!({"axis": "a"}));
    let side = f["pockets"].as_array().unwrap().iter().find(|p| near(&p["a_deg"], 90.0)).unwrap()["id"].as_str().unwrap().to_string();
    let job4 =
        |ops: Value| json!({"job": {"machine": "4axis_a", "post": "linuxcnc_4axis", "tools": feature_tools(), "operations": ops}, "dry_run": true});
    let r = run(&mut bar, "cam.job", job4(json!([{"name": "side", "type": "pocket", "feature": side, "tool": 4}])));
    assert_clean(&r);
    assert_eq!(r["operations"][0]["kind"], "indexed pocket", "{r}");
    assert!(near(&r["operations"][0]["a_deg"], 90.0), "{r}");
    let wrong = json!([{"type": "indexed", "a_deg": 0, "operation": {"name": "side", "type": "pocket", "feature": side, "tool": 4}}]);
    let e = bar.execute("cam.job", &job4(wrong)).unwrap_err();
    assert!(e.to_string().contains("`a_deg: 90`"), "{e}");
}

#[test]
fn a_suggested_job_passes_its_checks_on_a_plate_and_a_bar() {
    // The plate: faced, pocket roughed and finished, holes milled, profile cut, all checked.
    let mut s = feature_plate();
    let sug = run(&mut s, "cam.suggest", json!({"tools": feature_tools()}));
    let ops = sug["operations"].as_array().unwrap();
    let names: Vec<&str> = ops.iter().map(|o| o["name"].as_str().unwrap()).collect();
    assert_eq!(names.first(), Some(&"face top"), "{sug}");
    assert_eq!(names.last(), Some(&"profile profile:0"), "the profile is cut last: {sug}");
    assert!(names.contains(&"rough pocket:0") && names.contains(&"finish pocket:0 walls"), "{sug}");
    assert_eq!(names.iter().filter(|n| n.starts_with("mill hole:")).count(), 4, "{sug}");
    assert!(ops.iter().all(|o| o["type"] == "pocket" || o["type"] == "contour"), "{sug}");
    // Tools by size: the 6 mm tool fits the 4 mm corners, the holes take 4 and 2 mm tools.
    let tool_of = |name: &str| ops.iter().find(|o| o["name"] == name).unwrap()["tool"].as_u64().unwrap();
    assert_eq!((tool_of("rough pocket:0"), tool_of("finish pocket:0 walls"), tool_of("profile profile:0")), (6, 6, 6), "{sug}");
    assert_eq!((tool_of("mill hole:0"), tool_of("mill hole:2")), (4, 2), "{sug}");
    assert!(sug["assumptions"].as_array().unwrap().len() >= 4 && !sug["check"].as_array().unwrap().is_empty(), "{sug}");
    let job = sug["job"].clone();
    assert!(job["tools"].as_array().unwrap().iter().all(|t| t["number"] != 12), "the ball is not used: {job}");
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["operations"].as_array().unwrap().len(), names.len(), "{r}");
    // With `check`, the dry run comes back with the suggestion.
    let checked = run(&mut s, "cam.suggest", json!({"tools": feature_tools(), "check": true}));
    assert_eq!(checked["dry_run"]["passed"], true, "{checked}");
    assert_eq!(checked["job"], sug["job"], "the same part gives the same job");

    // The bar on a 4th axis: each pocket indexed to its side, roughed and finished.
    let mut bar4 = pocketed_bar();
    let sug = run(&mut bar4, "cam.suggest", json!({"tools": feature_tools(), "axis": "a"}));
    let job = sug["job"].clone();
    assert_eq!(job["machine"], "4axis_a", "{job}");
    let indexed = job["operations"].as_array().unwrap().iter().filter(|o| o["a_deg"].is_number()).count();
    assert_eq!(indexed, 6, "rough and finish on the three sides other than A = 0: {job}");
    let r = run(&mut bar4, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    let sides: std::collections::BTreeSet<i64> =
        r["operations"].as_array().unwrap().iter().map(|o| o["a_deg"].as_f64().unwrap_or(0.0).round() as i64).collect();
    assert_eq!(sides.len(), 4, "{r}");
    // A round bar's diameter is turned with a rotary finish.
    let mut round = bar();
    let f = run(&mut round, "cam.features", json!({"axis": "a"}));
    assert_eq!(f["turned"].as_array().unwrap().len(), 1, "{f}");
    let t = &f["turned"][0];
    assert!(near(&t["diameter"], 20.0) && near(&t["x_min"], -20.0) && near(&t["x_max"], 20.0), "{f}");
    let sug = run(&mut round, "cam.suggest", json!({"tools": feature_tools(), "axis": "a", "check": true}));
    assert_eq!(sug["dry_run"]["passed"], true, "{sug}");
    assert!(sug["operations"].as_array().unwrap().iter().any(|o| o["type"] == "rotary_finish"), "{sug}");
}

#[test]
fn feature_and_suggest_refusals_say_what_to_change() {
    let mut s = feature_plate();
    let dir = Dir::new("suggest");
    let cases = [
        (json!({"axis": "b"}), "axis"),
        (json!({"tolerance": 0}), "tolerance"),
        (json!({"origin": [1e308, 0, 0]}), "origin"),
        (json!({"bodies": ["Plat"]}), "did you mean `Plate`"),
        (json!({"bodies": []}), "no body to machine"),
        (json!({"bodies": ["Plate", "Plate"]}), "twice"),
    ];
    for (p, word) in cases {
        let e = s.execute("cam.features", &p).unwrap_err();
        assert!(e.to_string().contains(word), "{p}: {e}");
    }
    let lib = dir.file("lib.json");
    std::fs::write(&lib, json!([ball(12, 4.0), {"number": 9, "kind": "laser", "diameter": 1, "flute_length": 1}]).to_string()).unwrap();
    let profile = dir.file("mill.profile.json");
    std::fs::write(&profile, json!({"tool_library_path": "lib.json", "machine": "3axis", "spindel": 1}).to_string()).unwrap();
    let three = dir.file("three.profile.json");
    std::fs::write(&three, json!({"tools": feature_tools(), "machine": "3axis"}).to_string()).unwrap();
    let cases = [
        (json!({}), "one way"),
        (json!({"tools": feature_tools(), "tool_library_path": lib}), "one way"),
        (json!({"tool_library_path": lib}), "no flat end mill"),
        (json!({"profile_path": profile}), "unknown key `spindel`"),
        (json!({"tools": feature_tools(), "stock_top": 50}), "stock_top"),
        (json!({"profile_path": three, "axis": "a"}), "rotary axis about X"),
        (json!({"tools": [{"number": 1, "kind": "flat_endmill", "diameter": 4, "flute_length": 10}]}), "no `spindle_rpm`"),
    ];
    for (p, word) in cases {
        match s.execute("cam.suggest", &p) {
            Err(EngineError::Internal(_, msg)) => panic!("{p}: panicked: {msg}"),
            Err(e) => assert!(e.to_string().contains(word), "{p}: {e}"),
            Ok(v) => panic!("{p}: accepted: {v}"),
        }
    }
    // A profile file names its tools beside it; a tool it cannot use is listed, not fatal.
    std::fs::write(&lib, json!([flat(6, 6.0), flat(2, 2.0), {"number": 9, "kind": "laser", "diameter": 1, "flute_length": 1}]).to_string()).unwrap();
    std::fs::write(&profile, json!({"tool_library_path": "lib.json", "machine": "3axis", "post": "grblhal_4axis"}).to_string()).unwrap();
    let sug = run(&mut s, "cam.suggest", json!({"profile_path": profile, "stock_top": 0}));
    assert_eq!(sug["job"]["post"], "grblhal_4axis", "{sug}");
    assert!(sug["skipped"].as_array().unwrap().iter().any(|x| x.as_str().unwrap().starts_with("tool 9 not used")), "{sug}");
    assert!(!sug["operations"].as_array().unwrap().iter().any(|o| o["name"] == "face top"), "nothing to face: {sug}");
    // A hole no tool fits is skipped with the reason.
    let sug = run(&mut s, "cam.suggest", json!({"tools": [flat(6, 6.0)]}));
    assert!(sug["skipped"].as_array().unwrap().iter().any(|x| x.as_str().unwrap().starts_with("hole:0 (diameter 6 mm)")), "{sug}");
}

// Review of the features and suggestions: defaults that misled an agent or slipped past a check.

/// A block 60 × 40 × 10, its top at Z 0, with a 30 × 20 pocket 5 deep and a round island of
/// radius 4 standing in it up to the top, rounded 1 mm where it meets the floor.
fn filleted_island() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 10, "corner": [-30, -20, -10], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 30, "width": 20, "height": 6, "corner": [-15, -10, -5], "operation": "cut"}));
    run(&mut s, "solid.cylinder", json!({"radius": 4, "height": 5, "base": [0, 0, -5], "operation": "join"}));
    run(&mut s, "solid.fillet", json!({"edges": [[4, 0, -5]], "radius": 1}));
    s
}

/// A block 60 × 40 × 10, its top at Z 0, with a round pocket of radius 10, 5 deep, rounded
/// 1.5 mm where its wall meets its floor.
fn filleted_pocket() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 10, "corner": [-30, -20, -10], "body_name": "Block"}));
    run(&mut s, "solid.cylinder", json!({"radius": 10, "height": 6, "base": [0, 0, -5], "operation": "cut"}));
    run(&mut s, "solid.fillet", json!({"edges": [[10, 0, -5]], "radius": 1.5}));
    s
}

/// The highest Z a program moves to.
fn max_z(text: &str) -> f64 {
    text.lines()
        .filter(|l| !l.trim_start().starts_with('('))
        .flat_map(|l| l.split_whitespace())
        .filter_map(|w| w.strip_prefix('Z'))
        .filter_map(|v| v.parse::<f64>().ok())
        .fold(f64::NEG_INFINITY, f64::max)
}

#[test]
fn a_wall_rounded_at_its_foot_still_rises_to_its_top() {
    // The island's wall starts on a round, not on the floor: its top is still the island's.
    let mut s = filleted_island();
    let f = run(&mut s, "cam.features", json!({}));
    let p = &f["pockets"][0];
    assert!(near(&p["top_z"], 0.0) && near(&p["floor_z"], -5.0), "{p}");
    let islands = p["islands"].as_array().unwrap();
    assert_eq!(islands.len(), 1, "{p}");
    assert!(near(&islands[0]["top_z"], 0.0), "the island stands up to the top, not to its round: {p}");
    // So the suggested job does not cut the island down to the top of its round.
    let sug = run(&mut s, "cam.suggest", json!({"tools": feature_tools()}));
    assert!(!sug["operations"].as_array().unwrap().iter().any(|o| o["name"].as_str().unwrap().contains("band")), "{sug}");
    let r = run(&mut s, "cam.job", json!({"job": sug["job"], "dry_run": true}));
    assert_clean(&r);
    // A pocket rounded where its wall meets its floor is as deep as its rim, and its walls are
    // not upright: the 2D pocket on the floor's outline does not reach them, and the reply says so.
    let mut s = filleted_pocket();
    let f = run(&mut s, "cam.features", json!({}));
    let p = f["pockets"].as_array().unwrap().iter().find(|p| near(&p["floor_z"], -5.0)).cloned().unwrap_or_else(|| panic!("{f}"));
    assert!(near(&p["top_z"], 0.0) && near(&p["depth"], 5.0), "{p}");
    assert_eq!(p["walls_vertical"], false, "{p}");
    // Its round wall is a hole whose floor is the round: blind, not a hole coming out at the
    // round's top because the axis below it is air.
    let holes = f["holes"].as_array().unwrap();
    assert_eq!(holes.len(), 1, "{f}");
    assert!(holes[0]["through"] == false && holes[0]["floor"] == "round" && near(&holes[0]["bottom_z"], -3.5), "{f}");
    let sug = run(&mut s, "cam.suggest", json!({"tools": feature_tools()}));
    let id = p["id"].as_str().unwrap();
    let said = sug["check"].as_array().unwrap().iter().any(|c| c.as_str().unwrap().contains(id) && c.as_str().unwrap().contains("not upright"));
    assert!(said, "{sug}");
}

#[test]
fn heights_left_to_a_feature_follow_the_ones_given() {
    let mut s = feature_plate();
    let f = run(&mut s, "cam.features", json!({}));
    let top = f["faces"].as_array().unwrap().iter().find(|x| x["role"] == "top").unwrap()["id"].as_str().unwrap().to_string();
    let job =
        |ops: Value| json!({"job": {"machine": "3axis", "post": "linuxcnc_4axis", "tools": feature_tools(), "operations": ops}, "dry_run": true});
    // Stock 5 mm above the top, as the refusal for the top face asks: the retract and clearance
    // left out go above it, not 3 and 10 mm above the part.
    let r = run(&mut s, "cam.job", job(json!([{"name": "face", "type": "pocket", "feature": top, "tool": 6, "heights": {"top": 5}}])));
    assert_clean(&r);
    let r = run(&mut s, "cam.job", job(json!([{"name": "deep", "type": "pocket", "feature": "pocket:0", "tool": 6, "heights": {"retract": 20}}])));
    assert_clean(&r);
    // Heights given out of order name the values and which were given.
    let wrong = json!([{"name": "p", "type": "pocket", "feature": "pocket:0", "tool": 6, "heights": {"clearance": 5, "retract": 8}}]);
    let e = s.execute("cam.job", &job(wrong)).unwrap_err().to_string();
    assert!(e.contains("clearance 5 (given)") && e.contains("retract 8 (given)"), "{e}");
}

#[test]
fn facing_the_top_face_reaches_its_edges() {
    let mut s = feature_plate();
    let dir = Dir::new("face-top");
    let f = run(&mut s, "cam.features", json!({}));
    let top = f["faces"].as_array().unwrap().iter().find(|x| x["role"] == "top").unwrap()["id"].as_str().unwrap().to_string();
    let out = dir.file("face.ngc");
    let job = json!({"machine": "3axis", "post": "linuxcnc_4axis", "tools": feature_tools(),
        "operations": [{"name": "face", "type": "pocket", "feature": top, "tool": 6, "heights": {"top": 1}}]});
    let r = run(&mut s, "cam.job", json!({"job": job, "output": out}));
    assert_clean(&r);
    let c = run(&mut s, "cam.check", json!({"path": out}));
    // The 6 mm tool's centre goes out to the plate's edges (80 × 60), so its edge passes them.
    let b = &c["bounds"];
    assert!(b[0][0].as_f64().unwrap() <= -40.0 && b[1][0].as_f64().unwrap() >= 40.0, "{c}");
    assert!(b[0][1].as_f64().unwrap() <= -30.0 && b[1][1].as_f64().unwrap() >= 30.0, "{c}");
}

#[test]
fn default_heights_clear_the_fixtures() {
    // A clamp 25 mm tall beside the plate: travel between cuts goes over it.
    let mut s = feature_plate();
    run(&mut s, "solid.box", json!({"length": 10, "width": 20, "height": 35, "corner": [45, -10, -10], "body_name": "Clamp"}));
    let dir = Dir::new("fixture");
    let out = dir.file("pocket.ngc");
    let job = json!({"machine": "3axis", "post": "linuxcnc_4axis", "tools": feature_tools(), "fixtures": ["Clamp"],
        "operations": [{"name": "pocket", "type": "pocket", "feature": "pocket:0", "tool": 6}]});
    let r = run(&mut s, "cam.job", json!({"job": job, "output": out}));
    assert_clean(&r);
    let text = std::fs::read_to_string(&out).unwrap();
    assert!(max_z(&text) >= 30.0 - 1e-6, "the clearance must be above the clamp's top (25): {}", max_z(&text));
}

#[test]
fn a_feature_id_from_another_axis_says_which_axis_the_job_uses() {
    let mut s = pocketed_bar();
    let job = json!({"job": {"machine": "3axis", "post": "linuxcnc_4axis", "tools": feature_tools(),
        "operations": [{"name": "p", "type": "pocket", "feature": "pocket:2", "tool": 4}]}, "dry_run": true});
    let e = s.execute("cam.job", &job).unwrap_err().to_string();
    assert!(e.contains("along Z") && e.contains("4axis_a"), "{e}");
}

#[test]
fn a_profile_names_its_machine_file_beside_it() {
    let mut s = feature_plate();
    let dir = Dir::new("profile-machine");
    let machine = json!({"name": "Mill with A", "rotaries": [{"letter": "A", "direction": [1, 0, 0], "center": [0, 0, 0], "carrier": "table",
        "limits": [-std::f64::consts::TAU, std::f64::consts::TAU]}], "work_origin": [0, 0, 0]});
    std::fs::write(dir.file("mill.machine.json"), machine.to_string()).unwrap();
    let profile = dir.file("mill.profile.json");
    let with_file = json!({"tools": feature_tools(), "machine_path": "mill.machine.json", "post": "linuxcnc_4axis"});
    std::fs::write(&profile, with_file.to_string()).unwrap();
    let sug = run(&mut s, "cam.suggest", json!({"profile_path": profile, "check": true}));
    assert_eq!(sug["job"]["machine"]["name"], "Mill with A", "{sug}");
    assert_eq!(sug["dry_run"]["passed"], true, "{sug}");
    let both = json!({"tools": feature_tools(), "machine_path": "mill.machine.json", "machine": "3axis"});
    std::fs::write(&profile, both.to_string()).unwrap();
    let e = s.execute("cam.suggest", &json!({"profile_path": profile})).unwrap_err().to_string();
    assert!(e.contains("`machine` or `machine_path`"), "{e}");
}

#[test]
fn a_hole_left_out_says_what_tool_to_add_not_an_operation_that_is_not_there() {
    let mut s = feature_plate();
    let drill = json!({"number": 60, "kind": "drill", "diameter": 6, "flute_length": 30, "spindle_rpm": 3000, "feed": 300});
    let sug = run(&mut s, "cam.suggest", json!({"tools": [flat(6, 6.0), drill]}));
    let why = sug["skipped"].as_array().unwrap().iter().filter_map(Value::as_str).find(|x| x.starts_with("hole:0 ")).unwrap_or_default().to_string();
    assert!(why.contains("(4.8 mm)") && why.contains("\"feature\": \"hole:0\""), "{sug}");
    assert!(why.contains("no drilling operation") && !why.contains("drill it with"), "{why}");
}
