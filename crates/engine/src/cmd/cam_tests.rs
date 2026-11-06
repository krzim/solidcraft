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

// Drilling.

fn drill_tool(number: u32, diameter: f64) -> Value {
    json!({
        "number": number, "kind": "drill", "diameter": diameter, "flute_length": 30, "point_angle_deg": 118,
        "spindle_rpm": 6000, "feed": 300, "plunge_feed": 250
    })
}

/// The Z words of a program, in order.
fn z_words(text: &str) -> Vec<f64> {
    text.lines().flat_map(|l| l.split_whitespace().filter_map(|w| w.strip_prefix('Z')?.parse::<f64>().ok())).collect()
}

fn count_z(text: &str, z: f64) -> usize {
    z_words(text).into_iter().filter(|w| (w - z).abs() < 1e-6).count()
}

/// A 60 × 40 × 20 plate, its top at Z 0, with four 6 mm holes `depth` deep at (-15, 0),
/// (15, 0), (0, 10) and (0, -10), and a sketch `Holes` on its top: 6 mm circles at the first
/// three, a 30 mm circle round the middle (no hole), and a point at the fourth.
fn drilled_plate(depth: f64) -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 20, "corner": [-30, -20, -20], "body_name": "Plate"}));
    for [x, y] in [[-15, 0], [15, 0], [0, 10], [0, -10]] {
        run(&mut s, "solid.cylinder", json!({"diameter": 6, "height": depth + 1.0, "base": [x, y, -depth], "operation": "cut"}));
    }
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Holes"}));
    for [x, y] in [[-15, 0], [15, 0], [0, 10]] {
        run(&mut s, "sketch.circle.center", json!({"center": [x, y], "diameter": 6}));
    }
    run(&mut s, "sketch.circle.center", json!({"center": [0, 0], "diameter": 30}));
    run(&mut s, "sketch.point", json!({"point": [0, -10]}));
    run(&mut s, "sketch.finish", json!({}));
    s
}

fn drill_job(op: Value) -> Value {
    json!({"machine": "3axis", "post": "mach3_4axis", "tools": [drill_tool(3, 6.0)], "operations": [op]})
}

fn peck_op() -> Value {
    json!({
        "name": "holes", "type": "drill", "tool": 3, "sketch": "Holes", "diameter_range": [5, 7],
        "heights": {"clearance": 15, "retract": 2, "top": 0, "bottom": -10}, "cycle": "peck", "peck": 3
    })
}

#[test]
fn drilling_takes_the_holes_from_a_sketch_pecks_exactly_and_reads_back() {
    let mut s = drilled_plate(10.0);
    let dir = Dir::new("drill");
    let out = dir.file("holes.tap");
    let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
        run(&mut s, "cam.job", json!({"job": drill_job(peck_op()), "output": out}))
    });
    assert_clean(&r);
    let op = &r["operations"][0];
    assert_eq!(op["kind"], "drill", "{r}");
    // Four holes (three circles in the range and the point; not the 30 mm circle), each:
    // over it, down to R, four pecks to -3, -6, -9 and -10 with a full retract and a return to
    // 0.5 mm above the depth after the first three, out to R and up to clearance.
    assert_eq!(op["moves"], 4 * 14, "{r}");
    assert!((op["cut_length"].as_f64().unwrap() - 4.0 * (12.0 + 1.5)).abs() < 1e-9, "{r}");
    let text = std::fs::read_to_string(&out).unwrap();
    for (z, n) in [(-3.0, 4), (-6.0, 4), (-9.0, 4), (-10.0, 4), (-2.5, 4), (-5.5, 4), (-8.5, 4)] {
        assert_eq!(count_z(&text, z), n, "Z{z}: {text}");
    }
    assert!(z_words(&text).iter().all(|z| *z >= -10.0 - 1e-9), "never below the bottom: {text}");
    for canned in ["G73", "G81", "G82", "G83"] {
        assert!(!text.contains(canned), "expanded moves, no {canned}: {text}");
    }
    let c = run(&mut s, "cam.check", json!({"path": out}));
    assert!(c["errors"].as_array().is_some_and(Vec::is_empty), "{c}");
    assert_eq!(c["bounds"], json!([[-15.0, -10.0, -10.0], [15.0, 10.0, 15.0]]), "{c}");
    // The drill feeds at its plunge feed.
    assert!(text.lines().any(|l| l.contains("F250")), "{text}");
}

#[test]
fn a_drills_point_is_gouge_checked_and_full_diameter_depth_goes_deeper_by_it() {
    let full = |bottom: f64| {
        let mut op = peck_op();
        op["to_full_diameter"] = json!(true);
        op["heights"]["bottom"] = json!(bottom);
        drill_job(op)
    };
    // The holes are modelled flat-bottomed 10 mm deep: to full diameter at -10 the point is
    // 1.8 mm into their floors.
    let r = run(&mut drilled_plate(10.0), "cam.job", json!({"job": full(-10.0), "dry_run": true}));
    assert_eq!(r["passed"], false, "{r}");
    assert!(r["gouge"]["worst"].as_f64().is_some_and(|w| w > 1.5 && w < 2.1), "{r}");
    assert!(r["failures"].as_array().unwrap().iter().any(|f| f.as_str().unwrap_or_default().contains("gouge")), "{r}");
    // Modelled 12 deep, the point (to -11.8) clears the floor.
    let r = run(&mut drilled_plate(12.0), "cam.job", json!({"job": full(-10.0), "dry_run": true}));
    assert_clean(&r);
    // No holes in the model: the drill cuts where the part shows material.
    let mut solid = Session::default();
    run(&mut solid, "solid.box", json!({"length": 60, "width": 40, "height": 20, "corner": [-30, -20, -20]}));
    run(&mut solid, "sketch.create", json!({"plane": "XY", "name": "Holes"}));
    run(&mut solid, "sketch.circle.center", json!({"center": [0, 0], "diameter": 6}));
    run(&mut solid, "sketch.finish", json!({}));
    let r = run(&mut solid, "cam.job", json!({"job": drill_job(peck_op()), "dry_run": true}));
    assert_eq!(r["passed"], false, "{r}");
    assert!(r["gouge"]["worst"].as_f64().is_some_and(|w| w > 9.0), "{r}");
    // A flat end mill may plunge the holes; a ball end mill may not drill.
    let mut job = drill_job(peck_op());
    job["tools"] = json!([flat(3, 6.0)]);
    assert_clean(&run(&mut drilled_plate(10.0), "cam.job", json!({"job": job, "dry_run": true})));
    job["tools"] = json!([ball(3, 6.0)]);
    let e = drilled_plate(10.0).execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("drill, or a flat end mill"), "{e}");
}

#[test]
fn a_drill_indexed_at_90_degrees_drills_the_blocks_side() {
    // A block 40 × 30 × 30 about the X axis with two 5 mm holes 8 deep into its +Y face, and a
    // sketch on that face (its y running down work Z) with circles at them.
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 30, "corner": [-20, -15, -15], "body_name": "Block"}));
    for [x, z] in [[-8, 0], [8, 4]] {
        run(&mut s, "solid.cylinder", json!({"diameter": 5, "height": 9, "base": [x, 7, z], "axis": "Y", "operation": "cut"}));
    }
    let side = json!({"plane": {"origin": [0, 15, 0], "x_dir": [1, 0, 0], "y_dir": [0, 0, -1]}, "name": "Side holes"});
    run(&mut s, "sketch.create", side.clone());
    for [x, y] in [[-8, 0], [8, -4]] {
        run(&mut s, "sketch.circle.center", json!({"center": [x, y], "diameter": 5}));
    }
    run(&mut s, "sketch.finish", json!({}));
    let job = |sketch: &str| {
        json!({
            "machine": "4axis_a", "post": "mach3_4axis", "tools": [drill_tool(4, 5.0)],
            "operations": [{"type": "indexed", "a_deg": 90, "operation": {
                "name": "side holes", "type": "drill", "tool": 4, "sketch": sketch,
                "heights": {"clearance": 40, "retract": 18, "top": 15, "bottom": 7},
                "cycle": "chip_break", "peck": 2.5, "back_off": 0.3
            }}]
        })
    };
    let r = run(&mut s, "cam.job", json!({"job": job("Side holes"), "dry_run": true}));
    assert_clean(&r);
    let op = &r["operations"][0];
    assert_eq!(op["kind"], "indexed drill", "{r}");
    assert_eq!(op["a_deg"].as_f64(), Some(90.0), "{r}");
    assert!((op["max_tilt_deg"].as_f64().unwrap() - 90.0).abs() < 1e-6, "the drill comes in along work +Y: {r}");
    // Two holes, each: over it, to R, four pecks (to 12.5, 10, 7.5 and 7) with three
    // back-offs, out to R, up to clearance.
    assert_eq!(op["moves"], 2 * 11, "{r}");
    let head: Vec<&str> = r["program_head"].as_array().unwrap().iter().filter_map(Value::as_str).collect();
    assert!(head.iter().any(|l| l.contains("A90")), "{head:?}");
    // The holes are where the sketch says only if the frame maps them right: drilled into the
    // solid side, the same job gouges.
    let mut solid = Session::default();
    run(&mut solid, "solid.box", json!({"length": 40, "width": 30, "height": 30, "corner": [-20, -15, -15]}));
    run(&mut solid, "sketch.create", side);
    run(&mut solid, "sketch.circle.center", json!({"center": [-8, 0], "diameter": 5}));
    run(&mut solid, "sketch.finish", json!({}));
    let r = run(&mut solid, "cam.job", json!({"job": job("Side holes"), "dry_run": true}));
    assert_eq!(r["passed"], false, "{r}");
    // A sketch on the top is not square to the indexed tool axis.
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Top"}));
    run(&mut s, "sketch.circle.center", json!({"center": [0, 0], "diameter": 5}));
    run(&mut s, "sketch.finish", json!({}));
    let e = s.execute("cam.job", &json!({"job": job("Top"), "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("not on a plane square"), "{e}");
}

#[test]
fn drilling_stays_at_the_r_height_only_above_the_part_and_its_fixtures() {
    let mut s = drilled_plate(10.0);
    // A vise jaw beside the plate, 5 mm above its top.
    run(&mut s, "solid.box", json!({"length": 60, "width": 8, "height": 25, "corner": [-30, -30, -20], "operation": "new", "body_name": "Jaw"}));
    let job = |retract: f64| {
        let mut op = peck_op();
        op["stay_at_retract"] = json!(true);
        op["heights"]["retract"] = json!(retract);
        let mut j = drill_job(op);
        j["fixtures"] = json!(["Jaw"]);
        j
    };
    let e = s.execute("cam.job", &json!({"job": job(2.0), "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("stay_at_retract"), "{e}");
    let r = run(&mut s, "cam.job", json!({"job": job(8.0), "dry_run": true}));
    assert_clean(&r);
    // From hole to hole at R: no rise to clearance and back, except at the start and the end.
    assert_eq!(r["operations"][0]["moves"], 13 + 12 + 12 + 13, "{r}");
}

#[test]
fn hostile_drilling_jobs_are_refused_without_panics() {
    let mut s = drilled_plate(10.0);
    let with = |f: &dyn Fn(&mut Value)| {
        let mut op = peck_op();
        f(&mut op);
        json!({"job": drill_job(op), "dry_run": true})
    };
    let points_op = |extra: Value| {
        let mut op = json!({"name": "p", "type": "drill", "tool": 3, "heights": {"clearance": 15, "retract": 2, "top": 0, "bottom": -10}});
        if let (Some(o), Value::Object(e)) = (op.as_object_mut(), extra) {
            o.extend(e);
        }
        json!({"job": drill_job(op), "dry_run": true})
    };
    let cases = [
        (with(&|o| o["sketch"] = json!("Nope")), "Nope"),
        (with(&|o| o["points"] = json!([[0, 0]])), "not both"),
        (points_op(json!({})), "give a `sketch`"),
        (points_op(json!({"points": []})), "no holes"),
        (points_op(json!({"points": [[1e308, 0]]})), "finite"),
        (points_op(json!({"points": [[0, 0]], "circles": true})), "go with `sketch`"),
        (with(&|o| o["diameter_range"] = json!([7, 5])), "smallest first"),
        (with(&|o| o["diameter_range"] = json!([-1, 5])), "smallest first"),
        (with(&|o| o["diameter_range"] = json!([0, 1e308])), "finite"),
        (
            with(&|o| {
                o["diameter_range"] = json!([50, 60]);
                o["sketch_points"] = json!(false);
            }),
            "no circles in `diameter_range`",
        ),
        (
            with(&|o| {
                o["circles"] = json!(false);
                o["sketch_points"] = json!(false);
            }),
            "no holes",
        ),
        (with(&|o| o["cycle"] = json!("simple")), "`peck` goes with"),
        (with(&|o| o["dwell_seconds"] = json!(1)), "`dwell_seconds` goes with"),
        (with(&|o| o["back_off"] = json!(1)), "`back_off` goes with"),
        (with(&|o| o["peck"] = json!(1e-9)), "pecks"),
        (with(&|o| o["peck"] = json!(-1)), "peck"),
        (with(&|o| o["peck_clearance"] = json!(0)), "clearance"),
        (with(&|o| o["cycle"] = json!("pecking")), "pecking"),
        (with(&|o| o["pek"] = json!(1)), "pek"),
        (
            with(&|o| {
                o["stay_at_retract"] = json!(true);
                o["heights"]["retract"] = json!(0);
            }),
            "stay_at_retract",
        ),
        (with(&|o| o["heights"]["bottom"] = json!(-40)), "flutes"),
        (with(&|o| o["heights"]["bottom"] = json!(5)), "ordered"),
        (with(&|o| o["heights"]["bottom"] = json!(0)), "no depth"),
        (with(&|o| o["feeds"] = json!({"cut": 100, "plunge": -1})), "feed"),
    ];
    for (p, word) in &cases {
        match s.execute("cam.job", p) {
            Err(EngineError::Internal(_, msg)) => panic!("{p}: panicked: {msg}"),
            Err(e) => assert!(e.to_string().contains(word), "{p}: {e}"),
            Ok(v) => panic!("{p}: accepted: {v}"),
        }
    }
    // A drill's point angle is checked; a drill still cannot mill.
    let mut job = drill_job(peck_op());
    job["tools"][0]["point_angle_deg"] = json!(0);
    assert!(s.execute("cam.job", &json!({"job": job, "dry_run": true})).is_err());
    let mut job = drill_job(json!({
        "name": "c", "type": "contour", "tool": 3, "polygon": [[-10, -10], [10, -10], [10, 10], [-10, 10]],
        "heights": {"clearance": 15, "retract": 2, "top": 0, "bottom": -1}
    }));
    let e = s.execute("cam.job", &json!({"job": job.clone(), "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("cannot mill"), "{e}");
    job["operations"][0]["type"] = json!("drill");
    assert!(s.execute("cam.job", &json!({"job": job, "dry_run": true})).is_err(), "a drill operation has no polygon");
}

#[test]
fn drilling_refuses_an_r_height_below_the_part_round_a_hole() {
    // A plate, its top at Z 0, with a 20 × 20 boss 10 high round (15, 0), and 6 mm holes 12 deep
    // into the plate at (-15, 0) and through the boss into the plate at (15, 0).
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 20, "corner": [-30, -20, -20], "body_name": "Plate"}));
    run(&mut s, "solid.box", json!({"length": 20, "width": 20, "height": 10, "corner": [5, -10, 0], "operation": "join"}));
    for [x, y] in [[-15, 0], [15, 0]] {
        run(&mut s, "solid.cylinder", json!({"diameter": 6, "height": 25, "base": [x, y, -12], "operation": "cut"}));
    }
    let job = |points: Value, top: f64, retract: f64| {
        let op = json!({
            "name": "holes", "type": "drill", "tool": 3, "points": points,
            "heights": {"clearance": 25, "retract": retract, "top": top, "bottom": -10}
        });
        json!({"job": drill_job(op), "dry_run": true})
    };
    // Both holes with the plate's heights: the drill would rapid from clearance down to R 2
    // through 8 mm of the boss. The gouge check cannot see it (the finished part has the hole).
    let e = s.execute("cam.job", &job(json!([[-15, 0], [15, 0]]), 0.0, 2.0)).unwrap_err();
    assert!(e.to_string().contains("(15, 0)") && e.to_string().contains("retract"), "{e}");
    // The plate's hole alone, and the boss's from the boss's top, are clean.
    assert_clean(&run(&mut s, "cam.job", job(json!([[-15, 0]]), 0.0, 2.0)));
    assert_clean(&run(&mut s, "cam.job", job(json!([[15, 0]]), 10.0, 12.0)));
    // Indexed: the part is checked in the operation's frame. At A 90 the block's +Y face (work Y
    // 15) is under the tool; an R height of 12 is inside the block.
    let mut b = Session::default();
    run(&mut b, "solid.box", json!({"length": 40, "width": 30, "height": 30, "corner": [-20, -15, -15], "body_name": "Block"}));
    run(&mut b, "solid.cylinder", json!({"diameter": 5, "height": 9, "base": [0, 7, 0], "axis": "Y", "operation": "cut"}));
    let side = |retract: f64| {
        json!({"job": {
            "machine": "4axis_a", "post": "mach3_4axis", "tools": [drill_tool(4, 5.0)],
            "operations": [{"type": "indexed", "a_deg": 90, "operation": {
                "name": "side", "type": "drill", "tool": 4, "points": [[0, 0]],
                "heights": {"clearance": 40, "retract": retract, "top": 10, "bottom": 8}
            }}]
        }, "dry_run": true})
    };
    let e = b.execute("cam.job", &side(12.0)).unwrap_err();
    assert!(e.to_string().contains("Z 15") && e.to_string().contains("retract"), "{e}");
    assert_clean(&run(&mut b, "cam.job", side(18.0)));
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
    // Up from -8 to 0, counter-clockwise: the lead-in and lead-out each rise a quarter pitch
    // (a quarter turn about the axis) and every half turn of the helix exactly half a pitch.
    assert_eq!(z.len(), 17, "{text}");
    assert!(z.iter().all(|(_, g)| *g == 3), "{z:?}");
    let mut last = -8.0;
    for (k, (zk, _)) in z.iter().enumerate() {
        let rise = if k == 0 || k == z.len() - 1 { 0.25 } else { 0.5 };
        assert!((zk - last - rise).abs() < 1e-9, "{z:?}");
        last = *zk;
    }
    assert!((last - 0.0).abs() < 1e-9, "{z:?}");
    let turns: Vec<f64> = z[..16].chunks(2).filter_map(|t| t.first().map(|(zk, _)| *zk)).collect();
    assert!(turns.windows(2).all(|w| (w[1] - w[0] - 1.0).abs() < 1e-9), "one pitch per turn: {turns:?}");
    // The program names the thread mill itself; only the checks take its core.
    assert!(text.contains("T2 D4.5 thread mill"), "{text}");
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

/// A 40 mm block, its top at Z 0, with a 24 mm square pocket 3 mm deep.
fn pocket_block() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 40, "height": 10, "corner": [-20, -20, -10], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 24, "width": 24, "height": 3, "corner": [-12, -12, -3], "operation": "cut"}));
    s
}

fn adaptive_job(op: Value) -> Value {
    json!({"name": "adaptive", "machine": "3axis", "post": "mach3_4axis", "tools": [flat(1, 6.0)], "operations": [op]})
}

fn adaptive_pocket_op() -> Value {
    json!({"name": "clear", "type": "adaptive", "tool": 1, "polygon": [[-12, -12], [12, -12], [12, 12], [-12, 12]],
           "heights": {"clearance": 20, "retract": 2, "top": 0, "bottom": -3}, "optimal_load": 0.6, "max_engagement_deg": 60})
}

/// The adaptive pocket is written, read back and gouge checked, never slots outside its helix,
/// and is reported beside a plain offset pocket that does.
#[test]
fn an_adaptive_pocket_reads_back_and_reports_its_engagement() {
    let mut s = pocket_block();
    let r = run(&mut s, "cam.job", json!({"job": adaptive_job(adaptive_pocket_op()), "dry_run": true}));
    assert_clean(&r);
    let op = &r["operations"][0];
    assert_eq!(op["kind"], "adaptive", "{op}");
    let a = &op["adaptive"];
    assert!(a["max_engagement_deg"].as_f64().is_some_and(|m| m > 20.0 && m <= 63.0), "{a}");
    assert_eq!(a["full_slot_share"].as_f64(), Some(0.0), "{a}");
    assert_eq!(a["limit_deg"].as_f64().map(f64::round), Some(60.0), "{a}");
    assert!(a["helices"].as_u64().is_some_and(|h| h >= 1), "{a}");
    let p = &a["pocket"];
    assert!(p["max_engagement_deg"].as_f64().is_some_and(|m| m >= 170.0), "{a}");
    assert!(p["full_slot_share"].as_f64().is_some_and(|f| f > 0.0), "{a}");
    assert!(r["program_lines"].as_u64().is_some_and(|n| n > 100), "{r}");
}

/// Without engagement keys the limit follows the load (10% of the diameter here: 36.9° on a
/// straight edge, the limit half as much again); `compare: false` plans no pocket.
#[test]
fn an_adaptive_pocket_takes_its_limits_from_the_load() {
    let mut s = pocket_block();
    let mut op = adaptive_pocket_op();
    op.as_object_mut().unwrap().remove("max_engagement_deg");
    op["compare"] = json!(false);
    op["stepdown"] = json!(1.5);
    let r = run(&mut s, "cam.job", json!({"job": adaptive_job(op), "dry_run": true}));
    assert_clean(&r);
    let a = &r["operations"][0]["adaptive"];
    assert!(a["limit_deg"].as_f64().is_some_and(|l| (l - 55.3).abs() < 0.1), "{a}");
    assert!(a.get("pocket").is_none(), "{a}");
    assert_eq!(a["levels"].as_u64(), Some(2), "{a}");
}

/// 3D adaptive roughing of the dome over its block: entered from open air, read back and
/// gouge checked, within the engagement limit.
#[test]
fn adaptive_roughing_of_the_dome_reads_back_and_passes_the_gouge_check() {
    let mut s = dome();
    let op = json!({"name": "rough", "type": "adaptive_roughing", "tool": 1,
                    "heights": {"clearance": 30, "retract": 10, "top": 7, "bottom": 0.25}, "stepdown": 2.5,
                    "optimal_load": 0.6, "max_engagement_deg": 60, "boundary": {"rect": [[-20, -20], [20, 20]]}});
    let r = run(&mut s, "cam.job", json!({"job": adaptive_job(op), "dry_run": true}));
    assert_clean(&r);
    let op = &r["operations"][0];
    assert_eq!(op["kind"], "adaptive_roughing", "{op}");
    let a = &op["adaptive"];
    assert!(a["max_engagement_deg"].as_f64().is_some_and(|m| m > 10.0 && m <= 63.0), "{a}");
    assert_eq!(a["full_slot_share"].as_f64(), Some(0.0), "{a}");
    assert!(a["levels"].as_u64().is_some_and(|l| l >= 3), "{a}");
    assert_eq!(a["helices"].as_u64(), Some(0), "{a}");
}

/// Hostile adaptive operations are refused with an error, never a panic or a hang.
#[test]
fn hostile_adaptive_operations_are_refused() {
    let mut s = pocket_block();
    let with = |key: &str, v: Value| {
        let mut op = adaptive_pocket_op();
        op[key] = v;
        json!({"job": adaptive_job(op), "dry_run": true})
    };
    let rough = |extra: Value| {
        let mut op =
            json!({"name": "r", "type": "adaptive_roughing", "tool": 1, "heights": {"clearance": 30, "retract": 10, "top": 0, "bottom": -1}});
        if let (Some(o), Some(e)) = (op.as_object_mut(), extra.as_object()) {
            o.extend(e.clone());
        }
        json!({"job": adaptive_job(op), "dry_run": true})
    };
    let cases = [
        with("optimal_load", json!(-1)),
        with("optimal_load", json!(7)),
        with("max_engagement_deg", json!(1)),
        with("max_engagement_deg", json!(500)),
        with("min_engagement_deg", json!(90)),
        with("helix_diameter", json!(20)),
        with("ramp_angle_deg", json!(0)),
        with("stepdown", json!(0)),
        with("tolerance", json!(0)),
        with("optimal_lode", json!(1)),
        with("polygon", json!([[-1e5, -1e5], [1e5, -1e5], [1e5, 1e5], [-1e5, 1e5]])),
        with("polygon", json!([[0, 0], [1, 0]])),
        rough(json!({"heights": {"clearance": 30, "retract": 0.1, "top": 0, "bottom": -1}})),
        rough(json!({"boundary": {"rect": [[-1e5, -1e5], [1e5, 1e5]]}})),
        rough(json!({"flats": "yes"})),
    ];
    let start = std::time::Instant::now();
    for p in &cases {
        match s.execute("cam.job", p) {
            Err(EngineError::Internal(_, msg)) => panic!("{p}: panicked: {msg}"),
            Err(_) => {}
            Ok(v) => panic!("{p}: accepted: {v}"),
        }
    }
    assert!(start.elapsed().as_secs() < 60, "{:?}", start.elapsed());
    // A helix with no room is refused, saying so.
    let e = s.execute("cam.job", &with("polygon", json!([[-12, -3.5], [12, -3.5], [12, 3.5], [-12, 3.5]]))).unwrap_err();
    assert!(e.to_string().contains("no room for the entry helix"), "{e}");
}

/// A full-width load capped at 55°: the default least engagement (a third of the load's, 60°)
/// would be above the limit, so it is kept below it instead of the operation being refused for
/// a key it was not given.
#[test]
fn an_adaptive_pocket_takes_a_limit_below_a_third_of_the_load() {
    assert!(super::adaptive::limits(3.0, 6.0, None, Some(55.0)).0 < 55.0);
    let mut s = pocket_block();
    let op = json!({"name": "capped", "type": "adaptive", "tool": 1, "polygon": [[-7, -7], [7, -7], [7, 7], [-7, 7]],
                    "heights": {"clearance": 20, "retract": 2, "top": -2, "bottom": -3}, "optimal_load": 6,
                    "max_engagement_deg": 55, "compare": false});
    let r = run(&mut s, "cam.job", json!({"job": adaptive_job(op), "dry_run": true}));
    assert_clean(&r);
    let a = &r["operations"][0]["adaptive"];
    assert_eq!(a["measured"], true, "{a}");
    assert!(a["max_engagement_deg"].as_f64().is_some_and(|m| m <= 58.0), "{a}");
}

/// A measured path above the limit, or in a full slot outside its entries, is warned about; an
/// unmeasured one says nothing it does not know.
#[test]
fn adaptive_engagement_above_the_limit_is_warned_about() {
    use crate::schema::cam_adaptive::AdaptiveReport;
    let ok = AdaptiveReport { measured: true, max_engagement_deg: 61.0, limit_deg: 60.0, ..AdaptiveReport::default() };
    assert!(super::adaptive::engagement_warnings(&ok).is_empty());
    let over = AdaptiveReport { max_engagement_deg: 75.0, ..ok.clone() };
    assert!(super::adaptive::engagement_warnings(&over).iter().any(|w| w.contains("above the limit")));
    let slot = AdaptiveReport { full_slot_share: 0.02, ..ok.clone() };
    assert!(super::adaptive::engagement_warnings(&slot).iter().any(|w| w.contains("full slot")));
    let unmeasured = AdaptiveReport { measured: false, ..over };
    assert!(super::adaptive::engagement_warnings(&unmeasured).is_empty());
}

/// Adaptive clearing of the side pocket at A = 90°: written with A held, read back and gouge
/// checked in the turned frame, within the engagement limit.
#[test]
fn an_adaptive_pocket_indexed_at_90_degrees_reads_back() {
    let mut s = pocketed_block();
    let job = json!({
        "machine": "4axis_a", "post": "mach3_4axis", "tools": [flat(1, 4.0)],
        "operations": [{"type": "indexed", "a_deg": 90, "operation": {
            "name": "side adaptive", "type": "adaptive", "tool": 1, "polygon": [[-6, -6], [6, -6], [6, 6], [-6, 6]],
            "heights": {"clearance": 40, "retract": 20, "top": 15, "bottom": 10}, "stepdown": 2.5,
            "optimal_load": 0.4, "max_engagement_deg": 60, "compare": false}}]
    });
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    let op = &r["operations"][0];
    assert_eq!(op["kind"], "indexed adaptive", "{r}");
    assert!((op["max_tilt_deg"].as_f64().unwrap() - 90.0).abs() < 1e-6, "{r}");
    let a = &op["adaptive"];
    assert_eq!(a["measured"], true, "{a}");
    assert_eq!(a["levels"].as_u64(), Some(2), "{a}");
    assert!(a["max_engagement_deg"].as_f64().is_some_and(|m| m > 10.0 && m <= 63.0), "{a}");
    assert_eq!(a["full_slot_share"].as_f64(), Some(0.0), "{a}");
    let head: Vec<&str> = r["program_head"].as_array().unwrap().iter().filter_map(Value::as_str).collect();
    assert!(head.iter().any(|l| l.contains("A90")), "{head:?}");
}

/// Adaptive roughing with no boundary takes the part's extent as the stock and says so; the
/// engagement is measured inside both the boundary's rectangle and its polygon.
#[test]
fn adaptive_roughing_says_when_it_takes_the_stock_from_the_part() {
    let mut s = dome();
    let base = json!({"name": "rough", "type": "adaptive_roughing", "tool": 1,
                      "heights": {"clearance": 30, "retract": 10, "top": 7, "bottom": 3}, "stepdown": 4,
                      "optimal_load": 0.6, "max_engagement_deg": 60});
    let r = run(&mut s, "cam.job", json!({"job": adaptive_job(base.clone()), "dry_run": true}));
    assert_clean(&r);
    let op = &r["operations"][0];
    assert!(op["warnings"].as_array().unwrap().iter().any(|w| w.as_str().is_some_and(|w| w.contains("the part's extent"))), "{op}");
    // A polygon larger than the rectangle: the stock is where both are, and so is the material
    // the engagement is measured in (the polygon alone would show engagement in open air).
    let mut op = base;
    op["boundary"] = json!({"rect": [[-20, -20], [20, 20]], "polygon": [[-40, -40], [40, -40], [40, 40], [-40, 40]]});
    let r = run(&mut s, "cam.job", json!({"job": adaptive_job(op), "dry_run": true}));
    assert_clean(&r);
    let op = &r["operations"][0];
    assert!(!op["warnings"].as_array().unwrap().iter().any(|w| w.as_str().is_some_and(|w| w.contains("the part's extent"))), "{op}");
    let a = &op["adaptive"];
    assert_eq!(a["measured"], true, "{a}");
    assert!(a["max_engagement_deg"].as_f64().is_some_and(|m| m <= 63.0), "{a}");
    assert_eq!(a["full_slot_share"].as_f64(), Some(0.0), "{a}");
}

/// A block 30 × 20 × 10, its top at Z 0, with its four top edges chamfered 1 mm at 45° when
/// `chamfered`.
fn plate(chamfered: bool) -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 30, "width": 20, "height": 10, "corner": [-15, -10, -10], "body_name": "Plate"}));
    if chamfered {
        run(&mut s, "solid.chamfer", json!({"edges": [[0, -10, 0], [15, 0, 0], [0, 10, 0], [-15, 0, 0]], "distance": 1, "tangent_chain": false}));
    }
    s
}

fn chamfer_mill(number: u32) -> Value {
    json!({"number": number, "kind": "chamfer", "diameter": 6, "tip_diameter": 0, "angle_deg": 90, "flute_length": 10, "flutes": 3,
           "spindle_rpm": 20000, "feed": 1600, "plunge_feed": 100})
}

fn face_mill(number: u32) -> Value {
    json!({"number": number, "kind": "face_mill", "diameter": 10, "corner_radius": 0.2, "flute_length": 6, "flutes": 1,
           "spindle_rpm": 18000, "feed": 1600, "plunge_feed": 50})
}

/// Faces the plate's top with a face mill, then chamfers its top edges `width` wide.
fn chamfer_job(width: f64) -> Value {
    json!({
        "machine": "3axis",
        "post": "mach3_4axis",
        "tools": [face_mill(1), chamfer_mill(7)],
        "operations": [
            {"name": "face", "type": "pocket", "tool": 1, "polygon": [[-14, -9], [14, -9], [14, 9], [-14, 9]],
             "heights": {"clearance": 10, "retract": 2, "top": 0.5, "bottom": 0}, "stepover": 7, "ramp_angle_deg": 2},
            {"name": "deburr", "type": "chamfer", "tool": 7, "polygon": [[-15, -10], [15, -10], [15, 10], [-15, 10]],
             "heights": {"clearance": 10, "retract": 2, "top": 0}, "width": width}
        ]
    })
}

#[test]
fn a_chamfer_mill_cuts_the_modelled_chamfer_as_wide_as_asked() {
    let mut s = plate(true);
    let dir = Dir::new("chamfer");
    // The tool list of a post that writes one names both tools as the approved post does.
    let mut post = run(&mut s, "cam.posts", json!({"name": "mach3_4axis"}))["post"].clone();
    post["base"]["tool_list"] = json!(true);
    post["base"]["upper_case_comments"] = json!(true);
    post["base"]["comment_chars"] = json!(" .,=_-");
    post["base"]["trailing_decimal"] = json!(true);
    post["base"]["decimals"] = json!(3);
    std::fs::write(dir.0.join("mill.post.json"), post.to_string()).unwrap();
    let mut job = chamfer_job(1.0);
    job.as_object_mut().unwrap().remove("post");
    job["post_path"] = json!(dir.file("mill.post.json"));
    let out = dir.file("plate.tap");
    let r = run(&mut s, "cam.job", json!({"job": job, "output": out}));
    assert_clean(&r);
    assert_eq!(r["operations"][1]["kind"], "chamfer", "{r}");
    let text = std::fs::read_to_string(&out).unwrap();
    assert!(text.lines().any(|l| l.starts_with("(T1 D=10. CR=0.2 - ZMIN=") && l.ends_with(" - FACE MILL)")), "{text}");
    assert!(text.contains("(T7 D=6. CR=0. - ZMIN=-1.2 - CHAMFER MILL)"), "{text}");
    // The width as the program cuts it: along the +X side, the 90° cone's radius at the top,
    // 1.2 above its tip, less the axis's distance from the wall.
    let blocks = solidcraft_cam::gcode5::parse5(&text, &[]).unwrap();
    let cuts: Vec<solidcore_record::Vec3> = blocks
        .iter()
        .filter_map(|b| match b {
            solidcraft_cam::gcode5::JointWords::Feed { linear, .. } => Some(*linear),
            _ => None,
        })
        .collect();
    let bottom = cuts.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    assert!((bottom + 1.2).abs() < 1e-6, "{bottom}");
    let side: Vec<f64> =
        cuts.iter().filter(|p| (p.z - bottom).abs() < 1e-6 && p.x > 7.5 && p.y.abs() <= 10.0 + 1e-6).map(|p| 1.2 - (p.x - 15.0)).collect();
    assert!(!side.is_empty(), "{text}");
    for w in side {
        assert!((w - 1.0).abs() <= 1e-3, "the chamfer is {w} mm wide, not 1");
    }
    // A chamfer wider than modelled cuts into the part.
    let r = run(&mut s, "cam.job", json!({"job": chamfer_job(1.2), "dry_run": true}));
    assert_eq!(r["passed"], false, "{r}");
    assert!(r["operations"][1]["gouge"].as_f64().is_some_and(|g| g > 0.1), "{r}");
}

#[test]
fn a_chamfer_the_model_does_not_show_is_a_gouge_unless_unchecked() {
    let mut s = plate(false);
    let r = run(&mut s, "cam.job", json!({"job": chamfer_job(1.0), "dry_run": true}));
    assert_eq!(r["passed"], false, "{r}");
    assert!(r["operations"][1]["gouge"].as_f64().is_some_and(|g| g > 0.5), "the cone cuts the sharp edge 1 mm deep: {r}");
    assert_eq!(r["operations"][0]["gouge"].as_f64(), Some(0.0), "the face mill stays on the top: {r}");
    let mut job = chamfer_job(1.0);
    job["operations"][1]["gouge_check"] = json!(false);
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["gouge"]["skipped"], json!(["deburr"]), "{r}");
}

#[test]
fn hostile_chamfers_and_chamfer_mills_are_refused() {
    let mut s = plate(true);
    let refused = |s: &mut Session, job: Value, needle: &str| {
        let e = s.execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
        assert!(matches!(e, EngineError::BadParams { .. }) && e.to_string().contains(needle), "{needle}: {e}");
    };
    // Wider than the cone reaches: it ends 3 out, the axis runs 0.2 out with the default tip offset.
    refused(&mut s, chamfer_job(2.9), "widest chamfer is 2.8");
    let mut job = chamfer_job(1.0);
    job["operations"][1]["side"] = json!("on");
    refused(&mut s, job, "not `on`");
    let mut job = chamfer_job(1.0);
    job["operations"][1]["tool"] = json!(1);
    refused(&mut s, job, "chamfer mill");
    let mut job = chamfer_job(1.0);
    job["operations"][1]["depth"] = json!(1);
    refused(&mut s, job, "depth");
    let mut job = chamfer_job(1.0);
    job["operations"][1]["heights"]["bottom"] = json!(-1);
    refused(&mut s, job, "bottom");
    let mut job = chamfer_job(1.0);
    job["operations"][0]["tool"] = json!(7);
    refused(&mut s, job, "tapered");
    for (k, v) in [("angle_deg", json!(180)), ("angle_deg", json!(null)), ("tip_diameter", json!(6)), ("tip_diameter", json!(-1))] {
        let mut job = chamfer_job(1.0);
        job["tools"][1][k] = v;
        refused(&mut s, job, "chamfer mill");
    }
    let mut job = chamfer_job(1.0);
    job["tools"][0]["angle_deg"] = json!(90);
    refused(&mut s, job, "for chamfer mills");
    for (w, t) in [(json!(-1), json!(0.2)), (json!(0), json!(0.2)), (json!(1e300), json!(0.2)), (json!(1), json!(-0.5)), (json!(1), json!(1e300))] {
        let mut job = chamfer_job(1.0);
        job["operations"][1]["width"] = w;
        job["operations"][1]["tip_offset"] = t;
        assert!(s.execute("cam.job", &json!({"job": job, "dry_run": true})).is_err());
    }
    // Indexed, the chamfer runs in the frame the A angle brings under the tool.
    let mut job = chamfer_job(1.0);
    job["machine"] = json!("4axis_a");
    let inner = job["operations"][1].take();
    job["operations"] = json!([{"type": "indexed", "a_deg": 0, "operation": inner}]);
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["operations"][0]["kind"], "indexed chamfer", "{r}");
}

#[test]
fn face_mills_ramp_in_and_chamfer_mills_never_rough() {
    // A face mill's inserts do not cut at its centre: a pocket that plunges it straight down is
    // refused, and with a ramp it runs clean. A chamfer mill's point leaves ridges between
    // roughing passes, so roughing with it is refused; so is roughing with a face mill.
    let mut s = plate(true);
    let refused = |s: &mut Session, job: Value, needle: &str| {
        let e = s.execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
        assert!(matches!(e, EngineError::BadParams { .. }) && e.to_string().contains(needle), "{needle}: {e}");
    };
    let mut job = chamfer_job(1.0);
    job["operations"][0].as_object_mut().unwrap().remove("ramp_angle_deg");
    refused(&mut s, job, "ramp");
    let r = run(&mut s, "cam.job", json!({"job": chamfer_job(1.0), "dry_run": true}));
    assert_clean(&r);
    for tool in [7, 1] {
        let mut job = chamfer_job(1.0);
        job["operations"] = json!([{"name": "rough", "type": "roughing", "tool": tool, "heights": {"clearance": 10, "retract": 2, "top": 1, "bottom": -3},
                                    "stepdown": 1, "stepover": 2, "boundary": {"rect": [[-18, -13], [18, 13]]}}]);
        refused(&mut s, job, "rough with an end mill");
    }
}

/// The motion blocks of a written 3-axis program.
fn blocks(path: &str) -> Vec<solidcraft_cam::gcode5::JointWords> {
    let text = std::fs::read_to_string(path).unwrap();
    solidcraft_cam::gcode5::parse5(&text, &[]).unwrap()
}

/// The tip after each motion block, and whether the block was a rapid.
fn tips(path: &str) -> Vec<(solidcore_record::Vec3, bool)> {
    use solidcraft_cam::gcode5::JointWords;
    blocks(path)
        .into_iter()
        .filter_map(|b| match b {
            JointWords::Rapid { linear, .. } => Some((linear, true)),
            JointWords::Feed { linear, .. } | JointWords::Arc { linear, .. } => Some((linear, false)),
            JointWords::Dwell { .. } => None,
        })
        .collect()
}

/// A 40 × 30 plate 6 mm thick, its top at Z 0.
fn leads_plate() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 6, "corner": [-20, -15, -6], "body_name": "Plate"}));
    s
}

fn profile_job(extra: Value) -> Value {
    let mut op = json!({
        "name": "profile", "type": "contour", "tool": 1,
        "polygon": [[-20, -15], [20, -15], [20, 15], [-20, 15]],
        "heights": {"clearance": 20, "retract": 5, "top": 0, "bottom": -6}, "stepdown": 2
    });
    for (k, v) in extra.as_object().unwrap() {
        op[k] = v.clone();
    }
    json!({"machine": "3axis", "post": "mach3_4axis", "tools": [flat(1, 6.0)], "operations": [op]})
}

#[test]
fn a_contour_with_leads_tabs_and_finishing_passes_reads_back_and_misses_the_part() {
    let mut s = leads_plate();
    let dir = Dir::new("profile");
    let out = dir.file("profile.tap");
    let job = profile_job(json!({
        "lead": {"radius": 2, "line": 1},
        "entry": {"type": "ramp", "angle_deg": 3},
        "tabs": {"count": 4, "width": 4, "height": 1.5},
        "finish_passes": 1, "finish_stepover": 0.25, "spring_pass": true,
        "linking": "stay_down"
    }));
    let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
        run(&mut s, "cam.job", json!({"job": job, "output": out}))
    });
    assert_clean(&r);
    let text = std::fs::read_to_string(&out).unwrap();
    assert!(text.lines().any(|l| l.contains("G2") || l.contains("G3")), "the leads are written as arcs");
    let pts = tips(&out);
    // The tabs: the tool climbs to 1.5 above the bottom four times on each pass at the bottom
    // (the last depth pass, the finishing pass and the spring pass).
    let mut climbs = 0;
    for w in pts.windows(2) {
        if !w[1].1 && w[0].0.z < -4.5 - 1e-3 && (w[1].0.z + 4.5).abs() < 1e-3 {
            climbs += 1;
        }
    }
    assert_eq!(climbs, 3 * 4, "{text}");
    // Never inside the tool's radius of the plate below its top; the passes between levels and
    // the finishing passes go across at the retract height, above the part.
    let outside = |p: solidcore_record::Vec3| (p.x.clamp(-20.0, 20.0) - p.x).hypot(p.y.clamp(-15.0, 15.0) - p.y);
    assert!(pts.iter().filter(|(p, _)| p.z < 0.0).all(|(p, _)| outside(*p) >= 3.0 - 2e-3), "{text}");
    let hops = pts.windows(2).filter(|w| w[1].1 && (w[1].0.x - w[0].0.x).hypot(w[1].0.y - w[0].0.y) > 1e-6 && (w[1].0.z - 5.0).abs() < 1e-6).count();
    assert!(hops >= 3, "{text}");
    let c = run(&mut s, "cam.check", json!({"path": out}));
    assert!(c["errors"].as_array().is_some_and(Vec::is_empty), "{c}");
    assert!(c["arcs"].as_u64().is_some_and(|n| n >= 8), "{c}");
}

/// A 40 × 30 block, its top at Z 0, with a 16 × 12 pocket 4 mm deep in it.
fn top_pocket() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 10, "corner": [-20, -15, -10], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 16, "width": 12, "height": 4, "corner": [-8, -6, -4], "operation": "cut"}));
    s
}

#[test]
fn a_pocket_goes_in_by_a_helix_and_never_plunges_into_uncut_material() {
    let mut s = top_pocket();
    let dir = Dir::new("helix");
    let out = dir.file("pocket.tap");
    let job = json!({
        "machine": "3axis", "post": "mach3_4axis", "tools": [flat(1, 4.0)],
        "operations": [{"name": "pocket", "type": "pocket", "tool": 1, "polygon": [[-8, -6], [8, -6], [8, 6], [-8, 6]],
                        "heights": {"clearance": 20, "retract": 3, "top": 0, "bottom": -4}, "stepdown": 2, "stepover": 1.5}]
    });
    let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
        run(&mut s, "cam.job", json!({"job": job, "output": out}))
    });
    assert_clean(&r);
    let pts = tips(&out);
    // Helical arcs: cuts that go down while going round.
    let blocks = blocks(&out);
    let mut last: Option<solidcore_record::Vec3> = None;
    let mut helical = 0;
    for b in &blocks {
        if let (solidcraft_cam::gcode5::JointWords::Arc { linear, .. }, Some(p)) = (b, last)
            && linear.z < p.z - 1e-6
        {
            helical += 1;
        }
        last = match b {
            solidcraft_cam::gcode5::JointWords::Rapid { linear, .. }
            | solidcraft_cam::gcode5::JointWords::Feed { linear, .. }
            | solidcraft_cam::gcode5::JointWords::Arc { linear, .. } => Some(*linear),
            solidcraft_cam::gcode5::JointWords::Dwell { .. } => last,
        };
    }
    assert!(helical >= 4, "{blocks:?}");
    // Every straight-down cut below the top ends no lower than what was already cut there.
    let mut deepest: Vec<(f64, f64, f64)> = Vec::new();
    for w in pts.windows(2) {
        let (a, b) = (w[0].0, w[1].0);
        let here = |x: f64, y: f64| deepest.iter().filter(|d| (d.0 - x).hypot(d.1 - y) < 1e-3).map(|d| d.2).fold(f64::INFINITY, f64::min);
        if !w[1].1 && (b.x - a.x).hypot(b.y - a.y) < 1e-9 && b.z < a.z && b.z < -1e-6 {
            assert!(b.z >= here(b.x, b.y) - 1e-3, "a plunge into uncut material at {b:?}");
        }
        if !w[1].1 {
            deepest.push((b.x, b.y, b.z));
        }
    }
    // A plunge entry asked for is still a plunge, at the plunge feed.
    let mut plunge = job.clone();
    plunge["operations"][0]["entry"] = json!({"type": "plunge"});
    let r = run(&mut s, "cam.job", json!({"job": plunge, "dry_run": true}));
    assert_clean(&r);
}

/// A block with a U-shaped pocket 4 mm deep (two arms joined at the bottom) and, between the
/// arms, a pillar standing 10 mm above the top when `pillar`, half a millimetre off the pocket's wall.
fn u_pocket(pillar: bool) -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 32, "height": 10, "corner": [-20, -18, -10], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 10, "width": 24, "height": 4, "corner": [-15, -14, -4], "operation": "cut"}));
    run(&mut s, "solid.box", json!({"length": 10, "width": 24, "height": 4, "corner": [5, -14, -4], "operation": "cut"}));
    run(&mut s, "solid.box", json!({"length": 30, "width": 6, "height": 4, "corner": [-15, -14, -4], "operation": "cut"}));
    if pillar {
        run(&mut s, "solid.box", json!({"length": 6, "width": 15.5, "height": 10, "corner": [-3, -7.5, 0], "operation": "join"}));
    }
    s
}

#[test]
fn stay_down_links_go_over_the_part_at_the_retract_height_only_where_it_is_clear() {
    let job = json!({
        "machine": "3axis", "post": "mach3_4axis", "tools": [flat(1, 4.0)],
        "operations": [{"name": "u", "type": "pocket", "tool": 1,
                        "polygon": [[-15, -14], [15, -14], [15, 10], [5, 10], [5, -8], [-5, -8], [-5, 10], [-15, 10]],
                        "heights": {"clearance": 20, "retract": 3, "top": 0, "bottom": -4}, "stepdown": 2, "stepover": 1.5,
                        "linking": "stay_down"}]
    });
    let dir = Dir::new("u");
    let sideways = |pts: &[(solidcore_record::Vec3, bool)]| -> Vec<(solidcore_record::Vec3, solidcore_record::Vec3)> {
        pts.windows(2).filter(|w| w[1].1 && (w[1].0.x - w[0].0.x).hypot(w[1].0.y - w[0].0.y) > 1e-6).map(|w| (w[0].0, w[1].0)).collect()
    };
    for pillar in [false, true] {
        let mut s = u_pocket(pillar);
        let out = dir.file(if pillar { "pillar.tap" } else { "open.tap" });
        let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
            run(&mut s, "cam.job", json!({"job": job, "output": out}))
        });
        assert_clean(&r);
        let moves = sideways(&tips(&out));
        // From one arm to the other at the retract height when nothing stands in between.
        let low = moves.iter().filter(|(_, b)| (b.z - 3.0).abs() < 1e-6).count();
        // Over the pillar (with the tool's radius round it) only above its top.
        let over = |(a, b): &(solidcore_record::Vec3, solidcore_record::Vec3)| {
            (0..=100).any(|i| {
                let t = i as f64 / 100.0;
                let (x, y) = (a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
                (x.clamp(-3.0, 3.0) - x).hypot(y.clamp(-7.5, 8.0) - y) < 2.0
            })
        };
        if pillar {
            assert!(moves.iter().filter(|m| over(m)).all(|(_, b)| b.z >= 10.0), "{moves:?}");
            assert!(moves.iter().any(over), "the arms are linked over the pillar: {moves:?}");
        } else {
            assert!(low >= 2, "{moves:?}");
        }
    }
}

#[test]
fn hostile_leads_tabs_and_entries_are_refused_without_panics() {
    let mut s = leads_plate();
    let pocket = |extra: Value| {
        let mut op = json!({"name": "p", "type": "pocket", "tool": 1, "polygon": [[-8, -6], [8, -6], [8, 6], [-8, 6]],
                            "heights": {"clearance": 20, "retract": 3, "top": 0, "bottom": -4}});
        for (k, v) in extra.as_object().unwrap() {
            op[k] = v.clone();
        }
        json!({"job": {"machine": "3axis", "post": "mach3_4axis", "tools": [flat(1, 6.0)], "operations": [op]}, "dry_run": true})
    };
    let contour = |extra: Value| json!({"job": profile_job(extra), "dry_run": true});
    let cases = [
        (contour(json!({"lead": {"radius": 2, "radus": 1}})), "radus"),
        (contour(json!({"entry": {"type": "spiral"}})), "spiral"),
        (contour(json!({"linking": "hover"})), "hover"),
        (contour(json!({"tabs": {"count": 4, "spacing": 10, "width": 4, "height": 1}})), "not both"),
        (contour(json!({"tabs": {"width": 4, "height": 1}})), "tabs.count"),
        (contour(json!({"tabs": {"count": 4, "width": 4, "height": 1}, "side": "inside"})), "outside"),
        (contour(json!({"tabs": {"count": 4, "width": 4, "height": 6}})), "tab height"),
        (contour(json!({"tabs": {"count": 100, "width": 4, "height": 1}})), "no room"),
        (contour(json!({"entry": {"type": "plunge"}, "ramp_angle_deg": 3})), "not both"),
        (contour(json!({"entry": {"type": "helix", "radius": 10}})), "core"),
        (contour(json!({"finish_passes": 2, "finish_stepover": 2})), "finishing stepover"),
        (contour(json!({"finish_passes": 1000})), "finishing passes"),
        (contour(json!({"lead": {"radius": 2, "sweep_deg": 0}})), "sweep"),
        (contour(json!({"lead": {"radius": 1e308}})), "lead radius"),
        // Inside a 40 × 30 opening a lead of radius 14 reaches the far wall.
        (contour(json!({"lead": {"radius": 14, "sweep_deg": 180}, "side": "inside"})), "lead would cut into the part"),
        (pocket(json!({"entry": {"type": "helix", "angle_deg": 90}})), "helix angle"),
        (pocket(json!({"entry": {"type": "ramp", "angle_deg": -1}})), "ramp angle"),
        (pocket(json!({"lead": {"radius": 2}})), "lead"),
        (pocket(json!({"tabs": {"count": 2, "width": 1, "height": 1}})), "tabs"),
    ];
    for (p, word) in cases {
        match s.execute("cam.job", &p) {
            Err(EngineError::Internal(_, msg)) => panic!("{p}: panicked: {msg}"),
            Err(e) => assert!(e.to_string().contains(word), "{p}: {e}"),
            Ok(v) => panic!("{p}: accepted: {v}"),
        }
    }
    // Values at the limits plan or are refused, never panic.
    for v in [json!(0), json!(-1), json!(1e-300), json!(1e308), json!(-1e308), json!(4294967295u64)] {
        for extra in [
            json!({"lead": {"radius": v, "line": v}}),
            json!({"entry": {"type": "helix", "radius": v, "angle_deg": v}}),
            json!({"tabs": {"spacing": v, "width": v, "height": v, "ramp_angle_deg": v}}),
            json!({"tabs": {"count": v, "width": 4, "height": 1}}),
            json!({"finish_passes": v, "finish_stepover": v, "spring_pass": true}),
        ] {
            if let Err(EngineError::Internal(_, msg)) = s.execute("cam.job", &contour(extra.clone())) {
                panic!("{extra}: panicked: {msg}");
            }
        }
        if let Err(EngineError::Internal(_, msg)) = s.execute("cam.job", &pocket(json!({"entry": {"type": "helix", "radius": v}}))) {
            panic!("{v}: panicked: {msg}");
        }
    }
}

/// The fastest any cut of a written 3-axis program goes down, mm/min of Z.
fn fastest_descent(path: &str) -> f64 {
    use solidcraft_cam::gcode5::JointWords;
    use solidcraft_geom::prelude::*;
    let mut at: Option<solidcore_record::Vec3> = None;
    let mut fastest = 0.0f64;
    for b in blocks(path) {
        let (to, rate) = match b {
            JointWords::Rapid { linear, .. } => (linear, None),
            JointWords::Feed { linear, f, inverse_time, .. } => {
                assert!(!inverse_time, "a 3-axis program writes feeds per minute");
                (linear, at.map(|p| (p.dist(linear), f)))
            }
            JointWords::Arc { linear, center, ccw, f, .. } => {
                let len = at.map(|p| {
                    (solidcore_record::Vec2::new(p.x, p.y).dist(center) * solidcraft_cam::arc_sweep(p, linear, center, ccw)).hypot(p.z - linear.z)
                });
                (linear, len.map(|l| (l, f)))
            }
            JointWords::Dwell { .. } => continue,
        };
        if let (Some(p), Some((len, f))) = (at, rate)
            && to.z < p.z - 1e-6
        {
            fastest = fastest.max(f * (p.z - to.z) / len);
        }
        at = Some(to);
    }
    fastest
}

#[test]
fn written_programs_never_go_down_faster_than_the_plunge_feed() {
    // A 3 mm cutter's aluminium feeds on a small mill: 2880 mm/min across, 100 down. The tabs'
    // 30° ramps, the helix and the ramp entries are all slowed to go down at 100.
    let tool = json!({"number": 1, "kind": "flat_endmill", "diameter": 3, "flute_length": 8, "spindle_rpm": 24000, "feed": 2880, "plunge_feed": 100});
    let dir = Dir::new("descent");
    let mut jobs = Vec::new();
    let mut profile =
        profile_job(json!({"lead": {"radius": 2, "line": 1}, "entry": {"type": "helix"}, "tabs": {"count": 4, "width": 4, "height": 1.5}}));
    profile["tools"] = json!([tool]);
    jobs.push((leads_plate(), profile, "profile.tap"));
    let mut ramped = profile_job(json!({"entry": {"type": "ramp"}, "tabs": {"count": 2, "width": 3, "height": 1}}));
    ramped["tools"] = json!([tool]);
    jobs.push((leads_plate(), ramped, "ramped.tap"));
    let pocket = json!({
        "machine": "3axis", "post": "mach3_4axis", "tools": [tool],
        "operations": [{"name": "pocket", "type": "pocket", "tool": 1, "polygon": [[-8, -6], [8, -6], [8, 6], [-8, 6]],
                        "heights": {"clearance": 20, "retract": 3, "top": 0, "bottom": -4}, "stepdown": 2, "stepover": 1.2}]
    });
    jobs.push((top_pocket(), pocket, "pocket.tap"));
    for (mut s, job, name) in jobs {
        let out = dir.file(name);
        let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
            run(&mut s, "cam.job", json!({"job": job, "output": out}))
        });
        assert_clean(&r);
        // The written F has no decimals: a half unit of rounding.
        let fastest = fastest_descent(&out);
        assert!(fastest <= 100.0 * (1.0 + 0.5 / 2000.0) + 1e-9, "{name}: down at {fastest} mm/min");
    }
    // On the profile itself there is no side to lead in from or to turn a helix on.
    let mut s = leads_plate();
    for extra in [json!({"side": "on", "lead": {"radius": 2}}), json!({"side": "on", "entry": {"type": "helix"}})] {
        match s.execute("cam.job", &json!({"job": profile_job(extra.clone()), "dry_run": true})) {
            Err(EngineError::Internal(_, msg)) => panic!("{extra}: panicked: {msg}"),
            Err(e) => assert!(e.to_string().contains("on the profile"), "{extra}: {e}"),
            Ok(v) => panic!("{extra}: accepted: {v}"),
        }
    }
}

/// A 40 × 30 floor 10 mm thick, its top at Z 0, with a block 8 mm tall standing on its +X half:
/// a sharp concave corner along Y at X 0.
fn stepped_block() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 10, "corner": [-20, -15, -10], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 20, "width": 30, "height": 8, "corner": [0, -15, 0], "operation": "join"}));
    s
}

fn finish_job(tools: Value, op: Value) -> Value {
    json!({"name": "finish", "machine": "3axis", "post": "mach3_4axis", "tools": tools, "operations": [op]})
}

#[test]
fn the_3d_finishing_strategies_read_back_and_pass_the_gouge_check() {
    let h = json!({"clearance": 30, "retract": 15, "top": 7, "bottom": -1});
    let mut s = dome();
    let b = json!({"rect": [[-14, -14], [14, 14]]});
    for op in [
        json!({"name": "scallop", "type": "scallop", "tool": 2, "heights": h, "scallop_height": 0.01, "boundary": b}),
        json!({"name": "out", "type": "scallop", "tool": 2, "heights": h, "stepover": 1.0, "direction": "outward", "climb": false, "boundary": b}),
        json!({"name": "split", "type": "steep_shallow", "tool": 2, "heights": h, "scallop_height": 0.01, "threshold_deg": 35, "boundary": b}),
        json!({"name": "split raster", "type": "steep_shallow", "tool": 2, "heights": h, "stepover": 0.8, "shallow": "parallel", "angle_deg": 30, "boundary": b}),
        json!({"name": "pencil", "type": "pencil", "tool": 2, "heights": h, "passes": 1, "stepover": 0.5, "boundary": b}),
    ] {
        let r = run(&mut s, "cam.job", json!({"job": finish_job(json!([ball(2, 6.0)]), op.clone()), "dry_run": true}));
        assert_clean(&r);
        let o = &r["operations"][0];
        assert_eq!(o["kind"], op["type"], "{r}");
        assert!(o["moves"].as_u64().is_some_and(|m| m > 20), "{r}");
    }
    // An indexed scallop on the 4th axis, in the frame A = 90° brings under the tool.
    let mut bar = bar();
    let job = json!({
        "machine": "4axis_a", "post": "mach3_4axis", "tools": [ball(3, 4.0)],
        "operations": [{"type": "indexed", "a_deg": 90, "operation": {
            "name": "side", "type": "scallop", "tool": 3, "scallop_height": 0.01,
            "heights": {"clearance": 30, "retract": 15, "top": 10, "bottom": -1}, "boundary": {"rect": [[-10, -6], [10, 6]]}}}]
    });
    let r = run(&mut bar, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["operations"][0]["kind"], "indexed scallop", "{r}");
}

#[test]
fn pencil_and_rest_finishing_cut_the_corner_of_a_stepped_block() {
    let mut s = stepped_block();
    let dir = Dir::new("corner");
    let h = json!({"clearance": 30, "retract": 15, "top": 8, "bottom": -1});
    let b = json!({"rect": [[-14, -10], [10, 10]]});
    let tools = json!([ball(1, 10.0), ball(2, 4.0)]);
    let scope = Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()]));
    // The pencil runs along the corner: the 4 mm ball's tip stops 2 mm short of the wall.
    let pencil = json!({"name": "pencil", "type": "pencil", "tool": 2, "heights": h, "boundary": b});
    let out = dir.file("pencil.tap");
    let r = solidcraft_io::vfs::scoped(scope.clone(), || run(&mut s, "cam.job", json!({"job": finish_job(tools.clone(), pencil), "output": out})));
    assert_clean(&r);
    let c = run(&mut s, "cam.check", json!({"path": out}));
    let (lo, hi) = (&c["bounds"][0], &c["bounds"][1]);
    assert!((lo[0].as_f64().unwrap() + 2.0).abs() < 0.05 && (hi[0].as_f64().unwrap() + 2.0).abs() < 0.05, "{c}");
    assert!(lo[2].as_f64().unwrap().abs() < 0.01, "{c}");
    // Rest finishing after the 10 mm ball cuts only between where the two tips stop, and a
    // little round it.
    let rest = json!({"name": "rest", "type": "rest_finish", "tool": 2, "previous_tool": 1, "heights": h, "scallop_height": 0.01, "overlap": 0.5, "boundary": b});
    let out = dir.file("rest.tap");
    let r = solidcraft_io::vfs::scoped(scope, || run(&mut s, "cam.job", json!({"job": finish_job(tools.clone(), rest.clone()), "output": out})));
    assert_clean(&r);
    let c = run(&mut s, "cam.check", json!({"path": out}));
    let (lo, hi) = (c["bounds"][0][0].as_f64().unwrap(), c["bounds"][1][0].as_f64().unwrap());
    assert!(lo > -5.0 - 0.5 - 0.3 && hi < -2.0 + 0.05, "{c}");
    assert!(c["feed_length"].as_f64().is_some_and(|l| l > 100.0), "{c}");
    // Both a scallop height and a stepover, an unknown previous tool, values out of range and
    // a misspelt key: refused.
    for (f, word) in [
        (json!({"stepover": 0.5}), "not both"),
        (json!({"previous_tool": 9}), "previous_tool"),
        (json!({"threshold": -1}), "threshold"),
        (json!({"overlap": 1e308}), "overlap"),
        (json!({"previous_tol": 1}), "previous_tol"),
    ] {
        let mut op = rest.clone();
        for (k, v) in f.as_object().unwrap() {
            op[k] = v.clone();
        }
        let e = s.execute("cam.job", &json!({"job": finish_job(tools.clone(), op), "dry_run": true})).unwrap_err();
        assert!(matches!(e, EngineError::BadParams { .. }) && e.to_string().contains(word), "{f}: {e}");
    }
    for op in [
        json!({"name": "x", "type": "steep_shallow", "tool": 2, "heights": h, "threshold_deg": 95}),
        json!({"name": "x", "type": "steep_shallow", "tool": 2, "heights": h, "angle_deg": 10}),
        json!({"name": "x", "type": "pencil", "tool": 2, "heights": h, "passes": 100000}),
        json!({"name": "x", "type": "pencil", "tool": 2, "heights": h, "min_angle_deg": f64::MAX}),
        json!({"name": "x", "type": "scallop", "tool": 2, "heights": h, "scallop_height": 5}),
        json!({"name": "x", "type": "scallop", "tool": 2, "heights": h, "step": 1e-9}),
        json!({"name": "x", "type": "scallop", "tool": 2, "heights": h, "direction": "sideways"}),
    ] {
        match s.execute("cam.job", &json!({"job": finish_job(tools.clone(), op.clone()), "dry_run": true})) {
            Err(EngineError::Internal(_, msg)) => panic!("{op}: panicked: {msg}"),
            Err(_) => {}
            Ok(v) => panic!("{op}: accepted: {v}"),
        }
    }
}

#[test]
fn steep_shallow_and_outward_scallop_on_a_stepped_block_read_back_and_pass_the_gouge_check() {
    let mut s = stepped_block();
    let dir = Dir::new("steep");
    let scope = Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()]));
    let h = json!({"clearance": 30, "retract": 15, "top": 8, "bottom": -1});
    let b = json!({"rect": [[-14, -10], [14, 10]]});
    let tools = json!([ball(1, 10.0), ball(2, 6.0)]);
    // The z-levels down the wall at X 0 and the passes on the floor and the top, written,
    // read back and checked against the block; climb and conventional, inward and outward.
    for (file, op) in [
        (
            "steep.tap",
            json!({"name": "split", "type": "steep_shallow", "tool": 2, "heights": h, "stepover": 0.6, "threshold_deg": 45, "boundary": b}),
        ),
        (
            "raster.tap",
            json!({"name": "split", "type": "steep_shallow", "tool": 2, "heights": h, "stepover": 0.6, "shallow": "parallel", "climb": false, "boundary": b}),
        ),
        ("out.tap", json!({"name": "offsets", "type": "scallop", "tool": 2, "heights": h, "stepover": 0.6, "direction": "outward", "boundary": b})),
    ] {
        let out = dir.file(file);
        let r = solidcraft_io::vfs::scoped(scope.clone(), || {
            run(&mut s, "cam.job", json!({"job": finish_job(tools.clone(), op.clone()), "output": out}))
        });
        assert_clean(&r);
        assert!(r["operations"][0]["moves"].as_u64().is_some_and(|m| m > 100), "{r}");
        let c = run(&mut s, "cam.check", json!({"path": out}));
        // The ball's tip never below the floor's top, nor outside the boundary.
        let (lo, hi) = (&c["bounds"][0], &c["bounds"][1]);
        assert!(lo[2].as_f64().is_some_and(|z| z > -1e-3), "{file}: {c}");
        assert!(lo[0].as_f64().is_some_and(|x| x > -14.0 - 1e-6) && hi[0].as_f64().is_some_and(|x| x < 14.0 + 1e-6), "{file}: {c}");
    }
    // Rest finishing where the previous tool left nothing (the flat top): a job with no cut,
    // not an error.
    let rest =
        json!({"name": "rest", "type": "rest_finish", "tool": 2, "previous_tool": 1, "heights": h, "boundary": {"rect": [[4, -10], [14, 10]]}});
    let r = run(&mut s, "cam.job", json!({"job": finish_job(tools, rest), "dry_run": true}));
    assert_eq!(r["passed"], true, "{r}");
}
