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
