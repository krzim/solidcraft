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

/// The bar with a slot wrapped round it: 6 mm wide along X, 2 mm deep, a quarter turn round
/// from A −45° to +45° (a rectangle on XZ above the axis revolved about X both ways, cut).
fn slotted_bar() -> Session {
    let mut s = bar();
    run(&mut s, "sketch.create", json!({"plane": "XZ", "name": "Slot"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [-3, -8], "p1": [3, -12]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.revolve", json!({"sketch": "Slot", "axis": "X", "angle": "90 deg", "direction": "symmetric", "operation": "cut"}));
    s
}

/// The slot of `slotted_bar` as a wrapped operation of `kind` with a 4 mm flat endmill.
fn slot_job(kind: &str) -> Value {
    let q = 10.0 * std::f64::consts::FRAC_PI_4;
    let mut op = json!({
        "name": "slot", "type": kind, "tool": 1, "radius": 10,
        "polygon": [[-3, -q], [3, -q], [3, q], [-3, q]],
        "bottom": -2, "stepdown": 1
    });
    if kind == "wrap_contour" {
        op["side"] = json!("inside");
    } else {
        op["stepover"] = json!(1.5);
    }
    json!({"machine": "4axis_a", "post": "mach3_4axis", "tools": [flat(1, 4.0)], "operations": [op]})
}

/// The cuts of a written program as `(x, a in radians, z)`: on the 4th axis the tip is under
/// the tool at Y 0, Z its radius.
fn written_cuts(text: &str) -> Vec<(f64, f64, f64)> {
    use solidcraft_cam::gcode5::{JointWords, parse5};
    use solidcraft_cam::kinematics::AxisLetter;
    parse5(text, &[AxisLetter::A])
        .unwrap()
        .into_iter()
        .filter_map(|b| match b {
            JointWords::Feed { linear, rotary, .. } => Some((linear.x, rotary[0], linear.z)),
            _ => None,
        })
        .collect()
}

#[test]
fn a_wrapped_slot_is_cut_to_its_width_at_the_floor_radius() {
    for kind in ["wrap_pocket", "wrap_contour"] {
        let mut s = slotted_bar();
        let dir = Dir::new("slot");
        let out = dir.file("slot.tap");
        let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
            run(&mut s, "cam.job", json!({"job": slot_job(kind), "output": out}))
        });
        assert_clean(&r);
        assert_eq!(r["operations"][0]["kind"], kind, "{r}");
        let text = std::fs::read_to_string(&out).unwrap();
        assert!(text.contains("G93"), "cuts that turn A are in inverse time");
        let cuts = written_cuts(&text);
        // The floor is at radius 8, exactly, and nothing goes deeper.
        let lowest = cuts.iter().map(|c| c.2).fold(f64::INFINITY, f64::min);
        assert!((lowest - 8.0).abs() <= 1e-3, "{kind}: {lowest}");
        let floor: Vec<&(f64, f64, f64)> = cuts.iter().filter(|c| (c.2 - 8.0).abs() <= 1e-3).collect();
        let (xlo, xhi) = floor.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), c| (lo.min(c.0), hi.max(c.0)));
        let (alo, ahi) = floor.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), c| (lo.min(c.1), hi.max(c.1)));
        // Unwrapped at the floor: the slot's 6 mm along X and its quarter turn round, at
        // radius 8, are the centres' travel plus the tool's 4 mm.
        assert!((xhi - xlo + 4.0 - 6.0).abs() <= 2e-3, "{kind}: {xlo}..{xhi}");
        let arc = (ahi - alo) * 8.0 + 4.0;
        assert!((arc - 8.0 * std::f64::consts::FRAC_PI_2).abs() <= 0.02, "{kind}: {arc}");
        assert!(alo > -std::f64::consts::FRAC_PI_4 && ahi < std::f64::consts::FRAC_PI_4, "{kind}: A within the slot");
    }
    // On the bar without the slot the same pocket cuts what the model does not show.
    let r = run(&mut bar(), "cam.job", json!({"job": slot_job("wrap_pocket"), "dry_run": true}));
    assert_eq!(r["passed"], false, "{r}");
    assert!(r["gouge"]["worst"].as_f64().is_some_and(|w| w > 1.9), "the slot is 2 mm deep: {r}");
}

#[test]
fn rotary_rings_step_over_exactly_in_the_written_program() {
    let mut s = bar();
    let dir = Dir::new("rings");
    let out = dir.file("rings.tap");
    let mut job = barrel_job();
    job["operations"][0]["pattern"] = json!({"type": "rings", "stepover": 2.5});
    let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
        run(&mut s, "cam.job", json!({"job": job, "output": out}))
    });
    assert_clean(&r);
    let text = std::fs::read_to_string(&out).unwrap();
    let cuts = written_cuts(&text);
    // The rings: runs of cuts at one X, a turn of A each.
    let mut rings: Vec<(f64, f64, f64)> = Vec::new();
    for &(x, a, _) in &cuts {
        match rings.last_mut() {
            Some((rx, lo, hi)) if (x - *rx).abs() <= 1e-3 => {
                *lo = lo.min(a);
                *hi = hi.max(a);
            }
            _ => rings.push((x, a, a)),
        }
    }
    rings.retain(|(_, lo, hi)| hi - lo > 6.0);
    assert_eq!(rings.len(), 13, "{rings:?}");
    for (k, (x, lo, hi)) in rings.iter().enumerate() {
        assert!((x - (-15.0 + 2.5 * k as f64)).abs() <= 1e-3, "ring {k} at {x}");
        assert!(hi - lo >= std::f64::consts::TAU - 0.07, "ring {k} turns {}", hi - lo);
    }
    // Zigzag rings turn back and forth: A stays within a turn.
    let (alo, ahi) = cuts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), c| (lo.min(c.1), hi.max(c.1)));
    assert!(ahi - alo <= std::f64::consts::TAU + 1e-3, "{alo}..{ahi}");
}

/// A bar 40 long with a 30 mm square section about X: a flat face 15 mm out at each quarter turn.
fn square_bar() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 30, "corner": [-20, -15, -15], "body_name": "Block"}));
    s
}

fn face_op(name: &str, clearance: f64) -> Value {
    json!({
        "name": name, "type": "parallel", "tool": 1,
        "heights": {"clearance": clearance, "retract": 17, "top": 15, "bottom": 0},
        "stepover": 3, "boundary": {"rect": [[-10, -6], [10, 6]]}
    })
}

fn faces_job(operations: Value) -> Value {
    json!({"machine": "4axis_a", "post": "mach3_4axis", "tools": [flat(1, 4.0)], "operations": operations})
}

#[test]
fn an_indexed_pattern_is_the_indexed_operations_it_stands_for() {
    let mut s = square_bar();
    let dir = Dir::new("pattern");
    let write = |s: &mut Session, job: Value, file: &str| {
        let out = dir.file(file);
        let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
            run(s, "cam.job", json!({"job": job, "output": out}))
        });
        assert_clean(&r);
        (r, std::fs::read_to_string(&out).unwrap())
    };
    let pattern = faces_job(json!([{"type": "indexed_pattern", "a_deg": [0, 90, 180, 270], "operation": face_op("face", 40.0)}]));
    let (r, text) = write(&mut s, pattern, "pattern.tap");
    let four: Vec<Value> =
        [0, 90, 180, 270].iter().map(|a| json!({"type": "indexed", "a_deg": a, "operation": face_op(&format!("face A{a}"), 40.0)})).collect();
    let (r4, text4) = write(&mut s, faces_job(json!(four)), "four.tap");
    assert_eq!(text, text4, "the pattern writes what the four indexed operations write");
    assert_eq!(r["operations"], r4["operations"]);
    let ops = r["operations"].as_array().unwrap();
    let names: Vec<&str> = ops.iter().filter_map(|o| o["name"].as_str()).collect();
    assert_eq!(names, ["face A0", "face A90", "face A180", "face A270"]);
    let angles: Vec<f64> = ops.iter().filter_map(|o| o["a_deg"].as_f64()).collect();
    assert_eq!(angles, [0.0, 90.0, 180.0, 270.0]);
    assert!(ops.iter().all(|o| o["kind"] == "indexed parallel"), "{r}");
    for a in ["A90.", "A180.", "A270."] {
        assert!(text.contains(a), "{a}");
    }
    // A count spreads the angles evenly over a turn: the same four.
    let count = faces_job(json!([{"type": "indexed_pattern", "count": 4, "operation": face_op("face", 40.0)}]));
    let (_, text_count) = write(&mut s, count, "count.tap");
    assert_eq!(text_count, text);
}

#[test]
fn a_clearance_inside_the_part_is_refused() {
    // The square bar reaches 21.21 mm from the axis at its corners: A turning at a clearance
    // of 18 would swing them into the tool.
    let mut s = square_bar();
    let job = faces_job(json!([{"type": "indexed_pattern", "count": 4, "operation": face_op("face", 18.0)}]));
    let e = s.execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
    let msg = e.to_string();
    assert!(matches!(e, EngineError::BadParams { .. }) && msg.contains("face A0") && msg.contains("21.21") && msg.contains("part"), "{msg}");
    // A post that turns A at the machine's top instead is not refused for it.
    let mut job = job;
    job["post"] = json!("rs274_5axis");
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_eq!(r["operations"].as_array().map(Vec::len), Some(4), "{r}");
    // A wrapped pocket whose rapids would go round just outside the cylinder it is drawn on, in
    // a part that reaches farther: refused before anything is written.
    let mut slot = slot_job("wrap_pocket");
    slot["operations"][0]["clearance"] = json!(0.005);
    slot["operations"][0]["retract"] = json!(0.001);
    let e = slotted_bar().execute("cam.job", &json!({"job": slot, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("not outside the part"), "{e}");
    let mut slot = slot_job("wrap_pocket");
    slot["operations"][0]["radius"] = json!(8);
    slot["operations"][0]["bottom"] = json!(0);
    slot["operations"][0]["clearance"] = json!(1);
    slot["operations"][0]["retract"] = json!(0.5);
    let e = slotted_bar().execute("cam.job", &json!({"job": slot, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("not outside the part"), "{e}");
    // Roughing from round stock: the clearance must clear the stock as well.
    let mut job = barrel_job();
    job["operations"][0] = json!({
        "name": "rough", "type": "rotary_roughing", "tool": 3, "pattern": {"type": "rings", "stepover": 3},
        "stock_radius": 14, "clearance": 14, "retract": 14, "range": [-15, 15]
    });
    let e = bar().execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("not outside the stock"), "{e}");
}

#[test]
fn hostile_rotary_operations_are_refused_without_panics() {
    let mut s = slotted_bar();
    let slot = |f: &dyn Fn(&mut Value)| {
        let mut j = slot_job("wrap_pocket");
        f(&mut j["operations"][0]);
        json!({"job": j, "dry_run": true})
    };
    let face = |f: &dyn Fn(&mut Value)| {
        let mut op = json!({"type": "indexed_pattern", "count": 4, "operation": face_op("face", 40.0)});
        f(&mut op);
        json!({"job": faces_job(json!([op])), "dry_run": true})
    };
    // One operation and a pattern of 200: more than 200 in all, refused before the pattern is planned.
    let many = json!([face_op("plain", 40.0), {"type": "indexed_pattern", "count": 200, "operation": face_op("face", 40.0)}]);
    let cases = vec![
        slot(&|o| o["radius"] = json!(-10)),
        slot(&|o| o["radius"] = json!(1e308)),
        slot(&|o| o["bottom"] = json!(1)),
        slot(&|o| o["bottom"] = json!(-1e308)),
        slot(&|o| o["bottom"] = json!(-10)),
        slot(&|o| o["stepdown"] = json!(0)),
        slot(&|o| o["stepdown"] = json!(1e-9)),
        slot(&|o| o["stepover"] = json!(99)),
        slot(&|o| o["clearance"] = json!(1e9)),
        slot(&|o| o["retract"] = json!(9)),
        slot(&|o| o["max_angle_step_deg"] = json!(0)),
        slot(&|o| o["max_angle_step_deg"] = json!(1e308)),
        slot(&|o| o["tolerance"] = json!(5)),
        slot(&|o| o["ramp_angle_deg"] = json!(89)),
        slot(&|o| o["stock_to_leave"] = json!(-1)),
        slot(&|o| o["polygon"] = json!([[0, 0], [1, 1]])),
        slot(&|o| o["polygon"] = json!([[-3, 0], [3, 0], [3, 70], [-3, 70]])),
        slot(&|o| o["polygon"] = json!([[-3, 0], [3, 0], [3, 1e300], [-3, 1e300]])),
        slot(&|o| o["sketch"] = json!("Slot")),
        slot(&|o| {
            o.as_object_mut().unwrap().remove("polygon");
        }),
        slot(&|o| o["profiles"] = json!([0])),
        slot(&|o| o["sid"] = json!("inside")),
        slot(&|o| o["type"] = json!("wrap_contour")),
        slot(&|o| {
            o["type"] = json!("wrap_contour");
            o["side"] = json!("outside");
            o["polygon"] = json!([[-3, 0], [3, 0], [3, 60], [-3, 60]]);
        }),
        face(&|o| o["a_deg"] = json!([0, 90])),
        face(&|o| {
            o.as_object_mut().unwrap().remove("count");
        }),
        face(&|o| o["count"] = json!(0)),
        face(&|o| o["count"] = json!(201)),
        face(&|o| o["count"] = json!(4294967296u64)),
        face(&|o| o["count"] = json!(-1)),
        face(&|o| o["start_deg"] = json!(1e308)),
        face(&|o| {
            o.as_object_mut().unwrap().remove("count");
            o["a_deg"] = json!([]);
        }),
        face(&|o| {
            o.as_object_mut().unwrap().remove("count");
            o["a_deg"] = json!([0, 1e308]);
        }),
        face(&|o| {
            o.as_object_mut().unwrap().remove("count");
            o["a_deg"] = json!([0, 90]);
            o["start_deg"] = json!(10);
        }),
        face(&|o| o["operation"]["stepovr"] = json!(1)),
        face(&|o| o["operation"] = json!({"type": "rotary_finish"})),
        face(&|o| o["counts"] = json!(4)),
        json!({"job": faces_job(json!(many)), "dry_run": true}),
    ];
    for p in &cases {
        match s.execute("cam.job", p) {
            Err(EngineError::Internal(_, msg)) => panic!("{p}: panicked: {msg}"),
            Err(_) => {}
            Ok(v) => panic!("{p}: accepted: {v}"),
        }
    }
    // The new operations need a 4th axis; rings need a stepover.
    let mut on3 = slot_job("wrap_pocket");
    on3["machine"] = json!("3axis");
    assert!(s.execute("cam.job", &json!({"job": on3, "dry_run": true})).is_err());
    let mut pat = faces_job(json!([{"type": "indexed_pattern", "count": 4, "operation": face_op("face", 40.0)}]));
    pat["machine"] = json!("3axis");
    assert!(square_bar().execute("cam.job", &json!({"job": pat, "dry_run": true})).is_err());
    for stepover in [json!(0), json!(-1), json!(1e308), json!("x")] {
        let mut job = barrel_job();
        job["operations"][0]["pattern"] = json!({"type": "rings", "stepover": stepover});
        assert!(bar().execute("cam.job", &json!({"job": job, "dry_run": true})).is_err(), "{stepover}");
    }
}

/// A stepped bar about X: a 30 mm square section from X −20 to 0 (its corners 21.21 mm from
/// the axis) and a 20 mm one from 0 to 20 (14.14 mm).
fn stepped_bar() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 20, "width": 30, "height": 30, "corner": [-20, -15, -15], "body_name": "Big"}));
    run(&mut s, "solid.box", json!({"length": 20, "width": 20, "height": 20, "corner": [0, -10, -10], "body_name": "Small"}));
    s
}

#[test]
fn a_turn_between_operations_is_checked_where_the_tool_turns() {
    let mut s = stepped_bar();
    // A 3-axis finish of the big section's top (15 mm up), its rapids `clearance` mm up.
    let big = |clearance: f64| {
        json!({
            "name": "big", "type": "parallel", "tool": 1,
            "heights": {"clearance": clearance, "retract": 17, "top": 15, "bottom": 0},
            "stepover": 3, "boundary": {"rect": [[-16, -6], [-4, 6]]}
        })
    };
    // The small section's side at A 90°, its rapids 16 mm up: clear of the small section.
    let small = json!({"type": "indexed", "a_deg": 90, "operation": {
        "name": "small", "type": "parallel", "tool": 1,
        "heights": {"clearance": 16, "retract": 12, "top": 10, "bottom": 0},
        "stepover": 3, "boundary": {"rect": [[5, -5], [15, 5]]}
    }});
    // `big` leaves the tool 18 mm up over the big section, and A turns to 90° there: its corners,
    // 21.21 mm out, swing into the tool. Each operation is clear where it works.
    let e = s.execute("cam.job", &json!({"job": faces_job(json!([big(18.0), small.clone()])), "dry_run": true})).unwrap_err();
    let msg = e.to_string();
    assert!(matches!(e, EngineError::BadParams { .. }), "{msg}");
    assert!(msg.contains("`small`") && msg.contains("`big`") && msg.contains("21.21"), "{msg}");
    // From 25 mm up the turn is clear.
    let r = run(&mut s, "cam.job", json!({"job": faces_job(json!([big(25.0), small.clone()])), "dry_run": true}));
    assert_clean(&r);
    // The other way round A turns over the small section, at 18 mm (the post rises to `big`'s
    // clearance first): clear, and so not refused.
    let r = run(&mut s, "cam.job", json!({"job": faces_job(json!([small.clone(), big(18.0)])), "dry_run": true}));
    assert_clean(&r);
    // A post that goes to the machine's top before turning is never refused for it.
    let mut job = faces_job(json!([big(18.0), small]));
    job["post"] = json!("rs274_5axis");
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_eq!(r["operations"].as_array().map(Vec::len), Some(2), "{r}");
}
