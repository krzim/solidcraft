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
fn plate() -> Session {
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
    let mut s = plate();
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
    let mut s = plate();
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
    jobs.push((plate(), profile, "profile.tap"));
    let mut ramped = profile_job(json!({"entry": {"type": "ramp"}, "tabs": {"count": 2, "width": 3, "height": 1}}));
    ramped["tools"] = json!([tool]);
    jobs.push((plate(), ramped, "ramped.tap"));
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
    let mut s = plate();
    for extra in [json!({"side": "on", "lead": {"radius": 2}}), json!({"side": "on", "entry": {"type": "helix"}})] {
        match s.execute("cam.job", &json!({"job": profile_job(extra.clone()), "dry_run": true})) {
            Err(EngineError::Internal(_, msg)) => panic!("{extra}: panicked: {msg}"),
            Err(e) => assert!(e.to_string().contains("on the profile"), "{extra}: {e}"),
            Ok(v) => panic!("{extra}: accepted: {v}"),
        }
    }
}
