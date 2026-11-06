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

/// The dome finished with two ball end mills, the second with holder data; a third tool in the
/// list is never used.
fn two_tool_job() -> Value {
    let mut small = ball(3, 4.0);
    small["stickout"] = json!(25);
    small["shank_diameter"] = json!(6);
    small["holder"] = json!([{"diameter": 30, "length": 20}]);
    small["description"] = json!("4 mm ball, long reach");
    json!({
        "name": "dome two tools",
        "machine": "3axis",
        "post": "mach3_4axis",
        "tools": [ball(2, 6.0), small, flat(9, 10.0)],
        "operations": [
            {"name": "finish 6", "type": "parallel", "tool": 2,
             "heights": {"clearance": 30, "retract": 15, "top": 7, "bottom": -1}, "stepover": 2, "boundary": {"rect": [[-14, -14], [14, 14]]}},
            {"name": "finish 4", "type": "parallel", "tool": 3, "spindle_rpm": 18000, "feeds": {"cut": 700, "plunge": 150},
             "heights": {"clearance": 30, "retract": 15, "top": 7, "bottom": -1}, "stepover": 2, "angle_deg": 90,
             "boundary": {"rect": [[-14, -14], [14, 14]]}}
        ]
    })
}

fn scoped<T>(dir: &Dir, f: impl FnOnce() -> T) -> T {
    solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), f)
}

#[test]
fn a_setup_sheet_lists_the_tools_operations_and_checks_of_a_two_tool_job() {
    let mut s = dome();
    let dir = Dir::new("sheet");
    let (out, page, md) = (dir.file("dome.tap"), dir.file("dome-setup.html"), dir.file("dome-setup.md"));
    let r = scoped(&dir, || run(&mut s, "cam.job", json!({"job": two_tool_job(), "output": out, "setup_sheet": page})));
    assert_clean(&r);
    assert_eq!(r["outputs"]["setup_sheet"], page.as_str(), "{r}");
    let sum = &r["summary"];
    // Only the tools the operations use, by number, each with its operations and how low it goes.
    let tools = sum["tools"].as_array().unwrap();
    assert_eq!(tools.iter().map(|t| t["number"].as_u64().unwrap()).collect::<Vec<_>>(), [2, 3], "{sum}");
    assert_eq!(tools[0]["operations"], json!(["finish 6"]), "{sum}");
    assert_eq!(tools[1]["operations"], json!(["finish 4"]), "{sum}");
    assert_eq!(tools[1]["holder"], "Ø30 x 20", "{sum}");
    assert_eq!(tools[1]["stickout"].as_f64(), Some(25.0), "{sum}");
    assert!(tools[0].get("holder").is_none(), "{sum}");
    // Z min is the lowest Z the program writes: what cam.check reads from the file.
    let c = run(&mut s, "cam.check", json!({"path": out}));
    let low = c["bounds"][0][2].as_f64().unwrap();
    let zmins: Vec<f64> = tools.iter().map(|t| t["z_min"].as_f64().unwrap()).collect();
    assert!(zmins.iter().all(|z| (-1.0001..7.0).contains(z)), "{zmins:?}");
    assert!((zmins.iter().copied().fold(f64::INFINITY, f64::min) - low).abs() < 2e-3, "{zmins:?} against {c}");
    let ops = sum["operations"].as_array().unwrap();
    assert_eq!(ops.len(), 2);
    assert_eq!((ops[1]["spindle_rpm"].as_f64(), ops[1]["feed"].as_f64(), ops[1]["plunge_feed"].as_f64()), (Some(18000.0), Some(700.0), Some(150.0)));
    assert_eq!((ops[0]["spindle_rpm"].as_f64(), ops[0]["feed"].as_f64()), (Some(12000.0), Some(900.0)), "the tool's data: {sum}");
    assert!(ops[0].get("rotary").is_none(), "a 3-axis machine has no rotary: {sum}");
    assert!(ops.iter().all(|o| o["cut_length"].as_f64().is_some_and(|l| l > 100.0) && o["cut_minutes"].as_f64().is_some_and(|m| m > 0.0)));
    assert_eq!(sum["totals"]["tool_changes"], 2, "{sum}");
    assert_eq!(sum["totals"]["operations"], 2, "{sum}");
    assert_eq!(sum["totals"]["tools"], 2, "{sum}");
    let extent = sum["part_extent"].as_array().unwrap();
    let corner = |i: usize| -> Vec<f64> { extent[i].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect() };
    for (got, want) in corner(0).iter().chain(&corner(1)).zip([-20.0, -20.0, -20.0, 20.0, 20.0, 7.0]) {
        assert!((got - want).abs() < 0.01, "{sum}");
    }
    let zero = sum["work_zero"].to_string();
    assert!(zero.contains("X0 is in the middle of the part in X") && zero.contains("mm from its top"), "{zero}");
    assert!(sum["rotary"].as_array().is_some_and(Vec::is_empty), "{sum}");
    let checks = sum["checks"].to_string();
    assert!(checks.contains("Read-back: passed") && checks.contains("Gouges: passed") && checks.contains("Shank and holder: passed"), "{checks}");
    let warnings = sum["warnings"].to_string();
    assert!(warnings.contains("T2 has no holder data") && !warnings.contains("T3 has"), "{warnings}");
    // The page holds the same.
    let html = std::fs::read_to_string(&page).unwrap();
    for want in [
        "Setup sheet: dome two tools",
        "finish 6",
        "finish 4",
        "T2",
        "T3",
        "Ø30 x 20",
        "4 mm ball, long reach",
        "18000",
        "Read-back",
        "passed",
        "dome.tap",
        "@media print",
    ] {
        assert!(html.contains(want), "{want}: {html}");
    }
    assert!(!html.contains("<script") && !html.contains("http://") && !html.contains("https://") && !html.contains("<link"), "self-contained");
    // The Markdown sheet; the summary is the same whatever the sheet's format, and on a dry run.
    let r2 = scoped(&dir, || run(&mut s, "cam.job", json!({"job": two_tool_job(), "output": out, "setup_sheet": md})));
    assert_eq!(r2["summary"], r["summary"]);
    let text = std::fs::read_to_string(&md).unwrap();
    assert!(text.starts_with("# Setup sheet: dome two tools\n") && text.contains("| T3 | ball end mill | Ø4 |"), "{text}");
    let dry = run(&mut s, "cam.job", json!({"job": two_tool_job(), "dry_run": true}));
    assert_eq!(dry["summary"]["tools"], r["summary"]["tools"]);
    assert!(dry["outputs"].get("setup_sheet").is_none(), "{dry}");
}

#[test]
fn a_setup_sheet_escapes_hostile_names() {
    let mut s = dome();
    let dir = Dir::new("sheet-hostile");
    let mut job = two_tool_job();
    job["name"] = json!("<script>alert(1)</script> & 'x'");
    job["operations"][0]["name"] = json!("<img src=x onerror=alert(2)> | *b* `c`");
    job["tools"][1]["description"] = json!("</td></tr></table><script>alert(3)</script>");
    let (page, md) = (dir.file("hostile.html"), dir.file("hostile.md"));
    run(&mut s, "cam.job", json!({"job": job, "output": dir.file("a.tap"), "setup_sheet": page}));
    run(&mut s, "cam.job", json!({"job": job, "output": dir.file("b.tap"), "setup_sheet": md}));
    let html = std::fs::read_to_string(&page).unwrap();
    assert!(!html.contains("<script") && !html.contains("<img") && !html.contains("</td></tr></table><"), "{html}");
    assert!(
        html.contains("&lt;script&gt;alert(1)&lt;/script&gt; &amp; &#39;x&#39;") && html.contains("&lt;img src=x onerror=alert(2)&gt;"),
        "{html}"
    );
    let text = std::fs::read_to_string(&md).unwrap();
    assert!(!text.contains("<script") && !text.contains("<img"), "{text}");
    let row = text.lines().find(|l| l.starts_with("| 1 |")).unwrap();
    assert!(row.contains("\\| \\*b\\* \\`c\\`"), "{row}");
    assert_eq!(row.replace("\\|", "").matches('|').count(), 13, "the row keeps its cells: {row}");
}

#[test]
fn a_setup_sheet_is_written_only_inside_the_agents_folders() {
    let mut s = dome();
    let dir = Dir::new("sheet-scope");
    let root = dir.0.join("allowed");
    std::fs::create_dir_all(&root).unwrap();
    let inside = root.join("dome.tap").to_string_lossy().into_owned();
    let outside = dir.file("dome-setup.html");
    let scope = Some(solidcraft_io::vfs::PathScope::new([root.clone()]));
    solidcraft_io::vfs::scoped(scope, || {
        let e = s.execute("cam.job", &json!({"job": two_tool_job(), "output": inside, "setup_sheet": outside})).unwrap_err();
        assert!(e.to_string().contains("outside the folders this agent may use"), "{e}");
        let sheet_in = root.join("s.html").to_string_lossy().into_owned();
        let txt = root.join("s.txt").to_string_lossy().into_owned();
        for (p, word) in [
            (json!({"job": two_tool_job(), "output": inside, "setup_sheet": txt}), ".html or .md"),
            (json!({"job": two_tool_job(), "dry_run": true, "setup_sheet": "sheet.pdf"}), ".html or .md"),
            (json!({"job": two_tool_job(), "output": inside, "setup_sheet": ""}), ".html or .md"),
            (json!({"job": two_tool_job(), "output": sheet_in, "setup_sheet": sheet_in}), "file of its own"),
            (json!({"job": two_tool_job(), "output": inside, "preview": sheet_in, "setup_sheet": sheet_in}), "file of its own"),
            (json!({"job": two_tool_job(), "output": inside, "setup_sheet": 5}), "setup_sheet"),
        ] {
            let e = s.execute("cam.job", &p).unwrap_err();
            assert!(matches!(e, EngineError::BadParams { .. }) && e.to_string().contains(word), "{p}: {e}");
        }
    });
    assert!(!PathBuf::from(&outside).exists(), "nothing outside the folders");
    assert!(!PathBuf::from(&inside).exists(), "nothing at all: the paths are checked before anything is written");
}

#[test]
fn a_setup_sheet_is_the_same_bytes_every_time() {
    let dir = Dir::new("sheet-same");
    let (out, page, md) = (dir.file("p.tap"), dir.file("p.html"), dir.file("p.md"));
    let mut runs: Vec<(Vec<u8>, Vec<u8>, Value)> = Vec::new();
    for _ in 0..2 {
        // A fresh session each time: nothing carried over.
        let mut s = dome();
        let r = run(&mut s, "cam.job", json!({"job": two_tool_job(), "output": out, "setup_sheet": page}));
        run(&mut s, "cam.job", json!({"job": two_tool_job(), "output": out, "setup_sheet": md}));
        runs.push((std::fs::read(&page).unwrap(), std::fs::read(&md).unwrap(), r["summary"].clone()));
    }
    assert!(runs[0] == runs[1], "two runs give different sheets");
    let html = String::from_utf8(runs[0].0.clone()).unwrap();
    assert!(!html.contains("2026") && !html.contains("2025"), "no date on the sheet");
}

#[test]
fn a_4th_axis_setup_sheet_gives_the_a_convention_and_program_z() {
    // Indexed at A90: the pocket's floor, 10 mm from the axis, is the program's Z.
    let mut s = pocketed_block();
    let r = run(&mut s, "cam.job", json!({"job": side_pocket_job(), "dry_run": true}));
    assert_clean(&r);
    let sum = &r["summary"];
    assert_eq!(sum["operations"][0]["rotary"], "A90°", "{sum}");
    let z = sum["tools"][0]["z_min"].as_f64().unwrap();
    assert!((z - 10.0).abs() < 1e-6, "{sum}");
    let rotary = sum["rotary"].to_string();
    assert!(rotary.contains("A turns the table (and the part) about +X through work zero"), "{rotary}");
    assert!(rotary.contains("A+90° brings the part's +Y side up under the tool"), "{rotary}");
    assert!(rotary.contains("towards -Y"), "{rotary}");
    assert!(sum["work_zero"].to_string().contains("Y0 Z0 is on the A axis centreline"), "{sum}");
    // Reversed: positive A turns the other way, so the controller's A-90 brings +Y up.
    let mut job = side_pocket_job();
    job["a_reversed"] = json!(true);
    job["operations"][0]["a_deg"] = json!(-90);
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["summary"]["operations"][0]["rotary"], "A-90°", "{r}");
    assert!(r["summary"]["rotary"].to_string().contains("-Y side up"), "{r}");
    // A rotary finish: A over its range, the bar's radius in the work zero words.
    let mut bar = bar();
    let r = run(&mut bar, "cam.job", json!({"job": barrel_job(), "dry_run": true}));
    assert_clean(&r);
    let sum = &r["summary"];
    let range = sum["operations"][0]["rotary"].as_str().unwrap();
    assert!(range.starts_with("A ") && range.contains(" to "), "{sum}");
    assert!(sum["work_zero"].to_string().contains("reaches 10"), "{sum}");
    let z = sum["tools"][0]["z_min"].as_f64().unwrap();
    assert!((9.9..=10.1).contains(&z), "the ball's tip rides on the bar: {sum}");
}

/// The A angles (degrees) of the blocks after the program's comment naming operation `name`,
/// up to the next blank line or comment, as written.
fn written_a(text: &str, name: &str) -> Vec<f64> {
    let mut out = Vec::new();
    let named = |l: &&str| l.starts_with('(') && l.to_ascii_lowercase().contains(name);
    for l in text.lines().skip_while(|l| !named(l)).skip(1) {
        if l.starts_with('(') || l.trim().is_empty() {
            break;
        }
        for w in l.split_whitespace() {
            if let Some(v) = w.strip_prefix('A').and_then(|v| v.parse::<f64>().ok()) {
                out.push(v);
            }
        }
    }
    out
}

#[test]
fn a_setup_sheet_gives_each_indexed_operation_the_angle_the_program_writes() {
    // A90 then A-90: A-90 is written as asked, though A270 is the same turn nearer A90.
    let mut s = pocketed_block();
    let dir = Dir::new("sheet-angles");
    let out = dir.file("two-sides.tap");
    let mut job = side_pocket_job();
    let mut other = job["operations"][0].clone();
    other["a_deg"] = json!(-90);
    other["operation"]["name"] = json!("other side");
    other["operation"]["heights"] = json!({"clearance": 40, "retract": 20, "top": 18, "bottom": 16});
    job["operations"].as_array_mut().unwrap().push(other);
    let r = scoped(&dir, || run(&mut s, "cam.job", json!({"job": job, "output": out})));
    assert_clean(&r);
    let text = std::fs::read_to_string(&out).unwrap();
    let a = written_a(&text, "other side");
    assert!(!a.is_empty() && a.iter().all(|v| (v + 90.0).abs() < 1e-9), "the program turns to A-90: {a:?}\n{text}");
    let ops = &r["summary"]["operations"];
    assert_eq!(ops[0]["rotary"], "A90°", "{ops}");
    assert_eq!(ops[1]["rotary"], "A-90°", "as written: {ops}");
    let z = r["summary"]["tools"][0]["z_min"].as_f64().unwrap();
    assert!((z - 10.0).abs() < 1e-6, "{r}");
    // A post that writes no comments: the sheet works it out from the toolpaths, the same.
    let mut post = run(&mut s, "cam.posts", json!({"name": "mach3"}))["post"].clone();
    post["base"]["comments"] = json!("none");
    std::fs::write(dir.0.join("quiet.post.json"), post.to_string()).unwrap();
    let o = job.as_object_mut().unwrap();
    o.remove("post");
    o.insert("post_path".into(), json!(dir.file("quiet.post.json")));
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    let ops = &r["summary"]["operations"];
    assert_eq!((ops[0]["rotary"].as_str(), ops[1]["rotary"].as_str()), (Some("A90°"), Some("A-90°")), "{ops}");
}

#[test]
fn a_setup_sheet_gives_the_a_range_the_program_writes_with_rewinds() {
    // A table that turns only so far, with a post that rewinds: the rotary finish's range on
    // the sheet is the one the program turns through, rewinds and all.
    let mut s = bar();
    let dir = Dir::new("sheet-rewind");
    let mut post = run(&mut s, "cam.posts", json!({"name": "mach3"}))["post"].clone();
    post["rewind"] = json!({"distance": 25.0, "retract_feed": 500.0, "plunge_feed": 250.0});
    std::fs::write(dir.0.join("rewind.post.json"), post.to_string()).unwrap();
    for limits in [None, Some(json!([-180, 180])), Some(json!([-30, 330])), Some(json!([-360, 360]))] {
        let mut job = barrel_job();
        let o = job.as_object_mut().unwrap();
        o.remove("post");
        o.insert("post_path".into(), json!(dir.file("rewind.post.json")));
        if let Some(l) = &limits {
            o.insert("a_limits_deg".into(), l.clone());
        }
        let out = dir.file("bar.tap");
        let r = scoped(&dir, || run(&mut s, "cam.job", json!({"job": job, "output": out})));
        assert_clean(&r);
        let c = run(&mut s, "cam.check", json!({"path": out}));
        let a = &c["rotary_ranges"][0];
        let deg = |k: &str| solidcraft_cam::setup_sheet::fmt_num(a[k].as_f64().unwrap(), 3);
        let want = format!("A {}° to {}°", deg("min_deg"), deg("max_deg"));
        assert_eq!(r["summary"]["operations"][0]["rotary"].as_str(), Some(want.as_str()), "{limits:?}: {}", r["summary"]);
        let z = r["summary"]["tools"][0]["z_min"].as_f64().unwrap();
        assert!((z - c["bounds"][0][2].as_f64().unwrap()).abs() < 2e-3, "{limits:?}: {r}");
    }
}

#[test]
fn a_setup_sheet_spelt_like_the_preview_is_still_the_preview() {
    let mut s = dome();
    let dir = Dir::new("sheet-same-file");
    let out = dir.file("dome.tap");
    let page = dir.file("dome.html");
    let dotted = dir.0.join(".").join("dome.html").to_string_lossy().into_owned();
    let mut same = vec![dotted];
    if cfg!(windows) {
        same.push(dir.file("DOME.HTML"));
    }
    for sheet in same {
        let e =
            scoped(&dir, || s.execute("cam.job", &json!({"job": two_tool_job(), "output": out, "preview": page, "setup_sheet": sheet}))).unwrap_err();
        assert!(e.to_string().contains("file of its own"), "{sheet}: {e}");
    }
    assert!(!PathBuf::from(&page).exists() && !PathBuf::from(&out).exists(), "nothing written");
}
