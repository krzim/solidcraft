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

/// The program, the preview and the setup sheet are each written whole to a temporary file
/// beside the old one and renamed over it: a second name of the old file (a hard link, as a
/// controller's share may hold) keeps the old program, and nothing is left beside them.
#[test]
fn a_job_replaces_its_files_through_a_temporary_file() {
    let mut s = dome();
    let dir = Dir::new("atomic");
    let (tap, page, sheet) = (dir.file("dome.tap"), dir.file("dome.html"), dir.file("dome.md"));
    for (f, alias) in [(&tap, "tap.old"), (&page, "html.old"), (&sheet, "md.old")] {
        std::fs::write(f, "old").unwrap();
        std::fs::hard_link(f, dir.0.join(alias)).unwrap();
    }
    let r = run(&mut s, "cam.job", json!({"job": dome_job(), "output": tap, "preview": page, "setup_sheet": sheet}));
    assert_clean(&r);
    for (f, alias) in [(&tap, "tap.old"), (&page, "html.old"), (&sheet, "md.old")] {
        assert!(std::fs::read_to_string(f).unwrap().len() > 100, "{f} was written");
        assert_eq!(std::fs::read_to_string(dir.0.join(alias)).unwrap(), "old", "{f} was replaced, not written over in place");
    }
    let names: Vec<String> = std::fs::read_dir(&dir.0).unwrap().map(|e| e.unwrap().file_name().to_string_lossy().into_owned()).collect();
    assert!(names.iter().all(|n| !n.ends_with(".tmp")), "{names:?}");
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
    // The reply says where: the operation, the move, the line of the program that ends it, and
    // the tool tip in work coordinates, there and in the failure.
    let at = &r["gouge"]["at"];
    assert_eq!(at["operation"], "side pocket", "{r}");
    let moves = r["operations"][0]["moves"].as_u64().unwrap();
    assert!(at["move_index"].as_u64().is_some_and(|m| m < moves), "{r}");
    let line = at["line"].as_u64().unwrap_or_else(|| panic!("no line: {r}")) as usize;
    let text = std::fs::read_to_string(&out).unwrap();
    let written = text.lines().nth(line - 1).unwrap();
    assert!(written.starts_with('G') || written.starts_with('X') || written.starts_with('Y') || written.starts_with('Z'), "line {line}: {written}");
    // The tip is in the pocket, at its floor 5 mm into the +Y face.
    let xyz: Vec<f64> = at["xyz"].as_array().unwrap().iter().filter_map(Value::as_f64).collect();
    assert!(xyz.len() == 3 && xyz[0].abs() <= 6.0 + 1e-6 && (xyz[1] - 10.0).abs() <= 1e-6, "{xyz:?}");
    assert_eq!(r["operations"][0]["gouge_at"]["move_index"], at["move_index"], "{r}");
    let failure = r["failures"].as_array().unwrap().iter().filter_map(Value::as_str).find(|f| f.starts_with("gouge")).unwrap();
    assert!(failure.contains("side pocket") && failure.contains(&format!("G-code line {line}")) && failure.contains("work coordinates"), "{failure}");
    assert!(r["gouge"]["allowed"].as_f64().is_some_and(|a| (a - 0.0075).abs() < 1e-12), "{r}");
}

/// A plate with a 6 mm hole through it.
fn plate_with_a_hole() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 30, "width": 20, "height": 15, "corner": [-15, -10, -15], "body_name": "Plate"}));
    run(&mut s, "solid.cylinder", json!({"radius": 3, "height": 20, "axis": "Z", "base": [0, 0, -17], "operation": "cut"}));
    s
}

/// A 4 mm end mill with a shank of its own diameter, cutting round the inside of a 6 mm hole:
/// its shank above the flutes runs along the wall the cutter has just cut. That is no holder
/// collision (it used to be refused, with more collisions than the operation has moves), and a
/// shank wider than the cutter still is one, counted in moves.
#[test]
fn a_shank_as_wide_as_the_cutter_passes_the_hole_it_cuts() {
    let mut s = plate_with_a_hole();
    let mut tool = flat(4, 4.0);
    tool["flute_length"] = json!(12);
    tool["shank_diameter"] = json!(4);
    tool["stickout"] = json!(20);
    tool["holder"] = json!([{"diameter": 34, "length": 4}, {"diameter": 34, "length": 18}]);
    let mut job = json!({
        "machine": "3axis",
        "post": "mach3_4axis",
        "tools": [tool],
        "operations": [{
            "name": "hole", "type": "contour", "tool": 4, "polygon": circle(3.0, 256), "side": "inside",
            "heights": {"clearance": 10, "retract": 2, "top": 0, "bottom": -15.3}, "stepdown": 1, "ramp_angle_deg": 2
        }]
    });
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["holder_collisions"], 0, "{r}");
    // Wider than the cutter, the shank is into the wall.
    job["tools"][0]["shank_diameter"] = json!(5);
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_eq!(r["passed"], false, "{r}");
    let (hits, moves) = (r["operations"][0]["holder_collisions"].as_u64().unwrap(), r["operations"][0]["moves"].as_u64().unwrap());
    assert!(hits > 0 && hits <= moves, "{hits} collisions in {moves} moves");
}

/// The points of a circle of radius `r` about the origin, `n` round.
fn circle(r: f64, n: usize) -> Value {
    json!(
        (0..n)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / n as f64;
                [r * a.cos(), r * a.sin()]
            })
            .collect::<Vec<_>>()
    )
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
        job(&|j| j["operations"][0]["descent_limit_deg"] = json!(91)),
        job(&|j| j["operations"][0]["descent_limit_deg"] = json!(-1)),
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
    // The model shows the tap drill's hole, not the thread: the simulation cuts with the thread
    // mill's core, as the gouge check does, so the thread is no gouge there either.
    let mut flat_job = m6_job();
    flat_job["machine"] = json!("3axis");
    flat_job["stock"] = json!({"box": {"min": [-20, -15, -10], "max": [20, 15, 0]}});
    let out = dir.file("m6-sim.tap");
    let r = run(&mut s, "cam.job", json!({"job": flat_job, "output": out, "simulate": true}));
    assert_eq!(r["simulation"]["gouge"]["cells"], 0, "{r}");
    assert_clean(&r);
    let c = run(&mut s, "cam.simulate", json!({"path": out, "job": flat_job}));
    assert_eq!(c["gouge"]["cells"], 0, "{c}");
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
/// through the tool, refused (by the 4th-axis setup check while planning, before the rapid
/// check of the written program, which finds the same turn: see the CAM crate's rapids tests);
/// at 25 the turn is clear.
#[test]
fn an_a_turn_low_beside_a_square_bar_is_refused() {
    let mut s = square_bar();
    for p in [json!({"job": square_bar_job(12.0), "output": Dir::new("turn").file("bar.tap")}), json!({"job": square_bar_job(12.0), "dry_run": true})]
    {
        let e = s.execute("cam.job", &p).unwrap_err();
        assert!(matches!(e, EngineError::BadParams { .. }) && e.to_string().contains("clearance") && e.to_string().contains("14.1421"), "{e}");
    }
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

/// Facing with vertical leads, as the approved post's reference programs face: `G19` arcs down
/// into each level and up out of it for passes along Y, half circles between the passes, and
/// every check passed, the simulation's included. `cam.check` measures each arc in its plane.
#[test]
fn facing_with_vertical_leads_writes_g19_arcs_that_pass_every_check() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 50, "width": 50, "height": 10, "corner": [-25, -25, -11], "body_name": "Block"}));
    let mill = json!({"number": 5, "kind": "face_mill", "diameter": 10, "corner_radius": 0.2, "flute_length": 6, "spindle_rpm": 18000, "feed": 1600, "plunge_feed": 50});
    let job = json!({
        "machine": "3axis", "post": "mach3_4axis", "tools": [mill], "stock": {"box": {"min": [-25, -25, -11], "max": [25, 25, 0]}},
        "operations": [{"name": "face", "type": "face", "tool": 5, "bottom": -1, "stepdown": 0.5, "angle_deg": 90, "lead_radius": 0.3}]
    });
    let dir = Dir::new("face-leads");
    let out = dir.file("face.tap");
    let r = run(&mut s, "cam.job", json!({"job": job, "output": out, "simulate": true}));
    assert_clean(&r);
    assert_eq!(r["simulation"]["passed"], true, "{r}");
    let text = std::fs::read_to_string(&out).unwrap();
    // Down in, one level after the other, and up out of each level.
    let g19 = text.lines().filter(|l| l.starts_with("G19 G2") || l.starts_with("G19 G3")).count();
    assert!(g19 >= 2, "{text}");
    assert!(text.lines().any(|l| l.starts_with("G17 G2") || l.starts_with("G17 G3")), "{text}");
    assert!(text.contains("\nG17\n"), "back in G17 after the operation: {text}");
    let c = run(&mut s, "cam.check", json!({"path": out}));
    assert!(c["errors"].as_array().is_some_and(Vec::is_empty), "{c}");
    // The program's own cut length as `cam.check` reads it: the planned one, the arcs round.
    let planned = r["operations"][0]["cut_length"].as_f64().unwrap();
    let read = c["feed_length"].as_f64().unwrap();
    assert!((read - planned).abs() < 0.01 * planned, "{read} {planned}");
    // A quarter arc of 0.3 in YZ is 0.471 long, not its 0.42 chord.
    let quarter = dir.file("quarter.tap");
    std::fs::write(&quarter, "G90 G21\nG0 X0 Y10.3 Z5\nG1 Z0.15 F50\nG19 G2 Y10 Z-0.15 J-0.3 K0 F1600\nG17\n").unwrap();
    let c = run(&mut s, "cam.check", json!({"path": quarter}));
    let quarter_len = c["feed_length"].as_f64().unwrap() - 4.85;
    assert!((quarter_len - 0.15 * std::f64::consts::PI).abs() < 1e-6, "{c}");
}

/// A finish whose retract height is below the stock's top rapids through the stock: refused.
/// (Its passes all run one way, so it lifts to the retract height and crosses there between
/// them; its ways down stop above the stock's top.) A machine whose travel ends below the
/// clearance height: refused, naming the line.
#[test]
fn rapids_through_the_stock_and_beyond_the_travel_are_refused() {
    let mut s = dome();
    let mut job = dome_job();
    job["operations"][0]["zigzag"] = json!(false);
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
/// after it: the pocket's clearance and retract heights, -0.4 and -0.5, below the stock's first
/// top, are above what is left. Without the face, or with a face over half the top, its rapids
/// across at the clearance height run through the stock.
#[test]
fn rapids_after_a_face_are_held_against_the_faced_stock() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 50, "width": 50, "height": 10, "corner": [-25, -25, -11], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 10, "width": 10, "height": 1, "corner": [-5, -5, -2], "operation": "cut"}));
    let face = json!({"name": "face", "type": "face", "tool": 1, "bottom": -1, "stepdown": 0.5});
    let pocket = json!({"name": "pocket", "type": "pocket", "tool": 2, "polygon": [[-5, -5], [5, -5], [5, 5], [-5, 5]],
                        "heights": {"clearance": -0.4, "retract": -0.5, "top": -1, "bottom": -2}, "stepover": 2});
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
    let mut inner = job["operations"][1].take();
    // Clear of the plate's corners as A turns (14.1 mm from the axis): the 4th-axis setup check.
    inner["heights"]["clearance"] = json!(20);
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
    job["operations"][0]["entry"] = json!({"type": "plunge"});
    refused(&mut s, job, "ramp");
    // A pocket's default entry is a helix, which a face mill may take.
    let mut job = chamfer_job(1.0);
    job["operations"][0].as_object_mut().unwrap().remove("ramp_angle_deg");
    assert_clean(&run(&mut s, "cam.job", json!({"job": job, "dry_run": true})));
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
            JointWords::Feed { linear, .. } | JointWords::Arc { linear, .. } | JointWords::PlaneArc { linear, .. } => Some((linear, false)),
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
            | solidcraft_cam::gcode5::JointWords::Arc { linear, .. }
            | solidcraft_cam::gcode5::JointWords::PlaneArc { linear, .. } => Some(*linear),
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
            JointWords::PlaneArc { linear, f, .. } => (linear, at.and_then(|p| b.arc_from(p)).map(|a| (a.length(), f))),
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
fn square_block() -> Session {
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
    let mut s = square_block();
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
    for a in ["A90", "A180", "A270"] {
        assert!(text.split_whitespace().any(|w| w.trim_end_matches('.') == a), "{a}: {text}");
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
    let mut s = square_block();
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
    // Without `stock_radius` the job's round stock gives it: refused the same way ...
    let mut job = barrel_job();
    job["stock"] = json!({"cylinder": {"radius": 14, "x_min": -20, "x_max": 20}});
    job["operations"][0] = json!({
        "name": "rough", "type": "rotary_roughing", "tool": 3, "pattern": {"type": "rings", "stepover": 3},
        "clearance": 14, "retract": 14, "range": [-15, 15]
    });
    let e = bar().execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("not outside the stock"), "{e}");
    // ... and planned from it when the clearance is outside it.
    job["operations"][0]["clearance"] = json!(17);
    job["operations"][0]["retract"] = json!(15);
    let r = run(&mut bar(), "cam.job", json!({"job": job, "dry_run": true}));
    assert_eq!(r["operations"].as_array().map(Vec::len), Some(1), "{r}");
    // Neither the radius nor a stock: refused, saying what to give.
    job.as_object_mut().unwrap().remove("stock");
    let e = bar().execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("stock_radius"), "{e}");
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
    // Operations past 200 in all are refused before they are planned: the pattern's own error
    // never shows, as it never runs.
    let mut bad = face_op("face", 40.0);
    bad["stepover"] = json!(-1);
    let over = json!([face_op("plain", 40.0), {"type": "indexed_pattern", "count": 200, "operation": bad}]);
    let e = s.execute("cam.job", &json!({"job": faces_job(over), "dry_run": true})).unwrap_err().to_string();
    assert!(e.contains("at most 200 operations") && !e.contains("stepover"), "{e}");
    // The new operations need a 4th axis; rings need a stepover.
    let mut on3 = slot_job("wrap_pocket");
    on3["machine"] = json!("3axis");
    assert!(s.execute("cam.job", &json!({"job": on3, "dry_run": true})).is_err());
    let mut pat = faces_job(json!([{"type": "indexed_pattern", "count": 4, "operation": face_op("face", 40.0)}]));
    pat["machine"] = json!("3axis");
    assert!(square_block().execute("cam.job", &json!({"job": pat, "dry_run": true})).is_err());
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

/// A small mill's top rates: 8000 mm/min across, 4000 in Z, A at 8000°/min, the spindle from
/// 10000 to 24000 rpm with a six-second start.
fn mill_rates() -> Value {
    json!({
        "rapid_xyz_mm_min": 8000, "rapid_z_mm_min": 4000, "rapid_a_deg_min": 8000,
        "max_feed_mm_min": 8000, "max_feed_z_mm_min": 4000, "max_a_deg_min": 8000,
        "spindle_rpm": [10000, 24000], "spindle_start_seconds": 6, "travel_mm": [300, 170, 130]
    })
}

#[test]
fn machine_rates_time_the_program_and_warn_about_slowed_blocks() {
    let mut s = bar();
    // Without rates: the time at feed as written, no rapid time.
    let r = run(&mut s, "cam.job", json!({"job": barrel_job(), "dry_run": true}));
    assert_clean(&r);
    assert!(r["machine_time"].is_null() && r["rapid_minutes"].is_null(), "{r}");
    assert_eq!(r["cut_minutes"], r["readback"]["cut_minutes"], "{r}");

    // With the mill's rates: rapids timed, one spindle start, nothing slowed.
    let mut job = barrel_job();
    job["machine_rates"] = mill_rates();
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    let t = &r["machine_time"];
    assert!(r["rapid_minutes"].as_f64().is_some_and(|m| m > 0.0), "{r}");
    assert_eq!(r["rapid_minutes"], t["rapid_minutes"], "{r}");
    assert_eq!(r["cut_minutes"], t["cut_minutes"], "{r}");
    assert!((t["spindle_minutes"].as_f64().unwrap() - 0.1).abs() < 1e-12, "{t}");
    assert_eq!(t["slowed_blocks"], 0, "{t}");
    assert!(t["peak_rotary_deg_min"].as_f64().is_some_and(|p| p > 0.0), "the cuts turning A are in inverse time: {t}");
    let written = t["written_cut_minutes"].as_f64().unwrap();
    assert!((written - r["readback"]["cut_minutes"].as_f64().unwrap()).abs() < 1e-9, "{r}");
    let total = t["cut_minutes"].as_f64().unwrap() + t["rapid_minutes"].as_f64().unwrap() + t["spindle_minutes"].as_f64().unwrap();
    assert!((t["total_minutes"].as_f64().unwrap() - total).abs() < 1e-9, "{t}");

    // An A that turns slower than the cuts ask: the controller slows them, the reply says so,
    // and the program still passes its checks.
    job["machine_rates"]["max_a_deg_min"] = json!(200);
    job["machine_rates"]["spindle_rpm"] = json!([15000, 24000]);
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    let t = &r["machine_time"];
    assert!(t["rotary_over"].as_u64().is_some_and(|n| n > 0), "{t}");
    assert!(t["cut_minutes"].as_f64().unwrap() > written, "{t}");
    let notes = r["notes"].to_string();
    assert!(notes.contains("ask A to turn faster than its 200°/min"), "{notes}");
    assert!(notes.contains("outside the machine's 15000-24000 rpm (S12000 at line"), "{notes}");
    assert_eq!(t["spindle_out_of_range"][0]["rpm"].as_f64(), Some(12000.0), "{t}");

    // The rates may come from a file beside the job.
    let dir = Dir::new("rates");
    std::fs::write(dir.0.join("rates.json"), mill_rates().to_string()).unwrap();
    let mut job = barrel_job();
    job["machine_rates_path"] = json!("rates.json");
    let job_path = dir.file("bar.cam.json");
    std::fs::write(&job_path, job.to_string()).unwrap();
    let r = run(&mut s, "cam.job", json!({"job_path": job_path, "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["machine_time"]["slowed_blocks"], 0, "{r}");

    // Bad rates are refused before anything is planned, never a panic.
    let with = |f: &dyn Fn(&mut Value)| {
        let mut job = barrel_job();
        job["machine_rates"] = mill_rates();
        f(&mut job);
        json!({"job": job, "dry_run": true})
    };
    let cases = [
        (with(&|j| j["machine_rates"]["rapid_xyz"] = json!(1)), "rapid_xyz"),
        (with(&|j| j["machine_rates"]["rapid_xyz_mm_min"] = json!(0)), "machine rate"),
        (with(&|j| j["machine_rates"]["max_a_deg_min"] = json!(-5)), "machine rate"),
        (with(&|j| j["machine_rates"]["max_feed_mm_min"] = json!(1e300)), "machine rate"),
        (with(&|j| j["machine_rates"]["spindle_rpm"] = json!([24000, 10000])), "spindle"),
        (with(&|j| j["machine_rates"]["spindle_start_seconds"] = json!(1e9)), "spindle"),
        (with(&|j| j["machine_rates"]["travel_mm"] = json!([300, 0, 130])), "travel"),
        (with(&|j| j["machine_rates"] = json!({"max_a_deg_min": 100})), "rapid_xyz_mm_min"),
        (with(&|j| j["machine_rates"] = json!("8000")), "machine_rates"),
        (with(&|j| j["machine_rates_path"] = json!("rates.json")), "not both"),
    ];
    for (p, word) in cases {
        let e = s.execute("cam.job", &p).unwrap_err();
        assert!(matches!(e, EngineError::BadParams { .. }) && e.to_string().contains(word), "{p}: {e}");
    }
    std::fs::write(dir.0.join("rates.json"), json!({"rapid_xyz_mm_min": 8000, "max_a": 1}).to_string()).unwrap();
    let e = s.execute("cam.job", &json!({"job_path": job_path, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("max_a"), "{e}");
    std::fs::write(dir.0.join("rates.json"), "{not json").unwrap();
    assert!(matches!(s.execute("cam.job", &json!({"job_path": job_path, "dry_run": true})), Err(EngineError::BadParams { .. })));
    std::fs::remove_file(dir.0.join("rates.json")).unwrap();
    assert!(matches!(s.execute("cam.job", &json!({"job_path": job_path, "dry_run": true})), Err(EngineError::BadParams { .. })));
}

#[test]
fn a_post_file_may_switch_the_coolant_on_after_the_index() {
    let mut s = pocketed_block();
    let dir = Dir::new("coolant");
    let mut post = run(&mut s, "cam.posts", json!({"name": "mach3"}))["post"].clone();
    post["coolant_after_turn"] = json!(true);
    std::fs::write(dir.0.join("mill.post.json"), post.to_string()).unwrap();
    let mut job = side_pocket_job();
    job["operations"][0]["operation"]["coolant"] = json!(true);
    let r = run(&mut s, "cam.job", json!({"job": job.clone(), "dry_run": true}));
    assert_clean(&r);
    let head: Vec<String> = r["program_head"].as_array().unwrap().iter().filter_map(Value::as_str).map(str::to_string).collect();
    let at = |l: &str| head.iter().position(|h| h == l).unwrap_or_else(|| panic!("no `{l}` in {head:?}"));
    // The built-in post keeps the coolant before the index, as it always has.
    assert!(at("M8") < head.iter().position(|h| h.contains("A90")).unwrap(), "{head:?}");
    let o = job.as_object_mut().unwrap();
    o.remove("post");
    o.insert("post_path".into(), json!("mill.post.json"));
    let job_path = dir.file("side.cam.json");
    std::fs::write(&job_path, job.to_string()).unwrap();
    let r = run(&mut s, "cam.job", json!({"job_path": job_path, "dry_run": true}));
    assert_clean(&r);
    let head: Vec<&str> = r["program_head"].as_array().unwrap().iter().filter_map(Value::as_str).collect();
    let turn = head.iter().position(|h| h.contains("A90")).unwrap();
    assert_eq!(head.get(turn + 1), Some(&"M8"), "{head:?}");
    assert_eq!(head.iter().filter(|h| **h == "M8").count(), 1, "{head:?}");
}

/// `cam.simulate` and `cam.job`'s `simulate`: the written program swept through the stock (the
/// part's box here, the job giving none) and held against the part.
#[test]
fn the_simulation_passes_a_good_program_and_catches_a_gouge_and_a_rapid() {
    let mut s = dome();
    let dir = Dir::new("sim");
    let out = dir.file("dome.tap");
    let r = run(&mut s, "cam.job", json!({"job": dome_job(), "output": out, "simulate": true}));
    assert_clean(&r);
    let sim = &r["simulation"];
    assert_eq!(sim["passed"], true, "{sim}");
    assert_eq!(sim["gouge"]["cells"], 0, "{sim}");
    assert!(sim["rapids_in_material"].as_array().is_some_and(Vec::is_empty), "{sim}");
    assert_eq!(sim["stock"]["kind"], "box", "{sim}");
    assert_eq!(sim["stock"]["derived"], true, "{sim}");
    assert!((sim["stock"]["max"][2].as_f64().unwrap() - 7.0).abs() < 0.01, "the dome's top: {sim}");
    assert!(sim["removed_volume"].as_f64().is_some_and(|v| v > 1000.0), "{sim}");
    assert!(sim["minutes"].as_f64().is_some_and(|m| m > 0.0), "{sim}");
    // The file on its own, with the stock left as an STL file.
    let stl = dir.file("stock.stl");
    let c = run(&mut s, "cam.simulate", json!({"path": out, "job": dome_job(), "stock_stl": stl}));
    assert_eq!(c["passed"], true, "{c}");
    assert_eq!(c["path"], out.as_str());
    assert_eq!(c["removed_volume"], sim["removed_volume"]);
    assert_eq!(c["stock_stl"], stl.as_str());
    assert!(std::fs::metadata(&stl).is_ok_and(|m| m.len() > 84 + 50 * 1000));
    // 1 mm into the block's top, then a rapid at Z5 through the stock above it (beside the dome).
    let bad = dir.file("bad.tap");
    std::fs::write(&bad, "G21\nT2 M6\nG0 X-18. Y-18. Z10.\nG1 Z-1. F300\nG1 X-10.\nG0 Z5.\nG0 Y10.\nG0 Z30.\n").unwrap();
    let c = run(&mut s, "cam.simulate", json!({"path": bad, "job": dome_job(), "resolution": 0.1}));
    assert_eq!(c["passed"], false, "{c}");
    assert!((c["gouge"]["worst"].as_f64().unwrap() - 1.0).abs() < 0.01, "{c}");
    let lines: Vec<u64> = c["rapids_in_material"].as_array().unwrap().iter().filter_map(|m| m["line"].as_u64()).collect();
    assert_eq!(lines, vec![7], "{c}");
    assert_eq!(c["failures"].as_array().map(Vec::len), Some(2), "{c}");
}

/// On a 4th axis the stock is the cylinder through the part's farthest point; a finish and an
/// indexed pocket pass the simulation inside the job.
#[test]
fn the_simulation_runs_on_a_4th_axis() {
    let mut s = bar();
    let r = run(&mut s, "cam.job", json!({"job": barrel_job(), "dry_run": true, "simulate": true}));
    assert_clean(&r);
    let sim = &r["simulation"];
    assert_eq!(sim["stock"]["kind"], "cylinder", "{sim}");
    assert!((sim["stock"]["radius"].as_f64().unwrap() - 10.0).abs() < 1e-6, "{sim}");
    assert_eq!(sim["gouge"]["cells"], 0, "{sim}");
    assert!(sim["leftover"]["max"].as_f64().is_some_and(|m| m < 0.01), "the stock is the bar itself: {sim}");
    // The block is cut from round bar through its corners (radius 15√2 ≈ 21.2): the pocket's
    // retract at 20 is inside that bar, but its ways down stop 1 mm above the bar (they used
    // to stop at the retract height, cutting material at rapid), and it feeds from there.
    let mut s = pocketed_block();
    let r = run(&mut s, "cam.job", json!({"job": side_pocket_job(), "dry_run": true, "simulate": true}));
    let sim = &r["simulation"];
    assert!((sim["stock"]["radius"].as_f64().unwrap() - 15.0 * 2f64.sqrt()).abs() < 1e-6, "{sim}");
    assert_clean(&r);
    assert!(sim["rapids_in_material"].as_array().is_some_and(Vec::is_empty), "{sim}");
    assert!(r["program_head"].as_array().unwrap().iter().any(|l| l == "G0 Z22.2132"), "{r}");
    assert_eq!(sim["gouge"]["cells"], 0, "{sim}");
    // With the block given as a box stock, the simulation still models the round bar through
    // its corners, so the ways down stop above that bar too, not 1 mm above the box's face.
    let mut boxed = side_pocket_job();
    boxed["stock"] = json!({"box": {"min": [-20, -15, -15], "max": [20, 15, 15]}});
    let r = run(&mut s, "cam.job", json!({"job": boxed, "dry_run": true, "simulate": true}));
    assert_clean(&r);
    assert!(r["simulation"]["rapids_in_material"].as_array().is_some_and(Vec::is_empty), "{r}");
    assert!(r["program_head"].as_array().unwrap().iter().any(|l| l == "G0 Z22.2132"), "{r}");
    // Retracting outside the bar it passes.
    let mut job = side_pocket_job();
    job["operations"][0]["operation"]["heights"]["retract"] = json!(22);
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true, "simulate": true}));
    assert_clean(&r);
    let sim = &r["simulation"];
    assert!(sim["removed_volume"].as_f64().is_some_and(|v| v > 12.0 * 12.0 * 5.0 * 0.9), "the pocket: {sim}");
    assert!(sim["rapids_in_material"].as_array().is_some_and(Vec::is_empty), "{sim}");
    // The same pocket into the solid face: the job's own check and the simulation both fail it.
    let mut solid = Session::default();
    run(&mut solid, "solid.box", json!({"length": 40, "width": 30, "height": 30, "corner": [-20, -15, -15]}));
    let r = run(&mut solid, "cam.job", json!({"job": side_pocket_job(), "dry_run": true, "simulate": true}));
    assert_eq!(r["simulation"]["passed"], false, "{r}");
    assert!(r["simulation"]["gouge"]["worst"].as_f64().is_some_and(|w| w > 4.0), "{r}");
    assert!(r["failures"].as_array().unwrap().iter().any(|f| f.as_str().is_some_and(|f| f.starts_with("simulation:"))), "{r}");
}

#[test]
fn hostile_simulations_are_refused_without_panics() {
    let mut s = dome();
    let dir = Dir::new("sim-hostile");
    let gcode = dir.file("p.tap");
    std::fs::write(&gcode, "G21\nT2 M6\nG0 X0. Y0. Z10.\nG1 Z8. F300\n").unwrap();
    let junk = dir.file("junk.tap");
    std::fs::write(&junk, "G21\nT2 M6\nG0 X0 Y0 Z10\n\u{0}garbage QQQ\n").unwrap();
    let five = json!({"name": "5ax", "rotaries": [
        {"letter": "A", "direction": [1, 0, 0], "center": [0, 0, 0], "carrier": "table"},
        {"letter": "C", "direction": [0, 0, 1], "center": [0, 0, 0], "carrier": "table"}]});
    let mut five_job = dome_job();
    five_job["machine"] = five;
    let cases = [
        json!({"path": gcode}),
        json!({"job": dome_job()}),
        json!({"path": gcode, "job": dome_job(), "job_path": dir.file("j.json")}),
        json!({"path": gcode, "job": dome_job(), "resolution": 0}),
        json!({"path": gcode, "job": dome_job(), "resolution": -1}),
        json!({"path": gcode, "job": dome_job(), "resolution": 1e-5}),
        json!({"path": gcode, "job": dome_job(), "resolution": 0.001}),
        json!({"path": gcode, "job": dome_job(), "resolution": "fine"}),
        json!({"path": gcode, "job": dome_job(), "stock_stl": ""}),
        json!({"path": gcode, "job": dome_job(), "stock_stl": gcode}),
        json!({"path": dir.file("missing.tap"), "job": dome_job()}),
        json!({"path": junk, "job": dome_job()}),
        json!({"path": gcode, "job": five_job}),
        json!({"path": gcode, "job": dome_job(), "resolutoin": 0.1}),
    ];
    for p in &cases {
        match s.execute("cam.simulate", p) {
            Err(EngineError::Internal(_, msg)) => panic!("{p}: panicked: {msg}"),
            Err(_) => {}
            Ok(v) => panic!("{p}: accepted: {v}"),
        }
    }
    // A tool the program loads that the job lacks.
    std::fs::write(&gcode, "G21\nT9 M6\nG0 X0. Y0. Z10.\n").unwrap();
    let e = s.execute("cam.simulate", &json!({"path": gcode, "job": dome_job()})).unwrap_err();
    assert!(e.to_string().contains("tool 9"), "{e}");
}

/// A 4th-axis program that never writes A has no move the simulation can follow: that is a
/// failure, not a pass with nothing checked.
#[test]
fn a_simulation_that_follows_no_move_does_not_pass() {
    let mut s = bar();
    let dir = Dir::new("sim-no-a");
    let gcode = dir.file("no-a.tap");
    std::fs::write(&gcode, "G21\nT3 M6\nG0 X0. Y0. Z30.\nG0 Z5.\nG1 X5. F100\nG0 Z30.\n").unwrap();
    let r = run(&mut s, "cam.simulate", json!({"path": gcode, "job": barrel_job(), "resolution": 0.2}));
    assert_eq!(r["passed"], false, "{r}");
    assert!(r["failures"].as_array().unwrap().iter().any(|f| f.as_str().is_some_and(|f| f.contains("no move"))), "{r}");
    // With the A written, the plunge into the bar is caught.
    std::fs::write(&gcode, "G21\nT3 M6\nG0 X0. Y0. Z30. A0.\nG0 Z5.\nG1 X5. F100\nG0 Z30.\n").unwrap();
    let r = run(&mut s, "cam.simulate", json!({"path": gcode, "job": barrel_job(), "resolution": 0.2}));
    assert_eq!(r["passed"], false, "{r}");
    assert!(r["rapids_in_material"].as_array().is_some_and(|m| !m.is_empty()), "{r}");
    assert!(r["gouge"]["worst"].as_f64().is_some_and(|w| w > 4.0), "{r}");
}

/// A radial hole drilled into a bar on a 4th axis is no gouge in the simulation at any cell
/// size: the cells whose rays slant down the hole's air are not measured as cut into the part.
#[test]
fn a_radial_hole_is_no_gouge_in_the_simulation_at_any_cell_size() {
    let mut s = Session::default();
    run(&mut s, "solid.cylinder", json!({"radius": 16, "height": 60, "base": [-30, 0, 0], "axis": "X", "body_name": "Bar"}));
    run(&mut s, "solid.cylinder", json!({"radius": 2.5, "height": 10, "base": [0, 0, 8], "axis": "Z", "operation": "cut"}));
    let job = json!({
        "machine": "4axis_a", "post": "mach3_4axis", "tools": [drill_tool(4, 5.0)],
        "stock": {"cylinder": {"radius": 16, "x_min": -30, "x_max": 30}},
        "operations": [{"name": "hole", "type": "drill", "tool": 4, "points": [[0, 0]],
            "heights": {"clearance": 30, "retract": 18, "top": 16, "bottom": 8}}]
    });
    for resolution in [0.1, 0.07, 0.05] {
        let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true, "simulate": true, "simulate_resolution": resolution}));
        assert_eq!(r["simulation"]["gouge"]["cells"], 0, "at {resolution} mm: {}", r["simulation"]);
        assert_eq!(r["simulation"]["resolution"], resolution, "{}", r["simulation"]);
        assert_clean(&r);
    }
}

/// A job file that names its machine file by a path beside it is read the same by `cam.job`
/// and `cam.simulate`.
#[test]
fn a_machine_file_beside_the_job_is_read_by_the_job_and_the_simulation() {
    let mut s = bar();
    let dir = Dir::new("machine-path");
    let turn = std::f64::consts::TAU;
    let machine = json!({"name": "A about X", "rotaries": [{"letter": "A", "direction": [1, 0, 0], "center": [0, 0, 0], "carrier": "table",
        "limits": [-turn, turn]}]});
    std::fs::write(dir.file("mill.machine.json"), machine.to_string()).unwrap();
    let mut job = barrel_job();
    job.as_object_mut().unwrap().remove("machine");
    job["machine_path"] = json!("mill.machine.json");
    let job_path = dir.file("barrel.cam.json");
    std::fs::write(&job_path, job.to_string()).unwrap();
    let gcode = dir.file("barrel.tap");
    let r = run(&mut s, "cam.job", json!({"job_path": job_path, "output": gcode}));
    assert_eq!(r["passed"], true, "{r}");
    let r = run(&mut s, "cam.simulate", json!({"path": gcode, "job_path": job_path, "resolution": 0.2}));
    assert_eq!(r["gouge"]["cells"], 0, "{r}");
    assert!(r["rapids_in_material"].as_array().is_some_and(Vec::is_empty), "{r}");
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

// The features wired together: the job's stock in the simulation and on the setup sheet.

/// A job's stock is what the simulation starts from (in cam.job and in cam.simulate) and what
/// the setup sheet and the summary tell the operator to clamp; facing its top 1 mm removes a
/// 50 × 50 × 1 slab of it.
#[test]
fn the_simulation_and_the_setup_sheet_start_from_the_jobs_stock() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 50, "width": 50, "height": 10, "corner": [-25, -25, -11], "body_name": "Block"}));
    let mill = json!({"number": 5, "kind": "flat_endmill", "diameter": 32, "flute_length": 6, "spindle_rpm": 9000, "feed": 1500, "plunge_feed": 400});
    let job = json!({
        "machine": "3axis", "post": "mach3_4axis", "tools": [mill], "stock": {"box": {"min": [-25, -25, -11], "max": [25, 25, 0]}},
        "operations": [{"name": "face", "type": "face", "tool": 5, "bottom": -1, "stepdown": 0.5}]
    });
    let dir = Dir::new("stock-wired");
    let (out, md) = (dir.file("face.tap"), dir.file("face-setup.md"));
    let r = scoped(&dir, || run(&mut s, "cam.job", json!({"job": job, "output": out, "setup_sheet": md, "simulate": true})));
    assert_clean(&r);
    let sim = &r["simulation"];
    assert_eq!(sim["passed"], true, "{sim}");
    assert_eq!(sim["stock"]["derived"], false, "the job's stock: {sim}");
    assert_eq!(sim["stock"]["max"], json!([25.0, 25.0, 0.0]), "{sim}");
    assert_eq!(sim["stock"]["min"], json!([-25.0, -25.0, -11.0]), "{sim}");
    let removed = sim["removed_volume"].as_f64().unwrap();
    assert!((removed - 2500.0).abs() < 100.0, "a 50 x 50 x 1 slab: {removed}");
    assert!(sim["notes"].as_array().unwrap().iter().any(|n| n.as_str().is_some_and(|n| n.contains("the job's stock"))), "{sim}");
    // The summary and the sheet say what to clamp.
    let stock = r["summary"]["stock"].as_str().unwrap_or_default();
    assert!(stock.starts_with("A block 50 x 50 x 11 mm"), "{}", r["summary"]);
    let sheet = std::fs::read_to_string(&md).unwrap();
    assert!(sheet.contains("## Stock") && sheet.contains("A block 50 x 50 x 11 mm"), "{sheet}");
    // cam.simulate of the file on its own reads the same stock from the job.
    let c = run(&mut s, "cam.simulate", json!({"path": out, "job": job}));
    assert_eq!(c["stock"]["derived"], false, "{c}");
    assert_eq!(c["removed_volume"], sim["removed_volume"], "{c}");
    // A bar stock cannot be modelled on a 3-axis machine: the part's box, said so.
    let mut bar_job = job.clone();
    bar_job["stock"] = json!({"cylinder": {"radius": 40, "x_min": -25, "x_max": 25}});
    let c = run(&mut s, "cam.simulate", json!({"path": out, "job": bar_job}));
    assert_eq!(c["stock"]["derived"], true, "{c}");
    assert!(c["notes"].as_array().unwrap().iter().any(|n| n.as_str().is_some_and(|n| n.contains("not the job's bar stock"))), "{c}");
    // Without a stock the summary has none and the sheet no stock section.
    let mut bare = job.clone();
    bare["stock"] = Value::Null;
    if let Some(o) = bare.as_object_mut() {
        o.remove("stock");
    }
    bare["operations"][0]["top"] = json!(0);
    let r = run(&mut s, "cam.job", json!({"job": bare, "dry_run": true}));
    assert!(r["summary"]["stock"].is_null(), "{}", r["summary"]);
}

/// A helical groove `turns` times round the bar of radius 10, 10 mm along X a turn, 0.5 deep,
/// as a wrap operation's flat points.
fn groove_points(turns: f64) -> Vec<[f64; 3]> {
    let around = std::f64::consts::TAU * 10.0;
    let n = (turns * 120.0).round() as usize;
    (0..=n).map(|k| k as f64 / 120.0).map(|t| [-15.0 + 10.0 * t, around * t, -0.5]).collect()
}

#[test]
fn operation_minutes_add_up_with_their_rewinds() {
    let mut s = bar();
    let dir = Dir::new("rewind");
    let mut post = run(&mut s, "cam.posts", json!({"name": "mach3"}))["post"].clone();
    post["rewind"] = json!({"distance": 25.0, "retract_feed": 500.0, "plunge_feed": 250.0});
    let post_path = dir.file("rewind.post.json");
    std::fs::write(&post_path, post.to_string()).unwrap();
    let job = json!({
        "machine": "4axis_a", "a_limits_deg": [-360, 360], "post_path": post_path, "tools": [ball(3, 2.0)],
        "operations": [{"name": "groove", "type": "wrap", "tool": 3, "radius": 10, "points": groove_points(3.0), "gouge_check": false}]
    });
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_eq!(r["passed"], true, "{r}");
    // Three turns within ±360°: one rewind, and its feeds are in the operation's minutes.
    assert_eq!(r["operations"][0]["rewinds"], 1, "{r}");
    let total: f64 = r["operations"].as_array().unwrap().iter().filter_map(|o| o["cut_minutes"].as_f64()).sum();
    let read = r["readback"]["cut_minutes"].as_f64().unwrap();
    assert!((total - read).abs() <= 2e-3 * read, "{total} min against {read} min read back: {r}");
}

#[test]
fn an_operation_without_the_gouge_check_is_still_checked_against_the_fixtures() {
    let mut s = bar();
    // A clamp over the top of the bar where an engraving runs along it.
    run(&mut s, "solid.box", json!({"length": 10, "width": 6, "height": 4, "corner": [-5, -3, 8.5], "operation": "new", "body_name": "Clamp"}));
    let job = |fixtures: Value| {
        json!({
            "machine": "4axis_a", "post": "mach3_4axis", "tools": [ball(3, 2.0)], "bodies": ["Bar"], "fixtures": fixtures,
            "operations": [{"name": "engrave", "type": "wrap", "tool": 3, "radius": 10, "points": [[-12, 0, -0.5], [12, 0, -0.5]], "gouge_check": false}]
        })
    };
    let r = run(&mut s, "cam.job", json!({"job": job(json!(["Clamp"])), "dry_run": true}));
    assert_eq!(r["passed"], false, "{r}");
    assert_eq!(r["gouge"]["skipped"], json!(["engrave"]), "{r}");
    assert!(r["gouge"]["hits"].as_u64().is_some_and(|h| h > 0), "{r}");
    assert!(r["failures"].as_array().unwrap().iter().any(|f| f.as_str().is_some_and(|f| f.contains("into a fixture"))), "{r}");
    // Without the clamp as a fixture the engraving passes: the part itself is not checked.
    let r = run(&mut s, "cam.job", json!({"job": job(Value::Null), "dry_run": true}));
    assert_eq!(r["passed"], true, "{r}");
    assert_eq!(r["gouge"]["skipped"], json!(["engrave"]), "{r}");
}

#[test]
fn output_and_preview_are_never_the_same_file() {
    let mut s = dome();
    let dir = Dir::new("same");
    std::fs::create_dir_all(dir.0.join("sub")).unwrap();
    let out = dir.file("a.tap");
    let mut spellings = vec![dir.0.join(".").join("a.tap"), dir.0.join("sub").join("..").join("a.tap")];
    if cfg!(windows) {
        spellings.push(dir.0.join("A.TAP"));
        // Windows drops dots and spaces at the end of a name: these open `a.tap` too.
        spellings.push(dir.0.join("a.tap."));
        spellings.push(dir.0.join("a.tap. ."));
    }
    for page in spellings {
        let page = page.to_string_lossy().into_owned();
        let e = s.execute("cam.job", &json!({"job": dome_job(), "output": out, "preview": page})).unwrap_err();
        assert!(e.to_string().contains("different files"), "{page}: {e}");
    }
    assert!(!PathBuf::from(&out).exists(), "nothing was written");
    let r = run(&mut s, "cam.job", json!({"job": dome_job(), "output": out, "preview": dir.file("a.html")}));
    assert_clean(&r);
}

#[test]
fn a_dry_run_says_when_the_output_is_not_the_posts_extension() {
    let mut s = dome();
    let dir = Dir::new("ext");
    let r = run(&mut s, "cam.job", json!({"job": dome_job(), "output": dir.file("dome.nc"), "dry_run": true}));
    let notes: Vec<&str> = r["notes"].as_array().unwrap().iter().filter_map(Value::as_str).collect();
    assert!(notes.iter().any(|n| n.contains("writes .tap files; this one is .nc")), "{notes:?}");
    assert!(!PathBuf::from(dir.file("dome.nc")).exists());
}

#[test]
fn every_example_in_the_cam_jobs_doc_is_accepted() {
    let doc = include_str!("../../../../docs/cam-jobs.md");
    let blocks: Vec<&str> = doc.split("```json").skip(1).filter_map(|b| b.split("```").next()).collect();
    assert!(blocks.len() >= 2, "{blocks:?}");
    let mut jobs = 0;
    for block in blocks {
        // A block of a job's keys alone is read inside braces.
        let v: Value = serde_json::from_str(block)
            .or_else(|_| serde_json::from_str::<Value>(&format!("{{{block}}}")).map(|_| json!({})))
            .unwrap_or_else(|e| panic!("{e}: {block}"));
        if v.as_object().is_some_and(serde_json::Map::is_empty) {
            continue;
        }
        if v.get("operations").is_some() {
            serde_json::from_value::<crate::schema::cam::CamJob>(v.clone()).unwrap_or_else(|e| panic!("{e}: {block}"));
            // Planned on a part with a built-in post in place of the example's post file.
            let mut job = v;
            if let Some(o) = job.as_object_mut() {
                o.remove("post_path");
                o.insert("post".into(), json!("mach3_4axis"));
            }
            let r = pocketed_block().execute("cam.job", &json!({"job": job.clone(), "dry_run": true})).unwrap_or_else(|e| panic!("{e}"));
            assert_eq!(r["operations"].as_array().map(Vec::len), job["operations"].as_array().map(Vec::len), "{r}");
            jobs += 1;
        } else if v.get("path").is_some() {
            // `cam.simulate`'s parameters.
            serde_json::from_value::<crate::schema::cam_sim::CamSimulateParams>(v).unwrap_or_else(|e| panic!("{e}: {block}"));
        } else if v.get("type").is_some() {
            // One operation of a job.
            serde_json::from_value::<crate::schema::cam::OperationSpec>(v).unwrap_or_else(|e| panic!("{e}: {block}"));
        } else {
            // `cam.job`'s parameters, else `cam.features`' or `cam.suggest`'s.
            let job = serde_json::from_value::<crate::schema::cam::CamJobParams>(v.clone()).map(|_| ());
            let features = || serde_json::from_value::<crate::schema::cam_features::CamFeaturesParams>(v.clone()).map(|_| ());
            let suggest = || serde_json::from_value::<crate::schema::cam_features::CamSuggestParams>(v.clone()).map(|_| ());
            job.or_else(|_| features()).or_else(|_| suggest()).unwrap_or_else(|e| panic!("{e}: {block}"));
        }
    }
    assert!(jobs >= 1);
    // An indexed operation takes its name from itself when the inner one has none; with neither,
    // and an operation without a name, are refused.
    let mut job = side_pocket_job();
    job["operations"][0]["name"] = json!("wrapped");
    job["operations"][0]["operation"].as_object_mut().unwrap().remove("name");
    let r = pocketed_block().execute("cam.job", &json!({"job": job.clone(), "dry_run": true})).unwrap();
    assert_eq!(r["operations"][0]["name"], "wrapped", "{r}");
    job["operations"][0].as_object_mut().unwrap().remove("name");
    let e = pocketed_block().execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("needs a `name`"), "{e}");
    let mut job = dome_job();
    job["operations"][0].as_object_mut().unwrap().remove("name");
    let e = dome().execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
    assert!(e.to_string().contains("needs a `name`"), "{e}");
}

/// Below the stock's bottom are the parallels, the vise's floor or the table that hold it: a
/// contour cut through past the bottom, or holes drilled past it, are refused naming the
/// operation, how deep and the fix (it used to be checked for `face` alone). Down to the bottom
/// is fine, and without a stock nothing says where the bottom is.
#[test]
fn cutting_below_the_stocks_bottom_is_refused_for_every_operation() {
    let mut s = drilled_plate(20.0);
    let stock = json!({"box": {"min": [-35, -25, -20], "max": [35, 25, 0]}});
    let contour = |bottom: f64| {
        json!({
            "name": "outline", "type": "contour", "tool": 1, "polygon": [[-30, -20], [30, -20], [30, 20], [-30, 20]],
            "heights": {"clearance": 15, "retract": 2, "top": 0, "bottom": bottom}, "stepdown": 5
        })
    };
    let holes = |bottom: f64| {
        json!({
            "name": "holes", "type": "drill", "tool": 3, "sketch": "Holes", "diameter_range": [5, 7],
            "heights": {"clearance": 15, "retract": 2, "top": 0, "bottom": bottom}
        })
    };
    let job = |ops: Value, stock: Option<&Value>| {
        let mut j = json!({"machine": "3axis", "post": "mach3_4axis", "tools": [flat(1, 6.0), drill_tool(3, 6.0)], "operations": ops});
        if let Some(st) = stock {
            j["stock"] = st.clone();
        }
        json!({"job": j, "dry_run": true})
    };
    let below = |r: &Value| -> Vec<String> {
        r["failures"].as_array().unwrap().iter().filter_map(Value::as_str).filter(|f| f.starts_with("below the stock")).map(str::to_string).collect()
    };
    let r = run(&mut s, "cam.job", job(json!([contour(-21.0), holes(-22.5)]), Some(&stock)));
    let found = below(&r);
    assert_eq!(found.len(), 2, "{r}");
    assert!(found[0].contains("`outline`") && found[0].contains("1 mm below") && found[0].contains("Z-20"), "{found:?}");
    assert!(found[1].contains("`holes`") && found[1].contains("2.5 mm below"), "{found:?}");
    assert!(found.iter().all(|f| f.contains("raise")), "{found:?}");
    assert_eq!(r["passed"], false, "{r}");
    // Without a dry run it is refused, nothing written.
    let mut refused = job(json!([contour(-21.0)]), Some(&stock));
    refused["dry_run"] = json!(false);
    refused["output"] = json!(solidcraft_io::vfs::temp_dir().join("never-written.tap").to_string_lossy());
    let e = s.execute("cam.job", &refused).unwrap_err();
    assert!(e.to_string().contains("below the stock") && e.to_string().contains("outline"), "{e}");
    // Down to the bottom, and without a stock: no such failure.
    let r = run(&mut s, "cam.job", job(json!([contour(-20.0), holes(-20.0)]), Some(&stock)));
    assert!(below(&r).is_empty(), "{r}");
    let r = run(&mut s, "cam.job", job(json!([contour(-21.0), holes(-22.5)]), None));
    assert!(below(&r).is_empty(), "{r}");
    // The stock standing on a parallel given as a fixture (6 mm tall, clear of the cuts): the
    // cuts may go through into the room it stands the stock up in, down to its bottom (Z-26),
    // as they are checked against it; below that they are refused.
    run(&mut s, "solid.box", json!({"length": 50, "width": 2, "height": 6, "corner": [-25, -6, -26], "operation": "new", "body_name": "Parallel"}));
    let on_parallel = |ops: Value| {
        let mut j = job(ops, Some(&stock));
        j["job"]["bodies"] = json!(["Plate"]);
        j["job"]["fixtures"] = json!(["Parallel"]);
        j
    };
    let r = run(&mut s, "cam.job", on_parallel(json!([contour(-21.0), holes(-22.5)])));
    assert!(below(&r).is_empty() && r["gouge"]["pass"] == true, "{r}");
    let r = run(&mut s, "cam.job", on_parallel(json!([contour(-27.0)])));
    let found = below(&r);
    assert!(found.len() == 1 && found[0].contains("1 mm below the lowest fixture (Z-26)"), "{r}");
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
    // After an indexed pattern (one job operation, several in the program) each line is numbered
    // and given the worked-out heights of the job operation it comes from.
    let mut s = square_block();
    let mut last = face_op("last", 40.0);
    last["name"] = json!("last");
    last.as_object_mut().unwrap().remove("heights");
    let job = faces_job(json!([{"type": "indexed_pattern", "count": 2, "operation": face_op("face", 40.0)}, last]));
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true, "compact": true}));
    let lines: Vec<&str> = r["operations"].as_array().unwrap().iter().filter_map(Value::as_str).collect();
    assert_eq!(lines.len(), 3, "{r}");
    assert!(lines[0].starts_with("0 `face A0`") && !lines[0].contains("worked out"), "{lines:?}");
    assert!(lines[1].starts_with("0 `face A180`") && !lines[1].contains("worked out"), "{lines:?}");
    assert!(lines[2].starts_with("1 `last`") && lines[2].contains("worked out"), "{lines:?}");
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
    // A clearance given above the part's top but inside its reach is refused with the fix, by
    // the setup check where A turns (the part reaches 42.43 mm there).
    let job = turning_job("mach3_4axis", Some(json!({"clearance": 40, "retract": 32})));
    let e = s.execute("cam.job", &json!({"job": job, "dry_run": true})).unwrap_err();
    let r = refusals(&e);
    assert_eq!(r[0]["check"], "clearance", "{e}");
    let fix = r[0]["fix"].as_str().unwrap();
    assert!(fix.contains("axis") && fix.contains("42.43"), "{fix}");
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

/// With a stock standing 2 mm above the part, heights left out start at the stock's top (the
/// material is cut from there down) and the retract and clearance clear it, so the job's rapid
/// check against the stock passes; the part's floor stays the bottom.
#[test]
fn omitted_heights_start_at_the_stocks_top() {
    let mut s = block_with_pocket();
    let ops = json!([pocket_op(json!(4))]);
    let job = with_key(agent_job(ops), "stock", json!({"box": {"min": [-25, -20, 0], "max": [25, 20, 22]}}));
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": true}));
    assert_clean(&r);
    assert_eq!(r["rapids"]["pass"], true, "{r}");
    let d = &r["derived_heights"][0];
    let h = ["clearance", "retract", "top", "bottom"].map(|k| d[k].as_f64().unwrap());
    assert!(h.iter().zip([32.0, 24.0, 22.0, 15.0]).all(|(a, b)| (a - b).abs() < 1e-6), "{h:?} {r}");
    assert!(d["why"].as_str().unwrap().contains("stock's top"), "{r}");
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
    // A pocket cut from a polygon works out the heights it leaves out from the part (the
    // machine profiles' heights); this triangle is too small for the 6 mm tool, and the fix
    // says which fits.
    let poly = json!([{"name": "p", "type": "pocket", "tool": 6, "polygon": [[-5, -5], [5, -5], [5, 5]], "heights": {"top": 0}}]);
    let e = s.execute("cam.job", &job(poly)).unwrap_err();
    assert!(e.to_string().contains("does not fit") && e.to_string().contains("max_diameter"), "{e}");
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
    // Said in one line of words, not with the source's indentation in it.
    assert!(sug["check"].as_array().unwrap().iter().all(|c| !c.as_str().unwrap().contains("  ")), "{sug}");
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

/// Whether two colours are within `within` of each other in every channel.
fn near_rgb(a: [u8; 3], b: [u8; 3], within: i32) -> bool {
    a.iter().zip(b).all(|(x, y)| (i32::from(*x) - i32::from(y)).abs() <= within)
}

/// A PNG `path` decoded: its size and its RGB pixels, rows top to bottom.
fn png_pixels(path: &str) -> (u32, u32, Vec<[u8; 3]>) {
    let img = image::open(path).unwrap_or_else(|e| panic!("{path}: {e}")).to_rgb8();
    let (w, h) = img.dimensions();
    (w, h, img.pixels().map(|p| p.0).collect())
}

/// Where the camera of an `images` reply puts the design point `p` on a `w` × `h` picture.
fn projected(images: &Value, p: [f64; 3], w: u32, h: u32) -> (f64, f64) {
    let c = &images["camera"];
    let at = |v: &Value| v.as_f64().unwrap();
    let cam = solidcraft_render::Camera {
        target: solidcore_record::Vec3::new(at(&c["target"][0]), at(&c["target"][1]), at(&c["target"][2])),
        yaw: at(&c["yaw"]),
        pitch: at(&c["pitch"]),
        distance: at(&c["distance"]),
        fov: at(&c["fov"]),
        ..solidcraft_render::Camera::default()
    };
    let (x, y, _) = cam.to_screen(solidcore_record::Vec3::new(p[0], p[1], p[2]), f64::from(w), f64::from(h), 1.0e4).unwrap();
    (x, y)
}

/// `cam.job {image, images_dir}`: a picture an agent can look at, drawn headless (no GPU needed),
/// the operation's colour where its toolpath projects, written only inside the agent's folders;
/// and the toolpath's extents in work coordinates in the reply.
#[test]
fn a_dry_run_draws_the_toolpaths_in_their_colours_inside_the_agents_folders() {
    let mut s = dome();
    let dir = Dir::new("image");
    let root = dir.0.join("allowed");
    std::fs::create_dir_all(&root).unwrap();
    let picture = root.join("dome.png").to_string_lossy().into_owned();
    let pictures = root.join("ops").to_string_lossy().into_owned();
    let scope = Some(solidcraft_io::vfs::PathScope::new([root.clone()]));
    let r = solidcraft_io::vfs::scoped(scope.clone(), || {
        run(&mut s, "cam.job", json!({"job": dome_job(), "dry_run": true, "image": picture, "images_dir": pictures, "width": 320, "height": 200}))
    });
    assert_clean(&r);
    let images = &r["images"];
    assert_eq!((images["view"].as_str(), images["width"].as_u64(), images["height"].as_u64()), (Some("iso"), Some(320), Some(200)), "{r}");
    assert_eq!(images["image"], picture.as_str(), "{r}");
    assert_eq!(images["legend"][0], json!({"operation": "finish", "colour": "orange", "rgb": "#f07800"}), "{r}");
    assert!(images["key"].as_array().is_some_and(|k| k.len() >= 6), "{r}");
    let one = root.join("ops").join("01-finish.png");
    assert_eq!(images["images"], json!([one.to_string_lossy()]), "{r}");
    // A dry run writes the pictures, never a program.
    assert!(r["outputs"]["gcode"].is_null(), "{r}");
    for path in [picture.clone(), one.to_string_lossy().into_owned()] {
        let (w, h, px) = png_pixels(&path);
        assert_eq!((w, h), (320, 200), "{path}");
        let colours: std::collections::BTreeSet<[u8; 3]> = px.iter().copied().collect();
        assert!(colours.len() > 50, "{path}: a picture, not a blank: {} colours", colours.len());
        // The finish is orange where its toolpath projects: inside the box its extents span.
        let e = &r["operations"][0]["extents"]["all"];
        let (mut x0, mut x1, mut y0, mut y1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for k in 0..8usize {
            let c = |i: usize| e[(k >> i) & 1][i].as_f64().unwrap();
            let (x, y) = projected(images, [c(0), c(1), c(2)], w, h);
            (x0, x1, y0, y1) = (x0.min(x), x1.max(x), y0.min(y), y1.max(y));
        }
        let orange: Vec<(u32, u32)> =
            px.iter().enumerate().filter(|(_, c)| near_rgb(**c, [240, 120, 0], 30)).map(|(i, _)| (i as u32 % w, i as u32 / w)).collect();
        let on_path = |x: u32, y: u32| (x0 - 6.0..=x1 + 6.0).contains(&f64::from(x)) && (y0 - 6.0..=y1 + 6.0).contains(&f64::from(y));
        let inside = orange.iter().filter(|(x, y)| on_path(*x, *y)).count();
        assert!(orange.len() > 200 && inside * 10 >= orange.len() * 9, "{path}: {} orange pixels, {inside} on the path's box", orange.len());
    }
    // The extents, in work coordinates: the ball's tip stays above the floor; the feed moves start
    // at the retract height, where the plunges begin.
    let x = &r["operations"][0]["extents"];
    let (lo, hi) = (x["cut"][0][2].as_f64().unwrap(), x["cut"][1][2].as_f64().unwrap());
    assert!((-1.0 - 1e-6..1.0).contains(&lo) && (hi - 15.0).abs() < 1e-6, "{x}");
    assert_eq!(x["all"][1][2].as_f64(), Some(30.0), "rapids up to the clearance: {x}");
    assert!(x["rotary"].as_array().is_some_and(Vec::is_empty) && x["summary"].as_str().is_some_and(|t| t.starts_with("cut X -")), "{x}");
    // Outside the folders nothing is computed or written.
    let outside = dir.file("dome.png");
    let e = solidcraft_io::vfs::scoped(scope.clone(), || {
        s.execute("cam.job", &json!({"job": dome_job(), "dry_run": true, "image": outside})).unwrap_err()
    });
    assert!(e.to_string().contains("outside the folders this agent may use"), "{e}");
    let e =
        solidcraft_io::vfs::scoped(scope.clone(), || s.execute("cam.view", &json!({"job": dome_job(), "path": dir.file("view.png")})).unwrap_err());
    assert!(e.to_string().contains("outside the folders this agent may use"), "{e}");
    assert!(!PathBuf::from(&outside).exists() && !dir.0.join("view.png").exists());
    // Every refusal says what to change.
    for (p, says) in [
        (json!({"job": dome_job(), "dry_run": true, "view": "top"}), "give `image`"),
        (json!({"job": dome_job(), "dry_run": true, "image": picture, "view": "sideways"}), "iso, top, bottom"),
        (json!({"job": dome_job(), "dry_run": true, "image": picture, "width": 5}), "16 to 4096"),
        (json!({"job": dome_job(), "dry_run": true, "image": ""}), "must not be empty"),
        (json!({"job": dome_job(), "output": picture, "image": picture}), "a file of its own"),
    ] {
        let e = solidcraft_io::vfs::scoped(scope.clone(), || s.execute("cam.job", &p).unwrap_err());
        assert!(e.to_string().contains(says), "{p}: {e}");
    }
}

/// `cam.view` on a 4th-axis job looks from the front (from -Y, the A axis level across the
/// picture) and draws the A axis; the extents give the A range the program turns through.
#[test]
fn a_4axis_view_looks_along_the_a_axis_and_gives_its_range() {
    let mut s = bar();
    let dir = Dir::new("view");
    let path = dir.file("bar.png");
    let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
        run(&mut s, "cam.view", json!({"job": barrel_job(), "path": path, "width": 300, "height": 200}))
    });
    assert_eq!(r["passed"], true, "{r}");
    assert_eq!(r["path"], path.as_str(), "{r}");
    let image = &r["image"];
    assert_eq!(image["view"], "front", "{r}");
    assert_eq!((image["camera"]["yaw"].as_f64(), image["camera"]["pitch"].as_f64()), (Some(0.0), Some(0.0)), "{r}");
    assert!(image["key"].as_array().unwrap().iter().any(|k| k.as_str().is_some_and(|k| k.contains("rotary table's axis"))), "{r}");
    let (w, h, px) = png_pixels(&path);
    assert_eq!((w, h), (300, 200));
    // The A axis runs level across the picture through work zero.
    let (_, y) = projected(image, [0.0, 0.0, 0.0], w, h);
    let row = y.round() as u32;
    let dark = (0..w)
        .filter(|x| (row.saturating_sub(1)..=row + 1).any(|y| px.get((y * w + x) as usize).is_some_and(|c| near_rgb(*c, [35, 35, 45], 30))))
        .count();
    assert!(dark > 20, "{dark} pixels of the A axis on row {row}");
    assert!(px.iter().filter(|c| near_rgb(**c, [240, 120, 0], 30)).count() > 100, "the barrel finish is drawn");
    let op = &r["operations"][0];
    assert_eq!((op["name"].as_str(), op["drawn"].as_bool()), (Some("barrel"), Some(true)), "{r}");
    let a = &op["extents"]["rotary"][0];
    assert_eq!(a["letter"], "A", "{r}");
    assert!(a["max_deg"].as_f64().unwrap() - a["min_deg"].as_f64().unwrap() >= 330.0, "{r}");
    assert!(op["extents"]["summary"].as_str().is_some_and(|t| t.contains("; A ")), "{r}");
    // A refusal names what there is.
    let e = s.execute("cam.view", &json!({"job": barrel_job(), "path": path, "op": "nope"})).unwrap_err();
    assert!(e.to_string().contains("no operation `nope`") && e.to_string().contains("`barrel`"), "{e}");
    for p in [
        json!({}),
        json!({"path": ""}),
        json!({"job": barrel_job()}),
        json!({"job": barrel_job(), "path": path, "width": 0}),
        json!({"job": barrel_job(), "path": path, "view": 7}),
        json!({"job": barrel_job(), "path": path, "dry_run": true}),
        json!({"job": barrel_job(), "job_path": "x.json", "path": path}),
    ] {
        match s.execute("cam.view", &p) {
            Err(EngineError::Internal(_, msg)) => panic!("{p}: panicked: {msg}"),
            Err(_) => {}
            Ok(v) => panic!("{p}: accepted: {v}"),
        }
    }
}

/// The rotary extents are the angles the written program turns through: a long helix on an A
/// that stops at ±360° is unwound by rewinds, so its range stays within the limits and matches
/// what `cam.check` reads from the file, operation by operation (chaining the poses' angles
/// would report thousands of degrees the machine never turns).
#[test]
fn rotary_extents_are_what_the_written_program_turns() {
    let mut s = bar();
    let dir = Dir::new("unwound");
    let mut post = run(&mut s, "cam.posts", json!({"name": "mach3"}))["post"].clone();
    post["rewind"] = json!({"distance": 25.0, "retract_feed": 500.0, "plunge_feed": 250.0});
    std::fs::write(dir.0.join("rewind.post.json"), post.to_string()).unwrap();
    let helix = |name: &str| {
        json!({"name": name, "type": "rotary_finish", "tool": 3, "pattern": {"type": "helix", "pitch": 2},
               "clearance": 25, "retract": 15, "range": [-15, 15]})
    };
    let job = json!({
        "machine": "4axis_a", "a_limits_deg": [-360, 360], "post_path": "rewind.post.json", "tools": [ball(3, 4.0)],
        "operations": [helix("first helix"), helix("second helix")]
    });
    let job_path = dir.file("helix.cam.json");
    std::fs::write(&job_path, job.to_string()).unwrap();
    let out = dir.file("helix.tap");
    let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
        run(&mut s, "cam.job", json!({"job_path": job_path, "output": out}))
    });
    assert_eq!(r["passed"], true, "{r}");
    let c = run(&mut s, "cam.check", json!({"path": out}));
    let file = &c["rotary_ranges"][0];
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for op in r["operations"].as_array().unwrap() {
        let a = &op["extents"]["rotary"][0];
        assert_eq!(a["letter"], "A", "{op}");
        let (a0, a1) = (a["min_deg"].as_f64().unwrap(), a["max_deg"].as_f64().unwrap());
        assert!(a0 >= -360.0 - 1e-6 && a1 <= 360.0 + 1e-6, "{}: A {a0}..{a1} beyond the limits the program keeps to: {op}", op["name"]);
        assert!(a1 - a0 >= 330.0, "{op}");
        (lo, hi) = (lo.min(a0), hi.max(a1));
    }
    let (f0, f1) = (file["min_deg"].as_f64().unwrap(), file["max_deg"].as_f64().unwrap());
    assert!((lo - f0).abs() < 0.01 && (hi - f1).abs() < 0.01, "extents A {lo}..{hi}, the written program A {f0}..{f1}");
}

/// A dry run of a job that gouges says where: each failing operation's extents list the places
/// (work coordinates, move, depth), its summary names the first, and the picture marks them
/// with red crosses an agent can see.
#[test]
fn a_gouging_dry_run_shows_where_the_gouges_are() {
    let mut s = dome();
    let dir = Dir::new("gouge");
    let path = dir.file("gouge.png");
    // A contour round a 10 mm square down to Z 0 runs through the dome.
    let job = json!({
        "name": "through the dome", "machine": "3axis", "post": "mach3_4axis", "tools": [flat(1, 4.0)],
        "operations": [{
            "name": "ring", "type": "contour", "tool": 1, "polygon": [[-5, -5], [5, -5], [5, 5], [-5, 5]],
            "heights": {"clearance": 30, "retract": 15, "top": 7, "bottom": 0}, "stepdown": 2
        }]
    });
    let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
        run(&mut s, "cam.view", json!({"job": job, "path": path, "view": "top", "width": 320, "height": 240}))
    });
    assert_eq!(r["passed"], false, "{r}");
    let x = &r["operations"][0]["extents"];
    let problems = x["problems"].as_array().unwrap_or_else(|| panic!("no problems listed: {x}"));
    assert!(!problems.is_empty() && problems.len() <= 20, "{x}");
    let first = &problems[0];
    assert_eq!(first["check"], "gouge", "{x}");
    assert!(first["distance"].as_f64().is_some_and(|d| d > 0.0) && first["move_index"].is_u64(), "{x}");
    let at: Vec<f64> = first["at"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
    // The tool's centre runs 2 mm outside the square, inside the dome (radius 12, its top at 7).
    let out = at[0].abs().max(at[1].abs());
    assert!(out > 6.0 && out < 8.0 && (0.0..7.0).contains(&at[2]), "{at:?}");
    assert!(x["summary"].as_str().is_some_and(|t| t.contains("; gouge at X ")), "{x}");
    assert!(r["image"]["key"].as_array().unwrap().iter().any(|k| k.as_str().is_some_and(|k| k.contains("red crosses"))), "{r}");
    let (w, h, px) = png_pixels(&path);
    let (cx, cy) = projected(&r["image"], [at[0], at[1], at[2]], w, h);
    let red = (0..px.len())
        .filter(|&i| near_rgb(px[i], [255, 0, 0], 25))
        .filter(|&i| (f64::from(i as u32 % w) - cx).abs() <= 8.0 && (f64::from(i as u32 / w) - cy).abs() <= 8.0)
        .count();
    assert!(red >= 6, "{red} red pixels at the first gouge ({cx}, {cy})");
}

/// A machine model for the view tests: a 300 × 200 table, a plate on the spindle.
const VIEW_MODEL: &str = r#"{"format": "solidcraft.machine_model", "version": 1, "name": "test mill",
    "rates": {"rapid_xyz_mm_min": 6000, "accel_mm_s2": 1500, "tool_change_seconds": 20},
    "spindle": {"nose_gap": 5, "nose_max_above_table": 400},
    "table": {"size": [300, 200, 15], "cut_allowance": 0.1},
    "parts": [{"name": "the Z plate", "code": "z-plate", "mount": "spindle", "shape": {"box": {"min": [-40, 30, -1], "max": [40, 45, 200]}}}]}"#;

const VIEW_VISE: &str = r#"{"format": "solidcraft.workholding", "version": 1, "holding": "vise",
    "vise": {"model": {"length": 175, "width": 64, "height": 64, "floor": 32, "fixed_jaw": 35.6, "moving_jaw": 50, "max_opening": 85},
             "center": [150, 100]},
    "parallels": {"height": 10, "thickness": 3, "length": 150}}"#;

#[test]
fn the_simulation_view_loads_seeks_and_reports_a_job() {
    let mut s = dome();
    // Before a load nothing but load is possible.
    for id in ["cam.view.seek", "cam.view.state", "cam.view.play", "cam.view.pause", "cam.view.show"] {
        let p = if id == "cam.view.seek" { json!({"move": 0}) } else { json!({}) };
        let e = s.execute(id, &p).unwrap_err();
        assert!(e.to_string().contains("cam.view.load"), "{id}: {e}");
    }
    let dir = Dir::new("view");
    let (model, vise) = (dir.file("m.machine-model.json"), dir.file("v.workholding.json"));
    std::fs::write(&model, VIEW_MODEL).unwrap();
    std::fs::write(&vise, VIEW_VISE).unwrap();
    let mut job = dome_job();
    job["stock"] = json!({"box": {"min": [-20, -20, -20], "max": [20, 20, 8]}});
    let r = run(&mut s, "cam.view.load", json!({"job": job, "machine_model_path": model, "workholding_path": vise, "resolution": 0.25}));
    let moves = r["moves"].as_u64().unwrap() as usize;
    assert!(moves > 50, "{r}");
    assert_eq!(r["operations"].as_array().unwrap().len(), 1, "{r}");
    assert_eq!(r["operations"][0]["name"], "finish");
    assert_eq!(r["tool_changes"][0]["tool"], 2);
    assert_eq!(r["accelerated"], true, "the model's acceleration fills in");
    assert!(r["duration_s"].as_f64().unwrap() > 20.0, "{r}");
    assert_eq!(r["machine_model"]["parts"], 1);
    assert_eq!(r["workholding"]["holding"], "vise");
    assert!(r["workholding"]["obstacles"].as_array().unwrap().iter().any(|o| o["name"] == "the vise's fixed jaw"));
    assert_eq!(r["stock"]["kind"], "box");
    // At the start.
    let st = run(&mut s, "cam.view.state", json!({}));
    assert_eq!((st["move"].as_u64(), st["fraction"].as_f64()), (Some(0), Some(0.0)), "{st}");
    assert_eq!(st["tool"]["number"], 2);
    // By move, line, time, percent and operation.
    let st = run(&mut s, "cam.view.seek", json!({"move": 10}));
    assert_eq!((st["move"].as_u64(), st["fraction"].as_f64()), (Some(10), Some(1.0)));
    let line = st["line"].as_u64().unwrap();
    assert!(!st["gcode"].as_str().unwrap().is_empty());
    let st = run(&mut s, "cam.view.seek", json!({"line": line}));
    assert_eq!(st["move"].as_u64(), Some(10));
    let st = run(&mut s, "cam.view.seek", json!({"percent": 50}));
    let half = st["time_s"].as_f64().unwrap();
    assert!((half - 0.5 * st["duration_s"].as_f64().unwrap()).abs() < 1e-6, "{st}");
    assert!(st["removed_volume"].as_f64().unwrap() > 0.0);
    // (Not in the first seconds: those are the tool change's wait, before the first move.)
    let st = run(&mut s, "cam.view.seek", json!({"time_s": half * 1.5}));
    assert!((st["percent"].as_f64().unwrap() - 75.0).abs() < 1e-6, "{st}");
    let st = run(&mut s, "cam.view.seek", json!({"op": "finish", "at": "end"}));
    assert_eq!(st["move"].as_u64(), Some(moves as u64 - 1));
    let st = run(&mut s, "cam.view.seek", json!({"op": 0}));
    assert_eq!((st["move"].as_u64(), st["fraction"].as_f64()), (Some(0), Some(0.0)));
    let st = run(&mut s, "cam.view.seek", json!({"by": 5}));
    assert_eq!(st["move"].as_u64(), Some(5));
    let st = run(&mut s, "cam.view.seek", json!({"by": -3}));
    assert_eq!(st["move"].as_u64(), Some(2));
    let st = run(&mut s, "cam.view.seek", json!({"tool_change": 0}));
    assert_eq!(st["move"].as_u64(), Some(0));
    // The position: a vertical tool, no rotary words.
    let st = run(&mut s, "cam.view.seek", json!({"percent": 100}));
    assert_eq!(st["position"]["axis"], json!([0.0, 0.0, 1.0]));
    assert!(st["position"]["machine"]["a_deg"].is_null());
    // A clean job: no errors found by its end.
    assert_eq!(st["findings_so_far"]["errors"], 0, "{st}");
    // Play, pause, layers.
    let st = run(&mut s, "cam.view.play", json!({"speed": 4, "loop": true}));
    assert_eq!((st["playing"].as_bool(), st["speed"].as_f64(), st["loop"].as_bool()), (Some(true), Some(4.0), Some(true)));
    let st = run(&mut s, "cam.view.pause", json!({}));
    assert_eq!(st["playing"], false);
    let l = run(&mut s, "cam.view.show", json!({"rapids": false, "part": true, "colour_by": "operation"}));
    assert_eq!(
        (l["layers"]["rapids"].as_bool(), l["layers"]["part"].as_bool(), l["colour_by"].as_str()),
        (Some(false), Some(true), Some("operation"))
    );
    // The stock as cut, as STL; a picture of the model shows the layer.
    let stl = dir.file("stock.stl");
    let e = run(&mut s, "cam.view.export_stock", json!({"path": stl}));
    assert!(e["triangles"].as_u64().unwrap() > 1000);
    assert!(std::fs::metadata(&stl).unwrap().len() > 50_000);
    let b = crate::view::bounds(&s).unwrap();
    assert!(b.max.z >= 8.0 - 1e-9, "the stock is in the view's bounds: {b:?}");
    let mut sc = solidcraft_render::Scene::default();
    crate::cam_view::scene(&s, &mut sc);
    assert!(sc.meshes.len() >= 3 && !sc.lines.is_empty(), "stock, tool and holder meshes, paths");
    // Close drops it.
    assert_eq!(run(&mut s, "cam.view.close", json!({}))["closed"], true);
    assert!(s.cam_view.is_none());
    assert_eq!(run(&mut s, "cam.view.close", json!({}))["closed"], false);
}

#[test]
fn the_simulation_view_finds_a_gouge_and_steps_to_it() {
    let mut s = dome();
    let dir = Dir::new("view-gouge");
    // A plunge 2 mm into the block's top beside the dome, along, and away.
    let gcode = dir.file("bad.tap");
    std::fs::write(&gcode, "G21\nT2 M6\nS12000 M3\nG0 X15. Y15. Z10.\nG1 Z-2. F300\nG1 X16.\nG0 Z10.\nG0 X-15.\nG1 Z-0.5 F300\nG0 Z10.\nM5\nM30\n")
        .unwrap();
    let mut job = dome_job();
    job["stock"] = json!({"box": {"min": [-20, -20, -20], "max": [20, 20, 8]}});
    let r = run(&mut s, "cam.view.load", json!({"job": job, "program": gcode, "resolution": 0.2}));
    assert_eq!(r["operations"][0]["name"], "T2");
    assert!(r["findings"].as_array().unwrap().is_empty(), "nothing known before it is cut");
    // On to the next finding: the gouge of the plunge at line 5.
    let st = run(&mut s, "cam.view.seek", json!({"by": 1, "until": "finding"}));
    assert_eq!(st["line"], 5, "{st}");
    assert_eq!(st["findings_so_far"]["errors"], 1, "{st}");
    let all = run(&mut s, "cam.view.state", json!({"findings": "all"}));
    let f = &all["findings"][0];
    assert_eq!((f["code"].as_str(), f["line"].as_u64()), (Some("gouge"), Some(5)), "{all}");
    assert!((f["depth"].as_f64().unwrap() - 2.0).abs() < 0.05, "{f}");
    assert!(f["at"][2].as_f64().unwrap() < -1.9);
    // The findings so far grow with the tool: back to the start, none.
    let st = run(&mut s, "cam.view.seek", json!({"move": 0}));
    assert_eq!(st["findings_so_far"]["errors"], 0);
    // Seek to a finding by its number; one not known yet is refused.
    let st = run(&mut s, "cam.view.seek", json!({"finding": 0}));
    assert_eq!(st["line"], 5);
    assert!(s.execute("cam.view.seek", &json!({"finding": 5})).is_err());
    // The gouge goes on along the next move.
    let st = run(&mut s, "cam.view.seek", json!({"by": 1, "until": "finding"}));
    assert_eq!(st["line"], 6, "{st}");
    let st = run(&mut s, "cam.view.seek", json!({"percent": 100}));
    assert!(st["findings_so_far"]["by_code"][0]["count"].as_u64().unwrap() >= 3, "{st}");
}

#[test]
fn the_simulation_view_plays_a_bar_on_the_fourth_axis() {
    let mut s = bar();
    let r = run(&mut s, "cam.view.load", json!({"job": barrel_job(), "run_to_end": true}));
    assert_eq!(r["stock"]["kind"], "cylinder");
    assert!(r["moves"].as_u64().unwrap() > 10);
    let st = run(&mut s, "cam.view.seek", json!({"percent": 50}));
    assert!(st["position"]["machine"]["a_deg"].is_number(), "{st}");
    assert!(st["checked_moves"].as_u64().unwrap() >= r["moves"].as_u64().unwrap(), "run to the end at load");
    assert_eq!(st["findings_so_far"]["errors"], 0, "{st}");
    let mut sc = solidcraft_render::Scene::default();
    crate::cam_view::scene(&s, &mut sc);
    assert!(!sc.meshes.is_empty());
}

#[test]
fn hostile_view_commands_are_refused_without_panics() {
    let mut s = dome();
    let dir = Dir::new("view-hostile");
    let junk = dir.file("junk.json");
    std::fs::write(
        &junk,
        "{\"format\": \"solidcraft.machine_model\", \"version\": 1, \"spindle\": {\"nose_gap\": -1}, \"table\": {\"size\": [1, 1, 1]}}",
    )
    .unwrap();
    let vise = dir.file("v.json");
    std::fs::write(&vise, VIEW_VISE).unwrap();
    let loads = [
        json!({}),
        json!({"job": dome_job(), "job_path": dir.file("j.json")}),
        json!({"job": dome_job(), "resolution": 0}),
        json!({"job": dome_job(), "resolution": f64::MAX}),
        json!({"job": dome_job(), "resolution": "fine"}),
        json!({"job": dome_job(), "program": ""}),
        json!({"job": dome_job(), "program": dir.file("missing.tap")}),
        json!({"job": dome_job(), "machine_model_path": junk}),
        json!({"job": dome_job(), "machine_model_path": dir.file("missing.json")}),
        json!({"job": dome_job(), "workholding_path": vise}),
        json!({"job": dome_job(), "colour_by": "rainbow"}),
        json!({"job": dome_job(), "surprise": 1}),
    ];
    for p in &loads {
        match s.execute("cam.view.load", p) {
            Err(EngineError::Internal(_, msg)) => panic!("{p}: panicked: {msg}"),
            Err(_) => {}
            Ok(v) => panic!("{p}: accepted: {v}"),
        }
    }
    run(&mut s, "cam.view.load", json!({"job": dome_job(), "resolution": 0.5}));
    let bad = [
        ("cam.view.seek", json!({})),
        ("cam.view.seek", json!({"move": 1, "line": 3})),
        ("cam.view.seek", json!({"move": -1})),
        ("cam.view.seek", json!({"move": 1.5})),
        ("cam.view.seek", json!({"move": 1e12})),
        ("cam.view.seek", json!({"line": 0})),
        ("cam.view.seek", json!({"percent": 101})),
        ("cam.view.seek", json!({"percent": -0.1})),
        ("cam.view.seek", json!({"time_s": -1})),
        ("cam.view.seek", json!({"op": "nothing"})),
        ("cam.view.seek", json!({"op": 7})),
        ("cam.view.seek", json!({"at": "end"})),
        ("cam.view.seek", json!({"move": 1, "at": "end"})),
        ("cam.view.seek", json!({"move": 1, "until": "op"})),
        ("cam.view.seek", json!({"by": 1e9})),
        ("cam.view.seek", json!({"by": 0.5})),
        ("cam.view.seek", json!({"tool_change": 3})),
        ("cam.view.seek", json!({"finding": 0})),
        ("cam.view.play", json!({"speed": 0})),
        ("cam.view.play", json!({"speed": 1001})),
        ("cam.view.play", json!({"speed": -1})),
        ("cam.view.play", json!({"moves_per_s": 0})),
        ("cam.view.play", json!({"speed": 2, "moves_per_s": 3})),
        ("cam.view.show", json!({"stock": "yes"})),
        ("cam.view.show", json!({"glitter": true})),
        ("cam.view.state", json!({"limit": 0})),
        ("cam.view.state", json!({"limit": 1e6})),
        ("cam.view.state", json!({"findings": "some"})),
        ("cam.view.pause", json!({"now": true})),
        ("cam.view.export_stock", json!({"path": ""})),
    ];
    for (id, p) in &bad {
        match s.execute(id, p) {
            Err(EngineError::Internal(_, msg)) => panic!("{id} {p}: panicked: {msg}"),
            Err(_) => {}
            Ok(v) => panic!("{id} {p}: accepted: {v}"),
        }
    }
    // Still where it was, still answering.
    let st = run(&mut s, "cam.view.state", json!({}));
    assert_eq!(st["move"], 0);
}

/// The owner's example jobs, when `SOLIDCRAFT_CAM_EXAMPLES` names their folder (with `jobs/`
/// and `machines/` in it): the 3-axis dome job loaded with the machine model and the vise.
#[test]
fn an_example_job_loads_with_its_machine_and_vise() {
    let Some(root) = std::env::var_os("SOLIDCRAFT_CAM_EXAMPLES").map(PathBuf::from) else {
        eprintln!("SOLIDCRAFT_CAM_EXAMPLES is not set: the example job is not loaded");
        return;
    };
    let jobs = root.join("jobs").join("examples-3axis");
    let machines = root.join("machines");
    let model: Value = serde_json::from_slice(&std::fs::read(jobs.join("dome.model.json")).unwrap()).unwrap();
    let mut s = Session::default();
    for c in model["commands"].as_array().unwrap() {
        run(&mut s, c["command"].as_str().unwrap(), c["params"].clone());
    }
    let path = |p: PathBuf| p.to_string_lossy().into_owned();
    let started = std::time::Instant::now();
    let r = run(
        &mut s,
        "cam.view.load",
        json!({"job_path": path(jobs.join("dome.cam.json")), "machine_model_path": path(machines.join("mini-mill.machine-model.json")),
               "workholding_path": path(machines.join("mini-mill-vise.workholding.json")), "run_to_end": true}),
    );
    let loaded = started.elapsed().as_secs_f64();
    let names: Vec<&str> = r["operations"].as_array().unwrap().iter().map(|o| o["name"].as_str().unwrap()).collect();
    assert_eq!(names, ["rough", "walls", "finish"], "{r}");
    assert!(r["machine_model"]["meshes"].as_u64().unwrap() > 0, "{r}");
    let st = run(&mut s, "cam.view.seek", json!({"percent": 50}));
    eprintln!(
        "example dome: {} moves, {} cells of {} mm, {:.1} s of program (accelerated {}), loaded and run to the end in {loaded:.2} s; \
         keyframes {} ({} MB); findings {}; notes {}; at 50 %: move {} line {} `{}`",
        r["moves"],
        r["cells"],
        r["resolution"],
        r["duration_s"].as_f64().unwrap(),
        r["accelerated"],
        st["keyframes"],
        st["keyframe_bytes"].as_u64().unwrap() / (1 << 20),
        r["findings"],
        r["notes"],
        st["move"],
        st["line"],
        st["gcode"].as_str().unwrap()
    );
    assert!(r["duration_s"].as_f64().unwrap() > 60.0);
}

/// Pictures of the simulation view (the headless model picture, with the CAM layer), written to
/// the folder `SOLIDCRAFT_CAM_EVIDENCE` names: the dome job half way with a machine model and a
/// vise, the barrel half way on a 4th axis, and (with `SOLIDCRAFT_CAM_EXAMPLES`) an example job.
#[test]
fn pictures_of_the_simulation_view() {
    let Some(out) = std::env::var_os("SOLIDCRAFT_CAM_EVIDENCE").map(PathBuf::from) else { return };
    std::fs::create_dir_all(&out).unwrap();
    let picture = |s: &Session, name: &str, zoom: f64| {
        let mut cam = crate::view::home_camera(s).unwrap();
        cam.distance *= zoom;
        let sc = crate::view::scene(s, &cam).unwrap();
        let png = solidcraft_render::render_png(&sc, &cam, 1280, 900).unwrap();
        std::fs::write(out.join(name), png).unwrap();
    };
    let dir = Dir::new("view-pictures");
    let (model, vise) = (dir.file("m.machine-model.json"), dir.file("v.workholding.json"));
    std::fs::write(&model, VIEW_MODEL).unwrap();
    std::fs::write(&vise, VIEW_VISE).unwrap();
    let mut s = dome();
    let mut job = dome_job();
    job["stock"] = json!({"box": {"min": [-20, -20, -20], "max": [20, 20, 8]}});
    run(&mut s, "cam.view.load", json!({"job": job, "machine_model_path": model, "workholding_path": vise, "resolution": 0.1}));
    run(&mut s, "cam.view.seek", json!({"percent": 50}));
    run(&mut s, "cam.view.show", json!({"fixtures": false}));
    picture(&s, "dome-50.png", 0.8);
    run(&mut s, "cam.view.show", json!({"fixtures": true}));
    picture(&s, "dome-50-vise.png", 1.0);
    let mut s = bar();
    run(&mut s, "cam.view.load", json!({"job": barrel_job()}));
    run(&mut s, "cam.view.seek", json!({"percent": 50}));
    picture(&s, "barrel-50.png", 1.0);
    if let Some(root) = std::env::var_os("SOLIDCRAFT_CAM_EXAMPLES").map(PathBuf::from) {
        let jobs = root.join("jobs").join("examples-3axis");
        let machines = root.join("machines");
        let model: Value = serde_json::from_slice(&std::fs::read(jobs.join("dome.model.json")).unwrap()).unwrap();
        let mut s = Session::default();
        for c in model["commands"].as_array().unwrap() {
            run(&mut s, c["command"].as_str().unwrap(), c["params"].clone());
        }
        let path = |p: PathBuf| p.to_string_lossy().into_owned();
        run(
            &mut s,
            "cam.view.load",
            json!({"job_path": path(jobs.join("dome.cam.json")), "machine_model_path": path(machines.join("mini-mill.machine-model.json")),
                   "workholding_path": path(machines.join("mini-mill-vise.workholding.json")), "resolution": 0.1}),
        );
        for (pc, name) in [(25.0, "example-dome-25.png"), (60.0, "example-dome-60.png")] {
            run(&mut s, "cam.view.seek", json!({"percent": pc}));
            run(&mut s, "cam.view.show", json!({"fixtures": false, "machine": false}));
            picture(&s, name, 0.9);
        }
        run(&mut s, "cam.view.show", json!({"fixtures": true, "machine": true}));
        picture(&s, "example-dome-60-machine.png", 2.2);
    }
}

#[test]
fn the_pictures_machine_part_stands_where_the_collision_check_puts_it() {
    let mut s = dome();
    let dir = Dir::new("view-plate");
    let model = dir.file("m.machine-model.json");
    std::fs::write(&model, VIEW_MODEL).unwrap();
    let mut job = dome_job();
    job["stock"] = json!({"box": {"min": [-20, -20, -20], "max": [20, 20, 8]}});
    run(&mut s, "cam.view.load", json!({"job": job, "machine_model_path": model, "resolution": 0.5}));
    run(&mut s, "cam.view.seek", json!({"percent": 50}));
    let mut sc = solidcraft_render::Scene::default();
    crate::cam_view::scene(&s, &mut sc);
    let bounds = |m: &solidcraft_render::SceneMesh| {
        m.mesh.positions.iter().fold(([f64::MAX; 3], [f64::MIN; 3]), |(lo, hi), p| {
            ([lo[0].min(p.x), lo[1].min(p.y), lo[2].min(p.z)], [hi[0].max(p.x), hi[1].max(p.y), hi[2].max(p.z)])
        })
    };
    // The tool's axis: the middle of its cutting end.
    let (tl, th) = bounds(sc.meshes.iter().find(|m| m.color == crate::cam_view::colors::TOOL).expect("the tool is drawn"));
    let tip = [0.5 * (tl[0] + th[0]), 0.5 * (tl[1] + th[1])];
    // The Z plate (spindle frame X -40..40, Y 30..45).
    let (lo, hi) = bounds(sc.meshes.iter().find(|m| m.color == crate::cam_view::colors::MACHINE).expect("the plate is drawn"));
    let near = |a: f64, b: f64| (a - b).abs() < 1e-6;
    assert!(near(lo[0] - tip[0], -40.0) && near(hi[0] - tip[0], 40.0), "X {lo:?} {hi:?} about the tip {tip:?}");
    assert!(near(lo[1] - tip[1], 30.0) && near(hi[1] - tip[1], 45.0), "Y {lo:?} {hi:?} about the tip {tip:?}");
}

/// The simulation view plays the program `cam.job` writes: the job read through its profile as
/// `cam.job` reads it, A unwound by rewinds within ±360° with every block of a rewind under its
/// operation, a rewind's way back down fed at the operation's plunge feed (not the post's rewind
/// feed) and never taken for a rapid into the stock, and an operation `cam.job` refuses refused
/// at load the same way.
#[test]
fn the_simulation_view_plays_the_program_cam_job_writes() {
    let mut s = bar();
    let dir = Dir::new("view-merged");
    let mut post = run(&mut s, "cam.posts", json!({"name": "linuxcnc_4axis"}))["post"].clone();
    post["rewind"] = json!({"distance": 25.0, "retract_feed": 500.0, "plunge_feed": 400.0});
    std::fs::write(dir.0.join("rewind.post.json"), post.to_string()).unwrap();
    let profile = json!({"machine": "4axis_a", "a_limits_deg": [-360, 360], "post_path": "rewind.post.json", "tools": [ball(3, 4.0)]});
    std::fs::write(dir.0.join("bar.profile.json"), profile.to_string()).unwrap();
    let helix = |name: &str, tool: u32| {
        json!({"name": name, "type": "rotary_finish", "tool": tool, "pattern": {"type": "helix", "pitch": 2},
               "clearance": 25, "retract": 15, "range": [-15, 15]})
    };
    let job = json!({"profile_path": "bar.profile.json", "operations": [helix("first helix", 3), helix("second helix", 3)]});
    let job_path = dir.file("helix.cam.json");
    std::fs::write(&job_path, job.to_string()).unwrap();
    let out = dir.file("helix.tap");
    let r = solidcraft_io::vfs::scoped(Some(solidcraft_io::vfs::PathScope::new([dir.0.clone()])), || {
        run(&mut s, "cam.job", json!({"job_path": job_path, "output": out}))
    });
    assert_eq!(r["passed"], true, "{r}");
    let rewinds: u64 = r["operations"].as_array().unwrap().iter().map(|o| o["rewinds"].as_u64().unwrap_or(0)).sum();
    assert!(rewinds > 0, "the helices rewind A: {r}");
    let text = std::fs::read_to_string(&out).unwrap();

    let v = run(&mut s, "cam.view.load", json!({"job_path": job_path, "run_to_end": true, "resolution": 0.25}));
    assert_eq!(v["lines"], r["program_lines"], "the view plays the program as written, rewinds and all");
    let names: Vec<&str> = v["operations"].as_array().unwrap().iter().filter_map(|o| o["name"].as_str()).collect();
    assert_eq!(names, ["first helix", "second helix"], "every block is under its operation: {v}");
    let st = run(&mut s, "cam.view.seek", json!({"percent": 100}));
    assert_eq!(st["findings_so_far"]["errors"], 0, "no rapid into the stock where a rewind comes back: {st}");
    // Every feed at the tool's plunge feed is a plunge in the view, the rewinds' way back down
    // among them; nothing is fed at the post's rewind plunge feed.
    assert!(!text.lines().any(|l| l.contains("F400")), "the rewind comes down at the operation's plunge feed");
    let plunges: Vec<usize> = text.lines().enumerate().filter(|(_, l)| l.contains("F250")).map(|(i, _)| i + 1).collect();
    assert!(plunges.len() >= rewinds as usize, "{plunges:?}");
    for line in plunges {
        let st = run(&mut s, "cam.view.seek", json!({"line": line}));
        assert_eq!((st["kind"].as_str(), st["feed"].as_f64()), (Some("plunge"), Some(250.0)), "line {line}: {st}");
    }

    // An operation `cam.job` refuses is refused at load with the same refusal.
    let bad = json!({"profile_path": "bar.profile.json", "operations": [helix("first helix", 3), helix("no tool", 99)]});
    std::fs::write(&job_path, bad.to_string()).unwrap();
    let e_job = s.execute("cam.job", &json!({"job_path": job_path, "dry_run": true})).unwrap_err();
    let e_view = s.execute("cam.view.load", &json!({"job_path": job_path})).unwrap_err();
    let (rj, rv) = (refusals(&e_job), refusals(&e_view));
    assert_eq!(rj.len(), 1, "{e_job}");
    assert_eq!((rv.len(), &rv[0]["op"], &rv[0]["check"]), (1, &rj[0]["op"], &rj[0]["check"]), "{e_view}");
}

/// A write after a dry run reuses the dry run's plan, and writes what planning afresh would;
/// a changed tool library or a changed design plans again.
#[test]
fn a_write_after_a_dry_run_reuses_its_plan_until_an_input_changes() {
    let mut s = dome();
    let dir = Dir::new("plan-cache");
    std::fs::write(dir.0.join("tools.json"), json!([ball(2, 6.0)]).to_string()).unwrap();
    let mut job = dome_job();
    job.as_object_mut().unwrap().remove("tools");
    job["tool_library_path"] = json!("tools.json");
    let job_path = dir.file("dome.cam.json");
    std::fs::write(&job_path, job.to_string()).unwrap();
    let hits = |s: &Session| s.cam_plan.lock().unwrap().as_ref().map(|c| c.hits);
    let dry = run(&mut s, "cam.job", json!({"job_path": job_path, "dry_run": true}));
    assert_clean(&dry);
    assert_eq!(hits(&s), Some(0));
    let out = dir.file("dome.tap");
    let wrote = run(&mut s, "cam.job", json!({"job_path": job_path, "output": out}));
    assert_eq!(hits(&s), Some(1), "the write reused the dry run's plan");
    assert_eq!(dry["program_head"], wrote["program_head"]);
    assert_eq!(dry["operations"][0]["cut_length"], wrote["operations"][0]["cut_length"]);
    let reused = std::fs::read_to_string(&out).unwrap();
    // Planned afresh in a new session: the same program.
    let mut fresh = dome();
    let again = dir.file("fresh.tap");
    run(&mut fresh, "cam.job", json!({"job_path": job_path, "output": again}));
    assert_eq!(reused, std::fs::read_to_string(&again).unwrap());
    // A tool library changed since: planned again, at its new feed.
    let mut faster = ball(2, 6.0);
    faster["feed"] = json!(1234.5);
    std::fs::write(dir.0.join("tools.json"), json!([faster]).to_string()).unwrap();
    let r = run(&mut s, "cam.job", json!({"job_path": job_path, "dry_run": true}));
    assert_eq!(hits(&s), Some(0), "planned again");
    assert_ne!(r["cut_minutes"], dry["cut_minutes"], "at the new feed");
    // A changed design: planned again.
    run(&mut s, "solid.box", json!({"length": 4, "width": 4, "height": 4, "corner": [16, 16, -4], "operation": "cut"}));
    run(&mut s, "cam.job", json!({"job_path": job_path, "dry_run": true}));
    assert_eq!(hits(&s), Some(0), "planned again");
}

/// A tool's speed and feed from its surface speed and chip load, explicit values first; a
/// strategy's cutting data over the tool's own; and the machine's limits on the setup sheet,
/// first, with the fixtures.
#[test]
fn speeds_from_cutting_data_are_held_to_the_machine_and_stated_first_on_the_sheet() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 10, "corner": [-20, -15, -10], "body_name": "Plate"}));
    run(&mut s, "solid.box", json!({"length": 20, "width": 16, "height": 3, "corner": [-10, -8, -3], "operation": "cut"}));
    run(&mut s, "solid.box", json!({"length": 60, "width": 10, "height": 10, "corner": [-30, -25, -10], "body_name": "Jaw"}));
    // 150 m/min on a 6 mm cutter is 7958 rev/min; the machine's lowest is 10000.
    let mill = json!({"number": 1, "kind": "flat_endmill", "diameter": 6, "flute_length": 15, "flutes": 3,
        "surface_speed": 150, "chip_load": 0.02, "plunge_feed": 200,
        "cutting_data": [{"strategy": "slot", "stepdown": 1.5, "stepover": 2.5}]});
    let job = json!({
        "machine": "3axis", "post": "mach3_4axis", "tools": [mill], "bodies": ["Plate"], "fixtures": ["Jaw"],
        "machine_rates": mill_rates(),
        "operations": [{"name": "pocket", "type": "pocket", "tool": 1, "polygon": [[-10, -8], [10, -8], [10, 8], [-10, 8]],
            "heights": {"clearance": 20, "retract": 5, "top": 0, "bottom": -3}}]
    });
    let dir = Dir::new("speeds");
    let sheet = dir.file("sheet.md");
    let r = run(&mut s, "cam.job", json!({"job": job, "dry_run": false, "output": dir.file("p.tap"), "setup_sheet": sheet}));
    let notes = r["notes"].to_string();
    assert!(notes.contains("worked out from its surface speed of 150 m/min"), "{notes}");
    assert!(notes.contains("held to the spindle's range 10000 to 24000: 10000 rev/min"), "{notes}");
    // The feed follows the speed as held: 10000 × 3 × 0.02.
    assert!(notes.contains("feed 600 mm/min is worked out from its chip load"), "{notes}");
    assert!(notes.contains("slot cutting data is used"), "{notes}");
    let text = std::fs::read_to_string(dir.file("p.tap")).unwrap();
    assert!(text.contains("S10000") && text.contains("F600"), "{text}");
    // The slot data's 1.5 mm stepdown: two levels to 3 mm.
    assert!(text.contains("Z-1.5") && text.contains("Z-3"), "{text}");
    let md = std::fs::read_to_string(&sheet).unwrap();
    let first = md.find("## Before you start").expect("the limits come first");
    assert!(first < md.find("## Work zero").unwrap(), "{md}");
    assert!(md[first..].contains("10000 rev/min"), "{md}");
    assert!(md.contains("## Fixtures") && md.contains("- Jaw: X -30 to 30, Y -25 to -15, Z -10 to 0 (work coordinates)"), "{md}");
    // An explicit speed wins; a hostile surface speed is refused.
    let mut given = job.clone();
    given["tools"][0]["spindle_rpm"] = json!(18000);
    let r = run(&mut s, "cam.job", json!({"job": given, "dry_run": true}));
    assert!(!r["notes"].to_string().contains("surface speed"), "{r}");
    assert!(r["program_head"].to_string().contains("S18000") || r["summary"].to_string().contains("18000"), "{r}");
    let mut bad = job.clone();
    bad["tools"][0]["surface_speed"] = json!(-1);
    assert!(s.execute("cam.job", &json!({"job": bad, "dry_run": true})).is_err());
    // A feed from the chip load past the machine's top feed (10000 × 3 × 0.4 = 12000) is held
    // to it, and so is the plunge that follows it without a `plunge_feed` of its own: never
    // faster than the cut, and down no faster than Z's top feed.
    let mut fast = job.clone();
    fast["tools"][0]["chip_load"] = json!(0.4);
    fast["tools"][0].as_object_mut().unwrap().remove("plunge_feed");
    let out = dir.file("fast.tap");
    run(&mut s, "cam.job", json!({"job": fast, "output": out}));
    let text = std::fs::read_to_string(&out).unwrap();
    let feeds: Vec<f64> =
        text.lines().flat_map(|l| l.split_whitespace()).filter_map(|w| w.strip_prefix('F')).filter_map(|f| f.parse().ok()).collect();
    assert!(feeds.contains(&8000.0) && feeds.iter().all(|f| *f <= 8000.0), "{feeds:?}");
    assert!(feeds.contains(&4000.0), "the plunge is held to Z's top feed: {feeds:?}");
}
