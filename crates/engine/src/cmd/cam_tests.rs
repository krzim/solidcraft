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

/// A PNG `path` decoded: its size and its RGB pixels, rows top to bottom.
fn png_pixels(path: &str) -> (u32, u32, Vec<[u8; 3]>) {
    let img = image::open(path).unwrap_or_else(|e| panic!("{path}: {e}")).to_rgb8();
    let (w, h) = img.dimensions();
    (w, h, img.pixels().map(|p| p.0).collect())
}

fn near(a: [u8; 3], b: [u8; 3], within: i32) -> bool {
    a.iter().zip(b).all(|(x, y)| (i32::from(*x) - i32::from(y)).abs() <= within)
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
            px.iter().enumerate().filter(|(_, c)| near(**c, [240, 120, 0], 30)).map(|(i, _)| (i as u32 % w, i as u32 / w)).collect();
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
        .filter(|x| (row.saturating_sub(1)..=row + 1).any(|y| px.get((y * w + x) as usize).is_some_and(|c| near(*c, [35, 35, 45], 30))))
        .count();
    assert!(dark > 20, "{dark} pixels of the A axis on row {row}");
    assert!(px.iter().filter(|c| near(**c, [240, 120, 0], 30)).count() > 100, "the barrel finish is drawn");
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
        .filter(|&i| near(px[i], [255, 0, 0], 25))
        .filter(|&i| (f64::from(i as u32 % w) - cx).abs() <= 8.0 && (f64::from(i as u32 / w) - cy).abs() <= 8.0)
        .count();
    assert!(red >= 6, "{red} red pixels at the first gouge ({cx}, {cy})");
}
