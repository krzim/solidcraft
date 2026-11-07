//! Tests of picks: an operation's geometry by persistent face names, found again after model
//! edits that renumber the features, lost when the face goes; heights that follow a face; work
//! zero set by a pick, the stock or a cylinder's axis and kept with how it was set; what
//! `cam.op.defaults` says each value comes from; `cam.picks.resolve`'s suitable operations;
//! `cam.op.suggest`; hostile parameters refused.

use serde_json::{Value, json};

use crate::{EngineError, Session};

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    s.execute(id, &p).unwrap_or_else(|e| panic!("{id} {p}: {e}"))
}

fn refused(s: &mut Session, id: &str, p: Value) -> String {
    match s.execute(id, &p) {
        Err(e @ EngineError::BadParams { .. }) => e.to_string(),
        other => panic!("{id} {p}: expected a refusal, got {other:?}"),
    }
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap_or_else(|| panic!("not a number: {v}"))
}

/// A 40 × 30 × 20 block, its top at Z 20, with a 16 × 10 pocket 5 deep at X 0 and a 5 mm hole
/// through it at (12, 0).
fn block() -> Session {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 20, "corner": [-20, -15, 0], "body_name": "Block"}));
    run(&mut s, "solid.box", json!({"length": 16, "width": 10, "height": 5, "corner": [-8, -5, 15], "operation": "cut"}));
    run(&mut s, "solid.cylinder", json!({"radius": 2.5, "height": 20, "base": [14, 0, 0], "axis": "Z", "operation": "cut"}));
    s
}

const TOOLS: &str = r#"[
    {"number": 4, "kind": "flat_endmill", "diameter": 4, "flute_length": 12, "spindle_rpm": 12000, "feed": 800, "plunge_feed": 200},
    {"number": 150, "kind": "drill", "diameter": 5, "flute_length": 30, "spindle_rpm": 3000, "feed": 200, "plunge_feed": 100}
]"#;

/// A new open job for the block with the tools above.
fn open_job(s: &mut Session) {
    run(s, "cam.job.new", json!({"name": "picks"}));
    let tools: Value = serde_json::from_str(TOOLS).unwrap();
    run(s, "cam.job.edit", json!({"set": {"machine": "3axis", "post": "mach3_4axis", "tools": tools}}));
}

/// The picks `cam.features` gives feature `id`.
fn feature_picks(s: &mut Session, id: &str) -> Value {
    let r = run(s, "cam.features", json!({"picks": true}));
    r["picks"][id].clone()
}

fn dry(s: &mut Session) -> Value {
    run(s, "cam.job", json!({"dry_run": true}))
}

#[test]
fn a_pocket_picked_by_its_floor_plans_as_its_feature_and_follows_renumbering() {
    let mut s = block();
    open_job(&mut s);
    let floor = feature_picks(&mut s, "pocket:0");
    assert_eq!(floor.as_array().map(Vec::len), Some(1), "the floor alone: {floor}");
    assert!(floor[0]["face"].as_str().is_some_and(|n| !n.is_empty()), "{floor}");
    run(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "tool": 4, "name": "by pick"}, "picks": floor}));
    run(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "tool": 4, "name": "by id", "feature": "pocket:0"}}));
    let r = dry(&mut s);
    let (a, b) = (&r["operations"][0], &r["operations"][1]);
    assert_eq!(a["moves"], b["moves"], "the same path: {a} {b}");
    assert!((f(&a["cut_length"]) - f(&b["cut_length"])).abs() < 1e-9);
    let resolved = &r["picks_resolved"][0];
    assert_eq!(resolved["by"], "name", "{r}");
    assert_eq!(resolved["feature"], "pocket:0");
    // Nothing to check: a pick found by its exact name gives no finding.
    let g = run(&mut s, "cam.generate", json!({}));
    assert!(g["findings"].as_array().unwrap().iter().all(|x| x["check"] != "picks"), "{g}");

    // A second pocket further left: it is pocket:0 now, ours pocket:1. The pick still finds ours.
    run(&mut s, "solid.box", json!({"length": 4, "width": 4, "height": 3, "corner": [-18, -2, 17], "operation": "cut"}));
    let ids = run(&mut s, "cam.features", json!({}));
    let ours = ids["pockets"].as_array().unwrap().iter().find(|p| (f(&p["bounds"][0][0]) + 8.0).abs() < 1e-6).unwrap()["id"].clone();
    assert_eq!(ours, "pocket:1", "renumbered: {ids}");
    run(&mut s, "cam.op.remove", json!({"op": "by id"}));
    let r = dry(&mut s);
    assert_eq!(r["picks_resolved"][0]["feature"], "pocket:1", "{}", r["picks_resolved"]);
    let ext = &r["operations"][0]["extents"]["cut"];
    assert!(f(&ext[0][0]) > -8.0 && f(&ext[1][0]) < 8.0, "still the 16 x 10 pocket: {ext}");
}

#[test]
fn a_pick_whose_face_is_gone_is_lost_and_refuses_with_a_finding() {
    let mut s = block();
    open_job(&mut s);
    let floor = feature_picks(&mut s, "pocket:0");
    run(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "tool": 4}, "picks": floor}));
    // The hole and the pocket undone: the floor is gone, and where it was is solid.
    run(&mut s, "edit.undo", json!({}));
    run(&mut s, "edit.undo", json!({}));
    let g = run(&mut s, "cam.generate", json!({}));
    let findings = g["findings"].as_array().unwrap();
    let lost = findings.iter().find(|x| x["check"] == "picks").unwrap_or_else(|| panic!("{g}"));
    assert_eq!(lost["severity"], "error");
    assert_eq!(lost["op"]["index"], 0);
    assert!(lost["message"].as_str().unwrap().contains("was not found"), "{lost}");
    assert_eq!(g["passed"], false);
}

#[test]
fn a_pick_found_by_its_point_is_a_warning() {
    let mut s = block();
    open_job(&mut s);
    let mut floor = feature_picks(&mut s, "pocket:0");
    floor[0]["face"] = json!("F99:nothing:here");
    run(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "tool": 4}, "picks": floor}));
    let g = run(&mut s, "cam.generate", json!({}));
    let w = g["findings"].as_array().unwrap().iter().find(|x| x["check"] == "picks").unwrap_or_else(|| panic!("{g}")).clone();
    assert_eq!(w["severity"], "warning");
    assert!(w["message"].as_str().unwrap().contains("position"), "{w}");
    assert_eq!(dry(&mut s)["picks_resolved"][0]["by"], "point");
}

#[test]
fn holes_picked_are_drilled_at_their_centres_and_depth() {
    let mut s = block();
    open_job(&mut s);
    let wall = feature_picks(&mut s, "hole:0");
    run(&mut s, "cam.op.add", json!({"op": {"type": "drill", "tool": 150}, "picks": wall}));
    // No starting heights were written: the hole gives them.
    let op = &s.cam_job.as_ref().unwrap().value["operations"][0];
    assert!(op.get("heights").is_none(), "{op}");
    let r = dry(&mut s);
    let ext = &r["operations"][0]["extents"]["cut"];
    assert!((f(&ext[0][0]) - 14.0).abs() < 1e-6 && (f(&ext[1][0]) - 14.0).abs() < 1e-6, "{ext}");
    assert!((f(&ext[0][2]) - 0.0).abs() < 1e-6, "down to the hole's bottom: {ext}");
    assert_eq!(r["passed"], true, "{}", r["failures"]);
}

#[test]
fn a_height_from_a_face_follows_the_model() {
    let mut s = block();
    open_job(&mut s);
    let ff = run(&mut s, "cam.features", json!({"picks": true}));
    let top_id =
        ff["faces"].as_array().unwrap().iter().find(|x| x["role"] == "top").unwrap_or_else(|| panic!("{ff}"))["id"].as_str().unwrap().to_string();
    let top = ff["picks"][&top_id][0].clone();
    let square = json!([[-20, -15], [20, -15], [20, 15], [-20, 15]]);
    run(
        &mut s,
        "cam.op.add",
        json!({"op": {"type": "contour", "tool": 4, "polygon": square, "heights": {"top": {"from": top}, "bottom": {"from": top, "offset": -6}}}}),
    );
    let r = dry(&mut s);
    assert!((f(&r["operations"][0]["extents"]["cut"][0][2]) - 14.0).abs() < 1e-6, "{}", r["operations"][0]["extents"]["cut"]);
    // The block 2 mm taller: the contour's bottom follows its top face.
    run(&mut s, "parameters.change", json!({"name": "d3", "expression": "22 mm"}));
    let r = dry(&mut s);
    assert!((f(&r["operations"][0]["extents"]["cut"][0][2]) - 16.0).abs() < 1e-6, "{}", r["operations"][0]["extents"]["cut"]);
    // The job keeps the reference, not the number.
    assert!(s.cam_job.as_ref().unwrap().value["operations"][0]["heights"]["bottom"].is_object());
    // A height from a face that is not square to the tool is refused when planned.
    let wall = json!({"body": "Block", "at": [20, 0, 10]});
    run(&mut s, "cam.op.edit", json!({"op": 0, "set": {"heights.bottom": {"from": wall}}}));
    let e = s.execute("cam.job", &json!({"dry_run": true})).unwrap_err().to_string();
    assert!(e.contains("square to the tool axis"), "{e}");
    let e = refused(&mut s, "cam.op.edit", json!({"op": 0, "set": {"heights.bottom": {"from": wall, "offset": "x"}}}));
    assert!(e.contains("offset") || e.contains("bottom"), "{e}");
}

#[test]
fn work_zero_by_stock_and_pick_is_kept_and_reported_when_it_moves() {
    let mut s = block();
    open_job(&mut s);
    run(&mut s, "cam.job.edit", json!({"set": {"stock": {"around_part": {"margin": [2, 2, 1], "bottom": 0}}}}));
    let r = run(&mut s, "cam.setup.zero", json!({"stock": {"x": "low", "y": "low", "z": "top"}}));
    assert_eq!(r["origin"], json!([-22.0, -17.0, 21.0]), "{r}");
    let job = s.cam_job.as_ref().unwrap().value.clone();
    assert_eq!(job["origin_from"], json!({"stock": {"x": "low", "y": "low", "z": "top"}}));
    // The hole's round edge: its centre.
    let st = run(&mut s, "cam.features", json!({"picks": true}));
    let hole = st["picks"]["hole:0"][0].clone();
    let r = run(&mut s, "cam.setup.zero", json!({"pick": hole}));
    assert!((f(&r["origin"][0]) - 14.0).abs() < 1e-6 && f(&r["origin"][1]).abs() < 1e-6, "on the hole's axis: {r}");
    // Back to the stock's corner; then a taller margin: Generate warns, nothing moves by itself.
    run(&mut s, "cam.setup.zero", json!({"stock": {"x": "low", "y": "low", "z": "top"}}));
    run(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "tool": 4, "feature": "pocket:0"}}));
    run(&mut s, "cam.job.edit", json!({"set": {"stock.around_part.top": 3}}));
    let g = run(&mut s, "cam.generate", json!({}));
    let w = g["findings"].as_array().unwrap().iter().find(|x| x["check"] == "work_zero").unwrap_or_else(|| panic!("{g}")).clone();
    assert_eq!(w["severity"], "warning");
    assert_eq!(s.cam_job.as_ref().unwrap().value["origin"], json!([-22.0, -17.0, 21.0]), "not moved");
    let r = run(&mut s, "cam.setup.zero", json!({"refresh": true}));
    assert_eq!(r["origin"], json!([-22.0, -17.0, 23.0]), "{r}");
    let g = run(&mut s, "cam.generate", json!({}));
    assert!(g["findings"].as_array().unwrap().iter().all(|x| x["check"] != "work_zero"), "{g}");
    // One undo step each.
    run(&mut s, "cam.job.undo", json!({}));
    assert_eq!(s.cam_job.as_ref().unwrap().value["origin"], json!([-22.0, -17.0, 21.0]));
}

#[test]
fn a_box_stock_stays_put_when_work_zero_moves() {
    let mut s = block();
    open_job(&mut s);
    run(&mut s, "cam.job.edit", json!({"set": {"stock": {"box": {"min": [-21, -16, -1], "max": [21, 16, 21]}}}}));
    let r = run(&mut s, "cam.setup.zero", json!({"stock": {"x": "middle", "y": "middle", "z": "top"}}));
    assert_eq!(r["origin"], json!([0.0, 0.0, 21.0]));
    let st = &s.cam_job.as_ref().unwrap().value["stock"]["box"];
    assert_eq!(st["min"], json!([-21.0, -16.0, -22.0]), "{st}");
    assert_eq!(st["max"], json!([21.0, 16.0, 0.0]), "{st}");
}

#[test]
fn a_bar_s_axis_is_work_zero_on_a_fourth_axis() {
    let mut s = Session::default();
    run(&mut s, "solid.cylinder", json!({"radius": 16, "height": 90, "base": [-30, 0, 0], "axis": "X", "body_name": "Bar"}));
    run(&mut s, "solid.box", json!({"length": 20, "width": 10, "height": 4, "center": [-12, 0, 15], "operation": "cut"}));
    run(&mut s, "cam.job.new", json!({}));
    let tools: Value = serde_json::from_str(TOOLS).unwrap();
    run(
        &mut s,
        "cam.job.edit",
        json!({"set": {"machine": "4axis_a", "post": "mach3_4axis", "tools": tools, "stock": {"cylinder": {"radius": 16, "x_min": -30, "x_max": 60}}}}),
    );
    let f_ = run(&mut s, "cam.features", json!({"axis": "a", "picks": true}));
    let turned = f_["picks"]["turned:0"][0].clone();
    assert!(turned.is_object(), "{f_}");
    let r = run(&mut s, "cam.setup.zero", json!({"axis": turned, "x_end": "stock_low"}));
    assert_eq!(r["origin"], json!([-30.0, 0.0, 0.0]), "{r}");
    let r = run(&mut s, "cam.setup.zero", json!({"axis": turned, "x_end": "stock_high"}));
    assert_eq!(r["origin"], json!([60.0, 0.0, 0.0]), "{r}");
    let notes = r["notes"].to_string();
    assert!(r["summary"]["origin"][0] == 60.0, "{notes}");
}

#[test]
fn a_pick_on_another_side_turns_a_there() {
    let mut s = Session::default();
    run(&mut s, "solid.cylinder", json!({"radius": 16, "height": 90, "base": [-30, 0, 0], "axis": "X", "body_name": "Bar"}));
    // A pocket on the +Y side (A turns it up).
    run(&mut s, "solid.box", json!({"length": 20, "width": 4, "height": 10, "center": [-12, 15, 0], "operation": "cut"}));
    run(&mut s, "cam.job.new", json!({}));
    let tools: Value = serde_json::from_str(TOOLS).unwrap();
    run(
        &mut s,
        "cam.job.edit",
        json!({"set": {"machine": "4axis_a", "post": "mach3_4axis", "tools": tools, "stock": {"cylinder": {"radius": 16, "x_min": -30, "x_max": 60}}}}),
    );
    let ff = run(&mut s, "cam.features", json!({"axis": "a", "picks": true}));
    let pocket = ff["pockets"][0].clone();
    let a = f(&pocket["a_deg"]);
    assert!(a.abs() > 1.0, "on a side: {pocket}");
    let floor = ff["picks"][pocket["id"].as_str().unwrap()].clone();
    // A pocket on a feature turns itself; a drill or a light-load clearing is indexed when added.
    run(&mut s, "cam.op.add", json!({"op": {"type": "adaptive", "tool": 4, "name": "side"}, "picks": floor}));
    let op = s.cam_job.as_ref().unwrap().value["operations"][0].clone();
    assert_eq!(op["type"], "indexed", "{op}");
    assert!((f(&op["a_deg"]) - a).abs() < 1e-9, "{op}");
    let r = dry(&mut s);
    assert_eq!(r["passed"], true, "{}", r["failures"]);
    assert!((f(&r["operations"][0]["a_deg"]) - a).abs() < 1e-9);
    // The same pocket in a pattern of four is cut as its outline in its own frame.
    run(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "tool": 4, "name": "pattern"}, "picks": floor, "a_deg": 90}));
    run(&mut s, "cam.op.edit", json!({"op": "pattern", "a_deg": [0, 90, 180, 270]}));
    let r = dry(&mut s);
    assert_eq!(r["operations"].as_array().unwrap().len(), 5, "1 + 4: {}", r["operations"]);
    // A pattern that starts at the pocket's own angle too: planned at every angle (this bar has
    // the one pocket, so the other three cut into it, which the gouge check says).
    run(&mut s, "cam.op.edit", json!({"op": "pattern", "a_deg": [a, a + 90.0, a + 180.0, a + 270.0]}));
    let r = dry(&mut s);
    assert_eq!(r["operations"].as_array().unwrap().len(), 5, "{}", r["failures"]);
    assert_eq!(r["operations"][1]["gouge"], 0.0, "at its own angle it cuts the pocket: {}", r["operations"][1]);
}

#[test]
fn defaults_say_where_each_value_comes_from() {
    let mut s = block();
    open_job(&mut s);
    let floor = feature_picks(&mut s, "pocket:0");
    run(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "tool": 4, "stepover": 1.5}, "picks": floor}));
    let r = run(&mut s, "cam.op.defaults", json!({"op": 0}));
    let field = |p: &str| r["fields"].as_array().unwrap().iter().find(|x| x["path"] == p).unwrap_or_else(|| panic!("no {p}: {r}")).clone();
    assert_eq!(field("stepover")["source"], "given");
    assert_eq!(field("stepover")["value"], 1.5);
    let sd = field("stepdown");
    assert_eq!(sd["source"], "rule", "{sd}");
    assert_eq!(sd["value"], 2.0, "half the 4 mm tool: {sd}");
    let bottom = field("heights.bottom");
    assert_eq!(bottom["value"], 15.0, "the pocket's floor: {bottom}");
    assert!(matches!(bottom["source"].as_str(), Some("model") | Some("pick") | Some("feature")), "{bottom}");
    assert_eq!(field("spindle_rpm")["source"], "tool");
    assert_eq!(field("spindle_rpm")["value"], 12000.0);
    assert_eq!(field("feeds.cut")["value"], 800.0);
    assert_eq!(field("climb")["source"], "rule");
    assert!(field("climb")["why"].as_str().unwrap().contains("true"), "{}", field("climb"));
    assert_eq!(field("climb")["value"], true, "a rule that says its value outright gives it");
    let clearance = field("heights.clearance");
    assert_eq!(clearance["source"], "model", "{clearance}");
    assert_eq!(clearance["value"], 30.0, "10 above the block's top: {clearance}");
    // The tools: the end mill fits, the drill does not and says why.
    let drill = r["tools"].as_array().unwrap().iter().find(|t| t["number"] == 150).unwrap().clone();
    assert_eq!(drill["fits"], false);
    assert!(drill["why_not"].as_str().unwrap().contains("drill"), "{drill}");
    assert_eq!(r["tool"], 4);
    // A draft: not in the job, nothing changes.
    let before = s.cam_job.as_ref().unwrap().revision;
    let hole = feature_picks(&mut s, "hole:0");
    let d = run(&mut s, "cam.op.defaults", json!({"draft": {"type": "drill", "tool": 150, "picks": hole}}));
    assert_eq!(d["type"], "drill");
    assert_eq!(s.cam_job.as_ref().unwrap().revision, before);
}

#[test]
fn a_through_hole_with_a_drill_of_its_size_suits_drilling_first() {
    let mut s = block();
    open_job(&mut s);
    let wall = feature_picks(&mut s, "hole:0");
    let r = run(&mut s, "cam.picks.resolve", json!({"picks": wall}));
    assert_eq!(r["picks"][0]["feature"], "hole:0", "{r}");
    assert!(r["picks"][0]["reads"].as_str().unwrap().contains("5 mm"), "{r}");
    assert_eq!(r["suits"][0]["type"], "drill", "{r}");
    assert_eq!(r["suits"][0]["tool"], 150);
    // The pocket's floor: pocketing first, with the end mill.
    let floor = feature_picks(&mut s, "pocket:0");
    let r = run(&mut s, "cam.picks.resolve", json!({"picks": floor}));
    assert_eq!(r["suits"][0]["type"], "pocket", "{r}");
    assert_eq!(r["suits"][0]["tool"], 4);
    // From the design's selection.
    s.selection = vec![crate::Sel::Face { body: "Block".into(), index: 0, point: solidcore_record::Vec3::new(0.0, 0.0, 0.0) }];
    let r = s.execute("cam.picks.resolve", &json!({"from_selection": true}));
    assert!(r.is_ok(), "{r:?}");
}

#[test]
fn suggested_operations_are_added_as_picks_and_pass() {
    let mut s = block();
    open_job(&mut s);
    run(&mut s, "cam.job.edit", json!({"set": {"origin": [0, 0, 20]}}));
    let r = run(&mut s, "cam.op.suggest", json!({"stock_top": 0}));
    assert!(!r["added"].as_array().unwrap().is_empty(), "{r}");
    let ops = s.cam_job.as_ref().unwrap().value["operations"].as_array().unwrap().clone();
    assert!(ops.iter().any(|o| o.get("picks").is_some()), "{ops:?}");
    assert!(ops.iter().all(|o| o.get("feature").is_none()), "{ops:?}");
    assert_eq!(s.cam_job.as_ref().unwrap().undo.len(), 3, "one step for all of them (after the two edits)");
    let d = dry(&mut s);
    assert_eq!(d["passed"], true, "{}", d["failures"]);
}

#[test]
fn hostile_picks_are_refused() {
    let mut s = block();
    open_job(&mut s);
    let e = refused(&mut s, "cam.op.add", json!({"op": {"type": "pocket"}, "picks": [{"body": "Block", "face": "x", "at": [f64::MAX, 0, 0]}]}));
    assert!(e.contains("finite"), "{e}");
    let many: Vec<Value> = (0..1001).map(|i| json!({"body": "Block", "at": [i, 0, 0]})).collect();
    let e = refused(&mut s, "cam.op.add", json!({"op": {"type": "pocket"}, "picks": many}));
    assert!(e.contains("1000"), "{e}");
    let e = refused(
        &mut s,
        "cam.op.add",
        json!({"op": {"type": "pocket", "polygon": [[0, 0], [1, 0], [1, 1]]}, "picks": [{"body": "Block", "at": [0, 0, 15]}]}),
    );
    assert!(e.contains("not both"), "{e}");
    let e = refused(&mut s, "cam.op.add", json!({"op": {"type": "pocket"}, "picks": [{"sketch": "S", "face": "F1"}]}));
    assert!(e.contains("not both"), "{e}");
    let e = refused(&mut s, "cam.picks.resolve", json!({}));
    assert!(e.contains("picks"), "{e}");
    let e = refused(&mut s, "cam.setup.zero", json!({"at": [0, 0, 0], "stock": {"x": "low", "y": "low", "z": "top"}}));
    assert!(e.contains("one of"), "{e}");
    let e = refused(&mut s, "cam.setup.zero", json!({"stock": {"x": "top", "y": "low", "z": "top"}}));
    assert!(e.contains("low | middle | high"), "{e}");
    let e = refused(&mut s, "cam.setup.zero", json!({"refresh": true}));
    assert!(e.contains("by numbers"), "{e}");
    let e = refused(&mut s, "cam.job.edit", json!({"set": {"origin_from": {"pick": {"body": "Block"}}}}));
    assert!(e.contains("origin_from"), "{e}");
    let e = refused(&mut s, "cam.op.defaults", json!({"draft": {"type": "nonsense"}}));
    assert!(e.contains("draft"), "{e}");
}

#[test]
fn a_chamfer_face_picked_breaks_the_rim_of_its_hole() {
    // A block with a 12 mm hole through it, a 0.5 mm chamfer round its top edge.
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 20, "corner": [-20, -15, 0], "body_name": "Block"}));
    run(&mut s, "solid.cylinder", json!({"radius": 6, "height": 20, "base": [0, 0, 0], "axis": "Z", "operation": "cut"}));
    run(&mut s, "solid.chamfer", json!({"edges": [[6, 0, 20]], "distance": 0.5}));
    open_job(&mut s);
    let tools: Value = json!([
        {"number": 4, "kind": "flat_endmill", "diameter": 4, "flute_length": 12, "spindle_rpm": 12000, "feed": 800, "plunge_feed": 200},
        {"number": 30, "kind": "chamfer", "diameter": 4, "tip_diameter": 0, "angle_deg": 90, "flute_length": 2, "spindle_rpm": 10000, "feed": 600, "plunge_feed": 100}
    ]);
    run(&mut s, "cam.job.edit", json!({"set": {"tools": tools}}));
    // The hole reads as countersunk; its cone, clicked on the view.
    let ff = run(&mut s, "cam.features", json!({}));
    assert!(ff["holes"][0]["countersink"].is_object(), "{ff}");
    let id = "hole:0";
    let face = json!([{"body": "Block", "at": [6.25, 0, 19.75]}]);
    run(&mut s, "cam.op.add", json!({"op": {"type": "chamfer", "tool": 30}, "picks": face}));
    let r = dry(&mut s);
    assert_eq!(r["passed"], true, "{} {}", r["failures"], r["picks_resolved"]);
    let ext = &r["operations"][0]["extents"]["cut"];
    // Round the hole's rim, inside it.
    assert!(f(&ext[0][0]) > -6.5 && f(&ext[1][0]) < 6.5 && f(&ext[0][0]) < -4.0, "{ext}");
    assert_eq!(r["picks_resolved"][0]["feature"], json!(id));
}

#[test]
fn light_load_clearing_on_an_open_step_says_to_clear_it_as_a_pocket() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 20, "corner": [-20, -15, 0], "body_name": "Block"}));
    // A step 10 deep across the left end: open on three sides.
    run(&mut s, "solid.box", json!({"length": 15, "width": 40, "height": 10, "corner": [-25, -20, 10], "operation": "cut"}));
    open_job(&mut s);
    let ff = run(&mut s, "cam.features", json!({"picks": true}));
    let step =
        ff["faces"].as_array().unwrap().iter().find(|f| f["role"] == "step").unwrap_or_else(|| panic!("{ff}"))["id"].as_str().unwrap().to_string();
    run(&mut s, "cam.op.add", json!({"op": {"type": "adaptive", "tool": 4}, "picks": ff["picks"][&step]}));
    let e = s.execute("cam.job", &json!({"dry_run": true})).unwrap_err().to_string();
    assert!(e.contains("open on some sides") && e.contains("Clear Pocket") && e.contains("material along them"), "{e}");
    // Clear Pocket on it plans.
    run(&mut s, "cam.op.remove", json!({"op": 0}));
    run(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "tool": 4}, "picks": ff["picks"][&step]}));
    let r = dry(&mut s);
    assert_eq!(r["passed"], true, "{}", r["failures"]);
}

#[test]
fn a_radial_hole_opening_on_the_bar_is_drilled_on_its_side() {
    let mut s = Session::default();
    run(&mut s, "solid.cylinder", json!({"radius": 16, "height": 90, "base": [-30, 0, 0], "axis": "X", "body_name": "Bar"}));
    // A 5 mm hole at 45 degrees, from 8 mm off the axis out through the round face.
    let k = std::f64::consts::FRAC_1_SQRT_2;
    run(&mut s, "solid.cylinder", json!({"radius": 2.5, "height": 12, "base": [12, 8.0 * k, 8.0 * k], "axis": [0, k, k], "operation": "cut"}));
    run(&mut s, "cam.job.new", json!({}));
    let tools: Value = serde_json::from_str(TOOLS).unwrap();
    run(
        &mut s,
        "cam.job.edit",
        json!({"set": {"machine": "4axis_a", "post": "mach3_4axis", "tools": tools, "stock": {"cylinder": {"radius": 16, "x_min": -30, "x_max": 60}}}}),
    );
    // Its wall, 13 mm out along its axis.
    let wall = json!([{"body": "Bar", "at": [14.5, 13.0 * k, 13.0 * k]}]);
    let r = run(&mut s, "cam.picks.resolve", json!({"picks": wall}));
    assert_eq!(r["picks"][0]["kind"], "cylinder", "{r}");
    assert_eq!(r["suits"][0]["type"], "drill", "{r}");
    assert_eq!(r["suits"][0]["tool"], 150, "the 5 mm drill: {r}");
    run(&mut s, "cam.op.add", json!({"op": {"type": "drill", "tool": 150}, "picks": wall}));
    let op = s.cam_job.as_ref().unwrap().value["operations"][0].clone();
    assert_eq!(op["type"], "indexed", "added turned to its side: {op}");
    let d = dry(&mut s);
    assert_eq!(d["passed"], true, "{}", d["failures"]);
    let op = &d["operations"][0];
    assert!((f(&op["a_deg"]).abs() - 45.0).abs() < 1e-6, "turned to its side: {op}");
    // The bar's own round face is no hole.
    let r = run(&mut s, "cam.picks.resolve", json!({"picks": [{"body": "Bar", "at": [40, 0, 16]}]}));
    assert!(r["suits"].as_array().unwrap().iter().all(|x| x["type"] != "bore" && x["type"] != "drill"), "{r}");
    // Its floor picked reads the same: the round wall round it.
    let floor = json!([{"body": "Bar", "at": [12.5, 8.0 * k, 8.0 * k]}]);
    let r = run(&mut s, "cam.picks.resolve", json!({"picks": floor}));
    assert_eq!(r["suits"][0]["type"], "drill", "{r}");
    let ext = &op["extents"]["cut"];
    assert!((f(&ext[0][0]) - 12.0).abs() < 1e-6 && (f(&ext[1][0]) - 12.0).abs() < 1e-6, "{ext}");
    // In work coordinates: down its axis to its floor, 8 mm off the bar's axis.
    assert!((f(&ext[0][1]).hypot(f(&ext[0][2])) - 8.0).abs() < 1e-6, "down to its floor 8 mm off the axis: {ext}");
}

/// The holes of `cam.features` as picks, all of them.
fn all_hole_picks(s: &mut Session) -> Vec<Value> {
    let ff = run(s, "cam.features", json!({"picks": true}));
    let mut picks = Vec::new();
    for h in ff["holes"].as_array().unwrap() {
        picks.extend(ff["picks"][h["id"].as_str().unwrap()].as_array().unwrap().iter().cloned());
    }
    picks
}

#[test]
fn holes_of_different_depths_or_sizes_are_not_drilled_as_one() {
    // A blind 5 mm hole 6 deep beside the through one: one drilling would take both through.
    let mut s = block();
    run(&mut s, "solid.cylinder", json!({"radius": 2.5, "height": 6, "base": [-14, 8, 14], "axis": "Z", "operation": "cut"}));
    open_job(&mut s);
    let picks = all_hole_picks(&mut s);
    assert_eq!(picks.len(), 2);
    for ty in ["drill", "bore"] {
        run(&mut s, "cam.op.add", json!({"op": {"type": ty, "tool": if ty == "drill" { 150 } else { 4 }}, "picks": picks}));
        let e = s.execute("cam.job", &json!({"dry_run": true})).unwrap_err().to_string();
        assert!(e.contains("different depths") && e.contains("one operation each"), "{ty}: {e}");
        // The form says why in words, without the agent's wrapping.
        let d = run(&mut s, "cam.op.defaults", json!({"op": 0}));
        let why = d["refused"].as_str().unwrap_or_default().to_string();
        assert!(why.starts_with("the holes go to different depths"), "{why}");
        assert!(!why.contains("refusals") && !why.contains("invalid parameters") && !why.contains("Nothing was written"), "{why}");
        run(&mut s, "cam.op.remove", json!({"op": 0}));
    }
    // A 6 mm hole through beside the 5 mm one: one drill would leave it 5 mm. Boring takes each
    // its own size.
    let mut s = block();
    run(&mut s, "solid.cylinder", json!({"radius": 3, "height": 20, "base": [-14, 8, 0], "axis": "Z", "operation": "cut"}));
    open_job(&mut s);
    let picks = all_hole_picks(&mut s);
    run(&mut s, "cam.op.add", json!({"op": {"type": "drill", "tool": 150}, "picks": picks}));
    let e = s.execute("cam.job", &json!({"dry_run": true})).unwrap_err().to_string();
    assert!(e.contains("different sizes") && e.contains("Mill Round Holes, which takes each hole's own size"), "{e}");
    run(&mut s, "cam.op.remove", json!({"op": 0}));
    run(&mut s, "cam.op.add", json!({"op": {"type": "bore", "tool": 4}, "picks": picks}));
    let r = s.execute("cam.job", &json!({"dry_run": true}));
    assert!(r.is_ok(), "boring takes each hole's own size: {r:?}");
}

#[test]
fn an_operation_turned_off_does_not_need_its_picks() {
    let mut s = block();
    open_job(&mut s);
    let floor = feature_picks(&mut s, "pocket:0");
    run(&mut s, "cam.op.add", json!({"op": {"type": "pocket", "tool": 4, "name": "gone"}, "picks": floor}));
    run(&mut s, "cam.op.add", json!({"op": {"type": "face", "tool": 4, "top": 20.5, "bottom": 20, "name": "top"}}));
    // The hole and the pocket undone: the pocket's floor is lost. Turned off, it stops nothing.
    run(&mut s, "edit.undo", json!({}));
    run(&mut s, "edit.undo", json!({}));
    run(&mut s, "cam.op.enable", json!({"op": "gone", "enabled": false}));
    let g = run(&mut s, "cam.generate", json!({}));
    assert!(g["findings"].as_array().unwrap().iter().all(|x| x["check"] != "picks"), "{g}");
    assert_eq!(g["passed"], true, "{g}");
    // Turned on again, it refuses as before.
    run(&mut s, "cam.op.enable", json!({"op": "gone", "enabled": true}));
    let g = run(&mut s, "cam.generate", json!({}));
    assert!(g["findings"].as_array().unwrap().iter().any(|x| x["check"] == "picks" && x["severity"] == "error"), "{g}");
}

#[test]
fn the_form_shows_the_heights_the_picks_give() {
    let mut s = block();
    open_job(&mut s);
    let wall = feature_picks(&mut s, "hole:0");
    run(&mut s, "cam.op.add", json!({"op": {"type": "drill", "tool": 150}, "picks": wall}));
    let d = run(&mut s, "cam.op.defaults", json!({"op": 0}));
    let r = dry(&mut s);
    let field = |p: &str| d["fields"].as_array().unwrap().iter().find(|x| x["path"] == p).unwrap_or_else(|| panic!("no {p}: {d}")).clone();
    // The drill's moves: down from its retract height, which the form says.
    let top = f(&r["operations"][0]["extents"]["all"][1][2]);
    let (clearance, retract) = (field("heights.clearance"), field("heights.retract"));
    assert_eq!(clearance["source"], "pick", "{clearance}");
    assert_eq!(retract["source"], "pick", "{retract}");
    assert!((f(&clearance["value"]) - top).abs() < 1e-9, "the form's clearance is the one cut with ({top}): {clearance}");
    assert!((f(&retract["value"]) - 22.0).abs() < 1e-9, "2 above the hole's top: {retract}");
}

#[test]
fn a_selection_whose_point_is_off_its_face_is_picked_on_the_face() {
    let mut s = block();
    open_job(&mut s);
    let ff = run(&mut s, "cam.features", json!({}));
    let floor = ff["pockets"][0]["floor_face"].as_u64().unwrap() as usize;
    // The point a row of the job tree passes for each of a feature's faces may be on another.
    s.selection = vec![crate::Sel::Face { body: "Block".into(), index: floor, point: solidcore_record::Vec3::new(14.0, 0.0, 20.0) }];
    let r = run(&mut s, "cam.picks.resolve", json!({"from_selection": true}));
    let at = &r["picks"][0]["pick"]["at"];
    assert!((f(&at[2]) - 15.0).abs() < 1e-6, "on the floor: {r}");
    assert_eq!(r["picks"][0]["feature"], "pocket:0", "{r}");
}

#[test]
fn work_zero_stays_on_a_bar_s_axis() {
    // A bar turns about the A axis through work zero: work zero on its top would take the bar off
    // the part, so only X moves.
    let mut s = Session::default();
    run(&mut s, "solid.cylinder", json!({"radius": 16, "height": 90, "base": [-30, 0, 0], "axis": "X", "body_name": "Bar"}));
    run(&mut s, "cam.job.new", json!({}));
    let tools: Value = serde_json::from_str(TOOLS).unwrap();
    run(
        &mut s,
        "cam.job.edit",
        json!({"set": {"machine": "4axis_a", "tools": tools, "stock": {"cylinder": {"radius": 16, "x_min": -30, "x_max": 60}}}}),
    );
    let before = s.cam_job.as_ref().unwrap().value.clone();
    let e = refused(&mut s, "cam.setup.zero", json!({"stock": {"x": "low", "y": "middle", "z": "top"}}));
    assert!(e.contains("bar's axis") && e.contains("middle"), "{e}");
    // A point on its end face off the axis; its round face means its axis, which may.
    let e = refused(&mut s, "cam.setup.zero", json!({"pick": {"body": "Bar", "at": [-30, 0, 10]}}));
    assert!(e.contains("bar's axis"), "{e}");
    assert_eq!(s.cam_job.as_ref().unwrap().value, before, "nothing changed");
    let r = run(&mut s, "cam.setup.zero", json!({"pick": {"body": "Bar", "at": [0, 0, 16]}}));
    assert_eq!(r["origin"], json!([0.0, 0.0, 0.0]), "{r}");
    let r = run(&mut s, "cam.setup.zero", json!({"stock": {"x": "low", "y": "middle", "z": "middle"}}));
    assert_eq!(r["origin"], json!([-30.0, 0.0, 0.0]), "{r}");
    // Typed numbers are the person's own: kept, with the note.
    let r = run(&mut s, "cam.setup.zero", json!({"at": [-30, 0, 16]}));
    assert!(r["notes"].to_string().contains("bar"), "{r}");
}
