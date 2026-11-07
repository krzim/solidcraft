//! Operations by hand: the descriptor table covers every key of every operation type's schema
//! (or says why an agent alone writes it), every label is in the catalogs, and in the workspace a
//! pocket's floor picked in the view is machined from *Machine this*, a value is set and reset
//! from its row, the Add menu is grouped and filtered, a *Found in the part* row selects its faces,
//! and work zero is set on the stock and at the selection.

use serde_json::{Value, json};
use solidcraft_engine::cam_job_state::OP_TYPES;

use super::*;
use crate::scenario::Harness;

#[test]
fn every_key_of_every_operation_is_a_form_row_or_agent_only() {
    let mut missing = Vec::new();
    for (ty, _, _) in OP_TYPES {
        let keys = solidcraft_engine::schema::cam_picks::op_keys(ty);
        assert!(!keys.is_empty(), "{ty} has a schema");
        for (key, _) in keys {
            let shown = match key.as_str() {
                "heights" => field("heights.top").is_some(),
                "feeds" => field("feeds.cut").is_some() && field("feeds.plunge").is_some(),
                other => field(other).is_some(),
            };
            let agent = AGENT_ONLY_KEYS.iter().any(|(k, why)| *k == key && !why.is_empty());
            if !shown && !agent {
                missing.push(format!("{ty}.{key}"));
            }
        }
        // The rows the form draws for it are the schema's keys, none twice.
        let rows = rows(ty);
        let mut keys: Vec<&str> = rows.iter().map(|f| f.key).collect();
        keys.sort_unstable();
        let n = keys.len();
        keys.dedup();
        assert_eq!(keys.len(), n, "{ty}: a row twice");
    }
    assert!(missing.is_empty(), "keys the form does not show (add a row, or say why on AGENT_ONLY_KEYS): {missing:?}");
    // And nothing on both lists.
    for (k, _) in AGENT_ONLY_KEYS {
        assert!(field(k).is_none(), "{k} is a row and agent-only");
    }
}

#[test]
fn every_label_is_in_the_catalogs() {
    let mut ids: Vec<String> = crate::manufacture_op::label_ids();
    ids.extend(Section::ALL.iter().map(|s| s.label().to_string()));
    ids.extend(["top", "step", "plateau", "island_top"].iter().map(|r| format!("ui.manufacture.role.{r}")));
    ids.extend(["name", "point", "lost"].iter().map(|r| format!("ui.manufacture.found_by.{r}")));
    ids.extend(
        ["given", "tool", "cutting_data", "profile", "model", "stock", "feature", "pick", "rule"]
            .iter()
            .map(|r| format!("ui.manufacture.source.{r}")),
    );
    for f in FIELDS {
        if let Kind::Choice(words) = f.kind {
            ids.extend(words.iter().map(|w| format!("ui.manufacture.choice.{w}")));
        }
    }
    for lang in ["en", "ar", "he", "ja", "vi", "zh", "de", "ko"] {
        let locale = crate::i18n::Locale::for_language(lang);
        let missing: Vec<&String> = ids.iter().filter(|id| locale.text(id) == id.as_str()).collect();
        assert!(missing.is_empty(), "{lang} lacks {missing:?}");
    }
}

/// A 40 × 30 × 20 block with a 16 × 10 pocket 5 deep and a 5 mm hole, a new job open in the
/// Manufacture workspace with a 4 mm end mill and a 5 mm drill.
fn harness() -> Harness {
    let mut h = Harness::new();
    h.app.run("solid.box", json!({"length": 40, "width": 30, "height": 20, "corner": [-20, -15, 0], "body_name": "Block"})).unwrap();
    h.app.run("solid.box", json!({"length": 16, "width": 10, "height": 5, "corner": [-8, -5, 15], "operation": "cut"})).unwrap();
    h.app.run("solid.cylinder", json!({"radius": 2.5, "height": 20, "base": [14, 0, 0], "axis": "Z", "operation": "cut"})).unwrap();
    crate::manufacture_workspace::switch(&mut h.app, "manufacture").unwrap();
    h.app.run("cam.job.new", json!({"name": "by hand"})).unwrap();
    let tools = json!([
        {"number": 4, "kind": "flat_endmill", "diameter": 4, "flute_length": 12, "spindle_rpm": 12000, "feed": 800, "plunge_feed": 200},
        {"number": 150, "kind": "drill", "diameter": 5, "flute_length": 30, "spindle_rpm": 3000, "feed": 200, "plunge_feed": 100}
    ]);
    h.app.run("cam.job.edit", json!({"set": {"machine": "3axis", "post": "mach3_4axis", "tools": tools}})).unwrap();
    h.frames(2);
    h
}

fn ops(h: &Harness) -> Vec<Value> {
    h.app.session.cam_job.as_ref().map(|j| j.operations().to_vec()).unwrap_or_default()
}

/// Select feature `id`'s first face, as a click in the view does.
fn select_feature(h: &mut Harness, id: &str) {
    let f = h.app.session.execute("cam.features", &json!({"picks": true})).unwrap();
    let pick = &f["picks"][id][0];
    let st = h.app.session.world_state().unwrap();
    let body = st.bodies.iter().find(|b| Some(b.name.as_str()) == pick["body"].as_str()).unwrap();
    let face = solidcraft_engine::doc::naming::face_names(body).iter().position(|n| Some(n.as_str()) == pick["face"].as_str()).unwrap();
    h.app.run("select.set", json!({"items": [{"type": "face", "body": body.name, "index": face, "point": pick["at"]}]})).unwrap();
}

#[test]
fn a_pocket_floor_picked_in_the_view_is_machined_from_machine_this() {
    let mut h = harness();
    select_feature(&mut h, "pocket:0");
    h.step(&json!({"key": "M"})).unwrap();
    h.frames(3);
    assert_eq!(h.app.manufacture.stage, crate::manufacture_workspace::Stage::Operations);
    let resolve = h.app.manufacture.ops.resolve.clone().unwrap().value.unwrap();
    assert_eq!(resolve["suits"][0]["type"], "pocket", "{resolve}");
    let undo = h.app.session.cam_job.as_ref().unwrap().undo.len();
    h.step(&json!({"click": {"handle": "manufacture:suit:pocket"}})).unwrap();
    h.frames(3);
    let op = ops(&h)[0].clone();
    assert_eq!(op["type"], "pocket");
    assert_eq!(op["tool"], 4, "the tool Machine this offered");
    assert!(op["picks"][0]["face"].is_string(), "stored by the face's name: {op}");
    assert!(op.get("feature").is_none() && op.get("heights").is_none(), "{op}");
    assert_eq!(h.app.session.cam_job.as_ref().unwrap().undo.len(), undo + 1, "one step");
    assert_eq!(h.app.manufacture.selected, Some(0));
    // Its defaults: the floor from the pocket, the stepdown from the rule.
    let d = h.app.manufacture.ops.defaults.clone().unwrap().value.unwrap();
    let field = |p: &str| d["fields"].as_array().unwrap().iter().find(|f| f["path"] == p).unwrap().clone();
    assert_eq!(field("heights.bottom")["value"], 15.0, "{d}");
    assert_eq!(field("stepdown")["source"], "rule");
    // An override, then Reset from its row.
    h.app.run("cam.op.edit", json!({"op": 0, "set": {"heights.clearance": 25}})).unwrap();
    h.frames(3);
    assert!(crate::scenario::rect_of("manufacture:field:heights.clearance:reset").is_some());
    h.step(&json!({"click": {"handle": "manufacture:field:heights.clearance:reset"}})).unwrap();
    h.frames(2);
    assert!(
        ops(&h)[0].get("heights").and_then(|x| x.get("clearance")).is_none(),
        "reset takes the key out: {} {:?} {:?} {:?}",
        ops(&h)[0],
        h.app.manufacture.error,
        crate::scenario::rect_of("manufacture:field:heights.clearance:reset"),
        crate::scenario::rect_of("manufacture:stage")
    );
    // The tool list: the drill does not fit, and says why.
    let drill = d["tools"].as_array().unwrap().iter().find(|t| t["number"] == 150).unwrap().clone();
    assert_eq!(drill["fits"], false);
    assert!(crate::scenario::rect_of("manufacture:tool:150").is_none(), "the tool list is folded");
    h.step(&json!({"click": {"handle": "manufacture:tool:list"}})).unwrap();
    h.frames(1);
    h.step(&json!({"click": {"handle": "manufacture:tool:150"}})).unwrap();
    h.frames(1);
    assert_eq!(ops(&h)[0]["tool"], 4, "a tool that does not fit is not taken");
    // And Generate plans it.
    crate::manufacture_ui::generate_now(&mut h.app);
    let g = h.app.session.cam_job.as_ref().unwrap().generated.clone().unwrap();
    assert!(g.findings.iter().all(|f| f.severity != "error"), "{:?}", g.findings);
}

#[test]
fn the_add_menu_is_grouped_and_filtered() {
    let mut h = harness();
    h.step(&json!({"key": "A"})).unwrap();
    h.frames(2);
    assert!(h.app.manufacture.ops.add_open);
    assert!(crate::scenario::rect_of("manufacture:add:face").is_some());
    assert!(crate::scenario::rect_of("manufacture:add:rotary_finish").is_none(), "no A axis, no turning");
    h.app.manufacture.ops.add_filter = "hole".into();
    h.frames(2);
    assert!(crate::scenario::rect_of("manufacture:add:drill").is_some());
    assert!(crate::scenario::rect_of("manufacture:add:pocket").is_none(), "filtered out");
    h.step(&json!({"click": {"handle": "manufacture:add:drill"}})).unwrap();
    h.frames(2);
    assert_eq!(ops(&h)[0]["type"], "drill");
    assert!(!h.app.manufacture.ops.add_open);
}

#[test]
fn a_found_row_selects_its_faces_and_offers_what_suits_them() {
    let mut h = harness();
    h.frames(3);
    assert!(crate::scenario::rect_of("manufacture:found:hole:0").is_some(), "the hole group's row");
    h.step(&json!({"click": {"handle": "manufacture:found:hole:0"}})).unwrap();
    h.frames(3);
    assert!(h.app.session.selection.iter().any(|s| matches!(s, solidcraft_engine::Sel::Face { .. })));
    let r = h.app.manufacture.ops.resolve.clone().unwrap().value.unwrap();
    assert_eq!(r["suits"][0]["type"], "drill", "a drill of its size is in the tools: {r}");
    h.step(&json!({"click": {"handle": "manufacture:suit:drill"}})).unwrap();
    let op = ops(&h)[0].clone();
    assert_eq!((op["type"].clone(), op["tool"].clone()), (json!("drill"), json!(150)));
    // Suggest adds the rest, as picks.
    h.step(&json!({"click": {"handle": "manufacture:suggest"}})).unwrap();
    assert!(ops(&h).len() > 1);
}

#[test]
fn work_zero_is_set_on_the_stock_and_at_the_selection() {
    let mut h = harness();
    h.app.run("cam.job.edit", json!({"set": {"stock": {"around_part": {"margin": [2, 2, 1], "bottom": 0}}}})).unwrap();
    h.app.run("view.manufacture_stage", json!({"stage": "setup"})).unwrap();
    h.frames(2);
    h.step(&json!({"click": {"handle": "manufacture:zero:stock"}})).unwrap();
    let job = h.app.session.cam_job.as_ref().unwrap().value.clone();
    assert_eq!(job["origin"], json!([-22.0, -17.0, 21.0]), "the default place: low X, low Y, top");
    assert!(job["origin_from"]["stock"].is_object());
    select_feature(&mut h, "hole:0");
    h.frames(2);
    h.step(&json!({"click": {"handle": "manufacture:zero:selection"}})).unwrap();
    let o = h.app.session.cam_job.as_ref().unwrap().value["origin"].clone();
    assert!((o[0].as_f64().unwrap() - 14.0).abs() < 1e-6, "on the hole's axis: {o}");
    // Typed numbers drop the pick.
    h.app.run("cam.setup.zero", json!({"at": [0, 0, 20]})).unwrap();
    assert!(h.app.session.cam_job.as_ref().unwrap().value.get("origin_from").is_none());
}
