//! The CAM simulation view's layer of the model scene: what it draws, and that a frame with no
//! new cut builds no block again.

use serde_json::{Value, json};
use solidcore_record::Vec3;

use super::*;
use crate::scenario::Harness;

/// A 40 mm block with a dome on it, and a ball-nose parallel finish over the dome.
pub(crate) fn dome(h: &mut Harness) -> Value {
    h.app.home.open = false;
    h.app.run("solid.box", json!({"length": 40, "width": 40, "height": 20, "corner": [-20, -20, -20], "body_name": "Block"})).unwrap();
    h.app.run("solid.sphere", json!({"radius": 12, "center": [0, 0, -5], "operation": "join"})).unwrap();
    json!({
        "name": "dome",
        "machine": "3axis",
        "post": "mach3_4axis",
        "tools": [{"number": 2, "kind": "ball_endmill", "diameter": 6.0, "flute_length": 12, "spindle_rpm": 12000, "feed": 900, "plunge_feed": 250,
                   "holder": [{"diameter": 20, "length": 15}, {"diameter": 35, "length": 30}]}],
        "operations": [{
            "name": "finish", "type": "parallel", "tool": 2,
            "heights": {"clearance": 30, "retract": 15, "top": 7, "bottom": -1},
            "stepover": 1.5, "boundary": {"rect": [[-14, -14], [14, 14]]}
        }]
    })
}

pub(crate) fn loaded() -> Harness {
    let mut h = Harness::new();
    let job = dome(&mut h);
    h.app.run("cam.view.load", json!({"job": job, "resolution": 0.25})).unwrap();
    h
}

fn keys(sc: &GpuScene) -> Vec<u64> {
    sc.blocks.iter().map(|b| b.key).collect()
}

#[test]
fn the_layer_draws_the_stock_the_paths_and_the_tool_with_its_holder() {
    let mut h = loaded();
    h.app.run("cam.view.seek", json!({"percent": 50})).unwrap();
    let sc = crate::viewport::build_scene(&h.app);
    let v = h.app.session.cam_view.as_deref().unwrap();
    let p = v.playback().unwrap();
    let (tx, ty) = p.plan().grid().tiles();
    let stock_blocks = tx.div_ceil(GROUP) * ty.div_ceil(GROUP);
    let path_blocks = p.timeline().moves.len().div_ceil(CHUNK);
    assert!(sc.blocks.len() >= stock_blocks + path_blocks, "{} blocks for {stock_blocks} stock and {path_blocks} path", sc.blocks.len());
    // The tool's cutting end and its two holder sections (as one mesh), each placed once.
    assert!(sc.instanced.len() >= 2, "{} instanced meshes", sc.instanced.len());
    assert!(sc.instanced.iter().all(|m| m.count() == 1));
    let lines: usize = sc.blocks.iter().map(|b| b.bytes.lines.len()).sum();
    assert!(lines > 0, "the toolpaths are drawn");
    // The part is hidden: the stock is cut where it will be.
    assert!(!part_shown(&h.app));
    h.app.run("cam.view.show", json!({"part": true})).unwrap();
    assert!(part_shown(&h.app));
}

#[test]
fn a_frame_without_a_new_cut_builds_no_block_again_and_a_step_builds_few() {
    let mut h = loaded();
    h.app.run("cam.view.seek", json!({"percent": 30})).unwrap();
    let a = crate::viewport::build_scene(&h.app);
    let b = crate::viewport::build_scene(&h.app);
    assert_eq!(keys(&a), keys(&b));
    for (x, y) in a.blocks.iter().zip(&b.blocks) {
        assert!(Arc::ptr_eq(&x.bytes, &y.bytes), "block {} was built again", x.key);
    }
    // One move on: the tool's chunk of the paths and the stock it cut change, nothing else.
    h.app.run("cam.view.seek", json!({"by": 1})).unwrap();
    let c = crate::viewport::build_scene(&h.app);
    let changed = c.blocks.iter().filter(|x| !a.blocks.iter().any(|y| y.key == x.key)).count();
    // The path's chunk and the stock blocks round the cut: a few of them, not all.
    assert!((1..=4).contains(&changed) && changed < c.blocks.len(), "{changed} of {} blocks built again", c.blocks.len());
}

#[test]
fn the_key_follows_the_view_and_the_layers() {
    let mut h = loaded();
    let k0 = scene_key(&h.app);
    assert_eq!(k0, scene_key(&h.app));
    h.app.run("cam.view.seek", json!({"by": 3})).unwrap();
    let k1 = scene_key(&h.app);
    assert_ne!(k0, k1);
    h.app.run("cam.view.show", json!({"rapids": false})).unwrap();
    assert_ne!(k1, scene_key(&h.app));
    h.app.run("cam.view.close", json!({})).unwrap();
    assert_eq!(scene_key(&h.app), 0);
    assert!(part_shown(&h.app));
}

#[test]
fn tiles_built_ahead_are_found_and_only_changed_ones_are_built_again() {
    let mut h = loaded();
    let tiles = Mutex::new(Tiles::default());
    let v = h.app.session.cam_view.as_deref().unwrap();
    let (tx, ty) = v.playback().unwrap().plan().grid().tiles();
    assert_eq!(refresh_tiles(v, &tiles), tx * ty);
    assert_eq!(refresh_tiles(v, &tiles), 0, "nothing changed");
    h.app.run("cam.view.seek", json!({"percent": 40})).unwrap();
    let v = h.app.session.cam_view.as_deref().unwrap();
    let again = refresh_tiles(v, &tiles);
    assert!(again > 0 && again < tx * ty, "{again} of {} tiles built again", tx * ty);
}

#[test]
fn a_tool_mesh_is_shaded_round_and_placed_along_its_axis() {
    let m = cylinder(0.0, 0.0, 3.0, 0.0, 10.0);
    let s = shaded(&m);
    assert_eq!(s.positions.len(), s.normals.len());
    assert_eq!(s.positions.len(), 3 * s.triangles.len());
    for (p, n) in s.positions.iter().zip(&s.normals) {
        assert!((n.len() - 1.0).abs() < 1e-9);
        // On the side, the normal points away from the axis; on the caps, along it.
        if p.z > 1e-9 && p.z < 10.0 - 1e-9 {
            assert!(n.z.abs() < 1e-9);
        }
    }
    let side: Vec<&Vec3> = s.normals.iter().filter(|n| n.z.abs() < 1e-9).collect();
    assert!(side.iter().any(|n| n.x.abs() > 1e-3 && n.y.abs() > 1e-3), "the side is smooth, not faceted to the axes");
    // A tool lying along +X (the spindle turned a quarter about Y).
    let rows = placement(Vec3::new(1.0, 2.0, 3.0), [Vec3::new(0.0, 0.0, -1.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)]);
    let apply = |p: [f32; 3]| rows.map(|r| r[0] * p[0] + r[1] * p[1] + r[2] * p[2] + r[3]);
    assert_eq!(apply([0.0, 0.0, 0.0]), [1.0, 2.0, 3.0]);
    let up = apply([0.0, 0.0, 10.0]);
    assert!((up[0] - 11.0).abs() < 1e-5 && (up[1] - 2.0).abs() < 1e-5 && (up[2] - 3.0).abs() < 1e-5, "{up:?}");
}

#[test]
fn colours_come_from_the_theme_and_cycle() {
    let t = Tokens::view();
    assert_eq!(category(&t, 0), t.accent);
    assert_eq!(category(&t, series(&t).len()), t.accent);
}

/// A machine model whose one spindle part, a side plate, stands off to the spindle's +X.
const SIDE_PLATE: &str = r#"{"format": "solidcraft.machine_model", "version": 1, "name": "test mill",
    "spindle": {"nose_gap": 5, "nose_max_above_table": 400},
    "table": {"size": [300, 200, 15], "cut_allowance": 0.1},
    "parts": [{"name": "the side plate", "code": "side-plate", "mount": "spindle",
               "shape": {"box": {"min": [30, -5, 0], "max": [40, 5, 60]}}}]}"#;

#[test]
fn a_machine_part_is_drawn_on_the_side_of_the_spindle_the_collision_check_puts_it() {
    let mut h = Harness::new();
    let job = dome(&mut h);
    let file = std::env::temp_dir().join(format!("sc-side-plate-{}.machine-model.json", std::process::id()));
    std::fs::write(&file, SIDE_PLATE).unwrap();
    h.app.run("cam.view.load", json!({"job": job, "resolution": 0.25, "machine_model_path": file.to_string_lossy()})).unwrap();
    h.app.run("cam.view.seek", json!({"percent": 50})).unwrap();
    let _ = std::fs::remove_file(&file);
    let _ = crate::viewport::build_scene(&h.app);
    let v = h.app.session.cam_view.as_deref().unwrap();
    let p = v.playback().unwrap();
    let pose = p.pose_now().unwrap();
    let tool = p.tool(p.plan().step(p.at().0).unwrap().slot).unwrap().clone();
    let model = p.setup().model.clone().unwrap();
    let nose = pose.tip + v.origin + pose.axis * nose_above_tip(&tool, Some(model.as_ref()));
    let cache = h.app.cam_player.layer.lock().unwrap();
    let drawn = cache.last.as_ref().unwrap();
    let (_, _, places) = drawn.instanced.iter().find(|(k, _, _)| *k == hash_of(("machine", 0usize))).expect("the side plate is drawn");
    let rows = places[0].0;
    let apply = |q: [f32; 3]| rows.map(|r| r[0] * q[0] + r[1] * q[1] + r[2] * q[2] + r[3]);
    // The plate's centre, (35, 0, 30) from the nose, as the collision check's zone has it.
    let c = apply([35.0, 0.0, 30.0]);
    let want = nose + Vec3::new(35.0, 0.0, 30.0);
    assert!(
        (f64::from(c[0]) - want.x).abs() < 1e-3 && (f64::from(c[1]) - want.y).abs() < 1e-3 && (f64::from(c[2]) - want.z).abs() < 1e-3,
        "drawn at {c:?}, checked at {want:?}"
    );
}
