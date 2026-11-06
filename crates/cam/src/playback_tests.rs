//! Tests of playing a program: the cutting model against closed forms through the player (a
//! pocket's floor and walls, a ball slot's profile, a groove round a bar), seeking forward and
//! back through keyframes to the same stock, seeking by time, line and operation, what is found
//! where (a rapid in the stock, a gouge, cutting with the spindle stopped, the shank rubbing, the
//! holder on the stock, a machine part and the cutter against a clamp), and the display meshes.

use std::sync::Arc;

use solidcore_record::Vec3;

use crate::findings::Severity;
use crate::foundation_tests::*;
use crate::kinematics::Machine;
use crate::machine_model::{MachineModel, ObstacleKind};
use crate::machine_time::{DwellUnit, MachineRates};
use crate::playback::{Budget, MoveKind, OpsGiven, Playback, PlaybackSetup, Timeline};
use crate::sim::{SimParams, StockShape, plan_sim, simulate};
use crate::stock_mesh::{CLASS_TOOL, CLASS_UNCUT, display_step, tile_mesh};
use crate::tool_mesh::tool_meshes;
use crate::workholding::{Workholding, build};
use crate::*;

fn flat(d: f64) -> Tool {
    Tool::new(1, ToolKind::FlatEndmill, d, 20.0)
}

fn program(moves: &str) -> String {
    format!("%\nG90 G94 G91.1 G17\nG21\nT1 M6\nS12000 M3\nG0 X0. Y0.\nG43 Z10. H1\n{moves}\nM5\nG28 G91 Z0.\nG90\nM30\n%\n")
}

fn stock() -> StockShape {
    StockShape::Box { min: Vec3::new(0.0, 0.0, -10.0), max: Vec3::new(40.0, 40.0, 0.0) }
}

fn rates() -> MachineRates {
    MachineRates { rapid_xyz_mm_min: 6000.0, ..MachineRates::default() }
}

/// A player of `text` with `tools` on `stock` against `part`, keyframes every `visits` cells.
fn player(text: &str, machine: &Machine, tools: &[Tool], stock: &StockShape, part: Surface, setup: Option<PlaybackSetup>, visits: f64) -> Playback {
    let plan = Arc::new(plan_sim(text, machine, tools, stock, &SimParams { tolerance: 0.01, resolution: Some(0.1) }).unwrap());
    let timeline = Arc::new(Timeline::build(&plan, text, &rates(), DwellUnit::Seconds, None).unwrap());
    let setup = setup.unwrap_or(PlaybackSetup { tools: tools.to_vec(), ..PlaybackSetup::default() });
    Playback::new(plan, timeline, Arc::new(part), setup, Budget { keyframe_visits: visits, keyframe_bytes: 64 << 20 }, true)
}

fn height(p: &Playback, x: f64, y: f64) -> f64 {
    let crate::sim::Grid::Heights { x0, y0, dx, dy, .. } = *p.plan().grid() else { panic!() };
    p.state().value(p.plan(), ((x - x0) / dx).floor() as usize, ((y - y0) / dy).floor() as usize).unwrap()
}

fn pocket() -> String {
    let mut moves = String::from("G0 X13. Y13.\nG0 Z1.\nG1 Z-3. F200\nG1 X27. F800\nY27.\nX13.\nY13.\n");
    let mut y = 13.0;
    while y < 27.0 {
        moves.push_str(&format!("G1 Y{y:.3}\nX27.\nG1 Y{:.3}\nX13.\n", y + 2.0));
        y += 4.0;
    }
    moves.push_str("G0 Z10.\n");
    program(&moves)
}

/// Whether two stocks are the same to rounding.
fn same(a: &crate::sim::StockGrid, b: &crate::sim::StockGrid) -> bool {
    match (a, b) {
        (crate::sim::StockGrid::Heights { heights: x, .. }, crate::sim::StockGrid::Heights { heights: y, .. }) => {
            x.len() == y.len() && x.iter().zip(y).all(|(p, q)| (p - q).abs() < 1e-9)
        }
        _ => false,
    }
}

#[test]
fn a_pocket_played_has_its_floor_and_walls_and_seeking_back_gives_the_same_stock() {
    let part = block(Vec3::new(0.0, 0.0, -10.0), Vec3::new(40.0, 40.0, -3.0));
    let text = pocket();
    let whole =
        simulate(&text, &Machine::three_axis(), &[flat(6.0)], &stock(), &part, &SimParams { tolerance: 0.01, resolution: Some(0.1) }).unwrap();
    // Whole moves, keyframes every 20 000 cells: back and forth to the same stock, bit for bit.
    let mut p = player(&text, &Machine::three_axis(), &[flat(6.0)], &stock(), part.clone(), None, 20_000.0);
    let n = p.plan().moves();
    p.seek(n, 0.0, None).unwrap();
    assert!(p.keyframes() > 2, "{}", p.keyframes());
    assert_eq!(p.state().grid(p.plan()), whole.stock);
    let g = p.generation();
    p.seek(n / 3, 1.0, None).unwrap();
    assert!(p.generation() > g);
    assert_eq!(p.at(), (n / 3, 1.0));
    p.seek(n, 0.0, None).unwrap();
    assert_eq!(p.state().grid(p.plan()), whole.stock);
    // The floor and the walls, exactly.
    assert_eq!(height(&p, 20.0, 20.0), -3.0);
    assert_eq!(height(&p, 10.05, 20.0), -3.0);
    assert_eq!(height(&p, 9.95, 20.0), 0.0);
    assert_eq!(height(&p, 20.0, 29.95), -3.0);
    assert_eq!(height(&p, 20.0, 30.05), 0.0);
    assert_eq!(height(&p, 10.25, 10.25), 0.0);
    // Nothing wrong with this program.
    assert!(p.findings().is_empty(), "{:?}", p.findings().events());
    // A frame's worth at a time (parts of moves too): the same stock, to rounding.
    let mut q = player(&text, &Machine::three_axis(), &[flat(6.0)], &stock(), part, None, 20_000.0);
    let mut frames = 0;
    while !q.seek(n / 2, 0.5, Some(5_000.0)).unwrap().done {
        frames += 1;
        assert!(frames < 10_000);
    }
    assert!(frames > 2, "the budget split the seek: {frames}");
    assert_eq!(q.at(), (n / 2, 0.5));
    while !q.seek(n, 0.0, Some(5_000.0)).unwrap().done {}
    assert!(same(&q.state().grid(q.plan()), &whole.stock));
    // The display: tiles of the stock, coloured by the tool that cut them last.
    let step = display_step(p.plan(), 1024);
    let tile = tile_mesh(p.plan(), p.state(), 12, 12, step);
    assert!(!tile.triangles.is_empty() && tile.tri_face.len() == tile.triangles.len());
    assert!(tile.tri_face.contains(&CLASS_TOOL));
    assert!(tile_mesh(p.plan(), p.state(), 0, 0, step).tri_face.iter().all(|c| *c == CLASS_UNCUT));
}

#[test]
fn a_ball_slot_played_in_pieces_has_the_ball_profile() {
    let part = block(Vec3::new(0.0, 0.0, -10.0), Vec3::new(40.0, 40.0, -4.0));
    let ball = Tool::new(1, ToolKind::BallEndmill, 6.0, 20.0);
    let text = program("G0 X5. Y20.\nG0 Z1.\nG1 Z-3. F200\nG1 X35. F800\nG0 Z10.");
    let mut p = player(&text, &Machine::three_axis(), std::slice::from_ref(&ball), &stock(), part, None, 1.0e9);
    // The slot in ten pieces of its move, then the rest.
    let slot = p.timeline().at_line(11).unwrap();
    for k in 1..=10 {
        p.seek(slot, k as f64 * 0.1, None).unwrap();
    }
    p.seek(usize::MAX, 0.0, None).unwrap();
    for v in [0.05, 0.95, 1.55, 2.05, 2.55, 2.95] {
        let want = -3.0 + 3.0 - (9.0f64 - v * v).sqrt();
        close(height(&p, 20.0, 20.0 + v), want, 1e-9);
        close(height(&p, 20.0, 20.0 - v), want, 1e-9);
    }
    assert_eq!(height(&p, 20.0, 23.05), 0.0);
    // The timeline: a plunge, a feed, rapids, times in order.
    let t = p.timeline();
    assert_eq!(t.moves.get(slot).map(|m| m.kind), Some(MoveKind::Feed));
    assert_eq!(t.moves.get(slot - 1).map(|m| m.kind), Some(MoveKind::Plunge));
    assert!(t.moves.windows(2).all(|w| w[0].t1 <= w[1].t0 + 1e-12));
    // 30 mm at 800 mm/min.
    close(t.moves[slot].t1 - t.moves[slot].t0, 30.0 / 800.0 * 60.0, 1e-9);
    // By time: the middle of the slot.
    let mid = 0.5 * (t.moves[slot].t0 + t.moves[slot].t1);
    let (m, f) = t.at_time(mid);
    assert_eq!(m, slot);
    close(f, 0.5, 1e-9);
}

#[test]
fn a_groove_round_a_bar_is_as_deep_as_the_tool_went() {
    let machine = Machine::four_axis_a();
    let bar = StockShape::Cylinder { x_min: -10.0, x_max: 10.0, radius: 10.0 };
    let part = cylinder_x(6.0, -15.0, 15.0, 64);
    let text = "G21\nT1 M6\nS12000 M3\nG0 X0. Y0. Z30. A0.\nG1 Z8. F100\nG1 A360. F2000\nG0 Z30.\n";
    let mut p = player(text, &machine, &[flat(4.0)], &bar, part, None, 1.0e9);
    p.seek(usize::MAX, 0.0, None).unwrap();
    let crate::sim::Grid::Radii { x0, dx, nx, nt, .. } = *p.plan().grid() else { panic!() };
    for k in 0..nt {
        for i in 0..nx {
            let x = x0 + (i as f64 + 0.5) * dx;
            let r = p.state().value(p.plan(), i, k).unwrap();
            if x.abs() < 1.5 {
                // Within half the tolerance of the depth, all the way round.
                assert!((8.0 - 1e-9..=8.005 + 1e-9).contains(&r), "x {x} k {k}: {r}");
            } else if x.abs() > 2.1 {
                assert_eq!(r, 10.0, "x {x}");
            }
        }
    }
    assert!(p.findings().is_empty(), "{:?}", p.findings().events());
}

#[test]
fn what_goes_wrong_is_found_at_its_move_and_place() {
    let part = block(Vec3::new(0.0, 0.0, -10.0), Vec3::new(40.0, 40.0, -5.0));
    // A rapid through the stock at line 9, then a cut 1 mm into the part at line 12.
    let text = program("G0 X5. Y20.\nG0 Z-2.\nG0 X10.\nG1 Z-6. F200\nG1 X30. F800\nG0 Z10.");
    let mut p = player(&text, &Machine::three_axis(), &[flat(6.0)], &stock(), part.clone(), None, 1.0e9);
    p.seek(usize::MAX, 0.0, None).unwrap();
    let ev = p.findings().events();
    let rapid = ev.iter().find(|e| e.code == "rapid-in-stock").unwrap();
    assert_eq!((rapid.line, rapid.severity), (9, Severity::Error));
    close(rapid.depth, 2.0, 1e-9);
    let gouge = ev.iter().find(|e| e.code == "gouge").unwrap();
    assert_eq!(gouge.line, 11);
    close(gouge.depth, 1.0, 0.02);
    assert!(gouge.at.unwrap().z < -5.0);
    // Findings so far grow as the tool goes.
    assert!(p.findings().merged_until(p.timeline().at_line(9).unwrap()).is_empty());
    assert!(p.findings().merged_until(p.timeline().at_line(9).unwrap() + 1).has("rapid-in-stock"));

    // With the spindle stopped, and the shank rubbing.
    let short = Tool::new(1, ToolKind::FlatEndmill, 6.0, 2.0);
    let text = "G21\nT1 M6\nS12000 M3\nG0 X5. Y20. Z1.\nG1 Z-4. F200\nG1 X15. F800\nM5\nG1 X25.\nG0 Z10.\n";
    let mut p = player(text, &Machine::three_axis(), std::slice::from_ref(&short), &stock(), part.clone(), None, 1.0e9);
    p.seek(usize::MAX, 0.0, None).unwrap();
    let merged = p.findings().merged();
    assert!(merged.has("cut-spindle-off") && merged.has("shank-rubs"), "{:?}", merged.list());
    assert_eq!(p.findings().events().iter().find(|e| e.code == "cut-spindle-off").unwrap().line, 8);

    // A holder (Ø20) 10 mm above the tip, the tool plunged 12 deep: the holder comes down on it.
    let held =
        Tool { stickout: Some(10.0), holder: vec![HolderSection { diameter: 20.0, length: 20.0 }], ..Tool::new(1, ToolKind::FlatEndmill, 6.0, 10.0) };
    let text = program("G0 X20. Y20.\nG0 Z1.\nG1 Z-12. F200\nG0 Z10.");
    let mut p = player(&text, &Machine::three_axis(), std::slice::from_ref(&held), &stock(), Surface::new(Vec::new()).unwrap(), None, 1.0e9);
    p.seek(usize::MAX, 0.0, None).unwrap();
    let hit = p.findings().events().iter().find(|e| e.code == "holder-hits-stock").unwrap();
    assert_eq!(hit.line, 10);
    assert!(hit.at.unwrap().z > -2.0 - 1e-9);
    // Not when it goes in only as far as the stickout.
    let text = program("G0 X20. Y20.\nG0 Z1.\nG1 Z-9.9 F200\nG0 Z10.");
    let mut p = player(&text, &Machine::three_axis(), std::slice::from_ref(&held), &stock(), Surface::new(Vec::new()).unwrap(), None, 1.0e9);
    p.seek(usize::MAX, 0.0, None).unwrap();
    assert!(!p.findings().merged().has("holder-hits-stock"));
}

/// A machine with a plate on the spindle reaching 40 mm out in +X from the axis, from the nose
/// down to 5 mm below it, on a 300 × 200 table.
fn plate_machine() -> MachineModel {
    MachineModel::from_json(
        r#"{"format": "solidcraft.machine_model", "version": 1, "name": "test mill",
            "spindle": {"nose_gap": 0, "nose_max_above_table": 120},
            "table": {"size": [300, 200, 15], "cut_allowance": 0.1},
            "parts": [{"name": "the side plate", "code": "side-plate", "mount": "spindle",
                       "shape": {"box": {"min": [10, -10, -5], "max": [40, 10, 50]}}}]}"#,
    )
    .unwrap()
}

#[test]
fn a_machine_part_and_the_cutter_against_a_clamp() {
    // The stock clamped at (100, 80) on the table, a clamp 20 high on its +X end.
    let hold = Workholding::from_json(
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "clamps", "stock_at": [100, 80],
            "clamps": [{"center": [140, 100], "size": [20, 20], "height": 20}]}"#,
    )
    .unwrap();
    let model = plate_machine();
    let scene = build(&model.table, &hold, &stock()).unwrap();
    // Work zero at the stock's top front-left: the table top 10 below it.
    close3(scene.work_zero_table, Vec3::new(100.0, 80.0, 10.0), 1e-12);
    let clamp = scene.obstacles.iter().find(|o| o.kind == ObstacleKind::Clamp).unwrap();
    close3(clamp.min, Vec3::new(30.0, 10.0, 0.0), 1e-12);
    close3(clamp.max, Vec3::new(50.0, 30.0, 20.0), 1e-12);
    // 20 mm of stickout and no holder: the nose is 20 above the tip, the plate from 15 above it.
    let tool = Tool { stickout: Some(20.0), ..Tool::new(1, ToolKind::FlatEndmill, 6.0, 20.0) };
    let setup = |m: Option<MachineModel>| PlaybackSetup {
        tools: vec![tool.clone()],
        model: m.map(Arc::new),
        scene: Some(Arc::new(scene.clone())),
        deliberate: Vec::new(),
    };
    // Along Y at X5, Z1: the plate reaches X45, over the clamp, its bottom at Z16 below its top (20).
    let text = program("G0 X5. Y-20.\nG0 Z1.\nG1 Y60. F800\nG0 Z60.");
    let mut p = player(
        &text,
        &Machine::three_axis(),
        std::slice::from_ref(&tool),
        &stock(),
        Surface::new(Vec::new()).unwrap(),
        Some(setup(Some(plate_machine()))),
        1.0e9,
    );
    p.seek(usize::MAX, 0.0, None).unwrap();
    let hit = p.findings().events().iter().find(|e| e.code == "side-plate-hits-clamp").unwrap();
    assert_eq!((hit.line, hit.severity), (10, Severity::Error));
    // ... and nothing without the machine model.
    let mut p =
        player(&text, &Machine::three_axis(), std::slice::from_ref(&tool), &stock(), Surface::new(Vec::new()).unwrap(), Some(setup(None)), 1.0e9);
    p.seek(usize::MAX, 0.0, None).unwrap();
    assert!(!p.findings().merged().has("side-plate-hits-clamp"));
    // The cutter through the clamp, and into the table past its allowance.
    let text = program("G0 X40. Y-20.\nG0 Z5.\nG1 Y60. F800\nG0 Z60.\nG0 X-20. Y-20.\nG1 Z-10.5 F200\nG0 Z60.");
    let mut p =
        player(&text, &Machine::three_axis(), std::slice::from_ref(&tool), &stock(), Surface::new(Vec::new()).unwrap(), Some(setup(None)), 1.0e9);
    p.seek(usize::MAX, 0.0, None).unwrap();
    let merged = p.findings().merged();
    assert!(merged.has("cutter-hits-clamp"), "{:?}", merged.list());
    // The nose above the spindle's travel at Z60: 60 + 20 + 10 = 90 above the table... within 120;
    // at Z95 it is 125.
    assert!(!merged.has("z-travel"));
    let text = program("G0 X-20. Y-20.\nG0 Z95.");
    let mut p = player(
        &text,
        &Machine::three_axis(),
        std::slice::from_ref(&tool),
        &stock(),
        Surface::new(Vec::new()).unwrap(),
        Some(setup(Some(plate_machine()))),
        1.0e9,
    );
    p.seek(usize::MAX, 0.0, None).unwrap();
    assert!(p.findings().merged().has("z-travel"));
}

#[test]
fn operations_tool_changes_and_seeking_by_them() {
    let part = block(Vec3::new(0.0, 0.0, -10.0), Vec3::new(40.0, 40.0, -5.0));
    let text = "G21\nT2 M6\nS10000 M3\nG0 X10. Y10. Z5.\nG1 Z-1. F100\nG0 Z5.\nT7 M6\nS10000 M3\nG0 X30. Y30. Z5.\nG1 Z-1. F100\nG0 Z10.\n";
    let tools = [Tool { number: 2, ..flat(2.0) }, Tool { number: 7, ..flat(10.0) }];
    let plan = Arc::new(plan_sim(text, &Machine::three_axis(), &tools, &stock(), &SimParams { tolerance: 0.01, resolution: Some(0.2) }).unwrap());
    // By tool change when no operations are given.
    let t = Timeline::build(&plan, text, &rates(), DwellUnit::Seconds, None).unwrap();
    assert_eq!(t.ops.iter().map(|o| o.name.as_str()).collect::<Vec<_>>(), ["T2", "T7"]);
    assert_eq!(t.tool_changes.iter().map(|c| (c.at_move, c.tool)).collect::<Vec<_>>(), [(0, 2), (3, 7)]);
    // By the job's operations, one per block.
    let names = ["drill a".to_string(), "drill b".to_string()];
    let owners = [0, 0, 0, 1, 1, 1];
    let t = Timeline::build(&plan, text, &rates(), DwellUnit::Seconds, Some(OpsGiven { names: &names, owners: &owners })).unwrap();
    assert_eq!(t.ops.iter().map(|o| (o.name.as_str(), o.first, o.end)).collect::<Vec<_>>(), [("drill a", 0, 3), ("drill b", 3, 6)]);
    assert_eq!(t.at_line(8), Some(2));
    assert_eq!(t.at_line(10), Some(4));
    let mut p = Playback::new(
        plan,
        Arc::new(t),
        Arc::new(part),
        PlaybackSetup { tools: tools.to_vec(), ..PlaybackSetup::default() },
        Budget::default(),
        true,
    );
    let second = p.timeline().ops[1].first;
    p.seek(second, 0.0, None).unwrap();
    assert_eq!(p.tool(p.plan().step(second).unwrap().slot).map(|t| t.number), Some(7));
    close3(p.pose_now().unwrap().tip, Vec3::new(10.0, 10.0, 5.0), 1e-9);
    p.seek(second, 1.0, None).unwrap();
    close3(p.pose_now().unwrap().tip, Vec3::new(30.0, 30.0, 5.0), 1e-9);
}

#[test]
fn tools_are_drawn_from_their_profile() {
    let held = Tool {
        stickout: Some(30.0),
        holder: vec![HolderSection { diameter: 34.0, length: 4.0 }, HolderSection { diameter: 34.0, length: 18.0 }],
        ..Tool::new(1, ToolKind::BallEndmill, 6.0, 12.0)
    };
    let m = tool_meshes(&held, 24).unwrap();
    let top = |mesh: &solidcraft_geom::Mesh| mesh.positions.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
    let low = |mesh: &solidcraft_geom::Mesh| mesh.positions.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    close(low(&m.cutting), 0.0, 1e-12);
    close(top(&m.cutting), 12.0, 1e-12);
    close(top(&m.shank), 30.0, 1e-12);
    close(top(&m.holder), 52.0, 1e-12);
    // The ball's points are on its sphere.
    assert!(
        m.cutting.positions.iter().filter(|p| p.z < 3.0 - 1e-9).all(|p| ((p.x * p.x + p.y * p.y + (p.z - 3.0).powi(2)).sqrt() - 3.0).abs() < 1e-9)
    );
    let thread = Tool { thread_pitch: Some(1.5), tooth_height: Some(0.8), ..Tool::new(2, ToolKind::ThreadMill, 8.0, 9.0) };
    assert!(
        tool_meshes(&thread, 16).unwrap().cutting.positions.len()
            > tool_meshes(&Tool::new(3, ToolKind::FlatEndmill, 8.0, 9.0), 16).unwrap().cutting.positions.len()
    );
}
