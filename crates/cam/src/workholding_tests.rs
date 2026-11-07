//! Tests of workholding: a stock in a vise on parallels (where work zero lands, the jaws and the
//! bars), a turned vise, the setup findings, clamps on the stock, the table machined itself, a
//! rotary unit, and hostile files refused.

use solidcore_record::Vec3;

use crate::foundation_tests::*;
use crate::machine_model::{ObstacleKind, Table};
use crate::sim::StockShape;
use crate::workholding::*;

fn table() -> Table {
    Table { size: [302.0, 175.0, 15.0], corner_radius: 5.0, cut_allowance: 0.1, holes: Vec::new() }
}

/// A coupon 101.6 × 62 × 9.525 with work zero on its top, centred.
fn coupon() -> StockShape {
    StockShape::Box { min: Vec3::new(-50.8, -31.0, -9.525), max: Vec3::new(50.8, 31.0, 0.0) }
}

fn vise(turns: u32, parallels: f64) -> Workholding {
    Workholding::from_json(&format!(
        r#"{{"format": "solidcraft.workholding", "version": 1, "holding": "vise",
            "vise": {{"model": {{"length": 175, "width": 64, "height": 64, "floor": 32, "fixed_jaw": 35.6, "moving_jaw": 50, "max_opening": 85}},
                     "center": [227, 87.5], "quarter_turns": {turns}}},
            "parallels": {{"height": {parallels}, "thickness": 3.175, "length": 152.4}}}}"#
    ))
    .unwrap()
}

#[test]
fn a_coupon_in_the_vise_on_parallels() {
    let s = build(&table(), &vise(0, 25.4), &coupon()).unwrap();
    assert!(s.findings.is_empty(), "{:?}", s.findings);
    // Centred on the vise at (227, 87.5), against the fixed jaw at the back; its top is the
    // floor 32 + the 25.4 parallels + 9.525, 2.925 above the jaws.
    let face = 87.5 + 175.0 / 2.0 - 35.6;
    close3(s.work_zero_table, Vec3::new(227.0, face - 31.0, 66.925), 1e-9);
    let jaw = s.obstacles.iter().find(|o| o.name == "the vise's fixed jaw").unwrap();
    close(jaw.max.z, -2.925, 1e-9);
    close(s.stock_above_jaws.unwrap(), 2.925, 1e-9);
    close(jaw.min.y, 31.0, 1e-9);
    let moving = s.obstacles.iter().find(|o| o.name == "the vise's moving jaw").unwrap();
    close(moving.max.y, -31.0, 1e-9);
    // Two bars along the jaws, 152.4 long, under the stock's Y faces.
    let bars: Vec<_> = s.obstacles.iter().filter(|o| o.kind == ObstacleKind::Parallels).collect();
    assert_eq!(bars.len(), 2);
    for b in &bars {
        close(b.max.x - b.min.x, 152.4, 1e-9);
        close(b.max.y - b.min.y, 3.175, 1e-9);
        close(b.max.z, -9.525, 1e-9);
    }
    assert!(bars.iter().any(|b| (b.max.y - 31.0).abs() < 1e-9) && bars.iter().any(|b| (b.min.y + 31.0).abs() < 1e-9));
    // The table is under it all: its top 66.925 below work zero.
    close(s.table.corner.z, -66.925, 1e-9);
    // Tall parallels leave nothing in the jaws.
    assert!(build(&table(), &vise(0, 30.0), &coupon()).unwrap().findings.iter().any(|f| f.code == "shallow-grip"));
    assert!(build(&table(), &vise(0, 33.0), &coupon()).unwrap().findings.iter().any(|f| f.code == "stock-not-gripped"));
}

#[test]
fn a_turned_vise_grips_the_x_faces() {
    let s = build(&table(), &vise(1, 25.4), &coupon()).unwrap();
    let jaw = s.obstacles.iter().find(|o| o.name == "the vise's fixed jaw").unwrap();
    close(jaw.max.x, -50.8, 1e-9);
    // 101.6 across the jaws is past the 85 mm opening.
    assert!(s.findings.iter().any(|f| f.code == "vise-too-narrow"));
}

#[test]
fn clamps_sit_on_the_stock() {
    let hold = Workholding::from_json(
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "clamps", "stock_at": [30, 56.5],
            "parallels": {"height": 10, "thickness": 3.175, "length": 152.4},
            "clamps": [{"center": [10, 10], "size": [40, 20], "height": 20}, {"center": [20, 87.5], "size": [40, 20], "height": 20}]}"#,
    )
    .unwrap();
    let s = build(&table(), &hold, &coupon()).unwrap();
    let c = s.obstacles.iter().find(|o| o.name == "clamp 1").unwrap();
    close(c.min.z, 0.0, 1e-9);
    close(c.max.z, 20.0, 1e-9);
    assert!(s.obstacles.iter().any(|o| o.kind == ObstacleKind::Parallels));
    let codes: Vec<&str> = s.findings.iter().map(|f| f.code.as_str()).collect();
    assert_eq!(codes, ["clamp-off-stock"], "clamp 1 misses the stock, clamp 2 is on it");
    close(s.work_zero_table.z, 10.0 + 9.525, 1e-9);
    close(s.work_zero_table.x, 30.0 + 50.8, 1e-9);
    // Over the table's edge, and no clamps at all.
    let off = Workholding::from_json(r#"{"format": "solidcraft.workholding", "version": 1, "holding": "clamps", "stock_at": [250, 10]}"#).unwrap();
    let codes: Vec<String> = build(&table(), &off, &coupon()).unwrap().findings.into_iter().map(|f| f.code).collect();
    assert_eq!(codes, ["no-clamps", "stock-off-table"]);
}

#[test]
fn the_table_itself_and_a_rotary_unit() {
    let hold =
        Workholding::from_json(r#"{"format": "solidcraft.workholding", "version": 1, "holding": "table", "table_zero": [302, 0, 0]}"#).unwrap();
    let plate = StockShape::Box { min: Vec3::new(-302.0, 0.0, -15.0), max: Vec3::new(0.0, 175.0, 0.0) };
    let s = build(&table(), &hold, &plate).unwrap();
    assert!(s.obstacles.is_empty() && s.findings.is_empty());
    close3(s.table.corner, Vec3::new(-302.0, 0.0, 0.0), 1e-12);
    let hold = Workholding::from_json(
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "rotary",
            "rotary": {"axis": [60, 70], "headstock": {"min": [10, 20, 0], "max": [80, 100, 110]},
                       "chuck": {"diameter": 80, "length": 40, "face_x": 120, "side": "minus_x"},
                       "tailstock": {"min": [250, 40, 0], "max": [290, 80, 90]}}}"#,
    )
    .unwrap();
    let bar = StockShape::Cylinder { x_min: 0.0, x_max: 100.0, radius: 15.0 };
    let s = build(&table(), &hold, &bar).unwrap();
    // A chuck on the -X side: work zero on the A axis at the chuck's face (the bar's low end
    // there), the chuck reaching back from it towards -X.
    close3(s.work_zero_table, Vec3::new(120.0, 60.0, 70.0), 1e-12);
    let chuck = s.obstacles.iter().find(|o| o.name == "the rotary unit's chuck").unwrap();
    close3(chuck.max, Vec3::new(0.0, 40.0, 40.0), 1e-12);
    assert_eq!(s.obstacles.iter().filter(|o| o.kind == ObstacleKind::Rotary).count(), 3);
    // A rotary unit holds a bar the A axis runs through (a square one too), not a plate below
    // the axis.
    assert!(build(&table(), &hold, &coupon()).is_err());
    let square = StockShape::Box { min: Vec3::new(0.0, -15.0, -15.0), max: Vec3::new(100.0, 15.0, 15.0) };
    close3(build(&table(), &hold, &square).unwrap().work_zero_table, Vec3::new(120.0, 60.0, 70.0), 1e-12);
}

/// The usual layout: the unit and its chuck at the table's +X end, the bar standing out towards
/// -X with its +X end in the jaws; work zero on the A axis.
#[test]
fn a_rotary_unit_holds_the_bar_from_the_plus_x_side_by_default() {
    let hold = Workholding::from_json(
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "rotary",
            "rotary": {"axis": [57, 65], "headstock": {"min": [222, 12, 0], "max": [300, 102, 160]},
                       "chuck": {"diameter": 80, "length": 45, "face_x": 180,
                                 "jaws": {"radial": 12, "axial": 15}}}}"#,
    )
    .unwrap();
    // The bar: 30 mm across, its free end at X -50, 20 mm of it in the jaws (to X 20).
    let bar = StockShape::Cylinder { x_min: -50.0, x_max: 20.0, radius: 15.0 };
    let s = build(&table(), &hold, &bar).unwrap();
    // The bar's +X end at the chuck's face: work zero 20 mm towards -X from it.
    close3(s.work_zero_table, Vec3::new(160.0, 57.0, 65.0), 1e-12);
    // The chuck's body reaches from its face towards +X, into the unit.
    let chuck = s.obstacles.iter().find(|o| o.name == "the rotary unit's chuck").unwrap();
    close3(chuck.min, Vec3::new(20.0, -40.0, -40.0), 1e-12);
    close3(chuck.max, Vec3::new(65.0, 40.0, 40.0), 1e-12);
    // The jaws stand 15 mm out of the face towards the work, 12 mm beyond the bar, never inside it.
    let jaws: Vec<_> = s.obstacles.iter().filter(|o| o.name == "the rotary unit's jaws").collect();
    assert!(!jaws.is_empty());
    for j in &jaws {
        assert!(j.min.x >= 5.0 - 1e-9 && j.max.x <= 20.0 + 1e-9, "{j:?}");
        assert!(j.min.y >= -27.0 - 1e-9 && j.max.y <= 27.0 + 1e-9 && j.min.z >= -27.0 - 1e-9 && j.max.z <= 27.0 + 1e-9, "{j:?}");
        let inside_bar = j.min.y < 15.0 && j.max.y > -15.0 && j.min.z < 15.0 && j.max.z > -15.0;
        assert!(!inside_bar, "{j:?}");
    }
    // Above the bar in the jaws: a tool coming down there meets a jaw.
    let top = Vec3::new(12.0, 0.0, 20.0);
    let holds = |j: &&crate::machine_model::Obstacle| {
        j.min.x <= top.x && top.x <= j.max.x && j.min.y <= top.y && top.y <= j.max.y && j.min.z <= top.z && top.z <= j.max.z
    };
    assert!(jaws.iter().any(holds));
    assert!(s.findings.is_empty(), "{:?}", s.findings);
    // Work zero given: a bar that does not reach the jaws is not held.
    let mut short = hold.clone();
    if let Some(r) = short.rotary.as_mut() {
        r.zero_x = Some(100.0);
    }
    let s = build(&table(), &short, &bar).unwrap();
    assert_eq!(s.findings.iter().map(|f| f.code.as_str()).collect::<Vec<_>>(), ["stock-not-gripped"]);
    // ... and one with 2 mm in the jaws is held only just.
    if let Some(r) = short.rotary.as_mut() {
        r.zero_x = Some(147.0);
    }
    let s = build(&table(), &short, &bar).unwrap();
    assert_eq!(s.findings.iter().map(|f| f.code.as_str()).collect::<Vec<_>>(), ["shallow-grip"]);
    // Jaws of no size are refused, and so is a side that is not a side.
    for bad in [r#""jaws": {"radial": 0, "axial": 15}"#, r#""side": "up""#] {
        let text = format!(
            r#"{{"format": "solidcraft.workholding", "version": 1, "holding": "rotary",
                "rotary": {{"axis": [57, 65], "headstock": {{"min": [222, 12, 0], "max": [300, 102, 160]}},
                           "chuck": {{"diameter": 80, "length": 45, "face_x": 180, {bad}}}}}}}"#
        );
        assert!(Workholding::from_json(&text).is_err(), "{bad}");
    }
}

#[test]
fn hostile_workholding_is_refused() {
    let bad = [
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "vise"}"#,
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "clamps"}"#,
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "table"}"#,
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "rotary"}"#,
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "glue"}"#,
        r#"{"format": "other", "version": 1, "holding": "table", "table_zero": [0, 0, 0]}"#,
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "table", "table_zero": [0, 0, 1e300]}"#,
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "clamps", "stock_at": [0, 0], "clamps": [{"center": [0, 0], "size": [-1, 1], "height": 1}]}"#,
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "vise", "vise": {"model": {"length": 10, "width": 64, "height": 64, "floor": 32, "fixed_jaw": 6, "moving_jaw": 5, "max_opening": 85}, "center": [0, 0]}}"#,
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "vise", "vise": {"model": {"length": 175, "width": 64, "height": 30, "floor": 32, "fixed_jaw": 6, "moving_jaw": 5, "max_opening": 85}, "center": [0, 0]}}"#,
        r#"{"format": "solidcraft.workholding", "version": 1, "holding": "table", "table_zero": [0, 0, 0], "surprise": 1}"#,
        "not json",
    ];
    for (k, text) in bad.iter().enumerate() {
        assert!(Workholding::from_json(text).is_err(), "case {k} was taken");
    }
    let many = format!(
        r#"{{"format": "solidcraft.workholding", "version": 1, "holding": "clamps", "stock_at": [0, 0], "clamps": [{}]}}"#,
        vec![r#"{"center": [0, 0], "size": [1, 1], "height": 1}"#; 101].join(",")
    );
    assert!(Workholding::from_json(&many).is_err());
}
