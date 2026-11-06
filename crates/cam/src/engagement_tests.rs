//! Tests of the engagement measurement: a slot, a side cut of a known width, nothing to cut, and
//! hostile input.

use solidcore_record::{Loop2, Region2, Seg2, Vec2, Vec3};

use crate::adaptive::MoveRole;
use crate::engagement::{Material, engagement};
use crate::path::{Move, Toolpath};

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Region2 {
    let p = [Vec2::new(x0, y0), Vec2::new(x1, y0), Vec2::new(x1, y1), Vec2::new(x0, y1)];
    let segs = (0..4).map(|i| Seg2::Line { a: p[i], b: p[(i + 1) % 4] }).collect();
    Region2 { outer: Loop2 { segs }, holes: vec![] }
}

/// Along y = `y` at depth `z` from x -10 to 110 and back up.
fn pass(moves: &mut Vec<Move>, y: f64, z: f64) {
    moves.push(Move::Rapid { to: Vec3::new(-10.0, y, 10.0) });
    moves.push(Move::Feed { to: Vec3::new(-10.0, y, z), feed: 200.0 });
    moves.push(Move::Feed { to: Vec3::new(110.0, y, z), feed: 1000.0 });
    moves.push(Move::Rapid { to: Vec3::new(110.0, y, 10.0) });
}

fn bar() -> Material {
    Material { regions: vec![rect(0.0, 0.0, 100.0, 20.0)], top: 0.0 }
}

#[test]
fn a_slot_is_a_full_slot() {
    let mut moves = Vec::new();
    pass(&mut moves, 10.0, -1.0);
    let e = engagement(&Toolpath { moves }, None, 3.0, &bar(), 0.2).unwrap();
    assert!((e.max_deg - 180.0).abs() < 1e-9, "{e:?}");
    assert!(e.full_slot_share > 0.5, "{e:?}");
    assert!(e.samples > 100, "{e:?}");
}

/// The second pass, `w` beside the first, engages acos(1 - w/r) of the circumference.
#[test]
fn a_side_cut_engages_as_its_width_says() {
    let (r, w) = (3.0, 0.6);
    let mut moves = Vec::new();
    pass(&mut moves, 10.0, -1.0);
    let first = moves.len();
    pass(&mut moves, 10.0 + w, -1.0);
    // The slot is marked as an entry, so the figures outside entries are the side cut's.
    let roles: Vec<MoveRole> = (0..moves.len()).map(|i| if i < first { MoveRole::Entry } else { MoveRole::Cut }).collect();
    let e = engagement(&Toolpath { moves }, Some(&roles), r, &bar(), 0.1).unwrap();
    let expect = (1.0f64 - w / r).acos().to_degrees();
    assert!((e.max_deg - expect).abs() < 1.5, "{} against {expect}: {e:?}", e.max_deg);
    assert!((e.mean_deg - expect).abs() < 1.5, "{e:?}");
    assert_eq!(e.full_slot_share, 0.0, "{e:?}");
    assert!((e.max_entry_deg - 180.0).abs() < 1e-9, "{e:?}");
}

#[test]
fn nothing_to_cut_measures_nothing() {
    let mut moves = Vec::new();
    // Above the material, then beside it.
    pass(&mut moves, 10.0, 0.5);
    pass(&mut moves, 40.0, -1.0);
    let e = engagement(&Toolpath { moves }, None, 3.0, &bar(), 0.25).unwrap();
    assert_eq!(e.max_deg, 0.0, "{e:?}");
    assert_eq!(engagement(&Toolpath::default(), None, 3.0, &bar(), 0.25).unwrap().samples, 0);
}

#[test]
fn hostile_input_is_refused() {
    let mut moves = Vec::new();
    pass(&mut moves, 10.0, -1.0);
    let path = Toolpath { moves };
    for r in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e9] {
        assert!(engagement(&path, None, r, &bar(), 0.25).is_err(), "{r}");
    }
    for s in [0.0, -1.0, f64::NAN] {
        assert!(engagement(&path, None, 3.0, &bar(), s).is_err(), "{s}");
    }
    assert!(engagement(&path, None, 3.0, &Material { top: f64::NAN, ..bar() }, 0.25).is_err());
    let bad = Toolpath { moves: vec![Move::Rapid { to: Vec3::new(0.0, 0.0, 10.0) }, Move::Feed { to: Vec3::new(f64::NAN, 0.0, 0.0), feed: 100.0 }] };
    assert!(engagement(&bad, None, 3.0, &bar(), 0.25).is_err());
    // Roles shorter than the path are taken as clearing.
    assert!(engagement(&path, Some(&[MoveRole::Entry]), 3.0, &bar(), 0.25).is_ok());
}
