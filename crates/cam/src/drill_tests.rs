//! Tests of drilling: the drill's point, peck depths and counts, the moves of every cycle, the
//! order holes are drilled in, travel between holes, the gouge check of a drill's real shape,
//! and hostile settings refused without panics.

use std::f64::consts::TAU;

use solidcore_record::{Vec2, Vec3};
use solidcraft_geom::prelude::*;

use crate::drill::{
    Cycle, DEFAULT_POINT_ANGLE_DEG, DrillParams, MAX_HOLES, check_approach, check_drilling_tool, drill, hole_bottoms, order_holes, peck_depths,
    tip_length, tool_gouges,
};
use crate::{CamError, Frame, Heights, Move, Surface, Tool, ToolKind, Toolpath, Toolpath5, verify};

fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}

fn drill_tool(d: f64) -> Tool {
    Tool::new(5, ToolKind::Drill, d, 40.0)
}

fn heights(top: f64, bottom: f64) -> Heights {
    Heights { clearance: 15.0, retract: 2.0, top, bottom }
}

fn params(cycle: Cycle) -> DrillParams {
    DrillParams { cycle, heights: heights(0.0, -10.0), feed: 300.0, to_full_diameter: false, stay_at_retract: false, part_top: None }
}

fn close(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} is not within {tol} of {b}");
}

/// The Z of every move, `R`/`F`/`D` for rapid, feed and dwell.
fn trace(path: &Toolpath) -> Vec<(char, f64)> {
    path.moves
        .iter()
        .map(|m| match *m {
            Move::Rapid { to } => ('R', to.z),
            Move::Feed { to, .. } | Move::Arc { to, .. } | Move::PlaneArc { to, .. } => ('F', to.z),
            Move::Dwell { seconds } => ('D', seconds),
        })
        .collect()
}

fn assert_trace(path: &Toolpath, want: &[(char, f64)]) {
    let got = trace(path);
    let same = got.len() == want.len() && got.iter().zip(want).all(|(a, b)| a.0 == b.0 && (a.1 - b.1).abs() <= 1e-9);
    assert!(same, "got {got:?}, want {want:?}");
}

#[test]
fn a_drills_point_is_its_radius_over_the_tangent_of_half_its_angle() {
    let d = drill_tool(6.0);
    close(tip_length(&d), 3.0 / (DEFAULT_POINT_ANGLE_DEG.to_radians() * 0.5).tan(), 1e-12);
    close(tip_length(&d), 1.8026, 1e-4);
    close(tip_length(&Tool { point_angle_deg: Some(90.0), ..d.clone() }), 3.0, 1e-12);
    close(tip_length(&Tool { point_angle_deg: Some(180.0), ..d.clone() }), 0.0, 1e-9);
    assert_eq!(tip_length(&Tool::new(1, ToolKind::FlatEndmill, 6.0, 20.0)), 0.0);
    // The angle is kept only when given, and checked.
    let json = serde_json::to_value(&d).unwrap();
    assert!(json.get("point_angle_deg").is_none(), "{json}");
    let read: Tool =
        serde_json::from_value(serde_json::json!({"number": 2, "kind": "drill", "diameter": 3, "flute_length": 30, "point_angle_deg": 135})).unwrap();
    assert_eq!(read.point_angle_deg, Some(135.0));
    for bad in [0.0, -5.0, 181.0, f64::NAN, f64::INFINITY] {
        assert!(Tool { point_angle_deg: Some(bad), ..d.clone() }.validate().is_err(), "{bad}");
    }
}

#[test]
fn pecks_are_a_peck_apart_from_the_top_and_the_last_ends_at_the_bottom() {
    assert_eq!(peck_depths(0.0, -10.0, Some(3.0)).unwrap(), [-3.0, -6.0, -9.0, -10.0]);
    assert_eq!(peck_depths(0.0, -10.0, Some(2.5)).unwrap(), [-2.5, -5.0, -7.5, -10.0], "no sliver of a last peck");
    assert_eq!(peck_depths(0.0, -10.0, Some(12.0)).unwrap(), [-10.0]);
    // A last peck of a hair (0.002 mm) is merged into the one before; a tenth of a peck or
    // more is kept.
    assert_eq!(peck_depths(0.0, -9.002, Some(3.0)).unwrap(), [-3.0, -6.0, -9.002]);
    assert_eq!(peck_depths(0.0, -3.002, Some(3.0)).unwrap(), [-3.002]);
    assert_eq!(peck_depths(0.0, -9.3, Some(3.0)).unwrap(), [-3.0, -6.0, -9.0, -9.3]);
    assert_eq!(peck_depths(5.0, 1.0, None).unwrap(), [1.0]);
    assert!(peck_depths(0.0, 0.0, Some(1.0)).is_err());
    assert!(peck_depths(0.0, 1.0, Some(1.0)).is_err());
    assert!(matches!(peck_depths(0.0, -10.0, Some(1e-9)), Err(CamError::TooLarge(_))));
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(peck_depths(0.0, -10.0, Some(bad)).is_err(), "{bad}");
    }
}

#[test]
fn a_peck_cycle_retracts_fully_after_every_peck_and_comes_back_just_above_the_depth() {
    let p = params(Cycle::Peck { peck: 3.0, clearance: 0.5 });
    let path = drill(&[v(4.0, 5.0)], &drill_tool(6.0), &p).unwrap();
    let want = [
        ('R', 15.0),
        ('R', 2.0),
        ('F', -3.0),
        ('R', 2.0),
        ('R', -2.5),
        ('F', -6.0),
        ('R', 2.0),
        ('R', -5.5),
        ('F', -9.0),
        ('R', 2.0),
        ('R', -8.5),
        ('F', -10.0),
        ('R', 2.0),
        ('R', 15.0),
    ];
    assert_trace(&path, &want);
    // Every move is over the hole.
    assert!(path.moves.iter().filter_map(Move::target).all(|t| v(t.x, t.y).dist(v(4.0, 5.0)) == 0.0));
    let feeds = path.moves.iter().filter(|m| matches!(m, Move::Feed { feed, .. } if *feed == 300.0)).count();
    assert_eq!(feeds, 4, "four pecks");
    // Cut length: from R to the bottom once, plus each return's 0.5 mm of air.
    close(path.stats().cut_length, 12.0 + 3.0 * 0.5, 1e-9);
}

#[test]
fn chip_breaking_backs_off_inside_the_hole_and_dwell_pauses_at_depth() {
    let path = drill(&[v(0.0, 0.0)], &drill_tool(6.0), &params(Cycle::ChipBreak { peck: 4.0, back_off: 0.3 })).unwrap();
    assert_trace(&path, &[('R', 15.0), ('R', 2.0), ('F', -4.0), ('R', -3.7), ('F', -8.0), ('R', -7.7), ('F', -10.0), ('R', 2.0), ('R', 15.0)]);
    let path = drill(&[v(0.0, 0.0)], &drill_tool(6.0), &params(Cycle::Dwell { seconds: 0.75 })).unwrap();
    assert_trace(&path, &[('R', 15.0), ('R', 2.0), ('F', -10.0), ('D', 0.75), ('R', 2.0), ('R', 15.0)]);
    let path = drill(&[v(0.0, 0.0)], &drill_tool(6.0), &params(Cycle::Simple)).unwrap();
    assert_trace(&path, &[('R', 15.0), ('R', 2.0), ('F', -10.0), ('R', 2.0), ('R', 15.0)]);
}

#[test]
fn to_full_diameter_takes_the_tip_deeper_by_the_point_and_a_flat_end_mill_plunges_to_the_bottom() {
    let p = DrillParams { to_full_diameter: true, ..params(Cycle::Simple) };
    let path = drill(&[v(0.0, 0.0)], &drill_tool(6.0), &p).unwrap();
    close(path.stats().lowest_z.unwrap(), -10.0 - 1.8026, 1e-4);
    let mill = Tool::new(1, ToolKind::FlatEndmill, 6.0, 20.0);
    let path = drill(&[v(0.0, 0.0)], &mill, &p).unwrap();
    assert_eq!(path.stats().lowest_z, Some(-10.0));
    assert!(check_drilling_tool(&mill).is_ok());
    for kind in [ToolKind::BallEndmill, ToolKind::BullNose] {
        let t = Tool { corner_radius: Some(1.0), ..Tool::new(3, kind, 6.0, 20.0) };
        let e = drill(&[v(0.0, 0.0)], &t, &p).unwrap_err();
        assert!(e.to_string().contains("drill, or a flat end mill"), "{e}");
    }
}

#[test]
fn holes_are_drilled_nearest_first_from_the_lowest_x_and_repeats_once() {
    let pts = [v(10.0, 0.0), v(0.0, 0.0), v(30.0, 0.0), v(11.0, 0.0), v(0.0, 0.0), v(20.0, 0.0), v(0.0, 5.0)];
    let (order, repeats) = order_holes(&pts).unwrap();
    assert_eq!(order, [v(0.0, 0.0), v(0.0, 5.0), v(10.0, 0.0), v(11.0, 0.0), v(20.0, 0.0), v(30.0, 0.0)]);
    assert_eq!(repeats, 1);
    // Equally near: the first given wins, so the order is the same every time.
    let (order, _) = order_holes(&[v(0.0, 0.0), v(0.0, 10.0), v(10.0, 0.0), v(0.0, -10.0)]).unwrap();
    assert_eq!(order, [v(0.0, -10.0), v(0.0, 0.0), v(0.0, 10.0), v(10.0, 0.0)]);
    // A path drills each once, in that order.
    let path = drill(&pts, &drill_tool(3.0), &params(Cycle::Simple)).unwrap();
    let bottoms = hole_bottoms(&path);
    assert_eq!(bottoms.len(), 6);
    assert_eq!(bottoms.iter().map(|b| v(b.x, b.y)).collect::<Vec<_>>(), order_holes(&pts).unwrap().0);
    assert!(bottoms.iter().all(|b| b.z == -10.0));
}

#[test]
fn between_holes_the_tool_rises_to_clearance_unless_the_r_height_clears_everything() {
    let pts = [v(0.0, 0.0), v(10.0, 0.0), v(20.0, 0.0)];
    let sideways_at = |path: &Toolpath| -> Vec<f64> {
        let mut out = Vec::new();
        let mut at: Option<Vec3> = None;
        for m in &path.moves {
            if let (Move::Rapid { to }, Some(from)) = (*m, at)
                && v(from.x, from.y).dist(v(to.x, to.y)) > 1e-9
            {
                out.push(to.z);
            }
            at = m.target().or(at);
        }
        out
    };
    let path = drill(&pts, &drill_tool(3.0), &params(Cycle::Simple)).unwrap();
    assert_eq!(sideways_at(&path), [15.0, 15.0]);
    // Staying at R needs to know R is above the part and fixtures.
    for top in [None, Some(2.0), Some(5.0), Some(f64::NAN)] {
        let p = DrillParams { stay_at_retract: true, part_top: top, ..params(Cycle::Simple) };
        let e = drill(&pts, &drill_tool(3.0), &p).unwrap_err();
        assert!(e.to_string().contains("stay_at_retract"), "{top:?}: {e}");
    }
    let p = DrillParams { stay_at_retract: true, part_top: Some(1.0), ..params(Cycle::Simple) };
    let path = drill(&pts, &drill_tool(3.0), &p).unwrap();
    assert_eq!(sideways_at(&path), [2.0, 2.0]);
    let t = trace(&path);
    assert_eq!(t.first(), Some(&('R', 15.0)), "the path starts at the clearance height");
    assert_eq!(t.last(), Some(&('R', 15.0)), "and ends there");
    verify(&path, &heights(0.0, -10.0)).unwrap();
}

/// A funnel: the cone a drill's point leaves, apex at the origin, opening up to radius `rim`.
fn funnel(point_angle_deg: f64, rim: f64) -> Surface {
    let n = 256;
    let slope = 1.0 / (point_angle_deg.to_radians() * 0.5).tan();
    let ring = |i: usize| {
        let a = TAU * i as f64 / n as f64;
        Vec3::new(rim * a.cos(), rim * a.sin(), rim * slope)
    };
    // The cone, then a flat collar round it.
    let mut tris = Vec::new();
    for i in 0..n {
        let (p, q) = (ring(i), ring(i + 1));
        tris.push([Vec3::ZERO, p, q]);
        let (pp, qq) = (Vec3::new(p.x * 3.0, p.y * 3.0, p.z), Vec3::new(q.x * 3.0, q.y * 3.0, q.z));
        tris.push([p, pp, qq]);
        tris.push([p, qq, q]);
    }
    Surface::new(tris).unwrap()
}

fn plunge(z: f64) -> Toolpath5 {
    let path = Toolpath {
        moves: vec![
            Move::Rapid { to: Vec3::new(0.0, 0.0, 20.0) },
            Move::Rapid { to: Vec3::new(0.0, 0.0, 15.0) },
            Move::Feed { to: Vec3::new(0.0, 0.0, z), feed: 200.0 },
        ],
    };
    Toolpath5::from_frame(&path, &Frame::IDENTITY, 0.01).unwrap()
}

#[test]
fn the_gouge_check_holds_a_drills_point_against_the_part_not_a_flat_bottom() {
    let surface = funnel(118.0, 4.0);
    let d = drill_tool(6.0);
    // The drill's tip in the funnel's apex: its point fits the cone.
    assert!(tool_gouges(&surface, &d, &plunge(0.0), 0.01, 0.5).unwrap().is_empty());
    // A flat end mill there is deep in the cone: the deepest point of the cone inside it is
    // 1.125 mm from its surface (where the cone, rising 0.6 mm a millimetre out, is as far
    // above its end as in from its side), though it would have to back out the 1.8 mm of the
    // cone's height at its rim along its axis.
    let mill = Tool::new(1, ToolKind::FlatEndmill, 6.0, 40.0);
    let hits = tool_gouges(&surface, &mill, &plunge(0.0), 0.01, 0.5).unwrap();
    assert!(hits.first().is_some_and(|h| h.distance > 1.1 && h.distance < 1.15), "{hits:?}");
    // The drill 0.3 mm too deep gouges, found on the move and at the tip's pose: square to the
    // cone's wall, less deep than the 0.3 mm along its axis.
    let hits = tool_gouges(&surface, &d, &plunge(-0.3), 0.01, 0.5).unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    let h = hits.first().unwrap();
    assert_eq!(h.index, 2);
    assert!((0.15..=0.3).contains(&h.distance), "{h:?}");
    assert!(h.pose.tip.z < -0.2, "the tip, not a raised step: {h:?}");
    // A blunter drill (140°) does not fit a 118° cone: its shoulder meets the wall.
    let blunt = Tool { point_angle_deg: Some(140.0), ..d.clone() };
    assert!(!tool_gouges(&surface, &blunt, &plunge(0.0), 0.01, 0.5).unwrap().is_empty());
    // A sharper one (90°) fits.
    let sharp = Tool { point_angle_deg: Some(90.0), ..d };
    assert!(tool_gouges(&surface, &sharp, &plunge(0.0), 0.01, 0.5).unwrap().is_empty());
}

#[test]
fn hostile_drilling_settings_are_refused_without_panics() {
    let d = drill_tool(6.0);
    let ok = params(Cycle::Peck { peck: 2.0, clearance: 0.5 });
    assert!(drill(&[v(0.0, 0.0)], &d, &ok).is_ok());
    let cases: Vec<(Vec<Vec2>, Tool, DrillParams)> = vec![
        (vec![], d.clone(), ok),
        (vec![v(f64::NAN, 0.0)], d.clone(), ok),
        (vec![v(1e300, 0.0)], d.clone(), ok),
        (vec![v(0.0, 0.0)], Tool { diameter: -1.0, ..d.clone() }, ok),
        (vec![v(0.0, 0.0)], Tool { flute_length: 5.0, ..d.clone() }, ok),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { feed: 0.0, ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { feed: f64::NAN, ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { heights: heights(0.0, 0.0), ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { heights: Heights { clearance: 1.0, retract: 2.0, top: 0.0, bottom: -1.0 }, ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { heights: heights(0.0, f64::NEG_INFINITY), ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { cycle: Cycle::Peck { peck: 0.0, clearance: 0.5 }, ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { cycle: Cycle::Peck { peck: 1e-12, clearance: 0.5 }, ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { cycle: Cycle::Peck { peck: 1.0, clearance: 0.0 }, ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { cycle: Cycle::Peck { peck: f64::NAN, clearance: 0.5 }, ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { cycle: Cycle::ChipBreak { peck: 1.0, back_off: -1.0 }, ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { cycle: Cycle::Dwell { seconds: -1.0 }, ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { cycle: Cycle::Dwell { seconds: f64::INFINITY }, ..ok }),
        (vec![v(0.0, 0.0)], d.clone(), DrillParams { stay_at_retract: true, part_top: Some(f64::INFINITY), ..ok }),
        (vec![v(0.0, 0.0); MAX_HOLES + 1], d.clone(), ok),
    ];
    for (pts, tool, p) in &cases {
        assert!(drill(pts, tool, p).is_err(), "{pts:?} {tool:?} {p:?}");
    }
    // The gouge check refuses a bad tool or tolerance rather than looping or panicking.
    let surface = funnel(118.0, 4.0);
    assert!(tool_gouges(&surface, &Tool { diameter: f64::NAN, ..d.clone() }, &plunge(0.0), 0.01, 0.5).is_err());
    assert!(tool_gouges(&surface, &d, &plunge(0.0), f64::NAN, 0.5).is_err());
    assert!(tool_gouges(&surface, &d, &plunge(0.0), 0.01, 0.0).is_err());
    // A tiny tolerance caps the steps instead of making millions of checks.
    assert!(tool_gouges(&surface, &d, &plunge(0.0), 1e-9, 0.5).is_ok());
}

/// Flat squares at heights: each `[x0, y0, x1, y1, z]`.
fn flats(squares: &[[f64; 5]]) -> Surface {
    let mut tris = Vec::new();
    for &[x0, y0, x1, y1, z] in squares {
        let (a, b, c, d) = (Vec3::new(x0, y0, z), Vec3::new(x1, y0, z), Vec3::new(x1, y1, z), Vec3::new(x0, y1, z));
        tris.push([a, b, c]);
        tris.push([a, c, d]);
    }
    Surface::new(tris).unwrap()
}

#[test]
fn many_gouging_pecks_are_gouge_checked_in_linear_time() {
    // 2400 holes chip-broken 10 mm into a part that shows no holes: every feed gouges, and each
    // of the point's steps finds a hit on it to merge with the others'.
    let pts: Vec<Vec2> = (0..2400).map(|i| v(f64::from(i % 40) * 8.0, f64::from(i / 40) * 8.0)).collect();
    let p = params(Cycle::ChipBreak { peck: 0.5, back_off: 0.3 });
    let path = drill(&pts, &drill_tool(6.0), &p).unwrap();
    let feeds = path.moves.iter().filter(|m| matches!(m, Move::Feed { .. })).count();
    assert_eq!(feeds, 2400 * 20);
    let poses = Toolpath5::from_frame(&path, &Frame::IDENTITY, 0.01).unwrap();
    let surface = flats(&[[-10.0, -10.0, 330.0, 490.0, 0.0]]);
    let start = std::time::Instant::now();
    let hits = tool_gouges(&surface, &drill_tool(6.0), &poses, 0.01, 1.5).unwrap();
    let took = start.elapsed();
    assert_eq!(hits.len(), feeds, "one hit per feed, every feed below the top");
    assert!(hits.windows(2).all(|w| w[0].index < w[1].index), "in move order");
    // The deepest at the narrowest step, just above the tip at the bottom.
    let deepest = hits.iter().map(|h| h.distance).fold(0.0, f64::max);
    close(deepest, 10.0 - tip_length(&drill_tool(6.0)) / 64.0, 0.02);
    assert!(took.as_secs_f64() < 20.0, "the merge of the steps' hits is not quadratic: {took:?}");
}

#[test]
fn the_tool_never_rapids_down_into_the_part_round_a_hole() {
    let d = drill_tool(6.0);
    // The part's top at Z 0 round the hole: the rapid down to R 2 is in the air.
    assert!(check_approach(&flats(&[[-50.0, -50.0, 50.0, 50.0, 0.0]]), &d, &[v(0.0, 0.0)], 2.0, 0.01).is_ok());
    // A boss up to Z 10 round a hole whose `top` was given as the plate's, 0: the drill would
    // rapid from clearance down to R through 8 mm of it.
    let boss = flats(&[[-50.0, -50.0, 50.0, 50.0, 0.0], [10.0, -10.0, 30.0, 10.0, 10.0]]);
    assert!(check_approach(&boss, &d, &[v(0.0, 0.0)], 2.0, 0.01).is_ok(), "a hole away from the boss");
    let e = check_approach(&boss, &d, &[v(0.0, 0.0), v(20.0, 0.0)], 2.0, 0.01).unwrap_err();
    assert!(e.to_string().contains("(20, 0)") && e.to_string().contains("Z 10") && e.to_string().contains("retract"), "{e}");
    assert!(check_approach(&boss, &d, &[v(20.0, 0.0)], 12.0, 0.01).is_ok(), "R above the boss");
    assert!(check_approach(&boss, &d, &[v(20.0, 0.0)], 9.995, 0.01).is_ok(), "within the tolerance");
    // A wall beside the hole is not over it; a rim within a tenth of the radius past the
    // hole's edge is.
    assert!(check_approach(&flats(&[[4.0, -50.0, 50.0, 50.0, 10.0]]), &d, &[v(0.0, 0.0)], 2.0, 0.01).is_ok());
    assert!(check_approach(&flats(&[[3.2, -50.0, 50.0, 50.0, 10.0]]), &d, &[v(0.0, 0.0)], 2.0, 0.01).is_err());
    // Hostile values are refused, not looped over or panicked on.
    let plane = flats(&[[-50.0, -50.0, 50.0, 50.0, 0.0]]);
    assert!(check_approach(&plane, &d, &[v(0.0, 0.0)], f64::NAN, 0.01).is_err());
    assert!(check_approach(&plane, &d, &[v(0.0, 0.0)], 2.0, f64::NAN).is_err());
    assert!(check_approach(&plane, &d, &[v(f64::NAN, 0.0)], 2.0, 0.01).is_err());
    assert!(check_approach(&plane, &Tool { diameter: 0.0, ..d.clone() }, &[v(0.0, 0.0)], 2.0, 0.01).is_err());
    assert!(check_approach(&plane, &Tool { diameter: 1000.0, ..d.clone() }, &[v(0.0, 0.0)], 2.0, 1.0).is_ok(), "the widest tool");
    assert!(check_approach(&plane, &d, &vec![v(0.0, 0.0); MAX_HOLES + 1], 2.0, 0.01).is_err());
}
