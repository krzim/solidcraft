//! Tests of simultaneous 5-axis surface finishing and swarf cutting.

use std::f64::consts::PI;

use solidcore_record::{Vec2, Vec3};
use solidcraft_geom::prelude::*;

use crate::collision::gouges;
use crate::foundation_tests::*;
use crate::strategies5::*;
use crate::*;

fn deg(d: f64) -> f64 {
    d * PI / 180.0
}

fn tool(kind: ToolKind, diameter: f64) -> Tool {
    let mut t = Tool::new(1, kind, diameter, 20.0);
    if kind == ToolKind::BullNose {
        t.corner_radius = Some(1.0);
    }
    t
}

fn params(clearance: f64) -> Finish5Params {
    Finish5Params {
        feeds: Feeds { cut: 1000.0, plunge: 300.0 },
        tolerance: 0.01,
        max_axis_step: deg(3.0),
        stock_to_leave: 0.0,
        retract: 2.0,
        clearance,
        max_tilt: None,
        tilt_reference: None,
    }
}

fn raster(half: f64) -> DrivePattern {
    DrivePattern::Parallel { angle: 0.0, stepover: 3.0, step: 1.0, boundary: Some([Vec2::new(-half, -half), Vec2::new(half, half)]), direction: None }
}

/// The poses the tool cuts at (every feed move's end: the plunges end on the surface).
fn cutting(path: &Toolpath5) -> Vec<Pose> {
    path.moves.iter().filter_map(|m| if let Move5::Feed { to, .. } = *m { Some(to) } else { None }).collect()
}

#[test]
fn a_ball_normal_to_a_sphere() {
    let (rs, r) = (20.0, 3.0);
    let s = sphere(Vec3::ZERO, rs, 64);
    let chord = rs * (1.0 - (PI / 64.0).cos()) * 2.0;
    let p = params(30.0);
    for lead in [0.0, 10.0] {
        let control = AxisControl::Normal { lead: deg(lead), tilt: 0.0 };
        let path = surface_finish(&s, &tool(ToolKind::BallEndmill, 6.0), &raster(10.0), &control, &p).unwrap();
        verify5(&path, p.max_axis_step).unwrap();
        let poses = cutting(&path);
        assert!(poses.len() > 100);
        for pose in poses {
            // The ball's centre is r out from the sphere along the radial line through it.
            let centre = pose.up(r);
            let d = centre.len();
            assert!(d <= rs + r + 1e-6 && d >= rs + r - chord, "{d}");
            let radial = centre.normalized().unwrap();
            let angle = pose.axis.cross(radial).len().atan2(pose.axis.dot(radial));
            close(angle, deg(lead), 0.01);
        }
        // Between poses the tip strays from the surface by at most about the tolerance.
        let worst = gouges(&s, &Cutter::ball(r).unwrap(), &path, 1e-6, 0.25).unwrap().iter().map(|h| h.distance).fold(0.0, f64::max);
        assert!(worst <= p.tolerance * 1.25, "{worst}");
    }
    // A tilt sideways leans the same amount from the normal.
    let control = AxisControl::Normal { lead: 0.0, tilt: deg(-15.0) };
    let path = surface_finish(&s, &tool(ToolKind::BallEndmill, 6.0), &raster(8.0), &control, &p).unwrap();
    for pose in cutting(&path) {
        let radial = pose.up(r).normalized().unwrap();
        close(pose.axis.cross(radial).len().atan2(pose.axis.dot(radial)), deg(15.0), 0.01);
    }
}

#[test]
fn lead_leans_towards_the_feed_and_tilt_to_the_left() {
    // On a flat floor the raster's first pass runs along +X, the next back along −X.
    let s = plane(50.0, 0.0);
    let p = params(10.0);
    let pattern = DrivePattern::Parallel {
        angle: 0.0,
        stepover: 5.0,
        step: 2.0,
        boundary: Some([Vec2::new(-10.0, -10.0), Vec2::new(10.0, -5.0)]),
        direction: None,
    };
    let lead = surface_finish(&s, &tool(ToolKind::BallEndmill, 6.0), &pattern, &AxisControl::Normal { lead: deg(10.0), tilt: 0.0 }, &p).unwrap();
    let tilt = surface_finish(&s, &tool(ToolKind::BallEndmill, 6.0), &pattern, &AxisControl::Normal { lead: 0.0, tilt: deg(10.0) }, &p).unwrap();
    let (s10, c10) = (deg(10.0).sin(), deg(10.0).cos());
    for (path, first, second) in
        [(&lead, Vec3::new(s10, 0.0, c10), Vec3::new(-s10, 0.0, c10)), (&tilt, Vec3::new(0.0, s10, c10), Vec3::new(0.0, -s10, c10))]
    {
        let poses = cutting(path);
        let on = |y: f64| poses.iter().filter(move |q| (q.up(3.0).y - y).abs() < 1e-9).collect::<Vec<_>>();
        assert!(!on(-10.0).is_empty() && !on(-5.0).is_empty());
        for q in on(-10.0) {
            close3(q.axis, first, 1e-9);
        }
        for q in on(-5.0) {
            close3(q.axis, second, 1e-9);
        }
    }
}

#[test]
fn toward_and_away_from_a_point_and_the_tilt_limit() {
    let s = plane(50.0, 0.0);
    let r = 3.0;
    let ball = tool(ToolKind::BallEndmill, 6.0);
    let p = params(10.0);
    let above = Vec3::new(0.0, 0.0, 100.0);
    let below = Vec3::new(5.0, 0.0, -60.0);
    for (control, want) in [
        (AxisControl::TowardPoint { point: above }, Box::new(move |cc: Vec3| (above - cc).normalized().unwrap()) as Box<dyn Fn(Vec3) -> Vec3>),
        (AxisControl::AwayFromPoint { point: below }, Box::new(move |cc: Vec3| (cc - below).normalized().unwrap())),
        (
            AxisControl::AwayFromLine { point: Vec3::new(0.0, 0.0, -80.0), direction: Vec3::new(0.0, 1.0, 0.0) },
            Box::new(|cc: Vec3| (cc - Vec3::new(0.0, cc.y, -80.0)).normalized().unwrap()),
        ),
        (
            AxisControl::TowardLine { point: Vec3::new(0.0, 0.0, 80.0), direction: Vec3::new(1.0, 0.0, 0.0) },
            Box::new(|cc: Vec3| (Vec3::new(cc.x, 0.0, 80.0) - cc).normalized().unwrap()),
        ),
    ] {
        let path = surface_finish(&s, &ball, &raster(20.0), &control, &p).unwrap();
        for q in cutting(&path) {
            // The ball touches the floor straight under its centre.
            let centre = q.up(r);
            close(centre.z, r, 1e-9);
            close3(q.axis, want(centre - Vec3::new(0.0, 0.0, r)), 1e-9);
        }
    }
    // A fixed axis leaning 40° is held to 20° from +Z.
    let leaning = AxisControl::Fixed { axis: Vec3::new(deg(40.0).sin(), 0.0, deg(40.0).cos()) };
    let path = surface_finish(&s, &ball, &raster(20.0), &leaning, &Finish5Params { max_tilt: Some(deg(20.0)), ..p }).unwrap();
    for q in path.poses() {
        close3(q.axis, Vec3::new(deg(20.0).sin(), 0.0, deg(20.0).cos()), 1e-9);
    }
    // Measured from another reference.
    let reference = Vec3::new(0.0, deg(10.0).sin(), deg(10.0).cos());
    let path = surface_finish(
        &s,
        &ball,
        &raster(20.0),
        &AxisControl::TowardPoint { point: Vec3::new(0.0, 0.0, 30.0) },
        &Finish5Params { max_tilt: Some(deg(5.0)), tilt_reference: Some(reference), ..p },
    )
    .unwrap();
    for q in path.poses() {
        assert!(q.axis.cross(reference).len().atan2(q.axis.dot(reference)) <= deg(5.0) + 1e-9);
    }
}

#[test]
fn flat_and_bull_ends_sit_flush_on_a_slope() {
    let th = 0.3;
    let s = slope(50.0, th);
    let normal = Vec3::new(-th.sin(), 0.0, th.cos());
    let p = params(30.0);
    for kind in [ToolKind::FlatEndmill, ToolKind::BullNose] {
        let path = surface_finish(&s, &tool(kind, 6.0), &raster(20.0), &AxisControl::Normal { lead: 0.0, tilt: 0.0 }, &p).unwrap();
        let poses = cutting(&path);
        assert!(poses.len() > 50);
        for q in poses {
            close3(q.axis, normal, 1e-9);
            // The flat bottom is flush: the tip lies on the plane.
            close(q.tip.dot(normal), 0.0, p.tolerance);
        }
    }
    // Stock is left along the axis.
    let path = surface_finish(
        &s,
        &tool(ToolKind::FlatEndmill, 6.0),
        &raster(20.0),
        &AxisControl::Normal { lead: 0.0, tilt: 0.0 },
        &Finish5Params { stock_to_leave: 0.5, ..p },
    )
    .unwrap();
    for q in cutting(&path) {
        close(q.tip.dot(normal), 0.5, 1e-6);
    }
}

#[test]
fn curves_on_a_sphere_and_the_links() {
    let (rs, r) = (20.0, 3.0);
    let s = sphere(Vec3::ZERO, rs, 64);
    let chord = rs * (1.0 - (PI / 64.0).cos()) * 2.0;
    // Two arcs of meridians from the top.
    let arc = |az: f64| {
        (0..=20)
            .map(|i| {
                let phi = deg(60.0) * i as f64 / 20.0;
                Vec3::new(phi.sin() * az.cos(), phi.sin() * az.sin(), phi.cos()) * rs
            })
            .collect::<Vec<_>>()
    };
    let pattern = DrivePattern::Along { curves: vec![arc(0.0), arc(deg(90.0))], step: Some(1.0) };
    let p = params(30.0);
    let path = surface_finish(&s, &tool(ToolKind::BallEndmill, 6.0), &pattern, &AxisControl::AwayFromPoint { point: Vec3::ZERO }, &p).unwrap();
    verify5(&path, p.max_axis_step).unwrap();
    for q in cutting(&path) {
        let d = q.up(r).len();
        assert!(d <= rs + r + 1e-6 && d >= rs + r - chord, "{d}");
    }
    // Two passes: each starts from the clearance plane and ends back there.
    let rapids_at_clearance = path.moves.iter().filter(|m| matches!(m, Move5::Rapid { to } if (to.tip.z - 30.0).abs() < 1e-9)).count();
    assert_eq!(rapids_at_clearance, 4);
    assert!(matches!(path.moves.first(), Some(Move5::Rapid { to }) if to.tip.z >= 30.0));
    // The last stretch in is fed at the plunge feed, along the axis.
    let plunges: Vec<(Pose, Pose)> = path
        .moves
        .windows(2)
        .filter_map(|w| match w {
            [Move5::Rapid { to: a }, Move5::Feed { to: b, feed }] if *feed == 300.0 => Some((*a, *b)),
            _ => None,
        })
        .collect();
    assert_eq!(plunges.len(), 2);
    for (a, b) in plunges {
        close3(a.tip, b.up(p.retract), 1e-9);
    }
}

#[test]
fn swarf_on_a_planar_wall() {
    let bottom: Vec<Vec3> = (0..9).map(|i| Vec3::new(-20.0 + 5.0 * i as f64, 0.0, 0.0)).collect();
    let top: Vec<Vec3> = bottom.iter().map(|b| *b + Vec3::new(0.0, 0.0, 15.0)).collect();
    let flat = tool(ToolKind::FlatEndmill, 10.0);
    let p = SwarfParams {
        feeds: Feeds { cut: 800.0, plunge: 200.0 },
        stock_to_leave: 0.0,
        bottom_offset: 0.5,
        levels: 1,
        max_axis_step: deg(2.0),
        retract: 2.0,
        clearance: 30.0,
    };
    for (side, y) in [(WallSide::Left, 5.0), (WallSide::Right, -5.0)] {
        let cut = swarf(&bottom, &top, &flat, side, &p).unwrap();
        assert!(cut.max_deviation <= 1e-9, "{cut:?}");
        assert!(cut.overcut.iter().all(|o| *o <= 1e-9));
        let poses = cutting(&cut.path);
        assert_eq!(poses.len(), 9);
        for q in poses {
            close(q.tip.y, y, 1e-12);
            close(q.tip.z, -0.5, 1e-12);
            close3(q.axis, Vec3::new(0.0, 0.0, 1.0), 1e-12);
        }
    }
    // Depth passes from the top down; stock moves the tool off the wall.
    let cut = swarf(&bottom, &top, &flat, WallSide::Left, &SwarfParams { levels: 3, stock_to_leave: 0.25, bottom_offset: 0.0, ..p }).unwrap();
    let heights: Vec<f64> = cutting(&cut.path).iter().map(|q| q.tip.z).collect();
    assert_eq!(heights.len(), 27);
    for (k, z) in heights.iter().enumerate() {
        close(*z, [10.0, 5.0, 0.0][k / 9], 1e-9);
    }
    assert!(cutting(&cut.path).iter().all(|q| (q.tip.y - 5.25).abs() < 1e-12));
    // A leaning planar wall: its rulings lean the same way, so it is developable too.
    let lean: Vec<Vec3> = bottom.iter().map(|b| *b + Vec3::new(0.0, 6.0, 15.0)).collect();
    let cut = swarf(&bottom, &lean, &flat, WallSide::Right, &p).unwrap();
    assert!(cut.max_deviation <= 1e-9);
    let n = Vec3::new(0.0, 15.0, -6.0).normalized().unwrap();
    for q in cutting(&cut.path) {
        // The axis is 5 from the wall's plane, on its right (−Y, below the lean).
        close(q.tip.dot(n), -5.0, 1e-9);
    }
}

#[test]
fn swarf_on_a_twisted_wall() {
    // The top rail is the bottom one turned by β about Z and raised 15: a twisted ruled wall.
    let beta = deg(30.0);
    let bottom: Vec<Vec3> = (0..21).map(|i| Vec3::new(-20.0 + 2.0 * i as f64, 0.0, 0.0)).collect();
    let top: Vec<Vec3> = bottom.iter().map(|b| Vec3::new(b.x * beta.cos(), b.x * beta.sin(), 15.0)).collect();
    let flat = tool(ToolKind::FlatEndmill, 10.0);
    let p = SwarfParams {
        feeds: Feeds { cut: 800.0, plunge: 200.0 },
        stock_to_leave: 0.0,
        bottom_offset: 0.0,
        levels: 1,
        max_axis_step: deg(2.0),
        retract: 2.0,
        clearance: 30.0,
    };
    let cut = swarf(&bottom, &top, &flat, WallSide::Left, &p).unwrap();
    verify5(&cut.path, p.max_axis_step).unwrap();
    assert!(cut.max_deviation > 0.1, "{}", cut.max_deviation);
    // At the middle ruling (vertical, at x = 0) the wall's normal is taken halfway between the
    // rails' directions, at β/2 to each, so the cylinder (axis vertical, 5 off the ruling along
    // that normal) is R·cos(β/2) from both rails: it cuts in by R·(1 − cos(β/2)).
    close(cut.overcut[10], 5.0 * (1.0 - (beta / 2.0).cos()), 1e-9);
    assert!(cut.max_overcut >= cut.overcut[10]);
    // Untwisted, nothing.
    let straight: Vec<Vec3> = bottom.iter().map(|b| *b + Vec3::new(0.0, 0.0, 15.0)).collect();
    assert!(swarf(&bottom, &straight, &flat, WallSide::Left, &p).unwrap().max_deviation <= 1e-9);
}

#[test]
fn hostile_strategy_inputs_are_refused() {
    let s = plane(50.0, 0.0);
    let ball = tool(ToolKind::BallEndmill, 6.0);
    let normal = AxisControl::Normal { lead: 0.0, tilt: 0.0 };
    let p = params(10.0);
    assert!(surface_finish(&s, &ball, &raster(20.0), &normal, &p).is_ok());
    let empty = Surface::new(vec![]).unwrap();
    assert!(surface_finish(&empty, &ball, &raster(20.0), &normal, &p).is_err());
    for bad in [
        Finish5Params { tolerance: f64::NAN, ..p },
        Finish5Params { tolerance: 0.0, ..p },
        Finish5Params { max_axis_step: 0.0, ..p },
        Finish5Params { retract: f64::INFINITY, ..p },
        Finish5Params { clearance: 1.0, ..p },
        Finish5Params { stock_to_leave: -1.0, ..p },
        Finish5Params { max_tilt: Some(f64::NAN), ..p },
        Finish5Params { tilt_reference: Some(Vec3::ZERO), ..p },
        Finish5Params { feeds: Feeds { cut: 0.0, plunge: 1.0 }, ..p },
    ] {
        assert!(surface_finish(&s, &ball, &raster(20.0), &normal, &bad).is_err(), "{bad:?}");
    }
    for control in [
        AxisControl::Normal { lead: f64::NAN, tilt: 0.0 },
        AxisControl::Normal { lead: deg(90.0), tilt: 0.0 },
        AxisControl::Fixed { axis: Vec3::ZERO },
        AxisControl::TowardPoint { point: Vec3::new(f64::INFINITY, 0.0, 0.0) },
        AxisControl::TowardLine { point: Vec3::ZERO, direction: Vec3::ZERO },
    ] {
        assert!(surface_finish(&s, &ball, &raster(20.0), &control, &p).is_err(), "{control:?}");
    }
    for pattern in [
        DrivePattern::Parallel { angle: 0.0, stepover: 0.0, step: 1.0, boundary: None, direction: None },
        DrivePattern::Parallel { angle: f64::NAN, stepover: 1.0, step: 1.0, boundary: None, direction: None },
        DrivePattern::Parallel { angle: 0.0, stepover: 1.0, step: -1.0, boundary: None, direction: None },
        DrivePattern::Parallel { angle: 0.0, stepover: 1.0, step: 1.0, boundary: Some([Vec2::new(1.0, 1.0), Vec2::new(0.0, 0.0)]), direction: None },
        DrivePattern::Parallel { angle: 0.0, stepover: 1.0, step: 1.0, boundary: None, direction: Some(Vec3::new(f64::NAN, 0.0, 0.0)) },
        DrivePattern::Parallel { angle: 0.0, stepover: 1e-6, step: 1e-6, boundary: None, direction: None },
        DrivePattern::Along { curves: vec![vec![Vec3::ZERO, Vec3::new(f64::NAN, 0.0, 0.0)]], step: None },
        DrivePattern::Along { curves: vec![vec![Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)]], step: Some(0.0) },
        // Nowhere on the surface.
        DrivePattern::Parallel {
            angle: 0.0,
            stepover: 1.0,
            step: 1.0,
            boundary: Some([Vec2::new(100.0, 100.0), Vec2::new(110.0, 110.0)]),
            direction: None,
        },
        DrivePattern::Along { curves: vec![], step: None },
    ] {
        assert!(surface_finish(&s, &ball, &pattern, &normal, &p).is_err(), "{pattern:?}");
    }
    // An axis that points into the floor reaches nothing.
    assert!(surface_finish(&s, &ball, &raster(20.0), &AxisControl::Fixed { axis: Vec3::new(0.0, 0.0, -1.0) }, &p).is_err());
    assert!(surface_finish(&s, &Tool::new(1, ToolKind::Drill, 6.0, 20.0), &raster(20.0), &normal, &p).is_err());

    let bottom = [Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0)];
    let top = [Vec3::new(0.0, 0.0, 10.0), Vec3::new(10.0, 0.0, 10.0)];
    let flat = tool(ToolKind::FlatEndmill, 10.0);
    let sp = SwarfParams {
        feeds: Feeds { cut: 800.0, plunge: 200.0 },
        stock_to_leave: 0.0,
        bottom_offset: 0.0,
        levels: 1,
        max_axis_step: deg(2.0),
        retract: 2.0,
        clearance: 30.0,
    };
    assert!(swarf(&bottom, &top, &flat, WallSide::Left, &sp).is_ok());
    assert!(swarf(&bottom, &top[..1], &flat, WallSide::Left, &sp).is_err(), "mismatched rails");
    assert!(swarf(&bottom[..1], &top[..1], &flat, WallSide::Left, &sp).is_err(), "one ruling");
    assert!(swarf(&[], &[], &flat, WallSide::Left, &sp).is_err());
    assert!(swarf(&bottom, &[top[0], Vec3::new(f64::NAN, 0.0, 0.0)], &flat, WallSide::Left, &sp).is_err());
    assert!(swarf(&bottom, &[top[0], Vec3::new(10.0, 0.0, 25.0)], &flat, WallSide::Left, &sp).is_err(), "longer than the flutes");
    assert!(swarf(&bottom, &[top[0], bottom[1]], &flat, WallSide::Left, &sp).is_err(), "a ruling without length");
    // The rails run along the rulings: no wall.
    assert!(
        swarf(&[Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0)], &[Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 11.0)], &flat, WallSide::Left, &sp).is_err()
    );
    for bad in [
        SwarfParams { levels: 0, ..sp },
        SwarfParams { max_axis_step: f64::NAN, ..sp },
        SwarfParams { bottom_offset: f64::NAN, ..sp },
        SwarfParams { stock_to_leave: f64::INFINITY, ..sp },
        SwarfParams { clearance: 5.0, ..sp },
        SwarfParams { retract: 0.0, ..sp },
    ] {
        assert!(swarf(&bottom, &top, &flat, WallSide::Left, &bad).is_err(), "{bad:?}");
    }
    // Upside down, the passes cannot be left along the axis.
    assert!(matches!(swarf(&top, &bottom, &flat, WallSide::Left, &SwarfParams { clearance: 30.0, ..sp }), Err(CamError::Unsupported(_))));

    let json = serde_json::to_string(&AxisControl::Normal { lead: 0.1, tilt: -0.2 }).unwrap();
    assert!(json.contains("\"type\":\"normal\""));
    assert_eq!(serde_json::from_str::<AxisControl>(&json).unwrap(), AxisControl::Normal { lead: 0.1, tilt: -0.2 });
    let pattern = raster(5.0);
    assert_eq!(serde_json::from_str::<DrivePattern>(&serde_json::to_string(&pattern).unwrap()).unwrap(), pattern);
    let back: SwarfParams = serde_json::from_str(&serde_json::to_string(&sp).unwrap()).unwrap();
    assert_eq!(back, sp);
}

/// The path with its rapids (but the first move) made cutting moves, so that the checks of
/// cutting moves look at the links too.
fn rapids_as_feeds(path: &Toolpath5) -> Toolpath5 {
    let moves = path
        .moves
        .iter()
        .enumerate()
        .map(|(i, m)| if let (Move5::Rapid { to }, true) = (*m, i > 0) { Move5::Feed { to, feed: 1.0 } } else { *m })
        .collect();
    Toolpath5 { moves }
}

#[test]
fn a_rib_between_samples_is_not_cut_through() {
    // A rib 0.3 wide and 5 high on a floor, and a 1 mm flat end sampled every 3 mm: no sample
    // and no midpoint lands on the rib, but the moves between them must not cut through it.
    let mut tris = plane(20.0, 0.0).triangles().to_vec();
    tris.extend_from_slice(block(Vec3::new(0.6, -20.0, 0.0), Vec3::new(0.9, 20.0, 5.0)).triangles());
    let s = Surface::new(tris).unwrap();
    let flat = Tool::new(1, ToolKind::FlatEndmill, 1.0, 20.0);
    let pattern = DrivePattern::Parallel {
        angle: 0.0,
        stepover: 5.0,
        step: 3.0,
        boundary: Some([Vec2::new(-3.0, -5.0), Vec2::new(6.0, 5.0)]),
        direction: None,
    };
    let p = params(10.0);
    let path = surface_finish(&s, &flat, &pattern, &AxisControl::Normal { lead: 0.0, tilt: 0.0 }, &p).unwrap();
    verify5(&path, p.max_axis_step).unwrap();
    let cutter = Cutter::flat(0.5).unwrap();
    let g = gouges(&s, &cutter, &rapids_as_feeds(&path), p.tolerance, 0.02).unwrap();
    assert!(g.is_empty(), "{g:?}");
    // The floor either side is still cut, and the top of the rib.
    let poses = cutting(&path);
    assert!(poses.iter().any(|q| q.tip.z.abs() <= 1e-9 && q.tip.x < 0.0));
    assert!(poses.iter().any(|q| q.tip.z.abs() <= 1e-9 && q.tip.x > 2.0));
    assert!(poses.iter().any(|q| (q.tip.z - 5.0).abs() <= 1e-9));
}

#[test]
fn a_pass_at_the_edge_is_left_clear_of_an_overhang() {
    // A slope 3x + 4z = 300 with a plate over its low end at z = 62, cut with the axis leaning
    // back over the part. The tip starts outside the bounds the links keep clear of, but the
    // tool's body lies over the part: it must leave along its axis, not rise straight up
    // through the plate.
    let z = |x: f64| (300.0 - 3.0 * x) / 4.0;
    let v = Vec3::new;
    let tris = vec![
        [v(0.0, -20.0, z(0.0)), v(100.0, -20.0, z(100.0)), v(100.0, 20.0, z(100.0))],
        [v(0.0, -20.0, z(0.0)), v(100.0, 20.0, z(100.0)), v(0.0, 20.0, z(0.0))],
        [v(80.0, -20.0, 62.0), v(80.0, 20.0, 62.0), v(100.0, 20.0, 62.0)],
        [v(80.0, -20.0, 62.0), v(100.0, 20.0, 62.0), v(100.0, -20.0, 62.0)],
    ];
    let s = Surface::new(tris).unwrap();
    let ball = Tool::new(1, ToolKind::BallEndmill, 20.0, 30.0);
    let pattern = DrivePattern::Along { curves: vec![vec![v(99.9, -5.0, z(99.9)), v(99.9, 5.0, z(99.9))]], step: Some(1.0) };
    let p = Finish5Params { max_axis_step: 0.1, clearance: 86.0, ..params(86.0) };
    let path = surface_finish(&s, &ball, &pattern, &AxisControl::Fixed { axis: v(-0.6, 0.0, 0.8) }, &p).unwrap();
    let g = gouges(&s, &ball.cutter().unwrap(), &rapids_as_feeds(&path), p.tolerance, 0.5).unwrap();
    assert!(g.is_empty(), "{g:?}");
    assert!(cutting(&path).len() >= 10);
}

/// The distance from the vertical line through `p` to the segment `a–b` in the XY plane.
fn across_xy(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let flat = |q: Vec3| Vec3::new(q.x, q.y, 0.0);
    flat(p).dist_to_segment(flat(a), flat(b))
}

#[test]
fn swarf_round_a_corner_of_two_planar_faces() {
    // Two vertical planar faces at a right angle. Outside the corner the tool must keep its
    // radius from each face and roll round the corner; inside it, it stops where it touches
    // both. Neither may cut into a face.
    let bottom = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 0.0)];
    let top = bottom.map(|b| b + Vec3::new(0.0, 0.0, 10.0));
    let flat = tool(ToolKind::FlatEndmill, 6.0);
    let p = SwarfParams {
        feeds: Feeds { cut: 800.0, plunge: 200.0 },
        stock_to_leave: 0.0,
        bottom_offset: 0.0,
        levels: 1,
        max_axis_step: 0.1,
        retract: 2.0,
        clearance: 30.0,
    };
    // The chords of the roll round the corner come in by at most this.
    let roll = 3.0 * (1.0 - 0.05f64.cos());
    for side in [WallSide::Right, WallSide::Left] {
        let cut = swarf(&bottom, &top, &flat, side, &p).unwrap();
        verify5(&cut.path, p.max_axis_step).unwrap();
        assert!(cut.max_overcut <= roll + 1e-9, "{side:?}: {cut:?}");
        let poses = cutting(&cut.path);
        for w in poses.windows(2) {
            for k in 0..=100 {
                let q = w[0].tip.lerp(w[1].tip, k as f64 / 100.0);
                for (a, b) in [(bottom[0], bottom[1]), (bottom[1], bottom[2])] {
                    let d = across_xy(q, a, b);
                    assert!(d >= 3.0 - roll - 1e-9, "{side:?}: {q:?} is {d} from a face");
                }
            }
        }
        // It runs the length of both faces.
        let ends = (poses.first().unwrap().tip, poses.last().unwrap().tip);
        close(ends.0.x, 0.0, 1e-9);
        close(ends.1.y, 10.0, 1e-9);
    }
}

#[test]
fn swarf_of_a_densely_sampled_wall_is_quick() {
    // A short wall sampled very finely: every pose is close to most of it, which must not make
    // measuring the deviation take a time growing with the square of the rulings.
    let n = 5000;
    let bottom: Vec<Vec3> = (0..n).map(|i| Vec3::new(10.0 * i as f64 / (n - 1) as f64, 0.0, 0.0)).collect();
    let top: Vec<Vec3> = bottom.iter().map(|b| *b + Vec3::new(0.0, 0.0, 10.0)).collect();
    let flat = tool(ToolKind::FlatEndmill, 6.0);
    let p = SwarfParams {
        feeds: Feeds { cut: 800.0, plunge: 200.0 },
        stock_to_leave: 0.0,
        bottom_offset: 0.0,
        levels: 1,
        max_axis_step: deg(2.0),
        retract: 2.0,
        clearance: 30.0,
    };
    let start = std::time::Instant::now();
    let cut = swarf(&bottom, &top, &flat, WallSide::Left, &p).unwrap();
    let took = start.elapsed().as_secs_f64();
    assert!(cut.max_deviation <= 1e-9, "{}", cut.max_deviation);
    assert!(took < 10.0, "{took} s");
}
