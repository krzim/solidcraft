//! Tests of multi-axis G-code: writing, reading back through the kinematics, inverse-time
//! feeds, indexed operations with their arcs, and the posts.

use std::f64::consts::TAU;

use solidcore_record::{Loop2, Region2, Vec2, Vec3};
use solidcraft_geom::prelude::*;

use crate::foundation_tests::*;
use crate::gcode5::*;
use crate::indexed::{IndexedOperation, frame_for_index};
use crate::kinematics::*;
use crate::readback5::{readback_program, readback5};
use crate::*;

fn deg(d: f64) -> f64 {
    d.to_radians()
}

/// `+Z` at zero, `+Y` at 90°: what `A = θ` on a 4th axis brings up to the tool.
fn radial(theta: f64) -> Vec3 {
    Vec3::new(0.0, theta.sin(), theta.cos())
}

fn tool() -> Tool {
    Tool::new(3, ToolKind::BallEndmill, 6.0, 20.0)
}

fn op5(name: &str, path: Toolpath5) -> Operation5 {
    Operation5 { name: name.into(), tool: tool(), spindle_rpm: 12000.0, coolant: true, path }
}

fn program5(path: Toolpath5) -> Program5 {
    Program5 { name: "part".into(), operations: vec![op5("op", path)], work_offset: None }
}

/// A wrapped helix: three turns round the X axis at radius 20, 30 mm along X.
fn helix() -> Toolpath5 {
    let n = 360;
    let pose = |k: usize| {
        let th = 3.0 * TAU * k as f64 / n as f64;
        Pose::new(Vec3::new(30.0 * k as f64 / n as f64, 0.0, 0.0) + radial(th) * 20.0, radial(th))
    };
    let mut moves = vec![Move5::Rapid { to: Pose::new(pose(0).up(10.0), radial(0.0)) }, Move5::Feed { to: pose(0), feed: 200.0 }];
    moves.extend((1..=n).map(|k| Move5::Feed { to: pose(k), feed: 300.0 }));
    moves.push(Move5::Feed { to: Pose::new(pose(n).up(10.0), radial(3.0 * TAU)), feed: 1000.0 });
    Toolpath5 { moves }
}

/// Radial drops round the X axis every 30°: down from radius 30 to 15, along X, back up,
/// then a rapid turn to the next angle.
fn radial_drops() -> Toolpath5 {
    let mut moves = Vec::new();
    for k in 0..12 {
        let th = deg(30.0 * k as f64);
        let at = |x: f64, r: f64| Pose::new(Vec3::new(x, 0.0, 0.0) + radial(th) * r, radial(th));
        moves.push(Move5::Rapid { to: at(0.0, 30.0) });
        moves.push(Move5::Feed { to: at(0.0, 15.0), feed: 150.0 });
        moves.push(Move5::Feed { to: at(25.0, 15.0), feed: 400.0 });
        moves.push(Move5::Feed { to: at(25.0, 30.0), feed: 1000.0 });
    }
    Toolpath5 { moves }
}

/// A smooth cut with the tool axis wandering up to about 30° from vertical.
fn wander() -> Toolpath5 {
    let pose = |k: usize| {
        let t = k as f64;
        let axis = Vec3::new(0.4 * (t / 10.0).sin(), -0.3 * (t / 13.0).cos(), 1.0).normalized().unwrap();
        Pose::new(Vec3::new(t, 5.0 * (t / 8.0).sin(), 2.0), axis)
    };
    let mut moves = vec![Move5::Rapid { to: Pose::new(pose(0).up(20.0), pose(0).axis) }, Move5::Feed { to: pose(0), feed: 200.0 }];
    moves.extend((1..=40).map(|k| Move5::Feed { to: pose(k), feed: 500.0 }));
    moves.push(Move5::Rapid { to: Pose::new(pose(40).up(20.0), pose(40).axis) });
    Toolpath5 { moves }
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Region2 {
    let v = |x: f64, y: f64| Vec2::new(x, y);
    Region2 { outer: Loop2::polygon(&[v(x0, y0), v(x1, y0), v(x1, y1), v(x0, y1)]), holes: vec![] }
}

fn contour_path() -> Toolpath {
    let params = ContourParams { side: Side::Outside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 };
    let cut =
        Cut { heights: Heights { clearance: 10.0, retract: 2.0, top: 0.0, bottom: -2.0 }, feeds: Feeds { cut: 600.0, plunge: 200.0 }, stepdown: 1.0 };
    let path = contour(&[rect(0.0, 0.0, 20.0, 10.0)], &Tool::new(3, ToolKind::FlatEndmill, 6.0, 20.0), &params, &cut).unwrap();
    assert!(path.moves.iter().any(|m| matches!(m, Move::Arc { .. })));
    path
}

fn lines_with<'a>(text: &'a str, start: &str) -> Vec<&'a str> {
    text.lines().filter(|l| l.starts_with(start)).collect()
}

#[test]
fn three_axis_programs_match_the_3axis_emitter() {
    let post = Post5 { base: Post::rs274(), ..Post5::rs274_5axis() };
    let machine = Machine::three_axis();
    let mut path = contour_path();
    path.moves.push(Move::Dwell { seconds: 0.5 });
    let old = emit(
        &Program {
            name: "part".into(),
            operations: vec![Operation { name: "c".into(), tool: tool(), spindle_rpm: 9000.0, coolant: true, path: path.clone() }],
            work_offset: Some(2),
        },
        &Post::rs274(),
    )
    .unwrap();
    // As an indexed operation (no rotaries to turn) the arcs are kept: the very same text.
    let indexed = IndexedOperation { name: "c".into(), tool: tool(), spindle_rpm: 9000.0, coolant: true, rotary: vec![], path: path.clone() };
    let program = MachineProgram { name: "part".into(), operations: vec![MachineOperation::Indexed(indexed)], work_offset: Some(2) };
    assert_eq!(emit_program(&program, &post, &machine).unwrap(), old);
    // As a continuous one the arcs become lines; the lines-only toolpath gives the same text too.
    let lines_only = Toolpath { moves: path.moves.iter().copied().filter(|m| !matches!(m, Move::Arc { .. })).collect() };
    let old = emit(
        &Program {
            name: "p".into(),
            operations: vec![Operation { name: "l".into(), tool: tool(), spindle_rpm: 9000.0, coolant: false, path: lines_only.clone() }],
            work_offset: None,
        },
        &Post::rs274(),
    )
    .unwrap();
    let p5 = Program5 {
        name: "p".into(),
        operations: vec![Operation5 {
            name: "l".into(),
            tool: tool(),
            spindle_rpm: 9000.0,
            coolant: false,
            path: Toolpath5::from_3axis(&lines_only, 0.001).unwrap(),
        }],
        work_offset: None,
    };
    let new = emit5(&p5, &post, &machine).unwrap();
    assert_eq!(new, old);
    assert_eq!(parse5(&new, &[]).unwrap().len(), parse(&old).unwrap().len());
    assert!(readback5(&new, &p5, &post, &machine).unwrap().within());
}

#[test]
fn a_wrapped_helix_reads_back_in_inverse_time() {
    let machine = Machine::four_axis_a();
    let post = Post5::mach3_4axis();
    let program = program5(helix());
    let text = emit5(&program, &post, &machine).unwrap();
    assert!(text.starts_with("(part)\nG90 G91.1 G94 G17 G21 G40 G49 G80\n"));
    assert!(text.contains("M5\nM6 T3\nG43 H3\nM3 S12000\nM8\n"));
    assert!(text.contains("G93 G1"));
    assert!(text.contains("G94 G1"));
    // The last cut turns nothing, so G94 is back already.
    assert!(text.ends_with("M9\nM5\nM30\n") && !text.ends_with("G94\nM9\nM5\nM30\n"));
    // Three whole turns, never wrapped.
    assert!(text.contains(" A1080"), "{text}");
    let rb = readback5(&text, &program, &post, &machine).unwrap();
    assert!(rb.within(), "{rb:?}");
    close(rb.cut_length, rb.planned_cut_length, 1e-3 * rb.planned_cut_length);
    close(rb.cut_minutes, rb.planned_cut_minutes, 1e-4 * rb.planned_cut_minutes);

    // Every inverse-time F is 1 / the minutes of its piece.
    let pieces: Vec<(f64, f64)> = solve(&machine, &program.operations[0].path, &post.solve_options())
        .unwrap()
        .windows(2)
        .filter_map(|w| match (&w[0], &w[1]) {
            (a, JointMove::Feed { to, minutes, .. }) => a.joints().map(|j| (j.rotary_change(to), *minutes)),
            _ => None,
        })
        .filter(|(turn, _)| *turn > 1e-9)
        .collect();
    let written: Vec<f64> = parse5(&text, &machine.letters())
        .unwrap()
        .into_iter()
        .filter_map(|w| match w {
            JointWords::Feed { f, inverse_time: true, .. } => Some(f),
            _ => None,
        })
        .collect();
    assert_eq!(written.len(), pieces.len());
    for (f, (_, minutes)) in written.iter().zip(&pieces) {
        close(*f, 1.0 / minutes, 1e-4 / minutes);
    }
}

#[test]
fn rotaries_turn_only_at_the_clearance_height() {
    let machine = Machine::four_axis_a();
    for post in [Post5::mach3_4axis(), Post5::linuxcnc_4axis(), Post5::grblhal_4axis()] {
        let program = program5(radial_drops());
        let text = emit5(&program, &post, &machine).unwrap();
        let rb = readback5(&text, &program, &post, &machine).unwrap();
        assert!(rb.within(), "{}: {rb:?}", post.base.name);
        // Every block that turns A stands alone and is written with the tool at Z30 or above.
        let mut z: Option<f64> = None;
        let mut turns = 0;
        for line in text.lines() {
            if let Some(rest) = line.strip_prefix("G0 Z") {
                z = rest.parse().ok();
            } else if line.starts_with("G0 X") || line.starts_with("G1") {
                z = line.split(' ').find_map(|w| w.strip_prefix('Z')).and_then(|v| v.parse().ok()).or(z);
            }
            if line.contains(" A") && line.starts_with("G0") {
                assert!(line.starts_with("G0 A"), "{line}");
                assert!(z.is_some_and(|z| z >= 30.0 - 1e-9), "{}: {line} at Z {z:?}", post.base.name);
                turns += 1;
            }
        }
        assert_eq!(turns, 12, "{}", post.base.name);
    }
    // The grblHAL post writes no M6 or tool length offset and comments with semicolons.
    let text = emit5(&program5(radial_drops()), &Post5::grblhal_4axis(), &machine).unwrap();
    assert!(!text.contains("M6") && !text.contains("G43") && text.starts_with("; part\nG90 G91.1 G94 G17 G21\n"));
}

#[test]
fn indexed_operations_keep_their_arcs() {
    let machine = Machine::four_axis_a();
    let post = Post5::mach3_4axis();
    let path = contour_path();
    let at = |angle: f64, name: &str| {
        MachineOperation::Indexed(IndexedOperation {
            name: name.into(),
            tool: tool(),
            spindle_rpm: 10000.0,
            coolant: false,
            rotary: vec![angle],
            path: path.clone(),
        })
    };
    let program = MachineProgram {
        name: "sides".into(),
        operations: vec![at(deg(90.0), "side 1"), at(deg(180.0), "side 2"), MachineOperation::Continuous(op5("wrap", radial_drops()))],
        work_offset: None,
    };
    let text = emit_program(&program, &post, &machine).unwrap();
    assert!(!lines_with(&text, "G2 ").is_empty() || !lines_with(&text, "G3 ").is_empty());
    // At the start, where the tool's height is not known, up to the program's highest
    // clearance (the wrap's) before the first index; the next index at the operation's
    // clearance, where the last operation left the tool.
    assert!(text.contains("G43 H3\nM3 S10000\nG0 Z30\nG0 A90\n"), "{text}");
    assert!(text.contains("G0 Z10\n(side 2: T3 D6 ball end mill)\nG0 A180\n"), "{text}");
    let rb = readback_program(&text, &program, &post, &machine).unwrap();
    assert!(rb.within(), "{rb:?}");
    close(rb.cut_length, rb.planned_cut_length, 1e-3 * rb.planned_cut_length);
    // The written arcs are the 3-axis toolpath's own: in the index frame, the machine position
    // is the toolpath's coordinates.
    let frame = frame_for_index(&machine, &[deg(90.0)]).unwrap();
    let first_feed = path.moves.iter().find_map(|m| if let Move::Feed { to, .. } = m { Some(*to) } else { None }).unwrap();
    let j = inverse(&machine, &Pose::new(frame.to_world(first_feed), frame.z), None).unwrap();
    close3(j.linear, first_feed, 1e-9);
    // A tilted head cannot run an indexed operation with arcs in the machine's plane.
    let head = Machine::head_table_bc();
    let tilted_head = MachineProgram {
        name: "x".into(),
        operations: vec![MachineOperation::Indexed(IndexedOperation {
            name: "t".into(),
            tool: tool(),
            spindle_rpm: 1000.0,
            coolant: false,
            rotary: vec![deg(30.0), 0.0],
            path: path.clone(),
        })],
        work_offset: None,
    };
    assert!(matches!(emit_program(&tilted_head, &Post5::rs274_5axis(), &head), Err(CamError::Unsupported(_))));
    // With tool centre point control it is written as a continuous operation instead.
    let text = emit_program(&tilted_head, &Post5::tcp_g43_4(), &head).unwrap();
    assert!(lines_with(&text, "G2 ").is_empty() && text.contains("G43.4 H3"));
    assert!(readback_program(&text, &tilted_head, &Post5::tcp_g43_4(), &head).unwrap().within());
}

#[test]
fn five_axis_programs_read_back_on_every_preset() {
    let machines = [
        Machine::table_table_ac_at(Vec3::new(0.0, 0.0, -50.0), Vec3::new(10.0, 5.0, -50.0)).with_work_origin(Vec3::new(20.0, -10.0, 30.0)),
        Machine::table_table_bc(),
        Machine::head_table_bc_at(150.0, Vec3::new(30.0, -20.0, 0.0)),
        Machine::head_head_ca(),
    ];
    for machine in &machines {
        for post in [Post5::rs274_5axis(), Post5::tcp_g43_4()] {
            let program = program5(wander());
            let text = emit5(&program, &post, machine).unwrap();
            let rb = readback5(&text, &program, &post, machine).unwrap();
            assert!(rb.within(), "{} / {}: {rb:?}", machine.name, post.base.name);
            close(rb.cut_length, rb.planned_cut_length, 1e-3 * rb.planned_cut_length);
            if post.tcp.is_some() {
                // The tip in work coordinates: the planned tips themselves.
                assert!(text.contains("G43.4 H3") && text.contains("G49\n"));
                assert!(!text.contains("G93"));
                let planned: Vec<Vec3> = program.operations[0].path.poses().map(|p| p.tip).collect();
                for w in parse5(&text, &machine.letters()).unwrap() {
                    if let JointWords::Feed { linear, .. } = w {
                        assert!(planned.iter().any(|p| p.dist(linear) <= 1e-4), "{linear:?}");
                    }
                }
            } else {
                assert!(text.contains("G53 G0 Z0\nG0 "));
                assert!(text.contains("G93 G1"));
            }
        }
    }
    // A big turn on an offset table is broken into pieces without TCP, one block with it.
    let m = Machine::table_table_ac().with_work_origin(Vec3::new(0.0, 0.0, 20.0));
    let a0 = tool_axis(&m, &[deg(-30.0), 0.0]);
    let a1 = tool_axis(&m, &[deg(-30.0), deg(90.0)]);
    let program = program5(Toolpath5 {
        moves: vec![
            Move5::Rapid { to: Pose::new(Vec3::new(60.0, 0.0, 10.0), a0) },
            Move5::Feed { to: Pose::new(Vec3::new(60.0, 10.0, 10.0), a1), feed: 500.0 },
        ],
    });
    let text = emit5(&program, &Post5::rs274_5axis(), &m).unwrap();
    assert!(lines_with(&text, "G93 G1 ").len() + lines_with(&text, "G1 ").len() > 4);
    assert!(readback5(&text, &program, &Post5::rs274_5axis(), &m).unwrap().within());
    let text = emit5(&program, &Post5::tcp_g43_4(), &m).unwrap();
    assert_eq!(lines_with(&text, "G1 ").len(), 1);
}

#[test]
fn feeds_without_inverse_time_take_the_planned_time() {
    // Machine-coordinate output in G94: F is the linear axes' rate that takes the planned time.
    let machine = Machine::four_axis_a();
    let post = Post5 { inverse_time: false, ..Post5::linuxcnc_4axis() };
    let program = program5(helix());
    let text = emit5(&program, &post, &machine).unwrap();
    assert!(!text.contains("G93"));
    let rb = readback5(&text, &program, &post, &machine).unwrap();
    assert!(rb.within(), "{rb:?}");
    close(rb.cut_minutes, rb.planned_cut_minutes, 2e-3 * rb.planned_cut_minutes);
}

#[test]
fn reversed_and_offset_rotaries_read_back() {
    let post = Post5::mach3_4axis();
    for machine in [Machine::four_axis_a_with(Vec3::ZERO, true, None), Machine::four_axis_a_at(Vec3::new(0.0, 2.0, -30.0))] {
        let program = program5(helix());
        let text = emit5(&program, &post, &machine).unwrap();
        assert!(readback5(&text, &program, &post, &machine).unwrap().within());
        if machine.rotaries[0].reversed {
            assert!(text.contains(" A-1080"));
        }
        // Read back on the wrong machine, it does not fit.
        assert!(!readback5(&text, &program, &post, &Machine::four_axis_a_at(Vec3::new(0.0, 0.0, 5.0))).unwrap().within());
    }
}

#[test]
fn posts_round_trip_and_validate() {
    for name in Post5::BUILTIN {
        let post = Post5::builtin(name).unwrap();
        post.validate().unwrap();
        let text = serde_json::to_string(&post).unwrap();
        assert_eq!(serde_json::from_str::<Post5>(&text).unwrap(), post);
    }
    assert!(Post5::builtin("nope").is_none());
    assert!(Post5 { angle_decimals: 9, ..Post5::mach3_4axis() }.validate().is_err());
    assert!(Post5 { inverse_time: true, ..Post5::tcp_g43_4() }.validate().is_err());
    assert!(Post5 { tolerance: f64::NAN, ..Post5::mach3_4axis() }.validate().is_err());
    assert!(Post5 { clearance: Some(f64::INFINITY), ..Post5::mach3_4axis() }.validate().is_err());
    assert!(Post5 { safe_rotate: vec!["G0 {nope}".into()], ..Post5::mach3_4axis() }.validate().is_err());
    // A set clearance is used as it is.
    let text = emit5(&program5(radial_drops()), &Post5 { clearance: Some(55.0), ..Post5::mach3_4axis() }, &Machine::four_axis_a()).unwrap();
    assert!(text.contains("G0 Z55\nG0 A30\n"));
}

#[test]
fn hostile_programs_are_refused() {
    let machine = Machine::four_axis_a();
    let post = Post5::mach3_4axis();
    assert!(emit5(&Program5::default(), &post, &machine).is_err());
    let nan = Toolpath5 { moves: vec![Move5::Rapid { to: Pose::new(Vec3::new(f64::NAN, 0.0, 0.0), Vec3::Z) }] };
    assert!(emit5(&program5(nan), &post, &machine).is_err());
    let zero_feed =
        Toolpath5 { moves: vec![Move5::Rapid { to: Pose::vertical(Vec3::ZERO) }, Move5::Feed { to: Pose::vertical(Vec3::X), feed: 0.0 }] };
    assert!(emit5(&program5(zero_feed), &post, &machine).is_err());
    let mut bad_rpm = program5(helix());
    bad_rpm.operations[0].spindle_rpm = f64::NAN;
    assert!(emit5(&bad_rpm, &post, &machine).is_err());
    // The header of this post sets no work offset.
    let mut offset = program5(helix());
    offset.work_offset = Some(2);
    assert!(emit5(&offset, &post, &machine).is_err());
    assert!(emit5(&offset, &Post5::linuxcnc_4axis(), &machine).unwrap().contains("G55"));
    // A tilted tool on a 3-axis machine.
    let tilted = Toolpath5 { moves: vec![Move5::Rapid { to: Pose::new(Vec3::ZERO, radial(deg(10.0))) }] };
    assert!(emit5(&program5(tilted), &post, &Machine::three_axis()).is_err());
    // Indexed operations with the wrong angles or hostile coordinates.
    let indexed = |rotary: Vec<f64>, path: Toolpath| MachineProgram {
        name: "i".into(),
        operations: vec![MachineOperation::Indexed(IndexedOperation {
            name: "i".into(),
            tool: tool(),
            spindle_rpm: 1000.0,
            coolant: false,
            rotary,
            path,
        })],
        work_offset: None,
    };
    assert!(emit_program(&indexed(vec![], contour_path()), &post, &machine).is_err());
    assert!(emit_program(&indexed(vec![f64::NAN], contour_path()), &post, &machine).is_err());
    let bad = Toolpath { moves: vec![Move::Rapid { to: Vec3::ZERO }, Move::Feed { to: Vec3::new(f64::INFINITY, 0.0, 0.0), feed: 100.0 }] };
    assert!(emit_program(&indexed(vec![0.0], bad), &post, &machine).is_err());
    let cut_first = Toolpath { moves: vec![Move::Feed { to: Vec3::X, feed: 100.0 }] };
    assert!(emit_program(&indexed(vec![0.0], cut_first), &post, &machine).is_err());

    // Reading.
    let a = [AxisLetter::A];
    assert!(matches!(parse5("G0 X0 Y0 Z0 B10", &a), Err(CamError::Parse(1, _))));
    assert!(matches!(parse5("G0 X0 Y0 Z0 A0\nG93 G1 X1 A10", &a), Err(CamError::Parse(2, _))));
    assert!(parse5("G0 X0 Y0 Z0 A0\nG93 G1 X1 A10 F5\nG1 X2 A20", &a).is_err());
    assert!(parse5("G91\nG0 X1", &a).is_err());
    assert!(parse5("G0 X0 Y0 Z0 A0\nG2 X1 Y1 I1 J0 A5 F100", &a).is_err());
    assert!(parse5("G0 X0 Y0 Z0 A0\nG1 X1 F-5", &a).is_err());
    assert!(parse5("X1 Y2", &a).is_err());
    assert!(parse5("G0 X1..2", &a).is_err());
    // G93 F applies to its own block; G53 makes the axes it names unknown; G20 scales the
    // linear axes, not the angles.
    let w = parse5("G20\nG0 X1 Y0 Z1 A0\nG93 G1 X2 A90 F4\nG53 G0 Z0\nG0 Z2\nG94 G1 X3 F10", &a).unwrap();
    assert_eq!(w.len(), 4);
    let JointWords::Feed { linear, rotary, f, inverse_time: true, .. } = &w[1] else { panic!("{w:?}") };
    close3(*linear, Vec3::new(50.8, 0.0, 25.4), 1e-12);
    close(rotary[0], deg(90.0), 1e-12);
    close(*f, 4.0, 1e-12);
    let JointWords::Rapid { linear, .. } = &w[2] else { panic!() };
    close(linear.z, 50.8, 1e-12);
    let JointWords::Feed { f, inverse_time: false, mm_per_unit, .. } = &w[3] else { panic!() };
    close(f * mm_per_unit, 254.0, 1e-12);
}

fn indexed_op(name: &str, number: u32, rotary: Vec<f64>, moves: Vec<Move>) -> MachineOperation {
    MachineOperation::Indexed(IndexedOperation {
        name: name.into(),
        tool: Tool::new(number, ToolKind::FlatEndmill, 6.0, 20.0),
        spindle_rpm: 9000.0,
        coolant: false,
        rotary,
        path: Toolpath { moves },
    })
}

fn program_of(operations: Vec<MachineOperation>) -> MachineProgram {
    MachineProgram { name: "p".into(), operations, work_offset: None }
}

/// At every block that turns a rotary: the Z there (`None` until written; `G53` sends it to
/// the machine's top) and the highest Z since the last cut.
fn rotary_turns(text: &str) -> Vec<(Option<f64>, f64)> {
    let mut z: Option<f64> = None;
    let mut peak = f64::NEG_INFINITY;
    let mut out = Vec::new();
    for line in text.lines() {
        if line.starts_with("G53") {
            z = Some(f64::INFINITY);
        } else if let Some(v) = line.split(' ').find_map(|w| w.strip_prefix('Z')).and_then(|v| v.parse().ok()) {
            z = Some(v);
        }
        if line.contains("G1 ") || line.contains("G2 ") || line.contains("G3 ") {
            peak = f64::NEG_INFINITY;
        }
        peak = peak.max(z.unwrap_or(f64::NEG_INFINITY));
        if line.starts_with("G0 A") || line.starts_with("G0 B") {
            out.push((z, peak));
        }
    }
    out
}

#[test]
fn a_rotary_never_turns_after_the_tool_has_come_down() {
    // A tall index (rapids at Z35) between two flat ones (rapids at Z10), the same tool: the
    // tool stays at Z35 while A turns back, rather than rapiding down to Z10 at A90 first.
    let machine = Machine::four_axis_a();
    let tall = indexed_op(
        "tall",
        3,
        vec![deg(90.0)],
        vec![
            Move::Rapid { to: Vec3::new(20.0, 0.0, 35.0) },
            Move::Feed { to: Vec3::new(20.0, 0.0, 28.0), feed: 200.0 },
            Move::Feed { to: Vec3::new(30.0, 0.0, 28.0), feed: 400.0 },
            Move::Rapid { to: Vec3::new(30.0, 0.0, 35.0) },
        ],
    );
    let flat = indexed_op(
        "flat",
        3,
        vec![0.0],
        vec![
            Move::Rapid { to: Vec3::new(0.0, 0.0, 10.0) },
            Move::Feed { to: Vec3::new(0.0, 0.0, 3.0), feed: 200.0 },
            Move::Feed { to: Vec3::new(10.0, 0.0, 3.0), feed: 400.0 },
            Move::Rapid { to: Vec3::new(10.0, 0.0, 10.0) },
        ],
    );
    let program = program_of(vec![flat.clone(), tall, flat]);
    let posts = [
        Post5::mach3_4axis(),
        Post5::linuxcnc_4axis(),
        Post5::grblhal_4axis(),
        Post5::rs274_5axis(),
        Post5 { clearance_retract: true, ..Post5::rs274_5axis() },
        Post5 { clearance: Some(12.0), ..Post5::mach3_4axis() },
    ];
    for post in posts {
        let text = emit_program(&program, &post, &machine).unwrap();
        let turns = rotary_turns(&text);
        assert_eq!(turns.len(), 3, "{text}");
        for (z, peak) in &turns {
            // Never below where the tool was since its last cut, never at a height not known.
            assert!(z.is_some_and(|z| z >= *peak - 1e-9), "{}: a turn at Z {z:?} after Z {peak}\n{text}", post.base.name);
        }
        if post.safe_rotate.is_empty() {
            // At the start, up to the program's highest clearance; A0 again only from Z35.
            assert!(turns[0].0.is_some_and(|z| z >= 35.0 - 1e-9), "{text}");
            assert!(turns[2].0.is_some_and(|z| z >= 35.0 - 1e-9), "{text}");
        }
        assert!(readback_program(&text, &program, &post, &machine).unwrap().within());
    }
}

#[test]
fn an_operation_rises_before_it_moves_across() {
    // A slot that ends at depth, then a hole elsewhere with the same tool or another: the
    // tool rises to Z15 before it moves across, never across at Z5 through the part.
    let machine = Machine::four_axis_a();
    let slot = vec![
        Move::Rapid { to: Vec3::new(0.0, 0.0, 15.0) },
        Move::Feed { to: Vec3::new(0.0, 0.0, 5.0), feed: 200.0 },
        Move::Feed { to: Vec3::new(20.0, 0.0, 5.0), feed: 300.0 },
    ];
    let hole = vec![
        Move::Rapid { to: Vec3::new(-10.0, 5.0, 15.0) },
        Move::Feed { to: Vec3::new(-10.0, 5.0, 5.0), feed: 200.0 },
        Move::Rapid { to: Vec3::new(-10.0, 5.0, 15.0) },
    ];
    for post in [Post5::mach3_4axis(), Post5::linuxcnc_4axis(), Post5::grblhal_4axis()] {
        for second in [3, 4] {
            let program = program_of(vec![indexed_op("slot", 3, vec![0.0], slot.clone()), indexed_op("hole", second, vec![0.0], hole.clone())]);
            let text = emit_program(&program, &post, &machine).unwrap();
            let at = text.find("X-10 Y5").unwrap();
            assert!(text[..at].ends_with("G0 Z15\nG0 "), "{}: T{second}\n{text}", post.base.name);
            // The first operation's first rapid rises before it moves across as well.
            assert!(!text.contains("G0 X0 Y0\nG0 Z15"), "{text}");
        }
    }
    // The 3-axis emitter alike.
    let op = |name: &str, number: u32, moves: &[Move]| Operation {
        name: name.into(),
        tool: Tool::new(number, ToolKind::FlatEndmill, 6.0, 20.0),
        spindle_rpm: 9000.0,
        coolant: false,
        path: Toolpath { moves: moves.to_vec() },
    };
    for second in [3, 4] {
        let program = Program { name: "p".into(), operations: vec![op("slot", 3, &slot), op("hole", second, &hole)], work_offset: None };
        let text = emit(&program, &Post::rs274()).unwrap();
        let at = text.find("X-10 Y5").unwrap();
        assert!(text[..at].ends_with("G0 Z15\nG0 "), "T{second}\n{text}");
    }
}

/// A cut that turns A by 20° and ends there, the rapid after it turning nothing.
fn turning_cut(name: &str, number: u32) -> MachineOperation {
    let tool = Tool::new(number, ToolKind::BallEndmill, 6.0, 20.0);
    let path = Toolpath5 {
        moves: vec![
            Move5::Rapid { to: Pose::new(radial(0.0) * 30.0, radial(0.0)) },
            Move5::Feed { to: Pose::new(radial(0.0) * 20.0, radial(0.0)), feed: 200.0 },
            Move5::Feed { to: Pose::new(radial(deg(20.0)) * 20.0, radial(deg(20.0))), feed: 300.0 },
            Move5::Rapid { to: Pose::new(radial(deg(20.0)) * 30.0, radial(deg(20.0))) },
        ],
    };
    MachineOperation::Continuous(Operation5 { name: name.into(), tool, spindle_rpm: 12000.0, coolant: true, path })
}

#[test]
fn inverse_time_ends_before_a_tool_change() {
    let machine = Machine::four_axis_a();
    for post in [Post5::mach3_4axis(), Post5::linuxcnc_4axis(), Post5::grblhal_4axis(), Post5::rs274_5axis()] {
        let program = program_of(vec![turning_cut("a", 3), turning_cut("b", 4)]);
        let text = emit_program(&program, &post, &machine).unwrap();
        // G94 is in force from the end of the first operation's cuts: before the tool change,
        // spindle and coolant blocks of the next.
        let change = text.find("(b:").or_else(|| text.find("; b:")).unwrap();
        let before = &text[..change];
        assert!(before.rfind("G94").unwrap() > before.rfind("G93").unwrap(), "{}\n{text}", post.base.name);
        assert!(readback_program(&text, &program, &post, &machine).unwrap().within());
    }
}

#[test]
fn tool_changes_stop_the_spindle_and_coolant() {
    let machine = Machine::four_axis_a();
    let program = program_of(vec![turning_cut("a", 3), turning_cut("b", 4)]);
    // Mach3: coolant and spindle off before M6, both on again after.
    let text = emit_program(&program, &Post5::mach3_4axis(), &machine).unwrap();
    assert!(text.contains("M9\nM5\nM6 T4\nG43 H4\nM3 S12000\nM8\n"), "{text}");
    // grblHAL: a manual change, the operator told which tool, the program paused.
    let text = emit_program(&program, &Post5::grblhal_4axis(), &machine).unwrap();
    assert!(text.contains("M9\nM5\n; change to tool 4, diameter 6\nM0\nM3 S12000\nM8\n"), "{text}");
    assert!(!text.contains("M6") && !text.contains("G43"));
}

#[test]
fn presets_write_their_units_and_incremental_arc_centres() {
    let machine = Machine::four_axis_a();
    for post in [Post5::mach3_4axis(), Post5::linuxcnc_4axis(), Post5::grblhal_4axis()] {
        let text = emit5(&program5(helix()), &post, &machine).unwrap();
        let header = text.lines().nth(1).unwrap_or_default();
        assert!(header.contains("G91.1") && header.contains("G21"), "{header}");
        // In inches the header says so, and the program reads back where it was planned.
        let mut inch = post.clone();
        inch.base.units = Units::Inch;
        let program = program5(helix());
        let text = emit5(&program, &inch, &machine).unwrap();
        let header = text.lines().nth(1).unwrap_or_default();
        assert!(header.contains("G20") && !header.contains("G21"), "{header}");
        assert!(readback5(&text, &program, &inch, &machine).unwrap().within(), "{}", inch.base.name);
    }
    // G91.1 is read, G90.1 (absolute arc centres) refused.
    let a = [AxisLetter::A];
    assert_eq!(parse5("G91.1\nG0 X0 Y0 Z0 A0\nG2 X2 Y0 I1 J0 F100", &a).unwrap().len(), 2);
    assert!(matches!(parse5("G90.1\nG0 X0 Y0 Z0 A0", &a), Err(CamError::Parse(1, _))));
    assert!(matches!(parse("G90.1\nG0 X0 Y0 Z0"), Err(CamError::Parse(1, _))));
    assert_eq!(parse("G91.1 G0 X0 Y0 Z0\nG2 X2 Y0 I1 J0 F100").unwrap().len(), 2);
}

#[test]
fn feeds_are_never_written_as_zero_or_absurd() {
    let machine = Machine::four_axis_a();
    let post = Post5::mach3_4axis();
    let cut = |feed: f64| Toolpath5 {
        moves: vec![Move5::Rapid { to: Pose::vertical(Vec3::ZERO) }, Move5::Feed { to: Pose::vertical(Vec3::new(1.0, 1.0, 0.0)), feed }],
    };
    for feed in [1e-300, 1e-5, 4e-5, 2e6, 1e308] {
        assert!(emit5(&program5(cut(feed)), &post, &machine).is_err(), "{feed}");
    }
    let fine = Post5 { base: Post { feed_decimals: 4, ..post.base.clone() }, ..post.clone() };
    assert!(emit5(&program5(cut(1e-3)), &fine, &machine).is_ok());
    // At the post's one decimal a feed of 0.001 would be written as F0.
    assert!(emit5(&program5(cut(1e-3)), &post, &machine).is_err());
    let indexed = program_of(vec![indexed_op("i", 3, vec![0.0], vec![Move::Rapid { to: Vec3::ZERO }, Move::Feed { to: Vec3::X, feed: 1e-9 }])]);
    assert!(emit_program(&indexed, &post, &machine).is_err());
    let flat = Program {
        name: "p".into(),
        operations: vec![Operation {
            name: "o".into(),
            tool: tool(),
            spindle_rpm: 1000.0,
            coolant: false,
            path: Toolpath { moves: vec![Move::Rapid { to: Vec3::ZERO }, Move::Feed { to: Vec3::X, feed: 1e-5 }] },
        }],
        work_offset: None,
    };
    assert!(emit(&flat, &Post::rs274()).is_err());
    // A very long, slow cut that turns A: its inverse-time F is tiny but never F0.
    let slow = Toolpath5 {
        moves: vec![
            Move5::Rapid { to: Pose::new(radial(0.0) * 20.0, radial(0.0)) },
            Move5::Feed { to: Pose::new(Vec3::new(4.0e6, 0.0, 0.0) + radial(deg(1.0)) * 20.0, radial(deg(1.0))), feed: 1e-3 },
        ],
    };
    let text = emit5(&program5(slow), &fine, &machine).unwrap();
    assert!(text.contains("G93 G1") && !text.contains(" F0\n"), "{text}");
    let written = parse5(&text, &machine.letters()).unwrap();
    assert!(written.iter().any(|w| matches!(w, JointWords::Feed { inverse_time: true, f, .. } if *f > 0.0 && *f < 1e-9)), "{text}");
    // A tool with an absurd flute length is refused.
    let mut long = program5(helix());
    long.operations[0].tool = Tool::new(3, ToolKind::FlatEndmill, 6.0, 1e300);
    assert!(emit5(&long, &post, &machine).is_err());
}

#[test]
fn indexed_operations_with_tcp_keep_their_angles() {
    // A tilted head: the tool stays tilted 30°, as asked, rather than being solved upright.
    let head = Machine::head_table_bc();
    let tcp = Post5::tcp_g43_4();
    let path = vec![
        Move::Rapid { to: Vec3::new(0.0, 0.0, 10.0) },
        Move::Feed { to: Vec3::new(0.0, 0.0, -1.0), feed: 200.0 },
        Move::Feed { to: Vec3::new(10.0, 0.0, -1.0), feed: 300.0 },
    ];
    let program = program_of(vec![indexed_op("t", 3, vec![deg(30.0), 0.0], path.clone())]);
    let text = emit_program(&program, &tcp, &head).unwrap();
    let written = parse5(&text, &head.letters()).unwrap();
    assert!(!written.is_empty());
    for w in &written {
        if let JointWords::Rapid { rotary, .. } | JointWords::Feed { rotary, .. } = w {
            close(rotary[0], deg(30.0), 1e-9);
            close(rotary[1], 0.0, 1e-9);
        }
    }
    let rb = readback_program(&text, &program, &tcp, &head).unwrap();
    assert!(rb.within(), "{rb:?}");
    // The planned axis is the tilted tool's, not the frame's Z.
    let MachineOperation::Indexed(o) = &program.operations[0] else { panic!() };
    for p in o.poses(&head, 0.001).unwrap().poses() {
        close3(p.axis, tool_axis(&head, &o.rotary), 1e-12);
    }

    // A table machine after an operation near C = 180°: A30 C0 is written as asked, not the
    // equivalent A-30 C180 nearer the previous angles.
    let table = Machine::table_table_ac();
    let program =
        program_of(vec![indexed_op("near", 3, vec![deg(-20.0), deg(170.0)], path.clone()), indexed_op("asked", 3, vec![deg(30.0), 0.0], path)]);
    let text = emit_program(&program, &tcp, &table).unwrap();
    let rotaries: Vec<Vec<f64>> = parse5(&text, &table.letters())
        .unwrap()
        .into_iter()
        .filter_map(|w| match w {
            JointWords::Rapid { rotary, .. } | JointWords::Feed { rotary, .. } => Some(rotary),
            _ => None,
        })
        .collect();
    let first = rotaries.first().unwrap();
    close(first[0], deg(-20.0), 1e-9);
    close(first[1], deg(170.0), 1e-9);
    let last = rotaries.last().unwrap();
    close(last[0], deg(30.0), 1e-9);
    close(last[1], 0.0, 1e-9);
    assert!(readback_program(&text, &program, &tcp, &table).unwrap().within());
}

#[test]
fn a_large_diagonal_raster_reads_back() {
    // A 45° raster over a 300 mm square at 0.5 mm stepover: every pass one long diagonal G1.
    let machine = Machine::four_axis_a();
    let post = Post5::mach3_4axis();
    let step = 0.5 * 2f64.sqrt();
    let mut moves = Vec::new();
    let mut c = -300.0 + step * 0.5;
    while c < 300.0 {
        // The line x - y = c across the square.
        let (x0, x1) = (c.max(0.0), (300.0 + c).min(300.0));
        let (a, b) = (Vec3::new(x0, x0 - c, 0.0), Vec3::new(x1, x1 - c, 0.0));
        moves.push(Move5::Rapid { to: Pose::vertical(a + Vec3::Z * 5.0) });
        moves.push(Move5::Feed { to: Pose::vertical(a), feed: 300.0 });
        moves.push(Move5::Feed { to: Pose::vertical(b), feed: 1000.0 });
        moves.push(Move5::Rapid { to: Pose::vertical(b + Vec3::Z * 5.0) });
        c += step;
    }
    assert!(moves.len() > 3000);
    let program = program5(Toolpath5 { moves });
    let text = emit5(&program, &post, &machine).unwrap();
    let rb = readback5(&text, &program, &post, &machine).unwrap();
    assert!(rb.within(), "{rb:?}");
}

/// A post that writes, as data alone, the conventions of a Mach3 program from a common
/// commercial CAM post: a tool list, capital comments from a few characters, a blank line and
/// the operation's name before each operation, `S… M3`, `G17 G90 G94` and the work offset at
/// every operation, the tool length offset on the first Z, modal motion codes, three decimals
/// with a point on whole numbers and whole feeds, a `G28 G91 Z0.` retract before every tool
/// change, every turn of A and the end, inverse time with four decimals, and rewinds.
fn dialect() -> Post5 {
    let retract = vec!["G28 G91 Z0.".to_string(), "G90".to_string()];
    let lines = |l: &[&str]| l.iter().map(|s| s.to_string()).collect::<Vec<_>>();
    Post5 {
        base: Post {
            name: "mach3-dialect".into(),
            extension: "tap".into(),
            decimals: 3,
            feed_decimals: 0,
            trailing_decimal: true,
            header: lines(&["G90 G94 G91.1 G40 G49 G17", "{units}"]),
            tool_change: lines(&["T{tool} M6"]),
            tool_change_first: true,
            spindle_on: lines(&["S{rpm} M3"]),
            footer: lines(&["G28 G91 X0. Y0.", "G90", "M30"]),
            upper_case_comments: true,
            comment_chars: Some(" .,=_-".into()),
            tool_list: true,
            operation_name_only: true,
            blank_lines: true,
            operation_start: lines(&["G17 G90 G94", "{work_offset}"]),
            safe_retract: Some(SafeRetract { lines: retract.clone(), at_start: true, at_tool_change: true, at_end: true }),
            tool_length_on_first_z: true,
            modal_motion: true,
            ..Post::rs274()
        },
        safe_rotate: retract,
        safe_rotate_first: true,
        angle_decimals: 3,
        inverse_time_decimals: Some(4),
        rewind: Some(Rewind { distance: 25.0, retract_feed: 500.0, plunge_feed: 250.0 }),
        ..Post5::mach3_4axis()
    }
}

/// Two 3-axis operations with two tools: a contour with a flat end mill, then a pocket with a
/// ball end mill, both with coolant.
fn dialect_job() -> Program {
    let flat = Tool::new(1, ToolKind::FlatEndmill, 6.0, 20.0);
    let ball = Tool::new(2, ToolKind::BallEndmill, 4.0, 20.0);
    let cut = |bottom: f64| Cut {
        heights: Heights { clearance: 15.0, retract: 5.0, top: 0.0, bottom },
        feeds: Feeds { cut: 1600.0, plunge: 50.0 },
        stepdown: 1.0,
    };
    let contour_params = ContourParams { side: Side::Outside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 };
    let pocket_params = PocketParams { stepover: 1.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 };
    let outline = contour(&[rect(0.0, 0.0, 40.0, 20.0)], &flat, &contour_params, &cut(-2.0)).unwrap();
    let hollow = pocket(&[rect(10.0, 5.0, 30.0, 15.0)], &ball, &pocket_params, &cut(-1.25)).unwrap();
    Program {
        name: "part-1".into(),
        operations: vec![
            Operation { name: "Contour #1 (outer)".into(), tool: flat, spindle_rpm: 18000.0, coolant: true, path: outline },
            Operation { name: "Pocket1".into(), tool: ball, spindle_rpm: 24000.0, coolant: true, path: hollow },
        ],
        work_offset: None,
    }
}

/// A wrapped helix of `turns` turns round the X axis at radius 20, 10 mm along X a turn.
fn helix_of(turns: usize) -> Toolpath5 {
    let n = 120 * turns;
    let pose = |k: usize| {
        let th = TAU * k as f64 / 120.0;
        Pose::new(Vec3::new(10.0 * k as f64 / 120.0, 0.0, 0.0) + radial(th) * 20.0, radial(th))
    };
    let mut moves = vec![Move5::Rapid { to: Pose::new(pose(0).up(10.0), radial(0.0)) }, Move5::Feed { to: pose(0), feed: 200.0 }];
    moves.extend((1..=n).map(|k| Move5::Feed { to: pose(k), feed: 300.0 }));
    moves.push(Move5::Feed { to: Pose::new(pose(n).up(10.0), pose(n).axis), feed: 1000.0 });
    Toolpath5 { moves }
}

/// A 4th axis that turns at most a turn either way, its centreline on the work's X axis.
fn limited_a() -> Machine {
    Machine::four_axis_a_with(Vec3::ZERO, false, Some((-TAU, TAU)))
}

/// Checks that a motion code is written only when the mode changes (a template line, a turn
/// of A or a tool length offset line sets no mode the next block may leave out), that every
/// number of a motion word has a decimal point, and that every arc states both I and J.
fn assert_modal_with_points(text: &str) {
    let mut mode: Option<&str> = None;
    for line in text.lines() {
        let words: Vec<&str> = line.split(' ').collect();
        let code = words.iter().copied().find(|w| ["G0", "G1", "G2", "G3"].contains(w));
        let motion = words.iter().any(|w| w.len() > 1 && "XYZIJAF".contains(&w[..1]) && w[1..].parse::<f64>().is_ok());
        if !motion || line.starts_with("G28") {
            mode = None;
            continue;
        }
        for w in &words {
            if w.len() > 1 && "XYZIJAF".contains(&w[..1]) && w[1..].parse::<f64>().is_ok() {
                assert!(w.contains('.'), "`{w}` without a decimal point in `{line}`");
            }
        }
        match code {
            Some(c) => {
                assert!(mode != Some(c) || words.iter().any(|w| w.starts_with('A')), "`{c}` written again in `{line}`");
                mode = Some(c);
            }
            None => assert!(mode.is_some() || line.starts_with("G43"), "no motion mode for `{line}`"),
        }
        if matches!(mode, Some("G2" | "G3")) && code.is_some() {
            assert!(line.contains(" I") && line.contains(" J"), "an arc without both I and J: `{line}`");
        }
        if line.starts_with("G0 A") {
            mode = None;
        }
    }
}

#[test]
fn a_dialect_post_writes_its_conventions_as_data() {
    let post = dialect();
    post.validate().unwrap();
    let program = dialect_job();
    let text = emit(&program, &post.base).unwrap();
    let start = "\
(PART-1)
(T1 D=6. CR=0. - ZMIN=-2. - FLAT END MILL)
(T2 D=4. CR=2. - ZMIN=-1.25 - BALL END MILL)
G90 G94 G91.1 G40 G49 G17
G21
G28 G91 Z0.
G90

(CONTOUR 1 OUTER)
T1 M6
S18000 M3
G17 G90 G94
G54
M8
G0 X0. Y-3.
G43 Z15. H1
Z5.
G1 Z0. F50.
Z-1.
G2 X-3. Y0. I0. J3. F1600.
G1 Y20.
";
    assert!(text.starts_with(start), "{text}");
    // The spindle stops and the tool rises before the next tool; coolant goes off for the change.
    let change = "\
G0 Z15.
M5
G28 G91 Z0.
G90

(POCKET1)
M9
T2 M6
S24000 M3
G17 G90 G94
G54
M8
G0 X13.5 Y8.5
G43 Z15. H2
Z5.
G1 Z0. F50.
";
    assert!(text.contains(change), "{text}");
    let end = "\
G0 Z15.

M9
M5
G28 G91 Z0.
G90
G28 G91 X0. Y0.
G90
M30
";
    assert!(text.ends_with(end), "{text}");
    assert_modal_with_points(&text);
    // Feeds after a cut in the same mode leave the code out.
    assert!(text.lines().any(|l| l == "Z-1."), "{text}");
    assert!(readback(&text, &program, &post.base).unwrap().within());

    // The multi-axis writer writes the very same program for a 3-axis machine.
    let indexed = MachineProgram {
        name: program.name.clone(),
        operations: program
            .operations
            .iter()
            .map(|o| {
                MachineOperation::Indexed(IndexedOperation {
                    name: o.name.clone(),
                    tool: o.tool.clone(),
                    spindle_rpm: o.spindle_rpm,
                    coolant: o.coolant,
                    rotary: vec![],
                    path: o.path.clone(),
                })
            })
            .collect(),
        work_offset: None,
    };
    let machine = Machine::three_axis();
    let text5 = emit_program(&indexed, &post, &machine).unwrap();
    assert_eq!(text5, text);
    assert!(readback_program(&text5, &indexed, &post, &machine).unwrap().within());

    // The work offset is written at every operation.
    let mut offset = program.clone();
    offset.work_offset = Some(2);
    let text = emit(&offset, &post.base).unwrap();
    assert_eq!(lines_with(&text, "G55").len(), 2);
    assert!(lines_with(&text, "G54").is_empty());
    // An arc leaves out the end coordinate that does not change.
    let half = Toolpath {
        moves: vec![
            Move::Rapid { to: Vec3::new(1.0, 0.0, 5.0) },
            Move::Feed { to: Vec3::new(1.0, 0.0, 0.0), feed: 100.0 },
            Move::Arc { to: Vec3::new(-1.0, 0.0, -0.5), center: Vec2::ZERO, ccw: true, feed: 100.0 },
        ],
    };
    let mut arc = program.clone();
    arc.operations.truncate(1);
    arc.operations[0].path = half;
    let text = emit(&arc, &post.base).unwrap();
    assert!(text.contains("\nG3 X-1. Z-0.5 I-1. J0.\n"), "{text}");
    assert!(readback(&text, &arc, &post.base).unwrap().within());
    // Lines after every operation's motion.
    let closing = Post { operation_end: vec!["G17".into()], ..post.base.clone() };
    let text = emit(&program, &closing).unwrap();
    assert!(text.contains("G0 Z15.\nG17\nM5\n") && text.contains("G0 Z15.\nG17\n\nM9\n"), "{text}");
}

#[test]
fn a_dialect_post_retracts_before_every_turn_of_a() {
    let post = dialect();
    let machine = Machine::four_axis_a();
    let side = |name: &str, a: f64| indexed_op(name, 3, vec![deg(a)], contour_path().moves);
    let program = MachineProgram {
        name: "wrap".into(),
        operations: vec![side("side 0", 0.0), side("side 90", 90.0), MachineOperation::Continuous(op5("helix", helix_of(1)))],
        work_offset: None,
    };
    let text = emit_program(&program, &post, &machine).unwrap();
    // No lowest Z where A turns.
    assert!(text.starts_with("(WRAP)\n(T3 D=6. CR=0. - FLAT END MILL)\n"), "{text}");
    // The first index turns A after the start's retract; the next one retracts before its
    // comment, then A turns alone and the tool comes back across and down with its offset.
    assert!(text.contains("\n(SIDE 0)\nT3 M6\nS9000 M3\nG17 G90 G94\nG54\nG0 A0.\nG0 X0. Y-3.\nG43 Z10. H3\nZ2.\n"), "{text}");
    assert!(text.contains("\nG0 Z10.\nG28 G91 Z0.\nG90\n\n(SIDE 90)\nG17 G90 G94\nG54\nG0 A90.\nG0 X0. Y-3.\nG43 Z10. H3\nZ2.\n"), "{text}");
    assert_modal_with_points(&text);
    // Inverse time: F with at most four decimals on every block that turns A, G94 before the
    // next cut that does not.
    let mut inverse = false;
    let mut timed = 0;
    for line in text.lines() {
        if line.contains("G93") {
            inverse = true;
        }
        if line.contains("G94") {
            inverse = false;
        }
        if inverse && line.contains(" A") {
            let f = line.split(' ').find_map(|w| w.strip_prefix('F')).unwrap_or_else(|| panic!("no F in `{line}`"));
            assert!(f.split('.').nth(1).is_some_and(|d| d.len() <= 4), "`F{f}` in `{line}`");
            timed += 1;
        }
        if let Some(a) = line.split(' ').find_map(|w| w.strip_prefix('A')) {
            assert!(a.split('.').nth(1).is_some_and(|d| d.len() <= 3), "`A{a}` in `{line}`");
        }
        if !inverse && line.starts_with('X') {
            assert!(!line.contains(" A"), "a turn of A per minute: `{line}`");
        }
    }
    assert!(timed >= 120, "{timed}");
    assert!(text.contains("G94 Z30. F1000.\n\nM9\nM5\nG28 G91 Z0.\nG90\n"), "{text}");
    let rb = readback_program(&text, &program, &post, &machine).unwrap();
    assert!(rb.within(), "{rb:?}");
}

/// The lines of the rewinds in `text`: from the draw out of the cut to the first block after
/// the plunge back.
fn rewinds_in(text: &str) -> Vec<Vec<&str>> {
    let lines: Vec<&str> = text.lines().collect();
    lines.iter().enumerate().filter(|(_, l)| l.starts_with("G94 Z") && l.ends_with(" F500.")).map(|(i, _)| lines[i..i + 8].to_vec()).collect()
}

#[test]
fn a_long_helix_rewinds_within_the_limits_of_a() {
    let post = dialect();
    // The first plunge at the rewinds' plunge feed: a planned move like a rewind's is still
    // looked for in what is written.
    let mut path = helix_of(5);
    if let Some(Move5::Feed { feed, .. }) = path.moves.get_mut(1) {
        *feed = 250.0;
    }
    let program = program5(path);
    // An A that turns one turn either way could not cut five turns in one go.
    assert!(emit5(&program, &Post5 { rewind: None, ..post.clone() }, &limited_a()).is_err());
    let text = emit5(&program, &post, &limited_a()).unwrap();
    let angles: Vec<f64> = text.lines().filter_map(|l| l.split(' ').find_map(|w| w.strip_prefix('A'))).map(|a| a.parse().unwrap()).collect();
    assert!(angles.iter().all(|a| a.abs() <= 360.0), "{text}");
    // Up to 360°, back to -360° (the same position), on to 360° again, back, on to 1800° - 1440°.
    assert_eq!(angles.last(), Some(&360.0));
    let rewinds = rewinds_in(&text);
    assert_eq!(rewinds.len(), 2, "{text}");
    assert_eq!(
        rewinds[0],
        ["G94 Z45. F500.", "G28 G91 Z0.", "G90", "G0 A-360.", "G0 X10. Y0.", "G43 Z45. H3", "G1 Z20. F250.", "G93 X10.042 Z19.993 A-358.5 F571.2171"]
    );
    assert!(rewinds[1][0..7].join("\n").starts_with("G94 Z45. F500.\nG28 G91 Z0.\nG90\nG0 A-360.\nG0 X30. Y0.\nG43 Z45. H3\nG1 Z20. F250."));
    let rb = readback5(&text, &program, &post, &limited_a()).unwrap();
    assert!(rb.within(), "{rb:?}");
    // Each rewind draws out 25 mm and plunges 25 mm back at feed.
    close(rb.cut_length, rb.planned_cut_length + 2.0 * 50.0, 1e-3 * rb.planned_cut_length);
    let gone = Program5 { operations: vec![op5("op", helix_of(4))], ..program.clone() };
    assert!(!readback5(&text, &gone, &post, &limited_a()).unwrap().within());

    // Limits that a cut reaches part way through a piece: the piece is split there, each part
    // timed for its share.
    let tight = Machine::four_axis_a_with(Vec3::ZERO, false, Some((-TAU + 0.05, TAU - 0.05)));
    let text = emit5(&program, &post, &tight).unwrap();
    let limit = (TAU - 0.05).to_degrees();
    for a in text.lines().filter_map(|l| l.split(' ').find_map(|w| w.strip_prefix('A'))) {
        assert!(a.parse::<f64>().unwrap().abs() <= limit + 1e-3, "A{a}");
    }
    // Under two turns between the limits: one turn back each time.
    assert_eq!(rewinds_in(&text).len(), 5, "{text}");
    let rb = readback5(&text, &program, &post, &tight).unwrap();
    assert!(rb.within(), "{rb:?}");
    close(rb.cut_minutes, rb.planned_cut_minutes + 5.0 * (25.0 / 500.0 + 25.0 / 250.0), 2e-3 * rb.planned_cut_minutes);

    // Without limits the helix still adds up its turns.
    let text = emit5(&program, &post, &Machine::four_axis_a()).unwrap();
    assert!(rewinds_in(&text).is_empty() && text.contains(" A1800."), "{text}");
    // A rewind needs room for a whole turn.
    let narrow = Machine::four_axis_a_with(Vec3::ZERO, false, Some((-deg(170.0), deg(170.0))));
    assert!(emit5(&program, &post, &narrow).is_err());
}

#[test]
fn dialect_switches_validate_and_default_to_the_old_output() {
    let post = dialect();
    let text = serde_json::to_string(&post).unwrap();
    assert_eq!(serde_json::from_str::<Post5>(&text).unwrap(), post);
    // Posts without the switches write none of them, and read back as they were.
    let old = serde_json::to_string(&Post::rs274()).unwrap();
    for key in ["tool_list", "safe_retract", "modal_motion", "blank_lines", "comment_chars", "operation_start"] {
        assert!(!old.contains(key), "{old}");
    }
    let old5 = serde_json::to_string(&Post5::mach3_4axis()).unwrap();
    assert!(!old5.contains("rewind") && !old5.contains("inverse_time_decimals") && !old5.contains("safe_rotate_first"), "{old5}");
    let broken = |f: &dyn Fn(&mut Post5)| {
        let mut p = dialect();
        f(&mut p);
        p.validate().is_err()
    };
    assert!(broken(&|p| p.base.comment_chars = Some("()".into())));
    assert!(broken(&|p| p.base.comment_chars = Some("x".repeat(100))));
    assert!(broken(&|p| p.base.safe_retract = Some(SafeRetract { lines: vec![], at_start: true, at_tool_change: false, at_end: false })));
    assert!(broken(&|p| p.base.operation_start = vec!["G93".into()]));
    assert!(broken(&|p| p.base.operation_end = vec!["G0 {nope}".into()]));
    assert!(broken(&|p| p.inverse_time_decimals = Some(13)));
    assert!(broken(&|p| p.rewind = Some(Rewind { distance: f64::NAN, retract_feed: 500.0, plunge_feed: 250.0 })));
    assert!(broken(&|p| p.rewind = Some(Rewind { distance: 25.0, retract_feed: 0.0, plunge_feed: 250.0 })));
    assert!(broken(&|p| {
        p.tcp = Some(Tcp { on: vec!["G43.4 H{tool}".into()], off: vec!["G49".into()] });
        p.inverse_time = false;
    }));

    // A return home makes the axes it names unknown; other incremental moves are refused.
    let back = parse("G0 X1 Y1 Z1\nG28 G91 Z0.\nG90\nG0 X2 Y2\nG43 Z5. H1").unwrap();
    assert_eq!(back, vec![Move::Rapid { to: Vec3::new(1.0, 1.0, 1.0) }, Move::Rapid { to: Vec3::new(2.0, 2.0, 5.0) }]);
    assert!(parse("G28 G91 Z0.\nG0 X1 Y1 Z1").is_err());
    assert!(parse("G91 G0 X1").is_err());
    let a = [AxisLetter::A];
    let w = parse5("G0 X1 Y1 Z1 A0\nG28 G91 Z0.\nG90\nG0 A90.\nG0 X2 Y2\nG43 Z5. H1", &a).unwrap();
    assert_eq!(w.len(), 2);
    let JointWords::Rapid { linear, rotary, .. } = &w[1] else { panic!("{w:?}") };
    close3(*linear, Vec3::new(2.0, 2.0, 5.0), 1e-12);
    close(rotary[0], deg(90.0), 1e-12);
    assert!(parse5("G28 G91 Z0.\nG0 X1 Y1 Z1 A0", &a).is_err());
}

/// `op` with its coolant on.
fn with_coolant(op: MachineOperation) -> MachineOperation {
    match op {
        MachineOperation::Indexed(o) => MachineOperation::Indexed(IndexedOperation { coolant: true, ..o }),
        other => other,
    }
}

#[test]
fn coolant_can_come_on_after_the_first_turn_of_a() {
    let machine = Machine::four_axis_a();
    let side = |name: &str, a: f64| with_coolant(indexed_op(name, 3, vec![deg(a)], contour_path().moves));
    let program = MachineProgram { name: "sides".into(), operations: vec![side("side 0", 0.0), side("side 90", 90.0)], work_offset: None };
    // By default the coolant comes on before the index, as it always has.
    let before = emit_program(&program, &dialect(), &machine).unwrap();
    assert!(before.contains("\nG54\nM8\nG0 A0.\nG0 X0. Y-3.\n"), "{before}");
    // With the switch, right after it; the second side keeps the coolant on through its turn.
    let post = Post5 { coolant_after_turn: true, ..dialect() };
    post.validate().unwrap();
    let after = emit_program(&program, &post, &machine).unwrap();
    assert!(after.contains("\nG54\nG0 A0.\nM8\nG0 X0. Y-3.\nG43 Z10. H3\n"), "{after}");
    assert!(after.contains("\n(SIDE 90)\nG17 G90 G94\nG54\nG0 A90.\nG0 X0. Y-3.\n"), "{after}");
    assert_eq!(after.matches("M8").count(), 1, "{after}");
    assert!(after.ends_with("G0 Z10.\n\nM9\nM5\nG28 G91 Z0.\nG90\nG28 G91 X0. Y0.\nG90\nM30\n"), "{after}");
    // Only the coolant line moves.
    assert_eq!(before.replace("M8\n", ""), after.replace("M8\n", ""));
    assert_modal_with_points(&after);
    assert!(readback_program(&after, &program, &post, &machine).unwrap().within());

    // A wrapped helix starts with a turn of A too; its coolant follows that turn.
    let helix = MachineProgram { operations: vec![MachineOperation::Continuous(op5("helix", helix_of(1)))], ..program.clone() };
    let text = emit_program(&helix, &post, &machine).unwrap();
    let lines: Vec<&str> = text.lines().collect();
    let m8 = lines.iter().position(|l| *l == "M8").unwrap();
    assert!(lines[m8 - 1].starts_with("G0 A"), "{text}");
    assert!(!lines[..m8].iter().any(|l| l.starts_with("G1 ") || l.starts_with("G93")), "{text}");
    assert!(readback_program(&text, &helix, &post, &machine).unwrap().within());

    // Where nothing turns (a 3-axis machine), the coolant is written where it always was.
    let flat = Machine::three_axis();
    let level = MachineProgram { operations: vec![with_coolant(indexed_op("top", 3, vec![], contour_path().moves))], ..program.clone() };
    let plain = emit_program(&level, &dialect(), &flat).unwrap();
    assert_eq!(emit_program(&level, &post, &flat).unwrap(), plain);
    assert!(plain.contains("\nG54\nM8\nG0 X0. Y-3.\n"), "{plain}");

    // The switch is data: written only when on, read back as it was.
    let json = serde_json::to_string(&post).unwrap();
    assert!(json.contains("\"coolant_after_turn\":true"), "{json}");
    assert_eq!(serde_json::from_str::<Post5>(&json).unwrap(), post);
    assert!(!serde_json::to_string(&dialect()).unwrap().contains("coolant_after_turn"));
    assert!(!serde_json::to_string(&Post5::mach3_4axis()).unwrap().contains("coolant_after_turn"));
}
