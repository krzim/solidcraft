//! Tests of machine time: programs timed against a machine's rates, the warnings for blocks
//! the controller slows, and the dwell unit a post writes.

use solidcore_record::Vec3;

use crate::foundation_tests::close;
use crate::gcode5::{MachineOperation, MachineProgram, Post5, emit_program};
use crate::indexed::IndexedOperation;
use crate::kinematics::{AxisLetter, Machine};
use crate::machine_time::*;
use crate::readback5::readback_program;
use crate::*;

/// A small mill: 8000 mm/min across, 4000 up and down, A at 8000°/min, the spindle from 10000
/// to 24000 rpm with a six-second start.
fn rates() -> MachineRates {
    MachineRates {
        rapid_xyz_mm_min: 8000.0,
        rapid_z_mm_min: Some(4000.0),
        rapid_a_deg_min: Some(8000.0),
        max_feed_mm_min: Some(8000.0),
        max_feed_z_mm_min: Some(4000.0),
        max_a_deg_min: Some(8000.0),
        spindle_rpm: Some([10000.0, 24000.0]),
        spindle_start_seconds: Some(6.0),
        travel_mm: Some([300.0, 170.0, 130.0]),
        ..MachineRates::default()
    }
}

const PROGRAM: &str = "\
G90 G94 G91.1 G17 G21
M3 S5000
G0 X0. Y0. Z10. A0.
G0 X100.
G0 Z0.
G1 X110. F1000.
G93 G1 A90. F100.
G94 G1 X200. F12000.
G4 P1.5
G3 X210. Y10. I0. J10. F1000.
M5
M3 S12000
M30
";

fn line_of(text: &str, start: &str) -> usize {
    text.lines().position(|l| l.starts_with(start)).map_or(0, |i| i + 1)
}

#[test]
fn programs_are_timed_as_the_machine_runs_them() {
    let a = [AxisLetter::A];
    let t = machine_time(PROGRAM, &a, &rates(), DwellUnit::Seconds).unwrap();
    // Rapids: 100 mm across at 8000, 10 mm down at 4000; the first only places the axes.
    close(t.rapid_minutes, 100.0 / 8000.0 + 10.0 / 4000.0, 1e-12);
    // As written: the cut, the inverse-time turn, the fast cut, the dwell and the arc.
    let quarter = 10.0 * std::f64::consts::FRAC_PI_2;
    let written = 10.0 / 1000.0 + 1.0 / 100.0 + 90.0 / 12000.0 + 1.5 / 60.0 + quarter / 1000.0;
    close(t.written_cut_minutes, written, 1e-9);
    // As run: A held to 8000°/min, X to 8000 mm/min.
    let run = 10.0 / 1000.0 + 90.0 / 8000.0 + 90.0 / 8000.0 + 1.5 / 60.0 + quarter / 1000.0;
    close(t.cut_minutes, run, 1e-9);
    assert_eq!((t.slowed_blocks, t.rotary_over, t.feed_over), (2, 1, 1));
    assert_eq!(t.peak_rotary_deg_min.map(|(l, _)| l), Some(line_of(PROGRAM, "G93")));
    close(t.peak_rotary_deg_min.map_or(0.0, |(_, r)| r), 9000.0, 1e-6);
    assert_eq!(t.peak_feed_mm_min.map(|(l, _)| l), Some(line_of(PROGRAM, "G94 G1 X200")));
    // Two starts of the spindle, six seconds each; the first at a speed below its range.
    close(t.spindle_minutes, 12.0 / 60.0, 1e-12);
    assert_eq!(t.spindle_out_of_range, vec![(2, 5000.0)]);
    close(t.total_minutes, t.cut_minutes + t.rapid_minutes + t.spindle_minutes, 1e-12);
    assert_eq!(t.span_mm, [210.0, 10.0, 10.0]);
    let all = t.warnings.join("\n");
    assert_eq!(t.warnings.len(), 3, "{all}");
    assert!(all.contains("ask A to turn faster than its 8000°/min (up to 9000°/min at line 7)"), "{all}");
    assert!(all.contains("more than its top feed (up to 12000 mm/min at line 8)"), "{all}");
    assert!(all.contains("S5000 at line 2"), "{all}");

    // Less travel than the program spans.
    let short = MachineRates { travel_mm: Some([200.0, 170.0, 130.0]), ..rates() };
    let t = machine_time(PROGRAM, &a, &short, DwellUnit::Seconds).unwrap();
    assert!(t.warnings.iter().any(|w| w.contains("spans 210 mm along X, more than the machine's 200 mm")), "{:?}", t.warnings);
    // Without top rates nothing is slowed; without an A rapid rate a turn at rapid is untimed.
    let bare = MachineRates { rapid_xyz_mm_min: 8000.0, ..MachineRates::default() };
    let t = machine_time(PROGRAM, &a, &bare, DwellUnit::Seconds).unwrap();
    close(t.cut_minutes, t.written_cut_minutes, 1e-12);
    assert_eq!(t.slowed_blocks, 0);
    close(t.rapid_minutes, 110.0 / 8000.0, 1e-12);
    let turn = machine_time("G0 X0 Y0 Z0 A0\nG0 A90\n", &a, &bare, DwellUnit::Seconds).unwrap();
    assert!(turn.warnings.iter().any(|w| w.contains("1 rapids turn A, which is not timed")), "{:?}", turn.warnings);
    // A dwell written in milliseconds.
    let ms = machine_time(PROGRAM, &a, &rates(), DwellUnit::Milliseconds).unwrap();
    close(t.written_cut_minutes - ms.written_cut_minutes, 1.5 / 60.0 - 1.5e-3 / 60.0, 1e-9);
    // Nothing to time.
    let empty = machine_time("", &a, &rates(), DwellUnit::Seconds).unwrap();
    assert_eq!((empty.total_minutes, empty.span_mm, empty.warnings.len()), (0.0, [0.0; 3], 0));
}

#[test]
fn hostile_rates_and_programs_are_refused() {
    let a = [AxisLetter::A];
    let bad = [
        MachineRates { rapid_xyz_mm_min: 0.0, ..rates() },
        MachineRates { rapid_xyz_mm_min: f64::NAN, ..rates() },
        MachineRates { rapid_xyz_mm_min: f64::INFINITY, ..rates() },
        MachineRates { rapid_z_mm_min: Some(-1.0), ..rates() },
        MachineRates { rapid_a_deg_min: Some(f64::NAN), ..rates() },
        MachineRates { max_feed_mm_min: Some(1e9), ..rates() },
        MachineRates { max_feed_z_mm_min: Some(0.0), ..rates() },
        MachineRates { max_a_deg_min: Some(f64::NEG_INFINITY), ..rates() },
        MachineRates { spindle_rpm: Some([24000.0, 10000.0]), ..rates() },
        MachineRates { spindle_rpm: Some([f64::NAN, 10000.0]), ..rates() },
        MachineRates { spindle_rpm: Some([-1.0, 10000.0]), ..rates() },
        MachineRates { spindle_start_seconds: Some(-1.0), ..rates() },
        MachineRates { spindle_start_seconds: Some(f64::NAN), ..rates() },
        MachineRates { travel_mm: Some([300.0, 0.0, 130.0]), ..rates() },
        MachineRates { travel_mm: Some([300.0, f64::INFINITY, 130.0]), ..rates() },
    ];
    for r in &bad {
        assert!(r.validate().is_err(), "{r:?}");
        assert!(machine_time(PROGRAM, &a, r, DwellUnit::Seconds).is_err(), "{r:?}");
    }
    assert!(rates().validate().is_ok());
    for text in ["G0 X1.2.3", "G1 X1 Y1 Z1 A1 F0", "G0 X1 Y1 Z1 B1", "G91 G0 X1", "G93 G2 X1 Y1 I1 J0 F1", "G0 X(", "\u{0}\u{7f}é"] {
        assert!(machine_time(text, &a, &rates(), DwellUnit::Seconds).is_err(), "{text}");
    }
    // Unknown keys in a machine's rates are refused.
    let json = r#"{"rapid_xyz_mm_min": 8000, "rapid_xyz": 1}"#;
    assert!(serde_json::from_str::<MachineRates>(json).is_err());
    let back: MachineRates = serde_json::from_str(&serde_json::to_string(&rates()).unwrap()).unwrap();
    assert_eq!(back, rates());
}

/// A plunge with a dwell at the bottom, written through `post` and read back.
fn dwell_readback(post: &Post) -> (String, Readback) {
    let tool = Tool::new(1, ToolKind::Drill, 5.0, 20.0);
    let path = Toolpath {
        moves: vec![
            Move::Rapid { to: Vec3::new(0.0, 0.0, 5.0) },
            Move::Feed { to: Vec3::new(0.0, 0.0, -2.0), feed: 100.0 },
            Move::Dwell { seconds: 2.5 },
            Move::Feed { to: Vec3::new(0.0, 0.0, 5.0), feed: 100.0 },
        ],
    };
    let program = Program {
        name: "hole".into(),
        operations: vec![Operation { name: "hole".into(), tool, spindle_rpm: 12000.0, coolant: false, path }],
        work_offset: None,
    };
    let text = emit(&program, post).unwrap();
    let rb = readback(&text, &program, post).unwrap();
    (text, rb)
}

#[test]
fn dwells_are_read_in_the_unit_the_post_writes() {
    // In seconds, as a controller set to read `G4 P` in seconds wants it.
    let seconds = Post { dwell: "G4 P{seconds}".into(), trailing_decimal: true, ..Post::rs274() };
    assert_eq!(DwellUnit::of(&seconds).unwrap(), DwellUnit::Seconds);
    let (text, rb) = dwell_readback(&seconds);
    assert!(text.contains("\nG4 P2.5\n"), "{text}");
    close(rb.cut_minutes, rb.planned_cut_minutes, 1e-9);
    // In milliseconds, for a controller set to read it so: the read-back times it the same.
    let millis = Post { dwell: "G4 P{milliseconds}".into(), ..Post::rs274() };
    assert_eq!(DwellUnit::of(&millis).unwrap(), DwellUnit::Milliseconds);
    let (text, rb) = dwell_readback(&millis);
    assert!(text.contains("\nG4 P2500\n"), "{text}");
    close(rb.cut_minutes, rb.planned_cut_minutes, 1e-9);
    // A template that does not say how long, or says it twice, is refused.
    for dwell in ["G4 P1", "G4 P{seconds} (P{milliseconds})", ""] {
        let post = Post { dwell: dwell.into(), ..Post::rs274() };
        assert!(post.validate().is_err(), "{dwell}");
        assert!(DwellUnit::of(&post).is_err(), "{dwell}");
    }
    // The 4th-axis Mach3 post writes seconds.
    assert_eq!(DwellUnit::of(&Post5::mach3_4axis().base).unwrap(), DwellUnit::Seconds);
}

#[test]
fn a_multi_axis_program_with_a_dwell_reads_back_in_milliseconds() {
    let tool = Tool::new(1, ToolKind::FlatEndmill, 6.0, 20.0);
    let path = Toolpath {
        moves: vec![
            Move::Rapid { to: Vec3::new(0.0, 0.0, 30.0) },
            Move::Feed { to: Vec3::new(0.0, 0.0, 18.0), feed: 200.0 },
            Move::Dwell { seconds: 0.5 },
            Move::Feed { to: Vec3::new(0.0, 0.0, 30.0), feed: 200.0 },
        ],
    };
    let op = IndexedOperation { name: "spot".into(), tool, spindle_rpm: 12000.0, coolant: true, rotary: vec![90f64.to_radians()], path };
    let program = MachineProgram { name: "spot".into(), operations: vec![MachineOperation::Indexed(op)], work_offset: None };
    let machine = Machine::four_axis_a();
    for (dwell, written) in [("G4 P{seconds}", "G4 P0.5"), ("G4 P{milliseconds}", "G4 P500")] {
        let mut post = Post5::mach3_4axis();
        post.base.dwell = dwell.into();
        let text = emit_program(&program, &post, &machine).unwrap();
        assert!(text.lines().any(|l| l == written), "{text}");
        let rb = readback_program(&text, &program, &post, &machine).unwrap();
        assert!(rb.within(), "{rb:?}");
        close(rb.cut_minutes, rb.planned_cut_minutes, 1e-9);
        let t = machine_time(&text, &machine.letters(), &rates(), DwellUnit::of(&post.base).unwrap()).unwrap();
        close(t.written_cut_minutes, rb.cut_minutes, 1e-9);
    }
}

#[test]
fn rotary_rates_are_checked_outside_inverse_time_too() {
    // A post without inverse time writes a turn of A alone at an `F` in degrees a minute, and a
    // cut that moves and turns at an `F` along the linear axes: both can ask A for more than it
    // can turn, the controller slows them just the same, and the reply must say so.
    let a = [AxisLetter::A];
    let text = "G90 G94 G17 G21\nG0 X0. Y0. Z10. A0.\nG1 A90. F12000.\nG1 X10. A270. F1000.\nM30\n";
    let t = machine_time(text, &a, &rates(), DwellUnit::Seconds).unwrap();
    assert_eq!((t.slowed_blocks, t.rotary_over, t.feed_over), (2, 2, 0), "{t:?}");
    // The second asks for 180° in 0.01 minutes: 18000°/min.
    assert_eq!(t.peak_rotary_deg_min.map(|(l, _)| l), Some(4), "{t:?}");
    close(t.peak_rotary_deg_min.map_or(0.0, |(_, r)| r), 18000.0, 1e-6);
    close(t.cut_minutes, 90.0 / 8000.0 + 180.0 / 8000.0, 1e-12);
    let all = t.warnings.join("\n");
    assert!(all.contains("2 cutting blocks (the first at line 3) ask A to turn faster than its 8000°/min (up to 18000°/min at line 4)"), "{all}");
}

#[test]
fn the_feed_warning_names_a_block_that_is_slowed() {
    // A plunge at 5000 mm/min is slowed to Z's 4000; a diagonal at 7000 mm/min is not (X and Y
    // each move at 4950). The warning must point at the plunge, not at the faster diagonal.
    let a = [AxisLetter::A];
    let text = "G90 G94 G17 G21\nG0 X0. Y0. Z10. A0.\nG1 Z0. F5000.\nG1 X100. Y100. F7000.\nM30\n";
    let t = machine_time(text, &a, &rates(), DwellUnit::Seconds).unwrap();
    assert_eq!((t.slowed_blocks, t.feed_over), (1, 1), "{t:?}");
    // The highest feed asked for is still reported as it is.
    assert_eq!(t.peak_feed_mm_min.map(|(l, _)| l), Some(4), "{t:?}");
    let all = t.warnings.join("\n");
    assert!(
        all.contains("1 cutting blocks (the first at line 3) ask a linear axis for more than its top feed (up to 5000 mm/min at line 3)"),
        "{all}"
    );
}

#[test]
fn a_long_move_speeds_up_and_slows_down() {
    // 1000 mm at 100 mm/s, 2000 mm/s²: 2.5 mm to speed up and 2.5 to stop, 0.05 s each.
    close(trapezoid(1000.0, 0.0, 100.0, 0.0, 2000.0), 0.05 + 0.05 + 995.0 / 100.0, 1e-9);
    // Too short to reach the speed: a triangle, peak √(a·l).
    close(trapezoid(2.0, 0.0, 100.0, 0.0, 2000.0), 2.0 * (2000.0f64 * 2.0).sqrt() / 2000.0, 1e-9);
    assert_eq!(trapezoid(0.0, 0.0, 100.0, 0.0, 2000.0), 0.0);
    close(trapezoid(10.0, 0.0, 100.0, 0.0, f64::INFINITY), 0.1, 1e-12);
}

#[test]
fn blocks_are_timed_with_acceleration_when_the_rates_give_it() {
    let program = "G21\nT1 M6\nS12000 M3\nG0 X0. Y0. Z0.\nG1 X1000. F6000.\nG4 P2.\nM5\n";
    let plain =
        MachineRates { rapid_xyz_mm_min: 6000.0, spindle_start_seconds: Some(6.0), tool_change_seconds: Some(20.0), ..MachineRates::default() };
    let t = block_times(program, &[], &plain, DwellUnit::Seconds).unwrap();
    assert!(!t.accelerated);
    assert_eq!(t.lines, [4, 5, 6]);
    // The first block only says where the axes are; 1000 mm at 6000 mm/min is 10 s; the dwell 2.
    assert_eq!(t.seconds, [0.0, 10.0, 2.0]);
    assert_eq!(t.waits.iter().map(|w| (w.line, w.kind)).collect::<Vec<_>>(), [(2, WaitKind::ToolChange), (3, WaitKind::SpindleStart)]);
    close(t.total, 10.0 + 2.0 + 6.0 + 20.0, 1e-9);
    // With 2000 mm/s²: the hand calculation, 10.05 s.
    let accel = MachineRates { accel_mm_s2: Some(2000.0), ..plain.clone() };
    let t = block_times(program, &[], &accel, DwellUnit::Seconds).unwrap();
    assert!(t.accelerated);
    close(t.seconds[1], 10.05, 1e-9);
    // A square of short moves takes longer with it than without; a straight line of them does
    // not stop at their joins.
    let square = "G21\nG0 X0. Y0. Z0.\nG1 X5. F6000.\nY5.\nX0.\nY0.\n";
    let line = "G21\nG0 X0. Y0. Z0.\nG1 X5. F6000.\nX10.\nX15.\nX20.\n";
    let sum = |text: &str, r: &MachineRates| block_times(text, &[], r, DwellUnit::Seconds).unwrap().total;
    assert!(sum(square, &accel) > sum(square, &plain) + 0.1);
    close(sum(line, &accel), trapezoid(20.0, 0.0, 100.0, 0.0, 2000.0), 1e-9);
    // Without acceleration the cut time is the one `machine_time` gives.
    let mt = machine_time(PROGRAM, &[AxisLetter::A], &rates(), DwellUnit::Seconds).unwrap();
    let bt = block_times(PROGRAM, &[AxisLetter::A], &rates(), DwellUnit::Seconds).unwrap();
    close(bt.total, (mt.total_minutes) * 60.0, 1e-6);
    // Hostile accelerations are refused.
    for bad in [Some(0.0), Some(-1.0), Some(f64::NAN), Some(1e9)] {
        assert!(MachineRates { accel_mm_s2: bad, ..plain.clone() }.validate().is_err());
    }
    assert!(MachineRates { accel_z_mm_s2: Some(100.0), ..plain.clone() }.validate().is_err(), "Z's acceleration alone");
    assert!(MachineRates { tool_change_seconds: Some(-1.0), ..plain }.validate().is_err());
}

#[test]
fn returns_home_are_timed_up_to_the_home_and_back_down() {
    // A rewind as the post writes one: out of the cut, Z home (G28), A back two turns, and down
    // into the cut again. The trip to Z's home and back is time at rapid too.
    let text = "\
G90 G94 G17 G21
G0 X0. Y0. Z5. A0.
G1 Z-1. F100.
G28 G91 Z0.
G90
G0 A-720.
G0 X0. Y0.
G0 Z5.
G1 Z-1. F100.
";
    let a = [AxisLetter::A];
    // Home 50 mm above work zero: 51 up at 4000, then down 45 while A turns 720° at 8000°/min.
    let home = MachineRates { home_z_mm: Some(50.0), ..rates() };
    let t = machine_time(text, &a, &home, DwellUnit::Seconds).unwrap();
    close(t.rapid_minutes, 51.0 / 4000.0 + (45.0f64 / 4000.0).max(720.0 / 8000.0), 1e-12);
    assert_eq!(t.home_returns, 1);
    // Without the home's height, at least up to the highest Z the program reaches (5).
    let t = machine_time(text, &a, &rates(), DwellUnit::Seconds).unwrap();
    close(t.rapid_minutes, 6.0 / 4000.0 + 720.0 / 8000.0, 1e-12);
    assert_eq!(t.home_returns, 1);
    // A home below the program, or not finite, is refused.
    for z in [f64::NAN, f64::INFINITY, -1e9] {
        assert!(machine_time(text, &a, &MachineRates { home_z_mm: Some(z), ..rates() }, DwellUnit::Seconds).is_err(), "{z}");
    }
}
