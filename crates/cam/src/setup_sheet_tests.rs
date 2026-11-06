//! Tests of setup sheets: the formats by extension, numbers and escaping, work zero and the
//! rotary convention in words, how low a toolpath reaches as written, and the pages themselves.

use std::f64::consts::{FRAC_PI_2, PI};

use solidcore_record::Vec3;

use crate::kinematics::{AxisLetter, Carrier, Machine, RotaryAxis};
use crate::multiaxis::{Move5, Pose, Toolpath5};
use crate::setup_sheet::*;

#[test]
fn the_format_comes_from_the_extension() {
    assert_eq!(SheetFormat::from_path("C:/jobs/part.html").ok(), Some(SheetFormat::Html));
    assert_eq!(SheetFormat::from_path("part.HTM").ok(), Some(SheetFormat::Html));
    assert_eq!(SheetFormat::from_path("/x/y/part.md").ok(), Some(SheetFormat::Markdown));
    assert_eq!(SheetFormat::from_path("part.Markdown").ok(), Some(SheetFormat::Markdown));
    for bad in ["", "part", "part.txt", "part.html.exe", "C:/a.html/part", "C:\\a.md\\part", ".", "part."] {
        assert!(SheetFormat::from_path(bad).is_err(), "{bad}");
    }
}

#[test]
fn numbers_are_short_and_never_minus_zero() {
    assert_eq!(fmt_num(1.5, 3), "1.5");
    assert_eq!(fmt_num(12.0, 3), "12");
    assert_eq!(fmt_num(-0.0001, 3), "0");
    assert_eq!(fmt_num(1234.6, 0), "1235");
    assert_eq!(fmt_num(-2.26, 1), "-2.3");
    assert_eq!(fmt_num(f64::NAN, 3), "-");
    assert_eq!(fmt_num(f64::INFINITY, 0), "-");
    assert_eq!(fmt_num(1e300, 0).len(), 301);
}

#[test]
fn hostile_text_is_escaped() {
    let nasty = "<script>alert('x')</script> & \"q\"\n\u{0}|*_`[a](b)#~\\";
    let h = escape_html(nasty);
    assert!(!h.contains('<') && !h.contains('>') && !h.contains('"') && !h.contains('\'') && !h.contains('\n') && !h.contains('\u{0}'), "{h}");
    assert!(h.contains("&lt;script&gt;") && h.contains("&amp;"), "{h}");
    let m = escape_markdown(nasty);
    assert!(!m.contains('<') && !m.contains('\n') && !m.contains('\u{0}'), "{m}");
    // Every character Markdown gives a meaning follows a backslash.
    let chars: Vec<char> = m.chars().collect();
    for (i, c) in chars.iter().enumerate() {
        if matches!(c, '|' | '*' | '_' | '`' | '[' | ']' | '#' | '~') {
            assert_eq!(i.checked_sub(1).and_then(|j| chars.get(j)), Some(&'\\'), "{m}");
        }
    }
    assert_eq!(escape_markdown("plain 1.5 - 2"), "plain 1.5 - 2");
}

#[test]
fn work_zero_is_put_in_words() {
    let z = work_zero(Vec3::new(1.0, 2.0, 3.0), Vec3::new(-20.0, -20.0, -20.0), Vec3::new(20.0, 20.0, 7.0), None, None);
    let all = z.words.join(" ");
    assert!(all.contains("design point (1, 2, 3)"), "{all}");
    assert!(all.contains("X0 is in the middle of the part in X"), "{all}");
    assert!(all.contains("Y0 is in the middle of the part in Y"), "{all}");
    assert!(all.contains("Z0 is inside the part's extent, 20 mm from its bottom and 7 mm from its top"), "{all}");
    assert!(all.contains("40 x 40 x 27 mm"), "{all}");
    // On a corner, on the top; and away from the part.
    let z = work_zero(Vec3::ZERO, Vec3::new(0.0, 0.0, -10.0), Vec3::new(50.0, 30.0, 0.0), None, None);
    let all = z.words.join(" ");
    assert!(
        all.contains("X0 is on the part's left side") && all.contains("Y0 is on the part's front side") && all.contains("Z0 is on the part's top"),
        "{all}"
    );
    let z = work_zero(Vec3::ZERO, Vec3::new(5.0, -30.0, 2.0), Vec3::new(50.0, -10.0, 8.0), None, None);
    let all = z.words.join(" ");
    assert!(all.contains("X0 is 5 mm left of the part's left side"), "{all}");
    assert!(all.contains("Y0 is 10 mm behind the part's back side"), "{all}");
    assert!(all.contains("Z0 is 2 mm below the part's bottom"), "{all}");
    // On a 4th axis, Y0 Z0 is the axis.
    let z = work_zero(Vec3::ZERO, Vec3::new(-20.0, -10.0, -10.0), Vec3::new(20.0, 10.0, 10.0), Some('A'), Some(10.0));
    let all = z.words.join(" ");
    assert!(all.contains("Y0 Z0 is on the A axis centreline") && all.contains("reaches 10 mm from the A axis"), "{all}");
    assert!(!all.contains("Y0 is"), "{all}");
    assert_eq!(z.part_min, [-20.0, -10.0, -10.0]);
}

#[test]
fn the_a_axis_convention_is_put_in_words() {
    let a = Machine::four_axis_a();
    let note = rotary_note(&a.rotaries[0]);
    let all = note.words.join(" ");
    assert_eq!(note.letter, "A");
    assert!(all.contains("about +X through work zero"), "{all}");
    assert!(all.contains("counter-clockwise seen from the +X end"), "{all}");
    assert!(all.contains("A+90° brings the part's +Y side up"), "{all}");
    assert!(all.contains("top of the part moves towards -Y (towards the front"), "{all}");
    assert!(all.contains("no travel limits"), "{all}");
    assert!(note.limits_deg.is_none());
    // Reversed, and with limits: everything the other way round.
    let r = Machine::four_axis_a_with(Vec3::ZERO, true, Some((-2.0 * PI, 2.0 * PI)));
    let note = rotary_note(&r.rotaries[0]);
    let all = note.words.join(" ");
    assert!(all.contains("turns the table clockwise") && all.contains("reversed"), "{all}");
    assert!(all.contains("A+90° brings the part's -Y side up"), "{all}");
    assert!(all.contains("towards +Y"), "{all}");
    assert!(all.contains("from -360° to 360°"), "{all}");
    assert_eq!(note.limits_deg, Some([-360.0, 360.0]));
    // An axis off the coordinate directions, on the head, says so in numbers.
    let tilted = RotaryAxis::new(AxisLetter::B, Vec3::new(0.0, 1.0, 1.0), Vec3::new(0.0, 0.0, 100.0), Carrier::Head, None);
    let all = rotary_note(&tilted).words.join(" ");
    assert!(all.contains("tilts the spindle head about [0, 0.7071, 0.7071] through (0, 0, 100)"), "{all}");
}

#[test]
fn rotary_angles_are_one_angle_or_a_range() {
    let a = vec!["A".to_string()];
    assert_eq!(rotary_words(&a, &[(FRAC_PI_2, FRAC_PI_2)]), "A90°");
    assert_eq!(rotary_words(&a, &[(-PI, PI)]), "A -180° to 180°");
    assert_eq!(rotary_words(&a, &[(f64::INFINITY, f64::NEG_INFINITY)]), "");
    assert_eq!(rotary_words(&[], &[]), "");
    assert_eq!(rotary_words(&a, &[]), "");
}

fn path(poses: &[(Vec3, Vec3)]) -> Toolpath5 {
    let mut moves = Vec::new();
    for (i, (tip, axis)) in poses.iter().enumerate() {
        let to = Pose::new(*tip, *axis);
        moves.push(if i == 0 { Move5::Rapid { to } } else { Move5::Feed { to, feed: 500.0 } });
    }
    Toolpath5 { moves }
}

#[test]
fn reach_is_measured_as_the_program_writes_it() {
    // On the 4th axis the program's Z of a tip on the +Y side with the tool along +Y is its
    // distance from the axis, and A turns to 90°.
    let p = path(&[(Vec3::new(0.0, 0.0, 20.0), Vec3::Z), (Vec3::new(3.0, 10.0, 0.0), Vec3::Y), (Vec3::new(3.0, 12.0, 0.0), Vec3::Y)]);
    let a = Machine::four_axis_a();
    let r = reach(&a, &p, false, None).unwrap().unwrap();
    assert!((r.z_min - 10.0).abs() < 1e-9, "{r:?}");
    assert_eq!(r.rotary.len(), 1);
    assert!(r.rotary[0].0.abs() < 1e-9 && (r.rotary[0].1 - FRAC_PI_2).abs() < 1e-9, "{r:?}");
    assert!(r.last.is_some());
    // With tool centre point control the program writes the tip in work coordinates.
    let r = reach(&a, &p, true, None).unwrap().unwrap();
    assert!(r.z_min.abs() < 1e-9, "{r:?}");
    // A reversed table turns to -90° for the same pose.
    let rev = Machine::four_axis_a_with(Vec3::ZERO, true, None);
    let r = reach(&rev, &p, false, None).unwrap().unwrap();
    assert!((r.rotary[0].0 + FRAC_PI_2).abs() < 1e-9, "{r:?}");
    // 3-axis: the lowest tip.
    let flat = path(&[(Vec3::new(0.0, 0.0, 5.0), Vec3::Z), (Vec3::new(1.0, 1.0, -3.5), Vec3::Z)]);
    let r = reach(&Machine::three_axis(), &flat, false, None).unwrap().unwrap();
    assert!((r.z_min + 3.5).abs() < 1e-12 && r.rotary.is_empty(), "{r:?}");
    // Nothing to reach, and hostile poses, are no panic.
    assert!(reach(&Machine::three_axis(), &Toolpath5 { moves: vec![Move5::Dwell { seconds: 1.0 }] }, false, None).unwrap().is_none());
    let bad = path(&[(Vec3::new(f64::NAN, 0.0, 0.0), Vec3::Z)]);
    assert!(reach(&Machine::three_axis(), &bad, false, None).is_err());
    let bad = path(&[(Vec3::ZERO, Vec3::ZERO)]);
    assert!(reach(&a, &bad, false, None).is_err());
    let down = path(&[(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0))]);
    assert!(reach(&Machine::three_axis(), &down, false, None).is_err());
}

fn sample(name: &str) -> SetupSheet {
    SetupSheet {
        job: name.into(),
        machine: "4-axis A".into(),
        post: "mach3_4axis".into(),
        program_file: Some("part.tap".into()),
        work_offset: "G54".into(),
        work_zero: work_zero(Vec3::ZERO, Vec3::new(-20.0, -10.0, -10.0), Vec3::new(20.0, 10.0, 10.0), Some('A'), Some(10.0)),
        rotaries: vec![rotary_note(&Machine::four_axis_a().rotaries[0])],
        stock: None,
        tools: vec![
            SheetTool {
                number: 1,
                kind: "flat end mill".into(),
                description: format!("6 mm {name}"),
                diameter: 6.0,
                flutes: 3,
                flute_length: 15.0,
                stickout: Some(25.0),
                holder: vec![[34.0, 4.0], [34.0, 18.0]],
                z_min: Some(-2.5),
                operations: vec![format!("rough {name}")],
                ..SheetTool::default()
            },
            SheetTool {
                number: 2,
                kind: "ball end mill".into(),
                diameter: 4.0,
                flutes: 2,
                flute_length: 12.0,
                z_min: Some(8.0),
                operations: vec!["barrel".into()],
                ..SheetTool::default()
            },
        ],
        operations: vec![
            SheetOperation {
                name: format!("rough {name}"),
                kind: "indexed pocket".into(),
                tool: 1,
                spindle_rpm: 18000.0,
                feed: 800.0,
                plunge_feed: 200.0,
                rotary: "A90°".into(),
                cut_length: 1234.6,
                cut_minutes: 1.54,
                z_min: Some(-2.5),
                gouge_checked: true,
                ..SheetOperation::default()
            },
            SheetOperation {
                name: "barrel".into(),
                kind: "rotary_finish".into(),
                tool: 2,
                spindle_rpm: 24000.0,
                feed: 900.0,
                plunge_feed: 250.0,
                coolant: true,
                rotary: "A -180° to 180°".into(),
                cut_length: 3000.0,
                cut_minutes: 3.33,
                z_min: Some(8.0),
                gouge_checked: true,
            },
        ],
        totals: Totals {
            operations: 2,
            tools: 2,
            tool_changes: 2,
            cut_length: 4234.5,
            cut_minutes: 4.87,
            program_minutes: Some(4.9),
            program_lines: 1000,
        },
        extents: Some(ProgramExtents { min: [-20.0, -10.0, -2.5], max: [20.0, 10.0, 30.0], rotary: vec![("A".into(), -180.0, 180.0)] }),
        checks: vec![
            SheetCheck { name: "Read-back".into(), passed: true, detail: "within 0.004 mm".into() },
            SheetCheck { name: "Gouges".into(), passed: false, detail: format!("{name} cuts 1 mm in") },
        ],
        warnings: vec![format!("{name} is not checked")],
        notes: vec!["tessellated".into()],
    }
}

#[test]
fn the_html_page_is_self_contained_and_escaped() {
    let nasty = "<script>alert(1)</script><img src=x onerror=alert(2)>";
    let s = sample(nasty);
    let h = s.html();
    assert!(h.starts_with("<!DOCTYPE html>") && h.ends_with("</html>\n"));
    assert!(!h.contains("<script") && !h.contains("<img") && !h.contains("http://") && !h.contains("https://"), "{h}");
    assert!(h.contains("&lt;script&gt;alert(1)&lt;/script&gt;"), "{h}");
    assert!(h.contains("@media print"), "print styles");
    for want in
        ["Work zero", "A axis", "Tools", "Operations", "Checks", "Warnings", "T1", "T2", "Ø34 x 4 + Ø34 x 18", "A90°", "FAILED", "18000", "G54"]
    {
        assert!(h.contains(want), "{want}: {h}");
    }
    assert!(!h.contains("Stock"), "no stock section without a stock");
    // The same sheet is the same bytes.
    assert_eq!(h, sample(nasty).html());
    let with_stock = SetupSheet { stock: Some("bar Ø25 x 60 <6061>".into()), ..sample("p") };
    assert!(with_stock.html().contains("bar Ø25 x 60 &lt;6061&gt;"));
}

#[test]
fn the_markdown_sheet_keeps_its_tables_whole() {
    let s = sample("a | b\nc");
    let m = s.markdown();
    assert!(m.starts_with("# Setup sheet: a \\| b c\n"), "{m}");
    // Every table row has as many cells as its header.
    let tools = m.lines().filter(|l| l.strip_prefix("| T").is_some_and(|r| r.starts_with(|c: char| c.is_ascii_digit()))).collect::<Vec<_>>();
    assert_eq!(tools.len(), 2, "{m}");
    for row in m.lines().filter(|l| l.starts_with('|')) {
        let cells = row.replace("\\|", "").matches('|').count();
        assert!(cells == 12 || cells == 13 || cells == 4, "{row}");
    }
    assert!(m.contains("| 1 | rough a \\| b c | indexed pocket | T1 | 18000 | 800 | 200 | off | A90° | -2.5 | 1235 | 1.5 |"), "{m}");
    assert_eq!(m, sample("a | b\nc").render(SheetFormat::Markdown));
    assert_eq!(s.render(SheetFormat::Html), s.html());
}
