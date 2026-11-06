//! The setup sheet of `cam.job` (docs/cam-jobs.md, "Setup sheet"): what the machine operator
//! needs to set the job up, built from the planned operations, the written program and the
//! checks, and the reply's `summary` with the same tables in compact form. The sheet itself is
//! the CAM crate's [`SetupSheet`]; this fills it in. Nothing on it depends on the date or on
//! where it is made, so the same job gives the same bytes.

use std::path::Path;

use solidcore_record::Vec3;
use solidcraft_cam::gcode5::{JointWords, MachineOperation, Post5, parse5};
use solidcraft_cam::kinematics::{Carrier, Joints, Machine};
use solidcraft_cam::setup_sheet::{
    ProgramExtents, SetupSheet, SheetCheck, SheetFormat, SheetOperation, SheetTool, Totals, fmt_num, reach, rotary_note, rotary_words, work_zero,
};
use solidcraft_cam::{Surface, work_offset_code};
use solidcraft_geom::prelude::*;

use super::{Planned, num};
use crate::Result;
use crate::params::bad;
use crate::schema::cam::{GougeReport, ReadbackReport};
use crate::schema::cam_sheet::{SetupSummary, SummaryOperation, SummaryTool, SummaryTotals};

/// Largest angle a rotary turns to before the sheet warns (degrees): a full turn either way.
const FULL_TURN_DEG: f64 = 360.0;

/// The format of the sheet at `path`, refused before anything is computed when it is neither
/// HTML nor Markdown.
pub(super) fn format_of(cmd: &str, path: Option<&str>) -> Result<Option<SheetFormat>> {
    match path {
        None => Ok(None),
        Some(p) => SheetFormat::from_path(p.trim()).map(Some).map_err(|_| bad(cmd, format!("`setup_sheet` {p}: give a .html or .md file"))),
    }
}

/// Refuses a setup sheet written over the G-code or the preview.
pub(super) fn distinct(cmd: &str, output: Option<&str>, preview: Option<&str>, sheet: Option<&str>) -> Result<()> {
    let Some(sheet) = sheet else { return Ok(()) };
    let same = |other: Option<&str>| other.is_some_and(|o| Path::new(o) == Path::new(sheet));
    if same(output) || same(preview) {
        return Err(bad(cmd, "`setup_sheet` must be a file of its own, not the `output` or the `preview`"));
    }
    Ok(())
}

/// What the sheet is made from.
pub(super) struct Inputs<'a> {
    pub name: &'a str,
    pub machine: &'a Machine,
    pub post_name: &'a str,
    pub post: &'a Post5,
    pub work_offset: Option<u8>,
    /// The design point that is work zero.
    pub origin: Vec3,
    /// The part in work coordinates.
    pub part: &'a Surface,
    pub planned: &'a [Planned],
    /// The program as written.
    pub text: &'a str,
    pub readback: &'a ReadbackReport,
    pub gouge: &'a GougeReport,
    /// Whether any tool had holder data to check.
    pub holder_checked: bool,
    pub failures: &'a [String],
    pub notes: &'a [String],
    /// The G-code file, when one is written.
    pub program_file: Option<&'a str>,
}

/// The letter of the machine's one rotary when it is a table about work X through work zero
/// (a 4th axis set up the usual way), whose centreline Y0 Z0 is then on.
fn rotary_about_x(machine: &Machine) -> Option<char> {
    let [axis] = machine.rotaries.as_slice() else { return None };
    let d = axis.direction.normalized()?;
    let on_x = (d.x.abs() - 1.0).abs() <= 1e-6;
    let through = axis.center.y.abs() <= 1e-6 && axis.center.z.abs() <= 1e-6 && machine.work_origin.len() <= 1e-6;
    (axis.carrier == Carrier::Table && on_x && through).then(|| axis.letter.char())
}

/// The farthest the part's facets reach from work X, mm.
fn radius_about_x(part: &Surface) -> f64 {
    part.triangles().iter().flatten().map(|p| p.y.hypot(p.z)).fold(0.0, f64::max)
}

/// The work offset the program runs in, in words.
fn work_offset_words(post: &Post5, work_offset: Option<u8>) -> String {
    if !post.base.writes_work_offset() {
        return "not written by the post: the program runs in whichever work offset the controller has active".into();
    }
    work_offset_code(work_offset.unwrap_or(1)).unwrap_or_else(|_| "G54".into())
}

/// The lowest and highest position of every axis the written program reaches, as the reader
/// reads it; `None` when it cannot read the program or it has no motion.
fn extents(text: &str, machine: &Machine) -> Option<ProgramExtents> {
    let letters = machine.letters();
    let blocks = parse5(text, &letters).ok()?;
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut ranges: Vec<(f64, f64)> = vec![(f64::INFINITY, f64::NEG_INFINITY); letters.len()];
    for b in &blocks {
        let (linear, rotary) = match b {
            JointWords::Rapid { linear, rotary, .. } | JointWords::Feed { linear, rotary, .. } | JointWords::Arc { linear, rotary, .. } => {
                (*linear, rotary)
            }
            JointWords::Dwell { .. } => continue,
        };
        lo = lo.min(linear);
        hi = hi.max(linear);
        for (range, a) in ranges.iter_mut().zip(rotary) {
            range.0 = range.0.min(*a);
            range.1 = range.1.max(*a);
        }
    }
    if !(lo.is_finite() && hi.is_finite()) {
        return None;
    }
    let rotary =
        letters.iter().zip(&ranges).filter(|(_, r)| r.0 <= r.1).map(|(l, r)| (l.char().to_string(), r.0.to_degrees(), r.1.to_degrees())).collect();
    Some(ProgramExtents { min: [lo.x, lo.y, lo.z], max: [hi.x, hi.y, hi.z], rotary })
}

/// The setup sheet of a planned and checked job.
pub(super) fn build(i: &Inputs<'_>) -> SetupSheet {
    let machine = i.machine;
    let letters: Vec<String> = machine.letters().iter().map(|l| l.char().to_string()).collect();
    let tcp = i.post.tcp.is_some();
    let mut warnings: Vec<String> = i.failures.iter().map(|f| format!("Failed check: {f}")).collect();

    // The operations in order, each with how low its tip goes and where the rotaries are.
    let mut operations: Vec<SheetOperation> = Vec::with_capacity(i.planned.len());
    let mut previous: Option<Joints> = None;
    for p in i.planned {
        let (rpm, coolant) = match &p.op {
            MachineOperation::Indexed(o) => (o.spindle_rpm, o.coolant),
            MachineOperation::Continuous(o) => (o.spindle_rpm, o.coolant),
        };
        let (z_min, rotary) = match reach(machine, &p.poses, tcp, previous.as_ref()) {
            Ok(Some(r)) => {
                let words = rotary_words(&letters, &r.rotary);
                previous = r.last;
                (Some(r.z_min), words)
            }
            Ok(None) => (None, String::new()),
            Err(e) => {
                warnings.push(format!("operation `{}`: how low it reaches could not be worked out ({e})", p.report.name));
                (None, String::new())
            }
        };
        for w in &p.report.warnings {
            warnings.push(format!("operation `{}`: {w}", p.report.name));
        }
        operations.push(SheetOperation {
            name: p.report.name.clone(),
            kind: p.report.kind.clone(),
            tool: p.tool.number,
            spindle_rpm: rpm,
            feed: p.feeds.cut,
            plunge_feed: p.feeds.plunge,
            coolant,
            rotary,
            cut_length: p.report.cut_length,
            cut_minutes: p.report.cut_minutes,
            z_min,
            gouge_checked: p.gouge_check,
        });
    }

    // The tools the operations use, by number.
    let mut tools: Vec<SheetTool> = Vec::new();
    for (p, o) in i.planned.iter().zip(&operations) {
        let t = &p.tool;
        let row = match tools.iter_mut().position(|r| r.number == t.number) {
            Some(k) => tools.get_mut(k),
            None => {
                tools.push(SheetTool {
                    number: t.number,
                    kind: t.kind.label().into(),
                    description: t.description.clone(),
                    diameter: t.diameter,
                    corner_radius: t.corner_radius.filter(|_| t.kind == solidcraft_cam::ToolKind::BullNose),
                    flutes: t.flutes,
                    flute_length: t.flute_length,
                    shank_diameter: t.shank_diameter,
                    stickout: t.stickout,
                    holder: t.holder.iter().map(|h| [h.diameter, h.length]).collect(),
                    z_min: None,
                    operations: Vec::new(),
                });
                tools.last_mut()
            }
        };
        if let Some(row) = row {
            row.operations.push(o.name.clone());
            row.z_min = match (row.z_min, o.z_min) {
                (Some(a), Some(b)) => Some(a.min(b)),
                (a, b) => a.or(b),
            };
        }
    }
    tools.sort_by_key(|t| t.number);
    for t in &tools {
        if t.holder.is_empty() {
            warnings.push(format!("T{} has no holder data: its shank and holder were not checked against the part", t.number));
        }
    }
    let tool_changes = i.planned.iter().map(|p| p.tool.number).fold((0usize, None), |(n, last), t| (n + usize::from(last != Some(t)), Some(t))).0;

    // The program as written.
    let extents = extents(i.text, machine);
    match &extents {
        None => warnings.push("the written program could not be read for its extents".into()),
        Some(x) => {
            for (l, lo, hi) in &x.rotary {
                if lo.abs() > FULL_TURN_DEG + 1e-6 || hi.abs() > FULL_TURN_DEG + 1e-6 {
                    warnings.push(format!(
                        "the program turns {l} past a full turn (to {}°): check that the controller's {l} axis allows it",
                        fmt_num(if hi.abs() > lo.abs() { *hi } else { *lo }, 3)
                    ));
                }
            }
        }
    }

    // Work zero on the part.
    let bounds = i.part.bounds();
    let about_x = rotary_about_x(machine);
    let zero = work_zero(i.origin, bounds.min, bounds.max, about_x, about_x.map(|_| radius_about_x(i.part)));

    let rb = i.readback;
    let g = i.gouge;
    let mut checks = vec![
        SheetCheck {
            name: "Read-back".into(),
            passed: rb.pass,
            detail: format!(
                "the written program, read back through the machine's kinematics, is within {} mm and {}° of its toolpaths (the post's precision allows {} mm and {}°)",
                num(rb.max_tip_deviation),
                num(rb.max_axis_deviation_deg),
                num(rb.allowed_tip),
                num(rb.allowed_axis_deg)
            ),
        },
        SheetCheck {
            name: "Gouges".into(),
            passed: g.pass,
            detail: {
                let mut d = if g.pass {
                    format!("no cutting move cuts more than {} mm into the part or a fixture", num(g.tolerance))
                } else {
                    format!("cuts up to {} mm into the part or a fixture along {} moves", num(g.worst), g.hits)
                };
                if !g.skipped.is_empty() {
                    d.push_str(&format!("; not checked: {}", g.skipped.join(", ")));
                }
                d
            },
        },
    ];
    if i.holder_checked {
        let hits: usize = i.planned.iter().map(|p| p.report.holder_collisions).sum();
        checks.push(SheetCheck {
            name: "Shank and holder".into(),
            passed: hits == 0,
            detail: if hits == 0 {
                "the shank and holder of every tool with holder data stay clear of the part and fixtures".into()
            } else {
                format!("the shank or holder touches the part or a fixture along {hits} moves")
            },
        });
    }

    let totals = Totals {
        operations: operations.len(),
        tools: tools.len(),
        tool_changes,
        cut_length: operations.iter().map(|o| o.cut_length).sum(),
        cut_minutes: operations.iter().map(|o| o.cut_minutes).sum(),
        program_minutes: Some(rb.cut_minutes).filter(|m| m.is_finite()),
        program_lines: i.text.lines().count(),
    };
    SetupSheet {
        job: i.name.to_string(),
        machine: machine.name.clone(),
        post: i.post_name.to_string(),
        program_file: i.program_file.and_then(|f| Path::new(f).file_name()).map(|f| f.to_string_lossy().into_owned()),
        work_offset: work_offset_words(i.post, i.work_offset),
        work_zero: zero,
        rotaries: machine.rotaries.iter().map(rotary_note).collect(),
        stock: None,
        tools,
        operations,
        totals,
        extents,
        checks,
        warnings,
        notes: i.notes.to_vec(),
    }
}

fn round(v: f64, decimals: i32) -> f64 {
    let k = 10f64.powi(decimals);
    let r = (v * k).round() / k;
    if r == 0.0 { 0.0 } else { r }
}

fn holder_text(t: &SheetTool) -> Option<String> {
    (!t.holder.is_empty()).then(|| t.holder.iter().map(|[d, l]| format!("Ø{} x {}", fmt_num(*d, 3), fmt_num(*l, 3))).collect::<Vec<_>>().join(" + "))
}

/// The reply's compact form of the sheet.
pub(super) fn summary(s: &SetupSheet) -> SetupSummary {
    let z = &s.work_zero;
    SetupSummary {
        work_zero: z.words.clone(),
        part_extent: [z.part_min.map(|v| round(v, 3)), z.part_max.map(|v| round(v, 3))],
        work_offset: s.work_offset.clone(),
        rotary: s.rotaries.iter().flat_map(|r| r.words.iter().cloned()).collect(),
        tools: s
            .tools
            .iter()
            .map(|t| SummaryTool {
                number: t.number,
                kind: t.kind.clone(),
                diameter: t.diameter,
                flute_length: t.flute_length,
                stickout: t.stickout,
                holder: holder_text(t),
                z_min: t.z_min.map(|v| round(v, 3)),
                operations: t.operations.clone(),
            })
            .collect(),
        operations: s
            .operations
            .iter()
            .map(|o| SummaryOperation {
                name: o.name.clone(),
                kind: o.kind.clone(),
                tool: o.tool,
                spindle_rpm: o.spindle_rpm,
                feed: o.feed,
                plunge_feed: o.plunge_feed,
                rotary: (!o.rotary.is_empty()).then(|| o.rotary.clone()),
                cut_length: round(o.cut_length, 1),
                cut_minutes: round(o.cut_minutes, 2),
            })
            .collect(),
        totals: SummaryTotals {
            operations: s.totals.operations,
            tools: s.totals.tools,
            tool_changes: s.totals.tool_changes,
            cut_length: round(s.totals.cut_length, 1),
            cut_minutes: round(s.totals.cut_minutes, 2),
            program_minutes: s.totals.program_minutes.map(|m| round(m, 2)),
        },
        checks: s.checks.iter().map(|c| format!("{}: {} ({})", c.name, if c.passed { "passed" } else { "FAILED" }, c.detail)).collect(),
        warnings: s.warnings.clone(),
    }
}
