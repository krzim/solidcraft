//! The stock, facing and rapid safety of `cam.job` (a child of `cmd/cam.rs`): the job's stock
//! resolved and checked, the `face` operation planned on it, and every rapid and every position
//! of the written program checked against the part, the fixtures, the stock and the machine's
//! travel (docs/cam-jobs.md, "Stock, facing and rapid safety").

use solidcore_record::{Vec2, Vec3};
use solidcraft_cam::gcode5::{MachineOperation, MachineProgram, Post5};
use solidcraft_cam::indexed::{IndexedOperation, frame_for_index};
use solidcraft_cam::kinematics::Machine;
use solidcraft_cam::rapids::{RAPID_STEP, check_rapids_staged, check_travel};
use solidcraft_cam::stock::{FaceParams, Stock, default_face_overhang, default_face_stepover, face};
use solidcraft_cam::{Frame, Heights, Toolpath};

use super::{Ctx, Planned, Surfaces, cam_err, finite_point, num, v3};
use crate::Result;
use crate::params::bad;
use crate::schema::cam::{CamJob, OperationReport, OperationSpec};
use crate::schema::cam_stock::{FaceOp, RapidHitReport, RapidsReport, StockReport, StockSpec, TravelReport};

/// Hits the reply lists in full.
const LISTED_HITS: usize = 20;
/// How far above the top (or the stock's top) a face operation's retract height is by default,
/// and its clearance height above that (mm).
const FACE_RETRACT: f64 = 2.0;
const FACE_CLEARANCE: f64 = 10.0;

/// The job's stock, checked; notes what is worth knowing about it.
pub(super) fn stock_of(cmd: &str, job: &CamJob, surfaces: &Surfaces, tol: f64, notes: &mut Vec<String>) -> Result<Option<Stock>> {
    let Some(spec) = &job.stock else { return Ok(None) };
    let stock = match *spec {
        StockSpec::Box(b) => {
            finite_point(&[b.min, b.max].concat(), "`stock.box`").map_err(|e| bad(cmd, e))?;
            Stock::Box { min: v3(b.min), max: v3(b.max) }
        }
        StockSpec::Cylinder(c) => {
            finite_point(&[c.radius, c.x_min, c.x_max], "`stock.cylinder`").map_err(|e| bad(cmd, e))?;
            Stock::Cylinder { radius: c.radius, x_min: c.x_min, x_max: c.x_max }
        }
        StockSpec::AroundPart(a) => Stock::around(&surfaces.part.bounds(), v3(a.margin)).map_err(|e| cam_err(cmd, "`stock.around_part`", e))?,
    };
    stock.validate().map_err(|e| cam_err(cmd, "`stock`", e))?;
    if !stock.holds(&surfaces.part.bounds(), tol) {
        notes.push("the part reaches outside the stock: check the stock against the part's position in work coordinates".into());
    }
    Ok(Some(stock))
}

/// The stock as the reply shows it.
pub(super) fn stock_report(stock: &Stock) -> StockReport {
    match *stock {
        Stock::Box { min, max } => {
            StockReport { kind: "box".into(), min: Some([min.x, min.y, min.z]), max: Some([max.x, max.y, max.z]), ..StockReport::default() }
        }
        Stock::Cylinder { radius, x_min, x_max } => {
            StockReport { kind: "cylinder".into(), radius: Some(radius), x_min: Some(x_min), x_max: Some(x_max), ..StockReport::default() }
        }
    }
}

fn finite_in_range(v: f64) -> bool {
    v.is_finite() && v.abs() <= 1.0e6
}

/// A `face` operation planned with every rotary at zero.
pub(super) fn plan_face(ctx: &Ctx<'_>, o: &FaceOp, arc_tol: f64) -> Result<Planned> {
    let c = o.common();
    let r = ctx.resolve(&c)?;
    let stock = ctx.stock;
    let mut warnings = Vec::new();
    let area = match (o.rect, stock) {
        (Some([lo, hi]), _) => {
            finite_point(&[lo[0], lo[1], hi[0], hi[1]], "`rect`").map_err(|e| ctx.bad(c.name, e))?;
            if !(lo[0] < hi[0] && lo[1] < hi[1]) {
                return Err(ctx.bad(c.name, "`rect` must be [[low x, low y], [high x, high y]]"));
            }
            [Vec2::new(lo[0], lo[1]), Vec2::new(hi[0], hi[1])]
        }
        (None, Some(s)) => s.footprint(),
        (None, None) => {
            let b = ctx.part.bounds();
            [Vec2::new(b.min.x, b.min.y), Vec2::new(b.max.x, b.max.y)]
        }
    };
    let top = match (o.top, stock) {
        (Some(t), _) => t,
        (None, Some(s)) => s.top(),
        (None, None) => return Err(ctx.bad(c.name, "give `top` (the job has no `stock` to face the top of)")),
    };
    for (v, what) in [(Some(o.bottom), "`bottom`"), (Some(top), "`top`"), (o.retract, "`retract`"), (o.clearance, "`clearance`")] {
        if v.is_some_and(|v| !finite_in_range(v)) {
            return Err(ctx.bad(c.name, format!("{what} must be a finite height within 1000 m")));
        }
    }
    let stock_top = stock.map(Stock::top);
    if let Some(st) = stock_top
        && top < st - ctx.tol
    {
        warnings.push(format!("the top ({}) is below the stock's top ({}): the first level takes more than a stepdown", num(top), num(st)));
    }
    let retract = o.retract.unwrap_or(top.max(stock_top.unwrap_or(top)) + FACE_RETRACT);
    let clearance = o.clearance.unwrap_or(retract + FACE_CLEARANCE);
    let tool = &r.tool;
    let params = FaceParams {
        area,
        heights: Heights { clearance, retract, top, bottom: o.bottom },
        feeds: r.feeds,
        stepdown: o.stepdown.or(tool.stepdown).unwrap_or(1.0),
        stepover: o.stepover.or(tool.stepover).unwrap_or_else(|| default_face_stepover(tool)),
        angle: ctx.degrees(c.name, o.angle_deg.unwrap_or(0.0), "`angle_deg`")?,
        overhang: o.overhang.unwrap_or_else(|| default_face_overhang(tool)),
        zigzag: o.zigzag.unwrap_or(true),
        stock: stock.copied(),
    };
    let path: Toolpath = face(tool, &params).map_err(|e| ctx.bad(c.name, e))?;
    let rotary = vec![0.0; ctx.machine.rotaries.len()];
    // The passes are in work coordinates: the machine must hold the work square with every
    // rotary at zero (it does on `3axis` and `4axis_a`).
    if frame_for_index(ctx.machine, &rotary).map_err(|e| ctx.bad(c.name, e))? != Frame::IDENTITY {
        return Err(ctx.bad(c.name, "facing needs a machine whose rotaries at zero leave the work coordinates as they are"));
    }
    let stats = path.stats();
    let io = IndexedOperation { name: c.name.into(), tool: tool.clone(), spindle_rpm: r.rpm, coolant: c.coolant, rotary, path };
    let poses = io.poses(ctx.machine, arc_tol).map_err(|e| ctx.bad(c.name, e))?;
    let report = OperationReport {
        name: c.name.into(),
        kind: "face".into(),
        tool: tool.number,
        moves: io.path.moves.len(),
        cut_length: stats.cut_length,
        cut_minutes: stats.cut_minutes,
        max_tilt_deg: poses.stats().max_tilt.to_degrees(),
        warnings,
        ..OperationReport::default()
    };
    Ok(Planned { op: MachineOperation::Indexed(io), poses, feeds: r.feeds, tool: r.tool, gouge_check: c.gouge_check, report, axes: "3-axis".into() })
}

/// What is left of the job's stock as each of its operations starts: a `face` over the whole
/// top of a box stock lowers it to the face's bottom for the operations after it. Only with the
/// default overhang, which puts the outermost passes' centres outside the edges, so the flat of
/// the tool's end reaches them (a smaller one may leave a bull nose's corner along an edge).
fn stock_stages(job: &CamJob, stock: Option<&Stock>) -> Vec<Option<Stock>> {
    let mut left = stock.copied();
    job.operations
        .iter()
        .map(|spec| {
            let now = left;
            if let (OperationSpec::Face(o), Some(s)) = (spec, left)
                && o.overhang.is_none()
            {
                let area = o.rect.map_or_else(|| s.footprint(), |[lo, hi]| [Vec2::new(lo[0], lo[1]), Vec2::new(hi[0], hi[1])]);
                left = Some(s.faced(area, o.bottom));
            }
            now
        })
        .collect()
}

fn at(p: Vec3) -> String {
    format!("[{}, {}, {}]", num(p.x), num(p.y), num(p.z))
}

/// Every rapid of `text` against the part, fixtures and stock, and every position against the
/// machine's travel; each failed check adds to `failures`.
#[allow(clippy::too_many_arguments)]
pub(super) fn check_motion(
    cmd: &str,
    job: &CamJob,
    text: &str,
    program: &MachineProgram,
    post: &Post5,
    machine: &Machine,
    surfaces: &Surfaces,
    stock: Option<&Stock>,
    tol: f64,
    failures: &mut Vec<String>,
) -> Result<(RapidsReport, Option<TravelReport>)> {
    let stages = stock_stages(job, stock);
    let stages = if stages.len() == program.operations.len() { stages } else { Vec::new() };
    let check = check_rapids_staged(text, program, post, machine, &surfaces.all, &surfaces.part, stock, &stages, tol, RAPID_STEP)
        .map_err(|e| cam_err(cmd, "checking the rapids", e))?;
    let op_name = |k: Option<usize>| {
        k.and_then(|k| program.operations.get(k)).map(|o| match o {
            MachineOperation::Continuous(c) => c.name.clone(),
            MachineOperation::Indexed(i) => i.name.clone(),
        })
    };
    let first: Vec<RapidHitReport> = check
        .hits
        .iter()
        .take(LISTED_HITS)
        .map(|h| RapidHitReport {
            line: h.line,
            operation: op_name(h.operation),
            kind: serde_json::to_value(h.kind).ok().and_then(|v| v.as_str().map(str::to_string)).unwrap_or_default(),
            depth: h.depth,
            at: [h.at.x, h.at.y, h.at.z],
        })
        .collect();
    if let Some(h) = check.hits.first() {
        let op = op_name(h.operation).map_or(String::new(), |n| format!(" (operation `{n}`)"));
        failures.push(format!(
            "rapids: {} rapid blocks run into something; the first, line {}{op}, {} by {} mm at {}",
            check.hit_count,
            h.line,
            h.kind.label(),
            num(h.depth),
            at(h.at)
        ));
    }
    let rapids = RapidsReport {
        checked: check.checked,
        hits: check.hit_count,
        worst: check.worst,
        pass: check.pass(),
        from_unknown: check.from_unknown,
        first,
    };
    let travel = check_travel(text, machine).map_err(|e| cam_err(cmd, "checking the travel", e))?.map(|t| {
        let violation =
            t.violation.map(|v| format!("line {}: {} at {} is beyond its travel {} to {}", v.line, v.axis, num(v.value), num(v.low), num(v.high)));
        if let Some(v) = &violation {
            failures.push(format!("travel: {v}"));
        }
        TravelReport { checked: t.checked, pass: violation.is_none(), violation }
    });
    Ok((rapids, travel))
}
