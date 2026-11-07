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
use solidcraft_cam::{Frame, Heights, Move5, Toolpath};
use solidcraft_geom::Vec3Geom;

use super::{Ctx, Planned, Surfaces, cam_err, finite_point, num, v3};
use crate::Result;
use crate::params::bad;
use crate::schema::cam::{CamJob, OperationReport, OperationSpec};
use crate::schema::cam_stock::{FaceOp, RapidHitReport, RapidsReport, StockReport, StockSpec, TravelReport};
use solidcraft_cam::adaptive::HeightMap;

/// Hits the reply lists in full.
const LISTED_HITS: usize = 20;
/// How far above the top (or the stock's top) a face operation's retract height is by default,
/// and its clearance height above that (mm).
const FACE_RETRACT: f64 = 2.0;
const FACE_CLEARANCE: f64 = 10.0;

/// The job's stock, checked; notes what is worth knowing about it.
pub(super) fn stock_of(
    cmd: &str,
    s: &crate::Session,
    job: &CamJob,
    base: Option<&std::path::Path>,
    surfaces: &Surfaces,
    tol: f64,
    notes: &mut Vec<String>,
) -> Result<Option<Stock>> {
    Ok(stock_and_start(cmd, s, job, base, surfaces, tol, notes)?.0)
}

/// The most bytes of a stock height map file.
const MAX_HEIGHTS_BYTES: usize = 1 << 30;

/// A stock height map as `cam.simulate` writes it (`stock_heights`): the map in the work
/// coordinates of the job that wrote it, and that job's placement of the design.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub(super) struct StockHeightsFile {
    pub format: String,
    pub map: HeightMap,
    /// The stock's lowest and highest Z.
    pub bottom: f64,
    pub top: f64,
    /// The design point that was work zero, and the work axes in the design when turned.
    pub origin: [f64; 3],
    pub axes: Option<[[f64; 3]; 3]>,
}

pub(super) const STOCK_HEIGHTS_FORMAT: &str = "solidcraft.stock_heights";

/// The job's stock, and the height map it starts from when it is what an earlier setup left
/// (`simulated`), in this job's work coordinates.
pub(super) fn stock_and_start(
    cmd: &str,
    s: &crate::Session,
    job: &CamJob,
    base: Option<&std::path::Path>,
    surfaces: &Surfaces,
    tol: f64,
    notes: &mut Vec<String>,
) -> Result<(Option<Stock>, Option<HeightMap>)> {
    let Some(spec) = &job.stock else { return Ok((None, None)) };
    let mut start = None;
    let stock = match *spec {
        StockSpec::Simulated(ref sim) => {
            let place = super::placement_of(cmd, s, job)?;
            let (stock, map) = simulated(cmd, &super::beside(base, &sim.path), &place)?;
            notes.push(format!("the stock is what an earlier setup left ({}), turned into this job's work coordinates", sim.path));
            start = Some(map);
            stock
        }
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
    if !stock.holds_surface(&surfaces.part, tol) {
        notes.push("the part reaches outside the stock: check the stock against the part's position in work coordinates".into());
    }
    Ok((Some(stock), start))
}

/// Reads the height map at `path` and turns it into the work coordinates of `place`: each of
/// this job's cells takes the column of the written map under its centre, turned (upright, or
/// upside down after a flip), its top the higher end of it. The turn between the two setups
/// must keep the columns upright or upside down, with X and Y along X or Y.
fn simulated(cmd: &str, path: &str, place: &super::Placement) -> Result<(Stock, HeightMap)> {
    let bytes = solidcraft_io::vfs::read(path, MAX_HEIGHTS_BYTES).map_err(|e| bad(cmd, format!("`stock.simulated`: {path}: {e}")))?;
    let file: StockHeightsFile =
        serde_json::from_slice(&bytes).map_err(|e| bad(cmd, format!("`stock.simulated`: {path} is not a stock height map: {e}")))?;
    if file.format != STOCK_HEIGHTS_FORMAT {
        return Err(bad(cmd, format!("`stock.simulated`: {path} is not a `{STOCK_HEIGHTS_FORMAT}` file")));
    }
    file.map.validate().map_err(|e| bad(cmd, format!("`stock.simulated`: {e}")))?;
    finite_point(&[file.bottom, file.top], "the height map's `bottom` and `top`").map_err(|e| bad(cmd, e))?;
    finite_point(&file.origin, "the height map's `origin`").map_err(|e| bad(cmd, e))?;
    if let Some(a) = &file.axes {
        finite_point(&a.concat(), "the height map's `axes`").map_err(|e| bad(cmd, e))?;
        // A turn: three unit directions square to each other, right-handed (a skewed or
        // mirrored file would put the material where it is not).
        let [x, y, z] = a.map(v3);
        let near = |v: f64, want: f64| (v - want).abs() <= 1e-6;
        let turn = [x, y, z].iter().all(|v| near(v.dot(*v), 1.0))
            && near(x.dot(y), 0.0)
            && near(y.dot(z), 0.0)
            && near(x.dot(z), 0.0)
            && near(x.cross(y).dot(z), 1.0);
        if !turn {
            return Err(bad(
                cmd,
                "`stock.simulated`: the height map's `axes` must be a turn: three unit directions square to each other, right-handed",
            ));
        }
    }
    let was = super::Placement { origin: v3(file.origin), axes: file.axes.map(|a| a.map(v3)) };
    // From the written job's work coordinates to this job's, and back.
    let to = |w: Vec3| place.in_work(was.unturn(w + was.origin));
    let from = |w: Vec3| was.in_work(place.unturn(w + place.origin));
    let o = to(Vec3::ZERO);
    let dir = |d: Vec3| to(d) - o;
    let (ux, uy, uz) = (dir(Vec3::new(1.0, 0.0, 0.0)), dir(Vec3::new(0.0, 1.0, 0.0)), dir(Vec3::new(0.0, 0.0, 1.0)));
    let along = |v: Vec3| [v.x.abs(), v.y.abs(), v.z.abs()].iter().filter(|c| (*c - 1.0).abs() < 1e-9).count() == 1;
    if !(along(ux) && along(uy) && (uz.z.abs() - 1.0).abs() < 1e-9) {
        return Err(bad(cmd, "`stock.simulated`: the two setups must turn the part so it stays upright or upside down, X and Y along X or Y"));
    }
    let m = &file.map;
    let (sx, sy) = if ux.x.abs() > 0.5 { (m.dx, m.dy) } else { (m.dy, m.dx) };
    let (nx, ny) = if ux.x.abs() > 0.5 { (m.nx, m.ny) } else { (m.ny, m.nx) };
    let corners = [Vec3::new(m.x0, m.y0, 0.0), Vec3::new(m.x0 + m.nx as f64 * m.dx, m.y0 + m.ny as f64 * m.dy, 0.0)].map(to);
    let (x0, y0) = (corners[0].x.min(corners[1].x), corners[0].y.min(corners[1].y));
    let (zb, zt) = (to(Vec3::new(0.0, 0.0, file.bottom)).z, to(Vec3::new(0.0, 0.0, file.top)).z);
    let (low, high) = (zb.min(zt), zb.max(zt));
    let mut heights = Vec::with_capacity(nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            let c = Vec3::new(x0 + (i as f64 + 0.5) * sx, y0 + (j as f64 + 0.5) * sy, 0.0);
            let w = from(c);
            let top = m.top_at(Vec2::new(w.x, w.y)).filter(|t| *t > file.bottom + 1e-9);
            heights.push(match top {
                // The column from the bottom to its top, turned: its higher end.
                Some(t) => to(Vec3::new(w.x, w.y, t)).z.max(to(Vec3::new(w.x, w.y, file.bottom)).z),
                None => low,
            });
        }
    }
    let map = HeightMap { x0, y0, dx: sx, dy: sy, nx, ny, heights };
    map.validate().map_err(|e| bad(cmd, format!("`stock.simulated`: {e}")))?;
    let stock = Stock::Box { min: Vec3::new(x0, y0, low), max: Vec3::new(x0 + nx as f64 * sx, y0 + ny as f64 * sy, high) };
    Ok((stock, map))
}

/// Fails each operation whose lowest cut goes below a box stock's bottom by more than `tol` on
/// a machine without rotaries: below it are the parallels, the vise's floor or the table that
/// hold the stock, or below the lowest fixture that reaches under it. The deepest point of every cutting move with the tool upright counts (a
/// drill's point included); facing is refused earlier, when it is planned.
pub(super) fn check_below_stock(
    planned: &[Planned],
    stock: &Stock,
    fixture_low: Option<f64>,
    machine: &Machine,
    tol: f64,
    failures: &mut Vec<String>,
) {
    let Stock::Box { min, .. } = *stock else { return };
    if !machine.rotaries.is_empty() {
        return;
    }
    // Fixtures that reach below the stock (parallels it stands on, a vise's jaws) are what holds
    // it there, and every cut is checked against them: a cut through the stock may go on down
    // to where the lowest of them reaches (what is below that, the vise's floor or the table, is
    // not modelled).
    let held = fixture_low.filter(|z| z.is_finite() && *z < min.z);
    let floor = held.unwrap_or(min.z);
    for p in planned {
        let lowest = p
            .poses
            .moves
            .iter()
            .filter_map(|m| match m {
                Move5::Feed { to, .. } if to.axis.z >= 1.0 - 1e-9 => Some(to.tip.z),
                _ => None,
            })
            .fold(f64::INFINITY, f64::min);
        if !(lowest.is_finite() && lowest < floor - tol) {
            continue;
        }
        failures.push(match held {
            Some(z) => format!(
                "below the stock: operation `{}` cuts down to Z{}, {} mm below the lowest fixture (Z{}) under the stock's bottom (Z{}), into what holds them (the vise's floor or the table): raise its bottom to Z{} or above, or model what the stock stands on as fixtures reaching as deep as the cut",
                p.report.name,
                num(lowest),
                num(z - lowest),
                num(z),
                num(min.z),
                num(z)
            ),
            None => format!(
                "below the stock: operation `{}` cuts down to Z{}, {} mm below the stock's bottom (Z{}), into what holds the stock (parallels, the vise's floor or the table): raise its bottom to Z{} or above, give a stock that reaches as deep as the cut (set on a spoil board), or give the parallels it stands on as `fixtures` (the cut is then checked against them)",
                p.report.name,
                num(lowest),
                num(min.z - lowest),
                num(min.z),
                num(min.z)
            ),
        });
    }
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
        lead_radius: o.lead_radius,
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
    let (poses, sources) = io.poses_mapped(ctx.machine, arc_tol).map_err(|e| ctx.bad(c.name, e))?;
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
    Ok(Planned {
        op: MachineOperation::Indexed(io),
        poses,
        sources,
        feeds: r.feeds,
        tool: r.tool,
        gouge_check: c.gouge_check,
        report,
        axes: "3-axis".into(),
    })
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
    unchecked: &[bool],
    tol: f64,
    failures: &mut Vec<String>,
) -> Result<(RapidsReport, Option<TravelReport>)> {
    let stages = stock_stages(job, stock);
    let stages = if stages.len() == program.operations.len() { stages } else { Vec::new() };
    let unchecked = if unchecked.len() == program.operations.len() { unchecked } else { &[] };
    let check = check_rapids_staged(text, program, post, machine, &surfaces.all, &surfaces.part, stock, &stages, unchecked, tol, RAPID_STEP)
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
