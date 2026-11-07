//! Planning of the adaptive clearing operations of `cam.job` (`adaptive`, `adaptive_roughing`):
//! the operation's keys and the tool's data made into the CAM crate's parameters, the path
//! planned, and its engagement measured against the material (and, for `adaptive`, against a
//! plain offset pocket of the same profiles) for the report.

use std::sync::Arc;

use solidcore_record::{Loop2, Region2, Seg2, Vec2, Vec3};
use solidcraft_cam::adaptive::{
    AdaptiveParams, AdaptivePath, AdaptiveRoughingParams, HeightMap, REST_TOLERANCE, StockModel, adaptive_in, adaptive_roughing,
};
use solidcraft_cam::engagement::{Engagement, Material, engagement};
use solidcraft_cam::gcode5::{MachineProgram, emit_program_report};
use solidcraft_cam::sim::{SimParams, StockGrid, StockShape, simulate};
use solidcraft_cam::stock::Stock;
use solidcraft_cam::{Cut, Frame, PocketParams, Surface, Toolpath, pocket};

use super::{Ctx, Resolved, heights, or_tool};
use super::{Planned, spec_name};
use crate::Result;
use crate::schema::cam::CamJob;
use crate::schema::cam_adaptive::{AdaptiveOp, AdaptiveReport, AdaptiveRoughingOp, ComparedPath, RestFromSpec};

/// The engagement limits from the optimal load and what the operation gives: `(least, most)`
/// in degrees. The default least stays below the most the operation gives.
pub(super) fn limits(radius: f64, load: f64, min_deg: Option<f64>, max_deg: Option<f64>) -> (f64, f64) {
    let opt = if radius > 0.0 { (1.0 - load / radius).clamp(-1.0, 1.0).acos().to_degrees() } else { 90.0 };
    let most = max_deg.unwrap_or_else(|| (opt * 1.5).max(opt + 15.0).min(180.0));
    (min_deg.unwrap_or_else(|| (opt / 3.0).min(most * 0.5)), most)
}

/// How far above the limit the measured engagement may be before the report warns (degrees):
/// the planner works on a raster a sixteenth of the radius fine.
const OVER_LIMIT: f64 = 3.0;

/// Warnings for a measured path that broke what the operation asked for.
pub(super) fn engagement_warnings(rep: &AdaptiveReport) -> Vec<String> {
    let mut out = Vec::new();
    if !rep.measured {
        return out;
    }
    if rep.max_engagement_deg > rep.limit_deg + OVER_LIMIT {
        out.push(format!(
            "the measured engagement reaches {:.1}° outside the entries, above the limit of {:.1}°: check the path, or use a lighter optimal load",
            rep.max_engagement_deg, rep.limit_deg
        ));
    }
    if rep.full_slot_share > 0.0 {
        out.push(format!("the tool runs in a full slot for {:.1}% of the cutting time outside the entries", rep.full_slot_share * 100.0));
    }
    out
}

/// The adaptive parameters of either operation, checked the way every angle of a job is.
fn params(
    ctx: &Ctx<'_>,
    name: &str,
    r: &Resolved,
    o: (Option<f64>, Option<f64>, Option<f64>, Option<f64>, Option<f64>, Option<bool>, f64, Option<f64>),
) -> Result<AdaptiveParams> {
    let (load, min_deg, max_deg, helix, ramp, climb, leave, tol) = o;
    let tool = &r.tool;
    let optimal_load = or_tool(load, tool.stepover, tool.diameter * 0.1);
    let (least, most) = limits(tool.radius(), optimal_load, min_deg, max_deg);
    Ok(AdaptiveParams {
        optimal_load,
        min_engagement: ctx.degrees(name, least, "`min_engagement_deg`")?,
        max_engagement: ctx.degrees(name, most, "`max_engagement_deg`")?,
        stock_to_leave: leave,
        tolerance: ctx.tolerance(name, tol)?,
        helix_diameter: helix.unwrap_or(tool.diameter * 0.8),
        ramp_angle: ctx.degrees(name, ramp.unwrap_or(2.0), "`ramp_angle_deg`")?,
        climb: climb.unwrap_or(true),
    })
}

/// Measures `path` against `material`; a path too large to measure is reported as not measured.
fn measure(
    path: &Toolpath,
    roles: Option<&[solidcraft_cam::adaptive::MoveRole]>,
    radius: f64,
    material: &Material,
    warnings: &mut Vec<String>,
) -> Option<Engagement> {
    match engagement(path, roles, radius, material, radius / 8.0) {
        Ok(e) => Some(e),
        Err(e) => {
            warnings.push(format!("the engagement was not measured: {e}"));
            None
        }
    }
}

fn report(a: &AdaptivePath, e: Option<Engagement>, limit_deg: f64) -> AdaptiveReport {
    let measured = e.is_some();
    let e = e.unwrap_or_default();
    AdaptiveReport {
        measured,
        max_engagement_deg: e.max_deg,
        max_entry_engagement_deg: e.max_entry_deg,
        mean_engagement_deg: e.mean_deg,
        full_slot_share: e.full_slot_share,
        entry_slot_share: e.entry_slot_share,
        limit_deg,
        levels: a.levels,
        helices: a.helices,
        lifts: a.lifts,
        left_area: a.left_area,
        pocket: None,
    }
}

fn rect_region(lo: Vec2, hi: Vec2) -> Region2 {
    let p = [lo, Vec2::new(hi.x, lo.y), hi, Vec2::new(lo.x, hi.y)];
    let segs = (0..4).filter_map(|i| Some(Seg2::Line { a: *p.get(i)?, b: *p.get((i + 1) % 4)? })).collect();
    Region2 { outer: Loop2 { segs }, holes: vec![] }
}

/// The job's stock box seen in the operation's frame `frame`, when the operation looks straight
/// down on it (its Z along work Z): the box round its corners there, and its top. Any other
/// stock, or none, says nothing (the regions are then taken as full of material).
fn stock_in(stock: Option<&Stock>, frame: &Frame) -> StockModel {
    let Some(Stock::Box { min, max }) = stock else { return StockModel::default() };
    if !(frame.z.z > 1.0 - 1e-9) {
        return StockModel::default();
    }
    let (mut lo, mut hi) =
        (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
    for k in 0..8 {
        let c = Vec3::new(if k & 1 == 0 { min.x } else { max.x }, if k & 2 == 0 { min.y } else { max.y }, if k & 4 == 0 { min.z } else { max.z });
        let l = frame.to_local(c);
        lo = Vec3::new(lo.x.min(l.x), lo.y.min(l.y), lo.z.min(l.z));
        hi = Vec3::new(hi.x.max(l.x), hi.y.max(l.y), hi.z.max(l.z));
    }
    StockModel { rect: Some([Vec2::new(lo.x, lo.y), Vec2::new(hi.x, hi.y)]), top: Some(hi.z), ..Default::default() }
}

/// What the earlier operations `from` picks left of the job's stock box: the program they
/// write simulated as `cam.simulate` does, the stock it leaves taken as a height map in work
/// coordinates; and how many operations those were. `planned` are the operations planned so
/// far, `op_of` the index in the job of the operation each comes from.
pub(super) fn rest_map(
    ctx: &Ctx<'_>,
    job: &CamJob,
    name: &str,
    from: &RestFromSpec,
    planned: &[Planned],
    op_of: &[usize],
) -> Result<(Arc<HeightMap>, usize)> {
    let Some(&Stock::Box { min, max }) = ctx.stock else {
        return Err(ctx.bad(name, "`rest_from` needs the job's `stock` box: what the earlier operations cut is simulated from it"));
    };
    if !ctx.machine.rotaries.is_empty() {
        return Err(ctx.bad(name, "`rest_from` works on a 3-axis machine"));
    }
    let (names, tool) = match from {
        RestFromSpec::Previous(k) if k == "previous" => (None, None),
        RestFromSpec::Previous(k) => {
            return Err(ctx.bad(name, format!("`rest_from` is \"previous\", {{\"operations\": [names]}} or {{\"tool\": n}}, not \"{k}\"")));
        }
        RestFromSpec::Some(p) if p.operations.is_none() && p.tool.is_none() => {
            return Err(ctx.bad(name, "`rest_from` needs `operations` or `tool`"));
        }
        RestFromSpec::Some(p) => (p.operations.as_deref(), p.tool),
    };
    let earlier: Vec<&str> = op_of.iter().filter_map(|k| job.operations.get(*k).and_then(spec_name)).collect();
    if let Some(ns) = names
        && let Some(unknown) = ns.iter().find(|n| !earlier.contains(&n.as_str()))
    {
        return Err(
            ctx.bad(name, format!("`rest_from` names `{unknown}`, which is not an operation before this one (those are: {})", earlier.join(", ")))
        );
    }
    let picked: Vec<_> = planned
        .iter()
        .zip(op_of)
        .filter(|(p, k)| {
            let n = job.operations.get(**k).and_then(spec_name);
            names.is_none_or(|ns| ns.iter().any(|x| Some(x.as_str()) == n)) && tool.is_none_or(|t| p.tool.number == t)
        })
        .map(|(p, _)| p.op.clone())
        .collect();
    if picked.is_empty() {
        return Err(ctx.bad(name, "`rest_from` picks no operation before this one"));
    }
    let count = picked.len();
    let program = MachineProgram { name: format!("{name}: rest"), operations: picked, work_offset: None };
    let (text, _) =
        emit_program_report(&program, ctx.post, ctx.machine).map_err(|e| ctx.bad(name, format!("writing the earlier operations: {e}")))?;
    let report =
        simulate(&text, ctx.machine, ctx.tools, &StockShape::Box { min, max }, ctx.part, &SimParams { tolerance: ctx.tol, resolution: None })
            .map_err(|e| ctx.bad(name, format!("simulating the earlier operations: {e}")))?;
    let StockGrid::Heights { x0, y0, dx, dy, nx, ny, heights, .. } = report.stock else {
        return Err(ctx.bad(name, "the simulation of the earlier operations left no height map"));
    };
    let map = HeightMap { x0, y0, dx, dy, nx, ny, heights };
    map.validate().map_err(|e| ctx.bad(name, e))?;
    Ok((Arc::new(map), count))
}

/// The map of what earlier operations left ([`rest_map`], work coordinates) in the operation's
/// frame, which must look straight down along work X and Y (a 3-axis operation's does), and
/// how thick a skin it leaves unchased: the simulation's cell and the tolerance.
fn rest_in(ctx: &Ctx<'_>, name: &str, frame: &Frame) -> Result<Option<(HeightMap, f64, String)>> {
    // What an earlier setup left, and what the operations before this one cut of it.
    let (map, note) = match (ctx.start, &ctx.rest) {
        (None, None) => return Ok(None),
        (Some(start), None) => (start.clone(), "planned on what an earlier setup left of the stock".to_string()),
        (start, Some((rest, count))) => (
            start.map_or_else(|| HeightMap::clone(rest), |s| s.lower(rest)),
            format!("rest machining: planned on what the {count} operation(s) before it left of the stock"),
        ),
    };
    let square = |a: Vec3, b: Vec3| (a.x - b.x).abs().max((a.y - b.y).abs()).max((a.z - b.z).abs()) <= 1e-9;
    if !(square(frame.x, Vec3::new(1.0, 0.0, 0.0)) && square(frame.y, Vec3::new(0.0, 1.0, 0.0)) && square(frame.z, Vec3::new(0.0, 0.0, 1.0))) {
        return Err(
            ctx.bad(name, "planning on what earlier operations or setups left works in an operation that looks straight down along work X and Y")
        );
    }
    let o = frame.origin;
    let skin = REST_TOLERANCE.max(2.0 * ctx.tol);
    let note = format!(
        "{note}, simulated on cells of {:.3} mm; material up to {skin:.3} mm thick is left; the engagement is not measured (the measure knows only the whole stock)",
        map.dx.max(map.dy)
    );
    let local = HeightMap { x0: map.x0 - o.x, y0: map.y0 - o.y, heights: map.heights.iter().map(|z| z - o.z).collect(), ..map };
    Ok(Some((local, skin, note)))
}

/// `adaptive`: 2D adaptive clearing of profiles.
pub(super) fn plan(ctx: &Ctx<'_>, o: &AdaptiveOp, frame: &Frame) -> Result<(Toolpath, Resolved, &'static str)> {
    let c = o.common();
    let r = ctx.resolve(&c)?;
    let regions = ctx.regions(c.name, o.sketch.as_ref(), o.profiles.as_deref(), o.polygon.as_deref(), frame)?;
    let cut = Cut { heights: heights(&o.heights), feeds: r.feeds, stepdown: or_tool(o.stepdown, r.tool.stepdown, r.tool.diameter) };
    let p = params(
        ctx,
        c.name,
        &r,
        (
            o.optimal_load,
            o.min_engagement_deg,
            o.max_engagement_deg,
            o.helix_diameter,
            o.ramp_angle_deg,
            o.climb,
            o.stock_to_leave.unwrap_or(0.0),
            o.tolerance,
        ),
    )?;
    let mut stock = if o.from_stock.unwrap_or(true) { stock_in(ctx.stock, frame) } else { StockModel::default() };
    let rest = rest_in(ctx, c.name, frame)?;
    if let Some((map, skin, _)) = &rest {
        stock.heights = Some(map.clone());
        stock.rest_tolerance = *skin;
    }
    let a = adaptive_in(&regions, &r.tool, &p, &cut, &stock).map_err(|e| ctx.bad(c.name, e))?;
    let mut warnings = a.warnings.clone();
    let resting = rest.is_some();
    warnings.extend(rest.map(|(_, _, note)| note));
    let material = Material { regions: regions.clone(), top: stock.top.map_or(cut.heights.top, |t| t.min(cut.heights.top)), within: stock.rect };
    let radius = r.tool.radius();
    let e = if resting { None } else { measure(&a.path, Some(&a.roles), radius, &material, &mut warnings) };
    let mut rep = report(&a, e, p.max_engagement.to_degrees());
    warnings.extend(engagement_warnings(&rep));
    if o.compare.unwrap_or(true) {
        let pp = PocketParams {
            stepover: r.tool.diameter * 0.4,
            climb: p.climb,
            stock_to_leave: p.stock_to_leave,
            tolerance: p.tolerance,
            ramp_angle: 0.0,
        };
        match pocket(&regions, &r.tool, &pp, &cut) {
            Ok(plain) => {
                let stats = plain.stats();
                if let Some(pe) = measure(&plain, None, radius, &material, &mut warnings) {
                    rep.pocket = Some(ComparedPath {
                        max_engagement_deg: pe.max_deg,
                        full_slot_share: pe.full_slot_share,
                        cut_length: stats.cut_length,
                        cut_minutes: stats.cut_minutes,
                    });
                }
            }
            Err(e) => warnings.push(format!("the plain pocket to compare with could not be planned: {e}")),
        }
    }
    *ctx.adaptive.borrow_mut() = Some((rep, warnings));
    Ok((a.path, r, "adaptive"))
}

/// `adaptive_roughing`: 3D adaptive roughing of a stock box.
pub(super) fn plan_roughing(ctx: &Ctx<'_>, o: &AdaptiveRoughingOp, frame: &Frame, surface: &Surface) -> Result<(Toolpath, Resolved, &'static str)> {
    let c = o.common();
    let r = ctx.resolve(&c)?;
    let p = params(
        ctx,
        c.name,
        &r,
        (
            o.optimal_load,
            o.min_engagement_deg,
            o.max_engagement_deg,
            o.helix_diameter,
            o.ramp_angle_deg,
            o.climb,
            o.stock_to_leave.unwrap_or(0.25),
            o.tolerance,
        ),
    )?;
    let boundary = ctx.boundary(c.name, o.boundary.as_ref())?;
    let rp = AdaptiveRoughingParams {
        heights: heights(&o.heights),
        feeds: r.feeds,
        stepdown: or_tool(o.stepdown, r.tool.stepdown, r.tool.diameter),
        boundary: boundary.clone(),
        flats: o.flats.unwrap_or(true),
        adaptive: p,
        rest: None,
    };
    let rest = rest_in(ctx, c.name, frame)?;
    let rp = AdaptiveRoughingParams { rest: rest.as_ref().map(|(m, _, _)| m.clone()), ..rp };
    let a = adaptive_roughing(surface, &r.tool, &rp).map_err(|e| ctx.bad(c.name, e))?;
    let mut warnings = a.warnings.clone();
    let resting = rest.is_some();
    warnings.extend(rest.map(|(_, _, note)| note));
    // The stock as the operation took it: the boundary's rectangle, else the part's extent, and
    // inside the boundary's polygon too when there is one.
    let stock = match boundary.rect {
        Some([lo, hi]) => Some((lo, hi)),
        None => {
            let b = surface.bounds();
            (!b.is_empty()).then(|| (Vec2::new(b.min.x, b.min.y), Vec2::new(b.max.x, b.max.y)))
        }
    };
    let e = match stock {
        Some((lo, hi)) if !resting => {
            let regions = vec![boundary.region.clone().unwrap_or_else(|| rect_region(lo, hi))];
            let material = Material { regions, top: rp.heights.top, within: Some([lo, hi]) };
            measure(&a.path, Some(&a.roles), r.tool.radius(), &material, &mut warnings)
        }
        _ => None,
    };
    let rep = report(&a, e, p.max_engagement.to_degrees());
    warnings.extend(engagement_warnings(&rep));
    *ctx.adaptive.borrow_mut() = Some((rep, warnings));
    Ok((a.path, r, "adaptive_roughing"))
}
