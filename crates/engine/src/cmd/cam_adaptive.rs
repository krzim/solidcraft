//! Planning of the adaptive clearing operations of `cam.job` (`adaptive`, `adaptive_roughing`):
//! the operation's keys and the tool's data made into the CAM crate's parameters, the path
//! planned, and its engagement measured against the material (and, for `adaptive`, against a
//! plain offset pocket of the same profiles) for the report.

use solidcore_record::{Loop2, Region2, Seg2, Vec2};
use solidcraft_cam::adaptive::{AdaptiveParams, AdaptivePath, AdaptiveRoughingParams, adaptive, adaptive_roughing};
use solidcraft_cam::engagement::{Engagement, Material, engagement};
use solidcraft_cam::{Cut, Frame, PocketParams, Surface, Toolpath, pocket};

use super::{Ctx, Resolved, heights, or_tool};
use crate::Result;
use crate::schema::cam_adaptive::{AdaptiveOp, AdaptiveReport, AdaptiveRoughingOp, ComparedPath};

/// The engagement limits from the optimal load and what the operation gives: `(least, most)`
/// in degrees.
fn limits(radius: f64, load: f64, min_deg: Option<f64>, max_deg: Option<f64>) -> (f64, f64) {
    let opt = if radius > 0.0 { (1.0 - load / radius).clamp(-1.0, 1.0).acos().to_degrees() } else { 90.0 };
    let most = max_deg.unwrap_or_else(|| (opt * 1.5).max(opt + 15.0).min(180.0));
    (min_deg.unwrap_or(opt / 3.0), most)
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
    let e = e.unwrap_or_default();
    AdaptiveReport {
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
    let a = adaptive(&regions, &r.tool, &p, &cut).map_err(|e| ctx.bad(c.name, e))?;
    let mut warnings = a.warnings.clone();
    let material = Material { regions: regions.clone(), top: cut.heights.top };
    let radius = r.tool.radius();
    let e = measure(&a.path, Some(&a.roles), radius, &material, &mut warnings);
    let mut rep = report(&a, e, p.max_engagement.to_degrees());
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
pub(super) fn plan_roughing(ctx: &Ctx<'_>, o: &AdaptiveRoughingOp, surface: &Surface) -> Result<(Toolpath, Resolved, &'static str)> {
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
    };
    let a = adaptive_roughing(surface, &r.tool, &rp).map_err(|e| ctx.bad(c.name, e))?;
    let mut warnings = a.warnings.clone();
    // The stock as the operation took it: the boundary's rectangle, else the part's extent.
    let stock = match boundary.rect {
        Some([lo, hi]) => Some(rect_region(lo, hi)),
        None => {
            let b = surface.bounds();
            (!b.is_empty()).then(|| rect_region(Vec2::new(b.min.x, b.min.y), Vec2::new(b.max.x, b.max.y)))
        }
    };
    let e = match stock {
        Some(region) => {
            let mut regions = vec![region];
            if let Some(extra) = &boundary.region {
                regions = vec![extra.clone()];
            }
            measure(&a.path, Some(&a.roles), r.tool.radius(), &Material { regions, top: rp.heights.top }, &mut warnings)
        }
        None => None,
    };
    let rep = report(&a, e, p.max_engagement.to_degrees());
    *ctx.adaptive.borrow_mut() = Some((rep, warnings));
    Ok((a.path, r, "adaptive_roughing"))
}
