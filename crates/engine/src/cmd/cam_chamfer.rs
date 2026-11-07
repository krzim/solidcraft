//! The `chamfer` operation of `cam.job`: a chamfer or deburr along closed profiles' top edges
//! with a chamfer mill, through the CAM crate's `chamfer` (docs/cam-jobs.md). The crate works
//! out the tool's offset and depth from its angle so that the chamfer is as wide as asked, and
//! refuses a chamfer the tool's cone cannot reach. The chamfer need not be in the model: the
//! gouge check allows the band the operation is told to cut ([`solidcraft_cam::band_gouges`]).

use solidcraft_cam::{ChamferParams, DEFAULT_TIP_OFFSET, Frame, Side, Toolpath, chamfer, chamfer_geometry};

use super::{Ctx, Resolved};
use crate::Result;
use crate::schema::cam::{ChamferOp, SideSpec};

/// The chamfer's toolpath in `frame`'s coordinates, and what it cuts with.
pub(super) fn plan(ctx: &Ctx<'_>, o: &ChamferOp, frame: &Frame) -> Result<(Toolpath, Resolved)> {
    let c = o.common();
    let r = ctx.resolve(&c)?;
    let regions = ctx.regions(c.name, o.sketch.as_ref(), o.profiles.as_deref(), o.polygon.as_deref(), frame)?;
    let side = match o.side.unwrap_or(SideSpec::Outside) {
        SideSpec::Outside => Side::Outside,
        SideSpec::Inside => Side::Inside,
        SideSpec::On => return Err(ctx.bad(c.name, "a chamfer is cut `outside` a part or `inside` an opening, not `on` the profile")),
    };
    let params = ChamferParams {
        width: o.width,
        tip_offset: o.tip_offset.unwrap_or(DEFAULT_TIP_OFFSET),
        side,
        climb: o.climb.unwrap_or(true),
        tolerance: ctx.tolerance(c.name, o.tolerance)?,
        clearance: o.heights.clearance,
        retract: o.heights.retract,
        top: o.heights.top,
        feeds: r.feeds,
    };
    let path = chamfer(&regions, &r.tool, &params).map_err(|e| ctx.bad(c.name, e))?;
    // The chamfer itself is the operation's own: its gouge check allows it.
    let g = chamfer_geometry(&r.tool, params.width, params.tip_offset).map_err(|e| ctx.bad(c.name, e))?;
    ctx.band.set(Some(g.height));
    Ok((path, r))
}
