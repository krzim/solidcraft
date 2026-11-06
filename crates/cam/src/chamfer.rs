//! 2D chamfers and deburring with a chamfer mill along the top edges of closed profiles.
//!
//! A chamfer mill of half angle `α` (half its included angle) with a flat tip of radius `rt`
//! cuts, with its tip `tip_offset` below the chamfer, a chamfer whose side is at `α` from the
//! tool axis. With the chamfer `width` wide across the top (its horizontal leg), it goes
//! `width / tan α` down the wall (its vertical leg), so the tip runs `width / tan α +
//! tip_offset` below the top and its axis `rt + tip_offset · tan α` out from the edge: the
//! cone's side then lies on the chamfer from the wall to the top. The cone must reach the top
//! within the tool's cutting radius `R`, so the widest chamfer is `R − rt − tip_offset · tan α`;
//! a wider one is refused, never cut narrower.
//!
//! The toolpath follows the profiles' offsets one pass deep, outside a part or inside an
//! opening as a contour does, and is checked like every 2D operation before it is returned.

use serde::{Deserialize, Serialize};
use solidcore_record::Region2;

use crate::ops::{ContourParams, Cut, Feeds, Heights, Side, contour};
use crate::path::Toolpath;
use crate::tool::{Tool, ToolKind};
use crate::{Result, invalid};

/// Largest chamfer width and tip offset (mm).
const MAX_SIZE: f64 = 1000.0;
/// Below this, the tool's axis runs on the edge itself (mm).
const ON_EDGE: f64 = 1e-9;

/// The tip offset operations start from (mm).
pub const DEFAULT_TIP_OFFSET: f64 = 0.2;

/// What a chamfer is cut with.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChamferParams {
    /// The chamfer's width across the top face (its horizontal leg, mm).
    pub width: f64,
    /// How far the tip goes below the chamfer's bottom edge (mm), so that the cone's side, not
    /// its tip, cuts the whole chamfer.
    pub tip_offset: f64,
    /// Outside a part or inside an opening; `On` is refused (the side decides the offset).
    pub side: Side,
    /// Climb milling, as for a contour.
    pub climb: bool,
    /// Chord error allowed on curved profiles (mm).
    pub tolerance: f64,
    /// Where the tool travels between cuts.
    pub clearance: f64,
    /// Where rapid motion towards the material stops and feed begins.
    pub retract: f64,
    /// The top face whose edges are chamfered.
    pub top: f64,
    pub feeds: Feeds,
}

/// Where a chamfer mill runs to cut a chamfer.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChamferGeometry {
    /// From the profile to the tool's axis, sideways (mm).
    pub offset: f64,
    /// From the top face down to the tip (mm).
    pub depth: f64,
    /// The chamfer's vertical leg (mm).
    pub height: f64,
    /// The widest chamfer the tool cuts with this tip offset (mm).
    pub max_width: f64,
}

/// Where `tool`, a chamfer mill, runs for a chamfer `width` wide with its tip `tip_offset`
/// below it; refused when its cone does not reach that far out, or its flutes that far down.
pub fn chamfer_geometry(tool: &Tool, width: f64, tip_offset: f64) -> Result<ChamferGeometry> {
    tool.validate()?;
    if tool.kind != ToolKind::Chamfer {
        return invalid(format!("a chamfer is cut with a chamfer mill, not a {}", tool.kind.label()));
    }
    if !(width > 0.0) || !(width <= MAX_SIZE) {
        return invalid("the chamfer width must be above 0 and at most 1000 mm");
    }
    if !(tip_offset >= 0.0) || !(tip_offset <= MAX_SIZE) {
        return invalid("the tip offset must be from 0 to 1000 mm");
    }
    let cutter = tool.cutter()?;
    let Some(cone) = cutter.cone else { return invalid("a chamfer mill's cutter is tapered") };
    let tan = cone.half_angle.tan();
    let offset = cone.tip_radius + tip_offset * tan;
    let max_width = cutter.radius - offset;
    if !(width <= max_width + 1e-9) {
        return if max_width > 0.0 {
            invalid(format!(
                "tool {} cannot reach a chamfer {width} mm wide: its cone ends {} mm from its axis, so with a tip offset of {tip_offset} mm \
                 the widest chamfer is {:.4} mm",
                tool.number, cutter.radius, max_width
            ))
        } else {
            invalid(format!("tool {}: with a tip offset of {tip_offset} mm its cone cannot reach the top at all; lower the tip offset", tool.number))
        };
    }
    let height = width / tan;
    let depth = height + tip_offset;
    if !(depth <= tool.flute_length + 1e-9) {
        return invalid(format!(
            "the chamfer goes {depth:.4} mm down to the tip, beyond tool {}'s {} mm flute length",
            tool.number, tool.flute_length
        ));
    }
    Ok(ChamferGeometry { offset, depth, height, max_width })
}

/// Chamfers the top edges of `regions`' boundaries on `params.side` with `tool`, a chamfer mill,
/// in one pass.
pub fn chamfer(regions: &[Region2], tool: &Tool, params: &ChamferParams) -> Result<Toolpath> {
    let g = chamfer_geometry(tool, params.width, params.tip_offset)?;
    if params.side == Side::On {
        return invalid("a chamfer is cut outside a part or inside an opening, not on the profile");
    }
    let heights = Heights { clearance: params.clearance, retract: params.retract, top: params.top, bottom: params.top - g.depth };
    let cut = Cut { heights, feeds: params.feeds, stepdown: g.depth };
    // The contour is planned for a tool whose radius is the axis's offset from the edge. With
    // no offset the axis runs on the profile, the travel direction still chosen for the side.
    let (proxy, side, climb) = if g.offset > ON_EDGE {
        (Tool::new(tool.number, ToolKind::FlatEndmill, g.offset * 2.0, tool.flute_length), params.side, params.climb)
    } else {
        (Tool::new(tool.number, ToolKind::FlatEndmill, tool.diameter, tool.flute_length), Side::On, params.climb == (params.side == Side::Inside))
    };
    let contour_params = ContourParams { side, climb, stock_to_leave: 0.0, tolerance: params.tolerance, ramp_angle: 0.0 };
    contour(regions, &proxy, &contour_params, &cut)
}
