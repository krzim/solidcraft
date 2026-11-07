//! `cam.features`: the machinable features of a part, found on the exact B-rep of its bodies
//! (never on a mesh): round holes along the tool axis (through or blind, counterbores and
//! countersinks), closed pockets (a planar floor walled all round, its islands and the smallest
//! radius of its inside corners), planar faces facing the tool, outer profiles, fillets and
//! chamfers on top edges, other curved areas, and on a 4th axis the A angle that brings each side
//! up and the diameters turned about the A axis.
//!
//! Everything is read from the record: a face's analytic surface (`face_surface`), its loops of
//! oriented edges and the faces on the other side of each edge, the edges' exact curves (lines,
//! circles through `edge_circle`, the control nets that bound a spline) and the loops read in a
//! plane by `Body::planar_edges` (lines and arcs exactly). Whether a round face is a hole or a
//! boss is read from the planar faces at its ends (in an inner loop with the wall going into the
//! material, or a floor's outer loop), else by classifying a point beside it.
//!
//! Each feature gets an id (`hole:0`, `pocket:1`, `face:2`, `profile:0`…), numbered per kind in
//! a fixed order (setup, then from the top down, then by position), so the same part and the
//! same parameters give the same ids. `cam.job` operations name them through [`Lazy`], which
//! finds the features the way `cam.features` does with the job's bodies, origin and machine.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::{PI, TAU};

use serde_json::Value;
use solidcore_record::brep::{Brep, Curve, Loop as BrepLoop};
use solidcore_record::{Loop2, Plane, Region2, Seg2, Vec2, Vec3};
use solidcraft_cam::indexed::{frame_for_index, index_for_axis};
use solidcraft_cam::kinematics::Machine;
use solidcraft_cam::{Frame, Heights};
use solidcraft_geom::prelude::*;
use solidcraft_kernel::{Body, EdgeCircle, FaceSurface, PlanarPiece, PointClass, edge_circle, face_surface};

use super::{Ctx, MAX_BODY_NAMES, TOLERANCE_RANGE, finite_point, num, parse, v3};
use crate::cmd::CommandSpec;
use crate::params::bad;
use crate::schema::cam::{HeightsPartial, SideSpec};
use crate::schema::cam_features::{
    AxisSpec, CamFeaturesParams, CamFeaturesReply, Countersink, EdgeBreakReport, FaceReport, HoleGroup, HoleReport, IslandReport, PocketReport,
    ProfileReport, SetupReport, SurfaceReport, TurnedReport,
};
use crate::schema::{TypedContract, output_schema_for};
use crate::{EngineError, Result, Session};

const FEATURES_CONTRACT: TypedContract<CamFeaturesParams> = TypedContract::new();

pub static COMMANDS: &[CommandSpec] = &[CommandSpec::new("cam.features", "CAM Features", crate::checked!("cam.features", features))
    .noundo()
    .contract(&FEATURES_CONTRACT)
    .output(output_schema_for::<CamFeaturesReply>)
    .params(
        "bodies?: [names] (default every visible body); origin?: [x,y,z] (work zero, as a job's); axis?: \"z\" (default) | \"a\" (a 4th \
         axis about work X: every side square to X, at the A angle that brings it up); tolerance?: mm (default 0.01) → setups: [{a_deg, \
         tool_axis, top_z, bottom_z, bounds, swing_radius?, features}], holes: [{id: \"hole:k\", a_deg?, center, center_work, diameter, \
         top_z, bottom_z, depth, through, floor: through|flat|cone|round, tip_z?, counterbore?, counterbore_of?, countersink?}], hole_groups, \
         pockets: [{id: \"pocket:k\", floor_z, top_z, depth, outline, area, min_corner_radius, sharp_corners, walls_vertical, islands}], \
         faces: [{id: \"face:k\", z, area, role: top|plateau|step|island_top|pocket_floor|hole_floor}], profiles: [{id: \"profile:k\", top_z, \
         bottom_z, outline, min_concave_radius?}], fillets, chamfers, surfaces, turned (axis a), notes. Read from the exact B-rep, in \
         each setup's frame (its Z the tool axis). A cam.job pocket or contour takes `feature: \"pocket:0\"` (heights default from the \
         feature); the ids are the same for the same bodies, origin and axis",
    )];

/// Most setups (A angles) looked at.
const MAX_SETUPS: usize = 360;
/// Most faces a body may have to be looked at.
const MAX_FACES: usize = 20_000;
/// Directions within this of each other (as 1 - cos) are the same.
const PARALLEL: f64 = 1e-9;
/// A face whose normal's Z is within this of 0 is a wall.
const SQUARE: f64 = 1e-7;
/// Turning more than this (radians) where two pieces of an outline meet is a corner.
const CORNER: f64 = 0.02;
/// Most points of a curve sampled into an outline.
const MAX_SAMPLES: usize = 720;

/// `cam.features`.
fn features(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.features";
    let params: CamFeaturesParams = parse(cmd, p, "")?;
    let tol = tolerance(cmd, params.tolerance)?;
    let origin = params.origin.unwrap_or([0.0; 3]);
    finite_point(&origin, "`origin`").map_err(|e| bad(cmd, e))?;
    let axis = params.axis.unwrap_or_default();
    let machine = match axis {
        AxisSpec::Z => Machine::three_axis(),
        AxisSpec::A => Machine::four_axis_a(),
    };
    let names = part_names(cmd, s, params.bodies.as_deref(), &[])?;
    let mut set = recognize(cmd, s, &names, v3(origin), axis, tol, &machine)?;
    if params.picks.unwrap_or(false) {
        let st = s.world_state()?;
        let ids: Vec<String> = set.geo.keys().cloned().collect();
        let mut picks = BTreeMap::new();
        for id in ids {
            let list = super::picks::feature_picks(&set, &st, &id);
            if !list.is_empty() {
                picks.insert(id, list);
            }
        }
        set.reply.picks = picks;
    }
    serde_json::to_value(&set.reply).map_err(|e| EngineError::Other(e.to_string()))
}

fn tolerance(cmd: &str, t: Option<f64>) -> Result<f64> {
    let tol = t.unwrap_or(0.01);
    if tol.is_finite() && (TOLERANCE_RANGE.0..=TOLERANCE_RANGE.1).contains(&tol) {
        Ok(tol)
    } else {
        Err(bad(cmd, "the `tolerance` must be from 0.0001 to 1 mm"))
    }
}

/// The part's bodies: `given`, or every visible body not in `fixtures`, each one there.
pub(super) fn part_names(cmd: &str, s: &Session, given: Option<&[String]>, fixtures: &[String]) -> Result<Vec<String>> {
    let st = s.world_state()?;
    let names: Vec<String> = match given {
        Some(b) => b.to_vec(),
        None => st.bodies.iter().map(|b| b.name.clone()).filter(|n| !fixtures.contains(n) && s.body_visible(n)).collect(),
    };
    if names.len() > MAX_BODY_NAMES {
        return Err(bad(cmd, format!("name at most {MAX_BODY_NAMES} bodies")));
    }
    if names.is_empty() {
        return Err(bad(cmd, "there is no body to machine: the design has no visible body, or give `bodies`"));
    }
    for n in &names {
        if !st.bodies.iter().any(|b| &b.name == n) {
            return Err(bad(cmd, crate::cmd::no_body(n, st.bodies.iter().map(|b| b.name.as_str()))));
        }
    }
    let mut seen = BTreeSet::new();
    if let Some(twice) = names.iter().find(|n| !seen.insert(n.as_str())) {
        return Err(bad(cmd, format!("body `{twice}` is named twice")));
    }
    Ok(names)
}

/// One tool direction, as recognised.
#[derive(Clone, Debug)]
pub(crate) struct Setup {
    pub a_deg: f64,
    /// The rotary angles (radians) of the machine for it.
    pub rotary: Vec<f64>,
    pub frame: Frame,
    pub top: f64,
    pub bottom: f64,
    pub swing: Option<f64>,
    /// The highest point of the job's fixtures along this setup's tool axis, from their exact
    /// boxes: the clearance height left out goes above it.
    pub fixture_top: Option<f64>,
}

/// What kind of outline a feature gives a 2D operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Kind {
    Hole,
    Pocket,
    Face,
    Profile,
    /// No outline: fillets, chamfers, surfaces, turned diameters.
    Area,
}

/// A feature's exact geometry, in its setup's frame.
#[derive(Clone, Debug)]
pub(crate) struct Geo {
    pub kind: Kind,
    pub setup: usize,
    pub outer: Option<Loop2>,
    /// Islands with the top of their walls.
    pub islands: Vec<(Loop2, f64)>,
    pub top: f64,
    pub bottom: f64,
    pub bounds: [[f64; 2]; 2],
    /// A hole's centre, diameter and whether it goes through.
    pub hole: Option<(Vec2, f64, bool)>,
    /// What a 2D operation on it is told when its depth is zero.
    pub flat_top: bool,
}

/// The features of a part.
pub(crate) struct FeatureSet {
    pub reply: CamFeaturesReply,
    pub setups: Vec<Setup>,
    pub geo: BTreeMap<String, Geo>,
}

impl FeatureSet {
    /// The feature `id`, or a refusal saying which there are.
    pub fn get(&self, id: &str) -> std::result::Result<&Geo, String> {
        let id = id.trim();
        if let Some(g) = self.geo.get(id) {
            return Ok(g);
        }
        let kind = id.split(':').next().unwrap_or_default();
        let same: Vec<&str> = self.geo.keys().filter(|k| k.split(':').next() == Some(kind)).map(String::as_str).collect();
        let known = match (same.first(), same.last()) {
            (Some(a), Some(b)) if a == b => format!("this part has only `{a}`"),
            (Some(a), Some(b)) => format!("this part has `{a}` to `{b}`"),
            _ => {
                let kinds: BTreeSet<&str> = self.geo.keys().filter_map(|k| k.split(':').next()).collect();
                if kinds.is_empty() {
                    "this part has no machinable features".to_string()
                } else {
                    format!("ids are kind:number, of the kinds {}", kinds.into_iter().collect::<Vec<_>>().join(", "))
                }
            }
        };
        let along = match self.reply.axis {
            AxisSpec::Z => {
                "found along Z only, as the job's machine has no A axis about X (with `machine: \"4axis_a\"` the sides are found too; \
                 `cam.features {axis: \"z\"}` gives these ids)"
            }
            AxisSpec::A => "found on every side about X, as the job's machine has an A axis (`cam.features {axis: \"a\"}` gives these ids)",
        };
        let shown: String = if id.chars().count() > 40 { format!("{}…", id.chars().take(40).collect::<String>()) } else { id.to_string() };
        Err(format!("no feature `{shown}`: {known}; features are {along}, with the job's bodies and origin"))
    }

    pub fn setup(&self, g: &Geo) -> Option<&Setup> {
        self.setups.get(g.setup)
    }
}

/// The features of a job's part, found the first time an operation names one.
pub(crate) struct Lazy {
    bodies: Option<Vec<String>>,
    fixtures: Vec<String>,
    origin: Vec3,
    tol: f64,
    machine: Machine,
    cell: std::sync::OnceLock<std::result::Result<FeatureSet, String>>,
}

impl Lazy {
    pub fn new(bodies: Option<Vec<String>>, fixtures: Vec<String>, origin: Vec3, tol: f64, machine: &Machine) -> Lazy {
        Lazy { bodies, fixtures, origin, tol, machine: machine.clone(), cell: std::sync::OnceLock::new() }
    }

    /// The axis features are found along for `machine`: `a` with one rotary about X, else `z`.
    pub fn axis_of(machine: &Machine) -> AxisSpec {
        match machine.rotaries.as_slice() {
            [one] if one.direction.normalized().is_some_and(|d| d.x.abs() > 1.0 - PARALLEL) => AxisSpec::A,
            _ => AxisSpec::Z,
        }
    }

    pub fn get(&self, cmd: &str, s: &Session) -> Result<&FeatureSet> {
        let found = self.cell.get_or_init(|| {
            let run = || -> Result<FeatureSet> {
                let names = part_names(cmd, s, self.bodies.as_deref(), &self.fixtures)?;
                let mut set = recognize(cmd, s, &names, self.origin, Lazy::axis_of(&self.machine), self.tol, &self.machine)?;
                fixture_tops(s, &self.fixtures, self.origin, &mut set.setups)?;
                Ok(set)
            };
            run().map_err(|e| match e {
                EngineError::BadParams { msg, .. } => msg,
                other => other.to_string(),
            })
        });
        found.as_ref().map_err(|e| bad(cmd, format!("finding the part's features: {e}")))
    }
}

/// What a 2D operation cuts for a feature: its regions, heights and the side a contour runs on.
pub(crate) struct FeatureCut {
    pub regions: Vec<Region2>,
    pub heights: Heights,
    pub side: SideSpec,
}

/// A hole as a drilling or boring operation takes it, in its setup's frame (`hole_targets`).
#[derive(Clone, Debug)]
#[allow(dead_code)] // The drilling and boring operations read every field; milling reads the circle.
pub(crate) struct HoleTarget {
    pub id: String,
    pub center: Vec2,
    pub top: f64,
    pub bottom: f64,
    pub diameter: f64,
    pub through: bool,
}

/// The highest point of the bodies `fixtures` along each setup's tool axis: the corners of each
/// body's exact box, in the setup's frame.
fn fixture_tops(s: &Session, fixtures: &[String], origin: Vec3, setups: &mut [Setup]) -> Result<()> {
    if fixtures.is_empty() {
        return Ok(());
    }
    let st = s.world_state()?;
    for n in fixtures.iter().take(MAX_BODY_NAMES) {
        let Some(b) = st.bodies.iter().find(|b| &b.name == n) else { continue };
        let bx = b.body.exact_bounds().unwrap_or_else(|_| b.body.outline_bounds());
        let (lo, hi) = (bx.min - origin, bx.max - origin);
        if !(lo.is_finite() && hi.is_finite()) {
            continue;
        }
        for su in setups.iter_mut() {
            for k in 0..8u8 {
                let p = Vec3::new(if k & 1 == 0 { lo.x } else { hi.x }, if k & 2 == 0 { lo.y } else { hi.y }, if k & 4 == 0 { lo.z } else { hi.z });
                let z = su.frame.to_local(p).z;
                su.fixture_top = Some(su.fixture_top.map_or(z, |m: f64| m.max(z)));
            }
        }
    }
    Ok(())
}

/// The heights of an operation on a feature of `setup` whose material runs from `top` down to
/// `bottom`, `given` laid over them. Those left out are worked out from those given: the retract
/// 3 mm above the material (the setup's top or the `top` given, the higher), the clearance 10 mm
/// above it, 5 mm above the retract given, 5 mm past the swing on a 4th axis and 5 mm above the
/// fixtures. Out of order, the refusal names each height and where it came from.
pub(super) fn feature_heights(setup: &Setup, top: f64, bottom: f64, given: Option<&HeightsPartial>) -> std::result::Result<Heights, String> {
    let g = given.copied().unwrap_or_default();
    let top_v = g.top.unwrap_or(top);
    let bottom_v = g.bottom.unwrap_or(bottom);
    let material = setup.top.max(top_v);
    let retract = g.retract.unwrap_or(material + 3.0);
    let mut clearance = (material + 10.0).max(retract + 5.0);
    if let Some(sw) = setup.swing {
        clearance = clearance.max(sw + 5.0);
    }
    if let Some(fx) = setup.fixture_top {
        clearance = clearance.max(fx + 5.0);
    }
    let clearance = g.clearance.unwrap_or(clearance);
    let h = Heights { clearance, retract, top: top_v, bottom: bottom_v };
    if h.clearance < h.retract || h.retract < h.top || h.top < h.bottom {
        let from = |v: Option<f64>, x: f64, k: &str, d: &str| format!("{k} {} ({})", num(x), if v.is_some() { "given" } else { d });
        return Err(format!(
            "the heights must be ordered clearance ≥ retract ≥ top ≥ bottom; here {}, {}, {}, {}",
            from(g.clearance, h.clearance, "clearance", "the default"),
            from(g.retract, h.retract, "retract", "the default"),
            from(g.top, h.top, "top", "the feature's"),
            from(g.bottom, h.bottom, "bottom", "the feature's")
        ));
    }
    Ok(h)
}

/// The rotary angles and A angle of the setup feature `id` is in, for an operation that names
/// it outside an `indexed` one: `None` for a feature at A = 0.
pub(super) fn feature_index(ctx: &Ctx<'_>, op: &str, id: &str) -> Result<Option<(Vec<f64>, f64)>> {
    let set = ctx.features.get(ctx.cmd, ctx.s)?;
    let g = set.get(id).map_err(|e| ctx.bad(op, e))?;
    let Some(setup) = set.setup(g) else { return Ok(None) };
    Ok((setup.rotary.iter().any(|a| a.abs() > 1e-12)).then(|| (setup.rotary.clone(), setup.a_deg)))
}

/// The feature `id` checked against the frame the operation runs in.
fn in_frame<'s>(ctx: &Ctx<'_>, set: &'s FeatureSet, op: &str, id: &str, frame: &Frame) -> Result<&'s Geo> {
    let g = set.get(id).map_err(|e| ctx.bad(op, e))?;
    let Some(setup) = set.setup(g) else { return Err(ctx.bad(op, format!("feature `{id}` has no setup"))) };
    let same = frame.z.dot(setup.frame.z) > 1.0 - PARALLEL && frame.origin.dist(setup.frame.origin) <= 1e-6;
    if !same {
        let a = num(setup.a_deg);
        return Err(ctx.bad(
            op,
            format!(
                "feature `{id}` is machined at A = {a}° (its tool axis is work [{}, {}, {}]) but this operation runs with its tool along work \
                 [{}, {}, {}]: put it in an `indexed` operation with `a_deg: {a}`, or give it outside `indexed` and it is indexed there itself",
                num(setup.frame.z.x),
                num(setup.frame.z.y),
                num(setup.frame.z.z),
                num(frame.z.x),
                num(frame.z.y),
                num(frame.z.z)
            ),
        ));
    }
    Ok(g)
}

/// The regions, heights and contour side of a 2D operation on feature `id`, in `frame`. A pocket
/// passes its tool's radius as `facing`: on a face at the setup's top (nothing of the part above
/// it) the pocket then covers the face's extent grown by the radius and 1 mm, so the tool's edge
/// passes the face's edges instead of stopping a radius inside them.
pub(super) fn feature_cut(
    ctx: &Ctx<'_>,
    op: &str,
    id: &str,
    frame: &Frame,
    given: Option<&HeightsPartial>,
    facing: Option<f64>,
) -> Result<FeatureCut> {
    let set = ctx.features.get(ctx.cmd, ctx.s)?;
    let g = in_frame(ctx, set, op, id, frame)?;
    let Some(setup) = set.setup(g) else { return Err(ctx.bad(op, format!("feature `{id}` has no setup"))) };
    let outer = match (g.kind, &g.outer) {
        (Kind::Hole, _) => {
            let hole = hole_targets(ctx, op, &[id.to_string()], frame)?;
            let Some(h) = hole.first() else { return Err(ctx.bad(op, format!("`{id}` is not a hole"))) };
            Loop2::circle(h.center, h.diameter * 0.5)
        }
        (Kind::Area, _) | (_, None) => {
            let [lo, hi] = g.bounds;
            return Err(ctx.bad(
                op,
                format!(
                    "`{id}` has no outline for a 2D operation: finish it with a `parallel` or `waterline` operation with \
                     `boundary: {{\"rect\": [[{}, {}], [{}, {}]]}}` and `heights.bottom` {}",
                    num(lo[0]),
                    num(lo[1]),
                    num(hi[0]),
                    num(hi[1]),
                    num(g.bottom)
                ),
            ));
        }
        (_, Some(o)) => o.clone(),
    };
    let heights = feature_heights(setup, g.top, g.bottom, given).map_err(|e| ctx.bad(op, e))?;
    if !(heights.top > heights.bottom + 1e-9) {
        let why = if g.flat_top {
            format!(
                "`{id}` is a face at Z {} with nothing above it in the model: give `heights.top` (the stock's top) to face it, or use its \
                 pocket or the profile round it",
                num(g.bottom)
            )
        } else {
            format!("`heights.top` ({}) must be above `heights.bottom` ({})", num(heights.top), num(heights.bottom))
        };
        return Err(ctx.bad(op, why));
    }
    // An island is in the way only where the cut goes below its top.
    let holes: Vec<Loop2> = g.islands.iter().filter(|(_, top)| *top > heights.bottom + ctx.tol).map(|(l, _)| l.clone()).collect();
    let outer = match facing {
        Some(r) if g.kind == Kind::Face && g.flat_top && holes.is_empty() && (g.bottom - setup.top).abs() <= ctx.tol && r.is_finite() && r > 0.0 => {
            let m = r + 1.0;
            let [lo, hi] = g.bounds;
            Loop2::polygon(&[
                Vec2::new(lo[0] - m, lo[1] - m),
                Vec2::new(hi[0] + m, lo[1] - m),
                Vec2::new(hi[0] + m, hi[1] + m),
                Vec2::new(lo[0] - m, hi[1] + m),
            ])
        }
        _ => outer,
    };
    let side = match g.kind {
        Kind::Profile => SideSpec::Outside,
        _ => SideSpec::Inside,
    };
    Ok(FeatureCut { regions: vec![Region2 { outer, holes }], heights, side })
}

/// The regions and heights of a contour or pocket, and the side a contour takes by default:
/// from the `feature` it names, or from its sketch or polygon and all four `heights`.
pub(super) fn op_cut(
    ctx: &Ctx<'_>,
    op: &str,
    sketch: Option<&crate::schema::SketchRef>,
    profiles: Option<&[usize]>,
    polygon: Option<&[[f64; 2]]>,
    feature: Option<&str>,
    heights: Option<&HeightsPartial>,
    frame: &Frame,
    facing: Option<f64>,
) -> Result<OpCut> {
    if let Some(id) = feature {
        if sketch.is_some() || polygon.is_some() || profiles.is_some() {
            return Err(ctx.bad(op, "give a `feature`, a `sketch` or a `polygon`, one of them"));
        }
        let cut = feature_cut(ctx, op, id, frame, heights, facing)?;
        return Ok(OpCut { regions: cut.regions, heights: cut.heights, side: Some(cut.side) });
    }
    let regions = ctx.regions(op, sketch, profiles, polygon, frame)?;
    let h = heights.copied().unwrap_or_default();
    match (h.clearance, h.retract, h.top, h.bottom) {
        (Some(clearance), Some(retract), Some(top), Some(bottom)) => {
            Ok(OpCut { regions, heights: Heights { clearance, retract, top, bottom }, side: None })
        }
        _ => {
            let missing: Vec<String> = [("clearance", h.clearance), ("retract", h.retract), ("top", h.top), ("bottom", h.bottom)]
                .iter()
                .filter(|(_, v)| v.is_none())
                .map(|(k, _)| format!("`{k}`"))
                .collect();
            Err(ctx.bad(
                op,
                format!(
                    "give `heights` with {} (an operation on a sketch or polygon needs clearance, retract, top and bottom; one on a \
                     `feature` takes them from it)",
                    missing.join(", ")
                ),
            ))
        }
    }
}

/// What [`op_cut`] gives.
pub(crate) struct OpCut {
    pub regions: Vec<Region2>,
    pub heights: Heights,
    pub side: Option<SideSpec>,
}

/// The rotary angles and A angle to index an operation naming `feature` at, when it is on
/// another side than A = 0.
pub(super) fn op_index(ctx: &Ctx<'_>, op: &str, feature: Option<&str>) -> Result<Option<(Vec<f64>, f64)>> {
    match feature {
        Some(id) => feature_index(ctx, op, id),
        None => Ok(None),
    }
}

/// The holes `ids` in `frame`, for a drilling or boring operation (each must be a hole of the
/// setup the operation runs in).
pub(super) fn hole_targets(ctx: &Ctx<'_>, op: &str, ids: &[String], frame: &Frame) -> Result<Vec<HoleTarget>> {
    let set = ctx.features.get(ctx.cmd, ctx.s)?;
    let mut out = Vec::with_capacity(ids.len());
    for id in ids {
        let g = in_frame(ctx, set, op, id, frame)?;
        let Some((center, diameter, through)) = g.hole else {
            return Err(ctx.bad(op, format!("`{id}` is not a hole (`features` takes ids of the form hole:k)")));
        };
        out.push(HoleTarget { id: id.trim().to_string(), center, top: g.top, bottom: g.bottom, diameter, through });
    }
    Ok(out)
}

// ---------------------------------------------------------------------------------------------
// Recognition.

/// An edge's exact shape, for its extent along a direction.
enum EdgeShape {
    Circle(EdgeCircle),
    /// Points whose convex hull holds the edge (its ends, a spline's control points).
    Hull(Vec<Vec3>),
}

/// How the faces round a loop of a face lie.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Lie {
    Up,
    Down,
    Mixed,
}

/// A loop of a planar face read in a setup's plane.
struct LoopGeo {
    segs: Vec<Seg2>,
    /// Signed area of the outline.
    area: f64,
    /// The faces on the other side of its edges.
    beside: Vec<usize>,
}

/// One body's topology, in work coordinates.
struct Topo<'a> {
    name: String,
    body: &'a Body,
    record: &'a Brep,
    origin: Vec3,
    surfaces: Vec<Option<FaceSurface>>,
    shapes: Vec<EdgeShape>,
    /// The faces each edge bounds.
    edge_faces: Vec<Vec<usize>>,
    /// The edges of each face, each loop as (edge, forward).
    loops: Vec<Vec<Vec<(usize, bool)>>>,
}

fn p3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

impl<'a> Topo<'a> {
    fn new(name: &str, body: &'a Body, origin: Vec3) -> Topo<'a> {
        let record = body.record();
        let vertex = |v: u32| record.vertices.get(v as usize).map(|v| p3(v.point) - origin);
        let shapes = record
            .edges
            .iter()
            .enumerate()
            .map(|(i, e)| {
                if let Some(c) = edge_circle(body, i) {
                    return EdgeShape::Circle(EdgeCircle { center: c.center - origin, start: c.start - origin, end: c.end - origin, ..c });
                }
                let mut pts: Vec<Vec3> = e.vertices.iter().filter_map(|v| vertex(*v)).collect();
                hull_points(&e.curve, origin, &mut pts, 0);
                EdgeShape::Hull(pts)
            })
            .collect();
        let mut edge_faces: Vec<Vec<usize>> = vec![Vec::new(); record.edges.len()];
        let loops: Vec<Vec<Vec<(usize, bool)>>> = record
            .faces
            .iter()
            .enumerate()
            .map(|(fi, f)| {
                f.loops
                    .iter()
                    .filter_map(|l| match l {
                        BrepLoop::Edges { edges } => Some(
                            edges
                                .iter()
                                .map(|oe| {
                                    if let Some(list) = edge_faces.get_mut(oe.edge as usize)
                                        && !list.contains(&fi)
                                    {
                                        list.push(fi);
                                    }
                                    (oe.edge as usize, oe.forward)
                                })
                                .collect(),
                        ),
                        BrepLoop::Vertex { .. } => None,
                    })
                    .collect()
            })
            .collect();
        let surfaces = (0..record.faces.len())
            .map(|f| {
                face_surface(body, f).map(|s| match s {
                    FaceSurface::Plane { normal, point } => FaceSurface::Plane { normal, point: point - origin },
                    FaceSurface::Cylinder { origin: o, axis, radius } => FaceSurface::Cylinder { origin: o - origin, axis, radius },
                    FaceSurface::Cone { apex, axis, half_angle } => FaceSurface::Cone { apex: apex - origin, axis, half_angle },
                    FaceSurface::Sphere { center, radius } => FaceSurface::Sphere { center: center - origin, radius },
                    FaceSurface::Torus { center, axis, major, minor } => FaceSurface::Torus { center: center - origin, axis, major, minor },
                })
            })
            .collect();
        Topo { name: name.to_string(), body, record, origin, surfaces, shapes, edge_faces, loops }
    }

    fn surface(&self, f: usize) -> Option<&FaceSurface> {
        self.surfaces.get(f).and_then(Option::as_ref)
    }

    fn face_edges(&self, f: usize) -> impl Iterator<Item = usize> + '_ {
        self.loops.get(f).into_iter().flatten().flatten().map(|(e, _)| *e)
    }

    /// The range of `p · dir` over edge `e`, exactly for lines and circles.
    fn edge_range(&self, e: usize, dir: Vec3) -> Option<(f64, f64)> {
        match self.shapes.get(e)? {
            EdgeShape::Circle(c) => Some(circle_range(c, dir)),
            EdgeShape::Hull(pts) => pts.iter().map(|p| p.dot(dir)).fold(None, |acc, v| Some(widen(acc, v, v))),
        }
    }

    /// The range of `p · dir` over face `f`'s boundary, and with `whole` over the face: the
    /// boundary bounds a face of lines and circles on a plane, cylinder or cone; a sphere, torus
    /// or free-form face may rise past it, so its own extent is added.
    fn face_range(&self, f: usize, dir: Vec3, whole: bool) -> Option<(f64, f64)> {
        let mut r = self.face_edges(f).filter_map(|e| self.edge_range(e, dir)).fold(None, |acc, (lo, hi)| Some(widen(acc, lo, hi)));
        if whole {
            let extra = match self.surface(f) {
                Some(FaceSurface::Sphere { center, radius }) => Some((center.dot(dir) - radius, center.dot(dir) + radius)),
                Some(FaceSurface::Torus { center, axis, major, minor }) => {
                    let c = center.dot(dir);
                    let along = axis.vec().dot(dir).abs();
                    let reach = major * (1.0 - along * along).max(0.0).sqrt() + minor;
                    Some((c - reach, c + reach))
                }
                Some(_) => None,
                None => self.record.faces.get(f).and_then(|face| {
                    let mut pts = Vec::new();
                    surface_hull(face.parameters(), self.origin, &mut pts);
                    pts.iter().map(|p| p.dot(dir)).fold(None, |acc, v| Some(widen(acc, v, v)))
                }),
            };
            if let Some((lo, hi)) = extra {
                r = Some(widen(r, lo, hi));
            }
        }
        r
    }

    /// The largest distance of face `f`'s edges from the line through `origin` along the unit
    /// `axis`: circles exactly, other edges by their control points (an upper bound).
    fn edges_reach(&self, f: usize, origin: Vec3, axis: Vec3) -> f64 {
        let off = |p: Vec3| {
            let d = p - origin;
            (d - axis * d.dot(axis)).len()
        };
        let mut out: f64 = 0.0;
        for e in self.face_edges(f) {
            match self.shapes.get(e) {
                Some(EdgeShape::Circle(c)) => out = out.max(off(c.center) + c.radius),
                Some(EdgeShape::Hull(pts)) => out = pts.iter().fold(out, |m, p| m.max(off(*p))),
                None => {}
            }
        }
        out
    }

    /// The largest distance of face `f` from the work X axis (an upper bound).
    fn face_swing(&self, f: usize) -> f64 {
        let yz = |p: Vec3| p.y.hypot(p.z);
        let mut out: f64 = 0.0;
        for e in self.face_edges(f) {
            match self.shapes.get(e) {
                Some(EdgeShape::Circle(c)) => out = out.max(yz(c.center) + c.radius),
                Some(EdgeShape::Hull(pts)) => out = pts.iter().fold(out, |m, p| m.max(yz(*p))),
                None => {}
            }
        }
        match self.surface(f) {
            Some(FaceSurface::Sphere { center, radius }) => out = out.max(yz(*center) + radius),
            Some(FaceSurface::Torus { center, major, minor, .. }) => out = out.max(yz(*center) + major + minor),
            Some(_) => {}
            None => {
                if let Some(face) = self.record.faces.get(f) {
                    let mut pts = Vec::new();
                    surface_hull(face.parameters(), self.origin, &mut pts);
                    out = pts.iter().fold(out, |m, p| m.max(yz(*p)));
                }
            }
        }
        out
    }

    /// Face `f`'s loops read in the plane at height `z` of `frame` (square to its Z).
    fn face_loops(&self, f: usize, frame: &Frame, z: f64, tol: f64) -> Vec<LoopGeo> {
        let Some(loops) = self.loops.get(f) else { return Vec::new() };
        let plane = Plane { origin: frame.to_world(Vec3::new(0.0, 0.0, z)) + self.origin, x: frame.x, y: frame.y };
        let all: Vec<usize> = loops.iter().flatten().map(|(e, _)| *e).collect();
        let read = self.body.planar_edges(&plane, &all);
        let mut at = 0usize;
        let mut out = Vec::with_capacity(loops.len());
        for lp in loops {
            let mut segs: Vec<Seg2> = Vec::new();
            let mut beside: Vec<usize> = Vec::new();
            for (e, forward) in lp {
                let pieces = read.get(at).cloned().flatten();
                at += 1;
                let mut these: Vec<Seg2> = match pieces {
                    Some(ps) => ps.iter().flat_map(piece_segs).collect(),
                    None => self.own_segs(*e, frame),
                };
                if !forward {
                    these.reverse();
                    for s in &mut these {
                        *s = reversed(s);
                    }
                }
                segs.extend(these);
                for g in self.edge_faces.get(*e).into_iter().flatten() {
                    if *g != f && !beside.contains(g) {
                        beside.push(*g);
                    }
                }
            }
            let area = loop_area(&segs, tol);
            out.push(LoopGeo { segs, area, beside });
        }
        out
    }

    /// Edge `e` in `frame`'s XY from its shape: a line between its ends, or its arc.
    fn own_segs(&self, e: usize, frame: &Frame) -> Vec<Seg2> {
        let local = |p: Vec3| {
            let l = frame.to_local(p);
            Vec2::new(l.x, l.y)
        };
        match self.shapes.get(e) {
            Some(EdgeShape::Circle(c)) => {
                let n = frame.dir_to_local(c.axis.vec());
                let center = local(c.center);
                if n.z.abs() > 1.0 - PARALLEL {
                    let s = local(c.start) - center;
                    let sweep = if n.z > 0.0 { c.angle } else { -c.angle };
                    vec![Seg2::Arc { center, radius: c.radius, start: s.y.atan2(s.x), sweep }]
                } else {
                    vec![Seg2::Line { a: local(c.start), b: local(c.end) }]
                }
            }
            _ => {
                let Some(edge) = self.record.edges.get(e) else { return Vec::new() };
                let at = |v: u32| self.record.vertices.get(v as usize).map(|v| local(p3(v.point) - self.origin));
                match (at(edge.vertices[0]), at(edge.vertices[1])) {
                    (Some(a), Some(b)) if a.dist(b) > 1e-12 => vec![Seg2::Line { a, b }],
                    _ => Vec::new(),
                }
            }
        }
    }
}

fn widen(acc: Option<(f64, f64)>, lo: f64, hi: f64) -> (f64, f64) {
    match acc {
        Some((a, b)) => (a.min(lo), b.max(hi)),
        None => (lo, hi),
    }
}

/// Points whose hull holds `curve` (its control net), moved by `-origin`.
fn hull_points(curve: &Curve, origin: Vec3, out: &mut Vec<Vec3>, depth: usize) {
    match curve {
        Curve::Line { .. } | Curve::Circle { .. } => {}
        Curve::Ellipse { frame, radii } => {
            let (o, x, y) = (p3(frame.origin), p3(frame.x_axis), p3(frame.y_axis));
            for (sx, sy) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
                out.push(o + x * (radii[0] * sx) + y * (radii[1] * sy) - origin);
            }
        }
        Curve::Polyline { points } | Curve::Spline { points, .. } => out.extend(points.iter().take(100_000).map(|p| p3(*p) - origin)),
        Curve::RationalSpline { homogeneous, .. } => out.extend(
            homogeneous.iter().take(100_000).filter(|h| h[3].abs() > 1e-300).map(|h| Vec3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3]) - origin),
        ),
        Curve::SurfaceIntersection { leader, .. } if depth < 4 => hull_points(leader, origin, out, depth + 1),
        Curve::SurfaceIntersection { .. } => {}
    }
}

/// The control net of a free-form surface, moved by `-origin` (nothing for the analytic kinds).
fn surface_hull(surface: &solidcore_record::brep::Surface, origin: Vec3, out: &mut Vec<Vec3>) {
    use solidcore_record::brep::Surface as S;
    match surface {
        S::Spline { points, .. } => out.extend(points.iter().flatten().take(1_000_000).map(|p| p3(*p) - origin)),
        S::RationalSpline { homogeneous, .. } => out.extend(
            homogeneous
                .iter()
                .flatten()
                .take(1_000_000)
                .filter(|h| h[3].abs() > 1e-300)
                .map(|h| Vec3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3]) - origin),
        ),
        _ => {}
    }
}

/// The range of `p · dir` over the arc `c`.
fn circle_range(c: &EdgeCircle, dir: Vec3) -> (f64, f64) {
    let ends = (c.start.dot(dir), c.end.dot(dir));
    let mut r = (ends.0.min(ends.1), ends.0.max(ends.1));
    let Some(u) = (c.start - c.center).normalized() else { return r };
    let w = c.axis.vec().cross(u);
    let (a, b) = (u.dot(dir), w.dot(dir));
    let amp = a.hypot(b) * c.radius;
    let peak = b.atan2(a).rem_euclid(TAU);
    let within = |t: f64| t.rem_euclid(TAU) <= c.angle + 1e-12;
    let base = c.center.dot(dir);
    if within(peak) {
        r.1 = r.1.max(base + amp);
    }
    if within(peak + PI) {
        r.0 = r.0.min(base - amp);
    }
    r
}

// ---------------------------------------------------------------------------------------------
// 2D outlines.

fn reversed(s: &Seg2) -> Seg2 {
    match *s {
        Seg2::Line { a, b } => Seg2::Line { a: b, b: a },
        Seg2::Arc { center, radius, start, sweep } => Seg2::Arc { center, radius, start: start + sweep, sweep: -sweep },
        Seg2::Cubic { p0, p1, p2, p3 } => Seg2::Cubic { p0: p3, p1: p2, p2: p1, p3: p0 },
        Seg2::Conic { a, apex, b, w } => Seg2::Conic { a: b, apex, b: a, w },
        Seg2::Bezier { n, p } => {
            let mut q = p;
            let k = usize::from(n).min(q.len().saturating_sub(1));
            if let Some(head) = q.get_mut(..=k) {
                head.reverse();
            }
            Seg2::Bezier { n, p: q }
        }
    }
}

/// A piece read in a plane as segments: a spline as lines within `tol`.
fn piece_segs(p: &PlanarPiece) -> Vec<Seg2> {
    match p {
        PlanarPiece::Seg(s) => vec![*s],
        PlanarPiece::Spline(sp) => {
            let (Some(t0), Some(t1)) = (sp.knots.get(sp.degree), sp.knots.len().checked_sub(sp.degree + 1).and_then(|i| sp.knots.get(i))) else {
                return Vec::new();
            };
            let n = (sp.points.len() * 16).clamp(16, MAX_SAMPLES);
            let pts: Vec<Vec2> = (0..=n).filter_map(|i| spline_point(sp, t0 + (t1 - t0) * i as f64 / n as f64)).collect();
            pts.windows(2)
                .filter_map(|w| match w {
                    [a, b] if a.dist(*b) > 1e-12 => Some(Seg2::Line { a: *a, b: *b }),
                    _ => None,
                })
                .collect()
        }
    }
}

/// A point of a planar B-spline (de Boor, rational).
fn spline_point(sp: &solidcraft_kernel::PlanarSpline, t: f64) -> Option<Vec2> {
    let p = sp.degree;
    let n = sp.points.len();
    if n == 0 || sp.knots.len() != n + p + 1 || sp.weights.len() != n {
        return None;
    }
    let mut k = p;
    while k + 1 < n && *sp.knots.get(k + 1)? <= t {
        k += 1;
    }
    let mut d: Vec<[f64; 3]> = (0..=p)
        .map(|j| {
            let i = (j + k).checked_sub(p)?;
            let (q, w) = (sp.points.get(i)?, *sp.weights.get(i)?);
            Some([q.x * w, q.y * w, w])
        })
        .collect::<Option<_>>()?;
    for r in 1..=p {
        for j in (r..=p).rev() {
            let i = (j + k).checked_sub(p)?;
            let (lo, hi) = (*sp.knots.get(i)?, *sp.knots.get(i + p + 1 - r)?);
            let alpha = if (hi - lo).abs() > 1e-300 { (t - lo) / (hi - lo) } else { 0.0 };
            let (a, b) = (*d.get(j - 1)?, *d.get(j)?);
            *d.get_mut(j)? = [a[0] + (b[0] - a[0]) * alpha, a[1] + (b[1] - a[1]) * alpha, a[2] + (b[2] - a[2]) * alpha];
        }
    }
    let h = d.get(p)?;
    (h[2].abs() > 1e-300).then(|| Vec2::new(h[0] / h[2], h[1] / h[2]))
}

/// A point of `s` at `t` in [0, 1].
fn seg_point(s: &Seg2, t: f64) -> Vec2 {
    match *s {
        Seg2::Line { a, b } => a.lerp(b, t),
        Seg2::Arc { center, radius, start, sweep } => center + Vec2::from_angle(start + sweep * t) * radius,
        Seg2::Cubic { p0, p1, p2, p3 } => {
            let u = 1.0 - t;
            p0 * (u * u * u) + p1 * (3.0 * u * u * t) + p2 * (3.0 * u * t * t) + p3 * (t * t * t)
        }
        Seg2::Conic { a, apex, b, w } => {
            let u = 1.0 - t;
            let (wa, wm, wb) = (u * u, 2.0 * u * t * w, t * t);
            let den = wa + wm + wb;
            if den.abs() > 1e-300 { (a * wa + apex * wm + b * wb) / den } else { a }
        }
        Seg2::Bezier { n, p } => {
            let k = usize::from(n).min(p.len().saturating_sub(1));
            let mut q: Vec<Vec2> = p.iter().take(k + 1).copied().collect();
            for r in 1..=k {
                for i in 0..=(k - r) {
                    if let (Some(a), Some(b)) = (q.get(i).copied(), q.get(i + 1).copied())
                        && let Some(slot) = q.get_mut(i)
                    {
                        *slot = a.lerp(b, t);
                    }
                }
            }
            q.first().copied().unwrap_or_default()
        }
    }
}

/// The points of a loop of segments, each segment's start and the points within `tol` along
/// its curve (the loop's last point, its first again, left out).
/// Points along `l` within `tol` of it (its outline as a polygon).
pub(crate) fn sample_loop(l: &Loop2, tol: f64) -> Vec<Vec2> {
    loop_points(&l.segs, tol.max(1e-4))
}

fn loop_points(segs: &[Seg2], tol: f64) -> Vec<Vec2> {
    let mut out = Vec::new();
    for s in segs {
        let n = match *s {
            Seg2::Line { .. } => 1,
            Seg2::Arc { radius, sweep, .. } => {
                let step = if tol < radius { 2.0 * (1.0 - tol / radius).acos() } else { PI / 4.0 };
                ((sweep.abs() / step.max(1e-3)).ceil() as usize).clamp(1, MAX_SAMPLES)
            }
            _ => 32,
        };
        for i in 0..n {
            let p = seg_point(s, i as f64 / n as f64);
            if out.last().is_none_or(|l: &Vec2| l.dist(p) > 1e-12) {
                out.push(p);
            }
        }
    }
    out
}

fn polygon_area(pts: &[Vec2]) -> f64 {
    pts.iter().zip(pts.iter().cycle().skip(1)).map(|(a, b)| a.cross(*b)).sum::<f64>() * 0.5
}

/// The loop turned so that its area is positive (`ccw`) or negative.
fn oriented(segs: &[Seg2], area: f64, ccw: bool) -> Vec<Seg2> {
    if (area > 0.0) == ccw { segs.to_vec() } else { segs.iter().rev().map(reversed).collect() }
}

fn unit2(v: Vec2) -> Option<Vec2> {
    v.normalized()
}

/// The directions `s` starts and ends in.
fn tangents(s: &Seg2) -> (Option<Vec2>, Option<Vec2>) {
    match *s {
        Seg2::Line { a, b } => (unit2(b - a), unit2(b - a)),
        Seg2::Arc { start, sweep, .. } => {
            let d = |t: f64| Vec2::new(-t.sin(), t.cos()) * sweep.signum();
            (Some(d(start)), Some(d(start + sweep)))
        }
        Seg2::Cubic { p0, p1, p2, p3 } => (unit2(p1 - p0).or(unit2(p2 - p0)), unit2(p3 - p2).or(unit2(p3 - p1))),
        Seg2::Conic { a, apex, b, .. } => (unit2(apex - a), unit2(b - apex)),
        Seg2::Bezier { n, p } => {
            let k = usize::from(n).min(p.len().saturating_sub(1));
            let at = |i: usize| p.get(i).copied().unwrap_or_default();
            (unit2(at(1) - at(0)), k.checked_sub(1).and_then(|j| unit2(at(k) - at(j))))
        }
    }
}

/// The inside corners of a loop with the free space on its left: the smallest radius of an arc
/// turning left (towards the free space), and how many sharp left turns it has.
fn corner_limits(segs: &[Seg2]) -> (Option<f64>, usize) {
    let mut radius: Option<f64> = None;
    let mut sharp = 0;
    for (s, next) in segs.iter().zip(segs.iter().cycle().skip(1)) {
        if let Seg2::Arc { radius: r, sweep, .. } = *s
            && sweep > 1e-9
        {
            radius = Some(radius.map_or(r, |m: f64| m.min(r)));
        }
        if let (Some(a), Some(b)) = (tangents(s).1, tangents(next).0) {
            let turn = a.cross(b).atan2(a.dot(b));
            if turn > CORNER {
                sharp += 1;
            }
        }
    }
    (radius, sharp)
}

fn bounds2(pts: &[Vec2]) -> [[f64; 2]; 2] {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in pts {
        lo = [lo[0].min(p.x), lo[1].min(p.y)];
        hi = [hi[0].max(p.x), hi[1].max(p.y)];
    }
    if lo[0].is_finite() { [lo, hi] } else { [[0.0; 2]; 2] }
}

/// A number as a reply gives it: rounded to 1e-9, no -0.
fn r9(v: f64) -> f64 {
    let r = (v * 1e9).round() / 1e9;
    if r == 0.0 { 0.0 } else { r }
}

fn r2(p: Vec2) -> [f64; 2] {
    [r9(p.x), r9(p.y)]
}

fn rb(b: [[f64; 2]; 2]) -> [[f64; 2]; 2] {
    [[r9(b[0][0]), r9(b[0][1])], [r9(b[1][0]), r9(b[1][1])]]
}

fn r3(p: Vec3) -> [f64; 3] {
    [r9(p.x), r9(p.y), r9(p.z)]
}

/// The signed area a loop encloses: exact for lines and arcs (each piece's share of
/// ½∮(x dy − y dx)), the other curves by points within `tol` along them.
fn loop_area(segs: &[Seg2], tol: f64) -> f64 {
    let mut twice = 0.0;
    for s in segs {
        match *s {
            Seg2::Line { a, b } => twice += a.cross(b),
            Seg2::Arc { center, radius, start, sweep } => {
                let (s0, c0) = start.sin_cos();
                let (s1, c1) = (start + sweep).sin_cos();
                twice += radius * radius * sweep + radius * (center.x * (s1 - s0) - center.y * (c1 - c0));
            }
            _ => {
                let mut pts = loop_points(std::slice::from_ref(s), tol);
                pts.push(seg_point(s, 1.0));
                twice += pts
                    .windows(2)
                    .map(|w| match w {
                        [a, b] => a.cross(*b),
                        _ => 0.0,
                    })
                    .sum::<f64>();
            }
        }
    }
    twice * 0.5
}

/// The extent of a loop: exact for lines and arcs (an arc's ends and the quarter points it
/// passes), the other curves by points within `tol` along them.
fn loop_bounds(segs: &[Seg2], tol: f64) -> [[f64; 2]; 2] {
    let mut pts: Vec<Vec2> = Vec::new();
    for s in segs {
        match *s {
            Seg2::Line { a, b } => pts.extend([a, b]),
            Seg2::Arc { center, radius, start, sweep } => {
                pts.extend([seg_point(s, 0.0), seg_point(s, 1.0)]);
                let (lo, hi) = if sweep >= 0.0 { (start, start + sweep) } else { (start + sweep, start) };
                let mut k = (lo / (PI * 0.5)).ceil();
                while k * PI * 0.5 <= hi + 1e-12 && pts.len() < 100_000 {
                    pts.push(center + Vec2::from_angle(k * PI * 0.5) * radius);
                    k += 1.0;
                }
            }
            _ => {
                pts.extend(loop_points(std::slice::from_ref(s), tol));
                pts.push(seg_point(s, 1.0));
            }
        }
    }
    bounds2(&pts)
}

/// The outline as a reply gives it: counter-clockwise, its exact area and extent.
fn outline(segs: &[Seg2], tol: f64) -> (Vec<[f64; 2]>, f64, [[f64; 2]; 2]) {
    let mut pts = loop_points(segs, tol);
    let area = loop_area(segs, tol);
    if polygon_area(&pts) < 0.0 {
        pts.reverse();
    }
    let b = loop_bounds(segs, tol);
    (pts.into_iter().map(r2).collect(), r9(area.abs()), rb(b))
}

// ---------------------------------------------------------------------------------------------
// The recognition itself.

/// A round face's cylinder: merged faces of one radius on one axis line.
struct Cyl {
    faces: Vec<usize>,
    origin: Vec3,
    axis: Vec3,
    radius: f64,
}

fn cylinders(t: &Topo<'_>, tol: f64) -> Vec<Cyl> {
    let mut out: Vec<Cyl> = Vec::new();
    for (f, s) in t.surfaces.iter().enumerate() {
        let Some(FaceSurface::Cylinder { origin, axis, radius }) = s else { continue };
        let a = axis.vec();
        let same = |c: &Cyl| {
            let off = (*origin - c.origin) - c.axis * (*origin - c.origin).dot(c.axis);
            (c.radius - radius).abs() <= tol && c.axis.dot(a).abs() > 1.0 - PARALLEL && off.len() <= tol
        };
        match out.iter_mut().find(|c| same(c)) {
            Some(c) => c.faces.push(f),
            None => out.push(Cyl { faces: vec![f], origin: *origin, axis: a, radius: *radius }),
        }
    }
    out
}

/// The circle edges of `c`'s faces that run round its axis, each with its height along `up`.
fn cyl_circles<'t>(t: &'t Topo<'_>, c: &Cyl, up: Vec3, tol: f64) -> Vec<(usize, f64, &'t EdgeCircle)> {
    let mut out: Vec<(usize, f64, &EdgeCircle)> = Vec::new();
    for f in &c.faces {
        for e in t.face_edges(*f) {
            if out.iter().any(|(x, ..)| *x == e) {
                continue;
            }
            let Some(EdgeShape::Circle(k)) = t.shapes.get(e) else { continue };
            let off = (k.center - c.origin) - c.axis * (k.center - c.origin).dot(c.axis);
            if (k.radius - c.radius).abs() <= tol && off.len() <= tol && k.axis.vec().dot(c.axis).abs() > 1.0 - PARALLEL {
                out.push((e, (k.center - c.origin).dot(up), k));
            }
        }
    }
    out
}

/// Whether the cylinder is a hole (material outside it): read from the planar faces at its
/// ends, else by classifying a point just outside it.
fn concave(t: &Topo<'_>, c: &Cyl, circles: &[(usize, f64, &EdgeCircle)], up: Vec3, tol: f64) -> Option<bool> {
    let (Some(lo), Some(hi)) = (circles.iter().map(|x| x.1).reduce(f64::min), circles.iter().map(|x| x.1).reduce(f64::max)) else {
        return None;
    };
    for (e, h, _) in circles {
        // From this end the wall runs towards the other.
        let into = if (h - hi).abs() <= tol { -up } else { up };
        if (hi - lo).abs() <= tol {
            continue;
        }
        for g in t.edge_faces.get(*e).into_iter().flatten() {
            if c.faces.contains(g) {
                continue;
            }
            let Some(FaceSurface::Plane { normal, .. }) = t.surface(*g) else { continue };
            let n = normal.vec();
            if n.dot(c.axis).abs() < 1.0 - PARALLEL {
                continue;
            }
            // Which loop of the plane holds this edge: the largest is its outer loop.
            let Some(loops) = t.loops.get(*g) else { continue };
            let Ok(frame) = Frame::from_z(Vec3::ZERO, n) else { continue };
            let z = match t.surface(*g) {
                Some(FaceSurface::Plane { point, .. }) => point.dot(n),
                _ => continue,
            };
            let geo = t.face_loops(*g, &frame, z, tol);
            let Some(outer) = geo.iter().enumerate().max_by(|a, b| a.1.area.abs().total_cmp(&b.1.area.abs())).map(|(i, _)| i) else { continue };
            let Some(at) = loops.iter().position(|l| l.iter().any(|(x, _)| x == e)) else { continue };
            let inner = at != outer;
            return Some(if inner { into.dot(n) < 0.0 } else { into.dot(n) > 0.0 });
        }
    }
    // No planar end: a point just outside the wall, halfway along it.
    let (_, h0, k) = circles.first()?;
    let mid = (lo + hi) * 0.5;
    let p = k.start + up * (mid - h0);
    let radial = ((p - c.origin) - c.axis * (p - c.origin).dot(c.axis)).normalized()?;
    let eps = (tol * 2.0).max(c.radius * 1e-3);
    match t.body.classify_point(p + radial * eps + t.origin).ok()? {
        PointClass::Inside => Some(true),
        PointClass::Outside => Some(false),
        PointClass::OnBoundary => None,
    }
}

/// A face as one setup sees it.
#[derive(Clone)]
struct Seen {
    body: usize,
    face: usize,
    setup: usize,
}

struct Found {
    reply: CamFeaturesReply,
    geo: Vec<(String, Geo)>,
}

#[derive(Default)]
struct Lists {
    holes: Vec<(Vec<i64>, HoleReport, Geo)>,
    pockets: Vec<(Vec<i64>, PocketReport, Geo, usize)>,
    faces: Vec<(Vec<i64>, FaceReport, Geo, Seen)>,
    profiles: Vec<(Vec<i64>, ProfileReport, Geo, Seen)>,
    fillets: Vec<(Vec<i64>, EdgeBreakReport, Geo)>,
    chamfers: Vec<(Vec<i64>, EdgeBreakReport, Geo)>,
    surfaces: Vec<(Vec<i64>, SurfaceReport, Geo)>,
    turned: Vec<(Vec<i64>, TurnedReport)>,
    /// Counterbore pairs: (larger hole, smaller hole) as indices into `holes`.
    counterbores: Vec<(usize, usize)>,
}

/// Every feature of the bodies `names`, along `axis`, on `machine`.
pub(crate) fn recognize(cmd: &str, s: &Session, names: &[String], origin: Vec3, axis: AxisSpec, tol: f64, machine: &Machine) -> Result<FeatureSet> {
    let st = s.world_state()?;
    let mut bodies: Vec<&solidcraft_doc::ModelBody> = Vec::with_capacity(names.len());
    let mut notes: Vec<String> = Vec::new();
    for n in names {
        let Some(b) = st.bodies.iter().find(|b| &b.name == n) else { return Err(bad(cmd, format!("no body `{n}`"))) };
        if b.body.is_mesh() {
            notes.push(format!("body `{n}` is a mesh: it has no exact faces, so no features are found on it"));
            continue;
        }
        if b.body.face_count() > MAX_FACES {
            return Err(bad(cmd, format!("body `{n}` has more than {MAX_FACES} faces")));
        }
        bodies.push(b);
    }
    let topos: Vec<Topo<'_>> = bodies.iter().map(|b| Topo::new(&b.name, &b.body, origin)).collect();

    // The setups: +Z, and with a 4th axis every direction square to X a face or hole faces.
    let mut dirs: Vec<Vec3> = vec![Vec3::Z];
    if axis == AxisSpec::A {
        for t in &topos {
            for s in t.surfaces.iter().flatten() {
                match s {
                    FaceSurface::Plane { normal, .. } if normal.vec().x.abs() <= SQUARE => dirs.push(normal.vec()),
                    FaceSurface::Cylinder { axis, .. } if axis.vec().x.abs() <= SQUARE => {
                        dirs.push(axis.vec());
                        dirs.push(-axis.vec());
                    }
                    _ => {}
                }
            }
        }
    }
    let mut setups: Vec<Setup> = Vec::new();
    let mut unreachable = 0usize;
    for d in dirs {
        let Some(d) = d.normalized() else { continue };
        if setups.iter().any(|s| s.frame.z.dot(d) > 1.0 - PARALLEL) {
            continue;
        }
        let Ok(rotary) = index_for_axis(machine, d) else {
            unreachable += 1;
            continue;
        };
        let Ok(frame) = frame_for_index(machine, &rotary) else { continue };
        if frame.z.dot(d) < 1.0 - 1e-6 {
            continue;
        }
        let a_deg = rotary.first().map_or(0.0, |a| r9(a.to_degrees()));
        if setups.len() >= MAX_SETUPS {
            notes.push(format!("more than {MAX_SETUPS} sides face square to X: only the first {MAX_SETUPS} are looked at"));
            break;
        }
        setups.push(Setup { a_deg, rotary, frame, top: 0.0, bottom: 0.0, swing: None, fixture_top: None });
    }
    if unreachable > 0 {
        notes.push(format!("{unreachable} directions faces look along cannot be reached by the machine's rotary axis"));
    }
    // A = 0 first, then going round.
    setups.sort_by(|a, b| {
        let key = |s: &Setup| (s.a_deg.rem_euclid(360.0) > 1e-9, s.a_deg.rem_euclid(360.0));
        key(a).partial_cmp(&key(b)).unwrap_or(std::cmp::Ordering::Equal)
    });

    // Each setup's extent.
    let mut setup_bounds: Vec<[[f64; 2]; 2]> = Vec::with_capacity(setups.len());
    for su in &mut setups {
        let (mut zr, mut xr, mut yr) = (None, None, None);
        let mut swing: f64 = 0.0;
        for t in &topos {
            for f in 0..t.record.faces.len() {
                let o = su.frame.origin;
                if let Some((lo, hi)) = t.face_range(f, su.frame.z, true) {
                    zr = Some(widen(zr, lo - o.dot(su.frame.z), hi - o.dot(su.frame.z)));
                }
                if let Some((lo, hi)) = t.face_range(f, su.frame.x, true) {
                    xr = Some(widen(xr, lo - o.dot(su.frame.x), hi - o.dot(su.frame.x)));
                }
                if let Some((lo, hi)) = t.face_range(f, su.frame.y, true) {
                    yr = Some(widen(yr, lo - o.dot(su.frame.y), hi - o.dot(su.frame.y)));
                }
                if axis == AxisSpec::A {
                    swing = swing.max(t.face_swing(f));
                }
            }
        }
        let (zlo, zhi) = zr.unwrap_or((0.0, 0.0));
        su.top = r9(zhi);
        su.bottom = r9(zlo);
        if axis == AxisSpec::A {
            su.swing = Some(r9(swing));
        }
        let ((xl, xh), (yl, yh)) = (xr.unwrap_or_default(), yr.unwrap_or_default());
        setup_bounds.push([[xl, yl], [xh, yh]]);
    }

    let found = find(&topos, &setups, axis, tol, &mut notes);
    let mut reply = found.reply;
    reply.axis = axis;
    reply.origin = r3(origin);
    reply.bodies = names.to_vec();
    reply.tolerance = tol;
    reply.notes.splice(0..0, notes);
    reply.setups = setups
        .iter()
        .zip(&setup_bounds)
        .enumerate()
        .map(|(i, (su, b))| SetupReport {
            a_deg: su.a_deg,
            tool_axis: r3(su.frame.z),
            top_z: su.top,
            bottom_z: su.bottom,
            bounds: rb(*b),
            swing_radius: su.swing,
            features: found.geo.iter().filter(|(_, g)| g.setup == i).map(|(id, _)| id.clone()).collect(),
        })
        .collect();
    Ok(FeatureSet { reply, setups, geo: found.geo.into_iter().collect() })
}

/// Exact facts about the faces and edges of one body, for picks (`cam_picks`): what each face is,
/// where it lies, which faces an edge bounds. Coordinates are the design's.
pub(crate) struct BodyFaces<'a> {
    topo: Topo<'a>,
}

impl<'a> BodyFaces<'a> {
    pub fn new(name: &str, body: &'a Body) -> BodyFaces<'a> {
        BodyFaces { topo: Topo::new(name, body, Vec3::ZERO) }
    }

    pub fn face_count(&self) -> usize {
        self.topo.record.faces.len()
    }

    /// Face `f`'s analytic surface.
    pub fn surface(&self, f: usize) -> Option<FaceSurface> {
        self.topo.surface(f).copied()
    }

    /// `planar`, `cylinder`, `cone`, `sphere`, `torus` or `other`.
    pub fn kind(&self, f: usize) -> &'static str {
        match self.topo.surface(f) {
            Some(FaceSurface::Plane { .. }) => "planar",
            Some(FaceSurface::Cylinder { .. }) => "cylinder",
            Some(FaceSurface::Cone { .. }) => "cone",
            Some(FaceSurface::Sphere { .. }) => "sphere",
            Some(FaceSurface::Torus { .. }) => "torus",
            None => "other",
        }
    }

    /// Face `f`'s exact extent along the axes of `frame` (work coordinates `origin` away from the
    /// design's), in the frame's coordinates: [[low x, low y, low z], [high x, high y, high z]].
    pub fn extent(&self, f: usize, origin: Vec3, frame: &Frame) -> Option<[[f64; 3]; 2]> {
        let mut lo = [0.0; 3];
        let mut hi = [0.0; 3];
        for (k, dir) in [frame.x, frame.y, frame.z].into_iter().enumerate() {
            let (a, b) = self.topo.face_range(f, dir, true)?;
            let shift = (origin + frame.origin).dot(dir);
            lo[k] = a - shift;
            hi[k] = b - shift;
        }
        Some([lo, hi])
    }

    /// The faces a design point lies on: within `tol` of the face's surface and inside its exact
    /// extent (grown by `tol`), nearest the middle of its extent first.
    pub fn faces_at(&self, p: Vec3, tol: f64) -> Vec<usize> {
        let mut hits: Vec<(f64, usize)> = Vec::new();
        for f in 0..self.face_count() {
            let Some(surface) = self.topo.surface(f) else { continue };
            let Some(on) = surface.nearest(p) else { continue };
            if on.dist(p) > tol {
                continue;
            }
            let Some([lo, hi]) = self.extent(f, Vec3::ZERO, &Frame::IDENTITY) else { continue };
            let q = [p.x, p.y, p.z];
            if (0..3).all(|k| q[k] >= lo[k] - tol && q[k] <= hi[k] + tol) {
                let mid = Vec3::new((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5);
                hits.push((mid.dist(p), f));
            }
        }
        hits.sort_by(|a, b| a.0.total_cmp(&b.0));
        hits.into_iter().map(|(_, f)| f).collect()
    }

    /// A point on face `f`'s surface near the middle of its extent: where a pick of it is
    /// looked for when its name finds nothing.
    pub fn point_on(&self, f: usize) -> Option<Vec3> {
        let [lo, hi] = self.extent(f, Vec3::ZERO, &Frame::IDENTITY)?;
        let mid = Vec3::new((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5);
        let s = self.topo.surface(f)?;
        // On a round face's axis every direction is as near: step off it along X or Y.
        s.nearest(mid).or_else(|| s.nearest(mid + Vec3::new(1e-3, 0.0, 0.0))).or_else(|| s.nearest(mid + Vec3::new(0.0, 1e-3, 0.0)))
    }

    /// The edges of face `f`'s loops.
    pub fn face_edges(&self, f: usize) -> Vec<usize> {
        self.topo.face_edges(f).collect()
    }

    /// The round wall face `f` is, or (a round hole's floor) the round wall round it.
    pub fn round_wall(&self, f: usize) -> Option<usize> {
        if matches!(self.topo.surface(f), Some(FaceSurface::Cylinder { .. })) {
            return Some(f);
        }
        if !matches!(self.topo.surface(f), Some(FaceSurface::Plane { .. })) {
            return None;
        }
        let edges = self.face_edges(f);
        let [e] = edges.as_slice() else { return None };
        self.edge_circle(*e)?;
        self.edge_faces(*e).into_iter().find(|g| *g != f && matches!(self.topo.surface(*g), Some(FaceSurface::Cylinder { .. })))
    }

    /// The faces edge `e` bounds.
    pub fn edge_faces(&self, e: usize) -> Vec<usize> {
        self.topo.edge_faces.get(e).cloned().unwrap_or_default()
    }

    /// Edge `e`'s circle, when it is one (design coordinates).
    pub fn edge_circle(&self, e: usize) -> Option<EdgeCircle> {
        match self.topo.shapes.get(e)? {
            EdgeShape::Circle(c) => Some(*c),
            EdgeShape::Hull(_) => None,
        }
    }
}

/// A quantised coordinate for ordering features.
fn q(v: f64, tol: f64) -> i64 {
    let k = (v / tol).round();
    if k.is_finite() { k.clamp(-9e15, 9e15) as i64 } else { 0 }
}

fn find(topos: &[Topo<'_>], setups: &[Setup], axis: AxisSpec, tol: f64, notes: &mut Vec<String>) -> Found {
    let mut l = Lists::default();
    let a_of = |s: usize| if axis == AxisSpec::A { setups.get(s).map(|x| x.a_deg) } else { None };
    // Faces claimed by a feature (body, face), so a later setup does not list them again.
    let mut claimed: BTreeSet<(usize, usize)> = BTreeSet::new();
    let mut side_holes = 0usize;
    for (bi, t) in topos.iter().enumerate() {
        let cyls = cylinders(t, tol);
        let mut cyl_done: BTreeSet<usize> = BTreeSet::new();
        for (si, su) in setups.iter().enumerate() {
            let fr = &su.frame;
            let z_of = |p: Vec3| fr.to_local(p).z;
            let lz = |n: Vec3| fr.dir_to_local(n).z;
            let zrange = |f: usize| t.face_range(f, fr.z, false).map(|(lo, hi)| (lo - fr.origin.dot(fr.z), hi - fr.origin.dot(fr.z)));
            let vertical = |g: usize| match t.surface(g) {
                Some(FaceSurface::Plane { normal, .. }) => lz(normal.vec()).abs() <= SQUARE,
                Some(FaceSurface::Cylinder { axis, .. }) => lz(axis.vec()).abs() > 1.0 - PARALLEL,
                _ => false,
            };
            let lie = |lp: &LoopGeo, z: f64| -> Lie {
                let mut up = false;
                let mut down = false;
                for g in &lp.beside {
                    match zrange(*g) {
                        Some((lo, hi)) if hi > z + tol && lo > z - tol => up = true,
                        Some((lo, hi)) if lo < z - tol && hi < z + tol => down = true,
                        _ => return Lie::Mixed,
                    }
                }
                match (up, down) {
                    (true, false) => Lie::Up,
                    (false, true) => Lie::Down,
                    _ => Lie::Mixed,
                }
            };
            // The top of the walls rising from a loop at `z`: up its faces and on through every
            // face joined to them that rises from `z` and is not flat (a round or chamfer at the
            // foot, the upright wall above it, a round at its top), up to the flat faces round
            // them. A wall that starts on a round is as tall as the wall above the round.
            let flat = |g: usize| matches!(t.surface(g), Some(FaceSurface::Plane { normal, .. }) if lz(normal.vec()).abs() > 1.0 - PARALLEL);
            let wall_top = |lp: &LoopGeo, z: f64| -> f64 {
                let mut seen: BTreeSet<usize> = lp.beside.iter().copied().collect();
                let mut todo: Vec<usize> = lp.beside.iter().copied().filter(|g| !flat(*g)).collect();
                let mut top = lp.beside.iter().filter_map(|g| zrange(*g)).map(|r| r.1).fold(f64::NEG_INFINITY, f64::max);
                while let Some(g) = todo.pop() {
                    for e in t.face_edges(g) {
                        for h in t.edge_faces.get(e).into_iter().flatten().copied() {
                            if !seen.insert(h) || flat(h) {
                                continue;
                            }
                            if let Some((lo, hi)) = zrange(h)
                                && lo >= z - tol
                            {
                                top = top.max(hi);
                                todo.push(h);
                            }
                        }
                    }
                }
                top
            };
            let wall_foot = |lp: &LoopGeo| lp.beside.iter().filter_map(|g| zrange(*g)).map(|r| r.0).fold(f64::NEG_INFINITY, f64::max);

            // Holes along this setup's axis.
            let mut setup_holes: Vec<usize> = Vec::new();
            for (ci, c) in cyls.iter().enumerate() {
                if cyl_done.contains(&ci) {
                    continue;
                }
                let along = lz(c.axis);
                if along.abs() < 1.0 - PARALLEL {
                    if si == 0 && axis == AxisSpec::Z && along.abs() <= SQUARE {
                        // Counted once below, if it is a whole hole.
                        let circles = cyl_circles(t, c, c.axis, tol);
                        if full(&circles, tol) && concave(t, c, &circles, c.axis, tol) == Some(true) {
                            side_holes += 1;
                        }
                    }
                    continue;
                }
                let up = if along > 0.0 { c.axis } else { -c.axis };
                let circles = cyl_circles(t, c, up, tol);
                if !full(&circles, tol) {
                    continue;
                }
                let (Some(lo), Some(hi)) = (circles.iter().map(|x| x.1).reduce(f64::min), circles.iter().map(|x| x.1).reduce(f64::max)) else {
                    continue;
                };
                if concave(t, c, &circles, up, tol) != Some(true) {
                    continue;
                }
                let at = |h: f64| circles.iter().filter(move |x| (x.1 - h).abs() <= tol).map(|x| x.0);
                let beside_at = |h: f64| -> Vec<usize> {
                    let mut v: Vec<usize> = Vec::new();
                    for e in at(h) {
                        for g in t.edge_faces.get(e).into_iter().flatten() {
                            if !c.faces.contains(g) && !v.contains(g) {
                                v.push(*g);
                            }
                        }
                    }
                    v
                };
                // Open at the top: a face there faces up (a top face, a counterbore's step) or a
                // countersink widens up from it.
                let mut countersink: Option<(usize, Countersink)> = None;
                let mut open = false;
                for g in beside_at(hi) {
                    match t.surface(g) {
                        Some(FaceSurface::Plane { normal, .. }) if lz(normal.vec()) > 1.0 - PARALLEL => open = true,
                        Some(FaceSurface::Cone { axis: ca, half_angle, .. }) if ca.vec().dot(c.axis).abs() > 1.0 - PARALLEL => {
                            if let Some((_, top)) = zrange(g)
                                && top > z_of(c.origin + up * hi) + tol
                            {
                                let widest = t
                                    .face_edges(g)
                                    .filter_map(|e| match t.shapes.get(e) {
                                        Some(EdgeShape::Circle(k)) => Some(k.radius),
                                        _ => None,
                                    })
                                    .fold(0.0, f64::max);
                                open = true;
                                countersink = Some((g, Countersink { diameter: r9(widest * 2.0), angle_deg: r9(half_angle.to_degrees() * 2.0) }));
                            }
                        }
                        _ => {}
                    }
                }
                if !open {
                    continue;
                }
                // How it ends at the bottom.
                let mut floor = "through";
                let mut tip: Option<f64> = None;
                let mut through = false;
                let mut decided = false;
                for g in beside_at(lo) {
                    match t.surface(g) {
                        Some(FaceSurface::Plane { normal, .. }) if lz(normal.vec()) < -1.0 + PARALLEL => {
                            through = true;
                            decided = true;
                        }
                        Some(FaceSurface::Plane { normal, .. }) if lz(normal.vec()) > 1.0 - PARALLEL => {
                            floor = "flat";
                            decided = true;
                        }
                        Some(FaceSurface::Cone { apex, axis: ca, .. }) if ca.vec().dot(c.axis).abs() > 1.0 - PARALLEL => {
                            let az = z_of(*apex);
                            if az < z_of(c.origin + up * lo) - tol {
                                floor = "cone";
                                tip = Some(r9(az));
                            } else {
                                through = true;
                            }
                            decided = true;
                        }
                        _ => {}
                    }
                }
                // A curved face below the bottom circle, its edges within the wall's radius, with
                // material just outside the wall under its end, closes the hole (a round at its
                // foot, a ball end): it is blind, its round floor under `bottom_z`. A point below
                // on the axis would be in the hole's air.
                let closes = beside_at(lo).iter().any(|g| {
                    !matches!(t.surface(*g), Some(FaceSurface::Plane { .. } | FaceSurface::Cone { .. }))
                        && zrange(*g).is_some_and(|(_, top)| top <= z_of(c.origin + up * lo) + tol)
                        && t.edges_reach(*g, c.origin, c.axis) <= c.radius + tol
                }) && circles.iter().find(|x| (x.1 - lo).abs() <= tol).is_some_and(|(_, _, k)| {
                    let eps = (tol * 2.0).max(c.radius * 1e-3);
                    let radial = ((k.start - c.origin) - c.axis * (k.start - c.origin).dot(c.axis)).normalized();
                    radial.is_some_and(|r| matches!(t.body.classify_point(k.start + r * eps - up * eps + t.origin), Ok(PointClass::Inside)))
                });
                if !decided && closes {
                    floor = "round";
                    decided = true;
                }
                if !decided {
                    // Just under the bottom on the axis: air means it comes out.
                    let below = c.origin + up * (lo - (tol * 2.0).max(1e-3)) + t.origin;
                    through = matches!(t.body.classify_point(below), Ok(PointClass::Outside));
                    floor = if through { "through" } else { "flat" };
                }
                if through {
                    floor = "through";
                }
                cyl_done.insert(ci);
                for f in &c.faces {
                    claimed.insert((bi, *f));
                }
                if let Some((g, _)) = &countersink {
                    claimed.insert((bi, *g));
                }
                let top_pt = c.origin + up * hi;
                let lt = fr.to_local(top_pt);
                let (top_z, bottom_z) = (r9(lt.z), r9(z_of(c.origin + up * lo)));
                let center = Vec2::new(lt.x, lt.y);
                let d = c.radius * 2.0;
                let hole = HoleReport {
                    id: String::new(),
                    a_deg: a_of(si),
                    body: t.name.clone(),
                    center: r2(center),
                    center_work: r3(top_pt),
                    axis_work: r3(fr.z),
                    diameter: r9(d),
                    top_z,
                    bottom_z,
                    depth: r9(top_z - bottom_z),
                    through,
                    floor: floor.into(),
                    tip_z: tip,
                    counterbore: None,
                    counterbore_of: None,
                    countersink: countersink.map(|(_, c)| c),
                    faces: c.faces.clone(),
                };
                let r = c.radius;
                let geo = Geo {
                    kind: Kind::Hole,
                    setup: si,
                    outer: Some(Loop2::circle(center, r)),
                    islands: Vec::new(),
                    top: top_z,
                    bottom: bottom_z,
                    bounds: [[center.x - r, center.y - r], [center.x + r, center.y + r]],
                    hole: Some((center, d, through)),
                    flat_top: false,
                };
                let key = vec![si as i64, q(-top_z, tol), q(center.y, tol), q(center.x, tol), q(d, tol), bi as i64];
                setup_holes.push(l.holes.len());
                l.holes.push((key, hole, geo));
            }
            // Counterbores: a larger hole on the same axis whose bottom is a smaller one's top.
            for big in &setup_holes {
                for small in &setup_holes {
                    let (Some(b), Some(s)) = (l.holes.get(*big), l.holes.get(*small)) else { continue };
                    let same_axis = b.1.center.iter().zip(&s.1.center).all(|(x, y)| (x - y).abs() <= tol);
                    if same_axis && b.1.diameter > s.1.diameter + tol && (b.1.bottom_z - s.1.top_z).abs() <= tol {
                        l.counterbores.push((*big, *small));
                    }
                }
            }

            // Planar faces facing the tool.
            let mut island_walls: BTreeSet<usize> = BTreeSet::new();
            let mut setup_faces: Vec<(usize, Vec<usize>)> = Vec::new();
            let mut candidates: Vec<(Vec<i64>, ProfileReport, Geo, Seen, Vec<usize>)> = Vec::new();
            for f in 0..t.record.faces.len() {
                let Some(FaceSurface::Plane { normal, point }) = t.surface(f) else { continue };
                if lz(normal.vec()) < 1.0 - PARALLEL {
                    continue;
                }
                let z = z_of(*point);
                let loops = t.face_loops(f, fr, z, tol);
                let Some(oi) = loops.iter().enumerate().max_by(|a, b| a.1.area.abs().total_cmp(&b.1.area.abs())).map(|(i, _)| i) else { continue };
                let Some(outer) = loops.get(oi) else { continue };
                if !(outer.area.abs() > tol * tol) {
                    continue;
                }
                claimed.insert((bi, f));
                let outer_lie = lie(outer, z);
                let hole_floor = outer_lie == Lie::Up
                    && outer
                        .beside
                        .iter()
                        .all(|g| matches!(t.surface(*g), Some(FaceSurface::Cylinder { axis, .. }) if lz(axis.vec()).abs() > 1.0 - PARALLEL))
                    && {
                        let r = outer.beside.iter().find_map(|g| match t.surface(*g) {
                            Some(FaceSurface::Cylinder { radius, .. }) => Some(*radius),
                            _ => None,
                        });
                        r.is_some_and(|r| (outer.area.abs() - PI * r * r).abs() <= (PI * r * r) * 1e-3 + tol)
                    };
                let role = match outer_lie {
                    Lie::Up if hole_floor => "hole_floor",
                    Lie::Up => "pocket_floor",
                    Lie::Down if (z - su.top).abs() <= tol => "top",
                    Lie::Down => "plateau",
                    Lie::Mixed => "step",
                };
                // Islands: inner loops whose walls rise from this face.
                let islands: Vec<(Loop2, f64, &LoopGeo)> = loops
                    .iter()
                    .enumerate()
                    .filter(|(i, lp)| *i != oi && lie(lp, z) == Lie::Up)
                    .map(|(_, lp)| (Loop2 { segs: oriented(&lp.segs, lp.area, false) }, wall_top(lp, z), lp))
                    .collect();
                let (pts, area, b) = outline(&outer.segs, tol);
                let net_area = r9(area - islands.iter().map(|(_, _, lp)| lp.area.abs()).sum::<f64>());
                let outer_ccw = oriented(&outer.segs, outer.area, true);
                let top = match outer_lie {
                    Lie::Up => wall_top(outer, z),
                    Lie::Mixed => wall_top(outer, z).max(z),
                    Lie::Down => z,
                };
                let geo = Geo {
                    kind: Kind::Face,
                    setup: si,
                    outer: Some(Loop2 { segs: outer_ccw.clone() }),
                    islands: islands.iter().map(|(lp, top, _)| (lp.clone(), *top)).collect(),
                    top: r9(top),
                    bottom: r9(z),
                    bounds: b,
                    hole: None,
                    flat_top: outer_lie == Lie::Down,
                };
                let face = FaceReport {
                    id: String::new(),
                    a_deg: a_of(si),
                    body: t.name.clone(),
                    face: f,
                    z: r9(z),
                    area: net_area,
                    bounds: b,
                    role: role.into(),
                    pocket: None,
                    normal_work: r3(normal.vec()),
                };
                let cx = (b[0][0] + b[1][0]) * 0.5;
                let cy = (b[0][1] + b[1][1]) * 0.5;
                let key = vec![si as i64, q(-z, tol), q(cy, tol), q(cx, tol), bi as i64, f as i64];
                let face_index = l.faces.len();
                l.faces.push((key.clone(), face, geo.clone(), Seen { body: bi, face: f, setup: si }));
                if role == "pocket_floor" {
                    // The walls' inside corners: free space (the pocket) on the left.
                    let (mut radius, mut sharp) = corner_limits(&outer_ccw);
                    for (lp, ..) in &islands {
                        let (r, s) = corner_limits(&lp.segs);
                        radius = match (radius, r) {
                            (Some(a), Some(b)) => Some(a.min(b)),
                            (a, b) => a.or(b),
                        };
                        sharp += s;
                    }
                    let half = ((b[1][0] - b[0][0]).min(b[1][1] - b[0][1]) * 0.5).max(0.0);
                    let min_corner = if sharp > 0 { 0.0 } else { radius.unwrap_or(half) };
                    let walls_vertical = outer.beside.iter().all(|g| vertical(*g));
                    let pocket = PocketReport {
                        id: String::new(),
                        a_deg: a_of(si),
                        body: t.name.clone(),
                        floor_face: f,
                        face_id: String::new(),
                        floor_z: r9(z),
                        top_z: r9(top),
                        depth: r9(top - z),
                        outline: pts.clone(),
                        area: net_area,
                        bounds: b,
                        min_corner_radius: r9(min_corner),
                        sharp_corners: sharp,
                        walls_vertical,
                        islands: islands
                            .iter()
                            .map(|(lp, top, _)| {
                                let (pts, area, _) = outline(&lp.segs, tol);
                                IslandReport { outline: pts, top_z: r9(*top), area }
                            })
                            .collect(),
                    };
                    let mut g = geo.clone();
                    g.kind = Kind::Pocket;
                    g.flat_top = false;
                    l.pockets.push((key.clone(), pocket, g, face_index));
                }
                if role == "pocket_floor" {
                    for (_, _, lp) in &islands {
                        island_walls.extend(lp.beside.iter().copied());
                    }
                }
                setup_faces.push((face_index, outer.beside.clone()));
                // An outline whose walls fall straight away all round is a profile (unless it is
                // an island's top, found once every pocket of the setup is).
                if outer_lie == Lie::Down && outer.beside.iter().all(|g| vertical(*g)) {
                    let foot = wall_foot(outer);
                    if foot.is_finite() && foot < z - tol {
                        let (rad, sharp) = corner_limits(&oriented(&outer.segs, outer.area, false));
                        let notch = if sharp > 0 { Some(0.0) } else { rad.map(r9) };
                        let profile = ProfileReport {
                            id: String::new(),
                            a_deg: a_of(si),
                            body: t.name.clone(),
                            top_z: r9(z),
                            bottom_z: r9(foot),
                            depth: r9(z - foot),
                            outline: pts.clone(),
                            area,
                            bounds: b,
                            min_concave_radius: notch,
                            face_id: String::new(),
                        };
                        let pg = Geo { kind: Kind::Profile, islands: Vec::new(), top: r9(z), bottom: r9(foot), flat_top: false, ..geo.clone() };
                        candidates.push((
                            vec![si as i64, q(-z, tol), q(-area, tol), bi as i64, f as i64],
                            profile,
                            pg,
                            Seen { body: bi, face: f, setup: si },
                            outer.beside.clone(),
                        ));
                    }
                }
                // Faces falling away from its edges that are not walls: rounds and chamfers.
                for lp in &loops {
                    for g in &lp.beside {
                        let g = *g;
                        if claimed.contains(&(bi, g)) || vertical(g) {
                            continue;
                        }
                        let Some((lo, hi)) = zrange(g) else { continue };
                        if !(lo < z - tol && hi < z + tol) {
                            continue;
                        }
                        let kind = match t.surface(g) {
                            Some(FaceSurface::Cylinder { axis, radius, .. }) if lz(axis.vec()).abs() <= SQUARE => Some((true, *radius)),
                            Some(FaceSurface::Torus { minor, .. }) => Some((true, *minor)),
                            Some(FaceSurface::Sphere { radius, .. }) => Some((true, *radius)),
                            Some(FaceSurface::Plane { normal, .. }) if lz(normal.vec()) > SQUARE => {
                                Some((false, lz(normal.vec()).acos().to_degrees()))
                            }
                            Some(FaceSurface::Cone { half_angle, axis, .. }) if lz(axis.vec()).abs() > 1.0 - PARALLEL => {
                                Some((false, half_angle.to_degrees()))
                            }
                            _ => None,
                        };
                        let Some((round, value)) = kind else { continue };
                        claimed.insert((bi, g));
                        let (xl, xh) = t.face_range(g, fr.x, true).unwrap_or_default();
                        let (yl, yh) = t.face_range(g, fr.y, true).unwrap_or_default();
                        let (ox, oy) = (fr.origin.dot(fr.x), fr.origin.dot(fr.y));
                        let b = [[xl - ox, yl - oy], [xh - ox, yh - oy]];
                        let report = EdgeBreakReport {
                            id: String::new(),
                            a_deg: a_of(si),
                            body: t.name.clone(),
                            faces: vec![g],
                            radius: round.then_some(r9(value)),
                            angle_deg: (!round).then_some(r9(value)),
                            top_z: r9(hi),
                            bottom_z: r9(lo),
                            bounds: rb(b),
                        };
                        let geo = Geo {
                            kind: Kind::Area,
                            setup: si,
                            outer: None,
                            islands: Vec::new(),
                            top: r9(hi),
                            bottom: r9(lo),
                            bounds: rb(b),
                            hole: None,
                            flat_top: false,
                        };
                        let key =
                            vec![si as i64, q(-hi, tol), q((b[0][1] + b[1][1]) * 0.5, tol), q((b[0][0] + b[1][0]) * 0.5, tol), bi as i64, g as i64];
                        if round {
                            merge_break(&mut l.fillets, t, key, report, geo, tol);
                        } else {
                            merge_break(&mut l.chamfers, t, key, report, geo, tol);
                        }
                    }
                }
            }
            // An island's top: every wall round it rises from a pocket's floor.
            let island = |beside: &[usize]| !beside.is_empty() && beside.iter().all(|g| island_walls.contains(g));
            for (fi, beside) in &setup_faces {
                if island(beside)
                    && let Some(entry) = l.faces.get_mut(*fi)
                {
                    entry.1.role = "island_top".into();
                }
            }
            l.profiles.extend(candidates.into_iter().filter(|c| !island(&c.4)).map(|(k, p, g, seen, _)| (k, p, g, seen)));
            // The outline of a bottom face whose walls rise straight all round is a profile too
            // (the silhouette), unless a face above gave the same.
            for f in 0..t.record.faces.len() {
                let Some(FaceSurface::Plane { normal, point }) = t.surface(f) else { continue };
                if lz(normal.vec()) > -1.0 + PARALLEL {
                    continue;
                }
                let z = z_of(*point);
                if (z - su.bottom).abs() > tol {
                    continue;
                }
                let loops = t.face_loops(f, fr, z, tol);
                let Some(outer) = loops.iter().max_by(|a, b| a.area.abs().total_cmp(&b.area.abs())) else { continue };
                if lie(outer, z) != Lie::Up || !outer.beside.iter().all(|g| vertical(*g)) {
                    continue;
                }
                let (pts, area, b) = outline(&outer.segs, tol);
                let dup = l.profiles.iter().any(|(_, p, ..)| {
                    p.a_deg == a_of(si)
                        && (p.bottom_z - r9(z)).abs() <= tol
                        && (p.area - area).abs() <= area * 1e-6 + tol * tol
                        && p.bounds.iter().flatten().zip(b.iter().flatten()).all(|(x, y)| (x - y).abs() <= tol)
                });
                if dup {
                    continue;
                }
                let (rad, sharp) = corner_limits(&oriented(&outer.segs, outer.area, false));
                let profile = ProfileReport {
                    id: String::new(),
                    a_deg: a_of(si),
                    body: t.name.clone(),
                    top_z: su.top,
                    bottom_z: r9(z),
                    depth: r9(su.top - z),
                    outline: pts,
                    area,
                    bounds: b,
                    min_concave_radius: if sharp > 0 { Some(0.0) } else { rad.map(r9) },
                    face_id: String::new(),
                };
                let pg = Geo {
                    kind: Kind::Profile,
                    setup: si,
                    outer: Some(Loop2 { segs: oriented(&outer.segs, outer.area, true) }),
                    islands: Vec::new(),
                    top: su.top,
                    bottom: r9(z),
                    bounds: b,
                    hole: None,
                    flat_top: false,
                };
                l.profiles.push((
                    vec![si as i64, q(-su.top, tol), q(-area, tol), bi as i64, f as i64],
                    profile,
                    pg,
                    Seen { body: bi, face: f, setup: si },
                ));
            }
            for f in 0..t.record.faces.len() {
                if let Some(FaceSurface::Plane { normal, .. }) = t.surface(f)
                    && lz(normal.vec()) < -1.0 + PARALLEL
                {
                    claimed.insert((bi, f));
                }
            }
        }
        // Diameters turned about the A axis.
        if axis == AxisSpec::A {
            for (ci, c) in cyls.iter().enumerate() {
                if cyl_done.contains(&ci) || c.axis.x.abs() < 1.0 - PARALLEL {
                    continue;
                }
                let off = c.origin - Vec3::X * c.origin.x;
                if off.len() > tol {
                    continue;
                }
                let circles = cyl_circles(t, c, Vec3::X, tol);
                if concave(t, c, &circles, Vec3::X, tol) != Some(false) {
                    continue;
                }
                let xs: Vec<f64> = circles.iter().map(|(_, _, k)| k.center.x).collect();
                let (Some(x0), Some(x1)) = (xs.iter().copied().reduce(f64::min), xs.iter().copied().reduce(f64::max)) else { continue };
                for f in &c.faces {
                    claimed.insert((bi, *f));
                }
                let report = TurnedReport {
                    id: String::new(),
                    body: t.name.clone(),
                    diameter: r9(c.radius * 2.0),
                    x_min: r9(x0),
                    x_max: r9(x1),
                    faces: c.faces.clone(),
                };
                l.turned.push((vec![q(x0, tol), q(c.radius, tol), bi as i64], report));
            }
        }
        // What is left that is not a wall: curved areas, reached from the first setup.
        if let Some(su) = setups.first() {
            let fr = &su.frame;
            let lz = |n: Vec3| fr.dir_to_local(n).z;
            let wall = |g: usize| match t.surface(g) {
                Some(FaceSurface::Plane { normal, .. }) => lz(normal.vec()) <= SQUARE,
                Some(FaceSurface::Cylinder { axis, .. }) => lz(axis.vec()).abs() > 1.0 - PARALLEL,
                _ => false,
            };
            let left: Vec<usize> = (0..t.record.faces.len()).filter(|f| !claimed.contains(&(bi, *f)) && !wall(*f)).collect();
            let mut groups: Vec<Vec<usize>> = Vec::new();
            for f in left {
                let touching: Vec<usize> =
                    groups.iter().enumerate().filter(|(_, g)| g.iter().any(|h| shares_edge(t, *h, f))).map(|(i, _)| i).collect();
                let mut merged = vec![f];
                for i in touching.iter().rev() {
                    if *i < groups.len() {
                        merged.extend(groups.remove(*i));
                    }
                }
                merged.sort_unstable();
                groups.push(merged);
            }
            groups.sort();
            for faces in groups {
                let o = fr.origin;
                let range = |d: Vec3| {
                    faces.iter().filter_map(|f| t.face_range(*f, d, true)).fold(None, |acc, (a, b)| Some(widen(acc, a - o.dot(d), b - o.dot(d))))
                };
                let (Some((zl, zh)), Some((xl, xh)), Some((yl, yh))) = (range(fr.z), range(fr.x), range(fr.y)) else { continue };
                if zh <= su.bottom + tol {
                    continue;
                }
                let mut kinds: BTreeSet<&str> = BTreeSet::new();
                let mut min_radius: Option<f64> = None;
                for f in &faces {
                    let (k, r) = match t.surface(*f) {
                        Some(FaceSurface::Sphere { radius, .. }) => ("sphere", Some(*radius)),
                        Some(FaceSurface::Torus { minor, .. }) => ("torus", Some(*minor)),
                        Some(FaceSurface::Cylinder { radius, .. }) => ("cylinder", Some(*radius)),
                        Some(FaceSurface::Cone { .. }) => ("cone", None),
                        Some(FaceSurface::Plane { .. }) => ("plane", None),
                        None => ("freeform", None),
                    };
                    kinds.insert(k);
                    if let Some(r) = r {
                        min_radius = Some(min_radius.map_or(r, |m: f64| m.min(r)));
                    }
                }
                let b = rb([[xl, yl], [xh, yh]]);
                let report = SurfaceReport {
                    id: String::new(),
                    a_deg: a_of(0),
                    body: t.name.clone(),
                    faces: faces.clone(),
                    kinds: kinds.into_iter().map(str::to_string).collect(),
                    min_radius: min_radius.map(r9),
                    top_z: r9(zh),
                    bottom_z: r9(zl.max(su.bottom)),
                    bounds: b,
                };
                let geo = Geo {
                    kind: Kind::Area,
                    setup: 0,
                    outer: None,
                    islands: Vec::new(),
                    top: r9(zh),
                    bottom: r9(zl.max(su.bottom)),
                    bounds: b,
                    hole: None,
                    flat_top: false,
                };
                let key =
                    vec![0, q(-zh, tol), q((yl + yh) * 0.5, tol), q((xl + xh) * 0.5, tol), bi as i64, faces.first().copied().unwrap_or(0) as i64];
                l.surfaces.push((key, report, geo));
            }
        }
    }
    if side_holes > 0 {
        notes.push(format!("{side_holes} holes run square to Z, not reachable on a 3-axis mill: find them with `axis: \"a\"` or another setup"));
    }
    number(l, tol)
}

/// Whether the circles at one height of a cylinder go all the way round.
fn full(circles: &[(usize, f64, &EdgeCircle)], tol: f64) -> bool {
    let mut heights: Vec<f64> = circles.iter().map(|x| x.1).collect();
    heights.sort_by(f64::total_cmp);
    heights.dedup_by(|a, b| (*a - *b).abs() <= tol);
    heights.iter().any(|h| circles.iter().filter(|x| (x.1 - h).abs() <= tol).map(|x| x.2.angle).sum::<f64>() >= TAU - 1e-6)
}

fn shares_edge(t: &Topo<'_>, a: usize, b: usize) -> bool {
    t.face_edges(a).any(|e| t.edge_faces.get(e).is_some_and(|fs| fs.contains(&b)))
}

/// A round or chamfer face added to the feature it touches with the same size, else a new one.
fn merge_break(list: &mut Vec<(Vec<i64>, EdgeBreakReport, Geo)>, t: &Topo<'_>, key: Vec<i64>, report: EdgeBreakReport, geo: Geo, tol: f64) {
    let size = report.radius.or(report.angle_deg).unwrap_or(0.0);
    let Some(face) = report.faces.first().copied() else { return };
    let near = list.iter_mut().find(|(_, r, g)| {
        r.body == report.body
            && g.setup == geo.setup
            && (r.radius.or(r.angle_deg).unwrap_or(0.0) - size).abs() <= tol
            && r.faces.iter().any(|h| shares_edge(t, *h, face))
    });
    match near {
        Some((k, r, g)) => {
            r.faces.push(face);
            r.faces.sort_unstable();
            r.top_z = r.top_z.max(report.top_z);
            r.bottom_z = r.bottom_z.min(report.bottom_z);
            let b = [
                [r.bounds[0][0].min(report.bounds[0][0]), r.bounds[0][1].min(report.bounds[0][1])],
                [r.bounds[1][0].max(report.bounds[1][0]), r.bounds[1][1].max(report.bounds[1][1])],
            ];
            r.bounds = b;
            g.bounds = b;
            g.top = r.top_z;
            g.bottom = r.bottom_z;
            if key < *k {
                *k = key;
            }
        }
        None => list.push((key, report, geo)),
    }
}

/// Sorts every list and numbers the features.
fn number(mut l: Lists, tol: f64) -> Found {
    let mut reply = CamFeaturesReply::default();
    let mut geo: Vec<(String, Geo)> = Vec::new();

    // Holes, with their counterbores.
    let mut order: Vec<usize> = (0..l.holes.len()).collect();
    order.sort_by(|a, b| l.holes.get(*a).map(|x| &x.0).cmp(&l.holes.get(*b).map(|x| &x.0)));
    let mut hole_id: BTreeMap<usize, String> = BTreeMap::new();
    for (k, i) in order.iter().enumerate() {
        hole_id.insert(*i, format!("hole:{k}"));
    }
    for (big, small) in &l.counterbores {
        let (Some(b), Some(s)) = (hole_id.get(big).cloned(), hole_id.get(small).cloned()) else { continue };
        if let Some(h) = l.holes.get_mut(*big) {
            h.1.counterbore_of = Some(s);
        }
        if let Some(h) = l.holes.get_mut(*small) {
            h.1.counterbore = Some(b);
        }
    }
    let mut holes: Vec<Option<(Vec<i64>, HoleReport, Geo)>> = l.holes.into_iter().map(Some).collect();
    for (k, i) in order.iter().enumerate() {
        let Some(Some((_, mut h, g))) = holes.get_mut(*i).map(Option::take) else { continue };
        h.id = format!("hole:{k}");
        geo.push((h.id.clone(), g));
        reply.holes.push(h);
    }
    // Groups of equal holes.
    for h in &reply.holes {
        let same = reply.hole_groups.iter_mut().find(|g| {
            (g.diameter - h.diameter).abs() <= tol.max(1e-9)
                && (g.depth - h.depth).abs() <= tol.max(1e-9)
                && g.through == h.through
                && g.a_deg == h.a_deg
        });
        match same {
            Some(g) => {
                g.count += 1;
                g.ids.push(h.id.clone());
            }
            None => reply.hole_groups.push(HoleGroup {
                diameter: h.diameter,
                depth: h.depth,
                through: h.through,
                a_deg: h.a_deg,
                count: 1,
                ids: vec![h.id.clone()],
            }),
        }
    }

    // Faces first, so pockets and profiles can name them.
    let mut face_order: Vec<usize> = (0..l.faces.len()).collect();
    face_order.sort_by(|a, b| l.faces.get(*a).map(|x| &x.0).cmp(&l.faces.get(*b).map(|x| &x.0)));
    let mut face_id: BTreeMap<usize, String> = BTreeMap::new();
    for (k, i) in face_order.iter().enumerate() {
        face_id.insert(*i, format!("face:{k}"));
    }
    l.pockets.sort_by(|a, b| a.0.cmp(&b.0));
    let mut pocket_of_face: BTreeMap<usize, String> = BTreeMap::new();
    for (k, (_, p, g, fi)) in l.pockets.iter_mut().enumerate() {
        p.id = format!("pocket:{k}");
        p.face_id = face_id.get(fi).cloned().unwrap_or_default();
        pocket_of_face.insert(*fi, p.id.clone());
        geo.push((p.id.clone(), g.clone()));
    }
    reply.pockets = l.pockets.into_iter().map(|(_, p, ..)| p).collect();
    let mut faces: Vec<Option<(Vec<i64>, FaceReport, Geo, Seen)>> = l.faces.into_iter().map(Some).collect();
    let mut seen_face: BTreeMap<(usize, usize, usize), String> = BTreeMap::new();
    for (k, i) in face_order.iter().enumerate() {
        let Some(Some((_, mut f, g, seen))) = faces.get_mut(*i).map(Option::take) else { continue };
        f.id = format!("face:{k}");
        f.pocket = pocket_of_face.get(i).cloned();
        seen_face.insert((seen.body, seen.face, seen.setup), f.id.clone());
        geo.push((f.id.clone(), g));
        reply.faces.push(f);
    }
    l.profiles.sort_by(|a, b| a.0.cmp(&b.0));
    for (k, (_, mut p, g, seen)) in l.profiles.into_iter().enumerate() {
        p.id = format!("profile:{k}");
        p.face_id = seen_face.get(&(seen.body, seen.face, seen.setup)).cloned().unwrap_or_default();
        geo.push((p.id.clone(), g));
        reply.profiles.push(p);
    }
    for (list, kind, out) in [(l.fillets, "fillet", &mut reply.fillets), (l.chamfers, "chamfer", &mut reply.chamfers)] {
        let mut list = list;
        list.sort_by(|a, b| a.0.cmp(&b.0));
        for (k, (_, mut r, g)) in list.into_iter().enumerate() {
            r.id = format!("{kind}:{k}");
            geo.push((r.id.clone(), g));
            out.push(r);
        }
    }
    l.surfaces.sort_by(|a, b| a.0.cmp(&b.0));
    for (k, (_, mut r, g)) in l.surfaces.into_iter().enumerate() {
        r.id = format!("surface:{k}");
        geo.push((r.id.clone(), g));
        reply.surfaces.push(r);
    }
    l.turned.sort_by(|a, b| a.0.cmp(&b.0));
    for (k, (_, mut r)) in l.turned.into_iter().enumerate() {
        r.id = format!("turned:{k}");
        let g = Geo {
            kind: Kind::Area,
            setup: 0,
            outer: None,
            islands: Vec::new(),
            top: r.diameter * 0.5,
            bottom: 0.0,
            bounds: [[r.x_min, -r.diameter * 0.5], [r.x_max, r.diameter * 0.5]],
            hole: None,
            flat_top: false,
        };
        geo.push((r.id.clone(), g));
        reply.turned.push(r);
    }
    Found { reply, geo }
}
