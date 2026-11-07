//! Picks (a child of `cmd/cam.rs`): an operation's geometry stored as persistent face, edge and
//! sketch names (`schema/cam_picks.rs`), found again each time the job is planned and read as
//! the keys the planners take (a pocket's `feature`, holes' points and heights, a 3D finish's
//! boundary...); heights that follow a face; work zero kept with how it was set (`origin_from`)
//! and `cam.setup.zero`; and `cam.picks.resolve`, which says what picks are and which operations
//! suit them.
//!
//! A pick is found by its name first (`naming::locate_face_named`, `locate_edge_named`), else by
//! the exact face or edge its stored point lies on (the B-rep's surfaces and exact extents, never
//! the display mesh), and is `lost` when neither finds it: the operation is then refused. One
//! found by its point, or by a name worked out from geometry, is a warning to check. Nothing here
//! changes the job as stored: the keys picks give go into the copy that is planned.

use std::collections::BTreeSet;

use serde_json::{Map, Value, json};
use solidcore_record::Vec3;
use solidcraft_cam::indexed::frame_for_index;
use solidcraft_cam::kinematics::Machine;
use solidcraft_cam::{Frame, Tool, ToolKind};
use solidcraft_doc::ModelState;
use solidcraft_geom::prelude::*;
use solidcraft_kernel::FaceSurface;

use super::features::{BodyFaces, FeatureSet, Lazy};
use super::{CommandSpec, num, parse};
use crate::params::bad;
use crate::schema::cam::CamJob;
use crate::schema::cam_agent::{OpRef, Refusal};
use crate::schema::cam_picks::{
    CamPicksResolveParams, CamPicksResolveReply, CamSetupZeroParams, CamSetupZeroReply, GridPlace, HeightRef, HeightSource, OriginFrom, Pick,
    PickRead, PickResolved, StockGrid, Suit, XEnd,
};
use crate::schema::{TypedContract, output_schema_for};
use crate::{EngineError, Result, Session};

const RESOLVE: TypedContract<CamPicksResolveParams> = TypedContract::new();
const ZERO: TypedContract<CamSetupZeroParams> = TypedContract::new();

/// Most picks one operation (or one call) holds.
pub(crate) const MAX_PICKS: usize = 1000;
/// The keys of an operation that picks take the place of.
pub(crate) const GEOMETRY_KEYS: &[&str] = &["sketch", "profiles", "polygon", "feature", "points", "holes", "centers", "curve"];
/// The height keys of a `heights` object.
const HEIGHT_KEYS: &[&str] = &["clearance", "retract", "top", "bottom"];
/// Operation types whose `heights` are all required.
const FULL_HEIGHTS: &[&str] = &["adaptive", "adaptive_roughing", "drill", "bore", "thread_mill", "scallop", "steep_shallow", "pencil", "rest_finish"];
/// The 3D operations a picked area bounds.
const AREA_TYPES: &[&str] = &["parallel", "roughing", "waterline", "scallop", "steep_shallow", "pencil", "rest_finish", "adaptive_roughing"];
/// The width of a chamfer that breaks a sharp edge, mm.
const DEBURR: f64 = 0.5;
/// Above the part (or stock) the clearance height goes, and above the top the retract height.
const CLEARANCE_ABOVE: f64 = 10.0;
const RETRACT_ABOVE: f64 = 2.0;

pub(crate) static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.picks.resolve", "Read Machining Picks", crate::checked!("cam.picks.resolve", resolve_cmd))
        .noundo()
        .doc_only()
        .contract(&RESOLVE)
        .output(output_schema_for::<CamPicksResolveReply>)
        .params(
            "picks?: [{body, face | edge, at: [x,y,z]} | {sketch, profile? | curve? | point?}] | from_selection?: the design's selected \
             faces and edges; for?: an operation type → picks: [{pick, by: name | point | lost, face?, edge?, kind, feature?, reads, \
             note?}], suits: [{type, label, tool?, why}] (the operations that suit them, best first, with the open job's tool each \
             would take)",
        ),
    CommandSpec::new("cam.setup.zero", "Set Work Zero", crate::checked!("cam.setup.zero", zero_cmd))
        .noundo()
        .doc_only()
        .contract(&ZERO)
        .output(output_schema_for::<CamSetupZeroReply>)
        .params(
            "one of at: [x,y,z] | pick: {body, face | edge, at} (a vertex: an edge's end nearest `at`; a circle: its centre; a face: the \
             point) | stock: {x: low|middle|high, y, z: top|middle|bottom} | axis: a cylinder face (4th axis: work zero on its axis) + \
             x_end?: stock_low | stock_high | pick | refresh: true (again from origin_from); keep_ref? (default true: keep origin_from) \
             → origin, origin_from?, moved, notes, summary. One job undo step; a box or bar stock stays where it was",
        ),
];

/// A face or edge found again.
#[derive(Clone, Debug)]
pub(crate) struct Hit {
    pub body: usize,
    pub body_name: String,
    pub face: Option<usize>,
    pub edge: Option<usize>,
    /// Where on it, design coordinates.
    pub point: Vec3,
    /// `name` or `point`.
    pub by: &'static str,
    pub note: Option<String>,
}

/// How far from an exact surface a stored point may be and still be on it: the display mesh a
/// click lands on is within a small share of the body's size of the exact surface.
fn point_tol(st: &ModelState, body: usize, tol: f64) -> f64 {
    let size = st.bodies.get(body).map_or(1.0, |b| b.body.size());
    (tol * 10.0).max(size * 2e-3).max(1e-4)
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

fn finite3(p: &[f64; 3]) -> bool {
    p.iter().all(|x| x.is_finite() && x.abs() <= super::MAX_COORD)
}

/// Checks a pick's shape: a face or an edge of a body (with a finite point), or a sketch's piece.
pub(crate) fn check_pick(p: &Pick) -> std::result::Result<(), String> {
    let text_ok =
        |t: &Option<String>| t.as_deref().is_none_or(|t| !t.trim().is_empty() && t.chars().count() <= 400 && !t.chars().any(char::is_control));
    if ![&p.body, &p.face, &p.edge, &p.sketch, &p.curve, &p.point].iter().all(|t| text_ok(t)) {
        return Err("a pick's names must be 1 to 400 characters, without control characters".into());
    }
    if p.at.as_ref().is_some_and(|a| !finite3(a)) {
        return Err("a pick's `at` must be a finite point [x, y, z] within 1000 m".into());
    }
    let solid = p.face.is_some() || p.edge.is_some();
    match (solid, p.sketch.is_some()) {
        (true, true) => Err("a pick is a face or edge of a body, or a sketch's piece, not both".into()),
        (true, false) if p.face.is_some() && p.edge.is_some() => Err("a pick names a `face` or an `edge`, not both".into()),
        (false, false) if p.at.is_none() => Err("a pick needs a `face`, an `edge` (with `at`), or a `sketch`".into()),
        (false, false) => Ok(()),
        (true, false) => Ok(()),
        (false, true) if [p.profile.is_some(), p.curve.is_some(), p.point.is_some()].iter().filter(|x| **x).count() > 1 => {
            Err("a sketch pick names one `profile`, `curve` or `point`".into())
        }
        (false, true) => Ok(()),
    }
}

/// The part's bodies: the job's `bodies`, else every visible body not a fixture.
fn part_bodies(st: &ModelState, bodies: Option<&[String]>, fixtures: &[String]) -> Vec<usize> {
    st.bodies
        .iter()
        .enumerate()
        .filter(|(_, b)| !b.body.is_mesh())
        .filter(|(_, b)| match bodies {
            Some(list) => list.contains(&b.name),
            None => !fixtures.contains(&b.name),
        })
        .map(|(i, _)| i)
        .collect()
}

/// Finds face or edge pick `p` again: by name, else by its point. `Err`: lost, and why.
pub(crate) fn find(st: &ModelState, part: &[usize], p: &Pick, tol: f64) -> std::result::Result<Hit, String> {
    let named_body = p.body.as_deref().and_then(|n| st.bodies.iter().position(|b| b.name == n));
    if p.body.is_some() && named_body.is_none() {
        // The body itself is gone or renamed: only its point can find it on another body.
        if p.at.is_none() {
            return Err(format!("no body `{}`", p.body.as_deref().unwrap_or_default()));
        }
    }
    let at = p.at.map(v3);
    let hit = |body: usize, face: Option<usize>, edge: Option<usize>, point: Vec3, by: &'static str, note: Option<String>| Hit {
        body,
        body_name: st.bodies.get(body).map(|b| b.name.clone()).unwrap_or_default(),
        face,
        edge,
        point,
        by,
        note,
    };
    if let Some(name) = &p.face {
        let near = at.unwrap_or(Vec3::ZERO);
        if let Some((b, f, verified, on_point)) = solidcraft_doc::naming::locate_face_named(st, p.body.as_deref(), name, near) {
            let note = (!verified && !on_point).then(|| "found by a name worked out from the shape, not exact history; check it".to_string());
            let point =
                at.filter(|_| on_point).or_else(|| st.bodies.get(b).and_then(|mb| BodyFaces::new(&mb.name, &mb.body).point_on(f))).unwrap_or(near);
            return Ok(hit(b, Some(f), None, point, "name", note));
        }
        let Some(at) = at else { return Err(format!("no face is named `{name}` any more")) };
        let bodies: Vec<usize> = named_body.into_iter().chain(part.iter().copied()).collect();
        for b in bodies {
            let Some(mb) = st.bodies.get(b) else { continue };
            if mb.body.is_mesh() {
                continue;
            }
            let faces = BodyFaces::new(&mb.name, &mb.body);
            if let Some(f) = faces.faces_at(at, point_tol(st, b, tol)).first() {
                let note = format!("found by its position, not its name (`{name}` names no face now); check it");
                return Ok(hit(b, Some(*f), None, at, "point", Some(note)));
            }
        }
        return Err(format!("face `{name}` is gone, and no face is at ({}, {}, {})", num(at.x), num(at.y), num(at.z)));
    }
    if let Some(name) = &p.edge {
        let near = at.unwrap_or(Vec3::ZERO);
        let bodies: Vec<usize> = named_body.into_iter().chain(part.iter().copied()).collect();
        for b in &bodies {
            let Some(mb) = st.bodies.get(*b) else { continue };
            if let Ok((e, verified, on_point)) = solidcraft_doc::naming::locate_edge_named(mb, name, near) {
                let note = (!verified && !on_point).then(|| "found by a name worked out from the shape, not exact history; check it".to_string());
                return Ok(hit(*b, None, Some(e), near, "name", note));
            }
        }
        let Some(at) = at else { return Err(format!("no edge is named `{name}` any more")) };
        for b in bodies {
            let Some(mb) = st.bodies.get(b) else { continue };
            if let Ok(Some((e, _))) = mb.body.nearest_edge(at, point_tol(st, b, tol)) {
                let note = format!("found by its position, not its name (`{name}` names no edge now); check it");
                return Ok(hit(b, None, Some(e), at, "point", Some(note)));
            }
        }
        return Err(format!("edge `{name}` is gone, and no edge is at ({}, {}, {})", num(at.x), num(at.y), num(at.z)));
    }
    // A point alone: the face it is on.
    let Some(at) = at else { return Err("the pick names nothing to find".into()) };
    let bodies: Vec<usize> = named_body.into_iter().chain(part.iter().copied()).collect();
    for b in bodies {
        let Some(mb) = st.bodies.get(b) else { continue };
        if mb.body.is_mesh() {
            continue;
        }
        if let Some(f) = BodyFaces::new(&mb.name, &mb.body).faces_at(at, point_tol(st, b, tol)).first() {
            return Ok(hit(b, Some(*f), None, at, "point", None));
        }
    }
    Err(format!("no face is at ({}, {}, {})", num(at.x), num(at.y), num(at.z)))
}

/// A face's or edge's persistent-name pick, with a point on it.
pub(crate) fn pick_of(st: &ModelState, body: usize, face: Option<usize>, edge: Option<usize>, at: Option<Vec3>) -> Option<Pick> {
    let b = st.bodies.get(body)?;
    let r = |p: Vec3| [super::round(p.x, 1e6), super::round(p.y, 1e6), super::round(p.z, 1e6)];
    if let Some(f) = face {
        let name = solidcraft_doc::naming::face_names(b).get(f).cloned()?;
        let at = at.or_else(|| BodyFaces::new(&b.name, &b.body).point_on(f));
        return Some(Pick { body: Some(b.name.clone()), face: Some(name), at: at.map(r), ..Pick::default() });
    }
    let e = edge?;
    let name = solidcraft_doc::naming::edge_names(b).get(e).cloned()?;
    Some(Pick { body: Some(b.name.clone()), edge: Some(name), at: at.map(r), ..Pick::default() })
}

/// The faces of feature `id` as picks: a hole's wall, a pocket's floor, a face, a profile's face,
/// the faces of a round, chamfer, curved area or turned diameter.
pub(crate) fn feature_picks(set: &FeatureSet, st: &ModelState, id: &str) -> Vec<Pick> {
    let r = &set.reply;
    let (body, faces): (&str, Vec<usize>) = if let Some(h) = r.holes.iter().find(|h| h.id == id) {
        (&h.body, h.faces.iter().take(1).copied().collect())
    } else if let Some(p) = r.pockets.iter().find(|p| p.id == id) {
        (&p.body, vec![p.floor_face])
    } else if let Some(f) = r.faces.iter().find(|f| f.id == id) {
        (&f.body, vec![f.face])
    } else if let Some(p) = r.profiles.iter().find(|p| p.id == id) {
        match r.faces.iter().find(|f| f.id == p.face_id) {
            Some(f) => (&p.body, vec![f.face]),
            None => return Vec::new(),
        }
    } else if let Some(x) = r.fillets.iter().chain(&r.chamfers).find(|x| x.id == id) {
        (&x.body, x.faces.clone())
    } else if let Some(x) = r.surfaces.iter().find(|x| x.id == id) {
        (&x.body, x.faces.clone())
    } else if let Some(x) = r.turned.iter().find(|x| x.id == id) {
        (&x.body, x.faces.clone())
    } else {
        return Vec::new();
    };
    let Some(b) = st.bodies.iter().position(|mb| mb.name == body) else { return Vec::new() };
    faces.into_iter().take(MAX_PICKS).filter_map(|f| pick_of(st, b, Some(f), None, None)).collect()
}

/// Operation `op` (as `cam.suggest` writes it) with its `feature` (or an indexed one's) given as
/// picks instead; an operation without one as it is.
pub(crate) fn as_picks(set: &FeatureSet, st: &ModelState, op: &Value) -> Value {
    let mut op = op.clone();
    let target = if op.get("operation").is_some() { op.get_mut("operation") } else { Some(&mut op) };
    if let Some(Value::Object(m)) = target
        && let Some(id) = m.get("feature").and_then(Value::as_str).map(str::to_string)
    {
        let picks = feature_picks(set, st, &id);
        if !picks.is_empty() {
            m.remove("feature");
            m.insert("picks".into(), json!(picks));
        }
    }
    op
}

/// The `cam.features` ids face `f` of body `name` belongs to, most specific first: a hole's
/// wall, a pocket's floor or wall, a face, a profile's face or side, a round, a chamfer, a
/// surface, a turned diameter.
pub(crate) fn features_of_face(set: &FeatureSet, faces: &BodyFaces, name: &str, f: usize, origin: Vec3, tol: f64) -> Vec<String> {
    let r = &set.reply;
    let mut out: Vec<String> = Vec::new();
    let mut push = |id: &str| {
        if !out.iter().any(|o| o == id) {
            out.push(id.to_string());
        }
    };
    for h in r.holes.iter().filter(|h| h.body == name && h.faces.contains(&f)) {
        push(&h.id);
    }
    // A countersink's cone: the hole on its axis.
    if let Some(FaceSurface::Cone { apex, axis, .. }) = faces.surface(f) {
        for h in r.holes.iter().filter(|h| h.body == name && h.countersink.is_some()) {
            let Ok(g) = set.get(&h.id) else { continue };
            let Some(su) = set.setup(g) else { continue };
            let local = su.frame.to_local(apex - origin);
            let along = axis.vec().dot(su.frame.z).abs() >= 1.0 - 1e-6;
            if along && (local.x - h.center[0]).abs() <= tol * 10.0 && (local.y - h.center[1]).abs() <= tol * 10.0 {
                push(&h.id);
            }
        }
    }
    for p in r.pockets.iter().filter(|p| p.body == name && p.floor_face == f) {
        push(&p.id);
    }
    // A wall: square to the tool axis of the feature's setup, within its outline and depth.
    let within = |id: &str, bounds: [[f64; 2]; 2], lo_z: f64, hi_z: f64| -> bool {
        let Ok(g) = set.get(id) else { return false };
        let Some(su) = set.setup(g) else { return false };
        let upright = match faces.surface(f) {
            Some(FaceSurface::Plane { normal, .. }) => normal.vec().dot(su.frame.z).abs() <= 1e-6,
            Some(FaceSurface::Cylinder { axis, .. }) => axis.vec().dot(su.frame.z).abs() >= 1.0 - 1e-9,
            _ => false,
        };
        let Some([lo, hi]) = faces.extent(f, origin, &su.frame).filter(|_| upright) else { return false };
        lo[0] >= bounds[0][0] - tol
            && hi[0] <= bounds[1][0] + tol
            && lo[1] >= bounds[0][1] - tol
            && hi[1] <= bounds[1][1] + tol
            && lo[2] >= lo_z - tol
            && hi[2] <= hi_z + tol
    };
    for p in r.pockets.iter().filter(|p| p.body == name) {
        if within(&p.id, p.bounds, p.floor_z, p.top_z) {
            push(&p.id);
        }
    }
    for x in r.faces.iter().filter(|x| x.body == name && x.face == f) {
        if let Some(p) = &x.pocket {
            push(p);
        }
        push(&x.id);
        for pr in r.profiles.iter().filter(|pr| pr.face_id == x.id) {
            push(&pr.id);
        }
    }
    for pr in r.profiles.iter().filter(|p| p.body == name) {
        if within(&pr.id, pr.bounds, pr.bottom_z, pr.top_z) {
            push(&pr.id);
        }
    }
    for x in r.fillets.iter().chain(&r.chamfers).filter(|x| x.body == name && x.faces.contains(&f)) {
        push(&x.id);
    }
    for x in r.surfaces.iter().filter(|x| x.body == name && x.faces.contains(&f)) {
        push(&x.id);
    }
    for x in r.turned.iter().filter(|x| x.body == name && x.faces.contains(&f)) {
        push(&x.id);
    }
    out
}

/// The features face `f` (or the faces either side of edge `e`) belongs to.
fn features_of_hit(set: &FeatureSet, st: &ModelState, h: &Hit, origin: Vec3, tol: f64) -> Vec<String> {
    let Some(mb) = st.bodies.get(h.body) else { return Vec::new() };
    let faces = BodyFaces::new(&mb.name, &mb.body);
    match (h.face, h.edge) {
        (Some(f), _) => features_of_face(set, &faces, &mb.name, f, origin, tol),
        (None, Some(e)) => {
            let mut out: Vec<String> = Vec::new();
            for f in faces.edge_faces(e) {
                for id in features_of_face(set, &faces, &mb.name, f, origin, tol) {
                    if !out.contains(&id) {
                        out.push(id);
                    }
                }
            }
            // An edge's own features (a pocket's rim) before the top face it also bounds.
            out.sort_by_key(|id| id.starts_with("face:"));
            out
        }
        _ => Vec::new(),
    }
}

/// The kind of a feature id (`pocket`, `hole`...).
fn kind_of(id: &str) -> &str {
    id.split(':').next().unwrap_or_default()
}

/// What feature kinds an operation type reads its picks as, best first.
fn wanted(ty: &str) -> &'static [&'static str] {
    match ty {
        "pocket" | "adaptive" => &["pocket", "hole", "face"],
        "contour" => &["profile", "pocket", "hole", "face"],
        "chamfer" => &["chamfer", "pocket", "hole", "profile"],
        "drill" | "bore" | "thread_mill" => &["hole"],
        "face" => &["face"],
        "rotary_roughing" | "rotary_finish" => &["turned"],
        _ => &["surface", "fillet", "chamfer", "pocket", "face", "profile", "hole", "turned"],
    }
}

/// Picks of an operation found again: the hits, their features, and what each reports.
struct Found {
    hits: Vec<(usize, Hit, Option<String>)>,
    sketches: Vec<Pick>,
    report: Vec<PickResolved>,
    lost: Vec<(usize, String)>,
}

/// Everything resolution needs about the job and the model.
pub(crate) struct Model<'a> {
    pub s: &'a Session,
    pub st: std::sync::Arc<ModelState>,
    pub part: Vec<usize>,
    pub origin: Vec3,
    pub tol: f64,
    pub machine: Machine,
    pub features: Lazy,
    pub cmd: &'a str,
    pub tools: Vec<Tool>,
    pub stock: Option<Value>,
}

impl<'a> Model<'a> {
    pub fn new(cmd: &'a str, s: &'a Session, job: &CamJob, machine: Machine, tools: Vec<Tool>) -> Result<Model<'a>> {
        let st = s.world_state()?;
        let fixtures = job.fixtures.clone().unwrap_or_default();
        let part = part_bodies(&st, job.bodies.as_deref(), &fixtures);
        let origin = v3(job.origin.unwrap_or([0.0; 3]));
        let tol = job.tolerance.unwrap_or(0.01).clamp(super::TOLERANCE_RANGE.0, super::TOLERANCE_RANGE.1);
        let features = Lazy::new(job.bodies.clone(), fixtures, origin, tol, &machine);
        Ok(Model { s, st, part, origin, tol, machine, features, cmd, tools, stock: None })
    }

    fn set(&self) -> Option<&FeatureSet> {
        self.features.get(self.cmd, self.s).ok()
    }

    /// The part's top and bottom along `frame`'s Z (exact extents of its faces).
    fn part_range(&self, frame: &Frame) -> Option<(f64, f64)> {
        let mut r: Option<(f64, f64)> = None;
        for b in &self.part {
            let mb = self.st.bodies.get(*b)?;
            let faces = BodyFaces::new(&mb.name, &mb.body);
            for f in 0..faces.face_count() {
                if let Some([lo, hi]) = faces.extent(f, self.origin, frame) {
                    r = Some(r.map_or((lo[2], hi[2]), |(a, b)| (a.min(lo[2]), b.max(hi[2]))));
                }
            }
        }
        r
    }

    /// The stock's top along `frame`'s Z, when the job's stock gives one there.
    fn stock_top(&self, frame: &Frame) -> Option<f64> {
        let st = self.stock.as_ref()?;
        if let Some(c) = st.get("cylinder") {
            return c.get("radius").and_then(Value::as_f64);
        }
        if frame.z.dot(Vec3::Z) < 1.0 - 1e-9 {
            return None;
        }
        if let Some(b) = st.get("box") {
            return b.get("max").and_then(|m| m.get(2)).and_then(Value::as_f64);
        }
        if let Some(a) = st.get("around_part") {
            let (_, top) = self.part_range(frame)?;
            let m = a.get("top").and_then(Value::as_f64).or_else(|| a.get("margin").and_then(|m| m.get(2)).and_then(Value::as_f64))?;
            return Some(top + m);
        }
        None
    }

    fn tool(&self, op: &Map<String, Value>) -> Option<&Tool> {
        let n = op.get("tool").and_then(Value::as_u64)?;
        self.tools.iter().find(|t| u64::from(t.number) == n)
    }
}

/// Finds an operation's picks again.
fn find_all(m: &Model<'_>, op: Option<&OpRef>, picks: &[Pick], ty: &str) -> Found {
    let mut found = Found { hits: Vec::new(), sketches: Vec::new(), report: Vec::new(), lost: Vec::new() };
    let set = if picks.iter().any(|p| p.sketch.is_none()) { m.set() } else { None };
    for (k, p) in picks.iter().enumerate() {
        if p.sketch.is_some() {
            found.sketches.push(p.clone());
            found.report.push(PickResolved { op: op.cloned(), pick: k, by: "name".into(), ..PickResolved::default() });
            continue;
        }
        match find(&m.st, &m.part, p, m.tol) {
            Ok(h) => {
                let ids = set.map(|set| features_of_hit(set, &m.st, &h, m.origin, m.tol)).unwrap_or_default();
                let want = wanted(ty);
                let feature = want.iter().find_map(|w| ids.iter().find(|id| kind_of(id) == *w)).cloned().or_else(|| ids.first().cloned());
                found.report.push(PickResolved {
                    op: op.cloned(),
                    pick: k,
                    by: h.by.into(),
                    body: Some(h.body_name.clone()),
                    face: h.face,
                    edge: h.edge,
                    feature: feature.clone(),
                    note: h.note.clone(),
                });
                found.hits.push((k, h, feature));
            }
            Err(why) => {
                found.report.push(PickResolved {
                    op: op.cloned(),
                    pick: k,
                    by: "lost".into(),
                    body: p.body.clone(),
                    note: Some(why.clone()),
                    ..PickResolved::default()
                });
                found.lost.push((k, why));
            }
        }
    }
    found
}

/// The heights an operation that needs all four takes from geometry `top` and `bottom` in
/// `frame`, those it gives itself kept.
fn fill_heights(m: &Model<'_>, op: &mut Map<String, Value>, frame: &Frame, top: f64, bottom: f64, all: bool) {
    let part_top = m.part_range(frame).map_or(top, |(_, t)| t);
    let stock_top = m.stock_top(frame).filter(|s| *s > part_top);
    let above = stock_top.unwrap_or(part_top);
    let h = op.entry("heights").or_insert_with(|| Value::Object(Map::new()));
    let Value::Object(h) = h else { return };
    let mut put = |k: &str, v: f64| {
        h.entry(k.to_string()).or_insert(json!(super::round(v, 1e6)));
    };
    put("top", top);
    put("bottom", bottom);
    if all {
        put("retract", top.max(stock_top.unwrap_or(f64::NEG_INFINITY)) + RETRACT_ABOVE);
        put("clearance", above + CLEARANCE_ABOVE);
    }
}

/// The outer outline of feature `id` as a polygon in its setup's frame.
fn outline_of(set: &FeatureSet, id: &str) -> Option<(Vec<[f64; 2]>, bool)> {
    let r = &set.reply;
    if let Some(p) = r.pockets.iter().find(|p| p.id == id) {
        return Some((p.outline.clone(), !p.islands.is_empty()));
    }
    if let Some(p) = r.profiles.iter().find(|p| p.id == id) {
        return Some((p.outline.clone(), false));
    }
    if let Some(h) = r.holes.iter().find(|h| h.id == id) {
        let n = 72;
        let rad = h.diameter * 0.5;
        let pts = (0..n)
            .map(|i| {
                let t = std::f64::consts::TAU * f64::from(i) / f64::from(n);
                [super::round(h.center[0] + rad * t.cos(), 1e6), super::round(h.center[1] + rad * t.sin(), 1e6)]
            })
            .collect();
        return Some((pts, false));
    }
    let g = set.get(id).ok()?;
    let outer = g.outer.as_ref()?;
    let pts: Vec<[f64; 2]> = super::features::sample_loop(outer, set.reply.tolerance).iter().map(|p| [p.x, p.y]).collect();
    Some((pts, !g.islands.is_empty()))
}

/// The pocket, hole or outline whose rim chamfer `id` breaks: the smallest whose extent, grown by
/// the chamfer's own size, holds the chamfer.
fn rim_of(set: &FeatureSet, id: &str) -> Option<String> {
    let r = &set.reply;
    let c = r.chamfers.iter().find(|c| c.id == id)?;
    let grow = (c.top_z - c.bottom_z).abs() * 2.0 + set.reply.tolerance.max(1e-3);
    // The chamfer lies along the rim: within the feature's extent grown by its own size.
    let near = |b: [[f64; 2]; 2]| (0..2).all(|k| c.bounds[0][k] >= b[0][k] - grow && c.bounds[1][k] <= b[1][k] + grow);
    let area = |b: [[f64; 2]; 2]| (b[1][0] - b[0][0]) * (b[1][1] - b[0][1]);
    let hole_bounds = |h: &crate::schema::cam_features::HoleReport| {
        let rad = h.diameter * 0.5;
        [[h.center[0] - rad, h.center[1] - rad], [h.center[0] + rad, h.center[1] + rad]]
    };
    let same_side = |a: Option<f64>| (a.unwrap_or(0.0) - c.a_deg.unwrap_or(0.0)).abs() <= 1e-6;
    let mut found: Vec<(f64, String)> = Vec::new();
    for p in r.pockets.iter().filter(|p| p.body == c.body && same_side(p.a_deg) && near(p.bounds)) {
        found.push((area(p.bounds), p.id.clone()));
    }
    for h in r.holes.iter().filter(|h| h.body == c.body && same_side(h.a_deg) && near(hole_bounds(h))) {
        found.push((area(hole_bounds(h)), h.id.clone()));
    }
    for p in r.profiles.iter().filter(|p| p.body == c.body && same_side(p.a_deg) && near(p.bounds)) {
        found.push((area(p.bounds), p.id.clone()));
    }
    found.sort_by(|a, b| a.0.total_cmp(&b.0));
    found.into_iter().next().map(|(_, id)| id)
}

/// The A angle of feature `id`'s setup (0 on top).
fn a_of(set: &FeatureSet, id: &str) -> f64 {
    set.get(id).ok().and_then(|g| set.setup(g)).map_or(0.0, |su| su.a_deg)
}

fn frame_of(set: &FeatureSet, id: &str) -> Frame {
    set.get(id).ok().and_then(|g| set.setup(g)).map_or(Frame::IDENTITY, |su| su.frame)
}

/// The op-level geometry the hits give an operation of type `ty`: keys written into `op`, and
/// the A angle it must be turned to (when its picks are on another side and it is not indexed).
/// `wrapped`: the A angle of the indexed operation it is inside.
fn apply(
    m: &Model<'_>,
    ty: &str,
    op: &mut Map<String, Value>,
    found: &Found,
    wrapped: Option<f64>,
    pattern: bool,
) -> std::result::Result<Option<f64>, String> {
    // Sketch picks: the sketch and its profiles or curve.
    if !found.sketches.is_empty() {
        let names: BTreeSet<&str> = found.sketches.iter().filter_map(|p| p.sketch.as_deref()).collect();
        let names: Vec<&str> = names.into_iter().collect();
        let [sketch] = names.as_slice() else { return Err("the picks are in more than one sketch: give one operation each".into()) };
        op.insert("sketch".into(), json!(sketch));
        let profiles: Vec<usize> = found.sketches.iter().filter_map(|p| p.profile).collect();
        if let Some(curve) = found.sketches.iter().find_map(|p| p.curve.clone()) {
            op.insert("curve".into(), json!(curve));
        } else if !profiles.is_empty() && !matches!(ty, "drill" | "bore" | "thread_mill" | "wrap") {
            op.insert("profiles".into(), json!(profiles));
        }
    }
    if found.hits.is_empty() {
        return Ok(None);
    }
    let set = m.set().ok_or_else(|| "the part's features could not be found".to_string())?;
    let ids: Vec<&str> = {
        let mut v: Vec<&str> = Vec::new();
        for (_, _, f) in &found.hits {
            if let Some(f) = f.as_deref()
                && !v.contains(&f)
            {
                v.push(f);
            }
        }
        v
    };
    // Round walls that are no recognised hole (one opening on a curved face): read as holes from
    // their exact cylinders.
    let flat_hole = |id: &&str| set.reply.holes.iter().any(|h| h.id == **id && h.depth <= m.tol);
    if matches!(ty, "drill" | "bore" | "thread_mill") && (!ids.iter().any(|id| kind_of(id) == "hole") || ids.iter().any(flat_hole)) {
        return holes_from_walls(m, set, ty, op, found, wrapped);
    }
    let side = ids.first().map_or(0.0, |id| a_of(set, id));
    if ids.iter().any(|id| (a_of(set, id) - side).abs() > 1e-6) {
        return Err("the picks are on different sides of the part (different A angles): give one operation each".into());
    }
    // A plain operation on another side is turned there (pockets and contours on a feature are
    // turned by the planner itself).
    let turn = (wrapped.is_none() && side.abs() > 1e-9).then_some(side);
    let frame = ids
        .first()
        .map_or_else(|| wrapped.and_then(|a| frame_for_index(&m.machine, &[a.to_radians()]).ok()).unwrap_or(Frame::IDENTITY), |id| frame_of(set, id));
    let one = |kinds: &[&str]| -> std::result::Result<&str, String> {
        let fit: Vec<&str> = ids.iter().copied().filter(|id| kinds.contains(&kind_of(id))).collect();
        match fit.as_slice() {
            [one] => Ok(one),
            [] => Err(format!(
                "the picks are not {} (they are {})",
                kinds.join(" or "),
                if ids.is_empty() { "on no feature this operation reads".to_string() } else { ids.join(", ") }
            )),
            many => Err(format!("the picks are {} features ({}): give one operation each", many.len(), many.join(", "))),
        }
    };
    let r = &set.reply;
    match ty {
        "pocket" | "contour" => {
            let id = one(wanted(ty))?;
            // In an indexed pattern (turned to several angles), or turned to another side than its
            // own, a feature is cut as its outline in its own frame; else by its id, islands and all.
            if !pattern && wrapped.is_none_or(|a| (a - a_of(set, id)).abs() <= 1e-9) {
                op.insert("feature".into(), json!(id));
                return Ok(None);
            }
            let (pts, islands) = outline_of(set, id).ok_or_else(|| format!("{id} has no outline"))?;
            if islands && ty == "pocket" {
                return Err(format!("{id} has islands, which an indexed pattern cannot cut round: pocket it on its own side"));
            }
            op.insert("polygon".into(), json!(pts));
            if ty == "contour" && !op.contains_key("side") && kind_of(id) != "profile" {
                op.insert("side".into(), json!("inside"));
            }
            Ok(turn)
        }
        "adaptive" => {
            let id = one(wanted(ty))?;
            if r.faces.iter().any(|f| f.id == id && matches!(f.role.as_str(), "step" | "plateau")) {
                return Err(format!(
                    "{id} is open on some sides: Light-Load Clearing stays inside its outline and would leave a wall of material along                      them; use Clear Pocket, which runs past the open sides"
                ));
            }
            let (pts, islands) = outline_of(set, id).ok_or_else(|| format!("{id} has no outline"))?;
            if islands {
                return Err(format!("{id} has islands: Light-Load Clearing cuts a whole outline; use Clear Pocket for it"));
            }
            let g = set.get(id)?;
            op.insert("polygon".into(), json!(pts));
            fill_heights(m, op, &frame, g.top, g.bottom, true);
            Ok(turn)
        }
        "chamfer" => {
            let picked = one(&["pocket", "hole", "profile", "chamfer"])?;
            // A chamfer face: the pocket, hole or outline whose rim it breaks.
            let rim = if kind_of(picked) == "chamfer" { rim_of(set, picked) } else { None };
            let id = rim.as_deref().unwrap_or(picked);
            let g = set.get(id)?;
            let (pts, _) = outline_of(set, id).ok_or_else(|| format!("{id} has no outline"))?;
            op.insert("polygon".into(), json!(pts));
            if !op.contains_key("side") {
                op.insert("side".into(), json!(if kind_of(id) == "profile" { "outside" } else { "inside" }));
            }
            // The edge's top and the chamfer's width across: a chamfer face's own, a countersunk
            // hole's countersink, else a 0.5 mm deburr at the feature's top.
            let countersink = r.holes.iter().find(|h| h.id == id).and_then(|h| h.countersink.as_ref().map(|c| (h, c)));
            let (top, width) = if let Some(c) = r.chamfers.iter().find(|c| c.id == picked) {
                let depth = c.top_z - c.bottom_z;
                (c.top_z, c.angle_deg.map_or(depth, |a| depth * a.to_radians().tan()))
            } else if let Some((h, cs)) = countersink {
                let w = ((cs.diameter - h.diameter) * 0.5).max(0.0);
                let half = (cs.angle_deg * 0.5).to_radians();
                (h.top_z + if half.tan() > 1e-9 { w / half.tan() } else { 0.0 }, w)
            } else {
                (g.top, DEBURR)
            };
            op.entry("width").or_insert(json!(super::round(width, 1e6)));
            let h = op.entry("heights").or_insert_with(|| Value::Object(Map::new()));
            if let Value::Object(h) = h {
                let part_top = m.part_range(&frame).map_or(top, |(_, t)| t);
                let above = m.stock_top(&frame).filter(|s| *s > part_top).unwrap_or(part_top);
                h.entry("top").or_insert(json!(top));
                h.entry("retract").or_insert(json!(super::round(top.max(above) + RETRACT_ABOVE, 1e6)));
                h.entry("clearance").or_insert(json!(super::round(above + CLEARANCE_ABOVE, 1e6)));
            }
            Ok(turn)
        }
        "drill" | "bore" | "thread_mill" => {
            let holes: Vec<&crate::schema::cam_features::HoleReport> =
                ids.iter().filter(|id| kind_of(id) == "hole").filter_map(|id| r.holes.iter().find(|h| h.id == *id)).collect();
            if holes.is_empty() || holes.len() != ids.len() {
                return Err(format!("the picks are not all holes ({})", ids.join(", ")));
            }
            let top = holes.iter().map(|h| h.top_z).fold(f64::NEG_INFINITY, f64::max);
            let bottom = holes.iter().map(|h| h.bottom_z).fold(f64::INFINITY, f64::min);
            match ty {
                "drill" => {
                    op.insert("points".into(), json!(holes.iter().map(|h| h.center).collect::<Vec<_>>()));
                }
                "bore" => {
                    op.insert("holes".into(), json!(holes.iter().map(|h| json!({"center": h.center, "diameter": h.diameter})).collect::<Vec<_>>()));
                }
                _ => {
                    op.insert("centers".into(), json!(holes.iter().map(|h| h.center).collect::<Vec<_>>()));
                }
            }
            fill_heights(m, op, &frame, top, bottom, true);
            Ok(turn)
        }
        "face" => {
            let id = one(&["face"])?;
            let z = r.faces.iter().find(|f| f.id == id).map(|f| f.z).ok_or_else(|| format!("{id} is not a face"))?;
            op.entry("bottom").or_insert(json!(z));
            Ok(turn)
        }
        "rotary_roughing" | "rotary_finish" => {
            let turned: Vec<&crate::schema::cam_features::TurnedReport> = r.turned.iter().filter(|t| ids.contains(&t.id.as_str())).collect();
            if turned.is_empty() {
                return Err(format!("the picks are not turned diameters ({})", ids.join(", ")));
            }
            let lo = turned.iter().map(|t| t.x_min).fold(f64::INFINITY, f64::min);
            let hi = turned.iter().map(|t| t.x_max).fold(f64::NEG_INFINITY, f64::max);
            op.entry("range").or_insert(json!([lo, hi]));
            Ok(None)
        }
        "wrap" | "wrap_pocket" | "wrap_contour" => {
            // A cylinder face about X gives the radius.
            for (_, h, _) in &found.hits {
                let Some(mb) = m.st.bodies.get(h.body) else { continue };
                if let Some(FaceSurface::Cylinder { radius, axis, .. }) = h.face.and_then(|f| BodyFaces::new(&mb.name, &mb.body).surface(f))
                    && axis.vec().x.abs() >= 1.0 - 1e-9
                {
                    op.entry("radius").or_insert(json!(radius));
                }
            }
            Ok(None)
        }
        _ if AREA_TYPES.contains(&ty) => {
            // The area: the picked faces' exact extent in the frame, grown by the tool's radius.
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for (_, h, _) in &found.hits {
                let Some(mb) = m.st.bodies.get(h.body) else { continue };
                let faces = BodyFaces::new(&mb.name, &mb.body);
                let list: Vec<usize> = match (h.face, h.edge) {
                    (Some(f), _) => vec![f],
                    (None, Some(e)) => faces.edge_faces(e),
                    _ => Vec::new(),
                };
                for f in list {
                    if let Some([a, b]) = faces.extent(f, m.origin, &frame) {
                        for k in 0..3 {
                            lo[k] = lo[k].min(a[k]);
                            hi[k] = hi[k].max(b[k]);
                        }
                    }
                }
            }
            if !lo.iter().chain(&hi).all(|x| x.is_finite()) {
                return Err("the picked faces have no extent".into());
            }
            let grow = m.tool(op).map_or(0.0, |t| t.radius());
            let rect = [[lo[0] - grow, lo[1] - grow], [hi[0] + grow, hi[1] + grow]].map(|p| p.map(|x| super::round(x, 1e6)));
            let b = op.entry("boundary").or_insert_with(|| Value::Object(Map::new()));
            if let Value::Object(b) = b {
                b.entry("rect").or_insert(json!(rect));
            }
            let all = FULL_HEIGHTS.contains(&ty);
            let top = m.part_range(&frame).map_or(hi[2], |(_, t)| t);
            if all {
                fill_heights(m, op, &frame, top, lo[2], true);
            } else if let Value::Object(h) = op.entry("heights").or_insert_with(|| Value::Object(Map::new())) {
                h.entry("bottom").or_insert(json!(super::round(lo[2], 1e6)));
            }
            Ok(turn)
        }
        _ => Err(format!("a `{ty}` operation takes no picks")),
    }
}

/// The side a round wall (face `f`, its axis `axis`) opens on: of the setups along its axis, the
/// one whose top it reaches; and its extent there.
fn wall_side(m: &Model<'_>, set: &FeatureSet, faces: &BodyFaces, f: usize, axis: Vec3) -> Option<(usize, [[f64; 3]; 2])> {
    let mut best: Option<(f64, usize, [[f64; 3]; 2])> = None;
    for (k, su) in set.setups.iter().enumerate() {
        if axis.dot(su.frame.z).abs() < 1.0 - 1e-6 {
            continue;
        }
        let Some(ext) = faces.extent(f, m.origin, &su.frame) else { continue };
        let gap = (su.top - ext[1][2]).abs();
        if best.as_ref().is_none_or(|b| gap < b.0) {
            best = Some((gap, k, ext));
        }
    }
    best.map(|(_, k, ext)| (k, ext))
}

/// Drilling, boring or thread milling on picked round walls: each wall's exact cylinder gives a
/// hole along the tool axis of the side it opens on (its centre, diameter, top and bottom there).
fn holes_from_walls(
    m: &Model<'_>,
    set: &FeatureSet,
    ty: &str,
    op: &mut Map<String, Value>,
    found: &Found,
    wrapped: Option<f64>,
) -> std::result::Result<Option<f64>, String> {
    let mut holes: Vec<([f64; 2], f64, f64, f64)> = Vec::new();
    let mut side: Option<usize> = None;
    for (_, h, _) in &found.hits {
        let mb = m.st.bodies.get(h.body).ok_or("its body is gone")?;
        let faces = BodyFaces::new(&mb.name, &mb.body);
        let f = h.face.and_then(|f| faces.round_wall(f)).ok_or("the picks are not holes: pick a hole's round wall or its floor")?;
        let Some(FaceSurface::Cylinder { origin, axis, radius }) = faces.surface(f) else {
            return Err("the picks are not holes: pick a hole's round wall".into());
        };
        let (k, ext) = wall_side(m, set, &faces, f, axis.vec()).ok_or("the hole is along no direction the machine can turn up")?;
        if side.is_some_and(|s| s != k) {
            return Err("the holes are on different sides of the part (different A angles): give one operation each".into());
        }
        side = Some(k);
        let su = set.setups.get(k).ok_or("its side is gone")?;
        let c = su.frame.to_local(origin - m.origin);
        // Where it opens: the top of its wall, or (an opening whose curve gives no extent) the
        // side's top.
        let top = if ext[1][2] - ext[0][2] > m.tol { ext[1][2].min(su.top) } else { su.top };
        holes.push(([super::round(c.x, 1e6), super::round(c.y, 1e6)], radius * 2.0, top, ext[0][2]));
    }
    let su = side.and_then(|k| set.setups.get(k)).ok_or("nothing picked")?;
    match ty {
        "drill" => {
            op.insert("points".into(), json!(holes.iter().map(|h| h.0).collect::<Vec<_>>()));
        }
        "bore" => {
            op.insert("holes".into(), json!(holes.iter().map(|h| json!({"center": h.0, "diameter": h.1})).collect::<Vec<_>>()));
        }
        _ => {
            op.insert("centers".into(), json!(holes.iter().map(|h| h.0).collect::<Vec<_>>()));
        }
    }
    let top = holes.iter().map(|h| h.2).fold(f64::NEG_INFINITY, f64::max);
    let bottom = holes.iter().map(|h| h.3).fold(f64::INFINITY, f64::min);
    fill_heights(m, op, &su.frame, top, bottom, true);
    Ok((wrapped.is_none() && su.a_deg.abs() > 1e-9).then_some(su.a_deg))
}

/// The height `r` gives in `frame`.
fn height_of(m: &Model<'_>, r: &HeightRef, frame: &Frame) -> std::result::Result<(f64, Option<String>), String> {
    let offset = r.offset.unwrap_or(0.0);
    if !(offset.is_finite() && offset.abs() <= super::MAX_COORD) {
        return Err("a height's `offset` must be a finite number of mm".into());
    }
    let (z, note) = match &r.from {
        HeightSource::Named(n) => match n.as_str() {
            "part_top" => (m.part_range(frame).ok_or("the part has no extent")?.1, None),
            "part_bottom" => (m.part_range(frame).ok_or("the part has no extent")?.0, None),
            "stock_top" => (m.stock_top(frame).ok_or("the job's stock gives no top here")?, None),
            other => return Err(format!("a height is from `stock_top`, `part_top`, `part_bottom` or a picked face, not `{other}`")),
        },
        HeightSource::Face(p) => {
            check_pick(p)?;
            let h = find(&m.st, &m.part, p, m.tol)?;
            let f = h.face.ok_or("a height follows a face, not an edge")?;
            let mb = m.st.bodies.get(h.body).ok_or("its body is gone")?;
            let faces = BodyFaces::new(&mb.name, &mb.body);
            match faces.surface(f) {
                Some(FaceSurface::Plane { normal, .. }) if normal.vec().dot(frame.z).abs() >= 1.0 - 1e-6 => {
                    let [lo, _] = faces.extent(f, m.origin, frame).ok_or("the face has no extent")?;
                    (lo[2], h.note)
                }
                _ => return Err("a height follows a planar face square to the tool axis".into()),
            }
        }
    };
    Ok((super::round(z + offset, 1e9), note))
}

/// Replaces the height references of `op` (`heights.*`, a face operation's `bottom` and `top`)
/// with the heights they give in `frame`.
fn heights_in(m: &Model<'_>, op: &mut Map<String, Value>, frame: &Frame, notes: &mut Vec<String>) -> std::result::Result<(), String> {
    let mut one = |v: &mut Value, key: &str| -> std::result::Result<(), String> {
        if !v.is_object() {
            return Ok(());
        }
        let r: HeightRef = serde_json::from_value(v.clone()).map_err(|e| format!("`{key}`: {e}"))?;
        let (z, note) = height_of(m, &r, frame).map_err(|e| format!("`{key}`: {e}"))?;
        notes.extend(note.map(|n| format!("`{key}`'s face {n}")));
        *v = json!(z);
        Ok(())
    };
    if let Some(Value::Object(h)) = op.get_mut("heights") {
        for k in HEIGHT_KEYS {
            if let Some(v) = h.get_mut(*k) {
                one(v, &format!("heights.{k}"))?;
            }
        }
    }
    for k in ["bottom", "top"] {
        if let Some(v) = op.get_mut(k) {
            one(v, k)?;
        }
    }
    Ok(())
}

/// Whether `v` (a job) has picks, height references or `origin_from`.
pub(crate) fn has_refs(v: &Value) -> bool {
    if v.get("origin_from").is_some() {
        return true;
    }
    let refs = |o: &Value| {
        o.get("picks").is_some()
            || o.get("heights").and_then(Value::as_object).is_some_and(|h| h.values().any(Value::is_object))
            || ["bottom", "top"].iter().any(|k| o.get(*k).is_some_and(Value::is_object))
    };
    v.get("operations").and_then(Value::as_array).is_some_and(|ops| ops.iter().any(|o| refs(o) || o.get("operation").is_some_and(refs)))
}

/// `job` as the schema reads it without the model: picks checked and taken out, height
/// references checked and read as 0, and the heights picks would give put in as 0. What a job
/// with picks would be once they are found, for checking its every other key.
pub(crate) fn placeholders(job: &Value) -> std::result::Result<Value, String> {
    let mut out = job.clone();
    if let Some(of) = out.get("origin_from") {
        serde_json::from_value::<OriginFrom>(of.clone()).map_err(|e| format!("`origin_from`: {e}"))?;
        check_origin_from(of)?;
    }
    let Some(Value::Array(ops)) = out.get_mut("operations") else { return Ok(out) };
    for (i, op) in ops.iter_mut().enumerate() {
        let at = format!("operations[{i}]");
        stub(op, &at)?;
        if let Some(inner) = op.get_mut("operation") {
            stub(inner, &format!("{at}.operation"))?;
        }
    }
    Ok(out)
}

fn check_origin_from(v: &Value) -> std::result::Result<(), String> {
    let of: OriginFrom = serde_json::from_value(v.clone()).map_err(|e| format!("`origin_from`: {e}"))?;
    match &of {
        OriginFrom::Pick(p) => check_pick(p),
        OriginFrom::Stock(g) => check_grid(g),
        OriginFrom::Axis { axis, x_end } => {
            check_pick(axis)?;
            match x_end {
                XEnd::Stock(s) if s == "stock_low" || s == "stock_high" => Ok(()),
                XEnd::Stock(s) => Err(format!("`x_end` is stock_low, stock_high or a pick, not `{s}`")),
                XEnd::Pick(p) => check_pick(p),
            }
        }
    }
    .map_err(|e| format!("`origin_from`: {e}"))
}

fn check_grid(g: &StockGrid) -> std::result::Result<(), String> {
    let xy = |p: GridPlace| matches!(p, GridPlace::Low | GridPlace::Middle | GridPlace::High);
    let z = |p: GridPlace| matches!(p, GridPlace::Top | GridPlace::Middle | GridPlace::Bottom);
    if xy(g.x) && xy(g.y) && z(g.z) { Ok(()) } else { Err("a place on the stock is x, y: low | middle | high; z: top | middle | bottom".into()) }
}

/// One operation of [`placeholders`].
fn stub(op: &mut Value, at: &str) -> std::result::Result<(), String> {
    let Some(map) = op.as_object_mut() else { return Ok(()) };
    let ty = map.get("type").and_then(Value::as_str).unwrap_or_default().to_string();
    let picked = match map.remove("picks") {
        None | Some(Value::Null) => false,
        Some(v) => {
            let picks: Vec<Pick> = serde_json::from_value(v).map_err(|e| format!("`{at}.picks`: {e}"))?;
            if picks.len() > MAX_PICKS {
                return Err(format!("`{at}.picks`: at most {MAX_PICKS} picks"));
            }
            for (k, p) in picks.iter().enumerate() {
                check_pick(p).map_err(|e| format!("`{at}.picks[{k}]`: {e}"))?;
            }
            if let Some(k) = GEOMETRY_KEYS.iter().find(|k| map.contains_key(**k) && !matches!(**k, "profiles" | "curve")) {
                return Err(format!("`{at}`: give `picks` or `{k}`, not both"));
            }
            !picks.is_empty()
        }
    };
    let mut refs = false;
    let mut zero = |v: &mut Value, key: &str| -> std::result::Result<(), String> {
        if v.is_object() {
            let r: HeightRef = serde_json::from_value(v.clone()).map_err(|e| format!("`{at}.{key}`: {e}"))?;
            if let HeightSource::Face(p) = &r.from {
                check_pick(p).map_err(|e| format!("`{at}.{key}`: {e}"))?;
            }
            *v = json!(0.0);
            refs = true;
        }
        Ok(())
    };
    if let Some(Value::Object(h)) = map.get_mut("heights") {
        for k in HEIGHT_KEYS {
            if let Some(v) = h.get_mut(*k) {
                zero(v, &format!("heights.{k}"))?;
            }
        }
    }
    for k in ["bottom", "top"] {
        if ty == "face"
            && let Some(v) = map.get_mut(k)
        {
            zero(v, k)?;
        }
    }
    if picked || refs {
        let full = FULL_HEIGHTS.contains(&ty.as_str());
        let chamfer = ty == "chamfer";
        if full || chamfer {
            let h = map.entry("heights").or_insert_with(|| Value::Object(Map::new()));
            if let Value::Object(h) = h {
                for k in HEIGHT_KEYS.iter().filter(|k| full || **k != "bottom") {
                    h.entry(k.to_string()).or_insert(json!(0.0));
                }
            }
        }
        if picked {
            match ty.as_str() {
                "face" => {
                    map.entry("bottom").or_insert(json!(0.0));
                }
                "wrap" | "wrap_pocket" | "wrap_contour" => {
                    map.entry("radius").or_insert(json!(1.0));
                }
                "chamfer" => {
                    map.entry("width").or_insert(json!(0.5));
                }
                _ => {}
            }
        }
    }
    Ok(())
}

/// The A angle (degrees) of an indexed operation's first turn: `a_deg`, the first of a list, or
/// `start_deg`.
fn wrapper_angle(op: &Map<String, Value>) -> Option<f64> {
    match op.get("a_deg") {
        Some(Value::Number(n)) => n.as_f64(),
        Some(Value::Array(a)) => a.first().and_then(Value::as_f64),
        _ => op.get("count").map(|_| op.get("start_deg").and_then(Value::as_f64).unwrap_or(0.0)),
    }
}

/// A refusal of operation `i` for its picks.
fn pick_refusal(i: usize, name: &str, error: String) -> Refusal {
    Refusal {
        error,
        op: Some(OpRef { index: i, name: name.to_string() }),
        check: "picks".into(),
        at: None,
        line: None,
        fix: format!("pick its geometry again (cam.op.edit {{op: {i}, picks: [...]}}, or Pick again in the Manufacture workspace)"),
    }
}

/// Finds every pick and height reference of the job `job` (as loaded, its profile merged) and
/// writes the keys they give into it, the picks taken out: the job the planners read. Returns how
/// each pick was found. A lost pick, or picks an operation cannot read, refuses the job (every
/// such operation listed).
pub(crate) fn resolve_job(m: &mut Model<'_>, job: &mut Map<String, Value>) -> Result<Vec<PickResolved>> {
    m.stock = job.get("stock").cloned();
    let mut report: Vec<PickResolved> = Vec::new();
    let mut refused: Vec<Refusal> = Vec::new();
    if let Some(of) = job.get("origin_from").cloned() {
        let origin = job.get("origin").and_then(|o| serde_json::from_value::<[f64; 3]>(o.clone()).ok()).unwrap_or([0.0; 3]);
        match zero_point(m, &of) {
            Ok((p, by, note)) => {
                let d = [p.x - origin[0], p.y - origin[1], p.z - origin[2]];
                let moved = d.iter().any(|x| x.abs() > m.tol);
                let note = if moved {
                    Some(format!(
                        "work zero no longer matches the way it was set: it would now be ({}, {}, {}), {} mm from the job's ({}, {}, {}); set it again \
                         (cam.setup.zero {{refresh: true}}) once the operator's machine is set the same way",
                        num(p.x),
                        num(p.y),
                        num(p.z),
                        num((d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()),
                        num(origin[0]),
                        num(origin[1]),
                        num(origin[2])
                    ))
                } else {
                    note
                };
                report.push(PickResolved { op: None, pick: 0, by: by.into(), note, ..PickResolved::default() });
            }
            Err(why) => report.push(PickResolved {
                op: None,
                pick: 0,
                by: "lost".into(),
                note: Some(format!("work zero's pick is lost ({why}): the job's `origin` numbers are used")),
                ..PickResolved::default()
            }),
        }
    }
    job.remove("origin_from");
    let Some(Value::Array(ops)) = job.get_mut("operations") else { return Ok(report) };
    for (i, op) in ops.iter_mut().enumerate() {
        let name = crate::cam_job_state::op_name(op);
        let Some(outer) = op.as_object_mut() else { continue };
        let outer_ty = outer.get("type").and_then(Value::as_str).unwrap_or_default().to_string();
        let wrapped = matches!(outer_ty.as_str(), "indexed" | "indexed_pattern").then(|| wrapper_angle(outer)).flatten();
        // The operation the picks are of: the one inside an indexed wrapper, taken out while
        // it is worked on.
        let nested = wrapped.is_some() && outer.get("operation").is_some_and(Value::is_object);
        let mut target: Map<String, Value> = if nested {
            match outer.remove("operation") {
                Some(Value::Object(inner)) => inner,
                _ => Map::new(),
            }
        } else {
            std::mem::take(outer)
        };
        let inner = &mut target;
        let inner_ty = inner.get("type").and_then(Value::as_str).unwrap_or_default().to_string();
        let r = OpRef { index: i, name: name.clone() };
        let picks: Option<Vec<Pick>> = match inner.remove("picks") {
            None | Some(Value::Null) => None,
            Some(v) => Some(serde_json::from_value(v).map_err(|e| bad(m.cmd, format!("operation {i} `{name}`: `picks`: {e}")))?),
        };
        let mut turn = None;
        let mut frame = wrapped.and_then(|a| frame_for_index(&m.machine, &[a.to_radians()]).ok()).unwrap_or(Frame::IDENTITY);
        if let Some(picks) = picks.filter(|p| !p.is_empty()) {
            if picks.len() > MAX_PICKS {
                refused.push(pick_refusal(i, &name, format!("at most {MAX_PICKS} picks")));
                continue;
            }
            if let Some(e) = picks.iter().find_map(|p| check_pick(p).err()) {
                refused.push(pick_refusal(i, &name, e));
                continue;
            }
            let found = find_all(m, Some(&r), &picks, &inner_ty);
            report.extend(found.report.iter().cloned());
            if !found.lost.is_empty() {
                let what: Vec<String> = found.lost.iter().map(|(k, why)| format!("pick {} was not found: {why}", k + 1)).collect();
                refused.push(pick_refusal(i, &name, what.join("; ")));
                continue;
            }
            match apply(m, &inner_ty, inner, &found, wrapped, outer_ty == "indexed_pattern") {
                Ok(t) => turn = t,
                Err(e) => {
                    refused.push(pick_refusal(i, &name, e));
                    continue;
                }
            }
            if let (Some(set), Some((_, _, Some(id)))) = (m.set(), found.hits.first()) {
                frame = frame_of(set, id);
            }
        }
        let mut notes = Vec::new();
        if let Err(e) = heights_in(m, inner, &frame, &mut notes) {
            refused.push(Refusal { check: "heights".into(), ..pick_refusal(i, &name, e) });
            continue;
        }
        for n in notes {
            report.push(PickResolved { op: Some(r.clone()), pick: 0, by: "point".into(), note: Some(n), ..PickResolved::default() });
        }
        if nested {
            outer.insert("operation".into(), Value::Object(target));
        } else {
            *outer = target;
        }
        if let Some(a) = turn {
            // Picks on another side: the operation runs indexed there.
            let mut plain = std::mem::take(outer);
            let enabled = plain.remove("enabled");
            let nm = plain.get("name").cloned();
            let mut wrap = Map::new();
            if let Some(nm) = nm {
                wrap.insert("name".into(), nm);
            }
            wrap.insert("type".into(), json!("indexed"));
            wrap.insert("a_deg".into(), json!(a));
            if let Some(e) = enabled {
                wrap.insert("enabled".into(), e);
            }
            wrap.insert("operation".into(), Value::Object(plain));
            *outer = wrap;
        }
    }
    if !refused.is_empty() {
        return Err(super::agent::refuse(m.cmd, "the job was not planned", &refused, "Nothing was written."));
    }
    Ok(report)
}

/// The stock's box in design coordinates: [low, high].
fn stock_box(m: &Model<'_>) -> std::result::Result<([f64; 3], [f64; 3]), String> {
    let st = m.stock.as_ref().ok_or("the job has no stock")?;
    let o = m.origin;
    let arr = |v: Option<&Value>| v.and_then(|v| serde_json::from_value::<[f64; 3]>(v.clone()).ok());
    if let Some(b) = st.get("box") {
        let (lo, hi) = (arr(b.get("min")).ok_or("the stock box has no `min`")?, arr(b.get("max")).ok_or("the stock box has no `max`")?);
        return Ok(([lo[0] + o.x, lo[1] + o.y, lo[2] + o.z], [hi[0] + o.x, hi[1] + o.y, hi[2] + o.z]));
    }
    if let Some(c) = st.get("cylinder") {
        let g = |k: &str| c.get(k).and_then(Value::as_f64).ok_or(format!("the bar stock has no `{k}`"));
        let (r, x0, x1) = (g("radius")?, g("x_min")?, g("x_max")?);
        return Ok(([x0 + o.x, o.y - r, o.z - r], [x1 + o.x, o.y + r, o.z + r]));
    }
    if let Some(a) = st.get("around_part") {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for b in &m.part {
            let Some(mb) = m.st.bodies.get(*b) else { continue };
            let bb = mb.body.exact_bounds().map_err(|e| e.to_string())?;
            for (k, (l, h)) in [(bb.min.x, bb.max.x), (bb.min.y, bb.max.y), (bb.min.z, bb.max.z)].into_iter().enumerate() {
                lo[k] = lo[k].min(l);
                hi[k] = hi[k].max(h);
            }
        }
        if !lo.iter().chain(&hi).all(|x| x.is_finite()) {
            return Err("the part has no extent".into());
        }
        let margin = arr(a.get("margin")).unwrap_or([0.0; 3]);
        let top = a.get("top").and_then(Value::as_f64).unwrap_or(margin[2]);
        let bottom = a.get("bottom").and_then(Value::as_f64).unwrap_or(margin[2]);
        return Ok(([lo[0] - margin[0], lo[1] - margin[1], lo[2] - bottom], [hi[0] + margin[0], hi[1] + margin[1], hi[2] + top]));
    }
    Err("the job's stock is not a box, a bar or around the part".into())
}

/// Where `from` (an `origin_from`) puts work zero now, design coordinates; how it was found and
/// what to check.
fn zero_point(m: &Model<'_>, from: &Value) -> std::result::Result<(Vec3, &'static str, Option<String>), String> {
    let of: OriginFrom = serde_json::from_value(from.clone()).map_err(|e| e.to_string())?;
    match of {
        OriginFrom::Pick(p) => {
            check_pick(&p)?;
            let h = find(&m.st, &m.part, &p, m.tol)?;
            Ok((point_of(m, &h)?, h.by, h.note))
        }
        OriginFrom::Stock(g) => {
            check_grid(&g)?;
            let (lo, hi) = stock_box(m)?;
            let at = |p: GridPlace, k: usize| match p {
                GridPlace::Low | GridPlace::Bottom => lo[k],
                GridPlace::High | GridPlace::Top => hi[k],
                GridPlace::Middle => (lo[k] + hi[k]) * 0.5,
            };
            Ok((Vec3::new(at(g.x, 0), at(g.y, 1), at(g.z, 2)), "stock", None))
        }
        OriginFrom::Axis { axis, x_end } => {
            check_pick(&axis)?;
            let h = find(&m.st, &m.part, &axis, m.tol)?;
            let mb = m.st.bodies.get(h.body).ok_or("its body is gone")?;
            let faces = BodyFaces::new(&mb.name, &mb.body);
            let (o, dir) = match h.face.and_then(|f| faces.surface(f)) {
                Some(FaceSurface::Cylinder { origin, axis, .. }) => (origin, axis.vec()),
                _ => match h.edge.and_then(|e| faces.edge_circle(e)) {
                    Some(c) => (c.center, c.axis.vec()),
                    None => return Err("the A axis is the axis of a cylinder face or a circular edge".into()),
                },
            };
            if dir.x.abs() < 1.0 - 1e-6 {
                return Err("the picked cylinder's axis is not along X, the A axis".into());
            }
            let x = match x_end {
                XEnd::Stock(s) => {
                    let (lo, hi) = stock_box(m)?;
                    match s.as_str() {
                        "stock_low" => lo[0],
                        "stock_high" => hi[0],
                        other => return Err(format!("`x_end` is stock_low, stock_high or a pick, not `{other}`")),
                    }
                }
                XEnd::Pick(p) => {
                    check_pick(&p)?;
                    let e = find(&m.st, &m.part, &p, m.tol)?;
                    point_of(m, &e)?.x
                }
            };
            let t = (x - o.x) / dir.x;
            let on = o + dir * t;
            Ok((Vec3::new(x, on.y, on.z), h.by, h.note))
        }
    }
}

/// The point a work-zero pick means: a circle's centre, the end of an edge nearest the pick (a
/// vertex), or the point on a face (on its exact surface).
fn point_of(m: &Model<'_>, h: &Hit) -> std::result::Result<Vec3, String> {
    let mb = m.st.bodies.get(h.body).ok_or("its body is gone")?;
    let faces = BodyFaces::new(&mb.name, &mb.body);
    if let Some(e) = h.edge {
        if let Some(c) = faces.edge_circle(e) {
            return Ok(c.center);
        }
        let ends = mb.body.record().edges.get(e).map(|ed| ed.vertices).ok_or("the edge is gone")?;
        let pts: Vec<Vec3> = ends.iter().filter_map(|v| mb.body.record().vertices.get(*v as usize)).map(|v| v3(v.point)).collect();
        return pts.into_iter().min_by(|a, b| a.dist(h.point).total_cmp(&b.dist(h.point))).ok_or_else(|| "the edge has no ends".into());
    }
    let f = h.face.ok_or("the pick is not a face or an edge")?;
    match faces.surface(f) {
        Some(FaceSurface::Cylinder { origin, axis, .. }) => {
            // A round face: on its axis, level with the pick.
            let d = h.point - origin;
            Ok(origin + axis.vec() * d.dot(axis.vec()))
        }
        Some(s) => s.nearest(h.point).ok_or_else(|| "no point on the face".into()),
        None => Ok(h.point),
    }
}

/// The A angle of the side picks `picks` are on, in the open job (none: on top, or not found).
pub(crate) fn picks_side(cmd: &str, s: &Session, picks: &[Pick]) -> Option<f64> {
    let (job, base) = open_loaded(cmd, s).ok()?;
    let m = model_of(cmd, s, &job, base.as_deref()).ok()?;
    let set = m.set()?;
    let found = find_all(&m, None, picks, "drill");
    let (_, h, feature) = found.hits.first()?;
    let hole = feature.as_deref().filter(|id| kind_of(id) == "hole" && set.reply.holes.iter().any(|x| x.id == *id && x.depth > m.tol));
    let wall = || {
        let mb = m.st.bodies.get(h.body)?;
        let faces = BodyFaces::new(&mb.name, &mb.body);
        let f = faces.round_wall(h.face?)?;
        let FaceSurface::Cylinder { axis, .. } = faces.surface(f)? else { return None };
        let (k, _) = wall_side(&m, set, &faces, f, axis.vec())?;
        set.setups.get(k).map(|su| su.a_deg)
    };
    let a = match (hole, feature) {
        (Some(id), _) => a_of(set, id),
        (None, Some(id)) if kind_of(id) != "surface" => a_of(set, id),
        _ => wall()?,
    };
    (a.abs() > 1e-9).then_some(a)
}

/// The job's model for picks: its machine and tools as loaded.
pub(crate) fn model_of<'a>(cmd: &'a str, s: &'a Session, job: &Value, base: Option<&std::path::Path>) -> Result<Model<'a>> {
    let probe_value = placeholders(job).map_err(|e| bad(cmd, e))?;
    let probe: CamJob = super::parse(cmd, &probe_value, "")?;
    let machine = super::machine_of(cmd, &probe)?;
    let tools = super::tools_of(cmd, &probe, base).unwrap_or_default();
    let mut m = Model::new(cmd, s, &probe, machine, tools)?;
    m.stock = job.get("stock").cloned();
    Ok(m)
}

/// The open job as loaded (its profile merged), and its folder.
fn open_loaded(cmd: &str, s: &Session) -> Result<(Value, Option<std::path::PathBuf>)> {
    let open = s.cam_job.as_deref().ok_or_else(|| bad(cmd, super::job_edit::NO_JOB))?;
    let base = open.folder();
    let merged = super::agent::merged_value(cmd, open.value.clone(), base.as_deref())?;
    Ok((merged, base))
}

fn cam_picks_from_selection(s: &Session) -> Vec<Pick> {
    let Ok(st) = s.world_state() else { return Vec::new() };
    s.selection
        .iter()
        .filter_map(|sel| match sel {
            crate::Sel::Face { body, index, point } => {
                st.bodies.iter().position(|b| &b.name == body).and_then(|b| pick_of(&st, b, Some(*index), None, Some(*point)))
            }
            crate::Sel::Edge { body, index, point } => {
                st.bodies.iter().position(|b| &b.name == body).and_then(|b| pick_of(&st, b, None, Some(*index), Some(*point)))
            }
            _ => None,
        })
        .take(MAX_PICKS)
        .collect()
}

/// The picks of a call: given, or the design's selection.
pub(crate) fn picks_param(cmd: &str, s: &Session, picks: Option<&Vec<Pick>>, from_selection: bool) -> Result<Vec<Pick>> {
    let picks = match (picks, from_selection) {
        (Some(_), true) => return Err(bad(cmd, "give `picks` or `from_selection`, not both")),
        (Some(p), false) => p.clone(),
        (None, true) => cam_picks_from_selection(s),
        (None, false) => return Err(bad(cmd, "give `picks`, or `from_selection: true`")),
    };
    if picks.is_empty() {
        return Err(bad(cmd, "nothing is picked: select faces or edges of the part"));
    }
    if picks.len() > MAX_PICKS {
        return Err(bad(cmd, format!("at most {MAX_PICKS} picks")));
    }
    for (k, p) in picks.iter().enumerate() {
        check_pick(p).map_err(|e| bad(cmd, format!("`picks[{k}]`: {e}")))?;
    }
    Ok(picks)
}

/// A feature in words.
fn reads(set: &FeatureSet, id: &str) -> String {
    let r = &set.reply;
    let side = |a: Option<f64>| a.filter(|a| a.abs() > 1e-9).map_or(String::new(), |a| format!(", on the A{} side", num(a)));
    if let Some(p) = r.pockets.iter().find(|p| p.id == id) {
        let size = format!("{} x {}", num(p.bounds[1][0] - p.bounds[0][0]), num(p.bounds[1][1] - p.bounds[0][1]));
        let corners = if p.min_corner_radius > 0.0 { format!(", corners R{}", num(p.min_corner_radius)) } else { ", sharp corners".into() };
        let islands = match p.islands.len() {
            0 => String::new(),
            1 => ", 1 island".into(),
            n => format!(", {n} islands"),
        };
        return format!("pocket {size} mm, {} deep{corners}{islands}{}", num(p.depth), side(p.a_deg));
    }
    if let Some(h) = r.holes.iter().find(|h| h.id == id) {
        let group = r.hole_groups.iter().find(|g| g.ids.iter().any(|x| x == id)).map_or(1, |g| g.count);
        let depth = if h.through { format!("through {}", num(h.depth)) } else { format!("{} deep", num(h.depth)) };
        let more = if group > 1 { format!(" (one of {group} alike)") } else { String::new() };
        return format!("hole {} mm, {depth}{more}{}", num(h.diameter), side(h.a_deg));
    }
    if let Some(f) = r.faces.iter().find(|f| f.id == id) {
        let size = format!("{} x {}", num(f.bounds[1][0] - f.bounds[0][0]), num(f.bounds[1][1] - f.bounds[0][1]));
        let role = match f.role.as_str() {
            "top" => "top face",
            "step" => "step",
            "plateau" => "raised face",
            "pocket_floor" => "pocket floor",
            "hole_floor" => "hole floor",
            "island_top" => "island top",
            _ => "flat face",
        };
        return format!("{role} {size} mm at Z {}{}", num(f.z), side(f.a_deg));
    }
    if let Some(p) = r.profiles.iter().find(|p| p.id == id) {
        let size = format!("{} x {}", num(p.bounds[1][0] - p.bounds[0][0]), num(p.bounds[1][1] - p.bounds[0][1]));
        return format!("outline {size} mm, Z {} to {}{}", num(p.top_z), num(p.bottom_z), side(p.a_deg));
    }
    if let Some(x) = r.fillets.iter().find(|x| x.id == id) {
        return format!("round{}{}", x.radius.map_or(String::new(), |r| format!(" R{}", num(r))), side(x.a_deg));
    }
    if let Some(x) = r.chamfers.iter().find(|x| x.id == id) {
        return format!("chamfer {} mm deep{}", num(x.top_z - x.bottom_z), side(x.a_deg));
    }
    if let Some(x) = r.surfaces.iter().find(|x| x.id == id) {
        return format!("curved area ({}), Z {} to {}{}", x.kinds.join(", "), num(x.bottom_z), num(x.top_z), side(x.a_deg));
    }
    if let Some(x) = r.turned.iter().find(|x| x.id == id) {
        return format!("turned diameter {} mm, X {} to {}", num(x.diameter), num(x.x_min), num(x.x_max));
    }
    id.to_string()
}

/// Whether operation type `ty` can cut with tool `t` at all.
pub(crate) fn takes_kind(ty: &str, t: &Tool) -> std::result::Result<(), String> {
    use ToolKind::*;
    let ok = match ty {
        "drill" => matches!(t.kind, Drill | FlatEndmill),
        "chamfer" => matches!(t.kind, Chamfer),
        "thread_mill" => matches!(t.kind, ThreadMill),
        "face" => matches!(t.kind, FaceMill | FlatEndmill | BullNose | BallEndmill),
        "contour" | "pocket" | "adaptive" | "bore" => matches!(t.kind, FlatEndmill | BullNose | BallEndmill | FaceMill),
        "roughing" | "rotary_roughing" | "adaptive_roughing" => matches!(t.kind, FlatEndmill | BullNose | BallEndmill),
        "parallel" | "waterline" | "scallop" | "steep_shallow" | "pencil" | "rest_finish" => {
            matches!(t.kind, FlatEndmill | BullNose | BallEndmill | FaceMill | Chamfer)
        }
        _ => matches!(t.kind, FlatEndmill | BullNose | BallEndmill),
    };
    if ok { Ok(()) } else { Err(format!("a {} does not cut {}", t.kind.label(), super::job_edit::type_label(ty))) }
}

/// Why tool `t` does not fit feature `id` for type `ty` (none: it fits).
pub(crate) fn misfit(set: Option<&FeatureSet>, ids: &[String], ty: &str, t: &Tool) -> Option<String> {
    if let Err(e) = takes_kind(ty, t) {
        return Some(e);
    }
    let set = set?;
    let r = &set.reply;
    for id in ids {
        if let Some(h) = r.holes.iter().find(|h| &h.id == id) {
            match ty {
                "drill" if t.kind == ToolKind::Drill && (t.diameter - h.diameter).abs() > 0.01 => {
                    return Some(format!("drills {} mm, the hole is {} mm", num(t.diameter), num(h.diameter)));
                }
                "drill" if t.kind == ToolKind::FlatEndmill && t.diameter > h.diameter + 1e-9 => {
                    return Some(format!("{} mm is wider than the {} mm hole", num(t.diameter), num(h.diameter)));
                }
                "bore" | "pocket" | "adaptive" | "contour" if t.diameter >= h.diameter - 1e-9 => {
                    return Some(format!("{} mm is as wide as the {} mm hole or wider", num(t.diameter), num(h.diameter)));
                }
                _ => {}
            }
            if t.flute_length + 1e-9 < h.depth && ty != "chamfer" && ty != "thread_mill" {
                return Some(format!("its flutes ({} mm) are shorter than the {} mm deep hole", num(t.flute_length), num(h.depth)));
            }
        }
        if let Some(p) = r.pockets.iter().find(|p| &p.id == id)
            && matches!(ty, "pocket" | "adaptive" | "contour")
        {
            let w = (p.bounds[1][0] - p.bounds[0][0]).min(p.bounds[1][1] - p.bounds[0][1]);
            if t.diameter >= w - 1e-9 {
                return Some(format!("{} mm is wider than the {} mm pocket", num(t.diameter), num(w)));
            }
            if p.min_corner_radius > 0.0 && t.radius() > p.min_corner_radius + 1e-9 {
                return Some(format!("wider than the pocket's R{} corners allow", num(p.min_corner_radius)));
            }
            if t.flute_length + 1e-9 < p.depth {
                return Some(format!("its flutes ({} mm) are shorter than the {} mm deep pocket", num(t.flute_length), num(p.depth)));
            }
        }
        if let Some(p) = r.profiles.iter().find(|p| &p.id == id)
            && ty == "contour"
        {
            if let Some(n) = p.min_concave_radius
                && t.radius() > n + 1e-9
            {
                return Some(format!("wider than the outline's R{} notches allow", num(n)));
            }
            if t.flute_length + 1e-9 < p.depth {
                return Some(format!("its flutes ({} mm) are shorter than the {} mm deep outline", num(t.flute_length), num(p.depth)));
            }
        }
    }
    None
}

/// The best tool of `tools` for type `ty` on features `ids`, and why.
pub(crate) fn best_tool(set: Option<&FeatureSet>, ids: &[String], ty: &str, tools: &[Tool]) -> Option<(u32, String)> {
    let mut fits: Vec<&Tool> = tools.iter().filter(|t| misfit(set, ids, ty, t).is_none()).collect();
    // Drills by number of an exact size; the rest: the largest that fits.
    fits.sort_by(|a, b| {
        let drill_first = |t: &Tool| u8::from(!(ty == "drill" && t.kind == ToolKind::Drill));
        drill_first(a).cmp(&drill_first(b)).then(b.diameter.total_cmp(&a.diameter)).then(a.number.cmp(&b.number))
    });
    let t = fits.first()?;
    Some((t.number, format!("T{} {} mm {}: the largest that fits", t.number, num(t.diameter), t.kind.label())))
}

/// The operations that suit features `ids`, best first.
fn suits(m: &Model<'_>, set: Option<&FeatureSet>, ids: &[String], kinds: &[&str], walls: &[f64]) -> Vec<Suit> {
    let r = set.map(|s| &s.reply);
    let four = m.machine.rotaries.len() == 1;
    let mut out: Vec<(&str, String)> = Vec::new();
    let has = |k: &str| ids.iter().any(|id| kind_of(id) == k);
    // Round walls that are no recognised hole (one opening on a curved face): drilled or milled
    // along their axis, the drill of their size first.
    let mut wall_tools: Vec<(&str, Option<u32>)> = Vec::new();
    if !has("hole") && !has("turned") && !walls.is_empty() {
        let drill = m.tools.iter().filter(|t| t.kind == ToolKind::Drill).find(|t| walls.iter().all(|d| (t.diameter - d).abs() <= 0.01));
        let mill = m
            .tools
            .iter()
            .filter(|t| matches!(t.kind, ToolKind::FlatEndmill) && walls.iter().all(|d| t.diameter < d - 1e-9))
            .max_by(|a, b| a.diameter.total_cmp(&b.diameter));
        if let Some(d) = drill {
            out.push(("drill", format!("a round wall {} mm across, drilled along its axis", num(d.diameter))));
            wall_tools.push(("drill", Some(d.number)));
        }
        out.push(("bore", "a round wall, milled round along its axis".into()));
        wall_tools.push(("bore", mill.map(|t| t.number)));
    }
    if has("hole") {
        let drillable = r.is_some_and(|r| {
            ids.iter()
                .filter_map(|id| r.holes.iter().find(|h| &h.id == id))
                .all(|h| m.tools.iter().any(|t| t.kind == ToolKind::Drill && (t.diameter - h.diameter).abs() <= 0.01))
        });
        if drillable {
            out.push(("drill", "a drill of its size is in the tools".into()));
        }
        out.push(("bore", "a round hole, milled round with an end mill".into()));
        if !drillable {
            out.push(("drill", "no drill of its size is in the tools".into()));
        }
        if m.tools.iter().any(|t| t.kind == ToolKind::ThreadMill) {
            out.push(("thread_mill", "a thread mill is in the tools".into()));
        }
        out.push(("chamfer", "breaks its top edge".into()));
    }
    if has("pocket") {
        out.push(("pocket", "clears the pocket round its islands".into()));
        out.push(("adaptive", "clears it with a light, even load".into()));
        out.push(("contour", "finishes its walls".into()));
        out.push(("chamfer", "breaks its rim".into()));
    }
    if has("face") {
        let top = r.is_some_and(|r| ids.iter().any(|id| r.faces.iter().any(|f| &f.id == id && f.role == "top")));
        if top {
            out.push(("face", "the part's top face".into()));
        } else {
            out.push(("pocket", "clears down to this face".into()));
            out.push(("adaptive", "clears down to it with a light, even load".into()));
        }
    }
    if has("profile") {
        out.push(("contour", "cuts round the outline".into()));
        out.push(("chamfer", "breaks its edge".into()));
    }
    if has("chamfer") {
        out.push(("chamfer", "machines the chamfer round its rim with a chamfer mill".into()));
    }
    if has("fillet") || has("surface") || has("chamfer") {
        out.push(("parallel", "finishes the curved area in passes side by side".into()));
        out.push(("scallop", "finishes it with even spacing over the surface".into()));
        out.push(("waterline", "finishes steep walls level by level".into()));
        out.push(("pencil", "cleans the inside corners".into()));
    }
    if has("turned") && four {
        out.push(("rotary_finish", "finishes the turned diameter as A turns".into()));
        out.push(("rotary_roughing", "roughs the bar down to it".into()));
    }
    if ids.is_empty() && kinds.contains(&"cylinder") && four {
        out.push(("wrap", "a path drawn on this cylinder".into()));
    }
    if ids.is_empty() && kinds.contains(&"sketch") {
        out.push(("pocket", "clears the sketch's profile".into()));
        out.push(("contour", "cuts round it".into()));
        out.push(("adaptive", "clears it with a light, even load".into()));
    }
    let mut seen: Vec<&str> = Vec::new();
    out.into_iter()
        .filter(|(ty, _)| {
            let new = !seen.contains(ty);
            seen.push(ty);
            new
        })
        .map(|(ty, why)| {
            let tool = match wall_tools.iter().find(|(t, _)| *t == ty) {
                Some((_, n)) => n
                    .and_then(|n| m.tools.iter().find(|t| t.number == n))
                    .map(|t| (t.number, format!("T{} {} mm {}", t.number, num(t.diameter), t.kind.label()))),
                None => best_tool(set, ids, ty, &m.tools),
            };
            Suit {
                kind: ty.to_string(),
                label: crate::cmd::cam::job_edit::type_label(ty).to_string(),
                tool: tool.as_ref().map(|(n, _)| *n),
                why: match tool {
                    Some((_, w)) => format!("{why}; {w}"),
                    None => format!("{why}; no tool of the job fits it"),
                },
            }
        })
        .collect()
}

fn resolve_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.picks.resolve";
    let params: CamPicksResolveParams = parse(cmd, p, "")?;
    let picks = picks_param(cmd, s, params.picks.as_ref(), params.from_selection.unwrap_or(false))?;
    let ty = params.for_type.clone().unwrap_or_default();
    if !ty.is_empty() && !crate::cam_job_state::OP_TYPES.iter().any(|(t, _, _)| *t == ty) {
        return Err(bad(cmd, format!("`for` is an operation type (face, pocket, drill, …), not `{}`", ty.chars().take(40).collect::<String>())));
    }
    let (job, base) = match s.cam_job.as_deref() {
        Some(_) => open_loaded(cmd, s)?,
        None => (json!({"machine": "3axis", "operations": []}), None),
    };
    let m = model_of(cmd, s, &job, base.as_deref())?;
    let found = find_all(&m, None, &picks, if ty.is_empty() { "pocket" } else { &ty });
    let set = m.set();
    let mut ids: Vec<String> = Vec::new();
    let mut kinds: Vec<&str> = Vec::new();
    let mut out: Vec<PickRead> = Vec::new();
    let mut hits = found.hits.iter();
    for (k, p) in picks.iter().enumerate() {
        let rep = found.report.iter().find(|r| r.pick == k);
        if p.sketch.is_some() {
            kinds.push("sketch");
            out.push(PickRead {
                pick: p.clone(),
                by: "name".into(),
                kind: "sketch".into(),
                reads: format!("sketch {}", p.sketch.as_deref().unwrap_or_default()),
                ..PickRead::default()
            });
            continue;
        }
        let Some(rep) = rep else { continue };
        if rep.by == "lost" {
            out.push(PickRead {
                pick: p.clone(),
                by: "lost".into(),
                kind: String::new(),
                reads: "not found".into(),
                note: rep.note.clone(),
                ..PickRead::default()
            });
            continue;
        }
        let Some((_, h, feature)) = hits.next() else { continue };
        let kind = match (h.face, h.edge) {
            (Some(f), _) => m.st.bodies.get(h.body).map_or("other", |mb| BodyFaces::new(&mb.name, &mb.body).kind(f)),
            (None, Some(e)) => {
                if m.st.bodies.get(h.body).and_then(|mb| BodyFaces::new(&mb.name, &mb.body).edge_circle(e)).is_some() {
                    "circle"
                } else {
                    "edge"
                }
            }
            _ => "other",
        };
        kinds.push(kind);
        let all = set.map(|set| features_of_hit(set, &m.st, h, m.origin, m.tol)).unwrap_or_default();
        for id in &all {
            if !ids.contains(id) && (feature.as_ref() == Some(id) || ids.is_empty() || kind_of(id) != "face") {
                ids.push(id.clone());
            }
        }
        let words = feature.as_deref().and_then(|id| set.map(|set| reads(set, id))).unwrap_or_else(|| format!("a {kind} face"));
        out.push(PickRead {
            pick: p.clone(),
            by: rep.by.clone(),
            face: h.face,
            edge: h.edge,
            kind: kind.to_string(),
            feature: feature.clone(),
            reads: words,
            note: rep.note.clone(),
        });
    }
    // The features the picks are on, without a top face also listed for a pocket's rim.
    if ids.iter().any(|id| kind_of(id) != "face") {
        let keep: Vec<String> = ids.iter().filter(|id| kind_of(id) != "face" || picks.len() == 1).cloned().collect();
        if !keep.is_empty() {
            ids = keep;
        }
    }
    // The diameters of the picked round walls.
    let walls: Vec<f64> = found
        .hits
        .iter()
        .filter_map(|(_, h, _)| {
            let mb = m.st.bodies.get(h.body)?;
            let faces = BodyFaces::new(&mb.name, &mb.body);
            match faces.surface(faces.round_wall(h.face?)?)? {
                FaceSurface::Cylinder { radius, .. } => Some(radius * 2.0),
                _ => None,
            }
        })
        .collect();
    let mut suits = suits(&m, set, &ids, &kinds, &walls);
    if !ty.is_empty() {
        suits.retain(|s| s.kind == ty);
    }
    serde_json::to_value(CamPicksResolveReply { picks: out, suits }).map_err(|e| EngineError::Other(e.to_string()))
}

fn zero_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.setup.zero";
    let params: CamSetupZeroParams = parse(cmd, p, "")?;
    let given = [params.at.is_some(), params.pick.is_some(), params.stock.is_some(), params.axis.is_some(), params.refresh.unwrap_or(false)];
    if given.iter().filter(|g| **g).count() != 1 {
        return Err(bad(cmd, "give one of `at`, `pick`, `stock`, `axis` or `refresh: true`"));
    }
    if params.x_end.is_some() && params.axis.is_none() {
        return Err(bad(cmd, "`x_end` goes with `axis`"));
    }
    let (job, base) = open_loaded(cmd, s)?;
    let stored = s.cam_job.as_deref().map(|j| j.value.clone()).unwrap_or(Value::Null);
    let old: [f64; 3] = stored.get("origin").and_then(|o| serde_json::from_value(o.clone()).ok()).unwrap_or([0.0; 3]);
    let (new, from) = {
        let m = model_of(cmd, s, &job, base.as_deref())?;
        let from: Option<Value> = if let Some(at) = params.at {
            if !finite3(&at) {
                return Err(bad(cmd, "`at` must be a finite point [x, y, z] within 1000 m"));
            }
            None
        } else if let Some(pk) = &params.pick {
            Some(json!({"pick": pk}))
        } else if let Some(g) = &params.stock {
            Some(json!({"stock": g}))
        } else if let Some(axis) = &params.axis {
            let x_end = params.x_end.clone().unwrap_or(XEnd::Stock("stock_low".into()));
            Some(json!({"axis": {"axis": axis, "x_end": x_end}}))
        } else {
            Some(
                stored
                    .get("origin_from")
                    .cloned()
                    .ok_or_else(|| bad(cmd, "the job's work zero was set by numbers: there is nothing to set it again from"))?,
            )
        };
        let new = match (&from, params.at) {
            (_, Some(at)) => v3(at),
            (Some(f), None) => {
                if let Err(e) = check_origin_from(f) {
                    return Err(bad(cmd, e));
                }
                zero_point(&m, f).map_err(|e| bad(cmd, e))?.0
            }
            (None, None) => return Err(bad(cmd, "nothing to set work zero from")),
        };
        (new, from)
    };
    let new = [super::round(new.x, 1e6), super::round(new.y, 1e6), super::round(new.z, 1e6)];
    let moved = [new[0] - old[0], new[1] - old[1], new[2] - old[2]];
    let mut next = stored.clone();
    let mut notes = Vec::new();
    if let Value::Object(map) = &mut next {
        map.insert("origin".into(), json!(new));
        match (&from, params.keep_ref.unwrap_or(true)) {
            (Some(f), true) => {
                map.insert("origin_from".into(), f.clone());
            }
            _ => {
                map.remove("origin_from");
            }
        }
        // A box or bar stock is in work coordinates: keep it where it was in the design.
        if let Some(Value::Object(st)) = map.get_mut("stock") {
            let shift = |v: &mut Value, k: usize| {
                if let Some(x) = v.as_f64() {
                    *v = json!(super::round(x - moved[k], 1e6));
                }
            };
            if let Some(Value::Object(b)) = st.get_mut("box") {
                for corner in ["min", "max"] {
                    if let Some(Value::Array(c)) = b.get_mut(corner) {
                        for (k, v) in c.iter_mut().enumerate().take(3) {
                            shift(v, k);
                        }
                    }
                }
                notes.push("the stock box is kept where it was: its corners moved with work zero".into());
            }
            if let Some(Value::Object(c)) = st.get_mut("cylinder") {
                for k in ["x_min", "x_max"] {
                    if let Some(v) = c.get_mut(k) {
                        shift(v, 0);
                    }
                }
                if moved[1].abs() > 1e-9 || moved[2].abs() > 1e-9 {
                    notes.push(
                        "the bar is about the A axis through work zero: it moved across with it; work zero is usually on the bar's axis".into(),
                    );
                }
            }
        }
    }
    let label = if params.refresh.unwrap_or(false) { "Move work zero to its pick again" } else { "Move work zero" };
    super::job_edit::commit(s, cmd, label.into(), next)?;
    let reply = CamSetupZeroReply {
        origin: new,
        origin_from: from.filter(|_| params.keep_ref.unwrap_or(true)),
        moved,
        notes,
        summary: super::job_edit::summary(s),
    };
    serde_json::to_value(reply).map_err(|e| EngineError::Other(e.to_string()))
}

#[cfg(test)]
#[path = "cam_picks_tests.rs"]
mod tests;
