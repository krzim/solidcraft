//! How the work is held on the table (the format `solidcraft.workholding`, version 1; see
//! docs/cam-jobs.md): a vise with its jaws and parallels, strap clamps on the stock, the table
//! itself, or a rotary unit for a 4th axis. [`build`] places the job's stock in it, finds where
//! work zero is on the table, turns everything into boxes in work coordinates for the collision
//! checks ([`crate::machine_model::clearance`]), and reports what is wrong with the setup itself
//! (the stock too wide for the vise, gripped too shallow, a clamp off the stock, ...).
//!
//! Ported from the author's own MIT-licensed G-code simulator (MIT, same author): the vise's placement (quarter turns, the
//! fixed jaw's face, the parallels along the jaws), clamps on the stock's top, parallels under
//! its long edges, the grip box less the stock, and the setup findings.

use serde::{Deserialize, Serialize};
use solidcore_record::Vec3;

use crate::findings::{FindingEvent, Severity};
use crate::machine_model::{Hole, Obstacle, ObstacleKind, Table, TablePlace, is_name};
use crate::sim::StockShape;
use crate::{Result, invalid};

/// The format id of a workholding file.
pub const FORMAT: &str = "solidcraft.workholding";
/// Most clamps.
pub const MAX_CLAMPS: usize = 100;
const MAX_LEN: f64 = 1.0e5;

/// How the work is held.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Holding {
    Vise,
    Clamps,
    /// The program machines the table itself.
    Table,
    /// A rotary unit (4th axis).
    Rotary,
}

/// A box (min and max corners).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Box3 {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

/// A precision vise: a base, a fixed jaw at one end and a moving jaw. Its own frame: `u` along
/// the jaws, `v` along the clamping axis towards the fixed jaw, `z` up from its bottom.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ViseModel {
    #[serde(default)]
    pub name: String,
    /// Body length along the clamping axis.
    pub length: f64,
    /// Body and jaw width.
    pub width: f64,
    /// The jaws' top above the vise's bottom.
    pub height: f64,
    /// What the stock (or the parallels) sits on, above the vise's bottom.
    pub floor: f64,
    pub fixed_jaw: f64,
    pub moving_jaw: f64,
    pub max_opening: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Vise {
    pub model: ViseModel,
    /// The centre of its footprint, table frame.
    pub center: [f64; 2],
    /// Quarter turns counter-clockwise seen from above; 0: the jaws along X, the fixed jaw at
    /// the back (+Y).
    #[serde(default)]
    pub quarter_turns: u32,
    /// The stock's centre along the jaws from the vise's centre.
    #[serde(default)]
    pub offset: f64,
    /// A box (work coordinates) the jaws close on instead of the stock: a fixture or sub-plate.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub grip: Option<Box3>,
}

/// A pair of parallels under the stock: in the vise one along each jaw, under clamps one under
/// each long (X) edge, centred on the stock unless moved.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Parallels {
    /// 0: none, the stock sits on the vise's floor or the table.
    pub height: f64,
    pub thickness: f64,
    pub length: f64,
    /// How far each bar sits in from its jaw (or the stock's edge).
    #[serde(default)]
    pub inset: f64,
    /// Along the bars, from centred on the stock.
    #[serde(default)]
    pub offset: f64,
}

/// A strap clamp, a box resting on the stock's top, table frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Clamp {
    pub center: [f64; 2],
    pub size: [f64; 2],
    /// Above the stock's top, bolt included.
    pub height: f64,
}

/// A rotary unit's chuck: a cylinder about the A axis ending at its face.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Chuck {
    pub diameter: f64,
    pub length: f64,
    /// Where its face is along X, table frame; the chuck reaches back from it (towards -X).
    pub face_x: f64,
}

/// A rotary unit (4th axis about X).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RotaryUnit {
    /// The A axis line: its Y and Z in the table frame.
    pub axis: [f64; 2],
    pub headstock: Box3,
    pub chuck: Chuck,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tailstock: Option<Box3>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plate: Option<Box3>,
    /// Work zero along X, table frame; left out, the stock's low end sits at the chuck's face.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zero_x: Option<f64>,
}

/// A workholding file (`solidcraft.workholding`, version 1).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Workholding {
    pub format: String,
    pub version: u32,
    #[serde(default)]
    pub name: String,
    pub holding: Holding,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vise: Option<Vise>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub parallels: Option<Parallels>,
    #[serde(default)]
    pub clamps: Vec<Clamp>,
    /// With clamps: the stock's front-left corner on the table (X, Y).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stock_at: Option<[f64; 2]>,
    /// Machining the table: work zero in the table frame.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub table_zero: Option<[f64; 3]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rotary: Option<RotaryUnit>,
    /// Machine coordinates of work zero, for the soft limits (not checked yet).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub work_zero_machine: Option<[f64; 3]>,
}

fn finite(v: &[f64]) -> bool {
    v.iter().all(|x| x.is_finite() && x.abs() <= MAX_LEN)
}

fn box_ok(b: &Box3) -> bool {
    finite(&b.min) && finite(&b.max) && b.min.iter().zip(&b.max).all(|(a, c)| a < c)
}

impl Workholding {
    /// Workholding read from JSON text and checked.
    pub fn from_json(text: &str) -> Result<Workholding> {
        let w: Workholding = serde_json::from_str(text).map_err(|e| crate::CamError::Invalid(format!("the workholding: {e}")))?;
        w.validate()?;
        Ok(w)
    }

    pub fn validate(&self) -> Result<()> {
        if self.format != FORMAT || self.version != 1 {
            return invalid(format!("a workholding file has `format: \"{FORMAT}\"` and `version: 1`"));
        }
        if !is_name(&self.name) {
            return invalid("the workholding's `name` must be at most 200 characters, without control characters");
        }
        if let Some(v) = &self.vise {
            let m = &v.model;
            let sizes = [m.length, m.width, m.height, m.floor, m.fixed_jaw, m.moving_jaw, m.max_opening];
            if !(finite(&sizes) && sizes.iter().all(|s| *s >= 0.0) && m.length > 0.0 && m.width > 0.0 && m.height > 0.0 && m.floor < m.height)
                || !is_name(&m.name)
            {
                return invalid("the vise's sizes must be finite and positive, its floor below its jaws' top");
            }
            if m.fixed_jaw + m.moving_jaw >= m.length {
                return invalid("the vise's jaws must be thinner together than its length");
            }
            if !finite(&[v.center[0], v.center[1], v.offset]) || v.grip.as_ref().is_some_and(|g| !box_ok(g)) {
                return invalid("the vise's `center`, `offset` and `grip` must be finite (a grip box min below max)");
            }
        }
        if let Some(p) = &self.parallels {
            let s = [p.height, p.thickness, p.length, p.inset, p.offset];
            if !(finite(&s) && p.height >= 0.0 && p.thickness >= 0.0 && p.length >= 0.0 && p.inset >= 0.0) {
                return invalid("the parallels' sizes must be finite, 0 or more");
            }
        }
        if self.clamps.len() > MAX_CLAMPS {
            return invalid(format!("at most {MAX_CLAMPS} clamps"));
        }
        for c in &self.clamps {
            if !(finite(&[c.center[0], c.center[1], c.size[0], c.size[1], c.height]) && c.size[0] > 0.0 && c.size[1] > 0.0 && c.height > 0.0) {
                return invalid("a clamp needs a finite centre and a size and height above 0");
            }
        }
        if self.stock_at.is_some_and(|s| !finite(&s))
            || self.table_zero.is_some_and(|z| !finite(&z))
            || self.work_zero_machine.is_some_and(|z| !finite(&z))
        {
            return invalid("`stock_at`, `table_zero` and `work_zero_machine` must be finite");
        }
        if let Some(r) = &self.rotary {
            let boxes_ok = box_ok(&r.headstock) && r.tailstock.as_ref().is_none_or(box_ok) && r.plate.as_ref().is_none_or(box_ok);
            let c = &r.chuck;
            if !(boxes_ok && finite(&[r.axis[0], r.axis[1], c.diameter, c.length, c.face_x]) && c.diameter > 0.0 && c.length > 0.0)
                || r.zero_x.is_some_and(|x| !finite(&[x]))
            {
                return invalid("the rotary unit's boxes must have min below max, its chuck a diameter and a length above 0, all finite");
            }
        }
        match self.holding {
            Holding::Vise if self.vise.is_none() => invalid("`holding: vise` needs the `vise`"),
            Holding::Clamps if self.stock_at.is_none() => invalid("`holding: clamps` needs `stock_at`, the stock's front-left corner on the table"),
            Holding::Table if self.table_zero.is_none() => invalid("`holding: table` needs `table_zero`, work zero on the table"),
            Holding::Rotary if self.rotary.is_none() => invalid("`holding: rotary` needs the `rotary` unit"),
            _ => Ok(()),
        }
    }
}

/// The workholding placed with the stock: everything in work coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct HoldScene {
    pub holding: Holding,
    /// Work zero in the table frame.
    pub work_zero_table: Vec3,
    /// The table: its top-front-left corner in work coordinates, its size.
    pub table: TablePlace,
    pub corner_radius: f64,
    /// The table's holes, centres in work coordinates (`x`, `y`).
    pub holes: Vec<Hole>,
    pub obstacles: Vec<Obstacle>,
    /// In the vise: how far the stock's top stands above the jaws.
    pub stock_above_jaws: Option<f64>,
    /// What is wrong with the setup itself.
    pub findings: Vec<FindingEvent>,
}

/// Local vise `(u, v)` to a table offset, after `k` quarter turns counter-clockwise.
fn turn(k: u32, u: f64, v: f64) -> (f64, f64) {
    match k % 4 {
        0 => (u, v),
        1 => (-v, u),
        2 => (-u, -v),
        _ => (v, -u),
    }
}

/// The table-frame box of a rectangle in the vise's frame.
fn vise_rect(c: [f64; 2], k: u32, (u0, u1): (f64, f64), (v0, v1): (f64, f64), (z0, z1): (f64, f64)) -> (Vec3, Vec3) {
    let (ax, ay) = turn(k, u0, v0);
    let (bx, by) = turn(k, u1, v1);
    (Vec3::new(c[0] + ax.min(bx), c[1] + ay.min(by), z0), Vec3::new(c[0] + ax.max(bx), c[1] + ay.max(by), z1))
}

/// `a` with `b` cut out, as up to six boxes.
fn box_minus(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> Vec<(Vec3, Vec3)> {
    let get = |v: Vec3, i: usize| match i {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    };
    let set = |mut v: Vec3, i: usize, x: f64| {
        match i {
            0 => v.x = x,
            1 => v.y = x,
            _ => v.z = x,
        }
        v
    };
    let (mut lo, mut hi) = a;
    if (0..3).any(|i| get(b.1, i) <= get(lo, i) || get(b.0, i) >= get(hi, i)) {
        return vec![a];
    }
    let mut out = Vec::new();
    for ax in 0..3 {
        if get(b.0, ax) > get(lo, ax) {
            out.push((lo, set(hi, ax, get(b.0, ax))));
            lo = set(lo, ax, get(b.0, ax));
        }
        if get(b.1, ax) < get(hi, ax) {
            out.push((set(lo, ax, get(b.1, ax)), hi));
            hi = set(hi, ax, get(b.1, ax));
        }
    }
    out
}

/// Each bar's inset and thickness across a stock `across` wide: both fit within it.
fn bar_across(par: &Parallels, across: f64) -> (f64, f64) {
    let half = across / 2.0;
    let inset = par.inset.clamp(0.0, half.max(0.0));
    (inset, par.thickness.max(0.0).min((half - inset).max(0.0)))
}

fn setup(code: &str, severity: Severity, message: String) -> FindingEvent {
    FindingEvent { code: code.into(), severity, at_move: None, line: 0, op: None, at: None, depth: 0.0, message }
}

/// Places `stock` (work coordinates) in `hold` on `table`; see the module documentation.
pub fn build(table: &Table, hold: &Workholding, stock: &StockShape) -> Result<HoldScene> {
    hold.validate()?;
    let (smin, smax) = match *stock {
        StockShape::Box { min, max } => (min, max),
        StockShape::Cylinder { x_min, x_max, radius } => (Vec3::new(x_min, -radius, -radius), Vec3::new(x_max, radius, radius)),
    };
    let size = smax - smin;
    let mut f: Vec<FindingEvent> = Vec::new();
    let mut obs: Vec<Obstacle> = Vec::new();
    let mut push = |name: &str, kind: ObstacleKind, (min, max): (Vec3, Vec3)| obs.push(Obstacle { name: name.into(), kind, min, max });
    let table_box = (Vec3::new(0.0, 0.0, -table.size[2]), Vec3::new(table.size[0], table.size[1], 0.0));
    let par = hold.parallels.clone().unwrap_or(Parallels { height: 0.0, thickness: 0.0, length: 0.0, inset: 0.0, offset: 0.0 });
    let mut stock_above_jaws = None;
    let on_table = |lo: Vec3, hi: Vec3| lo.x >= -1e-6 && lo.y >= -1e-6 && hi.x <= table.size[0] + 1e-6 && hi.y <= table.size[1] + 1e-6;
    let zero = match hold.holding {
        Holding::Table => {
            let z = hold.table_zero.unwrap_or([0.0; 3]);
            Vec3::new(z[0], z[1], z[2])
        }
        Holding::Vise => {
            let Some(v) = &hold.vise else { return invalid("`holding: vise` needs the `vise`") };
            let m = &v.model;
            let k = v.quarter_turns % 4;
            // The jaws close on the grip box when there is one, else on the stock.
            let held = v.grip.map_or((smin, smax), |g| (Vec3::new(g.min[0], g.min[1], g.min[2]), Vec3::new(g.max[0], g.max[1], g.max[2])));
            let hsize = held.1 - held.0;
            let what = if v.grip.is_some() { "fixture" } else { "stock" };
            let (su, sv) = if k % 2 == 0 { (hsize.x, hsize.y) } else { (hsize.y, hsize.x) };
            let (hl, hw) = (m.length / 2.0, m.width / 2.0);
            let face = hl - m.fixed_jaw;
            let bottom = m.floor + par.height.max(0.0);
            let top = bottom + hsize.z;
            stock_above_jaws = Some(top - m.height);
            let (u0, u1) = (v.offset - su / 2.0, v.offset + su / 2.0);
            let (sv0, sv1) = (face - sv, face);
            push("the table", ObstacleKind::Table, table_box);
            push("the vise's body", ObstacleKind::Vise, vise_rect(v.center, k, (-hw, hw), (-hl, hl), (0.0, m.floor)));
            push("the vise's fixed jaw", ObstacleKind::Vise, vise_rect(v.center, k, (-hw, hw), (face, hl), (0.0, m.height)));
            push("the vise's moving jaw", ObstacleKind::Vise, vise_rect(v.center, k, (-hw, hw), (sv0 - m.moving_jaw, sv0), (0.0, m.height)));
            if par.height > 1e-6 {
                let c = v.offset + par.offset;
                let (p0, p1) = (c - par.length / 2.0, c + par.length / 2.0);
                let (inset, t) = bar_across(&par, sv);
                if t > 1e-6 {
                    for (a, b) in [(sv1 - inset - t, sv1 - inset), (sv0 + inset, sv0 + inset + t)] {
                        push("the parallels", ObstacleKind::Parallels, vise_rect(v.center, k, (p0, p1), (a, b), (m.floor, bottom)));
                    }
                }
            }
            if sv > m.max_opening + 1e-6 {
                f.push(setup(
                    "vise-too-narrow",
                    Severity::Error,
                    format!("the {what} is {} mm across the jaws; this vise opens to {} mm", num(sv), num(m.max_opening)),
                ));
            }
            let grip = m.height - bottom;
            if grip <= 0.0 {
                f.push(setup("stock-not-gripped", Severity::Error, format!("the {what} sits entirely above the jaws: nothing holds it")));
            } else if grip < 3.0 {
                f.push(setup(
                    "shallow-grip",
                    Severity::Warning,
                    format!("the jaws grip only {} mm of the {what}: aim for 3 mm or more (lower parallels)", num(grip)),
                ));
            }
            let (lo, hi) = vise_rect(v.center, k, (-hw, hw), (-hl, hl), (0.0, 0.0));
            if !on_table(lo, hi) {
                f.push(setup("vise-overhangs-table", Severity::Info, "the vise overhangs the table".into()));
            }
            let (held_min, _) = vise_rect(v.center, k, (u0, u1), (sv0, sv1), (bottom, top));
            held_min - held.0
        }
        Holding::Clamps => {
            let at = hold.stock_at.unwrap_or([0.0, 0.0]);
            let h = par.height.max(0.0);
            let lo = Vec3::new(at[0], at[1], h);
            let hi = lo + size;
            push("the table", ObstacleKind::Table, table_box);
            if h > 1e-6 {
                let cx = (lo.x + hi.x) / 2.0 + par.offset;
                let (x0, x1) = (cx - par.length / 2.0, cx + par.length / 2.0);
                let (inset, t) = bar_across(&par, size.y);
                if t > 1e-6 {
                    for (a, b) in [(lo.y + inset, lo.y + inset + t), (hi.y - inset - t, hi.y - inset)] {
                        push("the parallels", ObstacleKind::Parallels, (Vec3::new(x0, a, 0.0), Vec3::new(x1, b, h)));
                    }
                }
            }
            for (n, c) in hold.clamps.iter().enumerate() {
                let cl = Vec3::new(c.center[0] - c.size[0] / 2.0, c.center[1] - c.size[1] / 2.0, hi.z);
                let ch = Vec3::new(c.center[0] + c.size[0] / 2.0, c.center[1] + c.size[1] / 2.0, hi.z + c.height);
                push(&format!("clamp {}", n + 1), ObstacleKind::Clamp, (cl, ch));
                if ch.x <= lo.x || cl.x >= hi.x || ch.y <= lo.y || cl.y >= hi.y {
                    f.push(setup("clamp-off-stock", Severity::Info, format!("clamp {} does not reach the stock", n + 1)));
                }
            }
            if hold.clamps.is_empty() {
                f.push(setup("no-clamps", Severity::Info, "no clamps are given: the stock is taken as held some other way".into()));
            }
            if !on_table(lo, hi) {
                f.push(setup("stock-off-table", Severity::Warning, "the stock hangs over the edge of the table".into()));
            }
            lo - smin
        }
        Holding::Rotary => {
            let Some(r) = &hold.rotary else { return invalid("`holding: rotary` needs the `rotary` unit") };
            if !matches!(stock, StockShape::Cylinder { .. }) {
                return invalid("a rotary unit holds a bar (a cylinder about X)");
            }
            let to = |b: &Box3| (Vec3::new(b.min[0], b.min[1], b.min[2]), Vec3::new(b.max[0], b.max[1], b.max[2]));
            push("the table", ObstacleKind::Table, table_box);
            push("the rotary unit's headstock", ObstacleKind::Rotary, to(&r.headstock));
            let cr = r.chuck.diameter * 0.5;
            push(
                "the rotary unit's chuck",
                ObstacleKind::Rotary,
                (
                    Vec3::new(r.chuck.face_x - r.chuck.length, r.axis[0] - cr, r.axis[1] - cr),
                    Vec3::new(r.chuck.face_x, r.axis[0] + cr, r.axis[1] + cr),
                ),
            );
            if let Some(t) = &r.tailstock {
                push("the rotary unit's tailstock", ObstacleKind::Rotary, to(t));
            }
            if let Some(p) = &r.plate {
                push("the rotary unit's plate", ObstacleKind::Rotary, to(p));
            }
            let zero_x = r.zero_x.unwrap_or(r.chuck.face_x - smin.x);
            Vec3::new(zero_x, r.axis[0], r.axis[1])
        }
    };
    // The grip box is an obstacle where the stock is not.
    if let (Holding::Vise, Some(g)) = (hold.holding, hold.vise.as_ref().and_then(|v| v.grip)) {
        let gb = (Vec3::new(g.min[0], g.min[1], g.min[2]) + zero, Vec3::new(g.max[0], g.max[1], g.max[2]) + zero);
        for b in box_minus(gb, (smin + zero, smax + zero)) {
            push("the fixture the vise grips", ObstacleKind::Grip, b);
        }
    }
    for o in &mut obs {
        o.min -= zero;
        o.max -= zero;
    }
    let holes = table.holes.iter().map(|h| Hole { x: h.x - zero.x, y: h.y - zero.y, ..h.clone() }).collect();
    Ok(HoldScene {
        holding: hold.holding,
        work_zero_table: zero,
        table: TablePlace { corner: Vec3::new(0.0, 0.0, 0.0) - zero, size: table.size },
        corner_radius: table.corner_radius,
        holes,
        obstacles: obs,
        stock_above_jaws,
        findings: f,
    })
}

fn num(v: f64) -> String {
    crate::gcode::number(v, 1)
}
