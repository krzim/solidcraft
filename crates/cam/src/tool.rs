//! Cutting tools.

use serde::{Deserialize, Serialize};

use crate::cutter::Cutter;
use crate::{Result, invalid};

/// Largest tool number a post processor is asked to write.
pub const MAX_TOOL_NUMBER: u32 = 9999;
/// Largest tool diameter (mm).
pub const MAX_TOOL_DIAMETER: f64 = 1000.0;
/// Largest surface speed (m/min).
pub const MAX_SURFACE_SPEED: f64 = 10_000.0;
/// Largest chip load (mm a tooth).
pub const MAX_CHIP_LOAD: f64 = 10.0;
/// Most cutting data entries on one tool.
pub const MAX_CUTTING_DATA: usize = 32;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolKind {
    FlatEndmill,
    BallEndmill,
    /// A flat end mill with rounded corners (a torus at the end): see [`Tool::corner_radius`].
    BullNose,
    Drill,
    /// A thread mill: teeth of a thread's form round a reduced core, single-point (one row of
    /// teeth, any pitch) or multi-form (rows [`Tool::thread_pitch`] apart). It cuts by helical
    /// interpolation round a hole or a boss (`crate::helical::thread_mill`).
    ThreadMill,
    /// A face mill: a flat end, its inserts' corners rounded by [`Tool::corner_radius`] when it
    /// is given.
    FaceMill,
    /// A chamfer mill: a flat tip of [`Tool::tip_diameter`], then a cone of
    /// [`Tool::angle_deg`] (the included angle) up to the cutting diameter.
    Chamfer,
}

impl ToolKind {
    pub fn label(self) -> &'static str {
        match self {
            ToolKind::FlatEndmill => "flat end mill",
            ToolKind::BallEndmill => "ball end mill",
            ToolKind::BullNose => "bull nose end mill",
            ToolKind::Drill => "drill",
            ToolKind::ThreadMill => "thread mill",
            ToolKind::FaceMill => "face mill",
            ToolKind::Chamfer => "chamfer mill",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Tool {
    /// The pocket the machine holds it in (`T` word).
    pub number: u32,
    pub kind: ToolKind,
    /// Cutting diameter (mm).
    pub diameter: f64,
    /// Length of the cutting edges (mm).
    pub flute_length: f64,
    #[serde(default)]
    pub description: String,
    /// Number of cutting edges.
    #[serde(default = "two")]
    pub flutes: u32,
    /// The cutting data operations start from: spindle speed (rev/min) and feeds (mm/min).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_rpm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plunge_feed: Option<f64>,
    /// The depth per pass and the distance between passes operations start from (mm).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepdown: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepover: Option<f64>,
    /// Radius of a bull nose's corners (mm), above 0 and below the tool radius; for a face
    /// mill, its inserts' corner radius, 0 to below the tool radius (left out: square). Ignored
    /// by other kinds: a flat end mill and a chamfer mill have none and a ball's is its radius.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub corner_radius: Option<f64>,
    /// A chamfer mill's tip diameter (mm), from 0 (pointed) to below the cutting diameter;
    /// left out, 0. Only for chamfer mills.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tip_diameter: Option<f64>,
    /// A chamfer mill's included angle (degrees), 1 to 179: 90 for a 45° chamfer. Required for
    /// chamfer mills, and only for them.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub angle_deg: Option<f64>,
    /// Diameter of the plain shank above the flutes (mm); left out, the cutting diameter.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shank_diameter: Option<f64>,
    /// How far the tool stands out of its holder, tip to holder face (mm); left out, the
    /// flute length. Collision checks use it with [`Tool::holder`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stickout: Option<f64>,
    /// The holder from its face up, as stacked cylinders; empty: no holder is checked.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub holder: Vec<HolderSection>,
    /// A drill's point angle (degrees, the included angle of its tip); left out, 118. Other
    /// kinds ignore it. See [`crate::drill::tip_length`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub point_angle_deg: Option<f64>,
    /// A multi-form thread mill's pitch (mm), the distance between its rows of teeth, which
    /// span the flute length; left out, a single-point thread mill, which cuts any pitch.
    /// Thread mills only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thread_pitch: Option<f64>,
    /// How far a thread mill's teeth stand out of its core (mm), tips to roots, below the tool
    /// radius: thread milling refuses a thread deeper than this. Left out, any depth up to
    /// the radius. Thread mills only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tooth_height: Option<f64>,
    /// The cutting speed at the cutting diameter (m/min): without `spindle_rpm`, the spindle
    /// speed is worked out from it ([`Tool::speeds`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub surface_speed: Option<f64>,
    /// The feed per tooth (mm): without `feed`, the cutting feed is the spindle speed times the
    /// flutes times this ([`Tool::speeds`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub chip_load: Option<f64>,
    /// Cutting data for a kind of operation, each over the tool's own values when an operation
    /// of that strategy uses the tool ([`Tool::for_strategy`]): an adaptive clearing's deep,
    /// thin cut and a slot's shallow, full-width one want different numbers.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub cutting_data: Vec<CuttingData>,
}

/// The kinds of operation a tool's [`CuttingData`] is for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Strategy {
    /// Adaptive clearing: deep, at a small, steady engagement.
    Adaptive,
    /// Cutting at up to the tool's whole width: pockets, contours, bores, roughing levels.
    Slot,
    /// Facing the stock's top.
    Face,
    /// 3D finishing passes.
    Finish,
    Chamfer,
    Drill,
}

impl Strategy {
    pub fn label(self) -> &'static str {
        match self {
            Strategy::Adaptive => "adaptive",
            Strategy::Slot => "slot",
            Strategy::Face => "face",
            Strategy::Finish => "finish",
            Strategy::Chamfer => "chamfer",
            Strategy::Drill => "drill",
        }
    }
}

/// A tool's cutting data for one [`Strategy`]: each value given takes the place of the tool's
/// own when an operation of that strategy uses it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CuttingData {
    pub strategy: Strategy,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_rpm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plunge_feed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepdown: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepover: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub surface_speed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub chip_load: Option<f64>,
}

/// Where a tool's spindle speed and cutting feed come from ([`Tool::speeds`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Speeds {
    /// Rev/min: the tool's `spindle_rpm`, else from its `surface_speed`.
    pub spindle_rpm: Option<f64>,
    /// Whether the spindle speed was worked out from the surface speed.
    pub rpm_derived: bool,
    /// Mm/min: the tool's `feed`, else the spindle speed times the flutes times `chip_load`.
    pub feed: Option<f64>,
    /// Whether the feed was worked out from the chip load.
    pub feed_derived: bool,
}

/// One cylinder of a tool holder.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct HolderSection {
    pub diameter: f64,
    pub length: f64,
}

/// A solid of revolution about the tool axis: a cylinder from `bottom` to `top` above the tip.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Envelope {
    pub bottom: f64,
    pub top: f64,
    pub radius: f64,
}

fn two() -> u32 {
    2
}

impl Tool {
    pub fn new(number: u32, kind: ToolKind, diameter: f64, flute_length: f64) -> Tool {
        Tool {
            number,
            kind,
            diameter,
            flute_length,
            description: String::new(),
            flutes: 2,
            spindle_rpm: None,
            feed: None,
            plunge_feed: None,
            stepdown: None,
            stepover: None,
            corner_radius: None,
            tip_diameter: None,
            angle_deg: None,
            shank_diameter: None,
            stickout: None,
            holder: Vec::new(),
            point_angle_deg: None,
            thread_pitch: None,
            tooth_height: None,
            surface_speed: None,
            chip_load: None,
            cutting_data: Vec::new(),
        }
    }

    /// The tool as an operation of `strategy` uses it: the values of its cutting data for that
    /// strategy (the first entry for it) in place of its own; without one, itself.
    pub fn for_strategy(&self, strategy: Strategy) -> Tool {
        let mut t = self.clone();
        if let Some(d) = self.cutting_data.iter().find(|d| d.strategy == strategy) {
            // Explicit speeds of the data win over the tool's, and so do its rates.
            if d.spindle_rpm.is_some() || d.surface_speed.is_some() {
                (t.spindle_rpm, t.surface_speed) = (d.spindle_rpm, d.surface_speed);
            }
            if d.feed.is_some() || d.chip_load.is_some() {
                (t.feed, t.chip_load) = (d.feed, d.chip_load);
            }
            t.plunge_feed = d.plunge_feed.or(t.plunge_feed);
            t.stepdown = d.stepdown.or(t.stepdown);
            t.stepover = d.stepover.or(t.stepover);
        }
        t
    }

    /// The spindle speed and cutting feed the tool gives, explicit values first: the spindle
    /// speed from the surface speed (`1000 · vc / (π · D)`) when it has no `spindle_rpm`, the
    /// feed from the chip load (`rpm · flutes · fz`) when it has no `feed`. `rpm` is the speed
    /// the feed is worked out at, when the caller has held the spindle speed to a range.
    pub fn speeds(&self, rpm: Option<f64>) -> Speeds {
        let (spindle_rpm, rpm_derived) = match (self.spindle_rpm, self.surface_speed) {
            (Some(r), _) => (Some(r), false),
            (None, Some(vc)) => (Some(1000.0 * vc / (std::f64::consts::PI * self.diameter)), true),
            (None, None) => (None, false),
        };
        let (feed, feed_derived) = match (self.feed, self.chip_load, rpm.or(spindle_rpm)) {
            (Some(f), _, _) => (Some(f), false),
            (None, Some(fz), Some(n)) => (Some(n * f64::from(self.flutes) * fz), true),
            _ => (None, false),
        };
        Speeds { spindle_rpm, rpm_derived, feed, feed_derived }
    }

    /// The cutting end's shape. A thread mill is the flat cylinder its teeth sweep, at their
    /// tips: whatever checks with it sees all of the tool. Thread milling alone checks with the
    /// core inside the teeth (`crate::helical::thread_check_tool`), since the thread they cut
    /// is not in the model.
    pub fn cutter(&self) -> Result<Cutter> {
        let r = self.radius();
        match self.kind {
            ToolKind::FlatEndmill => Cutter::flat(r),
            ToolKind::BallEndmill => Cutter::ball(r),
            ToolKind::BullNose => Cutter::bull(r, self.corner_radius.unwrap_or(0.0)),
            ToolKind::Drill => invalid("a drill cannot mill: it cuts only at its point"),
            ToolKind::ThreadMill => Cutter::flat(r),
            ToolKind::FaceMill => match self.corner_radius.filter(|c| *c > 0.0) {
                Some(c) => Cutter::bull(r, c),
                None => Cutter::flat(r),
            },
            ToolKind::Chamfer => {
                let Some(angle) = self.angle_deg else { return invalid("a chamfer mill needs its included angle (`angle_deg`)") };
                Cutter::tapered(r, self.tip_diameter.unwrap_or(0.0) * 0.5, angle.to_radians() * 0.5)
            }
        }
    }

    /// The parts of the tool above the flutes that must not touch the part: the shank from the
    /// flutes to the holder face, then the holder's sections. Heights are above the tip (mm).
    pub fn envelopes(&self) -> Vec<Envelope> {
        let stickout = self.stickout.unwrap_or(self.flute_length).max(self.flute_length);
        let shank = self.shank_diameter.unwrap_or(self.diameter) * 0.5;
        let mut out = Vec::new();
        if stickout > self.flute_length {
            out.push(Envelope { bottom: self.flute_length, top: stickout, radius: shank });
        }
        let mut z = stickout;
        for s in &self.holder {
            out.push(Envelope { bottom: z, top: z + s.length, radius: s.diameter * 0.5 });
            z += s.length;
        }
        out
    }
    pub fn radius(&self) -> f64 {
        self.diameter * 0.5
    }
    pub fn validate(&self) -> Result<()> {
        if self.number > MAX_TOOL_NUMBER {
            return invalid(format!("tool number {} is above {MAX_TOOL_NUMBER}", self.number));
        }
        if !(self.diameter > 0.0) || !(self.diameter <= MAX_TOOL_DIAMETER) {
            return invalid("the tool diameter must be above 0 and at most 1000 mm");
        }
        if !(self.flute_length > 0.0) || !(self.flute_length <= 10_000.0) {
            return invalid("the tool's flute length must be above 0 and at most 10 m");
        }
        if !(1..=100).contains(&self.flutes) {
            return invalid("a tool has 1 to 100 flutes");
        }
        if self.kind == ToolKind::BullNose && !self.corner_radius.is_some_and(|c| c > 0.0 && c < self.radius()) {
            return invalid("a bull nose's corner radius must be above 0 and below the tool radius");
        }
        if self.thread_pitch.is_some_and(|p| !(p > 0.0) || !(p <= self.flute_length)) {
            return invalid("a thread mill's pitch must be above 0 and at most its flute length");
        }
        if self.tooth_height.is_some_and(|h| !(h > 0.0) || !(h < self.radius())) {
            return invalid("a thread mill's tooth height must be above 0 and below the tool radius");
        }
        if self.kind != ToolKind::ThreadMill && (self.thread_pitch.is_some() || self.tooth_height.is_some()) {
            return invalid("`thread_pitch` and `tooth_height` are for thread mills");
        }
        if self.kind == ToolKind::FaceMill && self.corner_radius.is_some_and(|c| !(c >= 0.0) || !(c < self.radius())) {
            return invalid("a face mill's corner radius must be from 0 to below the tool radius");
        }
        if self.kind == ToolKind::Chamfer {
            if !self.angle_deg.is_some_and(|a| (1.0..=179.0).contains(&a)) {
                return invalid("a chamfer mill's included angle (`angle_deg`) must be given, from 1 to 179 degrees");
            }
            if self.tip_diameter.is_some_and(|t| !(t >= 0.0) || !(t < self.diameter)) {
                return invalid("a chamfer mill's tip diameter must be from 0 to below its diameter");
            }
        } else if self.tip_diameter.is_some() || self.angle_deg.is_some() {
            return invalid("`tip_diameter` and `angle_deg` are for chamfer mills");
        }
        if self.shank_diameter.is_some_and(|d| !(d > 0.0) || !(d <= MAX_TOOL_DIAMETER)) {
            return invalid("the shank diameter must be above 0 and at most 1000 mm");
        }
        if self.stickout.is_some_and(|s| !(s >= self.flute_length) || !(s <= 10_000.0)) {
            return invalid("the stickout must be at least the flute length and at most 10 m");
        }
        if self.holder.len() > 64 {
            return invalid("a holder has at most 64 sections");
        }
        if self.holder.iter().any(|s| !(s.diameter > 0.0) || !(s.diameter <= 10_000.0) || !(s.length > 0.0) || !(s.length <= 10_000.0)) {
            return invalid("a holder section's diameter and length must be above 0 and at most 10 m");
        }
        if self.point_angle_deg.is_some_and(|a| !(a >= 1.0) || !(a <= 180.0)) {
            return invalid("a drill's point angle must be from 1 to 180 degrees");
        }
        if [self.spindle_rpm, self.feed, self.plunge_feed, self.stepdown, self.stepover].iter().flatten().any(|v| !(*v > 0.0) || !(*v <= 1.0e6)) {
            return invalid("a tool's spindle speed, feeds, stepdown and stepover must be above 0");
        }
        if self.cutting_data.len() > MAX_CUTTING_DATA {
            return invalid(format!("a tool has at most {MAX_CUTTING_DATA} cutting data entries"));
        }
        let speeds = |vc: Option<f64>, fz: Option<f64>| {
            vc.is_some_and(|v| !(v > 0.0) || !(v <= MAX_SURFACE_SPEED)) || fz.is_some_and(|f| !(f > 0.0) || !(f <= MAX_CHIP_LOAD))
        };
        if speeds(self.surface_speed, self.chip_load) {
            return invalid(format!(
                "a tool's surface speed must be above 0 and at most {MAX_SURFACE_SPEED} m/min, its chip load above 0 and at most {MAX_CHIP_LOAD} mm"
            ));
        }
        for d in &self.cutting_data {
            let label = d.strategy.label();
            if [d.spindle_rpm, d.feed, d.plunge_feed, d.stepdown, d.stepover].iter().flatten().any(|v| !(*v > 0.0) || !(*v <= 1.0e6))
                || speeds(d.surface_speed, d.chip_load)
            {
                return invalid(format!("the tool's {label} cutting data must have every value above 0 (and within range)"));
            }
        }
        Ok(())
    }
}
