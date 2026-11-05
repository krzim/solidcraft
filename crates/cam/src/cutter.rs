//! The shape of a cutter's end, as every milling cutter is described here: a flat bottom of
//! radius `radius - corner` with a quarter-round corner of radius `corner` around it. A flat
//! end mill has no corner, a ball's corner is its radius, and a bull nose is in between.
//!
//! Heights are measured up the tool axis from the tip, the lowest point of the cutter.

use serde::{Deserialize, Serialize};

use crate::tool::MAX_TOOL_DIAMETER;
use crate::{Result, invalid};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Cutter {
    /// Cutting radius (mm).
    pub radius: f64,
    /// Corner radius (mm), from 0 (flat) to `radius` (ball).
    pub corner: f64,
}

impl Cutter {
    pub fn flat(radius: f64) -> Result<Cutter> {
        Cutter { radius, corner: 0.0 }.checked()
    }

    pub fn ball(radius: f64) -> Result<Cutter> {
        Cutter { radius, corner: radius }.checked()
    }

    pub fn bull(radius: f64, corner: f64) -> Result<Cutter> {
        Cutter { radius, corner }.checked()
    }

    fn checked(self) -> Result<Cutter> {
        self.validate()?;
        Ok(self)
    }

    pub fn validate(&self) -> Result<()> {
        if !(self.radius > 0.0) || !(self.radius <= MAX_TOOL_DIAMETER * 0.5) {
            return invalid("a cutter's radius must be above 0 and at most 500 mm");
        }
        if !(self.corner >= 0.0) || !(self.corner <= self.radius) {
            return invalid("a cutter's corner radius must be from 0 to its radius");
        }
        Ok(())
    }

    /// Radius of the flat part of the bottom.
    pub fn flat_radius(&self) -> f64 {
        (self.radius - self.corner).max(0.0)
    }

    pub fn is_ball(&self) -> bool {
        self.radius - self.corner <= 1e-12
    }

    /// Height of the cutter's surface above the tip at distance `d` from the axis; `None`
    /// beyond the cutting radius. Convex and rising in `d`, which the drop-cutter relies on.
    pub fn height(&self, d: f64) -> Option<f64> {
        if !(d <= self.radius + 1e-12) {
            return None;
        }
        let a = self.flat_radius();
        if d <= a {
            return Some(0.0);
        }
        let e = (d - a).min(self.corner);
        Some(self.corner - (self.corner * self.corner - e * e).max(0.0).sqrt())
    }

    /// The cutter grown by `by` all round (its tip `by` lower): the shape that keeps the real
    /// cutter `by` away from a surface, used to leave stock.
    pub fn grown(&self, by: f64) -> Result<Cutter> {
        if !(by >= 0.0) || !by.is_finite() {
            return invalid("the stock to leave must be 0 or more");
        }
        Cutter { radius: self.radius + by, corner: self.corner + by }.checked()
    }
}
