//! Safety rules every toolpath must keep, checked before a path leaves an operation.

use std::collections::HashMap;

use solidcore_record::{Vec2, Vec3};
use solidcraft_geom::prelude::*;

use crate::ops::Heights;
use crate::path::{MAX_MOVES, Move, Toolpath};
use crate::{CamError, Result};

const TOL: f64 = 1e-6;

fn fail<T>(i: usize, why: impl std::fmt::Display) -> Result<T> {
    Err(CamError::Unsafe(format!("move {i}: {why}")))
}

fn xy(p: Vec3) -> Vec2 {
    Vec2::new(p.x, p.y)
}

/// Side of the cells cutting moves are filed under (mm).
const CELL: f64 = 2.0;
/// Most cells one cutting move is filed under; a longer one is not looked for.
const MAX_CELLS: i64 = 1024;

/// The straight cutting moves so far, filed by where they are seen from above, to find how
/// deep the tool has been at a point.
#[derive(Default)]
pub(crate) struct Swept {
    cells: HashMap<(i64, i64), Vec<(Vec3, Vec3)>>,
}

impl Swept {
    fn cell(v: f64) -> Option<i64> {
        let c = (v / CELL).floor();
        (c.abs() < 1e15).then_some(c as i64)
    }

    pub(crate) fn add(&mut self, a: Vec3, b: Vec3) {
        let (Some(x0), Some(x1), Some(y0), Some(y1)) =
            (Self::cell(a.x.min(b.x)), Self::cell(a.x.max(b.x)), Self::cell(a.y.min(b.y)), Self::cell(a.y.max(b.y)))
        else {
            return;
        };
        if (x1 - x0 + 1).saturating_mul(y1 - y0 + 1) > MAX_CELLS {
            return;
        }
        for i in x0..=x1 {
            for j in y0..=y1 {
                self.cells.entry((i, j)).or_default().push((a, b));
            }
        }
    }

    /// Whether a cutting move so far went through `p` (seen from above) below `z`.
    pub(crate) fn below(&self, p: Vec2, z: f64) -> bool {
        let (Some(i), Some(j)) = (Self::cell(p.x), Self::cell(p.y)) else { return false };
        self.cells.get(&(i, j)).is_some_and(|segs| {
            segs.iter().any(|&(a, b)| {
                let (a2, b2) = (xy(a), xy(b));
                let d = b2 - a2;
                let len2 = d.dot(d);
                // A move straight down or up went through its point as low as its lower end.
                let (u, low) = if len2 > 1e-24 {
                    let u = ((p - a2).dot(d) / len2).clamp(0.0, 1.0);
                    (u, a.z + (b.z - a.z) * u)
                } else {
                    (0.0, a.z.min(b.z))
                };
                a2.lerp(b2, u).dist(p) <= 1e-6 && low < z - 1e-9
            })
        })
    }
}

/// Checks the rules:
/// - every coordinate is finite and every feed is above zero;
/// - the path starts with a rapid to the clearance height or above;
/// - a rapid that moves in XY stays at the retract height or above;
/// - a rapid below the retract height is vertical and stops at or above the top of the
///   material, above the deepest point already cut at that position (a drill returning into
///   its hole), or above a straight cutting move that went through that position before (a
///   level coming down to the one it cut);
/// - no move goes below the bottom height;
/// - an arc starts and ends at the same distance from its centre.
pub fn verify(path: &Toolpath, h: &Heights) -> Result<()> {
    if path.moves.len() > MAX_MOVES {
        return Err(CamError::TooLarge(format!("the toolpath has more than {MAX_MOVES} moves")));
    }
    let mut pos: Option<Vec3> = None;
    // Deepest Z cut at the current XY position since the tool arrived there.
    let mut cut_here: Option<f64> = None;
    let mut swept = Swept::default();
    for (i, m) in path.moves.iter().enumerate() {
        if let Some(to) = m.target() {
            if !to.is_finite() {
                return fail(i, "a coordinate is not a finite number");
            }
            if to.z < h.bottom - TOL {
                return fail(i, format!("Z {} is below the bottom height {}", to.z, h.bottom));
            }
        }
        match *m {
            Move::Rapid { to } => {
                let Some(from) = pos else {
                    if to.z < h.clearance - TOL {
                        return fail(i, "the first move must go to the clearance height");
                    }
                    pos = Some(to);
                    continue;
                };
                if xy(from).dist(xy(to)) > 1e-9 {
                    if from.z < h.retract - TOL || to.z < h.retract - TOL {
                        return fail(i, "a rapid moves sideways below the retract height");
                    }
                    cut_here = None;
                } else if to.z < h.retract - TOL && to.z < h.top - TOL && !cut_here.is_some_and(|z| to.z > z + 1e-9) && !swept.below(xy(to), to.z) {
                    return fail(i, "a rapid goes down into material that has not been cut");
                }
            }
            Move::Feed { to, feed } => {
                if pos.is_none() {
                    return fail(i, "the path must start with a rapid");
                }
                if !(feed > 0.0) || !feed.is_finite() {
                    return fail(i, "the feed must be above 0");
                }
                let moved = pos.is_some_and(|from| xy(from).dist(xy(to)) > 1e-9);
                cut_here = Some(if moved { to.z } else { cut_here.map_or(to.z, |z| z.min(to.z)) });
                if let Some(from) = pos {
                    swept.add(from, to);
                }
            }
            Move::Arc { to, center, feed, .. } => {
                let Some(from) = pos else { return fail(i, "the path must start with a rapid") };
                if !(feed > 0.0) || !feed.is_finite() {
                    return fail(i, "the feed must be above 0");
                }
                let (r0, r1) = (xy(from).dist(center), xy(to).dist(center));
                if !center.is_finite() || !(r0 > 0.0) || (r0 - r1).abs() > 1e-3 {
                    return fail(i, "an arc does not start and end at the same distance from its centre");
                }
                cut_here = Some(to.z);
                // An arc is filed by its ends: its chord is not where it cut.
                swept.add(from, from);
                swept.add(to, to);
            }
            Move::Dwell { seconds } => {
                if !(seconds >= 0.0) || !seconds.is_finite() {
                    return fail(i, "a dwell must be 0 seconds or longer");
                }
            }
        }
        if let Some(to) = m.target() {
            pos = Some(to);
        }
    }
    Ok(())
}
