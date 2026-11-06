//! How much of a cutter's circumference is in material along a finished 2.5D toolpath, measured
//! from the material itself rather than from any model a planner kept.
//!
//! The material is the stock above a height inside some regions. The path is followed in short
//! pieces; at the end of each piece the tool's circumference (a flat cutter's radius at its tip)
//! is looked at in 1° steps, and a point of it counts as in material when it is inside the
//! regions, below the top and not within the tool's radius of any earlier piece at or below the
//! tip's height (exact distances to the pieces, found through a spatial index, with a raster of
//! the distances at the current level settling the points far from the edge of the cut). The engagement is that
//! share of the full turn, at most 180° (a full slot: the half behind the tool is always cut). The
//! pieces of a rapid do not cut. Up to the sampling along the path and round the tool the result
//! is exact, so a planner's report can be checked against it.

use std::f64::consts::{PI, TAU};

use serde::{Deserialize, Serialize};
use solidcore_record::{Region2, Vec2, Vec3};
use solidcraft_geom::Ring;
use solidcraft_geom::prelude::*;

use crate::adaptive::{FULL_SLOT, MoveRole};
use crate::path::{Move, Toolpath, arc_sweep};
use crate::{CamError, Result, invalid};

/// Points round the tool looked at, one a degree.
const RING: usize = 360;
/// The points round the tool are this share of the radius inside its circle, so a point on the
/// edge of an earlier cut is settled the same way every time.
pub const SKIN: f64 = 1e-7;
/// Most distance tests of one measurement.
const MAX_TESTS: f64 = 4.0e9;
/// Most pieces the path is followed in.
const MAX_PIECES: f64 = 2.0e6;
/// Most cells of the spatial index.
const MAX_BUCKETS: f64 = 4.0e6;

/// The stock a path cuts: inside `regions` (holes are left out) up to `top`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Material {
    pub regions: Vec<Region2>,
    pub top: f64,
}

/// What [`engagement`] found.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Engagement {
    /// Largest engagement outside entry moves (degrees).
    pub max_deg: f64,
    /// Largest engagement in entry moves (degrees).
    pub max_entry_deg: f64,
    /// Mean engagement over the cutting time outside entries where the tool cuts at all (degrees).
    pub mean_deg: f64,
    /// Share of the cutting time outside entries at a full slot (170° or more).
    pub full_slot_share: f64,
    /// Share of all the cutting time spent at a full slot in entries.
    pub entry_slot_share: f64,
    /// Points of the path looked at.
    pub samples: usize,
    /// Where the largest engagement outside entries is, and the move it is in.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub worst_at: Option<Vec3>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub worst_move: Option<usize>,
}

/// Earlier pieces of a path by where they are (the adaptive planner keeps its own).
pub(crate) struct Index {
    origin: Vec2,
    size: f64,
    nx: usize,
    ny: usize,
    buckets: Vec<Vec<u32>>,
    pieces: Vec<(Vec2, Vec2, f64)>,
}

fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let d = b - a;
    let l2 = d.len2();
    let t = if l2 > 0.0 { ((p - a).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
    p.dist(a + d * t)
}

impl Index {
    /// An index over `lo..hi` with buckets `size` across (at least the radius it is asked about),
    /// coarser where that would take more than [`MAX_BUCKETS`].
    pub(crate) fn new(lo: Vec2, hi: Vec2, size: f64) -> Index {
        let mut size = size.max(1e-6);
        let cells = |s: f64| ((hi.x - lo.x) / s).ceil().max(1.0) * ((hi.y - lo.y) / s).ceil().max(1.0);
        while !(cells(size) <= MAX_BUCKETS) && size < 1e12 {
            size *= 2.0;
        }
        let (nx, ny) = (((hi.x - lo.x) / size).ceil().max(1.0) as usize, ((hi.y - lo.y) / size).ceil().max(1.0) as usize);
        Index { origin: lo, size, nx, ny, buckets: vec![Vec::new(); nx * ny], pieces: Vec::new() }
    }

    fn bucket(&self, p: Vec2) -> (isize, isize) {
        (((p.x - self.origin.x) / self.size).floor() as isize, ((p.y - self.origin.y) / self.size).floor() as isize)
    }

    pub(crate) fn insert(&mut self, a: Vec2, b: Vec2, z: f64) {
        let id = self.pieces.len() as u32;
        self.pieces.push((a, b, z));
        self.file(id, a, b);
    }

    /// Moves the last piece's end on to `b` (a piece along the same line).
    fn extend(&mut self, b: Vec2) {
        let Some(last) = self.pieces.len().checked_sub(1) else { return };
        let Some(piece) = self.pieces.get_mut(last) else { return };
        let from = piece.1;
        piece.1 = b;
        self.file(last as u32, from, b);
    }

    /// Files piece `id` under the buckets the stretch `a`-`b` of it crosses.
    fn file(&mut self, id: u32, a: Vec2, b: Vec2) {
        let (i0, j0) = self.bucket(Vec2::new(a.x.min(b.x), a.y.min(b.y)));
        let (i1, j1) = self.bucket(Vec2::new(a.x.max(b.x), a.y.max(b.y)));
        for j in j0.max(0)..=j1.min(self.ny as isize - 1) {
            for i in i0.max(0)..=i1.min(self.nx as isize - 1) {
                if let Some(list) = self.buckets.get_mut(j as usize * self.nx + i as usize)
                    && list.last() != Some(&id)
                {
                    list.push(id);
                }
            }
        }
    }

    /// Whether `q` is within `r` of an earlier piece at or below `z`; counts the tests.
    pub(crate) fn cut(&self, q: Vec2, z: f64, r: f64, tests: &mut f64) -> bool {
        let (ci, cj) = self.bucket(q);
        for j in (cj - 1).max(0)..=(cj + 1).min(self.ny as isize - 1) {
            for i in (ci - 1).max(0)..=(ci + 1).min(self.nx as isize - 1) {
                let Some(list) = self.buckets.get(j as usize * self.nx + i as usize) else { continue };
                // The latest pieces are the likeliest to have cut there.
                for id in list.iter().rev() {
                    *tests += 1.0;
                    if let Some(&(a, b, pz)) = self.pieces.get(*id as usize)
                        && pz <= z + 1e-6
                        && seg_dist(q, a, b) < r
                    {
                        return true;
                    }
                }
            }
        }
        false
    }
}

/// Distances from a raster's cells to the pieces at or below one level, which settle most points
/// without the exact search: a point whose cell is well inside or well outside the radius.
struct Raster {
    origin: Vec2,
    h: f64,
    nx: usize,
    ny: usize,
    r: f64,
    d: Vec<f64>,
    /// The level the distances are for.
    z: Option<f64>,
}

impl Raster {
    fn new(lo: Vec2, hi: Vec2, r: f64) -> Raster {
        let mut h = r / 32.0;
        let cells = |s: f64| (((hi.x - lo.x) / s).ceil() + 1.0).max(1.0) * (((hi.y - lo.y) / s).ceil() + 1.0).max(1.0);
        while !(cells(h) <= MAX_BUCKETS) && h < 1e12 {
            h *= 2.0;
        }
        let (nx, ny) = ((((hi.x - lo.x) / h).ceil() + 1.0).max(1.0) as usize, (((hi.y - lo.y) / h).ceil() + 1.0).max(1.0) as usize);
        Raster { origin: lo, h, nx, ny, r, d: vec![f64::INFINITY; nx * ny], z: None }
    }

    /// Makes the raster hold the pieces at or below `z`.
    fn hold(&mut self, z: f64, index: &Index, tests: &mut f64) {
        if self.z.is_some_and(|hz| (hz - z).abs() <= 1e-9) {
            return;
        }
        self.d.fill(f64::INFINITY);
        self.z = Some(z);
        *tests += self.d.len() as f64;
        for &(a, b, pz) in &index.pieces {
            if pz <= z + 1e-6 {
                self.paint(a, b, tests);
            }
        }
    }

    fn add(&mut self, a: Vec2, b: Vec2, pz: f64, tests: &mut f64) {
        if self.z.is_some_and(|z| pz <= z + 1e-6) {
            self.paint(a, b, tests);
        }
    }

    fn paint(&mut self, a: Vec2, b: Vec2, tests: &mut f64) {
        let reach = self.r + 2.0 * self.h;
        let at = |v: f64, o: f64, n: usize| -> usize {
            let f = ((v - o) / self.h).floor();
            if !(f > 0.0) {
                0
            } else if f >= n as f64 {
                n
            } else {
                f as usize
            }
        };
        let (i0, i1) =
            (at(a.x.min(b.x) - reach, self.origin.x, self.nx), at(a.x.max(b.x) + reach, self.origin.x, self.nx).saturating_add(2).min(self.nx));
        let (j0, j1) =
            (at(a.y.min(b.y) - reach, self.origin.y, self.ny), at(a.y.max(b.y) + reach, self.origin.y, self.ny).saturating_add(2).min(self.ny));
        *tests += ((i1 - i0) * (j1 - j0)) as f64;
        for j in j0..j1 {
            for i in i0..i1 {
                let c = Vec2::new(self.origin.x + i as f64 * self.h, self.origin.y + j as f64 * self.h);
                let dist = seg_dist(c, a, b);
                if let Some(d) = self.d.get_mut(j * self.nx + i)
                    && dist < *d
                {
                    *d = dist;
                }
            }
        }
    }

    /// Whether `q` is cut (`Some(true)`), surely not (`Some(false)`), or too near the edge of
    /// the cut to tell (`None`).
    fn settles(&self, q: Vec2) -> Option<bool> {
        let (fi, fj) = (((q.x - self.origin.x) / self.h).round(), ((q.y - self.origin.y) / self.h).round());
        if !(fi >= 0.0 && fj >= 0.0 && fi < self.nx as f64 && fj < self.ny as f64) {
            return None;
        }
        let d = self.d.get(fj as usize * self.nx + fi as usize).copied()?;
        let slack = self.h * std::f64::consts::FRAC_1_SQRT_2 + 1e-9;
        if d <= self.r - slack {
            Some(true)
        } else if d > self.r + slack {
            Some(false)
        } else {
            None
        }
    }
}

/// One piece of the path to look at: from, to, feed, whether it is an entry, its move, and
/// whether it goes on along the same straight line as the piece before.
type Piece = (Vec3, Vec3, f64, bool, usize, bool);

/// The path's cutting moves in pieces at most `ds` long.
fn pieces(path: &Toolpath, roles: Option<&[MoveRole]>, ds: f64) -> Result<Vec<Piece>> {
    let mut out = Vec::new();
    let mut pos: Option<Vec3> = None;
    for (i, m) in path.moves.iter().enumerate() {
        let entry = roles.and_then(|r| r.get(i)).is_some_and(|r| *r == MoveRole::Entry);
        if let Some(p) = pos {
            match *m {
                Move::Feed { to, feed } => {
                    let n = (p.dist(to) / ds).ceil().max(1.0);
                    if !(n <= MAX_PIECES) {
                        return Err(CamError::TooLarge("the path is too long to measure".into()));
                    }
                    let mut a = p;
                    for k in 1..=n as usize {
                        let b = p.lerp(to, k as f64 / n);
                        out.push((a, b, feed, entry, i, k > 1));
                        a = b;
                    }
                }
                Move::Arc { to, center, ccw, feed } => {
                    let start = Vec2::new(p.x, p.y) - center;
                    let (r, sweep) = (start.len(), arc_sweep(p, to, center, ccw));
                    let n = ((r * sweep.abs()).hypot(to.z - p.z) / ds).ceil().max(1.0);
                    if !(n <= MAX_PIECES) {
                        return Err(CamError::TooLarge("the path is too long to measure".into()));
                    }
                    let mut a = p;
                    for k in 1..=n as usize {
                        let t = k as f64 / n;
                        let b = if k == n as usize {
                            to
                        } else {
                            (center + Vec2::from_angle(start.angle() + sweep * t) * r).extend(p.z + (to.z - p.z) * t)
                        };
                        out.push((a, b, feed, entry, i, false));
                        a = b;
                    }
                }
                Move::Rapid { .. } | Move::Dwell { .. } => {}
            }
        }
        if let Some(to) = m.target() {
            pos = Some(to);
        }
        if out.len() as f64 > MAX_PIECES {
            return Err(CamError::TooLarge("the path is too long to measure".into()));
        }
    }
    Ok(out)
}

/// Measures the engagement of a cutter of `radius` along `path` in `material`. `roles` (one per
/// move, as an adaptive path gives them) tells the entries apart; without it every move counts as
/// clearing. `spacing` is the most distance between the points looked at (at most an eighth of
/// the radius is used; more for a very long path).
pub fn engagement(path: &Toolpath, roles: Option<&[MoveRole]>, radius: f64, material: &Material, spacing: f64) -> Result<Engagement> {
    if !(radius > 0.0) || !radius.is_finite() || radius > 500.0 {
        return invalid("the tool radius must be above 0 and at most 500 mm");
    }
    if !(spacing > 0.0) || !spacing.is_finite() {
        return invalid("the sample spacing must be above 0");
    }
    if !material.top.is_finite() {
        return invalid("the top of the material must be a finite number");
    }
    let fences: Vec<(Ring<Vec2>, Vec<Ring<Vec2>>)> =
        material.regions.iter().map(|r| (r.outer.polyline(radius * 1e-3), r.holes.iter().map(|h| h.polyline(radius * 1e-3)).collect())).collect();
    let inside = |q: Vec2| fences.iter().any(|(o, hs)| o.contains(q) && !hs.iter().any(|h| h.contains(q)));

    let stats = path.stats();
    let ds = spacing.min(radius / 8.0).max(stats.cut_length / 1.0e6).max(radius * 1e-3);
    let pieces = pieces(path, roles, ds)?;
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for (a, b, ..) in &pieces {
        for p in [a, b] {
            if !p.is_finite() {
                return invalid("the path has a coordinate that is not a finite number");
            }
            lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
            hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
        }
    }
    if pieces.is_empty() {
        return Ok(Engagement::default());
    }
    let pad = Vec2::new(2.0 * radius, 2.0 * radius);
    let mut index = Index::new(lo - pad, hi + pad, radius);
    let mut raster = Raster::new(lo - pad, hi + pad, radius);
    let ring: Vec<Vec2> = (0..RING).map(|k| Vec2::from_angle(TAU * k as f64 / RING as f64) * (radius * (1.0 - SKIN))).collect();

    let mut tests = 0.0;
    let mut out = Engagement { samples: pieces.len(), ..Engagement::default() };
    let (mut time, mut slot, mut entry_slot, mut all, mut sum, mut cutting) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for &(a, b, feed, entry, move_index, along) in &pieces {
        let (a2, b2) = (Vec2::new(a.x, a.y), Vec2::new(b.x, b.y));
        // What counts is the material there before this piece: the earlier pieces are in the index.
        let mut n = 0usize;
        if material.top > b.z + 1e-6 {
            // A level piece is looked up in the raster of that level first.
            let level = (a.z - b.z).abs() <= 1e-9;
            if level {
                raster.hold(b.z, &index, &mut tests);
            }
            for u in &ring {
                let q = b2 + *u;
                if !inside(q) {
                    continue;
                }
                let known = if level { raster.settles(q) } else { None };
                let cut = match known {
                    Some(c) => c,
                    None => index.cut(q, b.z, radius, &mut tests),
                };
                if !cut {
                    n += 1;
                }
            }
            if tests > MAX_TESTS {
                return Err(CamError::TooLarge("the path is too long or dense to measure its engagement".into()));
            }
        }
        if along && (a.z - b.z).abs() <= 1e-12 {
            index.extend(b2);
        } else {
            index.insert(a2, b2, a.z.max(b.z));
        }
        raster.add(a2, b2, a.z.max(b.z), &mut tests);
        let phi = (TAU * n as f64 / RING as f64).min(PI);
        let dt = if feed > 0.0 { a.dist(b) / feed } else { 0.0 };
        all += dt;
        if entry {
            out.max_entry_deg = out.max_entry_deg.max(phi.to_degrees());
            if phi >= FULL_SLOT {
                entry_slot += dt;
            }
        } else {
            if phi.to_degrees() > out.max_deg || out.worst_at.is_none() {
                out.max_deg = phi.to_degrees();
                out.worst_at = Some(b);
                out.worst_move = Some(move_index);
            }
            time += dt;
            if phi >= FULL_SLOT {
                slot += dt;
            }
            if phi > 0.0 {
                sum += phi.to_degrees() * dt;
                cutting += dt;
            }
        }
    }
    out.full_slot_share = if time > 0.0 { slot / time } else { 0.0 };
    out.entry_slot_share = if all > 0.0 { entry_slot / all } else { 0.0 };
    out.mean_deg = if cutting > 0.0 { sum / cutting } else { 0.0 };
    Ok(out)
}
