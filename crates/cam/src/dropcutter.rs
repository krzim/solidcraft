//! The drop-cutter: lower a cutter along its axis onto a surface until it first touches, and
//! report where its tip stops. Every 3D finishing and roughing toolpath here is built from it,
//! and every multi-axis pose is checked with it, because a tip placed where the dropped cutter
//! stops cannot cut into the surface.
//!
//! Against one triangle the cutter touches either a corner, the inside of the face, or an
//! edge. Corners and faces are worked out directly. Along an edge the height the tip must have
//! to touch the edge's point at parameter `t` is `z(t) - h(d(t))`, with `z` linear, `d` the
//! (convex) distance from the axis and `h` the cutter's (convex, rising) height profile, so it
//! is concave in `t`: a flat end's maximum is at the end of the stretch under the cutter, and
//! the others' is found by golden-section search to far below any machining tolerance.

use std::cell::Cell;

use solidcore_record::Vec3;
use solidcraft_geom::prelude::*;

use crate::cutter::Cutter;
use crate::frame::{Frame, unit};
use crate::surface::{Surface, capsule_meets_box};
use crate::{Result, invalid};

/// Golden-section steps along an edge for a bull nose: the parameter to 0.618^48 ≈ 1e-10 of
/// the stretch under the cutter. Near the maximum the height changes with the square of the
/// parameter's error, so the height is found far more closely still.
const EDGE_STEPS: usize = 48;

/// The highest tip height at which `cutter`, its axis vertical through `(x, y)`, touches the
/// triangle `t`; `None` when the cutter does not reach over it.
pub fn drop_triangle(cutter: &Cutter, t: &[Vec3; 3], x: f64, y: f64) -> Option<f64> {
    drop_triangle_above(cutter, t, x, y, f64::NEG_INFINITY)
}

/// [`drop_triangle`], skipping the work that cannot find a tip height above `above`: the
/// answer is exact when it is above `above`, and may be `None` or anything not above it
/// otherwise.
fn drop_triangle_above(cutter: &Cutter, t: &[Vec3; 3], x: f64, y: f64, above: f64) -> Option<f64> {
    let r = cutter.radius;
    let mut best: Option<f64> = None;
    let keep = |best: &mut Option<f64>, z: f64| {
        if z.is_finite() {
            *best = Some(best.map_or(z, |b: f64| b.max(z)));
        }
    };
    // Corners.
    for v in t {
        if let Some(h) = cutter.height((v.x - x).hypot(v.y - y)) {
            keep(&mut best, v.z - h);
        }
    }
    // The inside of the face.
    if let Some(mut n) = (t[1] - t[0]).cross(t[2] - t[0]).normalized() {
        if n.z < 0.0 {
            n = -n;
        }
        // A face this close to vertical is met at its edges and corners first.
        if n.z > 1e-9 {
            let s = n.x.hypot(n.y);
            let a = cutter.flat_radius();
            // The flat bottom's edge (or centre) on the side where the plane is highest.
            let (qx, qy) = if s > 1e-12 { (x - a * n.x / s, y - a * n.y / s) } else { (x, y) };
            let rc = cutter.corner;
            let zc = t[0].z + (rc - n.x * (qx - t[0].x) - n.y * (qy - t[0].y)) / n.z;
            let cc = Vec3::new(qx, qy, zc) - n * rc;
            // The contact is on the face and the cutter's surface is never below its tip, so a
            // tip above the face's highest corner is a point outside it accepted by rounding.
            let top = t[0].z.max(t[1].z).max(t[2].z);
            if inside_xy(t, cc.x, cc.y) && zc - rc <= top + 1e-9 {
                keep(&mut best, zc - rc);
            }
        }
    }
    // Edges: a tip touching an edge is never above the edge's higher end, so an edge whose
    // higher end is no higher than what is already found cannot raise it.
    for (p, q) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
        if p.z.max(q.z) <= best.unwrap_or(f64::NEG_INFINITY).max(above) {
            continue;
        }
        if let Some(z) = drop_edge(cutter, p, q, x, y, r) {
            keep(&mut best, z);
        }
    }
    best
}

/// The highest tip at which a ball of radius `r` on the vertical through `(x, y)` touches the
/// line through `p` and `q` within the segment: the centre is `r` from the line, a quadratic
/// in the centre's height. `None` when the touching point is outside the segment (a corner
/// is then the contact) or the ball does not reach the line.
fn ball_edge(p: Vec3, q: Vec3, x: f64, y: f64, r: f64) -> Option<f64> {
    let e = q - p;
    let len = e.len();
    let u = e.normalized()?;
    let a = 1.0 - u.z * u.z;
    if a <= 1e-12 {
        return None;
    }
    let w0 = Vec3::new(x - p.x, y - p.y, -p.z);
    let wu = w0.dot(u);
    let b = 2.0 * (w0.z - wu * u.z);
    let c = w0.len2() - wu * wu - r * r;
    let disc = b * b - 4.0 * a * c;
    if !(disc >= 0.0) {
        return None;
    }
    let zc = (-b + disc.sqrt()) / (2.0 * a);
    let s = wu + zc * u.z;
    (s >= 0.0 && s <= len).then_some(zc - r)
}

fn drop_edge(cutter: &Cutter, p: Vec3, q: Vec3, x: f64, y: f64, r: f64) -> Option<f64> {
    let (ex, ey, ez) = (q.x - p.x, q.y - p.y, q.z - p.z);
    let a = ex * ex + ey * ey;
    if a <= 1e-24 {
        // A vertical edge: its top corner is the highest point of it the cutter can meet.
        return None;
    }
    let (wx, wy) = (p.x - x, p.y - y);
    let b = 2.0 * (ex * wx + ey * wy);
    let c = wx * wx + wy * wy - r * r;
    let disc = b * b - 4.0 * a * c;
    if !(disc >= 0.0) {
        return None;
    }
    let root = disc.sqrt();
    let lo = ((-b - root) / (2.0 * a)).max(0.0);
    let hi = ((-b + root) / (2.0 * a)).min(1.0);
    if !(lo <= hi) {
        return None;
    }
    let f = |t: f64| {
        let d = (wx + ex * t).hypot(wy + ey * t);
        // The ends of the stretch are where `d` reaches the radius; rounding may put them a
        // hair outside it.
        let h = cutter.height(d.min(r)).unwrap_or(cutter.corner);
        p.z + ez * t - h
    };
    if cutter.corner <= 0.0 {
        return Some(f(lo).max(f(hi)));
    }
    if cutter.is_ball() {
        // The ends of the stretch are corners or rim points, which the corner test and `f`
        // cover; the closed form gives the inside.
        return Some(ball_edge(p, q, x, y, r).unwrap_or(f64::NEG_INFINITY).max(f(lo)).max(f(hi)));
    }
    let g = 0.5 * (5f64.sqrt() - 1.0);
    let (mut a0, mut b0) = (lo, hi);
    let (mut x1, mut x2) = (b0 - g * (b0 - a0), a0 + g * (b0 - a0));
    let (mut f1, mut f2) = (f(x1), f(x2));
    for _ in 0..EDGE_STEPS {
        if f1 < f2 {
            a0 = x1;
            (x1, f1) = (x2, f2);
            x2 = a0 + g * (b0 - a0);
            f2 = f(x2);
        } else {
            b0 = x2;
            (x2, f2) = (x1, f1);
            x1 = b0 - g * (b0 - a0);
            f1 = f(x1);
        }
    }
    Some(f(0.5 * (a0 + b0)).max(f(lo)).max(f(hi)))
}

/// Whether `(x, y)` is inside the triangle's outline seen from above (edges included).
fn inside_xy(t: &[Vec3; 3], x: f64, y: f64) -> bool {
    // Measured from the first corner, so the slack scales with the triangle, not with how far
    // it is from the origin.
    let o = t[0];
    let side = |a: Vec3, b: Vec3| ((b.x - o.x) - (a.x - o.x)) * ((y - o.y) - (a.y - o.y)) - ((b.y - o.y) - (a.y - o.y)) * ((x - o.x) - (a.x - o.x));
    let (s0, s1, s2) = (side(t[0], t[1]), side(t[1], t[2]), side(t[2], t[0]));
    let longest = [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])].iter().map(|(a, b)| (b.x - a.x).hypot(b.y - a.y)).fold(0.0, f64::max);
    let scale = 1e-12 * longest * longest;
    (s0 >= -scale && s1 >= -scale && s2 >= -scale) || (s0 <= scale && s1 <= scale && s2 <= scale)
}

/// The tip height at which `cutter`, its axis vertical through `(x, y)`, first touches the
/// surface coming down from above; `None` when no triangle is under it.
pub fn drop_z(surface: &Surface, cutter: &Cutter, x: f64, y: f64) -> Option<f64> {
    let r = cutter.radius;
    // The surface straight under the axis touches the cutter's centre, so the tip stops at
    // least that high: starting from it lets most triangles be passed over unopened.
    let top = surface.bounds().max.z + 1.0;
    let under = surface.raycast(Vec3::new(x, y, top), Vec3::new(0.0, 0.0, -1.0)).map_or(f64::NEG_INFINITY, |(t, _)| top - t);
    let best = Cell::new(under);
    surface.visit(
        |b| b.max.x >= x - r && b.min.x <= x + r && b.max.y >= y - r && b.min.y <= y + r && b.max.z > best.get(),
        |_, t| {
            if t.iter().all(|p| p.z <= best.get()) {
                return;
            }
            if let Some(z) = drop_triangle_above(cutter, t, x, y, best.get())
                && z > best.get()
            {
                best.set(z);
            }
        },
    );
    let z = best.get();
    z.is_finite().then_some(z)
}

/// Where the tip stops when `cutter`, its axis along the unit-or-not direction `axis` through
/// `point` (the tool body on the `+axis` side), first touches the surface coming in from
/// `+axis` far away: the distance along the unit axis from `point` (negative: behind it).
/// `None` when nothing is in the cutter's way.
pub fn drop_along(surface: &Surface, cutter: &Cutter, point: Vec3, axis: Vec3) -> Result<Option<f64>> {
    let axis = unit(axis, "the tool axis")?;
    if !point.is_finite() {
        return invalid("the point the tool is dropped through must be finite");
    }
    let frame = Frame::from_z(point, axis)?;
    let bounds = surface.bounds();
    if bounds.is_empty() {
        return Ok(None);
    }
    let reach = point.dist(bounds.center()) + bounds.diagonal() + cutter.radius + 1.0;
    let (a, b) = (point - axis * reach, point + axis * reach);
    let r = cutter.radius;
    // As in `drop_z`, the surface on the axis line is a height the tip reaches at least.
    let under = surface.raycast(b, -axis).map_or(f64::NEG_INFINITY, |(t, _)| reach - t);
    let best = Cell::new(under);
    surface.visit(
        |bx| {
            // The farthest the box reaches along the axis: nothing in it can stop the tool
            // higher than that.
            let c = bx.center() - point;
            let h = bx.size() * 0.5;
            let top = axis.dot(c) + axis.x.abs() * h.x + axis.y.abs() * h.y + axis.z.abs() * h.z;
            top > best.get() && capsule_meets_box(a, b, r, bx)
        },
        |_, t| {
            let local = t.map(|p| frame.to_local(p));
            if local.iter().all(|p| p.z <= best.get()) {
                return;
            }
            if let Some(z) = drop_triangle_above(cutter, &local, 0.0, 0.0, best.get())
                && z > best.get()
            {
                best.set(z);
            }
        },
    );
    let z = best.get();
    Ok(z.is_finite().then_some(z))
}
