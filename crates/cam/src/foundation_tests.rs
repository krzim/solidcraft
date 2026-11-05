//! Tests of frames, cutters, surfaces, the drop-cutter and multi-axis toolpaths, and the test
//! surfaces the other multi-axis tests share.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use solidcore_record::{Loop2, Region2, Vec2, Vec3};
use solidcraft_geom::Mesh;
use solidcraft_geom::prelude::*;

use super::*;

pub(crate) fn close(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} is not within {tol} of {b}");
}

pub(crate) fn close3(a: Vec3, b: Vec3, tol: f64) {
    assert!(a.dist(b) <= tol, "{a:?} is not within {tol} of {b:?}");
}

/// A square `[-s, s]²` at height `z`, as two triangles facing up.
pub(crate) fn plane(s: f64, z: f64) -> Surface {
    let p = |x: f64, y: f64| Vec3::new(x, y, z);
    Surface::new(vec![[p(-s, -s), p(s, -s), p(s, s)], [p(-s, -s), p(s, s), p(-s, s)]]).unwrap()
}

/// The plane `z = x·tan(slope)` over `[-s, s]²`.
pub(crate) fn slope(s: f64, slope: f64) -> Surface {
    let m = slope.tan();
    let p = |x: f64, y: f64| Vec3::new(x, y, x * m);
    Surface::new(vec![[p(-s, -s), p(s, -s), p(s, s)], [p(-s, -s), p(s, s), p(-s, s)]]).unwrap()
}

/// A sphere of radius `r` about `c`, `n` bands from pole to pole, wound outwards, with exact
/// vertex normals.
pub(crate) fn sphere_mesh(c: Vec3, r: f64, n: usize) -> Mesh {
    let (rings, segs) = (n, 2 * n);
    let mut m = Mesh::default();
    for i in 0..=rings {
        let phi = PI * i as f64 / rings as f64;
        for j in 0..segs {
            let th = TAU * j as f64 / segs as f64;
            let d = Vec3::new(phi.sin() * th.cos(), phi.sin() * th.sin(), phi.cos());
            m.positions.push(c + d * r);
            m.normals.push(d);
        }
    }
    let at = |i: usize, j: usize| (i * segs + j % segs) as u32;
    for i in 0..rings {
        for j in 0..segs {
            let (a, b, cc, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
            m.triangles.push([a, b, cc]);
            m.triangles.push([a, cc, d]);
            m.tri_face.push(0);
            m.tri_face.push(0);
        }
    }
    m
}

pub(crate) fn sphere(c: Vec3, r: f64, n: usize) -> Surface {
    Surface::from_mesh(&sphere_mesh(c, r, n)).unwrap()
}

/// The outside of a cylinder of radius `r` about the X axis from `x0` to `x1`, `n` facets
/// round, with exact vertex normals.
pub(crate) fn cylinder_x(r: f64, x0: f64, x1: f64, n: usize) -> Surface {
    let mut m = Mesh::default();
    for j in 0..n {
        let th = TAU * j as f64 / n as f64;
        let d = Vec3::new(0.0, th.cos(), th.sin());
        for x in [x0, x1] {
            m.positions.push(Vec3::new(x, 0.0, 0.0) + d * r);
            m.normals.push(d);
        }
    }
    for j in 0..n {
        let k = (j + 1) % n;
        let (a, b, c, d) = ((2 * j) as u32, (2 * j + 1) as u32, (2 * k + 1) as u32, (2 * k) as u32);
        m.triangles.push([a, d, c]);
        m.triangles.push([a, c, b]);
        m.tri_face.push(0);
        m.tri_face.push(0);
    }
    Surface::from_mesh(&m).unwrap()
}

/// A closed box from `lo` to `hi`, wound outwards.
pub(crate) fn block(lo: Vec3, hi: Vec3) -> Surface {
    let p = |i: usize| Vec3::new(if i & 1 == 0 { lo.x } else { hi.x }, if i & 2 == 0 { lo.y } else { hi.y }, if i & 4 == 0 { lo.z } else { hi.z });
    let quads = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];
    let mut tris = Vec::new();
    for q in quads {
        tris.push([p(q[0]), p(q[1]), p(q[2])]);
        tris.push([p(q[0]), p(q[2]), p(q[3])]);
    }
    Surface::new(tris).unwrap()
}

/// The frame tilted from the world by `a` about X then `b` about the turned Y, at `origin`.
pub(crate) fn tilted(origin: Vec3, a: f64, b: f64) -> Frame {
    let x = Vec3::new(1.0, 0.0, 0.0);
    let y = rotate(Vec3::new(0.0, 1.0, 0.0), x, a);
    let z = rotate(Vec3::new(0.0, 0.0, 1.0), x, a);
    Frame { origin, x: rotate(x, y, b), y, z: rotate(z, y, b) }
}

#[test]
fn frames_go_both_ways() {
    let f = tilted(Vec3::new(3.0, -2.0, 7.0), 0.4, -1.1);
    assert!(f.is_orthonormal());
    let p = Vec3::new(1.5, 2.5, -4.0);
    close3(f.to_world(f.to_local(p)), p, 1e-12);
    close3(f.dir_to_world(f.dir_to_local(p)), p, 1e-12);
    let inv = f.inverse();
    close3(inv.to_world(p), f.to_local(p), 1e-12);
    let g = tilted(Vec3::new(-1.0, 0.0, 2.0), -0.3, 0.8);
    close3(f.compose(&g).to_world(p), f.to_world(g.to_world(p)), 1e-12);
    let z = Frame::from_z(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    assert!(z.is_orthonormal());
    assert!(Frame::from_z(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0)).is_err());
    assert!(Frame::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 2.0)).is_err());
    close(angle_about(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 5.0), Vec3::new(0.0, 0.0, 1.0)).unwrap(), FRAC_PI_2, 1e-12);
    assert!(angle_about(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).is_none());
    close3(rotate_about(Vec3::new(2.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), PI), Vec3::new(0.0, 0.0, 0.0), 1e-12);
}

#[test]
fn cutter_profiles() {
    let flat = Cutter::flat(5.0).unwrap();
    let ball = Cutter::ball(5.0).unwrap();
    let bull = Cutter::bull(5.0, 1.0).unwrap();
    assert_eq!(flat.height(4.9), Some(0.0));
    assert_eq!(flat.height(5.1), None);
    close(ball.height(3.0).unwrap(), 1.0, 1e-12);
    close(ball.height(5.0).unwrap(), 5.0, 1e-12);
    assert_eq!(bull.height(3.9), Some(0.0));
    close(bull.height(5.0).unwrap(), 1.0, 1e-12);
    assert!(Cutter::bull(5.0, 6.0).is_err());
    assert!(Cutter::flat(f64::NAN).is_err());
    let grown = ball.grown(0.5).unwrap();
    assert!(grown.is_ball() && (grown.radius - 5.5).abs() < 1e-12);
    assert!(ball.grown(-1.0).is_err());

    let mut t = Tool::new(3, ToolKind::BullNose, 10.0, 20.0);
    assert!(t.validate().is_err(), "a bull nose needs its corner radius");
    t.corner_radius = Some(1.5);
    t.validate().unwrap();
    assert_eq!(t.cutter().unwrap(), Cutter::bull(5.0, 1.5).unwrap());
    t.stickout = Some(40.0);
    t.shank_diameter = Some(8.0);
    t.holder = vec![HolderSection { diameter: 30.0, length: 25.0 }, HolderSection { diameter: 50.0, length: 40.0 }];
    t.validate().unwrap();
    let e = t.envelopes();
    assert_eq!(e.len(), 3);
    assert_eq!((e[0].bottom, e[0].top, e[0].radius), (20.0, 40.0, 4.0));
    assert_eq!((e[2].bottom, e[2].top, e[2].radius), (65.0, 105.0, 25.0));
    t.stickout = Some(10.0);
    assert!(t.validate().is_err(), "the stickout cannot be shorter than the flutes");
    assert!(Tool::new(1, ToolKind::Drill, 6.0, 30.0).cutter().is_err());
    // An old tool without the new fields still reads.
    let old: Tool = serde_json::from_str(r#"{"number":1,"kind":"flat_endmill","diameter":6.0,"flute_length":20.0}"#).unwrap();
    assert!(old.holder.is_empty() && old.corner_radius.is_none());
}

#[test]
fn drop_onto_a_flat_and_a_slope() {
    let p = plane(50.0, 3.0);
    for c in [Cutter::flat(4.0).unwrap(), Cutter::ball(4.0).unwrap(), Cutter::bull(4.0, 1.0).unwrap()] {
        close(drop_z(&p, &c, 7.0, -3.0).unwrap(), 3.0, 1e-9);
        // Half over the edge still sits on the face's edge.
        close(drop_z(&p, &c, 51.0, 0.0).unwrap(), 3.0 - c.height(1.0).unwrap(), 1e-9);
        assert!(drop_z(&p, &c, 60.0, 0.0).is_none());
    }
    let th: f64 = 0.5;
    let s = slope(100.0, th);
    let (r, x) = (4.0, 10.0);
    // A ball's centre is r from the plane: its tip r/cos − r above the plane under its axis.
    let ball = Cutter::ball(r).unwrap();
    close(drop_z(&s, &ball, x, 2.0).unwrap(), x * th.tan() + r / th.cos() - r, 1e-9);
    // A flat end rests on its rim's highest point.
    let flat = Cutter::flat(r).unwrap();
    close(drop_z(&s, &flat, x, 2.0).unwrap(), (x + r) * th.tan(), 1e-9);
    // A bull nose: its corner circle's centre r_c from the plane.
    let (rc, a) = (1.0, 3.0);
    let bull = Cutter::bull(r, rc).unwrap();
    close(drop_z(&s, &bull, x, 2.0).unwrap(), (rc + th.sin() * (x + a)) / th.cos() - rc, 1e-9);
}

#[test]
fn drop_onto_an_edge() {
    // A wall in the plane x = 0 whose top edge is level at z = 10.
    let w = |y: f64, z: f64| Vec3::new(0.0, y, z);
    let wall = Surface::new(vec![[w(-50.0, 0.0), w(50.0, 0.0), w(50.0, 10.0)], [w(-50.0, 0.0), w(50.0, 10.0), w(-50.0, 10.0)]]).unwrap();
    let (r, rc) = (5.0, 2.0);
    for d in [0.0, 1.0, 2.5, 3.5, 4.5, 4.99] {
        let ball = drop_z(&wall, &Cutter::ball(r).unwrap(), d, 7.0).unwrap();
        close(ball, 10.0 + (r * r - d * d).sqrt() - r, 1e-9);
        let bull = drop_z(&wall, &Cutter::bull(r, rc).unwrap(), -d, 7.0).unwrap();
        let a = r - rc;
        let want = if d <= a { 10.0 } else { 10.0 + (rc * rc - (d - a) * (d - a)).sqrt() - rc };
        close(bull, want, 1e-9);
        close(drop_z(&wall, &Cutter::flat(r).unwrap(), d, 7.0).unwrap(), 10.0, 1e-9);
    }
    assert!(drop_z(&wall, &Cutter::ball(r).unwrap(), 5.5, 7.0).is_none());
}

#[test]
fn drop_onto_a_sphere_is_within_its_chords() {
    let (rs, r) = (20.0, 3.0);
    let s = sphere(Vec3::new(0.0, 0.0, 0.0), rs, 64);
    // The facets lie inside the sphere by at most its chord error.
    let chord = rs * (1.0 - (PI / 64.0).cos()) * 2.0;
    let ball = Cutter::ball(r).unwrap();
    for rho in [0.0, 5.0, 12.0, 19.0, 22.5] {
        let want = ((rs + r) * (rs + r) - rho * rho).sqrt() - r;
        let got = drop_z(&s, &ball, rho * 0.6, rho * 0.8).unwrap();
        assert!(got <= want + 1e-9 && got >= want - chord, "ρ {rho}: {got} vs {want}");
    }
}

#[test]
fn drop_along_any_axis_agrees_with_dropping_down() {
    let s = sphere(Vec3::new(1.0, 2.0, -3.0), 15.0, 32);
    let f = tilted(Vec3::new(5.0, -4.0, 2.0), 0.7, -0.4);
    // The same surface turned into a tilted frame, dropped along that frame's Z.
    let turned = Surface::new(s.triangles().iter().map(|t| t.map(|p| f.to_world(p))).collect()).unwrap();
    for c in [Cutter::flat(4.0).unwrap(), Cutter::ball(4.0).unwrap(), Cutter::bull(4.0, 1.5).unwrap()] {
        for (x, y) in [(0.0, 0.0), (6.0, -3.0), (-12.0, 9.0)] {
            let down = drop_z(&s, &c, x, y).unwrap();
            let along = drop_along(&turned, &c, f.to_world(Vec3::new(x, y, 0.0)), f.z).unwrap().unwrap();
            close(along, down, 1e-7);
            // The same with the axis given through another point on the line.
            let higher = drop_along(&turned, &c, f.to_world(Vec3::new(x, y, 40.0)), f.z * 3.0).unwrap().unwrap();
            close(higher, down - 40.0, 1e-7);
        }
    }
    let ball = Cutter::ball(4.0).unwrap();
    assert!(drop_along(&turned, &ball, f.to_world(Vec3::new(100.0, 0.0, 0.0)), f.z).unwrap().is_none());
    assert!(drop_along(&turned, &ball, Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0)).is_err());
    assert!(drop_along(&turned, &ball, Vec3::new(f64::NAN, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).is_err());
    // From underneath, the tool stops below the sphere.
    let under = drop_along(&s, &ball, Vec3::new(1.0, 2.0, -3.0), Vec3::new(0.0, 0.0, -1.0)).unwrap().unwrap();
    close(under, 15.0, 0.1);
}

#[test]
fn surfaces_from_meshes() {
    let m = sphere_mesh(Vec3::new(0.0, 0.0, 0.0), 10.0, 16);
    let s = Surface::from_mesh(&m).unwrap();
    assert!(!s.is_empty() && s.len() <= m.triangles.len(), "the pole's slivers are left out");
    let b = s.bounds();
    close(b.max.z, 10.0, 1e-9);
    // A ray from far above meets the top.
    let (t, i) = s.raycast(Vec3::new(0.3, 0.2, 50.0), Vec3::new(0.0, 0.0, -1.0)).unwrap();
    let hit = Vec3::new(0.3, 0.2, 50.0 - t);
    assert!(hit.z > 9.5);
    let n = s.normal_at(i, hit).unwrap();
    assert!(n.z > 0.99, "the blended normal points out: {n:?}");
    let mut bad = m.clone();
    bad.triangles.push([0, 1, 999_999]);
    assert!(Surface::from_mesh(&bad).is_err());
    assert!(Surface::new(vec![[Vec3::new(f64::INFINITY, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)]]).is_err());
    let f = tilted(Vec3::new(1.0, 1.0, 1.0), 0.3, 0.2);
    let local = s.in_frame(&f).unwrap();
    close3(local.bounds().center(), f.to_local(Vec3::new(0.0, 0.0, 0.0)), 0.5);
    // Geometry helpers.
    let tri = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0), Vec3::new(0.0, 4.0, 0.0)];
    close3(nearest_on_triangle(Vec3::new(1.0, 1.0, 5.0), &tri), Vec3::new(1.0, 1.0, 0.0), 1e-12);
    close3(nearest_on_triangle(Vec3::new(-2.0, -2.0, 0.0), &tri), tri[0], 1e-12);
    close(segment_triangle_distance(Vec3::new(1.0, 1.0, 3.0), Vec3::new(1.0, 1.0, 1.0), &tri), 1.0, 1e-12);
    close(segment_triangle_distance(Vec3::new(1.0, 1.0, 3.0), Vec3::new(1.0, 1.0, -1.0), &tri), 0.0, 1e-12);
    close(segment_triangle_distance(Vec3::new(6.0, 0.0, -1.0), Vec3::new(6.0, 0.0, 1.0), &tri), 2.0, 1e-12);
    let w = barycentric(&tri, Vec3::new(1.0, 2.0, 0.0)).unwrap();
    close(w[0] + w[1] + w[2], 1.0, 1e-12);
}

#[test]
fn multiaxis_paths_from_3axis_ones() {
    let square =
        Region2 { outer: Loop2::polygon(&[Vec2::new(0.0, 0.0), Vec2::new(30.0, 0.0), Vec2::new(30.0, 20.0), Vec2::new(0.0, 20.0)]), holes: vec![] };
    let tool = Tool::new(1, ToolKind::FlatEndmill, 6.0, 20.0);
    let cut =
        Cut { heights: Heights { clearance: 10.0, retract: 2.0, top: 0.0, bottom: -3.0 }, feeds: Feeds { cut: 600.0, plunge: 200.0 }, stepdown: 1.5 };
    let path =
        contour(&[square], &tool, &ContourParams { side: Side::Outside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.05 }, &cut)
            .unwrap();
    let p5 = Toolpath5::from_3axis(&path, 0.001).unwrap();
    verify5(&p5, 1e-9).unwrap();
    let (a, b) = (path.stats(), p5.stats());
    close(b.cut_length, a.cut_length, 0.05);
    close(b.cut_minutes, a.cut_minutes, 0.001);
    assert_eq!(b.max_tilt, 0.0);
    // Tilted, the tips are the tilted frame's and the axis its Z.
    let f = tilted(Vec3::new(10.0, 0.0, 5.0), 0.5, 0.0);
    let t5 = Toolpath5::from_frame(&path, &f, 0.001).unwrap();
    close(t5.stats().max_tilt, 0.5, 1e-12);
    for (pose, local) in t5.poses().zip(p5.poses()) {
        close3(pose.tip, f.to_world(local.tip), 1e-9);
        close3(pose.axis, f.z, 1e-12);
    }
    // The checks catch what must not leave.
    let start = Move5::Rapid { to: Pose::vertical(Vec3::new(0.0, 0.0, 10.0)) };
    let tilt = |a: f64| Pose::new(Vec3::new(1.0, 0.0, 0.0), Vec3::new(a.sin(), 0.0, a.cos()));
    assert!(verify5(&Toolpath5 { moves: vec![start, Move5::Feed { to: tilt(0.2), feed: 100.0 }] }, 0.1).is_err());
    verify5(&Toolpath5 { moves: vec![start, Move5::Feed { to: tilt(0.05), feed: 100.0 }] }, 0.1).unwrap();
    assert!(verify5(&Toolpath5 { moves: vec![Move5::Feed { to: tilt(0.0), feed: 100.0 }] }, 0.1).is_err());
    let bad = Pose::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 2.0));
    assert!(verify5(&Toolpath5 { moves: vec![Move5::Rapid { to: bad }] }, 0.1).is_err());
    assert!(verify5(&Toolpath5 { moves: vec![start, Move5::Feed { to: tilt(0.0), feed: 0.0 }] }, 0.1).is_err());
    // Turning evenly between two directions.
    let (x, z) = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    let mid = slerp(z, x, 0.5);
    close3(mid, Vec3::new(1.0, 0.0, 1.0).normalized().unwrap(), 1e-12);
    close((slerp(z, -z, 0.5).len() - 1.0).abs(), 0.0, 1e-12);
    let json = serde_json::to_string(&t5).unwrap();
    assert_eq!(serde_json::from_str::<Toolpath5>(&json).unwrap(), t5);
}

#[test]
fn drop_onto_test_shapes() {
    let b = block(Vec3::new(-10.0, -10.0, 0.0), Vec3::new(10.0, 10.0, 5.0));
    let ball = Cutter::ball(3.0).unwrap();
    close(drop_z(&b, &ball, 0.0, 0.0).unwrap(), 5.0, 1e-9);
    // Past the side, the ball rolls over the top edge.
    close(drop_z(&b, &ball, 12.0, 0.0).unwrap(), 5.0 + (9.0f64 - 4.0).sqrt() - 3.0, 1e-9);
    // A cylinder about X: dropped radially at any angle, the ball stops r_cyl + r from the axis.
    let c = cylinder_x(20.0, -50.0, 50.0, 256);
    for a in [0.0, 0.7, 2.0, -2.5] {
        let dir = Vec3::new(0.0, f64::cos(a), f64::sin(a));
        let t = drop_along(&c, &ball, Vec3::new(5.0, 0.0, 0.0), dir).unwrap().unwrap();
        assert!((20.0 - 0.01..=20.0 + 1e-9).contains(&t), "{t}");
    }
}

/// Random triangles against a brute-force drop: the highest tip touching any of a fine grid of
/// points on the triangle. The drop must never be below it (a gouge) and only above it by the
/// grid's own coarseness.
#[test]
fn drop_triangle_matches_brute_force() {
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut rnd = |lo: f64, hi: f64| {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        lo + (hi - lo) * ((seed >> 11) as f64 / (1u64 << 53) as f64)
    };
    let cutters = [Cutter::flat(3.0).unwrap(), Cutter::ball(3.0).unwrap(), Cutter::bull(3.0, 1.0).unwrap()];
    let n = 120;
    for _ in 0..150 {
        let t = [0, 1, 2].map(|_| Vec3::new(rnd(-6.0, 6.0), rnd(-6.0, 6.0), rnd(-4.0, 4.0)));
        let (x, y) = (rnd(-4.0, 4.0), rnd(-4.0, 4.0));
        for c in &cutters {
            let mut brute = f64::NEG_INFINITY;
            for i in 0..=n {
                for j in 0..=(n - i) {
                    let (u, v) = (i as f64 / n as f64, j as f64 / n as f64);
                    let p = t[0] + (t[1] - t[0]) * u + (t[2] - t[0]) * v;
                    if let Some(h) = c.height((p.x - x).hypot(p.y - y)) {
                        brute = brute.max(p.z - h);
                    }
                }
            }
            let got = drop_triangle(c, &t, x, y);
            if !brute.is_finite() {
                // Only a sliver of the triangle may be under the cutter, between grid points.
                continue;
            }
            let got = got.expect("the cutter reaches the triangle");
            assert!(got >= brute - 1e-9, "gouge: {got} below {brute} for {c:?} at ({x}, {y}) on {t:?}");
            assert!(got <= brute + 0.35, "{got} far above {brute} for {c:?} at ({x}, {y}) on {t:?}");
        }
    }
}

/// How long a drop takes on a fine sphere (run with `--ignored --nocapture` in release).
#[test]
#[ignore]
fn drop_speed() {
    let s = sphere(Vec3::new(0.0, 0.0, 0.0), 30.0, 136);
    let c = Cutter::ball(3.0).unwrap();
    let b = Cutter::bull(3.0, 1.0).unwrap();
    for (name, c) in [("ball", c), ("bull", b)] {
        let start = std::time::Instant::now();
        let mut k = 0;
        for i in 0..60 {
            for j in 0..60 {
                if drop_z(&s, &c, -30.0 + i as f64, -30.0 + j as f64).is_some() {
                    k += 1;
                }
            }
        }
        println!("{name}: {} triangles, {k} drops, {:.1} µs per drop", s.len(), start.elapsed().as_secs_f64() * 1e6 / 3600.0);
    }
}

/// A wall leaning by a hair, far from the origin: the face test must not accept a contact
/// outside the face and lift the tip far above it.
#[test]
fn near_vertical_faces_far_from_the_origin() {
    for (o, k) in [(100.0, 1e-11), (100.0, 1e-10), (500.0, 1e-9), (500.0, 1e-8), (0.0, 2e-12), (300.0, 1e-9)] {
        let t = [Vec3::new(o, o, 0.0), Vec3::new(o, o + 10.0, 0.0), Vec3::new(o + k * 10.0, o, 10.0)];
        for c in [Cutter::flat(3.0).unwrap(), Cutter::ball(3.0).unwrap(), Cutter::bull(3.0, 1.0).unwrap()] {
            for x in [o - 3.0 + 4.4531e-9, o - 2.5, o - 1.0] {
                if let Some(z) = drop_triangle(&c, &t, x, o + 9.9) {
                    assert!(z <= 10.0 + 1e-9, "tip {z} above the wall's top for {c:?} at x {x}");
                }
            }
        }
    }
    // A hint almost along Z is refused rather than giving a frame that is not square.
    let z = Vec3::new(0.36, 0.48, 0.8);
    assert!(Frame::new(Vec3::new(0.0, 0.0, 0.0), z, z * 1000.0 + Vec3::new(0.8, -0.6, 0.0) * 1e-9).is_err());
    let f = Frame::new(Vec3::new(0.0, 0.0, 0.0), z, z + Vec3::new(0.8, -0.6, 0.0) * 1e-3).unwrap();
    assert!(f.is_orthonormal());
}
