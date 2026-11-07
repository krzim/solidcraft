//! Tests of adaptive clearing: the engagement stays within the limit (measured independently),
//! the region is cleared to the walls, the 3D roughing stays off the part, and hostile input
//! is refused without hanging.

use std::f64::consts::PI;

use solidcore_record::{Loop2, Region2, Seg2, Vec2, Vec3};
use solidcraft_geom::prelude::*;

use crate::adaptive::{AdaptiveParams, AdaptivePath, AdaptiveRoughingParams, MoveRole, StockModel, adaptive, adaptive_in, adaptive_roughing};
use crate::dropcutter::drop_z;
use crate::engagement::{Material, engagement};
use crate::ops::{Cut, Feeds, Heights, PocketParams, pocket};
use crate::path::{Move, Toolpath};
use crate::surface::Surface;
use crate::surface_ops::Boundary;
use crate::tool::{Tool, ToolKind};
use crate::{CamError, Cutter, offset_regions, verify};

const DEG: f64 = PI / 180.0;
/// How far above the limit the independent measurement may find the engagement (degrees): the
/// planner works on a raster a sixteenth of the radius fine.
const ENGAGEMENT_TOL: f64 = 3.0;

fn flat(d: f64) -> Tool {
    Tool { flutes: 3, ..Tool::new(1, ToolKind::FlatEndmill, d, 20.0) }
}

fn poly(pts: &[(f64, f64)]) -> Loop2 {
    let p: Vec<Vec2> = pts.iter().map(|&(x, y)| Vec2::new(x, y)).collect();
    Loop2 { segs: (0..p.len()).map(|i| Seg2::Line { a: p[i], b: p[(i + 1) % p.len()] }).collect() }
}

fn region(outer: &[(f64, f64)], holes: &[&[(f64, f64)]]) -> Region2 {
    Region2 { outer: poly(outer), holes: holes.iter().map(|h| poly(h)).collect() }
}

fn square(s: f64) -> Region2 {
    region(&[(0.0, 0.0), (s, 0.0), (s, s), (0.0, s)], &[])
}

fn ell() -> Region2 {
    region(&[(0.0, 0.0), (40.0, 0.0), (40.0, 16.0), (16.0, 16.0), (16.0, 36.0), (0.0, 36.0)], &[])
}

fn with_island() -> Region2 {
    region(&[(0.0, 0.0), (40.0, 0.0), (40.0, 30.0), (0.0, 30.0)], &[&[(14.0, 10.0), (14.0, 20.0), (26.0, 20.0), (26.0, 10.0)]])
}

fn params() -> AdaptiveParams {
    AdaptiveParams {
        optimal_load: 0.6,
        min_engagement: 10.0 * DEG,
        max_engagement: 60.0 * DEG,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        helix_diameter: 4.8,
        ramp_angle: 3.0 * DEG,
        climb: true,
    }
}

fn cut(top: f64, bottom: f64) -> Cut {
    Cut { heights: Heights { clearance: 20.0, retract: 2.0, top, bottom }, feeds: Feeds { cut: 1000.0, plunge: 200.0 }, stepdown: 3.0 }
}

/// Every point of the region within reach of where the centre may go (less `slack`) is within
/// the radius of a cutting move at the bottom: a raster of its own, exact distances.
fn assert_cleared(regions: &[Region2], path: &Toolpath, r: f64, leave: f64, bottom: f64, slack: f64) {
    let centre = offset_regions(regions, r + leave, 1e-3).unwrap();
    let rings: Vec<Vec<Vec2>> = centre.iter().map(|l| l.segs.iter().flat_map(|s| s.polyline(1e-3)).collect()).collect();
    let inside = |q: Vec2| {
        let mut odd = false;
        for ring in &rings {
            for (a, b) in ring.iter().zip(ring.iter().cycle().skip(1)) {
                if (a.y > q.y) != (b.y > q.y) && q.x < a.x + (q.y - a.y) / (b.y - a.y) * (b.x - a.x) {
                    odd = !odd;
                }
            }
        }
        odd
    };
    let seg = |p: Vec2, a: Vec2, b: Vec2| {
        let d = b - a;
        let t = if d.len2() > 0.0 { ((p - a).dot(d) / d.len2()).clamp(0.0, 1.0) } else { 0.0 };
        p.dist(a + d * t)
    };
    let to_c = |q: Vec2| {
        rings.iter().flat_map(|ring| ring.iter().zip(ring.iter().cycle().skip(1))).map(|(a, b)| seg(q, *a, *b)).fold(f64::INFINITY, f64::min)
    };
    let pts = path.points(1e-3);
    let mut cuts: Vec<(Vec2, Vec2)> = Vec::new();
    for w in pts.windows(2) {
        let ((a, _), (b, cutting)) = (w[0], w[1]);
        if cutting && (a.z - bottom).abs() < 1e-6 && (b.z - bottom).abs() < 1e-6 {
            cuts.push((Vec2::new(a.x, a.y), Vec2::new(b.x, b.y)));
        }
    }
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for r in regions {
        for s in &r.outer.segs {
            for p in s.polyline(1e-3) {
                lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
                hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
            }
        }
    }
    let step = 0.25;
    let (mut checked, mut missed) = (0, Vec::new());
    let mut y = lo.y + 0.13;
    while y < hi.y {
        let mut x = lo.x + 0.07;
        while x < hi.x {
            let q = Vec2::new(x, y);
            if inside(q) || to_c(q) <= r - slack {
                checked += 1;
                if !cuts.iter().any(|(a, b)| seg(q, *a, *b) <= r + 1e-6) {
                    missed.push(q);
                }
            }
            x += step;
        }
        y += step;
    }
    assert!(checked > 100, "{checked}");
    assert!(missed.is_empty(), "{} of {checked} points left uncut, first {:?}", missed.len(), missed.first());
}

fn measure(regions: &[Region2], a: &AdaptivePath, r: f64, top: f64) -> crate::engagement::Engagement {
    engagement(&a.path, Some(&a.roles), r, &Material { regions: regions.to_vec(), top, within: None }, 0.1).unwrap()
}

fn check_pocket(regions: &[Region2], p: &AdaptiveParams) -> AdaptivePath {
    let tool = flat(6.0);
    let c = cut(0.0, -3.0);
    let a = adaptive(regions, &tool, p, &c).unwrap();
    verify(&a.path, &c.heights).unwrap();
    assert_eq!(a.roles.len(), a.path.moves.len());
    assert!(a.helices >= 1, "{a:?}");
    let e = measure(regions, &a, 3.0, 0.0);
    let limit = p.max_engagement / DEG;
    assert!(
        e.max_deg <= limit + ENGAGEMENT_TOL,
        "engagement {} above {limit}: {e:?} {:?} {:?}",
        e.max_deg,
        e.worst_move.map(|i| (a.roles[i], a.path.moves[i - 1], a.path.moves[i])),
        &a.path.moves[e.worst_move.unwrap_or(0).saturating_sub(6)..e.worst_move.unwrap_or(0) + 1]
    );
    assert_eq!(e.full_slot_share, 0.0, "{e:?}");
    assert!(e.mean_deg > 10.0, "{e:?}");
    assert_cleared(regions, &a.path, 3.0, p.stock_to_leave, -3.0, 0.4);
    a
}

#[test]
fn a_square_pocket_stays_within_the_engagement() {
    let a = check_pocket(&[square(30.0)], &params());
    // Entries are helices: arcs that go down.
    assert!(a.path.moves.iter().zip(&a.roles).any(|(m, r)| *r == MoveRole::Entry && matches!(m, Move::Arc { .. })));
}

#[test]
fn an_l_shaped_pocket_stays_within_the_engagement() {
    check_pocket(&[ell()], &params());
}

#[test]
fn a_pocket_with_an_island_stays_within_the_engagement() {
    let a = check_pocket(&[with_island()], &params());
    // Nothing comes within the radius of the island.
    for (p, cutting) in a.path.points(1e-3) {
        if cutting {
            let (dx, dy) = ((14.0 - p.x).max(p.x - 26.0).max(0.0), (10.0 - p.y).max(p.y - 20.0).max(0.0));
            assert!(dx.hypot(dy) >= 3.0 - 1e-6, "{p:?}");
        }
    }
}

#[test]
fn conventional_milling_and_stock_to_leave_still_clear_the_pocket() {
    let p = AdaptiveParams { climb: false, stock_to_leave: 0.5, ..params() };
    check_pocket(&[square(24.0)], &p);
}

/// Plain offset pocketing slots its first ring; the adaptive path never does.
#[test]
fn adaptive_against_offset_pocketing() {
    let regions = [square(30.0)];
    let tool = flat(6.0);
    let c = cut(0.0, -3.0);
    let a = adaptive(&regions, &tool, &params(), &c).unwrap();
    let pp = PocketParams { stepover: 2.4, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 };
    let plain = pocket(&regions, &tool, &pp, &c).unwrap();
    let material = Material { regions: regions.to_vec(), top: 0.0, within: None };
    let ea = engagement(&a.path, Some(&a.roles), 3.0, &material, 0.1).unwrap();
    let ep = engagement(&plain, None, 3.0, &material, 0.1).unwrap();
    assert!(ep.max_deg >= 170.0 && ep.full_slot_share > 0.0, "{ep:?}");
    assert_eq!(ea.full_slot_share, 0.0, "{ea:?}");
    assert!(ea.max_deg < ep.max_deg, "{ea:?} {ep:?}");
    // A lighter cut: a longer path.
    assert!(a.path.stats().cut_length > plain.stats().cut_length, "{} {}", a.path.stats().cut_length, plain.stats().cut_length);
}

/// Climb milling keeps the material on the right of the travel (spindle clockwise): the helices
/// go counter-clockwise and the walks round the cleared area turn counter-clockwise overall;
/// conventional milling the other way round, and the engagement holds either way.
#[test]
fn climb_and_conventional_turn_opposite_ways() {
    for climb in [true, false] {
        let p = AdaptiveParams { climb, ..params() };
        let a = adaptive(&[square(24.0)], &flat(6.0), &p, &cut(0.0, -1.0)).unwrap();
        let mut arcs = 0;
        for (m, role) in a.path.moves.iter().zip(&a.roles) {
            if let (Move::Arc { ccw, .. }, MoveRole::Entry) = (m, role) {
                assert_eq!(*ccw, climb, "a helix the wrong way round, climb {climb}");
                arcs += 1;
            }
        }
        assert!(arcs >= 2, "{arcs}");
        // The signed turn between consecutive cutting moves, summed.
        let mut turn = 0.0;
        let mut prev: Option<(Vec3, Option<Vec2>)> = None;
        for (m, role) in a.path.moves.iter().zip(&a.roles) {
            let Some(to) = m.target() else { continue };
            let dir = match (prev, role) {
                (Some((from, _)), MoveRole::Cut) => (Vec2::new(to.x, to.y) - Vec2::new(from.x, from.y)).normalized(),
                _ => None,
            };
            if let (Some((_, Some(d0))), Some(d1)) = (prev, dir) {
                turn += d0.cross(d1).atan2(d0.dot(d1));
            }
            prev = Some((to, dir));
        }
        let turns = turn / (2.0 * PI);
        assert!(if climb { turns > 1.0 } else { turns < -1.0 }, "climb {climb}: {turns} turns");
        let e = measure(&[square(24.0)], &a, 3.0, 0.0);
        assert!(e.max_deg <= 60.0 + ENGAGEMENT_TOL && e.full_slot_share == 0.0, "climb {climb}: {e:?}");
    }
}

/// A limit well below the load's engagement (a heavy load capped at 30°): the walk's steps and
/// raster follow the width the limit allows, not the load, so the pocket is cleared from its
/// helix within the limit.
#[test]
fn a_low_limit_under_a_heavy_load_still_clears_the_pocket() {
    let regions = [square(14.0)];
    let p = AdaptiveParams { optimal_load: 1.8, min_engagement: 3.0 * DEG, max_engagement: 30.0 * DEG, ..params() };
    let a = adaptive(&regions, &flat(6.0), &p, &cut(0.0, -1.0)).unwrap();
    assert_eq!(a.helices, 1, "{a:?}");
    let e = measure(&regions, &a, 3.0, 0.0);
    assert!(e.max_deg <= 30.0 + ENGAGEMENT_TOL, "{e:?}");
    assert_eq!(e.full_slot_share, 0.0, "{e:?}");
    assert_cleared(&regions, &a.path, 3.0, 0.0, -1.0, 0.4);
}

#[test]
fn the_helix_is_refused_without_room() {
    // 7 mm wide: the 6 mm tool's centre has a 1 mm strip, no room for a 4.8 mm helix.
    let slot = region(&[(0.0, 0.0), (40.0, 0.0), (40.0, 7.0), (0.0, 7.0)], &[]);
    let e = adaptive(std::slice::from_ref(&slot), &flat(6.0), &params(), &cut(0.0, -3.0)).unwrap_err();
    assert!(matches!(&e, CamError::Invalid(m) if m.contains("no room for the entry helix")), "{e}");
    // 10 mm wide: room for a 2 mm helix, not for the 4.8 mm one.
    let wide = region(&[(0.0, 0.0), (40.0, 0.0), (40.0, 10.0), (0.0, 10.0)], &[]);
    assert!(adaptive(std::slice::from_ref(&wide), &flat(6.0), &params(), &cut(0.0, -1.0)).is_err());
    let p = AdaptiveParams { helix_diameter: 2.0, ..params() };
    let a = adaptive(std::slice::from_ref(&wide), &flat(6.0), &p, &cut(0.0, -1.0)).unwrap();
    assert!(a.helices >= 1);
    let e = measure(&[wide], &a, 3.0, 0.0);
    assert!(e.max_deg <= 60.0 + ENGAGEMENT_TOL, "{e:?}");
    // A pocket the tool does not fit at all.
    assert!(adaptive(&[square(5.0)], &flat(6.0), &params(), &cut(0.0, -1.0)).is_err());
}

#[test]
fn hostile_parameters_are_refused() {
    let tool = flat(6.0);
    let regions = [square(20.0)];
    let c = cut(0.0, -1.0);
    let bad = [
        AdaptiveParams { optimal_load: 0.0, ..params() },
        AdaptiveParams { optimal_load: f64::NAN, ..params() },
        AdaptiveParams { optimal_load: 7.0, ..params() },
        AdaptiveParams { max_engagement: 0.0, ..params() },
        AdaptiveParams { max_engagement: 4.0, ..params() },
        AdaptiveParams { max_engagement: f64::NAN, ..params() },
        AdaptiveParams { min_engagement: 70.0 * DEG, ..params() },
        AdaptiveParams { min_engagement: -1.0, ..params() },
        AdaptiveParams { stock_to_leave: -1.0, ..params() },
        AdaptiveParams { stock_to_leave: f64::INFINITY, ..params() },
        AdaptiveParams { tolerance: 0.0, ..params() },
        AdaptiveParams { helix_diameter: 0.0, ..params() },
        AdaptiveParams { helix_diameter: 6.5, ..params() },
        AdaptiveParams { ramp_angle: 0.0, ..params() },
        AdaptiveParams { ramp_angle: 1.0, ..params() },
        AdaptiveParams { ramp_angle: f64::NAN, ..params() },
    ];
    for p in &bad {
        assert!(adaptive(&regions, &tool, p, &c).is_err(), "{p:?}");
    }
    let drill = Tool { kind: ToolKind::Drill, ..flat(6.0) };
    assert!(adaptive(&regions, &drill, &params(), &c).is_err());
    assert!(adaptive(&regions, &flat(f64::NAN), &params(), &c).is_err());
    assert!(adaptive(&[], &tool, &params(), &c).is_err());
    let nan = region(&[(0.0, 0.0), (f64::NAN, 0.0), (10.0, 10.0)], &[]);
    assert!(adaptive(&[nan], &tool, &params(), &c).is_err());
    let flat_region = region(&[(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)], &[]);
    assert!(adaptive(&[flat_region], &tool, &params(), &c).is_err());
    for h in [
        Heights { clearance: 1.0, retract: 2.0, top: 0.0, bottom: -1.0 },
        Heights { clearance: 20.0, retract: 2.0, top: 0.0, bottom: f64::NAN },
        Heights { clearance: 20.0, retract: 2.0, top: 0.0, bottom: -1e9 },
    ] {
        assert!(adaptive(&regions, &tool, &params(), &Cut { heights: h, ..c }).is_err(), "{h:?}");
    }
    assert!(adaptive(&regions, &tool, &params(), &Cut { stepdown: 0.0, ..c }).is_err());
    // A pass deeper than the flutes reach (20 mm) is refused; a stepdown larger than the whole
    // depth is not.
    let deep = Cut { heights: Heights { bottom: -25.0, ..c.heights }, stepdown: 25.0, ..c };
    let e = adaptive(&regions, &tool, &params(), &deep).unwrap_err();
    assert!(e.to_string().contains("flute length"), "{e}");
    assert!(adaptive(&regions, &tool, &params(), &Cut { stepdown: 50.0, ..c }).is_ok());
    assert!(adaptive(&regions, &tool, &params(), &Cut { feeds: Feeds { cut: 0.0, plunge: 1.0 }, ..c }).is_err());
}

/// Huge regions are refused before any work, not left running.
#[test]
fn huge_regions_are_refused_quickly() {
    let start = std::time::Instant::now();
    let e = adaptive(&[square(100_000.0)], &flat(6.0), &params(), &cut(0.0, -1.0)).unwrap_err();
    assert!(matches!(e, CamError::TooLarge(_)), "{e}");
    let e = adaptive(&[square(5.0e6)], &flat(6.0), &params(), &cut(0.0, -1.0)).unwrap_err();
    assert!(matches!(e, CamError::Invalid(_) | CamError::TooLarge(_)), "{e}");
    let surface = Surface::new(plate_and_boss()).unwrap();
    let mut p = roughing_params();
    p.boundary.rect = Some([Vec2::new(-50_000.0, -50_000.0), Vec2::new(50_000.0, 50_000.0)]);
    assert!(matches!(adaptive_roughing(&surface, &flat(6.0), &p).unwrap_err(), CamError::TooLarge(_)));
    assert!(start.elapsed().as_secs() < 5, "{:?}", start.elapsed());
}

/// The triangles of a box, wound outward.
fn box_tris(lo: Vec3, hi: Vec3, out: &mut Vec<[Vec3; 3]>) {
    let c = |i: usize| Vec3::new(if i & 1 == 0 { lo.x } else { hi.x }, if i & 2 == 0 { lo.y } else { hi.y }, if i & 4 == 0 { lo.z } else { hi.z });
    let quads = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];
    for q in quads {
        out.push([c(q[0]), c(q[1]), c(q[2])]);
        out.push([c(q[0]), c(q[2]), c(q[3])]);
    }
}

/// A 40 mm square plate, its top at Z 0, with a 12 mm square boss 6 mm high in the middle.
fn plate_and_boss() -> Vec<[Vec3; 3]> {
    let mut t = Vec::new();
    box_tris(Vec3::new(-20.0, -20.0, -10.0), Vec3::new(20.0, 20.0, 0.0), &mut t);
    box_tris(Vec3::new(-6.0, -6.0, 0.0), Vec3::new(6.0, 6.0, 6.0), &mut t);
    t
}

fn roughing_params() -> AdaptiveRoughingParams {
    AdaptiveRoughingParams {
        heights: Heights { clearance: 30.0, retract: 8.0, top: 6.0, bottom: 0.25 },
        feeds: Feeds { cut: 1000.0, plunge: 200.0 },
        stepdown: 2.5,
        boundary: Boundary::default(),
        flats: true,
        adaptive: AdaptiveParams { stock_to_leave: 0.25, ..params() },
        rest: None,
    }
}

#[test]
fn adaptive_roughing_stays_off_the_part_and_within_the_engagement() {
    let surface = Surface::new(plate_and_boss()).unwrap();
    let tool = flat(6.0);
    let p = roughing_params();
    let a = adaptive_roughing(&surface, &tool, &p).unwrap();
    verify(&a.path, &p.heights).unwrap();
    assert!(a.levels >= 2, "{a:?}");
    // Entered from the open air round the stock: no helix needed.
    assert_eq!(a.helices, 0);
    // The stock was taken from the part, which a larger blank would make wrong: said so.
    assert!(a.warnings.iter().any(|w| w.contains("the part's extent")), "{:?}", a.warnings);
    let rest = Cutter::flat(3.0).unwrap().grown(0.25).unwrap();
    let mut lowest = f64::INFINITY;
    for (q, cutting) in a.path.points(1e-3) {
        if cutting {
            lowest = lowest.min(q.z);
            if let Some(z) = drop_z(&surface, &rest, q.x, q.y) {
                assert!(q.z >= z + 0.25 - 1e-6, "{q:?} below {}", z + 0.25);
            }
        }
    }
    assert!((lowest - 0.25).abs() < 1e-6, "{lowest}");
    let stock = Material { regions: vec![region(&[(-20.0, -20.0), (20.0, -20.0), (20.0, 20.0), (-20.0, 20.0)], &[])], top: 6.0, within: None };
    let e = engagement(&a.path, Some(&a.roles), 3.0, &stock, 0.1).unwrap();
    assert!(e.max_deg <= 60.0 + ENGAGEMENT_TOL, "{e:?}");
    assert_eq!(e.full_slot_share, 0.0, "{e:?}");
    // The plate's top is cleared down to the stock to leave out to the boss.
    let bottom: Vec<(Vec2, Vec2)> = a
        .path
        .points(1e-3)
        .windows(2)
        .filter(|w| w[1].1 && (w[0].0.z - 0.25).abs() < 1e-6 && (w[1].0.z - 0.25).abs() < 1e-6)
        .map(|w| (Vec2::new(w[0].0.x, w[0].0.y), Vec2::new(w[1].0.x, w[1].0.y)))
        .collect();
    for q in [Vec2::new(-15.0, -15.0), Vec2::new(15.0, 0.0), Vec2::new(0.0, 12.0), Vec2::new(-19.5, 19.5), Vec2::new(-9.6, 0.0)] {
        let near = bottom.iter().any(|(a, b)| {
            let d = *b - *a;
            let t = if d.len2() > 0.0 { ((q - *a).dot(d) / d.len2()).clamp(0.0, 1.0) } else { 0.0 };
            q.dist(*a + d * t) <= 3.0
        });
        assert!(near, "{q:?} not cleared");
    }
}

/// A closed cavity in the part: the material is entered with a helix, and the cavity's walls
/// are respected.
#[test]
fn adaptive_roughing_enters_an_enclosed_cavity_with_a_helix() {
    let mut t = Vec::new();
    // A frame round a 24 mm square cavity, its floor at Z 0.
    box_tris(Vec3::new(-20.0, -20.0, -5.0), Vec3::new(20.0, 20.0, 0.0), &mut t);
    for (lo, hi) in [((-20.0, -20.0), (20.0, -12.0)), ((-20.0, 12.0), (20.0, 20.0)), ((-20.0, -12.0), (-12.0, 12.0)), ((12.0, -12.0), (20.0, 12.0))] {
        box_tris(Vec3::new(lo.0, lo.1, 0.0), Vec3::new(hi.0, hi.1, 6.0), &mut t);
    }
    let surface = Surface::new(t).unwrap();
    let p = AdaptiveRoughingParams {
        boundary: Boundary { rect: Some([Vec2::new(-12.0, -12.0), Vec2::new(12.0, 12.0)]), region: None },
        ..roughing_params()
    };
    let a = adaptive_roughing(&surface, &flat(6.0), &p).unwrap();
    assert!(a.helices >= 1, "{a:?}");
    // The stock was given: no warning about it.
    assert!(!a.warnings.iter().any(|w| w.contains("the part's extent")), "{:?}", a.warnings);
    let rest = Cutter::flat(3.0).unwrap().grown(0.25).unwrap();
    for (q, cutting) in a.path.points(1e-3) {
        if cutting && let Some(z) = drop_z(&surface, &rest, q.x, q.y) {
            assert!(q.z >= z + 0.25 - 1e-6, "{q:?}");
        }
    }
}

#[test]
fn adaptive_roughing_refuses_hostile_input() {
    let surface = Surface::new(plate_and_boss()).unwrap();
    let tool = flat(6.0);
    let mut p = roughing_params();
    p.heights.retract = 4.0;
    assert!(adaptive_roughing(&surface, &tool, &p).is_err(), "retract below the part");
    let mut p = roughing_params();
    p.boundary.rect = Some([Vec2::new(5.0, 5.0), Vec2::new(-5.0, -5.0)]);
    assert!(adaptive_roughing(&surface, &tool, &p).is_err());
    p.boundary.rect = Some([Vec2::new(f64::NAN, 5.0), Vec2::new(5.0, 6.0)]);
    assert!(adaptive_roughing(&surface, &tool, &p).is_err());
    let p = AdaptiveRoughingParams { stepdown: 0.0, ..roughing_params() };
    assert!(adaptive_roughing(&surface, &tool, &p).is_err());
    let p = AdaptiveRoughingParams { adaptive: AdaptiveParams { optimal_load: -1.0, ..params() }, ..roughing_params() };
    assert!(adaptive_roughing(&surface, &tool, &p).is_err());
    // Levels 1.9 mm apart with flutes 1.5 mm long.
    let short = Tool { flute_length: 1.5, ..tool.clone() };
    let e = adaptive_roughing(&surface, &short, &roughing_params()).unwrap_err();
    assert!(e.to_string().contains("flute length"), "{e}");
    let empty = Surface::new(vec![]).unwrap();
    assert!(adaptive_roughing(&empty, &tool, &roughing_params()).is_err());
}

/// Adaptive clearing takes the shared milling-tool rule: it enters along a helix, so a face mill
/// may clear, but a chamfer mill, a drill and a thread mill are refused.
#[test]
fn adaptive_clearing_refuses_tools_that_cannot_clear() {
    let mut face = Tool::new(1, ToolKind::FaceMill, 6.0, 20.0);
    face.corner_radius = Some(0.5);
    assert!(adaptive(&[square(24.0)], &face, &params(), &cut(0.0, -1.0)).is_ok());
    let mut chamfer = Tool::new(7, ToolKind::Chamfer, 6.0, 10.0);
    chamfer.angle_deg = Some(90.0);
    let refused = [
        (chamfer, "a chamfer mill's side is tapered"),
        (Tool::new(3, ToolKind::Drill, 6.0, 30.0), "a drill cannot mill"),
        (Tool::new(5, ToolKind::ThreadMill, 6.0, 12.0), "a thread mill cuts threads only"),
    ];
    for (t, why) in refused {
        let e = adaptive(&[square(24.0)], &t, &params(), &cut(0.0, -1.0)).unwrap_err();
        assert!(e.to_string().contains(why), "{:?}: {e}", t.kind);
    }
}

/// The work an adaptive roughing may take is bounded level by level, each level's bound scaled
/// to its raster and to the walks its area and load call for, not to how many levels there are,
/// so a larger stepdown (fewer levels) is never refused where a smaller one is planned. A job
/// that would take minutes is refused at once, with the numbers and what to change.
#[test]
fn adaptive_roughing_work_is_bounded_per_level_and_refused_at_once_when_too_large() {
    use crate::adaptive::{MAX_ROUGHING_WORK, TYPICAL_LEVEL_WORK_PER_CELL, check_roughing_work, level_work_limit};
    let cells = 400_000;
    let (steps, step_work) = (20_000.0, 4000.0);
    let hint = || "a hint".to_string();
    // Monotonic in the number of levels: once refused, refused for every larger number.
    let first = (1..=200).find(|&n| check_roughing_work(n, cells, steps, step_work, &hint).is_err()).unwrap();
    assert!((first..=200).all(|n| check_roughing_work(n, cells, steps, step_work, &hint).is_err()));
    assert!((1..first).all(|n| check_roughing_work(n, cells, steps, step_work, &hint).is_ok()));
    assert!(first as f64 * (cells as f64 * TYPICAL_LEVEL_WORK_PER_CELL + steps * step_work) > MAX_ROUGHING_WORK);
    // A 2 mm stepdown down 20 mm of that raster is planned.
    assert!(check_roughing_work(11, cells, steps, step_work, &hint).is_ok());
    // The level's own bound does not shrink with more levels; it grows with the raster and with
    // the steps its walks are expected to take.
    assert!(level_work_limit(2 * cells, steps, step_work) > level_work_limit(cells, steps, step_work));
    assert!(level_work_limit(cells, 2.0 * steps, step_work) > level_work_limit(cells, steps, step_work));
    // Refused at once, naming the fix.
    let start = std::time::Instant::now();
    let surface = Surface::new(plate_and_boss()).unwrap();
    let mut p = roughing_params();
    p.heights.top = 600.0;
    p.heights.retract = 610.0;
    p.heights.clearance = 620.0;
    p.stepdown = 0.5;
    p.boundary.rect = Some([Vec2::new(-60.0, -60.0), Vec2::new(60.0, 60.0)]);
    let e = adaptive_roughing(&surface, &Tool { flute_length: 40.0, ..flat(6.0) }, &p).unwrap_err();
    assert!(matches!(e, CamError::TooLarge(_)) && e.to_string().contains("larger stepdown"), "{e}");
    assert!(start.elapsed().as_secs() < 5, "{:?}", start.elapsed());
    // A plate round a boss at a 1 mm stepdown, more levels than the old bound on the whole
    // operation let through, is planned.
    let mut p = roughing_params();
    p.stepdown = 1.0;
    let a = adaptive_roughing(&surface, &flat(6.0), &p).unwrap();
    assert!(a.levels >= 6, "{a:?}");
}

/// A 6 mm end mill's adaptive parameters at `load` mm, the engagement limits as a job sets them
/// by default (one and a half times the load's, a third of it at least).
fn light(load: f64) -> AdaptiveParams {
    let opt = (1.0 - load / 3.0f64).acos();
    let most = (opt * 1.5).max(opt + 15.0 * DEG).min(PI);
    AdaptiveParams { optimal_load: load, min_engagement: (opt / 3.0).min(most * 0.5), max_engagement: most, ..params() }
}

/// A 60 x 50 mm plate 5 mm thick under a 30 x 20 mm boss 5 mm high, its top at Z 0.
fn plate_under_boss(half: Vec2) -> Surface {
    let mut t = Vec::new();
    box_tris(Vec3::new(-half.x, -half.y, -10.0), Vec3::new(half.x, half.y, -5.0), &mut t);
    box_tris(Vec3::new(-15.0, -10.0, -5.0), Vec3::new(15.0, 10.0, 0.0), &mut t);
    Surface::new(t).unwrap()
}

/// The light loads a small mill's tool library sets (5% of the diameter, and 2% for the larger
/// tools) are planned, not refused: the planner's raster follows the tool, not the load, and
/// its budget scales with the area and the load. Before, a 6 mm tool at 0.3 mm round a boss on
/// a 60 x 50 mm stock was refused after half a minute, and 0.45 mm on 100 x 60 mm took 26 s.
#[test]
fn light_loads_plan_round_a_boss() {
    let tool = flat(6.0);
    for (half, load) in [(Vec2::new(30.0, 25.0), 0.3), (Vec2::new(50.0, 30.0), 0.12)] {
        let surface = plate_under_boss(half);
        let p = AdaptiveRoughingParams {
            heights: Heights { clearance: 10.0, retract: 2.0, top: 0.0, bottom: -5.0 },
            feeds: Feeds { cut: 1000.0, plunge: 200.0 },
            stepdown: 5.0,
            boundary: Boundary { rect: Some([-half, half]), region: None },
            flats: true,
            adaptive: AdaptiveParams { stock_to_leave: 0.0, ..light(load) },
            rest: None,
        };
        let start = std::time::Instant::now();
        let a = adaptive_roughing(&surface, &tool, &p).unwrap();
        let took = start.elapsed();
        let area = 4.0 * half.x * half.y - 600.0;
        let ratio = a.path.stats().cut_length / (area / load);
        eprintln!("{half:?} at {load} mm: {took:?}, path {:.0} mm, {ratio:.2} x area/load", a.path.stats().cut_length);
        assert!(ratio <= 1.5, "{ratio}");
        assert!(took.as_secs() < 120, "{took:?}");
        assert_eq!(a.helices, 0);
    }
}

/// Where the walks would take more steps than a clearing may, the refusal comes at once and
/// names the smallest load that plans.
#[test]
fn a_refusal_names_the_smallest_load_that_plans() {
    let start = std::time::Instant::now();
    let tool = flat(6.0);
    let big = square(400.0);
    let e = adaptive(std::slice::from_ref(&big), &tool, &light(0.05), &cut(0.0, -1.0)).unwrap_err();
    let msg = e.to_string();
    assert!(matches!(e, CamError::TooLarge(_)) && msg.contains("optimal load of at least"), "{msg}");
    assert!(start.elapsed().as_secs() < 10, "{:?}", start.elapsed());
    // The load named is one the estimate lets through.
    let at: f64 = msg.split("at least ").nth(1).and_then(|t| t.split(' ').next()).and_then(|t| t.parse().ok()).unwrap();
    assert!(at > 0.05 && at <= 6.0, "{at}");
}

/// The cut length of `a` against the area it clears over the load (one level).
fn length_ratio(a: &AdaptivePath, area: f64, load: f64) -> f64 {
    a.path.stats().cut_length / (area / load)
}

/// A 60 x 50 mm ring round a 30 x 20 mm boss whose outside is the stock's edge.
fn ring() -> Region2 {
    region(&[(-30.0, -25.0), (30.0, -25.0), (30.0, 25.0), (-30.0, 25.0)], &[&[(-15.0, -10.0), (-15.0, 10.0), (15.0, 10.0), (15.0, -10.0)]])
}

fn ring_stock() -> StockModel {
    StockModel { rect: Some([Vec2::new(-30.0, -25.0), Vec2::new(30.0, 25.0)]), top: Some(0.0), ..Default::default() }
}

/// Given the stock, a 2D clearing goes down in the open air beside it and cuts in from there,
/// as an adaptive roughing does: no helix, and a path about as long as the area over the load.
/// Without it (the regions taken as full of material, entered with a helix) the same ring took
/// 30.5 m at 0.6 mm, 7.6 times the area over the load, and was refused at 0.3 mm.
#[test]
fn a_2d_clearing_given_the_stock_enters_from_open_air() {
    let tool = flat(6.0);
    for load in [0.6, 0.3] {
        let c = Cut {
            heights: Heights { clearance: 10.0, retract: 2.0, top: 0.0, bottom: -5.0 },
            feeds: Feeds { cut: 1000.0, plunge: 200.0 },
            stepdown: 5.0,
        };
        let p = light(load);
        let a = adaptive_in(&[ring()], &tool, &p, &c, &ring_stock()).unwrap();
        verify(&a.path, &c.heights).unwrap();
        assert_eq!(a.helices, 0, "{a:?}");
        let ratio = length_ratio(&a, 2400.0, load);
        eprintln!("ring at {load} mm: {:.0} mm, {ratio:.2} x area/load", a.path.stats().cut_length);
        assert!(ratio <= 1.5, "{ratio}");
        // Every move that cuts below the top keeps the tool off the boss.
        for (q, cutting) in a.path.points(1e-3) {
            if cutting && q.z < 0.0 {
                let (dx, dy) = ((-15.0 - q.x).max(q.x - 15.0).max(0.0), (-10.0 - q.y).max(q.y - 10.0).max(0.0));
                assert!(dx.hypot(dy) >= 3.0 - 1e-6, "{q:?}");
            }
        }
        let stock = Material { regions: vec![region(&[(-30.0, -25.0), (30.0, -25.0), (30.0, 25.0), (-30.0, 25.0)], &[])], top: 0.0, within: None };
        let e = engagement(&a.path, Some(&a.roles), 3.0, &stock, 0.1).unwrap();
        assert!(e.max_deg <= p.max_engagement / DEG + ENGAGEMENT_TOL, "{e:?}");
        assert_eq!(e.full_slot_share, 0.0, "{e:?}");
        assert_cleared(&[ring()], &a.path, 3.0, 0.0, -5.0, 0.4);
    }
}

/// Where a region reaches past the stock, the part outside it is air: the clearing does not cut
/// there, so its path is as long as the stock's part of the region needs. A level at or above
/// the stock's top is left out.
#[test]
fn a_2d_clearing_does_not_cut_air_past_the_stock() {
    let tool = flat(6.0);
    let wide = region(&[(-40.0, -25.0), (40.0, -25.0), (40.0, 25.0), (-40.0, 25.0)], &[]);
    let stock = StockModel { rect: Some([Vec2::new(-20.0, -25.0), Vec2::new(20.0, 25.0)]), top: Some(-1.0), ..Default::default() };
    let c = Cut {
        heights: Heights { clearance: 10.0, retract: 2.0, top: 0.0, bottom: -3.0 },
        feeds: Feeds { cut: 1000.0, plunge: 200.0 },
        stepdown: 1.0,
    };
    let a = adaptive_in(&[wide], &tool, &params(), &c, &stock).unwrap();
    // Levels at -1 (the stock's top: nothing to cut), -2 and -3.
    assert_eq!(a.levels, 2, "{a:?}");
    assert_eq!(a.helices, 0);
    let ratio = a.path.stats().cut_length / (2.0 * 40.0 * 50.0 / 0.6);
    assert!(ratio <= 1.5, "{ratio}");
    // Nothing cuts past the stock but to come in from the air.
    for (q, cutting) in a.path.points(1e-3) {
        if cutting && q.z < -1.0 {
            assert!(q.x.abs() <= 20.0 + 3.0 + 0.5 * 6.0, "{q:?}");
        }
    }
}

/// A pocket closed all round is entered with a helix and cleared out to its corners. The walk
/// ends where it runs out of material instead of wandering through what it cleared, and the next
/// starts where cleared space meets material again: a 40 mm square at 0.6 mm took 3.3 times the
/// area over the load, and takes under 2.1 (the corners need a way back to each new cut when
/// every cut climbs).
#[test]
fn a_closed_pocket_path_is_not_much_longer_than_its_area_over_the_load() {
    let a = adaptive(&[square(40.0)], &flat(6.0), &light(0.6), &cut(0.0, -1.0)).unwrap();
    let ratio = length_ratio(&a, 1600.0, 0.6);
    eprintln!("closed square: {:.0} mm, {ratio:.2} x area/load", a.path.stats().cut_length);
    assert!(ratio <= 2.1, "{ratio}");
    assert_cleared(&[square(40.0)], &a.path, 3.0, 0.0, -1.0, 0.4);
}
