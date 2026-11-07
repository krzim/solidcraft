//! The CAM simulation panel: its timeline maths, its steps and load parameters, and the player
//! behind it (commands in order, off the window thread, a scrub keeping only its latest seek,
//! play keeping to time, agents' requests answered in turn).

use std::time::Duration;

use serde_json::json;

use super::*;
use crate::cam_view_layer::tests::loaded;
use crate::cam_view_player::queued;

fn mark(index: usize, time_s: f64) -> FindingMark {
    FindingMark { index, time_s, line: index + 10, code: "gouge".into(), error: true, message: String::new() }
}

#[test]
fn time_and_x_on_the_strip_map_both_ways_and_clamp() {
    for t in [0.0, 12.5, 60.0, 100.0] {
        let x = time_to_x(t, 100.0, 10.0, 200.0);
        assert!((x_to_time(x, 100.0, 10.0, 200.0) - t).abs() < 1e-3);
    }
    assert_eq!(time_to_x(-5.0, 100.0, 10.0, 200.0), 10.0);
    assert_eq!(time_to_x(500.0, 100.0, 10.0, 200.0), 210.0);
    assert_eq!(x_to_time(0.0, 100.0, 10.0, 200.0), 0.0);
    assert_eq!(x_to_time(999.0, 100.0, 10.0, 200.0), 100.0);
    // An empty program, a hostile width.
    assert_eq!(time_to_x(5.0, 0.0, 10.0, 200.0), 10.0);
    assert_eq!(x_to_time(50.0, f64::NAN, 10.0, 0.0), 0.0);
}

#[test]
fn the_nearest_finding_within_reach_is_found() {
    let marks = [mark(0, 10.0), mark(1, 20.0), mark(2, 22.0)];
    assert_eq!(finding_near(&marks, 21.5, 3.0).map(|f| f.index), Some(2));
    assert_eq!(finding_near(&marks, 15.0, 3.0).map(|f| f.index), None);
    let ops = [
        OpBand { index: 0, name: "rough".into(), start_s: 0.0, end_s: 10.0 },
        OpBand { index: 1, name: "finish".into(), start_s: 10.0, end_s: 30.0 },
    ];
    assert_eq!(op_at(&ops, 15.0).map(|o| o.name.as_str()), Some("finish"));
    assert!(op_at(&ops, 31.0).is_none());
}

#[test]
fn steps_and_loads_are_the_commands_parameters() {
    assert_eq!(StepBy::Move.params(true), json!({"by": 1}));
    assert_eq!(StepBy::Op.params(false), json!({"by": -1, "until": "op"}));
    assert_eq!(StepBy::ToolChange.params(true), json!({"by": 1, "until": "tool_change"}));
    assert_eq!(StepBy::Finding.params(true), json!({"by": 1, "until": "finding"}));
    let d = CamPanel { job: " a.cam.json ".into(), holding: "v.workholding.json".into(), ..CamPanel::default() };
    assert_eq!(load_params(&d), json!({"job_path": "a.cam.json", "colour_by": "operation", "workholding_path": "v.workholding.json"}));
    assert_eq!(clock(75.4), "01:15");
    assert_eq!(clock(3725.0), "1:02:05");
    assert_eq!(clock(f64::NAN), "00:00");
}

/// Frames until the player has nothing left to do (the worker is done).
fn settle(h: &mut crate::scenario::Harness) {
    for _ in 0..2000 {
        h.frame();
        if !h.app.cam_player.away() && !h.app.cam_player.pending() {
            return;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    panic!("the player did not settle");
}

#[test]
fn a_scrub_keeps_only_its_latest_seek_and_it_runs_off_the_window_thread() {
    let mut h = loaded();
    h.app.apply_off_thread = Some(Duration::ZERO);
    h.app.cam_player.ask("cam.view.seek", json!({"time_s": 1.0}));
    h.app.cam_player.ask("cam.view.show", json!({"rapids": false}));
    h.app.cam_player.ask("cam.view.seek", json!({"percent": 60}));
    let q = queued(&h.app);
    assert_eq!(q.iter().filter(|(c, _)| c == "cam.view.seek").count(), 1, "{q:?}");
    assert_eq!(q.last().map(|(_, p)| p.clone()), Some(json!({"percent": 60})));
    // The next frame takes the view to the worker; the window keeps drawing it as it was.
    h.frame();
    h.frame();
    settle(&mut h);
    let s = h.app.cam_player.snapshot.as_ref().unwrap();
    assert!((s.state.percent - 60.0).abs() < 0.5, "{}", s.state.percent);
    assert!(!s.state.layers.rapids);
    assert!(h.app.cam_player.tiles.lock().unwrap().changed > 0, "the worker built the tiles");
    assert!(h.app.cam_player.error.is_none(), "{:?}", h.app.cam_player.error);
}

#[test]
fn play_keeps_on_and_stops_at_the_end() {
    let mut h = loaded();
    h.app.apply_off_thread = Some(Duration::ZERO);
    settle(&mut h);
    h.app.cam_player.ask("cam.view.play", json!({"speed": 1000}));
    let t0 = h.app.session.cam_view.as_deref().and_then(|v| v.playback()).map_or(0.0, |p| p.time_now());
    for _ in 0..400 {
        h.frame();
        std::thread::sleep(Duration::from_millis(5));
        if h.app.cam_player.snapshot.as_ref().is_some_and(|s| !s.state.playing && s.state.percent >= 100.0) {
            break;
        }
    }
    settle(&mut h);
    let s = h.app.cam_player.snapshot.as_ref().unwrap();
    assert!(s.state.time_s > t0, "the tool moved on");
    assert!(!s.state.playing && s.state.percent >= 99.9, "{} {}", s.state.playing, s.state.percent);
}

#[test]
fn an_agents_request_waits_for_the_view_and_is_answered() {
    let mut h = loaded();
    h.app.apply_off_thread = Some(Duration::ZERO);
    let (req, rx) = crate::control::ControlRequest::new("engine.execute", json!({"command": "cam.view.seek", "params": {"percent": 25}}));
    assert!(crate::cam_view_player::intercept(&mut h.app, req).is_none());
    let (state, rx2) = crate::control::ControlRequest::new("engine.execute", json!({"command": "cam.view.state", "params": {}}));
    assert!(crate::cam_view_player::intercept(&mut h.app, state).is_none());
    settle(&mut h);
    let a = rx.recv_timeout(Duration::from_secs(30)).unwrap();
    assert_eq!(a["ok"], true, "{a}");
    let b = rx2.recv_timeout(Duration::from_secs(30)).unwrap();
    assert_eq!(b["ok"], true, "{b}");
    assert!((b["result"]["percent"].as_f64().unwrap() - 25.0).abs() < 0.5, "{b}");
    // Other requests are not the player's.
    let (other, _rx) = crate::control::ControlRequest::new("engine.execute", json!({"command": "solid.box", "params": {}}));
    assert!(crate::cam_view_player::intercept(&mut h.app, other).is_some());
}

#[test]
fn the_panel_opens_with_a_loaded_view_and_closing_it_closes_the_view() {
    let mut h = loaded();
    h.frames(2);
    assert!(h.app.cam_panel.is_some());
    assert!(h.app.cam_player.snapshot.is_some());
    h.app.cam_panel = None;
    h.app.cam_player.ask("cam.view.close", json!({}));
    h.frames(2);
    assert!(h.app.session.cam_view.is_none());
    assert!(h.app.cam_player.snapshot.is_none());
}

#[test]
fn findings_still_to_come_keep_the_panel_its_own_width() {
    let mut h = loaded();
    h.frames(2);
    let s = h.app.cam_player.snapshot.as_mut().unwrap();
    s.findings = vec![mark(0, 1.0e6), mark(1, 2.0e6)];
    let width = |h: &mut crate::scenario::Harness| {
        h.frames(1);
        crate::scenario::rect_of("dialog:list:sc_cam_finding_0").map(|r| r.right())
    };
    let first = width(&mut h).expect("the finding is listed");
    for _ in 0..20 {
        h.frames(1);
    }
    let later = width(&mut h).unwrap();
    assert!((later - first).abs() < 0.5, "the row grew from {first} to {later}: the panel widens every frame");
}
