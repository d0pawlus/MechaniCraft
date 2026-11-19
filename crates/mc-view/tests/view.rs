//! Behaviour of the view model on the bundled slider-crank demo.

use approx::assert_abs_diff_eq;
use mc_view::{
    Bounds, Channel, Derivative, Document, LinkKind, LoadError, Playback, ViewTransform, load,
};

const DEMO: &str = include_str!("../../../fixtures/demo/slider-crank.sam");
/// Crank length of the demo (node 1 to node 2).
const CRANK: f64 = 0.1;
/// Number of states: 120 steps plus the initial state.
const STATES: usize = 121;

fn demo() -> Document {
    load("slider-crank.sam", DEMO.as_bytes()).expect("demo loads")
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

// --- document -------------------------------------------------------------

#[test]
fn demo_loads_with_kinematics_and_forces() {
    let doc = demo();
    assert_eq!(doc.name, "slider-crank.sam");
    assert_eq!(doc.trajectory.states.len(), STATES);
    let forces = doc.forces.as_ref().expect("forces computed");
    assert_eq!(forces.states.len(), STATES);
    assert!(doc.warnings.is_empty(), "{:?}", doc.warnings);
    assert_eq!(doc.frame_count(), STATES);
}

#[test]
fn garbage_bytes_give_import_error() {
    let err = load("x.sam", b"not a sam file").unwrap_err();
    assert!(matches!(err, LoadError::Import(_)), "{err:?}");
}

#[test]
fn mechanism_without_inputs_gives_analysis_error() {
    let start = DEMO.find("INPUT:").unwrap();
    let end = DEMO.find("GRAVITY:").unwrap();
    let text = format!("{}{}", &DEMO[..start], &DEMO[end..]);
    let err = load("no-input.sam", text.as_bytes()).unwrap_err();
    assert!(matches!(err, LoadError::Analysis(_)), "{err:?}");
    assert!(err.to_string().contains("no inputs"), "{err}");
}

#[test]
fn times_follow_the_trajectory() {
    let doc = demo();
    let times = doc.times();
    assert_eq!(times.len(), STATES);
    assert_abs_diff_eq!(times[0], 0.0);
    assert_abs_diff_eq!(times[STATES - 1], 1.0, epsilon = 1e-12);
}

// --- scene ----------------------------------------------------------------

#[test]
fn scene_has_nodes_with_supports_and_two_beams() {
    let doc = demo();
    let nodes: Vec<_> = doc
        .scene
        .nodes
        .iter()
        .map(|n| (n.nr, n.fixed_x, n.fixed_y))
        .collect();
    assert_eq!(
        nodes,
        vec![(1, true, true), (2, false, false), (3, false, true)]
    );
    let links: Vec<_> = doc
        .scene
        .links
        .iter()
        .map(|l| (l.element_nr, l.kind, l.n1, l.n2))
        .collect();
    assert_eq!(
        links,
        vec![(1, LinkKind::Beam, 1, 2), (2, LinkKind::Beam, 2, 3)]
    );
}

#[test]
fn node_positions_start_at_initial_values() {
    let doc = demo();
    let p3 = doc.node_position(0, 3).unwrap();
    assert_abs_diff_eq!(p3[0], 0.4, epsilon = 1e-9);
    assert_abs_diff_eq!(p3[1], 0.0, epsilon = 1e-12);
    assert!(doc.node_position(0, 99).is_none());
    assert!(doc.node_position(STATES, 1).is_none());
}

#[test]
fn link_segments_keep_crank_length_in_every_frame() {
    let doc = demo();
    for frame in 0..doc.frame_count() {
        let segs = doc.link_segments(frame);
        assert_eq!(segs.len(), 2);
        assert_abs_diff_eq!(dist(segs[0].p, segs[0].q), CRANK, epsilon = 1e-9);
    }
    assert!(doc.link_segments(STATES).is_empty());
}

#[test]
fn bounds_cover_every_node_in_every_frame() {
    let doc = demo();
    let b = doc.bounds().expect("bounds");
    for frame in 0..doc.frame_count() {
        for n in &doc.scene.nodes {
            let p = doc.node_position(frame, n.nr).unwrap();
            assert!(b.min[0] <= p[0] && p[0] <= b.max[0], "x out of bounds");
            assert!(b.min[1] <= p[1] && p[1] <= b.max[1], "y out of bounds");
        }
    }
    assert!(b.max[0] - b.min[0] > 0.0 && b.max[1] - b.min[1] > 0.0);
}

// --- channels -------------------------------------------------------------

#[test]
fn channels_list_coordinates_and_drive() {
    let doc = demo();
    let labels: Vec<String> = doc.channels().iter().map(|c| doc.label(*c)).collect();
    for expected in ["x3", "x3'", "x3''", "y2", "A1001", "DRV3", "PWR3"] {
        assert!(
            labels.iter().any(|l| l == expected),
            "missing {expected} in {labels:?}"
        );
    }
}

#[test]
fn fixed_coordinates_are_not_offered_as_channels() {
    let doc = demo();
    let labels: Vec<String> = doc.channels().iter().map(|c| doc.label(*c)).collect();
    assert!(!labels.iter().any(|l| l == "x1" || l == "y3"), "{labels:?}");
}

#[test]
fn node_channel_samples_value_over_time() {
    let doc = demo();
    let ch = doc.node_channel(3, 'x', Derivative::Value).unwrap();
    let s = doc.samples(ch);
    assert_eq!(s.len(), STATES);
    assert_abs_diff_eq!(s[0][0], 0.0);
    assert_abs_diff_eq!(s[0][1], 0.4, epsilon = 1e-9);
    assert!(doc.node_channel(99, 'x', Derivative::Value).is_none());
}

#[test]
fn velocity_channel_matches_finite_difference_of_position() {
    let doc = demo();
    let x = doc.samples(doc.node_channel(2, 'x', Derivative::Value).unwrap());
    let v = doc.samples(doc.node_channel(2, 'x', Derivative::Rate).unwrap());
    for i in 1..x.len() - 1 {
        let fd = (x[i + 1][1] - x[i - 1][1]) / (x[i + 1][0] - x[i - 1][0]);
        assert_abs_diff_eq!(
            fd,
            v[i][1],
            epsilon = 1e-2 * CRANK * 2.0 * std::f64::consts::PI
        );
    }
}

#[test]
fn driving_channel_is_nonzero_under_gravity() {
    let doc = demo();
    let s = doc.samples(Channel::Driving(0));
    assert_eq!(s.len(), STATES);
    assert!(
        s.iter().any(|p| p[1].abs() > 1e-3),
        "torque is zero everywhere"
    );
}

#[test]
fn force_channels_are_absent_without_forces() {
    let mut doc = demo();
    doc.forces = None;
    assert!(
        doc.channels()
            .iter()
            .all(|c| matches!(c, Channel::Coord { .. }))
    );
    assert!(doc.samples(Channel::Driving(0)).is_empty());
}

#[test]
fn node_path_of_crank_pin_is_a_circle() {
    let doc = demo();
    let path = doc.node_path(2);
    assert_eq!(path.len(), STATES);
    for p in &path {
        assert_abs_diff_eq!(dist(*p, [0.0, 0.05]), CRANK, epsilon = 1e-9);
    }
    assert!(doc.node_path(99).is_empty());
}

// --- playback -------------------------------------------------------------

fn three_frames() -> Playback {
    Playback::new(vec![0.0, 0.5, 1.0])
}

#[test]
fn playback_starts_paused_at_first_frame() {
    let mut p = three_frames();
    assert!(!p.playing);
    assert_eq!(p.frame(), Some(0));
    p.advance(0.3);
    assert_eq!(p.time, 0.0);
}

#[test]
fn playback_advances_with_speed() {
    let mut p = three_frames();
    p.playing = true;
    p.speed = 2.0;
    p.advance(0.3);
    assert_abs_diff_eq!(p.time, 0.6);
    assert_eq!(p.frame(), Some(1));
}

#[test]
fn playback_loops_or_stops_at_end() {
    let mut p = three_frames();
    p.playing = true;
    p.looping = true;
    p.advance(1.25);
    assert_abs_diff_eq!(p.time, 0.25, epsilon = 1e-12);
    assert!(p.playing);

    let mut p = three_frames();
    p.playing = true;
    p.looping = false;
    p.advance(1.25);
    assert_abs_diff_eq!(p.time, 1.0);
    assert!(!p.playing);
    assert_eq!(p.frame(), Some(2));
}

#[test]
fn frame_is_last_state_not_after_time() {
    let mut p = three_frames();
    for (t, frame) in [
        (-1.0, 0),
        (0.49, 0),
        (0.5, 1),
        (0.99, 1),
        (1.0, 2),
        (5.0, 2),
    ] {
        p.time = t;
        assert_eq!(p.frame(), Some(frame), "t = {t}");
    }
}

#[test]
fn seek_frame_jumps_to_its_time() {
    let mut p = three_frames();
    p.seek_frame(2);
    assert_abs_diff_eq!(p.time, 1.0);
    p.seek_frame(10);
    assert_abs_diff_eq!(p.time, 1.0);
}

#[test]
fn empty_playback_has_no_frame_and_does_not_panic() {
    let mut p = Playback::new(Vec::new());
    p.playing = true;
    p.advance(1.0);
    p.seek_frame(0);
    assert_eq!(p.frame(), None);
}

#[test]
fn zero_length_timeline_does_not_spin() {
    let mut p = Playback::new(vec![0.0]);
    p.playing = true;
    p.looping = true;
    p.advance(1.0);
    assert_eq!(p.frame(), Some(0));
    assert!(p.time.is_finite());
}

// --- defaults and view transform -----------------------------------------

#[test]
fn default_channel_is_x_of_last_node_moving_in_x() {
    let doc = demo();
    let ch = doc.default_channel().unwrap();
    assert_eq!(doc.label(ch), "x3");
}

#[test]
fn default_path_node_is_first_fully_free_node() {
    assert_eq!(demo().default_path_node(), Some(2));
}

#[test]
fn view_transform_fits_bounds_and_flips_y() {
    let b = Bounds {
        min: [0.0, 0.0],
        max: [2.0, 1.0],
    };
    // Screen rectangle 400 x 400 px at (100, 50), no margin.
    let vt = ViewTransform::fit(b, [100.0, 50.0], [400.0, 400.0], 0.0);
    // Width limits the scale: 400 px / 2 m.
    assert_abs_diff_eq!(vt.scale, 200.0);
    let lo = vt.to_screen([0.0, 0.0]);
    let hi = vt.to_screen([2.0, 1.0]);
    assert_abs_diff_eq!(lo[0], 100.0);
    assert_abs_diff_eq!(hi[0], 500.0);
    // y grows upwards in the model and downwards on screen; content is centred.
    assert!(hi[1] < lo[1]);
    assert_abs_diff_eq!((lo[1] + hi[1]) / 2.0, 250.0);
}

#[test]
fn view_transform_keeps_margin_and_handles_degenerate_bounds() {
    let b = Bounds {
        min: [1.0, 1.0],
        max: [1.0, 1.0],
    };
    let vt = ViewTransform::fit(b, [0.0, 0.0], [100.0, 50.0], 10.0);
    assert!(vt.scale.is_finite() && vt.scale > 0.0);
    let c = vt.to_screen([1.0, 1.0]);
    assert_abs_diff_eq!(c[0], 50.0);
    assert_abs_diff_eq!(c[1], 25.0);
}
