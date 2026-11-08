//! Analytic and consistency tests of the Newton kinematic solver.

use std::f64::consts::PI;

use approx::assert_abs_diff_eq;
use mc_core::{Mechanism, MechanismBuilder, SolveError, State, analyze};
use mc_motion::{InputLaw, Segment};

fn law(duration: f64, steps: u32) -> InputLaw {
    InputLaw {
        segments: vec![Segment::Linear {
            duration,
            steps,
            displacement: 2.0 * PI,
        }],
    }
}

fn pos(mech: &Mechanism, s: &State, node: u32) -> (f64, f64) {
    let (x, y) = mech.node(node).unwrap();
    (s.q[x.0], s.q[y.0])
}

fn vel(mech: &Mechanism, s: &State, node: u32) -> (f64, f64) {
    let (x, y) = mech.node(node).unwrap();
    (s.qd[x.0], s.qd[y.0])
}

/// Slider-crank: pivot node 1 at (0, e), crank beam 1, rod beam 2, slider node 3 on y = 0.
fn slider_crank(theta0: f64, drive: InputLaw, fix_slider_y: bool) -> Mechanism {
    let (r, l, e) = (0.1, 0.35, 0.05);
    let (bx, by) = (r * theta0.cos(), e + r * theta0.sin());
    let x3 = bx + (l * l - by * by).sqrt();
    let mut b = MechanismBuilder::new();
    b.node(1, 0.0, e, true, true)
        .node(2, bx, by, false, false)
        .node(3, x3, 0.0, false, fix_slider_y)
        .angle(1001, theta0, false)
        .angle(1002, theta0, false)
        .angle(1003, (-by).atan2(x3 - bx), false)
        .angle(1004, (-by).atan2(x3 - bx), false)
        .angle(1005, 0.0, true)
        .beam(1, 1, 2, 1001, 1002)
        .beam(2, 2, 3, 1003, 1004)
        .rot_motor(3, 1005, 1001, false)
        .input(3, drive);
    b.build().unwrap()
}

#[test]
fn slider_crank_full_revolution_matches_analytic() {
    let (r, l, e) = (0.1_f64, 0.35_f64, 0.05_f64);
    let theta0 = 0.5_f64;
    let drive = law(1.0, 36);
    let mech = slider_crank(theta0, drive.clone(), true);
    let traj = analyze(&mech).unwrap();
    assert!(traj.failure.is_none(), "failure: {:?}", traj.failure);
    assert_eq!(traj.states.len(), 37);

    let theta_dot = 2.0 * PI; // constant rate, so the angular acceleration is 0
    for s in &traj.states {
        let (u, _, _) = drive.eval(s.t);
        let th = theta0 + u;
        let w = e + r * th.sin();
        let root = (l * l - w * w).sqrt();
        let x3 = r * th.cos() + root;
        // dx3/dθ and d²x3/dθ².
        let dw = r * th.cos();
        let dx = -r * th.sin() - w * dw / root;
        let g2 = (dw * dw - w * r * th.sin()) / root + w * w * dw * dw / root.powi(3);
        let ddx = -r * th.cos() - g2;

        let (px, py) = pos(&mech, s, 3);
        assert_abs_diff_eq!(px, x3, epsilon = 1e-9);
        assert_abs_diff_eq!(py, 0.0, epsilon = 1e-9);
        let (bx, by) = pos(&mech, s, 2);
        assert_abs_diff_eq!(bx, r * th.cos(), epsilon = 1e-9);
        assert_abs_diff_eq!(by, e + r * th.sin(), epsilon = 1e-9);

        let (vx2, vy2) = vel(&mech, s, 2);
        assert_abs_diff_eq!(vx2, -r * th.sin() * theta_dot, epsilon = 1e-8);
        assert_abs_diff_eq!(vy2, r * th.cos() * theta_dot, epsilon = 1e-8);

        let (vx, _) = vel(&mech, s, 3);
        assert_abs_diff_eq!(vx, dx * theta_dot, epsilon = 1e-8);

        let (x, _) = mech.node(3).unwrap();
        assert_abs_diff_eq!(s.qdd[x.0], ddx * theta_dot * theta_dot, epsilon = 1e-6);
    }
}

/// Four-bar: ground pivots A(0,0), D(0.4,0), crank AB, coupler BC, rocker CD.
fn four_bar(
    crank: f64,
    coupler: f64,
    rocker: f64,
    ground: f64,
    start: f64,
    drive: InputLaw,
) -> Mechanism {
    let a = (0.0, 0.0);
    let d = (ground, 0.0);
    let bpt = (crank * start.cos(), crank * start.sin());
    // Upper intersection of circle(B, coupler) and circle(D, rocker).
    let (dx, dy) = (d.0 - bpt.0, d.1 - bpt.1);
    let dist = dx.hypot(dy);
    let along = (coupler * coupler - rocker * rocker + dist * dist) / (2.0 * dist);
    let h = (coupler * coupler - along * along).sqrt();
    let (ux, uy) = (dx / dist, dy / dist);
    let cands = [
        (bpt.0 + along * ux - h * uy, bpt.1 + along * uy + h * ux),
        (bpt.0 + along * ux + h * uy, bpt.1 + along * uy - h * ux),
    ];
    let c = if cands[0].1 >= cands[1].1 { cands[0] } else { cands[1] };
    let ang = |p: (f64, f64), q: (f64, f64)| (q.1 - p.1).atan2(q.0 - p.0);
    let (a1, a2, a3) = (ang(a, bpt), ang(bpt, c), ang(c, d));
    let mut b = MechanismBuilder::new();
    b.node(1, a.0, a.1, true, true)
        .node(2, bpt.0, bpt.1, false, false)
        .node(3, c.0, c.1, false, false)
        .node(4, d.0, d.1, true, true)
        .angle(1, a1, false)
        .angle(2, a1, false)
        .angle(3, a2, false)
        .angle(4, a2, false)
        .angle(5, a3, false)
        .angle(6, a3, false)
        .angle(7, 0.0, true)
        .beam(1, 1, 2, 1, 2)
        .beam(2, 2, 3, 3, 4)
        .beam(3, 3, 4, 5, 6)
        .rot_motor(4, 7, 1, false)
        .input(4, drive);
    b.build().unwrap()
}

fn dist(mech: &Mechanism, s: &State, n1: u32, n2: u32) -> f64 {
    let (p, q) = (pos(mech, s, n1), pos(mech, s, n2));
    (q.0 - p.0).hypot(q.1 - p.1)
}

#[test]
fn four_bar_lengths_preserved_and_velocity_consistent() {
    let mech = four_bar(0.1, 0.35, 0.3, 0.4, PI / 3.0, law(1.0, 72));
    let traj = analyze(&mech).unwrap();
    assert!(traj.failure.is_none(), "failure: {:?}", traj.failure);
    assert_eq!(traj.states.len(), 73);
    for s in &traj.states {
        assert_abs_diff_eq!(dist(&mech, s, 1, 2), 0.1, epsilon = 1e-10);
        assert_abs_diff_eq!(dist(&mech, s, 2, 3), 0.35, epsilon = 1e-10);
        assert_abs_diff_eq!(dist(&mech, s, 3, 4), 0.3, epsilon = 1e-10);
    }
    let mut max_speed: f64 = 0.0;
    // Five-point central stencil: truncation error is about 6e-5 at peak speed.
    let st = &traj.states;
    for k in 2..st.len() - 2 {
        let cur = &st[k];
        let dt = st[k + 1].t - st[k].t;
        for i in 0..cur.q.len() {
            let fd = (-st[k + 2].q[i] + 8.0 * st[k + 1].q[i] - 8.0 * st[k - 1].q[i]
                + st[k - 2].q[i])
                / (12.0 * dt);
            assert_abs_diff_eq!(cur.qd[i], fd, epsilon = 1e-4);
            max_speed = max_speed.max(cur.qd[i].abs());
        }
    }
    assert!(max_speed > 1e-3, "velocities are suspiciously small");
}

#[test]
fn four_bar_coarse_steps_keep_branch() {
    let mech = four_bar(0.1, 0.35, 0.3, 0.4, PI / 3.0, law(1.0, 12));
    let traj = analyze(&mech).unwrap();
    assert!(traj.failure.is_none(), "failure: {:?}", traj.failure);
    assert_eq!(traj.states.len(), 13);
    let sign = |s: &State| {
        let (b, c, d) = (pos(&mech, s, 2), pos(&mech, s, 3), pos(&mech, s, 4));
        let coupler = (c.0 - b.0, c.1 - b.1);
        let rocker = (d.0 - c.0, d.1 - c.1);
        (coupler.0 * rocker.1 - coupler.1 * rocker.0).signum()
    };
    let first = sign(&traj.states[0]);
    for s in &traj.states {
        assert_eq!(sign(s), first, "branch flip at t = {}", s.t);
    }
}

#[test]
fn unreachable_input_reports_step_and_keeps_partial() {
    let mech = four_bar(0.3, 0.1, 0.3, 0.4, PI / 3.0, law(1.0, 72));
    let traj = analyze(&mech).unwrap();
    // The coupler cannot span B-D once |BD| > coupler + rocker. Expected first
    // failing step: the first grid angle beyond that geometric limit.
    let expected = (1..=72_u32)
        .find(|k| {
            let phi = PI / 3.0 + 2.0 * PI * f64::from(*k) / 72.0;
            let (bx, by) = (0.3 * phi.cos(), 0.3 * phi.sin());
            (0.4 - bx).hypot(by) > 0.1 + 0.3
        })
        .unwrap();
    match traj.failure {
        Some(SolveError::Singular { step, .. }) | Some(SolveError::NoConvergence { step, .. }) => {
            assert!(step > 0);
            assert_eq!(traj.states.len(), step);
            assert_eq!(step, expected as usize);
            let last = traj.states.last().unwrap();
            assert_abs_diff_eq!(dist(&mech, last, 1, 2), 0.3, epsilon = 1e-10);
            assert_abs_diff_eq!(dist(&mech, last, 2, 3), 0.1, epsilon = 1e-10);
            assert_abs_diff_eq!(dist(&mech, last, 3, 4), 0.3, epsilon = 1e-10);
        }
        other => panic!("expected Singular or NoConvergence, got {other:?}"),
    }
}

#[test]
fn under_constrained_is_not_square() {
    let mech = slider_crank(0.5, law(1.0, 36), false);
    assert_eq!(
        analyze(&mech).unwrap_err(),
        SolveError::NotSquare {
            equations: 7,
            unknowns: 8
        }
    );
}

#[test]
fn linear_actuator_matches_analytic() {
    let drive = InputLaw {
        segments: vec![Segment::Linear {
            duration: 1.0,
            steps: 10,
            displacement: -0.05,
        }],
    };
    let mut b = MechanismBuilder::new();
    b.node(1, 0.0, 0.0, true, true)
        .node(2, 1.0, 0.0, false, true)
        .angle(1001, 0.0, false)
        .angle(1002, 0.0, false)
        .beam(1, 1, 2, 1001, 1002)
        .input(1, drive.clone());
    let mech = b.build().unwrap();
    let traj = analyze(&mech).unwrap();
    assert!(traj.failure.is_none(), "failure: {:?}", traj.failure);
    assert_eq!(traj.states.len(), 11);
    let (x2_id, _) = mech.node(2).unwrap();
    for s in &traj.states {
        let (u, _, _) = drive.eval(s.t);
        let (x2, _) = pos(&mech, s, 2);
        let (vx2, _) = vel(&mech, s, 2);
        assert_abs_diff_eq!(x2, 1.0 + u, epsilon = 1e-12);
        assert_abs_diff_eq!(vx2, -0.05, epsilon = 1e-10);
        assert_abs_diff_eq!(s.qdd[x2_id.0], 0.0, epsilon = 1e-9);
    }
}
