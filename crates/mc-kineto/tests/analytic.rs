//! Analytic tests of the kinetostatic solver: static holding forces, power
//! balance, global force balance and a closed-form slider-crank case.

use std::f64::consts::{FRAC_PI_2, PI};

use approx::assert_abs_diff_eq;
use mc_core::{CoordLabel, Mechanism, MechanismBuilder, SolveError, State, Trajectory, analyze};
use mc_kineto::{
    ElementInertia, ForceArg, ForceTable, ForceTrajectory, Gravity, KinetoError, LoadError, Loads,
    NodalForce, NodeMass, kinetostatics,
};
use mc_motion::{InputLaw, Segment};

const G: f64 = 9.81;

fn gravity() -> Option<Gravity> {
    Some(Gravity {
        value: G,
        angle: -FRAC_PI_2,
    })
}

fn static_law() -> InputLaw {
    InputLaw {
        segments: vec![Segment::Linear {
            duration: 1.0,
            steps: 4,
            displacement: 0.0,
        }],
    }
}

/// Crank with a free end node 2 at `(x2, y2)`, node 1 pinned at the origin.
fn crank(x2: f64, y2: f64) -> Mechanism {
    let mut b = MechanismBuilder::new();
    b.node(1, 0.0, 0.0, true, true)
        .node(2, x2, y2, false, false)
        .angle(1001, y2.atan2(x2), false)
        .angle(1002, y2.atan2(x2), false)
        .angle(1005, 0.0, true)
        .beam(1, 1, 2, 1001, 1002)
        .rot_motor(2, 1005, 1001, false)
        .input(2, static_law());
    b.build().unwrap()
}

fn solve(mech: &Mechanism, loads: &Loads) -> (Trajectory, ForceTrajectory) {
    let traj = analyze(mech).unwrap();
    assert!(traj.failure.is_none(), "failure: {:?}", traj.failure);
    let ft = kinetostatics(mech, loads, &traj).unwrap();
    assert!(ft.failure.is_none(), "failure: {:?}", ft.failure);
    assert_eq!(ft.states.len(), traj.states.len());
    (traj, ft)
}

fn coord(mech: &Mechanism, label: CoordLabel) -> usize {
    mech.coords
        .iter()
        .position(|c| c.label == label)
        .expect("coordinate")
}

fn reaction(mech: &Mechanism, ft: &ForceTrajectory, k: usize, label: CoordLabel) -> f64 {
    ft.states[k].reactions[coord(mech, label)]
}

fn mass2() -> Loads {
    Loads {
        node_masses: vec![NodeMass { node: 2, mass: 2.0 }],
        gravity: gravity(),
        ..Loads::default()
    }
}

#[test]
fn static_horizontal_crank_holding_torque() {
    let mech = crank(0.5, 0.0);
    let (_, ft) = solve(&mech, &mass2());
    assert_eq!(ft.states.len(), 5);
    for k in 0..5 {
        assert_abs_diff_eq!(ft.states[k].driving[0], 9.81, epsilon = 1e-9);
        assert_abs_diff_eq!(ft.states[k].power[0], 0.0, epsilon = 1e-9);
        assert_abs_diff_eq!(ft.stress(k, 1, 0).unwrap(), 0.0, epsilon = 1e-9);
        assert_abs_diff_eq!(ft.stress(k, 1, 1).unwrap(), 9.81, epsilon = 1e-9);
        assert_abs_diff_eq!(ft.stress(k, 1, 2).unwrap(), 0.0, epsilon = 1e-9);
        assert_abs_diff_eq!(
            reaction(&mech, &ft, k, CoordLabel::NodeX(1)),
            0.0,
            epsilon = 1e-9
        );
        assert_abs_diff_eq!(
            reaction(&mech, &ft, k, CoordLabel::NodeY(1)),
            19.62,
            epsilon = 1e-9
        );
        assert_abs_diff_eq!(
            reaction(&mech, &ft, k, CoordLabel::Angle(1005)),
            9.81,
            epsilon = 1e-9
        );
    }
}

#[test]
fn static_vertical_crank_compression() {
    let mech = crank(0.0, 0.5);
    let (_, ft) = solve(&mech, &mass2());
    for k in 0..5 {
        assert_abs_diff_eq!(ft.stress(k, 1, 0).unwrap(), -19.62, epsilon = 1e-9);
        assert_abs_diff_eq!(ft.states[k].driving[0], 0.0, epsilon = 1e-9);
        assert_abs_diff_eq!(
            reaction(&mech, &ft, k, CoordLabel::NodeY(1)),
            19.62,
            epsilon = 1e-9
        );
    }
}

#[test]
fn static_beam_mass_halves_torque() {
    let mech = crank(0.5, 0.0);
    let loads = Loads {
        element_inertias: vec![ElementInertia {
            element: 1,
            mass: 2.0,
            cog: 0.5,
            inertia: 0.0,
        }],
        gravity: gravity(),
        ..Loads::default()
    };
    let (_, ft) = solve(&mech, &loads);
    for k in 0..5 {
        assert_abs_diff_eq!(ft.states[k].driving[0], 4.905, epsilon = 1e-9);
        assert_abs_diff_eq!(
            reaction(&mech, &ft, k, CoordLabel::NodeY(1)),
            19.62,
            epsilon = 1e-9
        );
    }
}

#[test]
fn static_actuator_pushes_mass() {
    let mut b = MechanismBuilder::new();
    b.node(1, 0.0, 0.0, true, true)
        .node(2, 0.0, 0.5, true, false)
        .angle(1001, FRAC_PI_2, false)
        .angle(1002, FRAC_PI_2, false)
        .beam(1, 1, 2, 1001, 1002)
        .input(1, static_law());
    let mech = b.build().unwrap();
    let (_, ft) = solve(&mech, &mass2());
    for k in 0..5 {
        assert_abs_diff_eq!(ft.states[k].driving[0], 19.62, epsilon = 1e-9);
        assert_abs_diff_eq!(ft.stress(k, 1, 0).unwrap(), -19.62, epsilon = 1e-9);
    }
}

fn law(duration: f64, steps: u32) -> InputLaw {
    InputLaw {
        segments: vec![Segment::Linear {
            duration,
            steps,
            displacement: 2.0 * PI,
        }],
    }
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

const CRANK_M: f64 = 1.0;
const CRANK_S: f64 = 0.5;
const CRANK_J: f64 = 0.01;
const ROD_M: f64 = 2.0;
const ROD_S: f64 = 0.3;
const ROD_J: f64 = 0.05;
const SLIDER_M: f64 = 25.0;
const FORCE: [f64; 2] = [10.0, 0.0];

fn full_loads() -> Loads {
    Loads {
        node_masses: vec![NodeMass {
            node: 3,
            mass: SLIDER_M,
        }],
        element_inertias: vec![
            ElementInertia {
                element: 1,
                mass: CRANK_M,
                cog: CRANK_S,
                inertia: CRANK_J,
            },
            ElementInertia {
                element: 2,
                mass: ROD_M,
                cog: ROD_S,
                inertia: ROD_J,
            },
        ],
        gravity: gravity(),
        forces: vec![NodalForce {
            node: 3,
            arg: ForceArg::Time,
            table: ForceTable::new(vec![[0.0, 10.0, 0.0], [1.0, 10.0, 0.0]]).unwrap(),
        }],
    }
}

type Kin = ([f64; 2], [f64; 2], [f64; 2]);

fn node_q(mech: &Mechanism, s: &State, node: u32) -> Kin {
    let (x, y) = mech.node(node).unwrap();
    (
        [s.q[x.0], s.q[y.0]],
        [s.qd[x.0], s.qd[y.0]],
        [s.qdd[x.0], s.qdd[y.0]],
    )
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

/// Centre of mass kinematics of an element between nodes `n1`, `n2`.
struct Cm {
    v: [f64; 2],
    a: [f64; 2],
    alpha_d: f64,
    alpha_dd: f64,
}

fn element_cm(mech: &Mechanism, s: &State, n1: u32, n2: u32, frac: f64) -> Cm {
    let (p, pd, pdd) = node_q(mech, s, n1);
    let (q, qd, qdd) = node_q(mech, s, n2);
    let mix = |a: [f64; 2], b: [f64; 2]| {
        [
            (1.0 - frac) * a[0] + frac * b[0],
            (1.0 - frac) * a[1] + frac * b[1],
        ]
    };
    let d = sub(q, p);
    let dd = sub(qd, pd);
    let ddd = sub(qdd, pdd);
    let l2 = dot(d, d);
    Cm {
        v: mix(pd, qd),
        a: mix(pdd, qdd),
        alpha_d: cross(d, dd) / l2,
        alpha_dd: cross(d, ddd) / l2 - 2.0 * cross(d, dd) * dot(d, dd) / (l2 * l2),
    }
}

#[test]
fn slider_crank_power_balance() {
    let mech = slider_crank(0.5, law(1.0, 36), true);
    let loads = full_loads();
    let (traj, ft) = solve(&mech, &loads);
    assert_eq!(ft.states.len(), 37);
    let g = [0.0, -G];
    let scale = ft
        .states
        .iter()
        .map(|f| f.power[0].abs())
        .fold(1.0_f64, f64::max);
    for (s, f) in traj.states.iter().zip(&ft.states) {
        let crank = element_cm(&mech, s, 1, 2, CRANK_S);
        let rod = element_cm(&mech, s, 2, 3, ROD_S);
        let (_, v3, a3) = node_q(&mech, s, 3);
        let d_t = SLIDER_M * dot(v3, a3)
            + CRANK_M * dot(crank.v, crank.a)
            + CRANK_J * crank.alpha_d * crank.alpha_dd
            + ROD_M * dot(rod.v, rod.a)
            + ROD_J * rod.alpha_d * rod.alpha_dd;
        let d_v = -(SLIDER_M * dot(g, v3) + CRANK_M * dot(g, crank.v) + ROD_M * dot(g, rod.v));
        let p_ext = dot(FORCE, v3);
        let expected = d_t + d_v - p_ext;
        assert!(
            (f.power[0] - expected).abs() <= 1e-6 * scale,
            "t = {}: power {} vs balance {}",
            s.t,
            f.power[0],
            expected
        );
    }
}

#[test]
fn slider_mass_only_matches_closed_form() {
    let mech = slider_crank(0.5, law(1.0, 36), true);
    let loads = Loads {
        node_masses: vec![NodeMass {
            node: 3,
            mass: SLIDER_M,
        }],
        ..Loads::default()
    };
    let (traj, ft) = solve(&mech, &loads);
    let omega = 2.0 * PI;
    let expected: Vec<f64> = traj
        .states
        .iter()
        .map(|s| {
            let (_, v, a) = node_q(&mech, s, 3);
            SLIDER_M * a[0] * v[0] / omega
        })
        .collect();
    let scale = expected.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
    assert!(scale > 0.0);
    for (f, e) in ft.states.iter().zip(&expected) {
        assert!((f.driving[0] - e).abs() <= 1e-8 * scale);
    }
}

#[test]
fn global_force_balance_slider_crank() {
    let mech = slider_crank(0.5, law(1.0, 36), true);
    let loads = full_loads();
    let (traj, ft) = solve(&mech, &loads);
    let g = [0.0, -G];
    let ang = coord(&mech, CoordLabel::Angle(1005));
    for (k, s) in traj.states.iter().enumerate() {
        let f = &ft.states[k];
        let crank = element_cm(&mech, s, 1, 2, CRANK_S);
        let rod = element_cm(&mech, s, 2, 3, ROD_S);
        let (_, _, a3) = node_q(&mech, s, 3);
        let ma = [
            SLIDER_M * a3[0] + CRANK_M * crank.a[0] + ROD_M * rod.a[0],
            SLIDER_M * a3[1] + CRANK_M * crank.a[1] + ROD_M * rod.a[1],
        ];
        let weight = (SLIDER_M + CRANK_M + ROD_M) * g[1];
        let rx = reaction(&mech, &ft, k, CoordLabel::NodeX(1));
        let ry = reaction(&mech, &ft, k, CoordLabel::NodeY(1))
            + reaction(&mech, &ft, k, CoordLabel::NodeY(3));
        let lhs_x = rx + FORCE[0];
        let lhs_y = ry + FORCE[1] + weight;
        let scale = [lhs_x, lhs_y, ma[0], ma[1], rx, ry]
            .iter()
            .fold(1.0_f64, |m, v| m.max(v.abs()));
        assert!((lhs_x - ma[0]).abs() <= 1e-9 * scale, "x balance at t={}", s.t);
        assert!((lhs_y - ma[1]).abs() <= 1e-9 * scale, "y balance at t={}", s.t);
        assert!((f.reactions[ang] - f.driving[0]).abs() <= 1e-9 * scale);
    }
}

#[test]
fn kinetostatics_rejects_bad_loads() {
    let mech = slider_crank(0.5, law(1.0, 36), true);
    let traj = analyze(&mech).unwrap();
    let loads = Loads {
        node_masses: vec![NodeMass {
            node: 99,
            mass: 1.0,
        }],
        ..Loads::default()
    };
    assert_eq!(
        kinetostatics(&mech, &loads, &traj).unwrap_err(),
        KinetoError::Loads(LoadError::UnknownNode(99))
    );
}

#[test]
fn short_state_is_error_not_panic() {
    let mech = slider_crank(0.5, law(1.0, 36), true);
    let mut traj = analyze(&mech).unwrap();
    traj.states[2].qdd.pop();
    assert_eq!(
        kinetostatics(&mech, &full_loads(), &traj).unwrap_err(),
        KinetoError::StateLength { step: 2 }
    );
}

#[test]
fn partial_trajectory_is_processed() {
    let mech = slider_crank(0.5, law(1.0, 36), true);
    let mut traj = analyze(&mech).unwrap();
    traj.states.truncate(10);
    traj.failure = Some(SolveError::NoInputs);
    let ft = kinetostatics(&mech, &full_loads(), &traj).unwrap();
    assert_eq!(ft.states.len(), 10);
    assert!(ft.failure.is_none());
}
