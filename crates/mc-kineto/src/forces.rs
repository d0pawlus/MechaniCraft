//! Generalized applied and inertia force vectors.
//!
//! Both functions return vectors indexed like `Mechanism::coords`. The
//! kinetostatic equation uses `f_app - f_in` on the free coordinates.

use mc_core::{ElementKind, Mechanism, line_angle};

use crate::loads::{ForceArg, Loads};

fn gravity_vector(loads: &Loads) -> [f64; 2] {
    loads.gravity.map_or([0.0, 0.0], |g| g.vector())
}

/// Reads `v[i]`, treating an out-of-range index as zero (never panics).
fn at(v: &[f64], i: usize) -> f64 {
    v.get(i).copied().unwrap_or(0.0)
}

fn add(out: &mut [f64], i: usize, value: f64) {
    if let Some(slot) = out.get_mut(i) {
        *slot += value;
    }
}

/// Generalized force vector of gravity and nodal forces at time `t` and
/// configuration `q`.
///
/// Assumes `loads.validate(mech)` succeeded and `q.len() == mech.coords.len()`.
/// References that cannot be resolved are skipped without panicking; the
/// validation and length checks are the caller's responsibility
/// (`kinetostatics` performs them).
pub fn applied_forces(mech: &Mechanism, loads: &Loads, t: f64, q: &[f64]) -> Vec<f64> {
    let mut f = vec![0.0; mech.coords.len()];
    let g = gravity_vector(loads);
    for nm in &loads.node_masses {
        if let Some((x, y)) = mech.node(nm.node) {
            add(&mut f, x.0, nm.mass * g[0]);
            add(&mut f, y.0, nm.mass * g[1]);
        }
    }
    for ei in &loads.element_inertias {
        let Some(el) = mech.elements.iter().find(|e| e.nr == ei.element) else {
            continue;
        };
        if let ElementKind::Beam { p, q: qn, .. } | ElementKind::Slider { p, q: qn, .. } = &el.kind
        {
            let s = ei.cog;
            for c in 0..2 {
                add(&mut f, p[c].0, ei.mass * (1.0 - s) * g[c]);
                add(&mut f, qn[c].0, ei.mass * s * g[c]);
            }
        }
    }
    for force in &loads.forces {
        let Some((nx, ny)) = mech.node(force.node) else {
            continue;
        };
        let arg = match force.arg {
            ForceArg::Time => t,
            // The argument follows the current state, not the initial position.
            ForceArg::NodeX(n) => match mech.node(n) {
                Some((x, _)) => at(q, x.0),
                None => continue,
            },
            ForceArg::NodeY(n) => match mech.node(n) {
                Some((_, y)) => at(q, y.0),
                None => continue,
            },
        };
        let (value, angle) = force.table.eval(arg);
        add(&mut f, nx.0, value * angle.cos());
        add(&mut f, ny.0, value * angle.sin());
    }
    f
}

/// Generalized inertia force vector (to be subtracted from the applied
/// forces) at state `(q, qd, qdd)`.
///
/// Assumes `loads.validate(mech)` succeeded and all slices have length
/// `mech.coords.len()`. References that cannot be resolved are skipped
/// without panicking; the caller validates.
pub fn inertia_forces(
    mech: &Mechanism,
    loads: &Loads,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
) -> Vec<f64> {
    let mut f = vec![0.0; mech.coords.len()];
    for nm in &loads.node_masses {
        if let Some((x, y)) = mech.node(nm.node) {
            add(&mut f, x.0, nm.mass * at(qdd, x.0));
            add(&mut f, y.0, nm.mass * at(qdd, y.0));
        }
    }
    for ei in &loads.element_inertias {
        let Some(el) = mech.elements.iter().find(|e| e.nr == ei.element) else {
            continue;
        };
        if let ElementKind::Beam { p, q: qn, .. } | ElementKind::Slider { p, q: qn, .. } = &el.kind
        {
            let s = ei.cog;
            for c in 0..2 {
                let acc = (1.0 - s) * at(qdd, p[c].0) + s * at(qdd, qn[c].0);
                add(&mut f, p[c].0, ei.mass * (1.0 - s) * acc);
                add(&mut f, qn[c].0, ei.mass * s * acc);
            }
            if ei.inertia != 0.0
                && let Some(la) = line_angle(mech, el, q, qd)
            {
                // alpha_ddot = grad . qdd + convective term
                let alpha_dd: f64 =
                    la.grad.iter().map(|(k, gk)| gk * at(qdd, k.0)).sum::<f64>() + la.conv;
                for (k, gk) in &la.grad {
                    add(&mut f, k.0, ei.inertia * alpha_dd * gk);
                }
            }
        }
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::loads::{ElementInertia, ForceTable, Gravity, NodalForce, NodeMass};
    use approx::assert_abs_diff_eq;
    use mc_core::MechanismBuilder;
    use std::f64::consts::FRAC_PI_2;

    /// Two free nodes, one beam. Returns the mechanism and the coordinate
    /// indices `[x1, y1, x2, y2]`.
    fn setup() -> (Mechanism, [usize; 4]) {
        let mut b = MechanismBuilder::new();
        b.node(1, 0.0, 0.0, false, false)
            .node(2, 1.0, 0.0, false, false)
            .angle(1001, 0.0, false)
            .angle(1002, 0.0, false)
            .beam(1, 1, 2, 1001, 1002);
        let mech = b.build().unwrap();
        let (x1, y1) = mech.node(1).unwrap();
        let (x2, y2) = mech.node(2).unwrap();
        (mech, [x1.0, y1.0, x2.0, y2.0])
    }

    fn q0(mech: &Mechanism) -> Vec<f64> {
        mech.coords.iter().map(|c| c.value0).collect()
    }

    fn gravity() -> Option<Gravity> {
        Some(Gravity {
            value: 10.0,
            angle: -FRAC_PI_2,
        })
    }

    fn assert_vec(got: &[f64], expect: &[(usize, f64)]) {
        for (i, v) in got.iter().enumerate() {
            let e = expect.iter().find(|(k, _)| *k == i).map_or(0.0, |(_, e)| *e);
            assert_abs_diff_eq!(*v, e, epsilon = 1e-12);
        }
    }

    #[test]
    fn node_mass_inertia_and_weight() {
        let (mech, [_, _, x2, y2]) = setup();
        let loads = Loads {
            node_masses: vec![NodeMass { node: 2, mass: 3.0 }],
            gravity: gravity(),
            ..Loads::default()
        };
        let q = q0(&mech);
        let qd = vec![0.0; q.len()];
        let mut qdd = vec![0.0; q.len()];
        qdd[x2] = 2.0;
        qdd[y2] = -1.0;
        let f_in = inertia_forces(&mech, &loads, &q, &qd, &qdd);
        assert_vec(&f_in, &[(x2, 6.0), (y2, -3.0)]);
        let f_app = applied_forces(&mech, &loads, 0.0, &q);
        assert_vec(&f_app, &[(y2, -30.0)]);
    }

    #[test]
    fn beam_mass_lumps_by_cog() {
        let (mech, [x1, y1, x2, y2]) = setup();
        let loads = Loads {
            element_inertias: vec![ElementInertia {
                element: 1,
                mass: 4.0,
                cog: 0.25,
                inertia: 0.0,
            }],
            gravity: gravity(),
            ..Loads::default()
        };
        let q = q0(&mech);
        let qd = vec![0.0; q.len()];
        let mut qdd = vec![0.0; q.len()];
        qdd[x1] = 1.0;
        qdd[x2] = 3.0;
        let f_in = inertia_forces(&mech, &loads, &q, &qd, &qdd);
        assert_vec(&f_in, &[(x1, 4.5), (x2, 1.5)]);
        let f_app = applied_forces(&mech, &loads, 0.0, &q);
        assert_vec(&f_app, &[(y1, -30.0), (y2, -10.0)]);
    }

    fn inertia_only() -> Loads {
        Loads {
            element_inertias: vec![ElementInertia {
                element: 1,
                mass: 0.0,
                cog: 0.5,
                inertia: 0.5,
            }],
            ..Loads::default()
        }
    }

    #[test]
    fn rotational_inertia_uses_angular_acceleration() {
        let (mech, [_, y1, _, y2]) = setup();
        let q = q0(&mech);
        let qd = vec![0.0; q.len()];
        let mut qdd = vec![0.0; q.len()];
        qdd[y2] = 2.0;
        let f_in = inertia_forces(&mech, &inertia_only(), &q, &qd, &qdd);
        assert_vec(&f_in, &[(y2, 1.0), (y1, -1.0)]);
    }

    #[test]
    fn rotational_inertia_includes_convective_term() {
        let (mech, [_, y1, x2, y2]) = setup();
        let q = q0(&mech);
        let mut qd = vec![0.0; q.len()];
        qd[x2] = 1.0;
        qd[y2] = 1.0;
        let qdd = vec![0.0; q.len()];
        let f_in = inertia_forces(&mech, &inertia_only(), &q, &qd, &qdd);
        assert_vec(&f_in, &[(y2, -1.0), (y1, 1.0)]);
    }

    #[test]
    fn force_table_follows_node_y() {
        let (mech, [_, _, x2, y2]) = setup();
        let loads = Loads {
            forces: vec![NodalForce {
                node: 2,
                arg: ForceArg::NodeY(2),
                table: ForceTable::new(vec![[0.0, 0.0, 0.0], [1.0, 100.0, 0.0]]).unwrap(),
            }],
            ..Loads::default()
        };
        let mut q = q0(&mech);
        let f = applied_forces(&mech, &loads, 0.0, &q);
        assert_abs_diff_eq!(f[x2], 0.0, epsilon = 1e-12);
        q[y2] = 0.25;
        let f = applied_forces(&mech, &loads, 0.0, &q);
        assert_abs_diff_eq!(f[x2], 25.0, epsilon = 1e-12);
    }

    #[test]
    fn time_force_uses_t() {
        let (mech, [_, _, x2, y2]) = setup();
        let loads = Loads {
            forces: vec![NodalForce {
                node: 2,
                arg: ForceArg::Time,
                table: ForceTable::new(vec![[0.0, 0.0, FRAC_PI_2], [2.0, 8.0, FRAC_PI_2]])
                    .unwrap(),
            }],
            ..Loads::default()
        };
        let q = q0(&mech);
        let f = applied_forces(&mech, &loads, 0.5, &q);
        assert_abs_diff_eq!(f[y2], 2.0, epsilon = 1e-12);
        assert_abs_diff_eq!(f[x2], 0.0, epsilon = 1e-12);
    }
}
