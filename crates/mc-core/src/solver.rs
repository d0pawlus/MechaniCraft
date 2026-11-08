//! Newton-Raphson kinematic solver with velocity and acceleration analysis.

use nalgebra::{DMatrix, DVector};
use thiserror::Error;

use crate::model::{Mechanism, ModelError};
use crate::modes::{ModeEval, ModeRole, element_modes, mode_roles};

/// Convergence threshold on the infinity norm of the residual.
const NEWTON_TOL: f64 = 1e-12;
/// Maximum number of Newton linear solves per time step.
const MAX_ITERATIONS: usize = 50;

/// Kinematic state at one time point. Vectors are indexed by `CoordId`.
#[derive(Debug, Clone, PartialEq)]
pub struct State {
    pub t: f64,
    pub q: Vec<f64>,
    pub qd: Vec<f64>,
    pub qdd: Vec<f64>,
}

/// Solved states, plus the error that stopped the analysis early, if any.
#[derive(Debug, Clone, PartialEq)]
pub struct Trajectory {
    pub states: Vec<State>,
    pub failure: Option<SolveError>,
}

/// Errors reported by [`analyze`].
#[derive(Debug, Clone, PartialEq, Error)]
pub enum SolveError {
    #[error("system is not square: {equations} equations, {unknowns} unknowns")]
    NotSquare { equations: usize, unknowns: usize },
    #[error("invalid mechanism: {0}")]
    InvalidModel(ModelError),
    #[error("mechanism has no inputs")]
    NoInputs,
    #[error("inputs have different time grids")]
    InputGridMismatch,
    #[error("singular Jacobian at step {step} (t = {t})")]
    Singular { step: usize, t: f64 },
    #[error("no convergence at step {step} (t = {t}), residual {residual}")]
    NoConvergence { step: usize, t: f64, residual: f64 },
}

/// One equation: a mode that is rigid or driven.
struct Equation {
    element: usize,
    mode: usize,
    /// `Some(i)` when driven by `mech.inputs[i]`, `None` when held at zero.
    input: Option<usize>,
}

/// Mode values, Jacobian over the free coordinates and convective terms.
struct Assembly {
    value: DVector<f64>,
    jacobian: DMatrix<f64>,
    conv: DVector<f64>,
}

struct Problem<'a> {
    mech: &'a Mechanism,
    equations: Vec<Equation>,
    /// Column of each coordinate in the Jacobian, `None` for fixed ones.
    column: Vec<Option<usize>>,
    free: Vec<usize>,
}

impl Problem<'_> {
    /// Targets `(u, v, a)` of every equation at time `t`.
    fn targets(&self, t: f64) -> [DVector<f64>; 3] {
        let n = self.equations.len();
        let mut out = [DVector::zeros(n), DVector::zeros(n), DVector::zeros(n)];
        for (row, eq) in self.equations.iter().enumerate() {
            if let Some(i) = eq.input {
                let (u, v, a) = self.mech.inputs[i].law.eval(t);
                out[0][row] = u;
                out[1][row] = v;
                out[2][row] = a;
            }
        }
        out
    }

    /// Evaluates the modes at position `q` and velocity `qd`.
    fn assemble(&self, q: &[f64], qd: &[f64]) -> Assembly {
        let n = self.equations.len();
        let mut a = Assembly {
            value: DVector::zeros(n),
            jacobian: DMatrix::zeros(n, self.free.len()),
            conv: DVector::zeros(n),
        };
        // Equations are ordered by element, so each element is evaluated once.
        let mut current: Option<(usize, Vec<ModeEval>)> = None;
        for (row, eq) in self.equations.iter().enumerate() {
            if current.as_ref().is_none_or(|(el, _)| *el != eq.element) {
                let modes = element_modes(self.mech, &self.mech.elements[eq.element], q, qd);
                current = Some((eq.element, modes));
            }
            if let Some((_, modes)) = &current {
                let mode = &modes[eq.mode];
                a.value[row] = mode.value;
                a.conv[row] = mode.conv;
                for &(c, g) in &mode.grad {
                    if let Some(col) = self.column[c.0] {
                        a.jacobian[(row, col)] += g;
                    }
                }
            }
        }
        a
    }

    /// Newton iteration on the free coordinates of `q`.
    fn newton(
        &self,
        q: &mut [f64],
        target: &DVector<f64>,
        step: usize,
        t: f64,
    ) -> Result<(), SolveError> {
        let zeros = vec![0.0; q.len()];
        for iteration in 0..=MAX_ITERATIONS {
            let a = self.assemble(q, &zeros);
            let r = &a.value - target;
            // `amax` ignores NaN, so check every entry first.
            let norm = if r.iter().all(|v| v.is_finite()) {
                r.amax()
            } else {
                f64::NAN
            };
            if !norm.is_finite() || (norm >= NEWTON_TOL && iteration == MAX_ITERATIONS) {
                return Err(SolveError::NoConvergence {
                    step,
                    t,
                    residual: norm,
                });
            }
            if norm < NEWTON_TOL {
                return Ok(());
            }
            let dq = self.solve(&a.jacobian, &r, step, t)?;
            for (col, &coord) in self.free.iter().enumerate() {
                q[coord] -= dq[col];
            }
        }
        // The last iteration always returns above.
        Err(SolveError::Singular { step, t })
    }

    fn solve(
        &self,
        j: &DMatrix<f64>,
        rhs: &DVector<f64>,
        step: usize,
        t: f64,
    ) -> Result<DVector<f64>, SolveError> {
        match j.clone().lu().solve(rhs) {
            Some(x) if x.iter().all(|v| v.is_finite()) => Ok(x),
            _ => Err(SolveError::Singular { step, t }),
        }
    }

    /// Expands a solution over the free coordinates to all coordinates.
    fn scatter(&self, x: &DVector<f64>, len: usize) -> Vec<f64> {
        let mut out = vec![0.0; len];
        for (col, &coord) in self.free.iter().enumerate() {
            out[coord] = x[col];
        }
        out
    }
}

/// Solves position, velocity and acceleration at every time point of the
/// input grid.
///
/// The model is validated first ([`Mechanism::validate`]). Structural
/// problems return `Err`. A failure at step `k` is stored in
/// [`Trajectory::failure`] and `states` then holds steps `0..k`.
pub fn analyze(mech: &Mechanism) -> Result<Trajectory, SolveError> {
    mech.validate().map_err(SolveError::InvalidModel)?;
    if mech.inputs.is_empty() {
        return Err(SolveError::NoInputs);
    }
    let grid = mech.inputs[0].law.time_grid();
    if mech.inputs.iter().any(|inp| inp.law.time_grid() != grid) {
        return Err(SolveError::InputGridMismatch);
    }

    let equations: Vec<Equation> = mode_roles(mech)
        .into_iter()
        .filter_map(|(element, mode, role)| match role {
            ModeRole::Free => None,
            ModeRole::Rigid => Some(Equation {
                element,
                mode,
                input: None,
            }),
            ModeRole::Driven(i) => Some(Equation {
                element,
                mode,
                input: Some(i),
            }),
        })
        .collect();
    let free: Vec<usize> = (0..mech.coords.len())
        .filter(|&i| !mech.coords[i].fixed)
        .collect();
    if equations.len() != free.len() {
        return Err(SolveError::NotSquare {
            equations: equations.len(),
            unknowns: free.len(),
        });
    }
    let mut column = vec![None; mech.coords.len()];
    for (col, &coord) in free.iter().enumerate() {
        column[coord] = Some(col);
    }
    let problem = Problem {
        mech,
        equations,
        column,
        free,
    };

    let mut states: Vec<State> = Vec::with_capacity(grid.len());
    for (step, &t) in grid.iter().enumerate() {
        match solve_step(&problem, states.last(), step, t) {
            Ok(state) => states.push(state),
            Err(err) => {
                return Ok(Trajectory {
                    states,
                    failure: Some(err),
                });
            }
        }
    }
    Ok(Trajectory {
        states,
        failure: None,
    })
}

fn solve_step(
    problem: &Problem,
    prev: Option<&State>,
    step: usize,
    t: f64,
) -> Result<State, SolveError> {
    let coords = &problem.mech.coords;
    let n = coords.len();
    let mut q: Vec<f64> = match prev {
        Some(p) => {
            let dt = t - p.t;
            (0..n)
                .map(|i| p.q[i] + p.qd[i] * dt + 0.5 * p.qdd[i] * dt * dt)
                .collect()
        }
        None => coords.iter().map(|c| c.value0).collect(),
    };
    for (i, c) in coords.iter().enumerate() {
        if c.fixed {
            q[i] = c.value0;
        }
    }

    let [pos, vel, acc] = problem.targets(t);
    problem.newton(&mut q, &pos, step, t)?;

    let zeros = vec![0.0; n];
    let jacobian = problem.assemble(&q, &zeros).jacobian;
    let qd = problem.scatter(&problem.solve(&jacobian, &vel, step, t)?, n);
    let conv = problem.assemble(&q, &qd).conv;
    let qdd = problem.scatter(&problem.solve(&jacobian, &(acc - conv), step, t)?, n);
    Ok(State { t, q, qd, qdd })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{CoordId, ElementKind, MechanismBuilder};
    use mc_motion::{InputLaw, Segment};

    fn actuator(displacement: f64) -> Mechanism {
        let mut b = MechanismBuilder::new();
        b.node(1, 0.0, 0.0, true, true)
            .node(2, 1.0, 0.0, false, true)
            .angle(1001, 0.0, false)
            .angle(1002, 0.0, false)
            .beam(1, 1, 2, 1001, 1002)
            .input(
                1,
                InputLaw {
                    segments: vec![Segment::Linear {
                        duration: 1.0,
                        steps: 4,
                        displacement,
                    }],
                },
            );
        b.build().unwrap()
    }

    #[test]
    fn invalid_coord_id_is_an_error_not_a_panic() {
        let mut mech = actuator(0.1);
        if let ElementKind::Beam { a, .. } = &mut mech.elements[0].kind {
            *a = CoordId(999);
        }
        assert_eq!(
            analyze(&mech).unwrap_err(),
            SolveError::InvalidModel(ModelError::InvalidCoordId { element: 1 })
        );
    }

    #[test]
    fn unknown_input_element_is_an_error_not_a_panic() {
        let mut mech = actuator(0.1);
        mech.inputs[0].element_nr = 42;
        assert_eq!(
            analyze(&mech).unwrap_err(),
            SolveError::InvalidModel(ModelError::UnknownInputElement(42))
        );
    }

    #[test]
    fn nan_residual_is_no_convergence() {
        let mech = actuator(f64::NAN);
        let traj = analyze(&mech).unwrap();
        assert!(
            matches!(traj.failure, Some(SolveError::NoConvergence { .. })),
            "got {:?}",
            traj.failure
        );
        assert!(traj.states.is_empty());
    }
}
