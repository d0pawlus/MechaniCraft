//! Kinetostatic solver: stresses, driving forces, power and reactions.
//!
//! At each state `(q, qd, qdd)` of a kinematic trajectory the equilibrium
//! equation `J^T sigma = (f_app - f_in)_free` is solved, where `J` is the
//! square Jacobian of the rigid and driven modes over the free coordinates.
//! Reactions on the fixed coordinates follow from `r = D^T sigma + f_in - f_app`
//! with `D` the full Jacobian.

use mc_core::{Mechanism, ModeSystem, SolveError, Trajectory};
use nalgebra::DVector;
use thiserror::Error;

use crate::forces::{applied_forces, inertia_forces};
use crate::loads::{LoadError, Loads};

/// Identifies one stress: an equation of the mode system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StressLabel {
    /// Element number (`Element::nr`).
    pub element_nr: u32,
    /// Mode index within the element.
    pub mode: usize,
    /// `Some(i)` when the mode is driven by `mech.inputs[i]`.
    pub input: Option<usize>,
}

/// Forces at one time point.
#[derive(Debug, Clone, PartialEq)]
pub struct ForceState {
    pub t: f64,
    /// Stress of each equation, ordered like `ForceTrajectory::stresses`.
    pub sigma: Vec<f64>,
    /// Driving force or torque of each input (`-sigma` of the driven mode).
    pub driving: Vec<f64>,
    /// Driving power of each input: `driving * v(t)`.
    pub power: Vec<f64>,
    /// Reaction on each coordinate (exactly 0.0 on free coordinates):
    /// force or moment exerted by the support on the mechanism.
    pub reactions: Vec<f64>,
}

/// Force results for a trajectory.
#[derive(Debug, Clone, PartialEq)]
pub struct ForceTrajectory {
    pub stresses: Vec<StressLabel>,
    pub states: Vec<ForceState>,
    pub failure: Option<KinetoError>,
}

impl ForceTrajectory {
    /// Stress of mode `mode` of element `element_nr` at `step`; `None` when
    /// the step does not exist or the mode has no equation (it is free).
    pub fn stress(&self, step: usize, element_nr: u32, mode: usize) -> Option<f64> {
        let state = self.states.get(step)?;
        let idx = self
            .stresses
            .iter()
            .position(|s| s.element_nr == element_nr && s.mode == mode)?;
        state.sigma.get(idx).copied()
    }
}

/// Errors reported by [`kinetostatics`].
#[derive(Debug, Clone, PartialEq, Error)]
pub enum KinetoError {
    #[error("invalid mechanism: {0}")]
    Setup(SolveError),
    #[error("invalid loads: {0}")]
    Loads(LoadError),
    #[error("state {step} does not match the mechanism")]
    StateLength { step: usize },
    #[error("singular equilibrium system at step {step} (t = {t})")]
    Singular { step: usize, t: f64 },
}

/// Computes stresses, driving forces, power and reactions for every state of
/// `traj`. States are processed regardless of `traj.failure`.
pub fn kinetostatics(
    mech: &Mechanism,
    loads: &Loads,
    traj: &Trajectory,
) -> Result<ForceTrajectory, KinetoError> {
    let sys = ModeSystem::new(mech).map_err(KinetoError::Setup)?;
    loads.validate(mech).map_err(KinetoError::Loads)?;
    let n_coords = mech.coords.len();
    for (step, s) in traj.states.iter().enumerate() {
        if s.q.len() != n_coords || s.qd.len() != n_coords || s.qdd.len() != n_coords {
            return Err(KinetoError::StateLength { step });
        }
    }

    let stresses: Vec<StressLabel> = sys
        .equations()
        .iter()
        .map(|eq| StressLabel {
            element_nr: mech.elements[eq.element].nr,
            mode: eq.mode,
            input: eq.input,
        })
        .collect();
    let free = sys.free();
    let mut out = ForceTrajectory {
        stresses,
        states: Vec::with_capacity(traj.states.len()),
        failure: None,
    };

    for (step, s) in traj.states.iter().enumerate() {
        let d = sys.full_jacobian(&s.q);
        let jac = d.select_columns(free);
        let f_app = applied_forces(mech, loads, s.t, &s.q);
        let f_in = inertia_forces(mech, loads, &s.q, &s.qd, &s.qdd);
        let rhs = DVector::from_iterator(free.len(), free.iter().map(|&c| f_app[c] - f_in[c]));

        let sigma = if free.is_empty() {
            Some(DVector::zeros(0))
        } else {
            jac.transpose()
                .lu()
                .solve(&rhs)
                .filter(|v| v.iter().all(|x| x.is_finite()))
        };
        let Some(sigma) = sigma else {
            out.failure = Some(KinetoError::Singular { step, t: s.t });
            break;
        };

        let dts = d.transpose() * &sigma;
        let mut reactions = vec![0.0; n_coords];
        for (c, r) in reactions.iter_mut().enumerate() {
            if mech.coords[c].fixed {
                *r = dts[c] + f_in[c] - f_app[c];
            }
        }

        let mut driving = vec![0.0; mech.inputs.len()];
        let mut power = vec![0.0; mech.inputs.len()];
        for (row, eq) in sys.equations().iter().enumerate() {
            if let Some(i) = eq.input {
                driving[i] = -sigma[row];
                power[i] = driving[i] * mech.inputs[i].law.eval(s.t).1;
            }
        }
        out.states.push(ForceState {
            t: s.t,
            sigma: sigma.iter().copied().collect(),
            driving,
            power,
            reactions,
        });
    }
    Ok(out)
}
