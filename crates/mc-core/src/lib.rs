//! Planar mechanism kinematics core: model, builder and Newton solver.

pub mod model;
pub mod modes;
pub mod solver;

pub use model::*;
pub use modes::{ModeEval, ModeRole, element_modes, mode_roles};
pub use solver::{SolveError, State, Trajectory, analyze};
