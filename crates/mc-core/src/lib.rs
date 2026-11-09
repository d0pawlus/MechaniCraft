//! Planar mechanism kinematics core: model, builder and Newton solver.

pub mod model;
pub mod modes;
pub mod solver;
pub mod system;

pub use model::*;
pub use modes::{ModeEval, line_angle, ModeRole, element_modes, mode_roles};
pub use solver::{SolveError, State, Trajectory, analyze};
pub use system::{Assembly, Equation, ModeSystem};
