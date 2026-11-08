//! Planar mechanism kinematics core: model, builder and (later) solver.

pub mod model;
pub mod modes;

pub use model::*;
pub use modes::{ModeEval, ModeRole, element_modes, mode_roles};
