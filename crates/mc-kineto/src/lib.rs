//! Quasi-static kinetostatics for planar mechanisms.
//!
//! This crate holds the load model (node masses, element inertias, gravity
//! and tabulated nodal forces). Later steps add the generalized force vectors
//! and the kinetostatic solver. Analysis is quasi-static: no time integration
//! is performed.

pub mod loads;

pub use loads::{
    ElementInertia, ForceArg, ForceTable, Gravity, LoadError, Loads, NodalForce, NodeMass,
};
