//! A loaded project together with its analysis results.

use mc_core::{SolveError, Trajectory, analyze};
use mc_kineto::{ForceTrajectory, Loads, kinetostatics};
use mc_samfile::{ImportError, Project, import_project, import_sam};
use thiserror::Error;

use crate::scene::{Bounds, Scene, trajectory_bounds};

/// Errors that prevent a project from being shown at all.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum LoadError {
    /// The file is not a supported SAM project.
    #[error("import failed: {0}")]
    Import(#[from] ImportError),
    /// The mechanism was read but its analysis could not start.
    #[error("analysis failed: {0}")]
    Analysis(#[from] SolveError),
}

/// A project ready to be viewed: model, kinematics, forces and scene.
#[derive(Debug, Clone, PartialEq)]
pub struct Document {
    /// Display name, usually the file name.
    pub name: String,
    pub project: Project,
    pub trajectory: Trajectory,
    /// Kinetostatic results; `None` when they could not be computed (the
    /// reason is in `warnings`).
    pub forces: Option<ForceTrajectory>,
    pub scene: Scene,
    pub(crate) bounds: Option<Bounds>,
    /// Problems that leave partial results, in human-readable form.
    pub warnings: Vec<String>,
}

/// Imports `bytes` as a SAM project and runs kinematics and kinetostatics.
///
/// Import errors and analysis setup errors are returned as [`LoadError`].
/// When only the loads cannot be imported (e.g. external torques), the
/// mechanism is still shown without forces. A kinematic or kinetostatic
/// failure part way through the motion keeps the states solved so far. Both
/// cases add a message to [`Document::warnings`].
pub fn load(name: &str, bytes: &[u8]) -> Result<Document, LoadError> {
    let mut warnings = Vec::new();
    let (project, has_loads) = import(bytes, &mut warnings)?;
    let trajectory = analyze(&project.mechanism)?;
    if let Some(err) = &trajectory.failure {
        warnings.push(format!(
            "kinematics stopped after {} states: {err}",
            trajectory.states.len()
        ));
    }
    let forces = if has_loads {
        solve_forces(&project, &trajectory, &mut warnings)
    } else {
        None
    };
    let scene = Scene::from_mechanism(&project.mechanism);
    let bounds = trajectory_bounds(&scene, &trajectory);
    Ok(Document {
        name: name.to_owned(),
        project,
        trajectory,
        forces,
        scene,
        bounds,
        warnings,
    })
}

/// Full project, or the mechanism alone with empty loads when only the loads
/// are unsupported; the flag tells whether the loads were imported.
fn import(bytes: &[u8], warnings: &mut Vec<String>) -> Result<(Project, bool), LoadError> {
    match import_project(bytes) {
        Ok(project) => Ok((project, true)),
        Err(load_err) => {
            // Report the original error when the mechanism itself is invalid.
            let mechanism = import_sam(bytes).map_err(|_| load_err.clone())?;
            warnings.push(format!(
                "loads not imported, forces not computed: {load_err}"
            ));
            let project = Project {
                mechanism,
                loads: Loads::default(),
            };
            Ok((project, false))
        }
    }
}

fn solve_forces(
    project: &Project,
    trajectory: &Trajectory,
    warnings: &mut Vec<String>,
) -> Option<ForceTrajectory> {
    match kinetostatics(&project.mechanism, &project.loads, trajectory) {
        Ok(forces) => {
            if let Some(err) = &forces.failure {
                warnings.push(format!(
                    "forces stopped after {} states: {err}",
                    forces.states.len()
                ));
            }
            Some(forces)
        }
        Err(err) => {
            warnings.push(format!("forces not computed: {err}"));
            None
        }
    }
}

impl Document {
    /// Number of solved kinematic states (animation frames).
    pub fn frame_count(&self) -> usize {
        self.trajectory.states.len()
    }

    /// Time of every frame.
    pub fn times(&self) -> Vec<f64> {
        self.trajectory.states.iter().map(|s| s.t).collect()
    }
}
