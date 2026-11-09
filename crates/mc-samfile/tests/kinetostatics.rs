//! Kinetostatics on the SAM example projects and the SAM-export oracle hook.
//!
//! The example files are not part of the repository. They are read from the
//! directory in `SAM_EXAMPLES_DIR` (default: the SAM installation folder) and
//! the tests skip themselves when it does not exist.

use std::path::PathBuf;

use mc_core::analyze;
use mc_kineto::{applied_forces, inertia_forces, kinetostatics};
use mc_samfile::import_project;

const EXAMPLES: [&str; 10] = [
    "SLCR.SAM",
    "6BAR.SAM",
    "CRRK.SAM",
    "ORTHO.SAM",
    "DYAD.sam",
    "SCS.sam",
    "TRFR.SAM",
    "BED.SAM",
    "QKRT.SAM",
    "DXFDEMO (detached).SAM",
];

fn examples_dir() -> Option<PathBuf> {
    let dir = std::env::var_os("SAM_EXAMPLES_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(r"C:\Users\Public\Documents\Sam\Examples"));
    if dir.is_dir() {
        Some(dir)
    } else {
        eprintln!("skipping: SAM examples directory {} not found", dir.display());
        None
    }
}

fn load(
    dir: &std::path::Path,
    name: &str,
) -> Result<(mc_samfile::Project, mc_core::Trajectory, mc_kineto::ForceTrajectory), String> {
    let bytes = std::fs::read(dir.join(name)).map_err(|e| format!("{name}: read: {e}"))?;
    let project = import_project(&bytes).map_err(|e| format!("{name}: import: {e}"))?;
    let traj = analyze(&project.mechanism).map_err(|e| format!("{name}: analyze: {e}"))?;
    if let Some(f) = &traj.failure {
        return Err(format!("{name}: kinematic failure: {f}"));
    }
    let forces = kinetostatics(&project.mechanism, &project.loads, &traj)
        .map_err(|e| format!("{name}: kinetostatics: {e}"))?;
    Ok((project, traj, forces))
}

#[test]
fn phase1_examples_kinetostatics() {
    let Some(dir) = examples_dir() else { return };
    let mut errors = Vec::new();
    for name in EXAMPLES {
        let (_, traj, forces) = match load(&dir, name) {
            Ok(v) => v,
            Err(e) => {
                errors.push(e);
                continue;
            }
        };
        if let Some(f) = &forces.failure {
            errors.push(format!("{name}: force failure: {f}"));
            continue;
        }
        if forces.states.len() != traj.states.len() {
            errors.push(format!(
                "{name}: {} force states for {} kinematic states",
                forces.states.len(),
                traj.states.len()
            ));
            continue;
        }
        for (k, s) in forces.states.iter().enumerate() {
            let all = s
                .sigma
                .iter()
                .chain(&s.driving)
                .chain(&s.power)
                .chain(&s.reactions);
            if all.into_iter().any(|x| !x.is_finite()) {
                errors.push(format!("{name}: non-finite value at step {k}"));
                break;
            }
        }
    }
    assert!(errors.is_empty(), "errors:\n{}", errors.join("\n"));
}

#[test]
fn examples_power_balance() {
    let Some(dir) = examples_dir() else { return };
    let mut errors = Vec::new();
    for name in EXAMPLES {
        let (project, traj, forces) = match load(&dir, name) {
            Ok(v) => v,
            Err(e) => {
                errors.push(e);
                continue;
            }
        };
        let (mech, loads) = (&project.mechanism, &project.loads);
        for (k, (fs, s)) in forces.states.iter().zip(&traj.states).enumerate() {
            let f_app = applied_forces(mech, loads, s.t, &s.q);
            let f_in = inertia_forces(mech, loads, &s.q, &s.qd, &s.qdd);
            let terms: Vec<f64> = (0..s.qd.len())
                .map(|c| s.qd[c] * (f_in[c] - f_app[c]))
                .collect();
            let expected: f64 = terms.iter().sum();
            let scale = terms.iter().fold(1.0_f64, |m, x| m.max(x.abs()));
            let actual: f64 = fs.power.iter().sum();
            if (actual - expected).abs() > 1e-6 * scale {
                errors.push(format!(
                    "{name}: step {k}: power {actual} vs expected {expected}"
                ));
                break;
            }
        }
    }
    assert!(errors.is_empty(), "errors:\n{}", errors.join("\n"));
}

#[test]
fn examples_with_loads_have_nonzero_driving() {
    let Some(dir) = examples_dir() else { return };
    for name in ["TRFR.SAM", "BED.SAM"] {
        let (_, _, forces) = load(&dir, name).unwrap_or_else(|e| panic!("{e}"));
        let max = forces
            .states
            .iter()
            .filter_map(|s| s.driving.first())
            .fold(0.0_f64, |m, x| m.max(x.abs()));
        assert!(max > 0.0, "{name}: driving force is identically zero");
    }
}

#[test]
fn sam_export_oracle_hook() {
    let dir = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/sam-exports"));
    let has_exports = std::fs::read_dir(&dir)
        .map(|rd| {
            rd.flatten()
                .any(|e| e.path().extension().is_some_and(|x| x == "txt"))
        })
        .unwrap_or(false);
    if !has_exports {
        eprintln!("skipping: no SAM exports in {}", dir.display());
        return;
    }
    panic!(
        "SAM exports found in {}, but the comparison is not implemented yet; \
         adapt the parser to the real export format (see fixtures/sam-exports/README.md)",
        dir.display()
    );
}
